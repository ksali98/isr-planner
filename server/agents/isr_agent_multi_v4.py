"""
ISR Multi-Agent Mission Planning System (v4) - REASONING AGENTS

A TRUE reasoning-based multi-agent system where each agent:
1. Receives full context about the mission
2. REASONS about the problem before deciding
3. Explains WHY decisions were made
4. Can answer questions about configurations and trade-offs

Core Principles:
- DEFAULT: Maximize priority points with minimum fuel (optimize)
- COMMANDS: Direct orders from commander override everything
- CONSTRAINTS: Drone configs (accessible targets, fuel) are treated as commands
- REASONING: Explain allocation changes and trade-offs
- SUGGESTIONS: Proactively identify improvements

Architecture:
- STRATEGIST: Analyzes request, determines mode (optimize vs command), identifies constraints
- ALLOCATOR: Reasons about target-drone assignments with explanations
- ROUTE_OPTIMIZER: Computes routes, checks feasibility, reports trade-offs
- CRITIC: Reviews solution quality, suggests improvements
- RESPONDER: Formulates final answer with concise reasoning
"""

import os
import sys
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional, TypedDict, Annotated, Literal, Tuple
from dataclasses import dataclass, field
from enum import Enum

from dotenv import load_dotenv
from langchain_anthropic import ChatAnthropic
from langchain_core.messages import HumanMessage, AIMessage, SystemMessage, BaseMessage, ToolMessage
from langchain_core.tools import tool
from langgraph.graph import StateGraph, END, START
from langgraph.graph.message import add_messages

# Import solver components
from pathlib import Path
import sys

# Add legacy path for orienteering solver
legacy_path = Path(__file__).resolve().parents[3] / "legacy" / "isr_legacy_all"
if str(legacy_path) not in sys.path:
    sys.path.insert(0, str(legacy_path))

# Add root path for local development
root_path = Path(__file__).resolve().parents[3]
if str(root_path) not in sys.path:
    sys.path.insert(0, str(root_path))

# Add /app path for Docker deployment
if "/app" not in sys.path:
    sys.path.insert(0, "/app")

# Try importing solver interface
OrienteeringSolverInterface = None
try:
    from webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface
    print("✅ [v4] OrienteeringSolverInterface loaded (Docker path)")
except ImportError:
    try:
        from isr_web.webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface
        print("✅ [v4] OrienteeringSolverInterface loaded (local path)")
    except ImportError:
        print("⚠️ [v4] OrienteeringSolverInterface not available")

from ..solver.target_allocator import (
    allocate_targets as _allocate_targets_impl,
    parse_priority_constraints,
    allocate_with_priority_filters,
    set_allocator_matrix,
    clear_allocator_matrix,
)
from ..solver.sam_distance_matrix import calculate_sam_aware_matrix
from ..solver.post_optimizer import post_optimize_solution
from ..solver.trajectory_planner import ISRTrajectoryPlanner

# Import policy rules from mission_ledger for Learning v1
try:
    from ..database.mission_ledger import get_active_policy_rules
except ImportError:
    get_active_policy_rules = None
    print("⚠️ [v4] mission_ledger not available, policy rules disabled")

# Import Coordinator v4 for deterministic pre-pass
from .coordinator_v4 import run_coordinator, CoordinatorDecision

# Load environment variables
load_dotenv()

if not os.getenv("ANTHROPIC_API_KEY"):
    benchmark_env = os.path.join(os.path.dirname(__file__), "..", "..", "..", "isr_benchmark", ".env")
    if os.path.exists(benchmark_env):
        load_dotenv(benchmark_env)


# ============================================================================
# MISSION STATE - Shared context for all agents
# ============================================================================

class MissionState(TypedDict):
    """
    State shared across all reasoning agents.
    Each agent can read and write to this state.
    """
    # Conversation
    messages: Annotated[list, add_messages]

    # Mission data
    environment: Optional[Dict[str, Any]]
    drone_configs: Optional[Dict[str, Any]]
    distance_matrix: Optional[Dict[str, Any]]
    excluded_targets: Optional[List[str]]  # Targets inside SAM zones - DO NOT allocate

    # User request analysis
    user_request: str
    request_type: str  # "question", "optimize", "command"
    commands: Optional[List[Dict[str, Any]]]  # Explicit commands extracted

    # Coordinator v4 fields (injected by pre-pass)
    intent: Optional[str]  # plan|replan|explain|what_if|debug|unknown
    policy: Optional[Dict[str, Any]]  # allocation_strategy, solver_mode, etc.
    guardrails: Optional[Dict[str, Any]]  # validation results
    drone_contracts: Optional[Dict[str, Dict[str, Any]]]  # per-drone start/home/endpoint contracts
    allocation_strategy: Optional[str]  # efficient|greedy|balanced|geographic|exclusive

    # Agent reasoning outputs
    strategy_analysis: Optional[str]  # Strategist's analysis
    allocation_reasoning: Optional[str]  # Allocator's reasoning
    route_analysis: Optional[str]  # Router's analysis
    critic_review: Optional[str]  # Critic's review
    suggestions: Optional[List[str]]  # Improvement suggestions

    # Solution data
    allocation: Optional[Dict[str, List[str]]]
    routes: Optional[Dict[str, Dict[str, Any]]]

    # Solver metadata (for Decision Trace v1)
    solver_type: Optional[str]
    solver_runtime_ms: Optional[int]

    # Policy rules (Learning v1)
    policy_rules: Optional[List[Dict[str, Any]]]

    # Final response
    final_response: Optional[str]

    # Error handling
    error: Optional[str]


# Global state reference for tools
_current_state: Optional[MissionState] = None


def get_state() -> MissionState:
    if _current_state is None:
        raise ValueError("No mission state set")
    return _current_state


def set_state(state: MissionState):
    global _current_state
    _current_state = state


# ============================================================================
# HARD CONSTRAINTS - Shared by ALL agents (MUST NEVER BE VIOLATED)
# ============================================================================

HARD_CONSTRAINTS = """
═══════════════════════════════════════════════════════════════════════════════
                    HARD CONSTRAINTS - MUST NEVER BE VIOLATED
═══════════════════════════════════════════════════════════════════════════════

1. DRONE TARGET ELIGIBILITY (Sensor Type Restrictions):
   - Each drone has an "ELIGIBLE TARGETS" list based on its sensor configuration
   - A drone can ONLY be assigned targets from its eligible list
   - If D1's eligible targets are "T1, T3, T5" → D1 can ONLY visit T1, T3, T5
   - NEVER assign a target to a drone not in that drone's eligible list
   - This is a PHYSICAL constraint (sensor type) and CANNOT be overridden

2. EXCLUDED TARGETS (Inside SAM Zones):
   - Targets marked as "EXCLUDED (inside SAM zones)" are UNREACHABLE
   - No drone can visit excluded targets - they must be ignored entirely

3. FUEL BUDGET:
   - Each drone has a fuel budget that cannot be exceeded
   - Routes must respect fuel constraints

4. ALL ELIGIBLE TARGETS MUST BE COVERED:
   - Every target that is NOT excluded must be assigned to exactly one drone
   - The assigned drone MUST be eligible to visit that target

These constraints apply to ALL operations: allocation, optimization, reallocation,
and routing. No user request can override these physical constraints.
═══════════════════════════════════════════════════════════════════════════════
"""


# ============================================================================
# HELPER FUNCTIONS - Build context for agents
# ============================================================================

def build_mission_context(state: MissionState) -> str:
    """Build a comprehensive context string for agents to reason about."""
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    dist_matrix = state.get("distance_matrix", {})
    excluded_targets = set(state.get("excluded_targets", []))  # Targets inside SAM zones

    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    # Filter targets to only show accessible ones (not inside SAM zones)
    accessible_targets = [t for t in targets if t.get("id", t.get("label", "?")) not in excluded_targets]

    lines = [
        "=" * 60,
        "MISSION CONTEXT",
        "=" * 60,
        "",
        "ENVIRONMENT:",
        f"  Airports: {len(airports)}",
    ]

    for ap in airports:
        ap_id = ap.get("id", ap.get("label", "?"))
        lines.append(f"    {ap_id}: pos=({ap.get('x', 0):.0f}, {ap.get('y', 0):.0f})")

    # Show excluded targets prominently if any exist
    if excluded_targets:
        lines.append(f"\n  ⚠️  EXCLUDED TARGETS (inside SAM zones - DO NOT ALLOCATE): {len(excluded_targets)}")
        lines.append(f"      {', '.join(sorted(excluded_targets))}")

    lines.append(f"\n  ACCESSIBLE Targets: {len(accessible_targets)} (of {len(targets)} total)")
    total_priority = 0
    for t in accessible_targets:
        t_id = t.get("id", t.get("label", "?"))
        priority = t.get("priority", t.get("value", 5))
        total_priority += priority
        lines.append(f"    {t_id}: priority={priority}, pos=({t.get('x', 0):.0f}, {t.get('y', 0):.0f})")

    lines.append(f"  Total Priority: {total_priority}")

    if sams:
        lines.append(f"\n  SAMs/NFZs: {len(sams)}")
        for i, sam in enumerate(sams):
            pos = sam.get("pos", [0, 0])
            lines.append(f"    SAM{i+1}: range={sam.get('range', 0)}, pos=({pos[0]:.0f}, {pos[1]:.0f})")

    lines.append("\nDRONE CONFIGURATIONS:")
    if not configs:
        lines.append("  (No drone configurations)")
    else:
        # Build a map of target types for eligibility calculation
        target_type_map = {}
        for t in accessible_targets:
            t_id = t.get("id", t.get("label", "?"))
            t_type = str(t.get("type", "a")).upper()
            target_type_map[t_id] = t_type

        for did in sorted(configs.keys()):
            cfg = configs[did]
            enabled = cfg.get("enabled", did == "1")
            if not enabled:
                continue
            fuel = cfg.get("fuelBudget", cfg.get("fuel_budget", 200))
            airport = cfg.get("homeAirport", cfg.get("home_airport", "A1"))

            # Get allowed target types for this drone
            allowed_types = cfg.get("accessibleTargets", cfg.get("accessible_targets", []))
            if allowed_types:
                allowed_types_upper = {t.upper() for t in allowed_types}
            else:
                allowed_types_upper = None  # None means ALL types allowed

            # Calculate which specific targets this drone can visit
            if allowed_types_upper is None:
                eligible_targets = list(target_type_map.keys())
            else:
                eligible_targets = [
                    tid for tid, ttype in target_type_map.items()
                    if ttype in allowed_types_upper
                ]

            lines.append(f"  D{did}:")
            lines.append(f"    Home: {airport}, Fuel: {fuel}")
            if allowed_types_upper:
                lines.append(f"    Sensor types: {', '.join(sorted(allowed_types_upper))}")
                lines.append(f"    ⚠️  ELIGIBLE TARGETS (ONLY THESE): {', '.join(sorted(eligible_targets))}")
            else:
                lines.append(f"    Sensor types: ALL")
                lines.append(f"    Eligible targets: ALL ({len(eligible_targets)} targets)")

    # Add distance info if available
    if dist_matrix:
        lines.append("\nDISTANCE MATRIX: Available (SAM-aware paths computed)")

    # Add policy rules if any (Learning v1)
    policy_rules = state.get("policy_rules", [])
    if policy_rules:
        lines.append(f"\nACTIVE POLICY RULES ({len(policy_rules)}):")
        for rule in policy_rules[:5]:  # Show first 5 rules
            title = rule.get("title", "Untitled")
            category = rule.get("category", "unknown")
            lines.append(f"  - [{category}] {title}")
        if len(policy_rules) > 5:
            lines.append(f"  ... and {len(policy_rules) - 5} more rules")

    lines.append("=" * 60)
    return "\n".join(lines)

def compute_mission_metrics(
    env: Dict[str, Any],
    routes: Dict[str, Any],
    allocations: Dict[str, List[str]],
    drone_configs: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Compute numerical mission metrics for the current solution.

    Returns:
        {
            "per_drone": {
                "1": {
                    "route": [...],
                    "fuel_used": float,
                    "fuel_budget": float,
                    "fuel_margin": float,
                    "points": int,
                    "visited_targets": ["T1", "T3", ...],
                },
                "2": { ... },
                ...
            },
            "total_fuel": float,
            "total_points": int,
            "visited_targets": ["T1", "T2", ...],
            "unvisited_targets": ["T5", ...],
        }
    """
    targets_by_id = {t["id"]: t for t in env.get("targets", [])}
    all_target_ids = set(targets_by_id.keys())

    per_drone: Dict[str, Any] = {}
    total_fuel = 0.0
    total_points = 0
    visited_targets: set[str] = set()

    for did_raw, route_info in (routes or {}).items():
        did = str(did_raw)

        if isinstance(route_info, dict):
            route = route_info.get("route") or []
            # HK wrapper may use "total_points" or "points"
            points = int(route_info.get("total_points", route_info.get("points", 0)) or 0)
            distance = float(route_info.get("distance", 0.0) or 0.0)
        else:
            # Fallback for legacy formats
            route = route_info or []
            points = 0
            distance = 0.0

        # Targets visited on this route (ignore airports)
        visited_on_route = [node for node in route if isinstance(node, str) and node.startswith("T")]
        visited_targets.update(visited_on_route)

        fuel_used = distance
        cfg = drone_configs.get(did) or drone_configs.get(int(did)) or {}
        fuel_budget = float(cfg.get("fuel_budget", fuel_used))
        fuel_margin = fuel_budget - fuel_used

        per_drone[did] = {
            "route": route,
            "fuel_used": fuel_used,
            "fuel_budget": fuel_budget,
            "fuel_margin": fuel_margin,
            "points": points,
            "visited_targets": visited_on_route,
        }

        total_fuel += fuel_used
        total_points += points

    unvisited_targets = sorted(all_target_ids - visited_targets)

    return {
        "per_drone": per_drone,
        "total_fuel": total_fuel,
        "total_points": total_points,
        "visited_targets": sorted(visited_targets),
        "unvisited_targets": unvisited_targets,
    }

def build_current_solution_context(state: Dict[str, Any]) -> str:
    """
    Build a textual summary of the CURRENT solution for the LLM.

    Uses mission_metrics when available (fuel, margins, points, unvisited).
    """
    routes = state.get("routes") or {}
    allocation = state.get("allocation") or {}
    metrics = state.get("mission_metrics") or {}

    if not routes:
        return "CURRENT SOLUTION: None computed yet.\n\n"

    lines: List[str] = []
    lines.append("CURRENT SOLUTION SUMMARY:")

    total_points = metrics.get("total_points")
    total_fuel = metrics.get("total_fuel")
    per_drone = metrics.get("per_drone") or {}
    unvisited = metrics.get("unvisited_targets") or []

    # Totals
    if total_points is not None and total_fuel is not None:
        lines.append(f"- Total points collected: {total_points}")
        lines.append(f"- Total fuel used: {total_fuel:.1f}")
    else:
        # Fallback if metrics missing
        lines.append("- Total points collected: (not available)")
        lines.append("- Total fuel used: (not available)")

    # Unvisited
    if unvisited:
        lines.append(f"- Unvisited targets: {', '.join(unvisited)}")
    else:
        lines.append("- All targets are visited.")

    # Per-drone metrics
    if per_drone:
        lines.append("")
        lines.append("PER-DRONE BREAKDOWN:")
        # Sort by drone id numerically when possible
        for did in sorted(per_drone.keys(), key=lambda d: int(d)):
            d = per_drone[did]
            fuel_used = d.get("fuel_used", 0.0) or 0.0
            fuel_budget = d.get("fuel_budget", 0.0) or 0.0
            fuel_margin = d.get("fuel_margin", 0.0)
            points = d.get("points", 0) or 0
            n_targets = len(d.get("visited_targets", []))

            lines.append(
                f"- D{did}: fuel_used={fuel_used:.1f}, "
                f"fuel_budget={fuel_budget:.1f}, "
                f"margin={fuel_margin:.1f}, "
                f"points={points}, targets={n_targets}"
            )

    # Allocation summary (optional, but helpful)
    if allocation:
        lines.append("")
        lines.append("ALLOCATION SUMMARY (targets per drone):")
        for did in sorted(allocation.keys(), key=lambda d: int(d)):
            tlist = allocation[did]
            lines.append(f"- D{did}: {len(tlist)} targets → {', '.join(tlist)}")

    lines.append("")  # final blank line
    return "\n".join(lines)

def compute_mission_metrics(
    env: Dict[str, Any],
    routes: Dict[str, Any],
    allocations: Dict[str, List[str]],
    drone_configs: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Compute numerical mission metrics:

    - per-drone fuel usage and margins
    - per-drone points collected
    - total fuel / points
    - visited vs unvisited targets
    """
    targets = {t["id"]: t for t in env.get("targets", [])}
    all_target_ids = set(targets.keys())

    per_drone = {}
    total_fuel = 0.0
    total_points = 0
    visited_targets = set()

    for did, route_info in (routes or {}).items():
        # v4 routes normally look like: {"route": [...], "distance": float, "total_points": int}
        if isinstance(route_info, dict):
            route = route_info.get("route") or []
            distance = float(route_info.get("distance", 0.0))
            # Check both "total_points" (v4 standard) and "points" (legacy)
            points = int(route_info.get("total_points", route_info.get("points", 0)))
        else:
            # Fallback if some legacy format leaks through
            route = route_info or []
            distance = 0.0
            points = 0

        # Determine visited targets from route (skip airports)
        visited_on_route = [node for node in route if node.startswith("T")]
        visited_targets.update(visited_on_route)

        # Fuel usage = distance (1:1 by design)
        fuel_used = distance

        # Fuel budget and margin
        cfg = drone_configs.get(str(did)) or drone_configs.get(did) or {}
        budget = float(cfg.get("fuel_budget", distance))
        margin = budget - fuel_used

        per_drone[did] = {
            "route": route,
            "fuel_used": fuel_used,
            "fuel_budget": budget,
            "fuel_margin": margin,
            "points": points,
            "visited_targets": visited_on_route,
        }

        total_fuel += fuel_used
        total_points += points

    unvisited_targets = sorted(all_target_ids - visited_targets)

    return {
        "per_drone": per_drone,
        "total_fuel": total_fuel,
        "total_points": total_points,
        "visited_targets": sorted(visited_targets),
        "unvisited_targets": unvisited_targets,
    }


# ============================================================================
# STRATEGIST AGENT - Analyzes request, determines approach
# ============================================================================

STRATEGIST_PROMPT = """
You are the v4 ISR Mission Strategist in a multi-agent planning system.

You do NOT generate routes yourself. Your job is to:
- Understand the user's request.
- Decide whether this is:
  - a QUESTION about an existing mission solution,
  - a request to OPTIMIZE or RECOMPUTE the mission,
  - a request to REALLOCATE targets between drones,
  - a request to REROUTE without changing allocation,
  - or a DIAGNOSTIC / EVALUATION request.
- Respect hard constraints in the user text (e.g., "do not recompute routes", "keep allocations fixed").
- Decide which downstream agents/tools should act (Allocator, RouteOptimizer, Critic, Responder).

You have access to:
- MISSION CONTEXT (airports, targets, SAMs, drone configs).
- CURRENT SOLUTION SUMMARY and MISSION METRICS, including:
  - per-drone fuel_used, fuel_budget, fuel_margin,
  - per-drone points and target counts,
  - total_points and total_fuel,
  - list of unvisited_targets.

You MUST use these metrics when reasoning:
- If the user asks "which drone uses the most fuel", "what is the total fuel", or similar:
  - This is a QUESTION. Do NOT trigger re-optimization.
- If the user asks to "generate", "compute", "recompute", "optimize", or "find a better/cheaper/more efficient plan":
  - This is an OPTIMIZATION request.
- If the user asks to "reassign targets", "balance workloads", "move T19 to D4", etc.:
  - This is a REALLOCATION request.
- If the user asks to "adjust routes" or "shorten detours" while keeping allocation fixed:
  - This is a REROUTE request.
- If the user only wants a critique, explanation, or numbers:
  - This is a QUESTION / EVALUATION request.

CRITICAL CONSTRAINT:
- If the user explicitly says **"do not recompute routes"**, you MUST:
  - Treat this as a pure QUESTION/EVALUATION, even if they mention optimization.
  - Set REQUEST_TYPE to `question`.
  - Set MODE to `answer_question`.
  - Ensure downstream behavior does NOT change routes or allocation.

You MUST reply in the following STRICT format:

REQUEST_TYPE: <one of: question | optimize | reallocate | reroute | evaluate | debug>
MODE: <one of: answer_question | optimize_freely | suggest_reallocation | suggest_reroute | analyze_solution>
COMMANDS_DETECTED: <short comma-separated list of interpreted user commands, or "none">
CONSTRAINTS_SUMMARY: <one or two sentences summarizing key constraints from the user text>
NEEDS_ALLOCATION: <true or false>   # true if Allocator should run or re-run
NEEDS_ROUTING: <true or false>      # true if RouteOptimizer should run or re-run
FEASIBILITY: <feasible | infeasible | unknown>
RATIONALE:
- <bullet 1 citing specific metrics or facts from the mission context>
- <bullet 2 citing specific metrics or facts from the mission context>
- <bullet 3, etc.>

Guidelines:
- When REQUEST_TYPE is `question`, MODE must be `answer_question`, and both NEEDS_ALLOCATION and NEEDS_ROUTING must be false.
- Always ground your rationale in NUMBERS from the mission metrics when available (fuel_used, total_fuel, margins, unvisited_targets, points).
- Never say that "no solution is computed" if routes exist; instead, describe the existing solution using the metrics.
"""


def strategist_node(state: MissionState) -> Dict[str, Any]:
    """Strategist analyzes the request and determines approach."""
    print("\n🧠 [STRATEGIST] Analyzing request...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    # Check if Coordinator flagged this as explain_only (skip to responder)
    policy = state.get("policy") or {}
    if policy.get("explain_only"):
        intent = state.get("intent", "explain")
        print(f"🎯 [STRATEGIST] Coordinator set explain_only=True (intent={intent}), routing to responder", file=sys.stderr)
        return {
            "messages": [AIMessage(content=f"[STRATEGIST] Explain/debug mode - routing to responder")],
            "strategy_analysis": f"Coordinator classified intent as '{intent}'. Skipping allocation/routing.",
            "request_type": "question",  # Forces routing to responder
        }

    user_request = state.get("user_request", "")
    print(f"📝 [STRATEGIST] User request: '{user_request}'", file=sys.stderr)
    sys.stderr.flush()

    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    mission_context = build_mission_context(state)
    current_solution = build_current_solution_context(state)

    # Build prompt with full context
    analysis_prompt = f"""
{HARD_CONSTRAINTS}

{STRATEGIST_PROMPT}

{mission_context}

{current_solution}

USER REQUEST: "{user_request}"

Analyze this request and provide your strategic assessment.
"""

    messages = [HumanMessage(content=analysis_prompt)]

    # Attempt LLM call with error handling
    analysis = None
    llm_success = False
    error_message = None

    try:
        print(f"🌐 [STRATEGIST] Calling Anthropic API...", file=sys.stderr)
        sys.stderr.flush()

        response = llm.invoke(messages)

        print(f"✅ [STRATEGIST] API call successful", file=sys.stderr)
        sys.stderr.flush()

        analysis = response.content
        llm_success = True

        # Log first 200 chars of response for debugging
        preview = analysis[:200] if len(analysis) > 200 else analysis
        print(f"📄 [STRATEGIST] Response preview: {preview}...", file=sys.stderr)
        sys.stderr.flush()

    except Exception as e:
        error_message = str(e)
        print(f"❌ [STRATEGIST] API call FAILED: {error_message}", file=sys.stderr)
        sys.stderr.flush()

        # Fallback analysis if LLM fails
        analysis = f"""API call failed: {error_message}

REQUEST_TYPE: optimize

Defaulting to optimization mode due to API failure.
"""

    print(f"📋 [STRATEGIST] Analysis complete (LLM success: {llm_success})", file=sys.stderr)
    sys.stderr.flush()

    # Parse request type from analysis
    request_type = "optimize"  # default
    analysis_lower = analysis.lower()
    if "request_type: question" in analysis_lower:
        request_type = "question"
        print(f"🔍 [STRATEGIST] Detected REQUEST_TYPE: question", file=sys.stderr)
    elif "request_type: command" in analysis_lower:
        request_type = "command"
        print(f"🔍 [STRATEGIST] Detected REQUEST_TYPE: command", file=sys.stderr)
    else:
        print(f"⚠️  [STRATEGIST] No REQUEST_TYPE found in response, defaulting to: optimize", file=sys.stderr)

    sys.stderr.flush()

    return {
        "messages": [AIMessage(content=f"[STRATEGIST]\n{analysis}")],
        "strategy_analysis": analysis,
        "request_type": request_type,
        "llm_success": llm_success,
        "llm_error": error_message,
    }


# ============================================================================
# ALLOCATOR AGENT - Reasons about target assignments
# ============================================================================

ALLOCATOR_PROMPT = """You are the ALLOCATOR agent in an ISR mission planning system.

ROLE
Allocate a CANDIDATE SET of targets to drones to maximize total achievable priority points
under fuel budgets, while respecting ALL HARD CONSTRAINTS.

HARD CONSTRAINTS (absolute)
- A drone may ONLY be assigned targets from its ELIGIBLE TARGETS list.
- Targets in SAM zones are INELIGIBLE in hard-v1 mode (treat as excluded).
- Respect user forbiddances (e.g., forbidden priorities, forbidden airports).
- Each target may be assigned to at most one drone.

IMPORTANT
- Do NOT assign every accessible target. Downselect candidates to a manageable set.
- The exact orienteering solver performs best up to ~12 targets per drone. Prefer <= 12 candidates per drone.
- If you exclude a target that is eligible, you MUST provide a reason code.

AVAILABLE STRATEGIES (choose one)
- efficient: maximize priority/fuel ratio (auction style)
- greedy: highest priority to nearest capable drone
- balanced: distribute workload evenly
- geographic: minimize detours / corridor fit
- exclusive: prioritize targets only one drone can visit

ALLOCATION APPROACH
1) Start from eligible targets per drone.
2) Build a candidate list per drone (<= 12 preferred) that maximizes points with good fuel efficiency.
3) Assign each candidate target to exactly one best drone.
4) Produce an explicit excluded list with reason codes.

OUTPUT (MUST BE VALID JSON)
Return a single JSON object with keys:
- strategy_used: "efficient"|"greedy"|"balanced"|"geographic"|"exclusive"
- assignments: { "D1": ["T..."], "D2": [...], ... }
- excluded: [
    { "target": "T..", "reason": "IN_SAM_ZONE|TYPE_NOT_ACCESSIBLE|FORBIDDEN_PRIORITY|CANDIDATE_LIMIT|DOMINATED_LOW_VALUE", "notes": "optional" }
  ]
- rationale: {
    "D1": "brief reason",
    "D2": "brief reason"
  }
- tradeoffs: "brief summary"

Be concise but explicit.
"""


def allocator_node(state: MissionState) -> Dict[str, Any]:
    """Allocator reasons about and performs target allocation."""
    print("\n🎯 [ALLOCATOR] Reasoning about allocation...", file=sys.stderr)
    sys.stderr.flush()

    # CRITICAL: Clear any cached distance matrix from previous solves
    # This ensures we use fresh distances from the current environment
    clear_allocator_matrix()

    set_state(state)

    # Read Coordinator policy
    policy = state.get("policy") or {}
    force_algo = bool(policy.get("force_algorithmic_allocation", False))
    policy_strategy = policy.get("allocation_strategy", "efficient")
    use_existing = bool(policy.get("use_existing_allocation", False))
    allocation_mods = policy.get("allocation_modifications", [])

    print(f"🎯 [ALLOCATOR] Policy: force_algo={force_algo}, strategy={policy_strategy}", file=sys.stderr)

    # Priority 1: If use_existing_allocation is True, apply modifications to existing allocation
    if use_existing and allocation_mods:
        print(f"🔄 [ALLOCATOR] Applying {len(allocation_mods)} modifications to existing allocation", file=sys.stderr)

        # Get existing allocation from state (seeded from existing_solution)
        existing_allocation = state.get("allocation") or {}

        if not existing_allocation:
            print(f"⚠️ [ALLOCATOR] No existing allocation found, falling back to algorithmic", file=sys.stderr)
        else:
            # Deep copy to avoid mutation
            new_allocation = {did: list(targets) for did, targets in existing_allocation.items()}

            # Apply each modification
            mods_applied = []
            for mod in allocation_mods:
                target_id = mod.get("target")
                to_drone = mod.get("to_drone")

                if not target_id or not to_drone:
                    continue

                # Find and remove target from its current drone
                from_drone = None
                for did, targets in new_allocation.items():
                    if target_id in targets:
                        targets.remove(target_id)
                        from_drone = did
                        break

                # Add to new drone
                if to_drone not in new_allocation:
                    new_allocation[to_drone] = []

                if target_id not in new_allocation[to_drone]:
                    new_allocation[to_drone].append(target_id)

                mods_applied.append(f"{target_id}: D{from_drone or '?'} → D{to_drone}")
                print(f"   ✅ Moved {target_id} from D{from_drone or '?'} to D{to_drone}", file=sys.stderr)

            reasoning = (
                f"[Allocation modification - use_existing_allocation=True]\n"
                f"Applied {len(mods_applied)} modifications:\n"
                + "\n".join(f"  - {m}" for m in mods_applied) + "\n"
                f"New allocation: {new_allocation}"
            )

            print(f"📋 [ALLOCATOR] Modified allocation: {new_allocation}", file=sys.stderr)

            return {
                "messages": [AIMessage(content=f"[ALLOCATOR]\n{reasoning}")],
                "allocation_reasoning": reasoning,
                "allocation": new_allocation,
                "allocation_strategy": "user_modified",
                "excluded_by_allocator": [],
            }

    # Priority 2: If force_algorithmic_allocation is True, skip LLM entirely
    if force_algo:
        print(f"🔧 [ALLOCATOR] Force algorithmic allocation enabled - skipping LLM", file=sys.stderr)
        env = state.get("environment", {})
        configs = state.get("drone_configs", {})
        dist_matrix = state.get("distance_matrix")

        # Build matrix_data for allocator
        matrix_data = None
        if dist_matrix:
            labels = list(dist_matrix.keys())
            matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
            matrix_data = {"labels": labels, "matrix": matrix}
            set_allocator_matrix(matrix_data)

        # Call algorithmic allocation with policy-specified strategy
        allocation = _allocate_targets_impl(env, configs, policy_strategy, matrix_data)
        strategy_used = policy_strategy
        excluded_targets: List[Dict[str, Any]] = []

        reasoning = (
            f"[Algorithmic allocation - force_algorithmic_allocation=True]\n"
            f"Strategy: {strategy_used}\n"
            f"Allocation: {allocation}"
        )

        print(f"📋 [ALLOCATOR] Algorithmic allocation complete: {allocation}", file=sys.stderr)

        return {
            "messages": [AIMessage(content=f"[ALLOCATOR]\n{reasoning}")],
            "allocation_reasoning": reasoning,
            "allocation": allocation,
            "allocation_strategy": strategy_used,
            "excluded_by_allocator": excluded_targets,
        }

    # Otherwise, use LLM-based allocation
    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    mission_context = build_mission_context(state)
    strategy_analysis = state.get("strategy_analysis", "No strategy analysis available")

    allocation_prompt = f"""
{HARD_CONSTRAINTS}

{ALLOCATOR_PROMPT}

{mission_context}

STRATEGIST'S ANALYSIS:
{strategy_analysis}

Based on this context, determine the optimal target allocation.
Return your response as a single valid JSON object (no markdown fences).
"""

    messages = [HumanMessage(content=allocation_prompt)]

    # Attempt LLM call with error handling
    reasoning = None
    llm_success = False
    error_message = None

    try:
        print(f"🌐 [ALLOCATOR] Calling Anthropic API...", file=sys.stderr)
        sys.stderr.flush()

        response = llm.invoke(messages)

        print(f"✅ [ALLOCATOR] API call successful", file=sys.stderr)
        sys.stderr.flush()

        reasoning = response.content
        llm_success = True

        # Log first 200 chars of response for debugging
        preview = reasoning[:200] if len(reasoning) > 200 else reasoning
        print(f"📄 [ALLOCATOR] Response preview: {preview}...", file=sys.stderr)
        sys.stderr.flush()

    except Exception as e:
        error_message = str(e)
        print(f"❌ [ALLOCATOR] API call FAILED: {error_message}", file=sys.stderr)
        sys.stderr.flush()

        # Fallback to algorithmic allocation if LLM fails
        reasoning = "API call failed. Using fallback algorithmic allocation."

    print(f"📋 [ALLOCATOR] Reasoning complete (LLM success: {llm_success})", file=sys.stderr)
    sys.stderr.flush()

    # Parse allocation from JSON response (uses policy_strategy in fallback)
    allocation, strategy_used, excluded_targets = parse_allocation_from_json(
        reasoning, state, fallback_strategy=policy_strategy
    )

    print(f"📋 [ALLOCATOR] Strategy used: {strategy_used}", file=sys.stderr)
    if excluded_targets:
        print(f"📋 [ALLOCATOR] Excluded targets: {len(excluded_targets)}", file=sys.stderr)

    return {
        "messages": [AIMessage(content=f"[ALLOCATOR]\n{reasoning}")],
        "allocation_reasoning": reasoning,
        "allocation": allocation,
        "allocation_strategy": strategy_used,
        "excluded_by_allocator": excluded_targets,
    }


def parse_allocation_from_json(
    reasoning: str,
    state: MissionState,
    fallback_strategy: str = "efficient",
) -> Tuple[Dict[str, List[str]], str, List[Dict[str, Any]]]:
    """
    Parse allocation from JSON response.

    Args:
        reasoning: LLM response text
        state: Current mission state
        fallback_strategy: Strategy to use if parsing fails (from Coordinator policy)

    Returns:
        (allocation, strategy_used, excluded_targets)
    """
    import json
    import re

    configs = state.get("drone_configs", {})
    allocation: Dict[str, List[str]] = {}

    # Initialize all enabled drones
    for did in configs.keys():
        if configs[did].get("enabled", did == "1"):
            allocation[did] = []

    strategy_used = "unknown"
    excluded_targets: List[Dict[str, Any]] = []

    # Try to extract JSON from the response
    try:
        # Remove markdown code fences if present
        json_str = reasoning.strip()
        if json_str.startswith("```"):
            # Remove opening fence
            json_str = re.sub(r"^```(?:json)?\s*\n?", "", json_str)
            # Remove closing fence
            json_str = re.sub(r"\n?```\s*$", "", json_str)

        # Find JSON object in the response
        match = re.search(r"\{[\s\S]*\}", json_str)
        if match:
            json_str = match.group(0)

        data = json.loads(json_str)

        # Extract strategy
        strategy_used = data.get("strategy_used", "unknown")

        # Extract assignments
        assignments = data.get("assignments", {})
        for drone_key, targets in assignments.items():
            # Normalize drone ID (remove "D" prefix if present)
            did = str(drone_key).replace("D", "").strip()
            if did in allocation:
                allocation[did] = [str(t) for t in targets] if targets else []

        # Extract excluded targets
        excluded_targets = data.get("excluded", [])

        print(f"📋 [ALLOCATOR] Successfully parsed JSON allocation", file=sys.stderr)

    except (json.JSONDecodeError, KeyError, TypeError) as e:
        print(f"⚠️ [ALLOCATOR] JSON parsing failed: {e}, falling back to text parsing", file=sys.stderr)
        # Fall back to legacy text parsing
        allocation = parse_allocation_from_reasoning(reasoning, state)
        # Try to extract strategy from text
        for line in reasoning.split("\n"):
            if "strategy_used" in line.lower() or "STRATEGY_USED:" in line.upper():
                for strat in ["efficient", "greedy", "balanced", "geographic", "exclusive"]:
                    if strat in line.lower():
                        strategy_used = strat
                        break

    # Log what we parsed
    print(f"📋 [ALLOCATOR] Parsed allocation: {allocation}", file=sys.stderr)
    total_allocated = sum(len(targets) for targets in allocation.values())
    print(f"📋 [ALLOCATOR] Total targets allocated: {total_allocated}", file=sys.stderr)

    # If parsing failed completely, fall back to algorithmic allocation
    # Use fallback_strategy from Coordinator (not hardcoded "efficient")
    if all(len(v) == 0 for v in allocation.values()):
        print(f"⚠️ [ALLOCATOR] Parsing failed, using fallback allocation (strategy={fallback_strategy})", file=sys.stderr)
        env = state.get("environment", {})
        dist_matrix = state.get("distance_matrix")

        matrix_data = None
        if dist_matrix:
            labels = list(dist_matrix.keys())
            matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
            matrix_data = {"labels": labels, "matrix": matrix}
            set_allocator_matrix(matrix_data)

        allocation = _allocate_targets_impl(env, configs, fallback_strategy, matrix_data)
        strategy_used = fallback_strategy
        print(f"📋 [ALLOCATOR] Fallback allocation: {allocation}", file=sys.stderr)

    return allocation, strategy_used, excluded_targets


def parse_allocation_from_reasoning(reasoning: str, state: MissionState) -> Dict[str, List[str]]:
    """Parse allocation from LLM reasoning output (legacy text format)."""
    allocation = {}
    configs = state.get("drone_configs", {})

    # Initialize all enabled drones
    for did in configs.keys():
        if configs[did].get("enabled", did == "1"):
            allocation[did] = []

    # Look for ALLOCATION_RESULT section
    lines = reasoning.split("\n")
    in_result = False

    for line in lines:
        line = line.strip()
        if "ALLOCATION_RESULT" in line.upper():
            in_result = True
            continue
        if in_result and line.startswith("D"):
            # Parse "D1: T1, T3, T5" format
            parts = line.split(":")
            if len(parts) >= 2:
                did = parts[0].strip().replace("D", "")
                targets_str = parts[1].strip()
                if targets_str and targets_str.lower() not in ["none", "(none)", ""]:
                    targets = [t.strip() for t in targets_str.split(",") if t.strip()]
                    if did in allocation:
                        allocation[did] = targets
        elif in_result and line.startswith("TRADE") or line.startswith("==="):
            in_result = False

    return allocation


# ============================================================================
# ROUTE OPTIMIZER AGENT - Computes routes, checks feasibility
# ============================================================================

ROUTE_OPTIMIZER_PROMPT = """You are the ROUTE OPTIMIZER agent in an ISR mission planning system.

Your job is to compute optimal routes for each drone given their allocated targets.
You use the Held-Karp algorithm (optimal TSP solver) for route computation.

For each drone, you must:
1. Compute the shortest route visiting all allocated targets
2. Verify the route fits within fuel budget
3. If infeasible, explain why and what would need to change

OUTPUT FORMAT:
ROUTE_ANALYSIS:
- D1: [route] uses [X] fuel (budget: [Y]) - [feasible/infeasible]
  Reason: [why this route was chosen]
- D2: ...

FEASIBILITY_ISSUES:
[List any routes that exceed fuel budget and why]

ROUTES_COMPUTED: YES/NO
"""


def route_optimizer_node(state: MissionState) -> Dict[str, Any]:
    """Route optimizer computes and validates routes."""
    import time as _time
    solver_start = _time.time()

    print("\n🛣️ [ROUTE_OPTIMIZER] Computing routes...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    allocation = state.get("allocation", {})
    dist_matrix = state.get("distance_matrix", {})

    # Read Coordinator policy for solver_mode override
    policy = state.get("policy") or {}
    policy_solver_mode = policy.get("solver_mode")  # May override per-drone mode

    # Read drone contracts from state (set by Coordinator pre-pass)
    # Fallback to policy.drone_contracts for backwards compatibility
    drone_contracts = state.get("drone_contracts") or policy.get("drone_contracts", {})

    # Source synthetic starts from environment (authoritative data)
    # NOT from guardrails (which are for validation outcomes only)
    synthetic_starts = env.get("synthetic_starts", {}) or {}

    # Build real airport IDs set for defensive home_airport validation
    # A real airport must:
    # 1. NOT have is_synthetic=True flag
    # 2. NOT have an ID ending in _START (synthetic start pattern)
    real_airport_ids: set[str] = set()
    for a in (env.get("airports") or []):
        aid = str(a.get("id", a.get("label", "")))
        # Skip if marked as synthetic
        if a.get("is_synthetic", False):
            continue
        # Skip if ID matches synthetic start pattern (e.g., D1_START, D2_START)
        if aid.endswith("_START"):
            continue
        real_airport_ids.add(aid)

    routes = {}
    route_analysis_lines = ["ROUTE_ANALYSIS:"]
    feasibility_issues = []

    # Get solver instance
    solver = None
    if OrienteeringSolverInterface:
        solver = OrienteeringSolverInterface()

    for did, target_ids in allocation.items():
        cfg = configs.get(did, {})
        if not cfg.get("enabled", did == "1"):
            continue

        fuel_budget = cfg.get("fuelBudget", cfg.get("fuel_budget", 200))

        # Get drone contract from Coordinator (handles synthetic starts correctly)
        contract = drone_contracts.get(str(did), {})

        # start_id: Coordinator contract is authoritative, then cfg.start_id, then home_airport
        # Avoid cfg.start_airport to prevent older UI mismatch issues
        start_id = (
            contract.get("start_id")
            or cfg.get("start_id")
            or cfg.get("home_airport")
            or "A1"
        )

        is_synthetic_start = contract.get(
            "is_synthetic_start",
            start_id.endswith("_START") if start_id else False
        )

        # home_airport: MUST be a real airport (never synthetic)
        # Defensive enforcement even if upstream misconfigures
        home_airport = contract.get("home_airport") or cfg.get("home_airport")
        if not home_airport or str(home_airport) not in real_airport_ids:
            home_airport = start_id if start_id in real_airport_ids else next(iter(sorted(real_airport_ids)), "A1")

        # end_airport from config (contract may have None for flexible)
        cfg_end = cfg.get("end_airport", home_airport)
        flexible_endpoint = contract.get("flexible_endpoint", cfg_end == "-")

        # Determine solver mode
        if policy_solver_mode:
            mode = policy_solver_mode
        elif flexible_endpoint:
            mode = "best_end"
        elif cfg_end and cfg_end != "-" and cfg_end != start_id:
            mode = "end"
        else:
            mode = "return"

        # For synthetic starts with return mode, we need best_end (can't return to synthetic)
        if is_synthetic_start and mode == "return":
            mode = "best_end"
            print(f"[v4][ROUTE_OPT] D{did}: Synthetic start detected, switching mode to best_end", flush=True)

        end_airport = cfg_end if mode == "end" else None

        print(f"[v4][ROUTE_OPT] D{did} start_id={start_id} home={home_airport} end={end_airport} "
              f"synthetic={is_synthetic_start} flexible={flexible_endpoint} mode={mode}", flush=True)

        if not target_ids:
            # No targets: for synthetic starts, need to go to nearest real airport
            effective_end = home_airport if is_synthetic_start else start_id
            routes[did] = {
                "route": [start_id, effective_end] if start_id != effective_end else [start_id],
                "distance": 0,
                "total_points": 0,
                "visited_targets": [],
            }
            route_analysis_lines.append(f"- D{did}: No targets assigned, goes from {start_id} to {effective_end}")
            continue

        # Build solver environment
        if solver and dist_matrix:
            try:
                # --- Build FILTERED matrix data ---
                all_labels = list(dist_matrix.keys())

                # Build requested labels based on mode
                # Always include start_id (may be synthetic)
                if flexible_endpoint or mode == "best_end":
                    # For best_end mode, include ALL real airports so solver can choose optimal end
                    # Filter out synthetic airports from endpoint candidates
                    real_airport_ids = [
                        a["id"] for a in env.get("airports", [])
                        if not a.get("is_synthetic", False)
                    ]
                    requested_labels = [start_id] + real_airport_ids + list(target_ids)
                else:
                    # For fixed end, include start, end, and targets
                    requested_labels = [start_id]
                    if end_airport and end_airport != start_id:
                        requested_labels.append(end_airport)
                    requested_labels.extend(target_ids)

                # Keep only labels that actually exist in the distance matrix
                labels = [lab for lab in requested_labels if lab in all_labels]
                # Deduplicate while preserving order
                seen = set()
                labels = [x for x in labels if not (x in seen or seen.add(x))]

                if not labels or start_id not in labels:
                    print(
                        f"⚠️ [ROUTE_OPTIMIZER] No valid labels for D{did} "
                        f"(start_id={start_id}, targets={target_ids})",
                        file=sys.stderr,
                    )
                    routes[did] = {
                        "route": [start_id],
                        "distance": 0.0,
                        "total_points": 0,
                        "visited_targets": [],
                    }
                    continue

                # Build submatrix only over these labels
                matrix = [
                    [dist_matrix[r].get(c, 1000) for c in labels]
                    for r in labels
                ]

                # Filter targets to only those allocated (env target dicts)
                targets = [
                    t for t in env.get("targets", [])
                    if str(t.get("id", t.get("label"))) in target_ids
                ]

                # Build airports list - include synthetic starts as airports
                solver_airports = list(env.get("airports", []))
                if is_synthetic_start and start_id not in [a["id"] for a in solver_airports]:
                    # Add synthetic start as an airport
                    synth_coords = synthetic_starts.get(start_id, {"x": 0, "y": 0})
                    solver_airports.append({
                        "id": start_id,
                        "x": synth_coords.get("x", 0),
                        "y": synth_coords.get("y", 0),
                        "is_synthetic": True,
                    })
                    print(f"[v4][ROUTE_OPT] Added synthetic start {start_id} to airports", flush=True)

                print(f"[v4][ROUTE_OPT] D{did} start_id={start_id} end={end_airport} mode={mode} "
                      f"labels={labels[:5]}{'...' if len(labels) > 5 else ''}", flush=True)

                solver_env = {
                    "airports": solver_airports,
                    "targets": targets,
                    "matrix": matrix,
                    "matrix_labels": labels,
                    "start_airport": start_id,  # Use start_id (may be synthetic)
                    "mode": mode,
                }

                # For best_end mode, set valid_end_airports to real airports only
                # CRITICAL: Synthetic starts (e.g., D1_START, D2_START) must NEVER be endpoints
                if mode == "best_end":
                    valid_end_airports_for_solver = [
                        a["id"] for a in env.get("airports", [])
                        if not a.get("is_synthetic", False)
                        and not str(a.get("id", "")).endswith("_START")
                    ]
                    solver_env["valid_end_airports"] = valid_end_airports_for_solver
                    print(f"   ✈️ [v4] Valid end airports (real only): {valid_end_airports_for_solver}", flush=True)

                # Only set end_airport if fixed end mode
                if mode == "end" and end_airport:
                    solver_env["end_airport"] = end_airport

                solution = solver.solve(solver_env, fuel_budget)

                # Extract the actual end airport from solution (solver may have chosen it)
                actual_end = solution.get("end_airport", end_airport if not flexible_endpoint else home_airport)
                if flexible_endpoint or mode == "best_end":
                    print(f"   🎯 [v4] Solver chose endpoint: {actual_end}", flush=True)

                route = solution.get("route", [start_id])
                distance = solution.get("distance", 0)
                points = solution.get("total_points", 0)
                visited = solution.get("visited_targets", [])

                routes[did] = {
                    "route": route,
                    "distance": distance,
                    "total_points": points,
                    "visited_targets": visited,
                    "start_id": start_id,  # Track start for trajectory generation
                    "end_airport": actual_end,
                    "is_synthetic_start": is_synthetic_start,
                }

                status = "FEASIBLE" if distance <= fuel_budget else "OVER BUDGET"
                route_analysis_lines.append(
                    f"- D{did}: {' → '.join(route)} uses {distance:.0f} fuel "
                    f"(budget: {fuel_budget}) - {status}"
                )

                if distance > fuel_budget:
                    feasibility_issues.append(
                        f"D{did} exceeds fuel by {distance - fuel_budget:.0f} units"
                    )

            except Exception as e:
                print(f"⚠️ [ROUTE_OPTIMIZER] Error solving D{did}: {e}", file=sys.stderr)
                # Fallback route uses start_id (may be synthetic)
                fallback_end = home_airport if is_synthetic_start else start_id
                routes[did] = {
                    "route": [start_id] + list(target_ids) + [fallback_end],
                    "distance": 999,
                    "total_points": 0,
                    "visited_targets": [],
                    "error": str(e),
                    "start_id": start_id,
                    "is_synthetic_start": is_synthetic_start,
                }
        else:
            # Fallback: simple route without optimization
            # Use start_id (may be synthetic) and go to home_airport
            fallback_end = home_airport if is_synthetic_start else start_id
            routes[did] = {
                "route": [start_id] + list(target_ids) + [fallback_end],
                "distance": 0,
                "total_points": sum(
                    t.get("priority", 5) for t in env.get("targets", [])
                    if str(t.get("id", t.get("label"))) in target_ids
                ),
                "visited_targets": list(target_ids),
                "start_id": start_id,
                "is_synthetic_start": is_synthetic_start,
            }

    route_analysis = "\n".join(route_analysis_lines)
    if feasibility_issues:
        route_analysis += "\n\nFEASIBILITY_ISSUES:\n" + "\n".join(feasibility_issues)
    route_analysis += f"\n\nROUTES_COMPUTED: YES"

    # Calculate solver runtime
    solver_runtime_ms = int((_time.time() - solver_start) * 1000)
    solver_type = "orienteering_exact" if solver else "heuristic"

    print(f"📋 [ROUTE_OPTIMIZER] Routes computed for {len(routes)} drones in {solver_runtime_ms}ms", file=sys.stderr)

    return {
        "messages": [AIMessage(content=f"[ROUTE_OPTIMIZER]\n{route_analysis}")],
        "route_analysis": route_analysis,
        "routes": routes,
        "solver_type": solver_type,
        "solver_runtime_ms": solver_runtime_ms,
    }


# ============================================================================
# CRITIC AGENT - Reviews solution, suggests improvements
# ============================================================================

CRITIC_PROMPT = """You are the CRITIC agent in an ISR mission planning system.

Your job is to review the solution and identify:
1. Any issues or violations (commands not followed, constraints violated)
2. Potential improvements (targets that could be reassigned for better efficiency)
3. Suggestions for the commander

Be CONCISE but THOROUGH. Focus on actionable insights.

OUTPUT FORMAT:
REVIEW:
[Overall assessment - is this a good solution?]

ISSUES:
[List any problems found, or "None"]

SUGGESTIONS:
[List improvement suggestions, e.g., "Allowing D2 to access T5 would add 15 points with only 20 extra fuel"]

FINAL_VERDICT: [APPROVED|NEEDS_REVISION|REJECTED]
"""


def critic_node(state: MissionState) -> Dict[str, Any]:
    """Critic reviews the solution and suggests improvements."""
    print("\n🔍 [CRITIC] Reviewing solution...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    mission_context = build_mission_context(state)
    allocation_reasoning = state.get("allocation_reasoning", "")
    route_analysis = state.get("route_analysis", "")
    current_solution = build_current_solution_context(state)

    critic_prompt = f"""
{HARD_CONSTRAINTS}

{CRITIC_PROMPT}

{mission_context}

ALLOCATION REASONING:
{allocation_reasoning}

ROUTE ANALYSIS:
{route_analysis}

{current_solution}

Review this solution and provide your assessment.
Flag any violations of HARD CONSTRAINTS (e.g., drone assigned targets it cannot visit).
"""

    messages = [HumanMessage(content=critic_prompt)]
    response = llm.invoke(messages)

    review = response.content
    print(f"📋 [CRITIC] Review complete", file=sys.stderr)

    # Extract suggestions
    suggestions = []
    lines = review.split("\n")
    in_suggestions = False
    for line in lines:
        if "SUGGESTIONS:" in line.upper():
            in_suggestions = True
            continue
        if in_suggestions and line.strip().startswith("-"):
            suggestions.append(line.strip()[1:].strip())
        elif in_suggestions and ("FINAL_VERDICT" in line.upper() or line.startswith("===")):
            in_suggestions = False

    return {
        "messages": [AIMessage(content=f"[CRITIC]\n{review}")],
        "critic_review": review,
        "suggestions": suggestions,
    }


# ============================================================================
# RESPONDER AGENT - Formulates final answer
# ============================================================================

RESPONDER_PROMPT = """
You are the ISR Mission Responder for the v4 multi-agent ISR planner.

Your role:
- Answer the user's questions using ONLY the stored mission context and the CURRENT SOLUTION SUMMARY.
- You never recompute routes unless explicitly instructed by the user (e.g., "recompute", "optimize", "rerun", "generate a new plan").

When a mission solution exists (routes found):
- ALWAYS cite specific **numbers** from the provided state:
  - fuel usage per drone
  - fuel budget and fuel margin (budget - used)
  - points collected per drone and total points
  - total fuel used across all drones
  - count of visited vs unvisited targets
  - list of unvisited targets, if any
- ALWAYS leverage the numeric metrics in `mission_metrics`.

When answering:
- Be **precise**, **concise**, and **numerical**.
- Prefer clear statements and bullet points over vague text.
- If the user asks a comparative or evaluative question, always compare using numeric values.
- If the mission has unvisited targets, describe them explicitly.
- Never generalize or use fluff language ("likely", "maybe", "probably") when numbers exist.

When NO solution exists yet (routes empty):
- Clearly state that no mission plan exists.
- Suggest generating a plan first.

Follow the user's constraints exactly (e.g., "do not recompute routes").
"""


def responder_node(state: MissionState) -> Dict[str, Any]:
    """Responder formulates the final answer."""
    print("\n📝 [RESPONDER] Formulating response...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    request_type = state.get("request_type", "optimize")

    # For questions, use simpler response
    if request_type == "question":
        return handle_question_response(state)

    # For solve/optimize, summarize the solution
    return handle_solution_response(state)


def handle_question_response(state: MissionState) -> Dict[str, Any]:
    """Handle question-type requests."""
    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    user_request = state.get("user_request", "")
    mission_context = build_mission_context(state)
    current_solution = build_current_solution_context(state)

    prompt = f"""
{HARD_CONSTRAINTS}

{RESPONDER_PROMPT}

{mission_context}

{current_solution}

USER QUESTION: "{user_request}"

Provide a clear, direct answer to this question.
"""

    messages = [HumanMessage(content=prompt)]
    response = llm.invoke(messages)

    return {
        "messages": [AIMessage(content=response.content)],
        "final_response": response.content,
    }


def handle_solution_response(state: MissionState) -> Dict[str, Any]:
    """Handle solution-type requests."""
    routes = state.get("routes", {})
    allocation = state.get("allocation", {})
    suggestions = state.get("suggestions", [])
    env = state.get("environment", {})
    allocation_reasoning = state.get("allocation_reasoning", "")

    # Build response
    lines = ["MISSION PLAN COMPUTED", "=" * 40, ""]

    total_points = 0
    total_fuel = 0
    all_visited = []

    for did in sorted(routes.keys()):
        route_data = routes[did]
        route = route_data.get("route", [])
        dist = route_data.get("distance", 0)
        points = route_data.get("total_points", 0)
        visited = route_data.get("visited_targets", [])

        total_points += points
        total_fuel += dist
        all_visited.extend(visited)

        lines.append(f"D{did}: {' → '.join(route)}")
        lines.append(f"    Fuel: {dist:.0f}, Points: {points}, Targets: {len(visited)}")

    lines.append("")
    lines.append(f"TOTALS: {total_points} points, {total_fuel:.0f} fuel")

    # Check for unvisited targets
    all_targets = [str(t.get("id", t.get("label"))) for t in env.get("targets", [])]
    unvisited = set(all_targets) - set(all_visited)
    if unvisited:
        lines.append(f"UNVISITED: {', '.join(sorted(unvisited))}")

    # Add key reasoning (extract from allocation reasoning)
    if allocation_reasoning and "TRADE_OFFS:" in allocation_reasoning:
        trade_off_start = allocation_reasoning.find("TRADE_OFFS:")
        trade_offs = allocation_reasoning[trade_off_start:trade_off_start+200].split("\n")[1:3]
        if trade_offs:
            lines.append("")
            lines.append("KEY TRADE-OFFS:")
            for t in trade_offs:
                if t.strip():
                    lines.append(f"  {t.strip()}")

    # Add suggestions if any
    if suggestions:
        lines.append("")
        lines.append("SUGGESTIONS:")
        for s in suggestions[:3]:  # Top 3 suggestions
            lines.append(f"  • {s}")

    response_text = "\n".join(lines)

    return {
        "messages": [AIMessage(content=response_text)],
        "final_response": response_text,
    }


# ============================================================================
# WORKFLOW BUILDER
# ============================================================================

def build_reasoning_workflow():
    """Build the v4 reasoning-based multi-agent workflow."""
    print("🔧 [v4] Building reasoning workflow...", file=sys.stderr)

    workflow = StateGraph(MissionState)

    # Add nodes
    workflow.add_node("strategist", strategist_node)
    workflow.add_node("allocator", allocator_node)
    workflow.add_node("route_optimizer", route_optimizer_node)
    workflow.add_node("critic", critic_node)
    workflow.add_node("responder", responder_node)

    # Routing function after strategist
    def after_strategist(state: MissionState) -> str:
        request_type = state.get("request_type", "optimize")
        if request_type == "question":
            return "responder"  # Skip solving for questions
        return "allocator"

    # Set entry point
    workflow.set_entry_point("strategist")

    # Add edges
    workflow.add_conditional_edges("strategist", after_strategist, {
        "responder": "responder",
        "allocator": "allocator",
    })

    workflow.add_edge("allocator", "route_optimizer")
    workflow.add_edge("route_optimizer", "critic")
    workflow.add_edge("critic", "responder")
    workflow.add_edge("responder", END)

    print("✅ [v4] Workflow built successfully", file=sys.stderr)
    return workflow.compile()


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def run_multi_agent_v4(
    user_message: str,
    environment: Dict[str, Any],
    drone_configs: Dict[str, Any],
    sam_matrix: Optional[Dict[str, Any]] = None,
    existing_solution: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Run the v4 reasoning-based multi-agent system.

    Args:
        user_message: The user's request
        environment: Environment data (airports, targets, sams)
        drone_configs: Drone configurations from the UI
        sam_matrix: Pre-computed SAM-aware distance matrix with metadata
                   (env_hash, routing_model_hash, distance_matrix_id)
        existing_solution: Previous routes/allocation for continuation

    Returns:
        Result dictionary with response, routes, trajectories, etc.
        Includes trace.env with matrix metadata for reproducibility.
    """
    print("### V4: run_multi_agent_v4 (isr_agent_multi_v4.py) CALLED ###", flush=True)
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"[v4] Processing: {user_message[:80]}...", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    sys.stderr.flush()

    # ------------------------------------------------------------------
    # 0) Load active policy rules (Learning v1)
    # ------------------------------------------------------------------
    policy_rules: List[Dict[str, Any]] = []
    if get_active_policy_rules is not None:
        try:
            policy_rules = get_active_policy_rules(mode="agentic")
            if policy_rules:
                print(f"📜 [v4] Loaded {len(policy_rules)} active policy rules", file=sys.stderr)
        except Exception as e:
            print(f"⚠️ [v4] Failed to load policy rules: {e}", file=sys.stderr)

    # ------------------------------------------------------------------
    # 1) Normalize inputs
    # ------------------------------------------------------------------
    env: Dict[str, Any] = environment or {}

    # UI-level configs passed in, or from env if not provided
    ui_configs: Dict[str, Any] = drone_configs or env.get("drone_configs") or {}

    # Translate UI configs into what v4 expects:
    # UI:  enabled, fuel_budget, start_airport, end_airport, target_access:{a..e}
    # v4:  enabled, fuelBudget, home_airport, accessible_targets:['A','B',...]
    normalized_configs: Dict[str, Dict[str, Any]] = {}

    # Build set of real airport IDs (exclude synthetic starts like D1_START)
    real_airport_ids: set[str] = set()
    for a in env.get("airports", []):
        if not a.get("is_synthetic", False):
            aid = a.get("id", a.get("label", ""))
            if aid:
                real_airport_ids.add(str(aid))
    default_real_airport = next(iter(sorted(real_airport_ids)), "A1")

    for did, cfg in ui_configs.items():
        if cfg is None:
            continue

        # Make a shallow copy so we don't mutate the original
        nc: Dict[str, Any] = dict(cfg)

        # Ensure ID is a string like "1", "2", ...
        did_str = str(did)

        # Fuel budget: accept both "fuelBudget" and "fuel_budget"
        fuel_budget = (
            cfg.get("fuelBudget")
            if cfg.get("fuelBudget") is not None
            else cfg.get("fuel_budget")
        )
        if fuel_budget is not None:
            nc["fuelBudget"] = fuel_budget

        # start_id: may be synthetic (D1_START) for checkpoint replanning
        cfg_start = cfg.get("start_airport") or cfg.get("startAirport") or ""
        nc["start_id"] = str(cfg_start) if cfg_start else default_real_airport

        # home_airport: MUST be a real airport (never synthetic)
        # Used for endpoint candidates in best_end mode
        cfg_home = cfg.get("homeAirport") or cfg.get("home_airport") or ""
        if cfg_home and cfg_home in real_airport_ids:
            nc["home_airport"] = str(cfg_home)
        elif cfg_start and cfg_start in real_airport_ids:
            nc["home_airport"] = str(cfg_start)
        else:
            nc["home_airport"] = default_real_airport

        # end_airport: explicit end, "-" for flexible, or None
        cfg_end = cfg.get("end_airport") or cfg.get("endAirport") or ""
        if cfg_end == "-":
            nc["end_airport"] = "-"  # Flexible endpoint
        elif cfg_end:
            nc["end_airport"] = str(cfg_end)
        else:
            nc["end_airport"] = nc["home_airport"]  # Default to home

        # Accessible targets: build list from target_access dict if present
        if "accessible_targets" not in nc and "accessibleTargets" not in nc:
            ta = cfg.get("target_access")
            if isinstance(ta, dict):
                accessible = [str(k).upper() for k, v in ta.items() if v]
                nc["accessible_targets"] = accessible

        normalized_configs[did_str] = nc

    # ------------------------------------------------------------------
    # 2) Build / compute distance matrix (so the HK solver can run)
    # ------------------------------------------------------------------
    # Store matrix metadata for trace (Wiring Point C)
    matrix_env_hash: Optional[str] = None
    matrix_routing_model_hash: Optional[str] = None
    matrix_distance_matrix_id: Optional[str] = None
    matrix_cache_hit: Optional[bool] = None
    matrix_routing_model: Optional[Dict[str, Any]] = None
    excluded_targets: List[str] = []  # Targets inside SAM zones

    dist_matrix: Dict[str, Dict[str, float]]

    if sam_matrix and sam_matrix.get("matrix"):
        # Use pre-computed SAM matrix from main.py (preferred path)
        print("✅ [v4] Using pre-computed SAM matrix from caller", file=sys.stderr)
        labels = sam_matrix.get("labels", [])
        matrix_data = sam_matrix.get("matrix", [])
        dist_matrix = {}
        for i, from_id in enumerate(labels):
            dist_matrix[from_id] = {}
            for j, to_id in enumerate(labels):
                dist_matrix[from_id][to_id] = float(matrix_data[i][j])

        # Extract metadata from sam_matrix
        matrix_env_hash = sam_matrix.get("env_hash")
        matrix_routing_model_hash = sam_matrix.get("routing_model_hash")
        matrix_distance_matrix_id = sam_matrix.get("distance_matrix_id")
        matrix_cache_hit = sam_matrix.get("cache_hit")
        matrix_routing_model = sam_matrix.get("routing_model")
        excluded_targets = sam_matrix.get("excluded_targets", [])

        if excluded_targets:
            print(f"🚫 [v4] Excluded targets (inside SAM zones): {excluded_targets}", file=sys.stderr)
    else:
        # Fallback: compute matrix locally (for backwards compatibility)
        sams = env.get("sams", [])
        try:
            if sams:
                print("🎯 [v4] Computing SAM-aware distance matrix (fallback)...", file=sys.stderr)
                dist_data = calculate_sam_aware_matrix(env)
                labels = dist_data.get("labels", [])
                matrix_data = dist_data.get("matrix", [])
                dist_matrix = {}
                for i, from_id in enumerate(labels):
                    dist_matrix[from_id] = {}
                    for j, to_id in enumerate(labels):
                        dist_matrix[from_id][to_id] = float(matrix_data[i][j])

                # Extract metadata from computed matrix
                matrix_env_hash = dist_data.get("env_hash")
                matrix_routing_model_hash = dist_data.get("routing_model_hash")
                matrix_distance_matrix_id = dist_data.get("distance_matrix_id")
                matrix_cache_hit = dist_data.get("cache_hit")
                matrix_routing_model = dist_data.get("routing_model")
                excluded_targets = dist_data.get("excluded_targets", [])

                if excluded_targets:
                    print(f"🚫 [v4] Excluded targets (inside SAM zones): {excluded_targets}", file=sys.stderr)
            else:
                print("📏 [v4] Computing Euclidean distance matrix...", file=sys.stderr)
                # Simple Euclidean fallback (no SAMs)
                waypoints: Dict[str, Tuple[float, float]] = {}

                for airport in env.get("airports", []):
                    aid = airport.get("id", airport.get("label", "A1"))
                    waypoints[str(aid)] = (
                        float(airport.get("x", 0.0)),
                        float(airport.get("y", 0.0)),
                    )

                for target in env.get("targets", []):
                    tid = target.get("id", target.get("label", "T?"))
                    waypoints[str(tid)] = (
                        float(target.get("x", 0.0)),
                        float(target.get("y", 0.0)),
                    )

                dist_matrix = {}
                for from_id, (x1, y1) in waypoints.items():
                    dist_matrix[from_id] = {}
                    for to_id, (x2, y2) in waypoints.items():
                        if from_id == to_id:
                            dist_matrix[from_id][to_id] = 0.0
                        else:
                            dist = math.hypot(x2 - x1, y2 - y1)
                            dist_matrix[from_id][to_id] = float(dist)
        except Exception as e:
            print(f"⚠️ [v4] Failed to compute distance matrix: {e}", file=sys.stderr)
            dist_matrix = {}

    # ------------------------------------------------------------------
    # 3) Run Coordinator pre-pass (deterministic policy/guardrail layer)
    # ------------------------------------------------------------------
    coordinator_decision: CoordinatorDecision = run_coordinator(
        user_message=user_message,
        environment=env,
        drone_configs=normalized_configs,
        sam_matrix=sam_matrix,
        ui_state=None,  # Could be passed from main.py if available
        preferences=None,  # Could be passed from main.py if available
        debug=True,
    )

    print(f"🎯 [v4] Coordinator decision: intent={coordinator_decision.intent}, "
          f"confidence={coordinator_decision.confidence:.2f}", file=sys.stderr)

    # If Coordinator found validation errors, return early
    if not coordinator_decision.is_valid:
        print(f"❌ [v4] Coordinator validation failed: {coordinator_decision.errors}", file=sys.stderr)
        return {
            "response": f"Validation error: {'; '.join(coordinator_decision.errors)}",
            "routes": {},
            "total_points": 0,
            "total_fuel": 0.0,
            "trajectories": {},
            "allocation": {},
            "allocation_strategy": "unknown",
            "suggestions": [],
            "strategy_analysis": None,
            "allocation_reasoning": None,
            "route_analysis": None,
            "critic_review": None,
            "mission_metrics": {},
            "solver_type": "unknown",
            "solver_runtime_ms": 0,
            "valid": False,
            "trace": coordinator_decision.trace,
            "optimizer_steps": [],
        }

    # Extract policy decisions and drone contracts
    coordinator_policy = coordinator_decision.policy
    coordinator_guardrails = coordinator_decision.guardrails
    coordinator_drone_contracts = coordinator_decision.drone_contracts
    coordinator_synthetic_starts = coordinator_decision.synthetic_starts

    # ------------------------------------------------------------------
    # 3b) Populate env.synthetic_starts from decision (authoritative data)
    #     Coordinator.synthetic_starts is the single source of truth for
    #     synthetic start coordinates - NOT guardrails (which are for
    #     validation outcomes only)
    # ------------------------------------------------------------------
    if coordinator_synthetic_starts:
        env["synthetic_starts"] = coordinator_synthetic_starts
        print(f"📍 [v4] Populated env.synthetic_starts: {list(coordinator_synthetic_starts.keys())}", file=sys.stderr)

    # ------------------------------------------------------------------
    # 4) Build initial mission state for the v4 workflow
    # ------------------------------------------------------------------
    initial_state: MissionState = {
        "messages": [HumanMessage(content=user_message)],
        "environment": env,
        "drone_configs": normalized_configs,
        "distance_matrix": dist_matrix,
        "excluded_targets": excluded_targets,  # Targets inside SAM zones - do NOT allocate
        "user_request": user_message,
        "request_type": "optimize",  # default; strategist can refine
        "commands": None,
        # Coordinator v4 injected fields
        "intent": coordinator_decision.intent,
        "policy": coordinator_policy,
        "guardrails": coordinator_guardrails,
        "drone_contracts": coordinator_drone_contracts,  # Per-drone start/home/endpoint contracts
        "allocation_strategy": coordinator_policy.get("allocation_strategy", "efficient"),
        # Agent reasoning outputs
        "strategy_analysis": None,
        "allocation_reasoning": None,
        "route_analysis": None,
        "critic_review": None,
        "suggestions": None,
        "allocation": None,
        "routes": None,
        "solver_type": None,
        "solver_runtime_ms": None,
        "policy_rules": policy_rules,  # Learning v1: loaded rules for enforcement
        "final_response": None,
        "error": None,
    }

    # If we are continuing an existing mission (Q&A only), seed the state with
    # previously computed routes/allocation so agents can reason about them.
    if existing_solution:
        prev_routes = existing_solution.get("routes")
        if prev_routes:
            initial_state["routes"] = prev_routes

        prev_allocation = existing_solution.get("allocation")
        if prev_allocation:
            initial_state["allocation"] = prev_allocation

    # ------------------------------------------------------------------
    # 5) Run the multi-agent workflow
    # ------------------------------------------------------------------
    workflow = build_reasoning_workflow()

    try:
        final_state = workflow.invoke(initial_state)
    except Exception as e:
        print(f"❌ [v4] Workflow error: {e}", file=sys.stderr)
        return {
            "response": f"Error processing request: {str(e)}",
            "routes": {},
            "total_points": 0,
            "total_fuel": 0.0,
            "trajectories": {},
            "allocation": {},
            "allocation_strategy": "unknown",
            "suggestions": [],
            "strategy_analysis": None,
            "allocation_reasoning": None,
            "route_analysis": None,
            "critic_review": None,
            "mission_metrics": {},
            # Decision Trace v1 fields
            "solver_type": "unknown",
            "solver_runtime_ms": 0,
            "valid": False,
            "trace": {},
            "optimizer_steps": [],
        }

    # Extract routes + allocation from agent workflow
    routes = final_state.get("routes") or {}
    allocation = final_state.get("allocation") or {}

    # ---------------------------------------------------------------
    # 5) Compute mission metrics (fuel, margins, points, unvisited)
    # ---------------------------------------------------------------
    mission_metrics = {}
    if routes:
        try:
            mission_metrics = compute_mission_metrics(
                env=env,
                routes=routes,
                allocations=allocation,
                drone_configs=normalized_configs,
            )
        except Exception as e:
            print(f"[v4] Error computing mission metrics: {e}", flush=True)
            mission_metrics = {}

    # Attach metrics for Strategist / Responder
    final_state["mission_metrics"] = mission_metrics
    final_state["total_points"] = mission_metrics.get("total_points")
    final_state["total_fuel"] = mission_metrics.get("total_fuel")

    # ---------------------------------------------------------------
    # 6) Extract routes & compute trajectories (existing logic)
    # ---------------------------------------------------------------

    routes = final_state.get("routes", {}) or {}
    response_text = final_state.get("final_response", "No response generated")

    total_points = 0
    total_fuel = 0.0
    trajectories: Dict[str, List[List[float]]] = {}

    # Build waypoint index
    waypoint_positions: Dict[str, List[float]] = {}
    for airport in env.get("airports", []):
        aid = str(airport.get("id", airport.get("label")))
        waypoint_positions[aid] = [
            float(airport.get("x", 0.0)),
            float(airport.get("y", 0.0)),
        ]

    for target in env.get("targets", []):
        tid = str(target.get("id", target.get("label")))
        waypoint_positions[tid] = [
            float(target.get("x", 0.0)),
            float(target.get("y", 0.0)),
        ]

    # Add synthetic starts from environment (authoritative data, populated in step 3b)
    env_synthetic_starts = env.get("synthetic_starts", {})
    for synth_id, synth_coords in env_synthetic_starts.items():
        if synth_id not in waypoint_positions:
            waypoint_positions[synth_id] = [
                float(synth_coords.get("x", 0.0)),
                float(synth_coords.get("y", 0.0)),
            ]
            print(f"📍 [v4] Added synthetic start to waypoints: {synth_id}", file=sys.stderr)

    # Also check routes for synthetic starts that may not be in guardrails
    for did, route_data in routes.items():
        if isinstance(route_data, dict):
            start_id = route_data.get("start_id")
            if start_id and start_id not in waypoint_positions and start_id.endswith("_START"):
                # Try to get coords from sam_matrix waypoints
                if sam_matrix:
                    for wp in sam_matrix.get("waypoints", []):
                        if wp.get("id") == start_id:
                            waypoint_positions[start_id] = [
                                float(wp.get("x", 0.0)),
                                float(wp.get("y", 0.0)),
                            ]
                            print(f"📍 [v4] Added synthetic start from sam_matrix: {start_id}", file=sys.stderr)
                            break

    sams = env.get("sams", []) or []
    trajectory_planner: Optional[ISRTrajectoryPlanner] = None

    if sams:
        try:
            trajectory_planner = ISRTrajectoryPlanner(sams)
        except Exception as e:
            print(f"⚠️ [v4] Failed to initialize ISRTrajectoryPlanner: {e}", file=sys.stderr)
            trajectory_planner = None

    # Generate trajectories drone-by-drone
    for did, route_data in routes.items():
        if not isinstance(route_data, dict):
            continue

        points = route_data.get("total_points", 0) or 0
        distance = route_data.get("distance", 0.0) or 0.0

        total_points += points
        total_fuel += distance

        route = route_data.get("route", []) or []
        route_ids = [str(wp_id) for wp_id in route]

        def _straight_line_traj() -> List[List[float]]:
            traj = []
            for wp_id in route_ids:
                pos = waypoint_positions.get(wp_id)
                if pos is not None:
                    traj.append(pos)
            return traj

        if trajectory_planner and waypoint_positions:
            try:
                traj_pts = trajectory_planner.generate_trajectory(
                    route_ids,
                    waypoint_positions,
                    drone_id=str(did),
                )
            except Exception as e:
                print(f"⚠️ [v4] Trajectory planner error for D{did}: {e}", file=sys.stderr)
                traj_pts = _straight_line_traj()
        else:
            traj_pts = _straight_line_traj()

        trajectories[str(did)] = traj_pts

    # ---------------------------------------------------------------
    # 7) Build Decision Trace v1 structure
    # ---------------------------------------------------------------
    solver_type = final_state.get("solver_type", "unknown")
    solver_runtime_ms = final_state.get("solver_runtime_ms", 0)

    # Determine if solution is valid (all routes within fuel budget)
    is_valid = True
    for did, route_data in routes.items():
        if isinstance(route_data, dict):
            distance = route_data.get("distance", 0.0)
            cfg = normalized_configs.get(str(did), {})
            fuel_budget = cfg.get("fuelBudget", cfg.get("fuel_budget", 200))
            if distance > fuel_budget:
                is_valid = False
                break

    # Build the Decision Trace JSONB structure (Wiring Point C)
    trace = {
        # Coordinator v4 pre-pass decisions
        "coordinator": {
            "intent": {
                "name": coordinator_decision.intent,
                "confidence": coordinator_decision.confidence,
                "rules_hit": coordinator_decision.rules_hit,
            },
            "guards": coordinator_guardrails,
            "policy": coordinator_policy,
            "drone_contracts": coordinator_drone_contracts,
            "synthetic_starts": coordinator_synthetic_starts,  # Authoritative coordinate data
        },
        # Environment and matrix metadata for reproducibility
        "env": {
            "env_hash": matrix_env_hash,
            "routing_model_hash": matrix_routing_model_hash,
            "distance_matrix_id": matrix_distance_matrix_id,
            "cache_hit": matrix_cache_hit,
            "sam_mode": matrix_routing_model.get("sam_mode", "hard_v1") if matrix_routing_model else None,
            "routing_model": matrix_routing_model,
        },
        "eligibility": {},  # Which targets were eligible per drone
        "allocation": {
            "algorithm": final_state.get("allocation_strategy", "unknown"),
            "assignments": final_state.get("allocation", {}),
            "rationale": final_state.get("allocation_reasoning"),
        },
        "solver": {
            "type": solver_type,
            "runtime_ms": solver_runtime_ms,
        },
        "final_evidence": {},  # Per-drone route details
        "policy_rules_applied": [
            {"id": r.get("id"), "title": r.get("title"), "category": r.get("category")}
            for r in policy_rules
        ] if policy_rules else [],
    }

    # Populate eligibility and final_evidence from routes
    for did, route_data in routes.items():
        if isinstance(route_data, dict):
            trace["final_evidence"][str(did)] = {
                "waypoints": route_data.get("route", []),
                "fuel": route_data.get("distance", 0.0),
                "points": route_data.get("total_points", 0),
                "visited_targets": route_data.get("visited_targets", []),
            }

    # ---------------------------------------------------------------
    # 8) Return unified V4 result with Decision Trace v1 fields
    # ---------------------------------------------------------------
    # ---------------------------------------------------------------
    # 8a) Coordinator contract fields for the UI (intent/actions/policy)
    # ---------------------------------------------------------------
    actions: List[Dict[str, Any]] = []
    if coordinator_decision.explanation_only:
        actions.append({"type": "EXPLAIN"})
    else:
        actions.append({"type": "ALLOCATE", "strategy": coordinator_decision.policy.get("allocation_strategy", "unknown") if isinstance(coordinator_decision.policy, dict) else "unknown"})
        actions.append({"type": "SOLVE", "solver": solver_type, "mode": coordinator_decision.policy.get("solver_mode", "default") if isinstance(coordinator_decision.policy, dict) else "default"})
        actions.append({"type": "TRAJECTORY", "samAware": True})
        # Post-optimizer actions are conditional; we mark configured intent here
        actions.append({"type": "POST_OPT"})
        actions.append({"type": "VERIFY"})

    return {
        "response": response_text,
        "intent": coordinator_decision.intent,
        "actions": actions,
        "policy": coordinator_decision.policy,
        "constraints": coordinator_decision.constraints,
        "warnings": coordinator_decision.warnings,
        "errors": coordinator_decision.errors,
        "routes": routes,
        "total_points": mission_metrics.get("total_points", total_points),
        "total_fuel": mission_metrics.get("total_fuel", total_fuel),
        "trajectories": trajectories,
        "allocation": final_state.get("allocation", {}),
        "allocation_strategy": final_state.get("allocation_strategy", "unknown"),
        "suggestions": final_state.get("suggestions", []),
        "strategy_analysis": final_state.get("strategy_analysis"),
        "allocation_reasoning": final_state.get("allocation_reasoning"),
        "route_analysis": final_state.get("route_analysis"),
        "critic_review": final_state.get("critic_review"),
        "mission_metrics": mission_metrics,
        # Decision Trace v1 fields
        "solver_type": solver_type,
        "solver_runtime_ms": solver_runtime_ms,
        "valid": is_valid,
        "trace": trace,
        "optimizer_steps": [],  # TODO: populate from post_optimizer if used
        # Structured trace_events from Coordinator (for Agents Monitor tab)
        "coordinator_trace_events": coordinator_decision.trace_events,
    }
