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
)
from ..solver.sam_distance_matrix import calculate_sam_aware_matrix
from ..solver.post_optimizer import post_optimize_solution
from ..solver.trajectory_planner import ISRTrajectoryPlanner

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

    # User request analysis
    user_request: str
    request_type: str  # "question", "optimize", "command"
    commands: Optional[List[Dict[str, Any]]]  # Explicit commands extracted

    # Agent reasoning outputs
    strategy_analysis: Optional[str]  # Strategist's analysis
    allocation_reasoning: Optional[str]  # Allocator's reasoning
    route_analysis: Optional[str]  # Router's analysis
    critic_review: Optional[str]  # Critic's review
    suggestions: Optional[List[str]]  # Improvement suggestions

    # Solution data
    allocation: Optional[Dict[str, List[str]]]
    routes: Optional[Dict[str, Dict[str, Any]]]

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
# HELPER FUNCTIONS - Build context for agents
# ============================================================================

def build_mission_context(state: MissionState) -> str:
    """Build a comprehensive context string for agents to reason about."""
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    dist_matrix = state.get("distance_matrix", {})

    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

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

    lines.append(f"\n  Targets: {len(targets)}")
    total_priority = 0
    for t in targets:
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
        for did in sorted(configs.keys()):
            cfg = configs[did]
            enabled = cfg.get("enabled", did == "1")
            if not enabled:
                continue
            fuel = cfg.get("fuelBudget", cfg.get("fuel_budget", 200))
            airport = cfg.get("homeAirport", cfg.get("home_airport", "A1"))
            accessible = cfg.get("accessibleTargets", cfg.get("accessible_targets", []))

            lines.append(f"  D{did}:")
            lines.append(f"    Home: {airport}, Fuel: {fuel}")
            if accessible:
                lines.append(f"    Accessible: {', '.join(accessible)}")
            else:
                lines.append(f"    Accessible: ALL targets")

    # Add distance info if available
    if dist_matrix:
        lines.append("\nDISTANCE MATRIX: Available (SAM-aware paths computed)")

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
        # v4 routes normally look like: {"route": [...], "distance": float, "points": int}
        if isinstance(route_info, dict):
            route = route_info.get("route") or []
            distance = float(route_info.get("distance", 0.0))
            points = int(route_info.get("points", 0))
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

    user_request = state.get("user_request", "")
    print(f"📝 [STRATEGIST] User request: '{user_request}'", file=sys.stderr)
    sys.stderr.flush()

    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    mission_context = build_mission_context(state)
    current_solution = build_current_solution_context(state)

    # Build prompt with full context
    analysis_prompt = f"""
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

Your job is to allocate targets to drones to minimize fuel consumption.

MANDATORY REQUIREMENTS:
1. You MUST allocate ALL targets that are not inside SAM polygons
2. Every accessible target must be assigned to a drone - NO EXCEPTIONS
3. Allocate to minimize total fuel usage across all drones
4. Respect target type accessibility (e.g., if D1 can only access types A-C, don't assign type D)

AVAILABLE ALLOCATION STRATEGIES:
You have 5 allocation algorithms available, should you choose to use them:
- "efficient": Maximize priority/fuel ratio using auction-based algorithm
- "greedy": Assign highest priority targets to nearest capable drone
- "balanced": Distribute targets evenly by count across drones
- "geographic": Minimize detours based on drone corridors
- "exclusive": Prioritize targets only one drone can reach

IMPORTANT DISTINCTION:
Allocation assigns targets to drones. The route optimizer later decides which targets
actually fit in fuel budgets. Your job: allocate every accessible target to the most
fuel-efficient drone. Do not skip targets.

ALLOCATION APPROACH:
- Assign each target to the closest capable drone
- Balance workload when distances are similar
- Follow any user commands about target assignments

Briefly explain allocation decisions for each drone.

OUTPUT FORMAT:
STRATEGY_USED: [name of strategy you chose: efficient/greedy/balanced/geographic/exclusive]

ALLOCATION_REASONING:
- D1 gets [targets] because [reason]
- D2 gets [targets] because [reason]
...

ALLOCATION_RESULT:
D1: T1, T3, T5, T11
D2: T2, T4, T6
...

TRADE_OFFS:
[Any notable trade-offs or compromises made]
"""


def allocator_node(state: MissionState) -> Dict[str, Any]:
    """Allocator reasons about and performs target allocation."""
    print("\n🎯 [ALLOCATOR] Reasoning about allocation...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    mission_context = build_mission_context(state)
    strategy_analysis = state.get("strategy_analysis", "No strategy analysis available")

    allocation_prompt = f"""
{ALLOCATOR_PROMPT}

{mission_context}

STRATEGIST'S ANALYSIS:
{strategy_analysis}

Based on this context, determine the optimal target allocation.
Remember: Explain WHY each drone gets its targets.
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

    # Parse allocation from response
    allocation = parse_allocation_from_reasoning(reasoning, state)

    # Parse strategy used from reasoning
    strategy_used = "unknown"
    for line in reasoning.split("\n"):
        if "STRATEGY_USED:" in line.upper():
            # Extract strategy name after the colon
            parts = line.split(":")
            if len(parts) >= 2:
                strategy_used = parts[1].strip().lower()
                # Remove any extra text, keep only the strategy name
                for strat in ["efficient", "greedy", "balanced", "geographic", "exclusive"]:
                    if strat in strategy_used:
                        strategy_used = strat
                        break
            break

    print(f"📋 [ALLOCATOR] Strategy used: {strategy_used}", file=sys.stderr)

    return {
        "messages": [AIMessage(content=f"[ALLOCATOR]\n{reasoning}")],
        "allocation_reasoning": reasoning,
        "allocation": allocation,
        "allocation_strategy": strategy_used,
    }


def parse_allocation_from_reasoning(reasoning: str, state: MissionState) -> Dict[str, List[str]]:
    """Parse allocation from LLM reasoning output."""
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

    # Log what we parsed
    print(f"📋 [ALLOCATOR] Parsed allocation: {allocation}", file=sys.stderr)
    total_allocated = sum(len(targets) for targets in allocation.values())
    print(f"📋 [ALLOCATOR] Total targets allocated: {total_allocated}", file=sys.stderr)

    # If parsing failed, fall back to algorithmic allocation
    if all(len(v) == 0 for v in allocation.values()):
        print("⚠️ [ALLOCATOR] Parsing failed, using fallback allocation", file=sys.stderr)
        env = state.get("environment", {})
        dist_matrix = state.get("distance_matrix")

        matrix_data = None
        if dist_matrix:
            labels = list(dist_matrix.keys())
            matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
            matrix_data = {"labels": labels, "matrix": matrix}
            set_allocator_matrix(matrix_data)

        allocation = _allocate_targets_impl(env, configs, "efficient", matrix_data)
        print(f"📋 [ALLOCATOR] Fallback allocation: {allocation}", file=sys.stderr)

    # Final validation: check if all targets are allocated
    env = state.get("environment", {})
    all_targets = {str(t["id"]) for t in env.get("targets", [])}
    allocated_targets = set()
    for targets in allocation.values():
        allocated_targets.update(targets)

    missing_targets = all_targets - allocated_targets
    if missing_targets:
        print(f"⚠️ [ALLOCATOR] WARNING: {len(missing_targets)} targets NOT allocated: {sorted(missing_targets)}", file=sys.stderr)

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
    print("\n🛣️ [ROUTE_OPTIMIZER] Computing routes...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    allocation = state.get("allocation", {})
    dist_matrix = state.get("distance_matrix", {})

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
        home_airport = cfg.get("home_airport", "A1")
        end_airport = cfg.get("end_airport", home_airport)
        mode = "return" if end_airport == home_airport else "end"

        print(f"[v4][ROUTE_OPT] D{did} start={home_airport} end={end_airport} mode={mode}", flush=True)

        if not target_ids:
            routes[did] = {
                "route": [home_airport, home_airport],
                "distance": 0,
                "total_points": 0,
                "visited_targets": [],
            }
            route_analysis_lines.append(f"- D{did}: No targets assigned, stays at {home_airport}")
            continue

        # Build solver environment
        if solver and dist_matrix:
            try:
                # --- Build FILTERED matrix data ---
                all_labels = list(dist_matrix.keys())

                # For this drone, we only care about its home airport + its allocated targets
                requested_labels = [home_airport] + list(target_ids)

                # Keep only labels that actually exist in the distance matrix
                labels = [lab for lab in requested_labels if lab in all_labels]

                if not labels or home_airport not in labels:
                    print(
                        f"⚠️ [ROUTE_OPTIMIZER] No valid labels for D{did} "
                        f"(home={home_airport}, targets={target_ids})",
                        file=sys.stderr,
                    )
                    routes[did] = {
                        "route": [home_airport],
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

                print(f"[v4][ROUTE_OPT] D{did} home={home_airport} end={end_airport} mode={mode} cfg_end={cfg.get('end_airport')} cfg_keys={list(cfg.keys())}", flush=True)

                solver_env = {
                    "airports": env.get("airports", []),
                    "targets": targets,
                    "matrix": matrix,
                    "matrix_labels": labels,
                    "start_airport": home_airport,
                    "end_airport": end_airport,
                    "mode": mode,
                }

                solution = solver.solve(solver_env, fuel_budget)

                route = solution.get("route", [home_airport])
                distance = solution.get("distance", 0)
                points = solution.get("total_points", 0)
                visited = solution.get("visited_targets", [])

                routes[did] = {
                    "route": route,
                    "distance": distance,
                    "total_points": points,
                    "visited_targets": visited,
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
                routes[did] = {
                    "route": [home_airport] + target_ids + [home_airport],
                    "distance": 999,
                    "total_points": 0,
                    "visited_targets": [],
                    "error": str(e),
                }
        else:
            # Fallback: simple route without optimization
            routes[did] = {
                "route": [home_airport] + target_ids + [home_airport],
                "distance": 0,
                "total_points": sum(
                    t.get("priority", 5) for t in env.get("targets", [])
                    if str(t.get("id", t.get("label"))) in target_ids
                ),
                "visited_targets": target_ids,
            }

    route_analysis = "\n".join(route_analysis_lines)
    if feasibility_issues:
        route_analysis += "\n\nFEASIBILITY_ISSUES:\n" + "\n".join(feasibility_issues)
    route_analysis += f"\n\nROUTES_COMPUTED: YES"

    print(f"📋 [ROUTE_OPTIMIZER] Routes computed for {len(routes)} drones", file=sys.stderr)

    return {
        "messages": [AIMessage(content=f"[ROUTE_OPTIMIZER]\n{route_analysis}")],
        "route_analysis": route_analysis,
        "routes": routes,
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
{CRITIC_PROMPT}

{mission_context}

ALLOCATION REASONING:
{allocation_reasoning}

ROUTE ANALYSIS:
{route_analysis}

{current_solution}

Review this solution and provide your assessment.
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
    distance_matrix: Optional[Dict[str, Any]] = None,
    existing_solution: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:

    print("### V4: run_multi_agent_v4 (isr_agent_multi_v4.py) CALLED ###", flush=True)
    """
    Run the v4 reasoning-based multi-agent system.

    Args:
        user_message: The user's request
        environment: Environment data (airports, targets, sams)
        drone_configs: Drone configurations from the UI
        distance_matrix: Optional precomputed distance matrix

    Returns:
        Result dictionary with response, routes, trajectories, etc.
    """
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"[v4] Processing: {user_message[:80]}...", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    sys.stderr.flush()

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

        # Home airport: map from start_airport if not already set
        home_ap = (
            cfg.get("homeAirport")
            or cfg.get("home_airport")
            or cfg.get("start_airport")
            or cfg.get("startAirport")
        )

        if home_ap:
            nc["home_airport"] = home_ap

        # End airport: ALWAYS set (fallback to home_airport)
        end_ap = (
            cfg.get("end_airport")
            or cfg.get("endAirport")
        )
        # Use explicit end if provided; otherwise default to home airport
        nc["end_airport"] = end_ap if end_ap else nc.get("home_airport", home_ap)

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
    dist_matrix: Dict[str, Dict[str, float]]

    if distance_matrix:
        # Use provided matrix if given
        dist_matrix = distance_matrix
    else:
        sams = env.get("sams", [])
        try:
            if sams:
                print("🎯 [v4] Computing SAM-aware distance matrix...", file=sys.stderr)
                dist_data = calculate_sam_aware_matrix(env)
                labels = dist_data.get("labels", [])
                matrix = dist_data.get("matrix", [])
                dist_matrix = {}
                for i, from_id in enumerate(labels):
                    dist_matrix[from_id] = {}
                    for j, to_id in enumerate(labels):
                        dist_matrix[from_id][to_id] = float(matrix[i][j])
            else:
                print("📏 [v4] Computing Euclidean distance matrix...", file=sys.stderr)
                # Simple Euclidean fallback (similar to v3)
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
    # 3) Build initial mission state for the v4 workflow
    # ------------------------------------------------------------------
    initial_state: MissionState = {
        "messages": [HumanMessage(content=user_message)],
        "environment": env,
        "drone_configs": normalized_configs,
        "distance_matrix": dist_matrix,
        "user_request": user_message,
        "request_type": "optimize",  # default; strategist can refine
        "commands": None,
        "strategy_analysis": None,
        "allocation_reasoning": None,
        "route_analysis": None,
        "critic_review": None,
        "suggestions": None,
        "allocation": None,
        "routes": None,
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
    # 4) Run the multi-agent workflow
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
                pos = get_waypoint_position(wp_id, env)
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
    # 7) Return unified V4 result (NOW INCLUDING mission_metrics)
    # ---------------------------------------------------------------
    return {
        "response": response_text,
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
    }
