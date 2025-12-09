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


def build_current_solution_context(state: MissionState) -> str:
    """Build context about the current solution state."""
    allocation = state.get("allocation", {})
    routes = state.get("routes", {})

    if not allocation and not routes:
        return "CURRENT SOLUTION: None computed yet"

    lines = ["CURRENT SOLUTION:"]

    if allocation:
        lines.append("  Allocation:")
        for did in sorted(allocation.keys()):
            tgts = allocation[did]
            lines.append(f"    D{did}: {', '.join(tgts) if tgts else '(none)'}")

    if routes:
        lines.append("  Routes:")
        total_fuel = 0
        total_points = 0
        for did in sorted(routes.keys()):
            route_data = routes[did]
            route = route_data.get("route", [])
            dist = route_data.get("distance", 0)
            points = route_data.get("total_points", 0)
            total_fuel += dist
            total_points += points
            lines.append(f"    D{did}: {' → '.join(route)} (fuel={dist:.0f}, points={points})")
        lines.append(f"  TOTALS: fuel={total_fuel:.0f}, points={total_points}")

    return "\n".join(lines)


# ============================================================================
# STRATEGIST AGENT - Analyzes request, determines approach
# ============================================================================

STRATEGIST_PROMPT = """You are the STRATEGIST agent in an ISR mission planning system.

Your job is to analyze the user's request and determine:
1. REQUEST TYPE: Is this a question, optimization request, or direct command?
2. MODE: Should the system optimize freely or follow specific commands?
3. CONSTRAINTS: What drone configs and restrictions apply?
4. COMMANDS: Any explicit orders that must be followed exactly?

RULES:
- DEFAULT behavior: Maximize priority points with minimum fuel usage
- COMMANDS override everything: "D1 must visit T3" means D1 MUST visit T3
- CONSTRAINTS = COMMANDS: Drone accessibility restrictions are orders
- If a command is IMPOSSIBLE (exceeds fuel), refuse and explain why

OUTPUT FORMAT:
Analyze the request and output your analysis in this exact format:

REQUEST_TYPE: [question|optimize|command]
MODE: [optimize_freely|follow_commands]
COMMANDS_DETECTED: [list any explicit commands, or "none"]
CONSTRAINTS_SUMMARY: [summarize active constraints]
FEASIBILITY: [possible|impossible|needs_check]
REASONING: [brief explanation of your analysis]
"""


def strategist_node(state: MissionState) -> Dict[str, Any]:
    """Strategist analyzes the request and determines approach."""
    print("\n🧠 [STRATEGIST] Analyzing request...", file=sys.stderr)
    sys.stderr.flush()

    set_state(state)

    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)

    user_request = state.get("user_request", "")
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
    response = llm.invoke(messages)

    analysis = response.content
    print(f"📋 [STRATEGIST] Analysis complete", file=sys.stderr)

    # Parse request type from analysis
    request_type = "optimize"  # default
    if "REQUEST_TYPE: question" in analysis.lower():
        request_type = "question"
    elif "REQUEST_TYPE: command" in analysis.lower():
        request_type = "command"

    return {
        "messages": [AIMessage(content=f"[STRATEGIST]\n{analysis}")],
        "strategy_analysis": analysis,
        "request_type": request_type,
    }


# ============================================================================
# ALLOCATOR AGENT - Reasons about target assignments
# ============================================================================

ALLOCATOR_PROMPT = """You are the ALLOCATOR agent in an ISR mission planning system.

Your job is to decide which targets each drone should visit. You must REASON about:
1. Which drone is best suited for each target (distance, fuel, accessibility)
2. How to balance load across drones
3. How to maximize total priority within fuel constraints
4. Any commands that must be followed

RULES:
- Respect accessibility constraints (if D1 can only access T1-T4, don't give it T5)
- Consider fuel budgets (don't assign more targets than a drone can reach)
- Prioritize high-value targets
- If following commands, ensure commanded targets are assigned correctly

For each allocation decision, briefly explain WHY.

OUTPUT FORMAT:
ALLOCATION_REASONING:
- D1 gets [targets] because [reason]
- D2 gets [targets] because [reason]
...

ALLOCATION_RESULT:
D1: T1, T3, T5
D2: T2, T4
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
    response = llm.invoke(messages)

    reasoning = response.content
    print(f"📋 [ALLOCATOR] Reasoning complete", file=sys.stderr)

    # Parse allocation from response
    allocation = parse_allocation_from_reasoning(reasoning, state)

    return {
        "messages": [AIMessage(content=f"[ALLOCATOR]\n{reasoning}")],
        "allocation_reasoning": reasoning,
        "allocation": allocation,
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
        home_airport = cfg.get("homeAirport", cfg.get("home_airport", "A1"))

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
                # Build matrix data
                labels = list(dist_matrix.keys())
                matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]

                # Filter targets to only those allocated
                targets = [t for t in env.get("targets", [])
                          if str(t.get("id", t.get("label"))) in target_ids]

                solver_env = {
                    "airports": env.get("airports", []),
                    "targets": targets,
                    "matrix": matrix,
                    "matrix_labels": labels,
                    "start_airport": home_airport,
                    "end_airport": home_airport,
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

RESPONDER_PROMPT = """You are the RESPONDER agent in an ISR mission planning system.

Your job is to formulate the final response to the user. This should:
1. Directly answer their question or confirm the solution
2. Include key reasoning (concise, not verbose)
3. Mention any important suggestions from the critic
4. Be professional and helpful

For QUESTIONS: Give a clear, direct answer
For SOLUTIONS: Summarize routes, total points, fuel usage, and any notable trade-offs

Keep it concise but complete.
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
) -> Dict[str, Any]:
    """
    Run the v4 reasoning-based multi-agent system.

    Args:
        user_message: The user's request
        environment: Environment data (airports, targets, sams)
        drone_configs: Drone configurations
        distance_matrix: Optional precomputed distance matrix

    Returns:
        Result dictionary with response, routes, trajectories, etc.
    """
    print(f"\n{'='*60}", file=sys.stderr)
    print(f"[v4] Processing: {user_message[:50]}...", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    sys.stderr.flush()

    # Build initial state
    initial_state: MissionState = {
        "messages": [HumanMessage(content=user_message)],
        "environment": environment,
        "drone_configs": drone_configs,
        "distance_matrix": distance_matrix,
        "user_request": user_message,
        "request_type": "optimize",
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

    # Build and run workflow
    workflow = build_reasoning_workflow()

    try:
        final_state = workflow.invoke(initial_state)
    except Exception as e:
        print(f"❌ [v4] Workflow error: {e}", file=sys.stderr)
        return {
            "response": f"Error processing request: {str(e)}",
            "routes": {},
            "total_points": 0,
            "total_fuel": 0,
            "trajectories": {},
        }

    # Extract results
    routes = final_state.get("routes", {})
    response_text = final_state.get("final_response", "No response generated")

    # Calculate totals
    total_points = 0
    total_fuel = 0.0
    trajectories = {}

    for did, route_data in routes.items():
        total_points += route_data.get("total_points", 0)
        total_fuel += route_data.get("distance", 0)

        # Build trajectory (simplified - just waypoint coordinates)
        route = route_data.get("route", [])
        traj_points = []
        for wp_id in route:
            pos = get_waypoint_position(wp_id, environment)
            if pos:
                traj_points.append(pos)
        trajectories[f"D{did}"] = traj_points

    return {
        "response": response_text,
        "routes": routes,
        "total_points": total_points,
        "total_fuel": total_fuel,
        "trajectories": trajectories,
        "allocation": final_state.get("allocation", {}),
        "suggestions": final_state.get("suggestions", []),
    }


def get_waypoint_position(wp_id: str, env: Dict[str, Any]) -> Optional[List[float]]:
    """Get position of a waypoint by ID."""
    for airport in env.get("airports", []):
        aid = airport.get("id", airport.get("label"))
        if aid == wp_id:
            return [float(airport["x"]), float(airport["y"])]

    for target in env.get("targets", []):
        tid = target.get("id", target.get("label"))
        if tid == wp_id:
            return [float(target["x"]), float(target["y"])]

    return None


# Singleton workflow for FastAPI
_workflow = None


def get_workflow():
    """Get or create the v4 workflow singleton."""
    global _workflow
    if _workflow is None:
        _workflow = build_reasoning_workflow()
    return _workflow
