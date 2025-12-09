"""
ISR Multi-Agent Mission Planning System (v3)

A TRUE multi-agent LangGraph architecture where:
1. Each agent is an independent LLM with specialized expertise
2. Agents communicate through explicit state updates (not text parsing)
3. Tool calls are MANDATORY - agents must use their tools
4. Clear handoff protocol between agents
5. Retry logic for agents that don't produce proper output

Architecture:
- Coordinator: Understands user request, orchestrates workflow
- Allocator: Assigns targets to drones based on constraints
- Router: Computes optimal routes using Held-Karp algorithm
- Validator: Checks routes against constraints
- Optimizer: Post-optimization for better solutions

This design can be extended to delivery systems with:
- Fleet Manager (heterogeneous drone types)
- Task Planner (pickup/delivery sequences)
- Conflict Resolver (time deconfliction)
"""

import os
import sys
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional, TypedDict, Annotated, Literal
from dataclasses import dataclass, field
from enum import Enum
from concurrent.futures import ThreadPoolExecutor, as_completed

from dotenv import load_dotenv
from langchain_anthropic import ChatAnthropic
from langchain_core.messages import HumanMessage, AIMessage, SystemMessage, BaseMessage, ToolMessage
from langchain_core.tools import tool
from langgraph.graph import StateGraph, END, START
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolNode

# Import solver components
# Use the OrienteeringSolverInterface directly for route solving
import sys
from pathlib import Path

# Add legacy path for orienteering solver
legacy_path = Path(__file__).resolve().parents[3] / "legacy" / "isr_legacy_all"
if str(legacy_path) not in sys.path:
    sys.path.insert(0, str(legacy_path))

# Add root path for local development (isr_projects)
root_path = Path(__file__).resolve().parents[3]
if str(root_path) not in sys.path:
    sys.path.insert(0, str(root_path))

# Add /app path for Docker deployment
if "/app" not in sys.path:
    sys.path.insert(0, "/app")

# Try importing from different paths (Docker vs local)
OrienteeringSolverInterface = None
try:
    # Docker path: /app/webapp/editor/solver/...
    from webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface
    print("✅ OrienteeringSolverInterface loaded (Docker path)")
except ImportError as e:
    print(f"⚠️ Docker import failed: {e}")
    try:
        # Local development path: isr_web/webapp/editor/solver/...
        from isr_web.webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface
        print("✅ OrienteeringSolverInterface loaded (local path)")
    except ImportError as e2:
        print(f"⚠️ Local import failed: {e2}")
        print("❌ OrienteeringSolverInterface not available - orienteering features disabled")

from ..solver.target_allocator import (
    allocate_targets as _allocate_targets_impl,
    parse_priority_constraints,
    allocate_with_priority_filters,
    set_allocator_matrix,
)
from ..solver.sam_distance_matrix import calculate_sam_aware_matrix
from ..solver.post_optimizer import (
    post_optimize_solution,
    trajectory_swap_optimize,
    crossing_removal_optimize,
)
from ..solver.trajectory_planner import ISRTrajectoryPlanner

# Load environment variables
load_dotenv()

# Try to load from benchmark .env if not set
if not os.getenv("ANTHROPIC_API_KEY"):
    benchmark_env = os.path.join(os.path.dirname(__file__), "..", "..", "..", "isr_benchmark", ".env")
    if os.path.exists(benchmark_env):
        load_dotenv(benchmark_env)


# ============================================================================
# Agent Phases - Clear state machine
# ============================================================================

class AgentPhase(str, Enum):
    """Workflow phases - agents transition through these explicitly."""
    START = "start"
    UNDERSTANDING = "understanding"  # Coordinator understands the request
    ALLOCATING = "allocating"        # Allocator assigns targets
    ROUTING = "routing"              # Router computes optimal paths
    VALIDATING = "validating"        # Validator checks constraints
    OPTIMIZING = "optimizing"        # Optimizer improves solution
    COMPLETE = "complete"            # Done - ready to return


# ============================================================================
# Multi-Agent State - Explicit, typed state shared between agents
# ============================================================================

class MissionState(TypedDict):
    """
    Explicit state shared across all agents.

    Unlike v2 which used message parsing, v3 uses explicit typed fields
    that agents read and write to communicate.
    """
    # Conversation history (for LLM context)
    messages: Annotated[list, add_messages]

    # Current workflow phase
    phase: str

    # Mission data (set once at start)
    environment: Optional[Dict[str, Any]]
    drone_configs: Optional[Dict[str, Any]]
    distance_matrix: Optional[Dict[str, Any]]

    # Agent outputs - explicit structured data
    allocation: Optional[Dict[str, List[str]]]  # Drone ID -> target IDs
    routes: Optional[Dict[str, Dict[str, Any]]]  # Drone ID -> route data
    validation_results: Optional[Dict[str, Any]]  # Validation status
    optimization_results: Optional[Dict[str, Any]]  # Optimization stats

    # Error handling
    error: Optional[str]
    retry_count: int

    # User request info
    user_request: str
    priority_constraints: Optional[str]


# ============================================================================
# Global Context (for tool access to mission data)
# ============================================================================

_current_state: Optional[MissionState] = None


def get_state() -> MissionState:
    """Get current mission state for tools."""
    if _current_state is None:
        raise ValueError("No mission state set")
    return _current_state


def set_state(state: MissionState):
    """Set current mission state."""
    global _current_state
    _current_state = state


# ============================================================================
# COORDINATOR AGENT
# ============================================================================

COORDINATOR_SYSTEM_PROMPT = """You are the COORDINATOR agent in a multi-agent ISR mission planning system.

Your role is to understand user requests and either:
1. Answer questions about the current mission configuration
2. Start a planning workflow to compute routes
3. Request optimization of existing routes

TOOLS AVAILABLE:
1. get_mission_info: Get detailed drone configurations, targets, environment data, and current solution state
2. analyze_mission_request: Start new route planning (use for "solve", "plan", "compute" requests)
3. request_optimization: Improve existing routes (use for "optimize", "swap", "2-opt" requests)

HOW TO RESPOND:

FOR QUESTIONS about the mission (drone configs, fuel budgets, targets, SAMs, current routes, allocations):
- Call get_mission_info tool FIRST to get the current configuration
- Then respond with a clear, human-friendly answer based on the tool's output
- Example questions: "What drone configuration are you using?", "What is D1's fuel budget?", "How many targets?"

FOR SOLVE/PLANNING REQUESTS ("solve", "plan mission", "compute routes", "run planner"):
- Call analyze_mission_request tool to start the planning workflow

FOR OPTIMIZATION REQUESTS ("optimize", "swap closer", "2-opt", "remove crossings"):
- Call request_optimization tool to improve existing routes

IMPORTANT: When the user asks about configuration, use get_mission_info and then summarize the answer.
Do NOT just dump the raw tool output - provide a concise, helpful response.
"""


@tool
def analyze_mission_request() -> str:
    """
    Analyze the user's mission request and determine workflow.

    Returns analysis of what needs to be done.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    request = state.get("user_request", "")

    # Count assets
    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    # Count enabled drones
    enabled_drones = [
        did for did, cfg in configs.items()
        if cfg.get("enabled", did == "1")
    ]

    # Calculate total priority
    total_priority = sum(t.get("priority", t.get("value", 5)) for t in targets)

    # Check for priority constraints in request
    priority_constraints = ""
    if "priority>=" in request.lower() or "priority<=" in request.lower():
        # Extract constraints from request
        import re
        matches = re.findall(r'D\d+(?:,D\d+)*:\s*priority[<>=]+\d+', request, re.IGNORECASE)
        if matches:
            priority_constraints = "; ".join(matches)

    analysis = f"""
MISSION ANALYSIS
================
Airports: {len(airports)}
Targets: {len(targets)} (total priority: {total_priority})
SAMs/NFZs: {len(sams)}
Enabled Drones: {len(enabled_drones)} ({', '.join(['D' + d for d in enabled_drones])})

User Request: {request}
Priority Constraints: {priority_constraints if priority_constraints else "None specified"}

WORKFLOW PLAN:
1. ALLOCATOR will distribute {len(targets)} targets across {len(enabled_drones)} drones
2. ROUTER will compute optimal routes for each drone
3. Results will be returned with trajectories

READY TO PROCEED - Routing to ALLOCATOR agent.
"""

    return analysis


@tool
def request_optimization() -> str:
    """
    Request optimization of existing routes.

    Use this when the user asks for optimization (swap closer, 2-opt, insert unvisited, etc.)
    and routes already exist.

    Returns:
        Status indicating routing to optimizer.
    """
    state = get_state()
    routes = state.get("routes", {})
    request = state.get("user_request", "").lower()

    if not routes:
        return "ERROR: No routes exist to optimize. Run 'solve' first to generate routes."

    # Identify which optimization is requested
    opt_type = "general"
    tool_to_use = "optimize_routes"

    if "swap" in request or "closer" in request:
        opt_type = "swap_closer"
        tool_to_use = "swap_closer_optimize"
    elif "crossing" in request or "2-opt" in request or "2opt" in request:
        opt_type = "remove_crossings"
        tool_to_use = "remove_crossings"
    elif "unvisited" in request or "insert" in request or "add" in request:
        opt_type = "insert_unvisited"
        tool_to_use = "insert_unvisited"
    elif "fuel" in request or "distance" in request or "reduce" in request:
        # Fuel reduction -> swap closer moves targets to closer drones
        opt_type = "swap_closer"
        tool_to_use = "swap_closer_optimize"

    return f"""
OPTIMIZATION REQUEST DETECTED
=============================
Existing routes: {list(routes.keys())}
Requested optimization: {opt_type}

INSTRUCTION TO OPTIMIZER: You MUST call the tool: {tool_to_use}
"""


@tool
def get_mission_info() -> str:
    """
    Get current mission configuration details.

    Use this tool to answer questions about drone configurations, targets,
    airports, SAMs, fuel budgets, and current state of the mission.

    Returns:
        Detailed mission information including all drone configs and environment data.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    routes = state.get("routes", {})
    allocation = state.get("allocation", {})

    # Environment details
    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    lines = [
        "=" * 60,
        "CURRENT MISSION CONFIGURATION",
        "=" * 60,
        "",
        "ENVIRONMENT:",
        f"  Airports: {len(airports)}",
    ]

    for ap in airports:
        ap_id = ap.get("id", ap.get("label", "?"))
        lines.append(f"    - {ap_id}: ({ap.get('x', 0):.1f}, {ap.get('y', 0):.1f})")

    lines.append(f"  Targets: {len(targets)}")
    total_priority = 0
    for t in targets:
        t_id = t.get("id", t.get("label", "?"))
        priority = t.get("priority", t.get("value", 5))
        total_priority += priority
        lines.append(f"    - {t_id}: priority={priority}, pos=({t.get('x', 0):.1f}, {t.get('y', 0):.1f})")

    lines.append(f"  Total Priority Points: {total_priority}")
    lines.append(f"  SAMs/NFZs: {len(sams)}")
    for sam in sams:
        pos = sam.get("pos", [0, 0])
        lines.append(f"    - Range: {sam.get('range', 0)}, pos=({pos[0]:.1f}, {pos[1]:.1f})")

    lines.append("")
    lines.append("DRONE CONFIGURATIONS:")

    if not configs:
        lines.append("  (No drone configurations set)")
    else:
        for did in sorted(configs.keys()):
            cfg = configs[did]
            enabled = cfg.get("enabled", did == "1")
            fuel = cfg.get("fuelBudget", cfg.get("fuel_budget", 200))
            airport = cfg.get("homeAirport", cfg.get("home_airport", "A1"))
            accessible = cfg.get("accessibleTargets", cfg.get("accessible_targets", []))

            status = "ENABLED" if enabled else "DISABLED"
            lines.append(f"  D{did} [{status}]:")
            lines.append(f"    - Home Airport: {airport}")
            lines.append(f"    - Fuel Budget: {fuel}")
            if accessible:
                lines.append(f"    - Accessible Targets: {', '.join(accessible)}")
            else:
                lines.append(f"    - Accessible Targets: ALL")

    # Current solution state
    lines.append("")
    lines.append("CURRENT SOLUTION STATE:")

    if allocation:
        lines.append("  Target Allocation:")
        for did in sorted(allocation.keys()):
            tgts = allocation[did]
            lines.append(f"    D{did}: {', '.join(tgts) if tgts else '(none)'}")
    else:
        lines.append("  Target Allocation: Not computed yet")

    if routes:
        lines.append("  Routes Computed:")
        for did in sorted(routes.keys()):
            route_data = routes[did]
            route = route_data.get("route", [])
            dist = route_data.get("distance", 0)
            points = route_data.get("total_points", 0)
            lines.append(f"    D{did}: {' -> '.join(route)} (dist={dist:.1f}, points={points})")
    else:
        lines.append("  Routes: Not computed yet")

    lines.append("")
    lines.append("=" * 60)

    return "\n".join(lines)


COORDINATOR_TOOLS = [analyze_mission_request, request_optimization, get_mission_info]


# ============================================================================
# ALLOCATOR AGENT
# ============================================================================

ALLOCATOR_SYSTEM_PROMPT = """You are the ALLOCATOR agent specializing in target distribution.

Your job is to allocate targets to drones optimally. You MUST call one of your tools:
- allocate_targets: Standard allocation using a strategy
- allocate_with_constraints: When priority constraints are specified

IMPORTANT: You MUST call one of these tools. Do not describe allocation - actually do it.

Strategies available:
- "efficient": Maximize priority/distance ratio (recommended)
- "balanced": Distribute targets evenly
- "geographic": Divide by angular sectors
- "greedy": Nearest targets to each drone
"""


@tool
def allocate_targets(strategy: str = "efficient") -> str:
    """
    Allocate targets to drones using specified strategy.

    Args:
        strategy: "efficient", "balanced", "geographic", or "greedy"

    Returns:
        Allocation results showing which targets assigned to each drone.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    dist_matrix = state.get("distance_matrix")

    # Build matrix data for allocator
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}
        set_allocator_matrix(matrix_data)

    # Do allocation
    allocation = _allocate_targets_impl(env, configs, strategy, matrix_data)

    # Store in state
    state["allocation"] = allocation

    # Format result
    targets = env.get("targets", [])
    lines = [
        "=" * 50,
        "TARGET ALLOCATION COMPLETE",
        "=" * 50,
        f"Strategy: {strategy}",
        ""
    ]

    total_allocated = 0
    total_priority = 0
    for did in sorted(allocation.keys()):
        target_ids = allocation[did]
        total_allocated += len(target_ids)
        drone_priority = sum(
            t.get("priority", 5) for t in targets
            if str(t.get("id", t.get("label"))) in target_ids
        )
        total_priority += drone_priority
        lines.append(f"D{did}: {', '.join(target_ids) if target_ids else '(none)'}")
        lines.append(f"     {len(target_ids)} targets, {drone_priority} priority points")

    lines.append("")
    lines.append(f"Total: {total_allocated} targets allocated, {total_priority} priority points")
    lines.append("")
    lines.append("ALLOCATION_DATA:" + json.dumps(allocation))

    return "\n".join(lines)


@tool
def allocate_with_constraints(constraints: str, strategy: str = "efficient") -> str:
    """
    Allocate targets with priority constraints.

    Args:
        constraints: Priority constraints (e.g., "D1,D2: priority>=6; D3,D4: priority<=6")
        strategy: Allocation strategy

    Returns:
        Allocation results respecting constraints.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    dist_matrix = state.get("distance_matrix")

    # Parse constraints
    priority_filters = parse_priority_constraints(constraints)

    # Build matrix data
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}
        set_allocator_matrix(matrix_data)

    # Allocate with filters
    allocation = allocate_with_priority_filters(env, configs, priority_filters, strategy, matrix_data)

    # Store in state
    state["allocation"] = allocation
    state["priority_constraints"] = constraints

    # Format result
    targets = env.get("targets", [])
    lines = [
        "=" * 50,
        "CONSTRAINED ALLOCATION COMPLETE",
        "=" * 50,
        f"Constraints: {constraints}",
        f"Strategy: {strategy}",
        ""
    ]

    total_allocated = 0
    for did in sorted(allocation.keys()):
        target_ids = allocation[did]
        total_allocated += len(target_ids)
        lines.append(f"D{did}: {', '.join(target_ids) if target_ids else '(none)'}")

    lines.append("")
    lines.append(f"Total: {total_allocated} targets allocated")
    lines.append("")
    lines.append("ALLOCATION_DATA:" + json.dumps(allocation))

    return "\n".join(lines)


ALLOCATOR_TOOLS = [allocate_targets, allocate_with_constraints]


# ============================================================================
# ROUTER AGENT
# ============================================================================

ROUTER_SYSTEM_PROMPT = """You are the ROUTER agent specializing in optimal route computation.

Your job is to compute optimal routes for drones using the Held-Karp algorithm.

You MUST call one of your tools:
- solve_all_routes: Solve routes for ALL drones in parallel (RECOMMENDED - fastest)
- solve_single_route: Solve route for one specific drone

IMPORTANT: You MUST call a tool. Do not describe what routes would look like - compute them.

The tools use the Held-Karp algorithm which finds globally optimal TSP solutions.
"""


# Global solver instance
_solver = None

def get_solver():
    """Get or create solver singleton. Returns None if solver not available."""
    global _solver
    if _solver is None and OrienteeringSolverInterface is not None:
        _solver = OrienteeringSolverInterface()
    return _solver


def is_solver_available() -> bool:
    """Check if the orienteering solver is available."""
    return OrienteeringSolverInterface is not None


@tool
def solve_all_routes() -> str:
    """
    Solve optimal routes for ALL allocated drones in parallel.

    This is the recommended approach - solves all drones at once using parallel processing.

    Returns:
        All routes with points and fuel consumption.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    allocation = state.get("allocation", {})
    dist_matrix = state.get("distance_matrix")

    if not allocation:
        return "ERROR: No allocation found. Allocation must happen first."

    if not is_solver_available():
        return "ERROR: Route optimization solver is not available in this deployment. The orienteering solver requires additional dependencies (matplotlib/tkinter) that are not installed in the cloud environment. Please use the manual route planning features instead."

    # Build matrix data for solver
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        # Build waypoints list
        waypoints = []
        for label in labels:
            for a in env.get("airports", []):
                if str(a.get("id", a.get("label"))) == label:
                    waypoints.append({"id": label, "x": a["x"], "y": a["y"]})
                    break
            for t in env.get("targets", []):
                if str(t.get("id", t.get("label"))) == label:
                    waypoints.append({
                        "id": label,
                        "x": t["x"],
                        "y": t["y"],
                        "priority": t.get("priority", 5),
                        "type": t.get("type", "a")
                    })
                    break
        matrix_data = {"labels": labels, "matrix": matrix, "waypoints": waypoints}

    targets = env.get("targets", [])
    airports = env.get("airports", [])
    sams = env.get("sams", [])
    target_map = {str(t.get("id", t.get("label"))): t for t in targets}

    solver = get_solver()
    results = {}

    def solve_drone(drone_id: str) -> Dict[str, Any]:
        """Solve route for a single drone using OrienteeringSolverInterface."""
        allocated = allocation.get(drone_id, [])
        if not allocated:
            return {"drone_id": drone_id, "status": "no_targets", "route": [], "points": 0, "distance": 0}

        cfg = configs.get(drone_id, {})
        fuel_budget = cfg.get("fuel_budget", 200)
        start_airport = cfg.get("start_airport", "A1")
        end_airport = cfg.get("end_airport", "A1")
        if end_airport == "-":
            end_airport = start_airport  # Default to returning to start

        # Get allocated targets
        drone_targets = [t for t in targets if str(t.get("id", t.get("label"))) in allocated]

        if not drone_targets:
            return {"drone_id": drone_id, "status": "no_targets", "route": [start_airport, start_airport], "points": 0, "distance": 0}

        # Build filtered distance matrix for this drone's targets
        airport_ids = [a["id"] for a in airports]
        target_ids = [t["id"] for t in drone_targets]
        desired_ids = airport_ids + target_ids

        filtered_matrix = None
        if matrix_data:
            orig_labels = matrix_data["labels"]
            orig_matrix = matrix_data["matrix"]

            filtered_indices = []
            filtered_ids = []
            for fid in desired_ids:
                if fid in orig_labels:
                    filtered_indices.append(orig_labels.index(fid))
                    filtered_ids.append(fid)

            n = len(filtered_indices)
            filt_mat = [[0.0] * n for _ in range(n)]
            for i, orig_i in enumerate(filtered_indices):
                for j, orig_j in enumerate(filtered_indices):
                    filt_mat[i][j] = orig_matrix[orig_i][orig_j]

            filt_waypoints = [wp for wp in matrix_data.get("waypoints", []) if wp["id"] in filtered_ids]
            filtered_matrix = {"labels": filtered_ids, "matrix": filt_mat, "waypoints": filt_waypoints}

        # Build environment for solver
        env_for_solver = solver.build_environment_for_solver(
            airports=airports,
            targets=drone_targets,
            sams=sams,
            distance_matrix_data=filtered_matrix
        )
        env_for_solver["start_airport"] = start_airport
        env_for_solver["end_airport"] = end_airport

        # Call solver
        result = solver.solve(env_for_solver, fuel_budget)

        route = result.get("route", [])
        visited = result.get("visited_targets", [])
        distance = result.get("distance", 0)

        # Calculate points
        points = sum(target_map[tid].get("priority", 5) for tid in visited if tid in target_map)

        if route:
            return {
                "drone_id": drone_id,
                "status": "optimal",
                "route": route,
                "points": points,
                "distance": distance,
                "fuel_budget": fuel_budget
            }
        else:
            # Fallback
            fallback_route = [start_airport] + [t["id"] for t in drone_targets[:3]] + [end_airport]
            return {
                "drone_id": drone_id,
                "status": "partial",
                "route": fallback_route,
                "points": sum(t.get("priority", 5) for t in drone_targets[:3]),
                "distance": 0,
                "fuel_budget": fuel_budget
            }

    # Get drones to solve
    drones_to_solve = [did for did in allocation.keys() if allocation[did]]

    # Solve in parallel
    with ThreadPoolExecutor(max_workers=5) as executor:
        futures = {executor.submit(solve_drone, did): did for did in drones_to_solve}
        for future in as_completed(futures):
            result = future.result()
            results[result["drone_id"]] = result

    # Store routes in state
    routes = {}
    for did, r in results.items():
        routes[did] = {
            "route": r.get("route", []),
            "sequence": ",".join(r.get("route", [])),
            "points": r.get("points", 0),
            "distance": r.get("distance", 0),
            "fuel_budget": r.get("fuel_budget", 200)
        }
    state["routes"] = routes
    print(f"📍 [ROUTER TOOL] Stored routes in state: {list(routes.keys())}", file=sys.stderr)
    sys.stderr.flush()

    # Format output
    lines = [
        "=" * 50,
        "ROUTE COMPUTATION COMPLETE",
        "=" * 50,
        ""
    ]

    total_points = 0
    total_fuel = 0.0
    all_visited = set()

    for did in sorted(results.keys()):
        r = results[did]
        route = r.get("route", [])
        points = r.get("points", 0)
        distance = r.get("distance", 0)
        fuel_budget = r.get("fuel_budget", 200)

        for wp in route:
            if wp in target_map:
                all_visited.add(wp)

        total_points += points
        total_fuel += distance

        lines.append(f"D{did}: {' -> '.join(route)}")
        lines.append(f"     Points: {points}, Fuel: {distance:.1f} / {fuel_budget}")
        lines.append("")

    # Summary
    total_possible = sum(t.get("priority", 5) for t in targets)
    unvisited = [tid for tid in target_map.keys() if tid not in all_visited]

    lines.append("-" * 50)
    lines.append(f"TOTAL: {total_points} / {total_possible} points ({100*total_points/total_possible:.1f}%)")
    lines.append(f"FUEL: {total_fuel:.1f}")

    if unvisited:
        lines.append(f"UNVISITED: {', '.join(sorted(unvisited))}")

    lines.append("")
    lines.append("=" * 50)
    lines.append("ROUTES FOR UI:")
    for did in sorted(routes.keys()):
        route = routes[did].get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    return "\n".join(lines)


@tool
def solve_single_route(drone_id: str) -> str:
    """
    Solve optimal route for a single drone.

    Args:
        drone_id: Drone ID ("1", "2", etc.)

    Returns:
        Optimal route with points and fuel.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    allocation = state.get("allocation", {})
    dist_matrix = state.get("distance_matrix")

    drone_id = drone_id.replace("D", "").replace("d", "")

    allocated = allocation.get(drone_id, [])
    if not allocated:
        return f"D{drone_id}: No targets allocated"

    cfg = configs.get(drone_id, {})
    fuel_budget = cfg.get("fuel_budget", 200)
    start_airport = cfg.get("start_airport", "A1")
    end_airport = cfg.get("end_airport", "A1")
    if end_airport == "-":
        end_airport = start_airport

    targets = env.get("targets", [])
    airports = env.get("airports", [])
    sams = env.get("sams", [])
    drone_targets = [t for t in targets if str(t.get("id", t.get("label"))) in allocated]
    target_map = {str(t.get("id", t.get("label"))): t for t in targets}

    # Build matrix data with waypoints
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        waypoints = []
        for label in labels:
            for a in airports:
                if str(a.get("id", a.get("label"))) == label:
                    waypoints.append({"id": label, "x": a["x"], "y": a["y"]})
                    break
            for t in targets:
                if str(t.get("id", t.get("label"))) == label:
                    waypoints.append({
                        "id": label, "x": t["x"], "y": t["y"],
                        "priority": t.get("priority", 5), "type": t.get("type", "a")
                    })
                    break
        matrix_data = {"labels": labels, "matrix": matrix, "waypoints": waypoints}

    # Build filtered matrix for this drone
    airport_ids = [a["id"] for a in airports]
    target_ids = [t["id"] for t in drone_targets]
    desired_ids = airport_ids + target_ids

    filtered_matrix = None
    if matrix_data:
        orig_labels = matrix_data["labels"]
        orig_matrix = matrix_data["matrix"]

        filtered_indices = []
        filtered_ids = []
        for fid in desired_ids:
            if fid in orig_labels:
                filtered_indices.append(orig_labels.index(fid))
                filtered_ids.append(fid)

        n = len(filtered_indices)
        filt_mat = [[0.0] * n for _ in range(n)]
        for i, orig_i in enumerate(filtered_indices):
            for j, orig_j in enumerate(filtered_indices):
                filt_mat[i][j] = orig_matrix[orig_i][orig_j]

        filt_waypoints = [wp for wp in matrix_data.get("waypoints", []) if wp["id"] in filtered_ids]
        filtered_matrix = {"labels": filtered_ids, "matrix": filt_mat, "waypoints": filt_waypoints}

    # Solve using OrienteeringSolverInterface
    solver = get_solver()
    env_for_solver = solver.build_environment_for_solver(
        airports=airports,
        targets=drone_targets,
        sams=sams,
        distance_matrix_data=filtered_matrix
    )
    env_for_solver["start_airport"] = start_airport
    env_for_solver["end_airport"] = end_airport

    result = solver.solve(env_for_solver, fuel_budget)

    route = result.get("route", [])
    visited = result.get("visited_targets", [])
    distance = result.get("distance", 0)

    if not route:
        return f"D{drone_id}: No feasible route found"

    # Calculate points
    points = sum(target_map[tid].get("priority", 5) for tid in visited if tid in target_map)

    # Update state
    if state.get("routes") is None:
        state["routes"] = {}
    state["routes"][drone_id] = {
        "route": route,
        "sequence": ",".join(route),
        "points": points,
        "distance": distance,
        "fuel_budget": fuel_budget
    }

    return f"""
D{drone_id} OPTIMAL ROUTE
========================
Allocated: {', '.join(allocated)}
Route: {' -> '.join(route)}
Points: {points}
Fuel: {distance:.1f} / {fuel_budget}

ROUTE_D{drone_id}: {','.join(route)}
"""


ROUTER_TOOLS = [solve_all_routes, solve_single_route]


# ============================================================================
# VALIDATOR AGENT
# ============================================================================

VALIDATOR_SYSTEM_PROMPT = """You are the VALIDATOR agent specializing in constraint checking.

Your job is to validate routes against all mission constraints:
- Fuel budgets
- Start/end airports
- Target type accessibility
- No duplicate target assignments

You MUST call one of your tools:
- validate_all_routes: Validate all routes at once (RECOMMENDED)
- check_conflicts: Check for target conflicts between drones

After validation, the workflow will complete and return results.
"""


@tool
def validate_all_routes() -> str:
    """
    Validate all computed routes against constraints.

    Returns:
        Validation results for each drone.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    routes = state.get("routes", {})
    dist_matrix = state.get("distance_matrix", {})

    if not routes:
        return "ERROR: No routes to validate"

    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    results = {}
    all_valid = True

    for drone_id, route_data in routes.items():
        route = route_data.get("route", [])
        cfg = configs.get(drone_id, {})

        errors = []

        # Check fuel
        fuel_budget = cfg.get("fuel_budget", 200)
        total_fuel = 0.0
        for i in range(len(route) - 1):
            if dist_matrix and route[i] in dist_matrix:
                total_fuel += dist_matrix[route[i]].get(route[i+1], 0)

        if total_fuel > fuel_budget:
            errors.append(f"Fuel exceeded: {total_fuel:.1f} > {fuel_budget}")

        # Check start/end airports
        start = cfg.get("start_airport", "A1")
        end = cfg.get("end_airport", "A1")

        if route and route[0] != start:
            errors.append(f"Wrong start: {route[0]} != {start}")

        if end != "-" and route and route[-1] != end:
            errors.append(f"Wrong end: {route[-1]} != {end}")

        # Check target types
        target_access = cfg.get("target_access", {})
        if target_access:
            allowed = {t.lower() for t, enabled in target_access.items() if enabled}
            for wp in route:
                if wp in targets:
                    t_type = str(targets[wp].get("type", "a")).lower()
                    if allowed and t_type not in allowed:
                        errors.append(f"Type {t_type.upper()} not allowed for {wp}")

        if errors:
            all_valid = False
            results[drone_id] = {"valid": False, "errors": errors}
        else:
            results[drone_id] = {"valid": True, "fuel": total_fuel, "fuel_budget": fuel_budget}

    # Store validation
    state["validation_results"] = results

    # Format output
    lines = [
        "=" * 50,
        "VALIDATION RESULTS",
        "=" * 50,
        ""
    ]

    for did in sorted(results.keys()):
        r = results[did]
        if r["valid"]:
            lines.append(f"D{did}: ✅ VALID (fuel: {r['fuel']:.1f}/{r['fuel_budget']})")
        else:
            lines.append(f"D{did}: ❌ INVALID")
            for err in r["errors"]:
                lines.append(f"     - {err}")

    lines.append("")
    lines.append("=" * 50)

    if all_valid:
        lines.append("ALL ROUTES VALID - Ready for execution")
    else:
        lines.append("VALIDATION FAILED - Routes need adjustment")

    return "\n".join(lines)


@tool
def check_conflicts() -> str:
    """
    Check if any targets are assigned to multiple drones.

    Returns:
        Conflict report.
    """
    state = get_state()
    routes = state.get("routes", {})

    target_assignments = {}
    for did, route_data in routes.items():
        route = route_data.get("route", [])
        for wp in route:
            if wp.startswith("T"):
                if wp not in target_assignments:
                    target_assignments[wp] = []
                target_assignments[wp].append(did)

    conflicts = {t: drones for t, drones in target_assignments.items() if len(drones) > 1}

    if conflicts:
        lines = ["CONFLICTS FOUND:"]
        for target, drones in conflicts.items():
            lines.append(f"  {target} -> D{', D'.join(drones)}")
        return "\n".join(lines)
    else:
        return "No conflicts: Each target assigned to at most one drone"


VALIDATOR_TOOLS = [validate_all_routes, check_conflicts]


# ============================================================================
# OPTIMIZER AGENT (Optional - for post-optimization)
# ============================================================================

OPTIMIZER_SYSTEM_PROMPT = """You are the OPTIMIZER agent for post-optimization.

Your job is to improve existing routes using available tools:

Tools available:
- optimize_routes: Run all optimizations at once (recommended for general use)
- swap_closer_optimize: "Swap Closer" - move targets to closer drone trajectories
- remove_crossings: "2-opt" - remove route crossings/self-intersections
- insert_unvisited: Add unvisited targets to drones that have fuel capacity and can visit them

CRITICAL: Look for "INSTRUCTION TO OPTIMIZER" in the conversation. If you see it, call EXACTLY the tool specified.

If no specific instruction, choose based on what the user requested:
- "Swap Closer" or "swap optimization" → swap_closer_optimize
- "crossing removal" or "2-opt" → remove_crossings
- "insert unvisited" or "add unvisited targets" → insert_unvisited
- general "optimize" → optimize_routes

You MUST call one tool. Do NOT just describe what you would do.
"""


@tool
def optimize_routes() -> str:
    """
    Run ALL optimizations on computed routes in sequence.

    This runs:
    1. Swap Closer - move targets to closer drone trajectories
    2. Remove Crossings - 2-opt to remove self-intersections
    3. Insert Unvisited - add unvisited targets where possible

    Only keeps changes that improve efficiency.

    Returns:
        Optimized routes with all improvements applied.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    routes = state.get("routes", {})
    dist_matrix = state.get("distance_matrix")
    priority_constraints = state.get("priority_constraints", "")

    if not routes:
        return "ERROR: No routes to optimize"

    # Build solution dict
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    solution = {"routes": {}}

    for did, route_data in routes.items():
        route = route_data.get("route", [])
        cfg = configs.get(did, {})

        points = sum(targets[wp].get("priority", 5) for wp in route if wp in targets)
        distance = route_data.get("distance", 0)

        solution["routes"][did] = {
            "route": route,
            "points": points,
            "distance": distance,
            "fuel_budget": cfg.get("fuel_budget", 200)
        }

    # Build matrix
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Track what optimizations were applied
    optimizations_applied = []

    # Calculate initial metrics
    initial_fuel = sum(r.get("distance", 0) for r in solution["routes"].values())
    initial_points = sum(r.get("points", 0) for r in solution["routes"].values())

    # 1. Run Swap Closer optimization
    swap_result = trajectory_swap_optimize(solution, env, configs, matrix_data)
    swap_fuel = sum(r.get("distance", 0) for r in swap_result.get("routes", {}).values())
    if swap_fuel < initial_fuel:
        solution = swap_result
        optimizations_applied.append("Swap Closer")

    # 2. Run Crossing Removal (2-opt) optimization
    crossing_result = crossing_removal_optimize(solution, env, configs, matrix_data)
    crossing_fuel = sum(r.get("distance", 0) for r in crossing_result.get("routes", {}).values())
    current_fuel = sum(r.get("distance", 0) for r in solution["routes"].values())
    if crossing_fuel < current_fuel:
        solution = crossing_result
        optimizations_applied.append("Remove Crossings")

    # 3. Run Insert Unvisited optimization
    insert_result = post_optimize_solution(solution, env, configs, matrix_data, priority_constraints)
    insert_points = sum(r.get("points", 0) for r in insert_result.get("routes", {}).values())
    current_points = sum(r.get("points", 0) for r in solution["routes"].values())
    if insert_points > current_points:
        solution = insert_result
        optimizations_applied.append("Insert Unvisited")

    # Update state
    new_routes = {}
    for did, route_data in solution.get("routes", {}).items():
        new_routes[did] = {
            "route": route_data.get("route", []),
            "sequence": ",".join(route_data.get("route", [])),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0),
            "fuel_budget": route_data.get("fuel_budget", 200)
        }
    state["routes"] = new_routes

    # Calculate final metrics
    final_fuel = sum(r.get("distance", 0) for r in new_routes.values())
    final_points = sum(r.get("points", 0) for r in new_routes.values())

    # Format output
    lines = [
        "=" * 50,
        "ALL OPTIMIZATIONS COMPLETE",
        "=" * 50,
        "",
        f"Optimizations applied: {', '.join(optimizations_applied) if optimizations_applied else 'None (already optimal)'}",
        f"Fuel: {initial_fuel:.1f} → {final_fuel:.1f} ({initial_fuel - final_fuel:.1f} saved)",
        f"Points: {initial_points} → {final_points}",
        ""
    ]

    for did in sorted(new_routes.keys()):
        r = new_routes[did]
        lines.append(f"D{did}: {','.join(r['route'])}")
        lines.append(f"     Points: {r['points']}, Fuel: {r['distance']:.1f}")

    lines.append("")
    lines.append("OPTIMIZED ROUTES:")
    for did in sorted(new_routes.keys()):
        route = new_routes[did].get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    return "\n".join(lines)


@tool
def swap_closer_optimize() -> str:
    """
    Run trajectory swap optimization - moves targets to closer drone trajectories.

    This specifically does the "Swap Closer" optimization where targets are
    reassigned to drones whose trajectory passes closer to them.

    Only applies the optimization if it improves efficiency (reduces fuel).

    Returns:
        Optimized routes with swap improvements, or rejection message if no improvement.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    routes = state.get("routes", {})
    dist_matrix = state.get("distance_matrix")

    if not routes:
        return "ERROR: No routes to optimize"

    # Build solution dict
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    solution = {"routes": {}}

    for did, route_data in routes.items():
        route = route_data.get("route", [])
        cfg = configs.get(did, {})
        points = sum(targets[wp].get("priority", 5) for wp in route if wp in targets)
        distance = route_data.get("distance", 0)

        solution["routes"][did] = {
            "route": route,
            "points": points,
            "distance": distance,
            "fuel_budget": cfg.get("fuel_budget", 200)
        }

    # Build matrix
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Calculate initial metrics
    initial_fuel = sum(r.get("distance", 0) for r in solution["routes"].values())
    initial_points = sum(r.get("points", 0) for r in solution["routes"].values())

    # Run swap optimization only
    optimized = trajectory_swap_optimize(solution, env, configs, matrix_data)

    # Calculate optimized metrics
    optimized_fuel = sum(r.get("distance", 0) for r in optimized.get("routes", {}).values())
    optimized_points = sum(r.get("points", 0) for r in optimized.get("routes", {}).values())

    # Check if optimization improves efficiency (less fuel, same or more points)
    improves_fuel = optimized_fuel < initial_fuel
    maintains_points = optimized_points >= initial_points

    if not (improves_fuel and maintains_points):
        # Reject optimization - no improvement
        lines = [
            "=" * 50,
            "SWAP CLOSER OPTIMIZATION REJECTED",
            "=" * 50,
            "",
            "Reason: Optimization does not improve efficiency",
            f"Fuel: {initial_fuel:.1f} → {optimized_fuel:.1f} (would {'save' if improves_fuel else 'increase'} {abs(initial_fuel - optimized_fuel):.1f})",
            f"Points: {initial_points} → {optimized_points}",
            "",
            "Original routes preserved (no changes applied).",
            ""
        ]
        for did in sorted(routes.keys()):
            r = routes[did]
            lines.append(f"ROUTE_D{did}: {','.join(r.get('route', []))}")
        return "\n".join(lines)

    # Update state - optimization improves efficiency
    new_routes = {}
    for did, route_data in optimized.get("routes", {}).items():
        new_routes[did] = {
            "route": route_data.get("route", []),
            "sequence": ",".join(route_data.get("route", [])),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0),
            "fuel_budget": route_data.get("fuel_budget", 200)
        }
    state["routes"] = new_routes

    # Format output
    lines = [
        "=" * 50,
        "SWAP CLOSER OPTIMIZATION COMPLETE",
        "=" * 50,
        "",
        f"Fuel saved: {initial_fuel - optimized_fuel:.1f} ({initial_fuel:.1f} → {optimized_fuel:.1f})",
        f"Points: {initial_points} → {optimized_points}",
        ""
    ]

    for did in sorted(new_routes.keys()):
        r = new_routes[did]
        lines.append(f"D{did}: {','.join(r['route'])}")
        lines.append(f"     Points: {r['points']}, Fuel: {r['distance']:.1f}")

    lines.append("")
    lines.append("OPTIMIZED ROUTES:")
    for did in sorted(new_routes.keys()):
        route = new_routes[did].get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    return "\n".join(lines)


@tool
def remove_crossings() -> str:
    """
    Run 2-opt crossing removal optimization.

    This removes self-crossings in drone trajectories where segments
    cross over each other. Uses the 2-opt algorithm.

    Only applies the optimization if it improves efficiency (reduces fuel).

    Returns:
        Optimized routes with crossings removed, or rejection message if no improvement.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    routes = state.get("routes", {})
    dist_matrix = state.get("distance_matrix")

    if not routes:
        return "ERROR: No routes to optimize"

    # Build solution dict
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    solution = {"routes": {}}

    for did, route_data in routes.items():
        route = route_data.get("route", [])
        cfg = configs.get(did, {})
        points = sum(targets[wp].get("priority", 5) for wp in route if wp in targets)
        distance = route_data.get("distance", 0)

        solution["routes"][did] = {
            "route": route,
            "points": points,
            "distance": distance,
            "fuel_budget": cfg.get("fuel_budget", 200)
        }

    # Build matrix
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Calculate initial metrics
    initial_fuel = sum(r.get("distance", 0) for r in solution["routes"].values())
    initial_points = sum(r.get("points", 0) for r in solution["routes"].values())

    # Run crossing removal only
    optimized = crossing_removal_optimize(solution, env, configs, matrix_data)

    # Calculate optimized metrics
    optimized_fuel = sum(r.get("distance", 0) for r in optimized.get("routes", {}).values())
    optimized_points = sum(r.get("points", 0) for r in optimized.get("routes", {}).values())

    # Check if optimization improves efficiency (less fuel, same or more points)
    improves_fuel = optimized_fuel < initial_fuel
    maintains_points = optimized_points >= initial_points

    if not (improves_fuel and maintains_points):
        # Reject optimization - no improvement
        lines = [
            "=" * 50,
            "CROSSING REMOVAL OPTIMIZATION REJECTED",
            "=" * 50,
            "",
            "Reason: Optimization does not improve efficiency",
            f"Fuel: {initial_fuel:.1f} → {optimized_fuel:.1f} (would {'save' if improves_fuel else 'increase'} {abs(initial_fuel - optimized_fuel):.1f})",
            f"Points: {initial_points} → {optimized_points}",
            "",
            "Original routes preserved (no changes applied).",
            ""
        ]
        for did in sorted(routes.keys()):
            r = routes[did]
            lines.append(f"ROUTE_D{did}: {','.join(r.get('route', []))}")
        return "\n".join(lines)

    # Update state - optimization improves efficiency
    new_routes = {}
    for did, route_data in optimized.get("routes", {}).items():
        new_routes[did] = {
            "route": route_data.get("route", []),
            "sequence": ",".join(route_data.get("route", [])),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0),
            "fuel_budget": route_data.get("fuel_budget", 200)
        }
    state["routes"] = new_routes

    # Format output
    lines = [
        "=" * 50,
        "CROSSING REMOVAL COMPLETE",
        "=" * 50,
        "",
        f"Fuel saved: {initial_fuel - optimized_fuel:.1f} ({initial_fuel:.1f} → {optimized_fuel:.1f})",
        f"Points: {initial_points} → {optimized_points}",
        ""
    ]

    for did in sorted(new_routes.keys()):
        r = new_routes[did]
        lines.append(f"D{did}: {','.join(r['route'])}")
        lines.append(f"     Points: {r['points']}, Fuel: {r['distance']:.1f}")

    lines.append("")
    lines.append("OPTIMIZED ROUTES:")
    for did in sorted(new_routes.keys()):
        route = new_routes[did].get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    return "\n".join(lines)


@tool
def insert_unvisited() -> str:
    """
    Insert unvisited targets into drone routes.

    This optimization finds targets that were not assigned to any drone
    and inserts them into routes where:
    - The drone has remaining fuel capacity
    - The drone is allowed to visit that target type
    - The insertion minimizes additional fuel cost

    Only applies the optimization if it improves efficiency (increases points covered).

    Returns:
        Updated routes with unvisited targets inserted, or rejection message if no improvement.
    """
    state = get_state()
    env = state.get("environment", {})
    configs = state.get("drone_configs", {})
    routes = state.get("routes", {})
    dist_matrix = state.get("distance_matrix")

    if not routes:
        return "ERROR: No routes to optimize"

    # Build solution dict
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    solution = {"routes": {}}

    for did, route_data in routes.items():
        route = route_data.get("route", [])
        cfg = configs.get(did, {})
        points = sum(targets[wp].get("priority", 5) for wp in route if wp in targets)
        distance = route_data.get("distance", 0)

        solution["routes"][did] = {
            "route": route,
            "points": points,
            "distance": distance,
            "fuel_budget": cfg.get("fuel_budget", 200)
        }

    # Build matrix
    matrix_data = None
    if dist_matrix:
        labels = list(dist_matrix.keys())
        matrix = [[dist_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Calculate initial metrics
    initial_fuel = sum(r.get("distance", 0) for r in solution["routes"].values())
    initial_points = sum(r.get("points", 0) for r in solution["routes"].values())

    # Run post_optimize_solution which handles inserting unvisited targets
    optimized = post_optimize_solution(solution, env, configs, matrix_data)

    # Calculate optimized metrics
    optimized_fuel = sum(r.get("distance", 0) for r in optimized.get("routes", {}).values())
    optimized_points = sum(r.get("points", 0) for r in optimized.get("routes", {}).values())

    # Count targets before and after
    original_visited = set()
    for did, route_data in routes.items():
        for wp in route_data.get("route", []):
            if wp in targets:
                original_visited.add(wp)

    optimized_visited = set()
    for did, route_data in optimized.get("routes", {}).items():
        for wp in route_data.get("route", []):
            if wp in targets:
                optimized_visited.add(wp)

    inserted_count = len(optimized_visited) - len(original_visited)

    # Check if optimization improves efficiency (more points, or same points but less fuel)
    improves_points = optimized_points > initial_points
    same_points_less_fuel = (optimized_points == initial_points and optimized_fuel < initial_fuel)

    if not (improves_points or same_points_less_fuel):
        # Reject optimization - no improvement
        all_target_ids = set(targets.keys())
        remaining_unvisited = len(all_target_ids) - len(original_visited)
        lines = [
            "=" * 50,
            "INSERT UNVISITED OPTIMIZATION REJECTED",
            "=" * 50,
            "",
            "Reason: Optimization does not improve efficiency",
            f"Points: {initial_points} → {optimized_points} (no increase)",
            f"Fuel: {initial_fuel:.1f} → {optimized_fuel:.1f}",
            f"Targets that could be inserted: {inserted_count}",
            f"Remaining unvisited: {remaining_unvisited}",
            "",
            "Original routes preserved (no changes applied).",
            ""
        ]
        for did in sorted(routes.keys()):
            r = routes[did]
            lines.append(f"ROUTE_D{did}: {','.join(r.get('route', []))}")
        return "\n".join(lines)

    # Update state - optimization improves efficiency
    new_routes = {}
    for did, route_data in optimized.get("routes", {}).items():
        new_routes[did] = {
            "route": route_data.get("route", []),
            "sequence": ",".join(route_data.get("route", [])),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0),
            "fuel_budget": route_data.get("fuel_budget", 200)
        }
    state["routes"] = new_routes

    # Count remaining unvisited
    all_target_ids = set(targets.keys())
    remaining_unvisited = len(all_target_ids) - len(optimized_visited)

    # Format output
    lines = [
        "=" * 50,
        "INSERT UNVISITED TARGETS COMPLETE",
        "=" * 50,
        "",
        f"Targets inserted: {inserted_count}",
        f"Points gained: {optimized_points - initial_points} ({initial_points} → {optimized_points})",
        f"Fuel change: {optimized_fuel - initial_fuel:.1f} ({initial_fuel:.1f} → {optimized_fuel:.1f})",
        f"Remaining unvisited: {remaining_unvisited}",
        ""
    ]

    for did in sorted(new_routes.keys()):
        r = new_routes[did]
        lines.append(f"D{did}: {','.join(r['route'])}")
        lines.append(f"     Points: {r['points']}, Fuel: {r['distance']:.1f}")

    lines.append("")
    lines.append("OPTIMIZED ROUTES:")
    for did in sorted(new_routes.keys()):
        route = new_routes[did].get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    return "\n".join(lines)


OPTIMIZER_TOOLS = [optimize_routes, swap_closer_optimize, remove_crossings, insert_unvisited]


# ============================================================================
# Build LangGraph Workflow
# ============================================================================

def create_multi_agent_workflow():
    """
    Create the multi-agent LangGraph workflow.

    Key design decisions:
    1. Each agent node invokes LLM with tool_choice={"type": "any"} to force tool use
    2. Explicit phase transitions via state["phase"]
    3. Tool nodes execute tools and return results
    4. Routing based on phase, not message content parsing
    """

    # Create LLM with forced tool calling
    llm = ChatAnthropic(
        model="claude-sonnet-4-20250514",
        temperature=0,
        max_tokens=4096,
    )

    # ========== Agent Nodes ==========

    def coordinator_node(state: MissionState) -> Dict[str, Any]:
        """Coordinator analyzes request and starts workflow."""
        print("\n🎯 [COORDINATOR] Analyzing mission request...", file=sys.stderr)
        sys.stderr.flush()

        # Set global state for tool access
        set_state(state)

        messages = state.get("messages", [])
        system = SystemMessage(content=COORDINATOR_SYSTEM_PROMPT)

        # Build messages with system prompt
        if messages and isinstance(messages[0], SystemMessage):
            agent_messages = [system] + messages[1:]
        else:
            agent_messages = [system] + list(messages)

        # Let LLM decide whether to call a tool or respond directly
        coordinator_llm = llm.bind_tools(COORDINATOR_TOOLS, tool_choice="auto")
        response = coordinator_llm.invoke(agent_messages)

        return {"messages": [response], "phase": AgentPhase.UNDERSTANDING}

    def allocator_node(state: MissionState) -> Dict[str, Any]:
        """Allocator distributes targets to drones."""
        print("\n🎯 [ALLOCATOR] Distributing targets...", file=sys.stderr)
        sys.stderr.flush()

        set_state(state)

        messages = state.get("messages", [])
        system = SystemMessage(content=ALLOCATOR_SYSTEM_PROMPT)

        if messages and isinstance(messages[0], SystemMessage):
            agent_messages = [system] + messages[1:]
        else:
            agent_messages = [system] + list(messages)

        # Force tool call - use {"type": "any"} for Anthropic
        allocator_llm = llm.bind_tools(ALLOCATOR_TOOLS, tool_choice={"type": "any"})
        response = allocator_llm.invoke(agent_messages)

        return {"messages": [response], "phase": AgentPhase.ALLOCATING}

    def router_node(state: MissionState) -> Dict[str, Any]:
        """Router computes optimal routes."""
        print("\n🎯 [ROUTER] Computing optimal routes...", file=sys.stderr)
        sys.stderr.flush()

        set_state(state)

        messages = state.get("messages", [])
        system = SystemMessage(content=ROUTER_SYSTEM_PROMPT)

        if messages and isinstance(messages[0], SystemMessage):
            agent_messages = [system] + messages[1:]
        else:
            agent_messages = [system] + list(messages)

        # Force tool call - use {"type": "any"} for Anthropic
        router_llm = llm.bind_tools(ROUTER_TOOLS, tool_choice={"type": "any"})
        response = router_llm.invoke(agent_messages)

        return {"messages": [response], "phase": AgentPhase.ROUTING}

    def validator_node(state: MissionState) -> Dict[str, Any]:
        """Validator checks constraints."""
        print("\n🎯 [VALIDATOR] Checking constraints...", file=sys.stderr)
        sys.stderr.flush()

        set_state(state)

        messages = state.get("messages", [])
        system = SystemMessage(content=VALIDATOR_SYSTEM_PROMPT)

        if messages and isinstance(messages[0], SystemMessage):
            agent_messages = [system] + messages[1:]
        else:
            agent_messages = [system] + list(messages)

        # Force tool call - use {"type": "any"} for Anthropic
        validator_llm = llm.bind_tools(VALIDATOR_TOOLS, tool_choice={"type": "any"})
        response = validator_llm.invoke(agent_messages)

        return {"messages": [response], "phase": AgentPhase.VALIDATING}

    def optimizer_node(state: MissionState) -> Dict[str, Any]:
        """Optimizer improves solution with post-optimization."""
        print("\n🎯 [OPTIMIZER] Running post-optimization...", file=sys.stderr)
        sys.stderr.flush()

        set_state(state)

        messages = state.get("messages", [])
        system = SystemMessage(content=OPTIMIZER_SYSTEM_PROMPT)

        if messages and isinstance(messages[0], SystemMessage):
            agent_messages = [system] + messages[1:]
        else:
            agent_messages = [system] + list(messages)

        # Force tool call - use {"type": "any"} for Anthropic
        optimizer_llm = llm.bind_tools(OPTIMIZER_TOOLS, tool_choice={"type": "any"})
        response = optimizer_llm.invoke(agent_messages)

        return {"messages": [response], "phase": AgentPhase.OPTIMIZING}

    # ========== Tool Nodes ==========
    # Custom tool nodes that propagate state changes from _current_state back to LangGraph

    def create_stateful_tool_node(tools, name: str):
        """Create a tool node that captures state changes from tools."""
        base_node = ToolNode(tools)

        def stateful_node(state: MissionState) -> Dict[str, Any]:
            # Execute the tool
            result = base_node.invoke(state)

            # Get any state changes from _current_state
            global _current_state
            state_updates = {"messages": result.get("messages", [])}

            # Propagate allocation/routes from _current_state if set
            if _current_state:
                if _current_state.get("allocation") is not None:
                    state_updates["allocation"] = _current_state["allocation"]
                    print(f"📍 [{name}] Propagating allocation: {list(_current_state['allocation'].keys())}", file=sys.stderr)
                if _current_state.get("routes") is not None:
                    state_updates["routes"] = _current_state["routes"]
                    print(f"📍 [{name}] Propagating routes: {list(_current_state['routes'].keys())}", file=sys.stderr)
                if _current_state.get("validation_results") is not None:
                    state_updates["validation_results"] = _current_state["validation_results"]
                sys.stderr.flush()

            return state_updates

        return stateful_node

    coordinator_tools_node = create_stateful_tool_node(COORDINATOR_TOOLS, "COORDINATOR_TOOLS")
    allocator_tools_node = create_stateful_tool_node(ALLOCATOR_TOOLS, "ALLOCATOR_TOOLS")
    router_tools_node = create_stateful_tool_node(ROUTER_TOOLS, "ROUTER_TOOLS")
    validator_tools_node = create_stateful_tool_node(VALIDATOR_TOOLS, "VALIDATOR_TOOLS")
    optimizer_tools_node = create_stateful_tool_node(OPTIMIZER_TOOLS, "OPTIMIZER_TOOLS")

    # ========== Routing Functions ==========

    def after_coordinator(state: MissionState) -> str:
        """Route after coordinator."""
        messages = state.get("messages", [])
        if messages:
            last = messages[-1]
            if hasattr(last, "tool_calls") and last.tool_calls:
                return "coordinator_tools"
        # No tool call = direct response (question answered), end workflow
        return END

    def after_coordinator_tools(state: MissionState) -> str:
        """After coordinator tools, route based on which tool was called."""
        messages = state.get("messages", [])

        # Find the most recent tool message to see which tool was called
        for msg in reversed(messages):
            if isinstance(msg, ToolMessage):
                content = msg.content if hasattr(msg, "content") else ""
                # If optimization was requested, go directly to optimizer
                if "OPTIMIZATION REQUEST DETECTED" in content:
                    return "optimizer"
                # If mission info was requested, go back to coordinator to formulate response
                if "CURRENT MISSION CONFIGURATION" in content:
                    return "coordinator"  # Let coordinator summarize and respond
                break

        # Default: go to allocator for new solve requests
        return "allocator"

    def after_allocator(state: MissionState) -> str:
        """Route after allocator."""
        messages = state.get("messages", [])
        if messages:
            last = messages[-1]
            if hasattr(last, "tool_calls") and last.tool_calls:
                return "allocator_tools"
        return "router"  # Go to router after allocation

    def after_allocator_tools(state: MissionState) -> str:
        """After allocator tools, go to router."""
        return "router"

    def after_router(state: MissionState) -> str:
        """Route after router."""
        messages = state.get("messages", [])
        if messages:
            last = messages[-1]
            if hasattr(last, "tool_calls") and last.tool_calls:
                return "router_tools"
        return "validator"  # Go to validator after routing

    def after_router_tools(state: MissionState) -> str:
        """After router tools, go to validator."""
        return "validator"

    def after_validator(state: MissionState) -> str:
        """Route after validator."""
        messages = state.get("messages", [])
        if messages:
            last = messages[-1]
            if hasattr(last, "tool_calls") and last.tool_calls:
                return "validator_tools"
        return "optimizer"  # Go to optimizer after validation

    def after_validator_tools(state: MissionState) -> str:
        """After validator tools, go to optimizer."""
        return "optimizer"

    def after_optimizer(state: MissionState) -> str:
        """Route after optimizer."""
        messages = state.get("messages", [])
        if messages:
            last = messages[-1]
            if hasattr(last, "tool_calls") and last.tool_calls:
                return "optimizer_tools"
        return END  # Done after optimization

    def after_optimizer_tools(state: MissionState) -> str:
        """After optimizer tools, we're done."""
        return END

    # ========== Build Graph ==========

    workflow = StateGraph(MissionState)

    # Add nodes
    workflow.add_node("coordinator", coordinator_node)
    workflow.add_node("coordinator_tools", coordinator_tools_node)
    workflow.add_node("allocator", allocator_node)
    workflow.add_node("allocator_tools", allocator_tools_node)
    workflow.add_node("router", router_node)
    workflow.add_node("router_tools", router_tools_node)
    workflow.add_node("validator", validator_node)
    workflow.add_node("validator_tools", validator_tools_node)
    workflow.add_node("optimizer", optimizer_node)
    workflow.add_node("optimizer_tools", optimizer_tools_node)

    # Set entry point
    workflow.set_entry_point("coordinator")

    # Add edges
    workflow.add_conditional_edges("coordinator", after_coordinator, {
        "coordinator_tools": "coordinator_tools",
        END: END
    })
    workflow.add_conditional_edges("coordinator_tools", after_coordinator_tools, {
        "allocator": "allocator",
        "optimizer": "optimizer",
        "coordinator": "coordinator"  # For answering questions with get_mission_info
    })

    workflow.add_conditional_edges("allocator", after_allocator, {
        "allocator_tools": "allocator_tools",
        "router": "router"
    })
    workflow.add_conditional_edges("allocator_tools", after_allocator_tools, {
        "router": "router"
    })

    workflow.add_conditional_edges("router", after_router, {
        "router_tools": "router_tools",
        "validator": "validator"
    })
    workflow.add_conditional_edges("router_tools", after_router_tools, {
        "validator": "validator"
    })

    workflow.add_conditional_edges("validator", after_validator, {
        "validator_tools": "validator_tools",
        "optimizer": "optimizer"
    })
    workflow.add_conditional_edges("validator_tools", after_validator_tools, {
        "optimizer": "optimizer"
    })

    workflow.add_conditional_edges("optimizer", after_optimizer, {
        "optimizer_tools": "optimizer_tools",
        END: END
    })
    workflow.add_conditional_edges("optimizer_tools", after_optimizer_tools, {
        END: END
    })

    return workflow.compile()


# ============================================================================
# Entry Point - Same interface as v1/v2 for compatibility
# ============================================================================

_workflow = None


def get_workflow():
    """Get or create workflow singleton."""
    global _workflow
    if _workflow is None:
        _workflow = create_multi_agent_workflow()
    return _workflow


def run_isr_agent(
    env: Dict[str, Any],
    user_query: str,
    drone_configs: Optional[Dict[str, Any]] = None,
    sequences: Optional[Dict[str, str]] = None
) -> Dict[str, Any]:
    """
    Run the multi-agent ISR system.

    This is the main entry point - same interface as v1/v2 for compatibility.

    Args:
        env: Environment data (airports, targets, sams)
        user_query: User's mission planning request
        drone_configs: Drone configurations
        sequences: Current sequences (for context)

    Returns:
        Dict with response, routes, trajectories, and statistics
    """
    drone_configs = drone_configs or {}

    # Compute distance matrix
    sams = env.get("sams", [])
    if sams:
        print("🎯 Computing SAM-aware distance matrix...", file=sys.stderr)
        dist_data = calculate_sam_aware_matrix(env)
        # Convert to dict format
        labels = dist_data.get("labels", [])
        matrix = dist_data.get("matrix", [])
        dist_matrix = {}
        for i, from_id in enumerate(labels):
            dist_matrix[from_id] = {}
            for j, to_id in enumerate(labels):
                dist_matrix[from_id][to_id] = matrix[i][j]
    else:
        # Simple Euclidean
        dist_matrix = _compute_euclidean_matrix(env)

    # Build initial state
    initial_state: MissionState = {
        "messages": [HumanMessage(content=user_query)],
        "phase": AgentPhase.START,
        "environment": env,
        "drone_configs": drone_configs,
        "distance_matrix": dist_matrix,
        "allocation": None,
        "routes": None,
        "validation_results": None,
        "optimization_results": None,
        "error": None,
        "retry_count": 0,
        "user_request": user_query,
        "priority_constraints": None,
    }

    # Get workflow
    workflow = get_workflow()

    # Run
    config = {"recursion_limit": 50}

    print(f"\n{'='*60}", file=sys.stderr)
    print(f"🚀 MULTI-AGENT V3 WORKFLOW STARTED", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    sys.stderr.flush()

    final_state = None
    step_count = 0

    for step in workflow.stream(initial_state, config=config):
        step_count += 1
        for node_name, node_output in step.items():
            phase = node_output.get("phase", "?") if isinstance(node_output, dict) else "?"
            print(f"[Step {step_count}] Node: {node_name} | Phase: {phase}", file=sys.stderr)
            sys.stderr.flush()
        final_state = step

    print(f"{'='*60}", file=sys.stderr)
    print(f"🏁 WORKFLOW COMPLETED after {step_count} steps", file=sys.stderr)
    print(f"{'='*60}\n", file=sys.stderr)
    sys.stderr.flush()

    # Extract results from final state
    return _extract_results(final_state, env, dist_matrix)


def _compute_euclidean_matrix(env: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
    """Compute simple Euclidean distance matrix."""
    waypoints = {}
    for airport in env.get("airports", []):
        aid = airport.get("id", airport.get("label", "A1"))
        waypoints[aid] = (airport["x"], airport["y"])

    for target in env.get("targets", []):
        tid = target.get("id", target.get("label", "T?"))
        waypoints[tid] = (target["x"], target["y"])

    matrix = {}
    for from_id, from_pos in waypoints.items():
        matrix[from_id] = {}
        for to_id, to_pos in waypoints.items():
            if from_id == to_id:
                matrix[from_id][to_id] = 0.0
            else:
                dist = math.hypot(to_pos[0] - from_pos[0], to_pos[1] - from_pos[1])
                matrix[from_id][to_id] = round(dist, 2)

    return matrix


def _extract_results(
    final_state: Dict[str, Any],
    env: Dict[str, Any],
    dist_matrix: Dict[str, Dict[str, float]]
) -> Dict[str, Any]:
    """Extract results from final workflow state."""

    # Get state from last step
    state = None
    for node_output in final_state.values():
        if isinstance(node_output, dict):
            state = node_output
            break

    if not state:
        return {
            "response": "Workflow failed to produce results",
            "routes": {},
            "trajectories": {},
            "total_points": 0,
            "total_fuel": 0,
            "allocation": {},
        }

    # Get global state (tools may have updated it)
    global _current_state
    print(f"📍 [EXTRACT] _current_state routes: {_current_state.get('routes') if _current_state else 'None'}", file=sys.stderr)
    if _current_state:
        state = _current_state

    # Extract routes - handle None explicitly
    routes = state.get("routes") or {}
    print(f"📍 [EXTRACT] Final routes: {list(routes.keys()) if routes else 'empty'}", file=sys.stderr)
    sys.stderr.flush()
    allocation = state.get("allocation") or {}

    # Build response text
    response_lines = []

    if routes:
        response_lines.append("MULTI-AGENT MISSION PLANNING COMPLETE")
        response_lines.append("=" * 50)
        response_lines.append("")

        for did in sorted(routes.keys()):
            route_data = routes[did]
            route = route_data.get("route", [])
            if route:
                response_lines.append(f"ROUTE_D{did}: {','.join(route)}")

        response_text = "\n".join(response_lines)
    else:
        # No routes - extract coordinator's direct response from messages
        messages = state.get("messages", [])
        response_text = ""
        for msg in reversed(messages):
            if hasattr(msg, "content") and msg.content and not hasattr(msg, "tool_calls"):
                # Found an AI response without tool calls
                response_text = msg.content
                break
            elif hasattr(msg, "content") and msg.content:
                response_text = msg.content
                break
        if not response_text:
            response_text = "No response generated"

    # Calculate totals and build trajectories using SAM-avoiding paths
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    total_points = 0
    total_fuel = 0.0
    visited = set()
    trajectories = {}

    # Build waypoint position map for trajectory planner
    waypoint_positions = {}
    for airport in env.get("airports", []):
        aid = airport.get("id", airport.get("label"))
        waypoint_positions[aid] = [float(airport["x"]), float(airport["y"])]
    for target in env.get("targets", []):
        tid = target.get("id", target.get("label"))
        waypoint_positions[tid] = [float(target["x"]), float(target["y"])]

    # Initialize trajectory planner with SAMs for polygon-respecting paths
    sams = env.get("sams", [])
    trajectory_planner = ISRTrajectoryPlanner(sams) if sams else None

    for drone_id, route_data in routes.items():
        route = route_data.get("route", [])

        for i, wp in enumerate(route):
            if wp in targets and wp not in visited:
                total_points += targets[wp].get("priority", targets[wp].get("value", 5))
                visited.add(wp)

            if i < len(route) - 1 and wp in dist_matrix:
                total_fuel += dist_matrix[wp].get(route[i + 1], 0)

        # Generate SAM-avoiding trajectory using ISRTrajectoryPlanner
        if trajectory_planner and len(route) >= 2:
            trajectory = trajectory_planner.generate_trajectory(route, waypoint_positions, drone_id)
        else:
            # Fallback: direct waypoint connections (no SAMs)
            trajectory = []
            for wp in route:
                pos = waypoint_positions.get(wp)
                if pos and (not trajectory or trajectory[-1] != pos):
                    trajectory.append(pos)

        trajectories[drone_id] = trajectory

    # Convert routes to the expected format
    routes_output = {}
    for did, route_data in routes.items():
        routes_output[did] = route_data.get("route", [])

    return {
        "response": response_text,
        "routes": routes_output,
        "trajectories": trajectories,
        "total_points": total_points,
        "total_fuel": total_fuel,
        "route": routes_output.get("1"),  # Legacy single-drone field
        "points": total_points,
        "fuel": total_fuel,
        "allocation": allocation,
    }


def _get_waypoint_position(wp_id: str, env: Dict[str, Any]) -> Optional[List[float]]:
    """Get waypoint position from environment."""
    for airport in env.get("airports", []):
        aid = airport.get("id", airport.get("label"))
        if aid == wp_id:
            return [float(airport["x"]), float(airport["y"])]

    for target in env.get("targets", []):
        tid = target.get("id", target.get("label"))
        if tid == wp_id:
            return [float(target["x"]), float(target["y"])]

    return None
