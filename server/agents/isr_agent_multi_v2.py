"""
ISR Multi-Agent Mission Planning System (v2)

A LangGraph-based multi-agent architecture with specialized agents:
1. Coordinator Agent - Task decomposition and orchestration
2. Allocator Agent - Target allocation specialist
3. Router Agent - Optimal route computation
4. Validator Agent - Constraint checking and validation
5. Optimizer Agent - Post-optimization specialist

Each agent has a focused tool set and system prompt for its specialty.
"""

import os
import sys
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional, Literal, Annotated, TypedDict, Sequence
from operator import add

from dotenv import load_dotenv
from langchain_anthropic import ChatAnthropic
from langchain_core.messages import HumanMessage, AIMessage, SystemMessage, BaseMessage, ToolMessage
from langchain_core.tools import tool
from langgraph.graph import StateGraph, END, START
from langgraph.graph.message import add_messages
from langgraph.prebuilt import ToolNode

# Import optimization functions from post_optimizer
from ..solver.post_optimizer import (
    post_optimize_solution,
    trajectory_swap_optimize,
    crossing_removal_optimize,
    get_coverage_stats,
)

# Import SAM-aware distance matrix calculator
from ..solver.sam_distance_matrix import (
    calculate_sam_aware_matrix,
    get_path_between,
)

# Import the REAL orienteering solver (Held-Karp algorithm)
from ..solver.solver_bridge import solve_mission

# Import target allocator for multi-drone missions
from ..solver.target_allocator import (
    allocate_targets as _allocate_targets_impl,
    parse_priority_constraints,
    allocate_with_priority_filters,
)

# Load environment variables
load_dotenv()

# Try to load from benchmark .env if not set
if not os.getenv("ANTHROPIC_API_KEY"):
    benchmark_env = os.path.join(os.path.dirname(__file__), "..", "..", "..", "isr_benchmark", ".env")
    if os.path.exists(benchmark_env):
        load_dotenv(benchmark_env)


# ============================================================================
# Persistent Memory System (shared with v1)
# ============================================================================

MEMORY_FILE = Path(__file__).parent.parent.parent / "agent_memory.json"


def load_memory() -> List[Dict[str, Any]]:
    """Load persistent memory from file."""
    if MEMORY_FILE.exists():
        try:
            with open(MEMORY_FILE, "r") as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            return []
    return []


def get_active_memories() -> List[Dict[str, Any]]:
    """Get all active memories."""
    return [m for m in load_memory() if m.get("active", True)]


def format_memories_for_prompt() -> str:
    """Format active memories as text to include in system prompt."""
    memories = get_active_memories()
    if not memories:
        return ""

    lines = ["\n\n" + "=" * 60]
    lines.append("IMPORTANT MEMORIES & INSTRUCTIONS:")
    lines.append("=" * 60)

    for m in memories:
        category = m.get("category", "note").upper()
        content = m.get("content", "")
        lines.append(f"\n[{category}] {content}")

    lines.append("\n" + "=" * 60 + "\n")

    return "\n".join(lines)


# ============================================================================
# Shared State & Context
# ============================================================================

# Global context (set per request)
_current_env: Optional[Dict[str, Any]] = None
_drone_configs: Optional[Dict[str, Any]] = None
_current_sequences: Optional[Dict[str, str]] = None
_distance_matrix: Optional[Dict[str, Dict[str, float]]] = None
_sam_paths: Optional[Dict[str, List[List[float]]]] = None
_target_allocation: Optional[Dict[str, List[str]]] = None


def set_context(env: Dict[str, Any], drone_configs: Optional[Dict[str, Any]] = None,
               sequences: Optional[Dict[str, str]] = None):
    """Set the current context for tools to use."""
    global _current_env, _drone_configs, _current_sequences, _distance_matrix, _sam_paths
    _current_env = env
    _drone_configs = drone_configs or {}
    _current_sequences = sequences or {}
    _distance_matrix = env.get("distance_matrix")
    _sam_paths = None

    if not _distance_matrix:
        _distance_matrix, _sam_paths = _compute_distance_matrix(env)


def _compute_distance_matrix(env: Dict[str, Any]) -> tuple:
    """Compute distance matrix from environment."""
    import math
    sams = env.get("sams", [])

    if sams:
        try:
            result = calculate_sam_aware_matrix(env)
            labels = result.get("labels", [])
            matrix_data = result.get("matrix", [])
            paths = result.get("paths", {})

            matrix = {}
            for i, from_id in enumerate(labels):
                matrix[from_id] = {}
                for j, to_id in enumerate(labels):
                    matrix[from_id][to_id] = round(matrix_data[i][j], 2)

            print(f"✅ Using SAM-aware distance matrix ({len(labels)} waypoints)")
            return matrix, paths

        except Exception as e:
            print(f"⚠️ SAM-aware matrix failed: {e}, using Euclidean")

    # Fallback: Euclidean
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

    return matrix, None


def get_environment() -> Dict[str, Any]:
    if _current_env is None:
        raise ValueError("No environment loaded")
    return _current_env


def get_drone_configs() -> Dict[str, Any]:
    return _drone_configs or {}


def get_current_sequences() -> Dict[str, str]:
    return _current_sequences or {}


def get_sam_path(from_id: str, to_id: str) -> Optional[List[List[float]]]:
    if _sam_paths is None:
        return None
    return _sam_paths.get(f"{from_id}->{to_id}")


# ============================================================================
# Multi-Agent State Definition
# ============================================================================

class MultiAgentState(TypedDict):
    """State shared across all agents in the multi-agent system."""
    messages: Annotated[list, add_messages]  # Conversation history
    environment: Optional[Dict[str, Any]]
    drone_configs: Optional[Dict[str, Any]]
    current_sequences: Optional[Dict[str, str]]

    # Mission planning state
    mission_overview: Optional[str]  # Cached overview
    target_allocation: Optional[Dict[str, List[str]]]  # Drone -> targets
    computed_routes: Optional[Dict[str, Dict[str, Any]]]  # Drone -> route data
    validation_results: Optional[Dict[str, str]]  # Drone -> validation result
    priority_constraints: Optional[str]  # Parsed constraints

    # Agent orchestration
    current_agent: str  # Which agent is currently active
    next_agent: Optional[str]  # Which agent to route to next
    task_complete: bool  # Whether the mission is complete


# ============================================================================
# COORDINATOR AGENT TOOLS
# ============================================================================

@tool
def get_mission_overview() -> str:
    """
    Get complete mission overview including airports, targets, SAMs, and drone configs.
    Call this FIRST to understand the mission parameters.
    """
    env = get_environment()
    configs = get_drone_configs()
    sequences = get_current_sequences()

    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    lines = [
        "=" * 60,
        "MISSION OVERVIEW",
        "=" * 60,
        "",
        f"Grid Size: {env.get('grid_size', 100)}x{env.get('grid_size', 100)}",
        "",
        f"AIRPORTS ({len(airports)}):",
    ]

    for a in airports:
        aid = a.get("id", a.get("label", "A?"))
        lines.append(f"  {aid}: position ({a['x']}, {a['y']})")

    lines.append("")
    lines.append(f"TARGETS ({len(targets)}):")

    by_type = {}
    for t in targets:
        ttype = t.get("type", "a").upper()
        if ttype not in by_type:
            by_type[ttype] = []
        by_type[ttype].append(t)

    for ttype in sorted(by_type.keys()):
        type_targets = sorted(by_type[ttype], key=lambda x: x.get("priority", 5), reverse=True)
        lines.append(f"  Type {ttype}:")
        for t in type_targets:
            tid = t.get("id", t.get("label", "T?"))
            priority = t.get("priority", t.get("value", 5))
            lines.append(f"    {tid}: priority={priority}, pos=({t['x']}, {t['y']})")

    total_points = sum(t.get("priority", t.get("value", 5)) for t in targets)
    lines.append(f"\n  Total possible points: {total_points}")

    if sams:
        lines.append(f"\nSAMs/NFZs: {len(sams)} (handled by path planner)")

    lines.append("")
    lines.append("=" * 60)
    lines.append("DRONE CONFIGURATIONS")
    lines.append("=" * 60)

    enabled_drones = []
    for drone_id in ["1", "2", "3", "4", "5"]:
        cfg = configs.get(drone_id, {})
        enabled = cfg.get("enabled", drone_id == "1")

        if enabled:
            enabled_drones.append(drone_id)
            fuel = cfg.get("fuel_budget", 200)
            start = cfg.get("start_airport", "A1")
            end = cfg.get("end_airport", "A1")
            end_display = "ANY (optimal)" if end == "-" else end

            target_access = cfg.get("target_access", {})
            if target_access:
                accessible = [k.upper() for k, v in target_access.items() if v]
                types_str = ", ".join(accessible) if accessible else "NONE"
            else:
                types_str = "ALL"

            lines.append(f"\nDrone D{drone_id}:")
            lines.append(f"  Fuel budget: {fuel}")
            lines.append(f"  Start airport: {start}")
            lines.append(f"  End airport: {end_display}")
            lines.append(f"  Accessible target types: {types_str}")

            seq = sequences.get(drone_id, "")
            if seq:
                lines.append(f"  Current sequence: {seq}")

    lines.append("")
    lines.append(f"Enabled drones: {len(enabled_drones)} ({', '.join(['D' + d for d in enabled_drones])})")

    return "\n".join(lines)


@tool
def route_to_allocator(priority_constraints: str = "") -> str:
    """
    Route task to the Allocator Agent for target allocation.

    Args:
        priority_constraints: Priority constraints string (e.g., "D1,D2: priority>=6; D3,D4: priority<=6")

    Returns:
        Handoff message to allocator agent.
    """
    return f"HANDOFF:ALLOCATOR:{priority_constraints}"


@tool
def route_to_router(drone_ids: str) -> str:
    """
    Route task to the Router Agent to compute optimal routes.

    Args:
        drone_ids: Comma-separated drone IDs to route (e.g., "1,2,3")

    Returns:
        Handoff message to router agent.
    """
    return f"HANDOFF:ROUTER:{drone_ids}"


@tool
def route_to_validator(routes: Dict[str, str]) -> str:
    """
    Route task to the Validator Agent to validate routes.

    Args:
        routes: Dictionary of drone_id -> route string (e.g., {"1": "A1,T3,T7,A1"})

    Returns:
        Handoff message to validator agent.
    """
    routes_json = json.dumps(routes)
    return f"HANDOFF:VALIDATOR:{routes_json}"


@tool
def route_to_optimizer(routes: Dict[str, str], optimization_type: str = "all") -> str:
    """
    Route task to the Optimizer Agent for post-optimization.

    Args:
        routes: Dictionary of drone_id -> route string
        optimization_type: Type of optimization ("unvisited", "swap", "crossings", "all")

    Returns:
        Handoff message to optimizer agent.
    """
    routes_json = json.dumps(routes)
    return f"HANDOFF:OPTIMIZER:{optimization_type}:{routes_json}"


# ============================================================================
# ALLOCATOR AGENT TOOLS
# ============================================================================

@tool
def allocate_targets_to_drones(strategy: str = "efficient") -> str:
    """
    Allocate targets to drones using specified strategy.

    Args:
        strategy: Allocation strategy:
            - "efficient": Maximize priority/distance ratio (default)
            - "greedy": Assign highest priority targets to nearest drone
            - "balanced": Distribute targets evenly by count
            - "geographic": Divide by angular sectors
            - "exclusive": Prioritize targets only one drone can reach

    Returns:
        Allocation results showing which targets assigned to each drone.
    """
    global _target_allocation

    print(f"\n🎯 ALLOCATE_TARGETS_TO_DRONES CALLED with strategy={strategy}", file=sys.stderr)
    sys.stderr.flush()

    env = get_environment()
    configs = get_drone_configs()
    targets = env.get("targets", [])
    airports = env.get("airports", [])

    # Get enabled drones
    enabled = [did for did, cfg in configs.items() if cfg.get("enabled", did == "1")]

    if not enabled:
        return "ERROR: No enabled drones found"

    # Build matrix for allocator
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Do allocation
    from ..solver.target_allocator import allocate_targets as alloc_fn, set_allocator_matrix

    if matrix_data:
        set_allocator_matrix(matrix_data)

    allocation = alloc_fn(env, configs, strategy, matrix_data)
    _target_allocation = allocation

    # Debug: print actual allocation result
    total_targets = sum(len(v) for v in allocation.values())
    print(f"📊 ALLOCATION RESULT: {total_targets} targets across {len(allocation)} drones", file=sys.stderr)
    for did in sorted(allocation.keys()):
        print(f"   D{did}: {allocation[did]}", file=sys.stderr)
    sys.stderr.flush()

    # Format result
    lines = ["=== TARGET ALLOCATION ===", f"Strategy: {strategy}", ""]

    total_priority = 0
    for did in sorted(allocation.keys()):
        target_ids = allocation[did]
        drone_priority = sum(
            t.get("priority", 5) for t in targets if str(t.get("id", t.get("label"))) in target_ids
        )
        total_priority += drone_priority
        lines.append(f"D{did}: {', '.join(target_ids) if target_ids else '(none)'}")
        lines.append(f"  Count: {len(target_ids)}, Priority: {drone_priority}")

    lines.append("")
    lines.append(f"Total allocated priority: {total_priority}")

    return "\n".join(lines)


@tool
def allocate_with_priority_constraints(constraints: str, strategy: str = "efficient") -> str:
    """
    Allocate targets with priority constraints applied.

    Args:
        constraints: Priority constraints string (e.g., "D1,D2: priority>=6; D3,D4: priority<=6")
        strategy: Allocation strategy

    Returns:
        Allocation results respecting priority constraints.
    """
    global _target_allocation

    env = get_environment()
    configs = get_drone_configs()

    # Parse priority constraints
    priority_filters = parse_priority_constraints(constraints)

    # Build matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Allocate with filters
    allocation = allocate_with_priority_filters(env, configs, priority_filters, strategy, matrix_data)
    _target_allocation = allocation

    # Format result
    targets = env.get("targets", [])
    lines = ["=== PRIORITY-CONSTRAINED ALLOCATION ===", f"Constraints: {constraints}", ""]

    total_priority = 0
    for did in sorted(allocation.keys()):
        target_ids = allocation[did]
        drone_priority = sum(
            t.get("priority", 5) for t in targets if str(t.get("id", t.get("label"))) in target_ids
        )
        total_priority += drone_priority
        lines.append(f"D{did}: {', '.join(target_ids) if target_ids else '(none)'}")
        lines.append(f"  Count: {len(target_ids)}, Priority: {drone_priority}")

    lines.append("")
    lines.append(f"Total allocated priority: {total_priority}")

    return "\n".join(lines)


# ============================================================================
# ROUTER AGENT TOOLS
# ============================================================================

@tool
def solve_optimal_route(drone_id: str) -> str:
    """
    Compute optimal route for a single drone using Held-Karp algorithm.

    Args:
        drone_id: Drone ID ("1", "2", etc.)

    Returns:
        Optimal route with points and fuel consumption.
    """
    drone_id = drone_id.replace("D", "").replace("d", "")

    env = get_environment()
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})

    if not cfg.get("enabled", drone_id == "1"):
        return f"ERROR: Drone D{drone_id} is not enabled"

    fuel_budget = cfg.get("fuel_budget", 200)
    start_airport = cfg.get("start_airport", "A1")
    end_airport = cfg.get("end_airport", "A1")

    # Get target access
    target_access = cfg.get("target_access", {})
    if target_access:
        accessible_types = {t.lower() for t, enabled in target_access.items() if enabled}
    else:
        accessible_types = {"a", "b", "c", "d", "e"}

    # Filter targets
    targets = env.get("targets", [])
    accessible_targets = [
        t for t in targets
        if str(t.get("type", "a")).lower() in accessible_types
    ]

    if not accessible_targets:
        return f"D{drone_id}: No accessible targets"

    # Build matrix for solver
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Call solver
    result = solve_mission(
        targets=accessible_targets,
        airports=env.get("airports", []),
        start_airport=start_airport,
        end_airport=end_airport if end_airport != "-" else None,
        fuel_budget=fuel_budget,
        distance_matrix=matrix_data
    )

    if not result or result.get("status") != "optimal":
        return f"D{drone_id}: No feasible route found"

    route = result.get("route", [])
    points = result.get("total_priority", 0)
    distance = result.get("total_distance", 0)

    lines = [
        f"=== D{drone_id} OPTIMAL ROUTE ===",
        f"Route: {' -> '.join(route)}",
        f"Points: {points}",
        f"Fuel: {distance:.1f} / {fuel_budget}",
        "",
        f"ROUTE_D{drone_id}: {','.join(route)}"
    ]

    return "\n".join(lines)


@tool
def solve_allocated_route(drone_id: str) -> str:
    """
    Compute optimal route for a drone using ONLY its allocated targets.
    Must call allocate_targets_to_drones first.

    Args:
        drone_id: Drone ID

    Returns:
        Optimal route using allocated targets.
    """
    global _target_allocation

    drone_id = drone_id.replace("D", "").replace("d", "")

    if not _target_allocation:
        return "ERROR: No allocation found. Call allocate_targets_to_drones first."

    allocated_targets = _target_allocation.get(drone_id, [])
    if not allocated_targets:
        return f"D{drone_id}: No targets allocated"

    env = get_environment()
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})

    fuel_budget = cfg.get("fuel_budget", 200)
    start_airport = cfg.get("start_airport", "A1")
    end_airport = cfg.get("end_airport", "A1")

    # Filter to only allocated targets
    all_targets = env.get("targets", [])
    targets = [t for t in all_targets if str(t.get("id", t.get("label"))) in allocated_targets]

    # Build matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Solve
    result = solve_mission(
        targets=targets,
        airports=env.get("airports", []),
        start_airport=start_airport,
        end_airport=end_airport if end_airport != "-" else None,
        fuel_budget=fuel_budget,
        distance_matrix=matrix_data
    )

    if not result or result.get("status") != "optimal":
        return f"D{drone_id}: No feasible route for allocated targets"

    route = result.get("route", [])
    points = result.get("total_priority", 0)
    distance = result.get("total_distance", 0)

    lines = [
        f"=== D{drone_id} ALLOCATED ROUTE ===",
        f"Allocated: {', '.join(allocated_targets)}",
        f"Route: {' -> '.join(route)}",
        f"Points: {points}",
        f"Fuel: {distance:.1f} / {fuel_budget}",
        "",
        f"ROUTE_D{drone_id}: {','.join(route)}"
    ]

    return "\n".join(lines)


@tool
def solve_all_drones_parallel() -> str:
    """
    Solve optimal routes for ALL allocated drones in parallel.
    Must call allocate_targets_to_drones first.

    Returns:
        All routes with combined summary.
    """
    global _target_allocation

    if not _target_allocation:
        return "ERROR: No allocation found. Call allocate_targets_to_drones first."

    from concurrent.futures import ThreadPoolExecutor, as_completed

    env = get_environment()
    configs = get_drone_configs()

    results = {}

    def solve_drone(drone_id: str) -> Dict[str, Any]:
        allocated = _target_allocation.get(drone_id, [])
        if not allocated:
            return {"drone_id": drone_id, "status": "no_targets", "route": []}

        cfg = configs.get(drone_id, {})
        fuel_budget = cfg.get("fuel_budget", 200)
        start_airport = cfg.get("start_airport", "A1")
        end_airport = cfg.get("end_airport", "A1")

        all_targets = env.get("targets", [])
        targets = [t for t in all_targets if str(t.get("id", t.get("label"))) in allocated]

        matrix_data = None
        if _distance_matrix:
            labels = list(_distance_matrix.keys())
            matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
            matrix_data = {"labels": labels, "matrix": matrix}

        result = solve_mission(
            targets=targets,
            airports=env.get("airports", []),
            start_airport=start_airport,
            end_airport=end_airport if end_airport != "-" else None,
            fuel_budget=fuel_budget,
            distance_matrix=matrix_data
        )

        if result and result.get("status") == "optimal":
            return {
                "drone_id": drone_id,
                "status": "optimal",
                "route": result.get("route", []),
                "points": result.get("total_priority", 0),
                "distance": result.get("total_distance", 0),
                "fuel_budget": fuel_budget
            }
        else:
            return {"drone_id": drone_id, "status": "failed", "route": []}

    # Get enabled drones with allocations
    drones_to_solve = [did for did in _target_allocation.keys() if _target_allocation[did]]

    # Solve in parallel
    with ThreadPoolExecutor(max_workers=5) as executor:
        futures = {executor.submit(solve_drone, did): did for did in drones_to_solve}
        for future in as_completed(futures):
            result = future.result()
            results[result["drone_id"]] = result

    # Format output
    lines = ["=== PARALLEL ROUTE SOLUTION ===", ""]

    total_points = 0
    total_fuel = 0.0
    all_visited = set()
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}

    for did in sorted(results.keys()):
        r = results[did]
        route = r.get("route", [])
        points = r.get("points", 0)
        distance = r.get("distance", 0)
        fuel_budget = r.get("fuel_budget", 200)

        for wp in route:
            if wp in targets:
                all_visited.add(wp)

        total_points += points
        total_fuel += distance

        lines.append(f"D{did}:")
        lines.append(f"  Route: {' -> '.join(route)}")
        lines.append(f"  Points: {points}, Fuel: {distance:.1f} / {fuel_budget}")
        lines.append("")

    # Summary
    total_possible = sum(t.get("priority", 5) for t in targets.values())
    unvisited = [tid for tid in targets.keys() if tid not in all_visited]

    lines.append("-" * 40)
    lines.append(f"TOTAL: {total_points} / {total_possible} points ({100*total_points/total_possible:.1f}%)")
    lines.append(f"FUEL: {total_fuel:.1f}")

    if unvisited:
        lines.append(f"UNVISITED: {', '.join(sorted(unvisited))}")

    lines.append("")
    lines.append("=" * 40)
    lines.append("ROUTES (for UI):")
    for did in sorted(results.keys()):
        route = results[did].get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    return "\n".join(lines)


# ============================================================================
# VALIDATOR AGENT TOOLS
# ============================================================================

@tool
def validate_drone_route(drone_id: str, waypoints: List[str]) -> str:
    """
    Validate a drone's route against all constraints.

    Args:
        drone_id: Drone ID
        waypoints: Route as list of waypoint IDs

    Returns:
        Validation result (VALID or errors).
    """
    drone_id = drone_id.replace("D", "").replace("d", "")

    env = get_environment()
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})

    if not cfg.get("enabled", drone_id == "1"):
        return f"INVALID: Drone D{drone_id} is not enabled"

    fuel_budget = cfg.get("fuel_budget", 200)
    start_airport = cfg.get("start_airport", "A1")
    end_airport = cfg.get("end_airport", "A1")

    errors = []

    # Check start/end
    if waypoints and waypoints[0] != start_airport:
        errors.append(f"Route must start at {start_airport}, not {waypoints[0]}")

    if end_airport != "-" and waypoints and waypoints[-1] != end_airport:
        errors.append(f"Route must end at {end_airport}, not {waypoints[-1]}")

    # Check fuel
    total_fuel = 0.0
    for i in range(len(waypoints) - 1):
        if _distance_matrix and waypoints[i] in _distance_matrix:
            total_fuel += _distance_matrix[waypoints[i]].get(waypoints[i+1], 0)

    if total_fuel > fuel_budget:
        errors.append(f"Fuel budget exceeded: {total_fuel:.1f} > {fuel_budget}")

    # Check target types
    target_access = cfg.get("target_access", {})
    if target_access:
        accessible_types = {t.lower() for t, enabled in target_access.items() if enabled}
    else:
        accessible_types = {"a", "b", "c", "d", "e"}

    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}

    for wp in waypoints:
        if wp in targets:
            target_type = str(targets[wp].get("type", "a")).lower()
            if target_type not in accessible_types:
                errors.append(f"Target {wp} type '{target_type.upper()}' not accessible to D{drone_id}")

    if errors:
        return f"INVALID D{drone_id}:\n" + "\n".join(f"  - {e}" for e in errors)
    else:
        return f"VALID D{drone_id}: Route OK (fuel: {total_fuel:.1f}/{fuel_budget})"


@tool
def check_target_conflicts(routes: Dict[str, str]) -> str:
    """
    Check if any targets are assigned to multiple drones.

    Args:
        routes: Dictionary of drone_id -> route string

    Returns:
        Conflict report or "No conflicts".
    """
    target_assignments = {}

    for did, route_str in routes.items():
        waypoints = route_str.split(",")
        for wp in waypoints:
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


@tool
def get_mission_summary(routes: Dict[str, str]) -> str:
    """
    Get summary statistics for a multi-drone plan.

    Args:
        routes: Dictionary of drone_id -> route string

    Returns:
        Mission summary with points, fuel, and coverage.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}

    lines = ["=== MISSION SUMMARY ===", ""]

    total_points = 0
    total_fuel = 0.0
    all_visited = set()

    for did in sorted(routes.keys()):
        route_str = routes[did]
        waypoints = route_str.split(",")

        cfg = configs.get(did, {})
        fuel_budget = cfg.get("fuel_budget", 200)

        # Calculate points and fuel
        points = 0
        fuel = 0.0

        for i, wp in enumerate(waypoints):
            if wp in targets and wp not in all_visited:
                points += targets[wp].get("priority", 5)
                all_visited.add(wp)

            if i < len(waypoints) - 1 and _distance_matrix:
                if wp in _distance_matrix:
                    fuel += _distance_matrix[wp].get(waypoints[i+1], 0)

        total_points += points
        total_fuel += fuel

        lines.append(f"D{did}: {len([w for w in waypoints if w.startswith('T')])} targets, {points} pts, {fuel:.1f}/{fuel_budget} fuel")

    total_possible = sum(t.get("priority", 5) for t in targets.values())
    unvisited = [tid for tid in targets.keys() if tid not in all_visited]

    lines.append("")
    lines.append("-" * 40)
    lines.append(f"TOTAL POINTS: {total_points} / {total_possible} ({100*total_points/total_possible:.1f}%)")
    lines.append(f"TOTAL FUEL: {total_fuel:.1f}")
    lines.append(f"COVERAGE: {len(all_visited)} / {len(targets)} targets")

    if unvisited:
        lines.append(f"UNVISITED: {', '.join(sorted(unvisited))}")

    return "\n".join(lines)


# ============================================================================
# OPTIMIZER AGENT TOOLS
# ============================================================================

@tool
def optimize_assign_unvisited(routes: Dict[str, str], priority_constraints: str = "") -> str:
    """
    Assign unvisited targets to drones with spare fuel capacity.

    Args:
        routes: Dictionary of drone_id -> route string
        priority_constraints: Priority constraints to respect

    Returns:
        Optimized routes with unvisited targets inserted.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}

    # Build solution dict
    solution = {"routes": {}}
    for did, route_str in routes.items():
        waypoints = route_str.split(",")

        cfg = configs.get(did, {})
        fuel_budget = cfg.get("fuel_budget", 200)

        # Calculate current stats
        points = 0
        distance = 0.0
        trajectory = []

        for i, wp in enumerate(waypoints):
            if wp in targets:
                points += targets[wp].get("priority", 5)

            if i < len(waypoints) - 1 and _distance_matrix:
                if wp in _distance_matrix:
                    distance += _distance_matrix[wp].get(waypoints[i+1], 0)

        solution["routes"][did] = {
            "route": waypoints,
            "points": points,
            "distance": distance,
            "fuel_budget": fuel_budget,
            "trajectory": trajectory
        }

    # Build matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Run optimizer
    result = post_optimize_solution(solution, env, configs, matrix_data, priority_constraints)

    # Format result
    lines = ["=== UNVISITED TARGET ASSIGNMENT ===", ""]

    for did, route_data in sorted(result.get("routes", {}).items()):
        route = route_data.get("route", [])
        lines.append(f"D{did}: {','.join(route)}")
        lines.append(f"  Points: {route_data.get('points', 0)}, Fuel: {route_data.get('distance', 0):.1f}")

    lines.append("")
    lines.append("Updated routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        lines.append(f"ROUTE_D{did}: {','.join(route_data.get('route', []))}")

    return "\n".join(lines)


@tool
def optimize_reassign_targets(routes: Dict[str, str], priority_constraints: str = "") -> str:
    """
    Reassign targets to drones whose trajectories pass closer to them.

    Args:
        routes: Dictionary of drone_id -> route string
        priority_constraints: Priority constraints to respect

    Returns:
        Optimized routes with targets swapped to closer trajectories.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}

    # Build solution dict with trajectories
    solution = {"routes": {}}
    for did, route_str in routes.items():
        waypoints = route_str.split(",")

        cfg = configs.get(did, {})
        fuel_budget = cfg.get("fuel_budget", 200)

        points = 0
        distance = 0.0
        trajectory = []

        for i, wp in enumerate(waypoints):
            if wp in targets:
                points += targets[wp].get("priority", 5)
                t = targets[wp]
                trajectory.append([float(t["x"]), float(t["y"])])
            elif wp.startswith("A"):
                for a in env.get("airports", []):
                    if str(a.get("id", a.get("label"))) == wp:
                        trajectory.append([float(a["x"]), float(a["y"])])
                        break

            if i < len(waypoints) - 1 and _distance_matrix:
                if wp in _distance_matrix:
                    distance += _distance_matrix[wp].get(waypoints[i+1], 0)

        solution["routes"][did] = {
            "route": waypoints,
            "points": points,
            "distance": distance,
            "fuel_budget": fuel_budget,
            "trajectory": trajectory
        }

    # Build matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Run optimizer
    result = trajectory_swap_optimize(solution, env, configs, matrix_data, priority_constraints)

    # Format result
    lines = ["=== TARGET REASSIGNMENT OPTIMIZATION ===", ""]

    swaps = result.get("swaps_made", [])
    if swaps:
        lines.append(f"Made {len(swaps)} swaps:")
        for swap in swaps[:10]:
            lines.append(f"  {swap}")
    else:
        lines.append("No beneficial swaps found")

    lines.append("")
    for did, route_data in sorted(result.get("routes", {}).items()):
        route = route_data.get("route", [])
        lines.append(f"D{did}: {','.join(route)}")

    lines.append("")
    lines.append("Updated routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        lines.append(f"ROUTE_D{did}: {','.join(route_data.get('route', []))}")

    return "\n".join(lines)


@tool
def optimize_remove_crossings(routes: Dict[str, str]) -> str:
    """
    Remove self-crossing paths using 2-opt optimization.

    Args:
        routes: Dictionary of drone_id -> route string

    Returns:
        Optimized routes with crossings removed.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}

    # Build solution dict
    solution = {"routes": {}}
    for did, route_str in routes.items():
        waypoints = route_str.split(",")

        cfg = configs.get(did, {})
        fuel_budget = cfg.get("fuel_budget", 200)

        points = 0
        distance = 0.0

        for i, wp in enumerate(waypoints):
            if wp in targets:
                points += targets[wp].get("priority", 5)

            if i < len(waypoints) - 1 and _distance_matrix:
                if wp in _distance_matrix:
                    distance += _distance_matrix[wp].get(waypoints[i+1], 0)

        solution["routes"][did] = {
            "route": waypoints,
            "points": points,
            "distance": distance,
            "fuel_budget": fuel_budget
        }

    # Build matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Run optimizer
    result = crossing_removal_optimize(solution, env, configs, matrix_data)

    # Format result
    lines = ["=== CROSSING REMOVAL (2-opt) ===", ""]

    for did, route_data in sorted(result.get("routes", {}).items()):
        route = route_data.get("route", [])
        lines.append(f"D{did}: {','.join(route)}")
        lines.append(f"  Points: {route_data.get('points', 0)}, Fuel: {route_data.get('distance', 0):.1f}")

    lines.append("")
    lines.append("Updated routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        lines.append(f"ROUTE_D{did}: {','.join(route_data.get('route', []))}")

    return "\n".join(lines)


# ============================================================================
# Tool Sets for Each Agent
# ============================================================================

COORDINATOR_TOOLS = [
    get_mission_overview,
    route_to_allocator,
    route_to_router,
    route_to_validator,
    route_to_optimizer,
]

ALLOCATOR_TOOLS = [
    get_mission_overview,
    allocate_targets_to_drones,
    allocate_with_priority_constraints,
]

ROUTER_TOOLS = [
    solve_optimal_route,
    solve_allocated_route,
    solve_all_drones_parallel,
]

VALIDATOR_TOOLS = [
    validate_drone_route,
    check_target_conflicts,
    get_mission_summary,
]

OPTIMIZER_TOOLS = [
    optimize_assign_unvisited,
    optimize_reassign_targets,
    optimize_remove_crossings,
    get_mission_summary,
]


# ============================================================================
# System Prompts for Each Agent
# ============================================================================

HARD_CONSTRAINTS_SECTION = """
================================================================================
HARD CONSTRAINTS - NEVER VIOLATE
================================================================================
1. FUEL BUDGET: Route MUST NOT exceed drone's fuel capacity.
2. SAM/NFZ BOUNDARIES: Routes MUST avoid all SAM zones.
3. MISSION CONSTRAINTS:
   a) Target Type Accessibility: Respect target_access restrictions
   b) Priority Constraints: Respect priority-based filters
   c) Route Order Constraints: Respect start/end requirements
================================================================================
"""

COORDINATOR_PROMPT = f"""You are the COORDINATOR agent in a multi-agent ISR mission planning system.

{HARD_CONSTRAINTS_SECTION}

Your role is to:
1. Understand the user's mission request
2. Get a mission overview first
3. Decompose the task and route to specialist agents:
   - ALLOCATOR: For target allocation (call route_to_allocator with priority_constraints)
   - ROUTER: For computing optimal routes (call route_to_router with drone_ids)
   - VALIDATOR: For validating routes (call route_to_validator with routes)
   - OPTIMIZER: For post-optimization (call route_to_optimizer with routes and type)

IMPORTANT: After each specialist completes, you must continue the workflow by calling the next routing tool.

WORKFLOW - Follow these steps IN ORDER:
1. Call get_mission_overview to understand the mission
2. Call route_to_allocator (system will pass control to ALLOCATOR)
3. When you see allocation results, call route_to_router (system will pass control to ROUTER)
4. When you see ROUTE_D results, you are DONE - just repeat the routes in your response
5. (Optional) If validation or optimization is requested, route to those agents

CRITICAL: You MUST call route_to_router after seeing allocation results!
After ROUTER returns routes, just output those routes - no further routing needed.

OUTPUT FORMAT (copy from ROUTER results):
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
"""

ALLOCATOR_PROMPT = f"""You are the ALLOCATOR agent specializing in target allocation.

{HARD_CONSTRAINTS_SECTION}

Your role is to:
1. Allocate targets to drones optimally
2. Respect type restrictions and priority constraints
3. Use the appropriate allocation strategy

TOOLS:
- allocate_targets_to_drones: For standard allocation
- allocate_with_priority_constraints: When priority constraints are specified

After allocation, output the allocation results and return control.
"""

ROUTER_PROMPT = f"""You are the ROUTER agent specializing in optimal route computation.

{HARD_CONSTRAINTS_SECTION}

Your role is to:
1. Compute optimal routes for drones
2. Use Held-Karp algorithm for globally optimal solutions
3. Respect fuel budgets and airport constraints

TOOLS:
- solve_optimal_route: For single drone (all targets)
- solve_allocated_route: For single drone (allocated targets only)
- solve_all_drones_parallel: For all drones at once (FASTEST)

After routing, output ROUTE_Dx format and return control.

OUTPUT FORMAT:
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
"""

VALIDATOR_PROMPT = f"""You are the VALIDATOR agent specializing in constraint validation.

{HARD_CONSTRAINTS_SECTION}

Your role is to:
1. Validate each drone's route against its constraints
2. Check for target conflicts (same target assigned to multiple drones)
3. Provide mission summary statistics

TOOLS:
- validate_drone_route: Validate one drone's route
- check_target_conflicts: Check for duplicate assignments
- get_mission_summary: Get overall mission statistics

Report any violations found. If routes are valid, confirm validation passed.
"""

OPTIMIZER_PROMPT = f"""You are the OPTIMIZER agent specializing in post-optimization.

{HARD_CONSTRAINTS_SECTION}

Your role is to:
1. Optimize existing routes without violating constraints
2. Insert unvisited targets where possible
3. Swap targets to closer trajectories
4. Remove route crossings

TOOLS:
- optimize_assign_unvisited: Insert missed targets (respects constraints)
- optimize_reassign_targets: Swap to closer trajectories
- optimize_remove_crossings: 2-opt crossing removal

CRITICAL: Always pass priority_constraints to optimization tools if the mission
has priority restrictions. This ensures optimization respects constraints.

After optimization, output updated ROUTE_Dx format.

OUTPUT FORMAT:
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
"""


# ============================================================================
# Multi-Agent Graph Construction
# ============================================================================

def create_multi_agent_workflow():
    """Create the multi-agent LangGraph workflow."""

    # Create LLMs for each agent (can use different models/temps)
    llm = ChatAnthropic(
        model="claude-sonnet-4-20250514",
        temperature=0,
        max_tokens=4096,
    )

    # Bind tools to LLMs
    coordinator_llm = llm.bind_tools(COORDINATOR_TOOLS)
    allocator_llm = llm.bind_tools(ALLOCATOR_TOOLS)
    router_llm = llm.bind_tools(ROUTER_TOOLS)
    validator_llm = llm.bind_tools(VALIDATOR_TOOLS)
    optimizer_llm = llm.bind_tools(OPTIMIZER_TOOLS)

    def coordinator_node(state: MultiAgentState) -> Dict[str, Any]:
        """Coordinator agent node."""
        messages = state.get("messages", [])
        memories = format_memories_for_prompt()
        system = SystemMessage(content=COORDINATOR_PROMPT + memories)

        if not messages or not isinstance(messages[0], SystemMessage):
            messages = [system] + list(messages)

        response = coordinator_llm.invoke(messages)
        return {"messages": [response], "current_agent": "coordinator"}

    def allocator_node(state: MultiAgentState) -> Dict[str, Any]:
        """Allocator agent node."""
        messages = state.get("messages", [])
        memories = format_memories_for_prompt()
        system = SystemMessage(content=ALLOCATOR_PROMPT + memories)

        # Replace system message
        if messages and isinstance(messages[0], SystemMessage):
            messages = [system] + messages[1:]
        else:
            messages = [system] + list(messages)

        response = allocator_llm.invoke(messages)
        return {"messages": [response], "current_agent": "allocator"}

    def router_node(state: MultiAgentState) -> Dict[str, Any]:
        """Router agent node."""
        messages = state.get("messages", [])
        memories = format_memories_for_prompt()
        system = SystemMessage(content=ROUTER_PROMPT + memories)

        if messages and isinstance(messages[0], SystemMessage):
            messages = [system] + messages[1:]
        else:
            messages = [system] + list(messages)

        response = router_llm.invoke(messages)
        return {"messages": [response], "current_agent": "router"}

    def validator_node(state: MultiAgentState) -> Dict[str, Any]:
        """Validator agent node."""
        messages = state.get("messages", [])
        memories = format_memories_for_prompt()
        system = SystemMessage(content=VALIDATOR_PROMPT + memories)

        if messages and isinstance(messages[0], SystemMessage):
            messages = [system] + messages[1:]
        else:
            messages = [system] + list(messages)

        response = validator_llm.invoke(messages)
        return {"messages": [response], "current_agent": "validator"}

    def optimizer_node(state: MultiAgentState) -> Dict[str, Any]:
        """Optimizer agent node."""
        messages = state.get("messages", [])
        memories = format_memories_for_prompt()
        system = SystemMessage(content=OPTIMIZER_PROMPT + memories)

        if messages and isinstance(messages[0], SystemMessage):
            messages = [system] + messages[1:]
        else:
            messages = [system] + list(messages)

        response = optimizer_llm.invoke(messages)
        return {"messages": [response], "current_agent": "optimizer"}

    def route_decision(state: MultiAgentState) -> str:
        """Decide where to route next based on last message."""
        messages = state.get("messages", [])
        if not messages:
            return "end"

        last_message = messages[-1]

        # Check for tool calls
        if hasattr(last_message, "tool_calls") and last_message.tool_calls:
            return "tools"

        # Check for HANDOFF in tool results
        if isinstance(last_message, ToolMessage):
            content = last_message.content if hasattr(last_message, 'content') else ""
            if isinstance(content, str) and content.startswith("HANDOFF:"):
                parts = content.split(":", 2)
                if len(parts) >= 2:
                    target = parts[1].upper()
                    if target == "ALLOCATOR":
                        return "allocator"
                    elif target == "ROUTER":
                        return "router"
                    elif target == "VALIDATOR":
                        return "validator"
                    elif target == "OPTIMIZER":
                        return "optimizer"

        # Check current agent - if they finished, go back to coordinator or end
        current = state.get("current_agent", "coordinator")

        # If we have ROUTE_D in the response, we might be done
        if hasattr(last_message, "content"):
            content = last_message.content
            if isinstance(content, str) and "ROUTE_D" in content:
                # Check if this is a final response (no more tool calls pending)
                return "end"

        return "end"

    # Build the graph
    workflow = StateGraph(MultiAgentState)

    # Add nodes
    workflow.add_node("coordinator", coordinator_node)
    workflow.add_node("allocator", allocator_node)
    workflow.add_node("router", router_node)
    workflow.add_node("validator", validator_node)
    workflow.add_node("optimizer", optimizer_node)

    # Add tool nodes for each agent
    workflow.add_node("coordinator_tools", ToolNode(COORDINATOR_TOOLS))
    workflow.add_node("allocator_tools", ToolNode(ALLOCATOR_TOOLS))
    workflow.add_node("router_tools", ToolNode(ROUTER_TOOLS))
    workflow.add_node("validator_tools", ToolNode(VALIDATOR_TOOLS))
    workflow.add_node("optimizer_tools", ToolNode(OPTIMIZER_TOOLS))

    # Set entry point
    workflow.set_entry_point("coordinator")

    # Coordinator routing
    def coordinator_route(state: MultiAgentState) -> str:
        messages = state.get("messages", [])
        if not messages:
            return "end"
        last = messages[-1]

        # If coordinator has tool calls, execute them
        if hasattr(last, "tool_calls") and last.tool_calls:
            return "coordinator_tools"

        # Check if we have final routes in the coordinator's response - if so, we're done
        if hasattr(last, "content"):
            content = str(last.content)
            if "ROUTE_D" in content:
                return "end"

        # Check if we have routes anywhere in messages - if so, we're done
        for msg in messages:
            if hasattr(msg, "content") and "ROUTE_D" in str(msg.content):
                return "end"

        # If no tool calls and no routes, just end (avoid infinite loop)
        # The coordinator should have made tool calls or produced routes
        return "end"

    workflow.add_conditional_edges("coordinator", coordinator_route, {
        "coordinator_tools": "coordinator_tools",
        "end": END
    })

    # Tool results routing (handles HANDOFF)
    def tools_route(state: MultiAgentState) -> str:
        messages = state.get("messages", [])
        for msg in reversed(messages):
            if isinstance(msg, ToolMessage):
                content = str(msg.content) if hasattr(msg, 'content') else ""
                if content.startswith("HANDOFF:"):
                    parts = content.split(":", 2)
                    if len(parts) >= 2:
                        target = parts[1].upper()
                        if target == "ALLOCATOR":
                            return "allocator"
                        elif target == "ROUTER":
                            return "router"
                        elif target == "VALIDATOR":
                            return "validator"
                        elif target == "OPTIMIZER":
                            return "optimizer"
                break
        return "coordinator"

    workflow.add_conditional_edges("coordinator_tools", tools_route, {
        "coordinator": "coordinator",
        "allocator": "allocator",
        "router": "router",
        "validator": "validator",
        "optimizer": "optimizer"
    })

    # Allocator routing
    def allocator_route(state: MultiAgentState) -> str:
        messages = state.get("messages", [])
        if not messages:
            return "coordinator"
        last = messages[-1]
        if hasattr(last, "tool_calls") and last.tool_calls:
            return "allocator_tools"
        return "coordinator"

    workflow.add_conditional_edges("allocator", allocator_route, {
        "allocator_tools": "allocator_tools",
        "coordinator": "coordinator"
    })
    workflow.add_edge("allocator_tools", "allocator")

    # Router routing
    def router_route(state: MultiAgentState) -> str:
        messages = state.get("messages", [])
        if not messages:
            return "coordinator"
        last = messages[-1]
        if hasattr(last, "tool_calls") and last.tool_calls:
            return "router_tools"
        return "coordinator"

    workflow.add_conditional_edges("router", router_route, {
        "router_tools": "router_tools",
        "coordinator": "coordinator"
    })
    workflow.add_edge("router_tools", "router")

    # Validator routing
    def validator_route(state: MultiAgentState) -> str:
        messages = state.get("messages", [])
        if not messages:
            return "coordinator"
        last = messages[-1]
        if hasattr(last, "tool_calls") and last.tool_calls:
            return "validator_tools"
        return "coordinator"

    workflow.add_conditional_edges("validator", validator_route, {
        "validator_tools": "validator_tools",
        "coordinator": "coordinator"
    })
    workflow.add_edge("validator_tools", "validator")

    # Optimizer routing
    def optimizer_route(state: MultiAgentState) -> str:
        messages = state.get("messages", [])
        if not messages:
            return "coordinator"
        last = messages[-1]
        if hasattr(last, "tool_calls") and last.tool_calls:
            return "optimizer_tools"
        return "coordinator"

    workflow.add_conditional_edges("optimizer", optimizer_route, {
        "optimizer_tools": "optimizer_tools",
        "coordinator": "coordinator"
    })
    workflow.add_edge("optimizer_tools", "optimizer")

    return workflow.compile()


# ============================================================================
# Entry Points
# ============================================================================

_multi_agent_workflow = None


def get_multi_agent_workflow():
    """Get or create the multi-agent workflow singleton."""
    global _multi_agent_workflow
    if _multi_agent_workflow is None:
        _multi_agent_workflow = create_multi_agent_workflow()
    return _multi_agent_workflow


def run_isr_agent(env: Dict[str, Any], user_query: str,
                  drone_configs: Optional[Dict[str, Any]] = None,
                  sequences: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    """
    Run the multi-agent ISR system.

    This is the main entry point - same interface as v1 for compatibility.
    """
    # Set context
    set_context(env, drone_configs, sequences)

    # Get workflow
    workflow = get_multi_agent_workflow()

    # Initial state
    initial_state = {
        "messages": [HumanMessage(content=user_query)],
        "environment": env,
        "drone_configs": drone_configs,
        "current_sequences": sequences,
        "current_agent": "coordinator",
        "task_complete": False,
    }

    # Run
    config = {"recursion_limit": 150}  # Higher for multi-agent

    final_state = None
    step_count = 0

    print(f"\n{'='*60}", file=sys.stderr)
    print(f"🤖 MULTI-AGENT WORKFLOW STARTED", file=sys.stderr)
    print(f"{'='*60}", file=sys.stderr)
    sys.stderr.flush()

    all_steps = []
    for step in workflow.stream(initial_state, config=config):
        step_count += 1
        all_steps.append(step)
        for node_name, node_output in step.items():
            agent = node_output.get("current_agent", "?") if isinstance(node_output, dict) else "?"
            print(f"[Step {step_count}] Agent: {agent} | Node: {node_name}", file=sys.stderr)
            sys.stderr.flush()
        final_state = step

    print(f"{'='*60}", file=sys.stderr)
    print(f"🏁 MULTI-AGENT WORKFLOW COMPLETED after {step_count} steps", file=sys.stderr)
    print(f"{'='*60}\n", file=sys.stderr)
    sys.stderr.flush()

    print(f"🔍 [DEBUG] Starting response extraction from {len(all_steps)} steps", flush=True)

    # Extract response - look for ROUTE_D in all messages from all steps
    response_text = ""
    all_messages = []

    # Collect messages from ALL steps, not just the last one
    for step in all_steps:
        for node_output in step.values():
            if isinstance(node_output, dict):
                messages = node_output.get("messages", [])
                all_messages.extend(messages)

    # Search through all messages for routes
    for msg in all_messages:
        if hasattr(msg, "content"):
            content = msg.content
            # Handle both string and list content
            if isinstance(content, str):
                text = content
            elif isinstance(content, list):
                text_parts = []
                for item in content:
                    if isinstance(item, dict) and item.get("type") == "text":
                        text_parts.append(item.get("text", ""))
                    elif isinstance(item, str):
                        text_parts.append(item)
                text = "\n".join(text_parts)
            else:
                text = ""

            # Collect ALL messages containing ROUTE_D (don't break on first)
            if "ROUTE_D" in text:
                print(f"🔍 [DEBUG] Found ROUTE_D in message", flush=True)
                if response_text:
                    response_text += "\n" + text
                else:
                    response_text = text

    print(f"🔍 [DEBUG] After message scan: all_messages={len(all_messages)}, response_text={len(response_text)} chars", flush=True)

    # If no ROUTE_D found, search backwards for any message with text content
    if not response_text and all_messages:
        print(f"🔍 [DEBUG] Searching backwards for message with text content...", file=sys.stderr)
        for msg in reversed(all_messages):
            if hasattr(msg, "content"):
                content = msg.content
                extracted = ""
                if isinstance(content, str) and content.strip():
                    extracted = content
                elif isinstance(content, list):
                    text_parts = []
                    for item in content:
                        if isinstance(item, dict) and item.get("type") == "text":
                            text_parts.append(item.get("text", ""))
                        elif isinstance(item, str):
                            text_parts.append(item)
                    extracted = "\n".join(text_parts)

                if extracted.strip():
                    response_text = extracted
                    print(f"🔍 [DEBUG] Found text in {type(msg).__name__}: {extracted[:100]}...", file=sys.stderr)
                    break
        sys.stderr.flush()

    # Parse routes
    routes = {}
    if response_text:
        # Debug: show actual response_text content to diagnose regex issues
        print(f"\n📋 ==================== RESPONSE TEXT DEBUG ====================", flush=True)
        print(f"📋 response_text (first 500 chars):\n{response_text[:500]}", flush=True)
        print(f"📋 ===============================================================", flush=True)

        # Try multiple regex patterns to find routes
        route_matches = re.findall(r'ROUTE_D(\d+):\s*([A-Z0-9,]+)', response_text)
        print(f"\n📋 Parsing routes from response_text ({len(response_text)} chars)", flush=True)
        print(f"📋 Pattern 1 (ROUTE_D\\d+:\\s*[A-Z0-9,]+): Found {len(route_matches)} matches: {route_matches}", flush=True)

        # If no matches, try alternative patterns
        if not route_matches:
            # Try with more flexible spacing/formatting
            route_matches = re.findall(r'ROUTE_D(\d+)\s*:\s*([A-Z0-9,\s]+?)(?:\n|$)', response_text, re.MULTILINE)
            print(f"📋 Pattern 2 (multiline): Found {len(route_matches)} matches: {route_matches}", flush=True)

        if not route_matches:
            # Try with **bold** markdown formatting
            route_matches = re.findall(r'\*\*ROUTE_D(\d+)\*\*\s*:\s*([A-Z0-9,\s]+?)(?:\n|$)', response_text, re.MULTILINE)
            print(f"📋 Pattern 3 (bold markdown): Found {len(route_matches)} matches: {route_matches}", flush=True)

        if not route_matches:
            # Try finding any line containing ROUTE_D
            route_lines = [line for line in response_text.split('\n') if 'ROUTE_D' in line]
            print(f"📋 Lines containing ROUTE_D: {route_lines}", flush=True)
    else:
        route_matches = []
        print(f"\n⚠️ No response_text found to parse routes from", flush=True)
    for drone_id, route_str in route_matches:
        routes[drone_id] = route_str.split(',')
    print(f"📋 Final parsed routes: {routes}", flush=True)

    # Calculate totals
    total_points = 0
    total_fuel = 0.0
    targets = {str(t.get("id", t.get("label"))): t for t in env.get("targets", [])}
    visited = set()

    trajectories = {}

    for drone_id, route in routes.items():
        trajectory = []
        for i, wp in enumerate(route):
            if wp in targets and wp not in visited:
                total_points += targets[wp].get("priority", targets[wp].get("value", 5))
                visited.add(wp)

            if i < len(route) - 1 and _distance_matrix and wp in _distance_matrix:
                total_fuel += _distance_matrix[wp].get(route[i + 1], 0)

            # Build trajectory
            sam_path = get_sam_path(wp, route[i + 1] if i < len(route) - 1 else wp)
            if sam_path and len(sam_path) >= 2:
                if trajectory:
                    trajectory.extend(sam_path[1:])
                else:
                    trajectory.extend(sam_path)
            else:
                pos = _get_waypoint_position(wp, env)
                if pos and (not trajectory or trajectory[-1] != pos):
                    trajectory.append(pos)

        trajectories[drone_id] = trajectory

    # Include allocation from global state (set by allocator tool)
    global _target_allocation
    allocation = _target_allocation.copy() if _target_allocation else {}

    if allocation:
        print(f"\n📊 Returning allocation: {allocation}", file=sys.stderr)

    return {
        "response": response_text,
        "routes": routes,
        "trajectories": trajectories,
        "total_points": total_points,
        "total_fuel": total_fuel,
        "route": routes.get("1"),
        "points": total_points,
        "fuel": total_fuel,
        "allocation": allocation,  # Target allocation from allocator agent
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
