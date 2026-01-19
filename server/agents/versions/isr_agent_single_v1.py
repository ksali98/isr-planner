"""
ISR Mission Planning Agent for Web Planner

LangGraph-based agent with tools for planning MULTI-DRONE missions.
Integrates with the web planner UI to visualize routes.

The agent receives:
- Environment (airports, targets, SAMs)
- Drone configurations (fuel budgets, start/end airports, target access per drone)
- Current sequences (if any)
"""

import os
import sys
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional, Literal, Annotated

from dotenv import load_dotenv
from langchain_anthropic import ChatAnthropic
from langchain_core.messages import HumanMessage, AIMessage, SystemMessage, BaseMessage
from langchain_core.tools import tool
from langgraph.graph import StateGraph, END
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
# Persistent Memory System
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


def save_memory(memories: List[Dict[str, Any]]) -> None:
    """Save memories to persistent file."""
    with open(MEMORY_FILE, "w") as f:
        json.dump(memories, f, indent=2)


def add_memory(content: str, category: str = "correction") -> Dict[str, Any]:
    """
    Add a new memory entry.

    Categories:
    - correction: User corrected the agent's behavior
    - instruction: User gave a standing instruction
    - preference: User preference for how to do things
    - fact: Important fact to remember
    """
    memories = load_memory()
    entry = {
        "id": len(memories) + 1,
        "timestamp": datetime.now().isoformat(),
        "category": category,
        "content": content,
        "active": True
    }
    memories.append(entry)
    save_memory(memories)
    return entry


def get_active_memories() -> List[Dict[str, Any]]:
    """Get all active memories."""
    return [m for m in load_memory() if m.get("active", True)]


def clear_memory() -> int:
    """Clear all memories. Returns count of cleared entries."""
    memories = load_memory()
    count = len(memories)
    save_memory([])
    return count


def delete_memory(memory_id: int) -> bool:
    """Delete a specific memory by ID."""
    memories = load_memory()
    original_len = len(memories)
    memories = [m for m in memories if m.get("id") != memory_id]
    if len(memories) < original_len:
        save_memory(memories)
        return True
    return False


def format_memories_for_prompt() -> str:
    """Format active memories as text to include in system prompt."""
    memories = get_active_memories()
    if not memories:
        return ""

    lines = ["\n\n" + "=" * 60]
    lines.append("IMPORTANT MEMORIES & INSTRUCTIONS FROM PREVIOUS SESSIONS:")
    lines.append("=" * 60)

    for m in memories:
        category = m.get("category", "note").upper()
        content = m.get("content", "")
        lines.append(f"\n[{category}] {content}")

    lines.append("\n" + "=" * 60)
    lines.append("END OF MEMORIES - Follow these instructions carefully!")
    lines.append("=" * 60 + "\n")

    return "\n".join(lines)


# ============================================================================
# State Definition
# ============================================================================

class ISRAgentState(dict):
    """State for the ISR planning agent."""
    messages: Annotated[list, add_messages]
    environment: Optional[Dict[str, Any]]
    drone_configs: Optional[Dict[str, Any]]
    current_sequences: Optional[Dict[str, str]]
    proposed_routes: Optional[Dict[str, Any]]


# ============================================================================
# Environment & Config Context (set per request)
# ============================================================================

_current_env: Optional[Dict[str, Any]] = None
_drone_configs: Optional[Dict[str, Any]] = None
_current_sequences: Optional[Dict[str, str]] = None
_distance_matrix: Optional[Dict[str, Dict[str, float]]] = None
_sam_paths: Optional[Dict[str, List[List[float]]]] = None  # Pre-computed SAM-avoiding paths


def set_context(env: Dict[str, Any], drone_configs: Optional[Dict[str, Any]] = None,
               sequences: Optional[Dict[str, str]] = None):
    """Set the current context for tools to use."""
    global _current_env, _drone_configs, _current_sequences, _distance_matrix, _sam_paths
    _current_env = env
    _drone_configs = drone_configs or {}
    _current_sequences = sequences or {}
    _distance_matrix = env.get("distance_matrix")
    _sam_paths = None

    # If no distance matrix, compute one (SAM-aware if SAMs present)
    if not _distance_matrix:
        _distance_matrix, _sam_paths = _compute_distance_matrix(env)


def _compute_distance_matrix(env: Dict[str, Any]) -> tuple:
    """
    Compute distance matrix from environment.
    Uses SAM-aware distances if SAMs are present, otherwise Euclidean.

    Returns:
        Tuple of (distance_matrix_dict, sam_paths_dict)
    """
    import math

    sams = env.get("sams", [])

    # If SAMs exist, use SAM-aware distance calculation
    if sams:
        try:
            result = calculate_sam_aware_matrix(env)

            # Convert matrix format from list-of-lists to dict-of-dicts
            labels = result.get("labels", [])
            matrix_data = result.get("matrix", [])
            paths = result.get("paths", {})

            matrix = {}
            for i, from_id in enumerate(labels):
                matrix[from_id] = {}
                for j, to_id in enumerate(labels):
                    matrix[from_id][to_id] = round(matrix_data[i][j], 2)

            return matrix, paths

        except Exception as e:

    # Fallback: Simple Euclidean distances
    waypoints = {}

    # Add airports
    for airport in env.get("airports", []):
        aid = airport.get("id", airport.get("label", "A1"))
        waypoints[aid] = (airport["x"], airport["y"])

    # Add targets
    for target in env.get("targets", []):
        tid = target.get("id", target.get("label", "T?"))
        waypoints[tid] = (target["x"], target["y"])

    # Compute distances
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


def get_sam_path(from_id: str, to_id: str) -> Optional[List[List[float]]]:
    """Get the pre-computed SAM-avoiding path between two waypoints."""
    if _sam_paths is None:
        return None
    path_key = f"{from_id}->{to_id}"
    return _sam_paths.get(path_key)


def get_environment() -> Dict[str, Any]:
    """Get the current environment."""
    if _current_env is None:
        raise ValueError("No environment loaded")
    return _current_env


def get_drone_configs() -> Dict[str, Any]:
    """Get the drone configurations."""
    return _drone_configs or {}


def get_current_sequences() -> Dict[str, str]:
    """Get the current sequences."""
    return _current_sequences or {}


# ============================================================================
# Tools
# ============================================================================

@tool
def get_mission_overview() -> str:
    """
    Get complete mission overview including:
    - All airports with positions
    - All targets with priorities and types
    - All enabled drones with their constraints (fuel budget, start/end airport, accessible target types)
    - Current sequences if any

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

    # Group by type
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
        lines.append(f"\nSAMs/NFZs: {len(sams)} (handled by low-level planner)")

    lines.append("")
    lines.append("=" * 60)
    lines.append("DRONE CONFIGURATIONS")
    lines.append("=" * 60)

    enabled_drones = []
    for drone_id in ["1", "2", "3", "4", "5"]:
        cfg = configs.get(drone_id, {})
        enabled = cfg.get("enabled", drone_id == "1")  # D1 enabled by default

        if enabled:
            enabled_drones.append(drone_id)
            fuel = cfg.get("fuel_budget", 200)
            start = cfg.get("start_airport", "A1")
            end = cfg.get("end_airport", "A1")
            end_display = "ANY (optimal)" if end == "-" else end

            # Target access
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
def get_drone_info(drone_id: str) -> str:
    """
    Get detailed info for a specific drone including its constraints.

    Args:
        drone_id: The drone ID ("1", "2", etc. or "D1", "D2", etc.)
    """
    # Normalize drone ID
    drone_id = drone_id.replace("D", "").replace("d", "")

    configs = get_drone_configs()
    sequences = get_current_sequences()
    env = get_environment()
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    cfg = configs.get(drone_id, {})

    if not cfg:
        return f"Drone D{drone_id} has no configuration (using defaults: fuel=200, start/end=A1, all types)"

    enabled = cfg.get("enabled", drone_id == "1")
    fuel = cfg.get("fuel_budget", 200)
    start = cfg.get("start_airport", "A1")
    end = cfg.get("end_airport", "A1")
    end_display = "ANY (optimal)" if end == "-" else end

    # Target access
    target_access = cfg.get("target_access", {})
    accessible_types = []
    if target_access:
        accessible_types = [k.upper() for k, v in target_access.items() if v]
    else:
        accessible_types = ["A", "B", "C", "D", "E"]

    # Find accessible targets
    accessible_targets = []
    for tid, t in targets.items():
        ttype = t.get("type", "a").upper()
        if ttype in accessible_types or not accessible_types:
            accessible_targets.append((tid, t.get("priority", 5), ttype))

    accessible_targets.sort(key=lambda x: x[1], reverse=True)

    lines = [
        f"=== Drone D{drone_id} ===",
        f"Status: {'ENABLED' if enabled else 'DISABLED'}",
        f"Fuel budget: {fuel}",
        f"Start airport: {start}",
        f"End airport: {end_display}",
        f"Accessible types: {', '.join(accessible_types) if accessible_types else 'ALL'}",
        "",
        f"Accessible targets ({len(accessible_targets)}):",
    ]

    for tid, priority, ttype in accessible_targets:
        lines.append(f"  {tid}: priority={priority}, type={ttype}")

    total_accessible_points = sum(t[1] for t in accessible_targets)
    lines.append(f"\nMax possible points for this drone: {total_accessible_points}")

    seq = sequences.get(drone_id, "")
    if seq:
        lines.append(f"\nCurrent assigned sequence: {seq}")

    return "\n".join(lines)


@tool
def get_distance(from_point: str, to_point: str) -> str:
    """
    Get the distance between two waypoints.

    Args:
        from_point: Starting waypoint ID (e.g., "A1", "T3")
        to_point: Destination waypoint ID
    """
    if _distance_matrix is None:
        return "Error: No distance matrix available"

    if from_point not in _distance_matrix:
        return f"Error: Unknown waypoint '{from_point}'"
    if to_point not in _distance_matrix.get(from_point, {}):
        return f"Error: Unknown waypoint '{to_point}'"

    distance = _distance_matrix[from_point][to_point]
    return f"Distance from {from_point} to {to_point}: {distance:.2f} fuel units"


@tool
def calculate_route_fuel(drone_id: str, waypoints: List[str]) -> str:
    """
    Calculate total fuel consumption for a drone's route.

    Args:
        drone_id: The drone ID ("1" or "D1")
        waypoints: Ordered list of waypoint IDs (e.g., ["A1", "T3", "T7", "A1"])
    """
    if _distance_matrix is None:
        return "Error: No distance matrix available"

    drone_id = drone_id.replace("D", "").replace("d", "")
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})
    fuel_budget = cfg.get("fuel_budget", 200)

    if len(waypoints) < 2:
        return "Error: Route must have at least 2 waypoints"

    # Validate waypoints
    for wp in waypoints:
        if wp not in _distance_matrix:
            return f"Error: Unknown waypoint '{wp}'"

    # Calculate each leg
    legs = []
    total_fuel = 0.0

    for i in range(len(waypoints) - 1):
        from_wp = waypoints[i]
        to_wp = waypoints[i + 1]
        distance = _distance_matrix[from_wp][to_wp]
        total_fuel += distance
        legs.append(f"  {from_wp} -> {to_wp}: {distance:.2f} fuel")

    lines = [f"Route fuel calculation for D{drone_id}:"] + legs
    lines.append("")
    lines.append(f"Total fuel: {total_fuel:.2f}")
    lines.append(f"Fuel budget: {fuel_budget}")
    lines.append(f"Remaining: {fuel_budget - total_fuel:.2f}")

    if total_fuel <= fuel_budget:
        lines.append("Status: ✓ VALID (within budget)")
    else:
        lines.append(f"Status: ✗ INVALID (exceeds budget by {total_fuel - fuel_budget:.2f})")

    return "\n".join(lines)


@tool
def validate_drone_route(drone_id: str, waypoints: List[str]) -> str:
    """
    Fully validate a route for a specific drone against ALL constraints:
    - Fuel budget
    - Start/end airports
    - Target type access
    - No duplicate targets

    Args:
        drone_id: The drone ID ("1" or "D1")
        waypoints: Ordered list of waypoint IDs (e.g., ["A1", "T3", "T7", "A1"])
    """
    if _distance_matrix is None:
        return "Error: No distance matrix available"

    drone_id = drone_id.replace("D", "").replace("d", "")
    env = get_environment()
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})

    fuel_budget = cfg.get("fuel_budget", 200)
    start_airport = cfg.get("start_airport", "A1")
    end_airport = cfg.get("end_airport", "A1")
    target_access = cfg.get("target_access", {})

    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}
    airports = {a.get("id", a.get("label")): a for a in env.get("airports", [])}

    errors = []

    if len(waypoints) < 2:
        return f"Error: Route for D{drone_id} must have at least 2 waypoints"

    # Check all waypoints are valid
    for wp in waypoints:
        if wp not in _distance_matrix:
            errors.append(f"Unknown waypoint: {wp}")

    if errors:
        return f"Validation FAILED for D{drone_id}:\n" + "\n".join(f"  - {e}" for e in errors)

    # Check starts at correct airport
    if waypoints[0] != start_airport:
        errors.append(f"Must start at {start_airport}, but starts at {waypoints[0]}")

    # Check ends at correct airport (or any airport if flexible endpoint "-")
    if end_airport == "-":
        # Flexible endpoint: accept any airport as valid end
        if waypoints[-1] not in airports:
            errors.append(f"Must end at an airport (flexible mode), but ends at {waypoints[-1]}")
    elif waypoints[-1] != end_airport:
        errors.append(f"Must end at {end_airport}, but ends at {waypoints[-1]}")

    # Check target type access
    target_visits = []
    for wp in waypoints:
        if wp in targets:
            target_visits.append(wp)
            t = targets[wp]
            ttype = t.get("type", "a").lower()
            if target_access:
                if not target_access.get(ttype, True):
                    errors.append(f"D{drone_id} cannot access type {ttype.upper()} target {wp}")

    # Check for duplicate targets
    if len(target_visits) != len(set(target_visits)):
        duplicates = [wp for wp in set(target_visits) if target_visits.count(wp) > 1]
        errors.append(f"Duplicate target visits: {duplicates}")

    # Calculate fuel
    total_fuel = 0.0
    for i in range(len(waypoints) - 1):
        total_fuel += _distance_matrix[waypoints[i]][waypoints[i + 1]]

    if total_fuel > fuel_budget:
        errors.append(f"Exceeds fuel budget: {total_fuel:.2f} > {fuel_budget}")

    # Calculate points
    total_points = sum(targets[wp].get("priority", targets[wp].get("value", 5))
                       for wp in target_visits if wp in targets)

    # Build result
    lines = []
    if errors:
        lines.append(f"=== VALIDATION FAILED for D{drone_id} ===")
        for e in errors:
            lines.append(f"  ✗ {e}")
    else:
        lines.append(f"=== VALIDATION PASSED for D{drone_id} ===")

    lines.append("")
    lines.append(f"Route: {' -> '.join(waypoints)}")
    lines.append(f"Targets visited: {len(target_visits)}")
    lines.append(f"Total points: {total_points}")
    lines.append(f"Total fuel: {total_fuel:.2f} / {fuel_budget}")

    if not errors:
        lines.append("")
        lines.append(f"✓ This route is valid for D{drone_id}!")

    return "\n".join(lines)


@tool
def find_accessible_targets(drone_id: str, from_point: str) -> str:
    """
    Find targets accessible to a specific drone, sorted by efficiency (points per fuel).
    Takes into account the drone's target type restrictions.

    Args:
        drone_id: The drone ID ("1" or "D1")
        from_point: The starting waypoint ID
    """
    if _distance_matrix is None:
        return "Error: No distance matrix available"

    drone_id = drone_id.replace("D", "").replace("d", "")
    env = get_environment()
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})

    target_access = cfg.get("target_access", {})
    end_airport = cfg.get("end_airport", "A1")

    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    if from_point not in _distance_matrix:
        return f"Error: Unknown waypoint '{from_point}'"

    efficiencies = []
    for target_id, target in targets.items():
        ttype = target.get("type", "a").lower()

        # Check if drone can access this type
        if target_access:
            if not target_access.get(ttype, True):
                continue

        dist_to = _distance_matrix[from_point][target_id]
        dist_back = _distance_matrix[target_id][end_airport]
        total_cost = dist_to + dist_back
        priority = target.get("priority", target.get("value", 5))
        efficiency = priority / total_cost if total_cost > 0 else 0
        efficiencies.append((target_id, priority, ttype.upper(), dist_to, total_cost, efficiency))

    # Sort by efficiency
    efficiencies.sort(key=lambda x: x[5], reverse=True)

    lines = [f"Accessible targets for D{drone_id} from {from_point} (sorted by efficiency):"]
    lines.append(f"{'Target':<8} {'Pts':<5} {'Type':<6} {'Dist':<8} {'Round':<8} {'Eff':<8}")
    lines.append("-" * 55)
    for tid, pts, ttype, dist, total, eff in efficiencies:
        lines.append(f"{tid:<8} {pts:<5} {ttype:<6} {dist:<8.2f} {total:<8.2f} {eff:<8.3f}")

    return "\n".join(lines)


@tool
def suggest_drone_route(drone_id: str) -> str:
    """
    Suggest an optimized route for a specific drone using a greedy algorithm.
    Respects the drone's constraints (fuel, start/end, type access).

    Args:
        drone_id: The drone ID ("1" or "D1")
    """
    if _distance_matrix is None:
        return "Error: No distance matrix available"

    drone_id = drone_id.replace("D", "").replace("d", "")
    env = get_environment()
    configs = get_drone_configs()
    cfg = configs.get(drone_id, {})

    fuel_budget = cfg.get("fuel_budget", 200)
    start_airport = cfg.get("start_airport", "A1")
    end_airport = cfg.get("end_airport", "A1")
    target_access = cfg.get("target_access", {})

    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    # Filter by accessible types
    accessible_targets = {}
    for tid, t in targets.items():
        ttype = t.get("type", "a").lower()
        if target_access:
            if target_access.get(ttype, True):
                accessible_targets[tid] = t
        else:
            accessible_targets[tid] = t

    route = [start_airport]
    visited = set()
    fuel_used = 0.0
    total_points = 0
    current = start_airport

    while True:
        best_target = None
        best_efficiency = -1

        for tid, target in accessible_targets.items():
            if tid in visited:
                continue

            dist_to = _distance_matrix[current][tid]
            dist_back = _distance_matrix[tid][end_airport]

            if fuel_used + dist_to + dist_back <= fuel_budget:
                priority = target.get("priority", target.get("value", 5))
                efficiency = priority / dist_to if dist_to > 0 else float('inf')

                if efficiency > best_efficiency:
                    best_efficiency = efficiency
                    best_target = tid

        if best_target is None:
            break

        dist = _distance_matrix[current][best_target]
        route.append(best_target)
        visited.add(best_target)
        fuel_used += dist
        total_points += accessible_targets[best_target].get("priority",
                        accessible_targets[best_target].get("value", 5))
        current = best_target

    # Return to end airport
    fuel_used += _distance_matrix[current][end_airport]
    route.append(end_airport)

    lines = [
        f"=== SUGGESTED ROUTE for D{drone_id} ===",
        f"Route: {' -> '.join(route)}",
        f"Targets: {len(visited)}",
        f"Points: {total_points}",
        f"Fuel: {fuel_used:.2f} / {fuel_budget}",
        f"Remaining: {fuel_budget - fuel_used:.2f}",
        "",
        f"ROUTE_D{drone_id}: {','.join(route)}",
    ]

    return "\n".join(lines)


@tool
def solve_optimal_route(drone_id: str) -> str:
    """
    Solve for the OPTIMAL route for a specific drone using the Held-Karp algorithm.
    This finds the mathematically optimal solution that maximizes points within fuel budget.
    Use this instead of suggest_drone_route for better results.

    Args:
        drone_id: The drone ID ("1" or "D1")
    """
    drone_id = drone_id.replace("D", "").replace("d", "")
    env = get_environment()
    configs = get_drone_configs()

    if drone_id not in configs:
        return f"Error: Drone {drone_id} not found in configuration"

    # Create a single-drone config for the solver
    single_drone_config = {drone_id: configs[drone_id]}

    # Call the real Held-Karp solver
    result = solve_mission(env, single_drone_config)

    # Extract the route for this drone
    route_data = result.get("routes", {}).get(drone_id, {})
    route = route_data.get("route", [])
    points = route_data.get("points", 0)
    distance = route_data.get("distance", 0)
    fuel_budget = route_data.get("fuel_budget", 0)

    if not route:
        return f"No feasible route found for D{drone_id}"

    # Get target details
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}
    target_details = []
    for wp in route:
        if wp in targets:
            t = targets[wp]
            target_details.append(f"  {wp}: priority {t.get('priority', 5)}, type {t.get('type', 'A').upper()}")

    lines = [
        f"=== OPTIMAL ROUTE for D{drone_id} (Held-Karp) ===",
        f"Route: {' -> '.join(route)}",
        f"Targets visited: {len([w for w in route if w.startswith('T')])}",
        f"Points: {points}",
        f"Fuel: {distance:.2f} / {fuel_budget}",
        f"Remaining: {fuel_budget - distance:.2f}",
        "",
        "Target details:",
    ]
    lines.extend(target_details)
    lines.append("")
    lines.append(f"ROUTE_D{drone_id}: {','.join(route)}")

    return "\n".join(lines)


@tool
def solve_constrained_route(
    drone_id: str,
    start_waypoint: str,
    end_waypoint: str,
    fuel_budget: float,
    exclude_targets: str = ""
) -> str:
    """
    Solve for optimal route with CUSTOM start/end waypoints and fuel budget.
    Use this when the user specifies route constraints like "first target must be X".

    Args:
        drone_id: The drone ID ("1" or "D1") - used to get target type access
        start_waypoint: Starting waypoint ID (can be airport or target, e.g., "A1" or "T9")
        end_waypoint: Ending waypoint ID (can be airport or target, e.g., "A1" or "T2")
        fuel_budget: Available fuel for this segment
        exclude_targets: Comma-separated target IDs to exclude (e.g., "T9,T5" if they're in fixed segments)

    Example: For "first target must be T9" constraint:
        1. First get distance A1->T9 using get_distance
        2. Call solve_constrained_route(drone_id="1", start_waypoint="T9", end_waypoint="A1",
                                         fuel_budget=250-dist_A1_T9, exclude_targets="T9")
        3. Prepend A1,T9 to the returned route
    """
    import math

    drone_id = drone_id.replace("D", "").replace("d", "")
    env = get_environment()
    configs = get_drone_configs()

    if drone_id not in configs:
        return f"Error: Drone {drone_id} not found in configuration"

    cfg = configs[drone_id]

    # Parse excluded targets
    excluded = set(t.strip() for t in exclude_targets.split(",") if t.strip())

    # Get target access for this drone
    target_access = cfg.get("target_access", {})
    allowed_types = {t.lower() for t, enabled in target_access.items() if enabled}
    if not allowed_types:
        allowed_types = {"a", "b", "c", "d", "e"}

    # Filter targets
    airports = env.get("airports", [])
    all_targets = env.get("targets", [])

    # Filter by type access and exclusions
    filtered_targets = [
        t for t in all_targets
        if t.get("id", t.get("label")) not in excluded
        and t.get("type", "a").lower() in allowed_types
    ]

    # Build distance matrix
    # First, we need to include start/end waypoints (which might be targets)
    all_waypoints = list(airports)  # Start with airports

    # Add targets (filtered)
    all_waypoints.extend(filtered_targets)

    # Make sure start and end waypoints are in the list
    start_wp = None
    end_wp = None

    # Find start waypoint
    for wp in airports + all_targets:
        wp_id = wp.get("id", wp.get("label"))
        if wp_id == start_waypoint:
            start_wp = wp
            break

    # Find end waypoint
    for wp in airports + all_targets:
        wp_id = wp.get("id", wp.get("label"))
        if wp_id == end_waypoint:
            end_wp = wp
            break

    if not start_wp:
        return f"Error: Start waypoint '{start_waypoint}' not found"
    if not end_wp:
        return f"Error: End waypoint '{end_waypoint}' not found"

    # Build labels and ensure start/end are included
    labels = []
    waypoints_for_matrix = []

    # Add start waypoint first (as "airport" for solver)
    labels.append(start_waypoint)
    waypoints_for_matrix.append(start_wp)

    # Add end waypoint if different from start
    if end_waypoint != start_waypoint:
        labels.append(end_waypoint)
        waypoints_for_matrix.append(end_wp)

    # Add other airports (not start/end)
    for a in airports:
        aid = a.get("id")
        if aid not in labels:
            labels.append(aid)
            waypoints_for_matrix.append(a)

    # Add filtered targets (not start/end)
    for t in filtered_targets:
        tid = t.get("id", t.get("label"))
        if tid not in labels:
            labels.append(tid)
            waypoints_for_matrix.append(t)

    # Use the global SAM-aware distance matrix
    if _distance_matrix is None:
        return "Error: Distance matrix not initialized. Call get_mission_overview first."

    # Build matrix from global SAM-aware distances
    n = len(labels)
    matrix = [[0.0] * n for _ in range(n)]
    for i in range(n):
        from_id = labels[i]
        for j in range(n):
            to_id = labels[j]
            if from_id in _distance_matrix and to_id in _distance_matrix.get(from_id, {}):
                matrix[i][j] = _distance_matrix[from_id][to_id]
            else:
                # Fallback to Euclidean if not in SAM matrix
                xi, yi = waypoints_for_matrix[i]["x"], waypoints_for_matrix[i]["y"]
                xj, yj = waypoints_for_matrix[j]["x"], waypoints_for_matrix[j]["y"]
                matrix[i][j] = math.hypot(xj - xi, yj - yi)

    # Build solver environment
    # Treat start waypoint as airport for solver
    solver_airports = [{"id": start_waypoint, "x": start_wp["x"], "y": start_wp["y"]}]
    if end_waypoint != start_waypoint:
        solver_airports.append({"id": end_waypoint, "x": end_wp["x"], "y": end_wp["y"]})

    # Add real airports that aren't start/end
    for a in airports:
        if a.get("id") not in [start_waypoint, end_waypoint]:
            solver_airports.append(a)

    solver_targets = [
        {"id": t.get("id", t.get("label")), "x": t["x"], "y": t["y"],
         "priority": t.get("priority", 5), "type": t.get("type", "a")}
        for t in filtered_targets
        if t.get("id", t.get("label")) not in [start_waypoint, end_waypoint]
    ]

    solver_env = {
        "airports": solver_airports,
        "targets": solver_targets,
        "matrix_labels": labels,
        "distance_matrix": matrix,
        "start_airport": start_waypoint,
        "end_airport": end_waypoint,
        "fuel_budget": fuel_budget
    }

    # Call the solver
    from orienteering_with_matrix import solve_orienteering_with_matrix
    result = solve_orienteering_with_matrix(
        solver_env,
        start_id=start_waypoint,
        end_id=end_waypoint,
        fuel_cap=fuel_budget
    )

    route = result.get("route", [])
    distance = result.get("distance", 0)
    points = result.get("total_points", 0)
    visited = result.get("visited_targets", [])

    # Get target details
    target_map = {t.get("id", t.get("label")): t for t in all_targets}
    target_details = []
    for tid in visited:
        if tid in target_map:
            t = target_map[tid]
            target_details.append(f"  {tid}: priority {t.get('priority', 5)}")

    lines = [
        f"=== CONSTRAINED ROUTE ({start_waypoint} -> {end_waypoint}) ===",
        f"Route: {' -> '.join(route)}",
        f"Targets visited: {len(visited)}",
        f"Points: {points}",
        f"Distance: {distance:.2f} / {fuel_budget:.2f}",
        f"Remaining: {fuel_budget - distance:.2f}",
        "",
        "Targets in route:",
    ]
    lines.extend(target_details)
    lines.append("")
    lines.append(f"Route segment: {','.join(route)}")

    return "\n".join(lines)


@tool
def check_target_conflicts(routes: Dict[str, str]) -> str:
    """
    Check if multiple drones are assigned the same targets.

    Args:
        routes: Dictionary mapping drone IDs to route strings, e.g. {"1": "A1,T3,T7,A1", "2": "A2,T5,A2"}
    """
    env = get_environment()
    targets = {t.get("id", t.get("label")) for t in env.get("targets", [])}

    # Parse routes and find targets
    drone_targets = {}
    for drone_id, route_str in routes.items():
        drone_id = drone_id.replace("D", "").replace("d", "")
        waypoints = [wp.strip() for wp in route_str.split(",")]
        drone_targets[drone_id] = [wp for wp in waypoints if wp in targets]

    # Find conflicts
    all_targets = []
    for targets_list in drone_targets.values():
        all_targets.extend(targets_list)

    duplicates = [t for t in set(all_targets) if all_targets.count(t) > 1]

    lines = ["=== TARGET ASSIGNMENT CHECK ==="]
    for drone_id, targets_list in drone_targets.items():
        lines.append(f"D{drone_id}: {', '.join(targets_list) if targets_list else '(none)'}")

    if duplicates:
        lines.append("")
        lines.append("⚠️ CONFLICTS FOUND - These targets are assigned to multiple drones:")
        for dup in duplicates:
            assigned_to = [f"D{d}" for d, tl in drone_targets.items() if dup in tl]
            lines.append(f"  {dup}: assigned to {', '.join(assigned_to)}")
    else:
        lines.append("")
        lines.append("✓ No conflicts - each target assigned to at most one drone")

    return "\n".join(lines)


@tool
def get_mission_summary(routes: Dict[str, str]) -> str:
    """
    Get a complete summary of a multi-drone mission plan.

    Args:
        routes: Dictionary mapping drone IDs to route strings, e.g. {"1": "A1,T3,T7,A1", "2": "A2,T5,A2"}
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    total_points = 0
    total_fuel = 0.0
    all_visited = set()

    lines = ["=" * 60, "MISSION SUMMARY", "=" * 60, ""]

    for drone_id, route_str in sorted(routes.items()):
        drone_id = drone_id.replace("D", "").replace("d", "")
        cfg = configs.get(drone_id, {})
        fuel_budget = cfg.get("fuel_budget", 200)

        waypoints = [wp.strip() for wp in route_str.split(",")]

        # Calculate fuel
        fuel = 0.0
        for i in range(len(waypoints) - 1):
            if waypoints[i] in _distance_matrix and waypoints[i+1] in _distance_matrix.get(waypoints[i], {}):
                fuel += _distance_matrix[waypoints[i]][waypoints[i+1]]

        # Calculate points
        drone_points = 0
        drone_targets = []
        for wp in waypoints:
            if wp in targets and wp not in all_visited:
                drone_points += targets[wp].get("priority", targets[wp].get("value", 5))
                drone_targets.append(wp)
                all_visited.add(wp)

        total_points += drone_points
        total_fuel += fuel

        lines.append(f"D{drone_id}: {route_str}")
        lines.append(f"  Targets: {', '.join(drone_targets) if drone_targets else '(none)'}")
        lines.append(f"  Points: {drone_points}")
        lines.append(f"  Fuel: {fuel:.2f} / {fuel_budget}")
        lines.append("")

    # Total stats
    total_possible = sum(t.get("priority", t.get("value", 5)) for t in targets.values())
    unvisited = [tid for tid in targets.keys() if tid not in all_visited]

    lines.append("-" * 40)
    lines.append(f"TOTAL POINTS: {total_points} / {total_possible} ({100*total_points/total_possible:.1f}%)")
    lines.append(f"TOTAL FUEL: {total_fuel:.2f}")
    lines.append(f"TARGETS VISITED: {len(all_visited)} / {len(targets)}")

    if unvisited:
        lines.append(f"UNVISITED: {', '.join(sorted(unvisited))}")

    return "\n".join(lines)


@tool
def optimize_assign_unvisited(routes: Dict[str, str], priority_constraints: str = "") -> str:
    """
    Assign unvisited targets to drones that have spare fuel capacity.
    Inserts targets at optimal positions in existing routes to minimize
    additional fuel consumption while maximizing coverage.

    IMPORTANT: Always pass priority_constraints if the mission has priority restrictions.
    This ensures optimization respects mission constraints like "D1,D2 visit priority>=6 only".

    Args:
        routes: Dictionary mapping drone IDs to route strings, e.g. {"1": "A1,T3,T7,A1", "2": "A2,T5,A2"}
        priority_constraints: Priority constraints string, e.g. "D1,D2: priority>=6; D3,D4: priority<6"

    Returns:
        Optimized routes with unvisited targets assigned, or error message.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    # Build solution structure expected by post_optimizer
    solution = {"routes": {}}
    for drone_id, route_str in routes.items():
        drone_id = drone_id.replace("D", "").replace("d", "")
        waypoints = [wp.strip() for wp in route_str.split(",")]
        cfg = configs.get(drone_id, {})

        # Calculate distance
        total_dist = 0.0
        if _distance_matrix:
            for i in range(len(waypoints) - 1):
                if waypoints[i] in _distance_matrix:
                    total_dist += _distance_matrix[waypoints[i]].get(waypoints[i+1], 0)

        solution["routes"][drone_id] = {
            "route": waypoints,
            "sequence": route_str,
            "distance": total_dist,
            "fuel_budget": cfg.get("fuel_budget", 200),
            "points": sum(targets[wp].get("priority", 5) for wp in waypoints if wp in targets)
        }

    # Build distance matrix in expected format
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Run optimizer with priority constraints
    result = post_optimize_solution(solution, env, configs, matrix_data, priority_constraints)

    # Format result
    lines = ["=== UNVISITED TARGET ASSIGNMENT ===", ""]

    for did, route_data in sorted(result.get("routes", {}).items()):
        route = route_data.get("route", [])
        lines.append(f"D{did}: {','.join(route)}")
        lines.append(f"  Points: {route_data.get('points', 0)}, Fuel: {route_data.get('distance', 0):.1f}")

    lines.append("")
    lines.append("Updated routes (copy these):")
    for did, route_data in sorted(result.get("routes", {}).items()):
        lines.append(f"ROUTE_D{did}: {','.join(route_data.get('route', []))}")

    return "\n".join(lines)


@tool
def optimize_reassign_targets(routes: Dict[str, str], priority_constraints: str = "") -> str:
    """
    Reassign targets to drones whose trajectories pass closer to them.
    For each target, if another drone's path passes closer AND that drone
    can carry the target type AND meets priority constraints, move it.

    IMPORTANT: Always pass priority_constraints if the mission has priority restrictions.
    This ensures swaps respect mission constraints like "D1,D2 visit priority>=6 only".

    Args:
        routes: Dictionary mapping drone IDs to route strings, e.g. {"1": "A1,T3,T7,A1", "2": "A2,T5,A2"}
        priority_constraints: Priority constraints string, e.g. "D1,D2: priority>=6; D3,D4: priority<6"

    Returns:
        Optimized routes with targets reassigned, including swap details.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    # Build solution structure
    solution = {"routes": {}}
    for drone_id, route_str in routes.items():
        drone_id = drone_id.replace("D", "").replace("d", "")
        waypoints = [wp.strip() for wp in route_str.split(",")]
        cfg = configs.get(drone_id, {})

        total_dist = 0.0
        if _distance_matrix:
            for i in range(len(waypoints) - 1):
                if waypoints[i] in _distance_matrix:
                    total_dist += _distance_matrix[waypoints[i]].get(waypoints[i+1], 0)

        solution["routes"][drone_id] = {
            "route": waypoints,
            "sequence": route_str,
            "distance": total_dist,
            "fuel_budget": cfg.get("fuel_budget", 200),
            "points": sum(targets[wp].get("priority", 5) for wp in waypoints if wp in targets)
        }

    # Build distance matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Run trajectory swap optimizer with priority constraints
    result = trajectory_swap_optimize(solution, env, configs, matrix_data, priority_constraints)

    # Format result
    lines = ["=== TARGET REASSIGNMENT OPTIMIZATION ===", ""]

    swaps = result.get("swaps_made", [])
    if swaps:
        lines.append(f"Made {len(swaps)} swaps:")
        for swap in swaps[:10]:  # Show first 10
            lines.append(f"  {swap['target']}: D{swap['from_drone']} → D{swap['to_drone']} (saved {swap['savings']:.1f} fuel)")
        if len(swaps) > 10:
            lines.append(f"  ... and {len(swaps) - 10} more")
    else:
        lines.append("No beneficial swaps found - routes are already optimized.")

    lines.append("")
    lines.append("Updated routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        route = route_data.get("route", [])
        lines.append(f"D{did}: {','.join(route)}")
        lines.append(f"  Points: {route_data.get('points', 0)}, Fuel: {route_data.get('distance', 0):.1f}")

    lines.append("")
    lines.append("Copy these routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        lines.append(f"ROUTE_D{did}: {','.join(route_data.get('route', []))}")

    return "\n".join(lines)


@tool
def optimize_remove_crossings(routes: Dict[str, str]) -> str:
    """
    Remove self-crossings from drone trajectories using 2-opt algorithm.
    When segment A→B crosses segment C→D, reverse the middle portion
    to eliminate the crossing and reduce total distance.

    Args:
        routes: Dictionary mapping drone IDs to route strings, e.g. {"1": "A1,T3,T7,A1", "2": "A2,T5,A2"}

    Returns:
        Optimized routes with crossings removed, including fix details.
    """
    env = get_environment()
    configs = get_drone_configs()
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    # Build solution structure
    solution = {"routes": {}}
    for drone_id, route_str in routes.items():
        drone_id = drone_id.replace("D", "").replace("d", "")
        waypoints = [wp.strip() for wp in route_str.split(",")]
        cfg = configs.get(drone_id, {})

        total_dist = 0.0
        if _distance_matrix:
            for i in range(len(waypoints) - 1):
                if waypoints[i] in _distance_matrix:
                    total_dist += _distance_matrix[waypoints[i]].get(waypoints[i+1], 0)

        solution["routes"][drone_id] = {
            "route": waypoints,
            "sequence": route_str,
            "distance": total_dist,
            "fuel_budget": cfg.get("fuel_budget", 200),
            "points": sum(targets[wp].get("priority", 5) for wp in waypoints if wp in targets)
        }

    # Build distance matrix
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Run crossing removal optimizer
    result = crossing_removal_optimize(solution, env, configs, matrix_data)

    # Format result
    lines = ["=== CROSSING REMOVAL (2-OPT) ===", ""]

    fixes = result.get("fixes_made", [])
    if fixes:
        lines.append(f"Fixed {len(fixes)} crossings:")
        for fix in fixes[:10]:  # Show first 10
            lines.append(f"  D{fix['drone']}: reversed segment at positions {fix['segment_i']}-{fix['segment_j']}")
        if len(fixes) > 10:
            lines.append(f"  ... and {len(fixes) - 10} more")
    else:
        lines.append("No crossings found - routes are already optimal.")

    lines.append("")
    lines.append("Updated routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        route = route_data.get("route", [])
        lines.append(f"D{did}: {','.join(str(wp) for wp in route)}")
        lines.append(f"  Points: {route_data.get('points', 0)}, Fuel: {route_data.get('distance', 0):.1f}")

    lines.append("")
    lines.append("Copy these routes:")
    for did, route_data in sorted(result.get("routes", {}).items()):
        lines.append(f"ROUTE_D{did}: {','.join(str(wp) for wp in route_data.get('route', []))}")

    return "\n".join(lines)


@tool
def allocate_targets_to_drones(strategy: str = "efficient") -> str:
    """
    CRITICAL FOR MULTI-DRONE PLANNING: Allocate targets to drones BEFORE solving routes.

    This tool divides targets among enabled drones to:
    - Prevent solver from exploring all 2^N subsets for each drone
    - Ensure each target is assigned to exactly one drone
    - Respect drone type restrictions (which target types each drone can access)
    - Balance workload efficiently across drones

    Call this FIRST when planning multi-drone missions, BEFORE calling solve_allocated_route.

    Args:
        strategy: Allocation strategy to use:
            - "efficient": Maximize priority/distance ratio (recommended)
            - "greedy": Assign highest priority targets to nearest capable drone
            - "balanced": Distribute targets evenly by count
            - "geographic": Divide environment into angular sectors
            - "exclusive": Prioritize targets only one drone can reach first

    Returns:
        Target allocation showing which targets are assigned to each drone.
        Use these allocations with solve_allocated_route.
    """
    env = get_environment()
    configs = get_drone_configs()

    # Build distance matrix data for allocator
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Call the allocator
    allocation = _allocate_targets_impl(env, configs, strategy, matrix_data)

    # Store allocation globally for solve_allocated_route to use
    global _target_allocation
    _target_allocation = allocation

    # Format result
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    lines = [
        "=" * 60,
        f"TARGET ALLOCATION (strategy: {strategy})",
        "=" * 60,
        "",
    ]

    total_assigned = 0
    total_points = 0

    for drone_id in sorted(allocation.keys()):
        cfg = configs.get(drone_id, {})
        if not cfg.get("enabled", True):
            continue

        assigned_targets = allocation[drone_id]
        drone_points = sum(targets[tid].get("priority", 5) for tid in assigned_targets if tid in targets)
        total_assigned += len(assigned_targets)
        total_points += drone_points

        lines.append(f"D{drone_id} ({len(assigned_targets)} targets, {drone_points} pts):")
        for tid in assigned_targets:
            if tid in targets:
                t = targets[tid]
                lines.append(f"  {tid}: priority={t.get('priority', 5)}, type={t.get('type', 'A').upper()}")
        lines.append("")

    # Check for unassigned targets
    all_target_ids = set(targets.keys())
    assigned_ids = set()
    for tids in allocation.values():
        assigned_ids.update(tids)
    unassigned = all_target_ids - assigned_ids

    if unassigned:
        lines.append(f"⚠️ UNASSIGNED TARGETS: {', '.join(sorted(unassigned))}")
        lines.append("  (These targets may be inside SAM zones or have no capable drone)")
        lines.append("")

    lines.append("-" * 40)
    lines.append(f"Total assigned: {total_assigned} targets, {total_points} points")
    lines.append("")
    lines.append("Next: Call solve_allocated_route for each drone to get optimal routes")

    return "\n".join(lines)


# Store target allocation globally
_target_allocation: Optional[Dict[str, List[str]]] = None


@tool
def solve_allocated_route(drone_id: str) -> str:
    """
    Solve optimal route for a drone using ONLY its allocated targets.

    MUST call allocate_targets_to_drones FIRST to get allocations.
    This solves much faster than solve_optimal_route because it only
    considers the drone's assigned targets, not all targets.

    Args:
        drone_id: The drone ID ("1" or "D1")

    Returns:
        Optimal route for this drone visiting only its allocated targets.
    """
    global _target_allocation

    drone_id = drone_id.replace("D", "").replace("d", "")

    if _target_allocation is None:
        return "Error: Must call allocate_targets_to_drones first to get allocations!"

    allocated = _target_allocation.get(drone_id, [])

    if not allocated:
        # No targets allocated - return empty route
        env = get_environment()
        configs = get_drone_configs()
        cfg = configs.get(drone_id, {})
        start = cfg.get("start_airport", "A1")
        end = cfg.get("end_airport", "A1")
        if end == "-":
            end = start  # If flexible, just return to start

        return f"""=== ROUTE for D{drone_id} (no allocated targets) ===
Route: {start} -> {end}
Targets: 0
Points: 0
Fuel: {_distance_matrix.get(start, {}).get(end, 0):.2f}

ROUTE_D{drone_id}: {start},{end}"""

    env = get_environment()
    configs = get_drone_configs()

    # Filter environment to only include allocated targets
    original_targets = env.get("targets", [])
    filtered_targets = [
        t for t in original_targets
        if t.get("id", t.get("label")) in allocated
    ]

    # Create filtered environment
    filtered_env = env.copy()
    filtered_env["targets"] = filtered_targets

    # Create single-drone config
    single_drone_config = {drone_id: configs.get(drone_id, {})}

    # Call solver with filtered environment
    result = solve_mission(filtered_env, single_drone_config)

    # Extract route
    route_data = result.get("routes", {}).get(drone_id, {})
    route = route_data.get("route", [])
    points = route_data.get("points", 0)
    distance = route_data.get("distance", 0)
    fuel_budget = route_data.get("fuel_budget", 0)

    if not route:
        return f"No feasible route found for D{drone_id} with allocated targets: {allocated}"

    # Get target details
    targets = {t.get("id", t.get("label")): t for t in original_targets}
    target_details = []
    for wp in route:
        if wp in targets:
            t = targets[wp]
            target_details.append(f"  {wp}: priority {t.get('priority', 5)}, type {t.get('type', 'A').upper()}")

    lines = [
        f"=== OPTIMAL ROUTE for D{drone_id} (allocated targets) ===",
        f"Allocated targets: {', '.join(allocated)}",
        f"Route: {' -> '.join(route)}",
        f"Targets visited: {len([w for w in route if w.startswith('T')])}",
        f"Points: {points}",
        f"Fuel: {distance:.2f} / {fuel_budget}",
        f"Remaining: {fuel_budget - distance:.2f}",
        "",
        "Target details:",
    ]
    lines.extend(target_details)
    lines.append("")
    lines.append(f"ROUTE_D{drone_id}: {','.join(route)}")

    return "\n".join(lines)


@tool
def solve_with_constraints(constraints: str, strategy: str = "efficient") -> str:
    """
    🚀 FAST ONE-SHOT SOLVER: Parse constraints, allocate, and solve ALL drones in PARALLEL.

    This is the FASTEST way to solve multi-drone missions with priority constraints.
    Use this instead of calling allocate_targets_to_drones + solve_allocated_route for each drone.

    Args:
        constraints: Priority constraints in format "D1,D2: priority>=6; D3,D4: priority<=6"
                    Supported operators: >=, <=, >, <, =
                    Examples:
                      - "D1,D2: priority>=6; D3,D4: priority<=6"
                      - "D1: priority>5; D2,D3,D4: priority<8"
                      - "D1,D2: priority=10" (only priority 10 targets)
        strategy: Allocation strategy (efficient, greedy, balanced, geographic, exclusive)

    Returns:
        Complete solution with routes for all drones, ready for UI display.
    """
    from concurrent.futures import ThreadPoolExecutor, as_completed
    import time

    start_time = time.time()

    env = get_environment()
    configs = get_drone_configs()
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}

    # Build distance matrix data for allocator
    matrix_data = None
    if _distance_matrix:
        labels = list(_distance_matrix.keys())
        matrix = [[_distance_matrix[f].get(t, 1000) for t in labels] for f in labels]
        matrix_data = {"labels": labels, "matrix": matrix}

    # Step 1: Parse constraints

    priority_filters = parse_priority_constraints(constraints)

    if not priority_filters:
        return "Error: Could not parse constraints. Use format: 'D1,D2: priority>=6; D3,D4: priority<=6'"

    # Step 2: Allocate with priority filters
    # IMPORTANT: Only allocate to drones explicitly mentioned in constraints
    # Drones NOT in constraints get NO targets (user chose Option 2)
    constrained_configs = {
        did: cfg for did, cfg in configs.items()
        if did in priority_filters and cfg.get("enabled", True)
    }

    if not constrained_configs:
        return "Error: No enabled drones found in constraints. Make sure drone IDs match (D1, D2, etc.)"


    allocation = allocate_with_priority_filters(
        env, constrained_configs, priority_filters, strategy, matrix_data
    )

    for did, tids in allocation.items():
        if tids:
            pts = sum(targets[t].get("priority", 5) for t in tids if t in targets)

    # Step 3: Solve all drones IN PARALLEL
    def solve_drone(drone_id: str, allocated_targets: List[str]) -> Dict[str, Any]:
        """Solve a single drone's route (runs in thread pool)."""
        if not allocated_targets:
            cfg = configs.get(drone_id, {})
            start = cfg.get("start_airport", "A1")
            end = cfg.get("end_airport", "A1")
            if end == "-":
                end = start
            return {
                "drone_id": drone_id,
                "route": [start, end],
                "points": 0,
                "distance": _distance_matrix.get(start, {}).get(end, 0) if _distance_matrix else 0,
                "fuel_budget": cfg.get("fuel_budget", 200),
            }

        # Filter environment to only include allocated targets
        original_targets = env.get("targets", [])
        filtered_targets = [
            t for t in original_targets
            if t.get("id", t.get("label")) in allocated_targets
        ]

        # Create filtered environment
        filtered_env = env.copy()
        filtered_env["targets"] = filtered_targets

        # Create single-drone config
        single_drone_config = {drone_id: configs.get(drone_id, {})}

        # Call solver
        result = solve_mission(filtered_env, single_drone_config)

        route_data = result.get("routes", {}).get(drone_id, {})
        return {
            "drone_id": drone_id,
            "route": route_data.get("route", []),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0),
            "fuel_budget": route_data.get("fuel_budget", 0),
        }

    # Execute in parallel - ONLY solve drones mentioned in constraints
    results = {}
    enabled_drones = list(constrained_configs.keys())


    with ThreadPoolExecutor(max_workers=len(enabled_drones)) as executor:
        futures = {
            executor.submit(solve_drone, did, allocation.get(did, [])): did
            for did in enabled_drones
        }

        for future in as_completed(futures):
            drone_id = futures[future]
            try:
                result = future.result()
                results[drone_id] = result
            except Exception as e:
                results[drone_id] = {"drone_id": drone_id, "route": [], "points": 0, "distance": 0}

    elapsed = time.time() - start_time

    # Step 4: Format output
    lines = [
        "=" * 60,
        "CONSTRAINED MISSION SOLUTION",
        f"Constraints: {constraints}",
        f"Strategy: {strategy}",
        f"Solve time: {elapsed:.2f}s",
        "=" * 60,
        "",
    ]

    total_points = 0
    total_fuel = 0.0
    all_visited = set()

    for did in sorted(results.keys()):
        result = results[did]
        route = result.get("route", [])
        points = result.get("points", 0)
        distance = result.get("distance", 0)
        fuel_budget = result.get("fuel_budget", 200)

        # Track visited targets
        for wp in route:
            if wp in targets:
                all_visited.add(wp)

        total_points += points
        total_fuel += distance

        lines.append(f"D{did}:")
        lines.append(f"  Route: {' -> '.join(route)}")
        lines.append(f"  Targets: {len([w for w in route if w.startswith('T')])}")
        lines.append(f"  Points: {points}")
        lines.append(f"  Fuel: {distance:.2f} / {fuel_budget}")
        lines.append("")

    # Summary
    total_possible = sum(t.get("priority", 5) for t in targets.values())
    unvisited = [tid for tid in targets.keys() if tid not in all_visited]

    lines.append("-" * 40)
    lines.append(f"TOTAL POINTS: {total_points} / {total_possible} ({100*total_points/total_possible:.1f}%)")
    lines.append(f"TOTAL FUEL: {total_fuel:.2f}")
    lines.append(f"TARGETS VISITED: {len(all_visited)} / {len(targets)}")

    if unvisited:
        lines.append(f"UNVISITED: {', '.join(sorted(unvisited))}")

    lines.append("")
    lines.append("=" * 60)
    lines.append("ROUTES (copy for UI):")
    lines.append("=" * 60)

    for did in sorted(results.keys()):
        result = results[did]
        route = result.get("route", [])
        if route:
            lines.append(f"ROUTE_D{did}: {','.join(route)}")

    # Store allocation globally for potential follow-up
    global _target_allocation
    _target_allocation = allocation

    return "\n".join(lines)


# All tools
ALL_TOOLS = [
    get_mission_overview,
    get_drone_info,
    get_distance,
    calculate_route_fuel,
    validate_drone_route,
    find_accessible_targets,
    suggest_drone_route,
    # 🚀 FAST: One-shot constrained solver (use this for priority-based missions)
    solve_with_constraints,  # Parse constraints, allocate, solve ALL drones in parallel
    # Multi-drone allocation workflow (use for 2+ drones)
    allocate_targets_to_drones,  # STEP 1: Divide targets among drones
    solve_allocated_route,  # STEP 2: Solve each drone's allocated targets
    # Single-drone solver (use for 1 drone only)
    solve_optimal_route,  # Held-Karp solver - use this for single drone
    solve_constrained_route,  # For routes with user-specified constraints (first/last target)
    check_target_conflicts,
    get_mission_summary,
    # Optimization tools
    optimize_assign_unvisited,
    optimize_reassign_targets,
    optimize_remove_crossings,
]


# ============================================================================
# System Prompt
# ============================================================================

SYSTEM_PROMPT = """You are an expert ISR (Intelligence, Surveillance, Reconnaissance) multi-drone mission planner embedded in a web-based planning tool.

================================================================================
HARD CONSTRAINTS - NEVER VIOLATE UNDER ANY CIRCUMSTANCES
================================================================================
These constraints are ABSOLUTE and must NEVER be violated, even during optimization:

1. FUEL BUDGET: A drone's route MUST NOT exceed its fuel capacity. This is mission-critical.
   Violating fuel budget = drone crashes = mission failure.

2. SAM/NFZ BOUNDARIES: Routes MUST avoid all SAM zones and No-Fly Zones.
   The distance matrix already accounts for this - use it.

3. MISSION CONSTRAINTS - ALL constraints must be respected:
   a) Target Type Accessibility: If drone config says target_access.c=false, that drone
      CANNOT visit type C targets, period. Not during planning, not during optimization.
   b) Priority Constraints: If user says "D1 visits priority>=6 only", D1 can ONLY visit
      targets with priority 6 or higher. NEVER assign lower priority targets to D1.
   c) Route Order Constraints: If user says "start at T5" or "visit T3 first", respect it.
   d) Drone Exclusions: If user says "D5 not used", D5 gets NO targets.

When optimizing: ONLY add targets that the drone is ALLOWED to visit per ALL its constraints.
If a target violates ANY constraint, it CANNOT be added to that drone's route.

REJECT any action (planning, optimization, or reassignment) that would violate these constraints.
================================================================================

IMPORTANT: You are NOT a chatbot that needs to explain what you "would" do. You ARE the planner.
When asked to plan routes, you MUST actually use your tools and output the routes.
The UI will automatically parse your ROUTE_Dx: output and display it.

Your job is to help plan optimal routes for MULTIPLE DRONES that:
1. Maximize total priority points collected across all drones
2. Respect each drone's individual fuel budget (CRITICAL - exceeding means mission failure)
3. Respect each drone's start and end airport constraints
4. Respect each drone's target type access restrictions (some drones can only visit certain target types)
5. Respect all mission constraints from the user's command (priority filters, route order, etc.)
6. Avoid assigning the same target to multiple drones (each target should be visited at most once)

AVAILABLE TOOLS:
- get_mission_overview: Get complete overview (airports, targets, ALL drone configs) - CALL THIS FIRST
- get_drone_info: Get detailed info for a specific drone
- get_distance: Look up distance between any two waypoints
- calculate_route_fuel: Calculate total fuel for a drone's route
- validate_drone_route: Validate a route against a drone's specific constraints
- find_accessible_targets: Find targets accessible to a specific drone (respects type restrictions)

🚀 FAST CONSTRAINED SOLVER (use when user specifies priority-based constraints):
- solve_with_constraints: **FASTEST** - One-shot solver for priority-based missions!
  Call with constraints like: "D1,D2: priority>=6; D3,D4: priority<=6"
  This parses constraints, allocates targets, and solves ALL drones IN PARALLEL.
  Supported operators: >=, <=, >, <, =
  Examples:
    * "D1,D2: priority>=6; D3,D4: priority<=6" (D1,D2 get high priority, D3,D4 get low priority)
    * "D1: priority>5; D2,D3,D4: priority<8" (mixed constraints)
    * "D1,D2: priority=10" (only priority 10 targets)

MULTI-DRONE ALLOCATION TOOLS (use for 2+ drones without priority constraints):
- allocate_targets_to_drones: **CRITICAL** - Divide targets among drones BEFORE solving.
  Available strategies (pass as "strategy" parameter):
    * "efficient" (default): Maximize priority/distance ratio - best for most missions
    * "greedy": Assign highest priority targets to nearest capable drone
    * "balanced": Distribute targets evenly by count
    * "geographic": Divide environment into angular sectors per drone
    * "exclusive": Prioritize targets only one drone can reach first
- solve_allocated_route: Solve optimal route for a drone using ONLY its allocated targets (FAST!)

SINGLE-DRONE SOLVER (use for 1 drone only):
- solve_optimal_route: Get the OPTIMAL route using Held-Karp algorithm (maximizes points)
- solve_constrained_route: Solve with CUSTOM start/end waypoints and fuel - use when user specifies route constraints
- suggest_drone_route: Get a quick greedy route (suboptimal)

VALIDATION & SUMMARY:
- check_target_conflicts: Check if any targets are assigned to multiple drones
- get_mission_summary: Get summary stats for a multi-drone plan

OPTIMIZATION TOOLS (use ONLY when user explicitly requests):
- optimize_assign_unvisited: Insert unvisited targets into routes (UI: "Insert Missed" button)
- optimize_reassign_targets: Swap targets to closer drone trajectories (UI: "Swap Closer" button)
- optimize_remove_crossings: Fix self-crossing trajectories using 2-opt (UI: "Cross Remove" button)

PRIORITY-CONSTRAINED WORKFLOW (when user specifies priority constraints):
When user says something like "D1,D2 visit priority>=6 targets; D3,D4 visit priority<=6":
1. FIRST call get_mission_overview to understand the mission
2. Call solve_with_constraints(constraints="D1,D2: priority>=6; D3,D4: priority<=6")
   - This ONE call does EVERYTHING: parse, allocate, and solve ALL drones in PARALLEL
   - Returns routes for ALL drones in ~0.5-2 seconds
3. OUTPUT the routes in ROUTE_Dx format and STOP

MULTI-DRONE WORKFLOW (for 2+ enabled drones WITHOUT priority constraints):
1. FIRST call get_mission_overview to understand ALL drones and their constraints
2. Call allocate_targets_to_drones(strategy="efficient") to divide targets among drones
   - This ensures each target is assigned to exactly one drone
   - Respects drone type restrictions (which target types each can access)
   - Prevents the solver from exploring all 2^N subsets (MUCH faster!)
3. For each enabled drone, call solve_allocated_route(drone_id) to get its optimal route
4. Validate each drone's route with validate_drone_route
5. Use get_mission_summary to verify the complete plan
6. OUTPUT the routes in ROUTE_Dx format and STOP

SINGLE-DRONE WORKFLOW (for 1 drone):
1. Call get_mission_overview
2. Call solve_optimal_route(drone_id) directly
3. Validate and output

OPTIMIZATION RULES (CRITICAL - read carefully):
- Do NOT automatically call optimization tools during planning
- Only use them when user explicitly asks to "optimize", "insert missed", "swap closer", or "remove crossings"
- optimize_remove_crossings: Call ONCE, only if routes visually cross themselves
- optimize_reassign_targets: Call AT MOST 8 times total, stop if no swaps are made
- optimize_assign_unvisited: Call ONCE, only if there are unvisited targets
- After ANY optimization tool call, OUTPUT the new routes and STOP immediately
- **ALWAYS PASS priority_constraints**: If the mission has priority constraints (e.g., "D1,D2: priority>=6"),
  you MUST pass them to optimize_assign_unvisited and optimize_reassign_targets. This ensures optimization
  respects the mission constraints and doesn't assign targets to drones that are not allowed to visit them.

CRITICAL RULES:
- Each drone has its OWN fuel budget, start/end airports, and type restrictions
- NEVER exceed any drone's fuel budget
- Drones may have different starting airports (e.g., D1 starts at A1, D2 starts at A2)
- Drones may have different ending airports
- ANY ENDPOINT: If a drone's end_airport is "Any" or "-", it can end at ANY airport.
  The solver automatically chooses the optimal ending airport to maximize points.
  When you see "ANY (optimal)" as end airport, the route can end at any airport.
- Some drones can only visit certain target types (A, B, C, D, E)
- Each target should only be visited by ONE drone
- Always validate routes before presenting them
- DO NOT explain what you "would" do or ask for additional tools - USE the tools you have!

HANDLING ROUTE CONSTRAINTS (e.g., "first target must be X" or "visit Y before returning"):
When the user specifies constraints on the route order, decompose the problem:

1. FIXED PREFIX (e.g., "first target must be T9"):
   - The segment from start_airport → T9 is FIXED
   - Use get_distance to find the cost of this fixed segment
   - Remaining fuel = original_fuel - distance(start_airport, T9)
   - Now solve: start=T9, end=original_end, fuel=remaining_fuel
   - Final route = [start_airport] + [T9] + [solver_result excluding T9]

2. FIXED SUFFIX (e.g., "end with T2 then T12 before returning"):
   - The segment T2 → T12 → end_airport is FIXED
   - Use get_distance to find costs: dist(T2,T12) + dist(T12, end_airport)
   - Remaining fuel = original_fuel - fixed_suffix_cost
   - Now solve: start=original_start, end=T2, fuel=remaining_fuel
   - Final route = [solver_result] + [T12] + [end_airport]

3. BOTH PREFIX AND SUFFIX:
   - Combine both: subtract prefix + suffix costs from fuel
   - Solve the middle segment with reduced budget
   - Stitch together: prefix + middle + suffix

Example: User says "T9 must be the first target visited"
- Get distance A1→T9 (e.g., 74.6 units)
- Remaining fuel: 250 - 74.6 = 175.4
- Solve from T9 to A1 with 175.4 fuel budget (excluding T9 from targets)
- Result: T9→T4→T7→T6→T2→T8→A1
- Final route: A1→T9→T4→T7→T6→T2→T8→A1

OUTPUT FORMAT - When presenting your final plan, you MUST include routes in this EXACT format:
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
ROUTE_D3: A1,T1,A1
(etc. for each enabled drone)

The UI automatically parses lines starting with "ROUTE_D" and loads them into the planner.
You do NOT need any special capabilities to update the UI - just output the routes in this format.

Include for each drone:
- The route sequence (in ROUTE_Dx format above)
- Points collected
- Fuel used

Then provide a brief summary of the total mission performance.
"""


# ============================================================================
# Agent Graph
# ============================================================================

def create_isr_agent():
    """Create the LangGraph workflow for the ISR agent."""

    llm = ChatAnthropic(
        model="claude-sonnet-4-20250514",
        temperature=0,
        max_tokens=4096,
    )

    llm_with_tools = llm.bind_tools(ALL_TOOLS)

    def agent_node(state: ISRAgentState) -> Dict[str, Any]:
        """Main agent node."""
        messages = state.get("messages", [])

        # Build system prompt with persistent memories
        memories_text = format_memories_for_prompt()
        full_system_prompt = SYSTEM_PROMPT + memories_text

        # Add system message if not present
        if not messages or not isinstance(messages[0], SystemMessage):
            messages = [SystemMessage(content=full_system_prompt)] + list(messages)

        response = llm_with_tools.invoke(messages)
        return {"messages": [response]}

    def should_continue(state: ISRAgentState) -> Literal["tools", "end"]:
        """Decide whether to continue to tools or end."""
        messages = state.get("messages", [])
        if not messages:
            return "end"

        last_message = messages[-1]
        if hasattr(last_message, "tool_calls") and last_message.tool_calls:
            return "tools"
        return "end"

    # Build graph
    workflow = StateGraph(ISRAgentState)
    workflow.add_node("agent", agent_node)
    workflow.add_node("tools", ToolNode(ALL_TOOLS))
    workflow.set_entry_point("agent")
    workflow.add_conditional_edges(
        "agent",
        should_continue,
        {"tools": "tools", "end": END}
    )
    workflow.add_edge("tools", "agent")

    return workflow.compile()


# Singleton
_isr_workflow = None


def get_isr_workflow():
    """Get or create the ISR workflow singleton."""
    global _isr_workflow
    if _isr_workflow is None:
        _isr_workflow = create_isr_agent()
    return _isr_workflow


def run_isr_agent(env: Dict[str, Any], user_query: str,
                  drone_configs: Optional[Dict[str, Any]] = None,
                  sequences: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    """
    Run the ISR agent with given environment, drone configs, and query.

    Returns:
        Dict with:
            - response: The agent's text response
            - routes: Dict of drone_id -> route list (e.g., {"1": ["A1", "T3", "A1"], ...})
            - total_points: Total points across all drones
            - total_fuel: Total fuel used across all drones
    """
    # Set context for tools
    set_context(env, drone_configs, sequences)

    # Get workflow
    workflow = get_isr_workflow()

    # Initial state
    initial_state = {
        "messages": [HumanMessage(content=user_query)],
        "environment": env,
        "drone_configs": drone_configs,
        "current_sequences": sequences,
    }

    # Run with recursion limit sized for multi-drone missions
    # Each tool call = 2 steps (agent + tool execution)
    # For 5 drones: overview(2) + 5*suggest(10) + conflicts(2) + 5*validate(10) + summary(2) = ~26 minimum
    # Setting to 100 to observe actual usage and determine optimal limit
    config = {"recursion_limit": 100}

    final_state = None
    step_count = 0
    tool_call_history = []  # Track tool calls to detect loops
    sys.stderr.flush()

    for step in workflow.stream(initial_state, config=config):
        step_count += 1
        # Debug: Log tool calls to identify loops
        for node_name, node_output in step.items():
            if node_name == "agent":
                msg = node_output.get("messages", [])[-1] if node_output.get("messages") else None
                if msg and hasattr(msg, "tool_calls") and msg.tool_calls:
                    tool_names = [tc["name"] for tc in msg.tool_calls]
                    tool_call_history.append(tool_names)
                    sys.stderr.flush()
                elif msg:
                    sys.stderr.flush()
            elif node_name == "tools":
                sys.stderr.flush()
        final_state = step

    sys.stderr.flush()

    # Extract response
    response_text = ""
    if final_state:
        for node_output in final_state.values():
            messages = node_output.get("messages", [])
            if messages:
                response_text = messages[-1].content

    # Parse routes from response (supports multi-drone format)
    routes = {}

    # Match ROUTE_D1: A1,T3,A1 format
    route_matches = re.findall(r'ROUTE_D(\d+):\s*([A-Z0-9,]+)', response_text)
    for drone_id, route_str in route_matches:
        routes[drone_id] = route_str.split(',')

    # Fallback: old single-drone format ROUTE_DATA: A1,T3,A1
    if not routes:
        route_match = re.search(r'ROUTE_DATA:\s*([A-Z0-9,]+)', response_text)
        if route_match:
            routes["1"] = route_match.group(1).split(',')

    # Calculate totals
    total_points = 0
    total_fuel = 0.0
    targets = {t.get("id", t.get("label")): t for t in env.get("targets", [])}
    visited = set()

    # Build trajectories with SAM-avoiding paths
    trajectories = {}

    for drone_id, route in routes.items():
        # Calculate points (no double counting)
        for wp in route:
            if wp in targets and wp not in visited:
                total_points += targets[wp].get("priority", targets[wp].get("value", 5))
                visited.add(wp)

        # Calculate fuel and build trajectory
        trajectory = []
        for i in range(len(route) - 1):
            from_wp = route[i]
            to_wp = route[i + 1]

            if _distance_matrix and from_wp in _distance_matrix:
                total_fuel += _distance_matrix[from_wp].get(to_wp, 0)

            # Check if there's a SAM-avoiding path for this segment
            sam_path = get_sam_path(from_wp, to_wp)
            if sam_path and len(sam_path) > 2:
                # Use the SAM-avoiding path (skip first point if not first segment)
                if trajectory:
                    trajectory.extend(sam_path[1:])  # Skip first to avoid duplicate
                else:
                    trajectory.extend(sam_path)
            else:
                # Direct path - get coordinates from environment
                if not trajectory:
                    # Add start point
                    start_pos = _get_waypoint_position(from_wp, env)
                    if start_pos:
                        trajectory.append(start_pos)
                # Add end point
                end_pos = _get_waypoint_position(to_wp, env)
                if end_pos:
                    trajectory.append(end_pos)

        trajectories[drone_id] = trajectory

    return {
        "response": response_text,
        "routes": routes,
        "trajectories": trajectories,  # SAM-avoiding trajectories for drawing
        "total_points": total_points,
        "total_fuel": total_fuel,
        # Legacy single-route for backward compatibility (use D1 if available)
        "route": routes.get("1"),
        "points": total_points,
        "fuel": total_fuel,
    }


def _get_waypoint_position(wp_id: str, env: Dict[str, Any]) -> Optional[List[float]]:
    """Get the [x, y] position of a waypoint from the environment."""
    # Check airports
    for airport in env.get("airports", []):
        aid = airport.get("id", airport.get("label"))
        if aid == wp_id:
            return [float(airport["x"]), float(airport["y"])]

    # Check targets
    for target in env.get("targets", []):
        tid = target.get("id", target.get("label"))
        if tid == wp_id:
            return [float(target["x"]), float(target["y"])]

    return None
