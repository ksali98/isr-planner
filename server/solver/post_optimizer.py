"""
Post-Optimizer for Multi-Drone Mission Planning

After initial route optimization, this module:
1. Identifies unvisited targets
2. Attempts to assign them to drones with remaining fuel capacity
3. Re-optimizes affected drone routes

This is a LangGraph-compatible tool.
"""

import math
import re
from typing import Dict, Any, List, Optional, Set, Tuple, Callable


# ============================================================================
# Priority Constraint Handling
# ============================================================================

def parse_priority_constraint(constraint_str: str) -> Dict[str, Callable[[int], bool]]:
    """
    Parse priority constraints string into per-drone filter functions.

    Format: "D1,D2: priority>=6; D3,D4: priority<6"

    Returns:
        Dict mapping drone_id -> function(priority) -> bool
    """
    if not constraint_str or not constraint_str.strip():
        return {}

    drone_filters: Dict[str, Callable[[int], bool]] = {}

    # Split by semicolon for each constraint group
    parts = [p.strip() for p in constraint_str.split(";") if p.strip()]

    for part in parts:
        if ":" not in part:
            continue

        drones_part, condition_part = part.split(":", 1)
        drones_part = drones_part.strip()
        condition_part = condition_part.strip().lower()

        # Parse drones (e.g., "D1,D2" or "1,2")
        drone_ids = []
        for d in drones_part.split(","):
            d = d.strip().upper()
            if d.startswith("D"):
                drone_ids.append(d[1:])  # "D1" -> "1"
            else:
                drone_ids.append(d)

        # Parse priority condition (e.g., "priority>=6", "priority<6")
        # Match patterns like: priority>=6, priority>6, priority<=6, priority<6, priority=6
        match = re.search(r'priority\s*(>=|<=|>|<|=)\s*(\d+)', condition_part)
        if not match:
            continue

        op = match.group(1)
        threshold = int(match.group(2))

        # Create filter function based on operator
        if op == ">=":
            filter_fn = lambda p, t=threshold: p >= t
        elif op == ">":
            filter_fn = lambda p, t=threshold: p > t
        elif op == "<=":
            filter_fn = lambda p, t=threshold: p <= t
        elif op == "<":
            filter_fn = lambda p, t=threshold: p < t
        else:  # "="
            filter_fn = lambda p, t=threshold: p == t

        for did in drone_ids:
            drone_filters[did] = filter_fn

    return drone_filters


def target_allowed_for_drone(
    target: Dict[str, Any],
    drone_id: str,
    drone_config: Dict[str, Any],
    priority_filters: Optional[Dict[str, Callable[[int], bool]]] = None
) -> bool:
    """
    Check if a target is allowed for a specific drone based on ALL constraints.

    Args:
        target: Target dict with 'type' and 'priority' keys
        drone_id: Drone identifier (e.g., "1")
        drone_config: Drone configuration with 'target_access' and 'enabled'
        priority_filters: Optional dict of drone_id -> priority filter function

    Returns:
        True if target is allowed, False otherwise
    """
    # Check if drone is enabled
    if drone_config.get("enabled") is False:
        return False

    # Check target type accessibility
    target_type = str(target.get("type", "a")).lower()
    target_access = drone_config.get("target_access", {})

    if target_access:
        allowed_types = {t.lower() for t, enabled in target_access.items() if enabled}
    else:
        allowed_types = {"a", "b", "c", "d", "e"}

    if target_type not in allowed_types:
        return False

    # Check priority constraint
    if priority_filters and drone_id in priority_filters:
        priority = int(target.get("priority", 5))
        filter_fn = priority_filters[drone_id]
        if not filter_fn(priority):
            return False

    return True


class PostOptimizer:
    """
    Post-optimization for mission solutions.

    Handles unvisited targets by finding drones with spare fuel capacity
    and inserting targets into their routes optimally.
    """

    def __init__(self):
        self._distance_matrix: Optional[Dict[str, Any]] = None

    def set_distance_matrix(self, matrix_data: Dict[str, Any]):
        """Set the pre-calculated distance matrix for optimization decisions."""
        self._distance_matrix = matrix_data

    def optimize(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_constraints: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Post-optimize a mission solution to include unvisited targets.

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports, sams
            drone_configs: Drone configurations with fuel budgets
            priority_constraints: Optional priority constraints string
                                  e.g., "D1,D2: priority>=6; D3,D4: priority<6"

        Returns:
            Optimized solution with updated routes
        """
        # Parse priority constraints into per-drone filter functions
        priority_filters = parse_priority_constraint(priority_constraints) if priority_constraints else {}
        targets = env.get("targets", [])
        airports = env.get("airports", [])

        # Get all target IDs
        all_target_ids = {str(t["id"]) for t in targets}

        # Find visited targets from all routes
        visited_targets: Set[str] = set()
        for did, route_data in solution.get("routes", {}).items():
            route = route_data.get("route", [])
            for wp_id in route:
                if str(wp_id).startswith("T"):
                    visited_targets.add(str(wp_id))

        # Identify unvisited targets
        unvisited = all_target_ids - visited_targets

        if not unvisited:
            # All targets visited, no optimization needed
            return solution

        # Build target lookup
        target_by_id = {str(t["id"]): t for t in targets}

        # Sort unvisited targets by priority (highest first)
        unvisited_sorted = sorted(
            unvisited,
            key=lambda tid: int(target_by_id[tid].get("priority", 5)),
            reverse=True
        )

        # Calculate remaining fuel for each drone
        drone_remaining_fuel: Dict[str, float] = {}
        for did, route_data in solution.get("routes", {}).items():
            fuel_budget = route_data.get("fuel_budget", 0)
            distance_used = route_data.get("distance", 0)
            drone_remaining_fuel[did] = fuel_budget - distance_used

        # Try to assign each unvisited target
        # GOAL: Maximize total points by inserting ALL targets that fit within fuel budget
        # STRATEGY:
        #   1. Process targets by priority (highest first) - maximizes points
        #   2. For each target, find ALL drones that can take it (capability + fuel check)
        #   3. Among viable drones, pick the one with LOWEST insertion cost - minimizes fuel
        optimized_routes = dict(solution.get("routes", {}))

        for tid in unvisited_sorted:
            target = target_by_id[tid]
            priority = int(target.get("priority", 5))

            # Find ALL viable (drone, position, cost) options for this target
            viable_options = []  # List of (drone_id, insertion_index, insertion_cost)

            for did, cfg in drone_configs.items():
                # Check ALL constraints: enabled, target type, AND priority
                if not target_allowed_for_drone(target, did, cfg, priority_filters):
                    continue

                # Check if drone has enough fuel
                remaining_fuel = drone_remaining_fuel.get(did, 0)
                if remaining_fuel <= 0:
                    continue

                # Find best insertion point in this drone's route
                route_data = optimized_routes.get(did, {})
                route = route_data.get("route", [])

                if not route:
                    continue

                # Get frozen segments for this drone (indices where route[i]->route[i+1] is frozen)
                frozen_segments = set(route_data.get("frozen_segments", []))

                # Find the cheapest insertion position for THIS drone
                best_insertion_idx = None
                best_insertion_cost = float('inf')

                # Try inserting at each position
                for i in range(1, len(route)):  # Don't insert before start
                    # Skip if this segment is frozen (cannot insert into frozen segment)
                    if (i - 1) in frozen_segments:
                        continue

                    prev_wp = route[i - 1]
                    next_wp = route[i]

                    # CRITICAL: Only insert between targets or before end airport
                    # Skip positions where next_wp is an airport (prevents inserting after end airport)
                    if str(next_wp).startswith("A"):
                        continue

                    # Calculate insertion cost
                    # Cost = (new_distance) - (original_distance)
                    orig_dist = self._get_distance(prev_wp, next_wp)
                    new_dist = (
                        self._get_distance(prev_wp, tid) +
                        self._get_distance(tid, next_wp)
                    )
                    insertion_cost = new_dist - orig_dist

                    # Check if we have enough fuel for this insertion
                    if insertion_cost > remaining_fuel:
                        continue

                    # Track the cheapest insertion point for this drone
                    if insertion_cost < best_insertion_cost:
                        best_insertion_cost = insertion_cost
                        best_insertion_idx = i

                # If we found a viable insertion for this drone, add to options
                if best_insertion_idx is not None:
                    viable_options.append((did, best_insertion_idx, best_insertion_cost))

            # If no drone can take this target, skip it
            if not viable_options:
                continue

            # Among viable options, pick the drone with LOWEST insertion cost
            # This minimizes fuel usage when multiple drones can service the target
            best_drone, best_insertion, best_cost = min(viable_options, key=lambda x: x[2])

            # Apply the insertion
            route = list(optimized_routes[best_drone].get("route", []))
            route.insert(best_insertion, tid)

            # Recalculate route metrics
            new_distance = self._calculate_route_distance(route)
            new_points = self._calculate_route_points(route, target_by_id)

            optimized_routes[best_drone] = {
                "route": route,
                "sequence": ",".join(route),
                "points": new_points,
                "distance": new_distance,
                "fuel_budget": optimized_routes[best_drone].get("fuel_budget", 0)
            }

            # Update remaining fuel for this drone
            drone_remaining_fuel[best_drone] -= best_cost

        # Rebuild solution
        optimized_solution = {
            "sequences": {
                did: data.get("sequence", "")
                for did, data in optimized_routes.items()
            },
            "routes": optimized_routes
        }

        return optimized_solution

    def find_unvisited_targets(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any]
    ) -> List[Dict[str, Any]]:
        """
        Find all targets that are not in any drone's route.

        Returns:
            List of unvisited target dicts with full details
        """
        targets = env.get("targets", [])
        all_target_ids = {str(t["id"]) for t in targets}

        visited: Set[str] = set()
        for did, route_data in solution.get("routes", {}).items():
            for wp_id in route_data.get("route", []):
                if str(wp_id).startswith("T"):
                    visited.add(str(wp_id))

        unvisited_ids = all_target_ids - visited
        target_by_id = {str(t["id"]): t for t in targets}

        return [target_by_id[tid] for tid in unvisited_ids if tid in target_by_id]

    def calculate_coverage_stats(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Calculate coverage statistics for a solution.

        Returns:
            Dict with coverage metrics
        """
        targets = env.get("targets", [])
        target_by_id = {str(t["id"]): t for t in targets}

        visited: Set[str] = set()
        total_points = 0
        total_distance = 0

        for did, route_data in solution.get("routes", {}).items():
            for wp_id in route_data.get("route", []):
                if str(wp_id).startswith("T"):
                    visited.add(str(wp_id))
                    target = target_by_id.get(str(wp_id))
                    if target:
                        total_points += int(target.get("priority", 5))
            total_distance += route_data.get("distance", 0)

        max_possible_points = sum(
            int(t.get("priority", 5)) for t in targets
        )

        return {
            "targets_visited": len(visited),
            "targets_total": len(targets),
            "coverage_percent": (len(visited) / len(targets) * 100) if targets else 0,
            "points_collected": total_points,
            "points_possible": max_possible_points,
            "points_percent": (total_points / max_possible_points * 100) if max_possible_points else 0,
            "total_distance": total_distance,
            "unvisited_targets": list({str(t["id"]) for t in targets} - visited)
        }

    def _get_distance(self, from_id: str, to_id: str) -> float:
        """Get distance between two waypoints from matrix."""
        if self._distance_matrix:
            labels = self._distance_matrix.get("labels", [])
            matrix = self._distance_matrix.get("matrix", [])

            try:
                i = labels.index(from_id)
                j = labels.index(to_id)
                return matrix[i][j]
            except (ValueError, IndexError):
                pass

        # Default fallback - should not happen if matrix is set
        return 10.0

    def _calculate_route_distance(self, route: List[str]) -> float:
        """Calculate total distance of a route."""
        if len(route) < 2:
            return 0.0

        total = 0.0
        for i in range(len(route) - 1):
            total += self._get_distance(route[i], route[i + 1])

        return total

    def _calculate_route_points(
        self,
        route: List[str],
        target_by_id: Dict[str, Dict[str, Any]]
    ) -> int:
        """Calculate total priority points for a route."""
        points = 0
        for wp_id in route:
            if str(wp_id).startswith("T"):
                target = target_by_id.get(str(wp_id))
                if target:
                    points += int(target.get("priority", 5))
        return points


# Global optimizer instance
_optimizer = PostOptimizer()


def post_optimize_solution(
    solution: Dict[str, Any],
    env: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
    distance_matrix: Optional[Dict[str, Any]] = None,
    priority_constraints: Optional[str] = None
) -> Dict[str, Any]:
    """
    Post-optimize a solution to include unvisited targets.

    This is the main entry point for use as a LangGraph tool.

    Args:
        solution: Current solution with drone routes
        env: Environment data
        drone_configs: Drone configurations
        distance_matrix: Pre-calculated SAM-aware distance matrix
        priority_constraints: Optional priority constraints string
                              e.g., "D1,D2: priority>=6; D3,D4: priority<6"

    Returns:
        Optimized solution with updated routes
    """
    if distance_matrix:
        _optimizer.set_distance_matrix(distance_matrix)

    return _optimizer.optimize(solution, env, drone_configs, priority_constraints)


def get_unvisited_targets(
    solution: Dict[str, Any],
    env: Dict[str, Any]
) -> List[Dict[str, Any]]:
    """Get list of unvisited targets from a solution."""
    return _optimizer.find_unvisited_targets(solution, env)


def get_coverage_stats(
    solution: Dict[str, Any],
    env: Dict[str, Any]
) -> Dict[str, Any]:
    """Get coverage statistics for a solution."""
    return _optimizer.calculate_coverage_stats(solution, env)


def set_optimizer_matrix(matrix_data: Dict[str, Any]):
    """Set the distance matrix for the global optimizer."""
    _optimizer.set_distance_matrix(matrix_data)


# Tool definition for LangGraph
POST_OPTIMIZER_TOOL = {
    "name": "post_optimize_solution",
    "description": """Post-optimize a mission solution by assigning unvisited targets
    to drones with remaining fuel capacity. Inserts targets at optimal positions
    in existing routes to minimize additional fuel consumption while maximizing
    coverage.""",
    "parameters": {
        "type": "object",
        "properties": {
            "solution": {
                "type": "object",
                "description": "Current solution with drone routes"
            },
            "env": {
                "type": "object",
                "description": "Environment data with targets, airports, sams"
            },
            "drone_configs": {
                "type": "object",
                "description": "Drone configurations with fuel budgets"
            },
            "distance_matrix": {
                "type": "object",
                "description": "Pre-calculated SAM-aware distance matrix (optional)"
            }
        },
        "required": ["solution", "env", "drone_configs"]
    },
    "function": post_optimize_solution
}


COVERAGE_STATS_TOOL = {
    "name": "get_coverage_stats",
    "description": """Get coverage statistics for a mission solution including
    targets visited, points collected, and list of unvisited targets.""",
    "parameters": {
        "type": "object",
        "properties": {
            "solution": {
                "type": "object",
                "description": "Solution with drone routes"
            },
            "env": {
                "type": "object",
                "description": "Environment data with targets"
            }
        },
        "required": ["solution", "env"]
    },
    "function": get_coverage_stats
}


class TrajectorySwapOptimizer:
    """
    Reassign targets to drones whose trajectories pass closer to them.

    For each target, calculate its perpendicular distance to the line segment
    connecting its neighbors in the current route. If another drone's trajectory
    passes closer AND that drone can carry the target, move it.
    """

    def __init__(self):
        self._distance_matrix: Optional[Dict[str, Any]] = None

    def set_distance_matrix(self, matrix_data: Dict[str, Any]):
        """Set the pre-calculated distance matrix."""
        self._distance_matrix = matrix_data

    def optimize(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_constraints: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Reassign targets to drones with closer trajectories using SAM-avoiding trajectory segments.

        Algorithm (Swap Closer):
        1. For each target in a trajectory:
           - Calculate SSD (Self Segment Distance) from trajectory vertices
           - If SSD = 0: SKIP (NO SSD NO MOVEMENT rule)
           - If SSD > 0: Search for better segments within radius SSD
        2. Find all trajectory segments (OS) within circle of radius SSD
        3. For each OS where OSD < SSD:
           - Check if segment can accept target (capability, fuel, not frozen)
        4. Move target to segment with minimum OSD (if any valid ones exist)

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports, sams
            drone_configs: Drone configurations with capabilities
            priority_constraints: Optional priority constraints string

        Returns:
            Optimized solution with targets moved to closer trajectories
        """
        # Parse priority constraints into per-drone filter functions
        priority_filters = parse_priority_constraint(priority_constraints) if priority_constraints else {}

        targets = env.get("targets", [])
        target_by_id = {str(t["id"]): t for t in targets}

        # Build position lookup for all waypoints
        waypoint_positions = {}
        for t in targets:
            waypoint_positions[str(t["id"])] = (float(t["x"]), float(t["y"]))
        for a in env.get("airports", []):
            waypoint_positions[str(a["id"])] = (float(a["x"]), float(a["y"]))

        # Copy routes for modification, including trajectories and frozen segments
        optimized_routes = {}
        drone_trajectories = {}  # Store the actual SAM-avoiding trajectories
        target_to_traj_index = {}  # Map target_id to (drone_id, traj_vertex_index)

        for did, route_data in solution.get("routes", {}).items():
            optimized_routes[did] = {
                "route": list(route_data.get("route", [])),
                "sequence": route_data.get("sequence", ""),
                "points": route_data.get("points", 0),
                "distance": route_data.get("distance", 0),
                "fuel_budget": route_data.get("fuel_budget", 0),
                "frozen_segments": set(route_data.get("frozen_segments", [])),
            }
            # Extract trajectory as list of (x, y) tuples
            traj = route_data.get("trajectory", [])
            if traj:
                drone_trajectories[did] = [(float(p[0]), float(p[1])) for p in traj]

                # Build mapping from targets to trajectory vertex indices
                route = route_data.get("route", [])
                traj_idx = 0
                for wp_id in route:
                    if str(wp_id).startswith("T"):
                        # Find this target's position in trajectory
                        wp_pos = waypoint_positions.get(str(wp_id))
                        if wp_pos:
                            # Scan trajectory to find matching vertex
                            for i in range(traj_idx, len(drone_trajectories[did])):
                                if abs(drone_trajectories[did][i][0] - wp_pos[0]) < 0.001 and \
                                   abs(drone_trajectories[did][i][1] - wp_pos[1]) < 0.001:
                                    target_to_traj_index[str(wp_id)] = (did, i)
                                    traj_idx = i + 1
                                    break
            else:
                drone_trajectories[did] = []

        # Store drone_configs and priority_filters for constraint checking
        self._drone_configs = drone_configs
        self._priority_filters = priority_filters

        num_passes = 4  # Do 4 full passes through all targets
        all_swaps = []  # Track all swaps made

        print(f"\n🔧 [TrajectorySwapOptimizer] Starting {num_passes} passes...")

        for pass_num in range(num_passes):
            print(f"\n  📍 Pass {pass_num + 1}/{num_passes}")

            # Collect all targets to check in this pass
            targets_to_check = []
            for current_drone, route_data in optimized_routes.items():
                route = route_data["route"]
                for wp_id in route:
                    if str(wp_id).startswith("T"):
                        targets_to_check.append(wp_id)

            # Check each target once in this pass
            for wp_id in targets_to_check:
                # Find current location of this target
                current_drone = None
                current_idx = None
                for did, route_data in optimized_routes.items():
                    route = route_data["route"]
                    if wp_id in route:
                        current_drone = did
                        current_idx = route.index(wp_id)
                        break

                if current_drone is None:
                    continue  # Target no longer in any route

                route = optimized_routes[current_drone]["route"]
                frozen_segments = optimized_routes[current_drone]["frozen_segments"]

                # Skip if target is in a frozen segment
                if current_idx > 0 and (current_idx - 1) in frozen_segments:
                    continue
                if current_idx < len(route) - 1 and current_idx in frozen_segments:
                    continue

                target = target_by_id.get(str(wp_id))
                if not target:
                    continue

                target_pos = waypoint_positions.get(str(wp_id))
                if not target_pos:
                    continue

                # Get prev and next waypoints in route (not trajectory)
                # This is simpler and works for most cases
                if current_idx == 0 or current_idx == len(route) - 1:
                    continue  # Target at start or end, cannot have self-segment

                prev_wp = route[current_idx - 1]
                next_wp = route[current_idx + 1]

                prev_pos = waypoint_positions.get(str(prev_wp))
                next_pos = waypoint_positions.get(str(next_wp))

                if not prev_pos or not next_pos:
                    continue

                # Calculate SSD: Self Segment Distance
                ssd = self._point_to_line_distance(target_pos, prev_pos, next_pos)

                # Only log targets with significant SSD (skip trivial cases)
                if ssd > 5.0:
                    print(f"    🎯 {wp_id}: SSD={ssd:.2f} (route segment {prev_wp}→{next_wp})")

                # CRITICAL: NO SSD NO MOVEMENT
                if ssd == 0.0:
                    print(f"       ⏭️  SKIP: SSD=0, no movement allowed")
                    continue

                # Search for segments within circle of radius SSD
                best_drone = None
                best_route_segment = None
                best_osd = float('inf')
                best_insertion_cost = float('inf')

                # Check all route segments in all drones
                for other_drone, other_route_data in optimized_routes.items():
                    other_route = other_route_data["route"]
                    other_frozen = other_route_data["frozen_segments"]

                    # Check drone capability
                    if other_drone != current_drone:
                        other_cfg = drone_configs.get(other_drone, {})
                        if not target_allowed_for_drone(target, other_drone, other_cfg, priority_filters):
                            continue

                    # Check each segment in route
                    for j in range(len(other_route) - 1):
                        if j in other_frozen:
                            continue

                        # Skip current target's adjacent segments
                        # Don't allow inserting into the segment immediately before or after the target
                        if other_drone == current_drone:
                            if j == current_idx - 1:  # Segment: prev→target
                                continue
                            if j == current_idx:  # Segment: target→next
                                continue

                        seg_start_id = str(other_route[j])
                        seg_end_id = str(other_route[j + 1])

                        # Skip segments ending at airports
                        if str(seg_end_id).startswith("A"):
                            continue

                        # Get positions of route waypoints
                        seg_start_pos = waypoint_positions.get(seg_start_id)
                        seg_end_pos = waypoint_positions.get(seg_end_id)

                        if not seg_start_pos or not seg_end_pos:
                            continue

                        # Quick distance check: is segment within circle of radius SSD?
                        # Use bounding box approximation for speed
                        min_x = min(seg_start_pos[0], seg_end_pos[0]) - ssd
                        max_x = max(seg_start_pos[0], seg_end_pos[0]) + ssd
                        min_y = min(seg_start_pos[1], seg_end_pos[1]) - ssd
                        max_y = max(seg_start_pos[1], seg_end_pos[1]) + ssd

                        if not (min_x <= target_pos[0] <= max_x and min_y <= target_pos[1] <= max_y):
                            continue  # Segment bounding box too far away

                        # Calculate OSD: Other Segment Distance
                        osd = self._point_to_line_distance(target_pos, seg_start_pos, seg_end_pos)

                        # Debug: only show cross-drone candidates (skip same-drone for brevity)
                        if osd < ssd and other_drone != current_drone:
                            print(f"         📐 D{other_drone} segment {seg_start_id}→{seg_end_id}: OSD={osd:.2f} < SSD={ssd:.2f}")

                        # CRITICAL: Only move if OSD < SSD (improvement required)
                        if osd >= ssd:
                            continue  # Not an improvement

                        # Calculate insertion cost
                        d_start = self._get_matrix_distance(seg_start_id, str(wp_id))
                        d_end = self._get_matrix_distance(str(wp_id), seg_end_id)
                        d_direct = self._get_matrix_distance(seg_start_id, seg_end_id)
                        insertion_cost = d_start + d_end - d_direct

                        # Check fuel budget
                        if other_drone != current_drone:
                            other_fuel_budget = other_route_data.get("fuel_budget", 0)
                            if other_fuel_budget <= 0:
                                other_cfg = drone_configs.get(other_drone, {})
                                other_fuel_budget = other_cfg.get("fuel_budget", 200)

                            other_current_distance = self._calculate_route_distance(other_route)
                            if other_current_distance + insertion_cost > other_fuel_budget:
                                print(f"            ⛽ FUEL: D{other_drone} {seg_start_id}→{seg_end_id} exceeds budget ({other_current_distance:.1f}+{insertion_cost:.1f} > {other_fuel_budget:.1f})")
                                continue

                        # Calculate net savings for this swap
                        # For same-drone swaps: no insertion cost to other drone
                        # For cross-drone swaps: must account for insertion cost
                        if other_drone == current_drone:
                            # Same drone: just reordering within route
                            net_savings = ssd - osd  # Pure geometric improvement
                        else:
                            # Cross-drone swap: geometric improvement minus insertion cost
                            # We remove from current drone (saves distance) and add to other drone (costs distance)
                            # Net benefit = (what we save) - (what we add)
                            # Since we're using perpendicular distances as proxy for route improvement,
                            # the savings is approximately (ssd - osd)
                            # But we need to ensure the other drone can afford the insertion_cost
                            net_savings = ssd - osd  # Geometric improvement is the main factor

                        # Track best option (maximum net savings)
                        # Use net_savings as primary criterion, OSD as tiebreaker
                        if net_savings > (ssd - best_osd):  # Better savings than current best
                            print(f"            🔄 New best: D{other_drone} {seg_start_id}→{seg_end_id}, savings={net_savings:.2f} (was {ssd - best_osd:.2f})")
                            best_osd = osd
                            best_insertion_cost = insertion_cost
                            best_drone = other_drone
                            best_route_segment = j + 1  # Insert after seg_start

                # If found a better segment, swap
                if best_drone and best_osd < ssd:
                    print(f"       ✅ SWAP: {wp_id} from D{current_drone} to D{best_drone}, SSD={ssd:.2f}→OSD={best_osd:.2f}")

                    # Record the swap
                    all_swaps.append({
                        "target": wp_id,
                        "from_drone": current_drone,
                        "to_drone": best_drone,
                        "ssd": ssd,
                        "osd": best_osd,
                        "savings": ssd - best_osd,
                        "same_drone": (best_drone == current_drone),
                        "pass": pass_num + 1
                    })

                    # Remove from current route
                    optimized_routes[current_drone]["route"].remove(wp_id)

                    # Insert into new route
                    optimized_routes[best_drone]["route"].insert(best_route_segment, wp_id)

        # Recalculate metrics for all routes
        for did, route_data in optimized_routes.items():
            route = route_data["route"]
            route_data["sequence"] = ",".join(route)
            route_data["distance"] = self._calculate_route_distance(route)
            route_data["points"] = self._calculate_route_points(route, target_by_id)

        return {
            "sequences": {
                did: data["sequence"] for did, data in optimized_routes.items()
            },
            "routes": optimized_routes,
            "swaps_made": all_swaps,
            "passes": num_passes
        }

    def _point_to_line_distance(
        self,
        point: Tuple[float, float],
        line_start: Tuple[float, float],
        line_end: Tuple[float, float]
    ) -> float:
        """
        Calculate perpendicular distance from a point to a line segment.
        """
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        # Line segment length squared
        line_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2

        if line_len_sq == 0:
            # Line segment is a point
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        # Project point onto line, clamped to segment
        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq))

        # Closest point on segment
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)

        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

    def _get_matrix_distance(self, from_id: str, to_id: str) -> float:
        """
        Get SAM-aware distance between two waypoints from the distance matrix.

        Args:
            from_id: Source waypoint ID (e.g., 'A1', 'T5')
            to_id: Destination waypoint ID

        Returns:
            Distance from the pre-calculated SAM-aware matrix, or large fallback
        """
        if not self._distance_matrix:
            return 1000.0  # Large fallback if no matrix

        labels = self._distance_matrix.get("labels", [])
        matrix = self._distance_matrix.get("matrix", [])

        try:
            idx_from = labels.index(from_id)
            idx_to = labels.index(to_id)
            return matrix[idx_from][idx_to]
        except (ValueError, IndexError):
            return 1000.0  # Large fallback for missing entries

    def _point_to_polyline_distance(
        self,
        point: Tuple[float, float],
        polyline: List[Tuple[float, float]]
    ) -> float:
        """
        Calculate minimum distance from a point to a polyline (multiple connected segments).

        This is used to measure distance to actual SAM-avoiding trajectories,
        which may have bends/waypoints to navigate around SAM zones.

        Args:
            point: The point (x, y) to measure from
            polyline: List of (x, y) points forming the trajectory

        Returns:
            Minimum perpendicular distance to any segment of the polyline
        """
        if not polyline:
            return float('inf')

        if len(polyline) == 1:
            # Single point, return distance to that point
            px, py = point
            qx, qy = polyline[0]
            return math.sqrt((px - qx) ** 2 + (py - qy) ** 2)

        min_dist = float('inf')

        # Check distance to each segment in the polyline
        for i in range(len(polyline) - 1):
            seg_start = polyline[i]
            seg_end = polyline[i + 1]
            dist = self._point_to_line_distance(point, seg_start, seg_end)
            if dist < min_dist:
                min_dist = dist

        return min_dist

    def _calculate_route_distance(self, route: List[str]) -> float:
        """Calculate total distance of a route."""
        if len(route) < 2 or not self._distance_matrix:
            return 0.0

        labels = self._distance_matrix.get("labels", [])
        matrix = self._distance_matrix.get("matrix", [])

        total = 0.0
        for i in range(len(route) - 1):
            try:
                idx_from = labels.index(route[i])
                idx_to = labels.index(route[i + 1])
                total += matrix[idx_from][idx_to]
            except (ValueError, IndexError):
                total += 10.0  # Fallback

        return total

    def _calculate_route_points(
        self,
        route: List[str],
        target_by_id: Dict[str, Dict[str, Any]]
    ) -> int:
        """Calculate total priority points for a route."""
        points = 0
        for wp_id in route:
            if str(wp_id).startswith("T"):
                target = target_by_id.get(str(wp_id))
                if target:
                    points += int(target.get("priority", 5))
        return points


# Global trajectory swap optimizer instance
_trajectory_optimizer = TrajectorySwapOptimizer()


def trajectory_swap_optimize(
    solution: Dict[str, Any],
    env: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
    distance_matrix: Optional[Dict[str, Any]] = None,
    priority_constraints: Optional[str] = None
) -> Dict[str, Any]:
    """
    Reassign targets to drones whose trajectories pass closer to them.

    For each target, if another drone's path passes closer AND that drone
    can carry the target, move it to the closer trajectory.

    Args:
        solution: Current solution with drone routes
        env: Environment data with targets, airports
        drone_configs: Drone configurations with capabilities
        distance_matrix: Pre-calculated distance matrix
        priority_constraints: Optional priority constraints string
                              e.g., "D1,D2: priority>=6; D3,D4: priority<6"

    Returns:
        Optimized solution with targets moved to closer trajectories
    """
    if distance_matrix:
        _trajectory_optimizer.set_distance_matrix(distance_matrix)

    return _trajectory_optimizer.optimize(solution, env, drone_configs, priority_constraints)


def set_trajectory_optimizer_matrix(matrix_data: Dict[str, Any]):
    """Set the distance matrix for the trajectory swap optimizer."""
    _trajectory_optimizer.set_distance_matrix(matrix_data)


TRAJECTORY_SWAP_TOOL = {
    "name": "trajectory_swap_optimize",
    "description": """Reassign targets to drones whose trajectories pass closer to them.
    For each target, calculates its distance to the line connecting its neighbors
    in the current route. If another capable drone's trajectory passes closer,
    the target is moved to that drone. This reduces total travel distance.""",
    "parameters": {
        "type": "object",
        "properties": {
            "solution": {
                "type": "object",
                "description": "Current solution with drone routes"
            },
            "env": {
                "type": "object",
                "description": "Environment data with targets, airports"
            },
            "drone_configs": {
                "type": "object",
                "description": "Drone configurations with capabilities"
            },
            "distance_matrix": {
                "type": "object",
                "description": "Pre-calculated distance matrix (optional)"
            }
        },
        "required": ["solution", "env", "drone_configs"]
    },
    "function": trajectory_swap_optimize
}


# ---------- Crossing Removal Optimizer (2-opt) ----------

class CrossingRemovalOptimizer:
    """
    Remove self-crossings in drone trajectories using 2-opt.

    When segment A→B crosses segment C→D, reverse the middle portion
    to eliminate the crossing: A → B → C → D becomes A → C → B → D
    """

    def __init__(self):
        self._distance_matrix: Optional[Dict[str, Any]] = None

    def set_distance_matrix(self, matrix_data: Dict[str, Any]):
        """Set the pre-calculated distance matrix."""
        self._distance_matrix = matrix_data

    def _segments_cross(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
        p4: Tuple[float, float]
    ) -> bool:
        """
        Check if segment p1-p2 crosses segment p3-p4.
        Uses cross product to determine intersection.
        """
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

        # Check if segments share an endpoint (not a crossing)
        if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
            return False

        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def optimize(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Remove crossings from all drone routes.

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports
            drone_configs: Drone configurations

        Returns:
            Optimized solution with crossings removed
        """
        targets = env.get("targets", [])
        target_by_id = {str(t["id"]): t for t in targets}

        # Build position lookup for all waypoints
        waypoint_positions = {}
        for t in targets:
            waypoint_positions[str(t["id"])] = (float(t["x"]), float(t["y"]))
        for a in env.get("airports", []):
            waypoint_positions[str(a["id"])] = (float(a["x"]), float(a["y"]))

        # Copy routes for modification, including frozen segments
        optimized_routes = {}
        for did, route_data in solution.get("routes", {}).items():
            optimized_routes[did] = {
                "route": list(route_data.get("route", [])),
                "sequence": route_data.get("sequence", ""),
                "points": route_data.get("points", 0),
                "distance": route_data.get("distance", 0),
                "fuel_budget": route_data.get("fuel_budget", 0),
                "frozen_segments": set(route_data.get("frozen_segments", [])),
            }

        all_fixes = []  # Track all crossings fixed
        num_passes = 4  # Do multiple passes

        for pass_num in range(num_passes):
            # Process each drone's route
            for did, route_data in optimized_routes.items():
                route = route_data["route"]
                frozen_segments = route_data["frozen_segments"]

                if len(route) < 4:
                    continue  # Need at least 4 points to have a crossing

                # Keep looking for crossings until none found
                found_crossing = True
                max_iterations = 50  # Safety limit per drone per pass
                iterations = 0

                while found_crossing and iterations < max_iterations:
                    found_crossing = False
                    iterations += 1

                    # Check all pairs of non-adjacent segments
                    for i in range(len(route) - 1):
                        if found_crossing:
                            break

                        for j in range(i + 2, len(route) - 1):
                            # Skip adjacent segments
                            if j == i + 1:
                                continue

                            # Check if any segment in the reversal range is frozen
                            # We reverse route[i+1:j+1], which affects segments from i to j
                            reversal_affects_frozen = False
                            for k in range(i, j + 1):
                                if k in frozen_segments:
                                    reversal_affects_frozen = True
                                    break

                            if reversal_affects_frozen:
                                continue  # Cannot reverse if it involves frozen segments

                            # Get segment endpoints
                            p1 = waypoint_positions.get(str(route[i]))
                            p2 = waypoint_positions.get(str(route[i + 1]))
                            p3 = waypoint_positions.get(str(route[j]))
                            p4 = waypoint_positions.get(str(route[j + 1]))

                            if not all([p1, p2, p3, p4]):
                                continue

                            if self._segments_cross(p1, p2, p3, p4):
                                # Found a crossing! Reverse the segment between i+1 and j
                                # Route: ... A(i) → B(i+1) → ... → C(j) → D(j+1) ...
                                # After:  ... A(i) → C(j) → ... → B(i+1) → D(j+1) ...

                                old_segment = route[i+1:j+1]
                                route[i+1:j+1] = old_segment[::-1]

                                all_fixes.append({
                                    "drone": did,
                                    "segment_i": i,
                                    "segment_j": j,
                                    "reversed": old_segment,
                                    "pass": pass_num + 1
                                })

                                found_crossing = True
                                break

        # Recalculate metrics for all routes
        for did, route_data in optimized_routes.items():
            route = route_data["route"]
            route_data["sequence"] = ",".join(str(wp) for wp in route)
            # Use SAM-aware distance matrix if available, otherwise fallback to Euclidean
            if self._distance_matrix:
                route_data["distance"] = self._calculate_route_distance_from_matrix(route)
            else:
                route_data["distance"] = self._calculate_route_distance_euclidean(route, waypoint_positions)
            route_data["points"] = self._calculate_route_points(route, target_by_id)

        return {
            "sequences": {
                did: data["sequence"] for did, data in optimized_routes.items()
            },
            "routes": optimized_routes,
            "fixes_made": all_fixes,
            "passes": num_passes
        }

    def _calculate_route_distance_from_matrix(self, route: List[str]) -> float:
        """
        Calculate total SAM-aware distance of a route using the distance matrix.

        This is the correct distance calculation that accounts for SAM zones.
        """
        if len(route) < 2 or not self._distance_matrix:
            return 0.0

        labels = self._distance_matrix.get("labels", [])
        matrix = self._distance_matrix.get("matrix", [])

        total = 0.0
        for i in range(len(route) - 1):
            try:
                idx_from = labels.index(str(route[i]))
                idx_to = labels.index(str(route[i + 1]))
                total += matrix[idx_from][idx_to]
            except (ValueError, IndexError):
                # Fallback if waypoint not in matrix
                total += 10.0

        return total

    def _calculate_route_distance_euclidean(
        self,
        route: List[str],
        waypoint_positions: Dict[str, Tuple[float, float]]
    ) -> float:
        """
        Calculate total Euclidean distance of a route (fallback when no matrix available).

        WARNING: This does NOT account for SAM zones. Only use as fallback.
        """
        if len(route) < 2:
            return 0.0

        total = 0.0
        for i in range(len(route) - 1):
            p1 = waypoint_positions.get(str(route[i]))
            p2 = waypoint_positions.get(str(route[i + 1]))
            if p1 and p2:
                total += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

        return total

    def _calculate_route_points(
        self,
        route: List[str],
        target_by_id: Dict[str, Dict[str, Any]]
    ) -> int:
        """Calculate total priority points for a route."""
        points = 0
        for wp_id in route:
            if str(wp_id).startswith("T"):
                target = target_by_id.get(str(wp_id))
                if target:
                    points += int(target.get("priority", 5))
        return points


# Singleton instance
_crossing_optimizer = CrossingRemovalOptimizer()


def crossing_removal_optimize(
    solution: Dict[str, Any],
    env: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
    distance_matrix: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Remove self-crossings from drone trajectories using 2-opt.

    Args:
        solution: Current solution with drone routes
        env: Environment data with targets, airports
        drone_configs: Drone configurations
        distance_matrix: Pre-calculated distance matrix (optional)

    Returns:
        Optimized solution with crossings removed
    """
    if distance_matrix:
        _crossing_optimizer.set_distance_matrix(distance_matrix)

    return _crossing_optimizer.optimize(solution, env, drone_configs)
