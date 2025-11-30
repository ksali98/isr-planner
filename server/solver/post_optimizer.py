"""
Post-Optimizer for Multi-Drone Mission Planning

After initial route optimization, this module:
1. Identifies unvisited targets
2. Attempts to assign them to drones with remaining fuel capacity
3. Re-optimizes affected drone routes

This is a LangGraph-compatible tool.
"""

import math
from typing import Dict, Any, List, Optional, Set, Tuple


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
        drone_configs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Post-optimize a mission solution to include unvisited targets.

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports, sams
            drone_configs: Drone configurations with fuel budgets

        Returns:
            Optimized solution with updated routes
        """
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
        optimized_routes = dict(solution.get("routes", {}))

        for tid in unvisited_sorted:
            target = target_by_id[tid]
            target_type = str(target.get("type", "a")).lower()
            priority = int(target.get("priority", 5))

            best_drone = None
            best_insertion = None
            best_cost = float('inf')

            for did, cfg in drone_configs.items():
                # Check if drone is enabled
                if cfg.get("enabled") is False:
                    continue

                # Check capability
                target_access = cfg.get("target_access", {})
                if target_access:
                    allowed_types = {
                        t.lower() for t, enabled in target_access.items()
                        if enabled
                    }
                else:
                    allowed_types = {"a", "b", "c", "d"}

                if target_type not in allowed_types:
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

                # Try inserting at each position
                for i in range(1, len(route)):  # Don't insert before start
                    prev_wp = route[i - 1]
                    next_wp = route[i]

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

                    # Prefer insertions that minimize cost while considering priority
                    # Use efficiency metric: priority / cost
                    if insertion_cost <= 0:
                        insertion_cost = 0.1  # Avoid division by zero

                    efficiency = priority / insertion_cost

                    if insertion_cost < best_cost:
                        best_cost = insertion_cost
                        best_drone = did
                        best_insertion = i

            # Apply the best insertion if found
            if best_drone and best_insertion is not None:
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

                # Update remaining fuel
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
    distance_matrix: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """
    Post-optimize a solution to include unvisited targets.

    This is the main entry point for use as a LangGraph tool.

    Args:
        solution: Current solution with drone routes
        env: Environment data
        drone_configs: Drone configurations
        distance_matrix: Pre-calculated SAM-aware distance matrix

    Returns:
        Optimized solution with updated routes
    """
    if distance_matrix:
        _optimizer.set_distance_matrix(distance_matrix)

    return _optimizer.optimize(solution, env, drone_configs)


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
        drone_configs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Reassign targets to drones with closer trajectories.

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports, sams
            drone_configs: Drone configurations with capabilities

        Returns:
            Optimized solution with targets moved to closer trajectories
        """
        targets = env.get("targets", [])
        target_by_id = {str(t["id"]): t for t in targets}

        # Build position lookup for all waypoints
        waypoint_positions = {}
        for t in targets:
            waypoint_positions[str(t["id"])] = (float(t["x"]), float(t["y"]))
        for a in env.get("airports", []):
            waypoint_positions[str(a["id"])] = (float(a["x"]), float(a["y"]))

        # Copy routes for modification, including trajectories
        optimized_routes = {}
        drone_trajectories = {}  # Store the actual SAM-avoiding trajectories
        for did, route_data in solution.get("routes", {}).items():
            optimized_routes[did] = {
                "route": list(route_data.get("route", [])),
                "sequence": route_data.get("sequence", ""),
                "points": route_data.get("points", 0),
                "distance": route_data.get("distance", 0),
                "fuel_budget": route_data.get("fuel_budget", 0),
            }
            # Extract trajectory as list of (x, y) tuples
            traj = route_data.get("trajectory", [])
            if traj:
                drone_trajectories[did] = [(float(p[0]), float(p[1])) for p in traj]
            else:
                drone_trajectories[did] = []

        # Build capability map: which target types each drone can carry
        drone_capabilities = {}
        for did, cfg in drone_configs.items():
            if cfg.get("enabled") is False:
                drone_capabilities[did] = set()
                continue
            target_access = cfg.get("target_access", {})
            if target_access:
                drone_capabilities[did] = {
                    t.lower() for t, enabled in target_access.items() if enabled
                }
            else:
                drone_capabilities[did] = {"a", "b", "c", "d"}

        num_passes = 4  # Do 4 full passes through all targets
        all_swaps = []  # Track all swaps made

        for pass_num in range(num_passes):
            # Collect all targets to check in this pass
            # We need a snapshot because routes will change during the pass
            targets_to_check = []
            for current_drone, route_data in optimized_routes.items():
                route = route_data["route"]
                for i, wp_id in enumerate(route):
                    if str(wp_id).startswith("T"):
                        targets_to_check.append((current_drone, wp_id))

            # Check each target once in this pass
            for original_drone, wp_id in targets_to_check:
                # Find current location of this target (may have moved)
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

                target = target_by_id.get(str(wp_id))
                if not target:
                    continue

                target_pos = waypoint_positions.get(str(wp_id))
                if not target_pos:
                    continue

                target_type = str(target.get("type", "a")).lower()

                # Get prev and next waypoints in current route
                prev_wp = route[current_idx - 1] if current_idx > 0 else None
                next_wp = route[current_idx + 1] if current_idx < len(route) - 1 else None

                if not prev_wp or not next_wp:
                    continue

                prev_pos = waypoint_positions.get(str(prev_wp))
                next_pos = waypoint_positions.get(str(next_wp))

                if not prev_pos or not next_pos:
                    continue

                # Calculate distance from target to its current trajectory
                current_dist = self._point_to_line_distance(
                    target_pos, prev_pos, next_pos
                )

                # Check all other drones' trajectories
                best_drone = None
                best_segment = None
                best_dist = current_dist

                for other_drone, other_route_data in optimized_routes.items():
                    if other_drone == current_drone:
                        continue

                    # Check if other drone can carry this target type
                    if target_type not in drone_capabilities.get(other_drone, set()):
                        continue

                    other_route = other_route_data["route"]

                    # Use actual trajectory (SAM-avoiding polyline) if available
                    other_trajectory = drone_trajectories.get(other_drone, [])

                    if other_trajectory:
                        # Measure distance to the actual bent trajectory polyline
                        dist = self._point_to_polyline_distance(target_pos, other_trajectory)

                        if dist < best_dist:
                            # Find best insertion point based on route nodes
                            # Insert at segment whose endpoints are closest to target
                            best_insertion_idx = 1  # Default to after first waypoint
                            best_insertion_dist = float('inf')

                            for j in range(len(other_route) - 1):
                                seg_start_pos = waypoint_positions.get(str(other_route[j]))
                                seg_end_pos = waypoint_positions.get(str(other_route[j + 1]))

                                if seg_start_pos and seg_end_pos:
                                    seg_dist = self._point_to_line_distance(
                                        target_pos, seg_start_pos, seg_end_pos
                                    )
                                    if seg_dist < best_insertion_dist:
                                        best_insertion_dist = seg_dist
                                        best_insertion_idx = j + 1

                            best_dist = dist
                            best_drone = other_drone
                            best_segment = best_insertion_idx
                    else:
                        # Fallback: no trajectory data, use straight-line route segments
                        for j in range(len(other_route) - 1):
                            seg_start = other_route[j]
                            seg_end = other_route[j + 1]

                            seg_start_pos = waypoint_positions.get(str(seg_start))
                            seg_end_pos = waypoint_positions.get(str(seg_end))

                            if not seg_start_pos or not seg_end_pos:
                                continue

                            dist = self._point_to_line_distance(
                                target_pos, seg_start_pos, seg_end_pos
                            )

                            if dist < best_dist:
                                best_dist = dist
                                best_drone = other_drone
                                best_segment = j + 1  # Insert after seg_start

                # If found a closer trajectory, swap
                if best_drone and best_dist < current_dist:
                    # Record the swap
                    all_swaps.append({
                        "target": wp_id,
                        "from_drone": current_drone,
                        "to_drone": best_drone,
                        "old_distance": current_dist,
                        "new_distance": best_dist,
                        "pass": pass_num + 1
                    })

                    # Remove from current route
                    optimized_routes[current_drone]["route"].remove(wp_id)

                    # Insert into new route
                    optimized_routes[best_drone]["route"].insert(best_segment, wp_id)

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
    distance_matrix: Optional[Dict[str, Any]] = None
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

    Returns:
        Optimized solution with targets moved to closer trajectories
    """
    if distance_matrix:
        _trajectory_optimizer.set_distance_matrix(distance_matrix)

    return _trajectory_optimizer.optimize(solution, env, drone_configs)


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

        # Copy routes for modification
        optimized_routes = {}
        for did, route_data in solution.get("routes", {}).items():
            optimized_routes[did] = {
                "route": list(route_data.get("route", [])),
                "sequence": route_data.get("sequence", ""),
                "points": route_data.get("points", 0),
                "distance": route_data.get("distance", 0),
                "fuel_budget": route_data.get("fuel_budget", 0),
            }

        all_fixes = []  # Track all crossings fixed
        num_passes = 4  # Do multiple passes

        for pass_num in range(num_passes):
            # Process each drone's route
            for did, route_data in optimized_routes.items():
                route = route_data["route"]

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
            route_data["distance"] = self._calculate_route_distance(route, waypoint_positions)
            route_data["points"] = self._calculate_route_points(route, target_by_id)

        return {
            "sequences": {
                did: data["sequence"] for did, data in optimized_routes.items()
            },
            "routes": optimized_routes,
            "fixes_made": all_fixes,
            "passes": num_passes
        }

    def _calculate_route_distance(
        self,
        route: List[str],
        waypoint_positions: Dict[str, Tuple[float, float]]
    ) -> float:
        """Calculate total Euclidean distance of a route."""
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
