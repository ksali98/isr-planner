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
import sys
from pathlib import Path
from typing import Dict, Any, List, Optional, Set, Tuple, Callable

# Add paths for SAM navigation
root_path = Path(__file__).resolve().parents[2]
if str(root_path) not in sys.path:
    sys.path.insert(0, str(root_path))
if "/app" not in sys.path:
    sys.path.insert(0, "/app")

# Import SAM-aware path planning
try:
    from path_planning_core.boundary_navigation import plan_path as boundary_plan_path
    from path_planning_core.sam_wrapping import wrap_sams
    HAS_SAM_NAVIGATION = True
except ImportError:
    boundary_plan_path = None
    wrap_sams = None
    HAS_SAM_NAVIGATION = False


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
        self._env: Optional[Dict[str, Any]] = None
        self._waypoint_positions: Optional[Dict[str, List[float]]] = None
        self._sams: Optional[List[Dict[str, Any]]] = None
        self._wrapped_polygons: Optional[List[Any]] = None

    def set_distance_matrix(self, matrix_data: Dict[str, Any]):
        """Set the pre-calculated distance matrix for optimization decisions."""
        self._distance_matrix = matrix_data

    def set_environment(self, env: Dict[str, Any]):
        """
        Set environment data for SAM-aware distance fallback calculations.

        Args:
            env: Environment dict with airports, targets, and sams
        """
        self._env = env

        # Build waypoint position lookup
        waypoint_positions = {}
        airports = env.get("airports", [])
        targets = env.get("targets", [])

        print(f"   [set_environment] Building waypoint positions from {len(airports)} airports and {len(targets)} targets", flush=True)

        for a in airports:
            waypoint_positions[str(a["id"])] = [float(a["x"]), float(a["y"])]
        for t in targets:
            waypoint_positions[str(t["id"])] = [float(t["x"]), float(t["y"])]
        self._waypoint_positions = waypoint_positions

        print(f"   [set_environment] Waypoint positions: {list(waypoint_positions.keys())}", flush=True)

        # Store SAM data
        self._sams = env.get("sams", [])

        # Generate wrapped polygons for SAM-aware distance calculations
        # boundary_plan_path expects polygon format: List[List[List[float]]]
        if wrap_sams and self._sams:
            sams_for_wrapping = []
            for sam in self._sams:
                pos = sam.get("pos") or sam.get("position")
                if pos:
                    sams_for_wrapping.append({
                        'x': pos[0] if isinstance(pos, (list, tuple)) else pos,
                        'y': pos[1] if isinstance(pos, (list, tuple)) else pos,
                        'radius': float(sam.get("range", sam.get("radius", 15)))
                    })
            if sams_for_wrapping:
                wrapped_polygon_arrays, _ = wrap_sams(sams_for_wrapping)
                # Convert numpy arrays to lists of [x, y] points
                self._wrapped_polygons = []
                for poly in wrapped_polygon_arrays:
                    if hasattr(poly, 'tolist'):
                        self._wrapped_polygons.append(poly.tolist())
                    else:
                        self._wrapped_polygons.append(list(poly))
        else:
            self._wrapped_polygons = None

    def _is_inside_polygon(self, waypoint_id: str) -> bool:
        """
        Check if a waypoint is inside any SAM polygon (no-fly zone).

        Args:
            waypoint_id: Waypoint identifier (e.g., "T15", "A1")

        Returns:
            True if waypoint is inside any SAM polygon, False otherwise
        """
        if not self._waypoint_positions or not self._sams:
            return False

        if waypoint_id not in self._waypoint_positions:
            return False

        wx, wy = self._waypoint_positions[waypoint_id]

        # Check if waypoint is inside any SAM's circular range (polygon)
        for sam in self._sams:
            pos = sam.get("pos") or sam.get("position")
            if not pos:
                continue

            sam_x = pos[0] if isinstance(pos, (list, tuple)) else pos
            sam_y = pos[1] if isinstance(pos, (list, tuple)) else pos
            sam_range = float(sam.get("range", sam.get("radius", 15)))

            # Calculate distance from waypoint to SAM center
            dx = wx - sam_x
            dy = wy - sam_y
            distance = math.sqrt(dx * dx + dy * dy)

            # If waypoint is inside this SAM's range polygon, it's invalid
            if distance < sam_range:
                return True

        return False

    def _find_closest_trajectory_segment(
        self,
        target_pos: Tuple[float, float],
        trajectory: List[Tuple[float, float]],
        route: List[str],
        waypoint_positions: Dict[str, Tuple[float, float]]
    ) -> Tuple[int, bool, int]:
        """
        Find the closest segment in a trajectory to a target point.

        Args:
            target_pos: (x, y) position of the target to insert
            trajectory: List of (x, y) vertices in the SAM-avoiding trajectory
            route: Route sequence (e.g., ['A1', 'T9', 'T17', 'A2'])
            waypoint_positions: Mapping of waypoint IDs to (x, y) positions

        Returns:
            Tuple of (route_index, has_perpendicular, closest_vertex_offset) where:
            - route_index: Index in route where target should be inserted
            - has_perpendicular: True if perpendicular exists to closest segment
            - closest_vertex_offset: If no perpendicular, which vertex is closer (0=first, 1=second)
        """
        if not trajectory or len(trajectory) < 2:
            return (1, False, 0)

        tx, ty = target_pos
        min_dist = float('inf')
        best_route_idx = None
        best_has_perp = False
        best_closest_vertex = 0

        # Build mapping from trajectory indices to route waypoint positions
        traj_to_route_wp = {}  # Maps trajectory index to route waypoint index
        traj_idx = 0
        for route_wp_idx, wp_id in enumerate(route):
            wp_pos = waypoint_positions.get(str(wp_id))
            if wp_pos:
                # Find matching trajectory vertex
                for i in range(traj_idx, len(trajectory)):
                    if abs(trajectory[i][0] - wp_pos[0]) < 0.001 and \
                       abs(trajectory[i][1] - wp_pos[1]) < 0.001:
                        traj_to_route_wp[i] = route_wp_idx
                        traj_idx = i + 1
                        break

        # Iterate through trajectory segments
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]
            p2 = trajectory[i + 1]

            # Calculate point-to-segment distance
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            segment_length_sq = dx * dx + dy * dy

            if segment_length_sq == 0:
                # Degenerate segment
                dist = math.sqrt((tx - p1[0]) ** 2 + (ty - p1[1]) ** 2)
                has_perp = False
                t = 0
            else:
                # Calculate projection parameter t
                t = ((tx - p1[0]) * dx + (ty - p1[1]) * dy) / segment_length_sq
                t_clamped = max(0.0, min(1.0, t))

                # Find closest point on segment
                closest_x = p1[0] + t_clamped * dx
                closest_y = p1[1] + t_clamped * dy

                # Distance to closest point
                dist = math.sqrt((tx - closest_x) ** 2 + (ty - closest_y) ** 2)

                # Check if perpendicular exists (t strictly between 0 and 1)
                has_perp = (0 < t < 1)

            # Update if this is the closest segment
            if dist < min_dist:
                min_dist = dist

                # Find which route waypoints this trajectory segment is between
                # Look for the route waypoint at or after trajectory vertex i
                route_wp_start = None
                for traj_v_idx in range(i, -1, -1):
                    if traj_v_idx in traj_to_route_wp:
                        route_wp_start = traj_to_route_wp[traj_v_idx]
                        break

                if route_wp_start is None:
                    route_wp_start = 0

                # Insertion position is after route_wp_start
                best_route_idx = route_wp_start + 1
                best_has_perp = has_perp

                # If no perpendicular, determine which vertex is closer
                if not has_perp:
                    dist_to_p1 = math.sqrt((tx - p1[0]) ** 2 + (ty - p1[1]) ** 2)
                    dist_to_p2 = math.sqrt((tx - p2[0]) ** 2 + (ty - p2[1]) ** 2)
                    best_closest_vertex = 0 if dist_to_p1 < dist_to_p2 else 1

        if best_route_idx is None:
            best_route_idx = 1

        return (best_route_idx, best_has_perp, best_closest_vertex)

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
        print(f"\n📍 [OPTIMIZER.optimize] Starting optimization", flush=True)
        print(f"   _waypoint_positions has {len(self._waypoint_positions or {})} entries", flush=True)
        print(f"   _distance_matrix is {'SET' if self._distance_matrix else 'NOT SET'}", flush=True)

        # Parse priority constraints into per-drone filter functions
        priority_filters = parse_priority_constraint(priority_constraints) if priority_constraints else {}
        targets = env.get("targets", [])
        airports = env.get("airports", [])

        print(f"   env has {len(targets)} targets, {len(airports)} airports", flush=True)

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

        print(f"   all_target_ids = {sorted(all_target_ids)}", flush=True)
        print(f"   visited_targets = {sorted(visited_targets)}", flush=True)
        print(f"   unvisited = {sorted(unvisited)}", flush=True)

        if not unvisited:
            # All targets visited, no optimization needed
            print(f"   ⚡ EARLY RETURN: All targets already visited!", flush=True)
            return solution

        # Filter out targets inside SAM polygons (no-fly zones)
        # These targets cannot be visited safely and will cause trajectory failures
        targets_inside_polygons = set()
        for tid in unvisited:
            if self._is_inside_polygon(tid):
                targets_inside_polygons.add(tid)
                print(f"⚠️ Insert Missed: Skipping {tid} - inside SAM polygon (no-fly zone)", flush=True)

        unvisited = unvisited - targets_inside_polygons

        if not unvisited:
            # No valid unvisited targets to insert
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
        # Include ALL drones from drone_configs, not just those with existing routes
        drone_remaining_fuel: Dict[str, float] = {}
        for did, cfg in drone_configs.items():
            route_data = solution.get("routes", {}).get(did, {})
            if route_data:
                # Drone has an existing route - use its fuel budget and distance
                fuel_budget = route_data.get("fuel_budget", 0) or cfg.get("fuel_budget", 0)
                distance_used = route_data.get("distance", 0)
                drone_remaining_fuel[did] = fuel_budget - distance_used
            else:
                # Drone has no route yet - full fuel budget available
                fuel_budget = cfg.get("fuel_budget", 0)
                drone_remaining_fuel[did] = fuel_budget

        # DEBUG: Log state before insertion attempts
        print(f"\n🔍 INSERT MISSED OPTIMIZER DEBUG:", flush=True)
        print(f"   All targets: {sorted(all_target_ids)}", flush=True)
        print(f"   Visited: {sorted(visited_targets)}", flush=True)
        print(f"   Unvisited (sorted by priority): {unvisited_sorted}", flush=True)
        print(f"   Drone configs received: {list(drone_configs.keys())}", flush=True)
        for did in sorted(drone_configs.keys()):
            cfg = drone_configs[did]
            remaining = drone_remaining_fuel.get(did, 0)
            print(f"   D{did}: enabled={cfg.get('enabled')}, remaining_fuel={remaining:.1f}, target_access={cfg.get('target_access', {})}", flush=True)

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
            target_type = target.get("type", "a")

            # Find ALL viable (drone, position, cost) options for this target
            viable_options = []  # List of (drone_id, insertion_index, insertion_cost)
            skip_reasons = {}  # Track why each drone can't take this target

            for did, cfg in drone_configs.items():
                # Check ALL constraints: enabled, target type, AND priority
                if not target_allowed_for_drone(target, did, cfg, priority_filters):
                    skip_reasons[did] = "not allowed (type/priority/disabled)"
                    continue

                # Check if drone has enough fuel
                remaining_fuel = drone_remaining_fuel.get(did, 0)
                if remaining_fuel <= 0:
                    skip_reasons[did] = f"no fuel remaining ({remaining_fuel:.1f})"
                    continue

                # Find best insertion point in this drone's route
                route_data = optimized_routes.get(did, {})
                route = route_data.get("route", [])

                # Handle drones with no route (still at home airport)
                if not route:
                    # Get start and end airports from drone config
                    start_airport = cfg.get("start_airport", f"A{did}")
                    end_airport = cfg.get("end_airport", start_airport)

                    # Calculate insertion cost: start -> target -> end
                    d_start_to_target = self._get_distance(start_airport, tid)
                    d_target_to_end = self._get_distance(tid, end_airport)

                    if math.isinf(d_start_to_target) or math.isinf(d_target_to_end):
                        skip_reasons[did] = f"no path from {start_airport} to {tid} or {tid} to {end_airport}"
                        continue

                    # Total route distance for new route
                    insertion_cost = d_start_to_target + d_target_to_end

                    # Check if drone can afford this route
                    if insertion_cost > remaining_fuel:
                        skip_reasons[did] = f"new route cost {insertion_cost:.1f} > fuel budget {remaining_fuel:.1f}"
                        continue

                    # Mark with special insertion index -1 to indicate "create new route"
                    # Store (start_airport, end_airport) for later route creation
                    viable_options.append((did, -1, insertion_cost, start_airport, end_airport))
                    print(f"      🆕 D{did}: New route {start_airport}→{tid}→{end_airport}, cost={insertion_cost:.1f}", flush=True)
                    continue

                # Get frozen segments for this drone (indices where route[i]->route[i+1] is frozen)
                frozen_segments = set(route_data.get("frozen_segments", []))

                # Use trajectory-based insertion logic
                # Find closest segment in the drone's trajectory
                trajectory = route_data.get("trajectory", [])
                target_pos = self._waypoint_positions.get(tid)

                if not trajectory or not target_pos:
                    skip_reasons[did] = f"no trajectory ({len(trajectory) if trajectory else 0} pts) or target_pos ({target_pos})"
                    continue

                # Convert trajectory to list of tuples if needed
                if trajectory and not isinstance(trajectory[0], tuple):
                    trajectory = [(float(p[0]), float(p[1])) for p in trajectory]

                # Find closest trajectory segment and insertion position
                route_idx, has_perp, closest_vertex = self._find_closest_trajectory_segment(
                    target_pos, trajectory, route, self._waypoint_positions
                )

                # Adjust insertion index if no perpendicular exists
                if not has_perp and closest_vertex == 1:
                    # Closer to second vertex, insert after it
                    route_idx = min(route_idx + 1, len(route))

                # Check if this position is frozen
                if route_idx > 0 and (route_idx - 1) in frozen_segments:
                    skip_reasons[did] = f"frozen segment at position {route_idx}"
                    continue

                # Check if we're trying to insert after the end airport
                if route_idx >= len(route):
                    skip_reasons[did] = f"route_idx {route_idx} >= route length {len(route)}"
                    continue

                # Calculate insertion cost using proper SAM-avoiding distances
                if route_idx == 0:
                    prev_wp = None
                    next_wp = route[0] if route else None
                elif route_idx >= len(route):
                    prev_wp = route[-1] if route else None
                    next_wp = None
                else:
                    prev_wp = route[route_idx - 1]
                    next_wp = route[route_idx]

                # Calculate cost as difference in distances
                if prev_wp and next_wp:
                    orig_dist = self._get_distance(prev_wp, next_wp)
                    new_dist = (
                        self._get_distance(prev_wp, tid) +
                        self._get_distance(tid, next_wp)
                    )
                    insertion_cost = new_dist - orig_dist
                elif prev_wp:
                    insertion_cost = self._get_distance(prev_wp, tid)
                elif next_wp:
                    insertion_cost = self._get_distance(tid, next_wp)
                else:
                    continue

                # Check if we have enough fuel
                if insertion_cost > remaining_fuel:
                    skip_reasons[did] = f"insertion_cost {insertion_cost:.1f} > remaining_fuel {remaining_fuel:.1f}"
                    continue

                # Add this as a viable option (route_idx >= 0 means insert into existing route)
                viable_options.append((did, route_idx, insertion_cost))

            # If no drone can take this target, log why
            if not viable_options:
                print(f"   ❌ {tid} (type={target_type}, priority={priority}): NO VIABLE DRONES", flush=True)
                for did, reason in sorted(skip_reasons.items()):
                    print(f"      D{did}: {reason}", flush=True)
                continue
            else:
                print(f"   ✅ {tid} (type={target_type}, priority={priority}): {len(viable_options)} viable options", flush=True)

            # Among viable options, pick the drone with LOWEST insertion cost
            # This minimizes fuel usage when multiple drones can service the target
            best_option = min(viable_options, key=lambda x: x[2])
            best_drone = best_option[0]
            best_insertion = best_option[1]
            best_cost = best_option[2]

            # Apply the insertion
            if best_insertion == -1:
                # New route case: create route [start_airport, target, end_airport]
                start_airport = best_option[3]
                end_airport = best_option[4]
                route = [start_airport, tid, end_airport]
                print(f"      🚀 Creating new route for D{best_drone}: {start_airport}→{tid}→{end_airport}", flush=True)
            else:
                # Existing route case: insert target at specified position
                route = list(optimized_routes[best_drone].get("route", []))
                route.insert(best_insertion, tid)

            # Recalculate route metrics
            new_distance = self._calculate_route_distance(route)
            new_points = self._calculate_route_points(route, target_by_id)

            # Get fuel budget - from existing route data or from drone_configs
            existing_route_data = optimized_routes.get(best_drone, {})
            fuel_budget = existing_route_data.get("fuel_budget", 0)
            if not fuel_budget:
                fuel_budget = drone_configs.get(best_drone, {}).get("fuel_budget", 0)

            optimized_routes[best_drone] = {
                "route": route,
                "sequence": ",".join(route),
                "points": new_points,
                "distance": new_distance,
                "fuel_budget": fuel_budget
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
        """
        Get distance between two waypoints.

        First tries distance matrix lookup, then falls back to SAM-aware calculation.
        """
        # Try distance matrix first
        if self._distance_matrix:
            labels = self._distance_matrix.get("labels", [])
            matrix = self._distance_matrix.get("matrix", [])

            try:
                i = labels.index(from_id)
                j = labels.index(to_id)
                return matrix[i][j]
            except (ValueError, IndexError):
                pass

        # Fallback: Calculate SAM-aware distance on-the-fly
        if self._waypoint_positions and from_id in self._waypoint_positions and to_id in self._waypoint_positions:
            start_pos = self._waypoint_positions[from_id]
            end_pos = self._waypoint_positions[to_id]

            # If we have SAM navigation and polygons, use SAM-aware path
            if HAS_SAM_NAVIGATION and boundary_plan_path and self._wrapped_polygons:
                try:
                    path, distance, method = boundary_plan_path(start_pos, end_pos, self._wrapped_polygons)
                    if path and distance < float('inf'):
                        print(f"⚠️ Distance matrix fallback: {from_id}→{to_id} = {distance:.2f} (SAM-aware, {method})", flush=True)
                        return distance
                except Exception as e:
                    print(f"⚠️ SAM-aware fallback failed for {from_id}→{to_id}: {e}", flush=True)

            # Final fallback: Euclidean distance
            dx = end_pos[0] - start_pos[0]
            dy = end_pos[1] - start_pos[1]
            euclidean = math.hypot(dx, dy)
            print(f"⚠️ Distance matrix fallback: {from_id}→{to_id} = {euclidean:.2f} (Euclidean)", flush=True)
            return euclidean

        # Absolute fallback if no position data available
        print(f"❌ No distance data available for {from_id}→{to_id}, returning large sentinel value", flush=True)
        return 99999.0

    def _calculate_route_distance(self, route: List[str]) -> float:
        """Calculate total distance of a route using SAM-aware distance matrix."""
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

    DEBUG: Entry point logging
    """
    print(f"\n🔧 POST_OPTIMIZE_SOLUTION CALLED", flush=True)
    print(f"   solution keys: {list(solution.keys())}", flush=True)
    print(f"   env keys: {list(env.keys())}", flush=True)
    print(f"   drone_configs: {list(drone_configs.keys())}", flush=True)
    print(f"   distance_matrix provided: {distance_matrix is not None}", flush=True)
    print(f"   priority_constraints: {priority_constraints}", flush=True)

    # Debug: Check solution routes structure
    routes = solution.get("routes", {})
    print(f"   Solution has {len(routes)} routes:", flush=True)
    for did, rdata in routes.items():
        if isinstance(rdata, dict):
            route = rdata.get("route", [])
            traj = rdata.get("trajectory", [])
            fuel = rdata.get("fuel_budget", "N/A")
            dist = rdata.get("distance", "N/A")
            print(f"      D{did}: route={route}, trajectory_pts={len(traj)}, fuel_budget={fuel}, distance={dist}", flush=True)
        else:
            print(f"      D{did}: (not a dict) {type(rdata)}", flush=True)

    # Set environment for SAM-aware distance fallback calculations
    _optimizer.set_environment(env)

    # Set distance matrix if available
    if distance_matrix:
        _optimizer.set_distance_matrix(distance_matrix)

    print(f"   Calling _optimizer.optimize()...", flush=True)
    result = _optimizer.optimize(solution, env, drone_configs, priority_constraints)
    print(f"   _optimizer.optimize() returned {len(result.get('routes', {}))} routes", flush=True)
    return result


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

    def _get_route_signature(self, routes: Dict[str, Any]) -> str:
        """
        Create a unique signature for a route configuration.
        Used for cycle detection.
        """
        # Sort drone IDs for consistent ordering
        parts = []
        for did in sorted(routes.keys()):
            route = routes[did].get("route", [])
            parts.append(f"{did}:{','.join(str(wp) for wp in route)}")
        return "|".join(parts)

    def _calculate_total_distance(self, routes: Dict[str, Any]) -> float:
        """Calculate total distance across all drones."""
        total = 0.0
        for did, route_data in routes.items():
            total += route_data.get("distance", 0.0)
        return total

    def optimize(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_constraints: Optional[str] = None,
        auto_iterate: bool = True,
        max_iterations: int = 50,
        auto_regen: bool = False,
        debug: bool = False
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

        When auto_iterate=True (default):
        - Runs multiple iterations until no more swaps or cycle detected
        - Tracks fuel/distance at each iteration
        - Returns the solution with lowest total distance

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports, sams
            drone_configs: Drone configurations with capabilities
            priority_constraints: Optional priority constraints string
            auto_iterate: If True, run until convergence/cycle and return best fuel state
            max_iterations: Maximum iterations for auto mode (safety limit)

        Returns:
            Optimized solution with targets moved to closer trajectories
        """
        if auto_iterate:
            return self._optimize_auto(
                solution, env, drone_configs, priority_constraints, max_iterations, auto_regen, debug
            )
        else:
            return self._optimize_single(
                solution, env, drone_configs, priority_constraints, debug
            )

    def _optimize_auto(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_constraints: Optional[str] = None,
        max_iterations: int = 50,
        auto_regen: bool = False
    , debug: bool = False) -> Dict[str, Any]:
        """
        Auto-iterate Swap Closer until convergence or cycle detection.
        Returns the solution with the lowest total distance encountered.
        """
        print(f"\n🔄 [TrajectorySwapOptimizer] AUTO-ITERATE MODE (max {max_iterations} iterations)")

        # Track seen states for cycle detection
        seen_signatures: Set[str] = set()

        # Track best solution (lowest total distance)
        best_solution = None
        best_distance = float('inf')
        best_iteration = 0

        # History for reporting
        all_swaps = []
        iteration_history = []

        current_solution = solution
        iteration = 0

        while iteration < max_iterations:
            iteration += 1

            # Get signature of current state
            current_routes = current_solution.get("routes", {})
            signature = self._get_route_signature(current_routes)

            # Check for cycle
            if signature in seen_signatures:
                print(f"\n🔁 CYCLE DETECTED at iteration {iteration}")
                print(f"   Returning best solution from iteration {best_iteration} (distance: {best_distance:.2f})")
                break

            seen_signatures.add(signature)

            # Calculate current total distance
            current_distance = self._calculate_total_distance(current_routes)
            iteration_history.append({
                "iteration": iteration,
                "distance": current_distance,
                "signature": signature[:50] + "..." if len(signature) > 50 else signature
            })

            # Track best solution
            if current_distance < best_distance:
                best_distance = current_distance
                best_solution = self._deep_copy_solution(current_solution)
                best_iteration = iteration
                print(f"   ✨ Iteration {iteration}: NEW BEST distance = {best_distance:.2f}")
            else:
                print(f"   📍 Iteration {iteration}: distance = {current_distance:.2f}")

            # Run single swap iteration
            result = self._optimize_single(
                current_solution, env, drone_configs, priority_constraints, debug
            )

            swaps_made = result.get("swaps_made", [])

            # Check for convergence (no more swaps)
            if not swaps_made:
                print(f"\n✅ CONVERGED at iteration {iteration} (no more beneficial swaps)")
                print(f"   Final distance: {current_distance:.2f}")
                # Use current solution if it's the best, otherwise use tracked best
                if current_distance <= best_distance:
                    best_solution = self._deep_copy_solution(current_solution)
                    best_distance = current_distance
                    best_iteration = iteration
                break

            # Record swaps
            for swap in swaps_made:
                swap["iteration"] = iteration
                all_swaps.append(swap)

            # Update current solution for next iteration
            current_solution = {
                "routes": result.get("routes", {}),
                "sequences": result.get("sequences", {})
            }

            # Optionally regenerate SAM-aware trajectories for the updated routes
            # so the next iteration can compute SSD/OSD using fresh trajectory
            # vertices. This is controlled by the `auto_regen` flag to preserve
            # the previous client-click workflow by default.
            if auto_regen:
                try:
                    # Build waypoint positions lookup from env
                    waypoint_positions = {}
                    for a in env.get("airports", []):
                        waypoint_positions[str(a["id"])] = [float(a["x"]), float(a["y"])]
                    for t in env.get("targets", []):
                        waypoint_positions[str(t["id"])] = [float(t["x"]), float(t["y"])]

                    # Import planner locally to keep top-level imports safe
                    try:
                        from server.solver.trajectory_planner import ISRTrajectoryPlanner
                    except Exception:
                        # try alternate import path for some environments
                        from trajectory_planner import ISRTrajectoryPlanner

                    planner = ISRTrajectoryPlanner(env.get("sams", []))

                    for did, rd in current_solution.get("routes", {}).items():
                        route = rd.get("route", []) or []
                        # Generate SAM-aware trajectory (returns list of [x,y])
                        try:
                            traj = planner.generate_trajectory(route, waypoint_positions, did)
                        except Exception as e:
                            print(f"   ⚠️ Trajectory planner failed for D{did}: {e}", flush=True)
                            traj = [tuple(waypoint_positions.get(wp, [0, 0])) for wp in route]

                        # Store trajectory and recompute route distance
                        rd["trajectory"] = traj
                        rd["distance"] = self._calculate_route_distance(route)

                    print("   ✅ Regenerated trajectories for next iteration", flush=True)
                except Exception as e:
                    # Non-fatal: log and continue without trajectories
                    print(f"   ⚠️ Trajectory regeneration skipped due to error: {e}", flush=True)

        else:
            # Max iterations reached
            print(f"\n⚠️ MAX ITERATIONS ({max_iterations}) reached")
            print(f"   Returning best solution from iteration {best_iteration} (distance: {best_distance:.2f})")

        # Use best solution found
        if best_solution is None:
            best_solution = current_solution

        # Recalculate final metrics
        final_routes = best_solution.get("routes", {})
        targets = env.get("targets", [])
        target_by_id = {str(t["id"]): t for t in targets}

        for did, route_data in final_routes.items():
            route = route_data.get("route", [])
            route_data["sequence"] = ",".join(str(wp) for wp in route)
            route_data["distance"] = self._calculate_route_distance(route)
            route_data["points"] = self._calculate_route_points(route, target_by_id)

        print(f"\n📊 AUTO-ITERATE SUMMARY:")
        print(f"   Total iterations: {iteration}")
        print(f"   Total swaps made: {len(all_swaps)}")
        print(f"   Best iteration: {best_iteration}")
        print(f"   Best total distance: {best_distance:.2f}")

        return {
            "sequences": {
                did: data.get("sequence", "") for did, data in final_routes.items()
            },
            "routes": final_routes,
            "swaps_made": all_swaps,
            "iterations": iteration,
            "best_iteration": best_iteration,
            "best_distance": best_distance,
            "converged": len(all_swaps) == 0 or (iteration < max_iterations and not any(s.get("iteration") == iteration for s in all_swaps)),
            "cycle_detected": signature in seen_signatures if iteration > 0 else False,
        }

    def _deep_copy_solution(self, solution: Dict[str, Any]) -> Dict[str, Any]:
        """Create a deep copy of a solution to preserve state."""
        import copy
        return {
            "routes": {
                did: {
                    "route": list(data.get("route", [])),
                    "sequence": data.get("sequence", ""),
                    "points": data.get("points", 0),
                    "distance": data.get("distance", 0),
                    "fuel_budget": data.get("fuel_budget", 0),
                    "frozen_segments": list(data.get("frozen_segments", [])),
                }
                for did, data in solution.get("routes", {}).items()
            },
            "sequences": dict(solution.get("sequences", {}))
        }

    def _optimize_single(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_constraints: Optional[str] = None,
        debug: bool = False
    ) -> Dict[str, Any]:
        """
        Perform a SINGLE swap iteration (the original behavior).
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

        # Do ONE swap per invocation to avoid stale trajectory data
        # After swap, frontend regenerates trajectories and user can click again
        num_passes = 1
        all_swaps = []  # Track all swaps made

        print(f"\n🔧 [TrajectorySwapOptimizer] Searching for best swap...")

        for pass_num in range(num_passes):
            print(f"\n  📍 Evaluating all targets...")

            # Collect ALL possible swap candidates in this pass
            swap_candidates = []

            # Optional diagnostics per target when debug=True
            target_diagnostics: Dict[str, Any] = {}

            # Collect all targets to check in this pass
            targets_to_check = []
            for current_drone, route_data in optimized_routes.items():
                route = route_data["route"]
                for wp_id in route:
                    if str(wp_id).startswith("T"):
                        targets_to_check.append(wp_id)

            # Evaluate each target to find its best swap option
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

                # Use actual trajectory vertices instead of route waypoints
                # This correctly accounts for SAM-avoidance trajectory deviations
                if str(wp_id) not in target_to_traj_index:
                    print(f"    ⏭️  SKIP {wp_id}: Not found in trajectory index")
                    continue

                traj_drone_id, traj_idx = target_to_traj_index[str(wp_id)]

                # Ensure this target is still on the same drone
                if traj_drone_id != current_drone:
                    print(f"    ⏭️  SKIP {wp_id}: Drone mismatch (index:{traj_drone_id}, current:{current_drone})")
                    continue

                trajectory = drone_trajectories.get(current_drone)
                if not trajectory or len(trajectory) < 2:
                    print(f"    ⏭️  SKIP {wp_id}: No valid trajectory (len={len(trajectory) if trajectory else 0})")
                    continue

                # Get actual trajectory vertices immediately before/after this target
                # Handle boundary cases (targets at trajectory endpoints)
                if traj_idx == 0:
                    # Target at start of trajectory - use first two vertices
                    prev_pos = trajectory[0]
                    next_pos = trajectory[1] if len(trajectory) > 1 else trajectory[0]
                elif traj_idx >= len(trajectory) - 1:
                    # Target at end of trajectory - use last two vertices
                    prev_pos = trajectory[-2] if len(trajectory) > 1 else trajectory[-1]
                    next_pos = trajectory[-1]
                else:
                    # Target in middle of trajectory - use surrounding vertices
                    prev_pos = trajectory[traj_idx - 1]
                    next_pos = trajectory[traj_idx + 1]

                # Calculate removal delta: how much distance would be saved by removing this target
                remove_delta = self._removal_delta(wp_id, optimized_routes[current_drone], waypoint_positions)

                # Initialize diagnostics entry for this target
                if debug:
                    target_diagnostics[str(wp_id)] = {
                        "remove_delta": remove_delta,
                        "candidates": []
                    }

                # Get route waypoint IDs for logging
                prev_wp = route[current_idx - 1] if current_idx > 0 else "?"
                next_wp = route[current_idx + 1] if current_idx < len(route) - 1 else "?"

                if remove_delta is None or remove_delta <= 0.0:
                    print(f"    🎯 {wp_id}: remove_delta=None or <=0 (skip) ({prev_wp}→{wp_id}→{next_wp})")
                    continue

                print(f"    🎯 {wp_id}: remove_delta={remove_delta:.2f} (route neighbors {prev_wp}→{wp_id}→{next_wp})")

                # Search for segments where inserting this target yields the best delta (insertion cost)
                best_drone = None
                best_route_segment = None
                best_insert_delta = float('inf')
                best_osd = float('inf')
                best_net_savings = -float('inf')  # Track maximum gain (remove_delta - insert_delta)

                # Check all trajectory segments in all drones
                for other_drone, other_route_data in optimized_routes.items():
                    other_route = other_route_data["route"]
                    other_frozen = other_route_data["frozen_segments"]

                    # Check drone capability
                    if other_drone != current_drone:
                        other_cfg = drone_configs.get(other_drone, {})
                        if not target_allowed_for_drone(target, other_drone, other_cfg, priority_filters):
                            continue

                    # Get trajectory for this drone
                    other_trajectory = drone_trajectories.get(other_drone)
                    if not other_trajectory or len(other_trajectory) < 2:
                        continue

                    # Check each route segment (between waypoints)
                    for j in range(len(other_route) - 1):
                        if j in other_frozen:
                            continue

                        # Skip current target's adjacent segments
                        if other_drone == current_drone:
                            if j == current_idx - 1:  # Segment: prev→target
                                continue
                            if j == current_idx:  # Segment: target→next
                                continue

                        seg_start_id = str(other_route[j])
                        seg_end_id = str(other_route[j + 1])

                        # For LINEAR routes (start != end airport), we previously skipped
                        # the last segment. But this is WRONG - inserting into T→A creates
                        # T→X→A which is valid. The insertion point (j+1) places the target
                        # BEFORE the ending airport, not after.
                        #
                        # For CYCLIC routes (start == end airport, e.g., A1→...→A1), the
                        # last segment returns to the start, and inserting before it is
                        # perfectly valid.
                        #
                        # Therefore, we should NOT skip the last segment in either case.
                        # The only invalid insertion would be at position len(route), which
                        # would place a target AFTER the end airport - but that's handled
                        # elsewhere in the insertion logic.

                        # Find trajectory indices for this route segment
                        # We need to check all trajectory vertices between these route waypoints
                        start_traj_idx = None
                        end_traj_idx = None

                        # Find start waypoint in trajectory
                        start_wp_pos = waypoint_positions.get(seg_start_id)
                        if start_wp_pos:
                            for ti, tv in enumerate(other_trajectory):
                                if abs(tv[0] - start_wp_pos[0]) < 0.001 and abs(tv[1] - start_wp_pos[1]) < 0.001:
                                    start_traj_idx = ti
                                    break

                        # Find end waypoint in trajectory
                        # Search AFTER start waypoint to handle return-to-base routes (e.g., A1→...→A1)
                        end_wp_pos = waypoint_positions.get(seg_end_id)
                        if end_wp_pos and start_traj_idx is not None:
                            for ti in range(start_traj_idx + 1, len(other_trajectory)):
                                tv = other_trajectory[ti]
                                if abs(tv[0] - end_wp_pos[0]) < 0.001 and abs(tv[1] - end_wp_pos[1]) < 0.001:
                                    end_traj_idx = ti
                                    break

                        if start_traj_idx is None or end_traj_idx is None:
                            continue

                        # Check all trajectory sub-segments between these waypoints
                        # This includes SAM-avoidance tangent points
                        best_osd_in_segment = float('inf')

                        for ti in range(start_traj_idx, end_traj_idx):
                            traj_seg_start = other_trajectory[ti]
                            traj_seg_end = other_trajectory[ti + 1]

                            # Quick bounding box check (use remove_delta as radius)
                            min_x = min(traj_seg_start[0], traj_seg_end[0]) - remove_delta
                            max_x = max(traj_seg_start[0], traj_seg_end[0]) + remove_delta
                            min_y = min(traj_seg_start[1], traj_seg_end[1]) - remove_delta
                            max_y = max(traj_seg_start[1], traj_seg_end[1]) + remove_delta

                            if not (min_x <= target_pos[0] <= max_x and min_y <= target_pos[1] <= max_y):
                                continue  # Trajectory segment too far away

                            # Calculate OSD to this trajectory sub-segment (geometric check)
                            osd = self._point_to_line_distance(target_pos, traj_seg_start, traj_seg_end)
                            if osd < best_osd_in_segment:
                                best_osd_in_segment = osd

                        # Use the best (minimum) OSD found across all sub-segments
                        osd = best_osd_in_segment

                        if osd == float('inf'):
                            continue

                        # Only consider segments that are geometrically closer
                        if osd >= remove_delta:
                            continue

                        # Calculate insertion delta (using distance helper _d)
                        d_start = self._d(seg_start_id, str(wp_id), waypoint_positions)
                        d_end = self._d(str(wp_id), seg_end_id, waypoint_positions)
                        d_direct = self._d(seg_start_id, seg_end_id, waypoint_positions)
                        if any(math.isinf(x) for x in (d_start, d_end, d_direct)):
                            continue
                        insertion_cost = d_start + d_end - d_direct

                        # Check fuel budget for target receiver drone
                        if other_drone != current_drone:
                            other_fuel_budget = other_route_data.get("fuel_budget", 0)
                            if other_fuel_budget <= 0:
                                other_cfg = drone_configs.get(other_drone, {})
                                other_fuel_budget = other_cfg.get("fuel_budget", 200)

                            other_current_distance = other_route_data.get("distance") or self._calculate_route_distance(other_route)
                            to_new = other_current_distance + insertion_cost
                            if to_new > other_fuel_budget:
                                print(f"            ⛽ FUEL: D{other_drone} {seg_start_id}→{seg_end_id} exceeds budget ({other_current_distance:.1f}+{insertion_cost:.1f} > {other_fuel_budget:.1f})")
                                continue

                        # Calculate net savings: remove_delta - insertion_cost
                        net_savings = remove_delta - insertion_cost

                        # Record candidate diagnostics
                        if debug:
                            candidate_info = {
                                "to_drone": other_drone,
                                "seg_start": seg_start_id,
                                "seg_end": seg_end_id,
                                "to_segment": j + 1,
                                "osd": osd,
                                "insert_delta": insertion_cost,
                                "net_savings": net_savings,
                                "fuel_ok": True if (other_drone == current_drone or (other_route_data.get("fuel_budget", 0) or drone_configs.get(other_drone, {}).get("fuel_budget", 0)) >= (other_route_data.get("distance") or self._calculate_route_distance(other_route) + insertion_cost)) else False
                            }
                            target_diagnostics[str(wp_id)]["candidates"].append(candidate_info)

                        # Track best option: select target with MAXIMUM net savings
                        if net_savings > best_net_savings:
                            print(f"            🔄 New best: D{other_drone} {seg_start_id}→{seg_end_id}, gain={net_savings:.2f} (remove={remove_delta:.2f} - insert={insertion_cost:.2f})")
                            best_net_savings = net_savings
                            best_osd = osd
                            best_insert_delta = insertion_cost
                            best_drone = other_drone
                            best_route_segment = j + 1  # Insert after seg_start

                # If found a better segment, add to candidates (don't execute yet)
                if best_drone and best_insert_delta < remove_delta:
                    gain = remove_delta - best_insert_delta
                    print(f"       📋 Candidate: {wp_id} from D{current_drone} to D{best_drone}, gain={gain:.2f}")

                    swap_candidates.append({
                        "target": wp_id,
                        "from_drone": current_drone,
                        "to_drone": best_drone,
                        "from_idx": current_idx,
                        "to_segment": best_route_segment,
                        "remove_delta": remove_delta,
                        "insert_delta": best_insert_delta,
                        "osd": best_osd,
                        "gain": gain,
                        "same_drone": (best_drone == current_drone),
                    })

            # After evaluating ALL targets, pick the SINGLE best swap
            if swap_candidates:
                # Sort by gain (descending) and pick the best one
                best_swap = max(swap_candidates, key=lambda x: x['gain'])

                print(f"\n  🏆 Selected best swap: {best_swap['target']} from D{best_swap['from_drone']} to D{best_swap['to_drone']}, gain={best_swap['gain']:.2f}")

                # Execute only this one swap
                wp_id = best_swap['target']
                from_drone = best_swap['from_drone']
                to_drone = best_swap['to_drone']
                to_segment = best_swap['to_segment']

                # Remove from current route
                optimized_routes[from_drone]["route"].remove(wp_id)

                # Insert into new route
                optimized_routes[to_drone]["route"].insert(to_segment, wp_id)

                # Record the swap
                all_swaps.append({
                    "target": wp_id,
                    "from_drone": from_drone,
                    "to_drone": to_drone,
                    "remove_delta": best_swap.get('remove_delta'),
                    "insert_delta": best_swap.get('insert_delta'),
                    "osd": best_swap.get('osd'),
                    "savings": best_swap['gain'],
                    "same_drone": best_swap['same_drone'],
                    "pass": pass_num + 1
                })
            else:
                print(f"\n  ⏹️  No beneficial swaps found in this pass")

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
            "passes": num_passes,
            "target_diagnostics": target_diagnostics if debug else {}
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

    def _d(self, ida: str, idb: str, positions: Dict[str, Tuple[float, float]]) -> float:
        """
        Distance metric between two waypoint ids. Prefer the SAM-aware distance matrix
        if available, otherwise fall back to Euclidean on provided positions.
        """
        # Use precomputed matrix if available
        if self._distance_matrix:
            labels = self._distance_matrix.get("labels", [])
            matrix = self._distance_matrix.get("matrix", [])
            try:
                idx_a = labels.index(ida)
                idx_b = labels.index(idb)
                return float(matrix[idx_a][idx_b])
            except Exception:
                pass

        # Fallback to Euclidean using positions dict
        pa = positions.get(str(ida))
        pb = positions.get(str(idb))
        if pa is None or pb is None:
            return float('inf')
        return math.hypot(pa[0] - pb[0], pa[1] - pb[1])

    def _removal_delta(self, tid: str, route_data: Dict[str, Any], positions: Dict[str, Tuple[float, float]]) -> Optional[float]:
        """
        Compute the delta (distance change) if `tid` is removed from the given route.
        Returns the extra distance currently incurred by visiting `tid` (positive means
        removing it reduces route distance). Returns None if removal is invalid/undefined.
        """
        route = route_data.get("route", [])
        if not route:
            return None
        try:
            i = route.index(tid)
        except ValueError:
            return None
        # cannot remove if at ends (shouldn't happen for targets if route has airports)
        if i <= 0 or i >= len(route) - 1:
            return None

        prev_id = route[i - 1]
        next_id = route[i + 1]
        if str(prev_id) not in positions or str(next_id) not in positions or str(tid) not in positions:
            return None

        prev_p = positions[str(prev_id)]
        t_p = positions[str(tid)]
        next_p = positions[str(next_id)]

        # Use distance metric helper for consistency with insertion cost
        d_prev_t = self._d(str(prev_id), str(tid), positions)
        d_t_next = self._d(str(tid), str(next_id), positions)
        d_prev_next = self._d(str(prev_id), str(next_id), positions)

        if any(math.isinf(x) for x in (d_prev_t, d_t_next, d_prev_next)):
            return None

        return d_prev_t + d_t_next - d_prev_next

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
    priority_constraints: Optional[str] = None,
    auto_iterate: bool = True,
    max_iterations: int = 50,
    auto_regen: bool = False
    , debug: bool = False) -> Dict[str, Any]:
    """
    Reassign targets to drones whose trajectories pass closer to them.

    For each target, if another drone's path passes closer AND that drone
    can carry the target, move it to the closer trajectory.

    When auto_iterate=True (default):
    - Runs multiple iterations until convergence or cycle detection
    - Returns the solution with the lowest total distance encountered

    Args:
        solution: Current solution with drone routes
        env: Environment data with targets, airports
        drone_configs: Drone configurations with capabilities
        distance_matrix: Pre-calculated distance matrix
        priority_constraints: Optional priority constraints string
                              e.g., "D1,D2: priority>=6; D3,D4: priority<6"
        auto_iterate: If True (default), run until convergence/cycle and return best fuel state
        max_iterations: Maximum iterations for auto mode (default 50)

    Returns:
        Optimized solution with targets moved to closer trajectories
    """
    if distance_matrix:
        _trajectory_optimizer.set_distance_matrix(distance_matrix)

    return _trajectory_optimizer.optimize(
        solution,
        env,
        drone_configs,
        priority_constraints,
        auto_iterate=auto_iterate,
        max_iterations=max_iterations,
        auto_regen=auto_regen,
        debug=debug,
    )


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


# ============================================================================
# Unified Optimizer: Interleaved Insert + Swap
# ============================================================================

class UnifiedOptimizer:
    """
    Unified optimizer that interleaves Insert and Swap operations.

    Unlike the fixed-order approach (Insert → Swap → Crossing Removal),
    this optimizer evaluates ALL possible operations at each iteration
    and executes the single best one. This allows:

    1. A swap to free up fuel capacity, enabling a later insert
    2. An insert to create a better swap opportunity
    3. The optimization landscape to adapt after each operation

    Crossing removal runs only after Insert/Swap convergence.

    Scoring Strategy (Hybrid):
    - Insert score: priority * 10 / max(cost, 1)  [maximize priority per fuel]
    - Swap score: gain (= removal_delta - insertion_cost) [maximize fuel savings]
    """

    def __init__(self):
        self._distance_matrix: Optional[Dict[str, Any]] = None
        self._env: Optional[Dict[str, Any]] = None
        self._waypoint_positions: Optional[Dict[str, List[float]]] = None
        self._target_by_id: Dict[str, Dict[str, Any]] = {}

    def set_distance_matrix(self, matrix_data: Dict[str, Any]):
        """Set the pre-calculated distance matrix."""
        self._distance_matrix = matrix_data

    def set_environment(self, env: Dict[str, Any]):
        """Set environment data for waypoint positions and target lookup."""
        self._env = env

        # Build waypoint position lookup
        waypoint_positions = {}
        airports = env.get("airports", [])
        targets = env.get("targets", [])

        for a in airports:
            waypoint_positions[str(a["id"])] = [float(a["x"]), float(a["y"])]
        for t in targets:
            waypoint_positions[str(t["id"])] = [float(t["x"]), float(t["y"])]
            self._target_by_id[str(t["id"])] = t

        self._waypoint_positions = waypoint_positions

    def _get_distance(self, from_id: str, to_id: str) -> float:
        """Get SAM-aware distance from distance matrix."""
        if self._distance_matrix:
            labels = self._distance_matrix.get("labels", [])
            matrix = self._distance_matrix.get("matrix", [])

            try:
                i = labels.index(from_id)
                j = labels.index(to_id)
                return matrix[i][j]
            except (ValueError, IndexError):
                pass

        # Fallback to Euclidean if matrix lookup fails
        if self._waypoint_positions:
            if from_id in self._waypoint_positions and to_id in self._waypoint_positions:
                p1 = self._waypoint_positions[from_id]
                p2 = self._waypoint_positions[to_id]
                return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

        return 99999.0

    def _calculate_insertion_cost(self, prev_wp: str, target_id: str, next_wp: str) -> float:
        """
        Calculate cost to insert target between two waypoints.

        insertion_cost = d(A, X) + d(X, B) - d(A, B)

        Where A=prev_wp, X=target, B=next_wp
        """
        d_ax = self._get_distance(prev_wp, target_id)
        d_xb = self._get_distance(target_id, next_wp)
        d_ab = self._get_distance(prev_wp, next_wp)
        return d_ax + d_xb - d_ab

    def _calculate_route_distance(self, route: List[str]) -> float:
        """Calculate total distance of a route."""
        if len(route) < 2:
            return 0.0
        total = 0.0
        for i in range(len(route) - 1):
            total += self._get_distance(route[i], route[i + 1])
        return total

    def _get_remaining_fuel(
        self,
        route_data: Dict[str, Any],
        drone_config: Dict[str, Any]
    ) -> float:
        """Calculate remaining fuel for a drone after its current route."""
        fuel_budget = float(drone_config.get("fuel_capacity", 100))
        distance_used = route_data.get("distance", 0.0)
        return max(0.0, fuel_budget - distance_used)

    def _deep_copy_solution(self, solution: Dict[str, Any]) -> Dict[str, Any]:
        """Create a deep copy of solution to avoid mutation issues."""
        import copy
        return copy.deepcopy(solution)

    def optimize(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_constraints: Optional[str] = None,
        max_iterations: int = 100,
        run_crossing_removal: bool = True,
        debug: bool = False
    ) -> Dict[str, Any]:
        """
        Main entry point for unified optimization.

        Args:
            solution: Current solution with routes for each drone
            env: Environment dict with targets, airports, sams
            drone_configs: Drone configurations with fuel budgets
            priority_constraints: Optional priority constraints string
            max_iterations: Maximum iterations before stopping
            run_crossing_removal: If True, run 2-opt after convergence
            debug: Enable verbose logging

        Returns:
            Optimized solution with improved routes
        """
        print(f"\n🔀 [UnifiedOptimizer] Starting interleaved Insert+Swap optimization", flush=True)
        print(f"   max_iterations={max_iterations}, run_crossing_removal={run_crossing_removal}", flush=True)

        # Set up environment
        self.set_environment(env)

        # Parse priority constraints
        priority_filters = parse_priority_constraint(priority_constraints) if priority_constraints else {}

        # Work on a copy to avoid mutating original
        current_solution = self._deep_copy_solution(solution)

        # Track operations performed
        operations_log = []
        total_inserts = 0
        total_swaps = 0

        for iteration in range(1, max_iterations + 1):
            # Evaluate all candidates
            insert_candidates = self._evaluate_insert_candidates(
                current_solution, env, drone_configs, priority_filters
            )
            swap_candidates = self._evaluate_swap_candidates(
                current_solution, env, drone_configs, priority_filters
            )

            all_candidates = insert_candidates + swap_candidates

            if not all_candidates:
                print(f"   ✅ Converged at iteration {iteration} - no more beneficial operations", flush=True)
                break

            # Pick best operation (highest score)
            best = max(all_candidates, key=lambda c: c["score"])

            if debug:
                print(f"   Iteration {iteration}: {len(insert_candidates)} insert candidates, "
                      f"{len(swap_candidates)} swap candidates", flush=True)
                print(f"   Best: {best['type']} {best.get('target', 'N/A')} "
                      f"score={best['score']:.3f}", flush=True)

            # Execute the operation
            if best["type"] == "insert":
                self._execute_insert(current_solution, best, drone_configs)
                total_inserts += 1
                op_desc = f"Insert {best['target']} into D{best['drone']} at pos {best['position']} (cost={best['cost']:.2f}, priority={best['priority']})"
            else:  # swap
                self._execute_swap(current_solution, best, drone_configs)
                total_swaps += 1
                op_desc = f"Swap {best['target']} from D{best['from_drone']} to D{best['to_drone']} (gain={best['gain']:.2f})"

            operations_log.append({
                "iteration": iteration,
                "type": best["type"],
                "description": op_desc
            })

            if debug or iteration <= 5 or iteration % 10 == 0:
                print(f"   [{iteration}] {op_desc}", flush=True)

        print(f"\n   📊 Unified optimization complete: {total_inserts} inserts, {total_swaps} swaps "
              f"in {len(operations_log)} iterations", flush=True)

        # Run crossing removal as final cleanup
        if run_crossing_removal:
            print(f"   Running crossing removal as final cleanup...", flush=True)
            current_solution = crossing_removal_optimize(
                current_solution, env, drone_configs, self._distance_matrix
            )

        # Add operation log to solution metadata
        if "metadata" not in current_solution:
            current_solution["metadata"] = {}
        current_solution["metadata"]["unified_optimizer"] = {
            "total_iterations": len(operations_log),
            "total_inserts": total_inserts,
            "total_swaps": total_swaps,
            "operations": operations_log
        }

        return current_solution

    def _evaluate_insert_candidates(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_filters: Dict[str, Callable[[int], bool]]
    ) -> List[Dict[str, Any]]:
        """
        Find all viable insert operations for unvisited targets.

        Returns list of candidates with:
        - type: "insert"
        - target: target ID
        - drone: drone ID to insert into
        - position: index in route to insert
        - cost: insertion cost (fuel)
        - priority: target priority
        - score: priority * 10 / max(cost, 1)
        """
        candidates = []
        targets = env.get("targets", [])

        # Find visited targets
        visited = set()
        for did, route_data in solution.get("routes", {}).items():
            for wp in route_data.get("route", []):
                if str(wp).startswith("T"):
                    visited.add(str(wp))

        # Find unvisited targets
        unvisited = [t for t in targets if str(t["id"]) not in visited]

        for target in unvisited:
            target_id = str(target["id"])
            target_priority = int(target.get("priority", 5))

            for drone_id, cfg in drone_configs.items():
                # Check if drone is enabled and can access this target
                if not target_allowed_for_drone(target, drone_id, cfg, priority_filters):
                    continue

                route_data = solution.get("routes", {}).get(drone_id, {})
                route = route_data.get("route", [])
                frozen_segments = set(route_data.get("frozen_segments", []))
                remaining_fuel = self._get_remaining_fuel(route_data, cfg)

                if remaining_fuel <= 0 or len(route) < 2:
                    continue

                # Find best insertion position
                best_cost = float('inf')
                best_position = None

                for i in range(1, len(route)):
                    # Skip if this segment is frozen
                    if (i - 1) in frozen_segments:
                        continue

                    prev_wp = str(route[i - 1])
                    next_wp = str(route[i])

                    cost = self._calculate_insertion_cost(prev_wp, target_id, next_wp)

                    if cost < best_cost and cost <= remaining_fuel:
                        best_cost = cost
                        best_position = i

                if best_position is not None and best_cost < float('inf'):
                    # Hybrid scoring: prioritize high-priority targets
                    score = target_priority * 10 / max(best_cost, 1.0)

                    candidates.append({
                        "type": "insert",
                        "target": target_id,
                        "drone": drone_id,
                        "position": best_position,
                        "cost": best_cost,
                        "priority": target_priority,
                        "score": score
                    })

        return candidates

    def _evaluate_swap_candidates(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]],
        priority_filters: Dict[str, Callable[[int], bool]]
    ) -> List[Dict[str, Any]]:
        """
        Find all viable swap operations for visited targets.

        A swap moves a target from one route position to another (same or different drone).
        Gain = removal_delta - insertion_cost

        Returns list of candidates with:
        - type: "swap"
        - target: target ID
        - from_drone: source drone ID
        - to_drone: destination drone ID
        - from_idx: index in source route
        - to_position: insertion index in destination route
        - removal_delta: fuel saved by removing from source
        - insertion_cost: fuel spent inserting at destination
        - gain: net fuel savings (removal_delta - insertion_cost)
        - score: gain (direct fuel savings)
        """
        candidates = []

        for drone_id, route_data in solution.get("routes", {}).items():
            route = route_data.get("route", [])
            frozen_segments = set(route_data.get("frozen_segments", []))

            for i, wp_id in enumerate(route):
                wp_id = str(wp_id)

                # Only process targets (not airports)
                if not wp_id.startswith("T"):
                    continue

                # Can't remove first or last waypoint (airports)
                if i <= 0 or i >= len(route) - 1:
                    continue

                # Check if target is in a frozen segment
                if (i - 1) in frozen_segments or i in frozen_segments:
                    continue

                prev_wp = str(route[i - 1])
                next_wp = str(route[i + 1])

                # Calculate removal delta (what we save by removing this target)
                removal_delta = self._calculate_insertion_cost(prev_wp, wp_id, next_wp)

                if removal_delta <= 0:
                    # No benefit to moving this target
                    continue

                # Get target info
                target = self._target_by_id.get(wp_id)
                if not target:
                    continue

                # Search for better positions across all drones
                for other_drone_id, other_route_data in solution.get("routes", {}).items():
                    other_route = other_route_data.get("route", [])
                    other_frozen = set(other_route_data.get("frozen_segments", []))
                    other_cfg = drone_configs.get(other_drone_id, {})

                    # Check capability for cross-drone swaps
                    if other_drone_id != drone_id:
                        if not target_allowed_for_drone(target, other_drone_id, other_cfg, priority_filters):
                            continue

                    for j in range(1, len(other_route)):
                        # Skip frozen segments
                        if (j - 1) in other_frozen:
                            continue

                        # Skip adjacent positions (same drone) - no net change
                        if other_drone_id == drone_id:
                            if j == i or j == i + 1:
                                continue

                        seg_start = str(other_route[j - 1])
                        seg_end = str(other_route[j])

                        insertion_cost = self._calculate_insertion_cost(seg_start, wp_id, seg_end)

                        # For cross-drone swaps, check fuel budget
                        if other_drone_id != drone_id:
                            other_remaining = self._get_remaining_fuel(other_route_data, other_cfg)
                            if insertion_cost > other_remaining:
                                continue

                        gain = removal_delta - insertion_cost

                        if gain > 0:
                            candidates.append({
                                "type": "swap",
                                "target": wp_id,
                                "from_drone": drone_id,
                                "to_drone": other_drone_id,
                                "from_idx": i,
                                "to_position": j,
                                "removal_delta": removal_delta,
                                "insertion_cost": insertion_cost,
                                "gain": gain,
                                "score": gain  # Direct fuel savings as score
                            })

        return candidates

    def _execute_insert(
        self,
        solution: Dict[str, Any],
        candidate: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]]
    ):
        """Apply an insert operation to the solution."""
        drone_id = candidate["drone"]
        target_id = candidate["target"]
        position = candidate["position"]
        cost = candidate["cost"]

        route_data = solution["routes"][drone_id]
        route = route_data["route"]

        # Insert target at position
        route.insert(position, target_id)

        # Update distance
        new_distance = self._calculate_route_distance(route)
        route_data["distance"] = new_distance

        # Update points
        target = self._target_by_id.get(target_id, {})
        current_points = route_data.get("points", 0)
        route_data["points"] = current_points + int(target.get("priority", 5))

    def _execute_swap(
        self,
        solution: Dict[str, Any],
        candidate: Dict[str, Any],
        drone_configs: Dict[str, Dict[str, Any]]
    ):
        """Apply a swap operation to the solution."""
        target_id = candidate["target"]
        from_drone = candidate["from_drone"]
        to_drone = candidate["to_drone"]
        from_idx = candidate["from_idx"]
        to_position = candidate["to_position"]

        from_route_data = solution["routes"][from_drone]
        from_route = from_route_data["route"]

        # Remove from source
        from_route.pop(from_idx)

        # Adjust to_position if same drone and after removal point
        if to_drone == from_drone and to_position > from_idx:
            to_position -= 1

        to_route_data = solution["routes"][to_drone]
        to_route = to_route_data["route"]

        # Insert at destination
        to_route.insert(to_position, target_id)

        # Update distances
        from_route_data["distance"] = self._calculate_route_distance(from_route)
        to_route_data["distance"] = self._calculate_route_distance(to_route)

        # Update points if cross-drone swap
        if from_drone != to_drone:
            target = self._target_by_id.get(target_id, {})
            priority = int(target.get("priority", 5))
            from_route_data["points"] = from_route_data.get("points", 0) - priority
            to_route_data["points"] = to_route_data.get("points", 0) + priority


# Singleton instance
_unified_optimizer = UnifiedOptimizer()


def unified_optimize(
    solution: Dict[str, Any],
    env: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
    distance_matrix: Optional[Dict[str, Any]] = None,
    priority_constraints: Optional[str] = None,
    max_iterations: int = 100,
    run_crossing_removal: bool = True,
    debug: bool = False
) -> Dict[str, Any]:
    """
    Unified optimization with interleaved Insert+Swap operations.

    This optimizer evaluates ALL possible insert and swap operations at each
    iteration and executes the single best one, allowing the optimization
    landscape to adapt. Crossing removal runs as final cleanup.

    Args:
        solution: Current solution with drone routes
        env: Environment data with targets, airports
        drone_configs: Drone configurations with fuel budgets
        distance_matrix: Pre-calculated SAM-aware distance matrix
        priority_constraints: Optional priority constraints string
        max_iterations: Maximum iterations before stopping (default 100)
        run_crossing_removal: If True, run 2-opt after convergence (default True)
        debug: Enable verbose logging

    Returns:
        Optimized solution with improved routes

    Example:
        result = unified_optimize(
            solution, env, drone_configs,
            distance_matrix=matrix_data,
            max_iterations=50
        )
    """
    if distance_matrix:
        _unified_optimizer.set_distance_matrix(distance_matrix)

    return _unified_optimizer.optimize(
        solution, env, drone_configs,
        priority_constraints=priority_constraints,
        max_iterations=max_iterations,
        run_crossing_removal=run_crossing_removal,
        debug=debug
    )
