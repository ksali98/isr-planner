# isr_web/server/solver_bridge.py

from typing import Dict, Any, List, Tuple, Optional
import math
import sys
import time
from pathlib import Path

# Add legacy path to sys.path for isr_editor imports
legacy_path = Path(__file__).resolve().parents[3] / "legacy" / "isr_legacy_all"
if str(legacy_path) not in sys.path:
    sys.path.insert(0, str(legacy_path))

# Add root Editable-ENV path for orienteering_with_matrix.py
root_path = Path(__file__).resolve().parents[3]
if str(root_path) not in sys.path:
    sys.path.insert(0, str(root_path))

# Use the modular solver - import directly to avoid __init__.py pulling in matplotlib
from isr_web.webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface  # type: ignore

# Import new solver components
from .sam_distance_matrix import (
    calculate_sam_aware_matrix,
    get_cached_matrix,
    clear_matrix_cache,
)
from .target_allocator import allocate_targets, set_allocator_matrix
from .post_optimizer import post_optimize_solution, set_optimizer_matrix
from .trajectory_planner import ISRTrajectoryPlanner

# Import polygon wrapping for visualization
from path_planning_core.sam_wrapping import wrap_sams

# Single global solver instance
print("📦 Initializing OrienteeringSolverInterface...", flush=True)
_solver = OrienteeringSolverInterface()
print("📦 Solver interface initialized", flush=True)

# Cached distance matrix for current environment
_cached_env_hash: Optional[str] = None


def _build_distance_matrix(
    airports: List[Dict[str, Any]],
    targets: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """
    FAST Euclidean distance matrix (no SAM avoidance yet).
    Mirrors _calculate_distance_matrix() from your ISREditor.
    """
    all_wp = airports + targets
    labels: List[str] = [wp["id"] for wp in all_wp]
    n = len(all_wp)

    matrix: List[List[float]] = [[0.0] * n for _ in range(n)]

    def get_pos(idx: int) -> Tuple[float, float]:
        wp = all_wp[idx]
        return float(wp["x"]), float(wp["y"])

    for i in range(n):
        xi, yi = get_pos(i)
        for j in range(n):
            if i == j:
                matrix[i][j] = 0.0
            else:
                xj, yj = get_pos(j)
                dx = xj - xi
                dy = yj - yi
                matrix[i][j] = math.hypot(dx, dy)

    waypoint_details: List[Dict[str, Any]] = []
    for wp in all_wp:
        detail = {
            "id": wp["id"],
            "x": float(wp["x"]),
            "y": float(wp["y"]),
        }
        # Add target fields if this is a target
        if str(wp["id"]).startswith("T"):
            detail["priority"] = int(wp.get("priority", 5))
            detail["type"] = wp.get("type", "a")
        waypoint_details.append(detail)

    return {
        "matrix": matrix,
        "labels": labels,
        "waypoints": waypoint_details,
        "excluded_targets": [],  # SAM filtering can be added here later
    }


def _parse_env_for_solver(
    env: Dict[str, Any]
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    """
    Pull airports/targets/sams out of the web environment JSON.
    We expect the web client to send the same basic structure as the editor export.
    """
    airports: List[Dict[str, Any]] = []
    for a in env.get("airports", []):
        airports.append({
            "id": str(a["id"]),
            "x": float(a["x"]),
            "y": float(a["y"]),
        })

    targets: List[Dict[str, Any]] = []
    for t in env.get("targets", []):
        targets.append({
            "id": str(t["id"]),
            "x": float(t["x"]),
            "y": float(t["y"]),
            "priority": int(t.get("priority", 5)),
            "type": t.get("type", "a"),
            "assigned_drone": t.get("assigned_drone"),
        })

    sams: List[Dict[str, Any]] = []
    for s in env.get("sams", []):
        # Handle different SAM formats: pos/position array or x/y fields
        pos = s.get("pos") or s.get("position")
        if pos is None and "x" in s and "y" in s:
            pos = [float(s["x"]), float(s["y"])]
        sams.append({
            "pos": pos,
            "range": float(s.get("range", s.get("radius", 15.0))),
            "id": s.get("id", "SAM"),
            # Add x,y for chain_rule compatibility
            "x": pos[0] if pos and len(pos) > 0 else None,
            "y": pos[1] if pos and len(pos) > 1 else None,
        })

    return airports, targets, sams


def solve_mission(
    env: Dict[str, Any],
    drone_configs: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Core entry point for the ISR web backend.

    Inputs:
      - env: environment JSON from the frontend
      - drone_configs: {
            "1": { "fuel_budget": 140, "start_airport": "A1", "end_airport": "A1", ... },
            ...
        }

    Outputs:
      {
        "sequences": { "1": "A1,T3,T11,A1", "2": "...", ... },
        "routes": {
           "1": {
               "route": ["A1","T3","T11","A1"],
               "sequence": "A1,T3,T11,A1",
               "points": 23,
               "distance": 87.4,
               "fuel_budget": 140.0
           },
           ...
        }
      }
    """
    print(f"\n🚀 solve_mission() called with {len(drone_configs)} drone configs", flush=True)

    airports, targets, sams = _parse_env_for_solver(env)
    print(f"📍 Parsed: {len(airports)} airports, {len(targets)} targets, {len(sams)} SAMs", flush=True)

    if not airports or not targets:
        print("⚠️ No airports or targets, returning empty", flush=True)
        return {
            "sequences": {},
            "routes": {},
        }

    # Build a global distance matrix; we'll reuse it per drone
    # TEMPORARILY using simple Euclidean distances (SAM-aware matrix is too slow)
    # TODO: Optimize calculate_sam_aware_matrix performance
    dist_data = _build_distance_matrix(airports, targets)
    if sams:
        print("🎯 SAMs present - using Euclidean distances (SAM avoidance applied to trajectory only)", flush=True)

    sequences: Dict[str, str] = {}
    routes_detail: Dict[str, Dict[str, Any]] = {}

    # For now we assume 4 drones max; later you can extend to N
    max_drone_id = max([int(did) for did in drone_configs.keys()] or [4])

    # Check if targets are pre-assigned
    any_assigned = any(t.get("assigned_drone") is not None for t in targets)

    # Track allocations for return value
    final_allocations: Dict[str, List[str]] = {}

    # If no pre-assignment, use the allocator to distribute targets to drones
    # This prevents each drone from solving ALL targets (exponential blowup)
    if not any_assigned:
        # Get enabled drones
        enabled_drones = [
            str(did_int) for did_int in range(1, max_drone_id + 1)
            if drone_configs.get(str(did_int), {}).get("enabled") is not False
        ]

        if len(enabled_drones) > 1:
            # Use the proper allocator with balanced strategy for even distribution
            allocations = allocate_targets(env, drone_configs, "balanced", dist_data)
            final_allocations = allocations

            # Debug: Print allocation sets
            print("\n" + "="*60, flush=True)
            print("🎯 TARGET ALLOCATION RESULTS (balanced strategy)", flush=True)
            print("="*60, flush=True)
            total_allocated = 0
            for did in sorted(allocations.keys(), key=lambda x: int(x)):
                target_ids = allocations[did]
                total_allocated += len(target_ids)
                if target_ids:
                    print(f"  Drone {did}: {target_ids}", flush=True)
                else:
                    cfg = drone_configs.get(did, {})
                    if cfg.get("enabled") is not False:
                        print(f"  Drone {did}: [] (enabled but no targets)", flush=True)
            print(f"  Total targets allocated: {total_allocated}/{len(targets)}", flush=True)
            print("="*60 + "\n", flush=True)

            # Apply allocations to targets
            for did, target_ids in allocations.items():
                for tid in target_ids:
                    for t in targets:
                        if str(t["id"]) == tid:
                            t["assigned_drone"] = did
                            break
            any_assigned = True

    for did_int in range(1, max_drone_id + 1):
        did = str(did_int)
        cfg = drone_configs.get(did, {})

        # Skip drones that are "disabled" (caller can enforce that)
        if cfg.get("enabled") is False:
            sequences[did] = ""
            routes_detail[did] = {
                "route": [],
                "sequence": "",
                "points": 0,
                "distance": 0.0,
                "fuel_budget": float(cfg.get("fuel_budget", 999.0)),
            }
            continue

        fuel_budget = float(cfg.get("fuel_budget", 999.0))

        # Start/end airports: default A1, A2, A3, A4 if they exist,
        # otherwise fall back to the first airport.
        airport_by_id = {a["id"]: a for a in airports}
        default_start = f"A{did_int}"
        start_id = cfg.get("start_airport") or (
            default_start if default_start in airport_by_id else airports[0]["id"]
        )
        end_id = cfg.get("end_airport") or start_id

        # Filter targets by type access (A, B, C, D types)
        target_access = cfg.get("target_access", {})
        allowed_types = {
            t_type.lower() for t_type, enabled in target_access.items()
            if enabled and isinstance(t_type, str)
        }

        # If no types specified, allow all types for backward compatibility
        if not allowed_types:
            allowed_types = {"a", "b", "c", "d"}

        # Filter targets by allowed types
        candidate_targets = [
            t for t in targets
            if str(t.get("type", "a")).lower() in allowed_types
        ]

        # Additionally filter by assigned_drone if any targets have assignments
        any_assigned = any(t.get("assigned_drone") is not None for t in targets)
        if any_assigned:
            candidate_targets = [
                t for t in candidate_targets
                if t.get("assigned_drone") is None or str(t.get("assigned_drone")) == did
            ]

        if not candidate_targets:
            # No targets: trivial route start->end
            if start_id and end_id:
                route_ids = [start_id, end_id]
            else:
                route_ids = []
            seq = ",".join(route_ids) if route_ids else ""
            sequences[did] = seq
            routes_detail[did] = {
                "route": route_ids,
                "sequence": seq,
                "points": 0,
                "distance": 0.0,
                "fuel_budget": fuel_budget,
            }
            print(f"🚁 Drone {did}: No targets assigned, trivial route", flush=True)
            continue

        # Debug: Print drone solve info
        target_ids = [t["id"] for t in candidate_targets]
        print(f"🚁 Drone {did}: Solving with {len(candidate_targets)} targets: {target_ids}", flush=True)

        # Build a FILTERED distance matrix containing only airports + this drone's targets
        # This is critical - the solver uses matrix_labels to determine which nodes to visit
        airport_ids = [a["id"] for a in airports]
        desired_ids = airport_ids + target_ids

        # Create index mapping from original labels
        orig_labels = dist_data["labels"]
        orig_matrix = dist_data["matrix"]

        # Build filtered matrix - ONLY include IDs that are actually in orig_labels
        # This ensures labels and matrix dimensions match
        filtered_indices = []
        filtered_ids = []  # Track which IDs are actually included
        for fid in desired_ids:
            if fid in orig_labels:
                filtered_indices.append(orig_labels.index(fid))
                filtered_ids.append(fid)

        n = len(filtered_indices)
        filtered_matrix = [[0.0] * n for _ in range(n)]
        for i, orig_i in enumerate(filtered_indices):
            for j, orig_j in enumerate(filtered_indices):
                filtered_matrix[i][j] = orig_matrix[orig_i][orig_j]

        filtered_dist_data = {
            "matrix": filtered_matrix,
            "labels": filtered_ids,  # Use the filtered list, not desired_ids
            "waypoints": [wp for wp in dist_data.get("waypoints", []) if wp["id"] in filtered_ids],
            "excluded_targets": [],
        }

        # Count actual airports and targets in filtered list
        actual_airport_count = sum(1 for fid in filtered_ids if fid in airport_ids)
        actual_target_count = len(filtered_ids) - actual_airport_count
        print(f"   📊 Filtered matrix: {len(filtered_ids)} nodes ({actual_airport_count} airports + {actual_target_count} targets)", flush=True)

        # Build a per-drone environment using your existing interface
        env_for_solver = _solver.build_environment_for_solver(
            airports=airports,
            targets=candidate_targets,
            sams=sams,
            distance_matrix_data=filtered_dist_data,
        )
        env_for_solver["start_airport"] = start_id
        env_for_solver["end_airport"] = end_id

        # Call your real orienteering solver
        solve_start = time.time()
        sol = _solver.solve(env_for_solver, fuel_budget)
        solve_time = time.time() - solve_start

        # The solver returns 'visited_goals' (target IDs only), not 'route'
        # We need to build the full route: [start_airport, ...targets..., end_airport]
        visited_targets: List[str] = sol.get("visited_goals", [])
        route_ids: List[str] = [start_id] + visited_targets + [end_id]

        # Calculate total points from visited targets
        target_priorities = {t["id"]: int(t.get("priority", 1)) for t in candidate_targets}
        total_points: int = sum(target_priorities.get(tid, 0) for tid in visited_targets)
        travel_distance: float = float(sol.get("distance", 0.0))

        print(f"   ✅ Drone {did} solved in {solve_time:.2f}s: {len(route_ids)} waypoints, {total_points} pts, {travel_distance:.1f} dist", flush=True)

        seq = ",".join(route_ids) if route_ids else ""

        # Generate trajectory using clean ISRTrajectoryPlanner (like Delivery Planner)
        trajectory_planner = ISRTrajectoryPlanner(sams)
        waypoint_positions = {wp["id"]: [wp["x"], wp["y"]] for wp in dist_data.get("waypoints", [])}
        trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions)

        print(f"   🛫 Generated trajectory: {len(trajectory)} points for route {route_ids}", flush=True)

        sequences[did] = seq
        routes_detail[did] = {
            "route": route_ids,
            "sequence": seq,
            "points": total_points,
            "distance": travel_distance,
            "fuel_budget": fuel_budget,
            "trajectory": trajectory,  # Full path with SAM avoidance
        }

    return {
        "sequences": sequences,
        "routes": routes_detail,
        "allocations": final_allocations,
    }


def solve_mission_with_allocation(
    env: Dict[str, Any],
    drone_configs: Dict[str, Any],
    allocation_strategy: str = "efficient",
    use_sam_aware_distances: bool = False,  # Default False for speed - SAM avoidance at trajectory time
    post_optimize: bool = True,
) -> Dict[str, Any]:
    """
    Enhanced mission solver with allocation, SAM-aware distances, and post-optimization.

    This is the recommended entry point for multi-drone mission solving.

    Pipeline:
    1. Calculate SAM-aware distance matrix (if enabled)
    2. Allocate targets to drones using specified strategy
    3. Solve orienteering for each drone with assigned targets
    4. Post-optimize to include any unassigned targets (if enabled)

    Args:
        env: Environment JSON from frontend
        drone_configs: Drone configurations
        allocation_strategy: "greedy", "balanced", "efficient", "geographic", "exclusive"
        use_sam_aware_distances: Whether to calculate SAM-aware distances
        post_optimize: Whether to run post-optimization for unvisited targets

    Returns:
        Solution dict with sequences and routes for each drone
    """
    global _cached_env_hash

    airports, targets, sams = _parse_env_for_solver(env)

    if not airports or not targets:
        return {
            "sequences": {},
            "routes": {},
        }

    # Step 1: Get distance matrix (use cached if available)
    if use_sam_aware_distances and sams:
        # First check if we have a cached SAM-aware matrix
        cached_matrix = get_cached_matrix()
        if cached_matrix is not None:
            print("⚡ Using cached SAM-aware distance matrix", flush=True)
            dist_data = cached_matrix
        else:
            # No cache - calculate SAM-aware distance matrix
            print("⏳ Calculating SAM-aware distance matrix (no cache available)...", flush=True)
            dist_data = calculate_sam_aware_matrix(env)
        set_allocator_matrix(dist_data)
        set_optimizer_matrix(dist_data)
    else:
        # Use simple Euclidean distances
        dist_data = _build_distance_matrix(airports, targets)

    # Step 2: Allocate targets to drones
    allocations = allocate_targets(
        env, drone_configs, allocation_strategy, dist_data
    )

    # Log allocation results
    print("\n" + "="*60, flush=True)
    print(f"🎯 TARGET ALLOCATION RESULTS (strategy: {allocation_strategy})", flush=True)
    print("="*60, flush=True)
    total_allocated = 0
    for did in sorted(allocations.keys(), key=lambda x: int(x)):
        target_ids = allocations[did]
        total_allocated += len(target_ids)
        cfg = drone_configs.get(did, {})
        if cfg.get("enabled") is not False:
            if target_ids:
                print(f"  Drone {did}: {target_ids}", flush=True)
            else:
                print(f"  Drone {did}: [] (enabled but no targets)", flush=True)
    print(f"  Total targets allocated: {total_allocated}/{len(targets)}", flush=True)
    print("="*60 + "\n", flush=True)

    # Step 3: Solve for each drone with their assigned targets
    sequences: Dict[str, str] = {}
    routes_detail: Dict[str, Dict[str, Any]] = {}

    max_drone_id = max([int(did) for did in drone_configs.keys()] or [4])

    for did_int in range(1, max_drone_id + 1):
        did = str(did_int)
        cfg = drone_configs.get(did, {})

        # Skip disabled drones
        if cfg.get("enabled") is False:
            sequences[did] = ""
            routes_detail[did] = {
                "route": [],
                "sequence": "",
                "points": 0,
                "distance": 0.0,
                "fuel_budget": float(cfg.get("fuel_budget", 999.0)),
            }
            continue

        fuel_budget = float(cfg.get("fuel_budget", 999.0))

        # Get start/end airports
        airport_by_id = {a["id"]: a for a in airports}
        default_start = f"A{did_int}"
        start_id = cfg.get("start_airport") or (
            default_start if default_start in airport_by_id else airports[0]["id"]
        )
        end_id = cfg.get("end_airport") or start_id

        # Get this drone's assigned targets
        assigned_target_ids = set(allocations.get(did, []))

        if not assigned_target_ids:
            # No targets assigned: trivial route from start to end airport
            route_ids = [start_id, end_id] if start_id and end_id else []
            seq = ",".join(route_ids)

            # Still need to generate SAM-avoiding trajectory for airport-to-airport!
            trajectory = []
            if route_ids and len(route_ids) >= 2 and sams:
                trajectory_planner = ISRTrajectoryPlanner(sams)
                waypoint_positions = {wp["id"]: [wp["x"], wp["y"]] for wp in dist_data.get("waypoints", [])}
                trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions, drone_id=did)

            sequences[did] = seq
            routes_detail[did] = {
                "route": route_ids,
                "sequence": seq,
                "points": 0,
                "distance": 0.0,
                "fuel_budget": fuel_budget,
                "trajectory": trajectory,  # SAM-avoiding path even for no-target routes
            }
            continue

        # Filter targets to only those assigned to this drone
        candidate_targets = [
            t for t in targets
            if str(t["id"]) in assigned_target_ids
        ]

        # CRITICAL: Limit targets to prevent exponential solver blowup
        # Orienteering solver has O(n!) complexity - cap at 12 targets max
        MAX_TARGETS_PER_SOLVE = 12
        if len(candidate_targets) > MAX_TARGETS_PER_SOLVE:
            # Sort by priority (highest first) and take top N
            candidate_targets = sorted(
                candidate_targets,
                key=lambda t: int(t.get("priority", 5)),
                reverse=True
            )[:MAX_TARGETS_PER_SOLVE]
            print(f"  ⚠️ Drone {did}: Limited to {MAX_TARGETS_PER_SOLVE} highest-priority targets (had {len(assigned_target_ids)})", flush=True)

        # Debug: Print drone solve info
        target_ids = [t["id"] for t in candidate_targets]
        print(f"🚁 Drone {did}: Solving with {len(candidate_targets)} targets: {target_ids}", flush=True)

        # Build a FILTERED distance matrix containing only airports + this drone's targets
        # This is critical - the solver uses matrix_labels to determine which nodes to visit
        airport_ids = [a["id"] for a in airports]
        desired_ids = airport_ids + target_ids

        # Create index mapping from original labels
        orig_labels = dist_data["labels"]
        orig_matrix = dist_data["matrix"]

        # Build filtered matrix - ONLY include IDs that are actually in orig_labels
        # This ensures labels and matrix dimensions match
        filtered_indices = []
        filtered_ids = []  # Track which IDs are actually included
        for fid in desired_ids:
            if fid in orig_labels:
                filtered_indices.append(orig_labels.index(fid))
                filtered_ids.append(fid)

        n = len(filtered_indices)
        filtered_matrix = [[0.0] * n for _ in range(n)]
        for i, orig_i in enumerate(filtered_indices):
            for j, orig_j in enumerate(filtered_indices):
                filtered_matrix[i][j] = orig_matrix[orig_i][orig_j]

        filtered_dist_data = {
            "matrix": filtered_matrix,
            "labels": filtered_ids,  # Use the filtered list, not desired_ids
            "waypoints": [wp for wp in dist_data.get("waypoints", []) if wp["id"] in filtered_ids],
            "excluded_targets": [],
        }

        # Count actual airports and targets in filtered list
        actual_airport_count = sum(1 for fid in filtered_ids if fid in airport_ids)
        actual_target_count = len(filtered_ids) - actual_airport_count
        print(f"   📊 Filtered matrix: {len(filtered_ids)} nodes ({actual_airport_count} airports + {actual_target_count} targets)", flush=True)

        # Build environment for solver with FILTERED matrix
        env_for_solver = _solver.build_environment_for_solver(
            airports=airports,
            targets=candidate_targets,
            sams=sams,
            distance_matrix_data=filtered_dist_data,
        )
        env_for_solver["start_airport"] = start_id
        env_for_solver["end_airport"] = end_id

        # Solve orienteering for this drone
        sol = _solver.solve(env_for_solver, fuel_budget)

        # The solver returns BOTH 'visited_goals' (unordered target IDs) AND 'route' (optimized order)
        # CRITICAL: Use 'route' for the optimized sequence from Held-Karp, NOT visited_goals!
        visited_targets: List[str] = sol.get("visited_goals", [])

        # Use the optimized route directly from solver (includes start/end airports in optimal order)
        route_ids: List[str] = sol.get("route", [])
        if not route_ids:
            # Fallback if solver didn't return route
            route_ids = [start_id] + visited_targets + [end_id]

        # Calculate total points from visited targets
        target_priorities = {t["id"]: int(t.get("priority", 1)) for t in candidate_targets}
        total_points: int = sum(target_priorities.get(tid, 0) for tid in visited_targets)
        travel_distance: float = float(sol.get("distance", 0.0))

        # DEBUG: Log solver output
        print(f"🔍 [DEBUG] Drone {did} solver returned:", flush=True)
        print(f"   route_ids = {route_ids}", flush=True)
        print(f"   total_points = {total_points}, distance = {travel_distance:.2f}", flush=True)

        seq = ",".join(route_ids) if route_ids else ""

        # Generate SAM-avoiding trajectory using ISRTrajectoryPlanner
        trajectory_planner = ISRTrajectoryPlanner(sams)
        waypoint_positions = {wp["id"]: [wp["x"], wp["y"]] for wp in dist_data.get("waypoints", [])}
        trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions, drone_id=did)

        sequences[did] = seq
        routes_detail[did] = {
            "route": route_ids,
            "sequence": seq,
            "points": total_points,
            "distance": travel_distance,
            "fuel_budget": fuel_budget,
            "trajectory": trajectory,  # Full path with SAM avoidance
        }

    # Compute wrapped polygons for visualization (same as delivery system)
    wrapped_polygons = []
    if sams:
        # Convert SAMs to simple format for wrapping
        sams_for_wrapping = []
        for sam in sams:
            pos = sam.get("pos") or sam.get("position")
            if pos:
                sams_for_wrapping.append({
                    'x': pos[0] if isinstance(pos, (list, tuple)) else pos,
                    'y': pos[1] if isinstance(pos, (list, tuple)) else pos,
                    'radius': float(sam.get("range", sam.get("radius", 15)))
                })
        if sams_for_wrapping:
            wrapped_polygon_arrays, _ = wrap_sams(sams_for_wrapping)
            wrapped_polygons = []
            for poly in wrapped_polygon_arrays:
                # If it's a NumPy array, convert to list; if it's already a list, keep it
                if hasattr(poly, "tolist"):
                    wrapped_polygons.append(poly.tolist())
                else:
                    wrapped_polygons.append(poly)

    # Build initial solution
    solution = {
        "sequences": sequences,
        "routes": routes_detail,
        "wrapped_polygons": wrapped_polygons,
        "allocations": allocations,  # initial allocator view
    }

    # Step 4: Post-optimize to include unvisited targets
    if post_optimize:
        solution = post_optimize_solution(solution, env, drone_configs, dist_data)

    # 🔁 IMPORTANT: recompute allocations from FINAL routes so UI + logs agree
    final_allocations: Dict[str, List[str]] = {}
    for did, route_data in solution.get("routes", {}).items():
        route = route_data.get("route", [])
        final_allocations[did] = [
            node_id for node_id in route
            if isinstance(node_id, str) and node_id.startswith("T")
        ]

    # Optional: debug print to compare initial vs final
    print("\n" + "="*60, flush=True)
    print("🎯 FINAL ALLOCATIONS (derived from routes)", flush=True)
    print("="*60, flush=True)
    total_final = 0
    for did in sorted(final_allocations.keys(), key=lambda x: int(x)):
        tids = final_allocations[did]
        total_final += len(tids)
        print(f"  Drone {did}: {tids}", flush=True)
    print(f"  Total targets in final routes: {total_final}", flush=True)
    print("="*60 + "\n", flush=True)

    # Overwrite allocations with the final view
    solution["allocations"] = final_allocations

    return solution

def prepare_distance_matrix(env: Dict[str, Any], buffer: float = 0.0) -> Dict[str, Any]:
    """
    Pre-calculate SAM-aware distance matrix for an environment.

    Call this after editing is complete to have the matrix ready for solving.

    Args:
        env: Environment data with airports, targets, sams
        buffer: Safety buffer around SAMs (default 0.0)

    Returns:
        Distance matrix data dict
    """
    return calculate_sam_aware_matrix(env, buffer)


def get_current_matrix() -> Dict[str, Any]:
    """Get the currently cached distance matrix."""
    return get_cached_matrix()


def clear_cached_matrix():
    """Clear the cached distance matrix."""
    clear_matrix_cache()
