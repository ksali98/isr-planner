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

# Add /app path for Docker deployment
if "/app" not in sys.path:
    sys.path.insert(0, "/app")

# Use the modular solver - import directly to avoid __init__.py pulling in matplotlib
# Try importing from different paths (Docker vs local)
OrienteeringSolverInterface = None
try:
    # Docker path: /app/webapp/editor/solver/...
    from webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface
except ImportError as e:
    try:
        # Local development path: isr_web/webapp/editor/solver/...
        from isr_web.webapp.editor.solver.orienteering_interface import OrienteeringSolverInterface  # type: ignore
    except ImportError as e2:

# Import new solver components
from .sam_distance_matrix import (
    calculate_sam_aware_matrix,
    get_cached_matrix,
    clear_matrix_cache,
)
from .target_allocator import allocate_targets, set_allocator_matrix, clear_allocator_matrix
from .post_optimizer import post_optimize_solution, set_optimizer_matrix
from .trajectory_planner import ISRTrajectoryPlanner

# Import polygon wrapping for visualization
from path_planning_core.sam_wrapping import wrap_sams

# Single global solver instance (only if interface is available)
_solver = None
if OrienteeringSolverInterface is not None:
    _solver = OrienteeringSolverInterface()
else:

# Cached distance matrix for current environment
_cached_env_hash: Optional[str] = None


def _compute_env_hash(env: Dict[str, Any]) -> str:
    """
    Compute a hash of the environment to detect changes.
    Uses airport/target/SAM IDs and positions to detect when cache is stale.
    """
    import hashlib
    import json

    # Extract key data that affects the distance matrix
    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    def get_pos(obj):
        """Get position from object - handles both x/y and pos/position formats."""
        if "x" in obj and "y" in obj:
            return (round(float(obj["x"]), 6), round(float(obj["y"]), 6))
        pos = obj.get("pos") or obj.get("position")
        if pos and len(pos) >= 2:
            return (round(float(pos[0]), 6), round(float(pos[1]), 6))
        return (0, 0)

    # Build a canonical representation
    key_data = {
        "airports": sorted([
            (a.get("id", ""), get_pos(a))
            for a in airports
        ]),
        "targets": sorted([
            (t.get("id", ""), get_pos(t))
            for t in targets
        ]),
        "sams": sorted([
            (s.get("id", ""), get_pos(s), round(float(s.get("range", s.get("radius", 0))), 2))
            for s in sams
        ]),
    }

    # Create hash
    data_str = json.dumps(key_data, sort_keys=True)
    return hashlib.md5(data_str.encode()).hexdigest()


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

    Also handles synthetic_starts for checkpoint replanning - these are added
    to the airports list so drones can start from mid-trajectory positions.
    """
    airports: List[Dict[str, Any]] = []
    for a in env.get("airports", []):
        # Handle both formats: x/y fields or pos array
        if "x" in a and "y" in a:
            ax, ay = float(a["x"]), float(a["y"])
        elif "pos" in a:
            ax, ay = float(a["pos"][0]), float(a["pos"][1])
        else:
            raise ValueError(f"Airport {a.get('id')} missing position (need x/y or pos)")
        airport_entry = {
            "id": str(a["id"]),
            "x": ax,
            "y": ay,
        }
        # CRITICAL: Preserve is_synthetic flag from frontend
        # This flag marks synthetic starts (D1_START, D2_START, etc.) added during checkpoint replanning
        # Synthetic starts can be START points but should NEVER be valid ENDpoints
        if a.get("is_synthetic", False):
            airport_entry["is_synthetic"] = True
        airports.append(airport_entry)

    # Add synthetic start nodes (for checkpoint replanning)
    # These act like airports - drones can start from these positions
    for node_id, node_data in env.get("synthetic_starts", {}).items():
        airports.append({
            "id": str(node_id),
            "x": float(node_data["x"]),
            "y": float(node_data["y"]),
            "is_synthetic": True,  # Mark as synthetic for reference
        })

    # Filter out visited targets for checkpoint replanning
    visited_target_ids = set(str(tid) for tid in env.get("visited_targets", []))

    targets: List[Dict[str, Any]] = []
    for t in env.get("targets", []):
        target_id = str(t["id"])
        # Skip visited targets - they are already completed in previous segments
        if target_id in visited_target_ids:
            continue

        # Handle both formats: x/y fields or pos array
        if "x" in t and "y" in t:
            tx, ty = float(t["x"]), float(t["y"])
        elif "pos" in t:
            tx, ty = float(t["pos"][0]), float(t["pos"][1])
        else:
            raise ValueError(f"Target {target_id} missing position (need x/y or pos)")

        targets.append({
            "id": target_id,
            "x": tx,
            "y": ty,
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

    airports, targets, sams = _parse_env_for_solver(env)

    # DEBUG: Show which airports have is_synthetic flag
    synthetic_airports = [a["id"] for a in airports if a.get("is_synthetic", False)]
    real_airports = [a["id"] for a in airports if not a.get("is_synthetic", False)]

    if not airports or not targets:
        return {
            "sequences": {},
            "routes": {},
        }

    # Build a global distance matrix; we'll reuse it per drone
    # ALWAYS use SAM-aware distances for consistency and exclusion detection
    sam_count = len(sams) if sams else 0
    clear_matrix_cache()
    dist_data = calculate_sam_aware_matrix(env)

    # Get excluded targets (inside SAM polygons)
    excluded_targets = set(dist_data.get("excluded_targets", []))
    if excluded_targets:

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
            total_allocated = 0
            for did in sorted(allocations.keys(), key=lambda x: int(x)):
                target_ids = allocations[did]
                total_allocated += len(target_ids)
                if target_ids:
                else:
                    cfg = drone_configs.get(did, {})
                    if cfg.get("enabled") is not False:

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

        # DEBUG: Check what start_airport is in the config
        cfg_start = cfg.get("start_airport")

        # Check if cfg_start is a synthetic start (e.g., D1_START for checkpoint replanning)
        # Synthetic starts should be used directly, not fall back to real airports
        is_synthetic_start = cfg_start and cfg_start.endswith("_START") and cfg_start not in airport_by_id

        if is_synthetic_start:
            # Use the synthetic start directly - it's in matrix_labels but not airports
            start_id = cfg_start
        else:
            start_id = cfg_start or (
                default_start if default_start in airport_by_id else airports[0]["id"]
            )

        # DEBUG: Final start_id decision

        # Handle flexible endpoint: "-" means solver chooses optimal endpoint
        raw_end_id = cfg.get("end_airport") or start_id
        flexible_endpoint = (raw_end_id == "-")
        # Don't set end_id to start_id for flexible endpoints - solver will choose
        end_id = raw_end_id if not flexible_endpoint else None


        # Filter targets by type access (A, B, C, D types)
        target_access = cfg.get("target_access", {})
        allowed_types = {
            t_type.lower() for t_type, enabled in target_access.items()
            if enabled and isinstance(t_type, str)
        }

        # If no types specified, allow all types for backward compatibility
        if not allowed_types:
            allowed_types = {"a", "b", "c", "d"}

        # Filter targets by allowed types AND exclude those inside SAM polygons
        candidate_targets = [
            t for t in targets
            if str(t.get("type", "a")).lower() in allowed_types
            and str(t.get("id", "")) not in excluded_targets
        ]

        # DEBUG: Check if any excluded targets would have been included
        excluded_here = [t["id"] for t in targets if str(t.get("id", "")) in excluded_targets]
        if excluded_here:

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
            continue

        # Debug: Print drone solve info
        target_ids = [t["id"] for t in candidate_targets]

        # Build a FILTERED distance matrix containing only airports + this drone's targets
        # This is critical - the solver uses matrix_labels to determine which nodes to visit
        airport_ids = [a["id"] for a in airports]
        desired_ids = airport_ids + target_ids

        # Create index mapping from original labels
        orig_labels = dist_data["labels"]
        orig_matrix = dist_data["matrix"]

        # Build filtered matrix - ONLY include IDs that are actually in orig_labels
        # This ensures labels and matrix dimensions match
        # CRITICAL: Always include start_id and end_id
        filtered_indices = []
        filtered_ids = []  # Track which IDs are actually included
        for fid in desired_ids:
            if fid in orig_labels:
                filtered_indices.append(orig_labels.index(fid))
                filtered_ids.append(fid)

        # Ensure start and end airports are ALWAYS in the filtered matrix
        # (skip end_id if None, which happens for flexible endpoints)
        for required_id in [start_id, end_id]:
            if required_id and required_id not in filtered_ids and required_id in orig_labels:
                filtered_indices.append(orig_labels.index(required_id))
                filtered_ids.append(required_id)

        n = len(filtered_indices)
        filtered_matrix = [[0.0] * n for _ in range(n)]
        for i, orig_i in enumerate(filtered_indices):
            for j, orig_j in enumerate(filtered_indices):
                filtered_matrix[i][j] = orig_matrix[orig_i][orig_j]

        filtered_dist_data = {
            "matrix": filtered_matrix,
            "labels": filtered_ids,  # Use the filtered list, not desired_ids
            "waypoints": [wp for wp in dist_data.get("waypoints", []) if wp["id"] in filtered_ids],
            "excluded_targets": list(excluded_targets) if excluded_targets else [],  # Propagate excluded targets
        }

        # Count actual airports and targets in filtered list
        actual_airport_count = sum(1 for fid in filtered_ids if fid in airport_ids)
        actual_target_count = len(filtered_ids) - actual_airport_count

        # Build airports list for solver - start with real airports
        solver_airports = list(airports)  # Copy to avoid mutating original
        # CRITICAL: Real airports are those that are NOT synthetic (no is_synthetic flag AND not ending in _START)
        # Synthetic starts should NEVER be valid endpoints
        real_airport_ids = [
            a["id"] for a in airports
            if not a.get("is_synthetic", False)
            and not str(a.get("id", "")).endswith("_START")
        ]

        # CRITICAL FIX: Add ALL synthetic starts that are in the filtered matrix
        # This ensures every node in matrix_labels is classified as either airport or target
        # Without this, the solver guard will reject unknown labels like D1_START, D2_START
        waypoint_positions_for_synthetic = {wp["id"]: wp for wp in dist_data.get("waypoints", [])}
        for fid in filtered_ids:
            if fid.endswith("_START") and fid not in real_airport_ids:
                # This is a synthetic start - add it as an airport
                if fid in waypoint_positions_for_synthetic:
                    wp = waypoint_positions_for_synthetic[fid]
                    synthetic_airport = {
                        "id": fid,
                        "x": wp["x"],
                        "y": wp["y"],
                        "is_synthetic": True,  # Mark so we can filter for endpoints
                    }
                    solver_airports.append(synthetic_airport)
                else:


        # Build a per-drone environment using your existing interface
        env_for_solver = _solver.build_environment_for_solver(
            airports=solver_airports,  # ← Include synthetic starts as airports
            targets=candidate_targets,
            sams=sams,
            distance_matrix_data=filtered_dist_data,
        )
        env_for_solver["start_airport"] = start_id

        # Handle flexible endpoint vs fixed endpoint
        if flexible_endpoint:
            env_for_solver["mode"] = "best_end"

            # CRITICAL: Remove end_airport from env if it was set by build_environment_for_solver
            # This prevents the solver from using a synthetic start as the endpoint
            if "end_airport" in env_for_solver:
                del env_for_solver["end_airport"]

            # CRITICAL: Filter out synthetic starts from valid end airports
            # Synthetic starts (e.g., D1_START) are used for checkpoint replanning
            # but should NEVER be valid endpoints - only real airports should be endpoints
            # Check both the is_synthetic flag AND the ID pattern (ends with _START)
            real_airports = [
                a for a in airports
                if not a.get("is_synthetic", False)
                and not str(a.get("id", "")).endswith("_START")
            ]
            synthetic_airports = [
                a for a in airports
                if a.get("is_synthetic", False) or str(a.get("id", "")).endswith("_START")
            ]
            if real_airports:
                env_for_solver["valid_end_airports"] = [a["id"] for a in real_airports]
            else:
        else:
            # Fixed endpoint
            env_for_solver["end_airport"] = end_id
            env_for_solver["mode"] = "return" if end_id == start_id else "end"

        # Call your real orienteering solver
        solve_start = time.time()
        sol = _solver.solve(env_for_solver, fuel_budget)
        solve_time = time.time() - solve_start

        # The solver returns 'route' (optimal ordered path) and 'visited_targets' (target IDs)
        # Use 'route' directly as it contains the optimal ordering from Held-Karp
        route_ids: List[str] = sol.get("route", [])
        visited_targets: List[str] = sol.get("visited_targets", [])

        # Fallback if solver didn't return route
        if not route_ids:
            route_ids = [start_id] + visited_targets + [end_id]

        # Calculate total points from visited targets
        target_priorities = {t["id"]: int(t.get("priority", 1)) for t in candidate_targets}
        total_points: int = sum(target_priorities.get(tid, 0) for tid in visited_targets)
        travel_distance: float = float(sol.get("distance", 0.0))


        seq = ",".join(route_ids) if route_ids else ""

        # Generate trajectory using clean ISRTrajectoryPlanner (like Delivery Planner)
        trajectory_planner = ISRTrajectoryPlanner(sams)
        waypoint_positions = {wp["id"]: [wp["x"], wp["y"]] for wp in dist_data.get("waypoints", [])}
        trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions)


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

    # Step 1: Get distance matrix
    # ALWAYS use SAM-aware matrix calculator for consistency
    # This ensures targets inside SAM circles are always excluded
    # When no SAMs present, it still computes correct Euclidean distances
    current_hash = _compute_env_hash(env)

    # Always clear and recalculate to ensure fresh exclusion detection
    sam_count = len(sams) if sams else 0
    clear_matrix_cache()
    clear_allocator_matrix()  # Also clear allocator's cached matrix
    dist_data = calculate_sam_aware_matrix(env)
    _cached_env_hash = current_hash

    # Always set distance matrices for allocator and optimizer
    # Even Euclidean distances are needed for fuel budget calculations
    set_allocator_matrix(dist_data)
    set_optimizer_matrix(dist_data)

    # DEBUG: Log excluded targets from distance matrix
    excluded_targets = set(dist_data.get("excluded_targets", []))
    if excluded_targets:
    else:

    # Step 2: Allocate targets to drones
    allocations = allocate_targets(
        env, drone_configs, allocation_strategy, dist_data
    )

    # Log allocation results
    total_allocated = 0
    for did in sorted(allocations.keys(), key=lambda x: int(x)):
        target_ids = allocations[did]
        total_allocated += len(target_ids)
        cfg = drone_configs.get(did, {})
        if cfg.get("enabled") is not False:
            if target_ids:
            else:

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
        cfg_start = cfg.get("start_airport")

        # Check if cfg_start is a synthetic start (e.g., D1_START for checkpoint replanning)
        # Synthetic starts should be used directly, not fall back to real airports
        is_synthetic_start = cfg_start and cfg_start.endswith("_START") and cfg_start not in airport_by_id

        if is_synthetic_start:
            # Use the synthetic start directly - it's in matrix_labels but not airports
            start_id = cfg_start
        else:
            start_id = cfg_start or (
                default_start if default_start in airport_by_id else airports[0]["id"]
            )

        # Handle flexible endpoint: "-" means solver chooses optimal endpoint
        raw_end_id = cfg.get("end_airport") or start_id
        flexible_endpoint = (raw_end_id == "-")
        # Don't set end_id to start_id for flexible endpoints - solver will choose
        end_id = raw_end_id if not flexible_endpoint else None

        # Debug logging

        # Get this drone's assigned targets
        assigned_target_ids = set(allocations.get(did, []))

        if not assigned_target_ids:
            # No targets assigned: trivial route from start to end airport
            # For flexible endpoint with no targets, just return to start
            effective_end = start_id if flexible_endpoint else end_id
            route_ids = [start_id, effective_end] if start_id and effective_end else []
            seq = ",".join(route_ids)

            # Still need to generate trajectory for airport-to-airport route!
            trajectory = []
            route_distance = 0.0
            if route_ids and len(route_ids) >= 2:
                trajectory_planner = ISRTrajectoryPlanner(sams)
                waypoint_positions = {wp["id"]: [wp["x"], wp["y"]] for wp in dist_data.get("waypoints", [])}
                trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions, drone_id=did)

                # Calculate actual distance from trajectory waypoints
                if trajectory and len(trajectory) >= 2:
                    for i in range(len(trajectory) - 1):
                        dx = trajectory[i+1][0] - trajectory[i][0]
                        dy = trajectory[i+1][1] - trajectory[i][1]
                        route_distance += math.sqrt(dx*dx + dy*dy)

            sequences[did] = seq
            routes_detail[did] = {
                "route": route_ids,
                "sequence": seq,
                "points": 0,
                "distance": route_distance,
                "fuel_budget": fuel_budget,
                "trajectory": trajectory,
            }
            continue

        # Filter targets to only those assigned to this drone
        # CRITICAL: Also filter out excluded targets (inside SAM polygons)
        candidate_targets = [
            t for t in targets
            if str(t["id"]) in assigned_target_ids and str(t["id"]) not in excluded_targets
        ]

        # DEBUG: Check if any excluded targets were in the assignment
        excluded_in_assignment = assigned_target_ids & excluded_targets
        if excluded_in_assignment:

        # CRITICAL: Limit targets to prevent exponential solver blowup
        # Orienteering solver has O(n!) complexity - cap at reasonable max
        MAX_TARGETS_PER_SOLVE = 12
        if len(candidate_targets) > MAX_TARGETS_PER_SOLVE:
            # Sort by priority (highest first) and take top N
            candidate_targets = sorted(
                candidate_targets,
                key=lambda t: int(t.get("priority", 5)),
                reverse=True
            )[:MAX_TARGETS_PER_SOLVE]

        # Debug: Print drone solve info
        target_ids = [t["id"] for t in candidate_targets]

        # Build a FILTERED distance matrix containing only airports + this drone's targets
        # This is critical - the solver uses matrix_labels to determine which nodes to visit
        airport_ids = [a["id"] for a in airports]
        desired_ids = airport_ids + target_ids

        # Create index mapping from original labels
        orig_labels = dist_data["labels"]
        orig_matrix = dist_data["matrix"]

        # Build filtered matrix - ONLY include IDs that are actually in orig_labels
        # This ensures labels and matrix dimensions match
        # CRITICAL: Always include start_id and end_id
        filtered_indices = []
        filtered_ids = []  # Track which IDs are actually included
        for fid in desired_ids:
            if fid in orig_labels:
                filtered_indices.append(orig_labels.index(fid))
                filtered_ids.append(fid)

        # Ensure start and end airports are ALWAYS in the filtered matrix
        # (skip end_id if None, which happens for flexible endpoints)
        for required_id in [start_id, end_id]:
            if required_id and required_id not in filtered_ids and required_id in orig_labels:
                filtered_indices.append(orig_labels.index(required_id))
                filtered_ids.append(required_id)

        n = len(filtered_indices)
        filtered_matrix = [[0.0] * n for _ in range(n)]
        for i, orig_i in enumerate(filtered_indices):
            for j, orig_j in enumerate(filtered_indices):
                filtered_matrix[i][j] = orig_matrix[orig_i][orig_j]

        filtered_dist_data = {
            "matrix": filtered_matrix,
            "labels": filtered_ids,  # Use the filtered list, not desired_ids
            "waypoints": [wp for wp in dist_data.get("waypoints", []) if wp["id"] in filtered_ids],
            "excluded_targets": list(excluded_targets) if excluded_targets else [],  # Propagate excluded targets
        }

        # Count actual airports and targets in filtered list
        actual_airport_count = sum(1 for fid in filtered_ids if fid in airport_ids)
        actual_target_count = len(filtered_ids) - actual_airport_count

        # Build airports list for solver - start with real airports
        solver_airports = list(airports)  # Copy to avoid mutating original
        # CRITICAL: Real airports are those that are NOT synthetic (no is_synthetic flag AND not ending in _START)
        # Synthetic starts should NEVER be valid endpoints
        real_airport_ids = [
            a["id"] for a in airports
            if not a.get("is_synthetic", False)
            and not str(a.get("id", "")).endswith("_START")
        ]

        # CRITICAL FIX: Add ALL synthetic starts that are in the filtered matrix
        # This ensures every node in matrix_labels is classified as either airport or target
        # Without this, the solver guard will reject unknown labels like D1_START, D2_START
        waypoint_positions_for_synthetic = {wp["id"]: wp for wp in dist_data.get("waypoints", [])}
        for fid in filtered_ids:
            if fid.endswith("_START") and fid not in real_airport_ids:
                # This is a synthetic start - add it as an airport
                if fid in waypoint_positions_for_synthetic:
                    wp = waypoint_positions_for_synthetic[fid]
                    synthetic_airport = {
                        "id": fid,
                        "x": wp["x"],
                        "y": wp["y"],
                        "is_synthetic": True,  # Mark so we can filter for endpoints
                    }
                    solver_airports.append(synthetic_airport)
                else:


        # Build environment for solver with FILTERED matrix
        env_for_solver = _solver.build_environment_for_solver(
            airports=solver_airports,  # ← Include synthetic starts as airports
            targets=candidate_targets,
            sams=sams,
            distance_matrix_data=filtered_dist_data,
        )
        env_for_solver["start_airport"] = start_id


        # If flexible endpoint, use best_end mode to let solver choose optimal end
        if flexible_endpoint:
            env_for_solver["mode"] = "best_end"

            # CRITICAL: Remove end_airport from env if it was set by build_environment_for_solver
            # This prevents the solver from using a synthetic start as the endpoint
            if "end_airport" in env_for_solver:
                del env_for_solver["end_airport"]

            # CRITICAL: Filter out synthetic starts from valid end airports
            # Synthetic starts (e.g., D1_START) are used for checkpoint replanning
            # but should NOT be valid endpoints - only real airports should be endpoints
            # Use real_airport_ids we defined earlier (before adding synthetic starts)
            if real_airport_ids:
                env_for_solver["valid_end_airports"] = real_airport_ids

            # Don't set end_airport - let solver choose from valid_end_airports
            sol = _solver.solve(env_for_solver, fuel_budget)
            # Extract the end airport chosen by solver
            end_id = sol.get("end_airport", start_id)
        else:
            env_for_solver["end_airport"] = end_id
            env_for_solver["mode"] = "return" if end_id == start_id else "end"
            # Solve orienteering for this drone
            sol = _solver.solve(env_for_solver, fuel_budget)

        # The solver returns BOTH 'visited_targets' (unordered target IDs) AND 'route' (optimized order)
        # CRITICAL: Use 'route' for the optimized sequence from Held-Karp, NOT visited_targets!
        visited_targets: List[str] = sol.get("visited_targets", [])

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

        # Validate: check if route ends at expected endpoint
        if route_ids:
            actual_end = route_ids[-1]
            expected_end = end_id if end_id else start_id  # flexible endpoint defaults to start
            if actual_end != expected_end:
            else:

        seq = ",".join(route_ids) if route_ids else ""

        # Generate SAM-avoiding trajectory using ISRTrajectoryPlanner
        trajectory_planner = ISRTrajectoryPlanner(sams)
        waypoint_positions = {wp["id"]: [wp["x"], wp["y"]] for wp in dist_data.get("waypoints", [])}
        trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions, drone_id=did)

        # DEBUG: Log trajectory endpoints vs expected airport position
        if trajectory and len(trajectory) >= 2:
            traj_start = trajectory[0]
            traj_end = trajectory[-1]
            # Find expected airport position
            if route_ids:
                end_id_in_route = route_ids[-1]
                expected_pos = waypoint_positions.get(end_id_in_route)
                if expected_pos:
                    end_diff = ((traj_end[0] - expected_pos[0])**2 + (traj_end[1] - expected_pos[1])**2)**0.5
                    if end_diff > 1.0:  # More than 1 unit off
                    else:

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
        "distance_matrix": dist_data,  # Include distance matrix for downstream use
    }

    # Step 4: Post-optimize to include unvisited targets
    if post_optimize:
        solution = post_optimize_solution(solution, env, drone_configs, dist_data)

    # Compute route-derived allocations for comparison (what's actually in trajectories)
    route_allocations: Dict[str, List[str]] = {}
    for did, route_data in solution.get("routes", {}).items():
        route = route_data.get("route", [])
        route_allocations[did] = [
            node_id for node_id in route
            if isinstance(node_id, str) and node_id.startswith("T")
        ]

    # Debug print to compare original allocator vs route-derived
    total_in_routes = 0
    for did in sorted(route_allocations.keys(), key=lambda x: int(x)):
        tids = route_allocations[did]
        total_in_routes += len(tids)

    # Keep the ORIGINAL allocator assignments (not route-derived)
    # UI will show what the allocator assigned, even if solver dropped some
    # Also include route_allocations for reference
    solution["route_allocations"] = route_allocations

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
