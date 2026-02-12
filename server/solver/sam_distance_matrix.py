"""
SAM-Aware Distance Matrix Calculator

Calculates distances between all waypoints (airports + targets) that account
for SAM avoidance paths. This should be called once after editing is complete
so the matrix is ready when solve requests come in.

This is a LangGraph-compatible tool.
"""

import sys
import math
import numpy as np
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional
from functools import lru_cache

# Add paths for SAM navigation modules
legacy_path = Path(__file__).resolve().parents[3] / "legacy" / "isr_legacy_all"
if str(legacy_path) not in sys.path:
    sys.path.insert(0, str(legacy_path))

root_path = Path(__file__).resolve().parents[3]
if str(root_path) not in sys.path:
    sys.path.insert(0, str(root_path))

# Add /app for Docker deployment
if "/app" not in sys.path:
    sys.path.insert(0, "/app")

# Import SAM navigation components - use boundary_navigation for proper SAM avoidance
boundary_plan_path = None
wrap_sams = None
HAS_SAM_NAVIGATION = False

try:
    from path_planning_core.boundary_navigation import plan_path as boundary_plan_path
    from path_planning_core.sam_wrapping import wrap_sams
    HAS_SAM_NAVIGATION = True
    print("✅ SAM navigation (boundary_navigation) loaded successfully", flush=True)
except ImportError as e:
    print(f"⚠️ SAM navigation modules not available: {e}", flush=True)
    HAS_SAM_NAVIGATION = False


def point_in_polygon(point: Tuple[float, float], polygon: np.ndarray) -> bool:
    """
    Check if a point is inside a polygon using ray casting algorithm.

    Args:
        point: (x, y) coordinates
        polygon: Nx2 array of polygon vertices

    Returns:
        True if point is inside polygon
    """
    x, y = point
    n = len(polygon)
    inside = False

    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i

    return inside


class SAMDistanceMatrixCalculator:
    """
    Calculator for SAM-aware distance matrix.

    Pre-calculates all pairwise distances between waypoints, using SAM avoidance
    paths where necessary. Results are cached for use by the solver.
    """

    def __init__(self):
        self._cached_matrix: Optional[Dict[str, Any]] = None
        self._cache_hash: Optional[str] = None

    def calculate_matrix(
        self,
        airports: List[Dict[str, Any]],
        targets: List[Dict[str, Any]],
        sams: List[Dict[str, Any]],
        buffer: float = 0.0
    ) -> Dict[str, Any]:
        """
        Calculate SAM-aware distance matrix for all waypoints.

        Args:
            airports: List of airport dicts with id, x, y
            targets: List of target dicts with id, x, y, priority, type
            sams: List of SAM dicts with pos/[x,y] and range
            buffer: Safety buffer around SAMs (default 3 units)

        Returns:
            Dict containing:
                - matrix: 2D list of distances
                - labels: List of waypoint IDs
                - waypoints: List of waypoint details
                - paths: Dict of pre-computed paths for SAM avoidance routes
                - excluded_targets: List of targets excluded due to inaccessibility
        """
        # Normalize SAM format
        normalized_sams = self._normalize_sams(sams)

        # Check for targets inside SAM zones and exclude them
        # We check BOTH:
        # 1. Inside individual SAM circles (matches frontend red X display)
        # 2. Inside wrapped polygon boundaries (convex hull of overlapping SAMs)
        excluded_inside_boundary = []
        valid_targets = []

        # Get wrapped polygon boundaries (for visualization)
        polygon_boundaries = []
        if wrap_sams and normalized_sams:
            # Convert to format expected by wrap_sams
            sams_for_wrapping = []
            for sam in normalized_sams:
                sams_for_wrapping.append({
                    'x': sam["pos"][0],
                    'y': sam["pos"][1],
                    'radius': sam["range"]
                })
            wrapped_polygons, _ = wrap_sams(sams_for_wrapping)
            polygon_boundaries = wrapped_polygons

        # --- DEBUG: show SAMs (brief) ---
        if normalized_sams:
            print(f"📐 [DEBUG] {len(normalized_sams)} SAM circles in distance-matrix calculator", flush=True)
        # --- END DEBUG ---

        for t in targets:
            tx, ty = float(t["x"]), float(t["y"])
            should_exclude = False

            # CRITICAL: Check if target is inside ANY individual SAM circle
            # This matches the frontend algorithm that shows red X marks
            for sam in normalized_sams:
                sam_x, sam_y = sam["pos"]
                sam_range = sam["range"]
                distance = math.sqrt((tx - sam_x)**2 + (ty - sam_y)**2)
                if distance < sam_range:
                    excluded_inside_boundary.append(t["id"])
                    print(f"⚠️ Target {t['id']} at ({tx:.1f}, {ty:.1f}) is INSIDE SAM circle "
                          f"(center=({sam_x:.1f},{sam_y:.1f}), range={sam_range:.1f}, dist={distance:.1f}) "
                          f"- excluding from mission", flush=True)
                    should_exclude = True
                    break

            # Also check polygon boundaries (for edge cases with overlapping SAMs)
            if not should_exclude:
                for polygon in polygon_boundaries:
                    if point_in_polygon((tx, ty), polygon):
                        excluded_inside_boundary.append(t["id"])
                        print(f"⚠️ Target {t['id']} at ({tx:.1f}, {ty:.1f}) is INSIDE polygon boundary "
                              f"- excluding from mission", flush=True)
                        should_exclude = True
                        break

            if not should_exclude:
                valid_targets.append(t)

        # Build list of all waypoints (using only valid targets)
        all_waypoints = self._build_waypoint_list(airports, valid_targets)

        n = len(all_waypoints)
        labels = [wp["id"] for wp in all_waypoints]

        # Initialize matrix and paths storage
        matrix: List[List[float]] = [[0.0] * n for _ in range(n)]
        paths: Dict[str, List[List[float]]] = {}

        # Calculate all pairwise distances
        for i in range(n):
            for j in range(n):
                if i == j:
                    matrix[i][j] = 0.0
                    continue

                start = [all_waypoints[i]["x"], all_waypoints[i]["y"]]
                end = [all_waypoints[j]["x"], all_waypoints[j]["y"]]

                if self._path_blocked(start, end, normalized_sams, buffer):
                    path, distance, method = self._plan_sam_avoiding_path(
                        start, end, normalized_sams, buffer
                    )

                    if path is None or distance == float('inf'):
                        # No valid SAM-safe path for this pair → mark as very large
                        matrix[i][j] = 99999.0
                    else:
                        matrix[i][j] = distance
                        # Store the path for later visualization
                        path_key = f"{labels[i]}->{labels[j]}"
                        paths[path_key] = path
                else:
                    # Direct Euclidean distance
                    distance = math.hypot(end[0] - start[0], end[1] - start[1])
                    matrix[i][j] = distance


        # Build waypoint details for solver
        waypoint_details = []
        for wp in all_waypoints:
            detail = {
                "id": wp["id"],
                "x": float(wp["x"]),
                "y": float(wp["y"]),
            }
            if str(wp["id"]).startswith("T"):
                detail["priority"] = int(wp.get("priority", 5))
                detail["type"] = wp.get("type", "a")
            waypoint_details.append(detail)

        # ------------------------------------------------------------
        # Determine targets that are unreachable from *all* airports
        # ------------------------------------------------------------
        airport_ids = [str(a["id"]) for a in airports]
        target_ids  = [str(t["id"]) for t in valid_targets]

        id_to_index = {str(wp["id"]): idx for idx, wp in enumerate(all_waypoints)}

        unreachable_targets: List[str] = []

        for tid in target_ids:
            tj = id_to_index.get(tid)
            if tj is None:
                continue

            reachable_from_any_airport = False
            for aid in airport_ids:
                ai = id_to_index.get(aid)
                if ai is None:
                    continue
                d = matrix[ai][tj]
                # 99999.0 is our "no path" sentinel
                if d < 99999.0:
                    reachable_from_any_airport = True
                    break

            if not reachable_from_any_airport:
                unreachable_targets.append(tid)

        # Combine all excluded targets:
        #   - ones inside polygon boundary (excluded_inside_boundary)
        #   - ones unreachable from *all* airports (unreachable_targets)
        all_excluded = excluded_inside_boundary + unreachable_targets

        if all_excluded:
            print(
                f"📊 Excluded {len(all_excluded)} targets from mission: {all_excluded} "
                f"(inside boundary: {excluded_inside_boundary}, unreachable: {unreachable_targets})",
                flush=True,
            )

        # Store wrapped polygons for visualization
        wrapped_polygon_lists = []
        if polygon_boundaries:
            for poly in polygon_boundaries:
                if hasattr(poly, 'tolist'):
                    wrapped_polygon_lists.append(poly.tolist())
                else:
                    wrapped_polygon_lists.append(list(poly))

        result = {
            "matrix": matrix,
            "labels": labels,
            "waypoints": waypoint_details,
            "paths": paths,
            "excluded_targets": all_excluded,
            "sams": normalized_sams,
            "buffer": buffer,
            "wrapped_polygons": wrapped_polygon_lists,
        }

        # Cache the result
        self._cached_matrix = result

        return result


    def get_cached_matrix(self) -> Optional[Dict[str, Any]]:
        """Return the cached distance matrix if available."""
        return self._cached_matrix

    def clear_cache(self):
        """Clear the cached distance matrix."""
        self._cached_matrix = None
        self._cache_hash = None

    def get_path(self, from_id: str, to_id: str) -> Optional[List[List[float]]]:
        """
        Get the pre-computed path between two waypoints.

        Returns None if path is direct (no SAM avoidance needed) or if
        the matrix hasn't been calculated yet.
        """
        if self._cached_matrix is None:
            return None

        path_key = f"{from_id}->{to_id}"
        return self._cached_matrix.get("paths", {}).get(path_key)

    def get_distance(self, from_id: str, to_id: str) -> Optional[float]:
        """
        Get the distance between two waypoints from the cached matrix.

        Returns None if matrix hasn't been calculated.
        """
        if self._cached_matrix is None:
            return None

        labels = self._cached_matrix["labels"]
        matrix = self._cached_matrix["matrix"]

        try:
            i = labels.index(from_id)
            j = labels.index(to_id)
            return matrix[i][j]
        except (ValueError, IndexError):
            return None

    def _normalize_sams(self, sams: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Normalize SAM format to have consistent structure."""
        normalized = []
        for sam in sams:
            norm = {}

            # Handle position - convert to 'pos' format for chain_rule
            if "pos" in sam:
                norm["pos"] = list(sam["pos"])
            elif "x" in sam and "y" in sam:
                norm["pos"] = [float(sam["x"]), float(sam["y"])]
            elif "position" in sam:
                norm["pos"] = list(sam["position"])
            else:
                continue  # Skip invalid SAM

            # Handle range
            norm["range"] = float(sam.get("range", sam.get("radius", 15)))

            # Keep ID if present
            if "id" in sam:
                norm["id"] = sam["id"]

            normalized.append(norm)

        return normalized

    def _build_waypoint_list(
        self,
        airports: List[Dict[str, Any]],
        targets: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Build combined list of all waypoints."""
        all_wp = []

        for a in airports:
            all_wp.append({
                "id": str(a.get("id", f"A{len(all_wp)+1}")),
                "x": float(a["x"]),
                "y": float(a["y"]),
            })

        for t in targets:
            all_wp.append({
                "id": str(t.get("id", f"T{len(all_wp)+1}")),
                "x": float(t["x"]),
                "y": float(t["y"]),
                "priority": int(t.get("priority", 5)),
                "type": t.get("type", "a"),
            })

        return all_wp

    def _path_blocked(
        self,
        start: List[float],
        end: List[float],
        sams: List[Dict[str, Any]],
        buffer: float
    ) -> bool:
        """Check if direct path between two points is blocked by any SAM."""
        if not sams:
            return False

        for sam in sams:
            pos = sam["pos"]
            radius = sam["range"] + buffer

            # Check if line segment intersects SAM buffer circle
            if self._line_intersects_circle(start, end, pos, radius):
                return True

        return False

    def _line_intersects_circle(
        self,
        start: List[float],
        end: List[float],
        center: List[float],
        radius: float
    ) -> bool:
        """Check if line segment intersects a circle."""
        # Vector from start to end
        dx = end[0] - start[0]
        dy = end[1] - start[1]

        # Vector from start to circle center
        fx = start[0] - center[0]
        fy = start[1] - center[1]

        # Quadratic equation coefficients
        a = dx * dx + dy * dy
        if a == 0:
            return False

        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - radius * radius

        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False

        # Check if intersection points are on the line segment
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)

        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)

    def _plan_sam_avoiding_path(
        self,
        start: List[float],
        end: List[float],
        sams: List[Dict[str, Any]],
        buffer: float  # Kept for API compatibility but not used
    ) -> Tuple[Optional[List[List[float]]], float, str]:
        """
        Plan a path from start to end that avoids all SAM zones.

        Uses boundary_navigation for proper SAM avoidance with internal BOUNDARY_BUFFER.
        """
        if not HAS_SAM_NAVIGATION:
            # Fallback: return direct distance with warning
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            return [start, end], distance, "direct (no SAM avoidance)"

        try:
            # Use boundary_navigation for proper SAM avoidance path planning
            # boundary_plan_path returns (path, distance, method)
            # It handles BOUNDARY_BUFFER internally - no external buffer needed
            path, distance, method = boundary_plan_path(start, end, sams)

            if path is not None and len(path) >= 2:
                return path, distance, method

            return None, float('inf'), "no_path_found"

        except Exception as e:
            print(f"⚠️ SAM path planning error: {e}")
            return None, float('inf'), f"error: {str(e)}"


# Global calculator instance
_calculator = SAMDistanceMatrixCalculator()


def calculate_sam_aware_matrix(
    env: Dict[str, Any],
    buffer: float = 0.0,
    use_supabase_cache: bool = True,
) -> Dict[str, Any]:
    """
    Calculate SAM-aware distance matrix for the given environment.

    This is the main entry point for use as a LangGraph tool.
    Integrates with Supabase caching for reproducibility and performance.

    Args:
        env: Environment dict with airports, targets, and sams
        buffer: Safety buffer around SAMs (default 0 units)
        use_supabase_cache: Whether to use Supabase for persistent caching

    Returns:
        Distance matrix data dict with additional fields:
        - env_hash: Hash of the environment geometry
        - routing_model_hash: Hash of the routing configuration
        - distance_matrix_id: UUID from Supabase (if cached)
    """
    import time

    airports = list(env.get("airports", []))
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    # Add checkpoints (C1, C2, etc.) for segmented mission replanning
    # Checkpoints act like airports for distance calculations
    for checkpoint in env.get("checkpoints", []):
        airports.append({
            "id": str(checkpoint["id"]),
            "x": float(checkpoint["x"]),
            "y": float(checkpoint["y"]),
            "is_checkpoint": True,
        })
        print(f"📍 [SAM Matrix] Added checkpoint: {checkpoint['id']} at ({checkpoint['x']:.1f}, {checkpoint['y']:.1f})", flush=True)

    # Add synthetic start nodes (for checkpoint replanning - legacy support)
    # These act like airports for distance calculations
    for node_id, node_data in env.get("synthetic_starts", {}).items():
        airports.append({
            "id": str(node_id),
            "x": float(node_data["x"]),
            "y": float(node_data["y"]),
            "is_synthetic": True,
        })
        print(f"📍 [SAM Matrix] Added synthetic start: {node_id} at ({node_data['x']:.1f}, {node_data['y']:.1f})", flush=True)

    # Try Supabase cache first
    if use_supabase_cache:
        try:
            from ..database.mission_ledger import (
                compute_env_hash,
                compute_routing_model_hash,
                get_default_routing_model,
                get_cached_matrix as get_supabase_cached_matrix,
                cache_matrix as cache_supabase_matrix,
            )

            # Compute hashes for cache lookup
            env_hash = compute_env_hash(airports, targets, sams)
            routing_model = get_default_routing_model()
            routing_model["sam_buffer"] = buffer
            routing_model_hash = compute_routing_model_hash(routing_model)

            # Check Supabase cache
            cached = get_supabase_cached_matrix(env_hash, routing_model_hash)
            if cached is not None:
                print(f"✅ [SAM Matrix] Using Supabase cached matrix (id={cached['id'][:8]}...)", flush=True)
                # Convert from Supabase format back to our internal format
                result = {
                    "matrix": cached["matrix"],
                    "labels": cached["labels"],
                    "waypoints": [],  # Will rebuild from labels
                    "paths": {},  # Paths not stored in cache
                    "excluded_targets": cached["excluded_targets"],
                    "sams": [],  # Will need to rebuild if needed
                    "buffer": buffer,
                    "wrapped_polygons": [],
                    # Add cache metadata
                    "env_hash": env_hash,
                    "routing_model": routing_model,
                    "routing_model_hash": routing_model_hash,
                    "distance_matrix_id": cached["id"],
                    "cache_hit": True,
                }
                # Rebuild waypoints from labels and original data
                all_wp = airports + targets
                wp_by_id = {str(wp.get("id", "")): wp for wp in all_wp}
                for label in cached["labels"]:
                    wp = wp_by_id.get(label)
                    if wp:
                        detail = {
                            "id": label,
                            "x": float(wp.get("x", 0)),
                            "y": float(wp.get("y", 0)),
                        }
                        if label.startswith("T"):
                            detail["priority"] = int(wp.get("priority", 5))
                            detail["type"] = wp.get("type", "a")
                        result["waypoints"].append(detail)

                # Cache locally too
                _calculator._cached_matrix = result
                return result

        except ImportError:
            print("⚠️ [SAM Matrix] Supabase caching unavailable - using local only", flush=True)
        except Exception as e:
            print(f"⚠️ [SAM Matrix] Supabase cache error: {e}", flush=True)

    # Calculate fresh matrix
    start_time = time.time()
    result = _calculator.calculate_matrix(airports, targets, sams, buffer)
    computation_ms = int((time.time() - start_time) * 1000)

    # Add hashes to result for traceability
    try:
        from ..database.mission_ledger import (
            compute_env_hash,
            compute_routing_model_hash,
            get_default_routing_model,
            cache_matrix as cache_supabase_matrix,
        )

        env_hash = compute_env_hash(airports, targets, sams)
        routing_model = get_default_routing_model()
        routing_model["sam_buffer"] = buffer
        routing_model_hash = compute_routing_model_hash(routing_model)

        result["env_hash"] = env_hash
        result["routing_model"] = routing_model
        result["routing_model_hash"] = routing_model_hash
        result["cache_hit"] = False

        # Cache in Supabase for future use
        if use_supabase_cache:
            matrix_id = cache_supabase_matrix(
                env_hash=env_hash,
                routing_model_hash=routing_model_hash,
                sam_mode=routing_model.get("sam_mode", "hard_v1"),
                matrix=result["matrix"],
                labels=result["labels"],
                excluded_targets=result.get("excluded_targets", []),
                computation_ms=computation_ms,
                num_sams=len(sams),
            )
            if matrix_id:
                result["distance_matrix_id"] = matrix_id
                print(f"✅ [SAM Matrix] Cached in Supabase (id={matrix_id[:8]}..., {computation_ms}ms)", flush=True)

    except ImportError:
        pass
    except Exception as e:
        print(f"⚠️ [SAM Matrix] Failed to cache in Supabase: {e}", flush=True)

    return result


def get_cached_matrix() -> Optional[Dict[str, Any]]:
    """Get the cached distance matrix if available."""
    return _calculator.get_cached_matrix()


def clear_matrix_cache():
    """Clear the cached distance matrix."""
    _calculator.clear_cache()


def get_path_between(from_id: str, to_id: str) -> Optional[List[List[float]]]:
    """Get the pre-computed SAM-avoiding path between two waypoints."""
    return _calculator.get_path(from_id, to_id)


def get_distance_between(from_id: str, to_id: str) -> Optional[float]:
    """Get the distance between two waypoints from the cached matrix."""
    return _calculator.get_distance(from_id, to_id)


def get_wrapped_polygons() -> List[List[List[float]]]:
    """Get the wrapped SAM polygons from the cached matrix."""
    cached = _calculator.get_cached_matrix()
    if cached is None:
        return []
    return cached.get("wrapped_polygons", [])


# Tool definition for LangGraph
SAM_DISTANCE_MATRIX_TOOL = {
    "name": "calculate_sam_aware_matrix",
    "description": """Calculate a distance matrix between all waypoints (airports and targets)
    that accounts for SAM avoidance paths. This should be called after editing is complete
    and before solving to ensure optimal path distances are available. Returns matrix data
    including distances, paths, and any inaccessible targets.""",
    "parameters": {
        "type": "object",
        "properties": {
            "env": {
                "type": "object",
                "description": "Environment data containing airports, targets, and sams arrays"
            },
            "buffer": {
                "type": "number",
                "description": "Safety buffer around SAMs in units (default: 3.0)",
                "default": 3.0
            }
        },
        "required": ["env"]
    },
    "function": calculate_sam_aware_matrix
}
