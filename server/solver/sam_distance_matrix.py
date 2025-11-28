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

        # Check for targets inside POLYGON BOUNDARY and exclude them
        # The polygon boundary is what matters, not individual SAM circles
        excluded_inside_boundary = []
        valid_targets = []

        # Get wrapped polygon boundaries (same as displayed on UI)
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

        for t in targets:
            tx, ty = float(t["x"]), float(t["y"])
            inside_boundary = False

            # Check if target is inside any wrapped polygon boundary
            for polygon in polygon_boundaries:
                if point_in_polygon((tx, ty), polygon):
                    excluded_inside_boundary.append(t["id"])
                    print(f"⚠️ Target {t['id']} at ({tx:.1f}, {ty:.1f}) is INSIDE polygon boundary "
                          f"- excluding from mission", flush=True)
                    inside_boundary = True
                    break

            if not inside_boundary:
                valid_targets.append(t)

        # Build list of all waypoints (using only valid targets)
        all_waypoints = self._build_waypoint_list(airports, valid_targets)

        n = len(all_waypoints)
        labels = [wp["id"] for wp in all_waypoints]

        # Initialize matrix and paths storage
        matrix: List[List[float]] = [[0.0] * n for _ in range(n)]
        paths: Dict[str, List[List[float]]] = {}
        excluded_targets: List[str] = []

        # Calculate all pairwise distances
        for i in range(n):
            for j in range(n):
                if i == j:
                    matrix[i][j] = 0.0
                    continue

                start = [all_waypoints[i]["x"], all_waypoints[i]["y"]]
                end = [all_waypoints[j]["x"], all_waypoints[j]["y"]]

                # Check if direct path is blocked by any SAM
                if self._path_blocked(start, end, normalized_sams, buffer):
                    # Use SAM-aware path planning
                    path, distance, method = self._plan_sam_avoiding_path(
                        start, end, normalized_sams, buffer
                    )

                    if path is None or distance == float('inf'):
                        # No valid path found - mark as very large distance
                        matrix[i][j] = 99999.0

                        # Track if this makes a target inaccessible
                        wp_id = all_waypoints[j]["id"]
                        if wp_id.startswith("T") and wp_id not in excluded_targets:
                            # Check if target is unreachable from any airport
                            if all_waypoints[i]["id"].startswith("A"):
                                excluded_targets.append(wp_id)
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

        # Combine all excluded targets (inside polygon boundary + path-unreachable)
        all_excluded = excluded_inside_boundary + excluded_targets

        if all_excluded:
            print(f"📊 Excluded {len(all_excluded)} targets from mission: {all_excluded}", flush=True)

        result = {
            "matrix": matrix,
            "labels": labels,
            "waypoints": waypoint_details,
            "paths": paths,
            "excluded_targets": all_excluded,
            "sams": normalized_sams,
            "buffer": buffer,
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
    buffer: float = 0.0
) -> Dict[str, Any]:
    """
    Calculate SAM-aware distance matrix for the given environment.

    This is the main entry point for use as a LangGraph tool.

    Args:
        env: Environment dict with airports, targets, and sams
        buffer: Safety buffer around SAMs (default 3 units)

    Returns:
        Distance matrix data dict
    """
    airports = env.get("airports", [])
    targets = env.get("targets", [])
    sams = env.get("sams", [])

    return _calculator.calculate_matrix(airports, targets, sams, buffer)


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
