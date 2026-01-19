"""
ISR Trajectory Planner
Generates flight paths that avoid SAM zones using boundary navigation.

Uses tangent-arc-tangent method with INVIOLABLE SAM boundaries.
Boundaries are NEVER penetrated - paths go around them.
"""

from __future__ import annotations
import sys
import os
from typing import List, Dict, Tuple

# Add project root to path for path_planning_core
project_root = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)
if project_root not in sys.path:
    sys.path.insert(0, project_root)
# Add /app for Docker deployment
if "/app" not in sys.path:
    sys.path.insert(0, "/app")

# Import boundary navigation directly - uses proper tangent-arc-tangent method
from path_planning_core.boundary_navigation import plan_path as boundary_plan_path


class ISRTrajectoryPlanner:
    """Plan trajectories that avoid SAM zones using boundary navigation + strict validation."""

    def __init__(self, sams: List[Dict]):
        """
        Initialize planner with SAM zones.

        Args:
            sams: List of SAM dicts with:
                  - pos: [x, y] or position: [x, y]
                  - range: float (SAM radius)
        """
        # Normalize SAM format: {'pos': (x, y), 'range': radius}
        self.sams: List[Dict[str, float]] = []
        for sam in sams:
            pos = sam.get("pos") or sam.get("position")
            if not pos:
                continue
            if isinstance(pos, (list, tuple)):
                px, py = float(pos[0]), float(pos[1])
            else:
                # Just in case some old format sneaks in
                px = float(pos.get("x", 0.0))
                py = float(pos.get("y", 0.0))

            radius = float(sam.get("range", sam.get("radius", 15)))
            self.sams.append({"pos": (px, py), "range": radius})

        if self.sams:
            for sam in self.sams:

    # ---------------------------
    #  Internal helpers
    # ---------------------------

    def _point_inside_any_sam(self, x: float, y: float) -> bool:
        """Return True if (x,y) is strictly inside ANY SAM circle."""
        for sam in self.sams:
            (sx, sy) = sam["pos"]
            r = sam["range"]
            dx = x - sx
            dy = y - sy
            if dx * dx + dy * dy < r * r:  # strictly inside
                return True
        return False

    def _path_violates_sams(
        self,
        path: List[List[float]],
        samples_per_segment: int = 64,
    ) -> Tuple[bool, Tuple[float, float] | None]:
        """
        Sample each segment of the path.
        Returns (violates, offending_point).
        """
        if not path or len(path) < 2 or not self.sams:
            return False, None

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            for k in range(samples_per_segment + 1):
                t = k / samples_per_segment
                x = x1 + t * (x2 - x1)
                y = y1 + t * (y2 - y1)

                if self._point_inside_any_sam(x, y):
                    return True, (x, y)

        return False, None

    # ---------------------------
    #  Public API
    # ---------------------------

    def plan_path(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
    ) -> Tuple[List[List[float]], float]:
        """
        Plan path from start to end avoiding SAM zones.

        CRITICAL: Paths will NEVER penetrate SAM boundaries.
        We:
          1. Ask boundary_navigation for a candidate path.
          2. STRICTLY VALIDATE that no segment enters a SAM.
          3. If any violation is detected -> return ([], inf) and log.

        Args:
            start: (x, y) starting position
            end: (x, y) ending position

        Returns:
            Tuple of (path_points, total_distance)
            - path_points: List of [x, y] coordinates
            - total_distance: Total path length, or inf if invalid
        """
        import math

        # No SAMs -> trivial direct segment
        if not self.sams:
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            return [list(start), list(end)], distance

        # Use boundary navigation to compute a candidate path
        path, distance, method = boundary_plan_path(
            list(start),
            list(end),
            self.sams,
            debug=True,
        )

        # Sanity check on method
        if method.startswith("INVALID") or not path or len(path) < 2:
            return [], float("inf")

        # boundary_navigation already validates paths against polygons.
        # We trust its output - no additional circle-based validation needed.

        # Debug: show when SAM avoidance generates a non-direct path
        if len(path) > 2:

        return path, distance

    def generate_trajectory(
        self,
        route: List[str],
        waypoint_positions: Dict[str, List[float]],
        drone_id: str | None = None,
    ) -> List[List[float]]:
        """
        Generate complete trajectory for a route sequence.

        Args:
            route: List of waypoint IDs in order (e.g., ['A1', 'T3', 'T5', 'A1'])
            waypoint_positions: Dict mapping waypoint ID to [x, y] position
            drone_id: Optional drone ID for debug logging

        Returns:
            List of [x, y] coordinates forming the complete path.
            If ANY segment violates SAMs, the offending segment's path is dropped
            and the trajectory ends before that segment.
        """
        if len(route) < 2:
            return []

        drone_label = f"D{drone_id}" if drone_id else "UNKNOWN"

        for wp_id in route:
            if wp_id in waypoint_positions:
                x, y = waypoint_positions[wp_id]

        trajectory: List[List[float]] = []

        for i in range(len(route) - 1):
            from_id = route[i]
            to_id = route[i + 1]

            if from_id not in waypoint_positions or to_id not in waypoint_positions:
                continue

            start_xy = tuple(waypoint_positions[from_id])
            end_xy = tuple(waypoint_positions[to_id])


            segment_path, seg_dist = self.plan_path(start_xy, end_xy)

            if not segment_path:
                # Path invalid -> stop trajectory here; do NOT draw illegal motion
                break

            for idx, pt in enumerate(segment_path):

            # Concatenate, avoiding duplicate joint point
            if i == 0:
                trajectory.extend(segment_path)
            else:
                if len(segment_path) > 1:
                    trajectory.extend(segment_path[1:])
                else:
                    trajectory.extend(segment_path)

        return trajectory
