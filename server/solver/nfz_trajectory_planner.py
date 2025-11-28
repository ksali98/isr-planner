"""
NFZ Trajectory Planner
Generates flight paths that avoid no-fly zones using tangent-arc-tangent method
"""
import numpy as np
from typing import List, Dict, Tuple
import sys
import os

# Import SAM navigation logic from ISR editor
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from isr_editor.path_planning import SAMNavigator


class NFZTrajectoryPlanner:
    """Plan trajectories that avoid no-fly zones"""

    def __init__(self, no_fly_zones: List[Dict]):
        """
        Initialize planner with no-fly zones

        Args:
            no_fly_zones: List of {'x': float, 'y': float, 'radius': float}
        """
        self.no_fly_zones = no_fly_zones

        # Convert NFZ format to SAM format for compatibility with SAMNavigator
        self.sams = [
            {'pos': (nfz['x'], nfz['y']), 'range': nfz['radius']}
            for nfz in no_fly_zones
        ]

        # Use ISR editor's proven SAM navigation
        self.sam_navigator = SAMNavigator()

    def plan_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
        """
        Plan path from start to end avoiding no-fly zones

        Args:
            start: (x, y) starting position
            end: (x, y) ending position

        Returns:
            Tuple of (path_points, total_distance)
            - path_points: List of (x, y) coordinates
            - total_distance: Total path length
        """
        # Use SAM navigation with NFZ as SAMs
        path, distance, _ = self.sam_navigator.plan_path_with_sam_avoidance(
            list(start), list(end), self.sams
        )

        return path, distance

    def check_line_intersects_nfz(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """
        Check if straight line from start to end intersects any NFZ

        Args:
            start: (x, y) starting point
            end: (x, y) ending point

        Returns:
            True if line intersects any NFZ, False otherwise
        """
        for nfz in self.no_fly_zones:
            center = np.array([nfz['x'], nfz['y']])
            radius = nfz['radius']

            # Check if line segment intersects circle
            if self._line_circle_intersection(start, end, center, radius):
                return True

        return False

    def _line_circle_intersection(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                   center: np.ndarray, radius: float) -> bool:
        """
        Check if line segment p1-p2 intersects circle

        Args:
            p1: Start point
            p2: End point
            center: Circle center
            radius: Circle radius

        Returns:
            True if intersection exists
        """
        # Vector from p1 to p2
        d = np.array(p2) - np.array(p1)
        # Vector from p1 to center
        f = np.array(p1) - center

        # Quadratic equation coefficients
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius**2

        discriminant = b**2 - 4*a*c

        if discriminant < 0:
            # No intersection
            return False

        # Check if intersection points are on line segment
        t1 = (-b - np.sqrt(discriminant)) / (2*a)
        t2 = (-b + np.sqrt(discriminant)) / (2*a)

        # Intersection on segment if 0 <= t <= 1
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)
