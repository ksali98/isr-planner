"""
NFZ-Aware Distance Calculator
Calculates distances between points accounting for no-fly zones
"""
import numpy as np
from typing import List, Dict, Tuple


class NFZDistanceCalculator:
    """Calculate distances with no-fly zone avoidance"""

    def __init__(self, no_fly_zones: List[Dict]):
        """
        Initialize calculator with no-fly zones

        Args:
            no_fly_zones: List of {'x': float, 'y': float, 'radius': float}
        """
        self.no_fly_zones = no_fly_zones

    def calculate_distance_matrix(self, waypoints: List[Dict]) -> Dict:
        """
        Calculate NFZ-aware distance matrix for all waypoint pairs

        Args:
            waypoints: List of waypoints with 'id', 'x', 'y' fields

        Returns:
            Dict with 'matrix', 'labels', and 'waypoints' data
        """
        n = len(waypoints)
        matrix = [[0.0] * n for _ in range(n)]
        labels = [wp['id'] for wp in waypoints]

        # Import trajectory planner for path calculation
        from .nfz_trajectory_planner import NFZTrajectoryPlanner
        planner = NFZTrajectoryPlanner(self.no_fly_zones)

        print(f"📊 Calculating NFZ-aware distance matrix for {n} waypoints...")

        for i in range(n):
            for j in range(n):
                if i != j:
                    start = (waypoints[i]['x'], waypoints[i]['y'])
                    end = (waypoints[j]['x'], waypoints[j]['y'])

                    # Calculate path with NFZ avoidance
                    path, distance = planner.plan_path(start, end)
                    matrix[i][j] = distance
                else:
                    matrix[i][j] = 0.0

        print(f"✅ Distance matrix calculated ({n}x{n})")

        # Print distance matrix for debugging
        print("\n📊 DISTANCE MATRIX:")
        print("     ", end="")
        for label in labels:
            print(f"{label:>8}", end="")
        print()
        for i, row_label in enumerate(labels):
            print(f"{row_label:>4} ", end="")
            for j in range(n):
                print(f"{matrix[i][j]:>8.1f}", end="")
            print()
        print()

        return {
            'matrix': matrix,
            'labels': labels,
            'waypoints': waypoints
        }

    def get_euclidean_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate straight-line Euclidean distance"""
        return np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
