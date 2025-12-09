#!/usr/bin/env python3
"""
Test the specific T19 -> A5 path planning case that's causing backtracking.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from path_planning_core.boundary_navigation import (
    plan_path,
    _find_tangent_vertices,
    _distance,
)
from path_planning_core.sam_wrapping import wrap_sams


def test_t19_to_a5():
    """Test the specific T19 -> A5 case."""
    # From the logs:
    # T19: (19.5, 85.0)
    # A5: (64.9, 3.8)
    # S2 is blocking

    start = (19.5, 85.0)  # T19
    goal = (64.9, 3.8)    # A5

    # We need to find what S2's parameters are
    # From the logs, S2 is blocking this path
    # Let's try a SAM that would be between these points

    # Looking at the path: start at (19.5, 85) going to (64.9, 3.8)
    # That's roughly going right and down
    # A SAM around (40, 50) would block this

    # Try different SAM positions to reproduce the issue
    sam_positions = [
        {'x': 40, 'y': 50, 'range': 20},  # Guess 1
        {'x': 35, 'y': 60, 'range': 25},  # Guess 2
        {'x': 45, 'y': 45, 'range': 20},  # Guess 3
    ]

    for sam in sam_positions:
        print(f"\n{'='*70}")
        print(f"Testing with SAM at ({sam['x']}, {sam['y']}) range={sam['range']}")
        print(f"{'='*70}")

        print(f"Start (T19): {start}")
        print(f"Goal (A5): {goal}")
        print(f"Direct distance: {_distance(start, goal):.2f}")

        # Get polygon
        polygons, _ = wrap_sams([sam], min_seg=2.0)
        if polygons:
            polygon = polygons[0]
            print(f"Polygon has {len(polygon)} vertices")

            # Find tangents from start and goal
            start_left, start_right = _find_tangent_vertices(start, polygon)
            goal_left, goal_right = _find_tangent_vertices(goal, polygon)

            print(f"\nTangent vertices from START ({start}):")
            print(f"  Left tangent: vertex {start_left} at {polygon[start_left]}")
            print(f"  Right tangent: vertex {start_right} at {polygon[start_right]}")
            print(f"  Distance to left: {_distance(start, polygon[start_left]):.2f}")
            print(f"  Distance to right: {_distance(start, polygon[start_right]):.2f}")

            print(f"\nTangent vertices from GOAL ({goal}):")
            print(f"  Left tangent: vertex {goal_left} at {polygon[goal_left]}")
            print(f"  Right tangent: vertex {goal_right} at {polygon[goal_right]}")
            print(f"  Distance to left: {_distance(goal, polygon[goal_left]):.2f}")
            print(f"  Distance to right: {_distance(goal, polygon[goal_right]):.2f}")

        # Plan path with debug
        path, dist, method = plan_path(start, goal, [sam], debug=True)

        print(f"\n--- Result ---")
        print(f"Method: {method}")
        print(f"Distance: {dist:.2f}")
        print(f"Path ({len(path)} waypoints):")
        for i, wp in enumerate(path):
            d_to_goal = _distance(wp, goal)
            if i > 0:
                d_prev = _distance(path[i-1], goal)
                direction = "→ toward" if d_to_goal < d_prev else "⚠️ AWAY"
            else:
                direction = "start"
            print(f"  [{i}] ({wp[0]:.2f}, {wp[1]:.2f}) dist_to_goal={d_to_goal:.2f} {direction}")


if __name__ == "__main__":
    test_t19_to_a5()
