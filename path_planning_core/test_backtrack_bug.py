#!/usr/bin/env python3
"""
Test script to debug the backtracking bug in path planning.

Issue: When navigating from T19 to A5 around S2, the path:
1. Approaches the polygon
2. Touches a tangent point
3. REVERSES direction instead of continuing forward
4. Goes the long way around to reach destination

This is a visibility graph tangent selection issue.
"""

import sys
import os

# Add the parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from path_planning_core.boundary_navigation import (
    plan_path,
    _find_tangent_vertices,
    _build_visibility_graph,
    _dijkstra,
    _distance,
    _segment_clear_of_all_polygons,
)
from path_planning_core.sam_wrapping import wrap_sams


def visualize_path(start, goal, path, polygons, title="Path"):
    """Simple ASCII visualization of the path."""
    print(f"\n{'='*60}")
    print(f"{title}")
    print(f"{'='*60}")
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Path ({len(path)} waypoints):")

    total_dist = 0
    for i, wp in enumerate(path):
        if i > 0:
            seg_dist = _distance(path[i-1], wp)
            total_dist += seg_dist
            # Check if this segment direction is "forward" or "backward"
            # relative to the goal
            prev_to_goal = _distance(path[i-1], goal)
            curr_to_goal = _distance(wp, goal)

            if curr_to_goal > prev_to_goal:
                direction = "⚠️ MOVING AWAY FROM GOAL"
            else:
                direction = "→ toward goal"

            print(f"  [{i}] ({wp[0]:.2f}, {wp[1]:.2f}) [seg: {seg_dist:.2f}] {direction}")
        else:
            print(f"  [{i}] ({wp[0]:.2f}, {wp[1]:.2f}) [start]")

    print(f"Total distance: {total_dist:.2f}")


def test_tangent_selection():
    """Test the tangent selection for a specific scenario."""
    print("\n" + "="*70)
    print("TESTING TANGENT SELECTION")
    print("="*70)

    # Create a simple SAM polygon
    sam = {'x': 50, 'y': 50, 'range': 15}
    polygons, _ = wrap_sams([sam], min_seg=2.0)
    polygon = polygons[0]

    print(f"Polygon has {len(polygon)} vertices")

    # Test point to the left of the polygon
    point = (20, 50)
    left_idx, right_idx = _find_tangent_vertices(point, polygon)
    print(f"\nFrom point {point}:")
    print(f"  Left tangent: vertex {left_idx} at {polygon[left_idx]}")
    print(f"  Right tangent: vertex {right_idx} at {polygon[right_idx]}")

    # Test point to the right of the polygon
    point = (80, 50)
    left_idx, right_idx = _find_tangent_vertices(point, polygon)
    print(f"\nFrom point {point}:")
    print(f"  Left tangent: vertex {left_idx} at {polygon[left_idx]}")
    print(f"  Right tangent: vertex {right_idx} at {polygon[right_idx]}")


def test_path_around_single_polygon():
    """Test path planning around a single polygon."""
    print("\n" + "="*70)
    print("TESTING PATH AROUND SINGLE POLYGON")
    print("="*70)

    # SAM at center
    sam = {'x': 50, 'y': 50, 'range': 15}

    # Path that needs to go around it
    start = (20, 50)
    goal = (80, 50)

    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"SAM at ({sam['x']}, {sam['y']}) with range {sam['range']}")

    path, dist, method = plan_path(start, goal, [sam], debug=True)

    visualize_path(start, goal, path, [], f"Path via {method}")

    return path, dist, method


def test_backtracking_scenario():
    """
    Test a scenario that might cause backtracking.

    The issue is when the shortest path via visibility graph selects
    a tangent that causes the drone to approach, touch, and reverse.
    """
    print("\n" + "="*70)
    print("TESTING BACKTRACKING SCENARIO")
    print("="*70)

    # Create a SAM that's positioned such that one tangent path
    # is shorter in graph terms but causes backtracking
    sam = {'x': 50, 'y': 50, 'range': 15}

    # Start point where one tangent is closer but leads to backtracking
    start = (30, 70)  # Upper left
    goal = (70, 30)   # Lower right (diagonal path)

    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Direct distance: {_distance(start, goal):.2f}")

    path, dist, method = plan_path(start, goal, [sam], debug=True)

    visualize_path(start, goal, path, [], f"Path via {method}")

    # Analyze the path for backtracking
    print("\n--- Path Analysis ---")
    for i in range(1, len(path)):
        prev = path[i-1]
        curr = path[i]

        prev_to_goal = _distance(prev, goal)
        curr_to_goal = _distance(curr, goal)

        if curr_to_goal > prev_to_goal + 1:  # Allow small tolerance
            print(f"⚠️ BACKTRACK at segment {i-1}→{i}:")
            print(f"   {prev} → {curr}")
            print(f"   Distance to goal went from {prev_to_goal:.2f} to {curr_to_goal:.2f}")


def test_multi_polygon_scenario():
    """Test with multiple polygons to see how visibility graph handles it."""
    print("\n" + "="*70)
    print("TESTING MULTI-POLYGON SCENARIO")
    print("="*70)

    # Two SAMs that might cause complex routing
    sams = [
        {'x': 40, 'y': 50, 'range': 15},
        {'x': 70, 'y': 50, 'range': 15},
    ]

    start = (10, 50)
    goal = (100, 50)

    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"SAMs: {sams}")

    path, dist, method = plan_path(start, goal, sams, debug=True)

    visualize_path(start, goal, path, [], f"Path via {method}")


def analyze_visibility_graph():
    """Analyze how the visibility graph is constructed."""
    print("\n" + "="*70)
    print("ANALYZING VISIBILITY GRAPH CONSTRUCTION")
    print("="*70)

    sam = {'x': 50, 'y': 50, 'range': 15}
    polygons, _ = wrap_sams([sam], min_seg=2.0)

    start = (30, 70)
    goal = (70, 30)

    print(f"Building visibility graph from {start} to {goal}")
    print(f"Polygon has {len(polygons[0])} vertices")

    graph = _build_visibility_graph(start, goal, polygons, debug=True)

    print(f"\nGraph has {len(graph)} nodes")
    print(f"Start node edges: {len(graph.get(start, []))}")
    print(f"Goal node edges: {len(graph.get(goal, []))}")

    # Print edges from start
    print(f"\nEdges from start {start}:")
    for neighbor, dist in graph.get(start, []):
        print(f"  → {neighbor} (dist: {dist:.2f})")

    # Run Dijkstra
    path, dist = _dijkstra(graph, start, goal, debug=True)

    print(f"\nDijkstra path: {path}")
    print(f"Total distance: {dist:.2f}")


if __name__ == "__main__":
    print("="*70)
    print("BOUNDARY NAVIGATION BACKTRACK BUG ANALYSIS")
    print("="*70)

    test_tangent_selection()
    test_path_around_single_polygon()
    test_backtracking_scenario()
    test_multi_polygon_scenario()
    analyze_visibility_graph()

    print("\n" + "="*70)
    print("TESTS COMPLETE")
    print("="*70)
