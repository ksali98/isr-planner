#!/usr/bin/env python3
"""
Debug T17 segment search - check if D5's T3→A1 is close to T17
"""
import json
import sys
import os
import math

# Add path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

from solver import solver_bridge

def distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def point_to_line_distance(point, line_start, line_end):
    """Calculate perpendicular distance from point to line segment"""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    # Vector from line_start to line_end
    dx = x2 - x1
    dy = y2 - y1

    # Handle degenerate case (line_start == line_end)
    if dx == 0 and dy == 0:
        return distance(point, line_start)

    # Calculate parameter t for closest point on line
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

    # Clamp t to [0, 1] to stay on segment
    t = max(0, min(1, t))

    # Calculate closest point on segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Return distance to closest point
    return distance(point, (closest_x, closest_y))

def main():
    # Load the test environment
    env_path = "test_env_t9.json"
    with open(env_path, 'r') as f:
        env = json.load(f)

    # Get drone configs
    drone_configs = env.get('drone_configs', {})

    print("="*80)
    print("T17 Segment Analysis - Check D5's T3→A1")
    print("="*80)

    # Solve the problem
    solution = solver_bridge.solve_mission_with_allocation(
        env,
        drone_configs,
        post_optimize=False
    )

    # T17 position
    t17_pos = (7.1001233552631575, 38.180509868421055)
    t17_ssd = 8.09  # From the output

    print(f"\nT17 Position: {t17_pos}")
    print(f"T17 SSD: {t17_ssd}")
    print(f"T17 Current route: D1: A1 → T17 → T9 → A1")

    # Analyze D5 trajectory
    d5_route_data = solution.get('routes', {}).get('5', {})
    d5_route = d5_route_data.get('route', [])
    d5_trajectory = d5_route_data.get('trajectory', [])

    print(f"\nD5 Route: {' → '.join(str(wp) for wp in d5_route)}")
    print(f"D5 Trajectory vertices: {len(d5_trajectory)}")

    # A1 position
    a1_pos = (18.23, 39.56)
    print(f"\nA1 Position: {a1_pos}")
    print(f"Distance T17 → A1: {distance(t17_pos, a1_pos):.2f}")

    # Check D5's T3→A1 segment specifically
    print(f"\n{'='*80}")
    print(f"Checking D5's T3→A1 segment:")
    print(f"{'='*80}")

    # Find T3 and A1 positions in D5 trajectory
    waypoint_positions = {}
    for t in env.get('targets', []):
        waypoint_positions[str(t['id'])] = (float(t['x']), float(t['y']))
    for a in env.get('airports', []):
        waypoint_positions[str(a['id'])] = (float(a['x']), float(a['y']))

    t3_pos = waypoint_positions.get('T3')

    # Find trajectory indices for T3→A1 segment
    start_traj_idx = None
    end_traj_idx = None

    for ti, tv in enumerate(d5_trajectory):
        if t3_pos and abs(tv[0] - t3_pos[0]) < 0.001 and abs(tv[1] - t3_pos[1]) < 0.001:
            start_traj_idx = ti
        if abs(tv[0] - a1_pos[0]) < 0.001 and abs(tv[1] - a1_pos[1]) < 0.001:
            end_traj_idx = ti

    if start_traj_idx is None or end_traj_idx is None:
        print("❌ Could not find T3→A1 segment in trajectory")
        return

    print(f"T3→A1 trajectory range: [{start_traj_idx}, {end_traj_idx})")
    print(f"Number of sub-segments: {end_traj_idx - start_traj_idx}")

    # Check all sub-segments
    segments_within_radius = []

    for ti in range(start_traj_idx, end_traj_idx):
        seg_start = tuple(d5_trajectory[ti])
        seg_end = tuple(d5_trajectory[ti + 1])

        # Calculate distance from T17 to this segment
        dist = point_to_line_distance(t17_pos, seg_start, seg_end)

        if dist <= t17_ssd:
            segments_within_radius.append({
                'index': ti,
                'start': seg_start,
                'end': seg_end,
                'distance': dist
            })
            print(f"✅ Sub-segment {ti}: ({seg_start[0]:.2f}, {seg_start[1]:.2f}) → ({seg_end[0]:.2f}, {seg_end[1]:.2f})")
            print(f"   Distance to T17: {dist:.2f} < {t17_ssd:.2f}")

    print(f"\n{'='*80}")
    print(f"SUMMARY:")
    print(f"{'='*80}")
    print(f"Sub-segments in T3→A1: {end_traj_idx - start_traj_idx}")
    print(f"Sub-segments within T17's search radius ({t17_ssd}): {len(segments_within_radius)}")

    if segments_within_radius:
        print(f"\n✅ Found {len(segments_within_radius)} segments within T17's search radius!")
        closest = min(segments_within_radius, key=lambda x: x['distance'])
        print(f"\nClosest segment to T17:")
        print(f"  Index: {closest['index']}")
        print(f"  Start: ({closest['start'][0]:.2f}, {closest['start'][1]:.2f})")
        print(f"  End: ({closest['end'][0]:.2f}, {closest['end'][1]:.2f})")
        print(f"  Distance (OSD): {closest['distance']:.2f}")
        print(f"\n⚠️  T17 could move from D1 to D5!")
        print(f"  Current SSD: {t17_ssd:.2f}")
        print(f"  D5 T3→A1 OSD: {closest['distance']:.2f}")
        if closest['distance'] < t17_ssd:
            print(f"  Potential gain: {t17_ssd - closest['distance']:.2f}")
    else:
        print(f"\n❌ No segments found within T17's search radius")
        print(f"T17 cannot be moved to D5's T3→A1 segment")

if __name__ == '__main__':
    main()
