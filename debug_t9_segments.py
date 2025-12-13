#!/usr/bin/env python3
"""
Debug T9 segment search - detailed analysis
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
    print("T9 Segment Analysis - Detailed Debug")
    print("="*80)

    # Solve the problem
    solution = solver_bridge.solve_mission_with_allocation(
        env,
        drone_configs,
        post_optimize=False
    )

    # T9 position
    t9_pos = (1.44, 53.18)
    t9_ssd = 16.03

    print(f"\nT9 Position: {t9_pos}")
    print(f"T9 SSD: {t9_ssd}")
    print(f"Search radius: {t9_ssd} units")

    # Analyze D5 trajectory
    d5_route_data = solution.get('routes', {}).get('5', {})
    d5_route = d5_route_data.get('route', [])
    d5_trajectory = d5_route_data.get('trajectory', [])

    print(f"\n{'='*80}")
    print(f"D5 Route: {' → '.join(str(wp) for wp in d5_route)}")
    print(f"D5 Trajectory vertices: {len(d5_trajectory)}")
    print(f"{'='*80}")

    if not d5_trajectory:
        print("❌ No trajectory data for D5!")
        return

    # Find A1 position in trajectory
    a1_pos = (18.23, 39.56)
    print(f"\nA1 Position: {a1_pos}")
    print(f"Distance T9 → A1: {distance(t9_pos, a1_pos):.2f}")

    # Check all segments in D5 trajectory
    print(f"\n{'='*80}")
    print(f"Analyzing ALL D5 trajectory segments:")
    print(f"{'='*80}")

    segments_within_radius = []

    for i in range(len(d5_trajectory) - 1):
        seg_start = tuple(d5_trajectory[i])
        seg_end = tuple(d5_trajectory[i + 1])

        # Calculate distance from T9 to this segment
        dist = point_to_line_distance(t9_pos, seg_start, seg_end)

        # Check if within search radius
        if dist <= t9_ssd:
            segments_within_radius.append({
                'index': i,
                'start': seg_start,
                'end': seg_end,
                'distance': dist
            })
            print(f"✅ Segment {i}: ({seg_start[0]:.2f}, {seg_start[1]:.2f}) → ({seg_end[0]:.2f}, {seg_end[1]:.2f})")
            print(f"   Distance to T9: {dist:.2f} < {t9_ssd:.2f}")

    print(f"\n{'='*80}")
    print(f"SUMMARY:")
    print(f"{'='*80}")
    print(f"Total D5 trajectory segments: {len(d5_trajectory) - 1}")
    print(f"Segments within radius {t9_ssd}: {len(segments_within_radius)}")

    if segments_within_radius:
        print(f"\n✅ Found {len(segments_within_radius)} segments within search radius!")
        print(f"\nClosest segment:")
        closest = min(segments_within_radius, key=lambda x: x['distance'])
        print(f"  Index: {closest['index']}")
        print(f"  Start: ({closest['start'][0]:.2f}, {closest['start'][1]:.2f})")
        print(f"  End: ({closest['end'][0]:.2f}, {closest['end'][1]:.2f})")
        print(f"  Distance: {closest['distance']:.2f}")
        print(f"\n⚠️  This should be detected as an OS with OSD={closest['distance']:.2f} < SSD={t9_ssd:.2f}")
        print(f"⚠️  But the optimizer didn't find it! Let's investigate why...")

        # Now map these segments to route waypoints
        print(f"\n{'='*80}")
        print(f"Mapping segments to route waypoints:")
        print(f"{'='*80}")

        # Build waypoint positions
        waypoint_positions = {}
        for t in env.get('targets', []):
            waypoint_positions[str(t['id'])] = (float(t['x']), float(t['y']))
        for a in env.get('airports', []):
            waypoint_positions[str(a['id'])] = (float(a['x']), float(a['y']))

        # Find which route segment these belong to
        for seg in segments_within_radius:
            seg_idx = seg['index']
            # Find which route waypoints this segment is between
            for j in range(len(d5_route) - 1):
                wp_start = str(d5_route[j])
                wp_end = str(d5_route[j + 1])
                wp_start_pos = waypoint_positions.get(wp_start)
                wp_end_pos = waypoint_positions.get(wp_end)

                # Find these waypoints in trajectory
                start_traj_idx = None
                end_traj_idx = None

                for ti, tv in enumerate(d5_trajectory):
                    if wp_start_pos and abs(tv[0] - wp_start_pos[0]) < 0.001 and abs(tv[1] - wp_start_pos[1]) < 0.001:
                        start_traj_idx = ti
                    if wp_end_pos and abs(tv[0] - wp_end_pos[0]) < 0.001 and abs(tv[1] - wp_end_pos[1]) < 0.001:
                        end_traj_idx = ti

                if start_traj_idx is not None and end_traj_idx is not None:
                    if start_traj_idx <= seg_idx < end_traj_idx:
                        print(f"  Segment {seg_idx} belongs to route segment: {wp_start} → {wp_end}")
                        print(f"    Route segment trajectory range: [{start_traj_idx}, {end_traj_idx})")
                        print(f"    This segment should be checked by the optimizer!")
    else:
        print(f"\n❌ No segments found within search radius!")
        print(f"This is unexpected - D5 goes to A1 which is close to T9")

if __name__ == '__main__':
    main()
