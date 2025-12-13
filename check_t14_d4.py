#!/usr/bin/env python3
"""
Check if D4's trajectory passes near T14
"""
import json
import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))
from solver import solver_bridge

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def point_to_line_distance(point, line_start, line_end):
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance(point, line_start)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return distance(point, (closest_x, closest_y))

def main():
    with open("/Users/kamalali/Downloads/isr_env2512122217_1.json", 'r') as f:
        env = json.load(f)

    drone_configs = env.get('drone_configs', {})

    print("="*80)
    print("T14 Analysis - Check D4 Trajectory")
    print("="*80)

    solution = solver_bridge.solve_mission_with_allocation(
        env, drone_configs, post_optimize=False
    )

    # T14 info
    t14_pos = (21.573807565789473, 81.6015625)
    t14_ssd = 11.47

    # A3 position
    a3_pos = (12.840254934210527, 74.16735197368422)

    # T7 position
    t7_pos = (34.46854440789473, 84.49629934210526)

    print(f"\nT14 Position: {t14_pos}")
    print(f"T14 SSD (search radius): {t14_ssd}")

    print(f"\n{'='*80}")
    print("Distance Calculations:")
    print(f"{'='*80}")

    # Calculate distances
    dist_a3_t14 = distance(a3_pos, t14_pos)
    dist_t7_t14 = distance(t7_pos, t14_pos)

    print(f"A3 position: {a3_pos}")
    print(f"T7 position: {t7_pos}")
    print(f"\nDistance A3 → T14: {dist_a3_t14:.2f}")
    print(f"Distance T7 → T14: {dist_t7_t14:.2f}")

    if dist_a3_t14 < dist_t7_t14:
        print(f"\n✅ A3 is CLOSER to T14 than T7 is")
        print(f"   A3→T14: {dist_a3_t14:.2f} < T7→T14: {dist_t7_t14:.2f}")
    else:
        print(f"\n❌ A3 is FARTHER from T14 than T7 is")
        print(f"   A3→T14: {dist_a3_t14:.2f} > T7→T14: {dist_t7_t14:.2f}")

    # Analyze D4 trajectory
    d4_route_data = solution.get('routes', {}).get('4', {})
    d4_route = d4_route_data.get('route', [])
    d4_trajectory = d4_route_data.get('trajectory', [])

    print(f"\n{'='*80}")
    print(f"D4 Route: {' → '.join(str(wp) for wp in d4_route)}")
    print(f"D4 Trajectory vertices: {len(d4_trajectory)}")
    print(f"{'='*80}")

    if not d4_trajectory:
        print("❌ No trajectory data for D4!")
        return

    # Check all D4 trajectory segments
    print(f"\nChecking ALL D4 trajectory segments:")

    segments_within_radius = []

    for i in range(len(d4_trajectory) - 1):
        seg_start = tuple(d4_trajectory[i])
        seg_end = tuple(d4_trajectory[i + 1])

        # Calculate distance from T14 to this segment
        dist = point_to_line_distance(t14_pos, seg_start, seg_end)

        if dist <= t14_ssd:
            segments_within_radius.append({
                'index': i,
                'start': seg_start,
                'end': seg_end,
                'distance': dist
            })
            print(f"✅ Segment {i}: ({seg_start[0]:.2f}, {seg_start[1]:.2f}) → ({seg_end[0]:.2f}, {seg_end[1]:.2f})")
            print(f"   Distance to T14: {dist:.2f} < {t14_ssd:.2f}")

    print(f"\n{'='*80}")
    print(f"RESULT:")
    print(f"{'='*80}")
    print(f"Total D4 trajectory segments: {len(d4_trajectory) - 1}")
    print(f"Segments within T14's search radius ({t14_ssd:.2f}): {len(segments_within_radius)}")

    if segments_within_radius:
        print(f"\n✅ Found {len(segments_within_radius)} segments within T14's search radius!")
        closest = min(segments_within_radius, key=lambda x: x['distance'])
        print(f"\nClosest segment to T14:")
        print(f"  Index: {closest['index']}")
        print(f"  Start: ({closest['start'][0]:.2f}, {closest['start'][1]:.2f})")
        print(f"  End: ({closest['end'][0]:.2f}, {closest['end'][1]:.2f})")
        print(f"  Distance (OSD): {closest['distance']:.2f}")
        print(f"\n⚠️  T14 could potentially move from D3 to D4!")
        print(f"  Current SSD: {t14_ssd:.2f}")
        print(f"  D4 segment OSD: {closest['distance']:.2f}")
        if closest['distance'] < t14_ssd:
            print(f"  Gain: {t14_ssd - closest['distance']:.2f}")
    else:
        print(f"\n❌ No D4 segments found within T14's search radius")
        print(f"T14 cannot move to D4")

if __name__ == '__main__':
    main()
