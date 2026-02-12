#!/usr/bin/env python3
"""
Analyze T14 - check if it can move to other drones
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
    with open("test_env_t9.json", 'r') as f:
        env = json.load(f)

    drone_configs = env.get('drone_configs', {})

    print("="*80)
    print("T14 Analysis - Can it move to another drone?")
    print("="*80)

    solution = solver_bridge.solve_mission_with_allocation(
        env, drone_configs, post_optimize=False
    )

    # T14 info
    t14_pos = (21.573807565789473, 81.6015625)
    t14_ssd = 11.47

    print(f"\nT14 Position: {t14_pos}")
    print(f"T14 SSD: {t14_ssd}")
    print(f"T14 Current: D3: A3 → T14 → A3")

    # Check all other drone trajectories
    print(f"\n{'='*80}")
    print("Checking all other drone trajectories:")
    print(f"{'='*80}")

    best_overall = None

    for drone_id in ['1', '2', '4', '5']:  # Skip D3 (current drone)
        route_data = solution.get('routes', {}).get(drone_id, {})
        route = route_data.get('route', [])
        trajectory = route_data.get('trajectory', [])

        if not trajectory:
            continue

        print(f"\nD{drone_id}: {' → '.join(str(wp) for wp in route)}")
        print(f"  Trajectory vertices: {len(trajectory)}")

        # Check all segments
        best_for_drone = None

        for i in range(len(trajectory) - 1):
            seg_start = tuple(trajectory[i])
            seg_end = tuple(trajectory[i + 1])
            dist = point_to_line_distance(t14_pos, seg_start, seg_end)

            if dist < t14_ssd:
                if best_for_drone is None or dist < best_for_drone['dist']:
                    best_for_drone = {
                        'drone': drone_id,
                        'seg_idx': i,
                        'start': seg_start,
                        'end': seg_end,
                        'dist': dist
                    }

        if best_for_drone:
            print(f"  ✅ Found segment within radius!")
            print(f"     Segment [{best_for_drone['seg_idx']}]: ({best_for_drone['start'][0]:.2f}, {best_for_drone['start'][1]:.2f}) → ({best_for_drone['end'][0]:.2f}, {best_for_drone['end'][1]:.2f})")
            print(f"     OSD: {best_for_drone['dist']:.2f} < SSD: {t14_ssd:.2f}")
            print(f"     Gain: {t14_ssd - best_for_drone['dist']:.2f}")

            if best_overall is None or best_for_drone['dist'] < best_overall['dist']:
                best_overall = best_for_drone
        else:
            print(f"  ❌ No segments within radius {t14_ssd:.2f}")

    print(f"\n{'='*80}")
    print("RESULT:")
    print(f"{'='*80}")

    if best_overall:
        print(f"✅ T14 CAN move to D{best_overall['drone']}!")
        print(f"   Best OSD: {best_overall['dist']:.2f}")
        print(f"   Gain: {t14_ssd - best_overall['dist']:.2f}")
    else:
        print(f"❌ T14 cannot move - no better trajectory found")

if __name__ == '__main__':
    main()
