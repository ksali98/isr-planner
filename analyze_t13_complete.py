#!/usr/bin/env python3
"""
Calculate T13's SSD and OSD with all drone routes
D1: A1,T9,T17,T24,T20,A1
D2: A2,T5,T16,T6,T8,T11,T10,T12,T13,A1 (T13 is here!)
"""

import sys
import json
import math
sys.path.insert(0, '/Users/kamalali/isr-planner/server')

from solver.trajectory_planner import ISRTrajectoryPlanner

def point_to_line_distance(point, line_start, line_end):
    """Calculate perpendicular distance from point to line segment"""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    line_len_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2

    if line_len_sq == 0:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

    t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq))

    closest_x = x1 + t * (x2 - x1)
    closest_y = y1 + t * (y2 - y1)

    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

def main():
    # Load environment
    env_file = '/Users/kamalali/Downloads/isr_env2512131725_3.json'
    with open(env_file, 'r') as f:
        env = json.load(f)

    # Build waypoint positions
    waypoint_positions = {}
    for t in env.get('targets', []):
        waypoint_positions[str(t['id'])] = (float(t['x']), float(t['y']))
    for a in env.get('airports', []):
        waypoint_positions[str(a['id'])] = (float(a['x']), float(a['y']))

    # Define drone routes
    drone_routes = {
        '1': ['A1', 'T9', 'T17', 'T24', 'T20', 'A1'],
        '2': ['A2', 'T5', 'T16', 'T6', 'T8', 'T11', 'T10', 'T12', 'T13', 'A1'],
        # Add D3, D4, D5 when provided
    }

    print("\n" + "="*80)
    print("DRONE ROUTES")
    print("="*80)
    for drone_id, route in drone_routes.items():
        marker = " ← T13 HERE" if 'T13' in route else ""
        print(f"D{drone_id}: {' → '.join(route)}{marker}")

    # Initialize trajectory planner
    sams = env.get('sams', [])
    planner = ISRTrajectoryPlanner(sams)

    # Generate trajectories for all drones
    print("\n" + "="*80)
    print("GENERATING TRAJECTORIES WITH SAM AVOIDANCE")
    print("="*80)

    drone_trajectories = {}
    for drone_id, route in drone_routes.items():
        print(f"\nGenerating trajectory for D{drone_id}...")
        trajectory = planner.generate_trajectory(route, waypoint_positions)
        drone_trajectories[drone_id] = trajectory
        print(f"✓ D{drone_id}: {len(trajectory)} vertices")

    # Find T13 position and calculate SSD
    t13_pos = waypoint_positions['T13']
    t13_drone = '2'
    t13_route = drone_routes['2']
    t13_route_idx = t13_route.index('T13')
    t13_trajectory = drone_trajectories['2']

    # Find T13 in trajectory
    t13_traj_idx = None
    for i, vertex in enumerate(t13_trajectory):
        if abs(vertex[0] - t13_pos[0]) < 0.001 and abs(vertex[1] - t13_pos[1]) < 0.001:
            t13_traj_idx = i
            break

    print("\n" + "="*80)
    print("CALCULATING SSD (Self-Segment Distance) FOR T13")
    print("="*80)

    # Get vertices before and after T13
    if t13_traj_idx is None:
        print("ERROR: Could not find T13 in trajectory")
        return

    if t13_traj_idx == 0:
        prev_pos = t13_trajectory[0]
        next_pos = t13_trajectory[1] if len(t13_trajectory) > 1 else t13_trajectory[0]
    elif t13_traj_idx >= len(t13_trajectory) - 1:
        prev_pos = t13_trajectory[-2] if len(t13_trajectory) > 1 else t13_trajectory[-1]
        next_pos = t13_trajectory[-1]
    else:
        prev_pos = t13_trajectory[t13_traj_idx - 1]
        next_pos = t13_trajectory[t13_traj_idx + 1]

    prev_wp = t13_route[t13_route_idx - 1]
    next_wp = t13_route[t13_route_idx + 1]

    ssd = point_to_line_distance(t13_pos, prev_pos, next_pos)

    print(f"\nDrone: D{t13_drone}")
    print(f"Route context: {prev_wp} → T13 → {next_wp}")
    print(f"Previous trajectory vertex: ({prev_pos[0]:.3f}, {prev_pos[1]:.3f})")
    print(f"T13 position:               ({t13_pos[0]:.3f}, {t13_pos[1]:.3f})")
    print(f"Next trajectory vertex:     ({next_pos[0]:.3f}, {next_pos[1]:.3f})")
    print(f"\n>>> SSD = {ssd:.3f} <<<")

    # Calculate OSD for all segments in all drones
    print("\n" + "="*80)
    print("CALCULATING OSD (Other-Segment Distance) FOR ALL DRONE SEGMENTS")
    print("="*80)

    osd_results = []

    for drone_id, route in drone_routes.items():
        trajectory = drone_trajectories[drone_id]

        # Check each route segment
        for j in range(len(route) - 1):
            seg_start_id = route[j]
            seg_end_id = route[j + 1]

            # Skip T13's own segments in D2
            if drone_id == '2' and (j == t13_route_idx - 1 or j == t13_route_idx):
                continue

            # Find trajectory indices for this route segment
            start_wp_pos = waypoint_positions.get(seg_start_id)
            end_wp_pos = waypoint_positions.get(seg_end_id)

            start_traj_idx = None
            end_traj_idx = None

            if start_wp_pos:
                for ti, tv in enumerate(trajectory):
                    if abs(tv[0] - start_wp_pos[0]) < 0.001 and abs(tv[1] - start_wp_pos[1]) < 0.001:
                        start_traj_idx = ti
                        break

            # Search for end waypoint AFTER start waypoint to handle return-to-base routes
            if end_wp_pos and start_traj_idx is not None:
                for ti in range(start_traj_idx + 1, len(trajectory)):
                    tv = trajectory[ti]
                    if abs(tv[0] - end_wp_pos[0]) < 0.001 and abs(tv[1] - end_wp_pos[1]) < 0.001:
                        end_traj_idx = ti
                        break

            if start_traj_idx is None or end_traj_idx is None:
                if start_traj_idx is None:
                    print(f"  ⚠️  Could not find {seg_start_id} in D{drone_id} trajectory")
                if end_traj_idx is None:
                    print(f"  ⚠️  Could not find {seg_end_id} in D{drone_id} trajectory")
                continue

            # Check all trajectory sub-segments
            min_osd = float('inf')
            best_subseg_idx = None

            for ti in range(start_traj_idx, end_traj_idx):
                traj_seg_start = trajectory[ti]
                traj_seg_end = trajectory[ti + 1]

                osd = point_to_line_distance(t13_pos, traj_seg_start, traj_seg_end)
                if osd < min_osd:
                    min_osd = osd
                    best_subseg_idx = ti

            if min_osd < float('inf'):
                gain = ssd - min_osd
                osd_results.append({
                    'drone': drone_id,
                    'segment': f"{seg_start_id}→{seg_end_id}",
                    'osd': min_osd,
                    'subseg_idx': best_subseg_idx,
                    'gain': gain
                })

    # Sort by OSD (ascending)
    osd_results.sort(key=lambda x: x['osd'])

    print(f"\nFound {len(osd_results)} trajectory segments")
    print(f"\nAll segments sorted by OSD (closest first):")
    print(f"\n{'Drone':<8} {'Segment':<20} {'OSD':<12} {'Gain (SSD-OSD)':<15} {'Better?'}")
    print("-" * 80)

    for r in osd_results:
        gain = r['gain']
        better = "YES ✓" if gain > 0 else "NO"
        drone_marker = f"D{r['drone']}" + (" (current)" if r['drone'] == t13_drone else "")
        print(f"{drone_marker:<8} {r['segment']:<20} {r['osd']:<12.3f} {gain:<15.3f} {better}")

    # Summary
    print("\n" + "="*80)
    print("SUMMARY FOR T13")
    print("="*80)
    print(f"\nCurrent assignment: Drone {t13_drone}")
    print(f"Route position: {t13_route_idx + 1}/{len(t13_route)}")
    print(f"SSD (Self-Segment Distance): {ssd:.3f}")

    better_segments = [r for r in osd_results if r['gain'] > 0]

    if better_segments:
        print(f"\n✓ Found {len(better_segments)} segments with OSD < SSD (gain > 0)")
        best = better_segments[0]
        print(f"\nBest alternative:")
        print(f"  Drone {best['drone']}, segment {best['segment']}")
        print(f"  OSD = {best['osd']:.3f}")
        print(f"  Gain = {best['gain']:.3f} (SSD - OSD)")
        print(f"\n>>> Swap Closer would move T13 from D{t13_drone} to D{best['drone']} <<<")
    else:
        print(f"\n✓ No better trajectory found - T13 is optimally placed on D{t13_drone}")

    print("\n" + "="*80)

if __name__ == "__main__":
    main()
