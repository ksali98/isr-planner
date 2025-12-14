#!/usr/bin/env python3
"""
Run solver and calculate T13's SSD and OSD values
"""

import sys
import json
import math
sys.path.insert(0, '/Users/kamalali/isr-planner/server')

from solver.solver_bridge import solve_mission

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

    print("\n" + "="*80)
    print("RUNNING SOLVER TO GET TRAJECTORIES")
    print("="*80)

    # Use drone_configs (the second set with 150 fuel budget and return to base)
    drone_configs = env.get('drone_configs', {})

    # Run solver using solve_mission
    result = solve_mission(env=env, drone_configs=drone_configs)

    if not result or 'routes' not in result:
        print("ERROR: Solver failed to produce routes")
        return

    print("\n✓ Solver completed successfully")
    print(f"✓ Generated routes for {len(result['routes'])} drones")

    # Build waypoint positions
    waypoint_positions = {}
    for t in env.get('targets', []):
        waypoint_positions[str(t['id'])] = (float(t['x']), float(t['y']))
    for a in env.get('airports', []):
        waypoint_positions[str(a['id'])] = (float(a['x']), float(a['y']))

    # Find T13's position
    t13_pos = waypoint_positions.get('T13')
    if not t13_pos:
        print("ERROR: T13 not found")
        return

    print(f"\nT13 Position: ({t13_pos[0]:.3f}, {t13_pos[1]:.3f})")

    # Find which drone has T13 and extract trajectory
    t13_drone = None
    t13_route_idx = None
    t13_traj_idx = None
    t13_trajectory = None

    for drone_id, route_data in result['routes'].items():
        route = route_data.get('route', [])
        if 'T13' in route:
            t13_drone = drone_id
            t13_route_idx = route.index('T13')
            t13_trajectory = route_data.get('trajectory', [])

            # Find T13's index in trajectory
            if t13_trajectory:
                for i, vertex in enumerate(t13_trajectory):
                    if abs(vertex[0] - t13_pos[0]) < 0.001 and abs(vertex[1] - t13_pos[1]) < 0.001:
                        t13_traj_idx = i
                        break
            break

    if not t13_drone:
        print("\n⚠️  T13 was not included in any drone's final route!")
        print("    (It was allocated but not visited - likely due to fuel constraints)")
        print("\n    Since T13 is not in any route, it has no SSD (Self-Segment Distance).")
        print("    However, we can calculate OSD (Other-Segment Distance) to all trajectories")
        print("    to see which drone's path passes closest to T13.")

        # Skip to OSD calculation
        ssd = None
    else:
        route_data = result['routes'][t13_drone]
        route = route_data['route']

        print(f"\n" + "="*80)
        print(f"T13 ROUTE ASSIGNMENT")
        print("="*80)
        print(f"Assigned to: Drone {t13_drone}")
        print(f"Route: {' → '.join(route)}")
        print(f"Position in route: {t13_route_idx + 1}/{len(route)}")
        print(f"Trajectory vertices: {len(t13_trajectory)}")
        print(f"T13 trajectory index: {t13_traj_idx}")

        # Calculate SSD (Self-Segment Distance)
        print(f"\n" + "="*80)
        print(f"CALCULATING SSD (Self-Segment Distance)")
        print("="*80)

        if t13_traj_idx is None:
            print("ERROR: Could not find T13 in trajectory")
            return

        # Get vertices before and after T13 in trajectory
        if t13_traj_idx == 0:
            prev_pos = t13_trajectory[0]
            next_pos = t13_trajectory[1] if len(t13_trajectory) > 1 else t13_trajectory[0]
        elif t13_traj_idx >= len(t13_trajectory) - 1:
            prev_pos = t13_trajectory[-2] if len(t13_trajectory) > 1 else t13_trajectory[-1]
            next_pos = t13_trajectory[-1]
        else:
            prev_pos = t13_trajectory[t13_traj_idx - 1]
            next_pos = t13_trajectory[t13_traj_idx + 1]

        # Get route waypoint IDs
        prev_wp = route[t13_route_idx - 1] if t13_route_idx > 0 else "START"
        next_wp = route[t13_route_idx + 1] if t13_route_idx < len(route) - 1 else "END"

        ssd = point_to_line_distance(t13_pos, prev_pos, next_pos)

        print(f"\nRoute context: {prev_wp} → T13 → {next_wp}")
        print(f"Previous trajectory vertex: ({prev_pos[0]:.3f}, {prev_pos[1]:.3f})")
        print(f"T13 position:               ({t13_pos[0]:.3f}, {t13_pos[1]:.3f})")
        print(f"Next trajectory vertex:     ({next_pos[0]:.3f}, {next_pos[1]:.3f})")
        print(f"\n>>> SSD = {ssd:.3f} <<<")
        print(f"\nInterpretation: T13 creates a {ssd:.3f} unit detour from the direct line")
        print(f"                between its neighboring vertices.")

        if ssd == 0:
            print("\n⚠️  SSD = 0: T13 is perfectly on the line between its neighbors!")
            print("    NO SSD NO MOVEMENT rule: This target cannot be swapped.")

    # Calculate OSD for all other drones
    print(f"\n" + "="*80)
    print(f"CALCULATING OSD (Other-Segment Distance) FOR ALL DRONES")
    print("="*80)

    osd_results = []

    for drone_id, route_data in result['routes'].items():
        other_route = route_data.get('route', [])
        other_trajectory = route_data.get('trajectory', [])

        if not other_trajectory or len(other_trajectory) < 2:
            continue

        # Check each route segment
        for j in range(len(other_route) - 1):
            seg_start_id = other_route[j]
            seg_end_id = other_route[j + 1]

            # Skip T13's own segments if on same drone
            if drone_id == t13_drone:
                if j == t13_route_idx - 1 or j == t13_route_idx:
                    continue

            # Find trajectory indices for this route segment
            start_wp_pos = waypoint_positions.get(seg_start_id)
            end_wp_pos = waypoint_positions.get(seg_end_id)

            start_traj_idx = None
            end_traj_idx = None

            if start_wp_pos:
                for ti, tv in enumerate(other_trajectory):
                    if abs(tv[0] - start_wp_pos[0]) < 0.001 and abs(tv[1] - start_wp_pos[1]) < 0.001:
                        start_traj_idx = ti
                        break

            if end_wp_pos:
                for ti, tv in enumerate(other_trajectory):
                    if abs(tv[0] - end_wp_pos[0]) < 0.001 and abs(tv[1] - end_wp_pos[1]) < 0.001:
                        end_traj_idx = ti
                        break

            if start_traj_idx is None or end_traj_idx is None:
                continue

            # Check all trajectory sub-segments
            min_osd = float('inf')
            best_subseg = None

            for ti in range(start_traj_idx, end_traj_idx):
                traj_seg_start = other_trajectory[ti]
                traj_seg_end = other_trajectory[ti + 1]

                osd = point_to_line_distance(t13_pos, traj_seg_start, traj_seg_end)
                if osd < min_osd:
                    min_osd = osd
                    best_subseg = (ti, traj_seg_start, traj_seg_end)

            if min_osd < float('inf'):
                improvement = 0
                if ssd is not None:
                    improvement = ssd - min_osd if min_osd < ssd else 0

                osd_results.append({
                    'drone': drone_id,
                    'segment': f"{seg_start_id}→{seg_end_id}",
                    'osd': min_osd,
                    'subseg_idx': best_subseg[0] if best_subseg else None,
                    'improvement': improvement
                })

    # Sort by OSD (ascending)
    osd_results.sort(key=lambda x: x['osd'])

    print(f"\nFound {len(osd_results)} trajectory segments across all drones")
    print(f"\nTop 10 closest segments to T13:")
    print(f"\n{'Drone':<8} {'Segment':<20} {'OSD':<12} {'Gain (SSD-OSD)':<15} {'Better?'}")
    print("-" * 80)

    for i, r in enumerate(osd_results[:10]):
        gain = r['improvement']
        better = "YES ✓" if gain > 0 else "NO"
        drone_marker = f"D{r['drone']}" + (" (current)" if r['drone'] == t13_drone else "")
        print(f"{drone_marker:<8} {r['segment']:<20} {r['osd']:<12.3f} {gain:<15.3f} {better}")

    # Summary
    print(f"\n" + "="*80)
    print(f"SUMMARY FOR T13")
    print("="*80)

    if ssd is not None:
        print(f"\nCurrent assignment: Drone {t13_drone}")
        print(f"Current SSD: {ssd:.3f}")

        better_segments = [r for r in osd_results if r['improvement'] > 0]

        if better_segments:
            best = better_segments[0]
            print(f"\nBest alternative trajectory:")
            print(f"  Drone {best['drone']}, segment {best['segment']}")
            print(f"  OSD = {best['osd']:.3f}")
            print(f"  Gain = {best['improvement']:.3f} (SSD - OSD)")
            print(f"\n✓ Swap Closer would move T13 to Drone {best['drone']}")
        else:
            print(f"\n✓ No better trajectory found - T13 is optimally placed on Drone {t13_drone}")
    else:
        print(f"\nT13 is currently UNVISITED (not in any route)")
        if osd_results:
            best = osd_results[0]
            print(f"\nClosest trajectory:")
            print(f"  Drone {best['drone']}, segment {best['segment']}")
            print(f"  OSD = {best['osd']:.3f}")
            print(f"\n✓ Insert Missed would likely try to add T13 to Drone {best['drone']}")

    print("\n" + "="*80)

if __name__ == "__main__":
    main()
