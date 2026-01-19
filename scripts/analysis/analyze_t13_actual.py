#!/usr/bin/env python3
"""
Calculate T13's actual SSD and OSD from the given route
Route: A2,T5,T16,T6,T8,T11,T10,T12,T13,A1
"""

import sys
import json
import math
sys.path.insert(0, '/Users/kamalali/isr-planner/server')

from solver.sam_distance_matrix import calculate_sam_aware_matrix
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
    env_file = '/Users/kamalali/Downloads/isr_env2512131921_1.json'
    with open(env_file, 'r') as f:
        env = json.load(f)

    # Build waypoint positions
    waypoint_positions = {}
    for t in env.get('targets', []):
        waypoint_positions[str(t['id'])] = (float(t['x']), float(t['y']))
    for a in env.get('airports', []):
        waypoint_positions[str(a['id'])] = (float(a['x']), float(a['y']))

    # Test route for Drone 5 - the problematic T22→A1 segment
    d5_route = ['A5', 'T22', 'A1']

    print("\n" + "="*80)
    print("DRONE 5 ROUTE ANALYSIS - T22→A1 SEGMENT")
    print("="*80)
    print(f"Route: {' → '.join(d5_route)}")

    # Generate trajectory for this route using ISR trajectory planner
    print("\n" + "="*80)
    print("GENERATING TRAJECTORY WITH SAM AVOIDANCE")
    print("="*80)

    # Initialize trajectory planner
    sams = env.get('sams', [])
    planner = ISRTrajectoryPlanner(sams)

    # Generate trajectory - this will show the debug output for T22→A1
    trajectory = planner.generate_trajectory(d5_route, waypoint_positions, drone_id='5')

    print(f"\n✓ Generated trajectory with {len(trajectory)} vertices")
    print("\n" + "="*80)

if __name__ == "__main__":
    main()
