#!/usr/bin/env python3
"""
Analyze T9 SSD and OSD values
"""
import json
import sys
import os

# Add path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

from solver import solver_bridge
from solver.post_optimizer import TrajectorySwapOptimizer

def main():
    # Load the test environment
    env_path = "test_env_t9.json"
    with open(env_path, 'r') as f:
        env = json.load(f)

    # Get drone configs
    drone_configs = env.get('drone_configs', {})

    print("="*60)
    print("Running solver to analyze T9...")
    print("="*60)

    # Solve the problem
    solution = solver_bridge.solve_mission_with_allocation(
        env,
        drone_configs,
        post_optimize=False
    )

    # Find which drone has T9
    print("\nRoutes:")
    t9_drone = None
    for drone_id, route_data in solution.get('routes', {}).items():
        route = route_data.get('route', [])
        sequence = ' -> '.join(str(wp) for wp in route)
        print(f"  D{drone_id}: {sequence}")
        if 'T9' in route:
            t9_drone = drone_id

    if not t9_drone:
        print("\n❌ T9 not found in any route!")
        return

    print(f"\n✅ T9 is in D{t9_drone}")

    # Now run Swap Closer to see T9's SSD/OSD
    print("\n" + "="*60)
    print("Running Swap Closer to calculate T9 SSD/OSD...")
    print("="*60)

    optimizer = TrajectorySwapOptimizer()

    # Set distance matrix if available
    if 'distance_matrix' in solution:
        optimizer.set_distance_matrix(solution['distance_matrix'])

    # Run optimizer (this will print T9's SSD and OSD)
    optimized = optimizer.optimize(solution, env, drone_configs)

    print("\n" + "="*60)
    print("Analysis complete!")
    print("="*60)

if __name__ == '__main__':
    main()
