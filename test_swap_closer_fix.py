#!/usr/bin/env python3
"""
Test script for Swap Closer optimization fix
Tests with isr_env2512122217_1.json
"""
import json
import sys
import os

# Add path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))

from solver import solver_bridge
from solver.post_optimizer import TrajectorySwapOptimizer

def load_environment(filepath):
    """Load environment from JSON file"""
    with open(filepath, 'r') as f:
        return json.load(f)

def main():
    # Load the test environment
    env_path = "/Users/kamalali/Downloads/isr_env2512122217_1.json"
    print(f"Loading environment from {env_path}")
    env = load_environment(env_path)

    print(f"\nEnvironment Summary:")
    print(f"  Airports: {len(env.get('airports', []))}")
    print(f"  Targets: {len(env.get('targets', []))}")
    print(f"  SAMs: {len(env.get('sams', []))}")
    print(f"  Drones: {len(env.get('drone_configs', {}))}")

    # Get drone configs
    drone_configs = env.get('drone_configs', {})

    # Solve the problem using solve_mission_with_allocation
    print("\nRunning initial solver with target allocation...")
    initial_solution = solver_bridge.solve_mission_with_allocation(
        env,
        drone_configs,
        post_optimize=False  # Don't use optimizer yet, we'll apply it separately
    )

    print("\nInitial Solution:")
    for drone_id, route_data in initial_solution.get('routes', {}).items():
        route = route_data.get('route', [])
        sequence = ' -> '.join(str(wp) for wp in route)
        print(f"  D{drone_id}: {sequence}")
        print(f"    Distance: {route_data.get('distance', 0):.1f}, Points: {route_data.get('points', 0)}")

    # Apply post-optimization
    print("\n" + "="*60)
    print("Running Post-Optimization (Swap Closer)...")
    print("="*60)

    print(f"\n  Solution keys: {list(initial_solution.keys())}")

    optimizer = TrajectorySwapOptimizer()

    # Set distance matrix if available
    if 'distance_matrix' in initial_solution:
        print(f"\n  Setting distance matrix with {len(initial_solution['distance_matrix'].get('labels', []))} nodes")
        optimizer.set_distance_matrix(initial_solution['distance_matrix'])
    else:
        print(f"\n  ⚠️  WARNING: No distance matrix in solution!")

    optimized_solution = optimizer.optimize(
        initial_solution,
        env,
        drone_configs
    )

    print("\nOptimized Solution:")
    changes_made = False
    for drone_id, route_data in optimized_solution.get('routes', {}).items():
        route = route_data.get('route', [])
        initial_route = initial_solution.get('routes', {}).get(drone_id, {}).get('route', [])

        sequence = ' -> '.join(str(wp) for wp in route)
        print(f"  D{drone_id}: {sequence}")
        print(f"    Distance: {route_data.get('distance', 0):.1f}, Points: {route_data.get('points', 0)}")

        # Check if route changed
        if route != initial_route:
            print(f"    ⚠️  CHANGED from: {' -> '.join(str(wp) for wp in initial_route)}")
            changes_made = True

    if not changes_made:
        print("\n  ℹ️  No changes made by optimizer (routes unchanged)")

    # Check for bugs: targets after end airports
    print("\n" + "="*60)
    print("Validation: Checking for targets after end airports...")
    print("="*60)

    bugs_found = False
    for drone_id, route_data in optimized_solution.get('routes', {}).items():
        route = route_data.get('route', [])
        if len(route) < 2:
            continue

        # Find the last occurrence of an airport
        last_airport_idx = None
        for i in range(len(route) - 1, -1, -1):
            if str(route[i]).startswith('A'):
                last_airport_idx = i
                break

        if last_airport_idx is not None and last_airport_idx < len(route) - 1:
            # Check if there are any targets after the last airport
            targets_after = [wp for wp in route[last_airport_idx + 1:] if str(wp).startswith('T')]
            if targets_after:
                print(f"  ❌ BUG in D{drone_id}: Targets after end airport {route[last_airport_idx]}: {targets_after}")
                bugs_found = True

    if not bugs_found:
        print("  ✅ No bugs found! All targets are before end airports.")

    print("\nTest complete!")

if __name__ == "__main__":
    main()
