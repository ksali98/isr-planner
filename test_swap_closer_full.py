#!/usr/bin/env python3
"""
Run full Swap Closer optimization to see all cascade moves
"""
import json
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'server'))
from solver import solver_bridge
from solver.post_optimizer import TrajectorySwapOptimizer

def main():
    with open("/Users/kamalali/Downloads/isr_env2512122217_1.json", 'r') as f:
        env = json.load(f)

    drone_configs = env.get('drone_configs', {})

    print("="*80)
    print("FULL SWAP CLOSER OPTIMIZATION TEST")
    print("="*80)

    # Solve initial mission
    print("\n1. Solving initial mission...")
    solution = solver_bridge.solve_mission_with_allocation(
        env, drone_configs, post_optimize=False
    )

    print("\nInitial Routes:")
    for drone_id, route_data in solution.get('routes', {}).items():
        route = route_data.get('route', [])
        sequence = ' → '.join(str(wp) for wp in route)
        print(f"  D{drone_id}: {sequence}")

    # Run Swap Closer optimization
    print("\n" + "="*80)
    print("2. Running Swap Closer Optimization...")
    print("="*80)

    optimizer = TrajectorySwapOptimizer()

    # Set distance matrix if available
    if 'distance_matrix' in solution:
        optimizer.set_distance_matrix(solution['distance_matrix'])

    # Run optimizer
    optimized = optimizer.optimize(solution, env, drone_configs)

    print("\n" + "="*80)
    print("3. FINAL OPTIMIZED ROUTES:")
    print("="*80)
    for drone_id, route_data in optimized.get('routes', {}).items():
        route = route_data.get('route', [])
        sequence = ' → '.join(str(wp) for wp in route)
        print(f"  D{drone_id}: {sequence}")

    # Show swaps made
    swaps = optimized.get('swaps_made', [])
    if swaps:
        print("\n" + "="*80)
        print(f"4. SWAPS MADE ({len(swaps)} total):")
        print("="*80)
        for i, swap in enumerate(swaps, 1):
            print(f"  {i}. {swap['target']}: D{swap['from_drone']} → D{swap['to_drone']}")
            print(f"     SSD: {swap.get('ssd', 0):.2f} → OSD: {swap.get('osd', 0):.2f}")
            print(f"     Savings: {swap.get('savings', 0):.2f}")
    else:
        print("\n" + "="*80)
        print("4. NO SWAPS MADE")
        print("="*80)

    print(f"\nTotal iterations: {optimized.get('iterations', 0)}")

if __name__ == '__main__':
    main()
