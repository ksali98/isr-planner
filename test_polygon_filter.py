"""
Test script to verify polygon filtering in Insert Missed optimizer.

This script tests that T15 is correctly identified as inside a SAM polygon
and filtered out by the Insert Missed optimizer.
"""
import json
import sys
sys.path.insert(0, 'server')

# Load the environment
with open('/Users/kamalali/Downloads/isr_env2512140810_1.json', 'r') as f:
    env = json.load(f)

drone_configs = env['drone_configs']

print("="*80)
print("TESTING POLYGON FILTERING IN INSERT MISSED")
print("="*80)

# Test target T15
print("\n1. VERIFYING T15 IS INSIDE SAM POLYGON:")
print("-" * 80)

targets = env['targets']
sams = env['sams']

# Find T15
t15 = next(t for t in targets if t['id'] == 'T15')
print(f"  T15 position: ({t15['x']:.2f}, {t15['y']:.2f})")
print(f"  T15 priority: {t15['priority']}")

# Check each SAM
import math
for i, sam in enumerate(sams):
    sam_x, sam_y = sam['pos']
    sam_range = sam['range']

    dx = t15['x'] - sam_x
    dy = t15['y'] - sam_y
    distance = math.sqrt(dx * dx + dy * dy)

    inside = "✓ INSIDE" if distance < sam_range else "✗ outside"
    print(f"\n  SAM #{i+1}:")
    print(f"    Center: ({sam_x:.2f}, {sam_y:.2f})")
    print(f"    Range: {sam_range}")
    print(f"    Distance to T15: {distance:.2f}")
    print(f"    Status: {inside}")

# Simulate a route with T15 for D5
print("\n2. TESTING INSERT MISSED WITH T15 IN ROUTE:")
print("-" * 80)

initial_routes = {
    '1': {
        'route': ['A1', 'T26', 'T28', 'T22', 'T17', 'A1'],
        'sequence': 'A1,T26,T28,T22,T17,A1',
        'distance': 100.0,
        'fuel_budget': 150,
        'points': 0
    },
    '3': {
        'route': ['A3', 'T29', 'T9', 'A3'],
        'sequence': 'A3,T29,T9,A3',
        'distance': 80.0,
        'fuel_budget': 150,
        'points': 0
    },
    '2': {'route': ['A2', 'A2'], 'sequence': 'A2,A2', 'distance': 0.0, 'fuel_budget': 150, 'points': 0},
    '4': {'route': ['A4', 'A4'], 'sequence': 'A4,A4', 'distance': 0.0, 'fuel_budget': 150, 'points': 0},
    '5': {'route': ['A5', 'A5'], 'sequence': 'A5,A5', 'distance': 0.0, 'fuel_budget': 150, 'points': 0}
}

# Run Insert Missed optimizer
from solver.post_optimizer import post_optimize_solution

solution = {
    'routes': initial_routes,
    'sequences': {k: v['sequence'] for k, v in initial_routes.items()}
}

try:
    optimized = post_optimize_solution(solution, env, drone_configs, None, None)

    print("\n3. RESULTS:")
    print("-" * 80)

    # Check if T15 was inserted
    t15_inserted = False
    for did in sorted(optimized['routes'].keys()):
        route = optimized['routes'][did]['route']
        if 'T15' in route:
            t15_inserted = True
            print(f"  ✗ FAIL: T15 was inserted into D{did}'s route: {route}")
            break

    if not t15_inserted:
        print(f"  ✓ SUCCESS: T15 was correctly filtered out (not inserted)")

    # Show all routes
    print("\n  Final routes:")
    for did in sorted(optimized['routes'].keys()):
        route = optimized['routes'][did]['route']
        complete = "COMPLETE" if (route and route[0].startswith('A') and route[-1].startswith('A')) else "INCOMPLETE"
        print(f"    D{did}: {route} ({complete})")

except Exception as e:
    print(f"\n✗ ERROR: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "="*80)
