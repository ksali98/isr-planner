"""
Test script to diagnose Insert Missed incomplete route bug.

This script simulates what happens when Insert Missed is run on the user's routes.
"""
import json
import sys
sys.path.insert(0, 'server')

# Load the environment
with open('/Users/kamalali/Downloads/isr_env2512140810_1.json', 'r') as f:
    env = json.load(f)

drone_configs = env['drone_configs']

print("="*80)
print("TESTING INSERT MISSED BUG")
print("="*80)

# Simulate the EXACT routes the user described BEFORE Insert Missed
print("\n1. BEFORE INSERT MISSED:")
print("-" * 80)
initial_routes = {
    '1': {
        'route': ['A1', 'T26', 'T28', 'T22', 'T17', 'A1'],
        'sequence': 'A1,T26,T28,T22,T17,A1',
        'distance': 100.0,  # placeholder
        'fuel_budget': 150,
        'points': 0
    },
    '3': {
        'route': ['A3', 'T29', 'T9', 'A3'],
        'sequence': 'A3,T29,T9,A3',
        'distance': 80.0,  # placeholder
        'fuel_budget': 150,
        'points': 0
    },
    '2': {
        'route': ['A2', 'A2'],  # Drone 2 has no targets
        'sequence': 'A2,A2',
        'distance': 0.0,
        'fuel_budget': 150,
        'points': 0
    },
    '4': {
        'route': ['A4', 'A4'],  # Drone 4 has no targets
        'sequence': 'A4,A4',
        'distance': 0.0,
        'fuel_budget': 150,
        'points': 0
    },
    '5': {
        'route': ['A5', 'A5'],  # Drone 5 has no targets
        'sequence': 'A5,A5',
        'distance': 0.0,
        'fuel_budget': 150,
        'points': 0
    }
}

for did, route_data in sorted(initial_routes.items()):
    route = route_data['route']
    print(f"  D{did}: {route} (length={len(route)})")

# Calculate which targets are missed
all_targets = {f"T{i}" for i in range(1, 34)}
visited_targets = set()
for route_data in initial_routes.values():
    for wp in route_data['route']:
        if wp.startswith('T'):
            visited_targets.add(wp)

missed_before = all_targets - visited_targets
print(f"\n  Missed targets ({len(missed_before)}): {sorted(missed_before)}")

# Run Insert Missed optimizer
print("\n2. RUNNING INSERT MISSED:")
print("-" * 80)

from solver.post_optimizer import post_optimize_solution

solution = {
    'routes': initial_routes,
    'sequences': {k: v['sequence'] for k, v in initial_routes.items()}
}

try:
    optimized = post_optimize_solution(solution, env, drone_configs, None, None)

    print("\n3. AFTER INSERT MISSED:")
    print("-" * 80)

    # Check which routes changed
    for did in sorted(optimized['routes'].keys()):
        new_route = optimized['routes'][did]['route']
        old_route = initial_routes[did]['route']

        changed = "✗ CHANGED" if new_route != old_route else "✓ SAME"
        complete = "✓ COMPLETE" if new_route[0].startswith('A') and new_route[-1].startswith('A') else "✗ INCOMPLETE"

        print(f"  D{did}: {new_route}")
        print(f"       Status: {changed}, {complete}, length={len(new_route)}")

        if new_route != old_route:
            print(f"       OLD: {old_route}")
            print(f"       NEW: {new_route}")

    # Check missed targets after
    visited_after = set()
    for route_data in optimized['routes'].values():
        for wp in route_data['route']:
            if wp.startswith('T'):
                visited_after.add(wp)

    missed_after = all_targets - visited_after
    print(f"\n  Missed targets after ({len(missed_after)}): {sorted(missed_after)}")

    # Show what was inserted
    newly_visited = visited_after - visited_targets
    if newly_visited:
        print(f"  ✓ Newly inserted ({len(newly_visited)}): {sorted(newly_visited)}")

    # Show what was lost
    lost_targets = visited_targets - visited_after
    if lost_targets:
        print(f"  ✗ LOST TARGETS ({len(lost_targets)}): {sorted(lost_targets)}")
        print("\n  🚨 BUG CONFIRMED: Insert Missed is removing existing targets!")

except Exception as e:
    print(f"\n✗ ERROR: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "="*80)
