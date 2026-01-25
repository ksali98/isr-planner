#!/usr/bin/env python3
"""
Test script to reproduce the checkpoint replan bug where solver returns ["D1_START"] with no trajectory.

The issue: At checkpoint replanning, drone fuel budgets need to account for distance already traveled.
"""

import json

# Simulate the 4th checkpoint replan from the logs
test_payload = {
    "env": {
        "targets": [
            {"id": "T3", "x": 40.7, "y": 69.8, "priority": 5},
            {"id": "T7", "x": 76.3, "y": 26.6, "priority": 5},
            {"id": "T8", "x": 7.0, "y": 62.5, "priority": 5},
            {"id": "T9", "x": 18.7, "y": 89.8, "priority": 5},
            {"id": "T10", "x": 59.6, "y": 73.3, "priority": 5},
            {"id": "T12", "x": 18.9, "y": 15.2, "priority": 5},
        ],
        "airports": [
            {"id": "A1", "x": 14.8, "y": 8.5},
            {"id": "A2", "x": 89.1, "y": 16.3},
        ],
        "synthetic_starts": {
            "D1_START": {"x": 45.0, "y": 69.2},  # D1's current position after 115km travel
            "D2_START": {"x": 63.4, "y": 59.9},  # D2's current position after 115km travel
        },
        "sams": [],
    },
    "drone_configs": {
        "1": {
            "enabled": True,
            "fuelBudget": 200,  # BUG: This should be (200 - 115) = 85 remaining!
            "start_id": "D1_START",
            "home_airport": "A1",
            "end_airport": "-",  # flexible
        },
        "2": {
            "enabled": True,
            "fuelBudget": 200,  # BUG: This should be (200 - 116) = 84 remaining!
            "start_id": "D2_START",
            "home_airport": "A2",
            "end_airport": "-",
        },
        "3": {"enabled": False},
        "4": {"enabled": False},
        "5": {"enabled": False},
    },
    "visited_targets": ["T1", "T2", "T4", "T5", "T6"],  # Already visited in segments 0-2
    "is_checkpoint_replan": True,
}

print("="*80)
print("CHECKPOINT REPLAN BUG REPRODUCTION TEST")
print("="*80)
print("\nProblem:")
print("- D1 traveled 115km, has ~85km fuel remaining")
print("- D2 traveled 116km, has ~84km fuel remaining")
print("- BUT drone_configs still shows fuelBudget=200")
print("- Solver can't find feasible routes with 200km budget from synthetic starts")
print("- Returns empty route: ['D1_START']")
print("\nSolution:")
print("- UI must track distanceTraveled per drone across segments")
print("- At checkpoint replan, calculate remaining_fuel = original_budget - distance_traveled")
print("- Send remaining_fuel as fuelBudget in drone_configs")
print("\nTest payload:")
print(json.dumps(test_payload, indent=2))

print("\n" + "="*80)
print("EXPECTED BEHAVIOR:")
print("="*80)
print("With correct remaining fuel (85km for D1, 84km for D2):")
print("- D1 can reach T3 (25km away) or T8 (38km away) and return")
print("- D2 can reach T10 (18km away) or T7 (43km away) and return")
print("- Solver should return valid routes, not empty ['D1_START']")
