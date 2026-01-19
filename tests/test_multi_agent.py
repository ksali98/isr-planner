#!/usr/bin/env python3
"""
Quick test script for the multi-agent ISR system.
Tests the multi-agent v2 directly without switching the main agent file.
"""

import sys
import os

# Add project to path
sys.path.insert(0, os.path.dirname(__file__))
os.chdir(os.path.dirname(__file__))

# Load environment
from dotenv import load_dotenv
load_dotenv()

# Import multi-agent module directly
from server.agents.isr_agent_multi_v2 import run_isr_agent

# Test environment: 2 airports, 8 targets, no SAMs
TEST_ENV = {
    "grid_size": 100,
    "airports": [
        {"id": "A1", "x": 10, "y": 10},
        {"id": "A2", "x": 90, "y": 90}
    ],
    "targets": [
        {"id": "T1", "x": 20, "y": 30, "priority": 8, "type": "a"},
        {"id": "T2", "x": 30, "y": 50, "priority": 6, "type": "b"},
        {"id": "T3", "x": 40, "y": 20, "priority": 9, "type": "a"},
        {"id": "T4", "x": 60, "y": 40, "priority": 5, "type": "c"},
        {"id": "T5", "x": 70, "y": 60, "priority": 7, "type": "b"},
        {"id": "T6", "x": 80, "y": 30, "priority": 4, "type": "a"},
        {"id": "T7", "x": 50, "y": 70, "priority": 10, "type": "a"},
        {"id": "T8", "x": 25, "y": 80, "priority": 3, "type": "c"},
    ],
    "sams": []
}

# Test drone configs: 2 drones
TEST_CONFIGS = {
    "1": {
        "enabled": True,
        "fuel_budget": 150,
        "start_airport": "A1",
        "end_airport": "A1",
        "target_access": {"a": True, "b": True, "c": True, "d": True, "e": True}
    },
    "2": {
        "enabled": True,
        "fuel_budget": 150,
        "start_airport": "A2",
        "end_airport": "A2",
        "target_access": {"a": True, "b": True, "c": True, "d": True, "e": True}
    },
    "3": {"enabled": False},
    "4": {"enabled": False},
    "5": {"enabled": False}
}


def test_basic_planning():
    """Test basic multi-drone planning."""
    print("\n" + "=" * 60)
    print("TEST 1: Basic Multi-Drone Planning")
    print("=" * 60)

    query = "Plan optimal routes for all enabled drones"

    print(f"\nQuery: {query}")
    print("-" * 40)

    result = run_isr_agent(TEST_ENV, query, TEST_CONFIGS)

    print("\n--- RESULT ---")
    print(f"Routes: {result.get('routes', {})}")
    print(f"Total Points: {result.get('total_points', 0)}")
    print(f"Total Fuel: {result.get('total_fuel', 0):.1f}")

    # Check for ROUTE_D in response
    response = result.get("response", "")
    if "ROUTE_D" in response:
        print("\n✅ Routes found in response")
    else:
        print("\n⚠️ No ROUTE_D found in response")
        print("\nFull response:")
        print(response[:1000])

    return result


def test_priority_constraints():
    """Test planning with priority constraints."""
    print("\n" + "=" * 60)
    print("TEST 2: Priority-Constrained Planning")
    print("=" * 60)

    query = "D1 should visit priority>=7 targets, D2 visits priority<7 targets"

    print(f"\nQuery: {query}")
    print("-" * 40)

    result = run_isr_agent(TEST_ENV, query, TEST_CONFIGS)

    print("\n--- RESULT ---")
    print(f"Routes: {result.get('routes', {})}")
    print(f"Total Points: {result.get('total_points', 0)}")

    return result


if __name__ == "__main__":
    print("=" * 60)
    print("MULTI-AGENT ISR SYSTEM TEST")
    print("=" * 60)

    # Run tests
    try:
        test_basic_planning()
    except Exception as e:
        print(f"\n❌ Test 1 failed: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "=" * 60)
    print("TESTS COMPLETE")
    print("=" * 60)
