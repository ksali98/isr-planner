#!/usr/bin/env python3
"""
Test script for ISR Multi-Agent v4 (Reasoning Agents)

Tests:
1. Question answering (drone configs, targets, etc.)
2. Basic solve request
3. Command-based request
"""

import sys
import os

# Add paths
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("PYTHONPATH", os.path.dirname(os.path.abspath(__file__)))

from dotenv import load_dotenv
# Try multiple .env locations
load_dotenv()  # Current directory
load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "isr_benchmark", ".env"))  # sibling dir

# Simple test environment
TEST_ENV = {
    "airports": [
        {"id": "A1", "label": "A1", "x": 0, "y": 0},
        {"id": "A2", "label": "A2", "x": 100, "y": 0},
    ],
    "targets": [
        {"id": "T1", "label": "T1", "x": 30, "y": 40, "priority": 8},
        {"id": "T2", "label": "T2", "x": 60, "y": 20, "priority": 5},
        {"id": "T3", "label": "T3", "x": 80, "y": 60, "priority": 10},
        {"id": "T4", "label": "T4", "x": 20, "y": 70, "priority": 3},
    ],
    "sams": [
        {"pos": [50, 40], "range": 15},
    ],
}

TEST_CONFIGS = {
    "1": {
        "enabled": True,
        "fuelBudget": 200,
        "homeAirport": "A1",
        "accessibleTargets": [],  # ALL
    },
    "2": {
        "enabled": True,
        "fuelBudget": 150,
        "homeAirport": "A2",
        "accessibleTargets": ["T2", "T3"],  # Limited
    },
}


def test_question():
    """Test question answering."""
    print("\n" + "=" * 60)
    print("TEST 1: Question Answering")
    print("=" * 60)

    from server.agents.isr_agent_multi_v4 import run_multi_agent_v4

    result = run_multi_agent_v4(
        user_message="What drone configuration are you using?",
        environment=TEST_ENV,
        drone_configs=TEST_CONFIGS,
    )

    print("\nRESPONSE:")
    print(result.get("response", "No response"))
    print("\n" + "-" * 40)

    return result


def test_solve():
    """Test basic solve request."""
    print("\n" + "=" * 60)
    print("TEST 2: Solve Request")
    print("=" * 60)

    from server.agents.isr_agent_multi_v4 import run_multi_agent_v4

    result = run_multi_agent_v4(
        user_message="Solve - plan optimal mission",
        environment=TEST_ENV,
        drone_configs=TEST_CONFIGS,
    )

    print("\nRESPONSE:")
    print(result.get("response", "No response"))

    print("\nROUTES:")
    for did, route_data in result.get("routes", {}).items():
        print(f"  D{did}: {route_data}")

    print("\nSUGGESTIONS:")
    for s in result.get("suggestions", []):
        print(f"  - {s}")

    print("\n" + "-" * 40)

    return result


def test_command():
    """Test command-based request."""
    print("\n" + "=" * 60)
    print("TEST 3: Command Request")
    print("=" * 60)

    from server.agents.isr_agent_multi_v4 import run_multi_agent_v4

    result = run_multi_agent_v4(
        user_message="D1 must visit T1 and T4. D2 handles T2 and T3.",
        environment=TEST_ENV,
        drone_configs=TEST_CONFIGS,
    )

    print("\nRESPONSE:")
    print(result.get("response", "No response"))

    print("\nROUTES:")
    for did, route_data in result.get("routes", {}).items():
        print(f"  D{did}: {route_data}")

    print("\n" + "-" * 40)

    return result


if __name__ == "__main__":
    print("=" * 60)
    print("ISR Multi-Agent v4 Test Suite")
    print("=" * 60)

    # Check for API key
    if not os.getenv("ANTHROPIC_API_KEY"):
        print("ERROR: ANTHROPIC_API_KEY not set")
        sys.exit(1)

    # Run tests
    try:
        test_question()
        test_solve()
        test_command()

        print("\n" + "=" * 60)
        print("ALL TESTS COMPLETED")
        print("=" * 60)

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
