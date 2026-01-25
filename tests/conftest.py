"""
Pytest configuration and fixtures for ISR Planner tests.

Fixtures provide common test data and setup for:
- Environments (airports, targets, SAMs)
- Drone configurations
- Solutions/routes
- API test client
"""

import os
import sys
from pathlib import Path

import pytest

# Ensure project root is in path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Set PYTHONPATH for subprocesses
os.environ["PYTHONPATH"] = str(PROJECT_ROOT)


# =============================================================================
# Test Environment Fixtures
# =============================================================================

@pytest.fixture
def simple_env():
    """Simple environment with 2 airports, 4 targets, no SAMs."""
    return {
        "airports": [
            {"id": "A1", "label": "A1", "x": 0, "y": 0},
            {"id": "A2", "label": "A2", "x": 100, "y": 0},
        ],
        "targets": [
            {"id": "T1", "label": "T1", "x": 30, "y": 40, "priority": 8, "type": "a"},
            {"id": "T2", "label": "T2", "x": 60, "y": 20, "priority": 5, "type": "b"},
            {"id": "T3", "label": "T3", "x": 80, "y": 60, "priority": 10, "type": "a"},
            {"id": "T4", "label": "T4", "x": 20, "y": 70, "priority": 3, "type": "c"},
        ],
        "sams": [],
    }


@pytest.fixture
def env_with_sams():
    """Environment with 2 airports, 4 targets, and 1 SAM."""
    return {
        "airports": [
            {"id": "A1", "label": "A1", "x": 0, "y": 0},
            {"id": "A2", "label": "A2", "x": 100, "y": 0},
        ],
        "targets": [
            {"id": "T1", "label": "T1", "x": 30, "y": 40, "priority": 8, "type": "a"},
            {"id": "T2", "label": "T2", "x": 60, "y": 20, "priority": 5, "type": "b"},
            {"id": "T3", "label": "T3", "x": 80, "y": 60, "priority": 10, "type": "a"},
            {"id": "T4", "label": "T4", "x": 20, "y": 70, "priority": 3, "type": "c"},
        ],
        "sams": [
            {"pos": [50, 40], "range": 15},
        ],
    }


@pytest.fixture
def complex_env():
    """Complex environment with multiple airports, targets, and SAMs."""
    return {
        "airports": [
            {"id": "A1", "label": "A1", "x": 0, "y": 0},
            {"id": "A2", "label": "A2", "x": 200, "y": 0},
            {"id": "A3", "label": "A3", "x": 100, "y": 200},
        ],
        "targets": [
            {"id": f"T{i}", "label": f"T{i}",
             "x": 20 + (i * 20) % 180,
             "y": 20 + (i * 15) % 160,
             "priority": 10 - (i % 8),
             "type": ["a", "b", "c"][i % 3]}
            for i in range(1, 13)
        ],
        "sams": [
            {"pos": [60, 80], "range": 20},
            {"pos": [140, 60], "range": 25},
            {"pos": [100, 140], "range": 15},
        ],
    }


# =============================================================================
# Drone Configuration Fixtures
# =============================================================================

@pytest.fixture
def single_drone_config():
    """Configuration for a single drone."""
    return {
        "1": {
            "enabled": True,
            "fuelBudget": 300,
            "homeAirport": "A1",
            "accessibleTargets": [],  # Empty = all targets
        }
    }


@pytest.fixture
def two_drone_config():
    """Configuration for two drones."""
    return {
        "1": {
            "enabled": True,
            "fuelBudget": 200,
            "homeAirport": "A1",
            "accessibleTargets": [],  # All targets
        },
        "2": {
            "enabled": True,
            "fuelBudget": 150,
            "homeAirport": "A2",
            "accessibleTargets": ["T2", "T3"],  # Limited access
        },
    }


@pytest.fixture
def five_drone_config():
    """Configuration for five drones (typical production use)."""
    return {
        str(i): {
            "enabled": True,
            "fuelBudget": 250,
            "homeAirport": "A1" if i <= 3 else "A2",
            "accessibleTargets": [],
        }
        for i in range(1, 6)
    }


# =============================================================================
# Solution Fixtures
# =============================================================================

@pytest.fixture
def simple_solution():
    """A simple single-drone solution."""
    return {
        "routes": {
            "1": {
                "route": ["A1", "T1", "T2", "A1"],
                "distance": 120.5,
                "trajectory": [[0, 0], [30, 40], [60, 20], [0, 0]],
            }
        },
        "sequences": {"1": "A1,T1,T2,A1"},
        "missed_targets": ["T3", "T4"],
    }


@pytest.fixture
def multi_drone_solution():
    """A solution with multiple drones."""
    return {
        "routes": {
            "1": {
                "route": ["A1", "T1", "T4", "A1"],
                "distance": 100.0,
                "trajectory": [[0, 0], [30, 40], [20, 70], [0, 0]],
            },
            "2": {
                "route": ["A2", "T2", "T3", "A2"],
                "distance": 90.0,
                "trajectory": [[100, 0], [60, 20], [80, 60], [100, 0]],
            }
        },
        "sequences": {
            "1": "A1,T1,T4,A1",
            "2": "A2,T2,T3,A2",
        },
        "missed_targets": [],
    }


# =============================================================================
# API Test Fixtures
# =============================================================================

@pytest.fixture
def test_client():
    """FastAPI test client for API testing."""
    from fastapi.testclient import TestClient
    from server.main import app

    return TestClient(app)


@pytest.fixture
def async_test_client():
    """Async FastAPI test client for async API testing."""
    import httpx
    from server.main import app

    return httpx.AsyncClient(app=app, base_url="http://test")


# =============================================================================
# Orchestrator Fixtures
# =============================================================================

@pytest.fixture
def orchestrator(simple_env):
    """Get orchestrator instance with simple environment."""
    from server.agents.mission_orchestration_tools import get_orchestrator

    orch = get_orchestrator()
    orch.inspector.set_context(simple_env)
    return orch


# =============================================================================
# Skip Markers
# =============================================================================

@pytest.fixture
def requires_anthropic_key():
    """Skip test if ANTHROPIC_API_KEY is not set."""
    if not os.getenv("ANTHROPIC_API_KEY"):
        pytest.skip("ANTHROPIC_API_KEY not set")


# =============================================================================
# Test Data Paths
# =============================================================================

@pytest.fixture
def test_data_dir():
    """Path to test data directory."""
    return PROJECT_ROOT / "tests" / "data"


@pytest.fixture
def sample_env_file(test_data_dir):
    """Path to a sample environment JSON file."""
    sample = test_data_dir / "sample_env.json"
    if not sample.exists():
        return None
    return sample
