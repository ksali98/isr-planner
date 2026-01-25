"""
Integration tests for the LangGraph Multi-Agent System.

Tests the v4 agent system:
- Question answering
- Mission planning/solving
- Command interpretation
- State management

Note: These tests require ANTHROPIC_API_KEY to be set.
"""

import pytest
import os


@pytest.fixture
def agent_env():
    """Test environment for agent tests."""
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
def agent_drone_configs():
    """Drone configs for agent tests."""
    return {
        "1": {
            "enabled": True,
            "fuelBudget": 200,
            "homeAirport": "A1",
            "accessibleTargets": [],  # All
        },
        "2": {
            "enabled": True,
            "fuelBudget": 150,
            "homeAirport": "A2",
            "accessibleTargets": ["T2", "T3"],  # Limited
        },
    }


def requires_api_key():
    """Check if ANTHROPIC_API_KEY is available."""
    return os.getenv("ANTHROPIC_API_KEY") is not None


@pytest.mark.integration
@pytest.mark.requires_api_key
@pytest.mark.skipif(not requires_api_key(), reason="ANTHROPIC_API_KEY not set")
class TestMultiAgentV4:
    """Integration tests for Multi-Agent v4 system."""

    def test_question_answering(self, agent_env, agent_drone_configs):
        """Test that the agent can answer questions about the environment."""
        from server.agents.isr_agent_multi_v4 import run_multi_agent_v4

        result = run_multi_agent_v4(
            user_message="How many targets are in this environment?",
            environment=agent_env,
            drone_configs=agent_drone_configs,
        )

        assert "response" in result
        response = result["response"].lower()
        # Should mention "4" targets somehow
        assert "4" in response or "four" in response

    def test_basic_solve(self, agent_env, agent_drone_configs):
        """Test that the agent can solve a mission."""
        from server.agents.isr_agent_multi_v4 import run_multi_agent_v4

        result = run_multi_agent_v4(
            user_message="Plan an optimal mission to visit all targets.",
            environment=agent_env,
            drone_configs=agent_drone_configs,
        )

        assert "response" in result
        # Should return routes for the drones
        if "routes" in result:
            assert isinstance(result["routes"], dict)

    def test_command_interpretation(self, agent_env, agent_drone_configs):
        """Test that the agent interprets specific commands."""
        from server.agents.isr_agent_multi_v4 import run_multi_agent_v4

        result = run_multi_agent_v4(
            user_message="D1 should visit T1 and T4. D2 should handle T2 and T3.",
            environment=agent_env,
            drone_configs=agent_drone_configs,
        )

        assert "response" in result
        # Agent should acknowledge the command
        response = result["response"].lower()
        assert "d1" in response or "drone 1" in response or "routes" in result


@pytest.mark.integration
@pytest.mark.requires_api_key
@pytest.mark.skipif(not requires_api_key(), reason="ANTHROPIC_API_KEY not set")
class TestCoordinatorV4:
    """Integration tests for Coordinator v4."""

    def test_intent_classification(self, agent_env):
        """Test that the coordinator classifies intent correctly."""
        from server.agents.coordinator_v4 import CoordinatorV4

        coordinator = CoordinatorV4()

        # Test question intent
        result = coordinator.classify_intent("How many drones do we have?")
        assert result["intent"] in ["question", "query", "info"]

        # Test solve intent
        result = coordinator.classify_intent("Solve the mission")
        assert result["intent"] in ["solve", "plan", "optimize"]

        # Test command intent
        result = coordinator.classify_intent("Drone 1 must visit target 3")
        assert result["intent"] in ["command", "assign", "constraint"]
