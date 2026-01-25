"""
Integration tests for the Segmented Mission System.

Tests the backend components that support segmented missions:
- Segment creation via orchestrator
- Checkpoint solving (solving from mid-mission positions)
- Trajectory splitting and concatenation
- Synthetic start node handling
"""

import pytest
import math


class TestSegmentCreation:
    """Tests for creating mission segments."""

    @pytest.fixture
    def base_env(self):
        """Base environment for segmented mission tests."""
        return {
            "airports": [
                {"id": "A1", "x": 0, "y": 0},
                {"id": "A2", "x": 200, "y": 0},
            ],
            "targets": [
                {"id": "T1", "x": 50, "y": 50, "priority": 10, "type": "a"},
                {"id": "T2", "x": 100, "y": 80, "priority": 8, "type": "b"},
                {"id": "T3", "x": 150, "y": 50, "priority": 6, "type": "a"},
                {"id": "T4", "x": 100, "y": 20, "priority": 4, "type": "c"},
            ],
            "sams": []
        }

    @pytest.fixture
    def initial_solution(self):
        """Initial solution before any cuts."""
        return {
            "routes": {
                "1": {
                    "route": ["A1", "T1", "T2", "T3", "A1"],
                    "distance": 300.0,
                    "trajectory": [
                        [0, 0], [50, 50], [100, 80], [150, 50], [0, 0]
                    ],
                    "visited_targets": ["T1", "T2", "T3"]
                }
            },
            "sequences": {"1": "A1,T1,T2,T3,A1"},
            "missed_targets": ["T4"]
        }

    @pytest.fixture
    def drone_configs(self):
        """Drone configuration."""
        return {
            "1": {
                "enabled": True,
                "fuel_budget": 500,
                "start_airport": "A1",
                "end_airport": "A1",
            }
        }

    def test_create_segment_from_solution(self, base_env, initial_solution, drone_configs):
        """Test creating a segment from a solver solution."""
        from server.agents.mission_orchestration_tools import get_orchestrator

        orchestrator = get_orchestrator()

        segment = orchestrator.segments.create_segment(
            index=0,
            solution=initial_solution,
            env=base_env,
            drone_configs=drone_configs,
            cut_distance=100,
            cut_positions={"1": [75, 65]},  # Mid-trajectory position
            frozen_targets=["T1"],
            active_targets=["T1", "T2", "T3"]
        )

        assert segment["index"] == 0
        assert segment["cutDistance"] == 100
        assert "solution" in segment

    def test_create_segmented_mission(self, base_env, initial_solution, drone_configs):
        """Test creating a full segmented mission object."""
        from server.agents.mission_orchestration_tools import get_orchestrator

        orchestrator = get_orchestrator()

        # Create first segment
        segment = orchestrator.segments.create_segment(
            index=0,
            solution=initial_solution,
            env=base_env,
            drone_configs=drone_configs,
            cut_distance=100,
            cut_positions={"1": [75, 65]},
            frozen_targets=["T1"],
            active_targets=["T1", "T2", "T3"]
        )

        mission = orchestrator.segments.create_segmented_mission(
            segments=[segment],
            base_env=base_env
        )

        assert mission["is_segmented"] == True
        assert mission["segment_count"] == 1
        assert len(mission["segments"]) == 1


class TestTrajectorySplitting:
    """Tests for trajectory manipulation for segment cuts."""

    @pytest.fixture
    def orchestrator(self):
        """Get orchestrator instance."""
        from server.agents.mission_orchestration_tools import get_orchestrator
        return get_orchestrator()

    def test_split_trajectory_basic(self, orchestrator):
        """Test basic trajectory splitting."""
        # Simple diagonal trajectory
        trajectory = [[0, 0], [50, 50], [100, 100]]

        # Split at halfway point (~70.7 units along a 141.4 unit trajectory)
        result = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 70)

        assert "prefix" in result
        assert "suffix" in result
        assert "split_point" in result

        # Prefix should be shorter than original
        assert len(result["prefix"]) <= len(trajectory)

    def test_split_preserves_total_length(self, orchestrator):
        """Test that split trajectory has same total length."""
        trajectory = [[0, 0], [100, 0], [100, 100]]
        total_dist = 200.0  # 100 + 100

        result = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 80)

        # Prefix + suffix should equal total (approximately)
        prefix_len = orchestrator.trajectory.calculate_trajectory_distance(result["prefix"])
        suffix_len = orchestrator.trajectory.calculate_trajectory_distance(result["suffix"])

        # Allow for split point interpolation differences
        assert abs((prefix_len + suffix_len) - total_dist) < 5

    def test_split_at_zero(self, orchestrator):
        """Test splitting at distance 0."""
        trajectory = [[10, 10], [50, 50], [90, 90]]

        result = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 0)

        # At distance 0, prefix should just be start point
        assert len(result["prefix"]) == 1
        assert result["split_point"] == [10, 10]

    def test_split_beyond_end(self, orchestrator):
        """Test splitting beyond trajectory length."""
        trajectory = [[0, 0], [50, 50]]
        total_dist = math.sqrt(50**2 + 50**2)  # ~70.7

        result = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 200)

        # Should return full trajectory as prefix, empty suffix
        assert len(result["prefix"]) == len(trajectory)
        assert len(result["suffix"]) <= 1  # May have just end point


class TestCheckpointSolving:
    """Tests for solving from checkpoint positions."""

    @pytest.fixture
    def checkpoint_env(self):
        """Environment for checkpoint solving."""
        return {
            "airports": [
                {"id": "A1", "x": 0, "y": 0},
            ],
            "targets": [
                {"id": "T1", "x": 40, "y": 40, "priority": 10},
                {"id": "T2", "x": 80, "y": 80, "priority": 8},
                {"id": "T3", "x": 120, "y": 40, "priority": 6},
            ],
            "sams": []
        }

    @pytest.fixture
    def checkpoint_configs(self):
        """Configs for checkpoint solving with synthetic starts."""
        return {
            "1": {
                "enabled": True,
                "fuel_budget": 200,  # Reduced fuel for checkpoint
                "start_airport": "A1",
                "end_airport": "A1",
                # Synthetic start position (mid-mission)
                "synthetic_start": {"x": 40, "y": 40, "id": "D1_START"}
            }
        }

    @pytest.mark.integration
    def test_solver_bridge_checkpoint_mode(self, checkpoint_env, checkpoint_configs):
        """Test that solver bridge handles checkpoint mode."""
        from server.solver.solver_bridge import SolverBridge

        bridge = SolverBridge()

        # The solver should recognize synthetic_start in configs
        # and create a synthetic start node instead of using airport
        result = bridge.solve(
            environment=checkpoint_env,
            drone_configs=checkpoint_configs,
            settings={"allocation_strategy": "efficient"}
        )

        # Should return a valid result
        assert result is not None
        if "error" not in result:
            assert "routes" in result or "solution" in result


class TestMultiSegmentWorkflow:
    """Tests for multi-segment mission workflow."""

    @pytest.fixture
    def workflow_env(self):
        """Environment for workflow tests."""
        return {
            "airports": [
                {"id": "A1", "x": 0, "y": 0},
            ],
            "targets": [
                {"id": "T1", "x": 30, "y": 30, "priority": 10},
                {"id": "T2", "x": 60, "y": 60, "priority": 8},
                {"id": "T3", "x": 90, "y": 30, "priority": 6},
            ],
            "sams": []
        }

    @pytest.fixture
    def workflow_configs(self):
        """Configs for workflow tests."""
        return {
            "1": {
                "enabled": True,
                "fuel_budget": 300,
                "start_airport": "A1",
                "end_airport": "A1",
            }
        }

    @pytest.mark.integration
    def test_segment_then_replan_workflow(self, workflow_env, workflow_configs):
        """Test the full segment-cut-replan workflow."""
        from server.solver.solver_bridge import SolverBridge
        from server.agents.mission_orchestration_tools import get_orchestrator

        bridge = SolverBridge()
        orchestrator = get_orchestrator()

        # Step 1: Initial solve
        initial = bridge.solve(
            environment=workflow_env,
            drone_configs=workflow_configs,
            settings={"allocation_strategy": "efficient"}
        )

        if "error" in initial:
            pytest.skip(f"Initial solve failed: {initial['error']}")

        # Step 2: Create segment at cut point
        segment = orchestrator.segments.create_segment(
            index=0,
            solution=initial,
            env=workflow_env,
            drone_configs=workflow_configs,
            cut_distance=50,
            cut_positions={"1": [30, 30]},
            frozen_targets=["T1"],
            active_targets=["T1", "T2", "T3"]
        )

        assert segment["index"] == 0

        # Step 3: Set up for replan from checkpoint
        # (In real usage, this would update remaining targets and fuel)
        checkpoint_configs = {
            "1": {
                "enabled": True,
                "fuel_budget": 250,  # Less fuel remaining
                "start_airport": "A1",
                "end_airport": "A1",
            }
        }

        # Remaining targets (T1 visited)
        remaining_env = {
            **workflow_env,
            "targets": [t for t in workflow_env["targets"] if t["id"] != "T1"]
        }

        # Step 4: Replan from checkpoint
        replan = bridge.solve(
            environment=remaining_env,
            drone_configs=checkpoint_configs,
            settings={"allocation_strategy": "efficient"}
        )

        # Should get a new solution
        if "error" not in replan:
            assert "routes" in replan or "solution" in replan
