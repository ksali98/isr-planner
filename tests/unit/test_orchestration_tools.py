"""
Unit tests for Mission Orchestration Tools.

Tests the orchestrator singleton and its component modules:
- Inspector: Target filtering, priority/type queries
- Trajectory: Distance calculation, splitting
- Constraints: Fuel calculations, loiter costs
- Segments: Segment and mission creation
"""

import pytest


class TestInspector:
    """Tests for the Inspector module."""

    def test_get_all_targets(self, orchestrator, simple_env):
        """Test retrieving all targets from environment."""
        targets = orchestrator.inspector.get_all_targets()
        assert len(targets) == 4
        assert all("id" in t for t in targets)

    def test_get_targets_by_priority_exact(self, orchestrator):
        """Test filtering targets by exact priority."""
        priority_8 = orchestrator.inspector.get_targets_by_priority(exact_priority=8)
        assert len(priority_8) == 1
        assert priority_8[0]["id"] == "T1"

    def test_get_targets_by_priority_min(self, orchestrator):
        """Test filtering targets by minimum priority."""
        high_priority = orchestrator.inspector.get_targets_by_priority(min_priority=8)
        assert len(high_priority) == 2  # T1 (8) and T3 (10)

    def test_get_targets_by_type(self, orchestrator):
        """Test filtering targets by type."""
        type_a = orchestrator.inspector.get_targets_by_type(["a"])
        assert len(type_a) == 2  # T1 and T3

    def test_get_targets_by_multiple_types(self, orchestrator):
        """Test filtering targets by multiple types."""
        types_ab = orchestrator.inspector.get_targets_by_type(["a", "b"])
        assert len(types_ab) == 3  # T1, T2, T3


class TestTrajectory:
    """Tests for the Trajectory module."""

    def test_calculate_trajectory_distance_straight_line(self, orchestrator):
        """Test distance calculation for straight line trajectory."""
        trajectory = [[0, 0], [100, 0]]
        distance = orchestrator.trajectory.calculate_trajectory_distance(trajectory)
        assert distance == pytest.approx(100.0, rel=0.01)

    def test_calculate_trajectory_distance_diagonal(self, orchestrator):
        """Test distance calculation for diagonal trajectory."""
        trajectory = [[0, 0], [100, 100]]
        expected = 141.42  # sqrt(100^2 + 100^2)
        distance = orchestrator.trajectory.calculate_trajectory_distance(trajectory)
        assert distance == pytest.approx(expected, rel=0.01)

    def test_calculate_trajectory_distance_multi_segment(self, orchestrator):
        """Test distance calculation for multi-segment trajectory."""
        trajectory = [[0, 0], [50, 0], [50, 50], [0, 50], [0, 0]]
        expected = 200.0  # 50 * 4 sides of square
        distance = orchestrator.trajectory.calculate_trajectory_distance(trajectory)
        assert distance == pytest.approx(expected, rel=0.01)

    def test_split_trajectory_at_distance(self, orchestrator):
        """Test splitting trajectory at specific distance."""
        trajectory = [[0, 0], [25, 25], [50, 50], [75, 75], [100, 100]]
        split = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 70)

        assert split["split_point"] is not None
        assert len(split["prefix"]) > 0
        assert len(split["suffix"]) > 0

    def test_split_trajectory_at_zero(self, orchestrator):
        """Test splitting at distance 0 returns full trajectory as suffix."""
        trajectory = [[0, 0], [50, 50], [100, 100]]
        split = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 0)

        # At distance 0, prefix should be just the start point
        assert len(split["prefix"]) == 1
        assert split["prefix"][0] == [0, 0]


class TestConstraints:
    """Tests for the Constraints module."""

    def test_calculate_remaining_fuel(self, orchestrator):
        """Test remaining fuel calculation."""
        remaining = orchestrator.constraints.calculate_remaining_fuel(150, 80)
        assert remaining == 70

    def test_calculate_remaining_fuel_zero_used(self, orchestrator):
        """Test remaining fuel with zero used."""
        remaining = orchestrator.constraints.calculate_remaining_fuel(200, 0)
        assert remaining == 200

    def test_get_loiter_costs_for_target_types(self, orchestrator):
        """Test loiter cost mapping."""
        targets = [
            {"id": "T1", "type": "a"},
            {"id": "T2", "type": "c"},
            {"id": "T3", "type": "c"},
            {"id": "T4", "type": "b"}
        ]

        loiter_costs = orchestrator.constraints.get_loiter_costs_for_target_types(
            targets,
            {"c": 20}
        )

        assert len(loiter_costs) == 2
        assert loiter_costs.get("T2") == 20
        assert loiter_costs.get("T3") == 20

    def test_get_loiter_costs_empty_config(self, orchestrator):
        """Test loiter costs with empty configuration."""
        targets = [{"id": "T1", "type": "a"}]
        loiter_costs = orchestrator.constraints.get_loiter_costs_for_target_types(targets, {})
        assert len(loiter_costs) == 0


class TestSegments:
    """Tests for the Segments module."""

    def test_create_segment(self, orchestrator):
        """Test creating a segment from solution data."""
        env = {
            "airports": [{"id": "A1", "x": 0, "y": 0}],
            "targets": [{"id": "T1", "x": 50, "y": 50, "priority": 10, "type": "a"}],
            "sams": []
        }

        solution = {
            "routes": {
                "1": {
                    "route": ["A1", "T1", "A1"],
                    "distance": 141.42,
                    "trajectory": [[0, 0], [50, 50], [0, 0]]
                }
            },
            "sequences": {"1": "A1,T1,A1"}
        }

        drone_configs = {
            "1": {"enabled": True, "fuel_budget": 150, "start_airport": "A1", "end_airport": "A1"}
        }

        segment = orchestrator.segments.create_segment(
            index=0,
            solution=solution,
            env=env,
            drone_configs=drone_configs,
            cut_distance=70,
            cut_positions={"1": [50, 50]},
            frozen_targets=["T1"],
            active_targets=["T1"]
        )

        assert segment["index"] == 0
        assert segment["cutDistance"] == 70
        assert "solution" in segment

    def test_create_segmented_mission(self, orchestrator):
        """Test creating a segmented mission from segments."""
        segment = {
            "index": 0,
            "cutDistance": 50,
            "solution": {"routes": {}},
            "frozenTargets": ["T1"]
        }

        env = {
            "airports": [{"id": "A1", "x": 0, "y": 0}],
            "targets": [{"id": "T1", "x": 50, "y": 50}],
            "sams": []
        }

        mission = orchestrator.segments.create_segmented_mission(
            segments=[segment],
            base_env=env
        )

        assert mission["segment_count"] == 1
        assert mission["is_segmented"] == True
        assert "segments" in mission
