"""
Unit tests for Solver Components.

Tests the core solver modules:
- SAM Distance Matrix Calculator
- Target Allocator (5 strategies)
- Post Optimizer (Insert, Swap, No-Cross)
"""

import pytest
import math


class TestSAMDistanceMatrix:
    """Tests for SAM Distance Matrix Calculator."""

    @pytest.fixture
    def matrix_calculator(self):
        """Create a SAMDistanceMatrixCalculator instance."""
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator
        return SAMDistanceMatrixCalculator()

    def test_calculate_matrix_no_sams(self, matrix_calculator, simple_env):
        """Test distance matrix calculation without SAMs."""
        result = matrix_calculator.calculate_matrix(
            airports=simple_env["airports"],
            targets=simple_env["targets"],
            sams=[]
        )

        assert "distances" in result
        assert "waypoints" in result
        assert "paths" in result

        # Check waypoints include all airports and targets
        waypoint_ids = [w["id"] for w in result["waypoints"]]
        assert "A1" in waypoint_ids
        assert "A2" in waypoint_ids
        assert "T1" in waypoint_ids

    def test_calculate_matrix_with_sams(self, matrix_calculator, env_with_sams):
        """Test distance matrix calculation with SAMs."""
        result = matrix_calculator.calculate_matrix(
            airports=env_with_sams["airports"],
            targets=env_with_sams["targets"],
            sams=env_with_sams["sams"]
        )

        assert "distances" in result
        # With SAMs, some paths should be longer than direct distance
        # (they need to go around SAMs)

    def test_get_distance_from_matrix(self, matrix_calculator, simple_env):
        """Test retrieving distance between two points."""
        result = matrix_calculator.calculate_matrix(
            airports=simple_env["airports"],
            targets=simple_env["targets"],
            sams=[]
        )

        # Distance A1 to A2 should be 100 (horizontal line)
        if "A1" in result["distances"] and "A2" in result["distances"]["A1"]:
            dist = result["distances"]["A1"]["A2"]
            assert dist == pytest.approx(100.0, rel=0.01)

    def test_matrix_is_symmetric(self, matrix_calculator, simple_env):
        """Test that distance matrix is symmetric."""
        result = matrix_calculator.calculate_matrix(
            airports=simple_env["airports"],
            targets=simple_env["targets"],
            sams=[]
        )

        distances = result["distances"]
        for from_id, to_dict in distances.items():
            for to_id, dist in to_dict.items():
                if from_id != to_id:
                    reverse_dist = distances.get(to_id, {}).get(from_id, None)
                    if reverse_dist is not None:
                        assert dist == pytest.approx(reverse_dist, rel=0.01), \
                            f"Matrix not symmetric: {from_id}->{to_id}={dist}, {to_id}->{from_id}={reverse_dist}"


class TestTargetAllocator:
    """Tests for Target Allocator."""

    @pytest.fixture
    def allocator(self):
        """Create a TargetAllocator instance."""
        from server.solver.target_allocator import TargetAllocator
        return TargetAllocator()

    @pytest.fixture
    def targets(self):
        """Sample targets for allocation tests."""
        return [
            {"id": "T1", "x": 30, "y": 40, "priority": 8, "type": "a"},
            {"id": "T2", "x": 60, "y": 20, "priority": 5, "type": "b"},
            {"id": "T3", "x": 80, "y": 60, "priority": 10, "type": "a"},
            {"id": "T4", "x": 20, "y": 70, "priority": 3, "type": "c"},
        ]

    @pytest.fixture
    def two_drones(self):
        """Two-drone configuration."""
        return {
            "1": {
                "enabled": True,
                "fuel_budget": 300,
                "start_airport": "A1",
                "end_airport": "A1",
                "target_access": {}  # All types
            },
            "2": {
                "enabled": True,
                "fuel_budget": 300,
                "start_airport": "A2",
                "end_airport": "A2",
                "target_access": {}  # All types
            }
        }

    @pytest.fixture
    def airports(self):
        """Sample airports."""
        return [
            {"id": "A1", "x": 0, "y": 0},
            {"id": "A2", "x": 100, "y": 0},
        ]

    def test_allocate_greedy(self, allocator, targets, two_drones, airports):
        """Test greedy allocation strategy."""
        from server.solver.target_allocator import AllocationStrategy

        result = allocator.allocate(
            targets=targets,
            drone_configs=two_drones,
            airports=airports,
            strategy=AllocationStrategy.GREEDY
        )

        # Should return allocations for both drones
        assert "1" in result
        assert "2" in result

        # All targets should be assigned
        all_assigned = set(result["1"]) | set(result["2"])
        all_target_ids = {t["id"] for t in targets}
        assert all_assigned == all_target_ids

    def test_allocate_balanced(self, allocator, targets, two_drones, airports):
        """Test balanced allocation strategy."""
        from server.solver.target_allocator import AllocationStrategy

        result = allocator.allocate(
            targets=targets,
            drone_configs=two_drones,
            airports=airports,
            strategy=AllocationStrategy.BALANCED
        )

        # Balanced should distribute evenly: 2 each for 4 targets
        count_diff = abs(len(result["1"]) - len(result["2"]))
        assert count_diff <= 1  # Allow 1 target difference for odd counts

    def test_allocate_efficient(self, allocator, targets, two_drones, airports):
        """Test efficient allocation strategy."""
        from server.solver.target_allocator import AllocationStrategy

        result = allocator.allocate(
            targets=targets,
            drone_configs=two_drones,
            airports=airports,
            strategy=AllocationStrategy.EFFICIENT
        )

        # All targets should be assigned
        all_assigned = set(result["1"]) | set(result["2"])
        all_target_ids = {t["id"] for t in targets}
        assert all_assigned == all_target_ids

    def test_allocate_geographic(self, allocator, targets, two_drones, airports):
        """Test geographic allocation strategy."""
        from server.solver.target_allocator import AllocationStrategy

        result = allocator.allocate(
            targets=targets,
            drone_configs=two_drones,
            airports=airports,
            strategy=AllocationStrategy.GEOGRAPHIC
        )

        # Geographic should assign targets near each drone's airport
        # D1 from A1 (0,0) should get targets closer to left side
        # D2 from A2 (100,0) should get targets closer to right side

        # All targets should be assigned
        all_assigned = set(result["1"]) | set(result["2"])
        all_target_ids = {t["id"] for t in targets}
        assert all_assigned == all_target_ids

    def test_allocate_exclusive(self, allocator, airports):
        """Test exclusive allocation strategy with restricted access."""
        from server.solver.target_allocator import AllocationStrategy

        # Set up targets where some are only accessible by one drone
        targets = [
            {"id": "T1", "x": 30, "y": 40, "priority": 10, "type": "a"},
            {"id": "T2", "x": 60, "y": 20, "priority": 5, "type": "b"},
        ]

        # D1 can only access type "a", D2 can access all
        drones = {
            "1": {
                "enabled": True,
                "fuel_budget": 300,
                "start_airport": "A1",
                "end_airport": "A1",
                "target_access": {"a": True}  # Only type a
            },
            "2": {
                "enabled": True,
                "fuel_budget": 300,
                "start_airport": "A2",
                "end_airport": "A2",
                "target_access": {}  # All types
            }
        }

        result = allocator.allocate(
            targets=targets,
            drone_configs=drones,
            airports=airports,
            strategy=AllocationStrategy.EXCLUSIVE
        )

        # T1 (type a) could go to either, T2 (type b) must go to D2
        assert "T2" in result["2"]

    def test_allocate_disabled_drone(self, allocator, targets, airports):
        """Test that disabled drones don't receive allocations."""
        from server.solver.target_allocator import AllocationStrategy

        drones = {
            "1": {
                "enabled": True,
                "fuel_budget": 300,
                "start_airport": "A1",
                "end_airport": "A1",
                "target_access": {}
            },
            "2": {
                "enabled": False,  # Disabled
                "fuel_budget": 300,
                "start_airport": "A2",
                "end_airport": "A2",
                "target_access": {}
            }
        }

        result = allocator.allocate(
            targets=targets,
            drone_configs=drones,
            airports=airports,
            strategy=AllocationStrategy.BALANCED
        )

        # D2 should have no targets (disabled)
        assert len(result.get("2", [])) == 0
        # D1 should get all targets
        assert len(result["1"]) == len(targets)


class TestPostOptimizer:
    """Tests for Post Optimizer."""

    @pytest.fixture
    def optimizer(self):
        """Create a PostOptimizer instance."""
        from server.solver.post_optimizer import PostOptimizer
        return PostOptimizer()

    @pytest.fixture
    def basic_solution(self):
        """Basic solution with one drone."""
        return {
            "routes": {
                "1": {
                    "route": ["A1", "T1", "T2", "A1"],
                    "distance": 150.0,
                    "trajectory": [[0, 0], [30, 40], [60, 20], [0, 0]],
                    "visited_targets": ["T1", "T2"]
                }
            },
            "sequences": {"1": "A1,T1,T2,A1"},
            "missed_targets": ["T3", "T4"],
        }

    @pytest.fixture
    def solution_env(self):
        """Environment for the basic solution."""
        return {
            "airports": [
                {"id": "A1", "x": 0, "y": 0},
            ],
            "targets": [
                {"id": "T1", "x": 30, "y": 40, "priority": 8},
                {"id": "T2", "x": 60, "y": 20, "priority": 5},
                {"id": "T3", "x": 20, "y": 10, "priority": 10},  # Missed, near route
                {"id": "T4", "x": 90, "y": 90, "priority": 3},   # Missed, far
            ],
            "sams": []
        }

    @pytest.fixture
    def drone_config(self):
        """Drone config with sufficient fuel."""
        return {
            "1": {
                "enabled": True,
                "fuel_budget": 500,  # Plenty of fuel
                "start_airport": "A1",
                "end_airport": "A1",
            }
        }

    @pytest.mark.unit
    def test_insert_missed_finds_candidates(self, optimizer, basic_solution, solution_env, drone_config):
        """Test that insert_missed identifies insertion candidates."""
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator

        # Pre-calculate distance matrix
        calc = SAMDistanceMatrixCalculator()
        matrix = calc.calculate_matrix(
            airports=solution_env["airports"],
            targets=solution_env["targets"],
            sams=[]
        )
        optimizer.set_distance_matrix(matrix)

        result = optimizer.insert_missed(
            solution=basic_solution,
            env=solution_env,
            drone_configs=drone_config,
            dry_run=True  # Just identify candidates
        )

        # Should identify some candidates
        assert "candidates" in result or "insertions" in result


class TestPointInPolygon:
    """Tests for geometry utility functions."""

    def test_point_inside_square(self):
        """Test point inside a square polygon."""
        import numpy as np
        from server.solver.sam_distance_matrix import point_in_polygon

        square = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
        assert point_in_polygon((5, 5), square) == True

    def test_point_outside_square(self):
        """Test point outside a square polygon."""
        import numpy as np
        from server.solver.sam_distance_matrix import point_in_polygon

        square = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
        assert point_in_polygon((15, 5), square) == False

    def test_point_on_edge(self):
        """Test point on polygon edge."""
        import numpy as np
        from server.solver.sam_distance_matrix import point_in_polygon

        square = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
        # Point on edge behavior depends on implementation
        # Just verify it doesn't crash
        result = point_in_polygon((5, 0), square)
        assert isinstance(result, bool)
