"""
Performance benchmarks for the ISR Planner solver components.

These tests measure execution time for key operations:
- Distance matrix calculation
- Target allocation
- Route optimization (Held-Karp)
- Post-optimization (Insert, Swap, No-Cross)

Run with pytest:
    pytest tests/benchmarks/ -v --benchmark-enable

Or manually time:
    pytest tests/benchmarks/test_solver_performance.py -v -s
"""

import pytest
import time
import random
import math


def generate_random_env(num_targets: int, num_sams: int = 0) -> dict:
    """Generate a random environment for testing."""
    random.seed(42)  # Reproducible results

    targets = [
        {
            "id": f"T{i}",
            "x": random.uniform(10, 190),
            "y": random.uniform(10, 190),
            "priority": random.randint(1, 10),
            "type": random.choice(["a", "b", "c"])
        }
        for i in range(1, num_targets + 1)
    ]

    sams = [
        {
            "pos": [random.uniform(30, 170), random.uniform(30, 170)],
            "range": random.uniform(10, 20)
        }
        for _ in range(num_sams)
    ]

    return {
        "airports": [
            {"id": "A1", "x": 0, "y": 0},
            {"id": "A2", "x": 200, "y": 0},
        ],
        "targets": targets,
        "sams": sams
    }


def generate_drone_configs(num_drones: int) -> dict:
    """Generate drone configurations."""
    return {
        str(i): {
            "enabled": True,
            "fuel_budget": 500,
            "start_airport": "A1" if i <= num_drones // 2 else "A2",
            "end_airport": "A1" if i <= num_drones // 2 else "A2",
            "target_access": {}
        }
        for i in range(1, num_drones + 1)
    }


class BenchmarkTimer:
    """Simple context manager for timing operations."""

    def __init__(self, name: str):
        self.name = name
        self.elapsed = 0

    def __enter__(self):
        self.start = time.perf_counter()
        return self

    def __exit__(self, *args):
        self.elapsed = time.perf_counter() - self.start
        print(f"  {self.name}: {self.elapsed*1000:.2f}ms")


@pytest.mark.slow
class TestDistanceMatrixPerformance:
    """Benchmarks for distance matrix calculation."""

    def test_matrix_small_no_sams(self):
        """Benchmark distance matrix: 10 targets, no SAMs."""
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator

        env = generate_random_env(num_targets=10, num_sams=0)
        calc = SAMDistanceMatrixCalculator()

        with BenchmarkTimer("10 targets, 0 SAMs") as timer:
            result = calc.calculate_matrix(
                airports=env["airports"],
                targets=env["targets"],
                sams=[]
            )

        assert "distances" in result
        # Should be fast without SAMs
        assert timer.elapsed < 1.0, "Matrix calculation too slow"

    def test_matrix_medium_no_sams(self):
        """Benchmark distance matrix: 25 targets, no SAMs."""
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator

        env = generate_random_env(num_targets=25, num_sams=0)
        calc = SAMDistanceMatrixCalculator()

        with BenchmarkTimer("25 targets, 0 SAMs") as timer:
            result = calc.calculate_matrix(
                airports=env["airports"],
                targets=env["targets"],
                sams=[]
            )

        assert "distances" in result

    def test_matrix_small_with_sams(self):
        """Benchmark distance matrix: 10 targets, 3 SAMs."""
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator

        env = generate_random_env(num_targets=10, num_sams=3)
        calc = SAMDistanceMatrixCalculator()

        with BenchmarkTimer("10 targets, 3 SAMs") as timer:
            result = calc.calculate_matrix(
                airports=env["airports"],
                targets=env["targets"],
                sams=env["sams"]
            )

        assert "distances" in result

    def test_matrix_medium_with_sams(self):
        """Benchmark distance matrix: 20 targets, 5 SAMs."""
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator

        env = generate_random_env(num_targets=20, num_sams=5)
        calc = SAMDistanceMatrixCalculator()

        with BenchmarkTimer("20 targets, 5 SAMs") as timer:
            result = calc.calculate_matrix(
                airports=env["airports"],
                targets=env["targets"],
                sams=env["sams"]
            )

        assert "distances" in result


@pytest.mark.slow
class TestTargetAllocationPerformance:
    """Benchmarks for target allocation strategies."""

    def test_allocation_10_targets_2_drones(self):
        """Benchmark allocation: 10 targets, 2 drones."""
        from server.solver.target_allocator import TargetAllocator, AllocationStrategy

        env = generate_random_env(num_targets=10, num_sams=0)
        configs = generate_drone_configs(num_drones=2)
        allocator = TargetAllocator()

        strategies = [
            AllocationStrategy.GREEDY,
            AllocationStrategy.BALANCED,
            AllocationStrategy.EFFICIENT,
            AllocationStrategy.GEOGRAPHIC,
            AllocationStrategy.EXCLUSIVE
        ]

        print("\n  10 targets, 2 drones:")
        for strategy in strategies:
            with BenchmarkTimer(f"    {strategy.value}") as timer:
                result = allocator.allocate(
                    targets=env["targets"],
                    drone_configs=configs,
                    airports=env["airports"],
                    strategy=strategy
                )

            assert "1" in result

    def test_allocation_25_targets_5_drones(self):
        """Benchmark allocation: 25 targets, 5 drones."""
        from server.solver.target_allocator import TargetAllocator, AllocationStrategy

        env = generate_random_env(num_targets=25, num_sams=0)
        configs = generate_drone_configs(num_drones=5)
        allocator = TargetAllocator()

        print("\n  25 targets, 5 drones:")
        for strategy in [AllocationStrategy.EFFICIENT, AllocationStrategy.GEOGRAPHIC]:
            with BenchmarkTimer(f"    {strategy.value}") as timer:
                result = allocator.allocate(
                    targets=env["targets"],
                    drone_configs=configs,
                    airports=env["airports"],
                    strategy=strategy
                )

            # All drones should have allocations
            assert all(str(i) in result for i in range(1, 6))


@pytest.mark.slow
class TestSolverBridgePerformance:
    """Benchmarks for the full solver bridge."""

    def test_solve_8_targets_1_drone(self):
        """Benchmark solve: 8 targets, 1 drone (Held-Karp optimal range)."""
        from server.solver.solver_bridge import SolverBridge

        env = generate_random_env(num_targets=8, num_sams=0)
        configs = generate_drone_configs(num_drones=1)
        bridge = SolverBridge()

        with BenchmarkTimer("8 targets, 1 drone") as timer:
            result = bridge.solve(
                environment=env,
                drone_configs=configs,
                settings={"allocation_strategy": "efficient"}
            )

        if "error" not in result:
            assert "routes" in result or "solution" in result
        # Held-Karp is O(n^2 * 2^n), 8 targets should be fast
        assert timer.elapsed < 5.0, "Solve too slow for 8 targets"

    def test_solve_12_targets_2_drones(self):
        """Benchmark solve: 12 targets, 2 drones."""
        from server.solver.solver_bridge import SolverBridge

        env = generate_random_env(num_targets=12, num_sams=2)
        configs = generate_drone_configs(num_drones=2)
        bridge = SolverBridge()

        with BenchmarkTimer("12 targets, 2 drones, 2 SAMs") as timer:
            result = bridge.solve(
                environment=env,
                drone_configs=configs,
                settings={"allocation_strategy": "efficient"}
            )

        if "error" not in result:
            assert "routes" in result or "solution" in result

    def test_solve_20_targets_5_drones(self):
        """Benchmark solve: 20 targets, 5 drones (larger scale)."""
        from server.solver.solver_bridge import SolverBridge

        env = generate_random_env(num_targets=20, num_sams=3)
        configs = generate_drone_configs(num_drones=5)
        bridge = SolverBridge()

        with BenchmarkTimer("20 targets, 5 drones, 3 SAMs") as timer:
            result = bridge.solve(
                environment=env,
                drone_configs=configs,
                settings={"allocation_strategy": "efficient"}
            )

        if "error" not in result:
            assert "routes" in result or "solution" in result


@pytest.mark.slow
class TestPostOptimizerPerformance:
    """Benchmarks for post-optimization."""

    @pytest.fixture
    def solution_with_missed(self):
        """Generate a solution with missed targets for optimization."""
        return {
            "routes": {
                "1": {
                    "route": ["A1", "T1", "T3", "T5", "A1"],
                    "distance": 200.0,
                    "trajectory": [[0, 0], [30, 30], [70, 70], [110, 30], [0, 0]],
                    "visited_targets": ["T1", "T3", "T5"]
                },
                "2": {
                    "route": ["A2", "T2", "T4", "A2"],
                    "distance": 150.0,
                    "trajectory": [[200, 0], [150, 40], [170, 80], [200, 0]],
                    "visited_targets": ["T2", "T4"]
                }
            },
            "sequences": {"1": "A1,T1,T3,T5,A1", "2": "A2,T2,T4,A2"},
            "missed_targets": ["T6", "T7", "T8"]
        }

    def test_insert_missed_performance(self, solution_with_missed):
        """Benchmark insert missed optimization."""
        from server.solver.post_optimizer import PostOptimizer
        from server.solver.sam_distance_matrix import SAMDistanceMatrixCalculator

        env = generate_random_env(num_targets=8, num_sams=0)
        configs = generate_drone_configs(num_drones=2)

        # Calculate matrix
        calc = SAMDistanceMatrixCalculator()
        matrix = calc.calculate_matrix(
            airports=env["airports"],
            targets=env["targets"],
            sams=[]
        )

        optimizer = PostOptimizer()
        optimizer.set_distance_matrix(matrix)

        with BenchmarkTimer("Insert missed (8 targets, 3 missed)") as timer:
            result = optimizer.insert_missed(
                solution=solution_with_missed,
                env=env,
                drone_configs=configs
            )

        # Should complete in reasonable time
        assert timer.elapsed < 2.0


@pytest.mark.slow
class TestScalabilityBenchmarks:
    """Benchmarks for testing scalability limits."""

    @pytest.mark.parametrize("num_targets", [5, 10, 12, 15])
    def test_held_karp_scaling(self, num_targets):
        """Test Held-Karp solver scaling with target count."""
        from server.solver.solver_bridge import SolverBridge

        env = generate_random_env(num_targets=num_targets, num_sams=0)
        configs = generate_drone_configs(num_drones=1)
        bridge = SolverBridge()

        with BenchmarkTimer(f"Held-Karp {num_targets} targets") as timer:
            result = bridge.solve(
                environment=env,
                drone_configs=configs,
                settings={"allocation_strategy": "efficient"}
            )

        # Print scaling info
        print(f"    -> Completed in {timer.elapsed*1000:.2f}ms")

        # 15 targets is approaching practical limit for exact solver
        if num_targets <= 12:
            assert timer.elapsed < 10.0, f"Too slow for {num_targets} targets"
