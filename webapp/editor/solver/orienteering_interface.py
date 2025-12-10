"""
Orienteering Solver Interface for ISR Editor
Provides interface to the external orienteering solver (Held-Karp DP algorithm)
"""
import sys
from pathlib import Path

# Add paths for importing the REAL orienteering solver from project root
_project_root = Path(__file__).parent.parent.parent.parent  # isr_web/
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Add /app for Docker deployment
if "/app" not in sys.path:
    sys.path.insert(0, "/app")


class OrienteeringSolverInterface:
    """Interface to the Held-Karp orienteering solver"""

    def __init__(self):
        self.solver = None
        self._import_solver()

    def _import_solver(self):
        """Import the REAL orienteering solver (Held-Karp DP algorithm)"""
        # The REAL solver is at project root: orienteering_with_matrix.py
        # It has signature: solve_orienteering_with_matrix(env, start_id=None, mode=None, fuel_cap=None, end_id=None)

        try:
            # Docker path: /app/orienteering_with_matrix.py
            from orienteering_with_matrix import solve_orienteering_with_matrix
            self.solver = solve_orienteering_with_matrix
            print("✅ REAL Orienteering solver loaded (Held-Karp DP)")
        except ImportError as e1:
            print(f"⚠️ Direct import failed: {e1}")
            try:
                # Try isr_web prefix for local dev
                from isr_web.orienteering_with_matrix import solve_orienteering_with_matrix
                self.solver = solve_orienteering_with_matrix
                print("✅ REAL Orienteering solver loaded (local path)")
            except ImportError as e2:
                print(f"⚠️ Local import failed: {e2}")
                print("❌ Error: Could not import REAL orienteering solver")
                print("   The solver should be at: orienteering_with_matrix.py (project root)")
                self.solver = None

    def solve(self, env_data, fuel_budget):
        """
        Solve the orienteering problem using Held-Karp DP algorithm.

        Args:
            env_data: Dictionary containing environment data
                - airports: List of airport dictionaries with id, x, y
                - targets: List of target dictionaries with id, x, y, priority
                - sams: List of SAM dictionaries (optional)
                - distance_matrix: 2D distance matrix (list of lists)
                - matrix_labels: Labels for matrix rows/columns
                - start_airport: Starting airport ID
                - end_airport: Ending airport ID
            fuel_budget: Maximum fuel/distance allowed

        Returns:
            Solution dictionary with:
                - route: List of waypoint IDs in order
                - distance: Total distance of route
                - total_points: Total priority points collected
                - visited_targets: List of visited target IDs
        """
        if self.solver is None:
            raise RuntimeError("Orienteering solver not available - check imports")

        # Build environment dict in the format the REAL solver expects
        airports = env_data.get("airports", [])
        targets = env_data.get("targets", [])
        start_airport_id = env_data.get("start_airport", "A1")
        end_airport_id = env_data.get("end_airport", start_airport_id)

        # The REAL solver expects 'matrix' key, not 'distance_matrix'
        matrix = env_data.get("distance_matrix", env_data.get("matrix", []))
        labels = env_data.get("matrix_labels", env_data.get("labels", []))

        # Build solver environment
        solver_env = {
            "airports": airports,
            "targets": targets,
            "matrix": matrix,
            "matrix_labels": labels,
            "start_airport": start_airport_id,
            "end_airport": end_airport_id,
            "fuel_budget": fuel_budget,
        }

        # Determine mode based on start/end airports
        if end_airport_id == start_airport_id or end_airport_id == "-":
            mode = "return"  # Round trip
            end_id = None
        else:
            mode = "end"  # Fixed endpoint
            end_id = end_airport_id

        print(f"🧠 Calling REAL solver with mode={mode}, fuel_cap={fuel_budget}")
        print(f"   Start: {start_airport_id}, End: {end_airport_id or start_airport_id}")
        print(f"   Targets: {len(targets)}, Matrix size: {len(matrix)}x{len(matrix[0]) if matrix else 0}")

        # Call the REAL Held-Karp solver
        solution = self.solver(
            env=solver_env,
            start_id=start_airport_id,
            mode=mode,
            fuel_cap=fuel_budget,
            end_id=end_id
        )

        # The REAL solver returns:
        # {
        #     "sequence": "A1 T2 T5 A1",
        #     "route": ["A1", "T2", "T5", "A1"],
        #     "distance": 123.4,
        #     "total_points": 15,
        #     "visited_targets": ["T2", "T5"],
        #     "start_airport": "A1",
        #     "end_airport": "A1"
        # }

        result = {
            "route": solution.get("route", [start_airport_id]),
            "distance": solution.get("distance", 0.0),
            "total_points": solution.get("total_points", 0),
            "visited_targets": solution.get("visited_targets", [])
        }

        print(f"✅ Solver result: {len(result['visited_targets'])} targets, "
              f"{result['total_points']} points, {result['distance']:.1f} distance")

        # Validate solution doesn't exceed fuel budget
        if result['distance'] > fuel_budget:
            print(f"⚠️ WARNING: Solution exceeds fuel budget: {result['distance']:.1f} > {fuel_budget}")

        return result

    def find_valid_fuel_solution(self, env_data, fuel_budget, max_iterations=10):
        """
        Find a valid solution within fuel budget by iteratively reducing targets

        Args:
            env_data: Environment data dictionary
            fuel_budget: Maximum fuel allowed
            max_iterations: Maximum attempts to find valid solution

        Returns:
            Valid solution dictionary or None if no solution found
        """
        if self.solver is None:
            return None

        # The REAL solver already handles fuel constraints properly
        # via its enumeration of all target subsets
        return self.solve(env_data, fuel_budget)

    def build_environment_for_solver(self, airports, targets, sams, distance_matrix_data):
        """
        Build environment data structure for orienteering solver

        Args:
            airports: List of airport objects
            targets: List of target objects
            sams: List of SAM objects
            distance_matrix_data: Precomputed distance matrix data with 'matrix' and 'labels'

        Returns:
            Environment data dictionary formatted for solver
        """
        # Build airports list with proper IDs
        solver_airports = []
        for airport in airports:
            solver_airports.append({
                "id": airport.get("id", f"A{len(solver_airports)+1}"),
                "x": airport["x"],
                "y": airport["y"]
            })

        # Build targets list with priorities
        solver_targets = []
        excluded = distance_matrix_data.get('excluded_targets', [])
        for i, target in enumerate(targets):
            target_id = target.get("id", f"T{i+1}")
            if target_id not in excluded:
                solver_targets.append({
                    "id": target_id,
                    "x": target["x"],
                    "y": target["y"],
                    "priority": target.get("priority", 1)
                })

        # Build SAMs list (for reference, but solver uses precomputed matrix)
        solver_sams = []
        for sam in sams:
            solver_sams.append({
                "pos": list(sam["pos"]),
                "range": sam["range"]
            })

        env_data = {
            "airports": solver_airports,
            "targets": solver_targets,
            "sams": solver_sams,
            "matrix": distance_matrix_data.get('matrix', []),
            "matrix_labels": distance_matrix_data.get('labels', []),
            "excluded_targets": excluded
        }

        return env_data
