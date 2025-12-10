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
                
class OrienteeringSolverInterface:
    def __init__(self):
        self.solver = None
        self._import_solver()

    def _import_solver(self):
        """Import the REAL orienteering solver (Held-Karp DP algorithm)."""
        # The REAL solver is at project root: orienteering_with_matrix.py
        try:
            # Docker path: /app/orienteering_with_matrix.py
            from orienteering_with_matrix import solve_orienteering_with_matrix
            self.solver = solve_orienteering_with_matrix
            print("✅ REAL Orienteering solver loaded (Held-Karp DP)")
        except ImportError as e1:
            print(f"⚠️ Direct import failed: {e1}")
            try:
                from isr_web.orienteering_with_matrix import solve_orienteering_with_matrix
                self.solver = solve_orienteering_with_matrix
                print("✅ REAL Orienteering solver loaded (from isr_web.orienteering_with_matrix)")
            except ImportError as e2:
                print(f"⚠️ Local import failed: {e2}")
                print("❌ Error: Could not import REAL orienteering solver")
                print("   The solver should be at: orienteering_with_matrix.py (project root)")
                self.solver = None

    def solve(self, env_data, fuel_budget):
        """
        Call the REAL Held–Karp orienteering solver.

        env_data is expected to contain:
          - airports: list of airport dicts
          - targets: list of target dicts
          - distance_matrix: full matrix (airports + targets), or 'matrix'
          - matrix_labels: labels for rows/cols of the matrix, or 'labels'
          - start_airport: ID of the starting airport (e.g., "A1")
          - end_airport: ID of the ending airport (default = start_airport)
          - mode: "return" or "open" (optional)
        """
        if self.solver is None:
            raise RuntimeError("Orienteering solver not available - check imports")

        airports = env_data.get("airports", [])
        targets = env_data.get("targets", [])

        start_airport_id = env_data.get("start_airport", "A1")
        end_airport_id = env_data.get("end_airport", start_airport_id)
        mode = env_data.get("mode", "return")

        # The REAL solver expects 'distance_matrix' and 'matrix_labels'
        matrix = env_data.get("distance_matrix") or env_data.get("matrix")
        labels = env_data.get("matrix_labels") or env_data.get("labels")

        if matrix is None or labels is None:
            raise ValueError(
                "OrienteeringInterface.solve: distance_matrix and matrix_labels "
                "must be provided in env_data."
            )

        # Build environment for the REAL solver
        solver_env = {
            "airports": airports,
            "targets": targets,
            "distance_matrix": matrix,   # REQUIRED by /app/orienteering_with_matrix.py
            "matrix_labels": labels,
            "start_airport": start_airport_id,
            "end_airport": end_airport_id,
        }

        # Call the real Held–Karp solver
        solution = self.solver(
            env=solver_env,
            start_id=start_airport_id,
            mode=mode,
            fuel_cap=fuel_budget,
            end_id=end_airport_id,
        )

        return solution

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
