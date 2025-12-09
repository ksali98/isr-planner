"""
Orienteering Solver Interface for ISR Editor
Provides interface to the external orienteering solver
"""
import sys
from pathlib import Path
import numpy as np

# Add the solver directory to Python path for local imports
_solver_dir = Path(__file__).parent
if str(_solver_dir) not in sys.path:
    sys.path.insert(0, str(_solver_dir))


class OrienteeringSolverInterface:
    """Interface to the orienteering solver"""

    def __init__(self):
        self.solver = None
        self._import_solver()

    def _import_solver(self):
        """Import the orienteering solver module"""
        # Try multiple import paths for Docker vs local development
        try:
            # Direct import (works when solver dir is in path)
            from orienteering_with_matrix import solve_orienteering_with_matrix
            self.solver = solve_orienteering_with_matrix
            print("✅ Orienteering solver loaded successfully")
        except ImportError:
            try:
                # Docker path: webapp.editor.solver.orienteering_with_matrix
                from webapp.editor.solver.orienteering_with_matrix import solve_orienteering_with_matrix
                self.solver = solve_orienteering_with_matrix
                print("✅ Orienteering solver loaded (Docker path)")
            except ImportError:
                try:
                    # Local dev path: isr_web.webapp.editor.solver.orienteering_with_matrix
                    from isr_web.webapp.editor.solver.orienteering_with_matrix import solve_orienteering_with_matrix
                    self.solver = solve_orienteering_with_matrix
                    print("✅ Orienteering solver loaded (local path)")
                except ImportError:
                    print("❌ Error: Could not import orienteering solver")
                    print("Please ensure orienteering_with_matrix.py exists")
                    self.solver = None
    
    def solve(self, env_data, fuel_budget):
        """
        Solve the orienteering problem for the given environment
        
        Args:
            env_data: Dictionary containing environment data
                - airports: List of airport dictionaries
                - targets: List of target dictionaries  
                - sams: List of SAM dictionaries
                - distance_matrix: Precomputed distance matrix
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
            raise RuntimeError("Orienteering solver not available")
        
        # Ensure required parameters are in env_data
        env_data["fuel_budget"] = fuel_budget
        
        # Call the external solver
        solution = self.solver(env_data, fuel_cap=fuel_budget)
        
        # Validate solution doesn't exceed fuel budget
        if solution['distance'] > fuel_budget:
            print(f"⚠️ Solution exceeds fuel budget: {solution['distance']:.1f} > {fuel_budget}")
        
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
        
        # Start with all targets
        current_env = env_data.copy()
        
        for iteration in range(max_iterations):
            solution = self.solve(current_env, fuel_budget)
            
            # Check if solution is within budget
            if solution['distance'] <= fuel_budget:
                return solution
            
            # Remove lowest priority target and try again
            if len(current_env['targets']) <= 1:
                break
                
            # Sort targets by priority and remove lowest
            sorted_targets = sorted(current_env['targets'], 
                                  key=lambda t: t.get('priority', 1), 
                                  reverse=True)
            current_env['targets'] = sorted_targets[:-1]
            
            print(f"🔄 Iteration {iteration + 1}: Trying with {len(current_env['targets'])} targets")
        
        return None
    
    def build_environment_for_solver(self, airports, targets, sams, distance_matrix_data):
        """
        Build environment data structure for orienteering solver
        
        Args:
            airports: List of airport objects
            targets: List of target objects
            sams: List of SAM objects
            distance_matrix_data: Precomputed distance matrix data
            
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
        
        # Build targets list (only valid targets)
        solver_targets = []
        for i, target in enumerate(targets):
            target_id = target.get("id", f"T{i+1}")
            if target_id not in distance_matrix_data.get('excluded_targets', []):
                solver_targets.append({
                    "id": target_id,
                    "x": target["x"],
                    "y": target["y"],
                    "priority": target.get("priority", 1)
                })
        
        # Build SAMs list
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
            "distance_matrix": distance_matrix_data['matrix'],
            "matrix_labels": distance_matrix_data['labels'],
            "excluded_targets": distance_matrix_data.get('excluded_targets', [])
        }
        
        return env_data