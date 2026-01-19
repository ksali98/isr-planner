"""
Orienteering Solver Integration for Delivery Planning
Finds optimal address sequences using orienteering problem solver
"""
import sys
import os
from typing import List, Dict, Tuple

# Import orienteering solver from ISR editor
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from isr_editor.solver import OrienteeringSolverInterface


class DeliveryOrienteeringSolver:
    """Solve delivery routing using orienteering problem formulation"""

    def __init__(self):
        """Initialize orienteering solver"""
        self.solver = OrienteeringSolverInterface()

    def solve_delivery_route(self,
                             warehouse: Dict,
                             addresses: List[Dict],
                             distance_matrix: Dict,
                             fuel_budget: float) -> Dict:
        """
        Find optimal address visitation sequence

        Args:
            warehouse: {'x': float, 'y': float, 'id': str}
            addresses: List of address dicts with 'id', 'x', 'y'
            distance_matrix: NFZ-aware distance matrix
            fuel_budget: Maximum fuel/distance available

        Returns:
            Dict with:
                - route: List of address IDs in visit order
                - distance: Total route distance
                - addresses_visited: Number of addresses visited
        """

        # Build environment for solver
        env_data = self._build_solver_environment(
            warehouse, addresses, distance_matrix
        )

        # Solve with orienteering solver
        solution = self.solver.solve(env_data, fuel_budget)


        return {
            'route': solution['route'],
            'distance': solution['distance'],
            'addresses_visited': len([wp for wp in solution['route'] if wp.startswith('A') and wp != warehouse['id']])
        }

    def _build_solver_environment(self,
                                   warehouse: Dict,
                                   addresses: List[Dict],
                                   distance_matrix: Dict) -> Dict:
        """
        Build environment data structure for solver

        Args:
            warehouse: Warehouse dict
            addresses: List of address dicts
            distance_matrix: Distance matrix data

        Returns:
            Environment dict for solver
        """
        # Combine warehouse and addresses as waypoints
        all_waypoints = [warehouse] + addresses

        return {
            'start_airport': warehouse['id'],
            'end_airport': warehouse['id'],
            'distance_matrix': distance_matrix['matrix'],
            'matrix_labels': distance_matrix['labels'],
            'waypoints': all_waypoints,
            'airports': [warehouse],
            'targets': addresses
        }
