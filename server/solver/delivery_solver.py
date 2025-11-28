"""
Main Delivery Solver
Integrates all solver components for complete delivery planning
"""
from typing import List, Dict, Tuple
from .nfz_distance_calculator import NFZDistanceCalculator
from .nfz_trajectory_planner import NFZTrajectoryPlanner
from .orienteering_solver import DeliveryOrienteeringSolver

class DeliverySolver:
    """
    Main solver for delivery planning
    Combines NFZ-aware distance calculation, orienteering solving, and trajectory planning
    """

    def __init__(self, no_fly_zones: List[Dict]):
        """
        Initialize delivery solver

        Args:
            no_fly_zones: List of {'x': float, 'y': float, 'radius': float}
        """
        print("🚀 Initializing Delivery Solver...")
        self.no_fly_zones = no_fly_zones

        # Initialize modular components
        self.distance_calc = NFZDistanceCalculator(no_fly_zones)
        self.trajectory_planner = NFZTrajectoryPlanner(no_fly_zones)
        self.orienteering_solver = DeliveryOrienteeringSolver()

        print("✅ Solver components initialized")

    def calculate_distance_matrix(self, warehouse: Dict, addresses: List[Dict]) -> Dict:
        """
        Calculate NFZ-aware distance matrix

        Args:
            warehouse: Warehouse dict with 'id', 'x', 'y'
            addresses: List of address dicts with 'id', 'x', 'y'

        Returns:
            Distance matrix dict
        """
        # Combine warehouse and addresses
        all_waypoints = [warehouse] + addresses

        return self.distance_calc.calculate_distance_matrix(all_waypoints)

    def find_optimal_route(self,
                          warehouse: Dict,
                          addresses: List[Dict],
                          fuel_budget: float) -> Dict:
        """
        Find optimal delivery route

        Args:
            warehouse: Warehouse location
            addresses: Delivery addresses
            fuel_budget: Maximum distance budget

        Returns:
            Dict with route, distance, and trajectory
        """
        print("\n" + "="*60)
        print("📍 DELIVERY ROUTE OPTIMIZATION")
        print("="*60)

        # Step 1: Calculate NFZ-aware distance matrix
        print("\n[1/3] Calculating NFZ-aware distances...")
        distance_matrix = self.calculate_distance_matrix(warehouse, addresses)

        # Step 2: Solve orienteering problem for optimal sequence
        print("\n[2/3] Finding optimal address sequence...")
        route_solution = self.orienteering_solver.solve_delivery_route(
            warehouse, addresses, distance_matrix, fuel_budget
        )

        # Step 3: Generate detailed trajectory
        print("\n[3/3] Generating NFZ-avoiding trajectory...")
        trajectory = self._generate_trajectory(route_solution['route'], warehouse, addresses)

        print("\n" + "="*60)
        print("✅ ROUTE OPTIMIZATION COMPLETE")
        print("="*60)
        print(f"📊 Addresses visited: {route_solution['addresses_visited']}/{len(addresses)}")
        print(f"📏 Total distance: {route_solution['distance']:.1f} units")
        print(f"⛽ Fuel budget: {fuel_budget:.1f} units")
        print(f"💰 Remaining: {fuel_budget - route_solution['distance']:.1f} units")
        print("="*60 + "\n")

        return {
            'route': route_solution['route'],
            'distance': route_solution['distance'],
            'addresses_visited': route_solution['addresses_visited'],
            'trajectory': trajectory,
            'distance_matrix': distance_matrix
        }

    def _generate_trajectory(self,
                            route: List[str],
                            warehouse: Dict,
                            addresses: List[Dict]) -> List[Tuple[float, float]]:
        """
        Generate detailed trajectory following the route

        Args:
            route: List of waypoint IDs
            warehouse: Warehouse dict
            addresses: List of address dicts

        Returns:
            List of (x, y) coordinates forming the path
        """
        # Create lookup for waypoints
        waypoint_lookup = {warehouse['id']: warehouse}
        for addr in addresses:
            waypoint_lookup[addr['id']] = addr

        # Build complete trajectory
        full_trajectory = []

        for i in range(len(route) - 1):
            start_id = route[i]
            end_id = route[i + 1]

            start_wp = waypoint_lookup[start_id]
            end_wp = waypoint_lookup[end_id]

            start_pos = (start_wp['x'], start_wp['y'])
            end_pos = (end_wp['x'], end_wp['y'])

            # Plan NFZ-avoiding path
            segment_path, _ = self.trajectory_planner.plan_path(start_pos, end_pos)

            # Add to full trajectory (avoid duplicates)
            if i == 0:
                full_trajectory.extend(segment_path)
            else:
                full_trajectory.extend(segment_path[1:])

        return full_trajectory
