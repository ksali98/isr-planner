# Delivery Solver - Modular Architecture

## Overview
Modular solver system for package delivery routing with no-fly zone (NFZ) avoidance.

## Architecture

### Components

1. **NFZDistanceCalculator** (`nfz_distance_calculator.py`)
   - Calculates NFZ-aware distance matrices
   - Accounts for no-fly zones in distance calculations
   - Returns complete distance matrix for all waypoint pairs

2. **NFZTrajectoryPlanner** (`nfz_trajectory_planner.py`)
   - Generates flight paths avoiding no-fly zones
   - Uses tangent-arc-tangent method from ISR editor
   - Reuses proven SAMNavigator logic for NFZ avoidance

3. **DeliveryOrienteeringSolver** (`orienteering_solver.py`)
   - Finds optimal address visitation sequences
   - Integrates ISR editor's orienteering solver
   - Respects fuel/distance budgets

4. **DeliverySolver** (`delivery_solver.py`)
   - Main solver integrating all components
   - Orchestrates the complete solving workflow
   - Returns routes, distances, and trajectories

## Usage

```python
from delivery_solver import DeliverySolver

# Initialize solver with no-fly zones
no_fly_zones = [
    {'x': 50, 'y': 30, 'radius': 8},
    {'x': 65, 'y': 75, 'radius': 6}
]
solver = DeliverySolver(no_fly_zones)

# Define warehouse and addresses
warehouse = {'id': 'W', 'x': 50, 'y': 50}
addresses = [
    {'id': 'A1', 'x': 20, 'y': 70},
    {'id': 'A2', 'x': 80, 'y': 80},
    # ... more addresses
]

# Find optimal route
solution = solver.find_optimal_route(
    warehouse=warehouse,
    addresses=addresses,
    fuel_budget=200
)

# Access results
print(f"Route: {solution['route']}")
print(f"Distance: {solution['distance']}")
print(f"Addresses visited: {solution['addresses_visited']}")
print(f"Trajectory: {len(solution['trajectory'])} points")
```

## Workflow

1. **Distance Matrix Calculation**
   - For each pair of waypoints (warehouse + addresses)
   - Plan NFZ-avoiding path
   - Calculate actual path distance (not Euclidean)
   - Build NxN distance matrix

2. **Orienteering Optimization**
   - Use distance matrix as input
   - Find optimal sequence respecting fuel budget
   - Maximize addresses visited within constraints

3. **Trajectory Generation**
   - Follow optimal sequence
   - Generate detailed NFZ-avoiding paths
   - Return complete trajectory coordinates

## Integration with Flask Backend

Add to `delivery_planner_web.py`:

```python
from delivery_solver import DeliverySolver

@app.route('/api/solve', methods=['POST'])
def solve_delivery():
    # Initialize solver
    solver = DeliverySolver(current_scenario['no_fly_zones'])

    # Solve for optimal route
    solution = solver.find_optimal_route(
        warehouse=current_scenario['warehouse'],
        addresses=current_scenario['addresses'],
        fuel_budget=500  # Example budget
    )

    # Return solution
    return jsonify({
        'success': True,
        'solution': solution
    })
```

## Dependencies
- ISR Editor modules (SAMNavigator, OrienteeringSolver Interface)
- NumPy for geometric calculations
- Python 3.7+
