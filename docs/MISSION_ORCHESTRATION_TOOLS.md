# Mission Orchestration Tools - Documentation

## Overview

The Mission Orchestration Tools provide a comprehensive API for LLM agents to orchestrate complex multi-segment missions using natural language commands. These tools wrap existing infrastructure (solver, optimizers, trajectory planning) to make them accessible through high-level function calls.

## Architecture

```
MissionOrchestrator (Singleton)
├── inspector: MissionInspector (CATEGORY 1)
├── solver: MissionSolver (CATEGORY 2)
├── optimizer: MissionOptimizer (CATEGORY 3)
├── trajectory: TrajectoryManipulator (CATEGORY 4)
├── segments: SegmentedMissionManager (CATEGORY 5)
└── constraints: ConstraintHelper (CATEGORY 6)
```

## Usage Examples

### Example 1: "Avoid targets of priority 7"

```python
orchestrator = get_orchestrator()

# Get all targets with priority 7
priority_7_targets = orchestrator.inspector.get_targets_by_priority(exact_priority=7)

# Solve excluding those targets
solution = orchestrator.solver.solve_mission(
    env=env,
    drone_configs=configs,
    exclude_targets=priority_7_targets
)
```

### Example 2: "Score highest priorities without visiting priority 10"

```python
# Get priority 10 targets
priority_10 = orchestrator.inspector.get_targets_by_priority(exact_priority=10)

# Solve excluding them
solution = orchestrator.solver.solve_mission(
    env=env,
    drone_configs=configs,
    exclude_targets=priority_10
)

# Optimize to maximize coverage
solution = orchestrator.optimizer.optimize_all(solution, env, configs)
```

### Example 3: "Land with at least 25% fuel unused"

```python
# Reduce fuel budgets by 25%
configs = orchestrator.inspector.get_current_drone_configs()
for drone_id in configs:
    configs[drone_id]['fuel_budget'] *= 0.75

# Solve with reduced budgets
solution = orchestrator.solver.solve_mission(
    env=env,
    drone_configs=configs
)
```

### Example 4: "Loiter 20 steps at type C targets"

```python
# Get type C targets
type_c_targets = orchestrator.inspector.get_targets_by_type(['c'])

# Create loiter cost map
loiter_costs = {tid: 20 for tid in type_c_targets}

# Solve with loiter costs (future enhancement - requires distance matrix modification)
# For now, post-process trajectories:
for drone_id, route_data in solution['routes'].items():
    trajectory = route_data['trajectory']
    route = route_data['route']
    
    modified_traj = orchestrator.trajectory.inject_loiter_at_targets(
        trajectory=trajectory,
        route=route,
        loiter_targets=type_c_targets,
        loiter_steps=20,
        waypoint_positions=waypoint_map
    )
    
    route_data['trajectory'] = modified_traj
    route_data['distance'] += len(type_c_targets) * 20  # Add loiter fuel cost
```

### Example 5: "Drones 1 & 5 shot down after 200 steps"

```python
# Step 1: Solve initial mission
solution = orchestrator.solver.solve_mission(env, configs)

# Step 2: Get positions at step 200
positions_at_200 = orchestrator.inspector.get_all_drone_positions_at_distance(200)

# Step 3: Get visited targets
visited = orchestrator.inspector.get_targets_visited_before_distance(200)

# Step 4: Calculate remaining fuel
remaining_fuel = {}
for drone_id in ['2', '3', '4']:
    initial = configs[drone_id]['fuel_budget']
    remaining_fuel[drone_id] = orchestrator.constraints.calculate_remaining_fuel(
        initial, 200
    )

# Step 5: Solve continuation without D1 & D5
continuation = orchestrator.solver.solve_continuation(
    env=env,
    enabled_drones=['2', '3', '4'],
    synthetic_starts={
        '2': positions_at_200['2'],
        '3': positions_at_200['3'],
        '4': positions_at_200['4']
    },
    visited_targets=visited,
    drone_configs=configs,
    fuel_budgets=remaining_fuel
)

# Step 6: Create segmented mission
segment0 = orchestrator.segments.create_segment(
    index=0,
    solution=solution,
    env=env,
    drone_configs=configs,
    cut_distance=200,
    cut_positions=positions_at_200,
    frozen_targets=visited,
    active_targets=[t['id'] for t in env['targets']]
)

segment1 = orchestrator.segments.create_segment(
    index=1,
    solution=continuation,
    env=env,
    drone_configs=configs,
    lost_drones=['1', '5'],
    frozen_targets=visited,
    active_targets=orchestrator.inspector.get_unvisited_targets(visited)
)

segmented_mission = orchestrator.segments.create_segmented_mission(
    segments=[segment0, segment1],
    base_env=env
)
```

### Example 6: "Solve segmented JSON"

```python
# Load existing segmented JSON
mission = orchestrator.segments.load_segmented_json("isr_env2512292325_N5_2.json")

# Re-solve each segment
for seg in mission['segments']:
    if seg['index'] == 0:
        # First segment: normal solve
        new_solution = orchestrator.solver.solve_mission(
            env=seg['env'],
            drone_configs=seg['drone_configs']
        )
    else:
        # Continuation segment
        new_solution = orchestrator.solver.solve_continuation(
            env=seg['env'],
            enabled_drones=[d for d, c in seg['drone_configs'].items() if c.get('enabled')],
            synthetic_starts=seg['cutPositions'],
            visited_targets=seg.get('frozenTargets', []),
            drone_configs=seg['drone_configs']
        )
    
    # Optimize
    new_solution = orchestrator.optimizer.optimize_all(
        new_solution,
        seg['env'],
        seg['drone_configs']
    )
    
    # Update segment
    seg['solution'] = new_solution

# Export updated mission
orchestrator.segments.export_segmented_json(mission, "output.json")
```

## Tool Categories

### CATEGORY 1: Inspection Tools
- `get_current_environment()` - Get airports, targets, SAMs
- `get_current_drone_configs()` - Get drone configurations
- `get_current_solution()` - Get current routes and sequences
- `get_all_targets()` - Get all targets
- `get_targets_by_priority(min, max, exact)` - Filter by priority
- `get_targets_by_type(types)` - Filter by sensor type
- `get_unvisited_targets(visited)` - Get unvisited targets
- `get_route_info(drone_id)` - Get detailed route info
- `get_drone_position_at_distance(drone_id, distance)` - Position at distance
- `get_all_drone_positions_at_distance(distance)` - All positions at distance
- `get_targets_visited_before_distance(distance)` - Visited targets before distance

### CATEGORY 2: Solving Tools
- `solve_mission(...)` - Solve from scratch with filters
- `solve_continuation(...)` - Solve from synthetic starts

### CATEGORY 3: Optimization Tools
- `optimize_insert_missed(...)` - Insert unvisited targets
- `optimize_swap_closer(...)` - Reassign to closer trajectories
- `optimize_no_crossing(...)` - Remove route crossings (2-opt)
- `optimize_all(...)` - Run all optimizers in sequence

### CATEGORY 4: Trajectory Manipulation
- `split_trajectory_at_distance(trajectory, distance)` - Split trajectory
- `calculate_trajectory_distance(points)` - Calculate distance
- `inject_loiter_at_targets(...)` - Add loiter waypoints

### CATEGORY 5: Segmented Mission Management
- `create_segment(...)` - Create segment object
- `create_segmented_mission(...)` - Create segmented mission JSON
- `load_segmented_json(filepath)` - Load from file
- `export_segmented_json(mission, filepath)` - Export to file

### CATEGORY 6: Constraint Helpers
- `calculate_remaining_fuel(budget, traveled)` - Remaining fuel
- `get_loiter_costs_for_target_types(targets, type_map)` - Loiter cost map
- `validate_solution(solution, configs)` - Validate constraints

## Integration with Agent

These tools should be exposed to the LLM agent as callable functions. The agent can then orchestrate complex missions by:

1. **Understanding** the natural language command
2. **Decomposing** it into tool calls
3. **Executing** the tool calls in sequence
4. **Validating** results
5. **Presenting** the solution to the user

## Future Enhancements

1. **Loiter in solver** - Modify distance matrix to include loiter costs
2. **Fuel-aware allocation** - Pass remaining fuel to allocator
3. **Dynamic target addition** - Add/remove targets during execution
4. **Asset management** - Add/remove drones between segments
5. **Constraint DSL** - Domain-specific language for constraints
