# ISR Agent Versions

## Single-Agent Architecture (v1) - December 2024

### Files
- `isr_agent_single_v1.py` - Main single-agent implementation with 17 tools
- `agent_memory_v1.json` - Agent memory with hard constraints

### Key Features
- Single LangGraph agent with all tools
- Priority constraint support for optimization
- Hard constraints enforcement (fuel, SAM/NFZ, mission constraints)
- Parallel solving with ThreadPoolExecutor

### How to Use Single-Agent Version
```bash
# To revert to single-agent version:
cp server/agents/versions/isr_agent_single_v1.py server/agents/isr_agent.py
cp server/agents/versions/agent_memory_v1.json agent_memory.json
```

### Hard Constraints (Never Violate)
1. **Fuel Budget** - Route must not exceed drone's fuel capacity
2. **SAM/NFZ Boundaries** - Paths must avoid all no-fly zones
3. **Mission Constraints** - ALL constraints must be respected:
   - Target type accessibility from UI
   - Priority constraints from commands (e.g., "D1 visits priority>=6")
   - Route order constraints (e.g., "start at T5")

### Tools Available
- Discovery: get_mission_overview, get_drone_info, get_distance, find_accessible_targets
- Validation: calculate_route_fuel, validate_drone_route, check_target_conflicts, get_mission_summary
- Single-Drone: suggest_drone_route, solve_optimal_route, solve_constrained_route
- Multi-Drone: allocate_targets_to_drones, solve_allocated_route, solve_with_constraints
- Optimization: optimize_assign_unvisited, optimize_reassign_targets, optimize_remove_crossings

---

## Multi-Agent Architecture (v2) - December 2024

### Files
- `../isr_agent_multi_v2.py` - Multi-agent implementation with 5 specialist agents

### Key Features
- **Supervisor-Worker Pattern**: Coordinator routes tasks to specialists
- **5 Specialist Agents** with focused tool sets:
  1. **Coordinator Agent** - Task decomposition, orchestration, handoffs
  2. **Allocator Agent** - Target allocation (standard + priority-constrained)
  3. **Router Agent** - Optimal route computation (Held-Karp, parallel solving)
  4. **Validator Agent** - Constraint validation, conflict detection
  5. **Optimizer Agent** - Post-optimization (insert, swap, 2-opt)
- **HANDOFF Protocol**: Agents pass control via tool return values
- **Same Entry Point**: `run_isr_agent()` maintains v1 API compatibility

### How to Use Multi-Agent Version
```bash
# To switch to multi-agent version:
cp server/agents/isr_agent_multi_v2.py server/agents/isr_agent.py
```

### Architecture Diagram
```
                    ┌─────────────────┐
                    │   COORDINATOR   │
                    │  (Entry Point)  │
                    └────────┬────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│    ALLOCATOR    │ │     ROUTER      │ │    VALIDATOR    │
│ Target division │ │  Held-Karp TSP  │ │ Constraint check│
└────────┬────────┘ └────────┬────────┘ └────────┬────────┘
         │                   │                   │
         └───────────────────┼───────────────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │    OPTIMIZER    │
                    │  Post-optimize  │
                    └─────────────────┘
```

### Agent Tool Sets

| Agent | Tools |
|-------|-------|
| Coordinator | get_mission_overview, route_to_allocator, route_to_router, route_to_validator, route_to_optimizer |
| Allocator | get_mission_overview, allocate_targets_to_drones, allocate_with_priority_constraints |
| Router | solve_optimal_route, solve_allocated_route, solve_all_drones_parallel |
| Validator | validate_drone_route, check_target_conflicts, get_mission_summary |
| Optimizer | optimize_assign_unvisited, optimize_reassign_targets, optimize_remove_crossings, get_mission_summary |

### Handoff Protocol
Agents communicate via HANDOFF messages in tool returns:
```
HANDOFF:ALLOCATOR:D1,D2: priority>=6; D3,D4: priority<=6
HANDOFF:ROUTER:1,2,3,4
HANDOFF:VALIDATOR:{"1": "A1,T3,A1", "2": "A2,T5,A2"}
HANDOFF:OPTIMIZER:all:{"1": "A1,T3,A1"}
```

### Comparison: v1 vs v2

| Feature | v1 Single-Agent | v2 Multi-Agent |
|---------|-----------------|----------------|
| Architecture | 1 agent, 17 tools | 5 agents, 4-5 tools each |
| System Prompt | ~2000 tokens | ~500 tokens per agent |
| Specialization | None | Deep domain expertise |
| Debugging | One large trace | Per-agent traces |
| Extensibility | Add tools | Add agents or tools |
| API | run_isr_agent() | run_isr_agent() (same) |

