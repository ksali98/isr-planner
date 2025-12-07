# ISR Single-Agent Architecture Documentation

## Overview

The ISR (Intelligence, Surveillance, Reconnaissance) Mission Planning System is a LangGraph-based single-agent architecture designed to optimize multi-drone reconnaissance missions. The agent assists in creating optimal routes for drones to visit targets while respecting fuel budgets, start/end airports, target type restrictions, and avoiding SAM (Surface-to-Air Missile) zones.

### Key Characteristics
- **Single Agent with Multiple Tools**: One intelligent agent with 17 specialized tools
- **ReAct Pattern**: Reasoning + Acting loop for iterative problem solving
- **LangGraph Workflow**: State machine with agent → tools → agent loop
- **Parallel Solving**: ThreadPoolExecutor for multi-drone route optimization
- **SAM-Aware Navigation**: Pre-computed distance matrices that avoid no-fly zones
- **Persistent Memory**: JSON-based memory for cross-session learning
- **Hard Constraints**: Absolute rules that are never violated (fuel, SAM/NFZ, mission constraints)

---

## ReAct (Reasoning + Acting) Pattern

The system follows a ReAct pattern for problem solving:

```
1. Agent receives user query + environment context
2. Agent reasons about what tool to call
3. Tool executes and returns result
4. Agent reasons again (may call more tools)
5. Agent produces final response with ROUTE_Dx outputs
```

This iterative approach allows the agent to:
- Break complex missions into manageable steps
- Validate intermediate results before proceeding
- Recover from errors by trying alternative approaches
- Accumulate information needed for optimal planning

---

## System Integration & Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              SYSTEM ARCHITECTURE                             │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────┐     HTTP      ┌─────────────────┐  run_isr_agent()  ┌─────────────────┐
│     Web UI      │ ────────────► │  FastAPI Server │ ─────────────────► │    ISR Agent    │
│   index.html    │               │    main.py      │                    │  isr_agent.py   │
│                 │               │                 │                    │                 │
│ - Canvas visual │               │ - /api/agent    │                    │ - LangGraph     │
│ - Route editing │               │ - Context prep  │                    │ - 17 tools      │
└─────────────────┘               └─────────────────┘                    └────────┬────────┘
                                                                                  │
                                          ┌───────────────────────────────────────┘
                                          │
                                          ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    Solver Modules                            │
                    │                                                              │
                    │  post_optimizer.py      - Unvisited target insertion         │
                    │  target_allocator.py    - Multi-drone target allocation      │
                    │  sam_distance_matrix.py - SAM-aware distance calculation     │
                    │  solver_bridge.py       - Held-Karp orienteering solver      │
                    └────────────────────────────────┬────────────────────────────┘
                                                     │
                                                     ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │                   Path Planning Core                         │
                    │                                                              │
                    │  boundary_navigation.py                                      │
                    │  - SAM avoidance using convex polygon navigation             │
                    │  - Tangent-arc navigation for optimal paths                  │
                    │  - Both CW and CCW direction evaluation                      │
                    └─────────────────────────────────────────────────────────────┘
```

### Data Flow Steps

1. **User Interaction**: User interacts with Web UI (draws targets, edits drone configs, clicks "Run Agent")
2. **HTTP Request**: Web UI sends POST to `/api/agent` with environment + query
3. **Context Preparation**: FastAPI prepares context and calls `run_isr_agent()`
4. **Agent Processing**: Agent uses tools which access Solver modules for optimization
5. **Path Planning**: Solver uses Path Planning Core for SAM-aware distances
6. **Response**: Agent returns `ROUTE_Dx` formatted routes → Web UI renders on canvas

---

## LangGraph Workflow Architecture

```
                           ┌─────────────────────────────────────┐
                           │                                     │
       ┌───────┐           │          AGENT NODE                 │         ┌───────┐
       │ START │──────────►│   Claude claude-sonnet-4-20250514   │────────►│  END  │
       └───────┘           │        (with 17 tools bound)        │         └───────┘
                           │                                     │              ▲
                           └──────────────────┬──────────────────┘              │
                                              │                                 │
                                              │ has                             │
                                              │ tools?                          │
                                              │                            No   │
                                              ▼                                 │
                                        ┌───────────┐                           │
                                        │    Yes    │───────────────────────────┘
                                        └─────┬─────┘
                                              │
                                              ▼
                           ┌─────────────────────────────────────┐
                           │           TOOLS NODE                 │
                           │           (ToolNode)                 │
                           │                                     │
                           │   Executes tool calls from agent    │
                           └──────────────────┬──────────────────┘
                                              │
                                              │ Loop back to agent
                                              └───────────────────────────┐
                                                                          │
                                              ┌───────────────────────────┘
                                              │
                                              ▼
                                        [AGENT NODE]
```

### Workflow Execution Steps

1. User query enters at **START**
2. **AGENT NODE** calls LLM with system prompt + tools
3. Conditional edge checks for `tool_calls`
4. **YES** → **TOOLS NODE** executes the tool(s)
5. Loop back to **AGENT NODE**
6. **NO** → **END** (return response with ROUTE_Dx outputs)

### Graph Code Structure

```python
def create_isr_agent():
    workflow = StateGraph(ISRAgentState)
    workflow.add_node("agent", agent_node)
    workflow.add_node("tools", ToolNode(ALL_TOOLS))
    workflow.set_entry_point("agent")
    workflow.add_conditional_edges(
        "agent",
        should_continue,
        {"tools": "tools", "end": END}
    )
    workflow.add_edge("tools", "agent")
    return workflow.compile()
```

---

## Hard Constraints (NEVER VIOLATE)

These constraints are **ABSOLUTE** and must **NEVER** be violated, even during optimization:

### 1. Fuel Budget
- A drone's route **MUST NOT** exceed its fuel capacity
- Violating fuel budget = drone crashes = mission failure

### 2. SAM/NFZ Boundaries
- Routes **MUST** avoid all SAM zones and No-Fly Zones
- The distance matrix already accounts for this - use it

### 3. Mission Constraints
All constraints must be respected:
- **Target Type Accessibility**: If `target_access.c=false`, drone CANNOT visit type C targets
- **Priority Constraints**: "D1 visits priority>=6 only" means D1 can ONLY visit priority 6+ targets
- **Route Order Constraints**: "start at T5" or "visit T3 first" must be respected
- **Drone Exclusions**: "D5 not used" means D5 gets NO targets

When optimizing: ONLY add targets that the drone is ALLOWED to visit per ALL its constraints.

---

## Agent Tools (17 Total)

### Planning Tools (9)

| Tool | Purpose |
|------|---------|
| `get_mission_overview` | Get all airports, targets, SAMs, drone configs |
| `get_drone_info` | Get detailed constraints for a specific drone |
| `get_distance` | Look up distance between two waypoints |
| `calculate_route_fuel` | Calculate total fuel for a drone route |
| `validate_drone_route` | Validate route against all constraints |
| `find_accessible_targets` | Find targets drone can reach, sorted by efficiency |
| `suggest_drone_route` | Generate greedy baseline route for drone |
| `check_target_conflicts` | Check for duplicate target assignments |
| `get_mission_summary` | Get summary stats for multi-drone plan |

### Solving Tools (5)

| Tool | Purpose |
|------|---------|
| `solve_optimal_route` | Held-Karp optimal route for single drone |
| `solve_constrained_route` | Solve with custom start/end and fuel constraints |
| `allocate_targets_to_drones` | Divide targets among drones before solving |
| `solve_allocated_route` | Solve optimal route using only allocated targets |
| `solve_with_constraints` | Fast one-shot solver: parse, allocate, solve ALL drones in parallel |

### Optimization Tools (3)

| Tool | Purpose |
|------|---------|
| `optimize_assign_unvisited` | Insert unvisited targets into routes |
| `optimize_reassign_targets` | Swap targets to closer drone trajectories |
| `optimize_remove_crossings` | Fix self-crossing trajectories (2-opt) |

### Tool Input/Output Pattern

All tools follow a consistent pattern:
- **Input**: Tool-specific parameters (drone_id, waypoints, routes dict, etc.)
- **Output**: Formatted string with results

### Shared Context (Global Variables)

| Variable | Purpose |
|----------|---------|
| `_current_env` | Environment with airports, targets, SAMs |
| `_drone_configs` | Per-drone configuration (fuel, airports, access) |
| `_distance_matrix` | Pre-computed SAM-aware distances |
| `_sam_paths` | Pre-computed SAM-avoiding paths |
| `_current_sequences` | Current route sequences per drone |
| `_target_allocation` | Target assignments per drone |

---

## State Management

### ISRAgentState Class

```python
class ISRAgentState(dict):
    """State for the ISR planning agent."""
    messages: Annotated[list, add_messages]
    environment: Optional[Dict[str, Any]]
    drone_configs: Optional[Dict[str, Any]]
    current_sequences: Optional[Dict[str, str]]
    proposed_routes: Optional[Dict[str, Any]]
```

---

## Key Tool: solve_with_constraints

The most powerful tool - handles the complete workflow in one call:

```python
@tool
def solve_with_constraints(constraints: str, strategy: str = "efficient") -> str:
    """
    Fast one-shot solver: Parse constraints, allocate, and solve ALL drones in PARALLEL.

    Args:
        constraints: Priority constraints in format "D1,D2: priority>=6; D3,D4: priority<=6"
                    Supported operators: >=, <=, >, <, =
        strategy: Allocation strategy (efficient, greedy, balanced, geographic, exclusive)
    """
```

**Internal Flow:**
1. Parse constraints using `parse_priority_constraints()`
2. Filter to only constrained drones
3. Allocate targets with `allocate_with_priority_filters()`
4. Solve all drones in parallel using `ThreadPoolExecutor`
5. Return complete solution with routes

### Allocation Strategies

| Strategy | Description |
|----------|-------------|
| `efficient` | Maximize priority/distance ratio (default, best for most missions) |
| `greedy` | Assign highest priority targets to nearest capable drone |
| `balanced` | Distribute targets evenly by count |
| `geographic` | Divide environment into angular sectors per drone |
| `exclusive` | Prioritize targets only one drone can reach first |

---

## Persistent Memory System

### Memory File Location
```python
MEMORY_FILE = Path(__file__).parent.parent.parent / "agent_memory.json"
```

### Memory Categories
- `correction`: User corrected the agent's behavior
- `instruction`: User gave a standing instruction
- `preference`: User preference for how to do things
- `fact`: Important fact to remember

### Memory Functions

| Function | Purpose |
|----------|---------|
| `load_memory()` | Load memories from JSON file |
| `save_memory()` | Save memories to JSON file |
| `add_memory(content, category)` | Add new memory entry |
| `get_active_memories()` | Get all active memories |
| `clear_memory()` | Clear all memories |
| `delete_memory(id)` | Delete specific memory |
| `format_memories_for_prompt()` | Format memories for system prompt |

### Memory Injection
Memories are automatically injected into the system prompt:
```python
def agent_node(state: ISRAgentState) -> Dict[str, Any]:
    memories_text = format_memories_for_prompt()
    full_system_prompt = SYSTEM_PROMPT + memories_text
```

---

## SAM-Aware Distance Matrix

### Computation Flow
```python
def _compute_distance_matrix(env: Dict[str, Any]) -> tuple:
    """
    Compute distance matrix from environment.
    Uses SAM-aware distances if SAMs are present, otherwise Euclidean.
    """
```

1. If SAMs exist → Call `calculate_sam_aware_matrix(env)` from `sam_distance_matrix.py`
2. If no SAMs → Compute simple Euclidean distances
3. Returns `(distance_matrix_dict, sam_paths_dict)`

### Path Retrieval
```python
def get_sam_path(from_id: str, to_id: str) -> Optional[List[List[float]]]:
    """Get the pre-computed SAM-avoiding path between two waypoints."""
```

---

## Entry Point: run_isr_agent

```python
def run_isr_agent(env: Dict[str, Any], user_query: str,
                  drone_configs: Optional[Dict[str, Any]] = None,
                  sequences: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
```

**Returns:**
```python
{
    "response": str,           # Agent's text response
    "routes": Dict[str, List[str]],  # Drone routes ({"1": ["A1", "T3", "A1"]})
    "trajectories": Dict[str, List],  # SAM-avoiding trajectories for UI
    "total_points": int,       # Total points collected
    "total_fuel": float,       # Total fuel used
    "route": List[str],        # Legacy single-route (D1)
    "points": int,             # Legacy points
    "fuel": float,             # Legacy fuel
}
```

---

## File Structure

```
isr_web/
├── index.html                  # Web UI with canvas visualization
├── agent_memory.json           # Persistent memory storage
├── server/
│   ├── main.py                 # FastAPI server (/api/agent endpoint)
│   ├── agents/
│   │   ├── isr_agent.py        # Main single-agent implementation (17 tools)
│   │   ├── graph.py            # Alternative OpenAI-based agent
│   │   └── versions/           # Saved agent versions
│   │       ├── isr_agent_single_v1.py
│   │       ├── agent_memory_v1.json
│   │       └── README.md
│   └── solver/
│       ├── solver_bridge.py    # Held-Karp solver bridge
│       ├── target_allocator.py # Target allocation with priority filters
│       ├── post_optimizer.py   # Optimization functions (swap, 2-opt)
│       └── sam_distance_matrix.py  # SAM-aware distances
├── docs/
│   ├── Single_Agent_Architecture.md  # This document
│   └── ISR_Agent_Architecture.pdf    # Visual architecture diagrams
└── path_planning_core/
    └── boundary_navigation.py  # SAM avoidance, tangent-arc navigation
```

---

## Version Backup & Restoration

### Saved Version Files

The single-agent v1 implementation has been backed up to the `versions/` folder:

| File | Description |
|------|-------------|
| `server/agents/versions/isr_agent_single_v1.py` | Main agent with 17 tools (81KB) |
| `server/agents/versions/agent_memory_v1.json` | Agent memory with hard constraints |
| `server/solver/post_optimizer_v1.py` | Post-optimization with constraint enforcement |
| `server/solver/target_allocator_v1.py` | Target allocation with priority filters |

### How to Restore Single-Agent v1

To restore the single-agent system to its December 2024 v1 state:

```bash
cd /Users/kamalali/isr_projects/isr_web

# Restore agent files
cp server/agents/versions/isr_agent_single_v1.py server/agents/isr_agent.py
cp server/agents/versions/agent_memory_v1.json agent_memory.json

# Restore solver files
cp server/solver/post_optimizer_v1.py server/solver/post_optimizer.py
cp server/solver/target_allocator_v1.py server/solver/target_allocator.py
```

### Verification After Restoration

After restoring, verify the system works:

```bash
# Restart the server
source venv/bin/activate
export PYTHONPATH=/Users/kamalali/isr_projects
python -m uvicorn server.main:app --reload --port 8893
```

Then test via the Web UI at `http://localhost:8893` or via API.

---

## Usage Patterns

### Pattern 1: Priority-Constrained Multi-Drone Mission
```
User: "D1,D2 should visit priority>=6 targets, D3,D4 visit priority<=6"

Agent Flow:
1. get_mission_overview()
2. solve_with_constraints("D1,D2: priority>=6; D3,D4: priority<=6")
3. Output ROUTE_Dx format
```

### Pattern 2: Standard Multi-Drone Mission
```
User: "Plan routes for all enabled drones"

Agent Flow:
1. get_mission_overview()
2. allocate_targets_to_drones(strategy="efficient")
3. solve_allocated_route("1")
4. solve_allocated_route("2")
5. validate_drone_route(...)
6. get_mission_summary(...)
7. Output ROUTE_Dx format
```

### Pattern 3: Single-Drone Mission
```
User: "Plan optimal route for D1"

Agent Flow:
1. get_mission_overview()
2. solve_optimal_route("1")
3. validate_drone_route(...)
4. Output ROUTE_D1 format
```

### Pattern 4: Optimization Request
```
User: "Optimize and insert missed targets"

Agent Flow:
1. optimize_assign_unvisited(routes, priority_constraints)
2. optimize_remove_crossings(routes)
3. Output updated ROUTE_Dx format
```

---

## Configuration

### Environment Structure
```json
{
  "airports": [{"id": "A1", "x": 10, "y": 10}, ...],
  "targets": [{"id": "T1", "x": 50, "y": 30, "priority": 8, "type": "a"}, ...],
  "sams": [{"id": "S1", "pos": [50, 50], "range": 12}, ...]
}
```

### Drone Configuration Structure
```json
{
  "1": {
    "enabled": true,
    "fuel_budget": 150,
    "start_airport": "A1",
    "end_airport": "A1",
    "target_access": {"a": true, "b": true, "c": false, "d": true, "e": true}
  },
  ...
}
```

---

## LLM Configuration

```python
from langchain_anthropic import ChatAnthropic

llm = ChatAnthropic(
    model="claude-sonnet-4-20250514",
    temperature=0,
    max_tokens=4096,
)
```

---

## Limitations & Future Improvements

### Current Limitations
1. **Global State**: Module-level variables require careful management
2. **Single-Agent Bottleneck**: Complex reasoning handled by one agent
3. **No Real-Time Learning**: Memory only persists between sessions, not during
4. **Fixed Tool Set**: Tools cannot be added/removed dynamically

### Recommended Improvements for Multi-Agent Migration
1. **Replace globals with MissionContext class** for cleaner state management
2. **Tool Registry pattern** for dynamic tool loading
3. **Experience Store** for learning from mission outcomes
4. **Specialist Agents** for allocation, routing, optimization

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Dec 2024 | Initial single-agent architecture |
| 1.1 | Dec 2024 | Added `solve_with_constraints` parallel solver |
| 1.2 | Dec 2024 | Fixed D5 allocation bug in constrained solving |
| 1.3 | Dec 2024 | Documentation created |
| 1.4 | Dec 2024 | Added hard constraints enforcement in optimization tools |
| 1.5 | Dec 2024 | Merged PDF architecture diagrams into documentation |

---

## References

- [LangGraph Documentation](https://langchain-ai.github.io/langgraph/)
- [LangChain Anthropic](https://python.langchain.com/docs/integrations/chat/anthropic)
- [Held-Karp Algorithm](https://en.wikipedia.org/wiki/Held%E2%80%93Karp_algorithm)
- [ISR_Agent_Architecture.pdf](./ISR_Agent_Architecture.pdf) - Visual architecture diagrams
