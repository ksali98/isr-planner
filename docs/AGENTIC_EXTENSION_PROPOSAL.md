# Current Agentic Structure Analysis & Extension Proposal

## Current Architecture (isr_agent_multi_v3.py)

### Existing 5-Agent System

```
                    ┌──────────────┐
                    │ COORDINATOR  │
                    │ (Orchestrator)│
                    └──────┬───────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
      ┌────▼────┐    ┌────▼────┐    ┌────▼────┐
      │ALLOCATOR│    │ ROUTER  │    │VALIDATOR│
      │         │    │         │    │         │
      └─────────┘    └─────────┘    └─────────┘
                           │
                      ┌────▼────┐
                      │OPTIMIZER│
                      │         │
                      └─────────┘
```

### Agent Responsibilities

1. **Coordinator** - Task decomposition, workflow orchestration
   - Routes to specialist agents
   - Aggregates results
   - Presents final solution

2. **Allocator** - Target distribution across drones
   - Strategies: efficient, greedy, balanced, geographic, exclusive
   - Respects priority constraints

3. **Router** - Optimal route computation
   - Held-Karp algorithm (exact TSP)
   - Works with allocated targets

4. **Validator** - Constraint checking
   - Fuel budget validation
   - Capability checking
   - Conflict detection

5. **Optimizer** - Post-optimization
   - Insert Missed targets
   - Swap Closer (trajectory-based reassignment)
   - No Crossing (2-opt)

### Communication Mechanism

**Pattern**: Tool-based handoff via shared state
- Each agent has specialized tools
- `route_to_<agent>()` tools trigger handoffs
- Shared `MissionState` object contains:
  - `messages`: Conversation history
  - `environment`: Airports, targets, SAMs
  - `drone_configs`: Fuel budgets, capabilities
  - `target_allocation`: Allocator output
  - `computed_routes`: Router output
  - `validation_results`: Validator output

### Current Limitations

1. **No constraint orchestration agent** - Current agents handle specific tasks but don't orchestrate complex multi-constraint scenarios
2. **No segmented mission support** - Agents don't handle multi-segment missions with losses/checkpoints
3. **No natural language constraint parsing** - Coordinator doesn't decompose complex NL constraints into tool calls
4. **Limited trajectory manipulation** - No agent handles loiter, splitting, or advanced trajectory operations

---

## Proposed Extension: Add 2 New Agents

### Option 1: Add "Mission Planner" Agent (RECOMMENDED)

```
                    ┌──────────────┐
                    │ COORDINATOR  │
                    │ (Orchestrator)│
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │MISSION PLANNER│◄──── NEW!
                    │(Constraint   │
                    │ Orchestrator)│
                    └──────┬───────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
      ┌────▼────┐    ┌────▼────┐    ┌────▼────┐
      │ALLOCATOR│    │ ROUTER  │    │VALIDATOR│
      │         │    │         │    │         │
      └─────────┘    └─────────┘    └─────────┘
                           │
                      ┌────▼────┐
                      │OPTIMIZER│
                      │         │
                      └─────────┘
```

**Mission Planner Agent Responsibilities:**
- **Constraint decomposition** - Parse complex NL constraints into executable steps
- **Multi-segment orchestration** - Handle missions with losses, checkpoints, refueling
- **Trajectory manipulation** - Loiter injection, position calculations
- **Segmented JSON operations** - Load, solve, export segmented missions
- **Fuel management** - Track remaining fuel, adjust budgets

**Tools** (from mission_orchestration_tools.py):
- All Inspection tools (30+)
- Solving tools (solve_mission, solve_continuation)
- Trajectory tools (split, loiter, distances)
- Segmentation tools (create/load/export)
- Constraint helpers (fuel, validation)

**Integration:**
```python
@tool
def route_to_mission_planner(constraint_description: str) -> str:
    """Route complex constraint scenarios to Mission Planner."""
    return f"HANDOFF:mission_planner:{constraint_description}"
```

**Workflow Example:**
```
User: "D1 & D5 shot down at 200 steps, complete mission"
│
└─> COORDINATOR
    └─> MISSION_PLANNER
        ├─> solve_mission() via ROUTER
        ├─> get_positions_at_distance(200)
        ├─> get_visited_targets_before(200)
        ├─> solve_continuation() via ROUTER
        ├─> create_segmented_mission()
        └─> Return to COORDINATOR
```

---

### Option 2: Split into "Constraint Parser" + "Segment Manager" (MORE MODULAR)

```
                    ┌──────────────┐
                    │ COORDINATOR  │
                    └──────┬───────┘
                           │
           ┌───────────────┼────────────────────┐
           │               │                    │
      ┌────▼────────┐ ┌───▼────────┐  ┌───────▼────────┐
      │CONSTRAINT   │ │SEGMENT     │  │  (existing     │
      │PARSER       │ │MANAGER     │  │   agents)      │
      │(NL→Steps)   │ │(Multi-seg) │  │                │
      └─────────────┘ └────────────┘  └────────────────┘
```

**Constraint Parser Agent:**
- Parse natural language constraints
- Identify constraint type (pre-solve, post-solve, multi-segment)
- Create execution plan
- Route to appropriate agents

**Segment Manager Agent:**
- Handle multi-segment mission workflows
- Track drone losses, checkpoint replanning
- Manage segmented JSON operations
- Coordinate synthetic starts and visited targets

**Pros:**
- ✅ More modular (single responsibility)
- ✅ Easier to test and debug
- ✅ Can use Constraint Parser for non-segmented missions

**Cons:**
- ❌ More complex coordination
- ❌ Two handoffs instead of one

---

## Recommendation: **Option 1 - Single Mission Planner Agent**

### Rationale:
1. **Simpler coordination** - One agent handles all complex orchestration
2. **Natural abstraction** - Mission planning is cohesive task
3. **Fewer handoffs** - Better performance
4. **Easier to extend** - Can add capabilities without changing graph
5. **Matches existing pattern** - Similar to how Optimizer combines 3 operations

### Implementation Plan

#### Step 1: Create Mission Planner Agent Node
```python
# In isr_agent_multi_v3.py or new v5.py

MISSION_PLANNER_SYSTEM = """You are the Mission Planner Agent for ISR operations.

Your expertise: Complex constraint orchestration and multi-segment mission planning.

You handle:
- Complex natural language constraints (fuel reserves, priority filters, loiter)
- Multi-segment missions (drone losses, checkpoints, refueling)
- Trajectory manipulation (loiter injection, position tracking)
- Segmented JSON operations (load, solve, export)

You have access to the mission_orchestration_tools which provide:
- Inspector: Query targets, positions, routes
- Solver: solve_mission, solve_continuation
- Optimizer: Insert/Swap/NoCrossing (via other agents)
- Trajectory: Split, loiter, distance calculations
- Segments: Create/load/export segmented missions
- Constraints: Fuel management, validation

Your workflow:
1. Understand the constraint/requirement
2. Decompose into steps
3. Use orchestration tools to execute
4. Coordinate with other agents (Router, Optimizer) as needed
5. Report results clearly

Always explain your reasoning and show your work."""

def mission_planner_node(state: MissionState) -> MissionState:
    """Mission Planner agent - handles complex constraints and segmentation."""
    # Get orchestrator instance
    from mission_orchestration_tools import get_orchestrator
    orchestrator = get_orchestrator()
    
    # Set context
    if state.get("environment"):
        orchestrator.inspector.set_context(
            env=state["environment"],
            solution=state.get("computed_routes"),
            drone_configs=state.get("drone_configs")
        )
    
    # Create LLM with tools
    mission_planner_tools = create_mission_planner_tools()
    llm = ChatAnthropic(model="claude-3-5-sonnet-20241022")
    llm_with_tools = llm.bind_tools(mission_planner_tools)
    
    # Build messages
    messages = state["messages"]
    system_message = SystemMessage(content=MISSION_PLANNER_SYSTEM)
    
    # Invoke LLM
    response = llm_with_tools.invoke([system_message] + messages)
    
    return {"messages": [response]}
```

#### Step 2: Create Mission Planner Tools
```python
def create_mission_planner_tools():
    """Create tools that wrap mission_orchestration_tools for LLM access."""
    
    from mission_orchestration_tools import get_orchestrator
    orchestrator = get_orchestrator()
    
    @tool
    def get_targets_by_priority(min_priority: int = None, max_priority: int = None) -> str:
        """Get target IDs filtered by priority."""
        targets = orchestrator.inspector.get_targets_by_priority(min_priority, max_priority)
        return json.dumps(targets)
    
    @tool
    def get_drone_positions_at_distance(distance: float) -> str:
        """Get all drone positions at a specific distance along their trajectories."""
        positions = orchestrator.inspector.get_all_drone_positions_at_distance(distance)
        return json.dumps({k: list(v) for k, v in positions.items()})
    
    @tool
    def solve_mission_continuation(
        enabled_drones: str,  # "2,3,4"
        synthetic_starts_json: str,  # JSON dict
        visited_targets: str  # "T1,T5,T8"
    ) -> str:
        """Solve continuation from synthetic start positions."""
        enabled = enabled_drones.split(",")
        starts = json.loads(synthetic_starts_json)
        visited = visited_targets.split(",")
        
        # Use current state context
        result = orchestrator.solver.solve_continuation(
            env=orchestrator.inspector._current_env,
            enabled_drones=enabled,
            synthetic_starts=starts,
            visited_targets=visited,
            drone_configs=orchestrator.inspector._current_drone_configs
        )
        return json.dumps(result)
    
    # Add more tool wrappers...
    
    return [
        get_targets_by_priority,
        get_drone_positions_at_distance,
        solve_mission_continuation,
        # ... more tools
    ]
```

#### Step 3: Add to Graph
```python
workflow = StateGraph(MissionState)

# Existing nodes
workflow.add_node("coordinator", coordinator_node)
workflow.add_node("coordinator_tools", coordinator_tools_node)
workflow.add_node("allocator", allocator_node)
workflow.add_node("allocator_tools", allocator_tools_node)
workflow.add_node("router", router_node)
workflow.add_node("router_tools", router_tools_node)
workflow.add_node("validator", validator_node)
workflow.add_node("validator_tools", validator_tools_node)
workflow.add_node("optimizer", optimizer_node)
workflow.add_node("optimizer_tools", optimizer_tools_node)

# NEW: Mission Planner
workflow.add_node("mission_planner", mission_planner_node)
workflow.add_node("mission_planner_tools", mission_planner_tools_node)

# Add edges
workflow.add_edge(START, "coordinator")
workflow.add_conditional_edges("coordinator", coordinator_route)
workflow.add_edge("coordinator_tools", "coordinator")
# ... existing edges ...

# NEW: Mission Planner edges
workflow.add_conditional_edges("mission_planner", mission_planner_route)
workflow.add_edge("mission_planner_tools", "mission_planner")
```

#### Step 4: Add Handoff Tool to Coordinator
```python
@tool
def route_to_mission_planner(constraint_description: str = "") -> str:
    """
    Route complex constraint scenarios to Mission Planner Agent.
    
    Use when:
    - User specifies multi-segment missions (losses, checkpoints)
    - Complex fuel constraints (reserve %, refueling)
    - Loiter requirements
    - Segmented JSON operations
    - Position-based constraints
    """
    return f"HANDOFF:mission_planner:{constraint_description}"
```

---

## Migration Path

### Phase 1: Minimal Integration (Quick Win)
- Add Mission Planner agent to v3
- Expose 5-10 critical tools (target filtering, positions, continuation solving)
- Test with simple examples ("avoid priority 7")

### Phase 2: Full Integration
- Expose all orchestration tools (30+)
- Add segmented mission support
- Test complex scenarios (drone losses, loiter)

### Phase 3: Refinement
- Optimize tool descriptions for better LLM understanding
- Add error handling and retry logic
- Performance tuning

---

## Alternative: Enhance Coordinator Instead of New Agent

Instead of adding a new agent, we could **enhance the Coordinator** with orchestration tools:

**Pros:**
- ✅ No new agent (simpler graph)
- ✅ Coordinator already orchestrates

**Cons:**
- ❌ Coordinator becomes bloated (too many responsibilities)
- ❌ Harder to specialize prompts
- ❌ Mixing high-level orchestration with low-level operations

**Verdict**: Not recommended. Keeping Coordinator focused on delegation is cleaner.

---

## Conclusion

**Recommendation: Add Mission Planner Agent (Option 1)**

This provides:
1. Clean separation of concerns
2. Natural place for complex constraint handling
3. Easy integration with existing agents
4. Room for future expansion
5. Matches the orchestration tools we just built

The architecture becomes:
```
User → Coordinator → Mission Planner → [Allocator, Router, Validator, Optimizer]
                                    ↓
                           (uses orchestration tools)
```

Mission Planner acts as a "meta-orchestrator" for complex scenarios, while Coordinator remains the top-level workflow manager.
