# ISR Multi-Agent Architecture

## Overview

The ISR Planner uses a **LangGraph-based multi-agent architecture** where specialized agents communicate through a shared state graph and handle specific aspects of mission planning.

## Architecture Type: Multi-Agent with Shared State

**Framework:** LangGraph (built on LangChain)
**Communication Pattern:** Shared State + Message Passing + Tool-based Handoff
**Orchestration:** Coordinator-driven with conditional routing

---

## Agent Roster

### 1. **Coordinator Agent**
**Role:** Task decomposition and orchestration
**Responsibilities:**
- Receives user requests
- Decomposes complex tasks into subtasks
- Routes work to specialized agents via tool calls
- Aggregates results and presents final solution

**Tools:**
- `route_to_allocator()` - Delegate to Allocator Agent
- `route_to_router()` - Delegate to Router Agent
- `route_to_validator()` - Delegate to Validator Agent
- `route_to_optimizer()` - Delegate to Optimizer Agent
- `get_mission_overview()` - Get environment summary

**Location:** [isr_agent_multi_v2.py:1206-1305](../server/agents/isr_agent_multi_v2.py#L1206-L1305)

---

### 2. **Allocator Agent**
**Role:** Target allocation specialist
**Responsibilities:**
- Distributes targets across drones
- Considers fuel budgets, capabilities, and priority constraints
- Uses sophisticated allocation strategies

**Tools:**
- `allocate_targets_to_drones(strategy)` - Main allocation function
- `allocate_with_priority_constraints(constraints, strategy)` - Priority-filtered allocation

**Allocation Strategies:**
- `"efficient"` - Maximize priority/distance ratio (default)
- `"greedy"` - Assign highest priority to nearest drone
- `"balanced"` - Distribute targets evenly by count
- `"geographic"` - Divide by angular sectors
- `"exclusive"` - Prioritize targets only one drone can reach

**Location:** [isr_agent_multi_v2.py:1307-1366](../server/agents/isr_agent_multi_v2.py#L1307-L1366)

---

### 3. **Router Agent**
**Role:** Optimal route computation
**Responsibilities:**
- Solves orienteering problem for each drone
- Uses Held-Karp algorithm for optimal TSP solutions
- Works with allocated targets

**Tools:**
- `solve_optimal_route(drone_id)` - Solve for single drone (all targets)
- `solve_allocated_route(drone_id)` - Solve with allocated targets only
- `solve_all_drones_parallel()` - Solve all drones concurrently

**Algorithm:** Held-Karp dynamic programming (exact TSP solver)

**Location:** [isr_agent_multi_v2.py:1368-1393](../server/agents/isr_agent_multi_v2.py#L1368-L1393)

---

### 4. **Validator Agent**
**Role:** Constraint checking and validation
**Responsibilities:**
- Validates routes against fuel budgets
- Checks drone capabilities vs target types
- Detects target conflicts (double assignments)
- Ensures airport constraints are met

**Tools:**
- `validate_drone_route(drone_id, waypoints)` - Validate single route
- `check_target_conflicts(routes)` - Find duplicate target assignments
- `get_mission_summary(routes)` - Statistics and coverage report

**Location:** [isr_agent_multi_v2.py:1395-1418](../server/agents/isr_agent_multi_v2.py#L1395-L1418)

---

### 5. **Optimizer Agent**
**Role:** Post-optimization specialist
**Responsibilities:**
- Adds unvisited targets (Insert Missed)
- Reassigns targets to closer drones (Swap Closer)
- Removes route crossings (2-opt)

**Tools:**
- `optimize_assign_unvisited(routes, priority_constraints)` - Insert Missed
- `optimize_reassign_targets(routes, priority_constraints)` - Swap Closer
- `optimize_remove_crossings(routes)` - Crossing Remove (2-opt)

**Algorithms:** See [ISR_Optimization_Tools_Enhanced.md](ISR_Optimization_Tools_Enhanced.md)

**Location:** [isr_agent_multi_v2.py:1420-1460](../server/agents/isr_agent_multi_v2.py#L1420-L1460)

---

## Communication Architecture

### Shared State: `MultiAgentState`

All agents share a common state object that includes:

```python
class MultiAgentState(TypedDict):
    # Conversation
    messages: Annotated[list, add_messages]  # Full message history

    # Environment
    environment: Optional[Dict[str, Any]]     # Airports, targets, SAMs
    drone_configs: Optional[Dict[str, Any]]   # Fuel budgets, capabilities
    current_sequences: Optional[Dict[str, str]]  # Current routes

    # Planning State
    mission_overview: Optional[str]
    target_allocation: Optional[Dict[str, List[str]]]  # Allocator output
    computed_routes: Optional[Dict[str, Dict[str, Any]]]  # Router output
    validation_results: Optional[Dict[str, str]]  # Validator output
    priority_constraints: Optional[str]

    # Orchestration
    current_agent: str           # Active agent
    next_agent: Optional[str]    # Where to route next
    task_complete: bool          # Mission done?
```

**Location:** [isr_agent_multi_v2.py:206-224](../server/agents/isr_agent_multi_v2.py#L206-L224)

---

## Communication Patterns

### 1. **Message Passing**
- Agents communicate via LangChain `messages` list
- Message types:
  - `HumanMessage` - User input
  - `AIMessage` - Agent responses
  - `ToolMessage` - Tool execution results
  - `SystemMessage` - System prompts

### 2. **Tool-based Handoff**
- Coordinator uses special routing tools to delegate:
  ```python
  @tool
  def route_to_allocator(priority_constraints: str = "") -> str:
      """Route task to the Allocator Agent for target allocation."""
      return f"HANDOFF:allocator:{priority_constraints}"
  ```
- Tool returns `"HANDOFF:<agent_name>:<context>"`
- Workflow routing layer detects handoff and routes to target agent

### 3. **Conditional Routing**
- Each agent has a routing function that decides next step:
  - Execute tools? → Go to `<agent>_tools` node
  - Task complete? → Return to `coordinator`
  - End? → Go to `END`

**Example: Coordinator Routing**
```python
def coordinator_route(state: MultiAgentState) -> str:
    last_message = state["messages"][-1]

    if last_message.tool_calls:
        return "coordinator_tools"  # Execute tools

    if "ROUTE_D" in last_message.content:
        return "end"  # Final routes found

    return "end"  # No action, terminate
```

### 4. **Tool Node Processing**
- After agent decides to use tools, execution goes to `<agent>_tools` ToolNode
- ToolNode executes all tool calls in parallel
- Results appended as ToolMessages
- If tool returns `"HANDOFF:..."`, routing layer redirects to target agent
- Otherwise, returns to originating agent

---

## Workflow Graph Structure

```
┌─────────────┐
│   START     │
└──────┬──────┘
       │
       v
┌─────────────────────────────────────────────┐
│          COORDINATOR AGENT                  │
│  - Receives user request                    │
│  - Decides what needs to be done            │
│  - Routes to specialist agents              │
└──────┬──────────────────────────────────────┘
       │
       │ (tool calls)
       v
┌─────────────────────┐
│  COORDINATOR_TOOLS  │
│  - Execute tools    │
│  - Detect HANDOFFs  │
└──────┬──────────────┘
       │
       │ (conditional routing based on HANDOFF)
       │
   ┌───┴───┐
   │       │
   v       v       (similar pattern for each agent)
┌──────┐  ┌─────────┐
│ALLOC │  │ ROUTER  │  ...
└──┬───┘  └────┬────┘
   │           │
   v           v
┌──────────┐  ┌─────────────┐
│ALLOC_TOOL│  │ROUTER_TOOLS │
└──┬───────┘  └────┬─────────┘
   │               │
   └───────┬───────┘
           │
           v (back to coordinator after specialist work)
   ┌───────────────┐
   │  COORDINATOR  │
   └───────┬───────┘
           │
           v (when routes complete)
       ┌───────┐
       │  END  │
       └───────┘
```

**Graph Definition:** [isr_agent_multi_v2.py:1467-1610](../server/agents/isr_agent_multi_v2.py#L1467-L1610)

---

## Example Execution Flow

### User Request: "Solve this mission with efficient allocation"

1. **START** → **COORDINATOR**
   - Coordinator receives user message
   - Decides: "Need to allocate targets first"
   - Calls `route_to_allocator(priority_constraints="")`

2. **COORDINATOR** → **COORDINATOR_TOOLS**
   - Tool executes, returns `"HANDOFF:allocator:"`

3. **COORDINATOR_TOOLS** → **ALLOCATOR** (via conditional routing)
   - Allocator receives context
   - Calls `allocate_targets_to_drones(strategy="efficient")`

4. **ALLOCATOR** → **ALLOCATOR_TOOLS**
   - Tool executes allocation algorithm
   - Stores results in `state["target_allocation"]`
   - Returns allocation summary

5. **ALLOCATOR_TOOLS** → **ALLOCATOR**
   - Allocator formats results
   - No more tool calls

6. **ALLOCATOR** → **COORDINATOR**
   - Allocator reports: "Allocation complete, D1 has [T1, T3], D2 has [T2, T5]..."

7. **COORDINATOR** → **COORDINATOR_TOOLS**
   - Coordinator decides: "Now need to compute routes"
   - Calls `route_to_router(drone_ids="1,2,3,4,5")`

8. **COORDINATOR_TOOLS** → **ROUTER**
   - Router receives handoff
   - Calls `solve_all_drones_parallel()`

9. **ROUTER** → **ROUTER_TOOLS**
   - Tool runs Held-Karp for each drone
   - Stores routes in `state["computed_routes"]`

10. **ROUTER_TOOLS** → **ROUTER**
    - Router formats route results

11. **ROUTER** → **COORDINATOR**
    - Router reports: "Routes computed for all drones"

12. **COORDINATOR** → **END**
    - Coordinator formats final output
    - Presents routes to user
    - Graph terminates

---

## Key Design Decisions

### 1. **Why Multi-Agent?**
- **Separation of Concerns:** Each agent has focused responsibility
- **Modularity:** Easy to swap allocation strategies or routing algorithms
- **Clarity:** System prompt can be tailored to each agent's specialty
- **Parallel Execution:** Router can solve multiple drones concurrently

### 2. **Why LangGraph?**
- **State Management:** Shared state automatically passed between nodes
- **Conditional Routing:** Easy to route based on tool outputs or message content
- **Visualization:** Can render graph for debugging
- **Message History:** Automatic conversation tracking

### 3. **Why Tool-based Handoff?**
- **Explicit Control:** Coordinator explicitly decides routing
- **Context Passing:** Can pass parameters via handoff message
- **Debuggability:** Clear log of which agent ran when
- **LLM-friendly:** LLM can decide which specialist to call based on task

### 4. **Why Shared State vs Message-Only?**
- **Efficiency:** Don't serialize large data structures in every message
- **Structure:** Type-safe state with TypedDict
- **Access:** All agents can read environment, configs, etc. without parsing messages

---

## Comparison: Web UI "Run Planner" vs Agent "Solve"

### Web UI "Run Planner" Button
**Endpoint:** `/solve-with-allocation`
**Flow:**
```
User clicks button
    ↓
solve_mission_with_allocation()
    ↓
1. Calculate SAM-aware distances (if enabled)
2. allocate_targets(strategy="efficient")  ← Automatic
3. Solve orienteering per drone
4. Post-optimize (if enabled)
    ↓
Return routes + trajectories
```

**Allocation:** Always uses `"efficient"` strategy automatically

---

### Agent "Solve" Command
**Flow:**
```
User: "Solve this mission"
    ↓
COORDINATOR decides what to do
    ↓
May or may not call allocate_targets_to_drones()  ← LLM decides
    ↓
If allocation skipped, might solve with ALL targets per drone
    ↓
Results may differ!
```

**Allocation:** Depends on what the Coordinator LLM decides to do

---

## Why Results Differ

1. **Allocation Strategy:**
   - Web UI: Always `"efficient"`
   - Agent: LLM might choose `"balanced"`, `"greedy"`, etc., or skip allocation entirely

2. **Workflow:**
   - Web UI: Deterministic pipeline
   - Agent: LLM-driven, may skip steps or use different order

3. **Post-Optimization:**
   - Web UI: Currently `post_optimize=False` hardcoded ([main.py:746](../server/main.py#L746))
   - Agent: LLM may or may not call optimization tools

4. **Target Filtering:**
   - Web UI: Automatic filtering by drone capabilities
   - Agent: Depends on whether Allocator was called

---

## To Get Consistent Results

**Option 1:** Force agent workflow to match Web UI:
```
User: "Use these exact steps:
1. Call allocate_targets_to_drones with strategy='efficient'
2. Call solve_all_drones_parallel
3. Call optimize_assign_unvisited
4. Call optimize_reassign_targets
5. Call optimize_remove_crossings"
```

**Option 2:** Update Web UI to match what agent does (let LLM decide strategy)

**Option 3:** Add agent memory instructing it to always use efficient allocation:
```python
add_memory(
    category="instruction",
    content="ALWAYS use allocate_targets_to_drones(strategy='efficient') before solving"
)
```

---

## Agent System Prompts

Each agent receives a specialized system prompt that defines its role, constraints, and available tools. All prompts include a shared **HARD CONSTRAINTS** section that must NEVER be violated.

### Shared Hard Constraints

All agents receive these inviolable constraints:

```
================================================================================
HARD CONSTRAINTS - NEVER VIOLATE
================================================================================
1. FUEL BUDGET: Route MUST NOT exceed drone's fuel capacity.
2. SAM/NFZ BOUNDARIES: Routes MUST avoid all SAM zones.
3. MISSION CONSTRAINTS:
   a) Target Type Accessibility: Respect target_access restrictions
   b) Priority Constraints: Respect priority-based filters
   c) Route Order Constraints: Respect start/end requirements
================================================================================
```

**Location:** [isr_agent_multi_v2.py:1213-1224](../server/agents/isr_agent_multi_v2.py#L1213-L1224)

---

### 1. Coordinator Agent Prompt

**Model:** Claude Sonnet 4 (`claude-sonnet-4-20250514`)
**Temperature:** 0
**Max Tokens:** 4096

```
You are the COORDINATOR agent in a multi-agent ISR mission planning system.

[HARD CONSTRAINTS SECTION]

Your role is to:
1. Understand the user's mission request
2. Get a mission overview first
3. Decompose the task and route to specialist agents:
   - ALLOCATOR: For target allocation (call route_to_allocator with priority_constraints)
   - ROUTER: For computing optimal routes (call route_to_router with drone_ids)
   - VALIDATOR: For validating routes (call route_to_validator with routes)
   - OPTIMIZER: For post-optimization (call route_to_optimizer with routes and type)

IMPORTANT: After each specialist completes, you must continue the workflow by calling the next routing tool.

WORKFLOW - Follow these steps IN ORDER:
1. Call get_mission_overview to understand the mission
2. Call route_to_allocator (system will pass control to ALLOCATOR)
3. When you see allocation results, call route_to_router (system will pass control to ROUTER)
4. When you see ROUTE_D results, you are DONE - just repeat the routes in your response
5. (Optional) If validation or optimization is requested, route to those agents

CRITICAL: You MUST call route_to_router after seeing allocation results!
After ROUTER returns routes, just output those routes - no further routing needed.

OUTPUT FORMAT (copy from ROUTER results):
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
```

**Key Behaviors:**
- Must call `get_mission_overview()` first
- Routes to ALLOCATOR, then ROUTER in sequence
- Terminates after receiving ROUTE_D output
- Can optionally route to VALIDATOR or OPTIMIZER if requested

**Location:** [isr_agent_multi_v2.py:1226-1254](../server/agents/isr_agent_multi_v2.py#L1226-L1254)

---

### 2. Allocator Agent Prompt

**Model:** Claude Sonnet 4 (same LLM instance)
**Temperature:** 0

```
You are the ALLOCATOR agent specializing in target allocation.

[HARD CONSTRAINTS SECTION]

Your role is to:
1. Allocate targets to drones optimally
2. Respect type restrictions and priority constraints
3. Use the appropriate allocation strategy

TOOLS:
- allocate_targets_to_drones: For standard allocation
- allocate_with_priority_constraints: When priority constraints are specified

After allocation, output the allocation results and return control.
```

**Key Behaviors:**
- Chooses appropriate allocation strategy (defaults to "efficient")
- Respects priority constraints if provided
- Returns allocation summary and hands back to Coordinator
- Does NOT call routing tools (Coordinator handles that)

**Location:** [isr_agent_multi_v2.py:1256-1270](../server/agents/isr_agent_multi_v2.py#L1256-L1270)

---

### 3. Router Agent Prompt

**Model:** Claude Sonnet 4 (same LLM instance)
**Temperature:** 0

```
You are the ROUTER agent specializing in optimal route computation.

[HARD CONSTRAINTS SECTION]

Your role is to:
1. Compute optimal routes for drones
2. Use Held-Karp algorithm for globally optimal solutions
3. Respect fuel budgets and airport constraints

TOOLS:
- solve_optimal_route: For single drone (all targets)
- solve_allocated_route: For single drone (allocated targets only)
- solve_all_drones_parallel: For all drones at once (FASTEST)

After routing, output ROUTE_Dx format and return control.

OUTPUT FORMAT:
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
```

**Key Behaviors:**
- Prefers `solve_all_drones_parallel()` for speed
- Uses allocated targets from Allocator agent
- Outputs routes in ROUTE_Dx format
- Returns control to Coordinator after completion

**Location:** [isr_agent_multi_v2.py:1272-1291](../server/agents/isr_agent_multi_v2.py#L1272-L1291)

---

### 4. Validator Agent Prompt

**Model:** Claude Sonnet 4 (same LLM instance)
**Temperature:** 0

```
You are the VALIDATOR agent specializing in constraint validation.

[HARD CONSTRAINTS SECTION]

Your role is to:
1. Validate each drone's route against its constraints
2. Check for target conflicts (same target assigned to multiple drones)
3. Provide mission summary statistics

TOOLS:
- validate_drone_route: Validate one drone's route
- check_target_conflicts: Check for duplicate assignments
- get_mission_summary: Get overall mission statistics

Report any violations found. If routes are valid, confirm validation passed.
```

**Key Behaviors:**
- Checks fuel budget compliance
- Detects duplicate target assignments
- Validates airport/type constraints
- Reports violations or confirms validity

**Location:** [isr_agent_multi_v2.py:1293-1308](../server/agents/isr_agent_multi_v2.py#L1293-L1308)

---

### 5. Optimizer Agent Prompt

**Model:** Claude Sonnet 4 (same LLM instance)
**Temperature:** 0

```
You are the OPTIMIZER agent specializing in post-optimization.

[HARD CONSTRAINTS SECTION]

Your role is to:
1. Optimize existing routes without violating constraints
2. Insert unvisited targets where possible
3. Swap targets to closer trajectories
4. Remove route crossings

TOOLS:
- optimize_assign_unvisited: Insert missed targets (respects constraints)
- optimize_reassign_targets: Swap to closer trajectories
- optimize_remove_crossings: 2-opt crossing removal

CRITICAL: Always pass priority_constraints to optimization tools if the mission
has priority restrictions. This ensures optimization respects constraints.

After optimization, output updated ROUTE_Dx format.

OUTPUT FORMAT:
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
```

**Key Behaviors:**
- Runs Insert Missed, Swap Closer, Crossing Remove
- MUST pass priority_constraints to maintain constraint compliance
- Outputs optimized routes in ROUTE_Dx format
- May run iteratively if multiple optimization passes requested

**Location:** [isr_agent_multi_v2.py:1310-1333](../server/agents/isr_agent_multi_v2.py#L1310-L1333)

---

## Memory System Integration

All agent prompts are augmented with **persistent memories** before being sent to the LLM:

```python
def coordinator_node(state: MultiAgentState) -> Dict[str, Any]:
    """Coordinator agent node."""
    messages = state.get("messages", [])
    memories = format_memories_for_prompt()  # Load from agent_memory.json
    system = SystemMessage(content=COORDINATOR_PROMPT + memories)
    # ... rest of node logic
```

**Memory Categories:**
- `instruction` - Persistent instructions (e.g., "Always use efficient allocation")
- `note` - Important facts to remember
- `preference` - User preferences
- `warning` - Things to avoid

**Memory Format:**
```
================================================================================
IMPORTANT MEMORIES & INSTRUCTIONS:
================================================================================
[INSTRUCTION] Always use allocate_targets_to_drones(strategy='efficient') before solving
[NOTE] User prefers balanced allocation for missions with >20 targets
[WARNING] Never use greedy allocation - causes uneven workload
================================================================================
```

**Memory File:** [agent_memory.json](../agent_memory.json) (project root)

**Management Functions:**
- `load_memory()` - Load all memories from file
- `get_active_memories()` - Get only active (not deleted) memories
- `format_memories_for_prompt()` - Format memories as text for system prompt

**Location:** [isr_agent_multi_v2.py:69-109](../server/agents/isr_agent_multi_v2.py#L69-L109)

---

## LLM Configuration

All agents currently use the **same LLM instance** with tool binding:

```python
# Create base LLM
llm = ChatAnthropic(
    model="claude-sonnet-4-20250514",
    temperature=0,
    max_tokens=4096,
)

# Bind specialized tools to each agent
coordinator_llm = llm.bind_tools(COORDINATOR_TOOLS)
allocator_llm = llm.bind_tools(ALLOCATOR_TOOLS)
router_llm = llm.bind_tools(ROUTER_TOOLS)
validator_llm = llm.bind_tools(VALIDATOR_TOOLS)
optimizer_llm = llm.bind_tools(OPTIMIZER_TOOLS)
```

**Model:** `claude-sonnet-4-20250514` (Anthropic Claude Sonnet 4)
**Temperature:** 0 (deterministic)
**Max Tokens:** 4096

**Tool Binding:**
Each agent gets a different subset of tools bound to its LLM instance, ensuring it can only call tools appropriate to its role.

**Location:** [isr_agent_multi_v2.py:1344-1355](../server/agents/isr_agent_multi_v2.py#L1344-L1355)

---

## Prompt Augmentation Example

When the Coordinator agent is invoked, its final system message becomes:

```
You are the COORDINATOR agent in a multi-agent ISR mission planning system.

================================================================================
HARD CONSTRAINTS - NEVER VIOLATE
================================================================================
1. FUEL BUDGET: Route MUST NOT exceed drone's fuel capacity.
2. SAM/NFZ BOUNDARIES: Routes MUST avoid all SAM zones.
3. MISSION CONSTRAINTS:
   a) Target Type Accessibility: Respect target_access restrictions
   b) Priority Constraints: Respect priority-based filters
   c) Route Order Constraints: Respect start/end requirements
================================================================================

Your role is to:
1. Understand the user's mission request
2. Get a mission overview first
3. Decompose the task and route to specialist agents:
   ...

================================================================================
IMPORTANT MEMORIES & INSTRUCTIONS:
================================================================================
[INSTRUCTION] Always use allocate_targets_to_drones(strategy='efficient')
[NOTE] Last mission had 29 targets across 5 drones
================================================================================
```

This ensures the agent has:
1. Its core role definition
2. Hard constraints it must never violate
3. Persistent memories from previous interactions
4. Access to its specialized tools

---

## Files Reference

| Component | File | Lines |
|-----------|------|-------|
| Multi-Agent System | [isr_agent_multi_v2.py](../server/agents/isr_agent_multi_v2.py) | 1-1613 |
| State Definition | [isr_agent_multi_v2.py](../server/agents/isr_agent_multi_v2.py) | 206-224 |
| Coordinator Agent | [isr_agent_multi_v2.py](../server/agents/isr_agent_multi_v2.py) | 1206-1305 |
| Workflow Graph | [isr_agent_multi_v2.py](../server/agents/isr_agent_multi_v2.py) | 1467-1610 |
| Web Solver | [solver_bridge.py](../server/solver/solver_bridge.py) | 477-829 |
| Web Endpoint | [main.py](../server/main.py) | 731-751 |
| Target Allocator | [target_allocator.py](../server/solver/target_allocator.py) | - |
| Post-Optimizer | [post_optimizer.py](../server/solver/post_optimizer.py) | 1-1379 |

---

## Document Information

**Generated:** December 13, 2025
**Architecture Version:** Multi-Agent v2
**Framework:** LangGraph + LangChain + Anthropic Claude
