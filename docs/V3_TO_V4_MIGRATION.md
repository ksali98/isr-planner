# v3 to v4 Migration Guide

## Overview
This document tracks what v3 features/improvements need to be ported to v4 to make v4 production-ready with the Mission Planner agent.

---

## Key Differences Between v3 and v4

### Architecture
- **v3**: Task-based agents with **LangChain @tool decorators** and ToolNode
- **v4**: Reasoning-based agents with **direct LLM reasoning** (no tools exposed)

### Agent Flow
```
v3: Coordinator → Allocator (tools) → Router (tools) → Validator → Optimizer (tools)
v4: Strategist → Mission Planner → Allocator → Route_Optimizer → Critic → Responder
```

---

## Critical v3 Features to Port to v4

### ✅ **1. Coordinator v4 Integration** 
**Status**: ✅ Already in v4  
**Location**: `coordinator_v4.py` pre-pass  
**What it does**: Intent detection, policy rules, guardrails, synthetic starts

---

### ⚠️ **2. Optimizer Tools** 
**Status**: ❌ NOT in v4 - NEEDS PORTING  
**Location in v3**: Lines 1769-2000+ with `@tool` decorators  

**v3 has these optimizer tools**:
1. `insert_unvisited()` - Insert missed targets into routes
2. `swap_closer()` - Reassign targets to closer drones  
3. `remove_crossings()` - Apply 2-opt to eliminate route crossings
4. *(Potentially more)*

**Problem**: v4 doesn't expose these as callable tools. The Critic agent can only suggest improvements but can't execute them.

**Solution Options**:

#### **Option A: Add Optimizer Agent After Route_Optimizer**
```python
workflow.add_node("optimizer", optimizer_node)
workflow.add_edge("route_optimizer", "optimizer")
workflow.add_edge("optimizer", "critic")
```

The Optimizer agent would have access to:
- `insert_unvisited_tool()`
- `swap_closer_tool()`
- `remove_crossings_tool()`

#### **Option B: Give Route_Optimizer Agent the Optimization Tools**
Route_Optimizer could automatically run post-optimization after solving routes.

#### **Option C: Mission Planner Orchestrates Optimization**
Mission Planner decides if optimization is needed and calls orchestration tools.

**Recommendation**: **Option A** - Keep optimization as a separate specialized agent, maintains v4's reasoning chain.

---

### ⚠️ **3. Mandatory Tool Calls**
**Status**: ❌ NOT in v4  
**Location in v3**: `tool_choice="required"` in LLM calls

**v3 Pattern**:
```python
llm_with_tools = llm.bind_tools(allocation_tools, tool_choice="required")
response = llm_with_tools.invoke(messages)
```

**v4 Pattern**:
```python
# No tools bound - pure reasoning
response = llm.invoke(messages)
```

**Problem**: v4 agents might not call necessary functions (allocation, routing, etc.)

**Current v4 Approach**: Agents reason, then code directly calls solver functions:
- Allocator reasons → then calls `_allocate_targets_impl()`
- Route_Optimizer reasons → then calls HK solver

**Evaluation**: This is actually **safer** than v3's approach - reasoning first, execution second. No porting needed.

---

### ⚠️ **4. Frozen Segments Support**
**Status**: ✅ Partially in v4, ❌ Not tested  
**Location in v3**: `frozen_segments` field passed to optimizers

**v3 Code**:
```python
frozen_segments = route_data.get("frozen_segments", [])
solution["routes"][did] = {
    "route": route,
    "frozen_segments": frozen_segments,  # Preserved in optimization
}
```

**v4 Status**: 
- MissionState has `segments` field (line ~168)
- Mission Planner can create segments
- BUT: No code actually passes `frozen_segments` to solver/optimizer

**Action Required**: 
1. Ensure `frozen_segments` is extracted from route data
2. Pass to solver in Route_Optimizer
3. Pass to post-optimizers (insert/swap/2-opt)

---

### ⚠️ **5. Distance Matrix Caching**
**Status**: ✅ Already in v4  
**Location**: Lines 1900-2000+ in `run_multi_agent_v4()`

v4 has better matrix handling than v3:
- Accepts pre-computed `sam_matrix` from main.py
- Stores matrix metadata (env_hash, routing_model_hash, cache_hit)
- Decision Trace includes matrix provenance

**No porting needed** - v4 is superior here.

---

### ⚠️ **6. Retry Logic**
**Status**: ❌ NOT in v4  
**Location in v3**: `after_allocator()`, `after_router()` with retry counters

**v3 Pattern**:
```python
def after_allocator(state: MissionState) -> str:
    if not state.get("allocation"):
        retry = state.get("allocator_retry_count", 0) + 1
        if retry < 3:
            state["allocator_retry_count"] = retry
            return "allocator"  # Retry
    return "router"
```

**v4 Pattern**: No retry logic - if agent fails, workflow fails.

**Recommendation**: Add retry logic to v4 routing functions with max 2-3 retries per agent.

---

### ⚠️ **7. Constraint Parser Integration**
**Status**: ✅ Partially in v4  
**Location in v4**: Line ~2080 - `parse_constraints()` called

**v4 Code**:
```python
constraint_parse_result = parse_constraints(
    user_message=user_message,
    environment=env,
    drone_configs=normalized_configs,
)
sequencing_hints = constraint_parse_result.program.sequencing_hints
```

**What's Missing**: 
- Mission Planner doesn't USE these hints yet
- No tools to apply constraint operations

**Action Required**:
Mission Planner should:
1. Read `sequencing_hints` from state
2. Call orchestration tools to apply them:
   - `add_fuel_reserve_constraint()`
   - `add_loiter_constraint()`
   - `create_segment_at_checkpoint()`

---

### ⚠️ **8. Parallel Route Solving**
**Status**: ❌ NOT in v4  
**Location in v3**: Lines 1100-1200 - `ThreadPoolExecutor` for concurrent drone solving

**v3 Code**:
```python
with ThreadPoolExecutor(max_workers=min(8, len(drone_ids))) as executor:
    futures = {executor.submit(solve_single_drone, did, ...): did for did in drone_ids}
    for future in as_completed(futures):
        ...
```

**v4 Status**: Routes solved sequentially (slower for many drones)

**Recommendation**: Port ThreadPoolExecutor pattern to v4's Route_Optimizer agent for production performance.

---

### ⚠️ **9. Orienteering Solver Interface**
**Status**: ✅ Already in v4  
**Location**: Lines 60-70 import, used in Route_Optimizer

Both v3 and v4 use `OrienteeringSolverInterface` - no porting needed.

---

### ⚠️ **10. Mission Metrics Computation**
**Status**: ✅ Already in v4  
**Location**: Lines 340-420 `compute_mission_metrics()`

v4 has comprehensive metrics:
- Per-drone fuel/margins/points
- Total fuel/points
- Visited/unvisited targets

**No porting needed** - v4 has this.

---

### ⚠️ **11. Trajectory Planning**
**Status**: ✅ Already in v4  
**Location**: Lines 2200-2300+ in `run_multi_agent_v4()`

v4 includes full SAM-aware trajectory generation with `ISRTrajectoryPlanner`.

**No porting needed**.

---

### ⚠️ **12. Decision Trace v1**
**Status**: ✅ Already in v4 (BETTER than v3)  
**Location**: Lines 2250-2350+ comprehensive trace structure

v4 has superior tracing:
- Coordinator pre-pass decisions
- Matrix metadata (env_hash, cache_hit)
- Policy rules applied
- Per-drone evidence

**No porting needed** - v4 is superior.

---

## Priority Migration Tasks

### 🔴 **HIGH PRIORITY (Blocking v4 Production)**

1. **Add Optimizer Agent with Tools** ⏱️ 2-3 hours
   - Port `insert_unvisited()` tool
   - Port `swap_closer()` tool  
   - Port `remove_crossings()` tool
   - Add Optimizer node to workflow
   - Update routing after Route_Optimizer

2. **Implement Frozen Segments in Solver** ⏱️ 1-2 hours
   - Extract `frozen_segments` from routes
   - Pass to HK solver
   - Pass to post-optimizers
   - Test with checkpoint replanning

3. **Add Retry Logic** ⏱️ 1 hour
   - Retry counters in state
   - Conditional routing with retry checks
   - Max 2-3 retries per agent

### 🟡 **MEDIUM PRIORITY (Nice to Have)**

4. **Mission Planner Tool Integration** ⏱️ 2-3 hours
   - Read sequencing hints
   - Call orchestration tools
   - Apply constraint operations
   - Test multi-segment missions

5. **Parallel Route Solving** ⏱️ 1-2 hours
   - ThreadPoolExecutor in Route_Optimizer
   - Concurrent drone solving
   - Performance testing

### 🟢 **LOW PRIORITY (Future Enhancement)**

6. **Better Error Messages** ⏱️ 1 hour
   - Agent-specific error context
   - Retry attempt logging
   - User-friendly failure messages

---

## Testing Checklist

After migration, test these scenarios:

- [ ] Simple single-phase mission (all drones, all targets)
- [ ] Multi-segment mission (checkpoint replanning)
- [ ] Drone losses mid-mission
- [ ] Fuel reserve constraints (land with 25% fuel)
- [ ] Loiter constraints (20 steps at target type C)
- [ ] Skip every other target (trajectory filtering)
- [ ] Optimizer tools (insert/swap/2-opt)
- [ ] Frozen segments preservation
- [ ] Sequencing hints (start_with, end_with)
- [ ] Questions about existing solution (no re-optimization)

---

## Implementation Plan

### Phase 1: Core Functionality (Day 1)
1. Add Optimizer agent with 3 tools
2. Implement frozen_segments support
3. Add basic retry logic

### Phase 2: Orchestration (Day 2)
4. Mission Planner tool integration
5. Test multi-segment missions
6. Constraint operations

### Phase 3: Performance (Day 3)
7. Parallel route solving
8. Performance benchmarking
9. Production testing

---

## Code Snippets for Migration

### Adding Optimizer Agent

```python
# In isr_agent_multi_v4.py

@tool
def insert_unvisited_tool() -> str:
    """Insert unvisited targets into routes where fuel allows."""
    state = get_state()
    # ... port v3 logic ...
    return result_message

@tool  
def swap_closer_tool() -> str:
    """Reassign targets to closer drones for better efficiency."""
    state = get_state()
    # ... port v3 logic ...
    return result_message

@tool
def remove_crossings_tool() -> str:
    """Apply 2-opt to eliminate route crossings."""
    state = get_state()
    # ... port v3 logic ...
    return result_message

OPTIMIZER_PROMPT = """You are the OPTIMIZER agent.

You have THREE optimization tools:
1. insert_unvisited_tool() - Add missed targets
2. swap_closer_tool() - Reassign to closer drones
3. remove_crossings_tool() - Eliminate crossings

Analyze the current solution and decide which optimizations to apply.
"""

def optimizer_node(state: MissionState) -> Dict[str, Any]:
    """Optimizer applies post-optimization tools."""
    print("\n⚙️  [OPTIMIZER] Analyzing optimization opportunities...", file=sys.stderr)
    
    llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)
    optimizer_tools = [insert_unvisited_tool, swap_closer_tool, remove_crossings_tool]
    llm_with_tools = llm.bind_tools(optimizer_tools)
    
    # ... reasoning and tool execution ...
    
    return {
        "messages": [AIMessage(content=result)],
        "optimizer_analysis": result,
    }

# In build_reasoning_workflow()
workflow.add_node("optimizer", optimizer_node)
workflow.add_edge("route_optimizer", "optimizer")
workflow.add_edge("optimizer", "critic")
```

### Adding Frozen Segments Support

```python
# In route_optimizer_node()

for did in drone_ids:
    route_data = existing_routes.get(did, {})
    frozen_segments = route_data.get("frozen_segments", [])
    
    result = solve_route_with_hk(
        drone_id=did,
        targets=assigned_targets,
        start=start_id,
        end=end_id,
        fuel_budget=fuel_budget,
        frozen_segments=frozen_segments,  # Pass through
    )
```

### Adding Retry Logic

```python
# In MissionState
class MissionState(TypedDict):
    # ... existing fields ...
    allocator_retry_count: Optional[int]
    router_retry_count: Optional[int]
    optimizer_retry_count: Optional[int]

# In build_reasoning_workflow()
def after_allocator(state: MissionState) -> str:
    allocation = state.get("allocation")
    if not allocation or len(allocation) == 0:
        retry = state.get("allocator_retry_count", 0)
        if retry < 2:
            state["allocator_retry_count"] = retry + 1
            print(f"⚠️  [ALLOCATOR] Retry {retry + 1}/2", file=sys.stderr)
            return "allocator"
        else:
            print(f"❌ [ALLOCATOR] Failed after {retry + 1} attempts", file=sys.stderr)
            return "responder"  # Give up, explain failure
    return "route_optimizer"

workflow.add_conditional_edges("allocator", after_allocator, {
    "allocator": "allocator",
    "route_optimizer": "route_optimizer",
    "responder": "responder",
})
```

---

## Conclusion

**v4 with Mission Planner is the right architecture**, but it needs these v3 features ported:

**Critical**:
1. Optimizer agent with tools
2. Frozen segments support
3. Retry logic

**Important**:
4. Mission Planner tool integration
5. Parallel solving

Once these are ported, v4 will be production-ready and significantly more capable than v3 for complex mission scenarios.

**Estimated Total Time**: 6-8 hours of focused development + 2-3 hours testing
