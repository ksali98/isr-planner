# Drone Configuration Flow Analysis: UI → Run Planner → Agent Solve v4

## Executive Summary

This document traces how drone configurations (fuel_budget, start_airport, end_airport, target_access) flow from the UI through the Run Planner endpoint and Agent Solve v4 endpoint, documenting:
- Where configs are stored in UI state
- How they're transmitted to backend endpoints
- How they're processed and used in routing decisions
- Critical default behaviors and overrides

---

## 1. UI STATE STORAGE (webapp/isr.js)

### 1.1 DroneConfigs Object Location
**File**: `/Users/kamalali/isr-planner/webapp/isr.js`  
**Line**: 37

```javascript
droneConfigs: {},          // { "1": {enabled, fuel_budget, start_airport, end_airport, target_access}, ... }
```

### 1.2 Config Structure
**Lines**: 1075-1081 (initDroneConfigsFromEnv)

Each drone config dict contains:
```javascript
state.droneConfigs[idStr] = {
  enabled: boolean,                     // Is drone active
  fuel_budget: float,                   // Max fuel allowed (default 150)
  start_airport: string,                // Start location (e.g., "A1")
  end_airport: string,                  // End location (e.g., "A1" or "-" for flexible)
  target_access: {a, b, c, d, e},     // Target type accessibility flags
};
```

### 1.3 Initialization from Environment
**Function**: `initDroneConfigsFromEnv()`  
**Lines**: 1026-1105

When environment is loaded:
- **Saved configs**: Read from `state.env.drone_configs` (persisted from previous exports)
- **Defaults**:
  - `fuel_budget`: 150 (or from saved)
  - `start_airport`: `A{did}` if exists, else first airport (line 1071)
  - `end_airport`: Same as start (line 1072)
  - `target_access`: All types enabled (line 1073)

**Critical Line** (1010): start_airport sync to config
```javascript
state.droneConfigs[String(did)].start_airport = airportIds[0];
```

**Critical Line** (1020): end_airport sync to config
```javascript
state.droneConfigs[String(did)].end_airport = airportIds[0];
```

### 1.4 Config Change Listeners
**Function**: `attachConfigListeners()`  
**Lines**: 1107-1156

Changes trigger mission invalidation:
- **Line 1119**: Enabled toggle → `invalidateMission('Drone X enabled changed')`
- **Line 1126**: Fuel budget change → `state.droneConfigs[idStr].fuel_budget = parseFloat(...)`
- **Line 1135**: start_airport change → stored directly
- **Line 1142**: end_airport change → stored directly
- **Line 1151**: target_access checkbox → stored directly

### 1.5 Airport Dropdown Updates
**Function**: `updateAirportDropdowns()`  
**Lines**: 964-1024

When airports are added/deleted:
- **Lines 987-1002**: End dropdown gets **flexible "-" option** (="Any")
- **Lines 1010-1020**: Auto-sync configs to new first airport if previous is deleted

---

## 2. RUN PLANNER ENDPOINT (server/main.py)

### 2.1 API Endpoint Definition
**Function**: `runPlanner()` / `solve()` / `solve_with_allocation()`  
**Lines**: 640-801

Three main solving endpoints:
1. **`POST /api/solve`** (Line 640): Basic orienteering solver
2. **`POST /api/solve_with_allocation`** (Line 729): Recommended endpoint
3. **`POST /api/agents/chat-v4`** (Line 1019): Agent-based solver

### 2.2 Frontend Call to Run Planner
**Function**: `runPlanner()`  
**Lines**: 1711-1813 (webapp/isr.js)

Payload sent to `/api/solve_with_allocation`:
```javascript
const payload = {
  env: state.env,
  drone_configs: state.droneConfigs,      // ALL drone configs from UI state
  allocation_strategy: strategy,           // User-selected strategy
};
```

**Critical**: Line 1740 - drone_configs passed AS-IS from UI state

### 2.3 Backend Receives Configs
**Function**: `solve_with_allocation()`  
**File**: `/Users/kamalali/isr-planner/server/solver/solver_bridge.py`  
**Lines**: 477-510

```python
def solve_with_allocation(
    env: Dict[str, Any],
    drone_configs: Dict[str, Any],      # From request
    allocation_strategy: str = "efficient",
    use_sam_aware_distances: bool = False,
    post_optimize: bool = True,
) -> Dict[str, Any]:
```

### 2.4 How Solver Bridge Uses Configs
**Lines**: 584-596 (solver_bridge.py)

For EACH drone:
```python
fuel_budget = float(cfg.get("fuel_budget", 999.0))

# Start airport extraction (line 589)
default_start = f"A{did_int}"
start_id = cfg.get("start_airport") or (
    default_start if default_start in airport_by_id else airports[0]["id"]
)

# End airport extraction (line 594) WITH FLEXIBLE ENDPOINT SUPPORT
raw_end_id = cfg.get("end_airport") or start_id
flexible_endpoint = (raw_end_id == "-")           # <-- FLEXIBLE LOGIC
end_id = start_id if flexible_endpoint else raw_end_id
```

### 2.5 Mode Default Logic
**Line**: 427 (solver_bridge.py)

```python
env_for_solver["mode"] = "return" if end_id == start_id else "open"
```

**This ALWAYS defaults to "return" mode when start==end**

---

## 3. AGENT SOLVE V4 ENDPOINT (server/agents/isr_agent_multi_v4.py)

### 3.1 Endpoint Handler
**File**: `/Users/kamalali/isr-planner/server/main.py`  
**Lines**: 1018-1152

```python
@app.post("/api/agents/chat-v4", response_model=AgentChatResponse)
async def agent_chat_v4(req: AgentChatRequest):
```

Takes same config as input:
```python
drone_configs = req.drone_configs or env.get("drone_configs") or {}
```

### 3.2 Config Normalization in v4 Agent
**File**: `/Users/kamalali/isr-planner/server/agents/isr_agent_multi_v4.py`  
**Lines**: 1277-1334 (run_multi_agent_v4)

CRITICAL NORMALIZATION LAYER:

```python
# Lines 1300-1307: Fuel budget normalization
fuel_budget = (
    cfg.get("fuelBudget")
    if cfg.get("fuelBudget") is not None
    else cfg.get("fuel_budget")
)
if fuel_budget is not None:
    nc["fuelBudget"] = fuel_budget

# Lines 1310-1317: Home airport mapping
home_ap = (
    cfg.get("homeAirport")
    or cfg.get("home_airport")
    or cfg.get("start_airport")    # <-- MAPS start_airport → home_airport
    or cfg.get("startAirport")
)
if home_ap:
    nc["home_airport"] = home_ap

# Lines 1319-1324: End airport preservation
end_ap = (
    cfg.get("endAirport")
    or cfg.get("end_airport")
)
if end_ap:
    nc["end_airport"] = end_ap

# Lines 1327-1331: Target access normalization
ta = cfg.get("target_access")
if isinstance(ta, dict):
    accessible = [str(k).upper() for k, v in ta.items() if v]
    nc["accessible_targets"] = accessible
```

**Key Insight**: v4 agent treats `home_airport` as canonical, but also preserves `end_airport` from UI

### 3.3 Route Optimizer Node in v4
**Function**: `route_optimizer_node()`  
**Lines**: 825-973 (isr_agent_multi_v4.py)

Routes per drone:
```python
# Lines 851-853
fuel_budget = cfg.get("fuelBudget", cfg.get("fuel_budget", 200))
home_airport = cfg.get("homeAirport", cfg.get("home_airport", "A1"))
end_airport = cfg.get("endAirport", cfg.get("end_airport", home_airport))

# LINE 854 - MODE DEFAULT
mode = "return" if end_airport == home_airport else "open"

# LINE 904 - DEBUG OUTPUT OF CONFIG
print(f"[v4][ROUTE_OPT] D{did} home={home_airport} end={end_airport} mode={mode} cfg_end={cfg.get('end_airport')} cfg_keys={list(cfg.keys())}", flush=True)
```

### 3.4 Solver Environment Construction
**Lines**: 906-914 (isr_agent_multi_v4.py)

```python
solver_env = {
    "airports": env.get("airports", []),
    "targets": targets,
    "matrix": matrix,
    "matrix_labels": labels,
    "start_airport": home_airport,    # <-- FROM home_airport field
    "end_airport": end_airport,        # <-- FROM end_airport field
    "mode": mode,                      # <-- DEFAULT TO "return"
}
```

---

## 4. ALLOCATION AND CACHING

### 4.1 Distance Matrix Caching
**File**: `/Users/kamalali/isr-planner/server/solver/solver_bridge.py`

**Global Cache Variables** (Lines 62-63):
```python
_cached_env_hash: Optional[str] = None  # Environment hash for staleness detection
```

**Cache Usage** (Lines 516-534):
```python
if use_sam_aware_distances and sams:
    global _cached_env_hash
    current_hash = _compute_env_hash(env)
    cached_matrix = get_cached_matrix()

    # Only use cache if it matches the current environment
    if cached_matrix is not None and _cached_env_hash == current_hash:
        print("⚡ Using cached SAM-aware distance matrix (env hash match)", flush=True)
        dist_data = cached_matrix
    else:
        # Cache miss or env changed - recalculate
        if cached_matrix is not None:
            print(f"🔄 Environment changed (hash mismatch), recalculating distance matrix...", flush=True)
            clear_matrix_cache()
        else:
            print("⏳ Calculating SAM-aware distance matrix (no cache available)...", flush=True)
        dist_data = calculate_sam_aware_matrix(env)
        _cached_env_hash = current_hash  # Update hash after calculation
```

**Cache Implementation** (server/solver/sam_distance_matrix.py):
- Global `_calculator` instance maintains `_cached_matrix`
- Hash-based invalidation on environment changes
- Cleared via `clear_cached_matrix()` (Line 853, solver_bridge.py)

### 4.2 Allocation Caching in Mission Store
**File**: `/Users/kamalali/isr-planner/server/main.py`  
**Lines**: 132-133 (Global Store)

```python
MissionRecord = Dict[str, Any]
MISSION_STORE: Dict[str, MissionRecord] = {}  # In-memory per-process
```

**Mission Persistence** (Lines 1095-1110):
```python
if has_solution:
    if not mission_id:
        mission_id = f"m_{uuid4().hex}"
        print(f"[v4] Creating new mission {mission_id}", flush=True)
    else:
        print(f"[v4] Updating mission {mission_id}", flush=True)

    MISSION_STORE[mission_id] = {
        "env": env,
        "drone_configs": drone_configs,
        "routes": raw_routes,
        "allocation": allocations,           # <-- Cached allocation
        "total_points": result.get("total_points"),
        "total_fuel": result.get("total_fuel"),
        "updated_at": datetime.utcnow().isoformat(),
    }
```

**Retrieval** (Lines 1039-1050):
```python
if mission_id and mission_id in MISSION_STORE:
    mission = MISSION_STORE[mission_id]
    # For consistency, prefer stored env/configs over incoming ones
    env = mission.get("env", env)
    drone_configs = mission.get("drone_configs", drone_configs)

    existing_solution = {
        "routes": mission.get("routes") or {},
        "allocation": mission.get("allocation") or {},
    }
```

---

## 5. START_AIRPORT OVERRIDES (ALL LOCATIONS)

### 5.1 UI Level Overrides
1. **Line 1010** (webapp/isr.js): Auto-sync when airport deleted
   ```javascript
   state.droneConfigs[String(did)].start_airport = airportIds[0];
   ```

### 5.2 Solver Bridge Overrides
2. **Line 589** (solver_bridge.py): Default to `A{drone_id}` or first airport
   ```python
   default_start = f"A{did_int}"
   start_id = cfg.get("start_airport") or (
       default_start if default_start in airport_by_id else airports[0]["id"]
   )
   ```

### 5.3 v4 Agent Overrides
3. **Lines 1310-1317** (isr_agent_multi_v4.py): Maps `start_airport` → `home_airport`
   ```python
   home_ap = (
       cfg.get("homeAirport")
       or cfg.get("home_airport")
       or cfg.get("start_airport")    # <-- OVERRIDE HERE
       or cfg.get("startAirport")
   )
   ```

### 5.4 Route Optimizer v4 Overrides
4. **Lines 851-852** (isr_agent_multi_v4.py): Uses `homeAirport` config field
   ```python
   home_airport = cfg.get("homeAirport", cfg.get("home_airport", "A1"))
   ```

---

## 6. END_AIRPORT IGNORES/DEFAULTS (ALL LOCATIONS)

### 6.1 UI Level Defaults
1. **Line 1019** (webapp/isr.js): Default to start_airport if deleted
   ```javascript
   } else if (airportIds.length > 0) {
       endSel.value = airportIds[0];
       state.droneConfigs[String(did)].end_airport = airportIds[0];
   }
   ```

### 6.2 Solver Bridge Defaults
2. **Line 594** (solver_bridge.py): Default to start_airport OR FLEXIBLE
   ```python
   raw_end_id = cfg.get("end_airport") or start_id
   flexible_endpoint = (raw_end_id == "-")
   end_id = start_id if flexible_endpoint else raw_end_id
   ```

3. **Flexible Endpoint Logic** (Lines 694-726): Tries ALL airports if flexible
   ```python
   if flexible_endpoint:
       print(f"   🔄 Flexible endpoint: trying all {len(airports)} airports...", flush=True)
       best_sol = None
       best_points = -1
       best_distance = float('inf')
       best_end_id = start_id

       for try_airport in airports:
           try_end_id = try_airport["id"]
           env_for_solver["end_airport"] = try_end_id
           try_sol = _solver.solve(env_for_solver, fuel_budget)
           # ... pick best result
   ```

### 6.3 v4 Agent Defaults
4. **Lines 1319-1324** (isr_agent_multi_v4.py): Preserves end_airport IF present
   ```python
   end_ap = (
       cfg.get("endAirport")
       or cfg.get("end_airport")
   )
   if end_ap:
       nc["end_airport"] = end_ap
   ```

5. **Route Optimizer v4** (Lines 852-853):
   ```python
   end_airport = cfg.get("endAirport", cfg.get("end_airport", home_airport))
   ```

---

## 7. MODE DEFAULT TO "return" (ALL LOCATIONS)

### 7.1 Solver Bridge
1. **Line 427** (solver_bridge.py):
   ```python
   env_for_solver["mode"] = "return" if end_id == start_id else "open"
   ```

### 7.2 v4 Route Optimizer
2. **Line 854** (isr_agent_multi_v4.py):
   ```python
   mode = "return" if end_airport == home_airport else "open"
   ```

### 7.3 Orienteering Solver Interface Default
3. **webapp/editor/solver/orienteering_interface.py** (referenced in import):
   ```python
   mode = env_data.get("mode", "return")  # Default to "return"
   ```

---

## 8. MISSION_ID, DISTANCE MATRIX, AND ALLOCATION CACHING

### 8.1 Mission ID Generation
**File**: `/Users/kamalali/isr-planner/server/main.py`  
**Lines**: 1096-1097

```python
if not mission_id:
    mission_id = f"m_{uuid4().hex}"
    print(f"[v4] Creating new mission {mission_id}", flush=True)
```

**Returned to UI** (Line 1124):
```python
mission_id=mission_id,
```

**Stored in UI State** (webapp/isr.js, Lines 2356, 2363-2365):
```javascript
mission_id: state.missionId,   // Send current mission_id back
// ... backend response ...
if (data.mission_id) {
    state.missionId = data.mission_id;  // Store new mission_id
    appendDebugLine("Current mission_id: " + state.missionId);
}
```

### 8.2 Distance Matrix Caching
**File**: `/Users/kamalali/isr-planner/server/solver/sam_distance_matrix.py`

**Global Calculator Instance**:
- Singleton pattern: `_calculator = SAMDistanceMatrixCalculator()`
- Stores `_cached_matrix: Optional[Dict[str, Any]]`
- Invalidated on environment hash mismatch (Line 518-529, solver_bridge.py)

**Cache Keys**:
- `_cached_matrix["labels"]` - waypoint IDs
- `_cached_matrix["matrix"]` - distance values
- `_cached_matrix["paths"]` - SAM-avoiding paths cache
- `_cached_matrix["excluded_targets"]` - targets inside SAMs

**Access Methods**:
1. **solver_bridge.py Line 519**: `get_cached_matrix()`
2. **main.py Line 494**: `get_current_matrix()` (for frontend status)
3. **Invalidation**: `clear_cached_matrix()` (Line 481, main.py)

### 8.3 Allocation Caching in Mission Store
**Storage Structure** (Lines 1102-1110, main.py):
```python
MISSION_STORE[mission_id] = {
    "env": env,                           # Full environment snapshot
    "drone_configs": drone_configs,       # Drone configs snapshot
    "routes": raw_routes,                 # Computed routes
    "allocation": allocations,            # Target allocations per drone
    "total_points": result.get("total_points"),
    "total_fuel": result.get("total_fuel"),
    "updated_at": datetime.utcnow().isoformat(),
}
```

**Retrieval for Follow-up Questions** (Lines 1039-1050):
```python
if mission_id and mission_id in MISSION_STORE:
    mission = MISSION_STORE[mission_id]
    env = mission.get("env", env)
    drone_configs = mission.get("drone_configs", drone_configs)
    
    existing_solution = {
        "routes": mission.get("routes") or {},
        "allocation": mission.get("allocation") or {},
    }
```

**Passed to Workflow** (Line 1063):
```python
existing_solution=existing_solution,
```

---

## 9. CRITICAL FLOW DIAGRAMS

### 9.1 Configuration Flow: UI → Solver
```
User modifies config in UI
    ↓
attachConfigListeners() captures change
    ↓
state.droneConfigs[did] updated
    ↓
invalidateMission(reason) clears state.missionId
    ↓
User clicks "Run Planner"
    ↓
runPlanner() sends POST /api/solve_with_allocation
    ├─ payload.drone_configs = state.droneConfigs (AS-IS)
    ├─ payload.env = state.env
    └─ payload.allocation_strategy = user-selected
    ↓
solve_with_allocation() in solver_bridge.py
    ├─ Extracts fuel_budget, start_airport, end_airport per drone
    ├─ Applies DEFAULTS if missing
    └─ Passes to OrienteeringSolverInterface
    ↓
Routes computed and returned to UI
    ↓
state.routes updated with sequence, points, distance, fuel_budget
```

### 9.2 Configuration Flow: Agent v4
```
User sends message to Agent
    ↓
sendAgentMessage() in UI
    ├─ payload.message = user message
    ├─ payload.env = state.env
    ├─ payload.drone_configs = state.droneConfigs
    ├─ payload.sequences = state.sequences (current)
    └─ payload.mission_id = state.missionId (or null)
    ↓
agent_chat_v4() endpoint in main.py
    ├─ If mission_id exists, load from MISSION_STORE
    └─ Call run_multi_agent_v4()
    ↓
run_multi_agent_v4() in isr_agent_multi_v4.py
    ├─ NORMALIZE configs:
    │  ├─ Map start_airport → home_airport
    │  ├─ Preserve end_airport
    │  └─ Convert target_access dict → accessible_targets list
    ├─ Build/compute distance matrix
    └─ Invoke workflow (strategist → allocator → route_optimizer → ...)
    ↓
route_optimizer_node()
    ├─ Reads home_airport, end_airport from normalized config
    ├─ Sets mode = "return" if home==end else "open"
    └─ Passes to solver
    ↓
Results cached in MISSION_STORE[mission_id]
    └─ Contains env, drone_configs, routes, allocations
```

### 9.3 End Airport Flexible Logic
```
cfg.end_airport value:
    ├─ "-" (flexible)
    │  ├─ Solver tries ALL airports
    │  └─ Picks endpoint with max points (or min distance if tied)
    ├─ Specific airport (e.g., "A1")
    │  └─ Solver uses that endpoint
    └─ Missing (None/undefined)
       └─ Default to start_airport
```

---

## 10. KEY FINDINGS & SUMMARY TABLE

| Aspect | Location | Details |
|--------|----------|---------|
| **UI Config Storage** | webapp/isr.js:37 | state.droneConfigs[did] dict |
| **Fuel Budget Source** | webapp/isr.js:1093 | From input field, default 150 |
| **Start Airport Source** | webapp/isr.js:1094 | From dropdown, default A{did} |
| **End Airport Source** | webapp/isr.js:1095 | From dropdown (with "-" flexible), default start |
| **Target Access Source** | webapp/isr.js:1098-1103 | From checkboxes {a,b,c,d,e} |
| **Run Planner Payload** | webapp/isr.js:1740 | Sends state.droneConfigs AS-IS |
| **Solver Bridge Entry** | solver_bridge.py:477 | solve_mission_with_allocation() |
| **Start Override #1** | solver_bridge.py:589 | Default A{did} or first airport |
| **End Override #1** | solver_bridge.py:594 | Default start, support "-" flexible |
| **Flexible Endpoint Logic** | solver_bridge.py:694-726 | Tries all airports, picks best |
| **Mode Default #1** | solver_bridge.py:427 | "return" if start==end, else "open" |
| **v4 Normalization** | isr_agent_multi_v4.py:1300-1331 | Maps start→home, preserves end |
| **v4 Route Optimizer** | isr_agent_multi_v4.py:851-854 | Uses home_airport, end_airport |
| **Mode Default #2** | isr_agent_multi_v4.py:854 | "return" if home==end, else "open" |
| **Distance Matrix Cache** | solver_bridge.py:505-535 | Hash-based, env-keyed invalidation |
| **Mission ID Store** | main.py:133 | MISSION_STORE dict, per-mission snapshots |
| **Allocation Cache** | main.py:1102-1110 | Stored with env, configs, routes in MISSION_STORE |

---

## 11. CAVEATS & EDGE CASES

### 11.1 start_airport Variations
- UI uses `start_airport` (snake_case)
- v4 Agent normalizes to `home_airport` (camelCase)
- Solver Bridge accepts both
- Default: `A{drone_id}` → first airport if not found

### 11.2 end_airport Variations
- UI uses `end_airport` (snake_case)
- v4 Agent normalizes to `end_airport` (preserves)
- Flexible endpoint ("-") supported ONLY in solver_bridge, NOT in v4 agent
- Default: `start_airport` value

### 11.3 Mode Default Sequence
1. Always defaults to "return" when start==end (both systems)
2. No explicit "open mode" requested from UI config
3. Solver interface default if not specified: "return"

### 11.4 Cache Invalidation
- Distance matrix: Invalidated on env hash mismatch (airports/targets/SAMs moved)
- Mission store: In-memory only, lost on server restart
- No cache invalidation on drone config changes (config doesn't affect distance matrix)

### 11.5 Allocation Persistence
- Stored per mission_id in MISSION_STORE
- Same allocation used for follow-up questions on same mission
- New mission = new allocation (unless v4 uses existing_solution)

---

## 12. FILE REFERENCES (Absolute Paths)

| Purpose | File Path |
|---------|-----------|
| **UI State & Config** | /Users/kamalali/isr-planner/webapp/isr.js |
| **API Endpoints** | /Users/kamalali/isr-planner/server/main.py |
| **Solver Bridge** | /Users/kamalali/isr-planner/server/solver/solver_bridge.py |
| **v4 Agent** | /Users/kamalali/isr-planner/server/agents/isr_agent_multi_v4.py |
| **Distance Matrix** | /Users/kamalali/isr-planner/server/solver/sam_distance_matrix.py |
| **Target Allocator** | /Users/kamalali/isr-planner/server/solver/target_allocator.py |
| **Trajectory Planner** | /Users/kamalali/isr-planner/server/solver/trajectory_planner.py |
| **Orienteering Interface** | /Users/kamalali/isr-planner/webapp/editor/solver/orienteering_interface.py |

