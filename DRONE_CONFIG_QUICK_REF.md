# Drone Configuration Flow - Quick Reference

## Critical Paths

### 1. UI to Run Planner (solve_with_allocation)
```
webapp/isr.js:37
  state.droneConfigs = { "1": {enabled, fuel_budget, start_airport, end_airport, target_access}, ... }

webapp/isr.js:1740
  fetch("/api/solve_with_allocation", payload={drone_configs: state.droneConfigs})

server/main.py:729
  solve_with_allocation(req.env, req.drone_configs, ...)

server/solver/solver_bridge.py:477
  solve_mission_with_allocation(env, drone_configs, ...)
```

### 2. UI to Agent v4 (chat)
```
webapp/isr.js:2348-2356
  fetch("/api/agents/chat-v4", {
    message, env, drone_configs: state.droneConfigs, mission_id: state.missionId
  })

server/main.py:1019
  agent_chat_v4(req)

server/agents/isr_agent_multi_v4.py:1251
  run_multi_agent_v4(user_message, environment, drone_configs, ...)

server/agents/isr_agent_multi_v4.py:1290-1333
  NORMALIZE configs → normalized_configs[did_str]
```

---

## All start_airport Overrides

| # | File | Line | Behavior |
|---|------|------|----------|
| 1 | webapp/isr.js | 1010 | Auto-sync to first airport when airport deleted |
| 2 | webapp/isr.js | 1094 | Read from dropdown, default A{did} or first |
| 3 | solver_bridge.py | 589 | Default A{did} or first if missing |
| 4 | isr_agent_multi_v4.py | 1312 | Map start_airport → home_airport (normalization) |
| 5 | isr_agent_multi_v4.py | 852 | Read home_airport from normalized config |

---

## All end_airport Defaults/Ignores

| # | File | Line | Behavior |
|---|------|------|----------|
| 1 | webapp/isr.js | 1019 | Auto-sync to first airport if deleted |
| 2 | webapp/isr.js | 1020 | Default to first airport if missing |
| 3 | webapp/isr.js | 993 | Dropdown includes "-" (flexible) option |
| 4 | solver_bridge.py | 594 | Default to start_airport, support "-" flexible |
| 5 | solver_bridge.py | 694-726 | Flexible: try all airports, pick best |
| 6 | isr_agent_multi_v4.py | 1320 | Preserve end_airport if present (normalization) |
| 7 | isr_agent_multi_v4.py | 853 | Default to home_airport if missing |

---

## Mode Defaults to "return"

| # | File | Line | Code |
|---|------|------|------|
| 1 | solver_bridge.py | 427 | `env_for_solver["mode"] = "return" if end_id == start_id else "open"` |
| 2 | isr_agent_multi_v4.py | 854 | `mode = "return" if end_airport == home_airport else "open"` |
| 3 | orienteering_interface.py | N/A | `mode = env_data.get("mode", "return")` |

**Key**: Mode is COMPUTED from start==end, not from config. Always "return" when start==end.

---

## Caching Locations

### Distance Matrix
**Files**: 
- server/solver/solver_bridge.py (lines 62-63, 516-534)
- server/solver/sam_distance_matrix.py (singleton `_calculator`)

**Cache Key**: Environment hash (airports + targets + SAMs positions/IDs)
**Invalidation**: Cleared when env hash mismatches
**Access**: 
- solver_bridge.py:519 `get_cached_matrix()`
- main.py:494 `get_current_matrix()`

### Mission ID & Allocation
**File**: server/main.py:133 (global `MISSION_STORE`)

**Storage** (lines 1102-1110):
```python
MISSION_STORE[mission_id] = {
  "env": env,
  "drone_configs": drone_configs,
  "routes": routes,
  "allocation": allocations,
  "total_points": ...,
  "total_fuel": ...,
  "updated_at": ...
}
```

**Retrieval** (lines 1039-1050): Load existing mission for follow-up questions
**UI Storage** (webapp/isr.js:2356, 2363-2365): Store mission_id in state

---

## Config Normalization Mapping (v4 Agent)

| UI Field | v4 Expected | Line | Mapping |
|----------|-------------|------|---------|
| fuel_budget | fuelBudget | 1307 | Pass through |
| start_airport | home_airport | 1317 | start_airport → home_airport |
| end_airport | end_airport | 1324 | Pass through |
| target_access | accessible_targets | 1331 | {a,b,c,d,e} → ["A","B","C",...] |

---

## Payload Structures

### Run Planner (webapp/isr.js:1738-1742)
```javascript
{
  env: state.env,
  drone_configs: state.droneConfigs,
  allocation_strategy: strategy
}
```

### Agent v4 (webapp/isr.js:2351-2357)
```javascript
{
  message: message,
  env: state.env,
  sequences: state.sequences,
  drone_configs: state.droneConfigs,
  mission_id: state.missionId
}
```

---

## Key Functions

| Function | File | Line | Purpose |
|----------|------|------|---------|
| initDroneConfigsFromEnv | webapp/isr.js | 1026 | Load configs from env/defaults |
| attachConfigListeners | webapp/isr.js | 1107 | Listen for config changes, invalidate mission |
| updateAirportDropdowns | webapp/isr.js | 964 | Update dropdowns when airports change |
| runPlanner | webapp/isr.js | 1711 | Send to /api/solve_with_allocation |
| sendAgentMessage | webapp/isr.js | 2326 | Send to /api/agents/chat-v4 |
| solve_with_allocation | solver_bridge.py | 477 | Main solver entry point |
| run_multi_agent_v4 | isr_agent_multi_v4.py | 1251 | v4 agent entry point |
| route_optimizer_node | isr_agent_multi_v4.py | 825 | v4 workflow node: solves per drone |

---

## Debug Outputs

### v4 Route Optimizer Config
**File**: isr_agent_multi_v4.py, Line 904
```
[v4][ROUTE_OPT] D{did} home={home_airport} end={end_airport} mode={mode} cfg_end={cfg.get('end_airport')} cfg_keys={list(cfg.keys())}
```

### Allocation Results
**File**: solver_bridge.py, Lines 279-293
```
🎯 TARGET ALLOCATION RESULTS ({strategy})
  Drone 1: [T1, T2, ...]
  Total allocated: {count}/{total}
```

### Mission Creation
**File**: main.py, Lines 1096-1100
```
[v4] Creating new mission {mission_id}
[v4] Loading existing mission {mission_id} from store
[v4] Updating mission {mission_id}
```

---

## Edge Cases

1. **Airport Deleted**: UI syncs config to first remaining (or creates empty route)
2. **start_airport Missing**: Defaults to A{drone_id}, then first airport, then empty
3. **end_airport = "-"**: Flexible endpoint (solver_bridge ONLY, not v4 agent)
4. **mode Not Specified**: Always computed from start==end, never from config
5. **Mission Restart**: State.missionId = null on environment import, invalidates mission
6. **Distance Matrix Cache**: Invalidated on ANY env change (not config change)
7. **Allocation Persist**: Stored per mission_id, lost on server restart

---

## Testing Checklist

- [ ] Change start_airport in UI → verify solver uses new start
- [ ] Change end_airport in UI → verify solver uses new end (or flexible logic)
- [ ] Delete airport from env → verify configs sync to valid airport
- [ ] Set end_airport = "-" → verify solver tries all endpoints
- [ ] Run planner → verify mission_id created and stored in state
- [ ] Ask question on same mission → verify allocation reused (not recomputed)
- [ ] Import new environment → verify missionId cleared
- [ ] Check distance matrix status → verify cached/cleared correctly
