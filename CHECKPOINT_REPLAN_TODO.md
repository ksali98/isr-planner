# Checkpoint Replan Feature - Work in Progress

## Status: Partially Working - Needs Debugging

Last updated: 2024-12-16

## Feature Goal
Allow users to:
1. Run Planner → generates drone trajectories
2. Animate → drones move along trajectories
3. Press **C** during animation → freeze drones at current position
4. Edit environment (targets/SAMs/airports)
5. Run Planner again → compute new plan from frozen positions (synthetic starts)
6. Animate → continue from frozen position with new suffix trajectory
7. Press **R** → reset mission to initial state

## What's Been Implemented

### Frontend (webapp/isr.js)
- `freezeAtCheckpoint()` - Freezes drones at current `distanceTraveled` position
- `buildCheckpointEnv()` - Creates synthetic start nodes (`D1_START`, `D2_START`, etc.) at frozen positions
- `startAnimation()` - Modified to:
  - Block animation when checkpoint frozen but no replan done
  - Use `checkpointReplanPrefixDistances` to start from correct position after replan
- `resetMission()` - Resets to initial environment snapshot
- R key handler - Added but **NOT WORKING** (needs debugging)
- Trajectory splicing - Joins frozen prefix with new suffix after replan

### Backend
- `build_id_map()` in main.py - Now includes `synthetic_starts`
- `_parse_env_for_solver()` in solver_bridge.py - Adds synthetic starts to airports list
- `allocate_targets()` in target_allocator.py - Includes synthetic starts
- `calculate_sam_aware_matrix()` in sam_distance_matrix.py - Includes synthetic starts

## Known Issues (TO FIX)

### 1. R Key Reset Not Working
- Handler is installed at lines 183-215 in isr.js
- Added input focus check and debug logging
- Still not triggering reset
- **Debug steps:**
  - Check browser console for "R key pressed" message
  - Verify `state.initialEnvSnapshot` exists (only set after first planner run)
  - Check if any element is capturing the keypress

### 2. Animation Goes Back to Original Airport After Checkpoint Replan
- Synthetic starts are being sent to backend
- Backend should now recognize them (added to id_map, airports list, etc.)
- **Possible issues:**
  - Distance matrix not being recalculated with synthetic starts
  - Trajectory returned by backend doesn't start from synthetic position
  - Frontend trajectory splicing not working correctly
- **Debug steps:**
  - Check Railway logs for `📍 Added synthetic start:` messages
  - Verify `env.synthetic_starts` is populated in request payload
  - Check if returned trajectory starts at synthetic position

### 3. Drone Going Off Trajectory (D3 specifically)
- Related to synthetic starts issue above
- Once synthetic starts work, this should be resolved

## Key State Variables (frontend)

```javascript
state.checkpoint = {
  active: boolean,           // True when frozen
  pct: number,               // Legacy percentage (not used)
  segments: {                // Per-drone frozen data
    [droneId]: {
      prefix: [[x,y], ...],  // Traveled portion of trajectory
      suffix: [[x,y], ...],  // Remaining portion (discarded)
      splitPoint: [x, y],    // Exact frozen position
      checkpointDist: number // Distance traveled when frozen
    }
  }
}

state.checkpointReplanPrefixDistances = {
  [droneId]: number  // Distance of prefix, used to start animation correctly
}

state.initialEnvSnapshot = {...}  // Deep copy of env at first planner run
```

## Key Functions to Debug

1. **buildCheckpointEnv()** (line ~2057) - Creates synthetic starts
2. **runPlanner()** (line ~2178) - Sends request, splices trajectories
3. **startAnimation()** (line ~2558) - Uses prefix distances
4. **resetMission()** (line ~2134) - Restores initial state
5. **R key handler** (line ~183) - Triggers reset

## Testing Workflow

1. Load environment file
2. Run Planner
3. Click Animate (all drones)
4. Press C to freeze mid-animation
5. Check Debug tab for "Frozen at X% progress"
6. Run Planner again
7. Check Debug tab for "📏 Drone X prefix distance: Y"
8. Click Animate
9. Verify drones continue from frozen position (NOT from airport)
10. Press R to reset
11. Verify environment returns to initial state

## Files Modified

- `webapp/isr.js` - Frontend animation, checkpoint, reset logic
- `webapp/index.html` - Version string for cache busting
- `server/main.py` - build_id_map() synthetic starts
- `server/solver/solver_bridge.py` - _parse_env_for_solver() synthetic starts
- `server/solver/target_allocator.py` - allocate_targets() synthetic starts
- `server/solver/sam_distance_matrix.py` - calculate_sam_aware_matrix() synthetic starts

## Cache Busting

Current version string: `20251216checkpoint2`
Update in `webapp/index.html` line 462 when making JS changes.
