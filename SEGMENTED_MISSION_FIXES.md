# Comprehensive Segmented Mission Bug Fixes

This document details all bug fixes made to the segmented mission animation system in `webapp/isr.js`. Fixes are organized chronologically and by functional area.

---

## Part 1: Initial Bug Fixes (Previous Session)

### Bug 1: "Cannot cut in current state" after cutting at checkpoint
**Location:** `isr.js` ~line 7771
**Problem:** After cutting at a checkpoint, `checkpointSource` was set to `"replay_cut"` which blocked the solver from running.
**Fix:** Set `checkpointSource = null` instead of `"replay_cut"` to allow solving after cut.

---

### Bug 2: Double concatenation of frozen trajectory
**Location:** `runPlanner()` and `applyDraftSolutionToUI()`
**Problem:** When solving after a cut:
1. `runPlanner()` was prepending the frozen prefix to the solver's result
2. `applyDraftSolutionToUI()` was ALSO concatenating frozen + new
3. Result: trajectory went A → c1 → A → c1 → end (double prefix)

**Fix:** Remove the splice/concatenation logic from `runPlanner()`. Let the solver return segment-only trajectory, and only do concatenation in `applyDraftSolutionToUI()`.

---

### Bug 3: acceptSolution storing wrong trajectory
**Location:** `isr.js` ~line 1371-1390
**Problem:** `acceptSolution()` was using `state.routes` (combined display trajectory) instead of `draftSolution.routes` (segment-only trajectory) for checkpoint replans.
**Fix:** Add check: `if (!isCheckpointReplan && state.routes...)` - for checkpoint replans, keep `draftSolution.routes` as-is.

---

### Bug 4: Wrong truncation distance in buildCombinedRoutesFromSegments
**Location:** `isr.js` ~line 6200-6225
**Problem:** `cutDistance` is CUMULATIVE from mission start, but `split_polyline_at_distance()` measures from the START of each segment's trajectory. So truncating segment 1 at cutDistance 61.7 was wrong - it should be truncated at segment-relative distance (61.7 - 27.7 = 34.0).
**Fix:** Calculate segment-relative truncation:
```javascript
const segmentRelativeTruncation = nextCutDistance - thisCutDistance;
```

---

### Bug 5: buildCombinedRoutesFromSegments(0) not truncating segment 0
**Location:** `isr.js` ~line 6217
**Problem:** The truncation condition `i < segmentCount - 1` meant when building only segment 0, it wouldn't truncate (because 0 < 0 is false).
**Fix:** Change condition to truncate whenever there's a next segment with cutDistance, regardless of iteration limit.

---

### Bug 6: Reset function using segmentedImport when it's not active
**Location:** `isr.js` ~line 2458-2560
**Problem:** The Reset code checked `segmentedImport.isActive()` but segments created via the normal cut workflow don't use segmentedImport. So `missionReplay` had 4 segments but `segmentedImport.isActive()` was false.
**Fix:** Change condition to `if (resetSegCount > 1 || segmentedImport.isActive())`

---

### Bug 7: Environment vanishes after Reset
**Location:** `isr.js` ~line 2550
**Problem:** `state.env = segmentedImport.getEnvForDisplay()` returns `null` when segmentedImport is not active, making the environment disappear.
**Fix:** Check `if (segmentedImport.isActive())` before calling `getEnvForDisplay()`, otherwise get env from `missionReplay.getSegment(0)`.

---

### Bug 8: Targets marked as visited before drone reaches them
**Location:** `isr.js` ~line 7258
**Problem:** The proximity threshold for marking targets as visited was 20.0 units - too large. Targets near the cut point (but in seg-1) were being marked during seg-0 animation.
**Fix:** Reduce threshold from 20.0 to 5.0.

---

### Bug 9: Targets from later segments not showing after Reset
**Location:** `isr.js` ~line 2550-2565
**Problem:** After Reset, `state.env` only had segment 0's targets (7 targets). Targets added in later segments (T8, T9, T10) were missing because they weren't collected.
**Fix:** Collect all targets from ALL segments at Reset:
```javascript
const allTargetsMap = new Map();
for (let i = 0; i < missionReplay.getSegmentCount(); i++) {
  const seg = missionReplay.getSegment(i);
  if (seg?.env?.targets) {
    seg.env.targets.forEach(t => allTargetsMap.set(t.id, t));
  }
}
state.env.targets = Array.from(allTargetsMap.values());
state.initialEnvSnapshot.targets = allTargets; // For segment switch to use
```

---

## Part 2: Today's Bug Fixes (Current Session)

### Bug 10: Solver starting from airport instead of synthetic start
**Location:** `buildCheckpointEnv()` ~line 5973-6183
**Problem:** `buildCheckpointEnv()` only set `start_airport` for synthetic starts, but the solver might read `start_id` or `home_airport` instead, causing drones to start from the airport instead of the checkpoint position.
**Fix:** Set ALL three fields in all 5 paths of `buildCheckpointEnv()`:
```javascript
if (newDroneConfigs[did]) {
  newDroneConfigs[did].start_airport = nodeId;
  newDroneConfigs[did].start_id = nodeId;
  newDroneConfigs[did].home_airport = nodeId;
}
```

**Paths affected:**
1. segmentedImport checkpoint path
2. segmentedImport segIdx > 0 path
3. legacy segmentInfo path
4. live creation path
5. active checkpoint path

---

### Bug 11: applyDraftSolutionToUI using wrong frozen trajectory source
**Location:** `applyDraftSolutionToUI()` ~line 6606-6611
**Problem:** Used `mergedRoutes[did]?.trajectory` for frozen trajectory, which could be stale/mutated.
**Fix:** Use `draft.checkpointSegments?.[did]?.prefix` first (authoritative frozen data captured at cut time):
```javascript
const frozenTraj = draft.checkpointSegments?.[did]?.prefix
  || mergedRoutes[did]?.trajectory
  || [];
```

---

### Bug 12: applyDraftSolutionToUI using stale cutDistance
**Location:** `applyDraftSolutionToUI()` ~line 6666-6668
**Problem:** Used `state.pendingCutDistance` which gets cleared after solve.
**Fix:** Use `draft.cutDistance` first, fallback to `state.pendingCutDistance`:
```javascript
const cutDist = (draft.cutDistance != null) ? draft.cutDistance : state.pendingCutDistance;
```

---

### Bug 13: Route array not concatenated in buildCombinedRoutesFromSegments
**Location:** `buildCombinedRoutesFromSegments()` ~line 6336-6350
**Problem:** Only trajectory was being concatenated, not the route array. This caused targets from later segments (T8, T9, T10) to not get checkmarks during replay.
**Fix:** Add route concatenation with filtering of synthetic starts and duplicates:
```javascript
// CRITICAL: Also concatenate the route array so targets from later segments get checked
const existingRoute = combinedRoutes[droneId].route || [];
const newRoute = routeData.route || [];
const filteredNewRoute = newRoute.filter(wp => {
  // Skip synthetic starts (D1_START, D2_START, etc.)
  if (String(wp).match(/^D\d+_START$/)) return false;
  // Skip if already in existing route
  if (existingRoute.includes(wp)) return false;
  return true;
});
combinedRoutes[droneId].route = existingRoute.concat(filteredNewRoute);
```

---

### Bug 14: visited_targets not stored per segment
**Location:** `acceptSolution()` ~line 1465-1489
**Problem:** Visited targets were computed geometrically during animation, which was unreliable especially when drones are killed.
**Fix:** Store `visited_targets` per segment at accept time for deterministic replay:
```javascript
// Compute visited_targets: all targets in this segment's routes
const segmentVisitedTargets = new Set();
Object.values(solutionToStore.routes || {}).forEach(routeData => {
  (routeData.route || []).forEach(wp => {
    if (String(wp).startsWith('T')) {
      segmentVisitedTargets.add(wp);
    }
  });
});

segment = missionReplay.addSegment({
  // ... other fields ...
  visited_targets: [...segmentVisitedTargets],
});
```

---

### Bug 15: Segment switch handler not properly handling lost drones
**Location:** Segment switch handler ~line 7515-7608
**Problem:** Lost drones weren't being handled in the right order, and their state wasn't being properly updated during replay.
**Fix:** Implement 3-step process:

**STEP 1: Apply lost drones FIRST (before applying routes)**
```javascript
const allLostDrones = new Set();
for (let i = 0; i <= newSegment.index; i++) {
  const seg = missionReplay.getSegment(i);
  if (seg?.lostDrones) {
    seg.lostDrones.forEach(did => allLostDrones.add(String(did)));
  }
}
allLostDrones.forEach(did => {
  if (state.animation.drones[did]) {
    state.animation.drones[did].animating = false;
  }
  if (state.droneConfigs?.[did]) {
    state.droneConfigs[did].enabled = false;
  }
  state.trajectoryVisible[did] = true;
});
```

**STEP 2: Update visitedTargets from stored segment data**
```javascript
const storedVisited = new Set();
for (let i = 0; i <= newSegment.index; i++) {
  const seg = missionReplay.getSegment(i);
  if (seg?.visited_targets) {
    seg.visited_targets.forEach(t => storedVisited.add(t));
  }
}
storedVisited.forEach(t => {
  if (!state.visitedTargets.includes(t)) {
    state.visitedTargets.push(t);
  }
});
```

**STEP 3: Build combined routes and update drone states**

---

### Bug 16: Trajectories shrinking instead of growing during segment switches
**Location:** `buildCombinedRoutesFromSegments()` ~line 6300-6306
**Problem:** ALL segments were being truncated, including the current segment being animated. When switching to segment 1, both seg0 AND seg1 were truncated, causing trajectory to shrink from 9pts to 6pts.
**Fix:** Only truncate segments BEFORE the one being animated:
```javascript
// CRITICAL: Only truncate segments BEFORE the last one being built.
// The last segment (the one we're currently animating) should keep its FULL trajectory.
const isLastSegmentBeingBuilt = (i === segmentCount - 1);
if (!isLastSegmentBeingBuilt && segmentRelativeTruncation !== null && segmentRelativeTruncation > 0) {
  // truncate...
}
```

---

### Bug 17: Lost drone trajectories not updated at segment switch
**Location:** Segment switch handler ~line 7564-7608
**Problem:** Lost drones were being skipped entirely (`if (allLostDrones.has(did)) return;`), so their trajectories weren't updated to be truncated at the kill point.
**Fix:** Update lost drone trajectories but keep them stopped:
```javascript
Object.entries(combinedRoutes).forEach(([did, routeData]) => {
  const isLostDrone = allLostDrones.has(did);
  // ... update trajectory for all drones including lost ones ...

  if (isLostDrone) {
    droneState.animating = false;
    state.trajectoryVisible[did] = true;
    // Don't re-enable animation
  } else if (droneState.distanceTraveled < totalDistance) {
    droneState.animating = true;
  }
});
```

---

### Bug 18: draftSolution missing lostDrones field
**Location:** `runPlanner()` ~line 6545
**Problem:** `draftSolution` didn't include `lostDrones`, so `applyDraftSolutionToUI()` couldn't access it.
**Fix:** Add `lostDrones` to draftSolution:
```javascript
missionState.draftSolution = {
  // ... other fields ...
  lostDrones: state.pendingLostDrones?.length > 0 ? [...state.pendingLostDrones] : [],
};
```

---

## Key Concepts Reference

### 1. Segment trajectory vs Combined trajectory
Each segment stores only its portion (c_n → c_{n+1}). The combined trajectory for display is built by concatenating all segments up to the current one.

### 2. cutDistance is cumulative
Segment 1's cutDistance of 61.7 means 61.7 from mission START, not from segment 1's start. When truncating, calculate segment-relative distance.

### 3. Frozen trajectory
The portion already traveled should NEVER be removed. It persists throughout the mission. Use `draft.checkpointSegments?.[did]?.prefix` for authoritative frozen data.

### 4. segmentedImport vs missionReplay
- `segmentedImport` is for imported JSON missions
- Normal cut workflow uses `missionReplay` only
- Always check `missionReplay.getSegmentCount() > 1` rather than relying solely on `segmentedImport.isActive()`

### 5. Lost drones are first-class segment events
- Store `lostDrones` in each segment at accept time
- Apply lost drone state BEFORE applying routes at segment boundaries
- Lost drone trajectories should be truncated at kill point, not preserved beyond it

### 6. visited_targets should be deterministic
- Store `visited_targets` per segment at accept time
- During replay, union stored visited_targets from all segments played so far
- Don't rely on geometric proximity checks during animation

---

## Files Modified

All changes are in: `webapp/isr.js`

**Key functions affected:**
- `buildCheckpointEnv()` - synthetic start configuration
- `applyDraftSolutionToUI()` - frozen trajectory merging
- `buildCombinedRoutesFromSegments()` - trajectory/route concatenation and truncation
- `acceptSolution()` - segment storage with visited_targets
- `runPlanner()` - draftSolution creation
- Segment switch handler (in animation code) - lost drone handling and route application
