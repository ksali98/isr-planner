# Segmented Import Rewrite Notes

## Date: 2025-12-28

## Current State: BROKEN - Needs Complete Rewrite

The segmented import workflow has become overly complex with multiple conflicting code paths. Tomorrow we will rewrite from scratch.

---

## THE SIMPLE WORKFLOW (What User Wants)

### On Import:
1. Load JSON with segmentInfo (has all targets at top level, plus segmentCuts with cut positions and visitedTargets)
2. Show ONLY segment 0's targets (the ones NOT yet visited by anyone)
3. Show the first cut marker position
4. Mode: Ready to Solve

### On Solve (segment 0):
1. Send ONLY unfrozen targets to solver
2. Start from airport A1
3. Solver returns route visiting some targets
4. Display solution trajectory

### On Accept (segment 0):
1. Mark targets BEFORE the cut marker as visited (green X)
   - Use `segmentCuts[0].visitedTargets` from the imported JSON - NOT the solution's full route
   - Example: If solution visits T3,T4,T2 but marker is after T3,T4, only T3,T4 get green X
2. Show ALL targets (frozen ones with green X, unfrozen ones normal)
3. Show the cut marker (drone's start position for next segment)
4. Clear the old trajectory
5. Advance to segment 1
6. Mode: Ready to Solve

### On Solve (segment N > 0):
1. Send ONLY unfrozen targets to solver
2. Start from the cut marker position (NOT the airport)
3. End at airport A1
4. Solver returns route
5. Display solution trajectory

### On Accept (segment N > 0):
1. Mark additional targets as visited using `segmentCuts[N].visitedTargets`
2. Show next cut marker (or if last segment, ready to animate)
3. Repeat until all segments solved

---

## JSON FORMATS

### Format 1: segmentInfo (preferred)
```json
{
  "type": "segmented",
  "airports": [...],
  "targets": [ALL TARGETS HERE],  // T2,T3,T4,T5,T6,T7,T8,T9
  "sams": [...],
  "drone_configs": {...},
  "segmentInfo": {
    "segmentCuts": [
      {
        "segment": 1,
        "dronePositions": {"1": {"position": [x, y], "distanceTraveled": 83.35}},
        "visitedTargets": ["T3", "T4"]  // Targets before cut 1
      },
      {
        "segment": 2,
        "dronePositions": {"1": {"position": [x, y], "distanceTraveled": 150.6}},
        "visitedTargets": ["T3", "T4", "T2"]  // Targets before cut 2
      }
    ]
  }
}
```

### Format 2: segmentedMission (legacy)
```json
{
  "segments": [
    {"env": {"targets": [...]}, "solution": {...}, "cutDistance": 83.35},
    ...
  ]
}
```

---

## KEY STATE VARIABLES

- `state.env.targets` - Targets to DRAW on canvas
- `state.visitedTargets` - Array of target IDs that have green X
- `state.initialEnvSnapshot` - Should have ALL targets (for reset and for loading after Accept)
- `state.importedSegmentCuts` - Array of cut data from JSON
- `state.pendingCutPositions` - Position of cut marker to display
- `missionReplay` - Stores segments with solutions

---

## CURRENT BUGS (Why Rewrite Needed)

### Bug 1: `saveInitialEnvSnapshot()` overwrites good data
- Import sets `initialEnvSnapshot` with ALL 8 targets
- Later, `saveInitialEnvSnapshot()` is called (from solve?) and overwrites with only 3 targets
- Line 3952 shows: `[saveInitialEnvSnapshot] Saving with 3 targets: T2,T3,T4`

### Bug 2: visitedTargets uses ALL solution targets, not just before-marker
- Code extracts T3,T4,T2 from solution route
- Should only use T3,T4 (from `segmentCuts[0].visitedTargets`)
- T2 is AFTER the marker, should NOT have green X

### Bug 3: allTargetsEnv has only 3 targets after Accept
- Should have ALL 8 targets for drawing
- Only 3 because `initialEnvSnapshot` was overwritten

### Bug 4: Too many places modifying `state.visitedTargets`
- Lines 924, 1118, 3211, 3331, 3451, 3949, 3990, 3991, 4727, 5064, 5189, 5392
- No single source of truth

### Bug 5: Two loader functions with different logic
- `loadSegmentInfoFromJson` - for segmentInfo format
- `loadSegmentedMissionFromJson` - for segments array format
- Detection logic sometimes picks wrong one

---

## REWRITE PLAN

### Step 1: Single Source of Truth for All Targets
- Store ALL targets in `state.allTargets` (never modify this)
- `state.env.targets` becomes a COMPUTED view (filter allTargets by visitedTargets)

### Step 2: Simple Visited Targets Logic
- On Accept: `state.visitedTargets = segmentCuts[currentSegment].visitedTargets`
- That's it. No extraction from solution routes.

### Step 3: Clean Loader Function
```javascript
function loadSegmentedJson(data) {
  // 1. Store ALL targets
  state.allTargets = data.targets;

  // 2. Store cut data
  state.segmentCuts = data.segmentInfo.segmentCuts;

  // 3. Initialize visited as empty
  state.visitedTargets = [];

  // 4. Show segment 0 targets (all of them initially)
  state.env.targets = state.allTargets;

  // 5. Show first cut marker
  state.cutMarkerPosition = state.segmentCuts[0].dronePositions["1"].position;

  // 6. Ready to solve
}
```

### Step 4: Clean Accept Function
```javascript
function acceptSegment(segmentIndex) {
  // 1. Get visited targets from cut data
  if (segmentIndex < state.segmentCuts.length) {
    state.visitedTargets = [...state.segmentCuts[segmentIndex].visitedTargets];
  }

  // 2. state.env.targets stays as ALL targets (drawing handles green X)

  // 3. Move to next segment's cut marker
  if (segmentIndex + 1 < state.segmentCuts.length) {
    state.cutMarkerPosition = state.segmentCuts[segmentIndex + 1].dronePositions["1"].position;
  }

  // 4. Next solve starts from current marker position
}
```

### Step 5: Clean Solve Function
```javascript
function solveSegment(segmentIndex) {
  // Build env for solver - filter out visited targets
  const solverEnv = {
    ...state.env,
    targets: state.allTargets.filter(t => !state.visitedTargets.includes(t.id))
  };

  // For segment > 0, add synthetic start at cut marker position
  if (segmentIndex > 0) {
    solverEnv.synthetic_starts = {
      "D1_START": { x: state.cutMarkerPosition[0], y: state.cutMarkerPosition[1] }
    };
    solverEnv.drone_configs["1"].start_airport = "D1_START";
  }

  // Call solver
}
```

---

## FILES TO MODIFY

1. `/Users/kamalali/isr_projects/isr_web/webapp/isr.js`
   - Lines ~3176-3280: `loadSegmentInfoFromJson`
   - Lines ~3280-3420: `loadSegmentedMissionFromJson`
   - Lines ~870-1150: `acceptSolution`
   - Lines ~4000-4100: `buildCheckpointEnv`
   - Lines ~1970-2000: `drawEnvironment` (green X logic)

2. Remove or consolidate:
   - `saveInitialEnvSnapshot()` - should not exist, just use `state.allTargets`
   - Multiple `state.visitedTargets.push()` calls scattered everywhere

---

## TEST CASE

File: `isr_env2512281754_N6_1.json`
- 8 targets total: T2,T3,T4,T5,T6,T7,T8,T9
- 5 segment cuts
- Segment 0 targets: T2,T3,T4 (before any cuts)
- After Accept segment 0: T3,T4 should have green X (from segmentCuts[0].visitedTargets)
- T2 is AFTER the first marker, should NOT have green X
- T5,T6,T7,T8,T9 should appear for next segments

---

## GIT COMMITS TODAY

- 2c01fba: Fix segmented import: show marker and greenX on Accept
- f819e71: Fix: check both sequence and route fields for target extraction
- 9bae175: Add debug logging for Accept workflow
- 8307de4: Fix: collect ALL targets in initialEnvSnapshot, check state.routes for targets
- 144d95e: Fix: check top-level targets first for initialEnvSnapshot

All of these are incremental patches that didn't fully solve the problem. A clean rewrite is needed.

---

## RECOMMENDATION

Tomorrow: Create a new clean implementation in a separate function, test it thoroughly, then replace the old code. Don't patch anymore.
