# ISR Planner System Architecture

## Overview

The ISR Planner is a mission planning tool for UAV (drone) surveillance missions. Drones must visit targets while avoiding SAM (Surface-to-Air Missile) zones, starting from airports and returning to airports within fuel constraints.

---

## Core Concepts

### Environment
- **Targets**: Points to visit, each with a type (A-E) indicating priority/value
- **Airports**: Start and end points for drones
- **SAM Zones**: Circular no-fly zones that drones must path around
- **Drone Configurations**: Fuel budget, enabled/disabled status per drone

### Solution
- **Routes**: Ordered sequence of waypoints (airports + targets) for each drone
- **Trajectories**: Detailed path coordinates (including SAM-avoidance paths) for each drone
- **Allocations**: Which targets are assigned to which drones

---

## Two Operating Modes

### 1. Non-Segmented (Simple) Mission

**Workflow:**
1. Load/create environment
2. Solve → Get routes and trajectories
3. (Optional) Edit solution using optimizers
4. Accept solution
5. Animate to visualize
6. Can Reset and re-animate

**State:**
- Single solution covers entire mission
- Animation plays from start to finish
- All drones start at airports and end at airports

**Saving:**
- Save environment (targets, airports, SAMs, drone configs)
- Solution is NOT saved - regenerate by solving again

---

### 2. Segmented Mission

**What is it?**
A segmented mission is divided into time periods separated by "cuts". Each cut represents a point where:
- Something changed (drone disabled, new drone added, targets changed)
- Mission was re-planned from current positions

**Workflow:**
1. Load/create environment
2. Solve → Get initial solution (Segment 0)
3. Accept and Animate
4. At some point, CUT the mission (pause at current positions)
5. Make changes (disable drone, add targets, etc.)
6. Solve from current positions → Get new solution (Segment 1)
7. Accept and continue animating
8. Repeat cuts as needed
9. Reset returns to beginning, animation replays all segments

**Key Concepts:**

**Cut Position**: Where each drone is when the cut happens
- Stored as coordinates [x, y] per drone
- Becomes the "synthetic start airport" for the next solve

**Cut Distance**: How far into the mission (in distance units) the cut occurred
- Used to sync animation across segments

**Lost Drones**: Drones that were disabled at a cut
- Their trajectory ends at the cut position
- Marked with red diamond during animation
- Not included in subsequent segments

**Segment Storage:**
```
Segment 0: {
  solution: { routes, trajectories from airport to cut1 },
  cutPositions: { D1: [x,y], D2: [x,y] },  // Where each drone was at C1
  cutDistance: 45.5,  // Distance traveled when cut happened
}

Segment 1: {
  solution: { routes, trajectories from cut1 to cut2 },
  cutPositions: { D1: [x,y], D2: [x,y] },  // Where each drone was at C2
  cutDistance: 92.3,
  lostDrones: ["1"],  // D1 was disabled at this cut
}

Segment 2: {
  solution: { routes, trajectories from cut2 to airport },
  // No cutPositions - mission completed
}
```

---

## Animation System

### Non-Segmented Animation
- All drones animate together at same speed
- Single `missionDistance` tracks global progress
- Target marked visited when drone passes within threshold
- Animation ends when all drones reach their destinations

### Segmented Animation
- Same as above, but...
- When `missionDistance` reaches a `cutDistance`:
  - Switch to next segment
  - Extend trajectories with next segment's data
  - Lost drones stop animating, show red diamond
  - New drones (added in later segments) start animating
- Reset returns to segment 0, clears visited targets
- Full replay shows all segments concatenated

---

## Current Problems (My Understanding)

### Multiple Sources of Truth
- `state.routes` - current display routes
- `missionReplay.segments[].solution.routes` - stored segment data
- `missionState.draftSolution.routes` - pending solution
- `missionState.seg0FullSolution` - backup of segment 0
- `state.checkpoint.segments` - frozen trajectory data

### Complex Splicing
When solving from a checkpoint:
1. Solver returns trajectory starting from cut position
2. We splice: prefix (frozen) + suffix (new solver result)
3. At accept time, we try to extract suffix back out
4. This extraction has edge cases and errors

### Animation Sync Issues
- New drones added in later segments need special handling
- Lost drones need to stop at correct position
- Trajectory extension during segment switch is complex

---

## Proposed Simplification

### Single Source of Truth
**`missionReplay`** is the ONLY place segment data lives.

### Cleaner Segment Storage
Each segment stores EXACTLY what the solver returned:
- Segment 0: Full trajectory from airport
- Segment N: Trajectory from cut position (solver's raw output)

### On-Demand Concatenation
At animation/Reset time, build combined trajectory:
```javascript
function buildFullTrajectory(droneId) {
  let combined = [];
  for (let seg of missionReplay.segments) {
    const traj = seg.solution.routes[droneId]?.trajectory;
    if (traj) {
      // Remove duplicate join point
      if (combined.length > 0 && trajectoriesJoinAt(combined, traj)) {
        combined = combined.concat(traj.slice(1));
      } else {
        combined = combined.concat(traj);
      }
    }
  }
  return combined;
}
```

### Simpler Animation
- Each drone tracks its own `distanceTraveled`
- No global `missionDistance` sync needed
- Drones animate independently along their combined trajectories

---

## Questions for Clarification

1. **When a cut happens, what exactly gets saved?**
   - Just the cut positions?
   - Or the truncated trajectories up to that point?

2. **Should new drones (added in later segments) animate from when they join?**
   - Or should they "catch up" somehow?

3. **For lost drones, should they:**
   - Disappear after their cut?
   - Stay visible at cut position with red marker?
   - Stay visible with trajectory shown up to cut?

4. **What triggers a "cut"?**
   - User clicks Cut button during animation?
   - User pauses and makes changes?
   - Automatic based on events?

5. **When saving a segmented environment:**
   - Save segment structure for replay?
   - Or just save final state?

---

## Segmented Mission JSON Files

### Creating a Segmented Mission JSON

A segmented mission can be created through the UI workflow described above (solve → cut → solve → cut → ...). Once complete, the mission can be exported/saved as a segmented JSON file.

**When to create:**
- User completes a multi-segment mission (multiple cuts with re-solves)
- User wants to save the mission for later replay or sharing

**What gets saved in the JSON:**
```json
{
  "version": "segmented_v1",
  "environment": {
    "targets": [...],
    "airports": [...],
    "sams": [...],
    "drone_configs": {...}
  },
  "segments": [
    {
      "index": 0,
      "solution": {
        "routes": { "1": {...}, "2": {...} },
        "trajectories": { "1": [[x,y], ...], "2": [[x,y], ...] }
      },
      "cutPositions": { "1": [x,y], "2": [x,y] },
      "cutDistance": 45.5,
      "env_snapshot": { /* targets visible at this segment */ }
    },
    {
      "index": 1,
      "solution": { ... },
      "cutPositions": { ... },
      "cutDistance": 92.3,
      "lostDrones": ["1"],
      "env_snapshot": { ... }
    },
    {
      "index": 2,
      "solution": { ... },
      "env_snapshot": { ... }
    }
  ]
}
```

### Importing/Loading a Segmented Mission JSON

**Workflow when loading:**
1. Parse JSON file
2. Load environment (targets, airports, SAMs, drone configs)
3. Load all segments into `missionReplay`
4. Set UI to "ready to animate" state
5. User clicks Animate → Full mission replays from segment 0 through all segments

**Key behavior on import:**
- No solving needed - solutions are already in the JSON
- User can immediately animate to see the full mission replay
- Cut markers (C1, C2, etc.) are displayed based on segment cutPositions
- Lost drone markers shown at appropriate points

### Use Cases for Segmented JSONs

1. **Mission Replay**: Load a previously planned mission and watch it execute
2. **Mission Sharing**: Share a complex multi-segment mission with others
3. **Mission Review**: Analyze a completed mission's decisions at each cut point
4. **Template Missions**: Use as starting point, modify environment, re-solve segments

### Differences: Fresh Creation vs. Import

| Aspect | Fresh Creation | Import from JSON |
|--------|---------------|------------------|
| Solving | Required at each segment | Not needed - solutions included |
| Editing | Full edit capabilities | View/animate only (or re-solve) |
| Segments | Built incrementally | All loaded at once |
| State | `missionReplay` populated as user works | `missionReplay` populated from JSON |

---

## Rebuild Plan: Segmented Mission System

Both the **generation** (fresh creation/export) and **usage** (import/replay) of segmented missions need to be rebuilt from scratch with a clean, consistent design.

### Goals for Rebuild

1. **Single source of truth** - One place where segment data lives
2. **Simple data flow** - No splicing/extraction gymnastics
3. **Predictable behavior** - Same code path for fresh creation and import replay
4. **Clean separation** - Generation logic separate from playback logic

### Proposed Architecture

#### Data Store: `SegmentedMission` Object

```javascript
const segmentedMission = {
  // Base environment (targets, airports, SAMs visible at start)
  baseEnvironment: { targets, airports, sams, drone_configs },

  // Array of segments, each self-contained
  segments: [
    {
      index: 0,
      // What the solver returned (raw, unmodified)
      solution: {
        routes: { droneId: { route: [...], trajectory: [[x,y],...] } },
      },
      // Environment state for THIS segment (targets added/removed)
      environment: { targets, airports, sams, drone_configs },
      // Cut info (null for final segment)
      cut: {
        positions: { droneId: [x, y] },  // Where each drone was
        distance: 45.5,                   // Total distance at cut
      },
      // Drones lost AT THE END of this segment (disabled at the cut)
      lostDrones: [],
      // Drones added AT THE START of this segment
      addedDrones: [],
    },
    // ... more segments
  ],

  // Computed at load/build time (not stored in JSON)
  _computed: {
    combinedTrajectories: { droneId: [[x,y], ...] },  // Full path per drone
    cutMarkers: [{ label: "C1", position: [x,y], lostDroneId: "1" }, ...],
    totalDistance: 150.5,
  }
};
```

#### Generation (Fresh Creation) Flow

```
[Solve Segment 0]
    → Store raw solver output in segments[0].solution
    → segments[0].environment = current env state

[User Cuts at position P]
    → segments[0].cut = { positions: P, distance: D }
    → Freeze segment 0

[User makes changes, Solves Segment 1]
    → Store raw solver output in segments[1].solution
    → segments[1].environment = current env state (may have new targets)
    → If drones disabled: segments[0].lostDrones = [disabled drone ids]
    → If drones added: segments[1].addedDrones = [new drone ids]

[Repeat until done]

[Export to JSON]
    → Write segmentedMission to file (excluding _computed)
```

#### Usage (Import/Replay) Flow

```
[Load JSON]
    → Parse into segmentedMission object
    → Compute combined trajectories
    → Compute cut markers
    → Set UI to "ready to animate"

[Animate]
    → Use _computed.combinedTrajectories for each drone
    → Each drone animates along its full trajectory independently
    → At cut distances, show cut markers
    → Lost drones stop at their cut position, show red diamond
    → New drones appear at their segment's start

[Reset]
    → Clear visited targets
    → Reset all drone positions to trajectory start
    → Ready to animate again
```

#### Key Principle: Same Animation Code for Both Paths

Whether the mission was just created (fresh) or loaded from JSON (import), the animation uses the SAME code:

1. Build combined trajectories from segments (already done at load time for imports)
2. Each drone animates along its combined trajectory
3. Cut markers displayed based on segment cut data
4. Lost/added drones handled based on segment metadata

### What Changes from Current Implementation

| Current | Proposed |
|---------|----------|
| Multiple state objects (missionReplay, state.routes, draftSolution, seg0FullSolution, checkpoint.segments) | Single `segmentedMission` object |
| Splice prefix+suffix at solve, extract at accept | Store raw solver output, concatenate at animation time |
| Different code paths for fresh vs import | Same code path, just different data source |
| Complex segment switching during animation | Pre-computed combined trajectories |
| Global missionDistance syncing all drones | Each drone tracks own progress |

### Files to Rebuild

1. **`webapp/segmented_mission.js`** (NEW)
   - `SegmentedMission` class
   - `addSegment()`, `exportToJSON()`, `importFromJSON()`
   - `computeCombinedTrajectories()`

2. **`webapp/isr.js`** (MODIFY)
   - Remove: `missionReplay`, `segmentedImport`, `seg0FullSolution`, complex splicing
   - Add: Use new `SegmentedMission` class
   - Simplify: Animation to use pre-computed trajectories

3. **`webapp/segment_manager.js`** (POSSIBLY REMOVE or SIMPLIFY)
   - Current functionality absorbed into `SegmentedMission`

---

## File References

- Main UI: `webapp/isr.js`
- Segment Manager: `webapp/segment_manager.js`
- Segmented Import Manager: `webapp/isr.js` (segmentedImport object)
- Server Solver: `server/solver/solver_bridge.py`
- Distance Matrix: `server/solver/sam_distance_matrix.py`

---

*This document is a work in progress. Please edit to clarify requirements and intended behavior.*
