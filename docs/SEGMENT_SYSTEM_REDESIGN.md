# Segment System Redesign

## Problem Statement

The current segment/animation system has multiple sources of truth and branching code paths. This causes drone configs to get out of sync, lost/disabled drones to appear with trajectories they shouldn't have, and different behavior for imported vs authored missions.

## Scope: Complete Rewrite

This is NOT a patch. We are rewriting everything related to segments:

1. **Segment Creation** - How segments are built when accepting solutions
2. **Segment Storage** - The MissionReplay data structure
3. **Segment Import** - Loading segments from JSON
4. **Segment Export** - Saving segments to JSON
5. **Reset** - Returning to mission start
6. **Animation** - Playing through segments
7. **Segment Switch** - Transitioning between segments during animation

All existing segment-related code paths will be replaced with a single, unified implementation.

---

## Design Principle: Each Segment is Self-Contained

Each segment stores EVERYTHING needed to display and animate that segment. No reconstruction from previous segments required. This causes data duplication but guarantees simpler, more reliable code.

---

## Segment Data Structure

```javascript
class Segment {
  index: number;              // 0, 1, 2, ...
  timestamp: number;          // When accepted

  // ============================================
  // DRONE CONFIGURATIONS (per drone)
  // ============================================
  drone_configs: {
    [droneId: string]: {
      enabled: boolean;       // Is this drone active in this segment?
      fuel_capacity: number;
      fuel_remaining: number; // Fuel left at START of this segment
      start_airport: string;  // Real (A1) or synthetic (D1_START)
      end_airport: string;
      target_access: { [targetId: string]: boolean };
    }
  };

  // ============================================
  // WAYPOINTS
  // ============================================
  waypoints: {
    // Real airports (static - do NOT move between segments)
    airports: [{ id: string, x: number, y: number }];

    // Synthetic starts for this segment
    // If drone was inside a SAM zone at cut, this is the nearest point OUTSIDE the SAM
    // The escape path from cut_position to synthetic_start is prepended to trajectory
    synthetic_starts: {
      [droneId: string]: {
        id: string,           // e.g., "D1_START"
        x: number,
        y: number,
        cut_position?: [x, y] // Original cut position (if different due to SAM escape)
      }
    };
  };

  // ============================================
  // TARGETS
  // ============================================
  targets: {
    // Frozen = visited in previous segments, shown with green X
    // Position is LOCKED at time of visit - frozen targets do NOT move
    frozen: [{ id: string, x: number, y: number, priority: number }];

    // Active = available to visit in this segment
    // Position CAN change between segments (targets may move)
    active: [{ id: string, x: number, y: number, priority: number }];

    // All targets (frozen + active) for display
    all: [{ id: string, x: number, y: number, priority: number }];
  };

  // ============================================
  // SAMs (can be added, removed, or moved between segments)
  // ============================================
  sams: [{ id: string, pos: [x, y], range: number }];

  // ============================================
  // TRAJECTORIES (per drone)
  // ============================================
  trajectories: {
    [droneId: string]: {
      // RENDERING: Full trajectory from mission start (for polyline display)
      render_full: [[x, y], [x, y], ...];

      // ANIMATION: Only the portion to fly in THIS segment
      // Animator uses ONLY this - never render_full
      delta: [[x, y], [x, y], ...];

      // Where frozen portion ends in render_full (index)
      // Points 0..frozenEndIndex are frozen (gray), frozenEndIndex+1..end are this segment
      frozenEndIndex: number;

      // Route waypoints for this segment only
      route: [string];  // e.g., ["D1_START", "T5", "T8", "A2"]

      // Distance of delta trajectory only
      deltaDistance: number;

      // End state after this segment completes
      endState: {
        position: [x, y];
        fuel_remaining: number;
      };
    }
  };

  // ============================================
  // SEGMENT BOUNDARIES (explicit start/end)
  // ============================================
  startDist: number;           // Global cumulative distance where this segment STARTS
                               // Segment 0: 0
                               // Segment N: previous segment's endDist

  endDist: number | null;      // Global cumulative distance where this segment ENDS
                               // null only for last/open segment (not yet accepted)

  cutPositionsAtEnd: {         // Where each drone is at endDist (for next segment's synthetic starts)
    [droneId: string]: [x, y]
  } | null;                    // null for segment 0 (no cut) and last segment (open)
}
```

---

## Key Clarifications

### 1. Dual Trajectory Storage: render_full vs delta

**CRITICAL: Animator uses ONLY `delta`. Never `render_full`.**

Each segment stores TWO trajectory representations:

```
render_full: [A1 → ... → C1 → ... → C2 → ... → end]  // For polyline display
             \_________/   \________/   \_________/
             Seg 0 frozen  Seg 1 frozen  Seg 2 delta

delta: [C2 → ... → end]  // For animation - THIS SEGMENT ONLY

frozenEndIndex: 45  // In render_full, points 0-45 are frozen (gray)
```

**Why two arrays?**
- `render_full`: Polyline rendering shows entire history
- `delta`: Animation is bounded and local - no index confusion when switching segments

**Segment boundaries example:**
```
Segment 0: startDist=0,    endDist=45.2,  delta=[A1→...→C1]
Segment 1: startDist=45.2, endDist=110.0, delta=[C1→...→C2]
Segment 2: startDist=110.0, endDist=null,  delta=[C2→...→end]
```

Boundaries are explicit and unambiguous. No "where does this segment begin/end" confusion.

### 2. Drone Configs are Authoritative

If `segment.drone_configs["2"].enabled === false`, then:
- D2 does NOT animate in this segment
- D2's trajectory is NOT extended
- D2's trajectory up to its death point is still shown (frozen)

### 3. SAMs Can Change

Each segment stores its own SAM configuration. When displaying segment N:
- Use `segment.sams` for rendering
- The SAM-aware distance matrix is recomputed if SAMs changed

**SAM Engulfment Handling (Precise Geometry):**

If a SAM is added/moved such that a drone's cut position falls INSIDE the SAM zone:

```javascript
const MARGIN = 0.5;  // Safety buffer to avoid floating-point edge cases

function computeEscapePoint(cutPos, sam) {
  const [cx, cy] = cutPos;
  const [sx, sy] = sam.pos;
  const dx = cx - sx;
  const dy = cy - sy;
  const dist = Math.hypot(dx, dy);

  if (dist >= sam.range) {
    // Not inside SAM, no escape needed
    return null;
  }

  if (dist < 0.001) {
    // Exactly at SAM center - pick deterministic direction (+x)
    return [sx + sam.range + MARGIN, sy];
  }

  // Nearest exit point: along radial line from SAM center through cut position
  const scale = (sam.range + MARGIN) / dist;
  return [sx + dx * scale, sy + dy * scale];
}
```

**Rules:**
1. Escape path is prepended to delta trajectory
2. Escape path DOES consume fuel
3. Escape path DOES count in deltaDistance
4. synthetic_start.cut_position stores original position (for display)
5. synthetic_start x,y stores escape point (for solver)

```
Example:
- Cut position: (50, 60)
- SAM at (50, 55) with range 10
- Distance from SAM center: 5
- Escape direction: (0, 1) normalized
- Escape point: (50, 55 + 10.5) = (50, 65.5)
- Delta trajectory: [(50, 60), (50, 65.5), ...solver result...]
```

### 4. Airports are Static

Airports do NOT move between segments. They are defined once at mission start.

### 5. Targets: Frozen vs Active

**Frozen targets** (visited in previous segments):
- Position is locked - they do NOT move
- Displayed with green X overlay
- Stored with their position at the time they were visited

**Active targets** (available to visit in current segment):
- CAN move between segments (position may change)
- Each segment stores the current position of active targets

```
Segment 0: frozen=[], active=[T1,T2,T3,T4,T5]
Segment 1: frozen=[T1,T2], active=[T3,T4,T5]  // T1,T2 visited in seg 0
Segment 2: frozen=[T1,T2,T3], active=[T4,T5]  // T3 visited in seg 1
```

The `all` array is always `frozen + active` for easy rendering.

### 6. Target Identity Rules

**CRITICAL: Never compute "visited" by comparing coordinates. Always by ID.**

```javascript
// CORRECT
function isTargetVisited(targetId) {
  return visitedTargets.includes(targetId);
}

// WRONG - will break when targets move
function isTargetVisited(targetPos) {
  return visitedTargets.some(t => t.x === targetPos.x && t.y === targetPos.y);
}
```

**Rules:**
- Target IDs must remain stable across segments
- Frozen targets lock position at visit time
- If an active target moves, it keeps the same ID
- Moving a not-yet-visited target invalidates previously computed routes
  (user must re-solve after moving targets)

---

## Operations

### Reset (HARD CLEAR - Critical)

**NEVER call segment-switch logic as part of reset. Only loadSegment(0).**

Reset must atomically clear ALL replay/animation state to prevent "all trajectories appear at once" bugs:

```javascript
function reset() {
  // ========================================
  // STEP 1: Hard-clear ALL animation state
  // ========================================
  missionReplay.currentIndex = 0;

  animation.globalDist = 0;        // CRITICAL: Must be 0, not carryover
  animation.segmentDist = 0;
  animation.switchInProgress = false;  // Clear any stuck gate

  // Clear per-drone animation state
  Object.keys(animation.drones).forEach(did => {
    animation.drones[did] = {
      pathIndex: 0,
      distanceTraveled: 0,
      animating: false,
      position: null
    };
  });

  visitedTargets = [];  // Runtime visited set (display uses segment.targets.frozen)

  // ========================================
  // STEP 2: Load segment 0 (no switch logic!)
  // ========================================
  loadSegment(0);
}

function loadSegment(index) {
  const segment = missionReplay.getSegment(index);

  state.droneConfigs = deepCopy(segment.drone_configs);
  state.env.targets = segment.targets.all;
  state.env.sams = segment.sams;

  // Set up trajectories for rendering (render_full) and animation (delta)
  Object.entries(segment.trajectories).forEach(([did, traj]) => {
    state.routes[did] = {
      trajectory: traj.render_full,  // For polyline display
      delta: traj.delta,             // For animation
      route: traj.route,
      frozenEndIndex: traj.frozenEndIndex
    };
  });
}
```

### Animation Start

```javascript
function startAnimation() {
  const segment = missionReplay.current();

  // Select drones: enabled AND have delta to fly
  const animDrones = Object.keys(segment.trajectories)
    .filter(did => segment.drone_configs[did]?.enabled !== false)
    .filter(did => segment.trajectories[did]?.delta?.length >= 2);

  // Initialize per-drone animation state
  animDrones.forEach(did => {
    const traj = segment.trajectories[did];
    animation.drones[did] = {
      pathIndex: 0,                    // Index into DELTA
      distanceTraveled: 0,             // Distance within DELTA
      animating: true,
      position: traj.delta[0]          // Start at first delta point
    };
  });

  animation.globalDist = segment.startDist;
  animation.running = true;
}
```

**Animator loop uses DELTA only:**
```javascript
function animationStep(dt) {
  const segment = missionReplay.current();
  const prevGlobalDist = animation.globalDist;

  Object.entries(animation.drones).forEach(([did, droneState]) => {
    if (!droneState.animating) return;

    const delta = state.routes[did].delta;
    // Move along DELTA trajectory
    // Update droneState.pathIndex, droneState.distanceTraveled
    // ...
  });

  const newGlobalDist = segment.startDist + maxDroneDeltaDistance();
  animation.globalDist = newGlobalDist;

  // Check for segment switch (CROSSING, not "beyond")
  maybeSegmentSwitch(prevGlobalDist, newGlobalDist);
}
```

### Segment Switch (CROSSING detection + atomic gate)

**Switch trigger must be CROSSING, not "being beyond":**
```javascript
if (prevDist < endDist && newDist >= endDist) switch
```
This prevents repeated switching if distance is already past boundary after reset.

**Re-entrancy protection:**
```javascript
function maybeSegmentSwitch(prevGlobalDist, newGlobalDist) {
  // ========================================
  // GATE: Prevent re-entrant switches
  // ========================================
  if (animation.switchInProgress) return;

  const currentSeg = missionReplay.current();
  if (currentSeg.endDist === null) return;  // Last segment, no switch

  // ========================================
  // CROSSING detection (not just "beyond")
  // ========================================
  const crossed = (prevGlobalDist < currentSeg.endDist) &&
                  (newGlobalDist >= currentSeg.endDist);
  if (!crossed) return;

  // ========================================
  // ATOMIC SWITCH
  // ========================================
  animation.switchInProgress = true;

  try {
    const nextIndex = currentSeg.index + 1;
    const nextSeg = missionReplay.getSegment(nextIndex);

    // Update global state
    missionReplay.setCurrent(nextIndex);
    state.droneConfigs = deepCopy(nextSeg.drone_configs);
    state.env.targets = nextSeg.targets.all;
    state.env.sams = nextSeg.sams;
    visitedTargets = nextSeg.targets.frozen.map(t => t.id);

    // Update per-drone state
    Object.entries(nextSeg.trajectories).forEach(([did, traj]) => {
      const droneState = animation.drones[did];
      const config = nextSeg.drone_configs[did];

      // Update trajectory for rendering
      state.routes[did].trajectory = traj.render_full;
      state.routes[did].delta = traj.delta;
      state.routes[did].frozenEndIndex = traj.frozenEndIndex;

      if (config?.enabled === false) {
        // Disabled drone: stop animating, keep trajectory visible
        droneState.animating = false;
      } else {
        // Enabled drone: reset delta animation state
        droneState.pathIndex = 0;       // Start of delta
        droneState.distanceTraveled = 0; // Within delta
        droneState.animating = traj.delta.length >= 2;
      }
    });

    // Set global distance to segment start
    animation.globalDist = nextSeg.startDist;

  } finally {
    animation.switchInProgress = false;
  }
}
```

**Key invariants:**
- Animator uses `delta` for motion, `render_full` for display
- `droneState.pathIndex` indexes into `delta`, NOT `render_full`
- `droneState.distanceTraveled` is distance within `delta`, NOT global

### Accept Solution

```javascript
function acceptSolution(solverResult) {
  const prevSegment = missionReplay.count() > 0
    ? missionReplay.getSegment(missionReplay.count() - 1)
    : null;

  const newIndex = prevSegment ? prevSegment.index + 1 : 0;
  const startDist = prevSegment ? prevSegment.endDist : 0;

  // ========================================
  // Build trajectories with DE-DUPLICATION
  // ========================================
  const trajectories = {};
  let maxEndDist = startDist;

  Object.entries(solverResult.routes).forEach(([did, routeData]) => {
    const config = state.droneConfigs[did];
    const newDelta = routeData.trajectory;  // Solver result is this segment only

    if (config?.enabled === false) {
      // ========================================
      // DISABLED DRONE INVARIANTS
      // ========================================
      // Must not extend trajectory, route must be empty
      const prevTraj = prevSegment?.trajectories[did];
      trajectories[did] = {
        render_full: prevTraj?.render_full || [],
        delta: [],                    // NO delta for disabled drone
        frozenEndIndex: prevTraj?.render_full?.length - 1 || -1,
        route: [],                    // Empty route
        deltaDistance: 0,             // Zero distance
        endState: prevTraj?.endState || null
      };
    } else {
      // ========================================
      // ENABLED DRONE: Build render_full with de-duplication
      // ========================================
      const prevFull = prevSegment?.trajectories[did]?.render_full || [];

      // DE-DUPLICATE splice point
      let delta = newDelta;
      if (prevFull.length > 0 && delta.length > 0) {
        const prevLast = prevFull[prevFull.length - 1];
        const newFirst = delta[0];
        const EPS = 0.001;
        if (Math.hypot(prevLast[0] - newFirst[0], prevLast[1] - newFirst[1]) < EPS) {
          delta = delta.slice(1);  // Drop duplicate first point
        }
      }

      const render_full = prevFull.concat(delta);
      const deltaDistance = computePolylineLength(delta);

      trajectories[did] = {
        render_full: render_full,
        delta: delta,
        frozenEndIndex: prevFull.length - 1,
        route: routeData.route,
        deltaDistance: deltaDistance,
        endState: {
          position: delta.length > 0 ? delta[delta.length - 1] : prevFull[prevFull.length - 1],
          fuel_remaining: routeData.fuel_remaining
        }
      };

      maxEndDist = Math.max(maxEndDist, startDist + deltaDistance);
    }
  });

  // ========================================
  // Build segment
  // ========================================
  const segment = {
    index: newIndex,
    timestamp: Date.now(),
    drone_configs: deepCopy(state.droneConfigs),
    waypoints: {
      airports: state.env.airports,
      synthetic_starts: state.pendingCutPositions ? buildSyntheticStarts() : {}
    },
    targets: {
      frozen: visitedTargets.map(id => findTarget(id)),
      active: state.env.targets.filter(t => !visitedTargets.includes(t.id)),
      all: state.env.targets
    },
    sams: deepCopy(state.env.sams),
    trajectories: trajectories,
    startDist: startDist,
    endDist: maxEndDist,
    cutPositionsAtEnd: computeCutPositions(trajectories, maxEndDist)
  };

  // ========================================
  // VALIDATE before adding
  // ========================================
  validateSegment(segment);

  missionReplay.append(segment);
  clearPendingState();
}
```

---

---

## Segment Validation

**ALWAYS validate before adding a segment. Refuse invalid segments.**

```javascript
function validateSegment(segment) {
  const errors = [];

  // ========================================
  // Boundary validation
  // ========================================
  if (segment.index > 0) {
    const prev = missionReplay.getSegment(segment.index - 1);
    if (segment.startDist !== prev.endDist) {
      errors.push(`startDist ${segment.startDist} != prev.endDist ${prev.endDist}`);
    }
  } else {
    if (segment.startDist !== 0) {
      errors.push(`Segment 0 must have startDist=0`);
    }
  }

  if (segment.endDist !== null && segment.endDist <= segment.startDist) {
    errors.push(`endDist ${segment.endDist} must be > startDist ${segment.startDist}`);
  }

  // ========================================
  // Per-drone validation
  // ========================================
  Object.entries(segment.trajectories).forEach(([did, traj]) => {
    const config = segment.drone_configs[did];

    if (config?.enabled === false) {
      // Disabled drone invariants
      if (traj.delta.length > 0) {
        errors.push(`Disabled drone ${did} must have empty delta`);
      }
      if (traj.route.length > 0) {
        errors.push(`Disabled drone ${did} must have empty route`);
      }
      if (traj.deltaDistance !== 0) {
        errors.push(`Disabled drone ${did} must have deltaDistance=0`);
      }
    } else {
      // Enabled drone invariants
      if (traj.delta.length < 2) {
        errors.push(`Enabled drone ${did} must have delta.length >= 2`);
      }
    }

    // No duplicate consecutive points (within epsilon)
    const EPS = 0.001;
    for (let i = 1; i < traj.delta.length; i++) {
      const d = Math.hypot(
        traj.delta[i][0] - traj.delta[i-1][0],
        traj.delta[i][1] - traj.delta[i-1][1]
      );
      if (d < EPS) {
        errors.push(`Drone ${did} delta has duplicate point at index ${i}`);
      }
    }

    // frozenEndIndex consistency
    if (traj.frozenEndIndex >= traj.render_full.length) {
      errors.push(`Drone ${did} frozenEndIndex ${traj.frozenEndIndex} out of bounds`);
    }
  });

  if (errors.length > 0) {
    throw new Error(`Invalid segment:\n${errors.join('\n')}`);
  }
}
```

---

## MissionReplay API Contract

**Single source of truth. UI must not mutate internal arrays.**

```javascript
class MissionReplay {
  #segments = [];      // Private - no direct access
  #currentIndex = 0;

  // ========================================
  // READ operations
  // ========================================

  count() {
    return this.#segments.length;
  }

  getSegment(i) {
    if (i < 0 || i >= this.#segments.length) return null;
    return this.#segments[i];  // Consider returning deep copy for safety
  }

  current() {
    return this.getSegment(this.#currentIndex);
  }

  currentIndex() {
    return this.#currentIndex;
  }

  // ========================================
  // WRITE operations (controlled)
  // ========================================

  append(segment) {
    // Validate monotonic boundaries
    if (this.#segments.length > 0) {
      const last = this.#segments[this.#segments.length - 1];
      if (segment.startDist !== last.endDist) {
        throw new Error('Segment boundaries not monotonic');
      }
      if (segment.index !== last.index + 1) {
        throw new Error('Segment index not sequential');
      }
    }
    validateSegment(segment);
    this.#segments.push(segment);
  }

  setCurrent(index) {
    if (index < 0 || index >= this.#segments.length) {
      throw new Error(`Invalid segment index: ${index}`);
    }
    this.#currentIndex = index;
  }

  // ========================================
  // RESET (controlled)
  // ========================================

  resetToStart() {
    this.#currentIndex = 0;
    // Does NOT clear segments - preserves mission history
    // Animation state is cleared separately by reset() function
  }

  clear() {
    this.#segments = [];
    this.#currentIndex = 0;
  }
}
```

**Usage rules:**
- Never expose `#segments` array directly
- All mutations go through `append()`
- Index changes go through `setCurrent()` or `resetToStart()`
- UI reads via `getSegment()`, `current()`, `count()`

---

## What Gets Removed / Replaced

This is a complete rewrite. The following will be deleted and replaced:

**Classes to Remove:**
1. `segmentedImport` class - replaced by unified MissionReplay

**State Variables to Remove:**
2. `missionState.fullSolutionsPerSegment` - use segment.trajectories
3. `missionState.seg0FullSolution` - use segment 0
4. `missionState.droneLostAtSegment` - check drone_configs.enabled
5. `segment.lostDrones` array - check drone_configs.enabled

**Code Paths to Remove:**
6. All `if (segmentedImport.isActive())` branches - unified code path
7. `buildCombinedRoutesFromSegments()` reconstruction - trajectories already combined
8. Separate animation paths for imported vs authored missions
9. Multiple trajectory concatenation points
10. Drone config reconstruction from multiple sources

---

## Migration Path

### Phase 1: New Segment Structure
- Create new Segment class with full data structure
- Write converters: old segment format → new format
- Write tests for new structure

### Phase 2: Accept Updates
- Accept builds new segment format
- Full trajectory = previous segment's full + new solver result

### Phase 3: Animation Updates
- Reset loads from segment 0
- Segment switch loads from segment N
- Remove reconstruction logic

### Phase 4: Import Updates
- Import JSON directly into new segment format
- Remove segmentedImport class

### Phase 5: Cleanup
- Remove dead code
- Remove redundant state
- Update export to match new format

---

## JSON Export Format

```json
{
  "version": "2.0",
  "segments": [
    {
      "index": 0,
      "timestamp": 1706200000000,
      "drone_configs": {
        "1": {
          "enabled": true,
          "fuel_capacity": 100,
          "fuel_remaining": 100,
          "start_airport": "A1",
          "end_airport": "A1",
          "target_access": {}
        }
      },
      "waypoints": {
        "airports": [{"id": "A1", "x": 10, "y": 10}],
        "synthetic_starts": {}
      },
      "targets": {
        "frozen": [],
        "active": [{"id": "T1", "x": 50, "y": 30, "priority": 5}]
      },
      "sams": [{"id": "S1", "pos": [30, 30], "range": 8}],
      "trajectories": {
        "1": {
          "render_full": [[10,10], [25,20], [50,30], [10,10]],
          "delta": [[10,10], [25,20], [50,30], [10,10]],
          "frozenEndIndex": -1,
          "route": ["A1", "T1", "A1"],
          "deltaDistance": 45.2,
          "endState": {
            "position": [10, 10],
            "fuel_remaining": 54.8
          }
        }
      },
      "startDist": 0,
      "endDist": 45.2,
      "cutPositionsAtEnd": {"1": [50, 30]}
    },
    {
      "index": 1,
      "startDist": 45.2,
      "endDist": 110.0,
      "cutPositionsAtEnd": null,
      "waypoints": {
        "airports": [{"id": "A1", "x": 10, "y": 10}],
        "synthetic_starts": {
          "1": {"id": "D1_START", "x": 50, "y": 30}
        }
      },
      "trajectories": {
        "1": {
          "render_full": [[10,10], [25,20], [50,30], [60,40], [10,10]],
          "delta": [[50,30], [60,40], [10,10]],
          "frozenEndIndex": 2,
          "route": ["D1_START", "T2", "A1"],
          "deltaDistance": 64.8,
          "endState": {
            "position": [10, 10],
            "fuel_remaining": 0
          }
        }
      }
    }
  ]
}
```

---

## Success Criteria

**Data Model:**
- [ ] Each segment is self-contained - can display without other segments
- [ ] `drone_configs` is the single source for enabled/disabled
- [ ] Explicit `startDist`/`endDist` boundaries (no ambiguous cutDistance)
- [ ] Dual trajectory storage: `render_full` (display) + `delta` (animation)
- [ ] Disabled drones have empty delta, empty route, zero deltaDistance

**Animation:**
- [ ] Animator uses ONLY `delta`, never `render_full`
- [ ] Segment switch triggered by CROSSING detection, not "being beyond"
- [ ] Atomic switch gate prevents re-entrant switches
- [ ] Reset hard-clears ALL animation state (globalDist=0, per-drone indices=0)
- [ ] Never call segment-switch logic during reset

**Operations:**
- [ ] No `segmentedImport.isActive()` checks - unified code path
- [ ] No trajectory reconstruction during animation
- [ ] Accept validates segment before adding
- [ ] Concatenation de-duplicates splice points
- [ ] SAM engulfment calculates precise escape point

**MissionReplay:**
- [ ] Single source of truth for segments
- [ ] Private internal arrays, controlled mutations via API
- [ ] `append()` validates monotonic boundaries
