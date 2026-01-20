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

## File References

- Main UI: `webapp/isr.js`
- Segment Manager: `webapp/segment_manager.js`
- Server Solver: `server/solver/solver_bridge.py`
- Distance Matrix: `server/solver/sam_distance_matrix.py`

---

*This document is a work in progress. Please edit to clarify requirements and intended behavior.*
