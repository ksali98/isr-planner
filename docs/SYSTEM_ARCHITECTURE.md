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

## Segmented Mission Mechanics (Detailed)

### Cut Workflow

**How a Cut is Triggered:**
1. User clicks the **"Cut"** button (usually during or after animation pause)
2. System marks current drone positions with **white diamond** markers (C1, C2, etc.)
3. User can now edit the environment (add/remove targets, disable drones, etc.)
4. User clicks **Solve** to plan from the cut positions
5. User clicks **Accept** to freeze the segment
6. Animation can continue from the cut

**Future Enhancement:** Automatic cuts triggered by environment changes

### What Gets Saved at a Cut

When a cut occurs, the following data is stored:

1. **Cut Positions**: Exact [x, y] coordinates for each active drone
2. **Cut Distance**: Total distance traveled when cut occurred (for animation sync)
3. **Frozen Trajectory**: The complete trajectory up to the cut point (IMMUTABLE forever)
4. **Environment Snapshot**: All targets, airports, SAMs, and drone configs at this moment
5. **Lost Drones**: List of drone IDs that were disabled at this cut

**Key Principle:** Everything before a cut is **frozen forever** and NEVER re-solved.

### Frozen Segments Behavior

- **Segment 0** trajectory from airport to Cut 1: FROZEN
- **Segment 1** trajectory from Cut 1 to Cut 2: FROZEN (after Accept)
- **Segment N** trajectory from Cut N to airport: Current/editable until Accept

When solving from a cut, the solver receives:
- Synthetic start nodes at cut positions
- Current environment state (possibly modified)
- Remaining fuel budgets
- Solver returns NEW trajectory from synthetic start to destination

The new trajectory is then stored as Segment N+1.

### New Drones (Added Mid-Mission)

**Behavior:**
- New drones **DO NOT** start from the cut position
- They **take off from their home airport** at the cut time
- Their trajectory is planned from airport → targets → airport
- During animation, they appear when their segment begins
- They catch up in real-time (not instant teleportation)

**Example:**
- Cut 1 happens at distance 50
- Drone 3 added in Segment 1
- Drone 3 starts from airport A2, flies its full path
- Animation shows Drone 3 taking off when Segment 1 starts

### Disabled Drones (Lost Drones)

**Behavior:**
- Drone is marked as disabled at the cut
- **RED diamond** marker appears at its final position
- Full trajectory up to cut point **remains visible** on map
- Drone does NOT appear in subsequent segments
- Recorded in `lostDrones` array for that segment

**Visual States:**
- White diamond (C1, C2...): Active cut position
- Red diamond: Disabled drone's final position

### Animation Across Segments

**Segment Transitions:**
1. Animation reaches a `cutDistance` marker
2. **1-second pause** to show the cut
3. Visual indicators:
   - White diamond markers appear
   - New targets appear (if added)
   - Disabled targets fade out
   - Lost drones show red diamond
4. Animation continues with next segment's trajectories

**Reset Behavior:**
- Returns to beginning (Segment 0 start)
- Clears all visited target markers
- Full mission replays from segment 0 through all segments

### Segmented JSON Export

**What Gets Saved:**
```json
{
  "version": "segmented_v1",
  "segments": [
    {
      "index": 0,
      "solution": { /* routes, trajectories */ },
      "cutPositions": { "1": [x,y], "2": [x,y] },
      "cutDistance": 45.5,
      "environment": { /* targets, airports, SAMs, drone_configs */ },
      "lostDrones": [],
      "visitedTargets": []
    }
  ]
}
```

**Every segment includes:**
- Complete environment configuration (targets, airports, SAMs)
- Full drone configurations (fuel, capabilities)
- Complete solution (routes, trajectories, sequences)
- Cut metadata (positions, distance)
- Lost/added drone tracking

This allows full replay without the original environment file.

---

## Agentic Reasoning and Constraints

### Philosophy: Leverage LLM Reasoning Power

The ISR Planner's Agentic mode is designed to **leverage the full reasoning capabilities of LLMs** rather than constraining them to a fixed set of operations.

**Key Principle:**
- The LLM is NOT limited to predefined constraint types
- The LLM can understand ANY mission requirement expressed in natural language
- The LLM directly manipulates allocations and uses solver tools to find optimal solutions
- The LLM iterates and reasons about tradeoffs

### How Agentic Mode Works

**1. Natural Language Understanding**
User expresses constraints in ANY form:
- "Drone 2 should visit the high-priority targets in the north"
- "Balance fuel usage across all drones"
- "Avoid having drones cross paths near the SAM zone"
- "Prioritize targets near airports for Drone 1"
- "Make sure all Type A targets are visited before Type B"

**2. LLM Reasoning**
The LLM:
- Understands the intent behind the constraint
- Analyzes the current mission state (targets, drones, fuel, SAMs)
- Reasons about which allocation changes would satisfy the constraint
- Considers multiple approaches and tradeoffs

**3. Direct Allocation Editing**
The LLM can:
- Assign targets to specific drones
- Reorder target sequences
- Move targets between drones
- Add/remove targets from routes
- Adjust priorities and preferences

**4. Tool Usage**
The LLM uses solving tools to:
- Test proposed allocations
- Run the solver with different configurations
- Compare fuel usage and points collected
- Iterate until finding a satisfactory solution

**5. Explanation**
The LLM explains its reasoning:
- "I assigned T5 to Drone 2 because it's closer and Drone 1 is already at max fuel"
- "I avoided crossing paths by sequencing Drone 1 clockwise and Drone 2 counterclockwise"

### Example Agentic Flow

```
User: "I want Drone 1 to focus on the eastern targets and Drone 2 on the western ones"
  ↓
LLM Reasoning:
  - Analyzes geographic distribution of targets
  - Identifies eastern targets: T1, T3, T5
  - Identifies western targets: T2, T4, T6
  - Considers fuel budgets for each drone
  ↓
LLM Actions:
  - Allocates {T1, T3, T5} → Drone 1
  - Allocates {T2, T4, T6} → Drone 2
  - Calls solver tool to generate optimized routes
  ↓
LLM Response:
  "I've split the targets geographically. Drone 1 will handle T1, T3, T5
   in the east (35 fuel, 120 points). Drone 2 will handle T2, T4, T6
   in the west (32 fuel, 110 points). This minimizes total distance
   and avoids conflicts."
```

### Constraint Types: Unlimited

Unlike traditional systems with fixed constraint vocabularies, the Agentic mode can handle:

- **Geographic constraints**: "northern targets", "targets near SAM zones"
- **Fuel constraints**: "under 50 fuel", "balanced fuel usage"
- **Priority constraints**: "high-priority first", "Type A before Type B"
- **Operational constraints**: "avoid crossing paths", "minimize total time"
- **Sequencing constraints**: "visit T1 before T2", "cluster nearby targets"
- **Capability constraints**: "only drones with long-range capability"
- **Novel constraints**: ANY requirement the user can express in natural language

### Supporting Tools and Infrastructure

**Constraint Parsing Helper** ([server/memory/constraints.py](server/memory/constraints.py)):
- Provides structured constraint operations (FORCE_VISIT, MOVE, REMOVE, SWAP, INSERT) as *shortcuts*
- These are NOT limitations - just convenient building blocks
- The LLM can use these or bypass them entirely
- Includes sequencing hints: start_with, end_with, priority_order

**Agent Tools** ([server/agents/tools/](server/agents/tools/)):
- Allocation editing tools
- Solver invocation tools
- Route analysis tools
- Fuel/distance calculation tools

**LangGraph Multi-Agent System** ([server/agents/isr_agent_multi_v4.py](server/agents/isr_agent_multi_v4.py)):
- 6 specialized agents collaborate
- Strategist: Understands high-level intent
- Mission Planner: Decomposes constraints
- Allocator: Edits target assignments
- Route Optimizer: Runs solver tools
- Critic: Evaluates solutions
- Responder: Explains decisions

### Heuristic Mode vs Agentic Mode

**Heuristic Mode:**
- User manually selects allocation strategy (GREEDY, BALANCED, etc.)
- User manually edits allocations via UI
- Direct tool invocation, no reasoning layer

**Agentic Mode:**
- User expresses intent in natural language
- LLM reasons about how to satisfy intent
- LLM manipulates allocations and invokes tools
- Iterative refinement with explanation

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

### Frontend (Webapp)
- **Main UI**: [webapp/isr.js](webapp/isr.js) - Monolithic 328KB frontend (canvas, animation, mission control)
- **Segment Manager**: [webapp/segment_manager.js](webapp/segment_manager.js) - Segment state management
- **Segmented Mission**: [webapp/segmented_mission.js](webapp/segmented_mission.js) - Alternative segment implementation
- **Segmented Import**: [webapp/isr.js](webapp/isr.js) (segmentedImport object) - JSON import handler

### Backend - Core Solving
- **Solver Bridge**: [server/solver/solver_bridge.py](server/solver/solver_bridge.py) - Main orchestration pipeline
- **Distance Matrix**: [server/solver/sam_distance_matrix.py](server/solver/sam_distance_matrix.py) - SAM-aware distances (ALWAYS used)
- **Orienteering Solver**: [orienteering_with_matrix.py](orienteering_with_matrix.py) - Held-Karp DP solver (≤12 targets optimal)
- **Greedy Solver**: [server/solver/greedy_solver.py](server/solver/greedy_solver.py) - Fast suboptimal solver (>12 targets)
- **Trajectory Planner**: [server/solver/isr_trajectory.py](server/solver/isr_trajectory.py) - SAM-avoiding path generation

### Backend - Allocation & Optimization
- **Target Allocator**: [server/solver/target_allocator.py](server/solver/target_allocator.py) - 5 allocation strategies
  - GREEDY, BALANCED, EFFICIENT, GEOGRAPHIC, EXCLUSIVE
- **Post-Optimizer**: [server/solver/post_optimizer.py](server/solver/post_optimizer.py) - 3 post-optimization algorithms
  - Insert Missed, Swap Closer (cascade), No-Cross (2-opt)

### Backend - Constraint System (DSL)
- **Constraints**: [server/memory/constraints.py](server/memory/constraints.py) - Full DSL with compiler
  - ConstraintProgram, ConstraintCompiler
  - Operations: FORCE_VISIT, MOVE, REMOVE, SWAP, INSERT
  - Sequencing hints: start_with, end_with, priority_order

### Backend - Agentic System
- **Multi-Agent v4**: [server/agents/isr_agent_multi_v4.py](server/agents/isr_agent_multi_v4.py) - LangGraph 6-agent system
  - Strategist, Mission Planner, Allocator, Route Optimizer, Critic, Responder
- **Coordinator v4**: [server/agents/coordinator_v4.py](server/agents/coordinator_v4.py) - Deterministic pre-pass
- **Agent Tools**: [server/agents/tools/](server/agents/tools/) - Tool definitions for LangGraph agents
- **Supabase Memory**: [server/memory/](server/memory/) - Optional agent memory persistence

### API Entry Point
- **FastAPI Server**: [server/main.py](server/main.py) - All API endpoints
  - `/api/solve` - Heuristic solving
  - `/api/agent/chat` - Agentic mode (LLM-powered)
  - `/api/insert_missed`, `/api/swap_closer`, `/api/crossing_removal` - Post-optimizers

---

*This document reflects the current system architecture as of January 2026. Proposed refactoring plans are detailed in [MODULAR_ARCHITECTURE.md](MODULAR_ARCHITECTURE.md).*
