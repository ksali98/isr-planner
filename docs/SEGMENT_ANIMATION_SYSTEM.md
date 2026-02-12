# Segment Animation System

This document describes how the segment-based mission animation system works, including drone configuration persistence, target visit tracking, and segment switching during animation.

## Overview

The ISR Planner supports **segmented missions** where a mission can be "cut" at checkpoints during animation. Each segment represents a portion of the mission with its own:
- Drone configurations (enabled/disabled state, fuel)
- Target assignments (which targets to visit)
- Trajectories (paths for each drone)
- Lost drones (drones disabled at this segment)

## Key Concepts

### 1. Segment Structure

Each segment stores everything needed to replay that portion of the mission:

```javascript
{
  index: 0,                    // Segment number (0-based)
  cutDistance: 32.0,           // Distance at which this segment was created
  drone_configs: {...},        // Drone enabled/disabled state at solve time
  solution: {
    routes: {
      "1": { trajectory: [[x,y]...], route: ["A1","T1",...] },
      ...
    }
  },
  lostDrones: ["1"],          // Drones disabled at this segment
  visited_targets: ["T1","T2"] // Targets visited in this segment
}
```

### 2. Drone Configuration Flow

The key principle: **UI is the source of truth at solve/accept time, segments are the source of truth during animation.**

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  UI Table   │ ──► │   Solve     │ ──► │   Accept    │
│ (checkboxes)│     │             │     │  (segment)  │
└─────────────┘     └─────────────┘     └─────────────┘
                                              │
                                              ▼
                                        ┌─────────────┐
                                        │   Segment   │
                                        │ drone_configs│
                                        └─────────────┘
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    ▼                         ▼                         ▼
              ┌─────────────┐           ┌─────────────┐           ┌─────────────┐
              │  Animation  │           │  Animation  │           │  Animation  │
              │  Segment 0  │           │  Segment 1  │           │  Segment 2  │
              └─────────────┘           └─────────────┘           └─────────────┘
```

**Key function: `readDroneConfigsFromUI()`**
- Reads enabled/disabled state directly from DOM checkboxes
- Called at solve time and accept time
- Stored in segment.drone_configs

**During animation segment switches:**
- Load drone_configs from the segment
- Update state.droneConfigs
- Call `refreshDroneConfigUI()` to update the UI table

### 3. Target Visit Tracking

Targets are tracked in `state.visitedTargets` array (contains target IDs like "T1", "T8").

**Drawing green checkmarks:**
```javascript
// In drawEnvironment()
const isVisited = state.visitedTargets && state.visitedTargets.includes(t.id);
if (isVisited) {
  // Draw green X over target
}
```

**Marking targets as visited during animation:**
```javascript
// In animation loop
if (closestIdx >= 0 && minDist < 15.0 && currentDistance >= targetDistance) {
  state.visitedTargets.push(wp);
}
```

Key thresholds:
- `minDist < 15.0` - Trajectory must pass within 15 units of target
- Sequential order check - Targets must be visited in route order

### 4. Segment Switching During Animation

When animation reaches a segment boundary (cutDistance), the system switches to the next segment:

**Critical: Merge visited targets on segment switch**
```javascript
// In advanceToNextSegmentV2()
// MERGE visited targets: keep animation-visited + add segment's frozen targets
const frozenIds = new Set(nextSeg.targets.frozen.map(t => t.id));
state.visitedTargets.forEach(id => frozenIds.add(id));  // Keep animation-visited
state.visitedTargets = Array.from(frozenIds);
```

This prevents targets visited during animation from being lost when loading a new segment.

## Common Issues and Solutions

### Issue: Disabled drones "come back to life"

**Cause:** Drone configs being read from wrong source during animation

**Solution:**
1. Read UI at solve/accept time → save to segment
2. During animation, always load from segment.drone_configs
3. Never merge with state.env.drone_configs (initial env)

### Issue: Targets don't get green checkmarks

**Cause 1:** Threshold too strict (was 5.0)
**Solution:** Increased to 15.0 units

**Cause 2:** visitedTargets being replaced instead of merged on segment switch
**Solution:** Merge sets instead of replace

### Issue: Green checkmarks appear all at once at the end

**Cause:** Sequential order check blocking later targets until earlier ones are marked

**Solution:** Ensure trajectory passes close enough to each target in sequence

### Issue: All targets appear at Reset instead of just segment 0's targets

**Cause:** `seg0.env.targets` contains ALL targets because the environment snapshot at accept time had all targets present

**Solution:** Filter display targets based on segment 0's solution routes:
```javascript
// In reset function
if (seg0.solution && seg0.solution.routes) {
  const seg0RouteTargets = new Set();
  Object.values(seg0.solution.routes).forEach(routeData => {
    const route = routeData.route || [];
    route.forEach(stop => {
      if (stop && stop.startsWith('T')) {
        seg0RouteTargets.add(stop);
      }
    });
  });
  // Filter to only targets in segment 0's routes
  state.env.targets = allEnvTargets.filter(t => seg0RouteTargets.has(t.id));
}
```

**Important distinction:**
- `state.initialEnvSnapshot.targets` = ALL targets (for animation to reference when marking visited)
- `state.env.targets` = Only current segment's route targets (for display)

## Implementation Details

### Key Files

- `webapp/isr.js` - Main application with animation logic
- `webapp/segment_v2/` - V2 segment system (self-contained segments)
  - `Segment.js` - Segment validation
  - `SegmentBuilder.js` - Building segments from solver results
  - `MissionReplay.js` - Segment navigation and animation state
  - `utils.js` - Shared utilities

### Key Functions

| Function | Purpose |
|----------|---------|
| `readDroneConfigsFromUI()` | Read drone enabled state from DOM |
| `refreshDroneConfigUI()` | Update DOM from state.droneConfigs |
| `acceptSolution()` | Save segment with drone_configs |
| `advanceToNextSegmentV2()` | Switch segments during animation |
| `loadSegmentV2()` | Load segment for initial display |
| `drawEnvironment()` | Draw targets with visited checkmarks |

### Animation Loop Flow

```
1. Start animation
   ├── Load segment 0 routes
   ├── Initialize drone animation state
   └── Start requestAnimationFrame loop

2. Each frame:
   ├── Update drone positions based on distanceTraveled
   ├── Check if any targets should be marked visited
   ├── Check if segment boundary reached
   │   └── If yes: advanceToNextSegmentV2()
   └── drawEnvironment(true) // Redraw with checkmarks

3. Segment switch:
   ├── Merge visitedTargets (keep animation-visited + frozen)
   ├── Load drone_configs from new segment
   ├── Update UI table
   └── Extend trajectories for active drones
```

## Testing Checklist

When testing segment system changes:

1. **Disabled drone persistence**
   - Disable a drone at segment 1
   - Verify it stays disabled in segments 2, 3, etc.
   - Reset and replay - verify same behavior

2. **Target checkmarks**
   - Watch animation - targets should get checkmarks as drone passes
   - Checkmarks should appear progressively, not all at once
   - After segment switch, previous checkmarks should remain

3. **Animation replay**
   - Reset mission and replay
   - All targets should be re-visited with checkmarks
   - Drone configs should match what was set during original solve

4. **Export/Import**
   - Export segmented mission
   - Import and replay
   - Verify drone configs and target visits match original

5. **Progressive target reveal**
   - After Reset, only segment 0's targets should be visible
   - When animation crosses into segment 1, segment 1's targets should appear
   - Each segment reveals its own targets as the drone enters it
