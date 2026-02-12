# Expected Behaviors - ISR Mission Planner

This document defines the **expected user-facing behaviors** for the ISR Mission Planner. Use this as the source of truth when debugging or implementing features.

---

# PART A: CREATION WORKFLOW (Building a New Segmented Mission)

---

## A1. Initial State

### Starting Point
User creates an environment OR imports an N1 (non-segmented) JSON file.

### Display
- Environment visible: airports, targets, SAMs
- No trajectories yet
- Drones at starting airports

---

## A2. Run Planner (Segment 0)

### Action
User clicks "Run Planner"

### Expected Display
- Solver computes trajectory for segment 0
- **Segment 0's full trajectory** appears (airport → targets → airport)

### Next User Action
- **Option A**: Edit/optimize the solution (optional)
- **Option B**: Click **Accept** to commit

---

## A3. Accept (Segment 0 - No Cut Yet)

### Action
User clicks "Accept" after seeing/optimizing segment 0's trajectory

### Expected Display
- Trajectory saved as segment 0
- **NO checkmarks** on targets yet (user hasn't animated)
- **NO diamond** yet (no cut has happened)
- Animate button becomes active

### Next User Action
Click **Animate** to start the mission

---

## A4. Animate and Cut

### Action
User clicks "Animate", watches drones fly, then clicks "Cut" at desired position

### At Cut:
1. **Animation pauses** at cut position
2. **Diamond marker** appears at cut position (C1)
3. **Everything before the cut becomes FROZEN**:
   - Trajectory up to cut is frozen (unchangeable)
   - Visited targets are frozen with checkmarks
4. **Animate button greys out** (cannot animate until next Run Planner + Accept)
5. User can now **edit the environment** for the next segment:
   - Add/remove targets (only UNFROZEN ones - after the cut)
   - Add/remove SAMs
   - Disable drones
   - **Cannot modify frozen elements** (anything before the cut)

### Suggestion
Consider auto-activating the Editor on Cut to save the user a step.

---

## A5. Run Planner (Segment 1)

### Action
User clicks "Run Planner" after making environment changes

### Expected Display
- Solver computes trajectory for segment 1 (from C1)
- **Segment 1's full trajectory** appears (C1 → targets → airport)
- **Frozen elements remain**: Seg-0 frozen trajectory, visited targets with checkmarks

### Next User Action
Click **Accept** to commit segment 1

---

## A6. Accept (Segment 1)

### Action
User clicks "Accept"

### Expected Display
- Segment 1 trajectory saved
- **NO new checkmarks** (user hasn't animated this segment yet)
- Animate button becomes active again

### Next User Action
Click **Animate** to continue, optionally **Cut** again for more segments

---

## A7. Repeat Until Done

The cycle continues:
1. **Animate** → watch drones fly
2. **Cut** (optional) → freezes everything before cut, enables editing
3. **Edit environment** (optional) → add/remove targets/SAMs
4. **Run Planner** → get new trajectory
5. **Accept** → save segment
6. Repeat or let mission complete

---

## A8. Button State Rules

| State | Animate | Cut | Run Planner | Accept |
|-------|---------|-----|-------------|--------|
| After Accept (no animation yet) | ✓ Active | Disabled | Disabled | Disabled |
| During Animation | Disabled | ✓ Active | Disabled | Disabled |
| After Cut | Disabled | Disabled | ✓ Active | Disabled |
| After Run Planner | Disabled | Disabled | ✓ Active | ✓ Active |

**Key Rule**: Cut greys out Animate. User cannot cut again until they Run Planner and Accept.

---

## A9. What Can Be Modified After Cut

| Element | Can Modify? |
|---------|-------------|
| Frozen trajectory (before cut) | NO |
| Frozen/visited targets | NO |
| Cut position (diamond) | NO |
| Active targets (after cut) | YES - add/remove |
| SAMs | YES - add/remove/move |
| Drone enabled state | YES - disable drones |

---

# PART B: REPLAY WORKFLOW (Importing a Pre-Segmented JSON)

---

## B1. Import Segmented Mission JSON

### Action
User clicks "Import" and selects a V2 segmented JSON file (e.g., `isr_v2_*.json`)

### Expected Display
- **Segment 0's ENVIRONMENT only** is shown (airports, targets, SAMs)
- **NO trajectory** displayed yet - user must Run Planner
- Only segment 0's targets are visible
- Drone markers appear at their starting positions (airports)
- Cut point markers (white diamonds) are NOT shown yet

### What Should NOT Happen
- Any trajectories visible (neither segment 0 nor any other)
- Spider web of all trajectories from all segments
- All targets from all segments visible at once

### Next User Action
Click **Run Planner** to compute segment 0's trajectory

---

## B2. Run Planner (After Import - Segment 0)

### Action
User clicks "Run Planner" after importing a segmented JSON

### Expected Display
- Solver computes trajectory for segment 0
- **Segment 0's full trajectory** appears (airport → targets → airport)
- Trajectory shows the complete plan for segment 0

### Next User Action
- **Option A**: Click **Accept** to commit this solution and move to segment 1
- **Option B**: Edit/optimize the solution before accepting (see Section B2a)

---

## B2a. Edit/Optimize Before Accept (Optional)

### User Can Modify the Solution
Before clicking Accept, user may:
- Run optimization tools (Insert Missed, Swap Closer, No-Cross, etc.)
- Manually adjust parameters
- Re-run planner with different settings

### Key Rule: Accept Saves CURRENT State
- **What gets saved**: The trajectory/solution visible at Accept time
- **NOT saved**: The original solver output (if user modified it)
- The segmented JSON stores the ACCEPTED solution, not the initial solve

### Example Flow
1. Run Planner → see initial trajectory
2. Click "Swap Closer" → trajectory updates
3. Click "No-Cross" → trajectory updates again
4. Click **Accept** → THIS optimized trajectory is saved to segment 0
5. Move to segment 1

---

## B3. Accept (After Run Planner - Segment 0)

### Action
User clicks "Accept" after seeing segment 0's trajectory

### Expected Display
- **White diamond marker** appears at C1 (cut position)
- **Segment 0's trajectory becomes FROZEN** (always visible, in the past)
- **Segment 1's ENVIRONMENT** loads:
  - Segment 1's active targets appear
  - SAM changes applied (if any)
- **NO segment 1 trajectory yet** - user must Run Planner again

### Key Rule: Frozen Elements Are NEVER Removed
- Frozen trajectories (from previous segments) always remain visible
- Frozen targets (visited in previous segments) always remain visible with checkmarks
- These represent the PAST - they never disappear

### Next User Action
Click **Run Planner** to compute segment 1's trajectory

---

## B4. Run Planner (Segment 1)

### Action
User clicks "Run Planner" after accepting segment 0

### Expected Display
- Solver computes trajectory for segment 1
- **Segment 1's full trajectory** appears (C1 → targets → airport)
- **Segment 0's frozen trajectory remains visible** (it's in the past)
- **Frozen targets from segment 0 remain visible** with checkmarks

### Next User Action
Click **Accept** to commit segment 1 and move to segment 2

---

## B5. Accept (Segment 1) → Repeat Pattern

### Action
User clicks "Accept" after seeing segment 1's trajectory

### Expected Display
- **White diamond marker** appears at C2 (cut position)
- **Segment 2's ENVIRONMENT** loads
- Repeat pattern until all segments are processed

---

## B6. Reset (After Workflow Complete or During Animation)

### Action
User clicks "Reset" button

### Expected Display
- **Segment 0's FULL environment WITH trajectory** displayed
- Segment 0's full trajectory visible (airport → targets → airport)
- All visited target checkmarks cleared
- Drone positions reset to starting airports
- Animation stops if running
- Cut point markers (white diamonds) cleared
- Ready to Animate from the beginning

### Technical Note
- Animation state fully cleared: `globalDist=0`, all drone `distanceTraveled=0`
- `visitedTargets` array cleared
- Loads segment 0's full state (environment + trajectory)

---

## B7. Animate (After Reset or After All Segments Accepted)

### Action
User clicks "Animate"

### Expected Display - During Segment 0
- Drones move along segment 0's trajectory
- Green checkmarks appear as targets are visited
- Speed slider controls animation speed

### At Cut Point C1 (Segment Transition)
1. **White diamond marker** appears at C1
2. **Drone pauses for ~1 second**
3. **Segment 1's environment loads** (new targets, SAM changes)
4. **Segment 0's UNFROZEN trajectory removed** and replaced with **Segment 1's FULL trajectory**
5. Drone continues moving along segment 1's trajectory
6. **Frozen elements remain**: Seg-0 frozen trajectory, visited targets with checkmarks

### At Cut Point C2, C3, etc.
Same pattern repeats:
1. Diamond appears
2. Drone pauses
3. New environment loads
4. Previous unfrozen trajectory replaced with new segment's full trajectory
5. Drone continues

### Key Concept: Trajectory REPLACEMENT at Cut Points
- The UNFROZEN portion of the previous segment's trajectory is removed
- Replaced with the NEW segment's full trajectory (Cn → targets → airport)
- Frozen trajectories (completed history) remain visible

---

## B8. Animation - Mission Complete

### Trigger
Last drone returns to airport

### Expected Display
- Animation stops
- All assigned targets show green checkmarks
- Final trajectory remains visible
- All cut point markers (white diamonds) visible
- Status shows "Mission Complete" or similar

---

---

# PART C: COMMON BEHAVIORS (Both Creation and Replay)

---

## C1. Disabled Drones During Workflow

### Scenario
A drone is disabled between segments (e.g., marked as "lost" at segment 2)

### Expected Display
- Disabled drone's trajectory up to the disable point remains visible (frozen)
- Disabled drone does NOT animate in subsequent segments
- Disabled drone's marker remains at its last position
- Other drones continue their missions normally

### Technical Note
- `drone_configs[droneId].enabled === false` for disabled drones
- Disabled drones have empty `delta`, empty `route`, `deltaDistance=0`

---

## C2. Export Segmented Mission

### Action
User clicks "Export" after creating/playing a segmented mission

### Expected Output
- JSON file with `version: "2.0"`
- Contains all segments with:
  - `trajectories` including `solver_full`, `render_full`, `delta`
  - `drone_configs` for each segment
  - `targets.frozen` and `targets.active`
  - `startDist` and `endDist` boundaries
- File can be re-imported and replayed identically

---

## C3. Target and SAM Visibility During Workflow

### Rule
Targets and SAMs are revealed/hidden progressively as user moves through segments.

### After Import (Segment 0)
- Only segment 0's targets are visible
- Only segment 0's SAMs are visible
- Other targets/SAMs are hidden

### After Accept (Segment N → N+1)
- **Added targets/SAMs**: Appear at segment N+1 and stay visible to the end
- **Removed/deleted targets**: Disappear at segment N+1 and NEVER reappear
- **Removed/deleted SAMs**: Disappear at segment N+1 and NEVER reappear
- **Frozen targets (visited)**: ALWAYS remain visible with checkmarks

### Key Rules

**For ADDED elements:**
- Show up at the segment they were added
- Stay visible through all future segments and to the end
- ✓ Already working correctly

**For REMOVED/DELETED elements:**
- Disappear at the segment they were removed
- **NEVER reappear** in future segments or at the end
- This is currently a bug that needs fixing

**For FROZEN (visited) targets:**
- Once visited, stays visible with green checkmark forever
- Frozen = past = permanent

---

## C4. Cut Point Markers (White Diamonds)

### When They Appear
- Appear at **Accept** time for the segment just committed
- Positioned at cut position coordinates

### When They Disappear
- Cleared on Reset
- Cleared when starting a new mission/import

### Visual
- White diamond shape
- Positioned at `cutPositionsAtEnd` coordinates

---

## C5. Frozen Elements (Critical Rule)

### Frozen = Past = NEVER Removed

Once something becomes "frozen", it represents completed history and **must always remain visible**:

| Element | When Frozen | Display |
|---------|-------------|---------|
| Trajectory | After Accept | Gray/dimmed polyline, always visible |
| Target | After visited | Green checkmark, always visible |
| Cut Diamond | After Accept | White diamond at cut position |

### Why This Matters
- User needs to see the complete mission history
- Removing frozen elements would be confusing ("where did my trajectory go?")
- Animation replay depends on frozen elements being present

### What Gets Reset
Only on **Reset** or **New Import** do frozen elements clear:
- All trajectories cleared
- All target checkmarks cleared
- All cut diamonds cleared
- Returns to segment 0 environment only

---

## C6. Trajectory Types Reference

| Type | Purpose | When Used |
|------|---------|-----------|
| `solver_full` | Complete trajectory for ONE segment (cut → targets → airport) | Display on segment switch |
| `render_full` | Cumulative trajectory from mission start | Distance calculations, polyline length |
| `delta` | De-duplicated portion for animation | Animation loop movement |

### Display Rule
- On import/reset: Show segment 0's `solver_full`
- On segment switch: Replace with new segment's `solver_full`
- Never show cumulative `render_full` directly to user

---

## C7. Common Bug Patterns to Avoid

### Bug: Spider Web Trajectory on Import
- **Symptom**: All segments' trajectories visible immediately after import
- **Cause**: Loading trajectories instead of environment-only
- **Fix**: Import should load segment 0's ENVIRONMENT only (no trajectories)

### Bug: Trajectory Visible Before Run Planner
- **Symptom**: Trajectory appears immediately after Import or Accept
- **Cause**: Trajectory being displayed when only environment should show
- **Fix**: Trajectory only appears AFTER Run Planner, not on Import/Accept

### Bug: Truncated Segment Trajectory
- **Symptom**: After Run Planner, shows trajectory C1→C2 instead of C1→airport
- **Cause**: V2 system received truncated trajectory instead of full
- **Fix**: Pass full solution to solver/display

### Bug: Environment Not Changing at Accept
- **Symptom**: Accept doesn't show new targets or SAM changes
- **Cause**: Not loading next segment's environment
- **Fix**: Accept must load segment N+1's environment (targets, SAMs)

### Bug: Cut Markers Appear Before Accept
- **Symptom**: Diamonds visible before user accepts the segment
- **Cause**: Drawing cut positions too early
- **Fix**: Diamond only appears AFTER Accept

### Bug: Frozen Trajectories/Targets Disappear
- **Symptom**: Previous segment's trajectory or visited targets vanish
- **Cause**: Clearing/replacing frozen elements when loading new segment
- **Fix**: Frozen elements are NEVER removed (except on Reset/Import)

### Bug: Removed Targets Reappear
- **Symptom**: Targets deleted in segment N reappear in later segments or at mission end
- **Cause**: Using global target list instead of per-segment target list
- **Fix**: Each segment stores its own target list; removed targets stay removed

---

## Quick Reference Card

| Action | Display |
|--------|---------|
| **Import** | Seg-0 environment only (NO trajectory) |
| **Run Planner** | Seg-0 full trajectory appears |
| *(optional)* Edit/Optimize | Modify trajectory before commit |
| **Accept** | Saves CURRENT trajectory, Diamond at C1, Seg-1 environment loads (no new trajectory) |
| **Run Planner** | Seg-1 full trajectory appears (frozen Seg-0 remains) |
| *(optional)* Edit/Optimize | Modify trajectory before commit |
| **Accept** | Saves CURRENT trajectory, Diamond at C2, Seg-2 environment loads |
| ... | Repeat until all segments done |
| **Reset** | Seg-0 full environment WITH trajectory, ready to animate |
| **Animate** | Drone moves; at each Cn: pause, diamond, new env, REPLACE trajectory |

---

## Test Checklist

Before any release, verify:

1. [ ] Import shows segment 0's ENVIRONMENT only (no trajectory)
2. [ ] Run Planner shows segment 0's full trajectory
3. [ ] Accept shows diamond at C1 + segment 1's environment (no new trajectory yet)
4. [ ] Run Planner shows segment 1's full trajectory (C1 → airport), frozen Seg-0 visible
5. [ ] **Reset** returns to segment 0's full environment WITH trajectory
6. [ ] **Animate at C1**: drone pauses ~1sec, diamond appears, Seg-1 env loads, trajectory REPLACED
7. [ ] At C1: Seg-0 unfrozen trajectory removed, Seg-1 full trajectory appears
8. [ ] Frozen trajectories and targets remain visible throughout animation
9. [ ] **Added targets/SAMs** appear at their segment and stay visible to end
10. [ ] **Removed targets/SAMs** disappear at their segment and NEVER reappear
11. [ ] Export/import round-trip produces identical workflow
12. [ ] Disabled drones stop animating but keep frozen trajectory
