# ISR Planner UI State Machine - Implementation Progress

## Status: Step 1 Complete - UI State Machine Implemented

Last updated: 2024-12-17

## Feature Goal (Segmented Mission Replanning)
Allow users to:
1. Run Planner → generates drone trajectories (draft solution)
2. Accept Solution → commit the solution
3. Animate → drones move along trajectories
4. Press **P** during animation → pause (resumable)
5. Press **C** during animation → cut/checkpoint (truncates future)
6. Edit environment (targets/SAMs/airports)
7. Accept Edits → commit environment changes
8. Run Planner again → compute new plan from checkpoint
9. Accept Solution → commit the new segment
10. Animate → continue with new trajectory
11. Press **R** → reset mission to initial state

## What's Been Implemented (Step 1)

### MissionMode State Machine
7 states defined in `MissionMode`:
- `IDLE` - No committed plan exists. Can edit or solve.
- `EDITING_ENV` - User is editing. Must Accept Edits to proceed.
- `DRAFT_READY` - Draft solution exists. Must Accept or Discard.
- `READY_TO_ANIMATE` - Committed plan exists. Can animate.
- `ANIMATING` - Drones are moving. Can Pause (P) or Cut (C).
- `PAUSED_MID_ANIMATION` - Paused but resumable. Can resume or edit.
- `CHECKPOINT` - Cut occurred. Past locked. Must replan.

### State Variables (`missionState`)
```javascript
const missionState = {
  mode: MissionMode.IDLE,
  draftEnv: null,              // Editable environment shown in editor
  acceptedEnv: null,           // Committed environment for solving
  draftSolution: null,         // Last solver output (not yet accepted)
  committedSegments: [],       // Array of accepted solutions
  currentSegmentIndex: -1,     // Current segment being animated
  pauseContext: null,          // Saved state for resume
};
```

### UI Gating (`getUiPermissions()`)
Returns permissions object based on current mode:
- `canAnimate`, `canResume`, `canPause`, `canCut`
- `canEnterEdit`, `canAcceptEdits`
- `canSolve`, `canOptimize`, `canAcceptSolution`, `canDiscardDraft`
- `canReset`
- `isAnimating`, `isEditing`, `hasDraftSolution`, `hasCommittedPlan`

### Status Banner
- HTML element added to index.html with id `status-banner`
- Shows current mode and available actions
- Color-coded by state (green=animating, orange=editing, blue=draft, etc.)
- Action buttons: Accept Edits, Accept Solution, Discard

### Action Handlers
- `enterEditMode()` - Save draft env, transition to EDITING_ENV
- `acceptEdits()` - Commit draft env, transition to IDLE or CHECKPOINT
- `cancelEdits()` - Restore draft env, revert state
- `acceptSolution()` - Commit draft solution as segment, transition to READY_TO_ANIMATE
- `discardDraftSolution()` - Clear draft, restore previous state
- `resetMission()` - Full reset to initial state
- `pauseAnimation()` - Save context, transition to PAUSED
- `resumeAnimation()` - Restore context, transition to ANIMATING
- `cutAtCheckpoint()` - Freeze positions, transition to CHECKPOINT

### Keyboard Handlers Updated
- **C** - Cut at checkpoint (during animation)
- **P** - Pause animation (during animation)
- **R** - Reset mission (when not animating/editing)

### Run Planner Updated
- Now stores result as `draftSolution` (two-phase commit)
- Transitions to `DRAFT_READY` instead of directly applying
- User must click "Accept Solution" to commit

## Files Modified

- `webapp/isr.js` - State machine, handlers, UI updates
- `webapp/index.html` - Status banner HTML, action buttons
- `webapp/isr.css` - Status banner styles for each mode

## Cache Busting

Current version string: `20251217statemachine2`
Update in `webapp/index.html` when making JS/CSS changes.

## Testing Workflow

1. Load environment file
2. Verify status banner shows "Ready. Load environment, edit, or Run Planner."
3. Run Planner
4. Verify status shows "Draft solution ready..." with Accept/Discard buttons
5. Click Accept Solution
6. Verify status shows "Solution accepted. Click Animate..."
7. Click Animate
8. Verify status shows "Animating. Press Pause to stop, or C to cut..."
9. Press P to pause
10. Verify status shows "Paused. Click Animate to resume..."
11. Click Animate to resume
12. Press C to cut/checkpoint
13. Verify status shows "Checkpoint. Edit environment and Run Planner..."
14. Edit environment (add/move target)
15. Run Planner
16. Accept Solution
17. Animate to verify continuation
18. Press R to reset
19. Verify environment restored to initial state

## Next Steps (Step 2+)

1. **Segment History Display** - Show list of committed segments
2. **Mission Replay** - Replay all segments sequentially
3. **Replan Pause Markers** - Visual indicators where cuts occurred
4. **Target Visit Tracking** - Track which targets completed in each segment
