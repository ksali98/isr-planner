# Segmented Mission Rewrite - Handoff Notes

## Problem Solved
Disabled drones (D1, D4) were coming back enabled after pressing Accept during segmented mission workflow. The root cause was multiple places in the code resetting `state.droneConfigs` from stored segment data instead of preserving the user's UI choices.

## Solution: Clean Architecture Rewrite

### New File: `webapp/segmented_mission.js`
Contains the clean implementation with these classes/functions:
- `SegmentedMissionManager` - Single source of truth for segment data
- `MissionReplay` - Manages multi-segment playback
- `Segment` - Immutable data structure for one solved segment
- `exportSegmentedMission()` - Clean export function
- `importSegmentedMission()` - Clean import function
- `buildSolverEnv()` - Always uses state.droneConfigs as source of truth
- `acceptSolutionClean()` - Never modifies state.droneConfigs
- `syncUiWithDroneConfigs()` - Called ONCE after import

### Modified: `webapp/isr.js`
- `SegmentedImportManager` class (lines 494-745) rewritten as a **wrapper** that delegates to `window.SegmentedMissionManager`
- Added getter/setter for `_currentSegmentIndex` to proxy legacy direct access
- Fixed droneConfigs reset in "all segments complete" branch (line 1880)

### Modified: `webapp/index.html`
- Added `<script src="/static/segmented_mission.js">` before isr.js (line 507)

## Key Design Principle
**`state.droneConfigs` is ALWAYS the source of truth for UI:**
1. Solve READS from `state.droneConfigs`, never modifies it
2. Accept SAVES `state.droneConfigs` to segment, never modifies `state.droneConfigs`
3. Export READS each segment's saved droneConfigs
4. Import LOADS segment 0's droneConfigs into `state.droneConfigs` once

## Testing Checklist
1. Load a segmented mission JSON
2. Disable D1 and D4 in UI before first Solve
3. Press Solve - verify D1/D4 excluded from solution
4. Press Accept - verify D1/D4 stay disabled in UI
5. Repeat Solve/Accept for remaining segments
6. Verify D1/D4 never come back enabled

## Files Changed (commit 0194889)
```
webapp/segmented_mission.js  (NEW - 792 lines)
webapp/index.html            (1 line added)
webapp/isr.js                (refactored ~400 lines)
```

## Railway Deployment
Changes pushed to `main` branch. Railway auto-deploys from main, so the production site should have these changes within a few minutes.
