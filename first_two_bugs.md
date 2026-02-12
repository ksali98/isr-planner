# Summary of Segmented Mission Animation Bugs

## Bug 1: "Cannot cut in current state" after cutting at checkpoint
**Location:** `isr.js:7771` (approximate)
**Problem:** After cutting at a checkpoint, `checkpointSource` was set to `"replay_cut"` which blocked the solver from running.
**Fix:** Set `checkpointSource = null` instead of `"replay_cut"` to allow solving after cut.

---

## Bug 2: Double concatenation of frozen trajectory
**Location:** `runPlanner()` and `applyDraftSolutionToUI()`
**Problem:** When solving after a cut:
1. `runPlanner()` was prepending the frozen prefix to the solver's result
2. `applyDraftSolutionToUI()` was ALSO concatenating frozen + new
3. Result: trajectory went A → c1 → A → c1 → end (double prefix)

**Fix:** Remove the splice/concatenation logic from `runPlanner()`. Let the solver return segment-only trajectory, and only do concatenation in `applyDraftSolutionToUI()`.

---

## Bug 3: acceptSolution storing wrong trajectory
**Location:** `isr.js:1371-1390`
**Problem:** `acceptSolution()` was using `state.routes` (combined display trajectory) instead of `draftSolution.routes` (segment-only trajectory) for checkpoint replans.
**Fix:** Add check: `if (!isCheckpointReplan && state.routes...)` - for checkpoint replans, keep `draftSolution.routes` as-is.

---

## Bug 4: Wrong truncation distance in buildCombinedRoutesFromSegments
**Location:** `isr.js:6200-6225`
**Problem:** `cutDistance` is CUMULATIVE from mission start, but `split_polyline_at_distance()` measures from the START of each segment's trajectory. So truncating segment 1 at cutDistance 61.7 was wrong - it should be truncated at segment-relative distance (61.7 - 27.7 = 34.0).
**Fix:** Calculate segment-relative truncation: `segmentRelativeTruncation = nextCutDistance - thisCutDistance`

---

## Bug 5: buildCombinedRoutesFromSegments(0) not truncating segment 0
**Location:** `isr.js:6217`
**Problem:** The truncation condition `i < segmentCount - 1` meant when building only segment 0, it wouldn't truncate (because 0 < 0 is false).
**Fix:** Change condition to truncate whenever there's a next segment with cutDistance, regardless of iteration limit.

---

## Bug 6: Reset function using segmentedImport when it's not active
**Location:** `isr.js:2458-2560`
**Problem:** The Reset code checked `segmentedImport.isActive()` but segments created via the normal cut workflow don't use segmentedImport. So `missionReplay` had 4 segments but `segmentedImport.isActive()` was false.
**Fix:** Change condition to `if (resetSegCount > 1 || segmentedImport.isActive())`

---

## Bug 7: Environment vanishes after Reset
**Location:** `isr.js:2550`
**Problem:** `state.env = segmentedImport.getEnvForDisplay()` returns `null` when segmentedImport is not active, making the environment disappear.
**Fix:** Check `if (segmentedImport.isActive())` before calling `getEnvForDisplay()`, otherwise get env from `missionReplay.getSegment(0)`.

---

## Bug 8: Targets marked as visited before drone reaches them
**Location:** `isr.js:7258`
**Problem:** The proximity threshold for marking targets as visited was 20.0 units - too large. Targets near the cut point (but in seg-1) were being marked during seg-0 animation.
**Fix:** Reduce threshold from 20.0 to 5.0.

---

## Bug 9: Targets from later segments not showing after Reset
**Location:** `isr.js:2550-2565`
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

## Key Concepts to Understand

1. **Segment trajectory vs Combined trajectory**: Each segment stores only its portion (c_n → c_{n+1}). The combined trajectory for display is built by concatenating.

2. **cutDistance is cumulative**: Segment 1's cutDistance of 61.7 means 61.7 from mission START, not from segment 1's start.

3. **Frozen trajectory**: The portion already traveled should NEVER be removed. It persists throughout the mission.

4. **segmentedImport vs missionReplay**: `segmentedImport` is for imported JSON missions. Normal cut workflow uses `missionReplay` only, so always check `missionReplay.getSegmentCount() > 1` rather than relying solely on `segmentedImport.isActive()`.
