# Checkpoint-Based Distance Matrix System

## Overview

This document describes the changes made to implement a checkpoint-based distance calculation system. The key principle is that the **SAM-aware distance matrix is the single source of truth** for all distance calculations.

## Problem Statement

Previously, when a mission was cut into segments:
- Per-drone synthetic starts (`D1_START`, `D2_START`) were created at cut positions
- Distance calculations sometimes used Euclidean fallbacks instead of SAM-aware distances
- The naming was confusing and didn't clearly indicate which segment boundary was being referenced

## Solution: Per-Drone Checkpoints

### Naming Convention

**Format:** `C{segment}-{drone}`

Examples:
- `C1-1` = Checkpoint 1 for Drone 1 (first cut, drone 1's position)
- `C1-2` = Checkpoint 1 for Drone 2 (first cut, drone 2's position)
- `C2-1` = Checkpoint 2 for Drone 1 (second cut, drone 1's position)

### Route Structure

For a 2-drone mission with 2 cuts:

**Seg-0:**
- D1: `A1 → T1 → T2 → C1-1`
- D2: `A2 → T3 → C1-2`

**Seg-1:**
- D1: `C1-1 → T5 → C2-1`
- D2: `C1-2 → T6 → C2-2`

**Seg-2:**
- D1: `C2-1 → T7 → A1`
- D2: `C2-2 → T8 → A2`

### Distance Calculation

All distances come from the SAM-aware distance matrix:
```
seg0_d1_distance = matrix[A1][T1] + matrix[T1][T2] + matrix[T2][C1-1]
seg1_d1_distance = matrix[C1-1][T5] + matrix[T5][C2-1]
seg2_d1_distance = matrix[C2-1][T7] + matrix[T7][A1]
```

## Files Modified

### Frontend (`webapp/isr.js`)

1. **State Variables:**
   - Added `pendingCheckpoints` - object keyed by drone ID containing checkpoint info
   - Added `checkpointSegmentCounter` - counter for segment numbers (1, 2, 3, ...)
   - Removed old `pendingCheckpoint` (single checkpoint) approach

2. **`freezeAtCheckpoint()` function:**
   - Now creates per-drone checkpoints with format `C{segment}-{drone}`
   - Each drone's checkpoint has its own x, y position
   - Increments `checkpointSegmentCounter` on each cut

3. **`buildCheckpointEnv()` function:**
   - Adds all per-drone checkpoints to `env.checkpoints` array
   - Sets each drone's `start_airport` to its own checkpoint ID (e.g., `C1-1`)
   - Falls back to legacy `D{n}_START` synthetic starts if no pending checkpoints

4. **`buildCombinedRoutesFromSegments()` function:**
   - Updated route filtering regex to match new format: `/^C\d+-\d+$/`
   - Filters out checkpoints from combined routes (they're segment boundaries, not mission waypoints)

5. **Reset/Clear locations:**
   - Updated all reset points to clear `pendingCheckpoints` and `checkpointSegmentCounter`

### Backend (`server/solver/`)

#### `solver_bridge.py`

1. **Added `import re`** for regex pattern matching

2. **Checkpoint detection:**
   - Updated regex pattern: `r'^C\d+-\d+$'` to match `C1-1`, `C2-3`, etc.
   - Both `solve_mission()` and `solve_mission_with_allocation()` updated

3. **`build_environment_for_solver()` function:**
   - Added handling for `env.checkpoints` array
   - Checkpoints added to airports list with `is_checkpoint: True` flag

4. **Valid end airport filtering:**
   - Checkpoints excluded from valid end airports (drones shouldn't end at checkpoints)
   - Filter checks both `is_checkpoint` flag and ID pattern starting with "C"

#### `sam_distance_matrix.py`

1. **`calculate_sam_aware_matrix()` function:**
   - Added handling for `env.checkpoints` array
   - Checkpoints added to airports list for matrix computation
   - Each checkpoint gets SAM-aware distances to all other waypoints

#### `target_allocator.py`

1. **`allocate_targets()` function:**
   - Added handling for checkpoints alongside synthetic starts
   - Checkpoints treated as airports for allocation purposes

### Backend (`server/main.py`)

1. **`/api/apply_sequence` endpoint:**
   - Uses SAM-aware distance matrix as primary source
   - Euclidean fallback only for waypoints missing from matrix (with warning log)

## Testing Checklist

- [ ] **Creation workflow:**
  1. Load environment
  2. Solve segment 0
  3. Animate and Cut
  4. Verify debug panel shows `C1-1`, `C1-2`, etc.
  5. Solve segment 1
  6. Verify routes start from checkpoints
  7. Verify distances use matrix values

- [ ] **Import workflow:**
  1. Import segmented mission JSON
  2. Verify checkpoints are recognized
  3. Animate full mission
  4. Reset and re-animate

## Key Principles

1. **Single Source of Truth:** The SAM-aware distance matrix is computed on every solve and contains distances for ALL waypoints (airports, targets, checkpoints)

2. **No Euclidean Fallbacks:** If a waypoint is missing from the matrix, it indicates a bug in the flow. Warnings are logged but should not occur in normal operation.

3. **Checkpoints Are Not Endpoints:** Checkpoints can be START points but never END points. Drones always end at real airports.

4. **Per-Drone Positions:** Each drone has its own checkpoint position because drones may be at different locations when cut happens.

## Backward Compatibility

The system maintains backward compatibility with:
- Legacy `D{n}_START` synthetic starts (fallback in `buildCheckpointEnv`)
- Old import files that use `synthetic_starts` instead of `checkpoints`

## Debug Logging

Key log messages to look for:
- `✂️ Created checkpoint C1-1 at [x, y]` - checkpoint creation
- `📍 [SAM Matrix] Added checkpoint: C1-1` - checkpoint in matrix
- `🔍 D1 DEBUG: Using CHECKPOINT start 'C1-1'` - solver using checkpoint
- `⚠️ Distance matrix fallback` - indicates fallback to Euclidean (should be rare)
