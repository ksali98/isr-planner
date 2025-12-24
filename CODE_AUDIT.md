# ISR Planner - Code Audit & Issues

**Date**: 2024-12-23
**Purpose**: Comprehensive review of SAM validation, trajectory generation, and checkpoint logic

## Critical Bugs Identified

### 1. Target Inside SAM Zone Being Visited
**Symptom**: Solver includes a target that's inside a SAM zone in the route, but drone won't fly the trajectory.

**Impact**: Route is invalid from the start - wastes computation and confuses users.

**Root Cause Analysis**:
- [ ] Check: Does solver validate target positions against SAM zones?
- [ ] Check: Does target allocation filter out unreachable targets?
- [ ] Check: Is there SAM-aware distance calculation being used?

**Files to Investigate**:
- `server/solver/solver_bridge.py` - Target allocation logic
- `orienteering_with_matrix.py` - Solver target selection
- `server/solver/trajectory_planner.py` - SAM validation

### 2. Trajectory Termination During SAM Wrapping at Checkpoint
**Symptom**: When cut happens while trajectory is navigating around a SAM, the trajectory stops unexpectedly.

**Impact**: Partial trajectories, broken animations, incorrect checkpoint positions.

**Root Cause Analysis**:
- [ ] Check: How is trajectory truncated at checkpoint?
- [ ] Check: Does truncation respect SAM boundary navigation state?
- [ ] Check: Are partial arc trajectories being preserved correctly?

**Files to Investigate**:
- `webapp/isr.js` - Checkpoint cut logic
- `server/solver/trajectory_planner.py` - Trajectory generation
- `path_planning_core/boundary_navigation.py` - SAM avoidance

---

## Architecture Review

### Component Responsibilities (Current State)

#### 1. Orienteering Solver (`orienteering_with_matrix.py`)
**What it does**:
- Selects which targets to visit
- Orders targets for optimal route
- Uses distance matrix (may or may not be SAM-aware)

**What it SHOULD validate**:
- ❓ Target reachability (not inside SAMs)
- ❓ Path feasibility (enough fuel considering SAM avoidance)

**Current Issues**:
- Uses `use_sam_aware_distances=False` by default (disabled for speed)
- No validation that targets are outside SAM zones
- No validation that paths are actually flyable

#### 2. Target Allocation (`solver_bridge.py`)
**What it does**:
- Assigns targets to drones
- Filters by drone capabilities (target_access)
- Creates distance matrices

**What it SHOULD validate**:
- ❓ Targets are not inside SAM zones
- ❓ Targets are reachable considering fuel and SAMs

**Current Issues**:
- No SAM validation during allocation
- May allocate unreachable targets

#### 3. Trajectory Planner (`trajectory_planner.py`)
**What it does**:
- Generates SAM-avoiding paths between waypoints
- Uses boundary_navigation for tangent-arc-tangent paths
- Validates paths don't penetrate SAMs

**What it SHOULD do**:
- ✅ Generate SAM-safe paths
- ✅ Validate no SAM penetration
- ❓ Reject routes with unreachable targets

**Current Issues**:
- Only validates AFTER route is planned
- If segment is invalid, it breaks the trajectory silently
- No feedback to solver about infeasible routes

#### 4. Boundary Navigation (`boundary_navigation.py`)
**What it does**:
- Computes tangent-arc-tangent paths around SAM polygons
- Returns path waypoints and distance

**What it SHOULD do**:
- ✅ Avoid SAM boundaries
- ✅ Return valid paths or error

**Current Issues**:
- May return paths that are too long (exceed fuel)
- No coordination with orienteering solver's distance estimates

#### 5. Checkpoint Logic (`isr.js`)
**What it does**:
- Cuts trajectories at current drone positions
- Creates synthetic start positions
- Triggers replan from checkpoint

**What it SHOULD do**:
- ✅ Preserve completed trajectory segments
- ❓ Handle mid-arc SAM avoidance cuts
- ❓ Validate checkpoint positions are SAM-safe

**Current Issues**:
- May cut trajectories mid-arc around SAMs
- Truncation logic scattered across multiple functions
- No validation that checkpoint position is valid starting point

---

## Data Flow Analysis

### First Solve (No Checkpoint)
```
1. User loads environment (airports, targets, SAMs)
   ↓
2. Run Planner clicked
   ↓
3. solve_mission_with_allocation()
   ↓
4. Target allocation (NO SAM VALIDATION)
   ↓
5. Build distance matrix (use_sam_aware_distances=False)
   ↓
6. Orienteering solver picks targets (uses Euclidean distances)
   ↓
7. ISRTrajectoryPlanner.generate_trajectory()
   ↓
8. For each segment: plan_path() → boundary_navigation
   ↓
9. If path violates SAMs → segment dropped, trajectory ends early
   ↓
10. Return trajectory to frontend
```

**Problem**: Solver picks targets using Euclidean distance, but actual flyable distance may be much longer due to SAM avoidance. This causes:
- Invalid target selections (targets inside SAMs)
- Fuel constraint violations (actual path longer than estimated)

### Checkpoint Replan
```
1. Animation running
   ↓
2. User presses 'C' to cut
   ↓
3. cutAtCheckpoint() in isr.js
   ↓
4. Get current drone positions
   ↓
5. Truncate trajectories at current positions
   ↓
6. Create synthetic_starts at cut positions
   ↓
7. Run Planner (checkpoint mode)
   ↓
8. solve_mission_with_allocation() with synthetic_starts
   ↓
9. [SAME AS FIRST SOLVE - no special handling]
   ↓
10. New trajectory from checkpoint position
```

**Problem**: If cut happens mid-arc during SAM avoidance:
- Checkpoint position might be on the arc
- Truncation doesn't preserve SAM avoidance state
- New trajectory starts from arbitrary position

---

## Validation Gaps

### Pre-Flight Validation (MISSING)
- [ ] Check all targets are outside SAM zones
- [ ] Check all targets are reachable from start airports
- [ ] Check route distances account for SAM avoidance

### Post-Solve Validation (PARTIAL)
- [x] Trajectory planner validates each segment
- [ ] No overall route feasibility check
- [ ] No fuel constraint re-validation with actual paths

### Checkpoint Validation (MISSING)
- [ ] Check checkpoint position is SAM-safe
- [ ] Check checkpoint position allows valid path to next waypoint
- [ ] Check remaining targets are reachable from checkpoint

---

## Proposed Fixes (Priority Order)

### Phase 1: Immediate Safety Checks (Prevent Bad States)

#### Fix 1.1: Pre-Flight Target Validation
**File**: `server/solver/solver_bridge.py`

Add validation BEFORE solver runs:
```python
def validate_targets_outside_sams(targets, sams):
    """Validate that all targets are outside all SAM zones"""
    invalid_targets = []
    for target in targets:
        for sam in sams:
            dist = distance(target['pos'], sam['pos'])
            if dist < sam['range']:
                invalid_targets.append(target['id'])
                break
    return invalid_targets
```

Call before solving:
- Reject targets inside SAMs
- Log warning to user
- Continue with valid targets only

#### Fix 1.2: Post-Trajectory Validation
**File**: `server/solver/trajectory_planner.py`

Add validation AFTER trajectory is generated:
```python
def validate_trajectory_complete(route_ids, trajectory):
    """Ensure trajectory reaches all waypoints in route"""
    if not trajectory or len(trajectory) == 0:
        return False, "Empty trajectory"

    # Check trajectory has enough waypoints for the route
    # Add more sophisticated checks

    return True, ""
```

Reject routes with incomplete trajectories.

#### Fix 1.3: Checkpoint Position Validation
**File**: `webapp/isr.js`

Before creating checkpoint:
```javascript
function validateCheckpointPosition(droneId, position) {
    // Check position is not inside any SAM
    // Check position allows path to next waypoint
    // Return false if invalid
}
```

### Phase 2: Architectural Improvements

#### Fix 2.1: SAM-Aware Distance Matrix
**File**: `server/solver/solver_bridge.py`

Enable SAM-aware distances by default:
```python
use_sam_aware_distances=True  # Change default
```

But this is slow - need to:
- Cache distance matrices
- Only recompute when SAMs change
- Use faster SAM avoidance algorithm

#### Fix 2.2: Unified Validation Layer
Create new file: `server/solver/validation.py`

Central validation functions:
- `validate_environment(env)` - Check env is valid
- `validate_route(route, env)` - Check route is flyable
- `validate_checkpoint(position, env)` - Check checkpoint is safe

All components use this single source of truth.

#### Fix 2.3: Trajectory Truncation Fix
**File**: `webapp/isr.js`

Improve checkpoint truncation:
- Detect if cut happens during SAM avoidance
- If so, find safe position outside SAM arc
- Or complete the current SAM avoidance before cutting

### Phase 3: Long-Term Refactoring

#### Fix 3.1: Separation of Concerns
- Solver: Pure optimization (no SAM knowledge)
- Planner: SAM avoidance (converts routes to trajectories)
- Validator: Pre/post checks (single source of validation logic)

#### Fix 3.2: Contract Enforcement
- Clear interfaces between components
- Input/output validation at boundaries
- Error propagation to user

#### Fix 3.3: Test Coverage
- Unit tests for SAM validation
- Integration tests for full solve pipeline
- Regression tests for checkpoint scenarios

---

## Investigation Tasks

### Task 1: Target Inside SAM Bug
- [ ] Find the specific environment file that reproduces this
- [ ] Check target positions vs SAM positions
- [ ] Trace why solver selected this target
- [ ] Identify which validation is missing

### Task 2: Trajectory Truncation Bug
- [ ] Reproduce the SAM-wrapping cut scenario
- [ ] Log trajectory state at cut time
- [ ] Identify where trajectory ends prematurely
- [ ] Check boundary_navigation state preservation

### Task 3: Code Flow Documentation
- [ ] Map complete data flow for first solve
- [ ] Map complete data flow for checkpoint replan
- [ ] Document all validation points (or lack thereof)
- [ ] Create sequence diagrams

---

## Questions for Investigation

1. **Why is `use_sam_aware_distances=False`?**
   - Performance reasons?
   - Implementation issues?
   - Should this be configurable per-scenario?

2. **What happens if boundary_navigation fails?**
   - Does it return empty path?
   - Does it throw exception?
   - How does trajectory_planner handle this?

3. **Where exactly does trajectory get truncated at checkpoint?**
   - In cutAtCheckpoint()?
   - In startAnimation()?
   - In the trajectory planner?

4. **What is the expected behavior if a target is unreachable?**
   - Skip it?
   - Fail the entire solve?
   - Return partial route?

---

## Next Steps

1. Run specific tests to reproduce both bugs
2. Add extensive logging to trace data flow
3. Implement Phase 1 safety checks
4. Deploy to Railway with validation warnings
5. Plan Phase 2 architectural improvements

---

## Files to Review (Reading List)

Priority order for deep dive:

1. `server/solver/solver_bridge.py` - Central coordination
2. `server/solver/trajectory_planner.py` - SAM avoidance
3. `orienteering_with_matrix.py` - Target selection
4. `path_planning_core/boundary_navigation.py` - Path computation
5. `webapp/isr.js` - Checkpoint logic (cutAtCheckpoint, startAnimation)

---

## Notes

- Current system is "optimistic" - assumes everything will work
- No defensive validation layers
- Error handling is scattered and inconsistent
- SAM validation happens too late (after planning, during trajectory generation)
- Need "fail fast" approach with early validation
