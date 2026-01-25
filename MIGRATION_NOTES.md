# Migration Notes

This document tracks changes made during the hybrid rebuild of the ISR Planner.

## Migration Strategy

Following the hybrid approach outlined in `docs/TASKS_HYBRID.md`:
- **[MIGRATE]** - Copy existing working code, integrate into new architecture
- **[REFACTOR]** - Existing code works but needs simplification/cleanup
- **[BUILD]** - Write new code from scratch
- **[COPY]** - Copy existing code as-is with minimal changes

---

## Phase 1: Project Foundation

### Completed
- [x] Project on clean branch
- [x] All foundational files verified:
  - `requirements.txt` - Python dependencies
  - `.env.example` - Environment template
  - `run_planner.sh` - Server startup script
  - `.gitignore` - Git ignore rules
  - `orienteering_with_matrix.py` - Held-Karp DP solver at project root (required location)

### Notes
- Solver file must remain at project root for proper imports
- All documentation files from `docs/` directory preserved

---

## Phase 2: Core Solver (MIGRATED)

All solver components verified in place:

| Component | File | Status | Notes |
|-----------|------|--------|-------|
| SAM Distance Matrix | `server/solver/sam_distance_matrix.py` | Migrated | SAM collision detection, tangent paths |
| Orienteering Solver | `orienteering_with_matrix.py` | Migrated | Held-Karp DP, fuel constraints |
| Target Allocator | `server/solver/target_allocator.py` | Migrated | 5 strategies: GREEDY, BALANCED, EFFICIENT, GEOGRAPHIC, EXCLUSIVE |
| Trajectory Planner | `server/solver/trajectory_planner.py` | Migrated | SAM-avoiding trajectories |
| Post Optimizer | `server/solver/post_optimizer.py` | Migrated | Insert Missed, Swap Closer, No-Cross |
| Solver Bridge | `server/solver/solver_bridge.py` | Migrated | Main orchestration (needs simplification) |

---

## Phase 3: API Server Refactoring

### Goal
Break monolithic `server/main.py` (2,352 lines) into modular routers.

### New Structure
```
server/
  main.py              # App initialization, mount routers
  schemas.py           # Pydantic models (extracted from main.py)
  routers/
    __init__.py
    solve.py           # /api/solve, /api/distance_matrix, optimizers
    agent.py           # /api/agent/plan, /api/agent/chat
    environment.py     # Environment management endpoints
    session.py         # Session management endpoints
```

### Changes Log

| Date | Change | Details |
|------|--------|---------|
| 2026-01-25 | Created router structure | Added solve.py, agent.py, environment.py |
| 2026-01-25 | Extracted schemas | Created schemas.py with Pydantic models |

---

## Phase 4: Frontend Modularization

### Goal
Break monolithic `webapp/isr.js` (8,949 lines) into modules.

### Planned Structure
```
webapp/
  modules/
    state.js           # Single source of truth for app state
    renderer.js        # Canvas rendering (from isr.js)
    mission_control.js # Mission control UI (from isr.js)
    animation.js       # Animation system (from isr.js)
    editor.js          # Editing functionality (from isr.js)
    drone_config.js    # Drone configuration (from isr.js)
  isr.js              # Orchestrator only (imports modules)
```

### Created Modules

| Module | File | Status | Description |
|--------|------|--------|-------------|
| State | `webapp/modules/state.js` | Complete | Single source of truth for app state |
| Renderer | `webapp/modules/renderer.js` | Complete | Canvas rendering with double buffering |
| Animation | `webapp/modules/animation.js` | Complete | Animation playback and interpolation |
| Mission Control | `webapp/modules/mission_control.js` | Complete | API communication layer |
| Editor | `webapp/modules/editor.js` | Complete | Environment editing functionality |
| Drone Config | `webapp/modules/drone_config.js` | Complete | Drone configuration UI |
| Agent Chat | `webapp/modules/agent_chat.js` | Complete | Agentic mode UI |
| Segmented Mission | `webapp/modules/segmented_mission.js` | Complete | Multi-segment mission handling |
| Segment Manager UI | `webapp/modules/segment_manager_ui.js` | Complete | Segment navigation and UI controls |
| Index | `webapp/modules/index.js` | Complete | Module exports aggregator |

### Changes Log

| Date | Change | Details |
|------|--------|---------|
| 2026-01-25 | Created modules directory | Initial structure |
| 2026-01-25 | Created state.js | Centralized state management with pub/sub |
| 2026-01-25 | Created renderer.js | Canvas rendering extracted from isr.js |
| 2026-01-25 | Created animation.js | Animation system with interpolation |
| 2026-01-25 | Created mission_control.js | API calls for solving and optimization |
| 2026-01-25 | Created editor.js | Edit mode and element creation |
| 2026-01-25 | Created segmented_mission.js | Clean SegmentedMission class |
| 2026-01-25 | Created drone_config.js | Drone configuration management |
| 2026-01-25 | Created agent_chat.js | Agent chat UI and communication |
| 2026-01-25 | Enhanced segmented_mission.js | Added cut workflow, checkpoint solving, animation support |
| 2026-01-25 | Created segment_manager_ui.js | Segment navigation and UI controls |

---

## Phase 5: Segmented Mission System

### Goal
Build clean segmented mission implementation using lessons learned.

### Key Decisions
- No complex splicing - use on-demand concatenation
- Clean freeze/commit workflow
- Simplified state management

### Implementation

Created `webapp/modules/segmented_mission.js` with:

| Class | Description |
|-------|-------------|
| `Segment` | Immutable data structure for one solved segment |
| `SegmentedMission` | Main container for multi-segment missions |

Key features:
- **On-demand trajectory concatenation** - No splicing at solve time
- **Computed data caching** - Combined trajectories, cut markers, total distance
- **Clean import/export** - JSON format with schema versioning (supports both v2 schemas)
- **Environment builders** - Separate display and solver environments
- **Frozen immutability** - Segments are frozen after creation
- **Cut workflow** - `performCut()`, `_splitTrajectoryAtDistance()`, `_calculateVisitedTargets()`
- **Checkpoint solving** - `getSyntheticStarts()`, `isCheckpointMode()`, `getActiveDroneIds()`
- **Animation support** - `getStartingDistance()`, `getAnimationConfig()`, `getFrozenTrajectories()`

Created `webapp/modules/segment_manager_ui.js` with:
- Navigation controls (previous/next segment, go to segment)
- Status panel (active targets, visited targets, lost drones)
- Timeline rendering (clickable segment timeline)
- Import/export integration

### Remaining Work
- [ ] Integrate with isr.js (replace existing segment managers)
- [ ] Test cut workflow end-to-end
- [ ] Test JSON import/export

---

## Phase 6: Agentic System (MIGRATED)

All agentic components verified in place:

| Component | File | Status |
|-----------|------|--------|
| v4 Multi-Agent | `server/agents/isr_agent_multi_v4.py` | Migrated |
| Coordinator v4 | `server/agents/coordinator_v4.py` | Migrated |
| Mission Tools | `server/agents/mission_tools.py` | Migrated |
| Orchestration | `server/agents/mission_orchestration_tools.py` | Migrated |
| Constraint Parser | `server/memory/constraint_parser.py` | Migrated |
| Constraints | `server/memory/constraints.py` | Migrated |

---

## Known Issues & Solutions

### Import Path Issues
- **Problem**: `ModuleNotFoundError` for server modules
- **Solution**: Ensure `PYTHONPATH` set to project root before running

### Orienteering Solver Location
- **Problem**: Solver must be at project root, not in subdirectory
- **Solution**: Keep `orienteering_with_matrix.py` at root level

---

## Testing Notes

- Existing tests in `tests/` directory
- Run with: `python -m pytest tests/`

---

## Rollback Plan

If issues arise, the original monolithic structure remains functional:
- `server/main.py` contains all original endpoints
- `webapp/isr.js` contains all original frontend code
