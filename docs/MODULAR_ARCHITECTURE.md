# ISR Planner Modular Architecture Plan

## Executive Summary

This document proposes a comprehensive refactoring of the ISR Planner codebase to improve maintainability, testability, and developer productivity through modular organization.

**Key Issues Addressed:**
- 328KB monolithic JavaScript file (`isr.js`)
- 4 coexisting agent versions (v1-v4) totaling 9,596 lines
- 2,352-line server file mixing concerns
- Duplicate optimizer and allocator implementations

**Expected Benefits:**
- 10x easier debugging and feature development
- Clear module boundaries and responsibilities
- Improved onboarding (hours vs days)
- Enable unit testing and code splitting

---

## Current Architecture Analysis

### Frontend (JavaScript)

#### Critical Issue: Monolithic `isr.js` (328,896 bytes)

**File**: `/webapp/isr.js`

**Current State:**
- Single massive file handling ALL frontend responsibilities
- Estimated 8,000+ lines of mixed concerns
- **Responsibilities bundled together:**
  - Canvas rendering and drawing
  - Mission control UI state management
  - Drone configuration panels
  - File I/O (save/load/import/export)
  - API communication (solving, optimization, agent chat)
  - Animation and playback controls
  - SAM wrapping visualization
  - Post-optimization UI
  - Segment management integration
  - Trajectory display toggles
  - Debug output formatting

**Impact:**
- Difficult to debug specific features (must search entire file)
- Hard to test individual components
- Merge conflicts when multiple developers work
- Slow IDE performance due to file size
- No code splitting = slow initial page load

**Related Files:**
- `segment_manager.js` (18KB) - Partial extraction of segment UI logic
- `segmented_mission.js` (31KB) - Segmented mission state management

### Backend (Python)

#### Agent Version Sprawl

**Files:**
- `/server/agents/isr_agent_multi_v2.py` (1,842 lines) - Legacy multi-agent
- `/server/agents/isr_agent_multi_v3.py` (2,530 lines) - Task-based agents
- `/server/agents/isr_agent_multi_v4.py` (3,031 lines) - **ACTIVE** reasoning-based
- `/server/versions/isr_agent_single_v1.py` (2,193 lines) - Original single agent

**Total**: 9,596 lines of agent code with significant overlap

**Current State:**
- v4 is the only active version (documented in AGENTIC_SYSTEM_DESCRIPTION.md)
- v1-v3 remain in codebase with commented imports in `main.py`
- No clear deprecation path or migration guide
- Confusion about which version to reference for development

#### Oversized Server Entry Point

**File**: `/server/main.py` (2,352 lines)

**Responsibilities:**
- FastAPI app configuration
- 20+ API endpoint definitions
- Pydantic request/response models
- Business logic (solving, optimization, chat)
- Memory management endpoints
- File upload/download handling
- Executive tick/action endpoints
- SAM polygon wrapping logic

**Impact:**
- Difficult to navigate and locate specific endpoints
- Testing requires loading entire application
- Violates Single Responsibility Principle

#### Code Duplication

**Optimizer Duplication:**
- `/server/solver/post_optimizer.py` (2,078 lines) - Current version
- `/server/solver/post_optimizer_v1.py` (1,122 lines) - Legacy version
- Total: 3,200 lines with unclear differences

**Allocator Duplication:**
- `/server/solver/target_allocator.py` (911 lines) - Current version
- `/server/solver/target_allocator_v1.py` (892 lines) - Legacy version
- Total: 1,803 lines with nearly identical implementations

---

## Proposed Modular Structure

### Frontend Reorganization

#### Target Structure: `/webapp/src/`

```
webapp/
  index.html                # Main HTML entry point
  src/
    main.js                 # Application entry point (~100 lines)

    core/
      app.js                # Application orchestrator
      state.js              # Global state management
      config.js             # Configuration constants

    rendering/
      canvas-renderer.js    # Canvas drawing operations
      visualization.js      # SAM wrapping, trajectory rendering
      animation.js          # Playback and animation controls
      markers.js            # Draw targets, airports, SAMs, drones

    ui/
      mission-control.js    # Mission control panel logic
      drone-panel.js        # Drone configuration UI
      target-panel.js       # Target editing and display
      stats-panel.js        # Statistics display
      file-manager.js       # Save/load/import/export operations
      tabs.js               # Tab switching logic

    api/
      client.js             # API client abstraction
      endpoints.js          # Endpoint definitions
      models.js             # Request/response type definitions

    segments/
      segmented-mission.js  # SegmentedMission class (from SYSTEM_ARCHITECTURE.md)
      segment-manager.js    # Segment UI controls
      segment-animation.js  # Multi-segment animation logic

    utils/
      geometry.js           # Distance, angle calculations
      formatting.js         # Number, coordinate formatting
      validators.js         # Input validation helpers
      dom.js                # DOM manipulation utilities

  legacy/
    isr.js                  # Original monolithic file (kept for reference)
```

**Migration Strategy:**
1. Extract utility functions first (geometry, formatting)
2. Extract API client layer
3. Extract rendering pipeline
4. Extract UI components
5. Create new `main.js` that ties everything together
6. Test each module independently
7. Switch `index.html` to use modular version
8. Archive old `isr.js`

**Benefits:**
- Each module can be tested independently
- Code splitting reduces initial load time
- Multiple developers can work in parallel
- IDE performance improves
- Clear responsibility for each file

---

### Backend Reorganization

#### 1. Clean Up Agent Versions

**Action: Archive Legacy Agents**

```
server/
  agents/
    v4/                     # ACTIVE VERSION
      __init__.py
      isr_agent_multi_v4.py
      coordinator_v4.py
      mission_tools.py
      mission_orchestration_tools.py
      agent_memory.py

    archive/                # LEGACY VERSIONS (for reference)
      v1/
        isr_agent_single_v1.py
        README.md           # "Deprecated. Use v4."
      v2/
        isr_agent_multi_v2.py
        README.md
      v3/
        isr_agent_multi_v3.py
        README.md
```

**Benefits:**
- Clear "source of truth" (v4)
- Reduces cognitive load
- Preserves history for reference
- Easier git diffs (focused on active code)

**Migration Checklist:**
- [x] Move v1, v2, v3 to `archive/` subdirectories
- [x] Remove commented imports from `main.py`
- [ ] Create `MIGRATION_V3_TO_V4.md` explaining differences
- [ ] Update all documentation to reference v4 only
- [ ] Add deprecation notices in archived READMEs

---

#### 2. Split Server Entry Point

**Target Structure: `/server/`**

```
server/
  main.py                   # FastAPI app + config ONLY (~200 lines)

  api/
    __init__.py
    models.py               # Pydantic request/response models

    routes/
      __init__.py
      solving.py            # /solve, /solve-with-allocation
      optimization.py       # /insert_missed, /swap, /crossing
      agent.py              # /agent/plan, /agent/chat, /agent/memory
      executive.py          # /executive/tick, /executive/action
      files.py              # /environment/save, /environment/load
      matrix.py             # /calculate_wrapping, /distance-matrix
      health.py             # /health endpoint
```

**Example Router (`api/routes/solving.py`):**

```python
from fastapi import APIRouter, HTTPException
from ..models import SolveRequest, SolveResponse
from ...solver.solver_bridge import solve_mission

router = APIRouter(prefix="/api", tags=["solving"])

@router.post("/solve")
async def solve_mission_endpoint(request: SolveRequest) -> SolveResponse:
    """Solve a mission using orienteering algorithm."""
    try:
        result = solve_mission(
            environment=request.environment,
            drone_configs=request.drone_configs,
            strategy=request.strategy
        )
        return SolveResponse(**result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

**New `main.py` (simplified):**

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .api.routes import solving, optimization, agent, executive, files, matrix, health

app = FastAPI(title="ISR Mission Planner API")

# CORS
app.add_middleware(CORSMiddleware, ...)

# Static files
app.mount("/static", StaticFiles(directory="webapp"), name="static")

# Mount routers
app.include_router(solving.router)
app.include_router(optimization.router)
app.include_router(agent.router)
app.include_router(executive.router)
app.include_router(files.router)
app.include_router(matrix.router)
app.include_router(health.router)

@app.get("/")
async def root():
    return FileResponse("webapp/index.html")
```

**Benefits:**
- Logical grouping by feature
- Easy to locate specific endpoint
- Testable routers (mock dependencies)
- Standard FastAPI best practices

---

#### 3. Consolidate Duplicate Implementations

**Optimizers:**

**Action:** Determine differences between `post_optimizer.py` and `post_optimizer_v1.py`

**Options:**
- If v1 is truly legacy: Archive it
- If both needed: Rename clearly
  - `post_optimizer_cascade.py` (current, with auto-regen)
  - `post_optimizer_simple.py` (v1, basic swaps)
- Document differences in code comments

**Allocators:**

**Action:** Consolidate `target_allocator.py` and `target_allocator_v1.py`

**Likely Outcome:** Same codebase, just version drift
- Merge into single `target_allocator.py`
- Archive v1 with note "Merged into main allocator"

---

#### 4. Refactor Solver Bridge

**Current State:** `solver/solver_bridge.py` (1,087 lines) - God Object

**Responsibilities:**
- Distance matrix caching
- Environment hashing
- Solving orchestration
- Allocation strategy selection
- Post-optimization coordination
- Trajectory planning
- SAM wrapping for visualization

**Proposed Structure:**

```
server/solver/
  core/
    solver_engine.py        # Core solving logic
    distance_cache.py       # Matrix caching system
    environment_hash.py     # Hash computation

  allocation/
    allocator_engine.py     # Allocation orchestration
    strategies/
      efficient.py          # Auction-based
      greedy.py             # Nearest neighbor
      balanced.py           # Equal load
      geographic.py         # Corridor-based
      exclusive.py          # Unique targets first

  optimization/
    optimizer_engine.py     # Post-optimization orchestration
    insert_missed.py        # Insert unvisited targets
    swap_closer.py          # Swap for shorter routes
    crossing_removal.py     # 2-opt crossing removal

  trajectory/
    trajectory_engine.py    # Trajectory planning
    sam_avoidance.py        # SAM boundary navigation

  bridge.py                 # Facade/coordinator (~200 lines)
```

**New `bridge.py` (slim facade):**

```python
from .core.solver_engine import SolverEngine
from .allocation.allocator_engine import AllocatorEngine
from .optimization.optimizer_engine import OptimizerEngine
from .trajectory.trajectory_engine import TrajectoryEngine

class SolverBridge:
    """Facade for solving pipeline."""

    def __init__(self):
        self.solver = SolverEngine()
        self.allocator = AllocatorEngine()
        self.optimizer = OptimizerEngine()
        self.trajectory = TrajectoryEngine()

    def solve_mission(self, environment, drone_configs, strategy="efficient"):
        """Orchestrate full solving pipeline."""
        # 1. Allocate targets to drones
        allocation = self.allocator.allocate(environment, drone_configs, strategy)

        # 2. Solve routes per drone
        routes = self.solver.solve(allocation, environment)

        # 3. Generate SAM-aware trajectories
        trajectories = self.trajectory.plan(routes, environment)

        return {
            "routes": routes,
            "trajectories": trajectories,
            "allocation": allocation
        }
```

**Benefits:**
- Single Responsibility per module
- Easier unit testing (mock dependencies)
- Clear data flow
- Reusable components

---

## Module Dependency Graph

### Proposed Layer Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     API Layer                            │
│  (FastAPI routers - HTTP handling only)                 │
└─────────────────────┬────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────┐
│                  Agent Layer                             │
│  (LangGraph agents - reasoning and orchestration)       │
└─────────────────────┬────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────┐
│                 Solver Layer                             │
│  (Algorithms - allocation, optimization, trajectories)  │
└─────────────────────┬────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────┐
│                   Core Layer                             │
│  (Geometry, distance calculations, utilities)           │
└─────────────────────────────────────────────────────────┘
```

**Dependency Rules:**
- **API Layer**: Depends on Agents + Solvers (but not vice versa)
- **Agent Layer**: Depends on Solvers (but not API)
- **Solver Layer**: Depends on Core (but not Agents)
- **Core Layer**: No dependencies (pure utilities)

**No Circular Dependencies:**
- Use dependency injection when needed
- Pass interfaces/protocols, not concrete classes
- Clear "data down, events up" pattern

---

## Refactoring Roadmap

### Phase 1: Critical Wins (Week 1)

**Priority: High Impact, Low Risk**

#### Task 1.1: Archive Legacy Agent Versions
- Move v1, v2, v3 to `server/agents/archive/`
- Remove commented imports from `main.py`
- Create deprecation READMEs

**Estimated Effort:** 2 hours
**Benefit:** Immediate clarity on source of truth

#### Task 1.2: Extract JavaScript Utilities
- Create `webapp/src/utils/geometry.js`
- Create `webapp/src/utils/formatting.js`
- Create `webapp/src/utils/dom.js`
- Extract and test functions from `isr.js`

**Estimated Effort:** 1 day
**Benefit:** First step toward modular frontend

#### Task 1.3: Document Current Architecture
- Finalize SYSTEM_ARCHITECTURE.md
- Create module dependency diagrams
- Document API endpoints

**Estimated Effort:** 1 day
**Benefit:** Shared understanding for team

---

### Phase 2: API Refactoring (Weeks 2-4)

**Priority: Medium Impact, Medium Risk**

#### Task 2.1: Create API Router Structure
- Create `server/api/routes/` directory
- Create empty routers with placeholders
- Update `main.py` to use routers

**Estimated Effort:** 2 days
**Benefit:** Organized endpoint structure

#### Task 2.2: Migrate Endpoints to Routers
- Move endpoints from `main.py` to feature routers
- One router at a time (solving → optimization → agent → ...)
- Test after each migration

**Estimated Effort:** 1 week
**Benefit:** Maintainable server code

#### Task 2.3: Consolidate Duplicates
- Compare post_optimizer.py vs v1
- Compare target_allocator.py vs v1
- Merge or archive as appropriate

**Estimated Effort:** 3 days
**Benefit:** Reduced maintenance burden

---

### Phase 3: Frontend Modularization (Weeks 5-8)

**Priority: High Impact, High Risk**

#### Task 3.1: Extract API Client Layer
- Create `webapp/src/api/client.js`
- Create `webapp/src/api/endpoints.js`
- Replace fetch calls in `isr.js` with client methods

**Estimated Effort:** 3 days
**Benefit:** Centralized API logic

#### Task 3.2: Extract Rendering Layer
- Create `webapp/src/rendering/canvas-renderer.js`
- Create `webapp/src/rendering/markers.js`
- Extract canvas drawing logic

**Estimated Effort:** 1 week
**Benefit:** Testable rendering

#### Task 3.3: Extract UI Components
- Create `webapp/src/ui/mission-control.js`
- Create `webapp/src/ui/drone-panel.js`
- Extract event handlers and UI updates

**Estimated Effort:** 1 week
**Benefit:** Modular UI logic

#### Task 3.4: Create New Main Entry Point
- Create `webapp/src/main.js`
- Wire up all modules
- Test full application flow

**Estimated Effort:** 1 week
**Benefit:** Clean application structure

#### Task 3.5: Switch to Modular Version
- Update `index.html` to use `src/main.js`
- Run full regression tests
- Archive old `isr.js` to `legacy/`

**Estimated Effort:** 2 days
**Benefit:** Complete frontend modularization

---

### Phase 4: Solver Refactoring (Weeks 9-12)

**Priority: Medium Impact, High Risk**

#### Task 4.1: Extract Solver Components
- Create solver subdirectories (core, allocation, optimization, trajectory)
- Extract individual components from `solver_bridge.py`
- Create slim bridge facade

**Estimated Effort:** 2 weeks
**Benefit:** Clear solver architecture

#### Task 4.2: Add Type Hints
- Add comprehensive type hints to all solver modules
- Set up `mypy` for static type checking
- Create shared type definitions

**Estimated Effort:** 1 week
**Benefit:** Catch bugs early, better IDE support

---

### Phase 5: Testing Infrastructure (Ongoing)

**Priority: Medium Impact, Low Risk**

#### Task 5.1: Unit Tests
- Create tests for utility functions
- Create tests for API endpoints (mocked solvers)
- Create tests for solver components

**Estimated Effort:** 2 weeks (ongoing)
**Benefit:** Regression prevention

#### Task 5.2: Integration Tests
- Test full solving pipeline
- Test agent workflows
- Test segment creation/replay

**Estimated Effort:** 1 week
**Benefit:** Confidence in refactoring

---

## Code Organization Principles

### Single Responsibility Principle
- Each module has ONE clear purpose
- If a module description includes "and", split it

### Clear Module Boundaries
- Modules communicate through defined interfaces
- No reaching into other modules' internals
- Use dependency injection for testability

### Shared Type Definitions
- Create `types.py` or `models.py` for shared types
- Don't duplicate type definitions across modules
- Use TypedDict/Pydantic for structured data

### Documentation Standards
- Every module has a docstring explaining its purpose
- Every public function has a docstring
- Complex logic has inline comments
- Architecture decisions documented in ADRs (Architecture Decision Records)

---

## Migration Checklist

### Before Starting Each Phase
- [ ] Create feature branch
- [ ] Document current behavior (tests or manual checks)
- [ ] Identify dependencies and risks

### During Migration
- [ ] Make small, incremental commits
- [ ] Test after each change
- [ ] Update documentation as you go

### After Completing Phase
- [ ] Run full regression test suite
- [ ] Update relevant documentation
- [ ] Code review with team
- [ ] Merge to main branch

---

## Risks and Mitigation

### Risk: Breaking Existing Functionality
**Mitigation:**
- Comprehensive testing before each merge
- Keep old code in `legacy/` for reference
- Feature flags to toggle new/old implementations

### Risk: Scope Creep
**Mitigation:**
- Stick to roadmap phases
- Resist temptation to "fix everything at once"
- Each phase is independently valuable

### Risk: Team Confusion
**Mitigation:**
- Clear documentation of new structure
- Team meetings to explain changes
- Gradual rollout (one module at a time)

### Risk: Performance Regression
**Mitigation:**
- Benchmark critical paths before and after
- Code splitting should improve frontend performance
- Backend refactoring should be performance-neutral

---

## Success Metrics

### Quantitative Metrics
- **Frontend file size**: 328KB → <50KB per module (average)
- **Server endpoint density**: 20+ endpoints in one file → <5 per router
- **Code duplication**: 3,200 duplicate optimizer lines → 0
- **Test coverage**: 0% → 60%+ over 6 months

### Qualitative Metrics
- **Developer onboarding time**: Days → Hours
- **Feature development speed**: Faster (less code to navigate)
- **Bug fix time**: Faster (easier to isolate issues)
- **Code review quality**: Better (focused diffs)

---

## Related Documentation

- **SYSTEM_ARCHITECTURE.md**: Segmented mission system design
- **AGENTIC_SYSTEM_DESCRIPTION.md**: v4 agent architecture
- **V3_TO_V4_MIGRATION.md**: Agent version migration guide
- **MISSION_ORCHESTRATION_TOOLS.md**: Agent tools API

---

## Next Steps

1. **Review this plan** with team
2. **Get alignment** on priorities
3. **Start Phase 1** (legacy agent archive + JS utilities)
4. **Weekly check-ins** to track progress
5. **Adjust roadmap** based on learnings

---

*Last Updated: 2026-01-24*
*Status: Draft - Ready for Team Review*
