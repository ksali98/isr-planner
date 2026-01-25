# ISR Planner: Hybrid Build Strategy - Task List

This document outlines a **hybrid approach** to building the ISR Planner: create a clean architecture while selectively migrating working components from the existing codebase.

**Strategy:** "Build from scratch" means new architecture, but **reuse 60-70% of proven code**.

---

## Task Legend

- **[MIGRATE]** - Copy existing working code, integrate into new architecture
- **[REFACTOR]** - Existing code works but needs simplification/cleanup
- **[BUILD]** - Write new code from scratch
- **[COPY]** - Copy existing code as-is with minimal changes

---

## Phase 1: New Project Foundation

### 1.1 Project Setup
- [x] **[BUILD]** Initialize new Git branch `rebuild-clean`
- [x] **[COPY]** Copy project directory structure from existing
- [x] **[COPY]** Copy `requirements.txt` (add any missing dependencies)
- [x] **[COPY]** Copy `.env.example`
- [x] **[COPY]** Copy `run_planner.sh`
- [x] **[COPY]** Copy `.gitignore`

### 1.2 Documentation
- [x] **[COPY]** Copy `README.md`, update if needed
- [x] **[COPY]** Copy `SYSTEM_ARCHITECTURE.md` (already updated)
- [x] **[COPY]** Copy `DEPLOYMENT.md`
- [x] **[COPY]** Copy `CLAUDE.md` (already updated)
- [x] **[BUILD]** Create `MIGRATION_NOTES.md` to track changes

---

## Phase 2: Core Backend - Solver (Mostly Migrate!)

### 2.1 Distance Matrix & SAM Avoidance ✅
- [x] **[MIGRATE]** Copy `server/solver/sam_distance_matrix.py` **as-is**
  - ✅ SAM collision detection works
  - ✅ Tangent path algorithm works
  - ✅ Distance matrix caching works
- [x] **[COPY]** Copy existing unit tests
- [ ] **[BUILD]** Add integration tests if missing

**Estimated Savings:** 20 hours (already done!)

### 2.2 Orienteering Solver ✅
- [x] **[MIGRATE]** Copy `orienteering_with_matrix.py` **as-is**
  - ✅ Held-Karp DP implementation works
  - ✅ Fuel constraint handling works
  - ✅ Optimal for ≤12 targets
- [x] **[BUILD]** Ensure it stays at project root (document why)
- [x] **[COPY]** Copy existing tests

**Estimated Savings:** 30 hours (complex algorithm already implemented!)

### 2.3 Greedy Solver ✅
- [ ] **[MIGRATE]** Copy `server/solver/greedy_solver.py` **as-is**
  - ✅ Greedy heuristic works
  - ✅ Used for >12 targets
- [ ] **[COPY]** Copy existing tests

**Estimated Savings:** 8 hours

### 2.4 Target Allocation Strategies ✅
- [x] **[MIGRATE]** Copy `server/solver/target_allocator.py` **as-is**
  - ✅ All 5 strategies implemented: GREEDY, BALANCED, EFFICIENT, GEOGRAPHIC, EXCLUSIVE
  - ✅ Allocation validation works
- [x] **[COPY]** Copy existing tests

**Estimated Savings:** 25 hours (5 sophisticated algorithms!)

### 2.5 Trajectory Planning ✅
- [x] **[MIGRATE]** Copy `server/solver/trajectory_planner.py` **as-is**
  - ✅ SAM-avoiding trajectory generation works
  - ✅ Smooth path interpolation works
- [x] **[COPY]** Copy existing tests

**Estimated Savings:** 12 hours

### 2.6 Post-Optimizers ✅
- [x] **[MIGRATE]** Copy `server/solver/post_optimizer.py` **as-is**
  - ✅ Insert Missed works
  - ✅ Swap Closer with cascade works
  - ✅ No-Cross (2-opt) works
  - ✅ Auto-iterate with cycle detection works
- [x] **[COPY]** Copy existing tests

**Estimated Savings:** 35 hours (3 sophisticated optimizers!)

### 2.7 Solver Bridge
- [x] **[REFACTOR]** Copy `server/solver/solver_bridge.py`, simplify:
  - ✅ Keep 5-step pipeline logic (works)
  - ⚠️ Simplify segment/checkpoint handling (remove complex splicing)
  - ⚠️ Clean up state management
- [ ] **[BUILD]** Add clearer separation for single vs segmented missions
- [x] **[COPY]** Copy integration tests, add new ones

**Estimated Savings:** 15 hours (partial - pipeline logic exists)

**Phase 2 Total Savings: ~145 hours out of 180 hours (80% reuse!)**

---

## Phase 3: Backend - API Server

### 3.1 FastAPI Application
- [x] **[REFACTOR]** Copy `server/main.py`, reorganize:
  - ✅ Keep all endpoint logic (works)
  - ✅ Break into routers (solve_router, segment_router, agent_router) - **structure created**
  - ⚠️ Extract middleware to separate file
  - ✅ Extract Pydantic models to schemas.py - **created `server/schemas.py`**
- [x] **[BUILD]** Create `server/routers/` directory structure - **created solve.py, agent.py, environment.py**

**Estimated Savings:** 10 hours (endpoints exist, just reorganize)

### 3.2 Heuristic Mode Endpoints ✅
- [x] **[MIGRATE]** Copy endpoints from main.py:
  - ✅ `/api/solve` - works
  - ✅ `/api/distance_matrix` - works
  - ✅ `/api/insert_missed`, `/api/swap_closer`, `/api/crossing_removal` - work
- [x] **[BUILD]** Move to `server/routers/solve.py` - **structure created with endpoint stubs**

**Estimated Savings:** 5 hours

### 3.3 Environment Management Endpoints
- [x] **[MIGRATE]** Copy from main.py (if they exist)
- [x] **[BUILD]** Any missing endpoints - **created `server/routers/environment.py`**

**Estimated Savings:** 3 hours

### 3.4 Error Handling & Logging ✅
- [ ] **[COPY]** Existing error handling middleware
- [ ] **[REFACTOR]** Clean up if needed
- [ ] **[BUILD]** Add structured logging if missing

**Estimated Savings:** 2 hours

**Phase 3 Total Savings: ~20 hours out of 25 hours (80% reuse!)**

---

## Phase 4: Frontend - Web Application

### 4.1 HTML Structure ✅
- [x] **[MIGRATE]** Copy `webapp/index.html` **as-is**
  - ✅ Canvas, toolbar, panels all exist
  - ✅ Animation controls exist
- [ ] **[BUILD]** Minor cleanup/organization if needed

**Estimated Savings:** 5 hours

### 4.2 Core JavaScript - Rendering ✅
- [x] **[REFACTOR]** Extract from `webapp/isr.js`:
  - ✅ Keep `drawEnvironment()` logic (works well)
  - ✅ Keep canvas rendering code (works)
  - ✅ Move to new `webapp/modules/renderer.js` module - **CREATED**
  - ⚠️ Clean up monolithic structure
- [x] **[BUILD]** Create modular structure: **COMPLETED**
  ```
  webapp/
    modules/
      renderer.js       ✅ CREATED (extract from isr.js)
      mission_control.js ✅ CREATED (extract from isr.js)
      animation.js      ✅ CREATED (extract from isr.js)
      editor.js         ✅ CREATED (extract from isr.js)
      state.js          ✅ CREATED (new - single source of truth)
      index.js          ✅ CREATED (module exports)
    isr.js             (existing - to be updated as orchestrator)
  ```

**Estimated Savings:** 40 hours (rendering code exists, just modularize)

### 4.3 Editing & Environment Management ✅
- [x] **[REFACTOR]** Extract from isr.js to `editor.js`: **CREATED `webapp/modules/editor.js`**
  - ✅ Edit mode toggle works
  - ✅ Element creation works
  - ✅ Drag/delete works
  - ✅ Modularized into clean editor module

**Estimated Savings:** 15 hours

### 4.4 Drone Configuration UI ✅
- [x] **[MIGRATE]** Copy drone config panel code from isr.js
  - ✅ Config inputs work
  - ✅ Validation works
- [x] **[REFACTOR]** Extract to `drone_config.js` module - **CREATED `webapp/modules/drone_config.js`**

**Estimated Savings:** 8 hours

### 4.5 Mission Control ✅
- [x] **[REFACTOR]** Extract from isr.js to `mission_control.js`: **CREATED `webapp/modules/mission_control.js`**
  - ✅ Run Planner button logic works
  - ✅ API calls work
  - ✅ Post-optimizer buttons work
  - ✅ Modularized

**Estimated Savings:** 10 hours

### 4.6 Animation System ✅
- [x] **[REFACTOR]** Extract from isr.js to `animation.js`: **CREATED `webapp/modules/animation.js`**
  - ✅ Animation loop works
  - ✅ Drone position interpolation works
  - ✅ Target visiting logic works
  - ✅ Modularized
  - ⚠️ Simplify for future per-drone independent animation

**Estimated Savings:** 15 hours

### 4.7 Sequence Input ✅
- [ ] **[MIGRATE]** Copy sequence input panel code
  - ✅ Manual route entry works
  - ✅ Validation works

**Estimated Savings:** 3 hours

**Phase 4 Total Savings: ~96 hours out of 120 hours (80% reuse!)**

---

## Phase 5: Segmented Mission System

### 5.1 Segment Data Structure ✅
- [x] **[BUILD]** Create NEW `webapp/modules/segmented_mission.js` - **CREATED**
  - ✅ Don't copy existing implementations (two conflicting versions)
  - ✅ Design clean `SegmentedMission` class from architecture doc
  - ✅ On-demand trajectory concatenation (no splicing)
  - ✅ Computed data caching (combined trajectories, cut markers)

**Estimated Savings:** 10 hours (know what NOT to do from existing code)

### 5.2 Cut Workflow ✅
- [x] **[REFACTOR]** Copy cut button logic from isr.js, simplify:
  - ✅ Cut position recording works
  - ✅ Remove complex splicing (replaced with `performCut()`, `_splitTrajectoryAtDistance()`)
  - ✅ Implement freeze properly (immutable Segment class)

**Estimated Savings:** 15 hours (partial reuse)

### 5.3 Solving from Checkpoint ✅
- [x] **[MIGRATE]** Copy checkpoint solving logic from solver_bridge.py
  - ✅ Synthetic start nodes work (`getSyntheticStarts()`)
  - ✅ Simplified segment storage (`getSolverEnvironment()`)
  - ✅ Checkpoint mode detection (`isCheckpointMode()`)

**Estimated Savings:** 10 hours

### 5.4 Segmented Animation ✅
- [x] **[BUILD]** Implement trajectory concatenation (new clean approach)
  - ✅ `buildCombinedTrajectory()` for on-demand concatenation
  - ✅ `getStartingDistance()` for animation offset
  - ✅ `getFrozenTrajectories()` for completed segments
  - ✅ `getAnimationConfig()` for animation setup

**Estimated Savings:** 5 hours (know the requirements from existing code)

### 5.5 Segmented JSON Import/Export ✅
- [x] **[REFACTOR]** Copy JSON export logic, clean up schema
  - ✅ Export structure with cumulative frozen state
  - ✅ Support for both `isr_segmented_v2` and `isr_env_v2` schemas
  - ✅ Drone config normalization on import

**Estimated Savings:** 8 hours

### 5.6 Segment Manager UI ✅
- [x] **[BUILD]** Create `webapp/modules/segment_manager_ui.js` - **CREATED**
  - ✅ Navigation controls (prev/next/goTo/reset)
  - ✅ Status panel (active targets, visited targets, lost drones)
  - ✅ Timeline rendering (clickable segment timeline)
  - ✅ Import/export integration

**Estimated Savings:** 5 hours

**Phase 5 Total Savings: ~53 hours out of 80 hours (66% reuse)**

---

## Phase 6: Agentic System (Mostly Migrate!)

### 6.1 Constraint Parsing Infrastructure ✅
- [x] **[MIGRATE]** Copy `server/memory/constraints.py` **as-is**
  - ✅ ConstraintOp enum works
  - ✅ ConstraintCompiler works
  - ✅ Validation works
- [x] **[COPY]** Copy existing tests

**Estimated Savings:** 25 hours (fully implemented!)

### 6.2 Agent Tools ✅
- [ ] **[MIGRATE]** Copy `server/agents/tools/` **entire directory as-is**
  - ✅ All tools work
- [ ] **[COPY]** Copy existing tests

**Estimated Savings:** 20 hours

### 6.3 LangGraph Multi-Agent System ✅
- [x] **[MIGRATE]** Copy `server/agents/isr_agent_multi_v4.py` **as-is**
  - ✅ 6-agent system works
  - ✅ LangGraph workflow works
  - ✅ State transitions work
- [x] **[COPY]** Copy existing tests
- [ ] **[BUILD]** Add any missing edge cases

**Estimated Savings:** 40 hours (complex system already built!)

### 6.4 Coordinator ✅
- [x] **[MIGRATE]** Copy `server/agents/coordinator_v4.py` **as-is**
  - ✅ Deterministic pre-pass works
  - ✅ Regex parsing works

**Estimated Savings:** 8 hours

### 6.5 Supabase Memory ✅
- [x] **[MIGRATE]** Copy `server/memory/` directory **as-is** (if implemented)
  - ✅ Memory storage works
- [x] **[BUILD]** Add if missing

**Estimated Savings:** 5 hours

### 6.6 Agentic API Endpoint ✅
- [x] **[MIGRATE]** Copy `/api/agent/chat` from main.py
  - ✅ Endpoint works
- [x] **[REFACTOR]** Move to `server/routers/agent.py` - **structure created**

**Estimated Savings:** 5 hours

### 6.7 Agentic UI Integration ✅
- [x] **[MIGRATE]** Copy agent chat panel from isr.js (if exists)
- [x] **[BUILD]** Chat UI if missing
- [x] **[REFACTOR]** Extract to `modules/agent_chat.js` - **CREATED `webapp/modules/agent_chat.js`**

**Estimated Savings:** 10 hours

**Phase 6 Total Savings: ~113 hours out of 120 hours (94% reuse!)**

---

## Phase 7: Testing & QA ✅

### 7.1 Unit Tests ✅
- [x] **[BUILD]** Set up pytest configuration (`pytest.ini`, `conftest.py`)
- [x] **[BUILD]** Created `tests/unit/` directory structure
- [x] **[BUILD]** Unit tests for orchestration tools (`test_orchestration_tools.py`)
- [x] **[BUILD]** Unit tests for solver components (`test_solver_components.py`)
- [x] **[BUILD]** Unit tests for Pydantic schemas (`test_schemas.py`)

**Files Created:**
- `pytest.ini` - pytest configuration
- `tests/conftest.py` - shared fixtures
- `tests/unit/test_orchestration_tools.py` - Inspector, Trajectory, Constraints, Segments tests
- `tests/unit/test_solver_components.py` - SAM matrix, allocator, optimizer tests
- `tests/unit/test_schemas.py` - API schema validation tests

**Estimated Savings:** 30 hours (most tests exist)

### 7.2 Integration Tests ✅
- [x] **[BUILD]** Integration tests for agent system (`test_agent_system.py`)
- [x] **[BUILD]** Integration tests for API endpoints (`test_api_endpoints.py`)
- [x] **[BUILD]** Integration tests for segmented missions (`test_segmented_mission.py`)

**Files Created:**
- `tests/integration/test_agent_system.py` - LangGraph v4 agent tests
- `tests/integration/test_api_endpoints.py` - FastAPI endpoint tests
- `tests/integration/test_segmented_mission.py` - Segment workflow tests

**Estimated Savings:** 10 hours

### 7.3 UI/UX Testing
- [ ] **[BUILD]** Test refactored modular UI (requires browser-based testing)
- [x] **[COPY]** Reuse test cases from existing system

**Note:** Frontend is vanilla JavaScript without npm. Browser-based testing
(Playwright/Puppeteer) would need to be added separately.

**Estimated Savings:** 5 hours

### 7.4 Performance Testing ✅
- [x] **[BUILD]** Performance benchmarks for solver components

**Files Created:**
- `tests/benchmarks/test_solver_performance.py` - Benchmarks for:
  - Distance matrix calculation (varying targets/SAMs)
  - Target allocation strategies
  - Full solver bridge
  - Post-optimizers
  - Scalability testing (Held-Karp scaling)

**Estimated Savings:** 5 hours

### 7.5 Test Infrastructure ✅
- [x] **[BUILD]** Added testing dependencies to `requirements.txt`:
  - pytest>=8.0.0
  - pytest-cov>=4.1.0
  - pytest-asyncio>=0.23.0
  - pytest-timeout>=2.2.0
  - httpx>=0.27.0 (for FastAPI TestClient)
- [x] **[BUILD]** Created test data directory with sample environment
  - `tests/data/sample_env.json`

**Phase 7 Total Savings: ~50 hours out of 80 hours (63% reuse)**

---

## Phase 8: Documentation

### 8.1 User Documentation ✅
- [ ] **[COPY]** Copy `README.md`, update with new architecture
- [ ] **[COPY]** Copy example mission files

**Estimated Savings:** 15 hours

### 8.2 Developer Documentation ✅
- [ ] **[COPY]** Copy `SYSTEM_ARCHITECTURE.md` (already updated)
- [ ] **[COPY]** Copy `MODULAR_ARCHITECTURE.md`
- [ ] **[BUILD]** Update with actual refactoring results

**Estimated Savings:** 10 hours

### 8.3 AI Assistant Context ✅
- [ ] **[COPY]** Copy `CLAUDE.md`
- [ ] **[BUILD]** Update with migration notes

**Estimated Savings:** 3 hours

**Phase 8 Total Savings: ~28 hours out of 40 hours (70% reuse)**

---

## Phase 9: Deployment

### 9.1 Railway Deployment ✅
- [ ] **[COPY]** Copy Railway configuration
- [ ] **[BUILD]** Update if needed

**Estimated Savings:** 8 hours

### 9.2-9.4 Other Deployment
- [ ] **[COPY]** Copy existing configs
- [ ] **[BUILD]** Update as needed

**Estimated Savings:** 8 hours

**Phase 9 Total Savings: ~16 hours out of 25 hours (64% reuse)**

---

## Summary: Time Savings with Hybrid Approach

| Phase | Original Estimate | With Migration | Savings | Reuse % |
|-------|------------------|----------------|---------|---------|
| Phase 1: Infrastructure | 8 hours | 3 hours | 5 hours | 63% |
| Phase 2: Core Solver | 180 hours | 35 hours | **145 hours** | **80%** |
| Phase 3: API Server | 25 hours | 5 hours | 20 hours | 80% |
| Phase 4: Frontend | 120 hours | 24 hours | **96 hours** | **80%** |
| Phase 5: Segmented | 80 hours | 27 hours | 53 hours | 66% |
| Phase 6: Agentic | 120 hours | 7 hours | **113 hours** | **94%** |
| Phase 7: Testing | 80 hours | 30 hours | 50 hours | 63% |
| Phase 8: Docs | 40 hours | 12 hours | 28 hours | 70% |
| Phase 9: Deployment | 25 hours | 9 hours | 16 hours | 64% |
| **TOTAL** | **678 hours** | **152 hours** | **526 hours** | **78%** |

## Revised Effort Estimate

**Total Effort with Hybrid Approach:** **150-180 hours** (3.5-4.5 weeks full-time)

**vs Original "Build from Scratch":** 400-600 hours

**Time Saved:** 75-80% reduction by reusing working components!

---

## Migration Strategy

### Week 1: Foundation + Core Solver
1. Set up new branch
2. Migrate all Phase 2 solver components (mostly copy-paste!)
3. Set up new FastAPI structure
4. Run existing solver tests to confirm everything works

### Week 2: Frontend Refactoring
1. Create modular structure
2. Extract renderer, animation, editor from monolithic isr.js
3. Create clean state management
4. Test that UI still works

### Week 3: Segments + Agentic
1. Build clean segmented mission system
2. Migrate agentic system (mostly copy!)
3. Integrate everything

### Week 4: Testing + Deployment
1. Run all tests
2. Fix any integration issues
3. Deploy
4. Document migration

---

## Key Principles

1. **"Don't rewrite what works"** - 78% of code can be reused
2. **"Refactor, don't rebuild"** - Frontend needs modularization, not rewrite
3. **"Learn from mistakes"** - Segmented mission: build clean version using lessons learned
4. **"Test as you go"** - Existing tests prove components work

---

## What Makes This "From Scratch"?

Even though you're reusing code, this IS a "from scratch" rebuild because:

1. **New Architecture**: Clean separation, single source of truth
2. **Modular Design**: Break monolithic isr.js into modules
3. **Simplified Segments**: Clean implementation, no complex splicing
4. **Better Organization**: Routers, schemas, clear structure
5. **Clean Git History**: New branch, fresh start

You're building a **new system** that **integrates proven components**. This is the professional approach!
