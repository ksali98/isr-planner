# ISR Planner: Build From Scratch - Task List

This document outlines all tasks required to build the ISR Planner system from the ground up, organized by development phase.

---

## Phase 1: Project Infrastructure & Foundation

### 1.1 Project Setup
- [ ] Initialize Git repository
- [ ] Create project directory structure (`server/`, `webapp/`, `docs/`)
- [ ] Set up Python virtual environment
- [ ] Create `requirements.txt` with dependencies:
  - FastAPI, Uvicorn
  - NumPy, SciPy
  - LangGraph, LangChain
  - Anthropic SDK
  - Supabase client (optional)
- [ ] Create `.env.example` for API keys
- [ ] Write `run_planner.sh` startup script
- [ ] Create basic `.gitignore`

### 1.2 Documentation
- [ ] Write `README.md` with user guide
- [ ] Create `SYSTEM_ARCHITECTURE.md` with technical overview
- [ ] Create `DEPLOYMENT.md` for hosting instructions
- [ ] Create `CLAUDE.md` for AI assistant context

---

## Phase 2: Core Backend - Solving Infrastructure

### 2.1 Distance Matrix & SAM Avoidance
- [ ] Implement SAM-aware distance calculation
  - [ ] Create `sam_distance_matrix.py`
  - [ ] Implement SAM zone collision detection
  - [ ] Implement path-around-SAM algorithm (tangent paths)
  - [ ] Build distance matrix for all node pairs (airports + targets)
  - [ ] Add caching for performance
- [ ] Write unit tests for distance calculations

### 2.2 Orienteering Solver (Held-Karp DP)
- [ ] Create `orienteering_with_matrix.py` at project root
- [ ] Implement Held-Karp dynamic programming algorithm
  - [ ] State representation: (visited_set, current_node, remaining_fuel)
  - [ ] DP recurrence with memoization
  - [ ] Backtrack to reconstruct optimal path
- [ ] Handle constraints:
  - [ ] Fuel budget limits
  - [ ] Start/end at airports
  - [ ] Target priorities/points
- [ ] Optimize for ≤12 targets (exponential complexity)
- [ ] Write unit tests with known optimal solutions

### 2.3 Greedy Solver (Fallback)
- [ ] Create `greedy_solver.py`
- [ ] Implement greedy heuristic:
  - [ ] Nearest-neighbor selection
  - [ ] Priority-weighted selection
  - [ ] Fuel-aware termination
- [ ] Use for >12 targets (suboptimal but fast)
- [ ] Write unit tests

### 2.4 Target Allocation Strategies
- [ ] Create `target_allocator.py`
- [ ] Implement AllocationStrategy enum
- [ ] Implement 5 allocation algorithms:
  - [ ] **GREEDY**: Nearest drone, highest priority first
  - [ ] **BALANCED**: Even distribution by count
  - [ ] **EFFICIENT**: Auction-based (maximize priority/distance ratio)
  - [ ] **GEOGRAPHIC**: Angular sector division
  - [ ] **EXCLUSIVE**: Unique capabilities first (target type restrictions)
- [ ] Add allocation validation (feasibility checks)
- [ ] Write unit tests for each strategy

### 2.5 Trajectory Planning
- [ ] Create `isr_trajectory.py`
- [ ] Implement SAM-avoiding trajectory generation:
  - [ ] Take route (sequence of nodes)
  - [ ] Generate detailed [x, y] coordinates
  - [ ] Insert waypoints around SAM zones
  - [ ] Ensure smooth paths
- [ ] Calculate total distance along trajectory
- [ ] Write unit tests

### 2.6 Post-Optimizers
- [ ] Create `post_optimizer.py`
- [ ] Implement **Insert Missed** optimizer:
  - [ ] Find unvisited targets
  - [ ] Greedy insertion into routes (least cost increase)
  - [ ] Respect fuel constraints
- [ ] Implement **Swap Closer** optimizer:
  - [ ] Detect targets closer to other drones
  - [ ] Geometric swap detection
  - [ ] Auto-iterate mode with cycle detection
  - [ ] Cascade mode (run until convergence)
- [ ] Implement **No-Cross (2-opt)** optimizer:
  - [ ] Detect trajectory intersections
  - [ ] Apply 2-opt uncrossing
  - [ ] Validate improvement
- [ ] Write unit tests for each optimizer

### 2.7 Solver Bridge (Orchestration)
- [ ] Create `solver_bridge.py`
- [ ] Implement 5-step pipeline:
  1. Calculate SAM-aware distance matrix
  2. Allocate targets to drones
  3. Solve per-drone orienteering problem
  4. Generate SAM-avoiding trajectories
  5. Apply post-optimizers (optional)
- [ ] Handle single vs segmented missions
- [ ] Add checkpoint/cut support (synthetic start nodes)
- [ ] Calculate metrics (fuel, points, distance)
- [ ] Write integration tests

---

## Phase 3: Backend - API Server

### 3.1 FastAPI Application
- [ ] Create `server/main.py`
- [ ] Set up FastAPI app with CORS
- [ ] Add static file serving for webapp
- [ ] Add health check endpoint `/health`

### 3.2 Heuristic Mode Endpoints
- [ ] Implement `/api/solve` (POST)
  - [ ] Accept environment, drone configs, allocation strategy
  - [ ] Call solver_bridge
  - [ ] Return routes, trajectories, metrics
- [ ] Implement `/api/distance_matrix` (POST)
  - [ ] Return SAM-aware distance matrix
- [ ] Implement post-optimizer endpoints:
  - [ ] `/api/insert_missed` (POST)
  - [ ] `/api/swap_closer` (POST)
  - [ ] `/api/crossing_removal` (POST)

### 3.3 Environment Management Endpoints
- [ ] Implement `/api/save_environment` (POST)
- [ ] Implement `/api/load_environment` (POST)
- [ ] Implement `/api/export_segmented_mission` (POST)
- [ ] Implement `/api/import_segmented_mission` (POST)

### 3.4 Error Handling & Logging
- [ ] Add request validation with Pydantic models
- [ ] Add error handling middleware
- [ ] Add logging (INFO for requests, ERROR for failures)
- [ ] Return meaningful error messages to frontend

---

## Phase 4: Frontend - Web Application

### 4.1 HTML Structure
- [ ] Create `webapp/index.html`
- [ ] Add canvas element for map rendering
- [ ] Add toolbar (Edit, File, Segment, Mission Control)
- [ ] Add panels:
  - [ ] Drone configuration panel
  - [ ] Target info panel
  - [ ] Metrics panel (fuel, points)
  - [ ] Sequence input panel
- [ ] Add animation controls (Play, Pause, Reset, speed slider)
- [ ] Link CSS and JavaScript files

### 4.2 Core JavaScript (isr.js)
- [ ] Create `webapp/isr.js`
- [ ] Initialize state object:
  - [ ] targets, airports, sams
  - [ ] drone_configs
  - [ ] routes, trajectories
  - [ ] animation state
- [ ] Implement canvas rendering:
  - [ ] `drawEnvironment()` - render all elements
  - [ ] Draw targets (colored by type, priority)
  - [ ] Draw airports (blue triangles)
  - [ ] Draw SAMs (red circles with range)
  - [ ] Draw trajectories (colored per drone)
  - [ ] Draw drones (animated icons)
- [ ] Implement mouse interaction:
  - [ ] Click to add target/airport/SAM (edit mode)
  - [ ] Drag to reposition elements
  - [ ] Click to select element
  - [ ] Delete selected element

### 4.3 Editing & Environment Management
- [ ] Implement Edit mode toggle
- [ ] Implement element creation:
  - [ ] Add target (with type, priority dialogs)
  - [ ] Add airport
  - [ ] Add SAM (with range input)
- [ ] Implement element deletion
- [ ] Implement element property editing (double-click)
- [ ] Implement save/load environment:
  - [ ] Serialize to JSON
  - [ ] Call `/api/save_environment`
  - [ ] Load from file, call `/api/load_environment`

### 4.4 Drone Configuration UI
- [ ] Create drone config panel
- [ ] Add inputs per drone:
  - [ ] Fuel capacity
  - [ ] Fuel per unit distance
  - [ ] Max targets
  - [ ] Target type restrictions (A-E checkboxes)
  - [ ] Enabled/disabled toggle
- [ ] Implement drone selection (click on airport)
- [ ] Implement config validation

### 4.5 Mission Control
- [ ] Implement **Run Planner** button:
  - [ ] Gather environment + drone configs
  - [ ] Show allocation strategy dropdown
  - [ ] Call `/api/solve`
  - [ ] Render returned routes/trajectories
  - [ ] Update metrics panel
- [ ] Implement **Accept Solution** button
- [ ] Implement **Post-Optimizer** buttons:
  - [ ] Insert Missed
  - [ ] Swap Closer
  - [ ] No-Cross
- [ ] Add loading indicators during API calls

### 4.6 Animation System
- [ ] Implement `animateMission()` function:
  - [ ] Track `missionDistance` (global progress)
  - [ ] Interpolate drone positions along trajectories
  - [ ] Mark targets as visited (within threshold)
  - [ ] Update animation speed from slider
- [ ] Implement Play/Pause/Reset controls
- [ ] Implement animation loop with `requestAnimationFrame`
- [ ] Show current fuel usage during animation

### 4.7 Sequence Input
- [ ] Add sequence input panel
- [ ] Allow manual route entry (e.g., "A1,T1,T2,T3,A1")
- [ ] Validate sequences (valid nodes, closed loop)
- [ ] Apply sequence to selected drone
- [ ] Call solver to generate trajectory

---

## Phase 5: Segmented Mission System

### 5.1 Segment Data Structure
- [ ] Create `webapp/segmented_mission.js`
- [ ] Define `SM_Segment` class:
  - [ ] index
  - [ ] solution (routes, trajectories, sequences)
  - [ ] environment snapshot
  - [ ] cutPositions (per-drone [x, y])
  - [ ] cutDistance
  - [ ] lostDrones, visitedTargets
  - [ ] droneConfigs
  - [ ] timestamp
- [ ] Define `SegmentedMission` class:
  - [ ] segments array
  - [ ] currentSegmentIndex
  - [ ] addSegment(), getSegment()
  - [ ] exportToJSON(), importFromJSON()

### 5.2 Cut Workflow
- [ ] Add **Cut** button to UI
- [ ] Implement `freezeAtCheckpoint()`:
  - [ ] Pause animation
  - [ ] Record current drone positions
  - [ ] Record cutDistance
  - [ ] Save current segment with all data
  - [ ] Show white diamond markers (C1, C2, ...)
- [ ] Enable environment editing after cut
- [ ] Track disabled drones → lostDrones array

### 5.3 Solving from Checkpoint
- [ ] Modify solver API to accept:
  - [ ] Synthetic start nodes (cut positions)
  - [ ] Remaining fuel budgets
  - [ ] Modified environment
- [ ] Create new segment on solve
- [ ] Handle new drones (start from airport, not cut position)
- [ ] Store segment with frozen prefix

### 5.4 Segmented Animation
- [ ] Implement trajectory concatenation:
  - [ ] Combine trajectories across segments
  - [ ] Remove duplicate join points
- [ ] Detect segment transitions during animation:
  - [ ] When missionDistance reaches cutDistance
  - [ ] 1-second pause
  - [ ] Show cut markers
  - [ ] Update environment visuals (new targets, disabled targets)
  - [ ] Show lost drones (red diamonds)
  - [ ] Continue with next segment
- [ ] Implement Reset for segmented missions:
  - [ ] Return to Segment 0 start
  - [ ] Clear visited targets
  - [ ] Replay all segments

### 5.5 Segmented JSON Import/Export
- [ ] Implement export:
  - [ ] Serialize all segments to JSON
  - [ ] Include environment snapshots per segment
  - [ ] Include drone configs per segment
  - [ ] Add version field ("segmented_v1")
- [ ] Implement import:
  - [ ] Parse JSON
  - [ ] Load all segments into SegmentedMission
  - [ ] Set UI to "ready to animate" state
  - [ ] Enable immediate playback

### 5.6 Segment Manager UI
- [ ] Create `webapp/segment_manager.js`
- [ ] Add Segment dropdown/panel
- [ ] Show list of segments (S0, S1, S2...)
- [ ] Show cut markers on timeline
- [ ] Allow segment selection for review
- [ ] Show segment metadata (lost drones, environment changes)

---

## Phase 6: Agentic System (LLM-Powered)

### 6.1 Constraint Parsing Infrastructure
- [ ] Create `server/memory/constraints.py`
- [ ] Define ConstraintOp enum:
  - FORCE_VISIT, MOVE, REMOVE, SWAP, INSERT
- [ ] Create ConstraintProgram class
- [ ] Create ConstraintCompiler class:
  - [ ] Validate constraints
  - [ ] Generate solver patches
  - [ ] Detect conflicts
- [ ] Add sequencing hints support (start_with, end_with, priority_order)
- [ ] Write unit tests

### 6.2 Agent Tools
- [ ] Create `server/agents/tools/` directory
- [ ] Implement allocation editing tools:
  - [ ] assign_target_to_drone
  - [ ] move_target
  - [ ] remove_target
  - [ ] swap_targets
- [ ] Implement solver invocation tools:
  - [ ] run_solver
  - [ ] run_allocator
  - [ ] run_post_optimizer
- [ ] Implement analysis tools:
  - [ ] get_distance_matrix
  - [ ] get_fuel_usage
  - [ ] get_route_info
  - [ ] analyze_allocation

### 6.3 LangGraph Multi-Agent System
- [ ] Create `server/agents/isr_agent_multi_v4.py`
- [ ] Define MissionState schema:
  - [ ] messages, environment, drone_configs
  - [ ] intent, policy, constraint_program
  - [ ] sequencing_hints, allocation, routes
- [ ] Implement 6 agents:
  - [ ] **Strategist**: Parse user intent, understand constraints
  - [ ] **Mission Planner**: Decompose high-level goals into actions
  - [ ] **Allocator**: Edit target assignments based on reasoning
  - [ ] **Route Optimizer**: Call solver tools, evaluate results
  - [ ] **Critic**: Evaluate solutions, suggest improvements
  - [ ] **Responder**: Generate natural language explanations
- [ ] Define LangGraph workflow:
  - [ ] State transitions between agents
  - [ ] Conditional edges (iterate if critic rejects)
  - [ ] Terminal condition (solution accepted)
- [ ] Write unit tests for each agent

### 6.4 Coordinator (Deterministic Pre-Pass)
- [ ] Create `server/agents/coordinator_v4.py`
- [ ] Implement deterministic analysis before LLM:
  - [ ] Parse simple commands with regex
  - [ ] Validate feasibility (fuel, target existence)
  - [ ] Reject impossible requests early
  - [ ] Provide context to LLM agents
- [ ] Write unit tests

### 6.5 Supabase Memory (Optional)
- [ ] Set up Supabase project
- [ ] Create `server/memory/` directory
- [ ] Implement conversation memory storage:
  - [ ] Store user messages and LLM responses
  - [ ] Store successful allocations for learning
  - [ ] Store constraint patterns
- [ ] Implement memory retrieval:
  - [ ] Fetch relevant past conversations
  - [ ] Provide as context to agents
- [ ] Add toggle for memory (optional feature)

### 6.6 Agentic API Endpoint
- [ ] Implement `/api/agent/chat` (POST):
  - [ ] Accept user message + current mission state
  - [ ] Run coordinator pre-pass
  - [ ] Invoke LangGraph multi-agent system
  - [ ] Return:
    - LLM response message
    - Updated allocation/routes
    - Explanation
    - Success/failure status
- [ ] Add streaming support for long operations
- [ ] Add error handling for LLM failures

### 6.7 Agentic UI Integration
- [ ] Add "Agent Chat" panel to webapp
- [ ] Add message input box
- [ ] Add conversation history display
- [ ] Implement chat message sending:
  - [ ] Call `/api/agent/chat`
  - [ ] Display LLM response
  - [ ] Update routes/trajectories on map
  - [ ] Show allocation changes
- [ ] Add toggle between Heuristic/Agentic modes
- [ ] Show thinking/loading indicator during agent work

---

## Phase 7: Testing & Quality Assurance

### 7.1 Unit Tests
- [ ] Write tests for SAM-aware distance matrix
- [ ] Write tests for orienteering solver (verify optimal solutions)
- [ ] Write tests for all 5 allocation strategies
- [ ] Write tests for all 3 post-optimizers
- [ ] Write tests for constraint parser/compiler
- [ ] Write tests for each agent in multi-agent system
- [ ] Achieve >80% code coverage

### 7.2 Integration Tests
- [ ] Test full solve pipeline (end-to-end)
- [ ] Test segmented mission workflow (cut → solve → accept)
- [ ] Test agentic mode with sample conversations
- [ ] Test import/export of segmented missions
- [ ] Test API endpoints with various inputs

### 7.3 UI/UX Testing
- [ ] Test all editing operations (add, move, delete)
- [ ] Test animation with various missions
- [ ] Test segmented animation with multiple cuts
- [ ] Test responsive behavior (window resize)
- [ ] Test error handling (invalid inputs, solver failures)
- [ ] Test browser compatibility (Chrome, Firefox, Safari)

### 7.4 Performance Testing
- [ ] Benchmark solver with 5/10/20 targets
- [ ] Optimize orienteering solver (memoization, pruning)
- [ ] Optimize distance matrix calculation (caching)
- [ ] Test animation performance with complex trajectories
- [ ] Profile agent reasoning time
- [ ] Test concurrent API requests

---

## Phase 8: Documentation & Examples

### 8.1 User Documentation
- [ ] Write comprehensive README with:
  - [ ] Feature overview
  - [ ] Installation instructions
  - [ ] Usage tutorial with screenshots
  - [ ] Troubleshooting guide
- [ ] Create example mission files:
  - [ ] Simple 2-drone, 5-target mission
  - [ ] Complex 5-drone, 20-target mission with SAMs
  - [ ] Segmented mission example
- [ ] Create video walkthrough (optional)

### 8.2 Developer Documentation
- [ ] Write SYSTEM_ARCHITECTURE.md with:
  - [ ] High-level system overview
  - [ ] Component descriptions
  - [ ] Data flow diagrams
  - [ ] Segmented mission mechanics
  - [ ] Agentic reasoning philosophy
- [ ] Write MODULAR_ARCHITECTURE.md with:
  - [ ] Code organization
  - [ ] Refactoring plans
  - [ ] Module responsibilities
- [ ] Write API documentation:
  - [ ] Document all endpoints
  - [ ] Request/response schemas
  - [ ] Example API calls

### 8.3 AI Assistant Context
- [ ] Write CLAUDE.md with:
  - [ ] Common development commands
  - [ ] Architecture overview
  - [ ] Known gotchas
  - [ ] Testing instructions
- [ ] Create session summaries for complex sessions

---

## Phase 9: Deployment

### 9.1 Railway Deployment (Backend + Frontend)
- [ ] Create Railway project
- [ ] Configure build settings:
  - [ ] Python 3.9+
  - [ ] Install dependencies from requirements.txt
  - [ ] Set start command: `uvicorn server.main:app --port $PORT`
- [ ] Set environment variables:
  - [ ] ANTHROPIC_API_KEY
  - [ ] SUPABASE_URL, SUPABASE_KEY (if using)
- [ ] Configure static file serving for webapp
- [ ] Test deployment

### 9.2 GitHub Pages (Frontend Only - Alternative)
- [ ] Create separate frontend-only branch
- [ ] Configure API URL to point to Railway backend
- [ ] Deploy to GitHub Pages
- [ ] Test CORS configuration

### 9.3 Docker Containerization (Optional)
- [ ] Create Dockerfile
- [ ] Create docker-compose.yml
- [ ] Test local Docker deployment
- [ ] Document Docker deployment in DEPLOYMENT.md

### 9.4 Production Hardening
- [ ] Add rate limiting to API endpoints
- [ ] Add input validation and sanitization
- [ ] Add HTTPS enforcement
- [ ] Add error tracking (e.g., Sentry)
- [ ] Add analytics (optional)
- [ ] Set up monitoring/alerting

---

## Phase 10: Future Enhancements

### 10.1 Advanced Features
- [ ] Add real-time collaboration (multiple users)
- [ ] Add 3D visualization option
- [ ] Add terrain/elevation data
- [ ] Add weather/wind constraints
- [ ] Add time windows for targets
- [ ] Add multi-objective optimization (Pareto fronts)

### 10.2 Agentic Improvements
- [ ] Add voice input for natural language commands
- [ ] Add multi-turn reasoning with clarification questions
- [ ] Add learning from user feedback (RLHF-style)
- [ ] Add proactive suggestions ("I noticed Drone 2 is underutilized...")

### 10.3 Performance Optimizations
- [ ] Parallelize per-drone solving
- [ ] Add GPU acceleration for large problems
- [ ] Implement incremental solving (warm-start)
- [ ] Add problem decomposition for very large missions

### 10.4 Refactoring (from MODULAR_ARCHITECTURE.md)
- [ ] Break isr.js into modules (renderer, mission_control, etc.)
- [ ] Consolidate segment manager implementations
- [ ] Simplify state management (single source of truth)
- [ ] Remove agent version sprawl (keep only v4)

---

## Dependency Graph (Critical Path)

**Sequential Dependencies:**
1. Phase 1 (Infrastructure) → Phase 2 (Core Solver)
2. Phase 2 (Core Solver) → Phase 3 (API Server)
3. Phase 3 (API Server) + Phase 4 (Frontend) → Phase 5 (Segmented Missions)
4. Phase 5 (Segmented) → Phase 6 (Agentic System)
5. Phase 6 (Agentic) → Phase 7 (Testing)
6. Phase 7 (Testing) → Phase 8 (Documentation) → Phase 9 (Deployment)

**Parallelizable:**
- Phase 3 (API) and Phase 4 (Frontend) can be developed in parallel
- Phase 6.1-6.3 (Constraint parsing, tools, agents) can partially overlap
- Phase 8 (Documentation) can be written incrementally throughout

---

## Estimated Effort

**Total Estimated Hours:** 400-600 hours (full-time: 10-15 weeks)

**Phase Breakdown:**
- Phase 1: 8 hours
- Phase 2: 80 hours (solver algorithms are complex)
- Phase 3: 20 hours
- Phase 4: 100 hours (UI/UX is time-consuming)
- Phase 5: 60 hours (segmented missions are complex)
- Phase 6: 100 hours (multi-agent system is sophisticated)
- Phase 7: 60 hours
- Phase 8: 30 hours
- Phase 9: 20 hours
- Phase 10: 50+ hours (future work)

**Note:** This assumes one experienced developer. A team of 2-3 could parallelize and reduce calendar time significantly.

---

## Quick Start Minimum Viable Product (MVP)

If building an MVP first, complete these tasks in order:

1. Phase 1 (all)
2. Phase 2.1 (distance matrix), 2.2 (orienteering solver), 2.4 (GREEDY allocator only), 2.5 (trajectory)
3. Phase 3.1, 3.2 (basic solve endpoint only)
4. Phase 4.1, 4.2, 4.3, 4.5 (basic UI + mission control)
5. Phase 7.2 (basic integration tests)
6. Phase 8.1 (minimal README)

**MVP Estimated Effort:** 120-150 hours (3-4 weeks full-time)

This MVP would provide:
- Environment editing
- Basic solving (GREEDY allocation, orienteering solver)
- Visualization and animation
- Save/load missions

Then iterate to add:
- Segmented missions (Phase 5)
- Agentic system (Phase 6)
- Additional optimizers and allocators
