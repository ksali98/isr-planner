# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ISR Mission Planner is a web-based tool for planning multi-drone ISR (Intelligence, Surveillance, Reconnaissance) missions. Drones must visit prioritized targets while avoiding SAM (Surface-to-Air Missile) zones, starting and ending at airports within fuel constraints.

**Tech Stack:**
- **Backend**: Python FastAPI with LangGraph v4 multi-agent system
- **Frontend**: Vanilla JavaScript (single monolithic `isr.js` file)
- **AI**: Claude (Anthropic) for reasoning-based mission planning
- **Solver**: Held-Karp dynamic programming orienteering algorithm

## Development Commands

### Running the Server

```bash
# Setup environment (first time only)
cp .env.example .env
# Edit .env and add ANTHROPIC_API_KEY=sk-ant-xxxxx...

# Run server (preferred method)
./run_planner.sh

# Or manually
source venv/bin/activate  # if using venv
export PYTHONPATH=$(pwd)
python3 -m uvicorn server.main:app --reload --port 8893
```

Server runs on **http://localhost:8893**

### Installing Dependencies

```bash
pip install -r requirements.txt
```

### Environment Variables

Required in `.env`:
- `ANTHROPIC_API_KEY` - Claude API key for agent reasoning
- `PORT=8893` - Server port (default)
- `PYTHONPATH` - Set to project root for imports

Optional:
- `SUPABASE_URL`, `SUPABASE_KEY` - Database storage (falls back to local JSON)
- `OPENAI_API_KEY` - For GPT agent option (not actively used)

## Architecture: Big Picture

### Three-Layer System

```
┌─────────────────────────────────────────┐
│  Frontend (webapp/isr.js - 328KB!)      │  ← Monolithic JavaScript
│  - Canvas rendering                      │
│  - Mission control UI                    │
│  - Segment management                    │
└─────────────────┬───────────────────────┘
                  │ HTTP/JSON
┌─────────────────▼───────────────────────┐
│  FastAPI Server (server/main.py)        │  ← 2,352 lines, all endpoints
│  - /solve, /optimize, /agent endpoints  │
└─────────────────┬───────────────────────┘
                  │
        ┌─────────┴──────────┐
        │                    │
┌───────▼────────┐  ┌───────▼────────┐
│  LangGraph v4  │  │  Solver Bridge │
│  Agent System  │  │  (algorithms)  │
└────────────────┘  └────────────────┘
```

### Critical Architectural Constraints

1. **Agent Version**: Only **v4** (`server/agents/isr_agent_multi_v4.py`) is active
   - v1, v2, v3 exist but are legacy/archived
   - See `docs/AGENTIC_SYSTEM_DESCRIPTION.md` for v4 details

2. **PYTHONPATH Requirement**: Must set `PYTHONPATH` to project root or imports fail
   - Example: `export PYTHONPATH=/Users/you/isr-planner/isr-planner`
   - `run_planner.sh` handles this automatically

3. **Orienteering Solver Location**: `orienteering_with_matrix.py` must be at project root
   - NOT in subdirectories
   - Loaded by `webapp/editor/solver/orienteering_interface.py`

4. **Frontend is Monolithic**: `webapp/isr.js` is 328KB single file
   - Handles everything: rendering, UI, API, animation, segments
   - See `docs/MODULAR_ARCHITECTURE.md` for planned refactoring

### Key Architectural Patterns

**LangGraph Agent Flow (v4):**
```
User Request
    ↓
Coordinator V4 (deterministic pre-pass)
    ↓
LangGraph State Machine:
    Strategist → Allocator → Route Optimizer → Critic → Responder
    ↑___________________________________________|
```

**Solving Pipeline:**
```
Environment + Drone Configs
    ↓
Target Allocator (5 strategies: efficient/greedy/balanced/geographic/exclusive)
    ↓
Orienteering Solver (Held-Karp DP per drone)
    ↓
Trajectory Planner (SAM boundary navigation)
    ↓
Post-Optimization (Insert/Swap/No-Cross)
```

**Segmented Mission System:**
- Missions can be "cut" into segments at checkpoints
- Each segment has its own solution (routes, trajectories)
- Segments concatenate for animation/replay
- See `docs/SYSTEM_ARCHITECTURE.md` for detailed design

## Code Organization

### Backend Structure

```
server/
  main.py                      # FastAPI app + ALL endpoints (2,352 lines)

  agents/
    isr_agent_multi_v4.py      # ACTIVE - v4 multi-agent system (3,031 lines)
    coordinator_v4.py          # Deterministic pre-pass coordinator
    mission_tools.py           # Agent tools for mission analysis
    mission_orchestration_tools.py  # Orchestration singleton

    versions/
      isr_agent_multi_v3.py    # LEGACY - task-based agents
      isr_agent_multi_v2.py    # LEGACY - original multi-agent
      isr_agent_single_v1.py   # LEGACY - single agent GPT-based

  solver/
    solver_bridge.py           # Main solving orchestrator (1,087 lines)
    target_allocator.py        # 5 allocation strategies (911 lines)
    post_optimizer.py          # Insert/Swap/No-Cross optimizers (2,078 lines)
    sam_distance_matrix.py     # SAM-aware distance calculations
    trajectory_planner.py      # SAM boundary navigation
    orienteering_solver.py     # Delivery wrapper (not used for ISR)

  memory/
    constraints/               # Constraint parsing
    sessions/                  # Session state management

  database/
    mission_ledger.py          # Decision trace, agent run tracking
    supabase_client.py         # Database connection (optional)
```

### Frontend Structure

```
webapp/
  index.html                   # Main HTML
  isr.js                       # MONOLITHIC 328KB file (all frontend logic)
  isr.css                      # Styling
  segment_manager.js           # Segment UI controls
  segmented_mission.js         # Segmented mission state

  editor/                      # Legacy Python pygame editor (not actively used)
```

### Solver Algorithm Location

```
orienteering_with_matrix.py  # ROOT LEVEL - Held-Karp DP solver (436 lines)
```

## Important Files for Common Tasks

### Modifying Agent Behavior
- `server/agents/isr_agent_multi_v4.py` - Main agent logic
- `server/agents/coordinator_v4.py` - Intent classification, policy selection
- `docs/AGENTIC_SYSTEM_DESCRIPTION.md` - Agent prompts and architecture

### Modifying Solving Algorithm
- `server/solver/solver_bridge.py` - Orchestration logic
- `server/solver/target_allocator.py` - Target allocation strategies
- `orienteering_with_matrix.py` - Core TSP/orienteering solver

### Modifying Optimization
- `server/solver/post_optimizer.py` - Insert Missed, Swap Closer, No-Cross

### Modifying UI
- `webapp/isr.js` - ALL frontend logic (search for function names)
- Key sections:
  - Canvas rendering: `drawEnvironment()`, `renderCanvas()`
  - Mission control: `runPlanner()`, `acceptSolution()`
  - Segments: `freezeAtCheckpoint()`, `commitSegment()`
  - Animation: `animateMission()`, `updateAnimationFrame()`

### API Endpoints (all in `server/main.py`)
- `/solve` - Basic solving
- `/solve-with-allocation` - Solving with target allocation
- `/insert_missed` - Insert unvisited targets
- `/swap_closer` - Swap optimization
- `/crossing_removal` - Remove route crossings
- `/agent/plan` - LangGraph agent planning
- `/agent/chat` - Agent conversational interface

## Known Gotchas

### Import Errors
**Problem**: `ModuleNotFoundError: No module named 'server'` or `'orienteering_with_matrix'`
**Solution**: Ensure `PYTHONPATH` is set to project root before running server

### JavaScript Not Loading
**Problem**: Buttons don't work, UI frozen
**Solution**: Hard refresh browser (`Cmd+Shift+R`) to clear cached JS

### Solver Not Found
**Problem**: `Could not import REAL orienteering solver`
**Solution**: Ensure `orienteering_with_matrix.py` exists at project root (not in subdirectory)

### Port Already in Use
**Problem**: `Address already in use` on port 8893
**Solution**: Run `lsof -ti:8893 | xargs kill` to kill existing server

### Agent Reasoning Errors
**Problem**: Agent gives generic response or fails to plan
**Solution**: Check `ANTHROPIC_API_KEY` is valid and has credits

## Documentation Map

Essential reading for understanding the system:

- **SYSTEM_ARCHITECTURE.md** - Segmented mission system design
- **MODULAR_ARCHITECTURE.md** - Planned code refactoring (frontend/backend)
- **AGENTIC_SYSTEM_DESCRIPTION.md** - v4 agent architecture, prompts, tools
- **MISSION_ORCHESTRATION_TOOLS.md** - API for agents to orchestrate missions
- **V3_TO_V4_MIGRATION.md** - Differences between agent versions
- **DEPLOYMENT.md** - Railway deployment with Supabase

## Key Design Decisions

### Why Monolithic isr.js?
Historical evolution - grew from small prototype. Planned refactoring in `MODULAR_ARCHITECTURE.md`.

### Why Multiple Agent Versions?
Evolutionary development - v4 is production, v1-v3 kept for reference/comparison.

### Why PYTHONPATH Required?
Project uses relative imports (`from server.solver import ...`) that require project root in path.

### Why Held-Karp DP Solver?
Optimal solution for small TSP instances (up to ~15 targets per drone). Fast enough for real-time planning.

### Why Segmented Missions?
Real missions change mid-flight (drone failures, new targets). Segments allow replanning from current positions.
