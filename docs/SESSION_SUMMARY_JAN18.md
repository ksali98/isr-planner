# Session Summary - January 18, 2026

## Context
Continuing work on the ISR Planner agentic system, focusing on constraint handling for natural language mission planning.

## Key Discussion: LLM-Tool Architecture for Constraints

### User's Vision (Important!)
The user explicitly rejected regex-based constraint parsing as the primary approach:
> "Oh no. That is not the way to go... if we are going to write code for every constraint, we will never end"

**Desired Architecture:**
- **LLM's Role**: Understand natural language, parse intent, call appropriate tools with structured parameters
- **Tool's Role**: Execute deterministic logic (distance calculations, solving, trajectory joining)
- The constraint parser should be a fallback/helper, not the primary parsing mechanism

### Segmented Missions - Key Concept

User explained the segmented mission JSON format with naming convention:
- `isr_env2601070926_N5_2.json` = 5 segments (N5), version 2

**Segmented Mission Flow:**
1. Drone starts flying
2. Environment change happens (e.g., new targets appear)
3. Execution stops, trajectory behind drone is **frozen**
4. Drone position becomes a **synthetic_start**
5. Solve with remaining unfrozen targets
6. Accept solution, drone flies again
7. Repeat until mission complete

**JSON Structure (from `isr_env2601070926_N5_2.json`):**
```json
{
  "schema": "isr_env_v1",
  "is_segmented": true,
  "segment_count": 5,
  "drone_configs": {...},
  "segments": [
    {
      "index": 0,
      "env": { "airports": [...], "targets": [...], "sams": [...] },
      "cutDistance": null,  // null for first segment
      "cutPositions": null,  // null for first segment
      "drone_configs": {...}
    },
    {
      "index": 1,
      "env": {...},  // Targets filtered - visited ones removed, new ones added
      "cutDistance": 15.33,  // Distance traveled when cut happened
      "cutPositions": {"1": [x,y], "2": [x,y]},  // Drone positions at cut
      "drone_configs": {...}
    },
    // ... more segments
  ]
}
```

**Key Fields:**
- `cutPositions` = synthetic starts for next solve
- `env.targets` = only remaining/new targets (visited ones removed)
- `cutDistance` = distance flown when cut occurred

### Existing Infrastructure (Confirmed Working!)

User tested segmented JSONs and confirmed: **"Works."**

The following tools/infrastructure already support segmented solves:
1. **`synthetic_starts`** in env - handled in `solver_bridge.py` and `build_id_map()`
2. **`visited_targets`** filter in `solve_with_allocation` endpoint
3. **`is_checkpoint_replan`** flag
4. **Mission Executive** with `CUT_AND_FREEZE` action

### How to Implement "Start with Priority 10"

Using segmented solve approach:
1. **Segment 0**: Filter targets to only priority >= 10, solve
2. **Capture positions**: Get drone end positions after visiting priority-10 targets
3. **Segment 1**: Create synthetic starts from those positions, solve with remaining targets

## Files Changed This Session

### New Files
- `server/memory/constraint_parser.py` - Regex-based constraint parsing (fallback helper)
- `server/memory/constraints.py` - Constraint data structures
- `server/memory/session_store.py` - Session state management
- `server/memory/trim.py` - Utility functions
- `docs/AGENTIC_SYSTEM_DESCRIPTION.md` - System documentation

### Modified Files
- `server/main.py` - Agent imports refactored to use v3
- `server/agents/isr_agent_multi_v3.py` - Enhanced tool calling
- `server/agents/isr_agent_multi_v4.py` - Improved reasoning
- `webapp/isr.js` - Cleanup
- `webapp/index.html` - Minor updates

## Next Steps (User's Intent)

Design and implement tools the LLM can call to orchestrate segmented solves:
1. **Tool: Filter targets by priority** - Returns subset of targets meeting criteria
2. **Tool: Create synthetic starts** - From drone positions/route waypoints
3. **Tool: Segmented solve** - Solve with filtered targets and synthetic starts
4. **Tool: Join trajectories** - Combine frozen + new segments

The LLM should be able to understand "start with priority 10" and orchestrate:
```
1. Call filter_targets(priority_gte=10)
2. Call solve(targets=filtered_targets)
3. Extract end positions from solution
4. Call create_synthetic_starts(positions)
5. Call solve(targets=remaining_targets, synthetic_starts=positions)
6. Call join_trajectories(segment1, segment2)
```

## Important Code Locations

- Segmented JSON examples: `~/Downloads/isr_env*_N[2-9]*.json`
- Synthetic starts handling: `server/solver/solver_bridge.py:370-390`
- Visited targets filter: `server/main.py:1259-1269`
- Mission Executive: `server/agents/mission_executive.py`
- Constraint parser: `server/memory/constraint_parser.py`

## Commit
```
42ee2fd feat: add constraint_parser module and refactor agent imports
```

Pushed to `origin/main`.
