# ISR Multi-Agent Mission Planning System - Technical Description

## Overview

This document describes the agentic architecture of the ISR (Intelligence, Surveillance, Reconnaissance) Mission Planning System. The system uses a **reasoning-based multi-agent architecture** built on LangGraph and Claude (Anthropic) to plan optimal drone missions that visit prioritized targets while avoiding SAM (Surface-to-Air Missile) zones and respecting fuel constraints.

**Primary Goal**: Maximize total priority points collected by a fleet of drones while respecting hard constraints (fuel budgets, sensor compatibility, SAM avoidance).

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              USER REQUEST                                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         COORDINATOR V4 (Pre-pass)                            │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐              │
│  │ Intent Classify │→ │ Validate Env    │→ │ Select Policy   │              │
│  │ (deterministic) │  │ (guardrails)    │  │ (strategy)      │              │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘              │
│  Output: intent, policy, constraints, drone_contracts                        │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         LANGGRAPH STATE MACHINE                              │
│                                                                              │
│   ┌──────────────┐                                                          │
│   │  STRATEGIST  │──────────────────────────────────────────┐               │
│   │  (LLM Agent) │                                          │               │
│   └──────┬───────┘                                          │               │
│          │                                                  │               │
│          ├── question? ─────────────────────────────────────┤               │
│          │                                                  │               │
│          ▼ optimize/reallocate                              │               │
│   ┌──────────────┐                                          │               │
│   │  ALLOCATOR   │                                          │               │
│   │  (LLM Agent) │                                          │               │
│   └──────┬───────┘                                          │               │
│          │                                                  │               │
│          ▼                                                  │               │
│   ┌──────────────┐                                          │               │
│   │    ROUTE     │                                          │               │
│   │  OPTIMIZER   │                                          │               │
│   │ (Algorithmic)│                                          │               │
│   └──────┬───────┘                                          │               │
│          │                                                  │               │
│          ▼                                                  │               │
│   ┌──────────────┐                                          │               │
│   │    CRITIC    │                                          │               │
│   │  (LLM Agent) │                                          │               │
│   └──────┬───────┘                                          │               │
│          │                                                  │               │
│          ▼                                                  ▼               │
│   ┌─────────────────────────────────────────────────────────────┐           │
│   │                       RESPONDER                              │           │
│   │                       (LLM Agent)                            │           │
│   └─────────────────────────────────────────────────────────────┘           │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         POST-OPTIMIZATION TOOLS                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                          │
│  │Insert Missed│  │    Swap     │  │  No Cross   │                          │
│  │  (Insert)   │  │  (Cascade)  │  │  (2-opt)    │                          │
│  └─────────────┘  └─────────────┘  └─────────────┘                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Details

### 1. Coordinator V4 (Deterministic Pre-pass)

**File**: `server/agents/coordinator_v4.py`

The Coordinator is a **non-LLM, deterministic policy layer** that runs BEFORE the LangGraph workflow. It provides guardrails and injects policy decisions into the agent state.

#### Intent Classification

Uses token matching to classify user intent:

| Intent | Trigger Tokens | Example |
|--------|---------------|---------|
| `debug` | "error", "traceback", "crash", "bug", "broken" | "Why is the solver crashing?" |
| `explain` | "explain", "why", "how", "describe" | "Explain why D1 visits T5" |
| `what_if` | "what if", "compare", "instead", "different" | "What if we change fuel to 300?" |
| `replan` | "cut", "checkpoint", "continue", "replan" | "Continue from checkpoint" |
| `reallocate` | "move", "reassign", "transfer" + target patterns | "Move T5 to D1" |
| `plan` | "plan", "optimize", "solve", "compute" | "Generate optimal routes" |

#### Allocation Modification Parsing

Detects and parses commands like:
- "move T5 to D1"
- "reassign T8 and T10 to drone 2"
- "transfer T3 from D1 to D2"

Returns structured modifications: `[{"target": "T5", "to_drone": "1"}, ...]`

#### CoordinatorDecision Output

```python
@dataclass
class CoordinatorDecision:
    intent: str                    # plan|replan|reallocate|explain|what_if|debug
    confidence: float              # 0.0-1.0
    rules_hit: List[str]           # Which classification rules matched

    policy: Dict[str, Any]         # Controls downstream behavior
    # policy.allow_solver: bool
    # policy.force_allocation: bool
    # policy.allocation_strategy: str  ("efficient"|"greedy"|"balanced"|"geographic"|"exclusive")
    # policy.post_opt: {crossing_removal, trajectory_swap, insert_unvisited}

    constraints: Dict[str, Any]    # Extracted from user message
    # constraints.moves: List[{target, to_drone}]
    # constraints.required_targets: List[str]
    # constraints.forbidden_targets: List[str]

    explanation_only: bool         # If True, skip solver entirely
    warnings: List[str]
    errors: List[str]
    trace_events: List[Dict]       # Structured trace for debugging
```

---

### 2. Mission State (Shared Context)

**File**: `server/agents/isr_agent_multi_v4.py`

All agents share a typed state dictionary:

```python
class MissionState(TypedDict):
    # Conversation
    messages: Annotated[list, add_messages]

    # Mission data
    environment: Dict[str, Any]        # airports, targets, SAMs
    drone_configs: Dict[str, Any]      # fuel, sensors, home airports
    distance_matrix: Dict[str, Any]    # SAM-aware distances
    excluded_targets: List[str]        # Targets inside SAM zones

    # Request analysis
    user_request: str
    request_type: str                  # "question"|"optimize"|"command"

    # Coordinator outputs
    intent: str
    policy: Dict[str, Any]
    allocation_strategy: str

    # Agent reasoning outputs
    strategy_analysis: str
    allocation_reasoning: str
    route_analysis: str
    critic_review: str
    suggestions: List[str]

    # Solution data
    allocation: Dict[str, List[str]]   # {drone_id: [target_ids]}
    routes: Dict[str, Dict[str, Any]]  # {drone_id: {route, distance, points}}

    # Final
    final_response: str
    error: str
```

---

### 3. Hard Constraints (Inviolable)

These constraints are injected into ALL agent prompts:

```
1. DRONE TARGET ELIGIBILITY (Sensor Type Restrictions):
   - Each drone has an "ELIGIBLE TARGETS" list based on its sensor configuration
   - A drone can ONLY be assigned targets from its eligible list
   - This is a PHYSICAL constraint and CANNOT be overridden

2. EXCLUDED TARGETS (Inside SAM Zones):
   - Targets marked as "EXCLUDED (inside SAM zones)" are UNREACHABLE
   - No drone can visit excluded targets

3. FUEL BUDGET:
   - Each drone has a fuel budget that cannot be exceeded
   - Routes must respect fuel constraints

4. ALL ELIGIBLE TARGETS MUST BE COVERED:
   - Every target that is NOT excluded must be assigned to exactly one drone
   - The assigned drone MUST be eligible to visit that target
```

---

### 4. Agent Prompts

#### 4.1 STRATEGIST Agent

**Purpose**: Analyze user request and determine approach

**Input Context**:
- Hard constraints
- Mission context (airports, targets, SAMs, drone configs)
- Current solution summary with metrics (fuel_used, fuel_budget, fuel_margin, points, unvisited_targets)
- User request

**Output Format** (structured text):
```
REQUEST_TYPE: <question | optimize | reallocate | reroute | evaluate | debug>
MODE: <answer_question | optimize_freely | suggest_reallocation | suggest_reroute | analyze_solution>
COMMANDS_DETECTED: <comma-separated list or "none">
CONSTRAINTS_SUMMARY: <1-2 sentences>
NEEDS_ALLOCATION: <true | false>
NEEDS_ROUTING: <true | false>
FEASIBILITY: <feasible | infeasible | unknown>
RATIONALE:
- <bullet citing specific metrics>
- <bullet citing specific metrics>
```

**Key Behaviors**:
- If user says "do not recompute routes" → REQUEST_TYPE: question, skip allocation/routing
- Must ground reasoning in NUMBERS from mission metrics
- Routes to Responder if question, to Allocator if optimization needed

#### 4.2 ALLOCATOR Agent

**Purpose**: Assign targets to drones to maximize priority points

**Input Context**:
- Hard constraints
- Mission context
- Strategist's analysis
- Allocation strategy from Coordinator

**Output Format** (JSON):
```json
{
  "strategy_used": "efficient|greedy|balanced|geographic|exclusive",
  "assignments": {
    "D1": ["T1", "T3", "T5"],
    "D2": ["T2", "T4"]
  },
  "excluded": [
    {"target": "T10", "reason": "IN_SAM_ZONE", "notes": "optional"}
  ],
  "rationale": {
    "D1": "High-value targets T1, T3, T5 within fuel budget",
    "D2": "Remaining targets efficiently clustered"
  },
  "tradeoffs": "Prioritized coverage over fuel efficiency"
}
```

**Allocation Strategies**:
| Strategy | Description |
|----------|-------------|
| `efficient` | Maximize priority/fuel ratio (auction style) |
| `greedy` | Highest priority to nearest capable drone |
| `balanced` | Distribute workload evenly across drones |
| `geographic` | Minimize detours, corridor-based assignment |
| `exclusive` | Prioritize targets only one drone can visit |

**Key Constraint**: Prefers ≤12 targets per drone (optimal for Held-Karp solver)

#### 4.3 ROUTE_OPTIMIZER Agent

**Purpose**: Compute optimal routes for allocated targets

**Algorithm**: Uses **Held-Karp** (exact TSP solver) via `OrienteeringSolverInterface`

**Input**:
- Target allocation per drone
- Drone configurations (start/end airports, fuel budgets)
- SAM-aware distance matrix

**Output**:
- Per-drone routes with ordered waypoints
- Fuel usage and feasibility status
- Points collected per route

**Key Features**:
- Computes exact optimal TSP route for each drone's allocated targets
- Respects fuel budgets (infeasible routes flagged)
- Uses SAM-aware distances (paths avoid SAM zones)

#### 4.4 CRITIC Agent

**Purpose**: Review solution quality and suggest improvements

**Input**:
- Full mission context
- Computed routes and allocations
- Strategist's analysis

**Output Format**:
```
REVIEW: <overall assessment>
ISSUES: <problems found or "None">
SUGGESTIONS: <improvement ideas with quantified benefits>
FINAL_VERDICT: <APPROVED | NEEDS_REVISION | REJECTED>
```

**Checks**:
- Violations of user commands or constraints
- Potential reassignments for better efficiency
- Fuel margin optimization opportunities
- Unvisited targets that could be added

#### 4.5 RESPONDER Agent

**Purpose**: Formulate final answer to user

**Key Rules**:
1. Answer using ONLY stored mission context and current solution summary
2. Never recompute routes unless explicitly instructed
3. ALWAYS cite specific numbers: fuel usage, budgets, margins, points
4. Be precise, concise, and numerical
5. Compare using numeric values for evaluative questions

**Modes**:
- Question response: Answer about existing solution
- Solution response: Present new/modified plan

---

### 5. Analysis Tools (Read-Only)

**File**: `server/agents/mission_tools.py`

#### 5.1 `compute_solution_metrics()`

Computes mission metrics from a solution:
- `total_points`: Sum of priority points collected
- `total_possible_points`: Sum of all target priorities
- `points_coverage`: Fraction achieved
- `total_fuel`: Total fuel consumed
- Per-drone stats: points, fuel_used, fuel_budget, fuel_margin, visited_targets, unvisited_accessible_targets

#### 5.2 `analyze_solution_geometry()`

Analyzes solution geometry for problems:
- `crossings`: List of non-frozen segment crossings between drones
- `acute_angles`: Sharp turns at targets (angle < threshold)
- `frozen_segments`: All frozen segments per drone

---

### 6. Post-Optimization Tools

After the main planning workflow, three post-optimization tools can improve the solution:

#### 6.1 Insert (Insert Missed)
- Finds unvisited targets
- Attempts to insert them into existing routes with leftover fuel
- Uses greedy insertion at best position

#### 6.2 Swap (Cascade Swap Closer)
- Swaps targets between drones to reduce total distance
- Uses removal/insertion delta calculations
- Supports cascade mode: iteratively swaps until convergence
- Can regenerate SAM-aware trajectories between passes

#### 6.3 No Cross (Crossing Removal)
- Uses 2-opt algorithm to remove self-crossings within routes
- Improves route geometry without changing allocation

---

### 7. LangGraph Workflow

**File**: `server/agents/isr_agent_multi_v4.py` (lines 1642-1677)

```python
workflow = StateGraph(MissionState)

# Add nodes
workflow.add_node("strategist", strategist_node)
workflow.add_node("allocator", allocator_node)
workflow.add_node("route_optimizer", route_optimizer_node)
workflow.add_node("critic", critic_node)
workflow.add_node("responder", responder_node)

# Entry point
workflow.set_entry_point("strategist")

# Conditional routing from strategist
workflow.add_conditional_edges(
    "strategist",
    route_after_strategist,  # Routes to "responder" if question, "allocator" if optimize
    {"responder": "responder", "allocator": "allocator"}
)

# Sequential edges
workflow.add_edge("allocator", "route_optimizer")
workflow.add_edge("route_optimizer", "critic")
workflow.add_edge("critic", "responder")
workflow.add_edge("responder", END)
```

---

### 8. Mission Executive (Future)

**File**: `server/agents/mission_executive.py`

A persistent controller for observe-decide-act loops during mission execution:

**Actions**:
- `CONTINUE`: Keep executing current plan
- `PAUSE`: Pause execution
- `CUT_AND_FREEZE`: Cut at checkpoint, freeze completed segments
- `REPLAN_REMAINDER`: Replan from current position
- `DRAFT_READY`: New plan ready for review
- `COMMIT_PLAN`: User approved plan
- `RESET`: Start over

**Event Types**:
- `HUMAN_COMMAND`: User issued command (CUT, PAUSE, RESET, ACCEPT, REJECT)
- `ENV_EDITS`: Environment was modified (triggers replan)

---

## Data Flow Example

### User Request: "Optimize the mission to maximize points"

1. **Coordinator** classifies intent as `plan`, sets `allocation_strategy: efficient`

2. **Strategist** analyzes:
   ```
   REQUEST_TYPE: optimize
   MODE: optimize_freely
   NEEDS_ALLOCATION: true
   NEEDS_ROUTING: true
   ```

3. **Allocator** produces allocation:
   ```json
   {
     "strategy_used": "efficient",
     "assignments": {"D1": ["T1","T3","T5"], "D2": ["T2","T4","T6"]}
   }
   ```

4. **Route Optimizer** computes optimal TSP routes using Held-Karp

5. **Critic** reviews:
   ```
   REVIEW: Solution achieves 85% point coverage with 12% fuel margin
   ISSUES: None
   FINAL_VERDICT: APPROVED
   ```

6. **Responder** formulates answer with metrics

---

## LLM Configuration

All LLM agents use:
```python
llm = ChatAnthropic(model="claude-sonnet-4-20250514", temperature=0)
```

---

## Key Design Decisions

1. **Deterministic Pre-pass**: Coordinator handles intent classification without LLM to ensure consistent behavior

2. **Structured Outputs**: Each agent has a defined output format to enable reliable parsing

3. **Hard Constraint Injection**: Constraints are included in every agent prompt to prevent violations

4. **Separation of Concerns**:
   - Strategist: Request analysis
   - Allocator: Target assignment (what)
   - Route Optimizer: Path planning (how)
   - Critic: Quality assurance
   - Responder: User communication

5. **Metrics-Grounded Reasoning**: Agents must cite specific numbers from solution metrics

6. **Algorithmic Core**: Route optimization uses exact algorithms (Held-Karp), not LLM generation

---

## Areas for Potential Improvement

1. **Tool Use**: Agents could be given explicit tools (function calling) instead of structured text outputs

2. **Memory/Learning**: System could learn from past missions to improve allocation strategies

3. **Parallel Execution**: Allocator and Route Optimizer could potentially run in parallel for different drones

4. **Streaming**: Real-time streaming of agent reasoning to UI

5. **Feedback Loop**: Critic suggestions could automatically trigger re-optimization

6. **Multi-turn Refinement**: Allow iterative refinement through conversation

7. **Uncertainty Handling**: Better handling of uncertain or ambiguous user requests

8. **Explanation Quality**: More detailed explanations of trade-offs and decisions

---

## File Reference

| Component | File Path |
|-----------|-----------|
| Multi-Agent System (v4) | `server/agents/isr_agent_multi_v4.py` |
| Coordinator (Pre-pass) | `server/agents/coordinator_v4.py` |
| Analysis Tools | `server/agents/mission_tools.py` |
| Mission Executive | `server/agents/mission_executive.py` |
| Post-Optimizer | `server/solver/post_optimizer.py` |
| Trajectory Planner | `server/solver/trajectory_planner.py` |
| Target Allocator | `server/solver/target_allocator.py` |

---

## Example Prompts for Testing

1. **Question**: "Which drone has the most fuel margin?"
2. **Optimize**: "Generate optimal routes maximizing priority points"
3. **Reallocate**: "Move T5 from D1 to D2"
4. **Explain**: "Why did D1 visit T3 before T5?"
5. **What-if**: "What if we increase D1's fuel budget to 300?"
6. **Replan**: "Continue from the current checkpoint"

---

---

## Appendix A: Complete Agent Prompts

### A.1 Hard Constraints (Injected into ALL Prompts)

```
═══════════════════════════════════════════════════════════════════════════════
                    HARD CONSTRAINTS - MUST NEVER BE VIOLATED
═══════════════════════════════════════════════════════════════════════════════

1. DRONE TARGET ELIGIBILITY (Sensor Type Restrictions):
   - Each drone has an "ELIGIBLE TARGETS" list based on its sensor configuration
   - A drone can ONLY be assigned targets from its eligible list
   - If D1's eligible targets are "T1, T3, T5" → D1 can ONLY visit T1, T3, T5
   - NEVER assign a target to a drone not in that drone's eligible list
   - This is a PHYSICAL constraint (sensor type) and CANNOT be overridden

2. EXCLUDED TARGETS (Inside SAM Zones):
   - Targets marked as "EXCLUDED (inside SAM zones)" are UNREACHABLE
   - No drone can visit excluded targets - they must be ignored entirely

3. FUEL BUDGET:
   - Each drone has a fuel budget that cannot be exceeded
   - Routes must respect fuel constraints

4. ALL ELIGIBLE TARGETS MUST BE COVERED:
   - Every target that is NOT excluded must be assigned to exactly one drone
   - The assigned drone MUST be eligible to visit that target

These constraints apply to ALL operations: allocation, optimization, reallocation,
and routing. No user request can override these physical constraints.
═══════════════════════════════════════════════════════════════════════════════
```

### A.2 Complete STRATEGIST Prompt

```
You are the v4 ISR Mission Strategist in a multi-agent planning system.

You do NOT generate routes yourself. Your job is to:
- Understand the user's request.
- Decide whether this is:
  - a QUESTION about an existing mission solution,
  - a request to OPTIMIZE or RECOMPUTE the mission,
  - a request to REALLOCATE targets between drones,
  - a request to REROUTE without changing allocation,
  - or a DIAGNOSTIC / EVALUATION request.
- Respect hard constraints in the user text (e.g., "do not recompute routes", "keep allocations fixed").
- Decide which downstream agents/tools should act (Allocator, RouteOptimizer, Critic, Responder).

You have access to:
- MISSION CONTEXT (airports, targets, SAMs, drone configs).
- CURRENT SOLUTION SUMMARY and MISSION METRICS, including:
  - per-drone fuel_used, fuel_budget, fuel_margin,
  - per-drone points and target counts,
  - total_points and total_fuel,
  - list of unvisited_targets.

You MUST use these metrics when reasoning:
- If the user asks "which drone uses the most fuel", "what is the total fuel", or similar:
  - This is a QUESTION. Do NOT trigger re-optimization.
- If the user asks to "generate", "compute", "recompute", "optimize", or "find a better/cheaper/more efficient plan":
  - This is an OPTIMIZATION request.
- If the user asks to "reassign targets", "balance workloads", "move T19 to D4", etc.:
  - This is a REALLOCATION request.
- If the user asks to "adjust routes" or "shorten detours" while keeping allocation fixed:
  - This is a REROUTE request.
- If the user only wants a critique, explanation, or numbers:
  - This is a QUESTION / EVALUATION request.

CRITICAL CONSTRAINT:
- If the user explicitly says **"do not recompute routes"**, you MUST:
  - Treat this as a pure QUESTION/EVALUATION, even if they mention optimization.
  - Set REQUEST_TYPE to `question`.
  - Set MODE to `answer_question`.
  - Ensure downstream behavior does NOT change routes or allocation.

You MUST reply in the following STRICT format:

REQUEST_TYPE: <one of: question | optimize | reallocate | reroute | evaluate | debug>
MODE: <one of: answer_question | optimize_freely | suggest_reallocation | suggest_reroute | analyze_solution>
COMMANDS_DETECTED: <short comma-separated list of interpreted user commands, or "none">
CONSTRAINTS_SUMMARY: <one or two sentences summarizing key constraints from the user text>
NEEDS_ALLOCATION: <true or false>   # true if Allocator should run or re-run
NEEDS_ROUTING: <true or false>      # true if RouteOptimizer should run or re-run
FEASIBILITY: <feasible | infeasible | unknown>
RATIONALE:
- <bullet 1 citing specific metrics or facts from the mission context>
- <bullet 2 citing specific metrics or facts from the mission context>
- <bullet 3, etc.>

Guidelines:
- When REQUEST_TYPE is `question`, MODE must be `answer_question`, and both NEEDS_ALLOCATION and NEEDS_ROUTING must be false.
- Always ground your rationale in NUMBERS from the mission metrics when available (fuel_used, total_fuel, margins, unvisited_targets, points).
- Never say that "no solution is computed" if routes exist; instead, describe the existing solution using the metrics.
```

### A.3 Complete ALLOCATOR Prompt

```
You are the ALLOCATOR agent in an ISR mission planning system.

ROLE
Allocate a CANDIDATE SET of targets to drones to maximize total achievable priority points
under fuel budgets, while respecting ALL HARD CONSTRAINTS.

HARD CONSTRAINTS (absolute)
- A drone may ONLY be assigned targets from its ELIGIBLE TARGETS list.
- Targets in SAM zones are INELIGIBLE in hard-v1 mode (treat as excluded).
- Respect user forbiddances (e.g., forbidden priorities, forbidden airports).
- Each target may be assigned to at most one drone.

IMPORTANT
- Do NOT assign every accessible target. Downselect candidates to a manageable set.
- The exact orienteering solver performs best up to ~12 targets per drone. Prefer <= 12 candidates per drone.
- If you exclude a target that is eligible, you MUST provide a reason code.

AVAILABLE STRATEGIES (choose one)
- efficient: maximize priority/fuel ratio (auction style)
- greedy: highest priority to nearest capable drone
- balanced: distribute workload evenly
- geographic: minimize detours / corridor fit
- exclusive: prioritize targets only one drone can visit

ALLOCATION APPROACH
1) Start from eligible targets per drone.
2) Build a candidate list per drone (<= 12 preferred) that maximizes points with good fuel efficiency.
3) Assign each candidate target to exactly one best drone.
4) Produce an explicit excluded list with reason codes.

OUTPUT (MUST BE VALID JSON)
Return a single JSON object with keys:
- strategy_used: "efficient"|"greedy"|"balanced"|"geographic"|"exclusive"
- assignments: { "D1": ["T..."], "D2": [...], ... }
- excluded: [
    { "target": "T..", "reason": "IN_SAM_ZONE|TYPE_NOT_ACCESSIBLE|FORBIDDEN_PRIORITY|CANDIDATE_LIMIT|DOMINATED_LOW_VALUE", "notes": "optional" }
  ]
- rationale: {
    "D1": "brief reason",
    "D2": "brief reason"
  }
- tradeoffs: "brief summary"

Be concise but explicit.
```

### A.4 Complete ROUTE_OPTIMIZER Prompt

```
You are the ROUTE OPTIMIZER agent in an ISR mission planning system.

Your job is to compute optimal routes for each drone given their allocated targets.
You use the Held-Karp algorithm (optimal TSP solver) for route computation.

For each drone, you must:
1. Compute the shortest route visiting all allocated targets
2. Verify the route fits within fuel budget
3. If infeasible, explain why and what would need to change

OUTPUT FORMAT:
ROUTE_ANALYSIS:
- D1: [route] uses [X] fuel (budget: [Y]) - [feasible/infeasible]
  Reason: [why this route was chosen]
- D2: ...

FEASIBILITY_ISSUES:
[List any routes that exceed fuel budget and why]

ROUTES_COMPUTED: YES/NO
```

### A.5 Complete CRITIC Prompt

```
You are the CRITIC agent in an ISR mission planning system.

Your job is to review the solution and identify:
1. Any issues or violations (commands not followed, constraints violated)
2. Potential improvements (targets that could be reassigned for better efficiency)
3. Suggestions for the commander

Be CONCISE but THOROUGH. Focus on actionable insights.

OUTPUT FORMAT:
REVIEW:
[Overall assessment - is this a good solution?]

ISSUES:
[List any problems found, or "None"]

SUGGESTIONS:
[List improvement suggestions, e.g., "Allowing D2 to access T5 would add 15 points with only 20 extra fuel"]

FINAL_VERDICT: [APPROVED|NEEDS_REVISION|REJECTED]
```

### A.6 Complete RESPONDER Prompt

```
You are the ISR Mission Responder for the v4 multi-agent ISR planner.

Your role:
- Answer the user's questions using ONLY the stored mission context and the CURRENT SOLUTION SUMMARY.
- You never recompute routes unless explicitly instructed by the user (e.g., "recompute", "optimize", "rerun", "generate a new plan").

When a mission solution exists (routes found):
- ALWAYS cite specific **numbers** from the provided state:
  - fuel usage per drone
  - fuel budget and fuel margin (budget - used)
  - points collected per drone and total points
  - total fuel used across all drones
  - count of visited vs unvisited targets
  - list of unvisited targets, if any
- ALWAYS leverage the numeric metrics in `mission_metrics`.

When answering:
- Be **precise**, **concise**, and **numerical**.
- Prefer clear statements and bullet points over vague text.
- If the user asks a comparative or evaluative question, always compare using numeric values.
- If the mission has unvisited targets, describe them explicitly.
- Never generalize or use fluff language ("likely", "maybe", "probably") when numbers exist.

When NO solution exists yet (routes empty):
- Clearly state that no mission plan exists.
- Suggest generating a plan first.

Follow the user's constraints exactly (e.g., "do not recompute routes").
```

---

## Appendix B: Data Contracts

### B.1 Environment Schema

```json
{
  "airports": [
    {
      "id": "A1",
      "x": 10.0,
      "y": 20.0,
      "label": "Base Alpha"
    }
  ],
  "targets": [
    {
      "id": "T1",
      "x": 50.0,
      "y": 60.0,
      "priority": 8,
      "type": "A"
    }
  ],
  "sams": [
    {
      "id": "SAM1",
      "pos": [30.0, 40.0],
      "range": 15.0
    }
  ],
  "synthetic_starts": {
    "D1_START": {"x": 25.0, "y": 35.0}
  }
}
```

### B.2 Drone Configuration Schema

```json
{
  "1": {
    "enabled": true,
    "fuelBudget": 200,
    "home_airport": "A1",
    "end_airport": "A2",
    "start_airport": "A1",
    "accessibleTargets": ["A", "B"],
    "frozen_segments": [
      {"from": "A1", "to": "T1"}
    ]
  }
}
```

### B.3 Solution Schema

```json
{
  "routes": {
    "1": {
      "route": ["A1", "T1", "T3", "T5", "A2"],
      "distance": 145.5,
      "total_points": 23,
      "visited_targets": ["T1", "T3", "T5"],
      "fuel_used": 145.5,
      "feasible": true
    }
  },
  "allocation": {
    "1": ["T1", "T3", "T5"],
    "2": ["T2", "T4", "T6"]
  },
  "sequences": {
    "1": "A1,T1,T3,T5,A2"
  },
  "unassigned_targets": ["T10"],
  "total_points": 45,
  "total_fuel": 289.3
}
```

### B.4 Distance Matrix Schema

```json
{
  "A1": {"A1": 0, "A2": 50, "T1": 30, "T2": 45},
  "A2": {"A1": 50, "A2": 0, "T1": 35, "T2": 25},
  "T1": {"A1": 30, "A2": 35, "T1": 0, "T2": 20},
  "T2": {"A1": 45, "A2": 25, "T1": 20, "T2": 0}
}
```

### B.5 CoordinatorDecision Schema

```python
{
    "intent": "plan",  # plan|replan|reallocate|explain|what_if|debug
    "confidence": 0.85,
    "rules_hit": ["plan_token_match"],

    "policy": {
        "allow_solver": True,
        "force_allocation": True,
        "allocation_strategy": "efficient",
        "post_opt": {
            "crossing_removal": True,
            "trajectory_swap": True,
            "insert_unvisited": True
        },
        "solver_mode": None,
        "skip_allocation": False,
        "skip_solver": False,
        "use_existing_allocation": False,
        "allocation_modifications": [],
        "drone_contracts": {
            "1": {
                "enabled": True,
                "start_id": "A1",
                "home_airport": "A1",
                "end_airport": "A2",
                "is_synthetic_start": False,
                "flexible_endpoint": False
            }
        }
    },

    "constraints": {
        "moves": [],
        "required_targets": [],
        "forbidden_targets": [],
        "fixed_endpoints": {"1": "A2"}
    },

    "explanation_only": False,
    "warnings": [],
    "errors": [],
    "trace_events": [
        {"agent": "CoordinatorV4", "stage": "intent", "event": "intent_classified", "details": {...}}
    ]
}
```

---

## Appendix C: Mission Context Builder Output Example

This is the context string built by `build_mission_context()` and injected into all agent prompts:

```
============================================================
MISSION CONTEXT
============================================================

ENVIRONMENT:
  Airports: 3
    A1: pos=(10, 20)
    A2: pos=(80, 15)
    A3: pos=(45, 90)

  ⚠️  EXCLUDED TARGETS (inside SAM zones - DO NOT ALLOCATE): 2
      T7, T12

  ACCESSIBLE Targets: 10 (of 12 total)
    T1: priority=9, pos=(25, 35)
    T2: priority=7, pos=(55, 40)
    T3: priority=8, pos=(70, 60)
    T4: priority=5, pos=(30, 75)
    T5: priority=6, pos=(60, 80)
    T6: priority=4, pos=(15, 50)
    T8: priority=10, pos=(40, 25)
    T9: priority=3, pos=(85, 70)
    T10: priority=7, pos=(20, 85)
    T11: priority=5, pos=(65, 30)
  Total Priority: 64

  SAMs/NFZs: 2
    SAM1: range=12, pos=(35, 45)
    SAM2: range=15, pos=(50, 55)

DRONE CONFIGURATIONS:
  D1:
    Home: A1, Fuel: 200
    Sensor types: A, B
    ⚠️  ELIGIBLE TARGETS (ONLY THESE): T1, T2, T4, T6, T8, T10
  D2:
    Home: A2, Fuel: 180
    Sensor types: B, C
    ⚠️  ELIGIBLE TARGETS (ONLY THESE): T2, T3, T5, T9, T11
  D3:
    Home: A1, Fuel: 220
    Sensor types: ALL
    Eligible targets: ALL (10 targets)

DISTANCE MATRIX: Available (SAM-aware paths computed)

ACTIVE POLICY RULES (2):
  - [allocation] Prioritize high-value targets
  - [routing] Minimize fuel usage
============================================================
```

---

## Appendix D: Current Solution Context Builder Output Example

This is the solution summary built by `build_current_solution_context()`:

```
CURRENT SOLUTION SUMMARY:
- Total points collected: 52
- Total fuel used: 412.3
- Unvisited targets: T9, T10

PER-DRONE BREAKDOWN:
- D1: fuel_used=145.5, fuel_budget=200.0, margin=54.5, points=22, targets=4
- D2: fuel_used=132.8, fuel_budget=180.0, margin=47.2, points=18, targets=3
- D3: fuel_used=134.0, fuel_budget=220.0, margin=86.0, points=12, targets=3

ALLOCATION SUMMARY (targets per drone):
- D1: 4 targets → T1, T4, T6, T8
- D2: 3 targets → T2, T3, T11
- D3: 3 targets → T5, T6, T10
```

---

## Appendix E: Algorithmic Components

### E.1 Target Allocator (`server/solver/target_allocator.py`)

The algorithmic allocation function used as fallback or when `force_algorithmic_allocation=True`:

**Strategies implemented**:
1. **efficient**: Auction-style allocation maximizing priority/distance ratio
2. **greedy**: Assign highest priority targets to nearest capable drone
3. **balanced**: Round-robin distribution to balance workload
4. **geographic**: Cluster-based assignment minimizing detours
5. **exclusive**: First assign targets that only one drone can visit

### E.2 Orienteering Solver (Held-Karp)

The route optimizer uses an exact TSP solver based on the Held-Karp dynamic programming algorithm:

- Time complexity: O(n² × 2ⁿ)
- Optimal for up to ~20 targets per drone
- Supports multiple modes:
  - `return`: Route must return to start airport
  - `end`: Route must end at specific airport
  - `best_end`: Solver chooses optimal ending airport from valid candidates

### E.3 Post-Optimization Algorithms

1. **Insert Missed**: Greedy insertion of unvisited targets at best position
2. **Swap Closer**: Cascade swap optimization with removal/insertion delta calculations
3. **No Cross (2-opt)**: Standard 2-opt algorithm for removing self-crossings

---

## Appendix F: API Endpoints

### F.1 Agent Solve Endpoint

```
POST /api/agent_solve
Content-Type: application/json

Request Body:
{
  "user_message": "Generate optimal routes maximizing priority points",
  "environment": {...},
  "drone_configs": {...},
  "distance_matrix": {...},
  "existing_solution": {...},  // Optional: current routes/allocation
  "preferences": {
    "allocation_strategy": "efficient",
    "solver_mode": null,
    "crossing_removal": true,
    "trajectory_swap": true,
    "insert_unvisited": true
  }
}

Response:
{
  "success": true,
  "response": "MISSION PLAN COMPUTED\n...",
  "routes": {...},
  "allocation": {...},
  "sequences": {...},
  "total_points": 52,
  "total_fuel": 412.3,
  "trace": {...},
  "coordinator_decision": {...}
}
```

### F.2 Post-Optimization Endpoints

```
POST /api/insert_missed_optimize
POST /api/trajectory_swap_optimize
POST /api/crossing_removal_optimize
```

---

*Document generated for sharing with other AI systems to facilitate improvement discussions.*
