"""
Pydantic schemas for the ISR Planner API.

Extracted from server/main.py during Phase 3 refactoring.
"""

from typing import Any, Dict, List, Optional
from pydantic import BaseModel


# =============================================================================
# Solve Endpoint Schemas
# =============================================================================

class SolveRequest(BaseModel):
    """Request schema for /api/solve endpoint."""
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]
    allocation_strategy: str = "efficient"
    mission_id: Optional[str] = None  # optional continuity handle


class SolveResponse(BaseModel):
    """Response schema for solve endpoints."""
    success: bool
    routes: Dict[str, Any]
    sequences: Dict[str, str]
    wrapped_polygons: Optional[List[List[List[float]]]] = None
    allocations: Optional[Dict[str, List[str]]] = None
    distance_matrix: Optional[Dict[str, Any]] = None
    mission_id: Optional[str] = None
    env_hash: Optional[str] = None
    continuity: Optional[bool] = None
    errors: Optional[List[str]] = None


class SolveWithAllocationRequest(BaseModel):
    """Request schema for /api/solve_with_allocation endpoint."""
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]
    allocation_strategy: str = "efficient"
    use_sam_aware_distances: bool = False  # Disabled for speed - SAM avoidance happens at trajectory time
    post_optimize: bool = True
    visited_targets: List[str] = []  # Targets already visited (for checkpoint replanning)
    is_checkpoint_replan: bool = False  # Whether this is a checkpoint replan
    mission_id: Optional[str] = None  # optional continuity handle


class ApplySequenceRequest(BaseModel):
    """Request schema for /api/apply_sequence endpoint."""
    drone_id: str
    sequence: str  # comma-separated waypoint IDs, e.g., "A1,T1,T2,A1"
    env: Dict[str, Any]
    fuel_budget: float = 300.0


class ApplySequenceResponse(BaseModel):
    """Response schema for /api/apply_sequence endpoint."""
    success: bool
    route: List[str]
    distance: float
    points: int
    trajectory: List[List[float]]
    error: Optional[str] = None


# =============================================================================
# Distance Matrix Schemas
# =============================================================================

class PrepareMatrixRequest(BaseModel):
    """Request schema for /api/prepare_matrix endpoint."""
    env: Dict[str, Any]
    buffer: float = 0.0


class PrepareMatrixResponse(BaseModel):
    """Response schema for /api/prepare_matrix endpoint."""
    success: bool
    num_waypoints: int
    num_paths_with_sam_avoidance: int
    excluded_targets: List[str]


# =============================================================================
# Coverage & Optimization Schemas
# =============================================================================

class CoverageStatsRequest(BaseModel):
    """Request schema for /api/coverage_stats endpoint."""
    solution: Dict[str, Any]
    env: Dict[str, Any]


class CoverageStatsResponse(BaseModel):
    """Response schema for /api/coverage_stats endpoint."""
    targets_visited: int
    targets_total: int
    coverage_percent: float
    points_collected: int
    points_possible: int
    points_percent: float
    total_distance: float
    unvisited_targets: List[str]


class TrajectorySwapRequest(BaseModel):
    """Request schema for /api/trajectory_swap_optimize endpoint."""
    solution: Dict[str, Any]  # routes from solve_with_allocation
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]
    visited_targets: List[str] = []  # Frozen targets that should NOT be swapped
    # If True, the optimizer will auto-iterate until convergence (server-side)
    auto_iterate: bool = False
    # If True, the optimizer will regenerate SAM-aware trajectories between passes
    auto_regen: bool = False
    # If True, return per-target diagnostics to help debug swap decisions
    debug: bool = False


class InsertMissedRequest(BaseModel):
    """Request schema for /api/insert_missed_optimize endpoint."""
    solution: Dict[str, Any]  # routes from solve_with_allocation
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]


# =============================================================================
# Agent Schemas
# =============================================================================

class AgentChatRequest(BaseModel):
    """Request schema for /api/agents/chat endpoint."""
    message: str
    env: Dict[str, Any]
    sequences: Optional[Dict[str, str]] = None
    drone_configs: Optional[Dict[str, Any]] = None
    mission_id: Optional[str] = None  # Existing mission to continue chatting about
    existing_solution: Optional[Dict[str, Any]] = None  # Previous routes/allocation from frontend


class AgentTraceEvent(BaseModel):
    """Trace event for agent debugging."""
    t: str                                    # event type, e.g. "coordinator.intent", "allocator.result"
    msg: str                                  # short human readable
    data: Optional[Dict[str, Any]] = None     # structured payload
    ts_ms: Optional[int] = None               # optional timestamp


class AgentChatResponse(BaseModel):
    """Response schema for agent chat endpoints."""
    reply: str
    # Coordinator contract (typed decision envelope)
    intent: Optional[str] = None
    actions: Optional[List[Dict[str, Any]]] = None
    policy: Optional[Dict[str, Any]] = None
    constraints: Optional[Dict[str, Any]] = None
    valid: Optional[bool] = None
    warnings: Optional[List[str]] = None
    errors: Optional[List[str]] = None

    route: Optional[List[str]] = None
    routes: Optional[Dict[str, List[str]]] = None
    trajectories: Optional[Dict[str, List[List[float]]]] = None
    points: Optional[int] = None
    fuel: Optional[float] = None
    allocations: Optional[Dict[str, List[str]]] = None
    allocation_strategy: Optional[str] = None
    mission_id: Optional[str] = None

    # Trace fields for Agents Monitor tab
    trace: Optional[Dict[str, Any]] = None
    trace_events: Optional[List[AgentTraceEvent]] = None


class AddMemoryRequest(BaseModel):
    """Request schema for /api/agent/memory POST endpoint."""
    content: str
    category: str = "correction"  # correction, instruction, preference, fact


# =============================================================================
# Executive Schemas
# =============================================================================

class ExecutiveTickRequest(BaseModel):
    """Request schema for /api/executive/tick endpoint."""
    mission_id: Optional[str] = None
    ui_state: Dict[str, Any] = {}
    events: List[Dict[str, Any]] = []
    env: Optional[Dict[str, Any]] = None
    drone_configs: Optional[Dict[str, Any]] = None


class ExecutiveTickResponse(BaseModel):
    """Response schema for /api/executive/tick endpoint."""
    action: str
    mission_id: str
    draft_plan: Optional[Dict[str, Any]] = None
    joined_plan: Optional[Dict[str, Any]] = None
    visited_targets: List[str] = []
    markers: Dict[str, List[str]] = {}
    message: str = ""
