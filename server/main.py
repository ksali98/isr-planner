import os
import json
import hashlib
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Set
from uuid import uuid4

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from langchain_core.messages import HumanMessage
from pydantic import BaseModel

from .agents.graph import workflow  # <-- our LangGraph workflow (GPT-based)
# Use v4 multi-agent system with Mission Planner (reasoning-based orchestration)
from .agents.isr_agent_multi_v4 import run_multi_agent_v4
# Legacy v3 (task-based, kept for reference)
# from .agents.isr_agent_multi_v3 import run_isr_agent as run_multi_agent_v3
# Memory functions (shared across all agent versions)
from .agents.agent_memory import (
    load_memory,
    add_memory,
    clear_memory,
    delete_memory,
)

from .agents.mission_executive import get_executive, TickRequest, TickResponse
from .solver import sam_distance_matrix
from .solver.post_optimizer import get_coverage_stats, trajectory_swap_optimize, crossing_removal_optimize, post_optimize_solution
from .solver.solver_bridge import (
    clear_cached_matrix,
    get_current_matrix,
    prepare_distance_matrix,
    solve_mission,
    solve_mission_with_allocation,
)
from .solver.trajectory_planner import ISRTrajectoryPlanner

from server.database.mission_ledger import (
    # New Decision Trace v1 functions
    create_agent_run,
    update_agent_run,
    create_agent_trace,
    log_optimizer_step,
    get_active_policy_rules,
    # Legacy stubs for backwards compatibility
    create_mission_run,
    log_event,
    create_env_version,
    create_plan,
    # Distance matrix cache functions
    get_default_routing_model,
)

# Import SAM distance matrix calculator for v4 wiring
from .solver.sam_distance_matrix import calculate_sam_aware_matrix

import time

_current_env = None
_current_run_id = None
_current_env_version_id = None
_current_plan_id = None

# Import polygon wrapping for visualization (same as delivery planner)
# Add paths for both local dev and Docker deployment
project_root = Path(__file__).parent.parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))
if "/app" not in sys.path:
    sys.path.insert(0, "/app")
from path_planning_core.sam_wrapping import wrap_sams

class SolveRequest(BaseModel):
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]
    allocation_strategy: str = "geographic"
    mission_id: Optional[str] = None  # optional continuity handle


class SolveResponse(BaseModel):
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


class AgentChatRequest(BaseModel):
    message: str
    env: Dict[str, Any]
    sequences: Optional[Dict[str, str]] = None
    drone_configs: Optional[Dict[str, Any]] = None
    mission_id: Optional[str] = None  # Existing mission to continue chatting about
    existing_solution: Optional[Dict[str, Any]] = None  # Previous routes/allocation from frontend
    llm_config: Optional[Dict[str, Any]] = None  # LLM provider configuration (provider, model, api_key)


from typing import Any, Dict, List, Optional
from pydantic import BaseModel

class AgentTraceEvent(BaseModel):
    t: str                                    # event type, e.g. "coordinator.intent", "allocator.result"
    msg: str                                  # short human readable
    data: Optional[Dict[str, Any]] = None     # structured payload
    ts_ms: Optional[int] = None               # optional timestamp

class AgentChatResponse(BaseModel):
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


class ApplySequenceRequest(BaseModel):
    drone_id: str
    sequence: str  # comma-separated waypoint IDs, e.g., "A1,T1,T2,A1"
    env: Dict[str, Any]
    fuel_budget: float = 300.0


class ApplySequenceResponse(BaseModel):
    success: bool
    route: List[str]
    distance: float
    points: int
    trajectory: List[List[float]]
    error: Optional[str] = None


class ExecutiveTickRequest(BaseModel):
    mission_id: Optional[str] = None
    ui_state: Dict[str, Any] = {}
    events: List[Dict[str, Any]] = []
    env: Optional[Dict[str, Any]] = None
    drone_configs: Optional[Dict[str, Any]] = None


class ExecutiveTickResponse(BaseModel):
    action: str
    mission_id: str
    draft_plan: Optional[Dict[str, Any]] = None
    joined_plan: Optional[Dict[str, Any]] = None
    visited_targets: List[str] = []
    markers: Dict[str, List[str]] = {}
    message: str = ""


# Support both local development and Docker deployment paths
if Path("/app/webapp").exists():
    # Docker deployment
    ROOT = Path("/app")
    WEBAPP = ROOT / "webapp"
else:
    # Local development
    ROOT = Path(__file__).resolve().parents[1]        # isr_web/
    WEBAPP = ROOT / "webapp"

ENV_PATH = ROOT / "environment.json"
AI_SOLUTION_PATH = ROOT / "ai_solution.json"
MISSION_PATH = ROOT / "mission_solution.json"

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Middleware to add no-cache headers for JavaScript files
@app.middleware("http")
async def add_no_cache_for_js(request: Request, call_next):
    response = await call_next(request)
    if request.url.path.endswith(".js"):
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response

@app.post("/api/debug/supabase-test")
def supabase_test():
    run_id = create_mission_run(
        system_version="debug",
        mission_name="supabase-test"
    )

    if not run_id:
        return {
            "ok": False,
            "error": "Failed to create mission run (check Railway logs)"
        }

    ok = log_event(
        run_id,
        "SUPABASE_TEST_EVENT",
        {"ok": True}
    )

    return {
        "ok": ok,
        "run_id": run_id
    }


@app.post("/api/executive/tick", response_model=ExecutiveTickResponse)
def executive_tick(req: ExecutiveTickRequest):
    """
    Mission Executive tick endpoint.

    Called by UI to:
    - Report current state (positions, progress)
    - Send events (commands, edits)
    - Receive next action to take
    """
    executive = get_executive()

    tick_request = TickRequest(
        mission_id=req.mission_id,
        ui_state=req.ui_state,
        events=req.events,
        env=req.env,
        drone_configs=req.drone_configs
    )

    response = executive.tick(tick_request)

    return ExecutiveTickResponse(
        action=response.action.value,
        mission_id=response.mission_id,
        draft_plan=response.draft_plan,
        joined_plan=response.joined_plan,
        visited_targets=response.visited_targets,
        markers=response.markers,
        message=response.message
    )


# serve static files (index.html, isr.js, etc.)
# NOTE: Middleware above adds no-cache headers for all .js files
app.mount("/static", StaticFiles(directory=str(WEBAPP)), name="static")

_current_env: Optional[Dict[str, Any]] = None
_export_counter: int = 1  # Running number for export filenames

# ----------------------------------------------------------------------------
# Mission state store (in-memory; per-process)
# ----------------------------------------------------------------------------

MissionRecord = Dict[str, Any]
MISSION_STORE: Dict[str, MissionRecord] = {}


# ----------------------------------------------------------------------------
# Env / Result Guards (Step 3): stable hashing + continuity checks
# ----------------------------------------------------------------------------

def _stable_hash(obj: Any) -> str:
    """Deterministic short hash for JSON-serializable objects."""
    s = json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(s.encode("utf-8")).hexdigest()[:16]


def compute_env_hash(env: Dict[str, Any]) -> str:
    """Hash only the environment components that affect solving/feasibility."""
    # Keep this conservative: include geometry + identifiers.
    payload = {
        "airports": env.get("airports", []),
        "targets": env.get("targets", []),
        "sams": env.get("sams", []),
        # Optional fields seen in your UI/editor. Include if present.
        "polygons": env.get("polygons", env.get("obstacles", [])),
        "boundary": env.get("boundary", None),
        "synthetic_starts": env.get("synthetic_starts", {}),
    }
    return _stable_hash(payload)


def compute_cfg_hash(drone_configs: Optional[Dict[str, Any]]) -> str:
    return _stable_hash(drone_configs or {})


def _known_waypoint_ids(env: Dict[str, Any]) -> set:
    ids = set()
    for a in env.get("airports", []):
        aid = a.get("id") or a.get("label")
        if aid:
            ids.add(str(aid))
    for t in env.get("targets", []):
        tid = t.get("id") or t.get("label")
        if tid:
            ids.add(str(tid))
    # Synthetic starts can behave like airports/waypoints
    for sid in (env.get("synthetic_starts") or {}).keys():
        ids.add(str(sid))
# Checkpoints are valid waypoints in segmented missions
    for c in env.get("checkpoints", []):
        cid = c.get("id") or c.get("label")
        if cid:
            ids.add(str(cid))
    return ids


def verify_solution_basic(env: Dict[str, Any], routes: Dict[str, Any], drone_configs: Optional[Dict[str, Any]] = None) -> List[str]:
    """Lightweight, fast validation to prevent returning self-inconsistent results."""
    errors: List[str] = []
    known_ids = _known_waypoint_ids(env)

    if not env.get("airports"):
        errors.append("env.airports missing/empty")
    # if not env.get("targets"):
        # Targets can be empty in some edge cases, but treat as warning-level error here.
        # errors.append("env.targets missing/empty")

    if not isinstance(routes, dict):
        errors.append("routes is not a dict")
        return errors

    for drone_id, r in routes.items():
        if not isinstance(r, dict):
            errors.append(f"route[{drone_id}] is not an object")
            continue

        # Skip validation for disabled drones - empty routes are expected
        if drone_configs:
            cfg = drone_configs.get(drone_id) or drone_configs.get(str(drone_id))
            if cfg and not cfg.get("enabled", True):
                continue

        seq = r.get("route") or r.get("waypoints") or r.get("sequence")
        if isinstance(seq, str):
            seq = [s.strip() for s in seq.split(",") if s.strip()]
        if not isinstance(seq, list) or not seq:
            errors.append(f"route[{drone_id}].route missing/empty")
            continue

        # Waypoint ID validity
        for wid in seq:
            if str(wid) not in known_ids:
                errors.append(f"route[{drone_id}] contains unknown waypoint id: {wid}")
                break

        # Fuel check if distance and budget are provided
        dist = r.get("distance")
        budget = r.get("fuel_budget")
        if isinstance(dist, (int, float)) and isinstance(budget, (int, float)):
            # Check for blocked path (99999 sentinel)
            if dist >= 99999:
                errors.append(f"route[{drone_id}] has blocked path (SAM obstruction between waypoints)")
            elif dist > budget + 1e-6:
                errors.append(f"route[{drone_id}] exceeds fuel budget: {dist:.2f} > {budget:.2f}")

    return errors


def enforce_mission_continuity(
    mission_id: Optional[str],
    env: Dict[str, Any],
    drone_configs: Optional[Dict[str, Any]],
) -> Tuple[Optional[str], bool, str, str, Optional[str]]:
    """Return (mission_id, continuity, env_hash, cfg_hash, mismatch_reason)."""
    env_hash = compute_env_hash(env)
    cfg_hash = compute_cfg_hash(drone_configs)

    if not mission_id:
        return None, False, env_hash, cfg_hash, None

    rec = MISSION_STORE.get(mission_id)
    if not rec:
        return None, False, env_hash, cfg_hash, "mission_id not found"

    stored_env_hash = rec.get("env_hash")
    stored_cfg_hash = rec.get("cfg_hash")
    if stored_env_hash and stored_env_hash != env_hash:
        return None, False, env_hash, cfg_hash, "env_hash mismatch"
    if stored_cfg_hash and stored_cfg_hash != cfg_hash:
        # Config mismatch is less severe than env mismatch; treat as new run.
        return None, False, env_hash, cfg_hash, "cfg_hash mismatch"

    return mission_id, True, env_hash, cfg_hash, None




@app.get("/", response_class=HTMLResponse)
async def index():
    """Serve the main ISR Web UI page."""
    index_path = WEBAPP / "index.html"
    if not index_path.exists():
        # Helpful message if the file isn't there yet
        return HTMLResponse(
            "<h1>ISR Web UI</h1><p>index.html not found in /webapp</p>",
            status_code=200,
        )

    return HTMLResponse(index_path.read_text(encoding="utf-8"))

# ------------------------- helpers ---------------------------------

def _coord(env: Dict[str, Any], label: str) -> Optional[Dict[str, float]]:
    """Return {'x','y'} for airport or target label."""
    if label.startswith("A"):
        for a in env.get("airports", []):
            if a.get("id") == label:
                return {"x": float(a["x"]), "y": float(a["y"])}
    if label.startswith("T"):
        for t in env.get("targets", []):
            if t.get("id") == label:
                return {"x": float(t["x"]), "y": float(t["y"])}
    return None


def _dist(p1: Dict[str, float], p2: Dict[str, float]) -> float:
    return math.hypot(p1["x"] - p2["x"], p1["y"] - p2["y"])


def _nearest_neighbor_route(
    env: Dict[str, Any],
    start_id: str,
    end_id: str,
    candidate_ids: List[str],
    fuel_budget: float,
) -> List[str]:
    """Very simple TSP-like: start -> nearest -> ... -> end, obey fuel."""
    if not candidate_ids:
        if start_id == end_id:
            return [start_id, start_id]
        return [start_id, end_id]

    route = [start_id]
    cur = start_id
    total = 0.0

    remaining = candidate_ids[:]
    while remaining:
        p_cur = _coord(env, cur)
        if not p_cur:
            break

        best = None
        best_extra = None

        for tid in remaining:
            p_t = _coord(env, tid)
            if not p_t:
                continue
            to_t = _dist(p_cur, p_t)
            # distance from this target to end
            p_end = _coord(env, end_id) or p_cur
            t_to_end = _dist(p_t, p_end)
            projected = total + to_t + t_to_end
            if projected <= fuel_budget + 1e-6:
                if best is None or to_t < best_extra:
                    best = tid
                    best_extra = to_t

        if best is None:
            # can't add more
            break

        total += best_extra
        cur = best
        route.append(cur)
        remaining.remove(best)

    # go to end
    if cur != end_id:
        p_cur = _coord(env, cur)
        p_end = _coord(env, end_id)
        if p_cur and p_end:
            total += _dist(p_cur, p_end)
        route.append(end_id)

    return route

def distance(a: Dict[str, float], b: Dict[str, float]) -> float:
    dx = a["x"] - b["x"]
    dy = a["y"] - b["y"]
    return math.sqrt(dx * dx + dy * dy)


def build_id_map(env: Dict[str, Any]):
    """Return dict: id_str -> {'x':..., 'y':..., 'kind': 'airport'|'target'|'synthetic_start'}."""
    id_map: Dict[str, Dict[str, Any]] = {}

    for a in env.get("airports", []):
        id_map[str(a["id"])] = {
            "x": float(a["x"]),
            "y": float(a["y"]),
            "kind": "airport",
            "priority": 0,
            "type": None,
        }

    for t in env.get("targets", []):
        id_map[str(t["id"])] = {
            "x": float(t["x"]),
            "y": float(t["y"]),
            "kind": "target",
            "priority": int(t.get("priority", 0)),
            "type": str(t.get("type", "a")).lower(),
        }

    # Add synthetic start nodes (for checkpoint replanning)
    for node_id, node_data in env.get("synthetic_starts", {}).items():
        id_map[str(node_id)] = {
            "x": float(node_data["x"]),
            "y": float(node_data["y"]),
            "kind": "synthetic_start",
            "priority": 0,
            "type": None,
        }

    return id_map
def is_target_safe_from_sams(
    target: Dict[str, Any],
    env: Dict[str, Any],
    id_map: Dict[str, Dict[str, Any]],
) -> bool:
    """
    Return False if the target is inside any SAM range.
    env.sams is expected as a list of {'pos': [x, y], 'range': R}.
    """
    sams = env.get("sams", [])
    if not sams:
        return True  # no SAMs => everything is safe

    tx = float(target["x"])
    ty = float(target["y"])

    for sam in sams:
        pos = sam.get("pos") or sam.get("position")
        if not pos or len(pos) != 2:
            continue
        sx, sy = float(pos[0]), float(pos[1])
        r = float(sam.get("range", 0.0))

        dx = tx - sx
        dy = ty - sy
        d2 = dx * dx + dy * dy
        if d2 <= r * r:
            # Inside SAM range → not safe
            return False

    return True

def plan_for_drone(
    drone_id: str,
    env: Dict[str, Any],
    drone_cfg: Dict[str, Any],
    id_map: Dict[str, Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    """
    Priority-aware, SAM-aware greedy planner:

    - Start at cfg.start_airport
    - Build candidate targets:
        * type allowed for this drone
        * outside all SAM ranges
    - At each step:
        * pick target with best (priority / distance_to_target)
        * but only if we still have fuel to reach it AND then reach end_airport
    - End at cfg.end_airport
    """
    if not drone_cfg.get("enabled", False):
        return None

    env_targets = env.get("targets", [])
    env_airports = env.get("airports", [])

    # Resolve start/end airports
    start_id = str(drone_cfg.get("start_airport") or "")
    end_id = str(drone_cfg.get("end_airport") or "")

    if not start_id and env_airports:
        start_id = str(env_airports[0]["id"])
    if not end_id and env_airports:
        end_id = str(env_airports[0]["id"])

    if start_id not in id_map or end_id not in id_map:
        return None

    start_pos = id_map[start_id]
    end_pos = id_map[end_id]

    # Type access
    access = drone_cfg.get("target_access", {})
    allowed_types = {
        t for t, enabled in access.items()
        if enabled and isinstance(t, str)
    }
    if not allowed_types:
        return None

    fuel_budget = float(drone_cfg.get("fuel_budget") or 1e9)

    # Build list of candidate targets: allowed type AND outside all SAMs
    candidates: List[Dict[str, Any]] = []
    for t in env_targets:
        tid = str(t["id"])
        info = id_map.get(tid)
        if not info:
            continue

        t_type = info["type"]
        if t_type not in allowed_types:
            continue

        candidate = {
            "id": tid,
            "x": info["x"],
            "y": info["y"],
            "priority": info["priority"],
            "type": t_type,
        }

        if not is_target_safe_from_sams(candidate, env, id_map):
            # Skip targets inside any SAM range
            continue

        candidates.append(candidate)

    # Greedy loop: choose best priority / distance each step
    route_ids: List[str] = []
    points = 0.0
    dist_used = 0.0

    current = {
        "id": start_id,
        "x": start_pos["x"],
        "y": start_pos["y"],
    }
    route_ids.append(start_id)

    remaining = candidates.copy()

    while remaining:
        best_idx = None
        best_score = None   # score = priority / distance

        for i, tgt in enumerate(remaining):
            d = distance(current, tgt)
            if d == 0:
                d = 1e-6  # avoid division by zero

            # Priority-aware: more points per unit distance is better
            score = tgt["priority"] / d

            # We still must respect fuel, so estimate projected distance:
            d_to_target = d
            d_target_to_end = distance(
                {"x": tgt["x"], "y": tgt["y"]},
                {"x": end_pos["x"], "y": end_pos["y"]},
            )
            projected = dist_used + d_to_target + d_target_to_end

            # If visiting this target would exceed budget, we treat it as invalid
            if projected > fuel_budget:
                continue

            if best_score is None or score > best_score:
                best_score = score
                best_idx = i

        if best_idx is None:
            # No more feasible target (fuel/budget constraint or no candidates)
            break

        target = remaining[best_idx]

        # We already checked projected distance, so just update usage and points
        d_to_target = distance(current, target)
        dist_used += d_to_target
        points += target["priority"]
        route_ids.append(target["id"])
        current = {
            "id": target["id"],
            "x": target["x"],
            "y": target["y"],
        }
        remaining.pop(best_idx)

    # Finally go to end airport if not already there
    if current["id"] != end_id:
        d_back = distance(
            {"x": current["x"], "y": current["y"]},
            {"x": end_pos["x"], "y": end_pos["y"]},
        )
        dist_used += d_back
        route_ids.append(end_id)

    route_str = ",".join(route_ids)

    return {
        "route": route_ids,
        "points": points,
        "distance": dist_used,
        "fuel_budget": fuel_budget,
        "sequence": route_str,
    }


# ------------------------- routes ----------------------------------


@app.get("/health")
def health():
    return {"status": "ok"}


# =============================================================================
# LLM Provider Management
# =============================================================================

from .agents.llm_factory import (
    get_available_providers,
    test_connection as test_llm_connection,
    validate_provider_model,
    get_configured_providers,
)


class LLMTestRequest(BaseModel):
    provider: str
    model: str
    api_key: str


class LLMTestResponse(BaseModel):
    success: bool
    message: str


@app.get("/api/llm/providers")
def get_llm_providers():
    """
    Get list of available LLM providers and their models.
    Also indicates which providers have API keys configured in environment.
    """
    providers = get_available_providers()
    configured = get_configured_providers()

    # Add 'configured' flag to each provider
    for p in providers:
        p["configured"] = p["id"] in configured

    return {
        "providers": providers,
        "configured_providers": configured
    }


@app.post("/api/llm/test", response_model=LLMTestResponse)
def test_llm_provider(request: LLMTestRequest):
    """
    Test connection to an LLM provider with given credentials.
    """
    # Validate inputs
    is_valid, error = validate_provider_model(request.provider, request.model)
    if not is_valid:
        return LLMTestResponse(success=False, message=error)

    # Test the connection
    success, message = test_llm_connection(
        provider=request.provider,
        model=request.model,
        api_key=request.api_key
    )

    return LLMTestResponse(success=success, message=message)


@app.post("/api/environment")
def receive_environment(payload: Dict[str, Any]):
    """
    Payload: { "environment": { airports: [...], targets: [...], sams: [...] } }

    Automatically recalculates SAM-aware distance matrix after each edit.
    Also snapshots the env into Supabase (env_versions) and logs events.
    """
    global _current_env, _current_run_id, _current_env_version_id


    print("✅ HIT POST /api/environment", flush=True)
    print("✅ SUPABASE_URL set:", bool(os.environ.get("SUPABASE_URL")), flush=True)
    print("✅ SUPABASE_KEY set:", bool(os.environ.get("SUPABASE_KEY")), flush=True)
    print("✅ payload keys:", list(payload.keys()), flush=True)


    env = payload.get("environment")
    if not isinstance(env, dict):
        return {"success": False, "error": "Invalid payload"}

    # ---- Update in-memory + on-disk env (existing behavior) ----
    _current_env = env
    ENV_PATH.write_text(json.dumps(env, indent=2))
    print("📥 [ISR_WEB] Environment updated", flush=True)

    # ---- Supabase: ensure a run exists ----
    # We create ONE run per session (until you explicitly reset/close it).
    if _current_run_id is None:
        _current_run_id = create_mission_run(system_version="v4", mission_name="isr-run")
        if _current_run_id:
            log_event(
                _current_run_id,
                "RUN_CREATED",
                {"source": "api/environment"}
            )

    # ---- Supabase: snapshot env version on every environment update ----
    # Decide event type: first import vs subsequent edits
    event_type = "ENV_IMPORTED" if _current_env_version_id is None else "ENV_EDITED"

    if _current_run_id is not None:
        _current_env_version_id = create_env_version(
            run_id=_current_run_id,
            env_snapshot=env,
            source="human",
            reason=event_type.lower(),   # "env_imported" or "env_edited"
        )

        print(f"✅ create_env_version returned: {_current_env_version_id}", flush=True)

        # Log env event with lightweight summary (avoid storing huge payload twice)
        try:
            airports_n = len(env.get("airports", []))
            targets_n = len(env.get("targets", []))
            sams_n = len(env.get("sams", []))
        except Exception:
            airports_n = targets_n = sams_n = None

        log_event(
            _current_run_id,
            event_type,
            {
                "env_version_id": _current_env_version_id,
                "counts": {"airports": airports_n, "targets": targets_n, "sams": sams_n},
                "has_sams": bool(env.get("sams", [])),
            },
        )

    # ---- Existing behavior: matrix recompute ----
    sams = env.get("sams", [])
    if sams:
        print("⏳ Recalculating SAM-aware distance matrix...", flush=True)
        try:
            matrix_data = prepare_distance_matrix(env, buffer=0.0)
            num_waypoints = len(matrix_data.get("labels", []))
            num_paths = len(matrix_data.get("paths", {}))
            print(f"✅ Matrix ready: {num_waypoints} waypoints, {num_paths} SAM-avoiding paths", flush=True)
        except Exception as e:
            print(f"⚠️ Matrix calculation failed: {e}", flush=True)
    else:
        clear_cached_matrix()
        print("✅ No SAMs - using direct distances", flush=True)

    return {
        "success": True,
        "run_id": _current_run_id,
        "env_version_id": _current_env_version_id,
        "event_type": event_type,
    }



@app.get("/api/environment")
def get_environment():
    global _current_env
    if _current_env is None and ENV_PATH.exists():
        _current_env = json.loads(ENV_PATH.read_text())

    # Include cached matrix data in the response
    cached_matrix = get_current_matrix()
    matrix_info = None
    if cached_matrix is not None:
        matrix_info = {
            "num_waypoints": len(cached_matrix.get("labels", [])),
            "num_sam_avoiding_paths": len(cached_matrix.get("paths", {})),
            "excluded_targets": cached_matrix.get("excluded_targets", []),
            "wrapped_polygons": cached_matrix.get("wrapped_polygons", []),
        }

    return {
        "success": True,
        "environment": _current_env,
        "matrix_cached": cached_matrix is not None,
        "matrix_info": matrix_info
    }


@app.get("/api/export_environment")
def export_environment():
    """
    Export environment with SAM-aware distance matrix reference in JSON format.

    The exported JSON keeps the environment clean and includes only references
    to the distance matrix (stored in Supabase), not the matrix itself.

    Export format:
    {
        "environment": {...},                    # Clean environment data
        "env_hash": "abc123...",                 # Hash of env geometry for cache lookup
        "routing_model": {...},                  # Routing parameters used
        "routing_model_hash": "def456...",       # Hash for cache lookup
        "distance_matrix_id": "uuid...",         # Reference to Supabase matrix
        "matrix_info": {...},                    # Metadata (counts, excluded targets)
        "exported_at": "2025-12-30T..."
    }

    Returns JSON file with Content-Disposition header to force correct filename.
    Filename format: ise-env-YYMMDDHH-n.json where n is running number.
    """
    global _current_env, _export_counter

    if _current_env is None:
        if ENV_PATH.exists():
            _current_env = json.loads(ENV_PATH.read_text())
        else:
            return JSONResponse(
                status_code=404,
                content={"success": False, "error": "No environment loaded"}
            )

    # Generate filename: ise-env-YYMMDDHH-n.json
    now = datetime.now()
    timestamp = now.strftime("%y%m%d%H")  # YYMMDDHH format
    filename = f"ise-env-{timestamp}-{_export_counter}.json"

    # Increment counter for next export
    _export_counter += 1

    # Build export data with environment and matrix reference (not the matrix itself)
    cached_matrix = get_current_matrix()
    export_data = {
        "environment": _current_env,
        "exported_at": now.isoformat(),
        "matrix_cached": cached_matrix is not None,
    }

    # Include matrix REFERENCE and routing model (not the matrix itself)
    if cached_matrix is not None:
        # Add cache keys for reproducibility
        if "env_hash" in cached_matrix:
            export_data["env_hash"] = cached_matrix["env_hash"]
        if "routing_model" in cached_matrix:
            export_data["routing_model"] = cached_matrix["routing_model"]
        if "routing_model_hash" in cached_matrix:
            export_data["routing_model_hash"] = cached_matrix["routing_model_hash"]
        if "distance_matrix_id" in cached_matrix:
            export_data["distance_matrix_id"] = cached_matrix["distance_matrix_id"]

        # Include metadata (not the actual matrix)
        export_data["matrix_info"] = {
            "num_waypoints": len(cached_matrix.get("labels", [])),
            "num_sam_avoiding_paths": len(cached_matrix.get("paths", {})),
            "excluded_targets": cached_matrix.get("excluded_targets", []),
            "wrapped_polygons": cached_matrix.get("wrapped_polygons", []),
            "cluster_info": cached_matrix.get("cluster_info", {}),
            "buffer": cached_matrix.get("buffer", 0.0),
            "cache_hit": cached_matrix.get("cache_hit", False),
        }

    print(f"📤 [ISR_WEB] Exporting environment as {filename}", flush=True)
    if "distance_matrix_id" in export_data:
        print(f"   📎 Matrix ref: {export_data['distance_matrix_id'][:8]}...", flush=True)

    # Return JSON response with Content-Disposition header to force filename
    return JSONResponse(
        content=export_data,
        headers={
            "Content-Disposition": f'attachment; filename="{filename}"',
            "Content-Type": "application/json"
        }
    )


@app.post("/api/agent/run")
def run_agent(req: Dict[str, Any]):
    """
    Simple multi-drone planner.
    req = {
      "mode": "heuristic",
      "drone_configs": { "1": {...}, "2": {...}, ... },
      "enabled_drones": { "1": true, "2": false, ... }
    }
    """
    global _current_env
    if _current_env is None:
        if ENV_PATH.exists():
            _current_env = json.loads(ENV_PATH.read_text())
        else:
            return {"success": False, "error": "No environment loaded"}

    env = _current_env
    drone_cfgs = req.get("drone_configs") or {}
    enabled = req.get("enabled_drones") or {}

    # all target ids
    all_target_ids = [t["id"] for t in env.get("targets", [])]
    routes: Dict[str, Dict[str, Any]] = {}
    sequences: Dict[str, str] = {}

    # naive split: give all targets to drone 1, none to others
    # (we'll improve later)
    for did_str, cfg in drone_cfgs.items():
        did = int(did_str)
        if not enabled.get(did, False):
            # trivial route
            start_id = cfg.get("start_airport", "A1")
            end_id = cfg.get("end_airport", start_id)
            route = [start_id, end_id]
        else:
            start_id = cfg.get("start_airport", "A1")
            end_id = cfg.get("end_airport", start_id)
            fuel = float(cfg.get("fuel_budget", 999))

            if did == 1:
                candidate_ids = all_target_ids[:]  # all targets
            else:
                candidate_ids = []  # for now, no targets for others

            route = _nearest_neighbor_route(env, start_id, end_id, candidate_ids, fuel)

        seq = ",".join(route)
        sequences[did_str] = seq
        routes[did_str] = {"route": route, "sequence": seq}

    # Save like your current system for compatibility if needed
    AI_SOLUTION_PATH.write_text(json.dumps({"sequences": sequences}, indent=2))
    MISSION_PATH.write_text(json.dumps({"drone_configs": drone_cfgs}, indent=2))

    return {
        "success": True,
        "routes": routes,
        "sequences": sequences,
        "message": "Simple heuristic planner executed.",
    }


# debugging: allow viewing raw files
@app.get("/debug/env", response_class=PlainTextResponse)
def debug_env():
    if not ENV_PATH.exists():
        return "No environment.json yet."
    return ENV_PATH.read_text()

@app.post("/api/solve", response_model=SolveResponse)
def solve(req: SolveRequest):
    """
    Solve mission using the full orienteering solver (Held-Karp optimal).
    This replaces the greedy planner with the exact orienteering solver.
    """
    env = req.env
    drone_configs = req.drone_configs

    # Step 3 guards: mission continuity + env hash
    mission_id, continuity, env_hash, cfg_hash, mismatch_reason = enforce_mission_continuity(
        req.mission_id, env, drone_configs
    )
    if mismatch_reason:
        # Treat as new solve; do not attempt to "continue" a mismatched mission_id
        print(f"[solve] mission continuity broken ({mismatch_reason}); starting new solve", flush=True)
        mission_id = f"m_{uuid4().hex}"
        continuity = False
    elif not mission_id:
        mission_id = f"m_{uuid4().hex}"
        continuity = False



    global _current_run_id, _current_env_version_id, _current_plan_id

    # Ensure run exists (minimal)
    if _current_run_id is None:
        _current_run_id = create_mission_run(system_version="v4", mission_name="isr-run")

    # Snapshot env used for this solve
    if _current_run_id is not None:
        print("✅ About to create_env_version. run_id =", _current_run_id, flush=True)
        try:
            _current_env_version_id = create_env_version(
                run_id=_current_run_id,
                env_snapshot=env,
                source="human",
                reason="solve",
            )
            print("✅ create_env_version returned:", _current_env_version_id, flush=True)
        except Exception as e:
            print("❌ create_env_version raised:", repr(e), flush=True)
            raise

    # allocation_strategy = req.allocation_strategy  # TODO: integrate with solve_mission

    # Use the original solver (allocation strategy integration pending)
    result = solve_mission(env, drone_configs)

    routes: Dict[str, Any] = {}
    sequences: Dict[str, str] = {}

    # Convert solver output to API response format
    for drone_id_str, route_data in result.get("routes", {}).items():
        routes[drone_id_str] = {
            "route": route_data.get("route", []),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0.0),
            "fuel_budget": route_data.get("fuel_budget", 0.0),
            "trajectory": route_data.get("trajectory", []),  # SAM-avoiding path
        }
        sequences[drone_id_str] = route_data.get("sequence", "")

    # Compute wrapped polygons for visualization
    wrapped_polygons = []
    sams = env.get("sams", [])
    if sams:
        # Convert SAMs to format expected by wrap_sams
        sams_for_wrapping = []
        for sam in sams:
            pos = sam.get("pos") or sam.get("position")
            if pos:
                sams_for_wrapping.append({
                    'x': pos[0] if isinstance(pos, (list, tuple)) else pos,
                    'y': pos[1] if isinstance(pos, (list, tuple)) else pos,
                    'radius': float(sam.get("range", sam.get("radius", 15)))
                })
        if sams_for_wrapping:
            wrapped_polygon_arrays, _ = wrap_sams(sams_for_wrapping)
            wrapped_polygons = [poly.tolist() for poly in wrapped_polygon_arrays]
 
    # Store a draft plan for this solve
    if _current_run_id is not None:
        trajectories = {d: r.get("trajectory", []) for d, r in routes.items()}

        metrics: Dict[str, Any] = {}
        for d, r in routes.items():
            metrics[d] = {
                "distance": r.get("distance", 0.0),
                "points": r.get("points", 0),
                "fuel_budget": r.get("fuel_budget", 0.0),
                "route": r.get("route", []),
                "sequence": sequences.get(d, ""),
            }

        print("✅ About to create_plan. env_version_id =", _current_env_version_id, flush=True)

        try:
            _current_plan_id = create_plan(
                run_id=_current_run_id,
                env_version_id=_current_env_version_id,
                status="draft",
                starts_by_drone={},
                allocation={},
                trajectories=trajectories,
                metrics=metrics,
                notes="draft from /api/solve",
            )
            print("✅ create_plan returned:", _current_plan_id, flush=True)

        except Exception as e:
            print("❌ create_plan raised:", repr(e), flush=True)
            raise

    # Step 3 guards: verify output matches env/constraints (basic)
    errors = verify_solution_basic(env, routes, drone_configs)
    if errors:
        # Do not mutate mission store on invalid solve
        return SolveResponse(
            success=False,
            routes=routes or {},
            sequences=sequences or {},
            wrapped_polygons=wrapped_polygons,
            mission_id=mission_id,
            env_hash=env_hash,
            continuity=continuity,
            errors=errors,
        )

    # Persist mission snapshot for continuity (used by chat-v4 and future endpoints)
    MISSION_STORE[mission_id] = {
        "env": env,
        "drone_configs": drone_configs,
        "env_hash": env_hash,
        "cfg_hash": cfg_hash,
        "routes": routes,
        "allocation": None,
        "total_points": sum(r.get("points", 0) for r in routes.values()) if isinstance(routes, dict) else 0,
        "total_fuel": sum(r.get("distance", 0) for r in routes.values()) if isinstance(routes, dict) else 0.0,
        "updated_at": datetime.utcnow().isoformat(),
    }

    return SolveResponse(
        success=True,
        routes=routes,
        sequences=sequences,
        wrapped_polygons=wrapped_polygons,
        mission_id=mission_id,
        env_hash=env_hash,
        continuity=continuity,
        errors=None,
    )

class SolveWithAllocationRequest(BaseModel):
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]
    allocation_strategy: str = "geographic"
    use_sam_aware_distances: bool = False  # Disabled for speed - SAM avoidance happens at trajectory time
    post_optimize: bool = True
    visited_targets: List[str] = []  # Targets already visited (for checkpoint replanning)
    is_checkpoint_replan: bool = False  # Whether this is a checkpoint replan
    mission_id: Optional[str] = None  # optional continuity handle


class PrepareMatrixRequest(BaseModel):
    env: Dict[str, Any]
    buffer: float = 0.0


class PrepareMatrixResponse(BaseModel):
    success: bool
    num_waypoints: int
    num_paths_with_sam_avoidance: int
    excluded_targets: List[str]


class CoverageStatsRequest(BaseModel):
    solution: Dict[str, Any]
    env: Dict[str, Any]


class CoverageStatsResponse(BaseModel):
    targets_visited: int
    targets_total: int
    coverage_percent: float
    points_collected: int
    points_possible: int
    points_percent: float
    total_distance: float
    unvisited_targets: List[str]


@app.post("/api/solve_with_allocation", response_model=SolveResponse)
def solve_with_allocation(req: SolveWithAllocationRequest):
    """
    Solve mission using the enhanced solver with:
    1. SAM-aware distance matrix (if enabled)
    2. Target allocation across drones
    3. Per-drone orienteering optimization
    4. Post-optimization for unvisited targets

    This is the recommended endpoint for multi-drone missions.
    """
    env = req.env
    drone_configs = req.drone_configs

    # Step 3 guards: mission continuity + env hash
    mission_id, continuity, env_hash, cfg_hash, mismatch_reason = enforce_mission_continuity(
        req.mission_id, env, drone_configs
    )
    if mismatch_reason:
        print(f"[solve_with_allocation] mission continuity broken ({mismatch_reason}); starting new solve", flush=True)
        mission_id = f"m_{uuid4().hex}"
        continuity = False
    elif not mission_id:
        mission_id = f"m_{uuid4().hex}"
        continuity = False

    # Required fields guard (fail fast)
    missing = []
    if not env.get("airports"): missing.append("env.airports")
    if "targets" not in env: missing.append("env.targets")
    if not drone_configs: missing.append("drone_configs")
    if missing:
        return SolveResponse(success=False, routes={}, sequences={}, wrapped_polygons=None, allocations=None, distance_matrix=None,
                             mission_id=mission_id, env_hash=env_hash, continuity=continuity,
                             errors=["Missing required fields: " + ", ".join(missing)])

    # Create mission run for Supabase logging
    run_id = create_mission_run(
        system_version="solve_with_allocation",
        mission_name="planner-solve",
        objective_weights={"allocation_strategy": req.allocation_strategy},
        constraints={"is_checkpoint_replan": req.is_checkpoint_replan},
    )

    # Log solve request
    if run_id:
        num_targets = len(req.env.get("targets", []))
        num_drones = len(req.drone_configs) if req.drone_configs else 0
        log_event(run_id, "SOLVE_REQUESTED", {
            "num_targets": num_targets,
            "num_drones": num_drones,
            "allocation_strategy": req.allocation_strategy,
            "is_checkpoint_replan": req.is_checkpoint_replan,
            "visited_targets": req.visited_targets or [],
        })

    # Filter out visited targets (for checkpoint replanning or any solve with visited targets)
    env_to_solve = req.env
    if req.visited_targets:
        visited_set = set(req.visited_targets)
        env_to_solve = dict(req.env)  # shallow copy
        original_targets = env_to_solve.get("targets", [])
        env_to_solve["targets"] = [t for t in original_targets if t.get("id") not in visited_set]
        label = "CHECKPOINT REPLAN" if req.is_checkpoint_replan else "SOLVE WITH VISITED"
        print(f"🔄 {label}: Filtered {len(visited_set)} visited targets, {len(env_to_solve['targets'])} remaining", flush=True)
        print(f"   Visited: {req.visited_targets}", flush=True)
        print(f"   Synthetic starts: {list(env_to_solve.get('synthetic_starts', {}).keys())}", flush=True)
        print(f"   Checkpoints: {[c.get('id') for c in env_to_solve.get('checkpoints', [])]}", flush=True)

    # Debug: Always log checkpoints
    checkpoints_in_env = env_to_solve.get('checkpoints', [])
    if checkpoints_in_env:
        print(f"📍 Checkpoints in request: {[c.get('id') for c in checkpoints_in_env]}", flush=True)
    else:
        print(f"⚠️ No checkpoints in request env", flush=True)

    result = solve_mission_with_allocation(
        env=env_to_solve,
        drone_configs=req.drone_configs,
        allocation_strategy=req.allocation_strategy,
        use_sam_aware_distances=req.use_sam_aware_distances,
        post_optimize=False, # req.post_optimize,
    )

    routes: Dict[str, Any] = {}
    sequences: Dict[str, str] = {}

    for drone_id_str, route_data in result.get("routes", {}).items():
        routes[drone_id_str] = {
            "route": route_data.get("route", []),
            "points": route_data.get("points", 0),
            "distance": route_data.get("distance", 0.0),
            "fuel_budget": route_data.get("fuel_budget", 0.0),
            "trajectory": route_data.get("trajectory", []),  # SAM-avoiding path
        }
        sequences[drone_id_str] = route_data.get("sequence", "")

    # Compute wrapped polygons for visualization (same as delivery planner)
    wrapped_polygons = []
    sams = req.env.get("sams", [])
    if sams:
        # Convert SAMs to format expected by wrap_sams_to_polygons
        sams_for_wrapping = []
        for sam in sams:
            pos = sam.get("pos") or sam.get("position")
            if pos:
                sams_for_wrapping.append({
                    'x': pos[0] if isinstance(pos, (list, tuple)) else pos,
                    'y': pos[1] if isinstance(pos, (list, tuple)) else pos,
                    'radius': float(sam.get("range", sam.get("radius", 15)))
                })
        if sams_for_wrapping:
            wrapped_polygon_arrays, _ = wrap_sams(sams_for_wrapping)
            wrapped_polygons = []
            for poly in wrapped_polygon_arrays:
                # If it's a NumPy array, convert to list; if it's already a list, keep it
                if hasattr(poly, "tolist"):
                    wrapped_polygons.append(poly.tolist())
                else:
                    wrapped_polygons.append(poly)
            # Debug: Print polygon bounds for comparison with boundary_navigation
            for i, poly in enumerate(wrapped_polygon_arrays):
                min_x = min(v[0] for v in poly)
                max_x = max(v[0] for v in poly)
                min_y = min(v[1] for v in poly)
                max_y = max(v[1] for v in poly)
                print(f"📐 Frontend polygon {i}: bounds X=[{min_x:.1f}, {max_x:.1f}], Y=[{min_y:.1f}, {max_y:.1f}]", flush=True)

    # Extract allocations from result
    allocations = result.get("allocations", {})

    # Extract distance matrix from result (needed for post-optimization)
    distance_matrix = result.get("distance_matrix", {})

    # Log solve completion
    if run_id:
        total_points = sum(r.get("points", 0) for r in routes.values())
        total_distance = sum(r.get("distance", 0) for r in routes.values())
        log_event(run_id, "SOLVE_COMPLETED", {
            "num_routes": len(routes),
            "total_points": total_points,
            "total_distance": round(total_distance, 2),
            "allocations": allocations,
        })

    # Step 3 guards: verify output matches env/constraints (basic)
    errors = verify_solution_basic(env_to_solve, routes, drone_configs)
    if errors:
        return SolveResponse(
            success=False,
            routes=routes or {},
            sequences=sequences or {},
            wrapped_polygons=wrapped_polygons,
            allocations=allocations,
            distance_matrix=distance_matrix,
            mission_id=mission_id,
            env_hash=env_hash,
            continuity=continuity,
            errors=errors,
        )

    # Persist mission snapshot for continuity
    MISSION_STORE[mission_id] = {
        "env": env,
        "drone_configs": drone_configs,
        "env_hash": env_hash,
        "cfg_hash": cfg_hash,
        "routes": routes,
        "allocation": allocations,
        "total_points": sum(r.get("points", 0) for r in routes.values()) if isinstance(routes, dict) else 0,
        "total_fuel": sum(r.get("distance", 0) for r in routes.values()) if isinstance(routes, dict) else 0.0,
        "updated_at": datetime.utcnow().isoformat(),
    }

    return SolveResponse(
        success=True,
        routes=routes,
        sequences=sequences,
        wrapped_polygons=wrapped_polygons,
        allocations=allocations,
        distance_matrix=distance_matrix,
        mission_id=mission_id,
        env_hash=env_hash,
        continuity=continuity,
        errors=None,
    )


@app.post("/api/prepare_matrix", response_model=PrepareMatrixResponse)
def api_prepare_matrix(req: PrepareMatrixRequest):
    """
    Pre-calculate SAM-aware distance matrix for the environment.

    Call this after editing is complete to have the matrix ready
    for fast solving. The matrix is cached and reused by solve endpoints.
    """
    matrix_data = prepare_distance_matrix(req.env, req.buffer)

    num_waypoints = len(matrix_data.get("labels", []))
    num_paths = len(matrix_data.get("paths", {}))
    excluded = matrix_data.get("excluded_targets", [])

    return PrepareMatrixResponse(
        success=True,
        num_waypoints=num_waypoints,
        num_paths_with_sam_avoidance=num_paths,
        excluded_targets=excluded,
    )


@app.get("/api/matrix_status")
def api_matrix_status():
    """
    Get the status of the cached distance matrix.
    """
    matrix = get_current_matrix()

    if matrix is None:
        return {
            "cached": False,
            "num_waypoints": 0,
            "num_paths": 0,
        }

    return {
        "cached": True,
        "num_waypoints": len(matrix.get("labels", [])),
        "num_paths": len(matrix.get("paths", {})),
        "excluded_targets": matrix.get("excluded_targets", []),
    }


@app.post("/api/clear_matrix")
def api_clear_matrix():
    """
    Clear the cached distance matrix.
    """
    clear_cached_matrix()
    return {"success": True}


@app.post("/api/apply_sequence", response_model=ApplySequenceResponse)
def api_apply_sequence(req: ApplySequenceRequest):
    """
    Apply a manually edited sequence for a single drone.

    Calculates the trajectory, distance, and points for the given sequence.
    Returns the updated route data for the drone.
    """
    try:
        # Parse sequence into route list
        route_ids = [wp.strip().upper() for wp in req.sequence.split(",") if wp.strip()]

        if not route_ids:
            return ApplySequenceResponse(
                success=False,
                route=[],
                distance=0.0,
                points=0,
                trajectory=[],
                error="Empty sequence"
            )

        env = req.env
        sams = env.get("sams", [])
        airports = env.get("airports", [])
        targets = env.get("targets", [])
        checkpoints = env.get("checkpoints", [])

        # Build waypoint position lookup
        waypoint_positions: Dict[str, List[float]] = {}
        for a in airports:
            aid = str(a.get("id", ""))
            waypoint_positions[aid] = [float(a["x"]), float(a["y"])]
        for t in targets:
            tid = str(t.get("id", ""))
            waypoint_positions[tid] = [float(t["x"]), float(t["y"])]
        # Add checkpoints (C1-1, C1-2, etc.) for segmented missions
        for cp in checkpoints:
            cpid = str(cp.get("id", ""))
            waypoint_positions[cpid] = [float(cp["x"]), float(cp["y"])]

        # Validate all waypoints exist
        for wp_id in route_ids:
            if wp_id not in waypoint_positions:
                return ApplySequenceResponse(
                    success=False,
                    route=route_ids,
                    distance=0.0,
                    points=0,
                    trajectory=[],
                    error=f"Unknown waypoint: {wp_id}"
                )

        # Generate SAM-avoiding trajectory
        trajectory_planner = ISRTrajectoryPlanner(sams)
        trajectory = trajectory_planner.generate_trajectory(route_ids, waypoint_positions)

        # Calculate total distance using SAM-aware distance matrix
        total_distance = 0.0
        distance_matrix = get_current_matrix()
        if distance_matrix:
            labels = distance_matrix.get("labels", [])
            matrix = distance_matrix.get("matrix", [])
            label_to_idx = {str(lbl): i for i, lbl in enumerate(labels)}

            print(f"[apply_sequence] D{req.drone_id}: Computing distance for route {route_ids}", flush=True)
            print(f"[apply_sequence] Matrix has {len(labels)} labels: {labels[:20]}{'...' if len(labels) > 20 else ''}", flush=True)
            # Check which route waypoints are missing from matrix
            missing = [wp for wp in route_ids if wp not in label_to_idx]
            if missing:
                print(f"[apply_sequence] ⚠️ MISSING from matrix: {missing}", flush=True)

            for i in range(len(route_ids) - 1):
                from_id = route_ids[i]
                to_id = route_ids[i + 1]
                from_idx = label_to_idx.get(from_id)
                to_idx = label_to_idx.get(to_id)
                if from_idx is not None and to_idx is not None:
                    seg_dist = matrix[from_idx][to_idx]
                    total_distance += seg_dist
                    print(f"   {from_id}→{to_id}: {seg_dist:.1f} (matrix) → cumulative: {total_distance:.1f}", flush=True)
                else:
                    # Fallback to Euclidean for unknown waypoints (e.g., synthetic starts)
                    from_pos = waypoint_positions.get(from_id)
                    to_pos = waypoint_positions.get(to_id)
                    if from_pos and to_pos:
                        dx = to_pos[0] - from_pos[0]
                        dy = to_pos[1] - from_pos[1]
                        seg_dist = math.hypot(dx, dy)
                        total_distance += seg_dist
                        print(f"   {from_id}→{to_id}: {seg_dist:.1f} (EUCLIDEAN fallback) → cumulative: {total_distance:.1f}", flush=True)
                    else:
                        print(f"   {from_id}→{to_id}: SKIPPED - missing position data!", flush=True)
        else:
            # Fallback: calculate from trajectory if no matrix available
            print(f"[apply_sequence] D{req.drone_id}: NO MATRIX - using trajectory distance", flush=True)
            for i in range(1, len(trajectory)):
                dx = trajectory[i][0] - trajectory[i-1][0]
                dy = trajectory[i][1] - trajectory[i-1][1]
                total_distance += math.hypot(dx, dy)

        print(f"[apply_sequence] D{req.drone_id}: FINAL distance={total_distance:.1f}", flush=True)

        # Calculate points from visited targets
        total_points = 0
        target_priorities = {str(t["id"]): int(t.get("priority", 5)) for t in targets}
        for wp_id in route_ids:
            if wp_id.startswith("T") and wp_id in target_priorities:
                total_points += target_priorities[wp_id]

        return ApplySequenceResponse(
            success=True,
            route=route_ids,
            distance=total_distance,
            points=total_points,
            trajectory=trajectory,
            error=None
        )

    except Exception as e:
        return ApplySequenceResponse(
            success=False,
            route=[],
            distance=0.0,
            points=0,
            trajectory=[],
            error=str(e)
        )


@app.post("/api/coverage_stats", response_model=CoverageStatsResponse)
def api_coverage_stats(req: CoverageStatsRequest):
    """
    Get coverage statistics for a mission solution.

    Returns metrics on targets visited, points collected,
    and lists any unvisited targets.
    """
    stats = get_coverage_stats(req.solution, req.env)

    return CoverageStatsResponse(
        targets_visited=stats.get("targets_visited", 0),
        targets_total=stats.get("targets_total", 0),
        coverage_percent=stats.get("coverage_percent", 0.0),
        points_collected=stats.get("points_collected", 0),
        points_possible=stats.get("points_possible", 0),
        points_percent=stats.get("points_percent", 0.0),
        total_distance=stats.get("total_distance", 0.0),
        unvisited_targets=stats.get("unvisited_targets", []),
    )


@app.post("/api/agents/chat", response_model=AgentChatResponse)
async def agent_chat(req: AgentChatRequest):
    print(">>> /api/agents/chat (v3 endpoint) HIT <<<", flush=True)
    """
    ISR Agent endpoint using Claude with tool-calling.

    Frontend sends:
      - message: user's natural language instructions
      - env: current environment (airports, targets, sams)
      - sequences: current sequences, if any (for context)
      - drone_configs: config table from the Config tab

    Returns:
      - reply: The agent's response text
      - route: Extracted route waypoints (if a route was planned)
      - points: Total points collected
      - fuel: Total fuel used
    """
    try:
        # Use the new ISR agent with Claude and tools
        # Pass drone_configs and sequences so agent knows all constraints
        # Use req.drone_configs if provided, else extract from env
        drone_configs = req.drone_configs or req.env.get("drone_configs")
        result = run_isr_agent(
            env=req.env,
            user_query=req.message,
            drone_configs=drone_configs,
            sequences=req.sequences
        )

        return AgentChatResponse(
            reply=result.get("response", "(No agent reply)"),
            intent=result.get("intent"),
            actions=result.get("actions"),
            policy=result.get("policy"),
            constraints=result.get("constraints"),
            valid=result.get("valid", True),
            warnings=result.get("warnings"),
            errors=result.get("errors"),
            route=result.get("route"),
            routes=result.get("routes"),  # Multi-drone routes
            trajectories=result.get("trajectories"),  # SAM-avoiding trajectories
            points=result.get("points"),
            fuel=result.get("fuel"),
            allocations=result.get("allocation"),  # Target allocations
            allocation_strategy=result.get("allocation_strategy"),  # Strategy used
        )
    except Exception as e:
        import traceback
        traceback.print_exc()
        return AgentChatResponse(
            reply=f"Error running agent: {str(e)}",
            route=None,
            routes=None,
            trajectories=None,
            points=None,
            fuel=None,
            allocations=None,
            allocation_strategy=None,
        )


# ===========================================================================
# Trace Events Builder for Agents Monitor Tab
# ===========================================================================

def _mk_event(t: str, msg: str, data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Create a single trace event dict."""
    return {"t": t, "msg": msg, "data": data or None}


def _build_trace_events(
    trace: Dict[str, Any],
    allocations: Dict[str, Any],
    allocation_strategy: str,
    routes_for_ui: Dict[str, List[str]],
    trajectories: Dict[str, Any],
    total_points: int,
    total_fuel: float,
    coordinator_trace_events: Optional[List[Dict[str, Any]]] = None,
) -> List[Dict[str, Any]]:
    """
    Build an ordered list of trace events for the Agents Monitor UI tab.
    This converts the raw trace object into a normalized, renderable timeline.

    If coordinator_trace_events is provided, those are used directly instead of
    extracting from the trace object.
    """
    ev: List[Dict[str, Any]] = []

    # If we have structured coordinator trace events, transform and use them
    if coordinator_trace_events:
        for cte in coordinator_trace_events:
            # Transform Coordinator format {agent, stage, event, details}
            # to AgentTraceEvent format {t, msg, data}
            agent = cte.get("agent", "Coordinator")
            stage = cte.get("stage", "unknown")
            event = cte.get("event", "")
            details = cte.get("details", {})

            # Build t (type) from agent.stage or just stage
            t = f"{agent.lower()}.{stage}" if agent else stage

            # Build msg (human readable) from event and key details
            msg_parts = [event.replace("_", " ").title()]
            if stage == "intent" and "intent" in details:
                msg_parts.append(f"→ {details['intent']}")
            elif stage == "policy" and "allocation_strategy" in details:
                msg_parts.append(f"→ {details['allocation_strategy']} allocation")
            elif stage == "validate":
                if details.get("errors"):
                    msg_parts.append(f"({len(details['errors'])} errors)")
                elif details.get("warnings"):
                    msg_parts.append(f"({len(details['warnings'])} warnings)")
            elif stage == "final":
                msg_parts.append(f"(intent={details.get('intent', '?')})")

            msg = " ".join(msg_parts)
            ev.append(_mk_event(t, msg, details))
    else:
        # Fall back to extracting from trace object (legacy path)
        coord = trace.get("coordinator", {}) if isinstance(trace, dict) else {}
        intent = coord.get("intent") or coord.get("intent_type") or "unknown"
        ev.append(_mk_event(
            "coordinator.intent",
            f"Coordinator intent: {intent}",
            {"intent": intent, "coordinator": coord}
        ))

        # Policy / flags
        policy = trace.get("policy_rules") or trace.get("policy") or {}
        if isinstance(policy, dict) and policy:
            ev.append(_mk_event(
                "coordinator.policy",
                "Coordinator policy selected",
                policy
            ))

    # Allocation result
    alloc_count = sum(len(v) for v in allocations.values()) if isinstance(allocations, dict) else 0
    ev.append(_mk_event(
        "allocator.result",
        f"Allocation ({allocation_strategy}): {alloc_count} targets assigned",
        {"strategy": allocation_strategy, "allocations": allocations}
    ))

    # Solver summary (routes)
    route_summary = {
        did: {"n": len(r), "start": r[0] if r else None, "end": r[-1] if r else None}
        for did, r in (routes_for_ui or {}).items()
    }
    ev.append(_mk_event(
        "solver.routes",
        f"Routes produced for {len(route_summary)} drones",
        {"routes_summary": route_summary, "routes": routes_for_ui}
    ))

    # Trajectories summary - count of [x,y] coordinate points per drone
    traj_summary = {
        did: (len(traj) if isinstance(traj, list) else 0)
        for did, traj in (trajectories or {}).items()
    }
    total_coords = sum(traj_summary.values())
    ev.append(_mk_event(
        "planner.trajectories",
        f"SAM-aware trajectories: {total_coords} total coords",
        {"coords_per_drone": traj_summary}
    ))

    # Metrics totals
    ev.append(_mk_event(
        "metrics.totals",
        f"Totals: points={total_points}, fuel={total_fuel:.1f}",
        {"points": total_points, "fuel": total_fuel}
    ))

    # Any embedded stepwise trace (optional future expansion)
    steps = trace.get("steps") if isinstance(trace, dict) else None
    if isinstance(steps, list) and steps:
        ev.append(_mk_event("trace.steps", f"{len(steps)} step records", {"steps": steps}))

    return ev


@app.post("/api/agents/chat-v4", response_model=AgentChatResponse)
async def agent_chat_v4(req: AgentChatRequest):
    """
    ISR Agent v4 endpoint using the reasoning-based multi-agent system.

    - If mission_id is omitted: create/solve a new mission and return mission_id.
    - If mission_id is provided: load existing mission state and allow
      question-only interaction (no recompute unless the request asks for it).
    """
    print(">>> /api/agents/chat-v4 (v4 endpoint) HIT <<<", flush=True)
    t0 = time.time()

    try:
        # -------------------------------
        # 1. Resolve env & configs FIRST
        # -------------------------------
        env = req.env
        drone_configs = req.drone_configs or env.get("drone_configs") or {}

        mission_id = req.mission_id
        existing_solution: Optional[Dict[str, Any]] = None

        # Priority 1: Use existing_solution from request (frontend sends current routes)
        # This enables follow-up requests like "move T5 to D1" to work
        if req.existing_solution:
            existing_solution = req.existing_solution
            print(f"[v3] Using existing_solution from request: {len(existing_solution.get('routes', {}))} routes", flush=True)
        # Priority 2: Load from mission store by mission_id
        elif mission_id and mission_id in MISSION_STORE:
            mission = MISSION_STORE[mission_id]

            # Step 3 guards: ensure the incoming env/configs match the stored mission snapshot
            incoming_env_hash = compute_env_hash(env)
            incoming_cfg_hash = compute_cfg_hash(req.drone_configs or drone_configs)
            stored_env_hash = mission.get("env_hash")
            stored_cfg_hash = mission.get("cfg_hash")

            if stored_env_hash and stored_env_hash != incoming_env_hash:
                print(f"[v3] mission_id={mission_id} env_hash mismatch; ignoring stored solution and treating as new mission", flush=True)
                mission_id = None
            elif stored_cfg_hash and stored_cfg_hash != incoming_cfg_hash:
                print(f"[v3] mission_id={mission_id} cfg_hash mismatch; ignoring stored solution and treating as new mission", flush=True)
                mission_id = None
            else:
                # Use incoming env if provided, fallback to stored env
                if not req.env:
                    env = mission.get("env", env)
                if not req.drone_configs:
                    drone_configs = mission.get("drone_configs", drone_configs)

                existing_solution = {
                    "routes": mission.get("routes") or {},
                    "allocation": mission.get("allocation") or {},
                }
                print(f"[v3] Loaded existing mission {mission_id} from store", flush=True)
        elif mission_id:
            print(f"[v3] mission_id={mission_id} not found; treating as new mission", flush=True)
            mission_id = None

        # -------------------------------
        # 2. Pre-compute SAM-aware distance matrix
        # -------------------------------
        # This ensures cache lookup happens ONCE before v4,
        # and gives us env_hash, routing_model_hash, distance_matrix_id
        sam_matrix: Optional[Dict[str, Any]] = None
        env_hash: Optional[str] = None
        routing_model_hash: Optional[str] = None
        distance_matrix_id: Optional[str] = None

        sams = env.get("sams", [])
        if sams:
            try:
                print(f"[v3] Pre-computing SAM-aware distance matrix...", flush=True)
                sam_matrix = calculate_sam_aware_matrix(env, use_supabase_cache=True)
                env_hash = sam_matrix.get("env_hash")
                routing_model_hash = sam_matrix.get("routing_model_hash")
                distance_matrix_id = sam_matrix.get("distance_matrix_id")
                cache_hit = sam_matrix.get("cache_hit", False)
                print(f"[v3] Matrix ready: env_hash={env_hash[:8] if env_hash else 'N/A'}..., "
                      f"matrix_id={distance_matrix_id[:8] if distance_matrix_id else 'N/A'}..., "
                      f"cache_hit={cache_hit}", flush=True)
            except Exception as e:
                print(f"[v3] Warning: SAM matrix pre-compute failed: {e}", flush=True)

        # -------------------------------
        # 3. Create agent_run row BEFORE calling v3 (Supabase-first)
        # -------------------------------
        agent_run_id = create_agent_run(
            request_text=req.message,
            agent_version="v4",
            mode="agentic",
        )

        # -------------------------------
        # 4. Call the v4 multi-agent planner (with Mission Planner orchestration)
        # -------------------------------
        result = run_multi_agent_v4(
            user_message=req.message,
            environment=env,
            drone_configs=drone_configs,
            sam_matrix=sam_matrix,  # Pass pre-computed matrix with metadata
            existing_solution=existing_solution,
            llm_config=req.llm_config,  # Pass LLM provider configuration
        )

        if not isinstance(result, dict):
            raise RuntimeError(f"run_multi_agent_v4 returned non-dict: {type(result)}")

        runtime_ms = int((time.time() - t0) * 1000)

        # -------------------------------
        # 4. Extract outputs from result
        # -------------------------------
        raw_routes = result.get("routes") or {}
        trajectories = result.get("trajectories") or {}
        allocations = result.get("allocation") or {}
        allocation_strategy = result.get("allocation_strategy", "unknown")

        # Extract Decision Trace v1 fields (if present)
        trace = result.get("trace", {})
        optimizer_steps = result.get("optimizer_steps", [])
        solver_type = result.get("solver_type")
        solver_runtime_ms = result.get("solver_runtime_ms")
        is_valid = result.get("valid", True)

        total_points = result.get("total_points", 0)
        total_fuel = result.get("total_fuel", 0.0)

        # Build routes for UI
        routes_for_ui: Dict[str, List[str]] = {}
        for did, route_info in raw_routes.items():
            if isinstance(route_info, dict):
                routes_for_ui[str(did)] = route_info.get("route", []) or []
            elif isinstance(route_info, list):
                routes_for_ui[str(did)] = route_info
            else:
                routes_for_ui[str(did)] = []

        # Legacy single route (D1) for older UI helpers
        legacy_route = routes_for_ui.get("1")

        # -------------------------------
        # 5. Update agent_run with results (after v4 completes)
        # -------------------------------
        if agent_run_id and raw_routes:
            # Build summary per-drone
            summary = {}
            for did, route_info in raw_routes.items():
                if isinstance(route_info, dict):
                    summary[str(did)] = {
                        "route": route_info.get("route", []),
                        "distance": route_info.get("distance", 0.0),
                        "points": route_info.get("points", 0),
                    }

            update_agent_run(
                agent_run_id,
                total_points=total_points,
                total_fuel_used=total_fuel,
                runtime_ms=runtime_ms,
                is_valid=is_valid,
                routes=raw_routes,
                summary=summary,
                solver_type=solver_type,
                solver_runtime_ms=solver_runtime_ms,
                # Distance matrix references for reproducibility
                env_hash=env_hash,
                routing_model_hash=routing_model_hash,
                distance_matrix_id=distance_matrix_id,
            )

            # -------------------------------
            # 6. Insert agent_trace (1:1 with run)
            # -------------------------------
            if trace:
                create_agent_trace(agent_run_id, trace)

            # -------------------------------
            # 7. Insert optimizer steps (if any)
            # -------------------------------
            for step in optimizer_steps:
                log_optimizer_step(
                    agent_run_id=agent_run_id,
                    step_index=step.get("step_index", 0),
                    operator=step.get("operator", "unknown"),
                    before_routes=step.get("before_routes", {}),
                    after_routes=step.get("after_routes", {}),
                    delta=step.get("delta", {}),
                    accepted=step.get("accepted", True),
                    notes=step.get("notes"),
                    validation_status=step.get("validation_status", "unknown"),
                    validation_details=step.get("validation_details", {}),
                )

        # -------------------------------
        # 8. Persist/refresh mission state in memory store
        # -------------------------------
        if raw_routes:
            if not mission_id:
                mission_id = f"m_{uuid4().hex}"
                print(f"[v3] Creating new mission {mission_id}", flush=True)
            else:
                print(f"[v3] Updating mission {mission_id}", flush=True)

            MISSION_STORE[mission_id] = {
                "env": env,
                "drone_configs": drone_configs,
                "env_hash": compute_env_hash(env),
                "cfg_hash": compute_cfg_hash(drone_configs),
                "routes": raw_routes,
                "allocation": allocations,
                "total_points": total_points,
                "total_fuel": total_fuel,
                "updated_at": datetime.utcnow().isoformat(),
            }

        # -------------------------------
        # 9. Build trace_events for Agents Monitor tab
        # -------------------------------
        trace = trace or {}
        # Get structured coordinator trace_events if available
        coordinator_trace_events = result.get("coordinator_trace_events", [])
        trace_events = _build_trace_events(
            trace=trace,
            allocations=allocations,
            allocation_strategy=allocation_strategy,
            routes_for_ui=routes_for_ui,
            trajectories=trajectories,
            total_points=total_points,
            total_fuel=total_fuel,
            coordinator_trace_events=coordinator_trace_events,
        )

        # -------------------------------
        # 10. Build and return response
        # -------------------------------
        return AgentChatResponse(
            reply=result.get("response", "(No v3 agent reply)"),
            route=legacy_route,
            routes=routes_for_ui,
            trajectories=trajectories,
            points=total_points,
            fuel=total_fuel,
            allocations=allocations,
            allocation_strategy=allocation_strategy,
            mission_id=mission_id,
            trace=trace,
            trace_events=trace_events,
        )

    except Exception as e:
        import traceback
        traceback.print_exc()
        return AgentChatResponse(
            reply=f"Error running v3 agent: {str(e)}",
            route=None,
            routes=None,
            trajectories=None,
            points=None,
            fuel=None,
            allocations=None,
            allocation_strategy=None,
            mission_id=None,
            trace={"error": str(e)},
            trace_events=[{"t": "error", "msg": "Agent error", "data": {"error": str(e)}}],
        )


# ============================================================================
# Agent Memory Endpoints
# ============================================================================

class AddMemoryRequest(BaseModel):
    content: str
    category: str = "correction"  # correction, instruction, preference, fact


@app.get("/api/agent/memory")
async def get_memories():
    """Get all agent memories."""
    memories = load_memory()
    return {"success": True, "memories": memories, "count": len(memories)}


@app.post("/api/agent/memory")
async def add_agent_memory(req: AddMemoryRequest):
    """Add a new memory entry."""
    entry = add_memory(req.content, req.category)
    return {"success": True, "memory": entry}


@app.delete("/api/agent/memory/{memory_id}")
async def delete_agent_memory(memory_id: int):
    """Delete a specific memory by ID."""
    success = delete_memory(memory_id)
    if success:
        return {"success": True, "message": f"Memory {memory_id} deleted"}
    return {"success": False, "message": f"Memory {memory_id} not found"}


@app.delete("/api/agent/memory")
async def clear_agent_memory():
    """Clear all memories."""
    count = clear_memory()
    return {"success": True, "message": f"Cleared {count} memories"}


@app.get("/api/wrapped_polygons")
async def get_wrapped_polygons():
    """
    Get wrapped SAM polygon boundaries from cached calculation.

    Returns the polygon boundaries created by wrapping overlapping SAMs.
    These are used for visualization on the frontend canvas.
    """
    try:
        # Get wrapped polygons from the cached distance matrix
        wrapped_polygons = sam_distance_matrix.get_wrapped_polygons()

        return {
            "success": True,
            "polygons": wrapped_polygons
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "polygons": []
        }


class TrajectorySwapRequest(BaseModel):
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


@app.post("/api/trajectory_swap_optimize")
async def api_trajectory_swap_optimize(req: TrajectorySwapRequest):
    """
    Optimize a solution by swapping targets to drones whose trajectories pass closer.

    For each target, calculates perpendicular distance to current trajectory
    (line between prev/next waypoints). If another capable drone's trajectory
    passes closer, moves the target to that drone.
    """
    print("\n" + "="*60)
    print("📥 TRAJECTORY SWAP ENDPOINT CALLED")
    print(f"   Received routes: {list(req.solution.get('routes', {}).keys())}")
    print("="*60)
    print(f"   debug flag: {bool(getattr(req, 'debug', False))}")

    # Use distance matrix from solution if available, else fall back to cache
    distance_matrix = req.solution.get('distance_matrix')
    if distance_matrix:
        print(f"   Using distance matrix from solution with {len(distance_matrix.get('labels', []))} labels")
    else:
        # Fall back to cached matrix
        distance_matrix = get_current_matrix()
        if distance_matrix:
            print(f"   Using cached distance matrix with {len(distance_matrix.get('labels', []))} labels")
        else:
            print("   WARNING: No distance matrix available!")

    try:
        # Allow client to request server-side auto-iteration / regeneration.
        # By default we keep single-pass behavior so the UI can click until no swaps.
        # By default run the full server-side cascade (auto-iterate + regen).
        # This makes the optimizer apply cascading swaps automatically unless
        # the client explicitly opts out by sending false for these flags.
        result = trajectory_swap_optimize(
            solution=req.solution,
            env=req.env,
            drone_configs=req.drone_configs,
            distance_matrix=distance_matrix,
            auto_iterate=bool(getattr(req, 'auto_iterate', True)),
            auto_regen=bool(getattr(req, 'auto_regen', True)),
            debug=bool(getattr(req, 'debug', False)),
        )

        # Log swaps made
        swaps = result.get("swaps_made", [])
        print(f"\n🔄 TRAJECTORY SWAP OPTIMIZATION: Returning {len(swaps)} swaps to frontend")
        if swaps:
            print(f"   Total swaps made: {len(swaps)}")
            for swap in swaps:
                print(f"   {swap['target']}: Drone {swap['from_drone']} → Drone {swap['to_drone']} "
                      f"(remove {swap.get('remove_delta', 0):.1f} → insert {swap.get('insert_delta', 0):.1f}, "
                      f"savings {swap.get('savings', 0):.1f})")
        else:
            print("   No beneficial swaps found")

        # Debug: Print route changes
        print(f"\n📋 Routes being returned to frontend:")
        for did, route_data in result.get("routes", {}).items():
            route = route_data.get("route", [])
            print(f"   D{did}: {' -> '.join(str(wp) for wp in route)}")

        return {
            "success": True,
            "routes": result.get("routes", {}),
            "sequences": {d: r.get("sequence", "") for d, r in result.get("routes", {}).items()},
            "swaps_made": swaps,
            "iterations": result.get("iterations", 0),
            "target_diagnostics": result.get("target_diagnostics", {})
        }
    except Exception as e:
        import traceback
        traceback.print_exc()
        return {
            "success": False,
            "error": str(e),
        }


@app.post("/api/crossing_removal_optimize")
async def api_crossing_removal_optimize(req: TrajectorySwapRequest):
    """
    Remove self-crossings from drone trajectories using 2-opt.

    Scans each drone's route for segment crossings and reverses the
    middle portion to eliminate them.
    """
    print("\n" + "="*60)
    print("📥 CROSSING REMOVAL ENDPOINT CALLED")
    print(f"   Received routes: {list(req.solution.get('routes', {}).keys())}")
    print("="*60)

    try:
        result = crossing_removal_optimize(
            solution=req.solution,
            env=req.env,
            drone_configs=req.drone_configs,
        )

        # Log fixes made
        fixes = result.get("fixes_made", [])
        if fixes:
            print(f"\n✂️ CROSSING REMOVAL: {len(fixes)} crossings fixed")
            for fix in fixes:
                print(f"   Drone {fix['drone']}: reversed segment {fix['segment_i']}-{fix['segment_j']} (pass {fix['pass']})")
        else:
            print("\n✂️ CROSSING REMOVAL: No crossings found")

        return {
            "success": True,
            "routes": result.get("routes", {}),
            "sequences": {d: r.get("sequence", "") for d, r in result.get("routes", {}).items()},
            "fixes_made": fixes,
            "passes": result.get("passes", 0),
        }
    except Exception as e:
        import traceback
        traceback.print_exc()
        return {
            "success": False,
            "error": str(e),
        }


class InsertMissedRequest(BaseModel):
    solution: Dict[str, Any]  # routes from solve_with_allocation
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]


@app.post("/api/insert_missed_optimize")
async def api_insert_missed_optimize(req: InsertMissedRequest):
    """
    Insert unvisited targets into existing routes.

    Takes the current solution and attempts to insert any unvisited targets
    into drone routes that have remaining fuel capacity. Targets are inserted
    at optimal positions to minimize additional fuel consumption.
    """
    print("\n" + "="*80, flush=True)
    print("📥 INSERT MISSED ENDPOINT CALLED", flush=True)
    print(f"   Received routes: {list(req.solution.get('routes', {}).keys())}", flush=True)

    # Log BEFORE routes
    print("\n🔍 ROUTES BEFORE INSERT MISSED:", flush=True)
    for did in sorted(req.solution.get('routes', {}).keys()):
        route = req.solution['routes'][did].get('route', [])
        print(f"   D{did}: {route} (length={len(route)})", flush=True)

    print("="*80, flush=True)

    # Use distance matrix from solution if available, else fall back to cache
    distance_matrix = req.solution.get('distance_matrix')
    if distance_matrix:
        print(f"   Using distance matrix from solution with {len(distance_matrix.get('labels', []))} labels")
    else:
        # Fall back to cached matrix
        distance_matrix = get_current_matrix()
        if distance_matrix:
            print(f"   Using cached distance matrix with {len(distance_matrix.get('labels', []))} labels")
        else:
            print("   WARNING: No distance matrix available!")

    try:
        result = post_optimize_solution(
            solution=req.solution,
            env=req.env,
            drone_configs=req.drone_configs,
            distance_matrix=distance_matrix,
        )

        # Count insertions by comparing routes
        insertions = []
        original_routes = req.solution.get("routes", {})
        optimized_routes = result.get("routes", {})

        for did, opt_data in optimized_routes.items():
            orig_data = original_routes.get(did, {})
            orig_route = set(orig_data.get("route", []))
            opt_route = set(opt_data.get("route", []))
            new_targets = opt_route - orig_route
            for tid in new_targets:
                if str(tid).startswith("T"):
                    insertions.append({"target": tid, "drone": did})

        # Log AFTER routes
        print("\n✅ ROUTES AFTER INSERT MISSED:", flush=True)
        for did in sorted(result.get('routes', {}).keys()):
            route = result['routes'][did].get('route', [])
            complete = "COMPLETE" if (route and route[0].startswith('A') and route[-1].startswith('A')) else "INCOMPLETE"
            print(f"   D{did}: {route} (length={len(route)}, {complete})", flush=True)

        if insertions:
            print(f"\n➕ INSERT MISSED OPTIMIZATION: {len(insertions)} targets inserted", flush=True)
            for ins in insertions:
                print(f"   {ins['target']} → Drone {ins['drone']}", flush=True)
        else:
            print("\n➕ INSERT MISSED OPTIMIZATION: No insertions possible (all targets visited or fuel exhausted)", flush=True)

        print("="*80 + "\n", flush=True)

        return {
            "success": True,
            "routes": result.get("routes", {}),
            "sequences": result.get("sequences", {}),
            "insertions": insertions,
        }
    except Exception as e:
        import traceback
        traceback.print_exc()
        return {
            "success": False,
            "error": str(e),
        }


@app.post("/api/calculate_wrapping")
async def calculate_wrapping(request: Request):
    """
    Calculate wrapped SAM polygon boundaries for the current environment.

    This endpoint recalculates wrapping on-demand when SAMs are edited.
    """
    try:
        data = await request.json()
        sams = data.get("sams", [])

        if not sams:
            return {
                "success": True,
                "polygons": []
            }

        # Use the SAM wrapping module directly
        import path_planning_core.sam_wrapping as sam_wrapping

        # Normalize SAM format for wrap_sams:
        #   expects: {'x': float, 'y': float, 'range': float, ...}
        # Normalize SAM format for wrap_sams:
        #   expects: {'x': float, 'y': float, 'range': float, ...}
        normalized_sams = []
        for sam in sams:
            # Get a position as [x, y]
            pos = None
            if "pos" in sam and sam["pos"] is not None:
                pos = sam["pos"]
            elif "position" in sam and sam["position"] is not None:
                pos = sam["position"]
            elif "x" in sam and "y" in sam:
                pos = [float(sam["x"]), float(sam["y"])]

            if pos is None or len(pos) < 2:
                continue

            x = float(pos[0])
            y = float(pos[1])

            norm = {
                "x": x,
                "y": y,
                "range": float(sam.get("range", sam.get("radius", 15.0))),
            }

            if "id" in sam:
                norm["id"] = sam["id"]

            normalized_sams.append(norm)


        # Calculate wrapping
        polygons, cluster_info = sam_wrapping.wrap_sams(normalized_sams)


        # Convert numpy arrays to Python lists
        polygons_as_lists = [polygon.tolist() if hasattr(polygon, 'tolist') else polygon for polygon in polygons]

        return {
            "success": True,
            "polygons": polygons_as_lists
        }
    except Exception as e:
        import traceback
        traceback.print_exc()
        return {
            "success": False,
            "error": str(e),
            "polygons": []
        }