"""
Mission Ledger Module

Provides functions for Decision Trace V1 + Learning V1 tables:
- agent_runs: One row per agentic solve
- agent_traces: Full decision trace JSONB blob
- agent_optimizer_steps: Normalized optimizer step history
- agent_policy_rules: Learned rules for enforcement
"""

from typing import Any, Dict, List, Optional
from .supabase_client import get_supabase_client


# =====================================================
# AGENT RUNS - Core run records
# =====================================================

def create_agent_run(
    agent_version: str = "v4",
    mode: str = "agentic",
    request_text: Optional[str] = None,
    objective: Optional[str] = None,
    hard_constraints: Optional[Dict[str, Any]] = None,
    soft_preferences: Optional[Dict[str, Any]] = None,
    env_version_id: Optional[str] = None,
    mission_id: Optional[str] = None,
    user_id: Optional[str] = None,
    solver_type: Optional[str] = None,
    solver_runtime_ms: Optional[int] = None,
    parent_run_id: Optional[str] = None,
    segment_index: Optional[int] = None,
) -> Optional[str]:
    """
    Create an agent run record at the START of a solve.
    Returns the run_id (UUID) for use in traces and optimizer steps.
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping create_agent_run")
        return None

    record = {
        "agent_version": agent_version,
        "mode": mode,
        "request_text": request_text,
        "objective": objective,
        "hard_constraints": hard_constraints or {},
        "soft_preferences": soft_preferences or {},
        "env_version_id": env_version_id,
        "mission_id": mission_id,
        "user_id": user_id,
        "solver_type": solver_type,
        "solver_runtime_ms": solver_runtime_ms,
        "parent_run_id": parent_run_id,
        "segment_index": segment_index,
    }

    try:
        result = client.table("agent_runs").insert(record).execute()
        run_id = result.data[0]["id"]
        print(f"[mission_ledger] Created agent_run: {run_id}")
        return run_id
    except Exception as e:
        print(f"[mission_ledger] Error creating agent_run: {e}")
        return None


def update_agent_run(
    run_id: str,
    total_points: Optional[float] = None,
    total_fuel_used: Optional[float] = None,
    runtime_ms: Optional[int] = None,
    is_valid: Optional[bool] = None,
    routes: Optional[Dict[str, Any]] = None,
    summary: Optional[Dict[str, Any]] = None,
    solver_type: Optional[str] = None,
    solver_runtime_ms: Optional[int] = None,
    # Distance matrix references (for reproducibility)
    env_hash: Optional[str] = None,
    routing_model_hash: Optional[str] = None,
    distance_matrix_id: Optional[str] = None,
) -> bool:
    """
    Update an agent run with results AFTER the solve completes.

    Includes optional distance matrix references for reproducibility:
    - env_hash: Hash of the environment geometry
    - routing_model_hash: Hash of the routing model configuration
    - distance_matrix_id: UUID reference to the cached matrix in distance_matrices table
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping update_agent_run")
        return False

    updates = {}
    if total_points is not None:
        updates["total_points"] = total_points
    if total_fuel_used is not None:
        updates["total_fuel_used"] = total_fuel_used
    if runtime_ms is not None:
        updates["runtime_ms"] = runtime_ms
    if is_valid is not None:
        updates["is_valid"] = is_valid
    if routes is not None:
        updates["routes"] = routes
    if summary is not None:
        updates["summary"] = summary
    if solver_type is not None:
        updates["solver_type"] = solver_type
    if solver_runtime_ms is not None:
        updates["solver_runtime_ms"] = solver_runtime_ms
    # Distance matrix references
    if env_hash is not None:
        updates["env_hash"] = env_hash
    if routing_model_hash is not None:
        updates["routing_model_hash"] = routing_model_hash
    if distance_matrix_id is not None:
        updates["distance_matrix_id"] = distance_matrix_id

    if not updates:
        return True  # Nothing to update

    try:
        client.table("agent_runs").update(updates).eq("id", run_id).execute()
        print(f"[mission_ledger] Updated agent_run: {run_id}")
        return True
    except Exception as e:
        print(f"[mission_ledger] Error updating agent_run: {e}")
        return False


def get_agent_run(run_id: str) -> Optional[Dict[str, Any]]:
    """Retrieve a single agent run by ID."""
    client = get_supabase_client()
    if client is None:
        return None

    try:
        result = client.table("agent_runs").select("*").eq("id", run_id).execute()
        if result.data:
            return result.data[0]
        return None
    except Exception as e:
        print(f"[mission_ledger] Error fetching agent_run: {e}")
        return None


# =====================================================
# AGENT TRACES - Full decision trace JSONB
# =====================================================

def create_agent_trace(
    agent_run_id: str,
    trace: Dict[str, Any],
) -> Optional[str]:
    """
    Create the full Decision Trace for a run.

    trace structure:
    {
        "env_hash": "abc123",
        "eligibility": { "1": { "eligible": ["T1","T2"], "excluded": [...] } },
        "allocation": { "algorithm": "greedy", "assignments": {...}, "rationale": {...} },
        "solver": { "type": "exact", "candidates": 100, "runtime_ms": 150 },
        "final_evidence": { "1": { "waypoints": [...], "fuel": 150.5, "points": 45 } }
    }
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping create_agent_trace")
        return None

    record = {
        "agent_run_id": agent_run_id,
        "trace": trace,
    }

    try:
        result = client.table("agent_traces").insert(record).execute()
        trace_id = result.data[0]["id"]
        print(f"[mission_ledger] Created agent_trace: {trace_id} for run {agent_run_id}")
        return trace_id
    except Exception as e:
        print(f"[mission_ledger] Error creating agent_trace: {e}")
        return None


def update_agent_trace(
    agent_run_id: str,
    trace: Dict[str, Any],
) -> bool:
    """Update or replace the trace for a run (upsert by agent_run_id)."""
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping update_agent_trace")
        return False

    try:
        # Use upsert with the unique constraint on agent_run_id
        record = {
            "agent_run_id": agent_run_id,
            "trace": trace,
        }
        client.table("agent_traces").upsert(record, on_conflict="agent_run_id").execute()
        print(f"[mission_ledger] Updated agent_trace for run {agent_run_id}")
        return True
    except Exception as e:
        print(f"[mission_ledger] Error updating agent_trace: {e}")
        return False


def get_agent_trace(agent_run_id: str) -> Optional[Dict[str, Any]]:
    """Retrieve the trace for a run."""
    client = get_supabase_client()
    if client is None:
        return None

    try:
        result = client.table("agent_traces").select("*").eq("agent_run_id", agent_run_id).execute()
        if result.data:
            return result.data[0]
        return None
    except Exception as e:
        print(f"[mission_ledger] Error fetching agent_trace: {e}")
        return None


# =====================================================
# AGENT OPTIMIZER STEPS - Normalized step history
# =====================================================

def log_optimizer_step(
    agent_run_id: str,
    step_index: int,
    operator: str,
    before_routes: Dict[str, Any],
    after_routes: Dict[str, Any],
    delta: Dict[str, Any],
    accepted: bool = True,
    notes: Optional[str] = None,
    validation_status: str = "unknown",
    validation_details: Optional[Dict[str, Any]] = None,
) -> Optional[str]:
    """
    Log a single optimizer step for analytics and debugging.

    delta structure: { "points": +5, "fuel": -10.5 }
    validation_status: 'pass' | 'fail' | 'unknown'
    validation_details: { "fuel_ok": true, "sam_ok": true, ... }
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping log_optimizer_step")
        return None

    record = {
        "agent_run_id": agent_run_id,
        "step_index": step_index,
        "operator": operator,
        "before_routes": before_routes,
        "after_routes": after_routes,
        "delta": delta,
        "accepted": accepted,
        "notes": notes,
        "validation_status": validation_status,
        "validation_details": validation_details or {},
    }

    try:
        result = client.table("agent_optimizer_steps").insert(record).execute()
        step_id = result.data[0]["id"]
        print(f"[mission_ledger] Logged optimizer step {step_index}: {operator} (accepted={accepted})")
        return step_id
    except Exception as e:
        print(f"[mission_ledger] Error logging optimizer_step: {e}")
        return None


def get_optimizer_steps(agent_run_id: str) -> List[Dict[str, Any]]:
    """Get all optimizer steps for a run, ordered by step_index."""
    client = get_supabase_client()
    if client is None:
        return []

    try:
        result = (
            client.table("agent_optimizer_steps")
            .select("*")
            .eq("agent_run_id", agent_run_id)
            .order("step_index")
            .execute()
        )
        return result.data or []
    except Exception as e:
        print(f"[mission_ledger] Error fetching optimizer_steps: {e}")
        return []


def get_optimizer_stats(operator: Optional[str] = None) -> Dict[str, Any]:
    """
    Get aggregate stats on optimizer performance.
    Returns counts of accepted/rejected steps, optionally filtered by operator.
    """
    client = get_supabase_client()
    if client is None:
        return {"total": 0, "accepted": 0, "rejected": 0}

    try:
        query = client.table("agent_optimizer_steps").select("accepted, operator")
        if operator:
            query = query.eq("operator", operator)

        result = query.execute()
        steps = result.data or []

        total = len(steps)
        accepted = sum(1 for s in steps if s.get("accepted"))
        rejected = total - accepted

        return {
            "total": total,
            "accepted": accepted,
            "rejected": rejected,
            "acceptance_rate": accepted / total if total > 0 else 0,
        }
    except Exception as e:
        print(f"[mission_ledger] Error fetching optimizer_stats: {e}")
        return {"total": 0, "accepted": 0, "rejected": 0}


# =====================================================
# AGENT POLICY RULES - Learning from corrections
# =====================================================

def create_policy_rule(
    category: str,
    rule: Dict[str, Any],
    scope: str = "global",
    mode: str = "agentic",
    title: Optional[str] = None,
    created_by: Optional[str] = None,
    notes: Optional[str] = None,
) -> Optional[str]:
    """
    Create a policy rule for enforcement on future runs.

    category: 'hard_constraint' | 'planning_policy' | 'optimizer_policy'
    scope: 'global' | 'scenario' | 'mission_type'
    mode: 'agentic' | 'manual' | 'all'
    rule: { "trigger": {...}, "action": {...} }

    Example rule:
    {
        "trigger": { "condition": "priority_3_targets_present" },
        "action": { "exclude_priority": 3 }
    }
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping create_policy_rule")
        return None

    record = {
        "category": category,
        "scope": scope,
        "mode": mode,
        "title": title,
        "rule": rule,
        "active": True,
        "created_by": created_by,
        "notes": notes,
    }

    try:
        result = client.table("agent_policy_rules").insert(record).execute()
        rule_id = result.data[0]["id"]
        print(f"[mission_ledger] Created policy_rule: {rule_id} category={category}")
        return rule_id
    except Exception as e:
        print(f"[mission_ledger] Error creating policy_rule: {e}")
        return None


def get_active_policy_rules(
    scope: Optional[str] = None,
    category: Optional[str] = None,
    mode: Optional[str] = None,
) -> List[Dict[str, Any]]:
    """
    Retrieve all active policy rules, optionally filtered.
    Called at the start of every v4 run to enforce learned constraints.
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, returning empty rules")
        return []

    try:
        query = client.table("agent_policy_rules").select("*").eq("active", True)
        if scope:
            query = query.eq("scope", scope)
        if category:
            query = query.eq("category", category)
        if mode:
            query = query.eq("mode", mode)

        result = query.execute()
        rules = result.data or []
        print(f"[mission_ledger] Loaded {len(rules)} active policy rules")
        return rules
    except Exception as e:
        print(f"[mission_ledger] Error fetching policy_rules: {e}")
        return []


def deactivate_policy_rule(rule_id: str) -> bool:
    """Deactivate a policy rule (soft delete)."""
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping deactivate_policy_rule")
        return False

    try:
        client.table("agent_policy_rules").update({"active": False}).eq("id", rule_id).execute()
        print(f"[mission_ledger] Deactivated policy_rule: {rule_id}")
        return True
    except Exception as e:
        print(f"[mission_ledger] Error deactivating policy_rule: {e}")
        return False


def update_policy_rule(
    rule_id: str,
    updates: Dict[str, Any],
) -> bool:
    """Update a policy rule (e.g., change title, notes, or rule content)."""
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping update_policy_rule")
        return False

    try:
        client.table("agent_policy_rules").update(updates).eq("id", rule_id).execute()
        print(f"[mission_ledger] Updated policy_rule: {rule_id}")
        return True
    except Exception as e:
        print(f"[mission_ledger] Error updating policy_rule: {e}")
        return False


# =====================================================
# LEGACY COMPATIBILITY STUBS
# These functions maintain backwards compatibility with
# older endpoints that use the legacy table names.
# They are no-ops that just log and return None/False.
# =====================================================

def create_mission_run(
    system_version: str = "v4",
    mission_name: str = "isr-run",
    objective_weights: Optional[Dict[str, Any]] = None,
    constraints: Optional[Dict[str, Any]] = None,
    notes: Optional[str] = None,
) -> Optional[str]:
    """Legacy stub - use create_agent_run instead."""
    print(f"[mission_ledger] LEGACY: create_mission_run called (no-op, use create_agent_run)")
    return None


def log_event(
    run_id: str,
    event_type: str,
    payload: Optional[Dict[str, Any]] = None,
) -> bool:
    """Legacy stub - events are now captured in agent_traces."""
    print(f"[mission_ledger] LEGACY: log_event called (no-op)")
    return False


def create_env_version(
    run_id: str,
    env_snapshot: Dict[str, Any],
    source: str = "human",
    reason: Optional[str] = None,
) -> Optional[str]:
    """Legacy stub - env snapshots are now in agent_traces.trace JSONB."""
    print(f"[mission_ledger] LEGACY: create_env_version called (no-op)")
    return None


def create_plan(
    run_id: str,
    env_version_id: Optional[str],
    status: str = "draft",
    starts_by_drone: Optional[Dict[str, Any]] = None,
    allocation: Optional[Dict[str, Any]] = None,
    trajectories: Optional[Dict[str, Any]] = None,
    metrics: Optional[Dict[str, Any]] = None,
    notes: Optional[str] = None,
) -> Optional[str]:
    """Legacy stub - plans are now in agent_runs.routes."""
    print(f"[mission_ledger] LEGACY: create_plan called (no-op)")
    return None


# =====================================================
# DISTANCE MATRIX CACHE - Hash functions and caching
# =====================================================

import hashlib
import json
import math


# =====================================================
# CANONICAL ENVIRONMENT UTILITIES
# =====================================================

def _sorted_dict(obj: Any) -> Any:
    """
    Recursively sort all dicts by key for deterministic JSON output.

    Handles nested dicts, lists, and primitives.
    Used by canonicalize_env() for consistent hashing.
    """
    if isinstance(obj, dict):
        return {k: _sorted_dict(v) for k, v in sorted(obj.items())}
    if isinstance(obj, (list, tuple)):
        return [_sorted_dict(x) for x in obj]
    return obj


def canonicalize_env(
    airports: List[Dict],
    targets: List[Dict],
    sams: List[Dict],
    drones: Optional[List[Dict]] = None,
    include_drones: bool = False,
) -> Dict[str, Any]:
    """
    Build a canonical, deterministic representation of environment geometry.

    This is a more comprehensive version that can optionally include
    drone configurations for scenarios where drone specs affect routing.

    Args:
        airports: List of airport dicts with x, y, name/id
        targets: List of target dicts with x, y, name/id
        sams: List of SAM dicts with x, y or pos, and range/radius
        drones: Optional list of drone dicts
        include_drones: Whether to include drone configs in the canonical form

    Returns:
        A deterministic dict suitable for hashing with json.dumps(sort_keys=True)
    """
    def get_sam_pos(s: Dict) -> tuple:
        """Extract (x, y) from SAM regardless of format."""
        if "x" in s and "y" in s:
            return (s["x"], s["y"])
        elif "pos" in s:
            pos = s["pos"]
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                return (pos[0], pos[1])
        return (0, 0)

    # Normalize airports: sorted by id/name, only position data
    normalized_airports = sorted(
        [
            {
                "id": a.get("id", a.get("name", f"A{i}")),
                "x": round(float(a.get("x", 0)), 4),
                "y": round(float(a.get("y", 0)), 4),
            }
            for i, a in enumerate(airports)
        ],
        key=lambda p: p["id"]
    )

    # Normalize targets: sorted by id/name, only position data
    normalized_targets = sorted(
        [
            {
                "id": t.get("id", t.get("name", f"T{i}")),
                "x": round(float(t.get("x", 0)), 4),
                "y": round(float(t.get("y", 0)), 4),
            }
            for i, t in enumerate(targets)
        ],
        key=lambda p: p["id"]
    )

    # Normalize SAMs: sorted by (x, y, range)
    normalized_sams = sorted(
        [
            {
                "x": round(float(get_sam_pos(s)[0]), 4),
                "y": round(float(get_sam_pos(s)[1]), 4),
                "range": round(float(s.get("range", s.get("radius", 0))), 4),
            }
            for s in sams
        ],
        key=lambda p: (p["x"], p["y"], p["range"])
    )

    canonical = {
        "airports": normalized_airports,
        "targets": normalized_targets,
        "sams": normalized_sams,
    }

    # Optionally include drone configurations (for future use)
    if include_drones and drones:
        normalized_drones = sorted(
            [
                {
                    "id": d.get("id", d.get("name", f"D{i}")),
                    "fuel_capacity": round(float(d.get("fuel_capacity", d.get("fuelCapacity", 0))), 2),
                    "speed": round(float(d.get("speed", 0)), 2),
                }
                for i, d in enumerate(drones)
            ],
            key=lambda d: d["id"]
        )
        canonical["drones"] = normalized_drones

    return _sorted_dict(canonical)


def compute_env_hash_v2(
    airports: List[Dict],
    targets: List[Dict],
    sams: List[Dict],
    drones: Optional[List[Dict]] = None,
    include_drones: bool = False,
) -> str:
    """
    Compute a deterministic SHA256 hash using canonicalize_env().

    This is the v2 hash function that uses the more comprehensive
    canonicalize_env() for normalization. Use this when you need
    to include drone configurations in the hash.

    Returns: SHA256 hash string (first 16 chars for brevity)
    """
    canonical = canonicalize_env(
        airports=airports,
        targets=targets,
        sams=sams,
        drones=drones,
        include_drones=include_drones,
    )
    json_str = json.dumps(canonical, sort_keys=True, separators=(",", ":"))
    hash_full = hashlib.sha256(json_str.encode()).hexdigest()
    return hash_full[:16]


def compute_env_hash(airports: List[Dict], targets: List[Dict], sams: List[Dict]) -> str:
    """
    Compute a deterministic SHA256 hash of environment geometry.

    Canonicalizes by:
    - Sorting airports by name/id
    - Sorting targets by name/id
    - Sorting SAMs by (pos, range)
    - Excluding derived fields (distances, paths)

    Result: identical env → identical hash.

    Returns: SHA256 hash string (first 16 chars for brevity)
    """
    def get_sam_pos(s: Dict) -> tuple:
        """Extract (x, y) from SAM regardless of format."""
        if "x" in s and "y" in s:
            return (s["x"], s["y"])
        elif "pos" in s:
            pos = s["pos"]
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                return (pos[0], pos[1])
        return (0, 0)

    # Normalize airports: sorted by id/name, only position data
    normalized_airports = sorted(
        [
            {
                "id": a.get("id", a.get("name", f"A{i}")),
                "x": round(float(a.get("x", 0)), 4),
                "y": round(float(a.get("y", 0)), 4),
            }
            for i, a in enumerate(airports)
        ],
        key=lambda p: p["id"]
    )

    # Normalize targets: sorted by id/name, only position data
    normalized_targets = sorted(
        [
            {
                "id": t.get("id", t.get("name", f"T{i}")),
                "x": round(float(t.get("x", 0)), 4),
                "y": round(float(t.get("y", 0)), 4),
            }
            for i, t in enumerate(targets)
        ],
        key=lambda p: p["id"]
    )

    # Normalize SAMs: sorted by (x, y, range)
    normalized_sams = sorted(
        [
            {
                "x": round(float(get_sam_pos(s)[0]), 4),
                "y": round(float(get_sam_pos(s)[1]), 4),
                "range": round(float(s.get("range", s.get("radius", 0))), 4),
            }
            for s in sams
        ],
        key=lambda p: (p["x"], p["y"], p["range"])
    )

    normalized = {
        "airports": normalized_airports,
        "targets": normalized_targets,
        "sams": normalized_sams,
    }

    json_str = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    hash_full = hashlib.sha256(json_str.encode()).hexdigest()
    return hash_full[:16]


def compute_routing_model_hash(routing_model: Dict[str, Any]) -> str:
    """
    Compute a deterministic SHA256 hash of routing model configuration.

    routing_model example:
    {
        "sam_mode": "hard_v1",
        "cost_model": "fuel_distance",
        "sam_buffer": 0,
        "path_resolution": 1.0,
        "planner_version": "2025.12.30"
    }

    This prevents mixing matrices computed under different assumptions.

    Returns: SHA256 hash string (first 16 chars for brevity)
    """
    # Normalize: sort keys, round floats
    normalized = {}
    for k in sorted(routing_model.keys()):
        v = routing_model[k]
        if isinstance(v, float):
            normalized[k] = round(v, 4)
        else:
            normalized[k] = v

    json_str = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    hash_full = hashlib.sha256(json_str.encode()).hexdigest()
    return hash_full[:16]


def get_default_routing_model() -> Dict[str, Any]:
    """
    Return the default routing model configuration.
    Update this when the routing algorithm changes.
    """
    return {
        "sam_mode": "hard_v1",
        "cost_model": "fuel_distance",
        "sam_buffer": 0,
        "path_resolution": 1.0,
        "planner_version": "2025.12.30",
    }


def matrix_to_flat(matrix: List[List[float]]) -> Dict[str, Any]:
    """
    Convert NxN matrix to flat array format for efficient storage.

    Returns: {"n": N, "flat": [...], "dtype": "float32"}
    """
    n = len(matrix)
    flat = []
    for row in matrix:
        flat.extend(row)
    return {
        "n": n,
        "flat": flat,
        "dtype": "float32",
    }


def flat_to_matrix(flat_data: Dict[str, Any]) -> List[List[float]]:
    """
    Convert flat array format back to NxN matrix.

    Input: {"n": N, "flat": [...]}
    Returns: NxN list of lists
    """
    n = flat_data["n"]
    flat = flat_data["flat"]
    matrix = []
    for i in range(n):
        row = flat[i * n : (i + 1) * n]
        matrix.append(row)
    return matrix


def build_node_index(labels: List[str]) -> Dict[str, int]:
    """
    Build stable node_index mapping from labels.

    Input: ["A1", "T1", "T2", "A2"]
    Returns: {"A1": 0, "T1": 1, "T2": 2, "A2": 3}
    """
    return {label: i for i, label in enumerate(labels)}


def get_cached_matrix(
    env_hash: str,
    routing_model_hash: str,
) -> Optional[Dict[str, Any]]:
    """
    Look up a cached distance matrix by env_hash and routing_model_hash.

    Returns:
        Dict with keys: id, matrix (NxN), node_index, metadata
        Or None if not found
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping get_cached_matrix")
        return None

    try:
        result = (
            client.table("distance_matrices")
            .select("id, matrix, node_index, metadata, sam_mode")
            .eq("env_hash", env_hash)
            .eq("routing_model_hash", routing_model_hash)
            .limit(1)
            .execute()
        )

        if result.data and len(result.data) > 0:
            row = result.data[0]
            print(f"[mission_ledger] Cache HIT for matrix (env={env_hash[:8]}..., routing={routing_model_hash[:8]}...)")

            # Convert flat to matrix if needed
            matrix_data = row["matrix"]
            if isinstance(matrix_data, dict) and "flat" in matrix_data:
                matrix = flat_to_matrix(matrix_data)
            else:
                matrix = matrix_data

            # Build labels from node_index
            node_index = row["node_index"]
            labels = [""] * len(node_index)
            for label, idx in node_index.items():
                labels[idx] = label

            metadata = row.get("metadata", {})

            return {
                "id": row["id"],
                "matrix": matrix,
                "node_index": node_index,
                "labels": labels,
                "excluded_targets": metadata.get("excluded_targets", []),
                "sam_mode": row["sam_mode"],
                "metadata": metadata,
            }

        print(f"[mission_ledger] Cache MISS for matrix (env={env_hash[:8]}..., routing={routing_model_hash[:8]}...)")
        return None

    except Exception as e:
        print(f"[mission_ledger] Error fetching cached matrix: {e}")
        return None


def cache_matrix(
    env_hash: str,
    routing_model_hash: str,
    sam_mode: str,
    matrix: List[List[float]],
    labels: List[str],
    excluded_targets: Optional[List[str]] = None,
    computation_ms: Optional[int] = None,
    num_sams: Optional[int] = None,
    use_flat: bool = True,
) -> Optional[str]:
    """
    Store a computed distance matrix in the cache.

    Args:
        env_hash: SHA256 of canonical environment
        routing_model_hash: SHA256 of routing model params
        sam_mode: "hard_v1", "risk_v2", etc.
        matrix: NxN distance matrix
        labels: Node labels in order ["A1", "T1", ...]
        excluded_targets: Targets inside SAM circles
        computation_ms: How long computation took
        num_sams: Number of SAMs in environment
        use_flat: Store as flat array (more efficient) vs 2D array

    Returns:
        The matrix ID (for use as distance_matrix_id) or None on failure
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping cache_matrix")
        return None

    node_index = build_node_index(labels)

    # Convert to flat format if requested
    if use_flat:
        matrix_data = matrix_to_flat(matrix)
    else:
        matrix_data = matrix

    metadata = {
        "excluded_targets": excluded_targets or [],
        "num_waypoints": len(labels),
        "num_sams": num_sams,
        "computation_ms": computation_ms,
    }

    record = {
        "env_hash": env_hash,
        "routing_model_hash": routing_model_hash,
        "sam_mode": sam_mode,
        "node_index": node_index,
        "matrix": matrix_data,
        "metadata": metadata,
    }

    try:
        # Use upsert to handle race conditions (unique on env_hash, routing_model_hash)
        result = (
            client.table("distance_matrices")
            .upsert(record, on_conflict="env_hash,routing_model_hash")
            .execute()
        )

        matrix_id = result.data[0]["id"]
        print(f"[mission_ledger] Cached matrix: {matrix_id} (env={env_hash[:8]}..., mode={sam_mode})")
        return matrix_id

    except Exception as e:
        print(f"[mission_ledger] Error caching matrix: {e}")
        return None


def get_matrix_by_id(matrix_id: str) -> Optional[Dict[str, Any]]:
    """
    Look up a distance matrix by its UUID (the distance_matrix_id).

    Returns:
        Dict with keys: matrix (NxN), node_index, labels, metadata, env_hash, routing_model_hash
        Or None if not found
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping get_matrix_by_id")
        return None

    try:
        result = (
            client.table("distance_matrices")
            .select("matrix, node_index, metadata, sam_mode, env_hash, routing_model_hash")
            .eq("id", matrix_id)
            .limit(1)
            .execute()
        )

        if result.data and len(result.data) > 0:
            row = result.data[0]
            print(f"[mission_ledger] Loaded matrix by id: {matrix_id}")

            # Convert flat to matrix if needed
            matrix_data = row["matrix"]
            if isinstance(matrix_data, dict) and "flat" in matrix_data:
                matrix = flat_to_matrix(matrix_data)
            else:
                matrix = matrix_data

            # Build labels from node_index
            node_index = row["node_index"]
            labels = [""] * len(node_index)
            for label, idx in node_index.items():
                labels[idx] = label

            metadata = row.get("metadata", {})

            return {
                "matrix": matrix,
                "node_index": node_index,
                "labels": labels,
                "excluded_targets": metadata.get("excluded_targets", []),
                "sam_mode": row["sam_mode"],
                "env_hash": row["env_hash"],
                "routing_model_hash": row["routing_model_hash"],
                "metadata": metadata,
            }

        print(f"[mission_ledger] Matrix not found for id: {matrix_id}")
        return None

    except Exception as e:
        print(f"[mission_ledger] Error fetching matrix by id: {e}")
        return None


# =====================================================
# SAM PATHS CACHE (optional - for visualization)
# =====================================================

def cache_sam_paths(
    env_hash: str,
    routing_model_hash: str,
    sam_mode: str,
    paths: Dict[str, List[List[float]]],
    metadata: Optional[Dict[str, Any]] = None,
) -> Optional[str]:
    """
    Store SAM-avoiding polylines for visualization.

    Args:
        env_hash: SHA256 of canonical environment
        routing_model_hash: SHA256 of routing model params
        sam_mode: "hard_v1", "risk_v2", etc.
        paths: {"A1->T3": [[x,y], ...], ...}
        metadata: Optional additional info

    Returns:
        The path record ID or None on failure
    """
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping cache_sam_paths")
        return None

    record = {
        "env_hash": env_hash,
        "routing_model_hash": routing_model_hash,
        "sam_mode": sam_mode,
        "paths": paths,
        "metadata": metadata or {},
    }

    try:
        result = (
            client.table("sam_paths")
            .upsert(record, on_conflict="env_hash,routing_model_hash")
            .execute()
        )

        path_id = result.data[0]["id"]
        print(f"[mission_ledger] Cached SAM paths: {path_id} (env={env_hash[:8]}...)")
        return path_id

    except Exception as e:
        print(f"[mission_ledger] Error caching SAM paths: {e}")
        return None


def get_cached_sam_paths(
    env_hash: str,
    routing_model_hash: str,
) -> Optional[Dict[str, List[List[float]]]]:
    """
    Look up cached SAM-avoiding paths.

    Returns:
        Dict of paths {"A1->T3": [[x,y], ...], ...} or None if not found
    """
    client = get_supabase_client()
    if client is None:
        return None

    try:
        result = (
            client.table("sam_paths")
            .select("paths")
            .eq("env_hash", env_hash)
            .eq("routing_model_hash", routing_model_hash)
            .limit(1)
            .execute()
        )

        if result.data and len(result.data) > 0:
            print(f"[mission_ledger] SAM paths cache HIT (env={env_hash[:8]}...)")
            return result.data[0]["paths"]

        return None

    except Exception as e:
        print(f"[mission_ledger] Error fetching SAM paths: {e}")
        return None
