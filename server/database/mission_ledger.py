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
) -> bool:
    """
    Update an agent run with results AFTER the solve completes.
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
