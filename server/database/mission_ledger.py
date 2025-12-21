"""
Mission Ledger Module

Provides functions to log mission runs and events to Supabase.
"""

from typing import Any, Dict, Optional
from .supabase_client import get_supabase_client

def create_mission_run(
    system_version: str = "v4",
    mission_name: str = "isr-run",
    objective_weights: Optional[Dict[str, Any]] = None,
    constraints: Optional[Dict[str, Any]] = None,
    notes: Optional[str] = None,
) -> Optional[str]: 
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping create_mission_run")
        return None

    record = {
        "system_version": system_version,
        "mission_name": mission_name,
        "objective_weights": objective_weights or {},
        "constraints": constraints or {},
        "notes": notes,
    }

    try:
        result = client.table("mission_runs").insert(record).execute()
        run_id = result.data[0]["id"]  # <-- IMPORTANT: use 'id' from DB
        print(f"[mission_ledger] Created mission run: {run_id}")
        return run_id
    except Exception as e:
        print(f"[mission_ledger] Error creating mission run: {e}")
        return None

def log_event(
    run_id: str,
    event_type: str,
    payload: Optional[Dict[str, Any]] = None,
) -> bool:
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping log_event")
        return False

    record = {
        "run_id": run_id,                 # must be mission_runs.id UUID
        "event_type": event_type,
        "payload": payload or {},
    }

    try:
        client.table("mission_events").insert(record).execute()
        print(f"[mission_ledger] Logged event: {event_type} for run {run_id}")
        return True
    except Exception as e:
        print(f"[mission_ledger] Error logging event: {e}")
        return False

def create_env_version(
    run_id: str,
    env_snapshot: Dict[str, Any],
    source: str = "human",
    reason: Optional[str] = None,
) -> Optional[str]:
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping create_env_version")
        return None

    record = {
        "run_id": run_id,
        "source": source,
        "reason": reason,
        "env_snapshot": env_snapshot,
    }

    try:
        result = client.table("env_versions").insert(record).execute()
        env_version_id = result.data[0]["id"]
        print(f"[mission_ledger] Created env_version: {env_version_id} for run {run_id}")
        return env_version_id
    except Exception as e:
        print(f"[mission_ledger] Error creating env_version: {e}")
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
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping create_plan")
        return None

    record = {
        "run_id": run_id,
        "env_version_id": env_version_id,
        "status": status,
        "starts_by_drone": starts_by_drone or {},
        "allocation": allocation or {},
        "trajectories": trajectories or {},
        "metrics": metrics or {},
        "notes": notes,
    }

    try:
        result = client.table("plans").insert(record).execute()
        plan_id = result.data[0]["id"]
        print(f"[mission_ledger] Created plan: {plan_id} status={status} run={run_id}")
        return plan_id
    except Exception as e:
        print(f"[mission_ledger] Error creating plan: {e}")
        return None
def set_plan_status(plan_id: str, status: str) -> bool:
    client = get_supabase_client()
    if client is None:
        print("[mission_ledger] Supabase not configured, skipping set_plan_status")
        return False
    try:
        client.table("plans").update({"status": status}).eq("id", plan_id).execute()
        print(f"[mission_ledger] Updated plan {plan_id} -> {status}")
        return True
    except Exception as e:
        print(f"[mission_ledger] Error updating plan status: {e}")
        return False
