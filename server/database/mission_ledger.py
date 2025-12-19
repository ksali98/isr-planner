"""
Mission Ledger Module

Provides functions to log mission runs and events to Supabase.
"""

import os
from datetime import datetime
from typing import Any, Dict, Optional
from uuid import uuid4

from .supabase_client import get_supabase_client


from typing import Any, Dict, Optional
from datetime import datetime
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
