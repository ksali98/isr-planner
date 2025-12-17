from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple
import hashlib
import json


def _stable_hash(obj: Any) -> str:
    s = json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(s.encode("utf-8")).hexdigest()[:16]


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def build_hashes(env: Dict[str, Any], drone_configs: Dict[str, Any]) -> Tuple[str, str]:
    env_hash = _stable_hash(
        {
            "airports": env.get("airports", []),
            "targets": env.get("targets", []),
            "sams": env.get("sams", []),
        }
    )
    cfg_hash = _stable_hash(drone_configs or {})
    return env_hash, cfg_hash


@dataclass
class MissionRecord:
    mission_id: str
    env: Dict[str, Any]
    drone_configs: Dict[str, Any]
    env_hash: str
    cfg_hash: str
    routes: Dict[str, Any]
    allocation: Dict[str, Any]
    mission_metrics: Dict[str, Any]
    attempts: List[Dict[str, Any]]
    updated_at: str


# In-memory store (RAM only). Resets on Railway restart / redeploy.
MISSION_STORE: Dict[str, MissionRecord] = {}
