"""
Session Store Abstraction Layer

Provides a unified interface for session state management, wrapping
the existing session_storage.py for Supabase operations.

Two implementations:
- InMemorySessionStore: Dev/fallback with TTL-based eviction
- SupabaseSessionStore: Production implementation using Supabase

The store manages:
- Session lifecycle (create, get, delete)
- Chat history (append, retrieve)
- Mission state snapshots (thin snapshots by default)
- Pending clarification state
- Revision-based optimistic concurrency
"""

import json
import time
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional, Protocol, runtime_checkable
from uuid import uuid4


# =============================================================================
# Exceptions
# =============================================================================

class ConcurrencyError(Exception):
    """Raised when optimistic concurrency check fails."""
    def __init__(self, session_id: str, expected_rev: int, current_rev: int):
        self.session_id = session_id
        self.expected_rev = expected_rev
        self.current_rev = current_rev
        super().__init__(
            f"Concurrency conflict for session {session_id}: "
            f"expected rev {expected_rev}, current rev {current_rev}"
        )


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class SessionData:
    """Internal representation of a session."""
    id: str
    rev: int = 0
    created_at: datetime = field(default_factory=datetime.utcnow)
    updated_at: datetime = field(default_factory=datetime.utcnow)
    metadata: Dict[str, Any] = field(default_factory=dict)
    pending_clarification: Optional[Dict[str, Any]] = None
    last_snapshot_id: Optional[str] = None
    messages: List[Dict[str, Any]] = field(default_factory=list)


@dataclass
class Snapshot:
    """A mission state snapshot (thin by default)."""
    id: str
    session_id: str
    created_at: datetime
    kind: str  # "thin" or "thick"

    # Core state (always present)
    environment: Optional[Dict[str, Any]] = None
    drone_configs: Optional[Dict[str, Any]] = None
    solution: Optional[Dict[str, Any]] = None
    metrics: Optional[Dict[str, Any]] = None

    # Constraint program
    constraint_program: Optional[Dict[str, Any]] = None
    compiled_patches: Optional[Dict[str, Any]] = None

    # Hashes for cache lookup
    env_hash: Optional[str] = None
    solution_hash: Optional[str] = None


# =============================================================================
# Protocol Definition
# =============================================================================

@runtime_checkable
class SessionStore(Protocol):
    """
    Abstract interface for session state management.

    Implementations must provide all methods. The interface is designed
    to work with both in-memory (dev) and Supabase (prod) backends.
    """

    def create_session(self, *, metadata: Optional[Dict[str, Any]] = None) -> str:
        """Create a new session. Returns session_id."""
        ...

    def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get session data by ID. Returns None if not found."""
        ...

    def delete_session(self, session_id: str) -> bool:
        """Delete a session and all associated data."""
        ...

    def append_message(
        self,
        session_id: str,
        role: str,
        content: str,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Optional[int]:
        """Append a message to conversation history. Returns message ID."""
        ...

    def get_history(
        self,
        session_id: str,
        limit: int = 50,
    ) -> List[Dict[str, Any]]:
        """Get conversation history (oldest first)."""
        ...

    def save_snapshot(
        self,
        session_id: str,
        snapshot: Dict[str, Any],
        kind: str = "thin",
    ) -> Optional[str]:
        """Save a mission state snapshot. Returns snapshot_id."""
        ...

    def get_latest_snapshot(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get the most recent snapshot for a session."""
        ...

    def set_pending_clarification(
        self,
        session_id: str,
        pending: Optional[Dict[str, Any]],
    ) -> bool:
        """Set or clear pending clarification state."""
        ...

    def get_pending_clarification(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get pending clarification state, if any."""
        ...

    def bump_rev(
        self,
        session_id: str,
        expected_rev: Optional[int] = None,
    ) -> int:
        """
        Increment session revision for optimistic concurrency.

        If expected_rev is provided and doesn't match current rev,
        raises ConcurrencyError.

        Returns the new revision number.
        """
        ...


# =============================================================================
# In-Memory Implementation (Dev/Fallback)
# =============================================================================

class InMemorySessionStore:
    """
    In-memory session store with TTL-based eviction.

    Used for development and as fallback when Supabase is unavailable.

    Configuration:
    - SESSION_TTL_SECONDS: How long sessions live (default 6 hours)
    - MAX_SESSIONS: Maximum number of sessions to keep (default 500)
    """

    SESSION_TTL_SECONDS = 6 * 60 * 60  # 6 hours
    MAX_SESSIONS = 500

    def __init__(self):
        self._sessions: Dict[str, SessionData] = {}
        self._snapshots: Dict[str, Snapshot] = {}  # snapshot_id -> Snapshot
        self._session_snapshots: Dict[str, List[str]] = {}  # session_id -> [snapshot_ids]
        self._lock = threading.RLock()
        self._last_prune = time.time()

    def _prune_if_needed(self) -> None:
        """Prune expired sessions and enforce size cap."""
        now = time.time()

        # Don't prune more than once per minute
        if now - self._last_prune < 60:
            return

        self._last_prune = now

        with self._lock:
            # Remove expired sessions
            cutoff = datetime.utcnow().timestamp() - self.SESSION_TTL_SECONDS
            expired = [
                sid for sid, session in self._sessions.items()
                if session.updated_at.timestamp() < cutoff
            ]
            for sid in expired:
                self._delete_session_internal(sid)

            # Enforce size cap (remove oldest first)
            if len(self._sessions) > self.MAX_SESSIONS:
                sorted_sessions = sorted(
                    self._sessions.items(),
                    key=lambda x: x[1].updated_at,
                )
                to_remove = len(self._sessions) - self.MAX_SESSIONS
                for sid, _ in sorted_sessions[:to_remove]:
                    self._delete_session_internal(sid)

    def _delete_session_internal(self, session_id: str) -> None:
        """Internal delete without lock (caller must hold lock)."""
        if session_id in self._sessions:
            del self._sessions[session_id]

        # Clean up snapshots
        if session_id in self._session_snapshots:
            for snap_id in self._session_snapshots[session_id]:
                if snap_id in self._snapshots:
                    del self._snapshots[snap_id]
            del self._session_snapshots[session_id]

    def create_session(self, *, metadata: Optional[Dict[str, Any]] = None) -> str:
        self._prune_if_needed()

        session_id = str(uuid4())
        with self._lock:
            self._sessions[session_id] = SessionData(
                id=session_id,
                metadata=metadata or {},
            )
            self._session_snapshots[session_id] = []

        return session_id

    def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None

            return {
                "id": session.id,
                "rev": session.rev,
                "created_at": session.created_at.isoformat(),
                "updated_at": session.updated_at.isoformat(),
                "metadata": session.metadata,
                "pending_clarification": session.pending_clarification,
                "last_snapshot_id": session.last_snapshot_id,
            }

    def delete_session(self, session_id: str) -> bool:
        with self._lock:
            if session_id not in self._sessions:
                return False
            self._delete_session_internal(session_id)
            return True

    def append_message(
        self,
        session_id: str,
        role: str,
        content: str,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Optional[int]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None

            msg_id = len(session.messages) + 1
            session.messages.append({
                "id": msg_id,
                "role": role,
                "content": content,
                "timestamp": datetime.utcnow().isoformat(),
                "metadata": metadata or {},
            })
            session.updated_at = datetime.utcnow()
            return msg_id

    def get_history(
        self,
        session_id: str,
        limit: int = 50,
    ) -> List[Dict[str, Any]]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return []

            # Return most recent messages, oldest first
            messages = session.messages[-limit:] if limit else session.messages
            return list(messages)

    def save_snapshot(
        self,
        session_id: str,
        snapshot: Dict[str, Any],
        kind: str = "thin",
    ) -> Optional[str]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None

            snapshot_id = str(uuid4())
            snap = Snapshot(
                id=snapshot_id,
                session_id=session_id,
                created_at=datetime.utcnow(),
                kind=kind,
                environment=snapshot.get("environment"),
                drone_configs=snapshot.get("drone_configs"),
                solution=snapshot.get("solution"),
                metrics=snapshot.get("metrics"),
                constraint_program=snapshot.get("constraint_program"),
                compiled_patches=snapshot.get("compiled_patches"),
                env_hash=snapshot.get("env_hash"),
                solution_hash=snapshot.get("solution_hash"),
            )

            self._snapshots[snapshot_id] = snap
            self._session_snapshots[session_id].append(snapshot_id)
            session.last_snapshot_id = snapshot_id
            session.updated_at = datetime.utcnow()

            return snapshot_id

    def get_latest_snapshot(self, session_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None or session.last_snapshot_id is None:
                return None

            snap = self._snapshots.get(session.last_snapshot_id)
            if snap is None:
                return None

            return {
                "id": snap.id,
                "session_id": snap.session_id,
                "created_at": snap.created_at.isoformat(),
                "kind": snap.kind,
                "environment": snap.environment,
                "drone_configs": snap.drone_configs,
                "solution": snap.solution,
                "metrics": snap.metrics,
                "constraint_program": snap.constraint_program,
                "compiled_patches": snap.compiled_patches,
                "env_hash": snap.env_hash,
                "solution_hash": snap.solution_hash,
            }

    def set_pending_clarification(
        self,
        session_id: str,
        pending: Optional[Dict[str, Any]],
    ) -> bool:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return False

            session.pending_clarification = pending
            session.updated_at = datetime.utcnow()
            return True

    def get_pending_clarification(self, session_id: str) -> Optional[Dict[str, Any]]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return None
            return session.pending_clarification

    def bump_rev(
        self,
        session_id: str,
        expected_rev: Optional[int] = None,
    ) -> int:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                raise ValueError(f"Session not found: {session_id}")

            if expected_rev is not None and session.rev != expected_rev:
                raise ConcurrencyError(session_id, expected_rev, session.rev)

            session.rev += 1
            session.updated_at = datetime.utcnow()
            return session.rev


# =============================================================================
# Supabase Implementation (Production)
# =============================================================================

class SupabaseSessionStore:
    """
    Supabase-backed session store.

    Wraps existing session_storage.py functions and adds:
    - Snapshot storage (using mission_plans table with kind field)
    - Pending clarification (stored in session metadata)
    - Revision-based concurrency (stored in session metadata)

    Note: Uses existing metadata jsonb column for new fields to avoid
    schema changes. Can be migrated to dedicated columns later.
    """

    def __init__(self):
        # Import lazily to avoid circular imports
        from server.database import session_storage
        self._storage = session_storage

    def _is_configured(self) -> bool:
        """Check if Supabase is available."""
        from server.database.supabase_client import is_supabase_configured
        return is_supabase_configured()

    def create_session(self, *, metadata: Optional[Dict[str, Any]] = None) -> str:
        if not self._is_configured():
            raise RuntimeError("Supabase not configured")

        # Initialize metadata with rev=0
        init_metadata = {
            **(metadata or {}),
            "_rev": 0,
            "_pending_clarification": None,
            "_last_snapshot_id": None,
        }

        session_id = self._storage.create_session(metadata=init_metadata)
        if session_id is None:
            raise RuntimeError("Failed to create session in Supabase")

        return session_id

    def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        if not self._is_configured():
            return None

        session = self._storage.get_session(session_id)
        if session is None:
            return None

        metadata = session.get("metadata", {})
        return {
            "id": session["id"],
            "rev": metadata.get("_rev", 0),
            "created_at": session.get("created_at"),
            "updated_at": session.get("updated_at"),
            "metadata": {k: v for k, v in metadata.items() if not k.startswith("_")},
            "pending_clarification": metadata.get("_pending_clarification"),
            "last_snapshot_id": metadata.get("_last_snapshot_id"),
        }

    def delete_session(self, session_id: str) -> bool:
        if not self._is_configured():
            return False
        return self._storage.delete_session(session_id)

    def append_message(
        self,
        session_id: str,
        role: str,
        content: str,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Optional[int]:
        if not self._is_configured():
            return None
        return self._storage.add_conversation_message(
            session_id=session_id,
            role=role,
            content=content,
            metadata=metadata,
        )

    def get_history(
        self,
        session_id: str,
        limit: int = 50,
    ) -> List[Dict[str, Any]]:
        if not self._is_configured():
            return []
        return self._storage.get_conversation_history(
            session_id=session_id,
            limit=limit,
        )

    def save_snapshot(
        self,
        session_id: str,
        snapshot: Dict[str, Any],
        kind: str = "thin",
    ) -> Optional[str]:
        """
        Save a snapshot using mission_plans table.

        Stores the snapshot in plan_data with additional metadata.
        """
        if not self._is_configured():
            return None

        # Build plan_data with snapshot fields
        plan_data = {
            "kind": kind,
            "environment": snapshot.get("environment"),
            "drone_configs": snapshot.get("drone_configs"),
            "solution": snapshot.get("solution"),
            "metrics": snapshot.get("metrics"),
            "constraint_program": snapshot.get("constraint_program"),
            "compiled_patches": snapshot.get("compiled_patches"),
            "env_hash": snapshot.get("env_hash"),
            "solution_hash": snapshot.get("solution_hash"),
        }

        # Calculate summary values
        solution = snapshot.get("solution", {})
        total_fuel = solution.get("total_fuel", 0.0)
        total_points = solution.get("total_points", 0)
        drone_count = len(solution.get("routes", {}))

        snapshot_id = self._storage.save_mission_plan(
            session_id=session_id,
            plan_data=plan_data,
            total_fuel=total_fuel,
            total_points=total_points,
            drone_count=drone_count,
        )

        if snapshot_id is not None:
            # Update session metadata with last_snapshot_id
            session = self._storage.get_session(session_id)
            if session:
                metadata = session.get("metadata", {})
                metadata["_last_snapshot_id"] = str(snapshot_id)
                self._storage.update_session_metadata(session_id, metadata)

        return str(snapshot_id) if snapshot_id else None

    def get_latest_snapshot(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get most recent snapshot for a session."""
        if not self._is_configured():
            return None

        plans = self._storage.get_mission_plans(session_id=session_id, limit=1)
        if not plans:
            return None

        plan = plans[0]
        plan_data = plan.get("plan_data", {})

        return {
            "id": str(plan["id"]),
            "session_id": session_id,
            "created_at": plan.get("created_at"),
            "kind": plan_data.get("kind", "thin"),
            "environment": plan_data.get("environment"),
            "drone_configs": plan_data.get("drone_configs"),
            "solution": plan_data.get("solution"),
            "metrics": plan_data.get("metrics"),
            "constraint_program": plan_data.get("constraint_program"),
            "compiled_patches": plan_data.get("compiled_patches"),
            "env_hash": plan_data.get("env_hash"),
            "solution_hash": plan_data.get("solution_hash"),
        }

    def set_pending_clarification(
        self,
        session_id: str,
        pending: Optional[Dict[str, Any]],
    ) -> bool:
        if not self._is_configured():
            return False

        session = self._storage.get_session(session_id)
        if not session:
            return False

        metadata = session.get("metadata", {})
        metadata["_pending_clarification"] = pending
        return self._storage.update_session_metadata(session_id, metadata)

    def get_pending_clarification(self, session_id: str) -> Optional[Dict[str, Any]]:
        if not self._is_configured():
            return None

        session = self._storage.get_session(session_id)
        if not session:
            return None

        metadata = session.get("metadata", {})
        return metadata.get("_pending_clarification")

    def bump_rev(
        self,
        session_id: str,
        expected_rev: Optional[int] = None,
    ) -> int:
        """
        Atomically increment session revision.

        Note: This implementation uses read-modify-write which is not truly atomic.
        For production with multiple workers, consider using a Postgres function
        or advisory locks.
        """
        if not self._is_configured():
            raise RuntimeError("Supabase not configured")

        session = self._storage.get_session(session_id)
        if not session:
            raise ValueError(f"Session not found: {session_id}")

        metadata = session.get("metadata", {})
        current_rev = metadata.get("_rev", 0)

        if expected_rev is not None and current_rev != expected_rev:
            raise ConcurrencyError(session_id, expected_rev, current_rev)

        new_rev = current_rev + 1
        metadata["_rev"] = new_rev

        if not self._storage.update_session_metadata(session_id, metadata):
            raise RuntimeError(f"Failed to update session rev: {session_id}")

        return new_rev


# =============================================================================
# Factory Function
# =============================================================================

# Singleton instance
_store_instance: Optional[SessionStore] = None
_store_lock = threading.Lock()


def get_session_store() -> SessionStore:
    """
    Get the session store instance.

    Returns SupabaseSessionStore if Supabase is configured,
    otherwise returns InMemorySessionStore.

    The instance is cached for the lifetime of the process.
    """
    global _store_instance

    if _store_instance is not None:
        return _store_instance

    with _store_lock:
        # Double-check after acquiring lock
        if _store_instance is not None:
            return _store_instance

        # Check if Supabase is configured
        try:
            from server.database.supabase_client import is_supabase_configured
            if is_supabase_configured():
                print("[session_store] Using SupabaseSessionStore")
                _store_instance = SupabaseSessionStore()
            else:
                print("[session_store] Supabase not configured, using InMemorySessionStore")
                _store_instance = InMemorySessionStore()
        except Exception as e:
            print(f"[session_store] Error checking Supabase config: {e}, using InMemorySessionStore")
            _store_instance = InMemorySessionStore()

        return _store_instance


def reset_session_store() -> None:
    """
    Reset the session store singleton.

    Useful for testing or when configuration changes.
    """
    global _store_instance
    with _store_lock:
        _store_instance = None
