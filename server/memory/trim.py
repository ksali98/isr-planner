"""
Snapshot Trimming Utilities

Provides functions to keep snapshots within size limits by progressively
removing optional fields. This prevents database failures from oversized
JSON blobs while preserving the most critical mission state.

Size limits:
- SNAPSHOT_MAX_BYTES: 512KB (hard cap)
- SNAPSHOT_WARN_BYTES: 256KB (triggers trimming)

Trimming order (least to most critical):
1. Per-edge geometry/polylines
2. Trace details (keep last 200 events)
3. Long message history (keep last 20)
4. Distance matrix (if present - should not be stored)
5. Detailed route waypoints (keep summary only)
"""

import json
from typing import Any, Dict, List, Optional, Tuple

# Size limits in bytes
SNAPSHOT_MAX_BYTES = 512 * 1024  # 512 KB
SNAPSHOT_WARN_BYTES = 256 * 1024  # 256 KB
MAX_TRACE_EVENTS = 200
MAX_INLINE_MESSAGES = 20


def approx_size_bytes(obj: Any) -> int:
    """
    Approximate the JSON-serialized size of an object in bytes.

    Uses json.dumps with compact separators for accurate estimation.
    Returns 0 if object is not serializable.
    """
    if obj is None:
        return 4  # "null"

    try:
        return len(json.dumps(obj, separators=(",", ":")))
    except (TypeError, ValueError):
        return 0


def _trim_trace_events(trace: Dict[str, Any], max_events: int = MAX_TRACE_EVENTS) -> Dict[str, Any]:
    """Trim trace events to the most recent N."""
    if "trace_events" not in trace:
        return trace

    events = trace.get("trace_events", [])
    if len(events) <= max_events:
        return trace

    trimmed = trace.copy()
    trimmed["trace_events"] = events[-max_events:]
    trimmed["_trace_truncated"] = True
    trimmed["_trace_original_count"] = len(events)
    return trimmed


def _trim_messages(snapshot: Dict[str, Any], max_messages: int = MAX_INLINE_MESSAGES) -> Dict[str, Any]:
    """Trim inline message history to the most recent N."""
    if "messages" not in snapshot:
        return snapshot

    messages = snapshot.get("messages", [])
    if len(messages) <= max_messages:
        return snapshot

    trimmed = snapshot.copy()
    trimmed["messages"] = messages[-max_messages:]
    trimmed["_messages_truncated"] = True
    trimmed["_messages_original_count"] = len(messages)
    return trimmed


def _remove_edge_geometry(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Remove per-edge polylines/geometry from solution."""
    solution = snapshot.get("solution")
    if not solution:
        return snapshot

    trimmed = snapshot.copy()
    trimmed_solution = solution.copy()

    # Remove sam_paths if present
    if "sam_paths" in trimmed_solution:
        del trimmed_solution["sam_paths"]
        trimmed["_sam_paths_removed"] = True

    # Remove polylines from routes
    routes = trimmed_solution.get("routes", {})
    if routes:
        trimmed_routes = {}
        for drone_id, route_data in routes.items():
            if isinstance(route_data, dict):
                route_copy = route_data.copy()
                # Remove geometry fields
                for field in ["polylines", "edge_paths", "geometry", "waypoint_coords"]:
                    if field in route_copy:
                        del route_copy[field]
                trimmed_routes[drone_id] = route_copy
            else:
                trimmed_routes[drone_id] = route_data
        trimmed_solution["routes"] = trimmed_routes

    trimmed["solution"] = trimmed_solution
    return trimmed


def _remove_distance_matrix(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Remove distance matrix (should be cached separately by hash)."""
    if "distance_matrix" not in snapshot:
        return snapshot

    trimmed = snapshot.copy()
    del trimmed["distance_matrix"]
    trimmed["_distance_matrix_removed"] = True
    return trimmed


def _remove_trace_details(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Remove detailed trace, keeping only summary."""
    trace = snapshot.get("trace")
    if not trace:
        return snapshot

    trimmed = snapshot.copy()

    # Keep only essential trace summary
    summary = {
        "intent": trace.get("intent"),
        "confidence": trace.get("confidence"),
        "rules_hit": trace.get("rules_hit"),
        "_full_trace_removed": True,
    }

    trimmed["trace"] = summary
    return trimmed


def _summarize_routes(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Replace full routes with summary (last resort)."""
    solution = snapshot.get("solution")
    if not solution:
        return snapshot

    routes = solution.get("routes", {})
    if not routes:
        return snapshot

    trimmed = snapshot.copy()
    trimmed_solution = solution.copy()

    summary_routes = {}
    for drone_id, route_data in routes.items():
        if isinstance(route_data, dict):
            # Keep only essential fields
            summary_routes[drone_id] = {
                "route": route_data.get("route", []),
                "fuel_used": route_data.get("fuel_used", route_data.get("distance")),
                "total_points": route_data.get("total_points"),
                "feasible": route_data.get("feasible"),
            }
        else:
            summary_routes[drone_id] = route_data

    trimmed_solution["routes"] = summary_routes
    trimmed_solution["_routes_summarized"] = True
    trimmed["solution"] = trimmed_solution
    return trimmed


def trim_snapshot(
    snapshot: Dict[str, Any],
    max_bytes: int = SNAPSHOT_MAX_BYTES,
    warn_bytes: int = SNAPSHOT_WARN_BYTES,
) -> Tuple[Dict[str, Any], List[str]]:
    """
    Trim a snapshot to fit within size limits.

    Applies trimming operations in order of importance (least critical first).
    Returns the trimmed snapshot and a list of warnings about what was removed.

    Args:
        snapshot: The snapshot to trim
        max_bytes: Hard size limit (default 512KB)
        warn_bytes: Size threshold to start trimming (default 256KB)

    Returns:
        Tuple of (trimmed_snapshot, warnings)
    """
    warnings: List[str] = []
    result = snapshot.copy()

    # Check initial size
    initial_size = approx_size_bytes(result)

    # If under warn threshold, return as-is
    if initial_size <= warn_bytes:
        return result, warnings

    warnings.append(f"Snapshot size {initial_size} bytes exceeds warn threshold {warn_bytes}")

    # Trimming operations in order of priority
    trimming_ops = [
        ("edge_geometry", _remove_edge_geometry),
        ("distance_matrix", _remove_distance_matrix),
        ("trace_events", lambda s: _trim_trace_events(s.get("trace", {})) if "trace" in s else s),
        ("messages", _trim_messages),
        ("trace_details", _remove_trace_details),
        ("route_details", _summarize_routes),
    ]

    for op_name, op_func in trimming_ops:
        current_size = approx_size_bytes(result)

        if current_size <= warn_bytes:
            break

        # Apply trimming operation
        if op_name == "trace_events" and "trace" in result:
            result["trace"] = op_func(result)
        else:
            result = op_func(result)

        new_size = approx_size_bytes(result)
        if new_size < current_size:
            warnings.append(f"Trimmed {op_name}: {current_size} -> {new_size} bytes")

    # Final size check
    final_size = approx_size_bytes(result)

    if final_size > max_bytes:
        # Emergency: keep only the absolute essentials
        warnings.append(f"EMERGENCY TRIM: size {final_size} still exceeds max {max_bytes}")

        essential = {
            "env_hash": result.get("env_hash"),
            "solution_hash": result.get("solution_hash"),
            "metrics": result.get("metrics"),
            "constraint_program": result.get("constraint_program"),
            "_emergency_trimmed": True,
            "_original_size": initial_size,
        }

        # Try to keep minimal solution
        solution = result.get("solution", {})
        if solution:
            essential["solution"] = {
                "total_points": solution.get("total_points"),
                "total_fuel": solution.get("total_fuel"),
                "routes": {
                    did: {"route": r.get("route", []) if isinstance(r, dict) else r}
                    for did, r in solution.get("routes", {}).items()
                },
            }

        result = essential
        warnings.append(f"Emergency trimmed to {approx_size_bytes(result)} bytes")

    return result, warnings


def create_thin_snapshot(
    environment: Optional[Dict[str, Any]] = None,
    drone_configs: Optional[Dict[str, Any]] = None,
    solution: Optional[Dict[str, Any]] = None,
    metrics: Optional[Dict[str, Any]] = None,
    constraint_program: Optional[Dict[str, Any]] = None,
    compiled_patches: Optional[Dict[str, Any]] = None,
    env_hash: Optional[str] = None,
    solution_hash: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Create a thin snapshot with only essential fields.

    Use this instead of manually constructing snapshots to ensure
    consistent structure and automatic trimming.
    """
    snapshot = {
        "kind": "thin",
    }

    # Environment (trimmed if large)
    if environment:
        # Keep only essential env fields
        snapshot["environment"] = {
            "airports": environment.get("airports", []),
            "targets": environment.get("targets", []),
            "sams": environment.get("sams", []),
        }
        # Don't include synthetic_starts, distance_matrix, etc.

    if drone_configs:
        snapshot["drone_configs"] = drone_configs

    if solution:
        # Remove geometry from solution
        snapshot["solution"] = _remove_edge_geometry({"solution": solution})["solution"]

    if metrics:
        snapshot["metrics"] = metrics

    if constraint_program:
        snapshot["constraint_program"] = constraint_program

    if compiled_patches:
        snapshot["compiled_patches"] = compiled_patches

    if env_hash:
        snapshot["env_hash"] = env_hash

    if solution_hash:
        snapshot["solution_hash"] = solution_hash

    # Apply trimming if needed
    trimmed, warnings = trim_snapshot(snapshot)

    if warnings:
        trimmed["_trim_warnings"] = warnings

    return trimmed


def create_thick_snapshot(
    thin_snapshot: Dict[str, Any],
    distance_matrix: Optional[Dict[str, Any]] = None,
    trace: Optional[Dict[str, Any]] = None,
    messages: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """
    Create a thick snapshot by adding optional data to a thin snapshot.

    Thick snapshots include:
    - Full distance matrix
    - Complete trace
    - Full message history

    Use sparingly - these are much larger and should only be saved
    on explicit user request or for debugging.
    """
    snapshot = thin_snapshot.copy()
    snapshot["kind"] = "thick"

    if distance_matrix:
        snapshot["distance_matrix"] = distance_matrix

    if trace:
        snapshot["trace"] = trace

    if messages:
        snapshot["messages"] = messages

    # Apply trimming with higher limit for thick snapshots
    trimmed, warnings = trim_snapshot(snapshot, max_bytes=SNAPSHOT_MAX_BYTES * 2)

    if warnings:
        trimmed["_trim_warnings"] = warnings

    return trimmed
