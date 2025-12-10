# server/agents/mission_tools.py

"""
Mission analysis tools for ISR planner.

These tools are READ-ONLY: they do not modify routes.
They operate on the shared data contracts:

Environment {
  "airports": [{ "id": str, "x": float, "y": float }, ...],
  "targets": [{ "id": str, "x": float, "y": float, "priority": int, "type": str }, ...],
  "sams": [...],  # not used directly here, included for completeness
  "distance_matrix": { from_id: { to_id: float, ... }, ... }
}

DroneConfig {
  "id": str,
  "enabled": bool,
  "fuelBudget": float,
  "home_airport": str,
  "end_airport": str | "ANY",
  "accessible_targets": [str, ...],
  "frozen_segments": [{ "from": str, "to": str }, ...]
}

Solution {
  "routes": {
    drone_id: {
      "route": [waypoint_id, ...],
      "distance": float,
      "total_points": int,
      "fuel_used": float,
      "feasible": bool
    },
    ...
  },
  "unassigned_targets": [target_id, ...],
  "total_points": int,
  "total_fuel": float
}
"""

from typing import Dict, Any, List, Tuple, Optional
import math


def _index_targets_by_id(environment: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Build a dict {target_id: target_dict} for fast lookup."""
    targets = environment.get("targets", [])
    return {str(t.get("id")): t for t in targets}


def _get_point(environment: Dict[str, Any], waypoint_id: str) -> Optional[Tuple[float, float]]:
    """
    Get (x, y) coordinates for a waypoint (airport or target) by id.
    Returns None if not found.
    """
    wid = str(waypoint_id)

    for a in environment.get("airports", []):
        if str(a.get("id")) == wid:
            return float(a.get("x", 0.0)), float(a.get("y", 0.0))

    for t in environment.get("targets", []):
        if str(t.get("id")) == wid:
            return float(t.get("x", 0.0)), float(t.get("y", 0.0))

    return None


# ---------------------------------------------------------------------------
# 1) Metrics tool
# ---------------------------------------------------------------------------

def compute_solution_metrics(
    environment: Dict[str, Any],
    solution: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Compute standard mission metrics:
    - total_points (already in solution, recomputed here)
    - total_possible_points (sum of all target priorities)
    - points_coverage (fraction of points captured)
    - total_fuel (already in solution, recomputed here)
    - per_drone stats (points, fuel_used, fuel_budget, fuel_margin, visited_targets, unvisited_accessible_targets)

    This does NOT modify the solution.
    """
    target_index = _index_targets_by_id(environment)

    # Total possible points = sum of all target priorities
    total_possible_points = 0
    for t in environment.get("targets", []):
        pri = int(t.get("priority", 0) or 0)
        total_possible_points += pri

    routes = solution.get("routes", {}) or {}

    # Recompute totals from routes to be safe
    total_points = 0
    total_fuel = 0.0

    for did, rdata in routes.items():
        total_points += int(rdata.get("total_points", 0) or 0)
        total_fuel += float(rdata.get("fuel_used", rdata.get("distance", 0.0)) or 0.0)

    per_drone: Dict[str, Any] = {}

    for did, cfg in drone_configs.items():
        did_str = str(did)
        rdata = routes.get(did_str, {})

        # Fuel budget
        fuel_budget = float(
            cfg.get("fuelBudget")
            if cfg.get("fuelBudget") is not None
            else cfg.get("fuel_budget", 0.0)
        )

        fuel_used = float(rdata.get("fuel_used", rdata.get("distance", 0.0)) or 0.0)
        points = int(rdata.get("total_points", 0) or 0)

        fuel_margin = fuel_budget - fuel_used

        route_list = rdata.get("route", []) or []
        visited_targets: List[str] = [
            wid for wid in route_list if wid.startswith("T")
        ]

        # Accessible targets by type
        accessible_types = set(
            str(t).upper() for t in cfg.get("accessible_targets", [])
        )

        unvisited_accessible: List[str] = []
        for tid, t in target_index.items():
            t_type = str(t.get("type", "")).upper()
            if t_type in accessible_types and tid not in visited_targets:
                unvisited_accessible.append(tid)

        per_drone[did_str] = {
            "points": points,
            "fuel_used": fuel_used,
            "fuel_budget": fuel_budget,
            "fuel_margin": fuel_margin,
            "visited_targets": visited_targets,
            "unvisited_accessible_targets": unvisited_accessible,
        }

    points_coverage = (
        float(total_points) / float(total_possible_points)
        if total_possible_points > 0
        else 0.0
    )

    return {
        "total_points": total_points,
        "total_possible_points": total_possible_points,
        "points_coverage": points_coverage,
        "total_fuel": total_fuel,
        "per_drone": per_drone,
    }


# ---------------------------------------------------------------------------
# 2) Geometry analysis tool
# ---------------------------------------------------------------------------

def _segment_intersection(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float],
) -> bool:
    """
    Check if segments p1-p2 and p3-p4 intersect (excluding shared endpoints).
    Standard 2D orientation test.
    """

    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_segment(a, b, c):
        # Check if c is on segment a-b (inclusive)
        return (
            min(a[0], b[0]) <= c[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= c[1] <= max(a[1], b[1])
        )

    o1 = orient(p1, p2, p3)
    o2 = orient(p1, p2, p4)
    o3 = orient(p3, p4, p1)
    o4 = orient(p3, p4, p2)

    # Proper intersection
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True

    # Collinear cases - treat shared endpoints as non-crossings
    if o1 == 0 and on_segment(p1, p2, p3):
        return False
    if o2 == 0 and on_segment(p1, p2, p4):
        return False
    if o3 == 0 and on_segment(p3, p4, p1):
        return False
    if o4 == 0 and on_segment(p3, p4, p2):
        return False

    return False


def _angle_at_point(
    prev_point: Tuple[float, float],
    mid_point: Tuple[float, float],
    next_point: Tuple[float, float],
) -> Optional[float]:
    """
    Compute the angle at mid_point formed by prev_point -> mid_point -> next_point, in degrees.
    Returns None if any vector is degenerate.
    """
    vx1 = prev_point[0] - mid_point[0]
    vy1 = prev_point[1] - mid_point[1]
    vx2 = next_point[0] - mid_point[0]
    vy2 = next_point[1] - mid_point[1]

    norm1 = math.hypot(vx1, vy1)
    norm2 = math.hypot(vx2, vy2)
    if norm1 == 0 or norm2 == 0:
        return None

    dot = vx1 * vx2 + vy1 * vy2
    cos_theta = dot / (norm1 * norm2)
    cos_theta = max(min(cos_theta, 1.0), -1.0)
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)
    return angle_deg


def analyze_solution_geometry(
    environment: Dict[str, Any],
    solution: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
    angle_threshold_degrees: float = 60.0,
) -> Dict[str, Any]:
    """
    Analyze the geometry of the current solution:

    - crossings: list of non-frozen crossings between segments
    - acute_angles: list of very sharp turns at targets (angle < angle_threshold_degrees)
    - frozen_segments: all frozen segments per drone

    NOTE:
    - A "crossing" is reported only if it does not involve any frozen segment.
    - Acute angles highlight potential candidates for reassignment or reordering.

    This does NOT modify the solution.
    """
    routes = solution.get("routes", {}) or {}

    # Build position cache for speed
    def get_pos(wid: str) -> Optional[Tuple[float, float]]:
        return _get_point(environment, wid)

    # Collect all segments with positions, drone, and frozen flag
    segments: List[Dict[str, Any]] = []
    frozen_lookup = set()

    # Build frozen lookup: (drone_id, from, to)
    for did, cfg in drone_configs.items():
        did_str = str(did)
        for seg in cfg.get("frozen_segments", []):
            f = str(seg.get("from"))
            t = str(seg.get("to"))
            frozen_lookup.add((did_str, f, t))

    for did, rdata in routes.items():
        did_str = str(did)
        route_list = rdata.get("route", []) or []
        for i in range(len(route_list) - 1):
            a = str(route_list[i])
            b = str(route_list[i + 1])
            pa = get_pos(a)
            pb = get_pos(b)
            if pa is None or pb is None:
                continue
            is_frozen = (did_str, a, b) in frozen_lookup
            segments.append(
                {
                    "drone": did_str,
                    "from": a,
                    "to": b,
                    "p_from": pa,
                    "p_to": pb,
                    "frozen": is_frozen,
                }
            )

    crossings: List[Dict[str, Any]] = []

    # Check pairs of segments for crossings
    for i in range(len(segments)):
        s1 = segments[i]
        for j in range(i + 1, len(segments)):
            s2 = segments[j]

            # Skip if they share an endpoint
            if (
                s1["from"] == s2["from"]
                or s1["from"] == s2["to"]
                or s1["to"] == s2["from"]
                or s1["to"] == s2["to"]
            ):
                continue

            p1, p2 = s1["p_from"], s1["p_to"]
            p3, p4 = s2["p_from"], s2["p_to"]

            if _segment_intersection(p1, p2, p3, p4):
                involves_frozen = s1["frozen"] or s2["frozen"]
                crossings.append(
                    {
                        "drone1": s1["drone"],
                        "segment1": {"from": s1["from"], "to": s1["to"]},
                        "drone2": s2["drone"],
                        "segment2": {"from": s2["from"], "to": s2["to"]},
                        "involves_frozen_segment": involves_frozen,
                    }
                )

    # Acute angles at targets
    acute_angles: List[Dict[str, Any]] = []

    for did, rdata in routes.items():
        did_str = str(did)
        route_list = rdata.get("route", []) or []
        for i in range(1, len(route_list) - 1):
            prev_id = str(route_list[i - 1])
            mid_id = str(route_list[i])
            next_id = str(route_list[i + 1])

            # We care about targets (turns at targets)
            if not mid_id.startswith("T"):
                continue

            p_prev = get_pos(prev_id)
            p_mid = get_pos(mid_id)
            p_next = get_pos(next_id)
            if p_prev is None or p_mid is None or p_next is None:
                continue

            angle = _angle_at_point(p_prev, p_mid, p_next)
            if angle is None:
                continue

            if angle < angle_threshold_degrees:
                acute_angles.append(
                    {
                        "drone": did_str,
                        "target": mid_id,
                        "incoming": {"from": prev_id, "to": mid_id},
                        "outgoing": {"from": mid_id, "to": next_id},
                        "angle_degrees": angle,
                    }
                )

    # Collect all frozen segments in a simpler form
    frozen_segments_out: List[Dict[str, Any]] = []
    for did, cfg in drone_configs.items():
        did_str = str(did)
        for seg in cfg.get("frozen_segments", []):
            frozen_segments_out.append(
                {
                    "drone": did_str,
                    "from": str(seg.get("from")),
                    "to": str(seg.get("to")),
                }
            )

    return {
        "crossings": crossings,
        "acute_angles": acute_angles,
        "frozen_segments": frozen_segments_out,
    }
