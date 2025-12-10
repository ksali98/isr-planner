"""
Boundary Navigation using Tangent-Arc-Tangent Algorithm

Plans paths around convex polygon obstacles (SAM zones) using:
1. Radar sweep to find tangent vertices from external point
2. Walk vertex-to-vertex along polygon boundary (arc)
3. Try both directions, pick shorter total path
4. Handle multiple polygons sequentially

INVIOLABLE CONSTRAINT: Paths NEVER enter polygon interiors.
"""

from __future__ import annotations

import math
from typing import List, Tuple, Dict, Any, Optional

# Import wrap_sams to convert SAM circles to convex polygons
from .sam_wrapping import wrap_sams

Point = Tuple[float, float]
Polygon = List[Point]


# ---------- Basic Geometry Utilities ----------

def _distance(a: Point, b: Point) -> float:
    """Euclidean distance between two points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _cross(o: Point, a: Point, b: Point) -> float:
    """
    2D cross product (OA x OB).
    >0: counter-clockwise, <0: clockwise, =0: collinear
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _point_in_polygon(p: Point, polygon: Polygon) -> bool:
    """
    Check if point is inside polygon using ray casting.
    Returns True if strictly inside.
    """
    x, y = p
    n = len(polygon)
    inside = False

    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i

    return inside


def _segment_intersects_segment(
    a1: Point, a2: Point, b1: Point, b2: Point
) -> bool:
    """Check if line segment a1-a2 intersects segment b1-b2."""
    d1 = _cross(b1, b2, a1)
    d2 = _cross(b1, b2, a2)
    d3 = _cross(a1, a2, b1)
    d4 = _cross(a1, a2, b2)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True

    # Check collinear cases
    eps = 1e-9
    if abs(d1) < eps and _on_segment(b1, a1, b2):
        return True
    if abs(d2) < eps and _on_segment(b1, a2, b2):
        return True
    if abs(d3) < eps and _on_segment(a1, b1, a2):
        return True
    if abs(d4) < eps and _on_segment(a1, b2, a2):
        return True

    return False


def _on_segment(p: Point, q: Point, r: Point) -> bool:
    """Check if q lies on segment p-r (assuming collinear)."""
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))


def _is_polygon_vertex(p: Point, polygon: Polygon, eps: float = 1e-9) -> bool:
    """Check if point p is a vertex of the polygon."""
    for v in polygon:
        if abs(p[0] - v[0]) < eps and abs(p[1] - v[1]) < eps:
            return True
    return False


def _segment_crosses_polygon(a: Point, b: Point, polygon: Polygon) -> bool:
    """
    Check if segment a-b crosses through the interior of polygon.
    Returns True if it enters the polygon (not just touches boundary).

    A tangent line that touches the polygon at a vertex is NOT a crossing.
    """
    n = len(polygon)
    if n < 3:
        return False

    # Check if either endpoint is inside (but not if it's a polygon vertex)
    a_is_vertex = _is_polygon_vertex(a, polygon)
    b_is_vertex = _is_polygon_vertex(b, polygon)

    if not a_is_vertex and _point_in_polygon(a, polygon):
        return True
    if not b_is_vertex and _point_in_polygon(b, polygon):
        return True

    # Check if segment intersects any polygon edge (proper crossing, not at endpoints)
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]

        # Skip edges that share a vertex with our segment endpoints
        if b_is_vertex:
            # If b is a polygon vertex, skip edges that include b
            if (_is_polygon_vertex(b, [p1]) or _is_polygon_vertex(b, [p2])):
                continue
        if a_is_vertex:
            # If a is a polygon vertex, skip edges that include a
            if (_is_polygon_vertex(a, [p1]) or _is_polygon_vertex(a, [p2])):
                continue

        if _segment_intersects_segment(a, b, p1, p2):
            return True

    # Sample multiple points along the segment (not just midpoint)
    # This catches cases where the segment clips a corner of the polygon
    n_samples = 20  # More samples for longer segments
    for i in range(1, n_samples):  # Skip endpoints (already checked above)
        t = i / n_samples
        px = a[0] + t * (b[0] - a[0])
        py = a[1] + t * (b[1] - a[1])
        if _point_in_polygon((px, py), polygon):
            return True

    return False


def _path_length(path: List[Point]) -> float:
    """Calculate total length of a path."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(len(path) - 1):
        total += _distance(path[i], path[i + 1])
    return total


# ---------- Tangent Finding for Convex Polygons ----------

def _line_of_sight_clear(point: Point, target: Point, polygon: Polygon, target_idx: int) -> bool:
    """
    Check if the line from point to target (a polygon vertex) is clear.

    The line is clear if it doesn't cross any polygon edge except for edges
    that share the target vertex.

    Args:
        point: External point
        target: Target vertex on polygon (polygon[target_idx])
        polygon: The convex polygon
        target_idx: Index of target in polygon

    Returns:
        True if line of sight is clear
    """
    n = len(polygon)

    for i in range(n):
        # Skip edges that include the target vertex
        if i == target_idx or (i + 1) % n == target_idx:
            continue

        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]

        if _segment_intersects_segment(point, target, p1, p2):
            return False

    # Also check if the midpoint is inside the polygon
    mid = ((point[0] + target[0]) / 2, (point[1] + target[1]) / 2)
    if _point_in_polygon(mid, polygon):
        return False

    return True


def _find_tangent_vertices(point: Point, polygon: Polygon) -> Tuple[int, int]:
    """
    Find the two tangent vertices from an external point to a convex polygon.

    For a convex polygon, the tangent vertices are found by checking where the
    external point transitions from being on one side of adjacent edges to the other.
    These are the "silhouette" vertices - the extreme points visible from the external point.

    A vertex V is a tangent point if:
    - For LEFT tangent: the polygon "turns away" from the point at V
      (point is LEFT of incoming edge, RIGHT of outgoing edge)
    - For RIGHT tangent: the polygon "turns toward" the point at V
      (point is RIGHT of incoming edge, LEFT of outgoing edge)

    CRITICAL: After finding candidate tangents, we verify that the tangent LINE
    from the external point to the tangent vertex doesn't cross the polygon.
    If it does, we walk along adjacent vertices to find the actual visible tangent.

    Returns (left_tangent_idx, right_tangent_idx)
    """
    n = len(polygon)
    if n < 3:
        return (0, 0)

    left_tangent_idx = -1
    right_tangent_idx = -1

    # Track best candidates with their "strength" (how clearly they satisfy the condition)
    # This handles the case where a point is nearly collinear with an edge
    left_candidates = []
    right_candidates = []

    for i in range(n):
        prev_idx = (i - 1) % n
        next_idx = (i + 1) % n

        v_prev = polygon[prev_idx]
        v_curr = polygon[i]
        v_next = polygon[next_idx]

        # Cross product to determine which side of each edge the point is on
        # _cross(A, B, P) computes (B-A) x (P-A)
        # > 0 means P is to the LEFT of edge A->B
        # < 0 means P is to the RIGHT of edge A->B

        # For edge prev->curr: is point to the left or right?
        cross_prev = _cross(v_prev, v_curr, point)
        # For edge curr->next: is point to the left or right?
        cross_next = _cross(v_curr, v_next, point)

        # Left tangent: point is LEFT of prev edge (cross_prev > 0)
        #               AND RIGHT of next edge (cross_next < 0)
        # This is where the polygon boundary goes from facing toward the point
        # to facing away - the "left silhouette edge"
        # Use STRICT inequality for left tangent
        if cross_prev > 0 and cross_next < 0:
            # Clear left tangent - strong candidate
            left_candidates.append((i, cross_prev - cross_next))  # larger difference = stronger
        elif cross_prev >= 0 and cross_next <= 0:
            # Weak candidate (one or both conditions are exactly 0)
            # Only add if we don't have a strong candidate yet
            left_candidates.append((i, 0.1))  # low priority

        # Right tangent: point is RIGHT of prev edge (cross_prev < 0)
        #                AND LEFT of next edge (cross_next > 0)
        # This is where the polygon boundary goes from facing away from the point
        # to facing toward - the "right silhouette edge"
        # Use STRICT inequality for right tangent
        if cross_prev < 0 and cross_next > 0:
            # Clear right tangent - strong candidate
            right_candidates.append((i, cross_next - cross_prev))  # larger difference = stronger
        elif cross_prev <= 0 and cross_next >= 0:
            # Weak candidate
            right_candidates.append((i, 0.1))

    # Select best candidates (highest strength score)
    if left_candidates:
        left_tangent_idx = max(left_candidates, key=lambda x: x[1])[0]
    if right_candidates:
        right_tangent_idx = max(right_candidates, key=lambda x: x[1])[0]

    # Fallback if not found (shouldn't happen for proper convex polygon with external point)
    if left_tangent_idx == -1:
        left_tangent_idx = 0
    if right_tangent_idx == -1:
        right_tangent_idx = n // 2

    # If both tangents ended up at the same vertex, we need to find the other one
    # This can happen when the point is nearly collinear with a polygon edge
    if left_tangent_idx == right_tangent_idx:
        # Find the vertex that is geometrically opposite (farthest around the polygon)
        # Use angular sweep from the point to find the two extreme vertices

        # Calculate angle from point to each vertex
        angles = []
        for i in range(n):
            vx, vy = polygon[i]
            dx, dy = vx - point[0], vy - point[1]
            angle = math.atan2(dy, dx)
            angles.append((angle, i))

        angles.sort(key=lambda x: x[0])

        # Find the largest angular gap between consecutive vertices
        max_gap = -1
        gap_start_idx = 0
        for i in range(n):
            curr_angle = angles[i][0]
            next_angle = angles[(i + 1) % n][0]

            # Handle wrap-around
            gap = next_angle - curr_angle
            if gap < 0:
                gap += 2 * math.pi

            if gap > max_gap:
                max_gap = gap
                gap_start_idx = i

        # The two tangent vertices are at the edges of this gap
        right_tangent_idx = angles[gap_start_idx][1]
        left_tangent_idx = angles[(gap_start_idx + 1) % n][1]

    # CRITICAL: Verify line-of-sight visibility for both tangents
    # If a tangent line crosses the polygon, walk along vertices to find the actual visible tangent

    # Check left tangent - if blocked, walk CW (decrement) to find visible vertex
    if not _line_of_sight_clear(point, polygon[left_tangent_idx], polygon, left_tangent_idx):
        # Walk CW from left tangent toward right tangent
        for step in range(1, n):
            candidate_idx = (left_tangent_idx - step) % n  # CW direction
            if _line_of_sight_clear(point, polygon[candidate_idx], polygon, candidate_idx):
                left_tangent_idx = candidate_idx
                break

    # Check right tangent - if blocked, walk CCW (increment) to find visible vertex
    if not _line_of_sight_clear(point, polygon[right_tangent_idx], polygon, right_tangent_idx):
        # Walk CCW from right tangent toward left tangent
        for step in range(1, n):
            candidate_idx = (right_tangent_idx + step) % n  # CCW direction
            if _line_of_sight_clear(point, polygon[candidate_idx], polygon, candidate_idx):
                right_tangent_idx = candidate_idx
                break

    return (left_tangent_idx, right_tangent_idx)


# ---------- Arc Walking (Vertex to Vertex) ----------

def _dot_product(v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
    """Compute dot product of two 2D vectors."""
    return v1[0] * v2[0] + v1[1] * v2[1]


def _walk_polygon_cw(polygon: Polygon, start_idx: int, end_idx: int) -> List[Point]:
    """
    Walk along polygon from start_idx to end_idx in clockwise direction.
    Returns list of vertices including start and end.
    """
    n = len(polygon)
    path = []
    i = start_idx
    max_steps = n + 1  # Safety limit
    steps = 0
    while steps < max_steps:
        path.append(polygon[i])
        if i == end_idx:
            break
        i = (i - 1) % n  # CW = decrement index (polygon is CCW)
        steps += 1
    return path


def _walk_polygon_ccw(polygon: Polygon, start_idx: int, end_idx: int) -> List[Point]:
    """
    Walk along polygon from start_idx to end_idx in counter-clockwise direction.
    Returns list of vertices including start and end.
    """
    n = len(polygon)
    path = []
    i = start_idx
    max_steps = n + 1  # Safety limit
    steps = 0
    while steps < max_steps:
        path.append(polygon[i])
        if i == end_idx:
            break
        i = (i + 1) % n  # CCW = increment index
        steps += 1
    return path


# ---------- Single Polygon Navigation ----------

def _tangent_continues_toward_goal(
    start: Point,
    tangent_vertex: Point,
    goal: Point,
    debug: bool = False
) -> bool:
    """
    Check if going from start to tangent_vertex continues toward the goal.

    A tangent is valid if the angle between:
    - Vector from start to tangent_vertex
    - Vector from start to goal
    is less than 90 degrees (dot product > 0).

    This rejects tangents that would cause the drone to fly AWAY from the goal.
    """
    # Vector from start to tangent
    to_tangent = (tangent_vertex[0] - start[0], tangent_vertex[1] - start[1])
    # Vector from start to goal
    to_goal = (goal[0] - start[0], goal[1] - start[1])

    dot = _dot_product(to_tangent, to_goal)

    if debug:
        print(f"    Tangent direction check: to_tangent={to_tangent}, to_goal={to_goal}, dot={dot:.2f}")

    return dot > 0


def _exit_tangent_continues_toward_goal(
    tangent_vertex: Point,
    goal: Point,
    prev_vertex: Point,
    debug: bool = False
) -> bool:
    """
    Check if exiting from tangent_vertex toward goal continues forward.

    The drone arrives at tangent_vertex from the polygon boundary (prev_vertex).
    Check if going from tangent_vertex to goal continues in roughly the same
    direction (doesn't reverse).

    We use the sign of the dot product between:
      - arrival_vec: prev_vertex -> tangent_vertex
      - exit_vec:    tangent_vertex -> goal

    If dot < 0, angle > 90 degrees => reversal => REJECT.
    """
    # Arrival direction (from previous polygon vertex to tangent)
    arrival_vec = (
        tangent_vertex[0] - prev_vertex[0],
        tangent_vertex[1] - prev_vertex[1],
    )
    # Exit direction (from tangent to goal)
    exit_vec = (
        goal[0] - tangent_vertex[0],
        goal[1] - tangent_vertex[1],
    )

    dot = _dot_product(arrival_vec, exit_vec)

    if debug:
        print(
            f"    Exit direction check: arrival={arrival_vec}, "
            f"exit={exit_vec}, dot={dot:.2f}"
        )

    # dot < 0 → angle > 90° → reversal → reject
    return dot >= 0.0

def _entry_tangent_continues_forward(
    start: Point,
    first_vertex: Point,
    second_vertex: Point,
    debug: bool = False
) -> bool:
    """
    Check if ENTERING the polygon boundary continues forward.

    We compare:
      - arrival_vec:  start -> first_vertex (entry direction)
      - boundary_vec: first_vertex -> second_vertex (direction of boundary walk)

    If the dot product is negative, the entry would reverse direction
    (angle > 90°) and must be rejected.
    """
    arrival_vec = (
        first_vertex[0] - start[0],
        first_vertex[1] - start[1],
    )
    boundary_vec = (
        second_vertex[0] - first_vertex[0],
        second_vertex[1] - first_vertex[1],
    )

    dot = _dot_product(arrival_vec, boundary_vec)

    if debug:
        print(
            f"    Entry direction check: arrival={arrival_vec}, "
            f"boundary={boundary_vec}, dot={dot:.2f}"
        )

    # dot < 0 → angle > 90° → reversal → reject
    return dot >= 0.0

def _entry_tangent_continues_forward(
    start: Point,
    first_vertex: Point,
    second_vertex: Point,
    debug: bool = False
) -> bool:
    """
    Ensure that when we ENTER the polygon boundary we do not reverse direction.

    We look at:
      - arrival_vec: from start to first boundary vertex (entry),
      - boundary_vec: from first vertex to second vertex (direction of travel on boundary).

    If the dot product is negative, we are turning back on ourselves
    (angle > 90°) -> reject.
    """
    arrival_vec = (first_vertex[0] - start[0],
                   first_vertex[1] - start[1])
    boundary_vec = (second_vertex[0] - first_vertex[0],
                    second_vertex[1] - first_vertex[1])

    dot = _dot_product(arrival_vec, boundary_vec)

    if debug:
        print(
            f"    Entry direction check: arrival={arrival_vec}, "
            f"boundary={boundary_vec}, dot={dot:.2f}"
        )

    # dot < 0 -> angle > 90° -> reversal -> reject
    return dot >= 0.0


def _find_first_visible_exit(
    polygon: Polygon,
    start_idx: int,
    direction: str,
    goal: Point,
    debug: bool = False
) -> Tuple[int, List[Point]]:
    """
    Walk around polygon in given direction from start_idx until we find a vertex
    from which we can see the goal (direct line doesn't cross polygon).

    This ensures we exit at the FIRST opportunity in our chosen direction,
    preventing unnecessary "going the long way around" the polygon.

    Returns (exit_idx, arc_path) where arc_path includes all vertices from start to exit.
    Returns (-1, []) if no valid exit found (shouldn't happen for convex polygons).
    """
    n = len(polygon)
    arc = [polygon[start_idx]]
    current_idx = start_idx

    # Walk around the polygon (at most n steps to avoid infinite loop)
    for step in range(n):
        # Check if we can see the goal from current vertex
        current_vertex = polygon[current_idx]
        if not _segment_crosses_polygon(current_vertex, goal, polygon):
            if debug:
                print(f"    Found visible exit at idx={current_idx} after {step} steps in {direction.upper()} direction")
            return current_idx, arc

        # Move to next vertex in chosen direction
        if direction == 'cw':
            current_idx = (current_idx - 1) % n
        else:  # ccw
            current_idx = (current_idx + 1) % n

        arc.append(polygon[current_idx])

    if debug:
        print(f"    No visible exit found after walking entire polygon")
    return -1, []

def _navigate_around_single_polygon(
    start: Point,
    goal: Point,
    polygon: Polygon,
    debug: bool = False
) -> Tuple[List[Point], float, str]:
    """
    Navigate from start to goal around a single convex polygon.

    CRITICAL BEHAVIOR (per user requirements):
    - Never enter the polygon interior (SAM zone).
    - When ENTERING the boundary: do not reverse direction.
    - When EXITING the boundary: do not reverse direction.
    - Try both tangents (left/right) and both directions (CW/CCW),
      then pick the SHORTEST valid path.

    Algorithm:
      1. Find the two tangent vertices from start (left and right).
      2. For each start tangent that does NOT fly away from the goal:
           - Try CW and CCW walks along the boundary to find the first
             vertex that can see the goal.
           - For each arc candidate:
               * Enforce entry-direction check (no reversal at entry).
               * Enforce exit-direction check (no reversal at exit).
               * Build full path start → arc → goal.
               * Keep only paths that pass polygon intersection validation.
      3. If no candidates, run a broader fallback search.
      4. Return the shortest valid path.
    """
    n = len(polygon)
    if n < 3:
        # Degenerate polygon – just go direct
        return [start, goal], _distance(start, goal), "direct (degenerate polygon)"

    # Find tangent vertices from start
    start_left_idx, start_right_idx = _find_tangent_vertices(start, polygon)

    if debug:
        print(f"  Start: {start}, Goal: {goal}")
        print(
            f"  Start tangents: left={start_left_idx} at {polygon[start_left_idx]}, "
            f"right={start_right_idx} at {polygon[start_right_idx]}"
        )

    candidate_paths: List[Tuple[List[Point], float, str]] = []

    # Try each start tangent: left and right
    for start_tangent_idx, tangent_name in [
        (start_left_idx, "left"),
        (start_right_idx, "right"),
    ]:
        tangent_vertex = polygon[start_tangent_idx]

        # Reject tangents that cause the drone to fly AWAY from the goal
        if not _tangent_continues_toward_goal(
            start, tangent_vertex, goal, debug=debug
        ):
            if debug:
                print(
                    f"  REJECTED {tangent_name} tangent (idx={start_tangent_idx}): "
                    f"would fly away from goal"
                )
            continue

        if debug:
            print(f"  Trying {tangent_name} tangent (idx={start_tangent_idx})")

        # TRY BOTH DIRECTIONS - CW and CCW
        for direction in ["cw", "ccw"]:
            exit_idx, arc = _find_first_visible_exit(
                polygon, start_tangent_idx, direction, goal, debug=debug
            )

            if exit_idx == -1:
                if debug:
                    print(
                        f"    No visible exit found walking "
                        f"{direction.upper()} from {tangent_name} tangent"
                    )
                continue

            # --- ENTRY ANGLE CHECK ---
            # We require at least 2 vertices to define the boundary direction.
            if len(arc) >= 2:
                first_vertex = arc[0]
                second_vertex = arc[1]
                if not _entry_tangent_continues_forward(
                    start, first_vertex, second_vertex, debug=debug
                ):
                    if debug:
                        print(
                            f"    REJECTED {direction.upper()} via {tangent_name}: "
                            f"entry would reverse direction"
                        )
                    continue

            # --- EXIT ANGLE CHECK ---
            if len(arc) >= 2:
                prev_vertex = arc[-2]   # second-to-last boundary vertex
                exit_vertex = arc[-1]   # exit vertex
                if not _exit_tangent_continues_toward_goal(
                    exit_vertex, goal, prev_vertex, debug=debug
                ):
                    if debug:
                        print(
                            f"    REJECTED {direction.upper()} via {tangent_name}: "
                            f"exit would reverse direction"
                        )
                    continue

            # Build the full candidate path: start → arc → goal
            path = [start] + arc + [goal]
            length = _path_length(path)
            method = (
                f"{direction.upper()} via {tangent_name} tangent, exit at idx {exit_idx}"
            )

            # Validate the path doesn't cross / enter the polygon
            if _validate_path(path, [polygon]):
                candidate_paths.append((path, length, method))
                if debug:
                    print(
                        f"    Valid path: {method}, length={length:.2f}, "
                        f"waypoints={len(path)}"
                    )
            elif debug:
                print(f"    Invalid path: {method} - crosses polygon")

    # If nothing survived the strict checks, try a broader fallback
    if not candidate_paths:
        if debug:
            print(
                "  No valid paths found under strict rules. "
                "Trying all combinations as fallback..."
            )

        for start_tangent_idx, tangent_name in [
            (start_left_idx, "left"),
            (start_right_idx, "right"),
        ]:
            for direction in ["cw", "ccw"]:
                exit_idx, arc = _find_first_visible_exit(
                    polygon, start_tangent_idx, direction, goal, debug=debug
                )

                if exit_idx == -1 or not arc:
                    continue

                path = [start] + arc + [goal]
                length = _path_length(path)
                method = (
                    f"FALLBACK {direction.upper()} via {tangent_name}, "
                    f"exit at idx {exit_idx}"
                )

                if _validate_path(path, [polygon]):
                    candidate_paths.append((path, length, method))
                    if debug:
                        print(
                            f"    Fallback valid path: {method}, "
                            f"length={length:.2f}, waypoints={len(path)}"
                        )
                elif debug:
                    print(f"    Fallback invalid path: {method} - crosses polygon")

    # Still nothing? Report failure clearly.
    if not candidate_paths:
        if debug:
            print("  No valid paths found even with fallback!")
        return [], float("inf"), "INVALID: no valid path around polygon"

    # Pick the shortest valid path
    best_path, best_length, best_method = min(
        candidate_paths, key=lambda x: x[1]
    )

    if debug:
        print(f"  Best path: {best_method}, length={best_length:.2f}")
        # Also show all candidates for debugging
        print("  All candidates:")
        for path, length, method in sorted(candidate_paths, key=lambda x: x[1]):
            print(
                f"    - {method}: {length:.2f} ({len(path)} waypoints)"
            )

    return best_path, best_length, best_method

# ---------- Multi-Polygon Navigation using Visibility Graph ----------

def _find_blocking_polygon(
    a: Point, b: Point, polygons: List[Polygon]
) -> Optional[int]:
    """
    Find the first polygon that blocks the segment a->b.
    Returns the index of the blocking polygon, or None if path is clear.
    """
    for i, poly in enumerate(polygons):
        if _segment_crosses_polygon(a, b, poly):
            return i
    return None


def _segment_clear_of_all_polygons(a: Point, b: Point, polygons: List[Polygon]) -> bool:
    """Check if segment a->b doesn't cross any polygon."""
    for poly in polygons:
        if _segment_crosses_polygon(a, b, poly):
            return False
    return True


def _build_visibility_graph(
    start: Point,
    goal: Point,
    polygons: List[Polygon],
    debug: bool = False
) -> Dict[Point, List[Tuple[Point, float]]]:
    """
    Build a visibility graph for path planning among multiple polygons.

    Nodes: start, goal, and all polygon vertices
    Edges: Pairs of nodes that can see each other (no polygon crossing)

    Returns adjacency list: {point: [(neighbor, distance), ...]}
    """
    # Collect all nodes
    nodes: List[Point] = [start, goal]
    for poly in polygons:
        nodes.extend(poly)

    # Remove duplicates while preserving order
    seen = set()
    unique_nodes = []
    for node in nodes:
        # Round for comparison
        key = (round(node[0], 6), round(node[1], 6))
        if key not in seen:
            seen.add(key)
            unique_nodes.append(node)
    nodes = unique_nodes

    if debug:
        print(f"  Visibility graph: {len(nodes)} nodes ({len(polygons)} polygons)")

    # Build adjacency list
    graph: Dict[Point, List[Tuple[Point, float]]] = {node: [] for node in nodes}

    for i, a in enumerate(nodes):
        for j, b in enumerate(nodes):
            if i >= j:
                continue

            # Check if a and b can see each other
            if _segment_clear_of_all_polygons(a, b, polygons):
                dist = _distance(a, b)
                graph[a].append((b, dist))
                graph[b].append((a, dist))

    # Also add edges along polygon boundaries (these are always valid)
    for poly in polygons:
        n = len(poly)
        for i in range(n):
            a = poly[i]
            b = poly[(i + 1) % n]
            dist = _distance(a, b)

            # Ensure both vertices are in the graph (handle floating point mismatches)
            if a not in graph:
                graph[a] = []
            if b not in graph:
                graph[b] = []

            # Add edge if not already present
            a_neighbors = [neighbor for neighbor, _ in graph.get(a, [])]
            if b not in a_neighbors:
                graph[a].append((b, dist))
                graph[b].append((a, dist))

    return graph

def _dijkstra(
    graph: Dict[Point, List[Tuple[Point, float]]],
    start: Point,
    goal: Point,
    debug: bool = False
) -> Tuple[List[Point], float]:
    """
    Find shortest path from start to goal using Dijkstra's algorithm.

    Returns (path, distance) or ([], inf) if no path found.
    """
    import heapq

    # Priority queue: (distance, node, path)
    pq = [(0.0, start, [start])]
    visited = set()

    while pq:
        dist, node, path = heapq.heappop(pq)

        # Round for comparison
        node_key = (round(node[0], 6), round(node[1], 6))

        if node_key in visited:
            continue
        visited.add(node_key)

        # Check if we reached the goal
        goal_key = (round(goal[0], 6), round(goal[1], 6))
        if node_key == goal_key:
            if debug:
                print(f"  Dijkstra found path: {len(path)} waypoints, dist={dist:.2f}")
            return path, dist

        # Explore neighbors
        for neighbor, edge_dist in graph.get(node, []):
            neighbor_key = (round(neighbor[0], 6), round(neighbor[1], 6))
            if neighbor_key not in visited:
                new_dist = dist + edge_dist
                new_path = path + [neighbor]
                heapq.heappush(pq, (new_dist, neighbor, new_path))

    if debug:
        print(f"  Dijkstra: no path found")
    return [], float('inf')


def _navigate_multi_polygon(
    start: Point,
    goal: Point,
    polygons: List[Polygon],
    debug: bool = False
) -> Tuple[List[Point], float, str]:
    """
    Navigate from start to goal avoiding multiple polygons using visibility graph.

    This properly handles complex multi-polygon scenarios where tangent lines
    may be blocked by other polygons.

    Algorithm:
    1. Build visibility graph with start, goal, and all polygon vertices
    2. Use Dijkstra to find shortest path through the graph
    3. Validate and return the path
    """
    # Check if direct path to goal is clear
    if _segment_clear_of_all_polygons(start, goal, polygons):
        dist = _distance(start, goal)
        return [start, goal], dist, "direct"

    if debug:
        print(f"  Direct path blocked, building visibility graph...")

    # Build visibility graph
    graph = _build_visibility_graph(start, goal, polygons, debug=debug)

    # Find shortest path using Dijkstra
    path, dist = _dijkstra(graph, start, goal, debug=debug)

    if not path:
        if debug:
            print(f"  Visibility graph: no path found")
        return [], float('inf'), "INVALID: no path in visibility graph"

    # Validate the path
    if not _validate_path(path, polygons, debug=debug):
        if debug:
            print(f"  Visibility graph path failed validation")
        return [], float('inf'), "INVALID: visibility graph path crosses polygon"

    return path, dist, f"visibility-graph ({len(path)} waypoints)"


# ---------- Path Validation ----------

def _is_point_on_polygon_boundary(p: Point, polygon: Polygon, eps: float = 1e-6) -> bool:
    """Check if point is on the polygon boundary (on an edge or vertex)."""
    n = len(polygon)
    for i in range(n):
        v1 = polygon[i]
        v2 = polygon[(i + 1) % n]

        # Check if on vertex
        if abs(p[0] - v1[0]) < eps and abs(p[1] - v1[1]) < eps:
            return True

        # Check if on edge v1-v2
        # Point is on edge if it's collinear and between v1 and v2
        dx = v2[0] - v1[0]
        dy = v2[1] - v1[1]

        if abs(dx) < eps and abs(dy) < eps:
            continue  # degenerate edge

        # Parameterize: p = v1 + t * (v2 - v1)
        if abs(dx) > abs(dy):
            t = (p[0] - v1[0]) / dx
            expected_y = v1[1] + t * dy
            if 0 <= t <= 1 and abs(p[1] - expected_y) < eps:
                return True
        else:
            t = (p[1] - v1[1]) / dy
            expected_x = v1[0] + t * dx
            if 0 <= t <= 1 and abs(p[0] - expected_x) < eps:
                return True

    return False


def _segment_enters_polygon(a: Point, b: Point, polygon: Polygon, n_samples: int = 20) -> bool:
    """
    Check if segment a-b enters the interior of the polygon.
    Samples points along the segment and checks if any are strictly inside.
    Points on the boundary are allowed.
    """
    for i in range(n_samples + 1):
        t = i / n_samples
        px = a[0] + t * (b[0] - a[0])
        py = a[1] + t * (b[1] - a[1])
        p = (px, py)

        # Check if this point is strictly inside (not on boundary)
        if _point_in_polygon(p, polygon):
            # But allow if it's on the boundary
            if not _is_point_on_polygon_boundary(p, polygon):
                return True

    return False

def _validate_path(path: List[Point], polygons: List[Polygon], debug: bool = False) -> bool:
    """
    Validate that no segment of the path enters any polygon interior.
    Points on polygon boundaries are allowed (they're part of the navigation).
    """
    if len(path) < 2:
        return True

    for i in range(len(path) - 1):
        for poly_idx, poly in enumerate(polygons):
            if _segment_enters_polygon(path[i], path[i + 1], poly):
                if debug:
                    print(f"    VALIDATION FAIL: segment {i} ({path[i]}) -> ({path[i+1]}) crosses polygon {poly_idx}")
                return False
    return True


# ---------- Main Entry Point ----------

def _choose_hull_direction(entry_point, prev_vertex, next_vertex, tangent_vec):
    """
    Decide whether to leave the entry vertex towards prev_vertex or next_vertex,
    based on the incoming tangent direction.

    entry_point: (x, y) where we hit the hull (can equal the vertex)
    prev_vertex, next_vertex: neighbor vertices on polygon
    tangent_vec: direction we were travelling before touching the hull (dx, dy)
    """

    tx, ty = tangent_vec

    # Edge from entry to prev / next
    v_prev = (prev_vertex[0] - entry_point[0], prev_vertex[1] - entry_point[1])
    v_next = (next_vertex[0] - entry_point[0], next_vertex[1] - entry_point[1])

    dot_prev = tx * v_prev[0] + ty * v_prev[1]
    dot_next = tx * v_next[0] + ty * v_next[1]

    # Prefer edges that keep us roughly in the same direction (dot >= 0)
    candidates = []
    if dot_prev >= 0:
        candidates.append(("prev", dot_prev))
    if dot_next >= 0:
        candidates.append(("next", dot_next))

    if candidates:
        # Pick the edge with the largest dot product (most aligned with tangent)
        return max(candidates, key=lambda x: x[1])[0]

    # Both are "backwards" (dot < 0); pick the less-bad one
    return "prev" if dot_prev > dot_next else "next"


def plan_path(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    sams: List[Dict[str, Any]] = None,
    debug: bool = False,
) -> Tuple[List[Tuple[float, float]], float, str]:
    """
    Plan a path from start to goal avoiding SAM zones.

    Uses tangent-arc-tangent algorithm:
    1. Convert SAMs to convex polygons via wrap_sams
    2. Check if direct path is clear
    3. If blocked, navigate around polygons using tangent-arc-tangent
    4. Try both directions, return shorter valid path

    Args:
        start: (x, y) starting position
        goal: (x, y) goal position
        sams: List of SAM dicts with 'pos'/'x','y' and 'range'/'radius'
        debug: Enable debug output

    Returns:
        (path, distance, method) where:
        - path: List of (x, y) waypoints from start to goal
        - distance: Total path length
        - method: Description of planning method used
    """
    start_pt: Point = (float(start[0]), float(start[1]))
    goal_pt: Point = (float(goal[0]), float(goal[1]))

    # No SAMs = direct path
    if not sams:
        dist = _distance(start_pt, goal_pt)
        return [start_pt, goal_pt], dist, "direct (no SAMs)"

    # Normalize SAM format for wrap_sams (needs 'x', 'y', 'range')
    normalized_sams = []
    for sam in sams:
        pos = sam.get("pos") or sam.get("position")
        if pos is None and "x" in sam and "y" in sam:
            pos = [sam["x"], sam["y"]]
        if pos is None:
            continue

        normalized_sams.append({
            'x': float(pos[0]) if isinstance(pos, (list, tuple)) else float(pos),
            'y': float(pos[1]) if isinstance(pos, (list, tuple)) else float(pos),
            'range': float(sam.get("range", sam.get("radius", 15)))
        })

    if not normalized_sams:
        dist = _distance(start_pt, goal_pt)
        return [start_pt, goal_pt], dist, "direct (no valid SAMs)"

    # Convert SAMs to convex polygons
    polygons, _ = wrap_sams(normalized_sams, min_seg=2.0)

    if not polygons:
        dist = _distance(start_pt, goal_pt)
        return [start_pt, goal_pt], dist, "direct (no polygons)"

    if debug:
        print(f"Planning path: {start_pt} -> {goal_pt}")
        print(f"  {len(polygons)} polygon(s) from {len(normalized_sams)} SAM(s)")

    # Check if start or goal is inside any polygon
    for i, poly in enumerate(polygons):
        if _point_in_polygon(start_pt, poly):
            if debug:
                print(f"  START is inside polygon {i}")
            return [], float('inf'), f"INVALID: start inside polygon {i}"
        if _point_in_polygon(goal_pt, poly):
            if debug:
                print(f"  GOAL is inside polygon {i}")
            return [], float('inf'), f"INVALID: goal inside polygon {i}"

    # Check if direct path is clear
    blocking_idx = _find_blocking_polygon(start_pt, goal_pt, polygons)

    if blocking_idx is None:
        # Direct path is clear
        dist = _distance(start_pt, goal_pt)
        if debug:
            print(f"  Direct path clear, distance={dist:.2f}")
        return [start_pt, goal_pt], dist, "direct"

    if debug:
        print(f"  Direct path blocked by polygon {blocking_idx}")

    # Single polygon case - simpler algorithm
    if len(polygons) == 1:
        path, dist, method = _navigate_around_single_polygon(
            start_pt, goal_pt, polygons[0], debug=debug
        )
    else:
        # Multiple polygons - use iterative navigation
        path, dist, method = _navigate_multi_polygon(
            start_pt, goal_pt, polygons, debug=debug
        )

    # Validate the path doesn't cross any polygon
    if path and not _validate_path(path, polygons, debug=debug):
        if debug:
            print(f"  Path validation FAILED - segments cross polygons")
        return [], float('inf'), "INVALID: path crosses polygon"

    if debug:
        print(f"  Final path: {len(path)} points, distance={dist:.2f}, method={method}")

    return path, dist, method
