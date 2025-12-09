from __future__ import annotations

from typing import List, Dict, Any, Tuple
import math


Point = Tuple[float, float]
Polygon = List[Point]


# ---------- basic utilities ----------

def _distance(a: Point, b: Point) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.hypot(dx, dy)


def _sample_circle(cx: float, cy: float, r: float, min_seg: float = 2.0) -> List[Point]:
    """
    Sample points around a circle such that chord lengths are ~>= min_seg.
    Returns a list of (x, y) points.
    """
    if r <= 0:
        return [(cx, cy)]

    # chord length s ≈ 2R sin(dθ/2) ~ min_seg
    # approximate dθ; clamp to reasonable range
    dtheta = min_seg / max(r, 1e-3)  # rough
    dtheta = max(dtheta, math.radians(5.0))
    dtheta = min(dtheta, math.radians(30.0))

    n_steps = max(8, int(math.ceil(2.0 * math.pi / dtheta)))
    theta_step = 2.0 * math.pi / n_steps

    pts: List[Point] = []
    for i in range(n_steps):
        theta = i * theta_step
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        pts.append((x, y))
    return pts


def _cross(o: Point, a: Point, b: Point) -> float:
    """
    2D cross product (OA x OB).
    >0: counter-clockwise
    <0: clockwise
    """
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _convex_hull(points: List[Point]) -> Polygon:
    """
    Monotone chain convex hull.
    Returns vertices in CCW order, without repeating the first point.
    """
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts

    lower: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and _cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: List[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # last point of each list is the starting point of the other list
    hull = lower[:-1] + upper[:-1]
    return hull


def _enforce_min_edge_length(poly: Polygon, min_seg: float = 2.0) -> Polygon:
    """
    Ensure consecutive vertices are at least min_seg apart by merging
    close ones. Returns a new polygon.
    """
    if len(poly) <= 2:
        return poly[:]

    pts = poly[:]
    changed = True
    max_iterations = len(poly) + 5  # Safety limit
    iteration = 0
    while changed and len(pts) > 2 and iteration < max_iterations:
        iteration += 1
        changed = False
        new_pts: List[Point] = []
        n = len(pts)
        i = 0
        while i < n:
            a = pts[i]
            b = pts[(i + 1) % n]
            if _distance(a, b) < min_seg:
                # merge a and b into their midpoint
                mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
                new_pts.append(mid)
                # skip b
                i += 2
                changed = True
            else:
                new_pts.append(a)
                i += 1

            # avoid runaway shrinking
            if len(new_pts) < 3 and i >= n:
                # fall back to original polygon if too small
                return poly[:]

        pts = new_pts

    return pts


# ---------- clustering utilities ----------

def _sams_overlap(sam1: Dict[str, Any], sam2: Dict[str, Any]) -> bool:
    """
    Check if two SAMs overlap (their circles intersect or touch).
    Returns True if distance between centers <= sum of radii.
    """
    x1 = float(sam1.get("x", 0.0))
    y1 = float(sam1.get("y", 0.0))
    r1 = float(sam1.get("range", sam1.get("radius", 0.0)))

    x2 = float(sam2.get("x", 0.0))
    y2 = float(sam2.get("y", 0.0))
    r2 = float(sam2.get("range", sam2.get("radius", 0.0)))

    dist = _distance((x1, y1), (x2, y2))
    return dist <= (r1 + r2)


def _cluster_overlapping_sams(sams: List[Dict[str, Any]]) -> List[List[Dict[str, Any]]]:
    """
    Cluster SAMs that overlap into groups using union-find approach.
    Returns a list of clusters, where each cluster is a list of SAMs.
    """
    if not sams:
        return []

    n = len(sams)
    # parent[i] = parent of SAM i in union-find
    parent = list(range(n))

    def find(x: int) -> int:
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x: int, y: int) -> None:
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    # Union SAMs that overlap
    for i in range(n):
        for j in range(i + 1, n):
            if _sams_overlap(sams[i], sams[j]):
                union(i, j)

    # Group SAMs by their root
    clusters_dict: Dict[int, List[Dict[str, Any]]] = {}
    for i in range(n):
        root = find(i)
        if root not in clusters_dict:
            clusters_dict[root] = []
        clusters_dict[root].append(sams[i])

    return list(clusters_dict.values())


# ---------- main entry point ----------

def wrap_sams(
    sams: List[Dict[str, Any]],
    min_seg: float = 2.0,
    *args: Any,
    **kwargs: Any,
) -> Tuple[List[Polygon], List[Dict[str, Any]]]:
    """
    Build wrapper polygons around SAM clusters.

    - Each SAM is assumed to have at least: 'x', 'y', 'range'.
    - SAMs that overlap (circles intersect) are grouped into clusters
    - Each cluster gets its own convex hull polygon
    - Isolated SAMs each get their own polygon

    Returns
    -------
    wrapped_polygons : list of polygons
        One convex polygon per cluster of overlapping SAMs.
        If sams is empty, this is [].
    metadata : list of SAM dicts (passed through)
    """
    if not sams:
        return [], sams

    # Cluster SAMs that overlap
    clusters = _cluster_overlapping_sams(sams)

    wrapped_polygons: List[Polygon] = []

    for cluster in clusters:
        # Sample points from all SAMs in this cluster
        all_points: List[Point] = []
        for s in cluster:
            cx = float(s.get("x", 0.0))
            cy = float(s.get("y", 0.0))
            r = float(s.get("range", s.get("radius", 0.0)))
            circle_pts = _sample_circle(cx, cy, r, min_seg=min_seg)
            all_points.extend(circle_pts)

        if not all_points:
            continue

        hull = _convex_hull(all_points)
        hull = _enforce_min_edge_length(hull, min_seg=min_seg)

        # If hull collapsed too much, skip this cluster
        if len(hull) < 3:
            continue

        wrapped_polygons.append(hull)

    return wrapped_polygons, sams
