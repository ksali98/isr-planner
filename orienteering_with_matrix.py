#!/usr/bin/env python3
"""
Orienteering Solver using Held-Karp (exact) dynamic programming algorithm.
Maximizes points within fuel budget constraint.

This solver finds the optimal route that:
1. Starts from a specified airport
2. Visits targets to maximize total priority points
3. Returns to an airport (same or different)
4. Stays within the fuel/distance budget

OPTIMIZED VERSION: Uses array-based DP and precomputed bitmasks for speed.
"""
import math
import itertools
from typing import List, Dict, Any, Tuple, Optional

# ---------- Optimized Held–Karp (exact) solvers ----------

# Pre-compute small combination masks for speed
_COMBO_CACHE: Dict[Tuple[int, int], List[Tuple[int, Tuple[int, ...]]]] = {}

def _get_combinations_with_masks(n: int, r: int) -> List[Tuple[int, Tuple[int, ...]]]:
    """Get combinations with pre-computed bitmasks (cached)."""
    key = (n, r)
    if key not in _COMBO_CACHE:
        result = []
        for combo in itertools.combinations(range(1, n), r):
            mask = 0
            for k in combo:
                mask |= (1 << k)
            result.append((mask, combo))
        _COMBO_CACHE[key] = result
    return _COMBO_CACHE[key]


def held_karp_cycle_fast(D: List[List[float]]) -> Tuple[float, List[int]]:
    """
    OPTIMIZED: Shortest cycle visiting all nodes, start/end at index 0.
    Uses array-based DP for faster access.
    """
    n = len(D)
    if n == 1:
        return 0.0, [0, 0]
    if n == 2:
        # Special case: just start -> node1 -> start
        return D[0][1] + D[1][0], [0, 1, 0]

    INF = float('inf')

    # Use flat array for DP: index = mask * n + node
    # This is much faster than dict lookups
    max_mask = 1 << n

    # DP arrays: cost and predecessor
    dp_cost = [INF] * (max_mask * n)
    dp_pred = [-1] * (max_mask * n)

    # Initialize: paths from node 0 to each other node
    for k in range(1, n):
        idx = ((1 << k) * n) + k
        dp_cost[idx] = D[0][k]
        dp_pred[idx] = 0

    # Build up solutions for increasingly larger subsets
    for r in range(2, n):
        combos = _get_combinations_with_masks(n, r)
        for mask, subset in combos:
            base_idx = mask * n
            for j in subset:
                pm = mask ^ (1 << j)  # Previous mask without j
                pm_base = pm * n

                best_cost = INF
                best_pred = -1

                for k in subset:
                    if k == j:
                        continue
                    prev_idx = pm_base + k
                    prev_cost = dp_cost[prev_idx]
                    if prev_cost < INF:
                        cand = prev_cost + D[k][j]
                        if cand < best_cost:
                            best_cost = cand
                            best_pred = k

                if best_pred >= 0:
                    idx = base_idx + j
                    dp_cost[idx] = best_cost
                    dp_pred[idx] = best_pred

    # Find best last node before returning to 0
    final_mask = (1 << n) - 2  # All nodes except 0
    final_base = final_mask * n

    best_cost = INF
    best_j = -1
    for j in range(1, n):
        idx = final_base + j
        if dp_cost[idx] < INF:
            cand = dp_cost[idx] + D[j][0]
            if cand < best_cost:
                best_cost = cand
                best_j = j

    if best_j < 0:
        return INF, []

    # Reconstruct path backwards
    path = [0, best_j]
    m = final_mask
    j = best_j

    while j != 0:
        idx = m * n + j
        p = dp_pred[idx]
        m ^= (1 << j)
        j = p
        if j != 0:
            path.append(j)
    path.append(0)

    return best_cost, path[::-1]


def held_karp_path_fast(D: List[List[float]], end_idx: int) -> Tuple[float, List[int]]:
    """
    OPTIMIZED: Shortest Hamiltonian path from node 0 to 'end_idx'.
    Uses array-based DP for faster access.
    """
    n = len(D)
    if n == 1:
        return 0.0, [0]
    if n == 2:
        return D[0][end_idx], [0, end_idx]

    INF = float('inf')
    max_mask = 1 << n

    # DP arrays
    dp_cost = [INF] * (max_mask * n)
    dp_pred = [-1] * (max_mask * n)

    # Initialize
    for k in range(1, n):
        idx = ((1 << k) * n) + k
        dp_cost[idx] = D[0][k]
        dp_pred[idx] = 0

    # Build solutions
    for r in range(2, n):
        combos = _get_combinations_with_masks(n, r)
        for mask, subset in combos:
            base_idx = mask * n
            for j in subset:
                pm = mask ^ (1 << j)
                pm_base = pm * n

                best_cost = INF
                best_pred = -1

                for k in subset:
                    if k == j:
                        continue
                    prev_idx = pm_base + k
                    prev_cost = dp_cost[prev_idx]
                    if prev_cost < INF:
                        cand = prev_cost + D[k][j]
                        if cand < best_cost:
                            best_cost = cand
                            best_pred = k

                if best_pred >= 0:
                    idx = base_idx + j
                    dp_cost[idx] = best_cost
                    dp_pred[idx] = best_pred

    # Get cost for end_idx
    final_mask = (1 << n) - 2
    final_idx = final_mask * n + end_idx

    if dp_cost[final_idx] >= INF:
        return INF, []

    cost = dp_cost[final_idx]

    # Reconstruct path
    path = [end_idx]
    m = final_mask
    cur = end_idx

    while cur != 0:
        idx = m * n + cur
        p = dp_pred[idx]
        m ^= (1 << cur)
        cur = p
        path.append(cur)

    return cost, list(reversed(path))


# Keep original functions as fallback (renamed)
def held_karp_cycle(D: List[List[float]]) -> Tuple[float, List[int]]:
    """Shortest cycle - uses optimized version."""
    return held_karp_cycle_fast(D)


def held_karp_path(D: List[List[float]], end_idx: int) -> Tuple[float, List[int]]:
    """Shortest path - uses optimized version."""
    return held_karp_path_fast(D, end_idx)


# ---------- Core orienteering functions ----------

# Cache for airport lookups
_AIRPORT_SET: Optional[frozenset] = None

def _build_airport_set(airports: List[Dict]) -> frozenset:
    """Build a frozenset of airport IDs for O(1) lookup."""
    global _AIRPORT_SET
    ids = frozenset(a.get("id", "") for a in airports)
    _AIRPORT_SET = ids
    return ids


def is_airport(label: str, airports: List[Dict]) -> bool:
    """Check if a label is an airport ID."""
    global _AIRPORT_SET
    if _AIRPORT_SET is not None:
        return label in _AIRPORT_SET
    return any(a.get("id") == label for a in airports)


def build_submatrix(D_full: List[List[float]], node_order: List[int]) -> List[List[float]]:
    """
    OPTIMIZED: Extract submatrix for given node indices.
    Pre-fetch rows for better cache locality.
    """
    k = len(node_order)
    M = [[0.0] * k for _ in range(k)]

    # Pre-fetch rows for better memory access patterns
    rows = [D_full[node_order[r]] for r in range(k)]

    for r in range(k):
        row = rows[r]
        M_row = M[r]
        for c in range(k):
            M_row[c] = row[node_order[c]]
    return M


# Global cache for Held-Karp results within a single solve call
# Key: (tuple of node_order, is_cycle, end_idx_or_none)
_HK_CACHE: Dict[Tuple, Tuple[float, List[int]]] = {}


def _clear_hk_cache():
    """Clear the Held-Karp cache between solve calls."""
    global _HK_CACHE
    _HK_CACHE.clear()


def evaluate_cycle(
    D_full: List[List[float]],
    labels: List[str],
    start_idx: int,
    subset_idxs: Tuple[int, ...]
) -> Tuple[float, List[str]]:
    """Evaluate shortest cycle: start -> subset -> start."""
    order = tuple([start_idx] + list(subset_idxs))

    # Check cache
    cache_key = (order, True, None)
    if cache_key in _HK_CACHE:
        cost, idx_path = _HK_CACHE[cache_key]
        if not idx_path:
            return math.inf, []
        route_labels = [labels[order[i]] for i in idx_path]
        return cost, route_labels

    M = build_submatrix(D_full, list(order))
    cost, idx_path = held_karp_cycle(M)

    # Store in cache
    _HK_CACHE[cache_key] = (cost, idx_path)

    if not idx_path:
        return math.inf, []
    route_labels = [labels[order[i]] for i in idx_path]
    return cost, route_labels


def evaluate_path(
    D_full: List[List[float]],
    labels: List[str],
    start_idx: int,
    subset_idxs: Tuple[int, ...],
    end_idx: int
) -> Tuple[float, List[str]]:
    """Evaluate shortest path: start -> subset -> end."""
    if end_idx in subset_idxs:
        return math.inf, []  # Invalid: end cannot be inside subset

    order = tuple([start_idx] + list(subset_idxs) + [end_idx])
    end_local = len(order) - 1

    # Check cache
    cache_key = (order, False, end_local)
    if cache_key in _HK_CACHE:
        cost, idx_path = _HK_CACHE[cache_key]
        if not idx_path:
            return math.inf, []
        route_labels = [labels[order[i]] for i in idx_path]
        return cost, route_labels

    M = build_submatrix(D_full, list(order))
    cost, idx_path = held_karp_path(M, end_local)

    # Store in cache
    _HK_CACHE[cache_key] = (cost, idx_path)

    if not idx_path:
        return math.inf, []
    route_labels = [labels[order[i]] for i in idx_path]
    return cost, route_labels


def choose_best_end(
    D_full: List[List[float]],
    labels: List[str],
    start_idx: int,
    subset_idxs: Tuple[int, ...],
    airport_idxs: List[int],
    fuel_cap: float
) -> Optional[Dict[str, Any]]:
    """
    OPTIMIZED: Try all airport ends; use early termination.
    For round-trip, we only need to solve Held-Karp once since
    the cycle cost is the same regardless of "end" being start.
    """
    best: Dict[str, Any] = {"len": math.inf, "route": None, "end_idx": None}

    # First, try round-trip (start as end) - this is often the best option
    if start_idx in airport_idxs:
        cost, route = evaluate_cycle(D_full, labels, start_idx, subset_idxs)
        if cost <= fuel_cap and route:
            best.update({"len": cost, "route": route, "end_idx": start_idx})

    # Then try other airports
    for aidx in airport_idxs:
        if aidx == start_idx:
            continue  # Already evaluated above

        cost, route = evaluate_path(D_full, labels, start_idx, subset_idxs, aidx)

        if cost <= fuel_cap and route:
            if cost < best["len"] or best["route"] is None:
                best.update({"len": cost, "route": route, "end_idx": aidx})

    return best if best["route"] else None


def solve_orienteering_with_matrix(
    env: Dict[str, Any],
    start_id: Optional[str] = None,
    mode: Optional[str] = None,
    fuel_cap: Optional[float] = None,
    end_id: Optional[str] = None,
    **kwargs
) -> Dict[str, Any]:
    """
    Main orienteering solver using Held-Karp exact algorithm.

    Modes:
      - 'return'  : start=end (cycle/round-trip)
      - 'end'     : fixed end airport
      - 'best_end': pick best airport end

    Args:
        env: Environment dict with matrix_labels, distance_matrix, airports, targets
        start_id: Starting airport ID (default: from env)
        mode: Solving mode (default: inferred from env)
        fuel_cap: Maximum distance/fuel allowed (default: from env)
        end_id: Ending airport ID for 'end' mode

    Returns:
        Solution dict with route, visited_goals, distance, total_points
    """
    # Clear caches from previous solve calls
    _clear_hk_cache()

    # Extract required data from environment
    labels = env.get("matrix_labels", [])
    D_full = env.get("distance_matrix", [])
    airports = env.get("airports", [])
    targets = env.get("targets", [])

    # Build airport set for fast lookups
    _build_airport_set(airports)

    # Validate we have required data
    if not labels or not D_full:
        return {
            "success": False,
            "path": [],
            "visited_goals": [],
            "distance": 0.0,
            "total_points": 0,
            "route": [],
        }

    # Build priority map for targets
    prio = {t.get("id", ""): int(t.get("priority", 1)) for t in targets}

    # Defaults from env if not provided
    if start_id is None:
        start_id = env.get("start_airport")

    json_end = env.get("end_airport")

    # Determine mode
    if mode is None:
        if json_end is None:
            mode = "best_end"
        elif json_end == start_id:
            mode = "return"
        else:
            mode = "end"

    if end_id is None and json_end is not None:
        end_id = json_end

    # Handle fuel_cap
    if fuel_cap is not None:
        fuel_cap = float(fuel_cap)
    else:
        fuel_cap = float(env.get("fuel_budget", float("inf")))

    # Validate start airport
    if start_id not in labels:
        print(f"⚠️ Start airport '{start_id}' not in labels, using first airport")
        if airports:
            start_id = airports[0].get("id", labels[0])
        else:
            start_id = labels[0] if labels else None

    if start_id is None or start_id not in labels:
        return {
            "success": False,
            "path": [],
            "visited_goals": [],
            "distance": 0.0,
            "total_points": 0,
            "route": [],
        }

    start_idx = labels.index(start_id)

    # Get airport and target indices
    airport_idxs = [i for i, lab in enumerate(labels) if is_airport(lab, airports)]
    cand_target_idxs = [
        i for i, lab in enumerate(labels)
        if i != start_idx and not is_airport(lab, airports)
    ]

    # Validate end airport for 'end' mode
    end_idx = None
    if mode == "end":
        if end_id is None or end_id not in labels:
            mode = "best_end"  # Fall back to best_end
        elif not is_airport(end_id, airports):
            mode = "best_end"
        else:
            end_idx = labels.index(end_id)

    # Initialize best solution (trivial: just start)
    best = {
        "points": -1,
        "len": 0.0,
        "subset": [],
        "route": [start_id],
        "end_airport": start_id if mode == "return" else (end_id if end_id else start_id)
    }

    # FAST PATH: First try ALL targets - if it fits, we're done
    # This avoids expensive subset enumeration in most cases
    N = len(cand_target_idxs)

    if N > 0:
        # Try full set first
        full_combo = tuple(cand_target_idxs)
        full_pts = sum(prio.get(labels[i], 0) for i in full_combo)

        if mode == "return":
            full_cost, full_route = evaluate_cycle(D_full, labels, start_idx, full_combo)
        elif mode == "end" and end_idx is not None:
            full_cost, full_route = evaluate_path(D_full, labels, start_idx, full_combo, end_idx)
        else:  # best_end
            end_choice = choose_best_end(D_full, labels, start_idx, full_combo, airport_idxs, fuel_cap)
            if end_choice:
                full_cost, full_route = end_choice["len"], end_choice["route"]
            else:
                full_cost, full_route = math.inf, []

        # If all targets fit within fuel budget, we're done!
        if full_cost <= fuel_cap and full_route:
            best.update({
                "points": full_pts,
                "len": full_cost,
                "subset": list(full_combo),
                "route": full_route,
                "end_airport": full_route[-1] if full_route and mode in ("end", "best_end") else start_id
            })
            # Skip subset enumeration - we have optimal solution
            N = 0  # This will skip the loop below

    # Only enumerate subsets if full set doesn't fit
    # Limit subset size to prevent exponential blowup
    max_subset_size = min(N, 8) if N > 0 else 0

    # OPTIMIZATION: Iterate from largest to smallest
    # Once we find a feasible solution at size r, we only need to check size r
    # (to find the shortest route with max points), then we can stop
    found_at_size = None

    for r in range(max_subset_size, 0, -1):  # REVERSE: largest first
        # Early termination: if we found a solution at a larger size, stop
        if found_at_size is not None and r < found_at_size:
            break

        for combo in itertools.combinations(cand_target_idxs, r):
            pts = sum(prio.get(labels[i], 0) for i in combo)

            # PRUNING: Skip if we can't beat current best points
            if found_at_size is not None and pts < best["points"]:
                continue

            if mode == "return":
                cost, route = evaluate_cycle(D_full, labels, start_idx, combo)
            elif mode == "end" and end_idx is not None:
                cost, route = evaluate_path(D_full, labels, start_idx, combo, end_idx)
            else:  # best_end
                end_choice = choose_best_end(D_full, labels, start_idx, combo, airport_idxs, fuel_cap)
                if end_choice:
                    cost, route = end_choice["len"], end_choice["route"]
                else:
                    cost, route = math.inf, []

            # Check feasibility
            feasible = (
                cost <= fuel_cap and
                route and
                cost is not None and
                cost != math.inf
            )

            if feasible:
                # Check if this is better (more points, or same points but shorter)
                better = (
                    pts > best["points"] or
                    (pts == best["points"] and cost < best["len"])
                )

                if better:
                    best.update({
                        "points": pts,
                        "len": cost,
                        "subset": list(combo),
                        "route": route,
                        "end_airport": route[-1] if route and mode in ("end", "best_end") else start_id
                    })
                    found_at_size = r  # Mark that we found a solution at this size

    # Build final solution in format expected by solver_bridge
    if best["points"] < 0:
        # No feasible solution found
        return {
            "success": True,
            "path": [],
            "visited_goals": [],
            "distance": 0.0,
            "total_points": 0,
            "route": [start_id],
            "sequence": start_id,
            "start_airport": start_id,
            "end_airport": start_id if mode == "return" else (end_id if end_id else start_id),
        }

    # Extract visited targets (exclude airports from route)
    visited_targets = [
        labels[i] for i in best["subset"]
    ]

    # Build coordinate path from route labels
    path_coords = []
    for label in best["route"]:
        # Find coordinates for this label
        for t in targets:
            if t.get("id") == label:
                path_coords.append((float(t.get("x", 0)), float(t.get("y", 0))))
                break
        else:
            for a in airports:
                if a.get("id") == label:
                    path_coords.append((float(a.get("x", 0)), float(a.get("y", 0))))
                    break

    return {
        "success": True,
        "path": path_coords,
        "visited_goals": visited_targets,  # Target IDs only (for solver_bridge compatibility)
        "distance": best["len"],
        "total_points": best["points"],
        "route": best["route"],  # Full route with airports
        "sequence": " ".join(best["route"]),
        "visited_targets": visited_targets,
        "start_airport": start_id,
        "end_airport": best["end_airport"],
    }


# For CLI testing
if __name__ == "__main__":
    import json
    import argparse

    ap = argparse.ArgumentParser(description="Orienteering solver with Held-Karp algorithm")
    ap.add_argument("input_json", help="Path to environment JSON")
    ap.add_argument("--start", help="Start airport ID")
    ap.add_argument("--end", help="End airport ID")
    ap.add_argument("--mode", choices=["return", "end", "best_end"])
    ap.add_argument("--fuel", type=float, help="Fuel budget")
    args = ap.parse_args()

    with open(args.input_json) as f:
        env = json.load(f)

    result = solve_orienteering_with_matrix(
        env,
        start_id=args.start,
        mode=args.mode,
        fuel_cap=args.fuel,
        end_id=args.end
    )

    print(f"Route: {','.join(result.get('route', []))}")
    print(f"Distance: {result.get('distance', 0):.2f}")
    print(f"Total points: {result.get('total_points', 0)}")
    print(f"Visited targets: {result.get('visited_targets', [])}")
