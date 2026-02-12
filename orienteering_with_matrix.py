#!/usr/bin/env python3
import json, math, itertools, argparse, time
from pathlib import Path

# ---------- Held–Karp (exact) solvers ----------
def held_karp_cycle(D):
    """Shortest cycle visiting all nodes, start/end at index 0."""
    n = len(D)
    if n == 1:
        return 0.0, [0,0]
    DP = {(1<<k, k): (D[0][k], 0) for k in range(1,n)}
    for r in range(2, n):
        for subset in itertools.combinations(range(1,n), r):
            mask = sum(1<<k for k in subset)
            for j in subset:
                pm = mask ^ (1<<j)
                best = (math.inf, None)
                for k in subset:
                    if k == j: 
                        continue
                    cand = DP[(pm,k)][0] + D[k][j]
                    if cand < best[0]:
                        best = (cand, k)
                DP[(mask,j)] = best
    mask = (1<<n) - 2
    best = (math.inf, None)
    for j in range(1,n):
        cand = DP[(mask,j)][0] + D[j][0]
        if cand < best[0]:
            best = (cand, j)
    cost, j = best
    # reconstruct
    path = [0, j]
    m = (1<<n) - 2
    while j != 0:
        _, p = DP[(m,j)]
        m ^= (1<<j)
        j = p
        if j != 0:
            path.append(j)
    path.append(0)
    return cost, path[::-1]

def held_karp_path(D, end_idx):
    """
    Shortest Hamiltonian path from node 0 to 'end_idx' visiting all nodes once.
    'end_idx' is an index in D (e.g., last node).
    """
    n = len(D)
    if n == 1:
        return 0.0, [0]
    DP = {(1<<k, k): (D[0][k], 0) for k in range(1,n)}
    for r in range(2, n):
        for subset in itertools.combinations(range(1,n), r):
            mask = sum(1<<k for k in subset)
            for j in subset:
                pm = mask ^ (1<<j)
                best = (math.inf, None)
                for k in subset:
                    if k == j: 
                        continue
                    cand = DP[(pm,k)][0] + D[k][j]
                    if cand < best[0]:
                        best = (cand, k)
                DP[(mask,j)] = best
    # reconstruct for fixed end_idx
    mask = (1 << n) - 2
    cost = DP[(mask, end_idx)][0]
    path = [end_idx]
    m = mask
    cur = end_idx
    while cur != 0:
        _, p = DP[(m, cur)]
        m ^= (1 << cur)
        cur = p
        path.append(cur)
    path = list(reversed(path))
    return cost, path

# ---------- Core orienteering over provided matrix ----------
def is_airport(label, airports):
    return any(a["id"] == label for a in airports)

def build_submatrix(D_full, node_order):
    """Return submatrix of D_full over 'node_order' indices, in that order."""
    k = len(node_order)
    M = [[0.0]*k for _ in range(k)]
    for r in range(k):
        fr = node_order[r]
        row_fr = D_full[fr]
        for c in range(k):
            M[r][c] = row_fr[node_order[c]]
    return M

def evaluate_cycle(D_full, labels, start_idx, subset_idxs):
    """Order: [start] + subset; shortest cycle via Held-Karp."""
    order = [start_idx] + list(subset_idxs)
    M = build_submatrix(D_full, order)
    cost, idx_path = held_karp_cycle(M)
    route_labels = [labels[order[i]] for i in idx_path]
    return cost, route_labels

def evaluate_path(D_full, labels, start_idx, subset_idxs, end_idx):
    """Order: [start] + subset + [end]; shortest path via Held-Karp path."""
    if end_idx in subset_idxs:
        return math.inf, []  # invalid; end cannot be inside subset
    order = [start_idx] + list(subset_idxs) + [end_idx]
    M = build_submatrix(D_full, order)
    end_local = len(order) - 1
    cost, idx_path = held_karp_path(M, end_local)
    route_labels = [labels[order[i]] for i in idx_path]
    return cost, route_labels

def choose_best_end(D_full, labels, start_idx, subset_idxs, airport_idxs, fuel_cap):
    """Try all airport ends (INCLUDING start for round-trip); pick best feasible."""
    best = {"len": math.inf, "route": None, "end_idx": None}

    for aidx in airport_idxs:
        # Don't skip start - we want to consider round trips too!
        if aidx == start_idx:
            # For round trip, use evaluate_cycle instead
            cost, route = evaluate_cycle(D_full, labels, start_idx, subset_idxs)
        else:
            cost, route = evaluate_path(D_full, labels, start_idx, subset_idxs, aidx)

        if cost <= fuel_cap and route:
            if cost < best["len"] or (best["route"] is None):
                best.update({"len": cost, "route": route, "end_idx": aidx})

    return best if best["route"] else None

def solve_orienteering_with_matrix(env, start_id=None, mode=None, fuel_cap=None, end_id=None):
    """
    mode:
      - 'return'  : start=end (cycle)
      - 'end'     : fixed end airport
      - 'best_end': pick best airport end (excl. start)
    If any of start_id / end_id / fuel_cap / mode are None, they are derived from JSON.
    """
    labels = env["matrix_labels"]
    D_full = env["distance_matrix"]
    airports = env["airports"]
    targets = env["targets"]
    prio = {t["id"]: int(t.get("priority", 1)) for t in targets}

    # =========================================================================
    # DEBUG: Dump raw inputs to diagnose D1_START / synthetic label issues
    # =========================================================================
    print("\n" + "="*70)
    print("🔬 RAW SOLVER INPUTS (for debugging)")
    print("="*70)
    print(f"  matrix_labels ({len(labels)} items): {labels}")
    print(f"  airports ({len(airports)} items): {[a.get('id', '?') for a in airports]}")
    print(f"  targets ({len(targets)} items): {[t.get('id', '?') for t in targets]}")
    print(f"  start_id param: {start_id}")
    print(f"  end_id param: {end_id}")
    print(f"  mode param: {mode}")
    print("="*70 + "\n")

    # Defaults from JSON if not provided
    if start_id is None:
        start_id = env.get("start_airport")
    json_end = env.get("end_airport")
    
    print(f"🔍 SOLVER MODE DETERMINATION:")
    print(f"  - start_id: {start_id}")
    print(f"  - json_end from env: {json_end}")
    print(f"  - end_id parameter: {end_id}")
    print(f"  - mode parameter: {mode}")
    
    if mode is None:
        if json_end is None:
            mode = "best_end"  # choose best end if JSON doesn't fix it
            print(f"  → Mode selected: 'best_end' (no end airport specified)")
        elif json_end == start_id:
            mode = "return"
            print(f"  → Mode selected: 'return' (end = start)")
        else:
            mode = "end"
            print(f"  → Mode selected: 'end' (fixed end airport: {json_end})")
    if end_id is None and json_end is not None:
        end_id = json_end
    # Handle fuel_cap parameter with proper precedence
    if fuel_cap is not None:
        # Explicit parameter takes precedence
        fuel_cap = float(fuel_cap)
        print(f"🔧 Using EXPLICIT fuel_cap parameter: {fuel_cap}")
    else:
        # Fall back to JSON value
        fuel_cap = float(env.get("fuel_budget", float("inf")))
        print(f"🔧 Using JSON fuel_budget: {fuel_cap}")
    
    print(f"🔧 ORIENTEERING SOLVER DEBUG:")
    print(f"  - Final fuel limit: {fuel_cap}")
    print(f"  - Environment has {len(labels)} locations")
    print(f"  - Mode: {mode}")
    print(f"  - End airport: {end_id if mode == 'end' else 'TO BE DETERMINED' if mode == 'best_end' else start_id}")

    # Validate basic fields
    if start_id not in labels:
        print(f"❌ ERROR: Start airport '{start_id}' not in matrix_labels", flush=True)
        print(f"   Available labels ({len(labels)}): {labels[:20]}{'...' if len(labels) > 20 else ''}", flush=True)
        raise ValueError(f"Start airport '{start_id}' not found in matrix_labels.")
    start_idx = labels.index(start_id)

    # =========================================================================
    # FIX: Use ID-based classification instead of "not airport => target"
    # This ensures synthetic nodes (D1_START, CUT_1, etc.) are never targets
    # =========================================================================

    # Build explicit ID sets for airports and targets
    airport_ids = {a["id"] for a in airports if "id" in a}
    target_ids = {t["id"] for t in targets if "id" in t}

    # Build indices using explicit ID membership (not negation)
    airport_idxs = [i for i, lab in enumerate(labels) if lab in airport_ids]
    cand_target_idxs = [i for i, lab in enumerate(labels) if lab in target_ids and i != start_idx]

    # =========================================================================
    # DEBUG: Diagnose D1_START / synthetic label misclassification
    # =========================================================================

    # Labels that look like airports but are NOT in env["airports"]
    suspicious_airport_like = [lab for lab in labels if lab.startswith("A") or lab.endswith("_START")]
    missing_airport_like = [lab for lab in suspicious_airport_like if lab not in airport_ids]

    print("🔍 AIRPORT DEBUG")
    print("  airport_ids:", sorted(list(airport_ids))[:50], ("..." if len(airport_ids) > 50 else ""))
    print("  labels airports (by is_airport):", [labels[i] for i in airport_idxs])
    print("  suspicious airport-like labels:", suspicious_airport_like)
    print("  airport-like labels missing from env['airports']:", missing_airport_like)
    print(f"  start_id={start_id}, is_airport(start_id)={is_airport(start_id, airports)}")

    # STRICT GUARD: Reject unknown labels (neither airport nor target)
    # This catches bugs where synthetic starts aren't properly added to airports list
    unknown_labels = [lab for lab in labels if lab not in airport_ids and lab not in target_ids]
    if unknown_labels:
        raise ValueError(
            f"matrix_labels contains unknown nodes (neither airport nor target): {unknown_labels}. "
            f"Fix env['airports'] (include synthetic starts like D1_START) or env['targets']. "
            f"Current airports: {sorted(airport_ids)}, targets: {sorted(target_ids)}"
        )

    # =========================================================================
    # Endpoint candidates for best_end: airports only, optionally filtered
    # Note: We do NOT exclude start_idx here - returning to start can be valid!
    # If start_id is a synthetic cut node, it won't be in airport_ids anyway.
    # =========================================================================
    best_end_airport_idxs = airport_idxs  # Default: all airports

    valid_end_airport_ids = env.get("valid_end_airports", None)
    if mode == "best_end" and valid_end_airport_ids:
        # Filter to only valid endpoints, but ensure they're real airports
        valid_end_airport_ids = [aid for aid in valid_end_airport_ids if aid in airport_ids]
        valid_end_idxs = [i for i in airport_idxs if labels[i] in valid_end_airport_ids]
        print(f"🔧 Filtering endpoints: {len(airport_idxs)} total airports → {len(valid_end_idxs)} valid endpoints")
        print(f"🔧 Valid endpoint airports: {[labels[i] for i in valid_end_idxs]}")
        # Use filtered list for best_end mode, fallback to all airports if empty
        best_end_airport_idxs = valid_end_idxs if valid_end_idxs else airport_idxs

    print(f"🔧 Available airports (by ID): {[labels[i] for i in airport_idxs]}")
    print(f"🔧 Start index: {start_idx} ({start_id})")
    print(f"🔧 Target candidates (by ID, excludes start): {[labels[i] for i in cand_target_idxs]}")

    # For mode=end, validate the provided/JSON end_id
    end_idx = None
    if mode == "end":
        if end_id is None:
            raise ValueError("mode 'end' requires an end airport (from JSON or --end).")
        if end_id not in labels:
            raise ValueError(f"End airport '{end_id}' not in matrix_labels.")
        if not is_airport(end_id, airports):
            raise ValueError(f"End node '{end_id}' is not an airport.")
        end_idx = labels.index(end_id)

    best = {
        "points": -1,
        "len": math.inf,
        "subset": [],
        "route": [start_id],  # trivial fallback
        "end_airport": start_id if mode == "return" else (end_id if end_id else None)
    }

    # Enumerate all non-empty subsets of targets
    N = len(cand_target_idxs)
    for r in range(1, N+1):
        for combo in itertools.combinations(cand_target_idxs, r):
            pts = sum(prio.get(labels[i], 0) for i in combo)

            if mode == "return":
                cost, route = evaluate_cycle(D_full, labels, start_idx, combo)
            elif mode == "end":
                cost, route = evaluate_path(D_full, labels, start_idx, combo, end_idx)
            else:  # best_end
                # Use filtered airport list to exclude synthetic starts from endpoints
                end_choice = choose_best_end(D_full, labels, start_idx, combo, best_end_airport_idxs, fuel_cap)
                if end_choice:
                    cost, route = end_choice["len"], end_choice["route"]
                else:
                    cost, route = math.inf, []

            # STRICT FUEL CONSTRAINT CHECK
            feasible = (cost <= fuel_cap) and route and (cost is not None) and (cost != math.inf)
            # Only print for feasible solutions or when we find a new best (reduce verbosity)
            if feasible:
                better = (pts > best["points"]) or \
                         (pts == best["points"] and cost < best["len"]) or \
                         (pts == best["points"] and cost == best["len"] and r > len(best["subset"]))
                if better:
                    # TRIPLE CHECK: Absolutely ensure constraint is satisfied
                    if cost > fuel_cap or cost is None or cost == math.inf:
                        print(f"🚨 CRITICAL BUG: Trying to accept infeasible solution!")
                        print(f"🚨 cost={cost}, fuel_cap={fuel_cap}, cost > fuel_cap = {cost > fuel_cap}")
                        continue  # Skip this solution
                        
                    print(f"    ✅ NEW BEST: {pts} points, {cost:.1f} distance (within {fuel_cap} budget)")
                    best.update({
                        "points": pts,
                        "len": cost,
                        "subset": list(combo),
                        "route": route,
                        "end_airport": route[-1] if mode in ("end","best_end") else start_id
                    })
            # Removed verbose rejected logging - only show feasible solutions

    # Nothing fit: return start->end route (0 points, no targets)
    if best["points"] < 0:
        actual_end = start_id if mode=="return" else (end_id if end_id else start_id)
        # Route must always have at least [start, end] for trajectory generation
        if actual_end and actual_end != start_id:
            route_for_no_targets = [start_id, actual_end]
            # Calculate the direct distance for start->end
            start_idx_for_dist = labels.index(start_id)
            end_idx_for_dist = labels.index(actual_end) if actual_end in labels else start_idx_for_dist
            direct_dist = D_full[start_idx_for_dist][end_idx_for_dist] if end_idx_for_dist != start_idx_for_dist else 0.0
        else:
            route_for_no_targets = [start_id]
            direct_dist = 0.0
        return {
            "sequence": " ".join(route_for_no_targets),
            "route": route_for_no_targets,
            "distance": direct_dist,
            "total_points": 0,
            "visited_targets": [],
            "start_airport": start_id,
            "end_airport": actual_end,
        }

    final_solution = {
        "sequence": " ".join(best["route"]),
        "route": best["route"],
        "distance": best["len"],
        "total_points": best["points"],
        "visited_targets": [labels[i] for i in best["subset"]],
        "start_airport": start_id,
        "end_airport": best["end_airport"]
    }
    
    print(f"🔧 FINAL SOLUTION CHECK:")
    print(f"  - Distance: {final_solution['distance']}")
    print(f"  - Fuel cap: {fuel_cap}")
    print(f"  - CONSTRAINT VIOLATION: {final_solution['distance'] > fuel_cap}")
    
    if final_solution['distance'] > fuel_cap:
        print(f"🚨 CRITICAL BUG: Orienteering solver is returning infeasible solution!")
        print(f"🚨 This should NEVER happen - solver has a bug!")
        print(f"🚨 FORCING FALLBACK TO MINIMAL SOLUTION...")
        
        # Force a minimal solution that definitely fits
        minimal_distance = D_full[start_idx][labels.index(best["end_airport"])] if best["end_airport"] in labels else 0
        if minimal_distance > fuel_cap:
            # Even start->end is too far, return start only
            return {
                "sequence": start_id,
                "route": [start_id],
                "distance": 0.0,
                "total_points": 0,
                "visited_targets": [],
                "start_airport": start_id,
                "end_airport": start_id
            }
        else:
            # Return just start->end
            return {
                "sequence": f"{start_id} {best['end_airport']}",
                "route": [start_id, best['end_airport']],
                "distance": minimal_distance,
                "total_points": 0,
                "visited_targets": [],
                "start_airport": start_id,
                "end_airport": best['end_airport']
            }
    
    return final_solution

# ---------- CLI ----------
def main():
    ap = argparse.ArgumentParser(
        description="Maximize points within fuel using a SAM-aware distance matrix (single drone)."
    )
    ap.add_argument("input_json", help="Path to env JSON (must include matrix_labels + distance_matrix)")
    # All flags are OPTIONAL; JSON provides defaults:
    ap.add_argument("--start", help="Override start airport id (default: from JSON)")
    ap.add_argument("--end", help="Override end airport id (default: from JSON; omit for best_end)")
    ap.add_argument("--mode", choices=["return","end","best_end"],
                    help="Override mode (default inferred from JSON start/end)")
    ap.add_argument("--fuel", type=float, help="Override fuel budget (default: from JSON)")
    ap.add_argument("--outdir", help="Output folder (default: alongside input JSON)")
    args = ap.parse_args()

    with open(args.input_json) as f:
        env = json.load(f)

    res = solve_orienteering_with_matrix(
        env,
        start_id=args.start,
        mode=args.mode,
        fuel_cap=args.fuel,
        end_id=args.end
    )

    # Print copy/paste summary
    print(",".join(res["route"]))
    print(f"Distance: {round(res['distance'], 3)}")
    print(f"Total points: {res['total_points']}")
    print(f"Visited targets: {res['visited_targets']}")
    print(f"Start: {res['start_airport']}  End: {res['end_airport']}")

    # Save solved JSON
    p = Path(args.input_json)
    outdir = Path(args.outdir) if args.outdir else p.parent
    outdir.mkdir(parents=True, exist_ok=True)
    out_path = outdir / f"{p.stem}_OPT_SOLVED_{time.strftime('%Y%m%d_%H%M%S')}.json"

    out_env = dict(env)
    out_env["sequence"] = res["sequence"]
    out_env["path"] = res["route"]
    out_env["path_distance"] = res["distance"]
    out_env["total_points"] = res["total_points"]
    out_env["visited_targets_orienteering"] = res["visited_targets"]
    out_env["start_airport"] = res["start_airport"]
    out_env["end_airport"] = res["end_airport"]

    with open(out_path, "w") as f:
        json.dump(out_env, f, indent=2)
    print(f"Saved: {out_path}")

if __name__ == "__main__":
    main()
