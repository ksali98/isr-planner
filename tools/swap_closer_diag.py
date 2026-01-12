#!/usr/bin/env python3
"""
Diagnostic helper for Swap Closer (TrajectorySwapOptimizer).

Usage:
  python tools/swap_closer_diag.py /path/to/env.json [optional_solution.json]

If no solution is provided, the script builds a simple initial solution by assigning
each target to the nearest drone's start airport and creating route: A -> targets -> A
with straight-line trajectories (no SAM avoidance). The script then runs the optimizer
in diagnostic mode and prints, for each target, SSD, best OSD found, insertion cost,
fuel check, capability check, and final decision.
"""
import json
import math
import sys
from pathlib import Path

# Import the optimizer from the server package
import server.solver.post_optimizer as post_opt

def euclid(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def build_distance_matrix(labels, positions):
    matrix = []
    for f in labels:
        row = []
        for t in labels:
            row.append(euclid(positions[f], positions[t]))
        matrix.append(row)
    return {"labels": labels, "matrix": matrix}

def build_baseline_solution(env):
    # Assign each target to nearest drone start airport
    airports = {str(a['id']): (float(a['x']), float(a['y'])) for a in env.get('airports', [])}
    targets = {str(t['id']): (float(t['x']), float(t['y'])) for t in env.get('targets', [])}
    drone_configs = env.get('drone_configs', {})

    # Map drone id to start airport position
    drone_starts = {}
    for did, cfg in drone_configs.items():
        start = cfg.get('start_airport')
        if start and start in airports:
            drone_starts[did] = airports[start]
        else:
            # fallback: first airport
            drone_starts[did] = list(airports.values())[0]

    # Nearest assignment
    assign = {did: [] for did in drone_configs.keys()}
    for tid, pos in targets.items():
        best_d = None
        best_d_dist = float('inf')
        for did, spos in drone_starts.items():
            d = euclid(pos, spos)
            if d < best_d_dist:
                best_d_dist = d
                best_d = did
        assign[best_d].append(tid)

    # Build routes
    solution = {'routes': {}}
    labels = []
    positions = {}
    # Include airports and targets in labels/positions
    for aid, apos in airports.items():
        labels.append(aid)
        positions[aid] = apos
    for tid, tpos in targets.items():
        labels.append(tid)
        positions[tid] = tpos

    for did, tlist in assign.items():
        cfg = drone_configs.get(did, {})
        start = cfg.get('start_airport') or list(airports.keys())[0]
        end = cfg.get('end_airport') or start
        route = [start] + tlist + [end]
        # Trajectory: straight line vertices matching waypoints
        traj = [positions[w] for w in route]
        # Distance from matrix later; set placeholder
        solution['routes'][did] = {
            'route': route,
            'sequence': ','.join(route),
            'trajectory': traj,
            'distance': sum(euclid(traj[i], traj[i+1]) for i in range(len(traj)-1)),
            'points': 0,
            'fuel_budget': cfg.get('fuel_budget', 200),
            'frozen_segments': []
        }

    return solution, {'labels': labels, 'positions': positions}


def main():
    if len(sys.argv) < 2:
        print('Usage: python tools/swap_closer_diag.py /path/to/env.json [solution.json]')
        sys.exit(1)

    env_path = Path(sys.argv[1])
    if not env_path.exists():
        print('Env file not found:', env_path)
        sys.exit(1)

    data = json.loads(env_path.read_text())
    env = data.get('env') or data

    # Load or build solution
    if len(sys.argv) >= 3:
        sol_path = Path(sys.argv[2])
        if not sol_path.exists():
            print('Solution file not found:', sol_path)
            sys.exit(1)
        solution = json.loads(sol_path.read_text())
    else:
        solution, posinfo = build_baseline_solution(env)

    # Build distance matrix (Euclidean) for labels present
    # Collect labels from env (airports + targets)
    labels = [str(a['id']) for a in env.get('airports', [])] + [str(t['id']) for t in env.get('targets', [])]
    positions = {str(a['id']): (float(a['x']), float(a['y'])) for a in env.get('airports', [])}
    for t in env.get('targets', []):
        positions[str(t['id'])] = (float(t['x']), float(t['y']))

    matrix_data = build_distance_matrix(labels, positions)

    # Set matrix
    opt = post_opt.TrajectorySwapOptimizer()
    opt.set_distance_matrix(matrix_data)

    # Run a single-pass optimize and capture results
    result = opt._optimize_single(solution, env, env.get('drone_configs', {}))

    print('\n=== SWAP CLOSER DIAGNOSTIC SUMMARY ===')
    print('Swaps made:', result.get('swaps_made'))

    # For targets, print SSD and best OSD computations by reusing logic
    # We'll emulate the core of _optimize_single but produce per-target diagnostic lines
    # This is a minimal reimplementation for clarity

    # Build waypoint_positions
    waypoint_positions = {}
    for t in env.get('targets', []):
        waypoint_positions[str(t['id'])] = (float(t['x']), float(t['y']))
    for a in env.get('airports', []):
        waypoint_positions[str(a['id'])] = (float(a['x']), float(a['y']))

    # Build drone trajectories mapping from solution
    drone_trajectories = {}
    for did, route_data in solution.get('routes', {}).items():
        traj = route_data.get('trajectory', [])
        drone_trajectories[did] = [(float(p[0]), float(p[1])) for p in traj]

    # Helper access to internal methods
    p2l = opt._point_to_line_distance

    print('\nPer-target diagnostics (SSD, best OSD, insertion_cost, fuel_ok, allowed):')

    targets_list = [str(t['id']) for t in env.get('targets', [])]
    for tid in targets_list:
        # find current drone and index
        cur_drone = None
        cur_idx = None
        for did, r in solution['routes'].items():
            if tid in r['route']:
                cur_drone = did
                cur_idx = r['route'].index(tid)
                break
        if cur_drone is None:
            print(f'{tid}: not in any route')
            continue

        traj = drone_trajectories.get(cur_drone, [])
        if tid not in waypoint_positions:
            print(f'{tid}: no position')
            continue
        tpos = waypoint_positions[tid]

        # find traj index matching target
        traj_idx = None
        for i, v in enumerate(traj):
            if abs(v[0]-tpos[0])<1e-6 and abs(v[1]-tpos[1])<1e-6:
                traj_idx = i
                break
        if traj_idx is None:
            print(f'{tid}: could not match trajectory vertex')
            continue

        # prev/next
        if traj_idx == 0:
            prev_pos = traj[0]
            next_pos = traj[1] if len(traj) > 1 else traj[0]
        elif traj_idx >= len(traj)-1:
            prev_pos = traj[-2] if len(traj) > 1 else traj[-1]
            next_pos = traj[-1]
        else:
            prev_pos = traj[traj_idx-1]
            next_pos = traj[traj_idx+1]

        ssd = p2l(tpos, prev_pos, next_pos)

        best_osd = float('inf')
        best_segment = None
        best_drone = None
        best_insert_cost = None
        # Check all other drones' trajectory subsegments
        for od, odata in solution['routes'].items():
            oroute = odata['route']
            otraj = drone_trajectories.get(od, [])
            for j in range(len(oroute)-1):
                # find start/end traj idx
                start_id = str(oroute[j])
                end_id = str(oroute[j+1])
                start_pos = waypoint_positions.get(start_id)
                end_pos = waypoint_positions.get(end_id)
                if not start_pos or not end_pos:
                    continue
                # find indices in otraj
                start_idx = None
                end_idx = None
                for ti, tv in enumerate(otraj):
                    if abs(tv[0]-start_pos[0])<1e-6 and abs(tv[1]-start_pos[1])<1e-6:
                        start_idx = ti
                        break
                if start_idx is None:
                    continue
                for ti in range(start_idx+1, len(otraj)):
                    tv = otraj[ti]
                    if abs(tv[0]-end_pos[0])<1e-6 and abs(tv[1]-end_pos[1])<1e-6:
                        end_idx = ti
                        break
                if end_idx is None:
                    continue
                # check subsegments
                for ti in range(start_idx, end_idx):
                    segs = (otraj[ti], otraj[ti+1])
                    # bounding box quick test
                    min_x = min(segs[0][0], segs[1][0]) - ssd
                    max_x = max(segs[0][0], segs[1][0]) + ssd
                    min_y = min(segs[0][1], segs[1][1]) - ssd
                    max_y = max(segs[0][1], segs[1][1]) + ssd
                    if not (min_x <= tpos[0] <= max_x and min_y <= tpos[1] <= max_y):
                        continue
                    osd = p2l(tpos, segs[0], segs[1])
                    if osd < best_osd:
                        best_osd = osd
                        best_drone = od
                        best_segment = j
                        # insertion cost using euclid
                        d_start = euclid(positions[start_id], positions[tid])
                        d_end = euclid(positions[tid], positions[end_id])
                        d_direct = euclid(positions[start_id], positions[end_id])
                        best_insert_cost = d_start + d_end - d_direct
        allowed = True
        # fuel check: simple
        fuel_ok = True
        od = best_drone
        if best_insert_cost is not None and od is not None and od != cur_drone:
            cur_dist = solution['routes'][od]['distance']
            fuel_budget = solution['routes'][od].get('fuel_budget', 200)
            if cur_dist + best_insert_cost > fuel_budget:
                fuel_ok = False

        moved = any(s['target']==tid for s in result.get('swaps_made', []))

        print(f"{tid}: SSD={ssd:.2f}, best_osd={best_osd if best_osd!=float('inf') else 'N/A'}, best_drone={best_drone}, insert_cost={best_insert_cost}, fuel_ok={fuel_ok}, moved={moved}")

if __name__ == '__main__':
    main()
