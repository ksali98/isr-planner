#!/usr/bin/env python3
"""Run a quick headless test of trajectory_swap_optimize in cascade mode.

This script builds a simple baseline solution (nearest-start assignment) from
an env JSON and calls trajectory_swap_optimize(..., auto_iterate=True, auto_regen=True).
It prints initial total distance, swaps applied, and final best distance.

Usage: PYTHONPATH=. python3 tools/run_swap_cascade_test.py /path/to/env.json
If no path provided, it tries the default used in diagnostics.
"""
import json
import math
import sys
from pathlib import Path

DEFAULT_ENV = Path('/Users/kamalali/Downloads/isr_env2601120011_N1_2.json')

# Helpers

def euclid(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_env(path):
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(p)
    data = json.loads(p.read_text())
    return data.get('env') or data


def build_baseline_solution(env):
    airports = {str(a['id']): (float(a['x']), float(a['y'])) for a in env.get('airports', [])}
    targets = {str(t['id']): (float(t['x']), float(t['y'])) for t in env.get('targets', [])}
    drone_configs = env.get('drone_configs', {})

    starts = {did: airports[cfg.get('start_airport')] for did, cfg in drone_configs.items()}
    assign = {did: [] for did in drone_configs.keys()}
    for tid, pos in targets.items():
        best = None
        bd = 1e9
        for did, spos in starts.items():
            d = euclid(pos, spos)
            if d < bd:
                bd = d
                best = did
        assign[best].append(tid)

    solution = {'routes': {}}
    labels = list(airports.keys()) + list(targets.keys())
    matrix = {'labels': labels, 'matrix': [[euclid((float(env['airports'][0]['x']), float(env['airports'][0]['y'])), (0,0)) for _ in labels] for _ in labels]}

    # Better matrix: all pairwise
    positions = {**airports, **targets}
    matrix = {'labels': labels, 'matrix': [[euclid(positions[f], positions[t]) for t in labels] for f in labels]}

    for did, tlist in assign.items():
        cfg = drone_configs.get(did, {})
        start = cfg.get('start_airport') or list(airports.keys())[0]
        end = cfg.get('end_airport') or start
        route = [start] + tlist + [end]
        traj = [positions[w] for w in route]
        dist = sum(euclid(traj[i], traj[i + 1]) for i in range(len(traj) - 1))
        solution['routes'][did] = {'route': route, 'sequence': ','.join(route), 'trajectory': traj, 'distance': dist, 'points': 0, 'fuel_budget': cfg.get('fuel_budget', 200), 'frozen_segments': []}

    return solution, matrix, drone_configs


def total_solution_distance(sol):
    return sum(r.get('distance', 0) for r in sol.get('routes', {}).values())


def main():
    env_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_ENV
    print('Loading env from', env_path)
    env = load_env(env_path)

    solution, matrix, drone_configs = build_baseline_solution(env)

    initial_dist = total_solution_distance(solution)
    print(f'Initial total distance (baseline): {initial_dist:.2f}')

    # Run optimizer
    try:
        from server.solver.post_optimizer import trajectory_swap_optimize
    except Exception as e:
        print('Error importing optimizer:', e)
        raise

    print('Calling trajectory_swap_optimize(..., auto_iterate=True, auto_regen=True)')
    res = trajectory_swap_optimize(solution=solution, env=env, drone_configs=drone_configs, distance_matrix=matrix, auto_iterate=True, auto_regen=True)

    swaps = res.get('swaps_made', [])
    print('Swaps applied:', len(swaps))
    for s in swaps[:20]:
        print(' ', s.get('target'), s.get('from_drone'), '->', s.get('to_drone'), f"savings={s.get('savings')}")
    if len(swaps) > 20:
        print('  ... (more swaps)')

    best_distance = res.get('best_distance') or total_solution_distance(res.get('routes', {}))
    print(f'Best distance reported: {best_distance:.2f}')

    after_dist = total_solution_distance({'routes': res.get('routes', {})})
    print(f'Computed total distance from returned routes: {after_dist:.2f}')

    if best_distance < initial_dist or after_dist < initial_dist:
        print('SUCCESS: distance improved')
    else:
        print('NOTE: no improvement detected')


if __name__ == '__main__':
    main()
