#!/usr/bin/env python3
import json
import math
import sys
sys.path.insert(0, 'server')
from solver import solver_bridge

def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def point_to_line_distance(point, line_start, line_end):
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return distance(point, line_start)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return distance(point, (closest_x, closest_y))

# Load environment
with open('test_env_t9.json') as f:
    env = json.load(f)

solution = solver_bridge.solve_mission_with_allocation(env, env['drone_configs'], post_optimize=False)

# Get T14 info
t14_pos = (21.573807565789473, 81.6015625)
d3_traj = solution['routes']['3']['trajectory']

print('T14 position:', t14_pos)
print('D3 trajectory (A3 → T14 → A3):')
for i, v in enumerate(d3_traj):
    print(f'  [{i}] ({v[0]:.2f}, {v[1]:.2f})')

# Calculate SSD - distance from T14 to the A3→A3 line
if len(d3_traj) == 3:
    prev_pos = d3_traj[0]  # A3
    next_pos = d3_traj[2]  # A3
    ssd = point_to_line_distance(t14_pos, prev_pos, next_pos)
    print(f'\nT14 SSD (distance to line A3→A3): {ssd:.2f}')
    print(f'Search radius: {ssd:.2f}')

    # Now check ALL trajectory vertices from all drones
    print('\n' + '='*80)
    print('Checking ALL trajectory vertices within search radius:')
    print('='*80)

    vertices_in_circle = []

    for drone_id in ['1', '2', '4', '5']:
        route_data = solution['routes'].get(drone_id, {})
        route = route_data.get('route', [])
        traj = route_data.get('trajectory', [])

        print(f'\nD{drone_id}: {" → ".join(str(wp) for wp in route)}')

        for i, v in enumerate(traj):
            v_pos = (v[0], v[1])
            dist = distance(t14_pos, v_pos)

            if dist <= ssd:
                vertices_in_circle.append({
                    'drone': drone_id,
                    'idx': i,
                    'pos': v_pos,
                    'dist': dist
                })
                print(f'  ✅ Vertex [{i}] ({v_pos[0]:.2f}, {v_pos[1]:.2f}): dist={dist:.2f} < {ssd:.2f}')

    if not vertices_in_circle:
        print('\n❌ No vertices found within search radius!')
    else:
        print('\n' + '='*80)
        print(f'Found {len(vertices_in_circle)} vertices within radius {ssd:.2f}:')
        print('='*80)
        for v in sorted(vertices_in_circle, key=lambda x: x['dist']):
            print(f"  D{v['drone']} vertex [{v['idx']}]: ({v['pos'][0]:.2f}, {v['pos'][1]:.2f}) - dist={v['dist']:.2f}")
