#!/usr/bin/env python3
"""
Analyze T13's SSD and OSD metrics for each drone trajectory
"""

import json
import math

def euclidean_distance(p1, p2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def calculate_osd(point, line_start, line_end):
    """
    Calculate Origin-Segment Distance (OSD): perpendicular distance from point to line segment
    """
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    # Vector from line_start to line_end
    dx = x2 - x1
    dy = y2 - y1

    # If the line segment is just a point
    if dx == 0 and dy == 0:
        return euclidean_distance(point, line_start)

    # Calculate the parameter t for the projection of point onto the line
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

    # Clamp t to [0, 1] to stay on the segment
    t = max(0, min(1, t))

    # Find the closest point on the segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Return the distance from point to closest point on segment
    return euclidean_distance(point, (closest_x, closest_y))

def analyze_t13(env_file):
    """Analyze T13's SSD and OSD for all drone trajectories"""

    # Load environment
    with open(env_file, 'r') as f:
        env = json.load(f)

    # Find T13
    t13 = None
    for target in env['targets']:
        if target['id'] == 'T13':
            t13 = (target['x'], target['y'])
            break

    if not t13:
        print("T13 not found!")
        return

    print(f"\n{'='*70}")
    print(f"T13 Position: ({t13[0]:.3f}, {t13[1]:.3f})")
    print(f"T13 Priority: 5, Type: d")
    print(f"{'='*70}\n")

    # Get airports
    airports = {a['id']: (a['x'], a['y']) for a in env['airports']}

    # Analyze for each drone using drone_configs (the second set with 150 fuel budget)
    drone_configs = env.get('drone_configs', {})

    results = []

    for drone_id in sorted(drone_configs.keys()):
        config = drone_configs[drone_id]

        if not config['enabled']:
            continue

        start_airport = config['start_airport']
        end_airport = config['end_airport']

        start_pos = airports[start_airport]

        # Handle end airport
        if end_airport == '-':
            # Return to start
            end_pos = start_pos
            trajectory_label = f"{start_airport} → {start_airport}"
        else:
            end_pos = airports[end_airport]
            trajectory_label = f"{start_airport} → {end_airport}"

        # Calculate SSD (Start-Segment Distance): distance from drone start to T13
        ssd = euclidean_distance(start_pos, t13)

        # Calculate OSD (Origin-Segment Distance): perpendicular distance from T13 to trajectory line
        osd = calculate_osd(t13, start_pos, end_pos)

        # Calculate distance from end airport to T13
        esd = euclidean_distance(end_pos, t13)

        results.append({
            'drone_id': f"Drone {drone_id}",
            'trajectory': trajectory_label,
            'start': start_airport,
            'end': end_airport if end_airport != '-' else start_airport,
            'fuel_budget': config['fuel_budget'],
            'ssd': ssd,
            'osd': osd,
            'esd': esd,
            'start_pos': start_pos,
            'end_pos': end_pos
        })

    # Print results table
    print(f"{'Drone':<10} {'Trajectory':<15} {'Fuel':<8} {'SSD':<12} {'OSD':<12} {'ESD':<12}")
    print(f"{'-'*70}")

    for r in results:
        print(f"{r['drone_id']:<10} {r['trajectory']:<15} {r['fuel_budget']:<8} "
              f"{r['ssd']:<12.3f} {r['osd']:<12.3f} {r['esd']:<12.3f}")

    print(f"\n{'='*70}")
    print("\nMetric Definitions:")
    print("  SSD (Start-Segment Distance): Distance from drone's starting airport to T13")
    print("  OSD (Origin-Segment Distance): Perpendicular distance from T13 to drone's trajectory line")
    print("  ESD (End-Segment Distance): Distance from drone's ending airport to T13")
    print(f"{'='*70}\n")

    # Find best drone for T13
    best_osd = min(results, key=lambda x: x['osd'])
    best_ssd = min(results, key=lambda x: x['ssd'])

    print("Analysis:")
    print(f"  • Closest trajectory (lowest OSD): {best_osd['drone_id']} with OSD = {best_osd['osd']:.3f}")
    print(f"  • Nearest start airport (lowest SSD): {best_ssd['drone_id']} with SSD = {best_ssd['ssd']:.3f}")

    # Detailed geometry for each drone
    print(f"\n{'='*70}")
    print("Detailed Geometry:")
    print(f"{'='*70}\n")

    for r in results:
        print(f"{r['drone_id']} ({r['trajectory']}):")
        print(f"  Start Position: ({r['start_pos'][0]:.3f}, {r['start_pos'][1]:.3f})")
        print(f"  End Position:   ({r['end_pos'][0]:.3f}, {r['end_pos'][1]:.3f})")
        print(f"  T13 Position:   ({t13[0]:.3f}, {t13[1]:.3f})")
        print(f"  SSD = {r['ssd']:.3f} (distance from {r['start']} to T13)")
        print(f"  OSD = {r['osd']:.3f} (perpendicular distance to trajectory)")
        print(f"  ESD = {r['esd']:.3f} (distance from {r['end']} to T13)")
        print()

if __name__ == "__main__":
    analyze_t13('/Users/kamalali/Downloads/isr_env2512122217_1.json')
