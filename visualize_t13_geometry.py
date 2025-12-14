#!/usr/bin/env python3
"""
Visualize T13's geometric relationship with drone trajectories
"""

import json
import math

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def calculate_projection_point(point, line_start, line_end):
    """Find the closest point on line segment to the given point"""
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return line_start, 0.0

    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t_clamped = max(0, min(1, t))

    closest_x = x1 + t_clamped * dx
    closest_y = y1 + t_clamped * dy

    return (closest_x, closest_y), t

# Load environment
with open('/Users/kamalali/Downloads/isr_env2512122217_1.json', 'r') as f:
    env = json.load(f)

# Get T13 and airports
t13 = (21.574, 30.944)
airports = {a['id']: (a['x'], a['y']) for a in env['airports']}

print("\n" + "="*80)
print("DETAILED OSD CALCULATION FOR T13")
print("="*80)

# Focus on Drone 5 (A5 → A1) as the interesting case
print("\nDrone 5: A5 → A1")
print("-" * 80)

start = airports['A5']  # (49.156, 84.562)
end = airports['A1']    # (18.235, 39.562)

print(f"\nStart Point (A5):  ({start[0]:.3f}, {start[1]:.3f})")
print(f"End Point (A1):    ({end[0]:.3f}, {end[1]:.3f})")
print(f"Target T13:        ({t13[0]:.3f}, {t13[1]:.3f})")

# Calculate trajectory vector
dx = end[0] - start[0]
dy = end[1] - start[1]
print(f"\nTrajectory Vector: dx={dx:.3f}, dy={dy:.3f}")
print(f"Trajectory Length: {math.sqrt(dx**2 + dy**2):.3f}")

# Calculate projection parameter t
numerator = (t13[0] - start[0]) * dx + (t13[1] - start[1]) * dy
denominator = dx * dx + dy * dy
t = numerator / denominator

print(f"\nProjection Calculation:")
print(f"  Numerator:   [(T13_x - A5_x) * dx + (T13_y - A5_y) * dy]")
print(f"             = [({t13[0]:.3f} - {start[0]:.3f}) * {dx:.3f} + ({t13[1]:.3f} - {start[1]:.3f}) * {dy:.3f}]")
print(f"             = [{t13[0]-start[0]:.3f} * {dx:.3f} + {t13[1]-start[1]:.3f} * {dy:.3f}]")
print(f"             = [{(t13[0]-start[0])*dx:.3f} + {(t13[1]-start[1])*dy:.3f}]")
print(f"             = {numerator:.3f}")

print(f"\n  Denominator: [dx² + dy²]")
print(f"             = [{dx:.3f}² + {dy:.3f}²]")
print(f"             = [{dx**2:.3f} + {dy**2:.3f}]")
print(f"             = {denominator:.3f}")

print(f"\n  t (unclamped) = {numerator:.3f} / {denominator:.3f} = {t:.6f}")

t_clamped = max(0, min(1, t))
print(f"  t (clamped to [0,1]) = {t_clamped:.6f}")

if t_clamped == 0:
    print(f"  → Closest point is at START (A5)")
elif t_clamped == 1:
    print(f"  → Closest point is at END (A1)")
else:
    print(f"  → Closest point is {t_clamped*100:.1f}% along trajectory from A5 to A1")

# Calculate closest point
closest_x = start[0] + t_clamped * dx
closest_y = start[1] + t_clamped * dy
closest = (closest_x, closest_y)

print(f"\nClosest Point on Trajectory:")
print(f"  x = {start[0]:.3f} + {t_clamped:.6f} * {dx:.3f} = {closest_x:.3f}")
print(f"  y = {start[1]:.3f} + {t_clamped:.6f} * {dy:.3f} = {closest_y:.3f}")
print(f"  Closest Point: ({closest_x:.3f}, {closest_y:.3f})")

# Calculate OSD
osd = euclidean_distance(t13, closest)
print(f"\nOSD Calculation:")
print(f"  OSD = √[(T13_x - closest_x)² + (T13_y - closest_y)²]")
print(f"      = √[({t13[0]:.3f} - {closest_x:.3f})² + ({t13[1]:.3f} - {closest_y:.3f})²]")
print(f"      = √[{(t13[0]-closest_x):.3f}² + {(t13[1]-closest_y):.3f}²]")
print(f"      = √[{(t13[0]-closest_x)**2:.3f} + {(t13[1]-closest_y)**2:.3f}]")
print(f"      = √{(t13[0]-closest_x)**2 + (t13[1]-closest_y)**2:.3f}")
print(f"      = {osd:.3f}")

# SSD calculation
ssd = euclidean_distance(start, t13)
print(f"\nSSD Calculation (for comparison):")
print(f"  SSD = √[(T13_x - A5_x)² + (T13_y - A5_y)²]")
print(f"      = √[({t13[0]:.3f} - {start[0]:.3f})² + ({t13[1]:.3f} - {start[1]:.3f})²]")
print(f"      = {ssd:.3f}")

print(f"\nSummary for Drone 5:")
print(f"  SSD = {ssd:.3f} (distance from starting airport A5 to T13)")
print(f"  OSD = {osd:.3f} (perpendicular distance from trajectory to T13)")

print("\n" + "="*80)
print("WHY OSD < SSD for Drone 5:")
print("="*80)
print(f"""
The trajectory from A5 to A1 passes VERY CLOSE to T13.
When we project T13 onto the line A5→A1, the closest point is at A1 (t=1.0).
T13 is essentially "near the end" of the trajectory.

Since T13 is close to A1 ({osd:.3f} units away), and the trajectory ends at A1,
the perpendicular distance (OSD={osd:.3f}) is much smaller than the distance
from the starting point A5 (SSD={ssd:.3f}).

This is why Swap Closer would favor assigning T13 to Drone 5!
""")

print("="*80)
print("COMPARISON: Drone 1 (A1 → A1)")
print("="*80)

start1 = airports['A1']
print(f"\nDrone 1: A1 → A1 (return to same airport)")
print(f"Start = End = ({start1[0]:.3f}, {start1[1]:.3f})")
print(f"T13 = ({t13[0]:.3f}, {t13[1]:.3f})")

ssd1 = euclidean_distance(start1, t13)
print(f"\nSince start = end (no actual trajectory line):")
print(f"  SSD = OSD = distance to A1 = {ssd1:.3f}")
print(f"\nThis is why Drone 1 has SSD = OSD = 9.243")

print("\n" + "="*80)
