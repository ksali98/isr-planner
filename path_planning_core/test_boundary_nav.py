#!/usr/bin/env python3
"""Test script for boundary navigation."""
import sys
sys.path.insert(0, '/Users/kamalali/isr_projects')

from path_planning_core.boundary_navigation import plan_path

# Test 1: Direct path (no SAMs)
print('Test 1: Direct path (no SAMs)')
path, dist, method = plan_path((0, 0), (100, 100), [])
print(f'  Path: {path}')
print(f'  Distance: {dist:.2f}')
print(f'  Method: {method}')
print()

# Test 2: Path around single SAM
print('Test 2: Path around single SAM at (50, 50) with range 20')
sams = [{'pos': [50, 50], 'range': 20}]
path, dist, method = plan_path((0, 0), (100, 100), sams, debug=True)
print(f'  Path: {len(path)} points')
for i, p in enumerate(path):
    print(f'    [{i}] ({p[0]:.2f}, {p[1]:.2f})')
print(f'  Distance: {dist:.2f}')
print(f'  Method: {method}')
print()

# Test 3: Two overlapping SAMs
print('Test 3: Two overlapping SAMs')
sams = [{'pos': [50, 50], 'range': 15}, {'pos': [60, 60], 'range': 15}]
path, dist, method = plan_path((0, 0), (100, 100), sams, debug=True)
print(f'  Path: {len(path)} points')
for i, p in enumerate(path):
    print(f'    [{i}] ({p[0]:.2f}, {p[1]:.2f})')
print(f'  Distance: {dist:.2f}')
print(f'  Method: {method}')
print()

# Test 4: Path that doesn't cross SAM
print('Test 4: Path that does not cross SAM')
sams = [{'pos': [50, 50], 'range': 10}]
path, dist, method = plan_path((0, 0), (0, 100), sams, debug=True)
print(f'  Path: {path}')
print(f'  Distance: {dist:.2f}')
print(f'  Method: {method}')
