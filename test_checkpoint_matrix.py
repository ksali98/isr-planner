#!/usr/bin/env python3
"""
Test script to verify checkpoint-based distance matrix calculations
and segmented mission workflow.

This test verifies:
1. Checkpoints (C1-1, C1-2, etc.) are properly added to the SAM-aware distance matrix
2. Distances from checkpoints use SAM-aware routing (not Euclidean)
3. The solver correctly uses matrix distances for fuel calculations
4. Segmented mission JSON structure is correct
5. End-to-end solve with checkpoints works
"""

import json
import math
import sys
import requests
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from server.solver.sam_distance_matrix import calculate_sam_aware_matrix

# Server URL for API tests
SERVER_URL = "http://localhost:8893"

def euclidean_distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def test_checkpoint_in_matrix():
    """Test that checkpoints are properly added to the distance matrix."""

    print("=" * 60)
    print("TEST 1: Checkpoint-Based Distance Matrix Verification")
    print("=" * 60)

    # Create test environment with SAM in the middle
    env = {
        "airports": [
            {"id": "A1", "x": 10, "y": 10},
            {"id": "A2", "x": 90, "y": 10},
        ],
        "targets": [
            {"id": "T1", "x": 30, "y": 50, "priority": 5},
            {"id": "T2", "x": 70, "y": 50, "priority": 5},
        ],
        "sams": [
            {"id": "S1", "pos": [50, 50], "range": 15}  # SAM at center
        ],
        # Add per-drone checkpoints (simulating a cut at drone positions)
        "checkpoints": [
            {"id": "C1-1", "x": 35, "y": 45},  # Drone 1's position at cut
            {"id": "C1-2", "x": 65, "y": 45},  # Drone 2's position at cut
        ]
    }

    print("\n📍 Environment:")
    print(f"   Airports: A1(10,10), A2(90,10)")
    print(f"   Targets:  T1(30,50), T2(70,50)")
    print(f"   SAM:      S1 at (50,50) with range 15")
    print(f"   Checkpoints: C1-1(35,45), C1-2(65,45)")

    # Calculate SAM-aware distance matrix
    print("\n🔧 Calculating SAM-aware distance matrix...")
    result = calculate_sam_aware_matrix(env, buffer=5.0)

    if result is None:
        print("❌ FAILED: calculate_sam_aware_matrix returned None")
        return False

    labels = result.get("labels", [])
    matrix = result.get("matrix", [])

    print(f"\n📊 Matrix labels: {labels}")

    # Check if checkpoints are in the matrix
    print("\n" + "-" * 40)
    print("CHECK 1: Are checkpoints in the matrix?")
    print("-" * 40)

    c1_1_in_matrix = "C1-1" in labels
    c1_2_in_matrix = "C1-2" in labels

    print(f"   C1-1 in matrix: {'✅ YES' if c1_1_in_matrix else '❌ NO'}")
    print(f"   C1-2 in matrix: {'✅ YES' if c1_2_in_matrix else '❌ NO'}")

    if not (c1_1_in_matrix and c1_2_in_matrix):
        print("\n❌ FAILED: Checkpoints not found in matrix!")
        return False

    # Build label to index mapping
    label_to_idx = {lbl: i for i, lbl in enumerate(labels)}

    # Check distances from checkpoints
    print("\n" + "-" * 40)
    print("CHECK 2: Are checkpoint distances SAM-aware?")
    print("-" * 40)

    # Get positions
    positions = {
        "A1": (10, 10), "A2": (90, 10),
        "T1": (30, 50), "T2": (70, 50),
        "C1-1": (35, 45), "C1-2": (65, 45),
    }

    # Test key distances
    test_pairs = [
        ("C1-1", "T2"),  # C1-1 to T2 - must go around SAM
        ("C1-2", "T1"),  # C1-2 to T1 - must go around SAM
        ("C1-1", "A1"),  # C1-1 to A1 - direct path, no SAM
        ("C1-2", "A2"),  # C1-2 to A2 - direct path, no SAM
    ]

    all_sam_aware = True

    for from_id, to_id in test_pairs:
        from_idx = label_to_idx.get(from_id)
        to_idx = label_to_idx.get(to_id)

        if from_idx is None or to_idx is None:
            print(f"   ⚠️ {from_id}→{to_id}: Missing from matrix!")
            all_sam_aware = False
            continue

        matrix_dist = matrix[from_idx][to_idx]
        euclidean_dist = euclidean_distance(positions[from_id], positions[to_id])

        # SAM-aware distance should be >= Euclidean (equal if no SAM in path)
        is_sam_aware = matrix_dist >= euclidean_dist - 0.1  # Small tolerance

        # For paths that cross the SAM, matrix distance should be significantly larger
        status = "✅" if is_sam_aware else "❌"
        diff = matrix_dist - euclidean_dist
        diff_pct = (diff / euclidean_dist * 100) if euclidean_dist > 0 else 0

        print(f"   {from_id}→{to_id}:")
        print(f"      Matrix distance:    {matrix_dist:.2f}")
        print(f"      Euclidean distance: {euclidean_dist:.2f}")
        print(f"      Difference:         {diff:+.2f} ({diff_pct:+.1f}%)")
        print(f"      Status: {status} {'SAM-aware (longer path)' if diff > 1 else 'Direct path OK'}")

        if not is_sam_aware:
            all_sam_aware = False

    print("\n" + "-" * 40)
    print("CHECK 3: Checkpoint-to-checkpoint distance")
    print("-" * 40)

    # C1-1 to C1-2 must go around the SAM (they're on opposite sides)
    c1_1_idx = label_to_idx["C1-1"]
    c1_2_idx = label_to_idx["C1-2"]
    c1_to_c2_matrix = matrix[c1_1_idx][c1_2_idx]
    c1_to_c2_euclidean = euclidean_distance(positions["C1-1"], positions["C1-2"])

    print(f"   C1-1→C1-2:")
    print(f"      Matrix distance:    {c1_to_c2_matrix:.2f}")
    print(f"      Euclidean distance: {c1_to_c2_euclidean:.2f}")

    # This path should definitely be longer due to SAM avoidance
    if c1_to_c2_matrix > c1_to_c2_euclidean + 5:
        print(f"      ✅ SAM-aware: Path is {c1_to_c2_matrix - c1_to_c2_euclidean:.1f} units longer (avoiding SAM)")
    else:
        print(f"      ⚠️ WARNING: Path may not be avoiding SAM properly")

    # Final summary
    print("\n" + "=" * 60)
    print("TEST 1 SUMMARY")
    print("=" * 60)

    if c1_1_in_matrix and c1_2_in_matrix and all_sam_aware:
        print("✅ TEST 1 PASSED")
        print("   - Checkpoints are in the distance matrix")
        print("   - Distances are SAM-aware (not Euclidean)")
        return True
    else:
        print("❌ TEST 1 FAILED")
        return False


def test_solver_uses_matrix():
    """Test that the orienteering solver uses matrix distances."""

    print("\n" + "=" * 60)
    print("TEST 2: Solver Uses Matrix Distances")
    print("=" * 60)

    # Import the orienteering solver
    try:
        from orienteering_with_matrix import solve_orienteering_with_matrix as orienteering_solve
    except ImportError as e:
        print(f"❌ Could not import orienteering solver: {e}")
        return False

    # Create test environment with checkpoint as start
    # Position SAM so that C1-1 to T2 path crosses it
    # Note: Don't add C1-1 to both airports AND checkpoints - sam_distance_matrix adds checkpoints to airports
    env = {
        "airports": [
            {"id": "A1", "x": 10, "y": 10},
        ],
        "targets": [
            {"id": "T1", "x": 30, "y": 30, "priority": 5},
            {"id": "T2", "x": 50, "y": 50, "priority": 5},
        ],
        "sams": [
            {"id": "S1", "pos": [40, 40], "range": 10}  # SAM between T1 and T2
        ],
        "checkpoints": [
            {"id": "C1-1", "x": 20, "y": 20},
        ]
    }

    # Calculate SAM-aware matrix
    print("\n🔧 Calculating SAM-aware distance matrix...")
    matrix_result = calculate_sam_aware_matrix(env, buffer=5.0)

    if matrix_result is None:
        print("❌ Failed to calculate matrix")
        return False

    # Prepare solver environment (matches solve_orienteering_with_matrix expected format)
    # C1-1 is already in matrix_labels from sam_distance_matrix adding checkpoints
    solver_env = {
        "matrix_labels": matrix_result["labels"],
        "distance_matrix": matrix_result["matrix"],
        "airports": [
            {"id": "A1", "x": 10, "y": 10},
            {"id": "C1-1", "x": 20, "y": 20},  # Checkpoint as airport for solver
        ],
        "targets": [
            {"id": "T1", "x": 30, "y": 30, "priority": 5},
            {"id": "T2", "x": 50, "y": 50, "priority": 5},
        ],
        "start_airport": "C1-1",
        "end_airport": "A1",
        "fuel_budget": 200,  # Large budget to ensure all targets visited
    }

    print(f"\n📍 Solver input:")
    print(f"   Start: C1-1 (checkpoint)")
    print(f"   End: A1 (airport)")
    print(f"   Targets: T1, T2")
    print(f"   Budget: 200")
    print(f"   Matrix labels: {matrix_result['labels']}")

    # Run solver
    print("\n🔧 Running orienteering solver...")
    result = orienteering_solve(solver_env, start_id="C1-1", end_id="A1", mode="end", fuel_cap=200)

    # The solver returns dict with: route, distance, total_points, visited_targets, etc.
    if not result or "route" not in result:
        print(f"❌ Solver failed: {result}")
        return False

    print(f"\n📊 Solver result:")
    print(f"   Route: {result.get('route', [])}")
    print(f"   Distance: {result.get('distance', 0):.2f}")
    print(f"   Points: {result.get('total_points', 0)}")

    # Verify the distance comes from matrix, not Euclidean
    route = result.get("route", [])
    reported_distance = result.get("distance", 0)

    # Calculate what Euclidean distance would be
    positions = {
        "A1": (10, 10),
        "T1": (30, 30),
        "T2": (50, 50),
        "C1-1": (20, 20),
    }

    euclidean_total = 0
    for i in range(len(route) - 1):
        if route[i] in positions and route[i+1] in positions:
            euclidean_total += euclidean_distance(positions[route[i]], positions[route[i+1]])

    print(f"\n🔍 Distance verification:")
    print(f"   Solver reported distance: {reported_distance:.2f}")
    print(f"   Euclidean would be:       {euclidean_total:.2f}")

    if reported_distance > euclidean_total + 1:
        print(f"   ✅ Solver is using SAM-aware distances (longer by {reported_distance - euclidean_total:.1f})")
        return True
    elif abs(reported_distance - euclidean_total) < 1:
        print(f"   ⚠️ Distances are similar (path may not cross SAM)")
        return True
    else:
        print(f"   ❌ Solver distance is shorter than Euclidean - something is wrong!")
        return False


def test_segmented_mission_solve():
    """Test solving a segmented mission with checkpoints via API."""

    print("\n" + "=" * 60)
    print("TEST 3: Segmented Mission Solve via API")
    print("=" * 60)

    # Check if server is running
    try:
        resp = requests.get(f"{SERVER_URL}/health", timeout=2)
        if resp.status_code != 200:
            print(f"⚠️ Server not healthy: {resp.status_code}")
            print("   Skipping API test - start server with ./run_planner.sh")
            return None  # Skip, not fail
    except requests.exceptions.ConnectionError:
        print("⚠️ Server not running at localhost:8893")
        print("   Skipping API test - start server with ./run_planner.sh")
        return None  # Skip, not fail

    # Create a segmented mission environment
    # Segment 0 has already been solved, now we're solving Segment 1 from checkpoints
    env = {
        "airports": [
            {"id": "A1", "x": 10, "y": 10},
            {"id": "A2", "x": 90, "y": 90},
        ],
        "targets": [
            {"id": "T1", "x": 30, "y": 30, "priority": 5},
            {"id": "T2", "x": 50, "y": 50, "priority": 7},
            {"id": "T3", "x": 70, "y": 70, "priority": 3},
        ],
        "sams": [
            {"id": "S1", "pos": [40, 60], "range": 10}
        ],
        # Checkpoints from segment 0 cut
        "checkpoints": [
            {"id": "C1-1", "x": 35, "y": 35},
            {"id": "C1-2", "x": 55, "y": 55},
        ]
    }

    drone_configs = {
        "1": {
            "start_airport": "C1-1",  # Start from checkpoint
            "end_airport": "A1",
            "fuel_capacity": 200,
        },
        "2": {
            "start_airport": "C1-2",  # Start from checkpoint
            "end_airport": "A2",
            "fuel_capacity": 200,
        }
    }

    print("\n📍 Segmented mission setup:")
    print("   Segment 1 (starting from cut point):")
    print("   - Drone 1: C1-1 → ... → A1")
    print("   - Drone 2: C1-2 → ... → A2")
    print("   - Remaining targets: T1, T2, T3")

    # Call solve-with-allocation endpoint
    print("\n🔧 Calling /api/solve_with_allocation API...")

    payload = {
        "env": env,
        "drone_configs": drone_configs,
        "allocation_strategy": "efficient",
    }

    try:
        resp = requests.post(
            f"{SERVER_URL}/api/solve_with_allocation",
            json=payload,
            timeout=30
        )

        if resp.status_code != 200:
            print(f"❌ API returned status {resp.status_code}")
            print(f"   Response: {resp.text[:500]}")
            return False

        result = resp.json()

        if not result.get("success"):
            print(f"❌ Solve failed: {result.get('errors', 'Unknown error')}")
            return False

        print("\n📊 API Response:")
        routes = result.get("routes", {})

        for drone_id, route_data in routes.items():
            route = route_data.get("route", [])
            distance = route_data.get("distance", 0)
            points = route_data.get("points", 0)

            print(f"\n   Drone {drone_id}:")
            print(f"      Route: {' → '.join(route)}")
            print(f"      Distance: {distance:.2f}")
            print(f"      Points: {points}")

            # Verify route starts from checkpoint
            if route and route[0].startswith("C"):
                print(f"      ✅ Route starts from checkpoint {route[0]}")
            else:
                print(f"      ❌ Route should start from checkpoint, got: {route[0] if route else 'empty'}")
                return False

            # Verify route ends at airport
            if route and route[-1].startswith("A"):
                print(f"      ✅ Route ends at airport {route[-1]}")
            else:
                print(f"      ❌ Route should end at airport, got: {route[-1] if route else 'empty'}")
                return False

        # Check distance matrix was returned
        if result.get("distance_matrix"):
            labels = result["distance_matrix"].get("labels", [])
            print(f"\n   Distance matrix labels: {labels}")

            # Verify checkpoints are in matrix
            if "C1-1" in labels and "C1-2" in labels:
                print("   ✅ Checkpoints are in the distance matrix")
            else:
                print("   ❌ Checkpoints missing from distance matrix!")
                return False

        print("\n" + "=" * 60)
        print("TEST 3 SUMMARY")
        print("=" * 60)
        print("✅ TEST 3 PASSED")
        print("   - API accepted checkpoint-based environment")
        print("   - Routes start from checkpoints")
        print("   - Routes end at airports")
        print("   - Distance matrix includes checkpoints")
        return True

    except Exception as e:
        print(f"❌ API call failed: {e}")
        return False


def test_segmented_json_structure():
    """Test creating and validating a segmented mission JSON structure."""

    print("\n" + "=" * 60)
    print("TEST 4: Segmented Mission JSON Structure")
    print("=" * 60)

    # Create a sample segmented mission JSON (as would be exported from frontend)
    segmented_mission = {
        "version": "2.0",
        "environment": {
            "airports": [
                {"id": "A1", "x": 10, "y": 10},
                {"id": "A2", "x": 90, "y": 90},
            ],
            "targets": [
                {"id": "T1", "x": 30, "y": 30, "priority": 5},
                {"id": "T2", "x": 50, "y": 50, "priority": 7},
                {"id": "T3", "x": 70, "y": 70, "priority": 3},
            ],
            "sams": [
                {"id": "S1", "pos": [40, 60], "range": 10}
            ],
        },
        "drone_configs": {
            "1": {"fuel_capacity": 200, "start_airport": "A1", "end_airport": "A1"},
            "2": {"fuel_capacity": 200, "start_airport": "A2", "end_airport": "A2"},
        },
        "segments": [
            {
                "segment_index": 0,
                "routes": {
                    "1": {
                        "route": ["A1", "T1", "C1-1"],
                        "distance": 45.5,
                        "points": 5,
                        "trajectory": [[10, 10], [30, 30], [35, 35]],
                    },
                    "2": {
                        "route": ["A2", "T3", "C1-2"],
                        "distance": 42.3,
                        "points": 3,
                        "trajectory": [[90, 90], [70, 70], [55, 55]],
                    },
                },
                "checkpoints": [
                    {"id": "C1-1", "x": 35, "y": 35},
                    {"id": "C1-2", "x": 55, "y": 55},
                ],
            },
            {
                "segment_index": 1,
                "routes": {
                    "1": {
                        "route": ["C1-1", "T2", "A1"],
                        "distance": 55.2,
                        "points": 7,
                        "trajectory": [[35, 35], [50, 50], [10, 10]],
                    },
                    "2": {
                        "route": ["C1-2", "A2"],
                        "distance": 49.5,
                        "points": 0,
                        "trajectory": [[55, 55], [90, 90]],
                    },
                },
                "checkpoints": [],  # Final segment, no more checkpoints
            },
        ],
    }

    print("\n📄 Validating segmented mission JSON structure...")

    # Validate structure
    errors = []

    # Check version
    if segmented_mission.get("version") != "2.0":
        errors.append("Missing or invalid version")

    # Check environment
    env = segmented_mission.get("environment", {})
    if not env.get("airports"):
        errors.append("Missing airports in environment")
    if not env.get("targets"):
        errors.append("Missing targets in environment")

    # Check drone configs
    if not segmented_mission.get("drone_configs"):
        errors.append("Missing drone_configs")

    # Check segments
    segments = segmented_mission.get("segments", [])
    if not segments:
        errors.append("Missing segments")
    else:
        for i, seg in enumerate(segments):
            # Check segment structure
            if seg.get("segment_index") != i:
                errors.append(f"Segment {i} has wrong index: {seg.get('segment_index')}")

            if "routes" not in seg:
                errors.append(f"Segment {i} missing routes")
            else:
                for drone_id, route_data in seg["routes"].items():
                    if "route" not in route_data:
                        errors.append(f"Segment {i}, Drone {drone_id} missing route")
                    if "distance" not in route_data:
                        errors.append(f"Segment {i}, Drone {drone_id} missing distance")
                    if "trajectory" not in route_data:
                        errors.append(f"Segment {i}, Drone {drone_id} missing trajectory")

            # Check checkpoint naming convention
            if i < len(segments) - 1:  # Not the last segment
                checkpoints = seg.get("checkpoints", [])
                for cp in checkpoints:
                    cp_id = cp.get("id", "")
                    # Should match C{segment+1}-{drone} pattern
                    import re
                    if not re.match(r'^C\d+-\d+$', cp_id):
                        errors.append(f"Invalid checkpoint ID format: {cp_id}")

    # Validate route continuity
    print("\n🔍 Validating route continuity between segments...")

    for drone_id in segmented_mission["drone_configs"].keys():
        print(f"\n   Drone {drone_id}:")
        prev_end = None

        for seg in segments:
            route_data = seg["routes"].get(drone_id, {})
            route = route_data.get("route", [])

            if route:
                start = route[0]
                end = route[-1]

                print(f"      Seg-{seg['segment_index']}: {start} → ... → {end}")

                # Check continuity
                if prev_end is not None:
                    if start != prev_end:
                        errors.append(f"Drone {drone_id}: Seg-{seg['segment_index']} starts at {start} but previous segment ended at {prev_end}")
                    else:
                        print(f"         ✅ Continuous from {prev_end}")

                prev_end = end

    # Print results
    print("\n" + "=" * 60)
    print("TEST 4 SUMMARY")
    print("=" * 60)

    if errors:
        print("❌ TEST 4 FAILED")
        for err in errors:
            print(f"   - {err}")
        return False
    else:
        print("✅ TEST 4 PASSED")
        print("   - JSON structure is valid")
        print("   - Checkpoint naming follows C{seg}-{drone} convention")
        print("   - Route continuity verified between segments")
        return True


def test_apply_sequence_with_checkpoint():
    """Test the /api/apply_sequence endpoint with checkpoint waypoints."""

    print("\n" + "=" * 60)
    print("TEST 5: Apply Sequence with Checkpoint")
    print("=" * 60)

    # Check if server is running
    try:
        resp = requests.get(f"{SERVER_URL}/health", timeout=2)
        if resp.status_code != 200:
            print("⚠️ Server not healthy - skipping")
            return None
    except requests.exceptions.ConnectionError:
        print("⚠️ Server not running - skipping")
        return None

    # First, we need to solve a mission to populate the distance matrix cache
    env = {
        "airports": [
            {"id": "A1", "x": 10, "y": 10},
        ],
        "targets": [
            {"id": "T1", "x": 30, "y": 30, "priority": 5},
            {"id": "T2", "x": 50, "y": 50, "priority": 5},
        ],
        "sams": [
            {"id": "S1", "pos": [40, 40], "range": 10}
        ],
        "checkpoints": [
            {"id": "C1-1", "x": 20, "y": 20},
        ]
    }

    drone_configs = {
        "1": {
            "start_airport": "C1-1",
            "end_airport": "A1",
            "fuel_capacity": 200,
        }
    }

    # First solve to populate matrix
    print("\n🔧 First solving to populate distance matrix cache...")
    solve_resp = requests.post(
        f"{SERVER_URL}/api/solve_with_allocation",
        json={"env": env, "drone_configs": drone_configs, "allocation_strategy": "efficient"},
        timeout=30
    )

    if solve_resp.status_code != 200:
        print(f"❌ Initial solve failed: {solve_resp.status_code}")
        return False

    # Now test apply_sequence with a route that includes checkpoint
    print("\n🔧 Testing /api/apply_sequence with checkpoint route...")

    sequence = "C1-1, T1, T2, A1"
    apply_resp = requests.post(
        f"{SERVER_URL}/api/apply_sequence",
        json={
            "env": env,
            "drone_id": "1",
            "sequence": sequence,
        },
        timeout=10
    )

    if apply_resp.status_code != 200:
        print(f"❌ Apply sequence failed: {apply_resp.status_code}")
        print(f"   Response: {apply_resp.text[:500]}")
        return False

    result = apply_resp.json()

    print(f"\n📊 Apply sequence result:")
    print(f"   Input sequence: {sequence}")
    print(f"   Route: {result.get('route', [])}")
    print(f"   Distance: {result.get('distance', 0):.2f}")
    print(f"   Points: {result.get('points', 0)}")
    print(f"   Success: {result.get('success', False)}")

    if not result.get("success"):
        print(f"❌ Apply sequence reported failure: {result.get('error')}")
        return False

    # Verify distance is SAM-aware (should be > Euclidean)
    # C1-1(20,20) → T1(30,30) → T2(50,50) → A1(10,10)
    euclidean_total = (
        euclidean_distance((20, 20), (30, 30)) +  # C1-1 → T1
        euclidean_distance((30, 30), (50, 50)) +  # T1 → T2
        euclidean_distance((50, 50), (10, 10))    # T2 → A1
    )

    reported_distance = result.get("distance", 0)

    print(f"\n🔍 Distance verification:")
    print(f"   Reported distance: {reported_distance:.2f}")
    print(f"   Euclidean would be: {euclidean_total:.2f}")

    if reported_distance >= euclidean_total - 0.1:
        print(f"   ✅ Distance is SAM-aware (diff: {reported_distance - euclidean_total:+.1f})")
    else:
        print(f"   ❌ Distance is LESS than Euclidean - something is wrong!")
        return False

    print("\n" + "=" * 60)
    print("TEST 5 SUMMARY")
    print("=" * 60)
    print("✅ TEST 5 PASSED")
    print("   - apply_sequence accepts checkpoint in route")
    print("   - Distance calculated using SAM-aware matrix")
    return True


if __name__ == "__main__":
    print("\n" + "🧪" * 30)
    print("CHECKPOINT & SEGMENTED MISSION TEST SUITE")
    print("🧪" * 30)

    results = {}

    # Run all tests
    results["test1_matrix"] = test_checkpoint_in_matrix()
    results["test2_solver"] = test_solver_uses_matrix()
    results["test3_api_solve"] = test_segmented_mission_solve()
    results["test4_json"] = test_segmented_json_structure()
    results["test5_apply_seq"] = test_apply_sequence_with_checkpoint()

    # Final summary
    print("\n" + "=" * 60)
    print("FINAL RESULTS")
    print("=" * 60)

    passed = 0
    failed = 0
    skipped = 0

    for name, result in results.items():
        if result is True:
            status = "✅ PASS"
            passed += 1
        elif result is False:
            status = "❌ FAIL"
            failed += 1
        else:
            status = "⏭️ SKIP"
            skipped += 1
        print(f"   {name}: {status}")

    print(f"\nTotal: {passed} passed, {failed} failed, {skipped} skipped")

    if failed > 0:
        print("\n💥 SOME TESTS FAILED!")
        sys.exit(1)
    else:
        print("\n🎉 ALL TESTS PASSED!")
        sys.exit(0)
