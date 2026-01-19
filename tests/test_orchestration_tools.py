"""
Test Mission Orchestration Tools

Simple tests to verify the orchestration tools work correctly.
"""

import sys
from pathlib import Path

# Add server to path
server_path = Path(__file__).resolve().parent / "server"
sys.path.insert(0, str(server_path))

from agents.mission_orchestration_tools import get_orchestrator


def test_basic_inspection():
    """Test basic inspection tools."""
    print("\n=== TEST: Basic Inspection ===")
    
    # Create simple environment
    env = {
        "airports": [
            {"id": "A1", "x": 0, "y": 0},
            {"id": "A2", "x": 100, "y": 100}
        ],
        "targets": [
            {"id": "T1", "x": 50, "y": 50, "priority": 10, "type": "a"},
            {"id": "T2", "x": 60, "y": 60, "priority": 7, "type": "b"},
            {"id": "T3", "x": 70, "y": 70, "priority": 10, "type": "c"},
            {"id": "T4", "x": 80, "y": 80, "priority": 5, "type": "a"}
        ],
        "sams": []
    }
    
    orchestrator = get_orchestrator()
    orchestrator.inspector.set_context(env)
    
    # Test target filtering
    all_targets = orchestrator.inspector.get_all_targets()
    print(f"✓ All targets: {len(all_targets)} targets")
    
    priority_10 = orchestrator.inspector.get_targets_by_priority(exact_priority=10)
    print(f"✓ Priority 10 targets: {priority_10}")
    assert len(priority_10) == 2, "Should find 2 priority-10 targets"
    
    type_a = orchestrator.inspector.get_targets_by_type(['a'])
    print(f"✓ Type A targets: {type_a}")
    assert len(type_a) == 2, "Should find 2 type-a targets"
    
    print("✅ Basic inspection tests passed!\n")


def test_trajectory_operations():
    """Test trajectory manipulation."""
    print("\n=== TEST: Trajectory Operations ===")
    
    orchestrator = get_orchestrator()
    
    # Create simple straight trajectory
    trajectory = [
        [0, 0],
        [25, 25],
        [50, 50],
        [75, 75],
        [100, 100]
    ]
    
    # Calculate distance
    distance = orchestrator.trajectory.calculate_trajectory_distance(trajectory)
    print(f"✓ Trajectory distance: {distance:.2f}")
    expected = 141.42  # sqrt(100^2 + 100^2)
    assert abs(distance - expected) < 1, f"Expected ~{expected}, got {distance}"
    
    # Split at 70 units
    split_result = orchestrator.trajectory.split_trajectory_at_distance(trajectory, 70)
    print(f"✓ Split at 70: prefix={len(split_result['prefix'])} pts, suffix={len(split_result['suffix'])} pts")
    print(f"  Split point: {split_result['split_point']}")
    
    assert split_result['split_point'] is not None, "Should have split point"
    
    print("✅ Trajectory operation tests passed!\n")


def test_constraint_helpers():
    """Test constraint helpers."""
    print("\n=== TEST: Constraint Helpers ===")
    
    orchestrator = get_orchestrator()
    
    # Test fuel calculation
    remaining = orchestrator.constraints.calculate_remaining_fuel(150, 80)
    print(f"✓ Remaining fuel: {remaining} (150 - 80 = 70)")
    assert remaining == 70, "Should have 70 fuel remaining"
    
    # Test loiter cost map
    targets = [
        {"id": "T1", "type": "a"},
        {"id": "T2", "type": "c"},
        {"id": "T3", "type": "c"},
        {"id": "T4", "type": "b"}
    ]
    
    loiter_costs = orchestrator.constraints.get_loiter_costs_for_target_types(
        targets,
        {"c": 20}
    )
    print(f"✓ Loiter costs: {loiter_costs}")
    assert len(loiter_costs) == 2, "Should have 2 type-c targets with loiter"
    assert loiter_costs.get("T2") == 20, "T2 should have 20 loiter steps"
    
    print("✅ Constraint helper tests passed!\n")


def test_segment_creation():
    """Test segment creation."""
    print("\n=== TEST: Segment Creation ===")
    
    orchestrator = get_orchestrator()
    
    env = {
        "airports": [{"id": "A1", "x": 0, "y": 0}],
        "targets": [{"id": "T1", "x": 50, "y": 50, "priority": 10, "type": "a"}],
        "sams": []
    }
    
    solution = {
        "routes": {
            "1": {
                "route": ["A1", "T1", "A1"],
                "distance": 100,
                "trajectory": [[0, 0], [50, 50], [0, 0]]
            }
        },
        "sequences": {"1": "A1,T1,A1"}
    }
    
    drone_configs = {
        "1": {"enabled": True, "fuel_budget": 150, "start_airport": "A1", "end_airport": "A1"}
    }
    
    # Create segment
    segment = orchestrator.segments.create_segment(
        index=0,
        solution=solution,
        env=env,
        drone_configs=drone_configs,
        cut_distance=50,
        cut_positions={"1": [50, 50]},
        frozen_targets=["T1"],
        active_targets=["T1"]
    )
    
    print(f"✓ Created segment: index={segment['index']}, cutDistance={segment['cutDistance']}")
    assert segment['index'] == 0, "Segment index should be 0"
    assert segment['cutDistance'] == 50, "Cut distance should be 50"
    
    # Create segmented mission
    mission = orchestrator.segments.create_segmented_mission(
        segments=[segment],
        base_env=env
    )
    
    print(f"✓ Created segmented mission: {mission['segment_count']} segments")
    assert mission['segment_count'] == 1, "Should have 1 segment"
    assert mission['is_segmented'] == True, "Should be marked as segmented"
    
    print("✅ Segment creation tests passed!\n")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("MISSION ORCHESTRATION TOOLS - TEST SUITE")
    print("="*60)
    
    try:
        test_basic_inspection()
        test_trajectory_operations()
        test_constraint_helpers()
        test_segment_creation()
        
        print("\n" + "="*60)
        print("✅ ALL TESTS PASSED!")
        print("="*60 + "\n")
        
    except Exception as e:
        print(f"\n❌ TEST FAILED: {e}")
        import traceback
        traceback.print_exc()
