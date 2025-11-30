"""
Target Allocator for Multi-Drone Mission Planning

Distributes targets among available drones based on various strategies
before individual drone route optimization.

This is a LangGraph-compatible tool.
"""

import math
from typing import Dict, Any, List, Optional, Set, Tuple
from enum import Enum


class AllocationStrategy(str, Enum):
    """Available allocation strategies."""
    GREEDY = "greedy"       # Assign highest priority targets to nearest capable drone
    BALANCED = "balanced"    # Distribute targets evenly by count
    EFFICIENT = "efficient"  # Maximize priority/fuel ratio (auction-based)
    GEOGRAPHIC = "geographic"  # Minimize detours based on drone corridors
    EXCLUSIVE = "exclusive"  # Prioritize targets only one drone can reach


class TargetAllocator:
    """
    Allocates targets to drones based on configurable strategies.

    Takes into account:
    - Drone capabilities (which target types each drone can handle)
    - Drone fuel budgets
    - Target priorities
    - Geographic distribution
    - Distance matrix (SAM-aware if available)
    """

    def __init__(self):
        self._distance_matrix: Optional[Dict[str, Any]] = None

    def set_distance_matrix(self, matrix_data: Dict[str, Any]):
        """Set the pre-calculated distance matrix for allocation decisions."""
        self._distance_matrix = matrix_data

    def allocate(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]],
        airports: List[Dict[str, Any]],
        strategy: AllocationStrategy = AllocationStrategy.EFFICIENT
    ) -> Dict[str, List[str]]:
        """
        Allocate targets to drones using the specified strategy.

        Args:
            targets: List of target dicts with id, x, y, priority, type
            drone_configs: Dict of drone configs {
                "1": {"fuel_budget": 300, "target_access": {"A": true, ...},
                      "start_airport": "A1", "enabled": true},
                ...
            }
            airports: List of airport dicts
            strategy: Allocation strategy to use

        Returns:
            Dict mapping drone IDs to lists of assigned target IDs:
            {"1": ["T1", "T3"], "2": ["T2", "T4"], ...}
        """
        # Filter to enabled drones only
        active_drones = {
            did: cfg for did, cfg in drone_configs.items()
            if cfg.get("enabled", True) is not False
        }

        if not active_drones or not targets:
            return {did: [] for did in drone_configs.keys()}

        # Determine which targets each drone can access
        drone_capabilities = self._calculate_capabilities(targets, active_drones)

        # Call appropriate strategy
        if strategy == AllocationStrategy.GREEDY:
            result = self._allocate_greedy(targets, active_drones, airports, drone_capabilities)
        elif strategy == AllocationStrategy.BALANCED:
            result = self._allocate_balanced(targets, active_drones, airports, drone_capabilities)
        elif strategy == AllocationStrategy.EFFICIENT:
            result = self._allocate_efficient(targets, active_drones, airports, drone_capabilities)
        elif strategy == AllocationStrategy.GEOGRAPHIC:
            result = self._allocate_geographic(targets, active_drones, airports, drone_capabilities)
        elif strategy == AllocationStrategy.EXCLUSIVE:
            result = self._allocate_exclusive_first(targets, active_drones, airports, drone_capabilities)
        else:
            # Default to efficient
            result = self._allocate_efficient(targets, active_drones, airports, drone_capabilities)

        # Log any unassigned targets for debugging
        all_target_ids = {str(t["id"]) for t in targets}
        assigned_ids = set()
        for tid_list in result.values():
            assigned_ids.update(tid_list)
        unassigned = all_target_ids - assigned_ids

        if unassigned:
            print(f"⚠️ ALLOCATOR: {len(unassigned)} targets NOT assigned: {sorted(unassigned)}", flush=True)
            # Debug: check why each was skipped
            for tid in sorted(unassigned):
                capable_drones = [did for did, caps in drone_capabilities.items() if tid in caps]
                if not capable_drones:
                    print(f"   {tid}: No drone has capability (type restriction)", flush=True)
                else:
                    print(f"   {tid}: Capable drones={capable_drones} but not assigned", flush=True)

        return result

    def _calculate_capabilities(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Set[str]]:
        """
        Calculate which targets each drone can access based on type restrictions.

        Returns:
            Dict mapping drone IDs to sets of accessible target IDs
        """
        capabilities: Dict[str, Set[str]] = {}

        for did, cfg in drone_configs.items():
            target_access = cfg.get("target_access", {})

            # If no types specified, allow all
            if not target_access:
                allowed_types = {"a", "b", "c", "d"}
            else:
                allowed_types = {
                    t.lower() for t, enabled in target_access.items()
                    if enabled and isinstance(t, str)
                }

            # Find all targets this drone can reach
            accessible = set()
            for target in targets:
                target_type = str(target.get("type", "a")).lower()
                if target_type in allowed_types:
                    accessible.add(str(target["id"]))

            capabilities[did] = accessible

        return capabilities

    def _get_distance(
        self,
        from_id: str,
        to_id: str,
        waypoints: Optional[List[Dict[str, Any]]] = None
    ) -> float:
        """Get distance between two waypoints from matrix or calculate directly."""
        if self._distance_matrix:
            labels = self._distance_matrix.get("labels", [])
            matrix = self._distance_matrix.get("matrix", [])

            try:
                i = labels.index(from_id)
                j = labels.index(to_id)
                return matrix[i][j]
            except (ValueError, IndexError):
                pass

        # Fallback: calculate from coordinates if waypoints provided
        if waypoints:
            from_wp = next((w for w in waypoints if str(w.get("id")) == from_id), None)
            to_wp = next((w for w in waypoints if str(w.get("id")) == to_id), None)

            if from_wp and to_wp:
                return math.hypot(
                    to_wp["x"] - from_wp["x"],
                    to_wp["y"] - from_wp["y"]
                )

        return float('inf')

    def _allocate_greedy(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]],
        airports: List[Dict[str, Any]],
        capabilities: Dict[str, Set[str]]
    ) -> Dict[str, List[str]]:
        """
        Greedy allocation: assign highest priority targets to nearest capable drone.

        Fast but may not be optimal for overall efficiency.
        """
        # Sort targets by priority (highest first)
        sorted_targets = sorted(
            targets,
            key=lambda t: int(t.get("priority", 5)),
            reverse=True
        )

        assignments: Dict[str, List[str]] = {did: [] for did in drone_configs.keys()}
        assigned_ids: Set[str] = set()

        # Build waypoints list for distance calculation
        waypoints = airports + targets

        # Get drone starting positions
        drone_positions: Dict[str, str] = {}
        airport_by_id = {str(a["id"]): a for a in airports}

        for did, cfg in drone_configs.items():
            start_id = cfg.get("start_airport", f"A{did}")
            if start_id not in airport_by_id:
                start_id = airports[0]["id"] if airports else None
            drone_positions[did] = start_id

        for target in sorted_targets:
            tid = str(target["id"])

            if tid in assigned_ids:
                continue

            # Find nearest capable drone
            best_drone = None
            best_distance = float('inf')

            for did in drone_configs.keys():
                if tid not in capabilities[did]:
                    continue

                # Calculate distance from drone's current position to target
                drone_pos = drone_positions.get(did)
                if not drone_pos:
                    continue

                distance = self._get_distance(drone_pos, tid, waypoints)

                if distance < best_distance:
                    best_distance = distance
                    best_drone = did

            if best_drone:
                assignments[best_drone].append(tid)
                assigned_ids.add(tid)
                # Update drone position to last assigned target
                drone_positions[best_drone] = tid

        return assignments

    def _allocate_balanced(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]],
        airports: List[Dict[str, Any]],
        capabilities: Dict[str, Set[str]]
    ) -> Dict[str, List[str]]:
        """
        Balanced allocation: distribute targets evenly by count.

        Ensures each drone gets approximately the same number of targets.
        """
        assignments: Dict[str, List[str]] = {did: [] for did in drone_configs.keys()}
        assigned_ids: Set[str] = set()

        # Sort targets by priority
        sorted_targets = sorted(
            targets,
            key=lambda t: int(t.get("priority", 5)),
            reverse=True
        )

        # Calculate target quota per drone
        num_drones = len(drone_configs)
        targets_per_drone = len(targets) // num_drones
        extra = len(targets) % num_drones

        # Build waypoints list for distance calculation
        waypoints = airports + targets

        # Get drone starting positions
        drone_positions: Dict[str, str] = {}
        airport_by_id = {str(a["id"]): a for a in airports}

        for did, cfg in drone_configs.items():
            start_id = cfg.get("start_airport", f"A{did}")
            if start_id not in airport_by_id:
                start_id = airports[0]["id"] if airports else None
            drone_positions[did] = start_id

        for target in sorted_targets:
            tid = str(target["id"])

            if tid in assigned_ids:
                continue

            # Find drone with fewest assignments that can handle this target
            best_drone = None
            min_assignments = float('inf')
            best_distance = float('inf')

            for did in drone_configs.keys():
                if tid not in capabilities[did]:
                    continue

                current_count = len(assignments[did])

                # Check if drone can take more targets
                max_for_drone = targets_per_drone + (1 if int(did) <= extra else 0)
                if current_count >= max_for_drone:
                    continue

                # Prefer drone with fewer assignments; break ties by distance
                drone_pos = drone_positions.get(did)
                distance = self._get_distance(drone_pos, tid, waypoints) if drone_pos else float('inf')

                if current_count < min_assignments or \
                   (current_count == min_assignments and distance < best_distance):
                    min_assignments = current_count
                    best_distance = distance
                    best_drone = did

            if best_drone:
                assignments[best_drone].append(tid)
                assigned_ids.add(tid)
                drone_positions[best_drone] = tid

        return assignments

    def _allocate_efficient(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]],
        airports: List[Dict[str, Any]],
        capabilities: Dict[str, Set[str]]
    ) -> Dict[str, List[str]]:
        """
        Efficient allocation: maximize priority/distance ratio (auction-style).

        Each target goes to the drone that gets the best "value" for reaching it.
        """
        assignments: Dict[str, List[str]] = {did: [] for did in drone_configs.keys()}
        assigned_ids: Set[str] = set()

        # Sort targets by priority
        sorted_targets = sorted(
            targets,
            key=lambda t: int(t.get("priority", 5)),
            reverse=True
        )

        # Build waypoints list for distance calculation
        waypoints = airports + targets

        # Get drone starting positions
        drone_positions: Dict[str, str] = {}
        airport_by_id = {str(a["id"]): a for a in airports}

        for did, cfg in drone_configs.items():
            start_id = cfg.get("start_airport", f"A{did}")
            if start_id not in airport_by_id:
                start_id = airports[0]["id"] if airports else None
            drone_positions[did] = start_id

        for target in sorted_targets:
            tid = str(target["id"])
            priority = int(target.get("priority", 5))

            if tid in assigned_ids:
                continue

            # Find drone with best efficiency for this target
            best_drone = None
            best_efficiency = float('-inf')  # Allow any efficiency, even 0 or negative

            for did in drone_configs.keys():
                if tid not in capabilities[did]:
                    continue

                # Calculate efficiency: priority / distance
                drone_pos = drone_positions.get(did)
                if not drone_pos:
                    continue

                distance = self._get_distance(drone_pos, tid, waypoints)

                # Avoid division by zero; treat infinity as very large distance
                if distance == float('inf'):
                    # Still allow assignment but with minimal efficiency
                    efficiency = 0.0
                elif distance <= 0:
                    distance = 0.1
                    efficiency = priority / distance
                else:
                    efficiency = priority / distance

                if efficiency > best_efficiency:
                    best_efficiency = efficiency
                    best_drone = did

            if best_drone:
                assignments[best_drone].append(tid)
                assigned_ids.add(tid)
                drone_positions[best_drone] = tid

        return assignments

    def _allocate_geographic(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]],
        airports: List[Dict[str, Any]],
        capabilities: Dict[str, Set[str]]
    ) -> Dict[str, List[str]]:
        """
        Geographic allocation: assign targets based on angular corridors from airports.

        Divides the environment into sectors, one per drone.
        """
        assignments: Dict[str, List[str]] = {did: [] for did in drone_configs.keys()}
        assigned_ids: Set[str] = set()

        # Calculate centroid of all targets
        if not targets:
            return assignments

        center_x = sum(t["x"] for t in targets) / len(targets)
        center_y = sum(t["y"] for t in targets) / len(targets)

        # Assign angular sectors to drones
        num_drones = len(drone_configs)
        sector_size = 2 * math.pi / num_drones

        drone_sectors: Dict[str, Tuple[float, float]] = {}
        for i, did in enumerate(sorted(drone_configs.keys())):
            start_angle = i * sector_size - math.pi
            end_angle = start_angle + sector_size
            drone_sectors[did] = (start_angle, end_angle)

        # Assign each target to appropriate sector drone
        for target in targets:
            tid = str(target["id"])

            if tid in assigned_ids:
                continue

            # Calculate angle from center to target
            dx = target["x"] - center_x
            dy = target["y"] - center_y
            angle = math.atan2(dy, dx)

            # Find sector this target belongs to
            for did, (start_angle, end_angle) in drone_sectors.items():
                # Normalize angle comparison
                test_angle = angle
                if test_angle < start_angle:
                    test_angle += 2 * math.pi

                if start_angle <= test_angle < end_angle:
                    # Check if drone can handle this target
                    if tid in capabilities[did]:
                        assignments[did].append(tid)
                        assigned_ids.add(tid)
                        break
            else:
                # No sector matched; assign to nearest capable drone
                best_drone = None
                best_distance = float('inf')

                for did in drone_configs.keys():
                    if tid not in capabilities[did]:
                        continue

                    # Use angular distance as metric
                    mid_angle = (drone_sectors[did][0] + drone_sectors[did][1]) / 2
                    angular_dist = abs(angle - mid_angle)
                    if angular_dist > math.pi:
                        angular_dist = 2 * math.pi - angular_dist

                    if angular_dist < best_distance:
                        best_distance = angular_dist
                        best_drone = did

                if best_drone:
                    assignments[best_drone].append(tid)
                    assigned_ids.add(tid)

        return assignments

    def _allocate_exclusive_first(
        self,
        targets: List[Dict[str, Any]],
        drone_configs: Dict[str, Dict[str, Any]],
        airports: List[Dict[str, Any]],
        capabilities: Dict[str, Set[str]]
    ) -> Dict[str, List[str]]:
        """
        Exclusive-first allocation: prioritize targets only one drone can reach.

        Ensures unique capabilities are utilized before general assignments.
        """
        assignments: Dict[str, List[str]] = {did: [] for did in drone_configs.keys()}
        assigned_ids: Set[str] = set()

        # Build waypoints list for distance calculation
        waypoints = airports + targets

        # Get drone starting positions
        drone_positions: Dict[str, str] = {}
        airport_by_id = {str(a["id"]): a for a in airports}

        for did, cfg in drone_configs.items():
            start_id = cfg.get("start_airport", f"A{did}")
            if start_id not in airport_by_id:
                start_id = airports[0]["id"] if airports else None
            drone_positions[did] = start_id

        # Sort targets by priority
        sorted_targets = sorted(
            targets,
            key=lambda t: int(t.get("priority", 5)),
            reverse=True
        )

        # First pass: assign targets only one drone can handle
        for target in sorted_targets:
            tid = str(target["id"])

            if tid in assigned_ids:
                continue

            # Find all drones that can handle this target
            capable_drones = [
                did for did in drone_configs.keys()
                if tid in capabilities[did]
            ]

            if len(capable_drones) == 1:
                # Only one drone can handle this target
                did = capable_drones[0]
                assignments[did].append(tid)
                assigned_ids.add(tid)
                drone_positions[did] = tid

        # Second pass: use efficient strategy for remaining targets
        remaining_targets = [t for t in sorted_targets if str(t["id"]) not in assigned_ids]

        for target in remaining_targets:
            tid = str(target["id"])
            priority = int(target.get("priority", 5))

            # Find drone with best efficiency
            best_drone = None
            best_efficiency = float('-inf')  # Allow any efficiency, even 0 or negative

            for did in drone_configs.keys():
                if tid not in capabilities[did]:
                    continue

                drone_pos = drone_positions.get(did)
                if not drone_pos:
                    continue

                distance = self._get_distance(drone_pos, tid, waypoints)

                # Avoid division by zero; treat infinity as very large distance
                if distance == float('inf'):
                    # Still allow assignment but with minimal efficiency
                    efficiency = 0.0
                elif distance <= 0:
                    distance = 0.1
                    efficiency = priority / distance
                else:
                    efficiency = priority / distance

                if efficiency > best_efficiency:
                    best_efficiency = efficiency
                    best_drone = did

            if best_drone:
                assignments[best_drone].append(tid)
                assigned_ids.add(tid)
                drone_positions[best_drone] = tid

        return assignments


# Global allocator instance
_allocator = TargetAllocator()


def allocate_targets(
    env: Dict[str, Any],
    drone_configs: Dict[str, Dict[str, Any]],
    strategy: str = "efficient",
    distance_matrix: Optional[Dict[str, Any]] = None
) -> Dict[str, List[str]]:
    """
    Allocate targets to drones using the specified strategy.

    This is the main entry point for use as a LangGraph tool.

    Args:
        env: Environment dict with targets, airports, sams
        drone_configs: Dict of drone configurations
        strategy: Allocation strategy ("greedy", "balanced", "efficient",
                  "geographic", "exclusive")
        distance_matrix: Pre-calculated SAM-aware distance matrix (optional)

    Returns:
        Dict mapping drone IDs to lists of target IDs
    """
    if distance_matrix:
        _allocator.set_distance_matrix(distance_matrix)

    targets = env.get("targets", [])
    airports = env.get("airports", [])

    # Filter out excluded targets (e.g., targets inside SAM zones)
    if distance_matrix:
        excluded = set(distance_matrix.get("excluded_targets", []))
        if excluded:
            original_count = len(targets)
            targets = [t for t in targets if str(t["id"]) not in excluded]
            if len(targets) < original_count:
                print(f"⚠️ Filtered out {original_count - len(targets)} excluded targets "
                      f"(inside SAM zones): {excluded}", flush=True)

    # Convert strategy string to enum
    try:
        strat_enum = AllocationStrategy(strategy.lower())
    except ValueError:
        strat_enum = AllocationStrategy.EFFICIENT

    return _allocator.allocate(targets, drone_configs, airports, strat_enum)


def set_allocator_matrix(matrix_data: Dict[str, Any]):
    """Set the distance matrix for the global allocator."""
    _allocator.set_distance_matrix(matrix_data)


# Tool definition for LangGraph
TARGET_ALLOCATOR_TOOL = {
    "name": "allocate_targets",
    "description": """Allocate targets to drones using various strategies before route optimization.
    Available strategies:
    - greedy: Assign highest priority targets to nearest capable drone
    - balanced: Distribute targets evenly by count
    - efficient: Maximize priority/distance ratio (recommended)
    - geographic: Divide environment into angular sectors
    - exclusive: Prioritize targets only one drone can reach first""",
    "parameters": {
        "type": "object",
        "properties": {
            "env": {
                "type": "object",
                "description": "Environment data containing targets, airports, and sams"
            },
            "drone_configs": {
                "type": "object",
                "description": "Drone configurations with capabilities and fuel budgets"
            },
            "strategy": {
                "type": "string",
                "description": "Allocation strategy to use",
                "enum": ["greedy", "balanced", "efficient", "geographic", "exclusive"],
                "default": "efficient"
            },
            "distance_matrix": {
                "type": "object",
                "description": "Pre-calculated SAM-aware distance matrix (optional)"
            }
        },
        "required": ["env", "drone_configs"]
    },
    "function": allocate_targets
}
