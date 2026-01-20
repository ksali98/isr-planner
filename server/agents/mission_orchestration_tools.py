"""
Mission Orchestration Tools for LLM Agent

Provides high-level tools for the LLM agent to orchestrate complex multi-segment
missions, including:
- Inspection (query state, positions, targets)
- Solving (initial + continuation)
- Optimization (Insert, Swap, NoCrossing)
- Trajectory manipulation (split, loiter, distance calculations)
- Segmented mission management (create, load, export)
- Constraint helpers (fuel, loiter, validation)

These tools wrap the existing infrastructure (solver_bridge, post_optimizer, etc.)
to make them accessible to natural language commands.
"""

import json
import math
import sys
from typing import Dict, Any, List, Optional, Tuple, Set
from pathlib import Path

# Add paths for imports
current_path = Path(__file__).resolve().parent.parent
if str(current_path) not in sys.path:
    sys.path.insert(0, str(current_path))

# Import existing solver components
try:
    from solver.solver_bridge import solve_mission_with_allocation
    from solver.post_optimizer import (
        post_optimize_solution,
        trajectory_swap_optimize,
        crossing_removal_optimize,
    )
    from solver.sam_distance_matrix import calculate_sam_aware_matrix
except ImportError:
    # Fallback for different import contexts
    from server.solver.solver_bridge import solve_mission_with_allocation
    from server.solver.post_optimizer import (
        post_optimize_solution,
        trajectory_swap_optimize,
        crossing_removal_optimize,
    )
    from server.solver.sam_distance_matrix import calculate_sam_aware_matrix


# ============================================================================
# CATEGORY 1: INSPECTION TOOLS
# ============================================================================

class MissionInspector:
    """Tools for inspecting current mission state."""
    
    def __init__(self):
        self._current_env: Optional[Dict[str, Any]] = None
        self._current_solution: Optional[Dict[str, Any]] = None
        self._current_drone_configs: Optional[Dict[str, Any]] = None
        self._distance_matrix: Optional[Dict[str, Any]] = None
    
    def set_context(
        self,
        env: Dict[str, Any],
        solution: Dict[str, Any] = None,
        drone_configs: Dict[str, Any] = None,
        distance_matrix: Dict[str, Any] = None
    ):
        """Set the current mission context."""
        self._current_env = env
        self._current_solution = solution
        self._current_drone_configs = drone_configs
        self._distance_matrix = distance_matrix
    
    def get_current_environment(self) -> Dict[str, Any]:
        """Get current environment (airports, targets, SAMs)."""
        if not self._current_env:
            raise ValueError("No environment context set")
        return self._current_env.copy()
    
    def get_current_drone_configs(self) -> Dict[str, Any]:
        """Get current drone configurations."""
        if not self._current_drone_configs:
            raise ValueError("No drone configs context set")
        return {k: v.copy() for k, v in self._current_drone_configs.items()}
    
    def get_current_solution(self) -> Optional[Dict[str, Any]]:
        """Get current solution (routes, sequences)."""
        if not self._current_solution:
            return None
        return {
            "routes": {k: v.copy() for k, v in self._current_solution.get("routes", {}).items()},
            "sequences": self._current_solution.get("sequences", {}).copy()
        }
    
    def get_all_targets(self) -> List[Dict[str, Any]]:
        """Get all targets from environment."""
        if not self._current_env:
            return []
        return [t.copy() for t in self._current_env.get("targets", [])]
    
    def get_targets_by_priority(
        self,
        min_priority: Optional[int] = None,
        max_priority: Optional[int] = None,
        exact_priority: Optional[int] = None
    ) -> List[str]:
        """
        Filter targets by priority.
        
        Args:
            min_priority: Minimum priority (inclusive)
            max_priority: Maximum priority (inclusive)
            exact_priority: Exact priority match
        
        Returns:
            List of target IDs matching criteria
        """
        targets = self.get_all_targets()
        result = []
        
        for t in targets:
            priority = t.get("priority", 0)
            
            if exact_priority is not None:
                if priority == exact_priority:
                    result.append(str(t["id"]))
            else:
                if min_priority is not None and priority < min_priority:
                    continue
                if max_priority is not None and priority > max_priority:
                    continue
                result.append(str(t["id"]))
        
        return result
    
    def get_targets_by_type(self, types: List[str]) -> List[str]:
        """
        Filter targets by sensor type.
        
        Args:
            types: List of target types (e.g., ["a", "b", "c"])
        
        Returns:
            List of target IDs matching any of the types
        """
        targets = self.get_all_targets()
        types_lower = [t.lower() for t in types]
        
        result = []
        for t in targets:
            if str(t.get("type", "")).lower() in types_lower:
                result.append(str(t["id"]))
        
        return result
    
    def get_unvisited_targets(self, visited: List[str] = None) -> List[str]:
        """
        Get targets not yet visited.
        
        Args:
            visited: List of visited target IDs (if None, checks all targets)
        
        Returns:
            List of unvisited target IDs
        """
        all_targets = [str(t["id"]) for t in self.get_all_targets()]
        
        if visited is None:
            # If no visited list provided, check current solution
            visited = []
            if self._current_solution:
                for route_data in self._current_solution.get("routes", {}).values():
                    route = route_data.get("route", [])
                    for wp in route:
                        if str(wp).startswith("T"):
                            visited.append(str(wp))
        
        return [t for t in all_targets if t not in visited]
    
    def get_route_info(self, drone_id: str) -> Dict[str, Any]:
        """
        Get detailed route information for a drone.
        
        Returns:
            {
                "route": List[str],
                "distance": float,
                "fuel_used": float,
                "targets": List[str],
                "priorities": List[int],
                "trajectory": List[[x,y]]
            }
        """
        if not self._current_solution:
            return {}
        
        route_data = self._current_solution.get("routes", {}).get(str(drone_id), {})
        route = route_data.get("route", [])
        
        targets = [str(wp) for wp in route if str(wp).startswith("T")]
        
        # Get priorities for targets
        priorities = []
        if self._current_env:
            target_map = {str(t["id"]): t for t in self._current_env.get("targets", [])}
            priorities = [target_map.get(tid, {}).get("priority", 0) for tid in targets]
        
        return {
            "route": route,
            "distance": route_data.get("distance", 0),
            "fuel_used": route_data.get("distance", 0),  # Distance = fuel for constant speed
            "targets": targets,
            "priorities": priorities,
            "trajectory": route_data.get("trajectory", [])
        }
    
    def get_drone_position_at_distance(
        self,
        drone_id: str,
        distance: float
    ) -> Optional[Tuple[float, float]]:
        """
        Calculate drone position at a specific distance along trajectory.
        
        Args:
            drone_id: Drone ID
            distance: Distance traveled
        
        Returns:
            [x, y] position or None if not found
        """
        if not self._current_solution:
            return None
        
        route_data = self._current_solution.get("routes", {}).get(str(drone_id), {})
        trajectory = route_data.get("trajectory", [])
        
        if not trajectory or len(trajectory) < 2:
            return None
        
        return self._interpolate_position_at_distance(trajectory, distance)
    
    def get_all_drone_positions_at_distance(
        self,
        distance: float
    ) -> Dict[str, Tuple[float, float]]:
        """
        Get positions of all drones at a specific distance.
        
        Args:
            distance: Distance traveled
        
        Returns:
            Dict mapping drone_id -> [x, y]
        """
        if not self._current_solution:
            return {}
        
        positions = {}
        for drone_id in self._current_solution.get("routes", {}).keys():
            pos = self.get_drone_position_at_distance(drone_id, distance)
            if pos:
                positions[str(drone_id)] = pos
        
        return positions
    
    def get_targets_visited_before_distance(self, distance: float) -> List[str]:
        """
        Get targets that would be visited before traveling a certain distance.
        
        Args:
            distance: Distance threshold
        
        Returns:
            List of target IDs visited before this distance
        """
        if not self._current_solution or not self._current_env:
            return []
        
        visited = []
        target_map = {str(t["id"]): (t["x"], t["y"]) for t in self._current_env.get("targets", [])}
        
        for drone_id, route_data in self._current_solution.get("routes", {}).items():
            trajectory = route_data.get("trajectory", [])
            route = route_data.get("route", [])
            
            if not trajectory or len(trajectory) < 2:
                continue
            
            # Calculate cumulative distances along trajectory
            cum_dist = [0.0]
            for i in range(1, len(trajectory)):
                p1 = trajectory[i - 1]
                p2 = trajectory[i]
                seg_dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                cum_dist.append(cum_dist[-1] + seg_dist)
            
            # Check each target in route
            for target_id in route:
                if not str(target_id).startswith("T"):
                    continue
                
                target_pos = target_map.get(str(target_id))
                if not target_pos:
                    continue
                
                # Find closest trajectory point to this target
                min_dist_to_traj = float('inf')
                closest_traj_idx = -1
                
                for i, traj_pt in enumerate(trajectory):
                    dist = math.sqrt(
                        (traj_pt[0] - target_pos[0])**2 +
                        (traj_pt[1] - target_pos[1])**2
                    )
                    if dist < min_dist_to_traj:
                        min_dist_to_traj = dist
                        closest_traj_idx = i
                
                # If target is close to trajectory and reached before distance threshold
                if closest_traj_idx >= 0 and min_dist_to_traj < 20.0:  # 20 unit visit threshold
                    target_distance = cum_dist[closest_traj_idx]
                    if target_distance <= distance:
                        visited.append(str(target_id))
        
        return list(set(visited))  # Remove duplicates
    
    def _interpolate_position_at_distance(
        self,
        trajectory: List[List[float]],
        distance: float
    ) -> Optional[Tuple[float, float]]:
        """
        Interpolate position along trajectory at given distance.
        """
        if not trajectory or len(trajectory) < 2:
            return None
        
        cum_dist = 0.0
        
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]
            p2 = trajectory[i + 1]
            seg_dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            if cum_dist + seg_dist >= distance:
                # Interpolate within this segment
                t = (distance - cum_dist) / seg_dist if seg_dist > 0 else 0
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                return (x, y)
            
            cum_dist += seg_dist
        
        # If distance exceeds trajectory, return last point
        return (trajectory[-1][0], trajectory[-1][1])


# ============================================================================
# CATEGORY 2: SOLVING TOOLS
# ============================================================================

class MissionSolver:
    """Tools for solving missions (initial and continuation)."""
    
    def __init__(self):
        self._inspector = MissionInspector()
    
    def set_inspector(self, inspector: MissionInspector):
        """Set the inspector instance for shared context."""
        self._inspector = inspector
    
    def solve_mission(
        self,
        env: Dict[str, Any],
        drone_configs: Dict[str, Any],
        allocation_strategy: str = "efficient",
        exclude_targets: List[str] = None,
        include_targets: List[str] = None,
        target_loiter_costs: Dict[str, int] = None,
        use_sam_aware: bool = True
    ) -> Dict[str, Any]:
        """
        Solve mission from scratch.
        
        Args:
            env: Environment with airports, targets, SAMs
            drone_configs: Drone configurations
            allocation_strategy: "efficient", "greedy", "balanced", "geographic"
            exclude_targets: Target IDs to exclude from solving
            include_targets: If set, ONLY solve for these targets
            target_loiter_costs: Dict of {target_id: loiter_steps} for loiter costs
            use_sam_aware: Use SAM-aware distance calculations
        
        Returns:
            Solution dict with routes and sequences
        """
        # Filter targets based on include/exclude
        filtered_env = env.copy()
        all_targets = filtered_env.get("targets", [])
        
        if include_targets:
            # Only include specified targets
            filtered_env["targets"] = [
                t for t in all_targets if str(t["id"]) in include_targets
            ]
        elif exclude_targets:
            # Exclude specified targets
            filtered_env["targets"] = [
                t for t in all_targets if str(t["id"]) not in exclude_targets
            ]
        
        # TODO: Handle target_loiter_costs by modifying distance matrix
        # This requires adding loiter cost to distances involving those targets
        
        result = solve_mission_with_allocation(
            env=filtered_env,
            drone_configs=drone_configs,
            allocation_strategy=allocation_strategy,
            use_sam_aware_distances=use_sam_aware,
            post_optimize=False  # We'll optimize separately if needed
        )
        
        # Update inspector context
        self._inspector.set_context(filtered_env, result, drone_configs)
        
        return result
    
    def solve_continuation(
        self,
        env: Dict[str, Any],
        enabled_drones: List[str],
        synthetic_starts: Dict[str, Tuple[float, float]],
        visited_targets: List[str],
        drone_configs: Dict[str, Any],
        allocation_strategy: str = "efficient",
        fuel_budgets: Dict[str, float] = None,
        exclude_targets: List[str] = None,
        target_loiter_costs: Dict[str, int] = None,
        use_sam_aware: bool = True
    ) -> Dict[str, Any]:
        """
        Solve continuation from synthetic start positions.
        
        Args:
            env: Environment (may have updated targets/SAMs)
            enabled_drones: List of drone IDs to solve for
            synthetic_starts: Dict of {drone_id: [x, y]} start positions
            visited_targets: List of already visited target IDs
            drone_configs: Drone configurations
            allocation_strategy: Allocation strategy
            fuel_budgets: Optional fuel budgets (if None, use original configs)
            exclude_targets: Additional targets to exclude
            target_loiter_costs: Loiter costs for targets
            use_sam_aware: Use SAM-aware calculations
        
        Returns:
            Solution dict with routes
        """
        # Filter visited targets from environment
        filtered_env = env.copy()
        all_targets = filtered_env.get("targets", [])
        
        exclude_set = set(visited_targets)
        if exclude_targets:
            exclude_set.update(exclude_targets)
        
        filtered_env["targets"] = [
            t for t in all_targets if str(t["id"]) not in exclude_set
        ]
        
        # Add synthetic starts to environment
        filtered_env["synthetic_starts"] = {
            str(did): list(pos) for did, pos in synthetic_starts.items()
        }
        
        # Update drone configs with enabled drones and synthetic starts
        continuation_configs = {}
        for did, config in drone_configs.items():
            did_str = str(did)
            if did_str in enabled_drones:
                new_config = config.copy()
                
                # Set synthetic start as start_airport
                if did_str in synthetic_starts:
                    new_config["start_airport"] = f"D{did_str}_START"
                
                # Update fuel budget if provided
                if fuel_budgets and did_str in fuel_budgets:
                    new_config["fuel_budget"] = fuel_budgets[did_str]
                
                continuation_configs[did_str] = new_config
            else:
                # Disable this drone
                new_config = config.copy()
                new_config["enabled"] = False
                continuation_configs[did_str] = new_config
        
        result = solve_mission_with_allocation(
            env=filtered_env,
            drone_configs=continuation_configs,
            allocation_strategy=allocation_strategy,
            use_sam_aware_distances=use_sam_aware,
            post_optimize=False
        )
        
        # Update inspector context
        self._inspector.set_context(filtered_env, result, continuation_configs)
        
        return result


# ============================================================================
# CATEGORY 3: POST-OPTIMIZATION TOOLS
# ============================================================================

class MissionOptimizer:
    """Tools for post-optimization (Insert, Swap, NoCrossing)."""
    
    def __init__(self):
        self._inspector = MissionInspector()
    
    def set_inspector(self, inspector: MissionInspector):
        """Set the inspector instance for shared context."""
        self._inspector = inspector
    
    def optimize_insert_missed(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Any],
        distance_matrix: Dict[str, Any] = None,
        priority_constraints: str = ""
    ) -> Dict[str, Any]:
        """
        Insert unvisited targets into routes where fuel allows.
        
        Args:
            solution: Current solution
            env: Environment
            drone_configs: Drone configs
            distance_matrix: Pre-calculated distance matrix
            priority_constraints: e.g., "D1,D2: priority>=6"
        
        Returns:
            Optimized solution
        """
        result = post_optimize_solution(
            solution=solution,
            env=env,
            drone_configs=drone_configs,
            distance_matrix=distance_matrix,
            priority_constraints=priority_constraints
        )
        
        # Update inspector context
        self._inspector.set_context(env, result, drone_configs, distance_matrix)
        
        return result
    
    def optimize_swap_closer(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Any],
        distance_matrix: Dict[str, Any] = None,
        priority_constraints: str = "",
        auto_iterate: bool = True,
        max_iterations: int = 50
    ) -> Dict[str, Any]:
        """
        Reassign targets to drones with closer trajectories.
        
        Args:
            solution: Current solution
            env: Environment
            drone_configs: Drone configs
            distance_matrix: Pre-calculated distance matrix
            priority_constraints: Priority constraints
            auto_iterate: Run until convergence
            max_iterations: Max iterations for auto mode
        
        Returns:
            Optimized solution
        """
        result = trajectory_swap_optimize(
            solution=solution,
            env=env,
            drone_configs=drone_configs,
            distance_matrix=distance_matrix,
            priority_constraints=priority_constraints,
            auto_iterate=auto_iterate,
            max_iterations=max_iterations
        )
        
        # Update inspector context
        self._inspector.set_context(env, result, drone_configs, distance_matrix)
        
        return result
    
    def optimize_no_crossing(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Any],
        distance_matrix: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Remove route crossings using 2-opt.
        
        Args:
            solution: Current solution
            env: Environment
            drone_configs: Drone configs
            distance_matrix: Pre-calculated distance matrix
        
        Returns:
            Optimized solution
        """
        result = crossing_removal_optimize(
            solution=solution,
            env=env,
            drone_configs=drone_configs,
            distance_matrix=distance_matrix
        )
        
        # Update inspector context
        self._inspector.set_context(env, result, drone_configs, distance_matrix)
        
        return result
    
    def optimize_all(
        self,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Any],
        distance_matrix: Dict[str, Any] = None,
        priority_constraints: str = ""
    ) -> Dict[str, Any]:
        """
        Run all three optimizers in sequence.
        
        Order: Insert Missed → Swap Closer → No Crossing
        
        Returns:
            Fully optimized solution
        """
        # 1. Insert missed targets
        result = self.optimize_insert_missed(
            solution, env, drone_configs, distance_matrix, priority_constraints
        )
        
        # 2. Swap to closer trajectories
        result = self.optimize_swap_closer(
            result, env, drone_configs, distance_matrix, priority_constraints
        )
        
        # 3. Remove crossings
        result = self.optimize_no_crossing(
            result, env, drone_configs, distance_matrix
        )
        
        return result


# ============================================================================
# CATEGORY 4: TRAJECTORY MANIPULATION TOOLS
# ============================================================================

class TrajectoryManipulator:
    """Tools for trajectory manipulation."""
    
    def split_trajectory_at_distance(
        self,
        trajectory: List[List[float]],
        distance: float
    ) -> Dict[str, Any]:
        """
        Split trajectory at a specific distance.
        
        Returns:
            {
                "prefix": List[[x,y]],  # Points before split
                "suffix": List[[x,y]],  # Points after split
                "split_point": [x, y]   # Interpolated split position
            }
        """
        if not trajectory or len(trajectory) < 2:
            return {"prefix": [], "suffix": [], "split_point": None}
        
        prefix = []
        suffix = []
        split_point = None
        cum_dist = 0.0
        
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]
            p2 = trajectory[i + 1]
            seg_dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            if cum_dist + seg_dist >= distance and split_point is None:
                # Split occurs in this segment
                prefix.append(p1)
                
                # Interpolate split point
                t = (distance - cum_dist) / seg_dist if seg_dist > 0 else 0
                split_x = p1[0] + t * (p2[0] - p1[0])
                split_y = p1[1] + t * (p2[1] - p1[1])
                split_point = [split_x, split_y]
                
                prefix.append(split_point)
                suffix.append(split_point)
                suffix.append(p2)
            elif split_point is None:
                # Before split
                prefix.append(p1)
            else:
                # After split
                suffix.append(p2)
            
            cum_dist += seg_dist
        
        # Handle edge cases
        if split_point is None:
            # Distance exceeds trajectory
            prefix = trajectory
            suffix = [trajectory[-1]]
            split_point = trajectory[-1]
        
        return {
            "prefix": prefix,
            "suffix": suffix,
            "split_point": split_point
        }
    
    def calculate_trajectory_distance(self, points: List[List[float]]) -> float:
        """Calculate total distance of a trajectory."""
        if not points or len(points) < 2:
            return 0.0
        
        total = 0.0
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            total += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        
        return total
    
    def inject_loiter_at_targets(
        self,
        trajectory: List[List[float]],
        route: List[str],
        loiter_targets: List[str],
        loiter_steps: int,
        waypoint_positions: Dict[str, Tuple[float, float]]
    ) -> List[List[float]]:
        """
        Inject loiter waypoints into trajectory at specified targets.
        
        Loiter is implemented by duplicating the target position multiple times
        so the animation stays at that position for the specified steps.
        
        Args:
            trajectory: Original trajectory
            route: Route waypoint IDs
            loiter_targets: Target IDs that should have loiter
            loiter_steps: Number of steps to loiter
            waypoint_positions: Map of waypoint_id -> (x, y)
        
        Returns:
            Modified trajectory with loiter points injected
        """
        if not trajectory or not loiter_targets:
            return trajectory
        
        # Find positions in trajectory where loiter targets appear
        modified_traj = []
        traj_idx = 0
        
        for i, wp_id in enumerate(route):
            # Find trajectory segment for this waypoint
            wp_pos = waypoint_positions.get(str(wp_id))
            if not wp_pos:
                continue
            
            # Find matching position in trajectory
            while traj_idx < len(trajectory):
                traj_pt = trajectory[traj_idx]
                if abs(traj_pt[0] - wp_pos[0]) < 0.001 and abs(traj_pt[1] - wp_pos[1]) < 0.001:
                    # Found waypoint in trajectory
                    modified_traj.append(traj_pt)
                    
                    # If this is a loiter target, duplicate the position
                    if str(wp_id) in loiter_targets:
                        for _ in range(loiter_steps):
                            modified_traj.append([traj_pt[0], traj_pt[1]])
                    
                    traj_idx += 1
                    break
                else:
                    # Intermediate trajectory point (SAM avoidance)
                    modified_traj.append(traj_pt)
                    traj_idx += 1
        
        # Add remaining trajectory points
        while traj_idx < len(trajectory):
            modified_traj.append(trajectory[traj_idx])
            traj_idx += 1
        
        return modified_traj


# ============================================================================
# CATEGORY 5: SEGMENTED MISSION MANAGEMENT
# ============================================================================

class SegmentedMissionManager:
    """Tools for creating and managing segmented missions."""
    
    def create_segment(
        self,
        index: int,
        solution: Dict[str, Any],
        env: Dict[str, Any],
        drone_configs: Dict[str, Any],
        cut_distance: float = None,
        cut_positions: Dict[str, Tuple[float, float]] = None,
        frozen_targets: List[str] = None,
        active_targets: List[str] = None,
        lost_drones: List[str] = None
    ) -> Dict[str, Any]:
        """
        Create a segment object.
        
        Returns:
            Segment dict with all required fields
        """
        segment = {
            "index": index,
            "solution": solution,
            "env": env,
            "drone_configs": drone_configs,
            "cutDistance": cut_distance,
            "cutPositions": cut_positions,
            "frozenTargets": frozen_targets or [],
            "activeTargets": active_targets or [],
            "lostDrones": lost_drones or []
        }
        
        return segment
    
    def create_segmented_mission(
        self,
        segments: List[Dict[str, Any]],
        base_env: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Create a segmented mission from multiple segments.
        
        Args:
            segments: List of segment dicts
            base_env: Base environment (airports, SAMs)
        
        Returns:
            Segmented mission JSON structure
        """
        return {
            "schema": "isr_env_v2",
            "is_segmented": True,
            "segment_count": len(segments),
            "airports": base_env.get("airports", []),
            "sams": base_env.get("sams", []),
            "drone_configs": segments[0]["drone_configs"] if segments else {},
            "segments": segments
        }
    
    def load_segmented_json(self, filepath: str) -> Dict[str, Any]:
        """Load segmented mission from JSON file."""
        with open(filepath, 'r') as f:
            return json.load(f)
    
    def export_segmented_json(self, mission: Dict[str, Any], filepath: str) -> bool:
        """Export segmented mission to JSON file."""
        try:
            with open(filepath, 'w') as f:
                json.dump(mission, f, indent=2)
            return True
        except Exception as e:
            print(f"Error exporting segmented JSON: {e}")
            return False


# ============================================================================
# CATEGORY 6: CONSTRAINT HELPERS
# ============================================================================

class ConstraintHelper:
    """Tools for handling constraints (fuel, loiter, validation)."""
    
    def calculate_remaining_fuel(
        self,
        initial_budget: float,
        distance_traveled: float
    ) -> float:
        """Calculate remaining fuel (distance = fuel for constant speed)."""
        return max(0.0, initial_budget - distance_traveled)
    
    def get_loiter_costs_for_target_types(
        self,
        targets: List[Dict[str, Any]],
        type_loiter_map: Dict[str, int]
    ) -> Dict[str, int]:
        """
        Create loiter cost map for targets based on their types.
        
        Args:
            targets: List of target dicts
            type_loiter_map: Map of {type: loiter_steps}, e.g., {"c": 20}
        
        Returns:
            Dict of {target_id: loiter_steps}
        """
        loiter_costs = {}
        
        for t in targets:
            target_type = str(t.get("type", "")).lower()
            if target_type in type_loiter_map:
                loiter_costs[str(t["id"])] = type_loiter_map[target_type]
        
        return loiter_costs
    
    def validate_solution(
        self,
        solution: Dict[str, Any],
        drone_configs: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Validate solution for basic constraints.
        
        Returns:
            {
                "valid": bool,
                "issues": List[str]
            }
        """
        issues = []
        
        for drone_id, route_data in solution.get("routes", {}).items():
            distance = route_data.get("distance", 0)
            fuel_budget = drone_configs.get(str(drone_id), {}).get("fuel_budget", 0)
            
            if distance > fuel_budget:
                issues.append(
                    f"D{drone_id}: Exceeds fuel budget "
                    f"(distance={distance:.1f}, budget={fuel_budget:.1f})"
                )
        
        return {
            "valid": len(issues) == 0,
            "issues": issues
        }


# ============================================================================
# UNIFIED ORCHESTRATOR
# ============================================================================

class MissionOrchestrator:
    """
    Unified orchestrator providing all mission tools.
    
    This is the main interface for LLM agents to interact with the mission
    planning system.
    """
    
    def __init__(self):
        self.inspector = MissionInspector()
        self.solver = MissionSolver()
        self.optimizer = MissionOptimizer()
        self.trajectory = TrajectoryManipulator()
        self.segments = SegmentedMissionManager()
        self.constraints = ConstraintHelper()
        
        # Link components
        self.solver.set_inspector(self.inspector)
        self.optimizer.set_inspector(self.inspector)


# ============================================================================
# SINGLETON INSTANCE
# ============================================================================

_orchestrator = None

def get_orchestrator() -> MissionOrchestrator:
    """Get or create the singleton orchestrator instance."""
    global _orchestrator
    if _orchestrator is None:
        _orchestrator = MissionOrchestrator()
    return _orchestrator
