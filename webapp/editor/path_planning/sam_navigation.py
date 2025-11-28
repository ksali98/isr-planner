"""
SAM Navigation module for ISR Editor
Handles path planning around SAM sites using polygon wrapping and tangent methods
"""
import numpy as np
from matplotlib.path import Path


class SAMNavigator:
    """Handles navigation around SAM sites"""
    
    def __init__(self):
        self.debug = False
        
    def plan_path_with_sam_avoidance(self, start, end, sams, debug=False):
        """
        Plan path from start to end avoiding SAM sites using SAM wrapping and polygon navigation
        
        Args:
            start: [x, y] starting position
            end: [x, y] ending position  
            sams: List of SAM objects
            debug: Enable debug output
            
        Returns:
            Tuple of (path, distance, method_used)
        """
        self.debug = debug
        
        # If no SAMs, return direct path
        if not sams:
            return self._direct_path(start, end)
        
        try:
            # Import from unified path_planning module
            import sys
            import os
            # Add parent directory to path to find the modules
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            if parent_dir not in sys.path:
                sys.path.append(parent_dir)

            from path_planning_core.sam_wrapping import wrap_sams
            import path_planning_core.chain_rule as cr
            
            # Step 1: Apply SAM wrapping
            wrapped_polygons, cluster_info = wrap_sams(sams)
            
            if debug:
                print(f"🔧 SAM Wrapping: {len(sams)} SAMs → {len(wrapped_polygons)} polygons")
            
            # Step 2: Convert to planning format
            planning_sams = []
            for polygon, info in zip(wrapped_polygons, cluster_info):
                if info['type'] == 'individual':
                    # Single SAM - keep original format
                    original_sam = info['original_sams'][0].copy()
                    planning_sams.append(original_sam)
                else:
                    # Wrapped cluster - create polygon SAM
                    center = np.mean(polygon, axis=0)
                    max_radius = np.max([np.linalg.norm(vertex - center) for vertex in polygon])
                    
                    wrapped_sam = {
                        'pos': center.tolist(),
                        'range': max_radius,
                        'vertices': polygon.tolist()
                    }
                    planning_sams.append(wrapped_sam)
            
            # Step 3: Check if we have wrapped polygons
            has_wrapped = any(info['type'] == 'wrapped' for info in cluster_info)
            
            if has_wrapped:
                # Use polygon navigation for wrapped SAMs
                return self._plan_polygon_path(start, end, planning_sams)
            else:
                # Use chain rule for individual SAMs
                result = cr.fixed_chain_rule_planner(start, end, planning_sams, 0, debug)
                if result and len(result) >= 2:
                    return result[0], result[1], "Chain Rule Navigation"
                else:
                    return self._direct_path(start, end)
                    
        except ImportError as e:
            if debug:
                print(f"❌ Could not import SAM avoidance modules: {e}")
            return self._direct_path(start, end)
    
    def _plan_polygon_path(self, start, end, planning_sams):
        """Plan path using polygon tangent navigation with boundary following"""
        try:
            import sys
            import os
            # Add parent directory to path to find the modules
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            if parent_dir not in sys.path:
                sys.path.append(parent_dir)
                
            from polygon_chain_rule_no_buffer import find_polygon_tangents
            import chain_rule_fixed_FINAL_COMPLETE as cr
        except ImportError:
            return self._direct_path(start, end)
            
        current_pos = start
        path = [start]
        
        for sam in planning_sams:
            if 'vertices' in sam:
                # This is a wrapped polygon SAM
                polygon_path = Path(sam['vertices'])
                line_points = self._sample_line_points(current_pos, end, 50)
                intersection_count = sum(polygon_path.contains_point(pt) for pt in line_points)
                
                if intersection_count > 0:
                    # Line intersects polygon - use tangent navigation
                    start_tangent1, start_tangent2 = find_polygon_tangents(current_pos, sam)
                    
                    if start_tangent1 and start_tangent2:
                        # Choose tangent with shortest total distance
                        dist1 = self._calculate_path_distance(current_pos, start_tangent1, end)
                        dist2 = self._calculate_path_distance(current_pos, start_tangent2, end)
                        chosen_tangent = start_tangent1 if dist1 < dist2 else start_tangent2
                        path.append(chosen_tangent)
                        
                        # Check if tangent→destination also intersects polygon
                        tangent_to_dest_points = self._sample_line_points(chosen_tangent, end, 20)
                        tangent_intersections = sum(polygon_path.contains_point(pt) for pt in tangent_to_dest_points)
                        
                        if tangent_intersections > 0:
                            # Need boundary following to destination tangent
                            dest_tangent1, dest_tangent2 = find_polygon_tangents(end, sam)
                            
                            if dest_tangent1 and dest_tangent2:
                                # Choose closest destination tangent
                                dist_to_dest1 = np.linalg.norm(np.array(dest_tangent1) - np.array(chosen_tangent))
                                dist_to_dest2 = np.linalg.norm(np.array(dest_tangent2) - np.array(chosen_tangent))
                                chosen_dest_tangent = dest_tangent1 if dist_to_dest1 < dist_to_dest2 else dest_tangent2
                                
                                # Follow polygon boundary
                                boundary_points = self._follow_polygon_boundary(chosen_tangent, chosen_dest_tangent, sam)
                                path.extend(boundary_points)
                                path.append(chosen_dest_tangent)
                                current_pos = chosen_dest_tangent
                        else:
                            current_pos = chosen_tangent
            else:
                # This is an individual circular SAM - use chain rule for this segment
                segment_result = cr.fixed_chain_rule_planner(current_pos, end, [sam], 0, False)
                if segment_result and segment_result[0]:
                    segment_path = segment_result[0]
                    if len(segment_path) > 1:
                        path.extend(segment_path[1:])  # Skip first point
                        current_pos = segment_path[-1]
        
        # Add final destination
        if len(path) == 0 or path[-1] != end:
            path.append(end)
        
        # Calculate total distance
        total_distance = sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) 
                            for i in range(len(path) - 1))
        
        return path, total_distance, "Mixed polygon and circular navigation"
    
    def _plan_circular_sams(self, start, end, sams):
        """Plan path for circular SAMs using chain rule"""
        try:
            import sys
            import os
            # Add parent directory to path to find the modules
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            if parent_dir not in sys.path:
                sys.path.append(parent_dir)
                
            import chain_rule_fixed_FINAL_COMPLETE as cr
            return cr.fixed_chain_rule_planner(start, end, sams, 0, self.debug)
        except ImportError:
            return self._direct_path(start, end)
    
    def _direct_path(self, start, end):
        """Return direct path when no avoidance needed"""
        path = [start, end]
        distance = np.linalg.norm(np.array(end) - np.array(start))
        return path, distance, "Direct path"
    
    def _sample_line_points(self, start, end, num_points):
        """Sample points along a line segment for intersection testing"""
        points = []
        for i in range(num_points + 1):
            t = i / num_points
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append([x, y])
        return points
    
    def _calculate_path_distance(self, start, waypoint, end):
        """Calculate total distance for path: start → waypoint → end"""
        return (np.linalg.norm(np.array(waypoint) - np.array(start)) + 
                np.linalg.norm(np.array(end) - np.array(waypoint)))
    
    def _follow_polygon_boundary(self, start_tangent, end_tangent, sam):
        """Follow polygon boundary from start tangent to end tangent"""
        if 'vertices' not in sam:
            return []
        
        polygon_vertices = np.array(sam['vertices'])
        start_point = np.array(start_tangent)
        end_point = np.array(end_tangent)
        
        # Find closest vertices to tangent points
        start_distances = [np.linalg.norm(vertex - start_point) for vertex in polygon_vertices]
        end_distances = [np.linalg.norm(vertex - end_point) for vertex in polygon_vertices]
        
        start_vertex_idx = np.argmin(start_distances)
        end_vertex_idx = np.argmin(end_distances)
        
        boundary_points = []
        
        if start_vertex_idx != end_vertex_idx:
            # Calculate both clockwise and counterclockwise paths
            cw_indices = []
            idx = start_vertex_idx
            while idx != end_vertex_idx:
                idx = (idx + 1) % len(polygon_vertices)
                cw_indices.append(idx)
            
            ccw_indices = []
            idx = start_vertex_idx
            while idx != end_vertex_idx:
                idx = (idx - 1) % len(polygon_vertices)
                ccw_indices.append(idx)
            
            # Choose shorter path
            chosen_indices = cw_indices[:-1] if len(cw_indices) <= len(ccw_indices) else ccw_indices[:-1]
            
            for idx in chosen_indices:
                boundary_points.append(polygon_vertices[idx].tolist())
        
        return boundary_points