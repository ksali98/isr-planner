"""
SAM Navigation module for ISR Editor
Handles path planning around SAM sites using boundary navigation with INVIOLABLE constraint.

Key Rules:
1. Polygon boundaries are NEVER penetrated
2. Navigation uses tangent-arc-tangent method
3. Arc traversal always moves forward (same direction of travel)
4. Vertex count based on minimum segment length (2 units)
5. Choose shorter path when two valid paths exist
"""
import numpy as np


class SAMNavigator:
    """Handles navigation around SAM sites using boundary navigation"""

    def __init__(self):
        self.debug = False

    def plan_path_with_sam_avoidance(self, start, end, sams, debug=False):
        """
        Plan path from start to end avoiding SAM sites.

        CRITICAL: Path will NEVER penetrate any SAM boundary.

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
            # Import boundary navigation module
            import sys
            import os
            # Navigate from isr_web/server/solver to project root (Editable-ENV)
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            if project_root not in sys.path:
                sys.path.append(project_root)

            from path_planning_core.boundary_navigation import plan_path

            # Use boundary navigation - NEVER penetrates boundaries
            path, distance, method = plan_path(start, end, sams, debug=debug)

            # Check if path is valid (method doesn't start with INVALID)
            if method.startswith("INVALID"):
                if debug:
                    print(f"⚠️ boundary_navigation returned invalid path: {method}")
                # Return None to indicate no valid path exists
                # DO NOT fall back to direct path - that would violate boundaries
                return None, float('inf'), f"NO_VALID_PATH ({method})"

            if path and len(path) >= 2:
                # Convert any numpy arrays to lists for JSON serialization
                clean_path = []
                for waypoint in path:
                    if hasattr(waypoint, 'tolist'):
                        clean_path.append(waypoint.tolist())
                    elif isinstance(waypoint, (list, tuple)):
                        clean_path.append(list(waypoint))
                    else:
                        clean_path.append(waypoint)
                return clean_path, distance, f"Boundary Navigation ({method})"
            else:
                if debug:
                    print(f"⚠️ boundary_navigation returned empty path")
                return None, float('inf'), "NO_VALID_PATH (empty)"

        except ImportError as e:
            if debug:
                print(f"❌ Could not import boundary_navigation module: {e}")
            # DO NOT fall back to direct path - that could violate boundaries
            return None, float('inf'), f"IMPORT_ERROR ({e})"
    
    def _direct_path(self, start, end):
        """Return direct path when no SAMs are present"""
        path = [start, end]
        distance = np.linalg.norm(np.array(end) - np.array(start))
        return path, distance, "Direct path"