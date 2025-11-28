"""
Drag and Drop Handler for ISR Editor
Handles mouse interactions for object manipulation
"""
import numpy as np


class DragDropHandler:
    """Handles drag and drop operations for environment objects"""
    
    def __init__(self, object_manager):
        self.object_manager = object_manager
        self.dragging = False
        self.drag_object = None
        self.drag_index = None
        self.drag_offset = (0, 0)
        
        # Grid snapping
        self.grid_size = 1  # Grid spacing
        self.snap_to_grid = False
        
    def on_mouse_press(self, event):
        """Handle mouse press event"""
        if event.inaxes is None:
            return False
            
        x, y = event.xdata, event.ydata
        
        # Find object at click position
        obj_type, obj_index = self.object_manager.find_object_at_position(x, y)
        
        if obj_type is not None:
            self.dragging = True
            self.drag_object = obj_type
            self.drag_index = obj_index
            
            # Calculate offset from object center to click position
            obj_pos = self._get_object_position(obj_type, obj_index)
            self.drag_offset = (x - obj_pos[0], y - obj_pos[1])
            return True
            
        return False
    
    def on_mouse_motion(self, event):
        """Handle mouse motion event"""
        if not self.dragging or event.inaxes is None:
            return False
            
        x, y = event.xdata, event.ydata
        
        # Apply offset
        new_x = x - self.drag_offset[0]
        new_y = y - self.drag_offset[1]
        
        # Apply grid snapping if enabled
        if self.snap_to_grid:
            new_x = round(new_x / self.grid_size) * self.grid_size
            new_y = round(new_y / self.grid_size) * self.grid_size
        
        # Constrain to grid bounds (0-100)
        new_x = max(0, min(100, new_x))
        new_y = max(0, min(100, new_y))
        
        # Update object position
        self.object_manager.update_object_position(self.drag_object, self.drag_index, new_x, new_y)
        
        return True
    
    def on_mouse_release(self, event):
        """Handle mouse release event"""
        if self.dragging:
            self.dragging = False
            
            # Log final position
            if self.drag_object and self.drag_index is not None:
                obj_pos = self._get_object_position(self.drag_object, self.drag_index)
                obj_id = self._get_object_id(self.drag_object, self.drag_index)
                print(f"Moved {obj_id} to ({obj_pos[0]:.1f}, {obj_pos[1]:.1f})")
            
            self.drag_object = None
            self.drag_index = None
            self.drag_offset = (0, 0)
            return True
            
        return False
    
    def _get_object_position(self, obj_type, index):
        """Get position of object by type and index"""
        if obj_type == 'airport':
            obj = self.object_manager.airports[index]
            return (obj['x'], obj['y'])
        elif obj_type == 'target':
            obj = self.object_manager.targets[index]
            return (obj['x'], obj['y'])
        elif obj_type == 'sam':
            obj = self.object_manager.sams[index]
            return obj['pos']
        return (0, 0)
    
    def _get_object_id(self, obj_type, index):
        """Get ID of object by type and index"""
        if obj_type == 'airport':
            return self.object_manager.airports[index]['id']
        elif obj_type == 'target':
            return self.object_manager.targets[index]['id']
        elif obj_type == 'sam':
            return f'SAM{index+1}'
        return 'Unknown'
    
    def set_grid_snap(self, enabled, grid_size=1):
        """Enable or disable grid snapping"""
        self.snap_to_grid = enabled
        self.grid_size = grid_size
    
    def is_dragging(self):
        """Check if currently dragging an object"""
        return self.dragging