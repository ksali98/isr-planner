"""
Object Manager module for ISR Editor
Handles creation, deletion, and modification of environment objects
"""
import numpy as np


class ObjectManager:
    """Manages environment objects (airports, targets, SAMs, D-waypoints)"""
    
    def __init__(self):
        self.airports = []
        self.targets = []
        self.sams = []
        
        # Default positions for new objects (avoid UI elements in corners)
        self.DEFAULT_POS = (50, 50)  # Center area, away from legend
        
        # Valid SAM ranges
        self.VALID_SAM_RANGES = [5, 8, 10, 12, 15, 18, 20, 25]
        
    def add_airport(self, x=None, y=None):
        """Add a new airport"""
        if x is None or y is None:
            x, y = self.DEFAULT_POS
        
        # Find next available airport ID
        existing_ids = {int(airport['id'][1:]) for airport in self.airports if airport['id'].startswith('A') and airport['id'][1:].isdigit()}
        next_id = 1
        while next_id in existing_ids:
            next_id += 1
            
        new_airport = {
            'id': f'A{next_id}',
            'x': x,
            'y': y
        }
        self.airports.append(new_airport)
        return new_airport
    
    def add_target(self, x=None, y=None, priority=5, target_type='a'):
        """Add a new target"""
        if x is None or y is None:
            x, y = self.DEFAULT_POS
        
        # Find next available target ID
        existing_ids = {int(target['id'][1:]) for target in self.targets if target['id'].startswith('T') and target['id'][1:].isdigit()}
        next_id = 1
        while next_id in existing_ids:
            next_id += 1
            
        new_target = {
            'id': f'T{next_id}',
            'x': x,
            'y': y,
            'priority': priority,
            'type': target_type
        }
        self.targets.append(new_target)
        return new_target
    
    def add_sam(self, x=None, y=None, range_val=15):
        """Add a new SAM site"""
        if x is None or y is None:
            x, y = self.DEFAULT_POS
            
        new_sam = {
            'pos': (x, y),
            'range': range_val
        }
        self.sams.append(new_sam)
        return new_sam
    
    
    def remove_airport(self, index=None):
        """Remove airport by index or last one"""
        if not self.airports:
            return None
            
        if index is None:
            return self.airports.pop()
        elif 0 <= index < len(self.airports):
            return self.airports.pop(index)
        return None
    
    def remove_target(self, index=None):
        """Remove target by index or last one"""
        if not self.targets:
            return None
            
        if index is None:
            return self.targets.pop()
        elif 0 <= index < len(self.targets):
            return self.targets.pop(index)
        return None
    
    def remove_sam(self, index=None):
        """Remove SAM by index or last one"""
        if not self.sams:
            return None
            
        if index is None:
            return self.sams.pop()
        elif 0 <= index < len(self.sams):
            return self.sams.pop(index)
        return None
    
    
    def update_object_position(self, obj_type, index, x, y):
        """Update position of an object"""
        if obj_type == 'airport' and 0 <= index < len(self.airports):
            self.airports[index]['x'] = x
            self.airports[index]['y'] = y
            return True
        elif obj_type == 'target' and 0 <= index < len(self.targets):
            self.targets[index]['x'] = x
            self.targets[index]['y'] = y
            return True
        elif obj_type == 'sam' and 0 <= index < len(self.sams):
            self.sams[index]['pos'] = (x, y)
            return True
        return False
    
    def edit_target_priority(self, index):
        """Cycle target priority from 1-10"""
        if 0 <= index < len(self.targets):
            current = self.targets[index]['priority']
            new_priority = (current % 10) + 1
            self.targets[index]['priority'] = new_priority
            return current, new_priority
        return None, None
    
    def edit_sam_range(self, index):
        """Cycle SAM range through valid values"""
        if 0 <= index < len(self.sams):
            current = self.sams[index]['range']
            
            try:
                current_idx = self.VALID_SAM_RANGES.index(current)
                new_idx = (current_idx + 1) % len(self.VALID_SAM_RANGES)
            except ValueError:
                new_idx = 2  # Default to 10
            
            new_range = self.VALID_SAM_RANGES[new_idx]
            self.sams[index]['range'] = new_range
            return current, new_range
        return None, None
    
    def find_object_at_position(self, x, y, tolerance=2):
        """Find object at given position within tolerance"""
        # Check airports
        for i, airport in enumerate(self.airports):
            if abs(airport['x'] - x) < tolerance and abs(airport['y'] - y) < tolerance:
                return 'airport', i
        
        # Check targets
        for i, target in enumerate(self.targets):
            if abs(target['x'] - x) < tolerance and abs(target['y'] - y) < tolerance:
                return 'target', i
        
        # Check SAMs
        for i, sam in enumerate(self.sams):
            if abs(sam['pos'][0] - x) < tolerance and abs(sam['pos'][1] - y) < tolerance:
                return 'sam', i
        
        return None, None
    
    def get_default_sequence(self):
        """Generate default sequence with all current targets in order"""
        if not self.airports or not self.targets:
            return ""
        
        # Sort targets numerically by ID (T1, T2, T3, ..., T10, T11, etc.)
        sorted_targets = sorted(self.targets, key=lambda t: int(t['id'][1:]))
        
        sequence = [self.airports[0]]
        sequence.extend(sorted_targets)
        
        # End at second airport or return to first
        if len(self.airports) >= 2:
            sequence.append(self.airports[1])
        else:
            sequence.append(self.airports[0])
        
        return ','.join([wp['id'] for wp in sequence])
    
    def clear_all(self):
        """Clear all objects"""
        self.airports.clear()
        self.targets.clear()
        self.sams.clear()
    
    def import_environment(self, env_data):
        """Import environment from dictionary"""
        self.airports = env_data.get('airports', [])
        self.targets = env_data.get('targets', [])
        self.sams = env_data.get('sams', [])
    
    def export_environment(self):
        """Export environment to dictionary"""
        return {
            'airports': self.airports,
            'targets': self.targets,
            'sams': self.sams
        }