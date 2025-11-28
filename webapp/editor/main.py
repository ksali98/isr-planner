"""
ISR Editor Main Module
Modular implementation with clean architecture
"""
import json
import pickle
import os
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog

# Import modular components
from .ui import UILayout
from .solver import OrienteeringSolverInterface
from .path_planning import SAMNavigator
from .editor import ObjectManager, DragDropHandler
from .visualization import Renderer


class ISREditor:
    """Main ISR Editor application with modular architecture"""
    
    def __init__(self):
        # Initialize modules
        self.object_manager = ObjectManager()
        self.solver_interface = OrienteeringSolverInterface()
        self.sam_navigator = SAMNavigator()
        
        # UI state
        self.edit_mode = False  # Start with edit mode OFF (buttons hidden)
        self.animation_running = False
        self.mission_path = None
        self.selected_drone = 1  # Default drone selection
        self.selected_target_type = 'a'  # Default target type
        self.updating_ui = False  # Flag to prevent validation during UI updates
        
        # Multi-drone state - each drone has independent state
        self.drone_states = {}
        for i in range(1, 5):
            self.drone_states[i] = {
                'position': None,
                'traveled_path': [],
                'fuel_counter': 0,
                'start_airport': 'A1',
                'end_airport': 'A1',
                'fuel_budget': 999,
                'loiter_time': 0,
                'effective_range': 0,
                'target_access': {'a': True, 'b': True, 'c': True, 'd': True, 'e': True}  # Default: all types accessible
            }

        # Drone checkbox states - which drones are enabled for solving (all checked by default)
        self.drone_enabled = {1: True, 2: True, 3: True, 4: True}
        
        # Mission statistics
        self.starting_fuel = 999
        self.fuel_used = 0
        self.points_collected = 0
        self.visited_targets = 0
        self.target_fuel_usage = {}
        
        # Multi-drone missions
        self.drone_missions = {}  # Store separate missions for each drone
        self.multi_drone_mode = False  # Toggle for multi-drone visualization
        
        # Setup matplotlib figure
        self._setup_figure()
        
        # Setup modules
        self.renderer = Renderer(self.ax)
        self.drag_handler = DragDropHandler(self.object_manager)
        
        # Setup UI
        self.ui_layout = UILayout(self.fig)
        self.buttons, self.textboxes, self.button_axes = self.ui_layout.create_all_buttons()

        # Connect event handlers
        self._connect_event_handlers()

        # Set initial drone selection and target type
        self._update_drone_selection()
        self._update_target_type_selection()
        self._update_checkbox_states()

        # Hide edit buttons initially (edit_mode = False)
        self.ui_layout.buttons = self.buttons
        self.ui_layout.button_axes = self.button_axes
        self.ui_layout.update_button_states(self.edit_mode)

        # Initialize environment
        self.setup_environment()
        
    def _setup_figure(self):
        """Setup matplotlib figure and axes"""
        self.fig, self.ax = plt.subplots(figsize=(13, 9))
        self.fig.canvas.manager.set_window_title('ISR Mission Editor - Modular Version')
        plt.subplots_adjust(bottom=0.15, top=0.95, left=0.08, right=0.98)
        
    def _connect_event_handlers(self):
        """Connect all UI event handlers"""
        # Button callbacks
        self.buttons['edit'].on_clicked(self.toggle_edit_mode)
        self.buttons['add_target'].on_clicked(self.add_target)
        self.buttons['add_airport'].on_clicked(self.add_airport)
        self.buttons['add_sam'].on_clicked(self.add_sam)
        self.buttons['del_target'].on_clicked(self.remove_target)
        self.buttons['del_airport'].on_clicked(self.remove_airport)
        self.buttons['del_sam'].on_clicked(self.remove_sam)
        self.buttons['animate'].on_clicked(self.animate_mission)
        self.buttons['import'].on_clicked(self.import_environment)
        self.buttons['export'].on_clicked(self.export_environment)
        self.buttons['counter_up'].on_clicked(self.increment_fuel_counter)
        self.buttons['counter_down'].on_clicked(self.decrement_fuel_counter)
        self.buttons['segment'].on_clicked(self.save_segment)

        # Multi-drone mode toggle (repurpose segment button when not in animation)
        self.buttons['segment'].on_clicked(self.toggle_multi_drone_mode)

        # Multi-Drone Planner button
        self.buttons['multi_drone_planner'].on_clicked(self.launch_multi_drone_planner)

        # Drone selection buttons
        for i in range(1, 5):
            self.buttons[f'drone_{i}'].on_clicked(lambda event, drone=i: self.select_drone(event, drone))

        # Drone checkbox buttons
        for i in range(1, 5):
            self.buttons[f'drone_checkbox_{i}'].on_clicked(lambda event, drone=i: self.toggle_drone_checkbox(event, drone))

        # Target type selection buttons
        for target_type in ['a', 'b', 'c', 'd', 'e']:
            self.buttons[f'target_type_{target_type}'].on_clicked(lambda event, t_type=target_type: self.select_target_type(event, t_type))
        
        # Mouse events
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Textbox handlers
        self.textboxes['sequence'].on_text_change(self._validate_sequence_input)
        
    def setup_environment(self):
        """Initialize a default environment"""
        self.object_manager.clear_all()
        
        # Add default airports in clearly visible positions
        self.object_manager.add_airport(10, 10)  # A1 - bottom left, away from legend
        self.object_manager.add_airport(90, 10)  # A2 - bottom right, clearly visible
        
        # Add some default targets
        for i in range(3):
            x = np.random.randint(20, 80)
            y = np.random.randint(20, 80)
            priority = np.random.randint(1, 10)
            self.object_manager.add_target(x, y, priority)
        
        # Add a SAM
        self.object_manager.add_sam(50, 50, 15)
        
        # Update default sequence
        self._update_default_sequence()
        
        # Draw environment
        self.draw_environment()
        
    def _check_targets_in_sam_zones(self):
        """Check which targets are inside SAM zones and update targets_in_polygons"""
        self.targets_in_polygons = set()

        for target in self.object_manager.targets:
            tx, ty = target['x'], target['y']
            target_id = target['id']

            # Check against each SAM
            for sam in self.object_manager.sams:
                sam_x, sam_y = sam['pos']
                sam_range = sam['range']

                # Calculate distance from target to SAM center
                distance = ((tx - sam_x)**2 + (ty - sam_y)**2)**0.5

                # If target is inside SAM range, mark it
                if distance < sam_range:
                    self.targets_in_polygons.add(target_id)
                    break  # No need to check other SAMs

    def draw_environment(self):
        """Draw the complete environment"""
        self.renderer.clear()
        self.renderer.draw_grid()
        self.renderer.set_limits()

        # Check which targets are in SAM zones
        self._check_targets_in_sam_zones()

        # Draw all objects
        self.renderer.draw_airports(self.object_manager.airports, self.target_fuel_usage)
        self.renderer.draw_targets(
            self.object_manager.targets,
            self.target_fuel_usage,
            getattr(self, 'unreachable_targets', set()),
            self.targets_in_polygons,
            getattr(self, 'visited_target_ids', set())
        )
        self.renderer.draw_sams(self.object_manager.sams)
        
        # Draw wrapped SAM boundaries if any
        if self.object_manager.sams:
            self.renderer.draw_wrapped_boundaries(self.object_manager.sams)
        
        # Draw mission paths
        if self.multi_drone_mode and self.drone_missions:
            # Multi-drone mode: show only missions for enabled drones
            self.renderer.draw_multi_drone_paths(
                self.drone_missions,
                animation_running=self.animation_running,
                drone_states=self.drone_states,
                drone_enabled=self.drone_enabled
            )
            self.renderer.draw_multi_drone_positions(self.drone_states)
        else:
            # Single drone mode: show current mission path
            if self.mission_path:
                self.renderer.draw_mission_path(
                    self.traveled_path if self.animation_running else self.mission_path,
                    is_traveled=self.animation_running
                )
            
            # Draw current drone
            current_drone = self.drone_states[self.selected_drone]
            if current_drone['position']:
                fuel_used = calculate_distance(current_drone['traveled_path']) if current_drone['traveled_path'] else None
                self.renderer.draw_drone(current_drone['position'], fuel_used, self.selected_drone)
        
        # Draw legend and stats
        self.renderer.draw_legend(show_drone_paths=self.multi_drone_mode)
        
        # Calculate cumulative and individual drone stats
        cumulative_stats, individual_drone_stats = self._calculate_all_stats()
        
        stats = {
            'cumulative': cumulative_stats,
            'individual': individual_drone_stats,
            # Keep old format for backward compatibility
            'points': cumulative_stats['points'],
            'fuel_used': cumulative_stats['fuel_used'],
            'fuel_budget': cumulative_stats['fuel_budget'],
            'visited': cumulative_stats['visited'],
            'total_targets': cumulative_stats['total_targets'],
            'points_per_fuel': cumulative_stats['points_per_fuel']
        }
        self.renderer.draw_stats(stats)
        
        # Draw individual drone stats boxes directly on plot
        self.renderer.draw_drone_stats_boxes(individual_drone_stats)
        
        plt.draw()
        
    def toggle_edit_mode(self, event):
        """Toggle edit mode on/off"""
        self.edit_mode = not self.edit_mode
        ui_layout = UILayout(self.fig)
        ui_layout.buttons = self.buttons
        ui_layout.button_axes = self.button_axes
        ui_layout.update_button_states(self.edit_mode)
        print(f"Edit mode: {'ON' if self.edit_mode else 'OFF'}")
        
    def add_target(self, event):
        """Add a new target of selected type"""
        if not self.edit_mode:
            return
        target = self.object_manager.add_target(target_type=self.selected_target_type)
        print(f"Added {target['id']} (Type {self.selected_target_type.upper()}) at ({target['x']}, {target['y']})")
        self._update_default_sequence()
        self.draw_environment()
        
    def add_airport(self, event):
        """Add a new airport"""
        if not self.edit_mode:
            return
        airport = self.object_manager.add_airport()
        print(f"Added {airport['id']} at ({airport['x']}, {airport['y']})")
        self._update_default_sequence()
        self.draw_environment()
        
    def add_sam(self, event):
        """Add a new SAM"""
        if not self.edit_mode:
            return
        sam = self.object_manager.add_sam()
        print(f"Added SAM at {sam['pos']}")
        self.draw_environment()
        
        
    def remove_target(self, event):
        """Remove the last target"""
        if not self.edit_mode:
            return
        removed = self.object_manager.remove_target()
        if removed:
            print(f"Removed {removed['id']}")
            self._update_default_sequence()
            self.draw_environment()
            
    def remove_airport(self, event):
        """Remove the last airport"""
        if not self.edit_mode:
            return
        removed = self.object_manager.remove_airport()
        if removed:
            print(f"Removed {removed['id']}")
            self._update_default_sequence()
            self.draw_environment()
            
    def remove_sam(self, event):
        """Remove the last SAM"""
        if not self.edit_mode:
            return
        removed = self.object_manager.remove_sam()
        if removed:
            print("Removed SAM")
            self.draw_environment()
            
        
    def clear_trajectory(self, event):
        """Clear all trajectories"""
        self.mission_path = None
        self.drone_position = None
        self.traveled_path = []

        # Clear all drone missions
        self.drone_missions = {}

        # Reset all drone positions
        for drone_id in range(1, 5):
            self.drone_states[drone_id]['position'] = None
            self.drone_states[drone_id]['traveled_path'] = []

        # Disable multi-drone mode
        self.multi_drone_mode = False

        self.draw_environment()
        print("All trajectories cleared")
        
    def plan_mission(self, event, skip_draw=False):
        """Plan mission from sequence in textbox"""
        sequence_text = self.textboxes['sequence'].text.strip()
        if not sequence_text:
            print("No sequence provided")
            return

        # Parse sequence - handle labeled format "D1: A1,T2,A1  D2:  D3:  D4:"
        if 'D1:' in sequence_text or 'D2:' in sequence_text or 'D3:' in sequence_text or 'D4:' in sequence_text:
            # Extract sequence for selected drone
            import re
            drone_label = f'D{self.selected_drone}:'

            # Find the sequence for this drone
            # Pattern: D1: sequence  D2: sequence  etc.
            pattern = f'{drone_label}\\s*([A-Z0-9,\\s]*?)(?:\\s+D\\d:|$)'
            match = re.search(pattern, sequence_text)

            if match:
                drone_sequence = match.group(1).strip()
                if drone_sequence:
                    waypoint_ids = [wp.strip() for wp in drone_sequence.split(',') if wp.strip()]
                else:
                    print(f"No sequence for D{self.selected_drone}")
                    return
            else:
                print(f"No sequence found for D{self.selected_drone}")
                return
        else:
            # Legacy format: plain comma-separated sequence
            waypoint_ids = [wp.strip() for wp in sequence_text.split(',')]

        # Plan path
        path = []
        for i in range(len(waypoint_ids) - 1):
            start_id = waypoint_ids[i]
            end_id = waypoint_ids[i + 1]

            start_pos = self._get_waypoint_position(start_id)
            end_pos = self._get_waypoint_position(end_id)

            if start_pos and end_pos:
                # Plan segment with SAM avoidance
                segment_path, _, _ = self.sam_navigator.plan_path_with_sam_avoidance(
                    start_pos, end_pos, self.object_manager.sams
                )

                # Add segment to path (avoiding duplicates)
                if i == 0:
                    path.extend(segment_path)
                else:
                    path.extend(segment_path[1:])

        if path:
            self.mission_path = path
            # Set current drone's position to start of path
            self.drone_states[self.selected_drone]['position'] = path[0]
            # Reset current drone's fuel counter
            self.drone_states[self.selected_drone]['fuel_counter'] = 0
            self.textboxes['fuel_counter'].set_val('0')

            # Calculate points from visited targets
            visited_targets = [wid for wid in waypoint_ids if wid.startswith('T')]
            total_points = 0
            for target_id in visited_targets:
                target = next((t for t in self.object_manager.targets if t['id'] == target_id), None)
                if target:
                    total_points += target.get('priority', 0)

            # Calculate loiter fuel for this drone
            loiter_time = self.drone_states[self.selected_drone].get('loiter_time', 0)
            loiter_fuel = len(visited_targets) * loiter_time

            # Store mission for current drone
            self.drone_missions[self.selected_drone] = {
                'path': path,
                'sequence': sequence_text,
                'description': f"Mission {len(waypoint_ids)-1} waypoints",
                'distance': calculate_distance(path),
                'loiter_fuel': loiter_fuel,
                'points': total_points,
                'num_targets': len(visited_targets)
            }

            print(f"Drone {self.selected_drone} mission planned: {len(path)} waypoints, "
                  f"distance: {calculate_distance(path):.1f}")

        # Only draw if not skipped (skip during multi-drone solving to prevent flicker)
        if not skip_draw:
            self.draw_environment()

    def animate_mission(self, event):
        """Toggle mission animation - supports multi-drone mode"""
        # Check if we have missions to animate
        if self.drone_missions:
            self._animate_multi_drone()
        elif self.mission_path:
            self._animate_single_drone()
        else:
            print("No mission path to animate")
            return
    
    def _animate_single_drone(self):
        """Animate single drone mission with smooth interpolation"""
        if self.animation_running:
            # Stop animation
            self.animation_running = False
            if hasattr(self, 'animation'):
                self.animation.event_source.stop()
            self.buttons['animate'].label.set_text('Start')
            print("Animation stopped")
        else:
            # Start smooth animation
            self.animation_running = True
            self.traveled_path = [self.mission_path[0]]
            
            # Initialize smooth animation for single drone
            total_distance = sum(
                np.linalg.norm(np.array(self.mission_path[i+1]) - np.array(self.mission_path[i]))
                for i in range(len(self.mission_path) - 1)
            )
            
            # Get loiter time for selected drone
            loiter_time = self.drone_states[self.selected_drone].get('loiter_time', 0)

            # Find target positions in path for loitering
            target_positions = self._find_target_positions_in_path(self.mission_path, self.selected_drone)

            self.single_animation_data = {
                'path': self.mission_path,
                'total_distance': total_distance,
                'current_distance': 0.0,
                'speed': max(1.0, total_distance / 200.0),
                'traveled_path': [self.mission_path[0]],
                'loiter_time': loiter_time,
                'target_positions': target_positions,
                'loiter_remaining': 0,  # Frames remaining at current target
                'visited_targets': set()  # Track which targets we've loitered at
            }
            
            # Set initial position for selected drone
            self.drone_states[self.selected_drone]['position'] = self.mission_path[0]
            self.drone_states[self.selected_drone]['traveled_path'] = [self.mission_path[0]]
            
            self.buttons['animate'].label.set_text('Stop')
            print("Smooth animation started")
            
            # Start smooth animation loop
            import matplotlib.animation as animation
            self.animation = animation.FuncAnimation(
                self.fig, self._update_single_drone_animation, 
                interval=50, blit=False, repeat=False
            )
    
    def _update_single_drone_animation(self, frame):
        """Update single drone animation with smooth interpolation and loitering"""
        if not self.animation_running or not hasattr(self, 'single_animation_data'):
            return

        anim_data = self.single_animation_data

        if anim_data['current_distance'] < anim_data['total_distance']:
            # Check if currently loitering at a target
            if anim_data['loiter_remaining'] > 0:
                # Loitering - don't advance distance
                anim_data['loiter_remaining'] -= 1
            else:
                # Check if we just reached a target
                current_pos = self._get_position_at_distance(anim_data)
                for target_id, target_distance in anim_data['target_positions'].items():
                    # If we're at or past this target and haven't loitered yet
                    if (target_id not in anim_data['visited_targets'] and
                        abs(anim_data['current_distance'] - target_distance) < anim_data['speed']):
                        # Start loitering at this target
                        anim_data['loiter_remaining'] = int(anim_data['loiter_time'])
                        anim_data['visited_targets'].add(target_id)
                        print(f"   Loitering at {target_id} for {anim_data['loiter_time']} frames")
                        break

                # Advance drone along path (unless we just started loitering)
                if anim_data['loiter_remaining'] == 0:
                    anim_data['current_distance'] += anim_data['speed']

            # Find current position by interpolating along path
            new_position = self._get_position_at_distance(anim_data)

            # Update drone state
            self.drone_states[self.selected_drone]['position'] = new_position

            # Add to traveled path
            if len(anim_data['traveled_path']) == 0 or \
               np.linalg.norm(np.array(new_position) - np.array(anim_data['traveled_path'][-1])) > 1.0:
                anim_data['traveled_path'].append(new_position)
                self.drone_states[self.selected_drone]['traveled_path'] = anim_data['traveled_path'].copy()
                self.traveled_path = anim_data['traveled_path'].copy()
        else:
            # Animation complete
            self.animation_running = False
            if hasattr(self, 'animation'):
                self.animation.event_source.stop()
            self.buttons['animate'].label.set_text('Start')
            print("Smooth animation completed")

        # Redraw environment
        self.draw_environment()
        return []
            
    def _animate_multi_drone(self):
        """Animate multiple drones simultaneously with smooth interpolation"""
        if self.animation_running:
            # Stop multi-drone animation
            self.animation_running = False
            if hasattr(self, 'multi_animation'):
                self.multi_animation.event_source.stop()
            self.buttons['animate'].label.set_text('Start')
            print("Multi-drone animation stopped")
        else:
            # Start multi-drone animation
            self.animation_running = True
            self.buttons['animate'].label.set_text('Stop')
            
            # Initialize smooth animation data for each drone
            self.animation_data = {}
            for drone_id, mission in self.drone_missions.items():
                if 'path' in mission and mission['path']:
                    path = mission['path']
                    # Calculate total distance for this drone's path
                    total_distance = sum(
                        np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
                        for i in range(len(path) - 1)
                    )

                    # Get loiter time for this drone
                    loiter_time = self.drone_states[drone_id].get('loiter_time', 0)

                    # Find target positions for loitering
                    target_positions = self._find_target_positions_in_path(path, drone_id)

                    self.animation_data[drone_id] = {
                        'path': path,
                        'total_distance': total_distance,
                        'current_distance': 0.0,
                        'speed': max(1.0, total_distance / 200.0),  # Adjust speed based on path length
                        'traveled_path': [path[0]],
                        'current_segment': 0,
                        'segment_progress': 0.0,
                        'loiter_time': loiter_time,
                        'target_positions': target_positions,
                        'loiter_remaining': 0,
                        'visited_targets': set()
                    }
                    # Set initial position
                    self.drone_states[drone_id]['position'] = path[0]
                    self.drone_states[drone_id]['traveled_path'] = [path[0]]
            
            print(f"Smooth multi-drone animation started: {len(self.animation_data)} drones")
            
            # Start animation loop with higher frequency for smoothness
            import matplotlib.animation as animation
            self.multi_animation = animation.FuncAnimation(
                self.fig, self._update_multi_drone_animation, 
                interval=50, blit=False, repeat=False  # 20 FPS for smooth animation
            )
    
    def _update_multi_drone_animation(self, frame):
        """Update animation for all drones with smooth interpolation and loitering"""
        if not self.animation_running:
            return

        all_complete = True

        # Update each drone's position smoothly
        for drone_id, anim_data in self.animation_data.items():
            if anim_data['current_distance'] < anim_data['total_distance']:
                all_complete = False

                # Check if currently loitering at a target
                if anim_data['loiter_remaining'] > 0:
                    # Loitering - don't advance distance
                    anim_data['loiter_remaining'] -= 1
                else:
                    # Check if we just reached a target
                    for target_id, target_distance in anim_data['target_positions'].items():
                        # If we're at or past this target and haven't loitered yet
                        if (target_id not in anim_data['visited_targets'] and
                            abs(anim_data['current_distance'] - target_distance) < anim_data['speed']):
                            # Start loitering at this target
                            anim_data['loiter_remaining'] = int(anim_data['loiter_time'])
                            anim_data['visited_targets'].add(target_id)
                            print(f"   Drone {drone_id} loitering at {target_id} for {anim_data['loiter_time']} frames")
                            break

                    # Advance drone along path (unless we just started loitering)
                    if anim_data['loiter_remaining'] == 0:
                        anim_data['current_distance'] += anim_data['speed']

                # Find current position by interpolating along path
                new_position = self._get_position_at_distance(anim_data)

                # Update drone state
                self.drone_states[drone_id]['position'] = new_position

                # Add to traveled path (sample every few updates to avoid too many points)
                if len(anim_data['traveled_path']) == 0 or \
                   np.linalg.norm(np.array(new_position) - np.array(anim_data['traveled_path'][-1])) > 1.0:
                    anim_data['traveled_path'].append(new_position)
                    self.drone_states[drone_id]['traveled_path'] = anim_data['traveled_path'].copy()

        # Redraw environment with updated positions
        self.draw_environment()

        # Stop animation if all drones are complete
        if all_complete:
            self.animation_running = False
            if hasattr(self, 'multi_animation'):
                self.multi_animation.event_source.stop()
            self.buttons['animate'].label.set_text('Start')
            print("Smooth multi-drone animation completed")

        return []
    
    def _find_target_positions_in_path(self, path, drone_id):
        """Find cumulative distance to each target in the path

        Returns:
            dict: {target_id: distance_along_path}
        """
        # Get mission sequence to identify targets
        mission = self.drone_missions.get(drone_id, {})
        if not mission or 'sequence' not in mission:
            return {}

        sequence = mission['sequence']
        waypoints = [wp.strip() for wp in sequence.split(',')]
        targets_in_sequence = [wp for wp in waypoints if wp.startswith('T')]

        # Calculate distance to each waypoint in sequence
        target_positions = {}
        accumulated_distance = 0.0

        # We need to map sequence waypoints to path segments
        # The path is a list of coordinates, but we need to know which segments correspond to which waypoint transitions
        # For simplicity, we'll estimate based on the waypoint positions

        for target_id in targets_in_sequence:
            target_pos = self._get_waypoint_position(target_id)
            if not target_pos:
                continue

            # Find closest point in path to this target
            min_dist_to_target = float('inf')
            best_path_distance = 0.0

            current_path_distance = 0.0
            for i in range(len(path)):
                path_point = path[i]
                dist = ((path_point[0] - target_pos[0])**2 + (path_point[1] - target_pos[1])**2)**0.5

                if dist < min_dist_to_target:
                    min_dist_to_target = dist
                    best_path_distance = current_path_distance

                # Calculate distance for next iteration
                if i < len(path) - 1:
                    segment_length = np.linalg.norm(np.array(path[i+1]) - np.array(path[i]))
                    current_path_distance += segment_length

            # Only add if we found a reasonable match (within 5 units)
            if min_dist_to_target < 5.0:
                target_positions[target_id] = best_path_distance

        return target_positions

    def _get_position_at_distance(self, anim_data):
        """Calculate drone position at given distance along path"""
        path = anim_data['path']
        target_distance = anim_data['current_distance']

        if target_distance <= 0:
            return path[0]

        # Walk along path segments to find current position
        accumulated_distance = 0.0

        for i in range(len(path) - 1):
            start_point = np.array(path[i])
            end_point = np.array(path[i + 1])
            segment_length = np.linalg.norm(end_point - start_point)

            if accumulated_distance + segment_length >= target_distance:
                # Position is on this segment
                remaining_distance = target_distance - accumulated_distance
                t = remaining_distance / segment_length if segment_length > 0 else 0

                # Linear interpolation between waypoints
                interpolated_pos = start_point + t * (end_point - start_point)
                return interpolated_pos.tolist()

            accumulated_distance += segment_length

        # If we've gone past the end, return the last point
        return path[-1]
            
    def import_environment(self, event):
        """Import environment from JSON file"""
        import subprocess
        
        # Use native macOS file dialog for opening
        script = '''
        tell application "Finder"
            activate
            try
                set openPath to choose file with prompt "Select environment file to import:" of type {"public.json"}
                return POSIX path of openPath
            on error
                return ""
            end try
        end tell
        '''
        
        try:
            # Show open dialog
            result = subprocess.run(['osascript', '-e', script], 
                                  capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0 and result.stdout.strip():
                filepath = result.stdout.strip()
                
                # Load the file
                with open(filepath, 'r') as f:
                    data = json.load(f)
                
                # Clear current environment
                self.object_manager.clear_all()
                
                # Import objects
                self.object_manager.import_environment(data)
                
                # Import sequence if present
                if 'sequence' in data:
                    self.textboxes['sequence'].set_val(data['sequence'])
                
                # Import drone states if present
                if 'drone_states' in data:
                    for drone_num, state in data['drone_states'].items():
                        drone_idx = int(drone_num)
                        if drone_idx in self.drone_states:
                            self.drone_states[drone_idx].update(state)
                
                # Import selections
                if 'selected_drone' in data:
                    self.selected_drone = data['selected_drone']
                    self._update_drone_selection()
                    self._update_ui_for_selected_drone()
                
                if 'selected_target_type' in data:
                    self.selected_target_type = data['selected_target_type']
                    self._update_target_type_selection()
                
                print(f"✅ Environment imported from: {filepath}")
                print(f"   Airports: {len(self.object_manager.airports)}")
                print(f"   Targets: {len(self.object_manager.targets)} (types: {', '.join(set(t.get('type', 'a') for t in self.object_manager.targets))})")
                print(f"   SAMs: {len(self.object_manager.sams)}")
                
                if 'distance_matrix' in data:
                    print(f"   Distance matrix: {len(data['matrix_labels'])}x{len(data['matrix_labels'])} (SAM-aware)")
                
                if 'metadata' in data:
                    print(f"   Version: {data['metadata'].get('version', 'unknown')}")
                    print(f"   Export date: {data['metadata'].get('export_timestamp', 'unknown')}")
                
                # Update default sequence if none provided
                if 'sequence' not in data:
                    self._update_default_sequence()
                
                self.draw_environment()
                
            else:
                print("Import cancelled")
                
        except subprocess.TimeoutExpired:
            print("❌ Import dialog timed out")
        except Exception as e:
            print(f"❌ Error importing environment: {e}")
            import traceback
            traceback.print_exc()
        
    def export_environment(self, event):
        """Export environment to JSON file"""
        import subprocess
        from datetime import datetime
        
        # Create default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"environment_{timestamp}.json"
        
        # Use native macOS file dialog
        script = f'''
        tell application "Finder"
            activate
            try
                set savePath to choose file name with prompt "Save environment as:" default name "{default_filename}"
                return POSIX path of savePath
            on error
                return ""
            end try
        end tell
        '''
        
        try:
            # Show save dialog
            result = subprocess.run(['osascript', '-e', script], 
                                  capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0 and result.stdout.strip():
                filepath = result.stdout.strip()
                
                # Ensure .json extension
                if not filepath.endswith('.json'):
                    filepath += '.json'
                
                # Build comprehensive export data
                data = self.object_manager.export_environment()

                # Add mission parameters
                data['sequence'] = self.textboxes['sequence'].text
                
                # Add drone states
                data['drone_states'] = {}
                for drone_num, state in self.drone_states.items():
                    data['drone_states'][drone_num] = {
                        'fuel_counter': state['fuel_counter'],
                        'start_airport': state['start_airport'],
                        'end_airport': state['end_airport'],
                        'fuel_budget': state['fuel_budget']
                    }
                
                # Add current selections
                data['selected_drone'] = self.selected_drone
                data['selected_target_type'] = self.selected_target_type
                
                # Calculate SAM-aware distance matrix
                print("Calculating SAM-aware distance matrix...")
                distance_matrix_data = self._calculate_distance_matrix()
                data['distance_matrix'] = distance_matrix_data['matrix']
                data['matrix_labels'] = distance_matrix_data['labels']
                
                # Add metadata
                data['metadata'] = {
                    'export_timestamp': datetime.now().isoformat(),
                    'version': '2.0',
                    'features': {
                        'multi_drone': True,
                        'target_types': True,
                        'sam_aware_distances': True
                    }
                }
                
                # Save the file
                with open(filepath, 'w') as f:
                    json.dump(data, f, indent=2)
                
                print(f"✅ Environment exported to: {filepath}")
                
            else:
                print("Export cancelled")
                
        except subprocess.TimeoutExpired:
            print("❌ Export dialog timed out")
        except Exception as e:
            print(f"❌ Error exporting environment: {e}")
            import traceback
            traceback.print_exc()
        
    def save_segment(self, event):
        """Placeholder for segment saving functionality"""
        print("Segment saving not implemented in modular version yet")
    
    def toggle_multi_drone_mode(self, event):
        """Toggle between single and multi-drone visualization"""
        self.multi_drone_mode = not self.multi_drone_mode

        if self.multi_drone_mode:
            print("🚁 Multi-drone mode: ON")
            print(f"   Active missions: {list(self.drone_missions.keys())}")
            for drone_id, mission in self.drone_missions.items():
                print(f"   Drone {drone_id}: {mission['description']} (dist: {mission['distance']:.1f})")
        else:
            print("🚁 Multi-drone mode: OFF (showing single drone)")

        self.draw_environment()

    def launch_multi_drone_planner(self, event):
        """Launch the web-based multi-drone mission planner"""
        import subprocess
        import os
        from datetime import datetime

        print("🚁 Launching Web-based Mission Planner...")

        # Step 1: Auto-export environment to temp file for planner
        try:
            # Export environment to a temp file
            export_data = self.object_manager.export_environment()
            export_data['sequence'] = self.textboxes['sequence'].text

            # Add drone states
            export_data['drone_states'] = {}
            for drone_num, state in self.drone_states.items():
                export_data['drone_states'][drone_num] = {
                    'fuel_counter': state['fuel_counter'],
                    'start_airport': state['start_airport'],
                    'end_airport': state['end_airport'],
                    'fuel_budget': state['fuel_budget']
                }

            # Calculate distance matrix
            distance_matrix_data = self._calculate_distance_matrix()
            export_data['distance_matrix'] = distance_matrix_data['matrix']
            export_data['matrix_labels'] = distance_matrix_data['labels']

            # Save to temp file for planner
            with open('tmp_environment.json', 'w') as f:
                json.dump(export_data, f, indent=2)

            print("✅ Environment saved for mission planner")

        except Exception as e:
            print(f"⚠️  Warning: Could not export environment: {e}")

        # Step 2: Launch the web planner
        try:
            # Check if planner is already running
            result = subprocess.run(['lsof', '-i', ':8891'],
                                  capture_output=True, text=True)

            if result.stdout:
                print("🌐 Web Mission Planner already running!")
                print("📍 Open http://localhost:8891 in your browser")
            else:
                # Start the clean planner in background with logging
                log_file = open('planner.log', 'w')
                subprocess.Popen(['python3', 'mission_planner_with_ai_chat.py'],
                               stdout=log_file,
                               stderr=log_file)
                print("🌐 Web Mission Planner launched!")
                print("📍 Open http://localhost:8891 in your browser")
                print("📝 Planner logs: planner.log")

        except Exception as e:
            print(f"❌ Error launching planner: {e}")

        # Step 3: Start polling for mission solution updates
        if not hasattr(self, 'solution_timer'):
            self._start_mission_solution_timer()

        print("💡 The UI will auto-update when the editor environment changes")

    def _start_mission_solution_timer(self):
        """Start a timer to poll for mission_solution.json updates"""
        from matplotlib.animation import FuncAnimation

        def check_for_updates(frame):
            try:
                if os.path.exists('mission_solution.json'):
                    with open('mission_solution.json', 'r') as f:
                        solution_data = json.load(f)

                    # Update drone configurations from solution
                    if 'drone_configs' in solution_data:
                        for drone_id, config in solution_data['drone_configs'].items():
                            drone_id = int(drone_id)
                            if drone_id in self.drone_states:
                                self.drone_states[drone_id]['fuel_budget'] = config.get('fuel_budget', 999)
                                self.drone_states[drone_id]['start_airport'] = config.get('start_airport', 'A1')
                                self.drone_states[drone_id]['end_airport'] = config.get('end_airport', 'A1')
                                self.drone_states[drone_id]['speed'] = config.get('speed', 3.0)
                                self.drone_states[drone_id]['loiter_time'] = config.get('loiter_time', 0)
                                self.drone_states[drone_id]['effective_range'] = config.get('effective_range', 0)

                                # Load target accessibility settings
                                if 'target_access' in config:
                                    self.drone_states[drone_id]['target_access'] = config['target_access']

                    os.remove('mission_solution.json')
                    print("✅ Mission configuration updated from planner")

                    # Update UI to reflect new drone settings
                    if self.selected_drone in self.drone_states:
                        self._update_ui_for_selected_drone()

                    # Redraw environment to update stats boxes
                    self.draw_environment()

                # Check for AI solution (sequences from orienteering solver)
                if os.path.exists('ai_solution.json'):
                    with open('ai_solution.json', 'r') as f:
                        ai_solution = json.load(f)

                    # Load sequences for all drones
                    if 'sequences' in ai_solution:
                        sequences = ai_solution['sequences']

                        # Format as "D1: seq1  D2: seq2  D3: seq3  D4: seq4"
                        formatted_sequences = []
                        for drone_id in range(1, 5):
                            seq = sequences.get(str(drone_id), '')
                            formatted_sequences.append(f"D{drone_id}: {seq}")

                        # Update the sequence textbox
                        sequence_text = "  ".join(formatted_sequences)
                        self.textboxes['sequence'].set_val(sequence_text)

                        print(f"✅ AI solution loaded:")
                        for drone_id in range(1, 5):
                            seq = sequences.get(str(drone_id), '')
                            if seq:
                                print(f"   D{drone_id}: {seq}")

                        # Auto-plan missions for all enabled drones
                        self._plan_all_enabled_drones()

                    os.remove('ai_solution.json')

            except:
                pass

            return []

        # Check every 2 seconds
        self.solution_timer = FuncAnimation(
            self.fig, check_for_updates, interval=2000,
            blit=False, cache_frame_data=False
        )

    def assign_target_to_drone(self, target_id, drone_id):
        """Assign a target to a specific drone"""
        # Find and update target
        for target in self.object_manager.targets:
            if target['id'] == target_id:
                target['assigned_drone'] = drone_id
                print(f"Assigned {target_id} to Drone {drone_id}")
                break
        self.draw_environment()
    
    def plan_all_drone_missions(self):
        """Plan optimal missions for all drones using the solver"""
        # Group targets by assigned drone
        drone_targets = {1: [], 2: [], 3: [], 4: []}
        
        for target in self.object_manager.targets:
            assigned_drone = target.get('assigned_drone', 1)
            drone_targets[assigned_drone].append(target['id'])
        
        # Plan mission for each drone that has targets
        for drone_id, target_ids in drone_targets.items():
            if not target_ids:
                continue
                
            drone_state = self.drone_states[drone_id]
            start_airport = drone_state['start_airport']
            end_airport = drone_state['end_airport']
            fuel_budget = drone_state['fuel_budget']
            
            print(f"🧠 Solving orienteering for Drone {drone_id}: {len(target_ids)} targets")
            
            # Build environment for solver with only this drone's targets
            drone_env = self._build_drone_environment(drone_id, target_ids, start_airport, end_airport)
            
            if drone_env:
                try:
                    # Solve orienteering problem for this drone
                    solution = self.solver_interface.solve(drone_env, fuel_budget)
                    
                    # Check if solution is valid
                    if solution['distance'] > fuel_budget:
                        print(f"⚠️  Drone {drone_id} solution exceeds fuel budget, finding valid solution...")
                        solution = self.solver_interface.find_valid_fuel_solution(drone_env, fuel_budget)
                        if not solution:
                            print(f"❌ No valid solution found for Drone {drone_id}")
                            continue
                    
                    # Plan the optimal path using solver sequence
                    optimal_sequence = ','.join(solution['route'])
                    waypoint_ids = solution['route']
                    path = []
                    
                    for i in range(len(waypoint_ids) - 1):
                        start_id = waypoint_ids[i]
                        end_id = waypoint_ids[i + 1]
                        
                        start_pos = self._get_waypoint_position(start_id)
                        end_pos = self._get_waypoint_position(end_id)
                        
                        if start_pos and end_pos:
                            segment_path, _, _ = self.sam_navigator.plan_path_with_sam_avoidance(
                                start_pos, end_pos, self.object_manager.sams
                            )
                            
                            if i == 0:
                                path.extend(segment_path)
                            else:
                                path.extend(segment_path[1:])
                    
                    if path:
                        # Calculate loiter fuel for this drone
                        loiter_time = self.drone_states[drone_id].get('loiter_time', 0)
                        loiter_fuel = len(target_ids) * loiter_time

                        # Store optimal mission
                        self.drone_missions[drone_id] = {
                            'path': path,
                            'sequence': optimal_sequence,
                            'description': f"{len(target_ids)} targets (optimized)",
                            'distance': calculate_distance(path),
                            'loiter_fuel': loiter_fuel,
                            'points': solution['total_points'],
                            'num_targets': len(target_ids)
                        }
                        
                        # Set drone position
                        self.drone_states[drone_id]['position'] = path[0]
                        
                        print(f"✅ Drone {drone_id}: {optimal_sequence}")
                        print(f"   Distance: {calculate_distance(path):.1f}, Points: {solution['total_points']}")
                    
                except Exception as e:
                    print(f"❌ Error solving for Drone {drone_id}: {e}")
                    # Fallback to simple sequence
                    self._plan_simple_drone_mission(drone_id, target_ids, start_airport, end_airport)
            else:
                print(f"⚠️  Could not build environment for Drone {drone_id}")
        
        # Switch to multi-drone mode
        self.multi_drone_mode = True
        self.draw_environment()
    
    def _build_drone_environment(self, drone_id, target_ids, start_airport, end_airport):
        """Build environment for solver with only assigned targets"""
        # Get only targets assigned to this drone
        drone_targets = []
        for target in self.object_manager.targets:
            if target['id'] in target_ids:
                drone_targets.append(target)
        
        if not drone_targets:
            return None
            
        # Calculate distance matrix for this drone's environment
        distance_matrix_data = self._calculate_drone_distance_matrix(drone_targets)
        
        # Build environment
        env_data = self.solver_interface.build_environment_for_solver(
            self.object_manager.airports,
            drone_targets,
            self.object_manager.sams,
            distance_matrix_data
        )
        
        env_data['start_airport'] = start_airport
        env_data['end_airport'] = end_airport
        
        return env_data
    
    def _calculate_drone_distance_matrix(self, drone_targets):
        """Calculate distance matrix for specific drone targets"""
        all_waypoints = self.object_manager.airports + drone_targets
        
        n = len(all_waypoints)
        matrix = [[0] * n for _ in range(n)]
        labels = []
        
        # Build labels
        for wp in all_waypoints:
            labels.append(wp['id'])
            
        # Calculate SAM-aware distances
        for i in range(n):
            for j in range(n):
                if i != j:
                    pos_i = self._get_waypoint_position(labels[i])
                    pos_j = self._get_waypoint_position(labels[j])
                    if pos_i and pos_j:
                        # Use SAM navigation to get actual path distance
                        path, distance, _ = self.sam_navigator.plan_path_with_sam_avoidance(
                            pos_i, pos_j, self.object_manager.sams
                        )
                        matrix[i][j] = distance
                else:
                    matrix[i][j] = 0
        
        # Include detailed waypoint info
        waypoint_details = []
        for wp in all_waypoints:
            detail = {
                'id': wp['id'],
                'x': wp['x'],
                'y': wp['y']
            }
            # Add target-specific info
            if wp['id'].startswith('T'):
                detail['priority'] = wp.get('priority', 5)
                detail['type'] = wp.get('type', 'a')
            waypoint_details.append(detail)
        
        return {
            'matrix': matrix,
            'labels': labels,
            'waypoints': waypoint_details,
            'excluded_targets': []
        }
    
    def _plan_simple_drone_mission(self, drone_id, target_ids, start_airport, end_airport):
        """Fallback: plan simple mission without solver"""
        sequence = f"{start_airport},{','.join(target_ids)},{end_airport}"
        waypoint_ids = [wp.strip() for wp in sequence.split(',')]
        path = []
        
        for i in range(len(waypoint_ids) - 1):
            start_id = waypoint_ids[i]
            end_id = waypoint_ids[i + 1]
            
            start_pos = self._get_waypoint_position(start_id)
            end_pos = self._get_waypoint_position(end_id)
            
            if start_pos and end_pos:
                segment_path, _, _ = self.sam_navigator.plan_path_with_sam_avoidance(
                    start_pos, end_pos, self.object_manager.sams
                )
                
                if i == 0:
                    path.extend(segment_path)
                else:
                    path.extend(segment_path[1:])
        
        if path:
            # Calculate loiter fuel for this drone
            loiter_time = self.drone_states[drone_id].get('loiter_time', 0)
            loiter_fuel = len(target_ids) * loiter_time

            self.drone_missions[drone_id] = {
                'path': path,
                'sequence': sequence,
                'description': f"{len(target_ids)} targets (simple)",
                'distance': calculate_distance(path),
                'loiter_fuel': loiter_fuel,
                'num_targets': len(target_ids)
            }
            self.drone_states[drone_id]['position'] = path[0]
            print(f"✅ Drone {drone_id}: {sequence} (simple plan, distance: {calculate_distance(path):.1f})")
        
    def select_drone(self, event, drone_number):
        """Select a drone and update button colors and UI values"""
        self.selected_drone = drone_number
        self._update_drone_selection()
        self._update_ui_for_selected_drone()
        print(f"Selected drone {drone_number}")
        
    def _update_drone_selection(self):
        """Update drone button colors based on selection"""
        # Drone colors (matching trajectory colors)
        drone_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']  # blue, orange, green, red

        for i in range(1, 5):
            drone_btn = self.buttons[f'drone_{i}']
            # Always show trajectory color (not selection-based)
            drone_btn.color = drone_colors[i-1]
            drone_btn.hovercolor = drone_colors[i-1]

        # Update checkbox states
        self._update_checkbox_states()

        plt.draw()

    def _update_checkbox_states(self):
        """Update checkbox visual states based on enabled status"""
        for i in range(1, 5):
            checkbox_btn = self.buttons[f'drone_checkbox_{i}']
            if self.drone_enabled[i]:
                # Checked - green with checkmark
                checkbox_btn.color = '#4CAF50'
                checkbox_btn.hovercolor = '#66BB6A'
                checkbox_btn.label.set_text('✓')
            else:
                # Unchecked - gray with empty box
                checkbox_btn.color = '0.85'
                checkbox_btn.hovercolor = '0.95'
                checkbox_btn.label.set_text('')

    def toggle_drone_checkbox(self, event, drone_number):
        """Toggle drone checkbox state and auto-display/hide trajectory"""
        self.drone_enabled[drone_number] = not self.drone_enabled[drone_number]
        self._update_checkbox_states()

        # Auto-display trajectory when checkbox is checked
        if self.drone_enabled[drone_number]:
            # Plan missions for ALL enabled drones (to preserve existing trajectories)
            sequence_text = self.textboxes['sequence'].text.strip()
            if sequence_text:
                self._plan_all_enabled_drones()
                print(f"Drone {drone_number}: Enabled (trajectory displayed)")
            else:
                print(f"Drone {drone_number}: Enabled (no sequence to display)")
        else:
            # Hide trajectory when checkbox is unchecked
            if self.selected_drone == drone_number:
                self.mission_path = None
            # Remove this drone's mission when unchecked
            if drone_number in self.drone_missions:
                del self.drone_missions[drone_number]
            self.draw_environment()
            print(f"Drone {drone_number}: Disabled (trajectory hidden)")

        # Automatically manage multi-drone mode based on enabled checkboxes
        enabled_count = sum(1 for enabled in self.drone_enabled.values() if enabled)
        if enabled_count > 1:
            # Multiple drones enabled - switch to multi-drone mode
            if not self.multi_drone_mode:
                self.multi_drone_mode = True
                print(f"🚁 Multi-drone mode: AUTO-ENABLED ({enabled_count} drones)")
        elif enabled_count == 1:
            # Only one drone enabled - switch to single-drone mode
            if self.multi_drone_mode:
                self.multi_drone_mode = False
                print(f"🚁 Multi-drone mode: AUTO-DISABLED (single drone)")
        else:
            # No drones enabled - turn off multi-drone mode
            self.multi_drone_mode = False

        plt.draw()
        
    def select_target_type(self, event, target_type):
        """Select target type for adding/editing targets"""
        self.selected_target_type = target_type
        self._update_target_type_selection()
        print(f"Selected target type {target_type.upper()}")
        
    def _update_target_type_selection(self):
        """Update target type button colors based on selection"""
        # Use the UI layout method to update colors
        ui_layout = UILayout(self.fig)
        ui_layout.buttons = self.buttons
        ui_layout.update_target_type_colors(self.selected_target_type)
        
    def _update_ui_for_selected_drone(self):
        """Update UI textboxes to show selected drone's values"""
        drone_state = self.drone_states[self.selected_drone]

        # Update fuel counter only
        self.textboxes['fuel_counter'].set_val(str(drone_state['fuel_counter']))

        # Redraw to show current drone's position
        self.draw_environment()
        
    def increment_fuel_counter(self, event):
        """Increment fuel counter for selected drone"""
        if not self.mission_path:
            return
        
        drone_state = self.drone_states[self.selected_drone]
        current = drone_state['fuel_counter']
        max_fuel = int(calculate_distance(self.mission_path))
        new_value = min(current + 1, max_fuel)
        
        drone_state['fuel_counter'] = new_value
        self.textboxes['fuel_counter'].set_val(str(new_value))
        self._move_drone_to_fuel_position(new_value)
        
    def decrement_fuel_counter(self, event):
        """Decrement fuel counter for selected drone"""
        if not self.mission_path:
            return
            
        drone_state = self.drone_states[self.selected_drone]
        current = drone_state['fuel_counter']
        new_value = max(current - 1, 0)
        
        drone_state['fuel_counter'] = new_value
        self.textboxes['fuel_counter'].set_val(str(new_value))
        self._move_drone_to_fuel_position(new_value)
        
    def _move_drone_to_fuel_position(self, fuel_value):
        """Move selected drone to position corresponding to fuel value"""
        if not self.mission_path:
            return
            
        # Calculate position along path
        total_distance = 0
        for i in range(len(self.mission_path) - 1):
            segment_dist = np.linalg.norm(
                np.array(self.mission_path[i+1]) - np.array(self.mission_path[i])
            )
            
            if total_distance + segment_dist >= fuel_value:
                # Drone is on this segment
                t = (fuel_value - total_distance) / segment_dist
                new_position = (
                    self.mission_path[i][0] + t * (self.mission_path[i+1][0] - self.mission_path[i][0]),
                    self.mission_path[i][1] + t * (self.mission_path[i+1][1] - self.mission_path[i][1])
                )
                
                # Update selected drone's position
                self.drone_states[self.selected_drone]['position'] = new_position
                break
                
            total_distance += segment_dist
        
        self.draw_environment()
        
    # Mouse event handlers
    def on_mouse_press(self, event):
        """Handle mouse press"""
        if self.edit_mode and self.drag_handler.on_mouse_press(event):
            self.draw_environment()
            
    def on_mouse_motion(self, event):
        """Handle mouse motion"""
        if self.edit_mode and self.drag_handler.on_mouse_motion(event):
            self.draw_environment()
            
    def on_mouse_release(self, event):
        """Handle mouse release"""
        if self.edit_mode and self.drag_handler.on_mouse_release(event):
            self.draw_environment()
            
    def on_key_press(self, event):
        """Handle keyboard shortcuts"""
        if event.key == 'e' and self.edit_mode:
            # Edit target priority at mouse position
            if event.inaxes:
                obj_type, index = self.object_manager.find_object_at_position(
                    event.xdata, event.ydata
                )
                if obj_type == 'target':
                    old, new = self.object_manager.edit_target_priority(index)
                    if old is not None:
                        print(f"Changed priority from {old} to {new}")
                        self.draw_environment()
                        
        elif event.key == 'r' and self.edit_mode:
            # Edit SAM range at mouse position
            if event.inaxes:
                obj_type, index = self.object_manager.find_object_at_position(
                    event.xdata, event.ydata
                )
                if obj_type == 'sam':
                    old, new = self.object_manager.edit_sam_range(index)
                    if old is not None:
                        print(f"Changed range from {old} to {new}")
                        self.draw_environment()
                        
        elif event.key in ['1', '2', '3', '4'] and self.edit_mode:
            # Assign target to drone at mouse position
            if event.inaxes:
                obj_type, index = self.object_manager.find_object_at_position(
                    event.xdata, event.ydata
                )
                if obj_type == 'target':
                    target = self.object_manager.targets[index]
                    drone_id = int(event.key)
                    self.assign_target_to_drone(target['id'], drone_id)
                    
        elif event.key == 'm':
            # Toggle multi-drone mode
            self.toggle_multi_drone_mode(None)
            
        elif event.key == 'p' and self.edit_mode:
            # Plan all drone missions
            self.plan_all_drone_missions()
                        
    # Validation methods
    def _validate_sequence_input(self, text):
        """Validate and uppercase sequence input"""
        # Convert to uppercase
        upper_text = text.upper()
        if upper_text != text:
            self.textboxes['sequence'].set_val(upper_text)
                    
    # Helper methods
    def _plan_all_enabled_drones(self):
        """Plan missions for all enabled drones based on current sequence"""
        sequence_text = self.textboxes['sequence'].text.strip()
        if not sequence_text:
            return

        # Parse multi-drone sequence format "D1: seq1  D2: seq2  D3: seq3  D4: seq4"
        import re

        for drone_id in range(1, 5):
            # Skip if drone is not enabled
            if not self.drone_enabled.get(drone_id, False):
                continue

            # Extract sequence for this drone
            drone_label = f'D{drone_id}:'
            pattern = f'{drone_label}\\s*([A-Z0-9,\\s]*?)(?:\\s+D\\d:|$)'
            match = re.search(pattern, sequence_text)

            if match:
                drone_sequence = match.group(1).strip()
                if not drone_sequence:
                    continue

                waypoint_ids = [wp.strip() for wp in drone_sequence.split(',') if wp.strip()]

                # Plan path for this drone
                path = []
                for i in range(len(waypoint_ids) - 1):
                    start_id = waypoint_ids[i]
                    end_id = waypoint_ids[i + 1]

                    start_pos = self._get_waypoint_position(start_id)
                    end_pos = self._get_waypoint_position(end_id)

                    if start_pos and end_pos:
                        segment_path, _, _ = self.sam_navigator.plan_path_with_sam_avoidance(
                            start_pos, end_pos, self.object_manager.sams
                        )

                        if i == 0:
                            path.extend(segment_path)
                        else:
                            path.extend(segment_path[1:])

                if path:
                    # Calculate points from visited targets
                    visited_targets = [wid for wid in waypoint_ids if wid.startswith('T')]
                    total_points = 0
                    for target_id in visited_targets:
                        target = next((t for t in self.object_manager.targets if t['id'] == target_id), None)
                        if target:
                            total_points += target.get('priority', 0)

                    # Calculate loiter fuel for this drone
                    loiter_time = self.drone_states[drone_id].get('loiter_time', 0)
                    loiter_fuel = len(visited_targets) * loiter_time

                    # Store mission for this drone
                    self.drone_missions[drone_id] = {
                        'path': path,
                        'sequence': drone_sequence,
                        'description': f"Mission {len(waypoint_ids)-1} waypoints",
                        'distance': calculate_distance(path),
                        'loiter_fuel': loiter_fuel,
                        'points': total_points,
                        'num_targets': len(visited_targets)
                    }

                    # Set drone position
                    self.drone_states[drone_id]['position'] = path[0]

        # Redraw to show all trajectories
        self.draw_environment()

    def _update_default_sequence(self):
        """Update sequence textbox with default sequence"""
        sequence = self.object_manager.get_default_sequence()
        if sequence:
            self.textboxes['sequence'].set_val(sequence)
            print(f"Updated default sequence: {sequence}")
            
    def _get_waypoint_position(self, waypoint_id):
        """Get position of waypoint by ID"""
        # Check airports
        for airport in self.object_manager.airports:
            if airport['id'] == waypoint_id:
                return [airport['x'], airport['y']]

        # Check targets
        for target in self.object_manager.targets:
            if target['id'] == waypoint_id:
                return [target['x'], target['y']]


        return None

    def _calculate_distance_matrix(self):
        """Calculate Euclidean distance matrix (fast - SAM-aware calc done on planner side)"""
        import math

        all_waypoints = (
            self.object_manager.airports +
            self.object_manager.targets
        )

        n = len(all_waypoints)
        matrix = [[0] * n for _ in range(n)]
        labels = []

        # Build labels
        for wp in all_waypoints:
            labels.append(wp['id'])

        # Calculate EUCLIDEAN distances (FAST - no SAM avoidance)
        for i in range(n):
            for j in range(n):
                if i != j:
                    pos_i = self._get_waypoint_position(labels[i])
                    pos_j = self._get_waypoint_position(labels[j])
                    if pos_i and pos_j:
                        # Simple Euclidean distance - instant calculation
                        dx = pos_j[0] - pos_i[0]
                        dy = pos_j[1] - pos_i[1]
                        matrix[i][j] = math.sqrt(dx*dx + dy*dy)
                else:
                    matrix[i][j] = 0

        # Include detailed waypoint info
        waypoint_details = []
        for wp in all_waypoints:
            detail = {
                'id': wp['id'],
                'x': wp['x'],
                'y': wp['y']
            }
            # Add target-specific info
            if wp['id'].startswith('T'):
                detail['priority'] = wp.get('priority', 5)
                detail['type'] = wp.get('type', 'a')
            waypoint_details.append(detail)

        # Build list of excluded targets (those inside SAM zones)
        excluded_targets = list(self.targets_in_polygons)

        return {
            'matrix': matrix,
            'labels': labels,
            'waypoints': waypoint_details,
            'excluded_targets': excluded_targets
        }

    def _calculate_all_stats(self):
        """Calculate cumulative stats for all drones and individual drone stats"""
        # Initialize cumulative totals
        cumulative_points = 0
        cumulative_fuel = 0
        cumulative_visited = 0
        cumulative_budget = 0

        # Individual drone stats
        individual_stats = {}

        for drone_id in range(1, 5):
            # Get drone mission and state
            mission = self.drone_missions.get(drone_id, {})
            state = self.drone_states.get(drone_id, {})

            # Calculate fuel used from mission distance + loiter fuel
            travel_distance = mission.get('distance', 0) if mission else 0
            loiter_fuel = mission.get('loiter_fuel', 0) if mission else 0
            drone_fuel = travel_distance + loiter_fuel

            # Get points from mission (stored by solver)
            drone_points = mission.get('points', 0) if mission else 0

            # Count targets from mission
            drone_targets = mission.get('num_targets', 0) if mission else 0

            # Get fuel budget
            drone_budget = state.get('fuel_budget', 999)

            # Calculate efficiency (Points / Fuel Used)
            drone_pfu = drone_points / drone_fuel if drone_fuel > 0 else 0

            # Store individual stats including configuration
            individual_stats[drone_id] = {
                'points': drone_points,
                'fuel_used': drone_fuel,
                'targets': drone_targets,
                'fuel_budget': drone_budget,
                'p_fu': drone_pfu,
                'start_airport': state.get('start_airport', 'A1'),
                'end_airport': state.get('end_airport', 'A1')
            }

            # Add to cumulative totals (only for drones that have missions)
            if mission:
                cumulative_points += drone_points
                cumulative_fuel += drone_fuel
                cumulative_visited += drone_targets
                cumulative_budget += drone_budget

        # Calculate cumulative stats
        cumulative_stats = {
            'points': cumulative_points,
            'fuel_used': cumulative_fuel,
            'fuel_budget': cumulative_budget,
            'visited': cumulative_visited,
            'total_targets': len(self.object_manager.targets),
            'points_per_fuel': cumulative_points / cumulative_fuel if cumulative_fuel > 0 else 0
        }

        return cumulative_stats, individual_stats
        
    def show(self):
        """Display the editor"""
        plt.show()


# Utility function
def calculate_distance(path):
    """Calculate total distance of a path"""
    if not path or len(path) < 2:
        return 0
    return sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) 
               for i in range(len(path) - 1))


if __name__ == "__main__":
    editor = ISREditor()
    editor.show()