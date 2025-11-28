"""
Renderer module for ISR Editor
Handles all drawing and visualization operations
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import PatchCollection


class Renderer:
    """Handles all visualization and drawing for the ISR Editor"""
    
    def __init__(self, ax):
        self.ax = ax
        self.grid_size = 100
        
        # Colors and styles
        self.AIRPORT_COLOR = 'blue'
        self.SAM_COLOR = 'orange'
        self.DEPARTURE_COLOR = 'orange'
        self.DRONE_COLOR = 'red'
        self.PATH_COLOR = 'blue'
        self.WRAPPED_BOUNDARY_COLOR = 'orange'
        
        # Target type colors
        self.TARGET_TYPE_COLORS = {
            'a': 'lightblue',
            'b': 'yellow', 
            'c': 'orange',
            'd': 'red',
            'e': 'darkred'
        }
        
    def draw_grid(self):
        """Draw the background grid"""
        grid_color = 'lightgray'
        grid_alpha = 0.3
        
        # Major grid lines
        for i in range(0, self.grid_size + 1, 10):
            self.ax.axhline(y=i, color=grid_color, alpha=grid_alpha, linewidth=0.5)
            self.ax.axvline(x=i, color=grid_color, alpha=grid_alpha, linewidth=0.5)
        
        # Minor grid lines
        for i in range(0, self.grid_size + 1, 5):
            if i % 10 != 0:
                self.ax.axhline(y=i, color=grid_color, alpha=grid_alpha/2, linewidth=0.25, linestyle=':')
                self.ax.axvline(x=i, color=grid_color, alpha=grid_alpha/2, linewidth=0.25, linestyle=':')
    
    def draw_airports(self, airports, fuel_usage=None):
        """Draw airports as blue squares"""
        if fuel_usage is None:
            fuel_usage = {}
            
        for airport in airports:
            x, y = airport['x'], airport['y']
            
            # Draw square
            square = Rectangle((x-1, y-1), 2, 2, 
                             facecolor=self.AIRPORT_COLOR, 
                             edgecolor='black', linewidth=1)
            self.ax.add_patch(square)
            
            # Label with fuel info if available
            fuel_info = ""
            if airport['id'] in fuel_usage:
                fuel_info = f"\nF:{fuel_usage[airport['id']]:.0f}"
            
            self.ax.text(x, y+2, f"{airport['id']}{fuel_info}", 
                        ha='center', va='bottom', fontweight='bold', 
                        color=self.AIRPORT_COLOR, fontsize=8)
    
    def draw_targets(self, targets, fuel_usage=None, unreachable=None, 
                     in_polygons=None, visited=None):
        """Draw targets with priority-based coloring and status markings"""
        if fuel_usage is None:
            fuel_usage = {}
        if unreachable is None:
            unreachable = set()
        if in_polygons is None:
            in_polygons = set()
        if visited is None:
            visited = set()
        
        for target in targets:
            x, y = target['x'], target['y']
            priority = target.get('priority', 5)
            target_id = target['id']
            target_type = target.get('type', 'a')  # Default to type 'a'
            
            # Get color based on target type
            color = self.TARGET_TYPE_COLORS.get(target_type, 'lightblue')
            
            # Check status
            is_unreachable = target_id in unreachable
            is_in_polygon = target_id in in_polygons
            is_visited = target_id in visited
            
            # Draw status markings
            if is_unreachable or is_in_polygon:
                edgecolor = 'black'
                linewidth = 3
                linestyle = '--'
                # Draw X mark
                x_color = 'red' if is_in_polygon else 'black'
                self.ax.plot(x, y, 'x', color=x_color, markersize=20, markeredgewidth=3)
            else:
                edgecolor = 'black'
                linewidth = 1
                linestyle = '-'
            
            # Draw green check for visited targets
            if is_visited:
                self.ax.plot(x, y, 'x', color='green', markersize=25, markeredgewidth=4)
            
            # Draw target circle
            circle = Circle((x, y), 0.75, facecolor=color, edgecolor=edgecolor, 
                          linewidth=linewidth, linestyle=linestyle)
            self.ax.add_patch(circle)
            
            # Label
            fuel_info = ""
            if target_id in fuel_usage:
                fuel_info = f"\nF:{fuel_usage[target_id]:.0f}"
            
            label_suffix = ""
            if is_visited:
                label_suffix = " ✅"
            elif is_in_polygon:
                label_suffix = " (IN POLY)"
            elif is_unreachable:
                label_suffix = " (X)"
            
            # Show drone assignment if present
            drone_assignment = ""
            if 'assigned_drone' in target:
                drone_assignment = f"→D{target['assigned_drone']}"
            
            self.ax.text(x, y+1.5, f"{target_id}{label_suffix}{drone_assignment}\nP:{priority}{fuel_info}", 
                        ha='center', va='bottom', fontsize=7, fontweight='bold')
    
    def draw_sams(self, sams):
        """Draw SAM sites with range circles"""
        for i, sam in enumerate(sams):
            x, y = sam['pos']
            sam_range = sam['range']
            
            # Range circle (yellow fill with orange edge, like original)
            range_circle = Circle((x, y), sam_range, facecolor='yellow', alpha=0.2, 
                                edgecolor='orange', linewidth=1)
            self.ax.add_patch(range_circle)
            
            # SAM triangle (orange with darker orange edge)
            triangle_size = 0.75  # Match original size
            triangle = plt.Polygon(
                [(x-triangle_size, y-triangle_size), 
                 (x+triangle_size, y-triangle_size), 
                 (x, y+triangle_size)],
                facecolor='orange', edgecolor='darkorange', linewidth=1
            )
            self.ax.add_patch(triangle)
            
            # Label with range info (match original format)
            self.ax.text(x, y+1.5, f'S{i+1}\nR:{sam_range}', ha='center', va='bottom', 
                        fontsize=7, fontweight='bold')
    
    
    def draw_wrapped_boundaries(self, sams):
        """Draw wrapped SAM polygon boundaries"""
        if not sams:
            return
            
        try:
            import sys
            import os
            # Add parent directory to path to find the modules
            parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            if parent_dir not in sys.path:
                sys.path.append(parent_dir)
                
            from sam_wrapping_no_buffer import wrap_sams
            wrapped_polygons, cluster_info = wrap_sams(sams)
            
            for i, (polygon, info) in enumerate(zip(wrapped_polygons, cluster_info)):
                if info['type'] == 'wrapped':
                    # Draw boundary only - orange dashed line (same as original)
                    polygon_closed = np.vstack([polygon, polygon[0]])
                    self.ax.plot(polygon_closed[:, 0], polygon_closed[:, 1], 
                               'orange', linestyle='--', 
                               linewidth=1, alpha=0.8)
        except ImportError:
            pass  # Silently fail if dependencies missing
    
    def draw_mission_path(self, path, is_traveled=False):
        """Draw the mission path"""
        if not path or len(path) < 2:
            return
            
        try:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            
            if is_traveled:
                self.ax.plot(px, py, self.PATH_COLOR, linewidth=1, 
                           alpha=0.7, linestyle='--', label='Path Traveled')
            else:
                self.ax.plot(px, py, self.PATH_COLOR, linewidth=1, 
                           alpha=0.7, linestyle='--', label='Mission Path')
        except (IndexError, TypeError) as e:
            print(f"Warning: Error drawing path: {e}")
    
    def draw_multi_drone_paths(self, drone_missions, animation_running=False, drone_states=None, drone_enabled=None):
        """Draw multiple drone mission paths with different colors and styles"""
        # Define colors and line styles for each drone - ALL DASHED
        # Colors match the drone selection buttons
        drone_styles = {
            1: {'color': '#1f77b4', 'linestyle': '--', 'linewidth': 1, 'alpha': 0.8},  # blue
            2: {'color': '#ff7f0e', 'linestyle': '--', 'linewidth': 1, 'alpha': 0.8},  # orange
            3: {'color': '#2ca02c', 'linestyle': '--', 'linewidth': 1, 'alpha': 0.8},  # green
            4: {'color': '#d62728', 'linestyle': '--', 'linewidth': 1, 'alpha': 0.8}   # red
        }

        for drone_id, mission_data in drone_missions.items():
            # Skip if drone is not enabled (checkbox not checked)
            if drone_enabled and not drone_enabled.get(drone_id, False):
                continue
            if 'path' not in mission_data or not mission_data['path']:
                continue
                
            path = mission_data['path']
            if len(path) < 2:
                continue
                
            try:
                style = drone_styles.get(drone_id, drone_styles[1])
                
                if animation_running and drone_states and drone_id in drone_states:
                    # During animation: show traveled path (solid) + future path (faded dashed)
                    drone_state = drone_states[drone_id]
                    traveled = drone_state.get('traveled_path', [])
                    
                    if len(traveled) >= 2:
                        # Draw traveled path as solid line
                        tx = [p[0] for p in traveled]
                        ty = [p[1] for p in traveled]
                        self.ax.plot(tx, ty, 
                                   color=style['color'],
                                   linestyle='-', 
                                   linewidth=style['linewidth'],
                                   alpha=1.0,
                                   label=f"Drone {drone_id} (traveled)")
                    
                    # Draw remaining path as faded dashed line
                    if len(traveled) < len(path):
                        remaining_path = path[len(traveled)-1:]  # Include last traveled point for continuity
                        if len(remaining_path) >= 2:
                            rx = [p[0] for p in remaining_path]
                            ry = [p[1] for p in remaining_path]
                            self.ax.plot(rx, ry, 
                                       color=style['color'],
                                       linestyle=':', 
                                       linewidth=2,
                                       alpha=0.3,
                                       label=f"Drone {drone_id} (planned)")
                else:
                    # Static view: show full planned path as dashed line
                    px = [p[0] for p in path]
                    py = [p[1] for p in path]
                    
                    label = f"Drone {drone_id}: {mission_data.get('description', 'Mission')}"
                    
                    self.ax.plot(px, py, 
                               color=style['color'],
                               linestyle=style['linestyle'], 
                               linewidth=style['linewidth'],
                               alpha=style['alpha'],
                               label=label)
                    
                    # Add direction arrows for static view
                    self._add_direction_arrows(path, style['color'])
                
            except (IndexError, TypeError) as e:
                print(f"Warning: Error drawing drone {drone_id} path: {e}")
    
    def _add_direction_arrows(self, path, color):
        """Add direction arrows along the path"""
        if len(path) < 2:
            return
            
        # Add arrows at strategic points along the path
        arrow_spacing = max(1, len(path) // 6)  # Show ~6 arrows per path
        
        for i in range(0, len(path) - 1, arrow_spacing):
            if i < len(path) - 1:
                start = path[i]
                end = path[i + 1]
                
                # Calculate arrow position (mid-point of segment)
                arrow_x = (start[0] + end[0]) / 2
                arrow_y = (start[1] + end[1]) / 2
                
                # Calculate direction
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                
                # Normalize and scale
                length = (dx**2 + dy**2)**0.5
                if length > 0:
                    dx = dx / length * 2  # Arrow length
                    dy = dy / length * 2
                    
                    self.ax.annotate('', xy=(arrow_x + dx/2, arrow_y + dy/2), 
                                   xytext=(arrow_x - dx/2, arrow_y - dy/2),
                                   arrowprops=dict(arrowstyle='->', color=color, 
                                                 lw=2, alpha=0.7))
    
    def draw_drone(self, position, fuel_used=None, drone_number=1):
        """Draw the drone at given position"""
        if not position:
            return
            
        dx, dy = position
        
        # Define drone colors matching path colors
        drone_colors = {
            1: {'face': 'blue', 'edge': 'darkblue'},
            2: {'face': 'red', 'edge': 'darkred'},
            3: {'face': 'green', 'edge': 'darkgreen'},
            4: {'face': 'purple', 'edge': 'indigo'}
        }
        
        colors = drone_colors.get(drone_number, drone_colors[1])
        
        # Drone diamond shape
        drone = plt.Polygon([(dx-1, dy), (dx, dy+1), 
                           (dx+1, dy), (dx, dy-1)], 
                          facecolor=colors['face'], 
                          edgecolor=colors['edge'], linewidth=2)
        self.ax.add_patch(drone)
        
        # Fuel display
        fuel_text = ""
        if fuel_used is not None:
            fuel_text = f"Fuel: {fuel_used:.0f}"
            
        self.ax.text(dx+2, dy, f'Drone{drone_number}\n{fuel_text}', 
                    fontsize=8, fontweight='bold', color=colors['face'])
    
    def draw_multi_drone_positions(self, drone_states):
        """Draw multiple drones at their current positions"""
        for drone_id, state in drone_states.items():
            if state.get('position'):
                fuel_used = None
                if state.get('traveled_path'):
                    fuel_used = sum(
                        np.linalg.norm(np.array(state['traveled_path'][i+1]) - 
                                     np.array(state['traveled_path'][i]))
                        for i in range(len(state['traveled_path']) - 1)
                    )
                self.draw_drone(state['position'], fuel_used, drone_id)
    
    def draw_legend(self, show_drone_paths=False):
        """Draw the legend explaining symbols"""
        legend_elements = [
            plt.Line2D([0], [0], marker='s', color='w', 
                      markerfacecolor=self.AIRPORT_COLOR, 
                      markersize=8, label='Airport'),
            plt.Line2D([0], [0], marker='^', color='w', 
                      markerfacecolor=self.SAM_COLOR, 
                      markersize=8, label='SAM Site'),
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='lightblue', 
                      markersize=8, label='Target Type A'),
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='yellow', 
                      markersize=8, label='Target Type B'),
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='orange', 
                      markersize=8, label='Target Type C'),
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='red', 
                      markersize=8, label='Target Type D'),
            plt.Line2D([0], [0], marker='o', color='w', 
                      markerfacecolor='darkred', 
                      markersize=8, label='Target Type E')
        ]
        
        # Add drone path legend if showing multi-drone paths
        if show_drone_paths:
            legend_elements.extend([
                plt.Line2D([0], [0], color='blue', linewidth=1, linestyle='--',
                          label='Drone 1 Path'),
                plt.Line2D([0], [0], color='red', linewidth=1, linestyle='--',
                          label='Drone 2 Path'),
                plt.Line2D([0], [0], color='green', linewidth=1, linestyle='--',
                          label='Drone 3 Path'),
                plt.Line2D([0], [0], color='purple', linewidth=1, linestyle='--',
                          label='Drone 4 Path')
            ])
        
        self.ax.legend(handles=legend_elements, loc='upper right', fontsize=8,
                      frameon=True, fancybox=True, shadow=True)
    
    def draw_stats(self, stats):
        """Draw mission statistics box"""
        stats_text = f"""Mission:
P: {stats.get('points', 0)}
FU: {stats.get('fuel_used', 0):.0f}/{stats.get('fuel_budget', 999):.0f}
T: {stats.get('visited', 0)}/{stats.get('total_targets', 0)}
P/FU: {stats.get('points_per_fuel', 0):.2f}"""
        
        self.ax.text(0.02, 0.98, stats_text, transform=self.ax.transAxes,
                    fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow',
                             edgecolor='orange', alpha=0.9))
    
    def draw_drone_stats_boxes(self, individual_stats):
        """Draw individual drone stats boxes with configuration info"""
        # Position these ABOVE the right column buttons using figure coordinates
        # Right panel starts at x=0.86, buttons are moved down by 3 button heights from 0.80
        # So we need to position these boxes above 0.80 in the right panel area

        box_positions = [
            (0.86, 0.92),      # D-1 top-left above buttons
            (0.93, 0.92),      # D-2 top-right above buttons
            (0.86, 0.82),      # D-3 bottom-left above buttons (moved down)
            (0.93, 0.82)       # D-4 bottom-right above buttons (moved down)
        ]

        for drone_id in range(1, 5):
            stats = individual_stats.get(drone_id, {
                'points': 0,
                'fuel_used': 0,
                'targets': 0,
                'p_fu': 0,
                'fuel_budget': 999,
                'start_airport': 'A1',
                'end_airport': 'A1'
            })

            # Format with configuration info
            stats_text = f"""$\\mathbf{{D-{drone_id}:}}$
P: {stats['points']}
FU: {stats['fuel_used']:.0f}/{stats.get('fuel_budget', 999):.0f}
{stats.get('start_airport', 'A1')}→{stats.get('end_airport', 'A1')}
P/FU: {stats['p_fu']:.2f}"""

            x, y = box_positions[drone_id - 1]
            # Use figure coordinates for UI panel positioning, but same styling as main stats box
            self.ax.text(x, y, stats_text, transform=self.ax.figure.transFigure,
                        fontsize=9, verticalalignment='top',
                        bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow',
                                 edgecolor='orange', alpha=0.9))
    
    def clear(self):
        """Clear the axes"""
        self.ax.clear()
    
    def set_limits(self):
        """Set axis limits and properties"""
        self.ax.set_xlim(-2, self.grid_size + 2)
        self.ax.set_ylim(-2, self.grid_size + 2)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('ISR Mission Environment')