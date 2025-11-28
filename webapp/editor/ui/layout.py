"""
UI Layout module for ISR Editor
Handles button positioning, styling, and layout management
"""
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox


class UILayout:
    """Manages the UI layout and button creation for the ISR Editor"""
    
    def __init__(self, figure):
        self.fig = figure
        self.button_axes = {}
        self.buttons = {}
        self.textboxes = {}
        
        # Layout constants
        self.BTN_HEIGHT = 0.032
        self.BTN_WIDTH = 0.062
        self.BTN_SPACING = 0.068
        self.Y_BOTTOM = 0.05
        self.START_X = 0.08 + 0.168  # 0.08 (left margin) + 0.168 (20% of plot width)
        
        # Right panel constants
        self.PANEL_X = 0.86
        self.PANEL_BTN_WIDTH = 0.08
        self.PANEL_BTN_HEIGHT = 0.035
        self.PANEL_SPACING_Y = 0.04
        
        # Font size
        self.BUTTON_FONTSIZE = 8
        
    def create_all_buttons(self):
        """Create all UI buttons and controls"""
        self._create_bottom_row_buttons()
        self._create_top_row_buttons()
        self._create_right_panel()
        self._set_button_fonts()
        return self.buttons, self.textboxes, self.button_axes
    
    def _create_bottom_row_buttons(self):
        """Create bottom row of buttons"""
        y = self.Y_BOTTOM
        width = self.BTN_WIDTH
        height = self.BTN_HEIGHT
        spacing = self.BTN_SPACING
        
        # Create buttons with adjusted positions for target types
        # First 4 buttons: Edit, Add Target, Add Airport, Add SAM
        first_buttons = [
            ('edit', 'Edit Mode'),
            ('add_target', 'Add Target'),
            ('add_airport', 'Add Airport'),
            ('add_sam', 'Add SAM')
        ]
        
        for i, (btn_id, label) in enumerate(first_buttons):
            ax = plt.axes([self.START_X + i*spacing, y, width, height])
            self.button_axes[btn_id] = ax
            self.buttons[btn_id] = Button(ax, label)
        
        # Target type buttons at position 4 (where Add D was)
        target_type_x = self.START_X + 4 * spacing
        target_type_width = spacing * 0.9  # Use most of the available space
        target_box_width = target_type_width / 5
        target_types = ['A', 'B', 'C', 'D', 'E']
        
        for i, target_type in enumerate(target_types):
            type_x = target_type_x + i * target_box_width
            ax_type = plt.axes([type_x, y, target_box_width, height])
            self.button_axes[f'target_type_{target_type.lower()}'] = ax_type
            self.buttons[f'target_type_{target_type.lower()}'] = Button(ax_type, target_type)
        
        # Delete buttons shifted right to make room for target types (positions 5, 6, 7)
        delete_buttons = [
            ('del_target', 'Del Target'),
            ('del_airport', 'Del Airport'),  
            ('del_sam', 'Del SAM')
        ]
        
        for i, (btn_id, label) in enumerate(delete_buttons):
            # Start at position 5 (after target type buttons)
            ax = plt.axes([self.START_X + (5 + i)*spacing, y, width, height])
            self.button_axes[btn_id] = ax
            self.buttons[btn_id] = Button(ax, label)
    
    def _create_top_row_buttons(self):
        """Create top row with sequence display (read-only)"""
        y = 0.10 - (0.25 * self.BTN_HEIGHT)  # Moved down by 0.25 button height
        height = self.BTN_HEIGHT

        # Sequence display - extend to right edge of plot area (before right panel)
        seq_textbox_width = 0.70  # Extended from 0.35 to reach close to right panel

        # Sequence textbox (read-only display)
        ax_seq_textbox = plt.axes([self.START_X, y, seq_textbox_width, height])
        self.textboxes['sequence'] = TextBox(ax_seq_textbox, '', initial='', textalignment='left')
    
    def _create_right_panel(self):
        """Create right side panel for Import/Export/Fuel controls"""
        # Starting Y position (aligned with plot area)
        start_y = 0.80

        # Reserve space at top for drone stats boxes (drawn directly on plot)
        # Move all buttons down by 3 button heights to make room for drone stats
        buttons_offset = 3 * self.PANEL_BTN_HEIGHT
        adjusted_start_y = start_y - buttons_offset

        # Main action buttons - keep Import and Export at original positions
        main_buttons = [
            ('import', 'Import', 0),
            ('export', 'Export', 1),
            ('multi_drone_planner', 'Update Planner', 3)  # Moved to position after Export
        ]

        for btn_id, label, offset in main_buttons:
            y_pos = adjusted_start_y - offset * self.PANEL_SPACING_Y
            ax = plt.axes([self.PANEL_X, y_pos, self.PANEL_BTN_WIDTH, self.PANEL_BTN_HEIGHT])
            self.button_axes[btn_id] = ax
            self.buttons[btn_id] = Button(ax, label)

        # Drone selection controls - moved down by 1 button height
        self._create_fuel_controls(adjusted_start_y - 4.5 * self.PANEL_SPACING_Y)  # Changed from 3.5 to 4.5

        # Fuel step counter - moved down by 1 button height
        counter_y = adjusted_start_y - 6.5 * self.PANEL_SPACING_Y  # Changed from 5.5 to 6.5
        self._create_fuel_counter(counter_y)

        # Segment button - moved down by 1 button height
        segment_y = counter_y - self.PANEL_BTN_HEIGHT - 0.8 * self.PANEL_BTN_HEIGHT
        ax = plt.axes([self.PANEL_X, segment_y, self.PANEL_BTN_WIDTH, self.PANEL_BTN_HEIGHT])
        self.button_axes['segment'] = ax
        self.buttons['segment'] = Button(ax, 'Segment')

        # Animate button - moved to position after Segment
        animate_y = segment_y - self.PANEL_BTN_HEIGHT - 0.02
        ax = plt.axes([self.PANEL_X, animate_y, self.PANEL_BTN_WIDTH, self.PANEL_BTN_HEIGHT])
        self.button_axes['animate'] = ax
        self.buttons['animate'] = Button(ax, 'Animate')
    
    def _create_fuel_controls(self, fuel_y):
        """Create drone selection controls"""
        label_height = 0.025
        checkbox_height = 0.020

        # Drone colors (matching trajectory colors)
        drone_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']  # blue, orange, green, red

        # Drone selection boxes (4 boxes in a row) - always show with trajectory color
        drone_box_width = self.PANEL_BTN_WIDTH / 4
        for i in range(4):
            x_pos = self.PANEL_X + i * drone_box_width
            ax_drone = plt.axes([x_pos, fuel_y, drone_box_width, label_height])
            btn = Button(ax_drone, f'{i+1}')
            btn.color = drone_colors[i]
            btn.hovercolor = drone_colors[i]
            self.buttons[f'drone_{i+1}'] = btn

        # Checkboxes below drone boxes (to control which drones are in solution)
        checkbox_y = fuel_y - checkbox_height - 0.002
        checkbox_width = drone_box_width * 0.6  # Smaller than drone box
        for i in range(4):
            x_pos = self.PANEL_X + i * drone_box_width + (drone_box_width - checkbox_width) / 2  # Center checkbox
            ax_checkbox = plt.axes([x_pos, checkbox_y, checkbox_width, checkbox_height])
            # Create a button that acts like a checkbox
            self.button_axes[f'drone_checkbox_{i+1}'] = ax_checkbox
            self.buttons[f'drone_checkbox_{i+1}'] = Button(ax_checkbox, '✓')
    
    
    def _create_fuel_counter(self, counter_y):
        """Create fuel step counter controls"""
        label_height = 0.025
        
        # Fuel-Step label
        ax_label = plt.axes([self.PANEL_X, counter_y, self.PANEL_BTN_WIDTH, label_height])
        self.buttons['fuel_step_label'] = Button(ax_label, 'Fuel-Step:')
        
        # Counter controls - 3 parts: down arrow, display, up arrow
        counter_controls_y = counter_y - self.PANEL_BTN_HEIGHT
        counter_btn_width = self.PANEL_BTN_WIDTH * 0.25
        counter_display_width = self.PANEL_BTN_WIDTH * 0.5
        
        # Down button
        down_ax = plt.axes([self.PANEL_X, counter_controls_y, counter_btn_width, self.PANEL_BTN_HEIGHT])
        self.button_axes['counter_down'] = down_ax
        self.buttons['counter_down'] = Button(down_ax, '\u25bc')  # ▼
        
        # Display
        display_ax = plt.axes([self.PANEL_X + counter_btn_width, counter_controls_y, 
                               counter_display_width, self.PANEL_BTN_HEIGHT])
        self.textboxes['fuel_counter'] = TextBox(display_ax, '', initial='0', textalignment='center')
        
        # Up button
        up_ax = plt.axes([self.PANEL_X + counter_btn_width + counter_display_width, 
                          counter_controls_y, counter_btn_width, self.PANEL_BTN_HEIGHT])
        self.button_axes['counter_up'] = up_ax
        self.buttons['counter_up'] = Button(up_ax, '\u25b2')  # ▲
    
    def _set_button_fonts(self):
        """Set font sizes for all buttons"""
        for button in self.buttons.values():
            if hasattr(button, 'label'):
                button.label.set_fontsize(self.BUTTON_FONTSIZE)
    
    def update_button_states(self, edit_mode):
        """Update button visibility and colors based on edit mode"""
        # Always show essential buttons
        essential_buttons = ['export', 'import', 'animate', 'segment', 'edit']
        for btn_id in essential_buttons:
            if btn_id in self.button_axes:
                self.button_axes[btn_id].set_visible(True)

        if edit_mode:
            # Show edit mode buttons
            edit_buttons = ['add_target', 'add_airport', 'add_sam',
                           'del_target', 'del_airport', 'del_sam']
            for btn_id in edit_buttons:
                if btn_id in self.button_axes:
                    self.button_axes[btn_id].set_visible(True)
            
            # Show target type buttons
            for target_type in ['a', 'b', 'c', 'd', 'e']:
                btn_id = f'target_type_{target_type}'
                if btn_id in self.button_axes:
                    self.button_axes[btn_id].set_visible(True)
            
            # Style add buttons (green)
            if 'add_target' in self.buttons:
                self.buttons['add_target'].color = '#4CAF50'
                self.buttons['add_target'].hovercolor = '#66BB6A'
            if 'add_airport' in self.buttons:
                self.buttons['add_airport'].color = '#4CAF50'
                self.buttons['add_airport'].hovercolor = '#66BB6A'
            if 'add_sam' in self.buttons:
                self.buttons['add_sam'].color = '#4CAF50'
                self.buttons['add_sam'].hovercolor = '#66BB6A'
            
            # Style remove buttons (red)
            for btn_id in ['del_target', 'del_airport', 'del_sam']:
                if btn_id in self.buttons:
                    self.buttons[btn_id].color = '#F44336'
                    self.buttons[btn_id].hovercolor = '#EF5350'
            
            # Style edit button (blue to show active)
            if 'edit' in self.buttons:
                self.buttons['edit'].color = '#2196F3'
                self.buttons['edit'].hovercolor = '#42A5F5'
        else:
            # Hide edit mode buttons
            edit_buttons = ['add_target', 'add_airport', 'add_sam', 
                           'del_target', 'del_airport', 'del_sam']
            for btn_id in edit_buttons:
                if btn_id in self.button_axes:
                    self.button_axes[btn_id].set_visible(False)
            
            # Hide target type buttons
            for target_type in ['a', 'b', 'c', 'd', 'e']:
                btn_id = f'target_type_{target_type}'
                if btn_id in self.button_axes:
                    self.button_axes[btn_id].set_visible(False)
            
            # Reset edit button to normal
            if 'edit' in self.buttons:
                self.buttons['edit'].color = '0.85'
                self.buttons['edit'].hovercolor = '0.95'
        
        plt.draw()
        
    def update_target_type_colors(self, selected_type):
        """Update target type button colors based on selection"""
        # Color mapping for target types
        color_map = {
            'a': '#ADD8E6',  # lightblue
            'b': '#FFFF00',  # yellow  
            'c': '#FFA500',  # orange
            'd': '#FF0000',  # red
            'e': '#8B0000'   # darkred
        }
        
        for target_type in ['a', 'b', 'c', 'd', 'e']:
            btn_id = f'target_type_{target_type}'
            if btn_id in self.buttons:
                if target_type == selected_type:
                    # Selected type - use corresponding color
                    self.buttons[btn_id].color = color_map[target_type]
                    self.buttons[btn_id].hovercolor = color_map[target_type]
                else:
                    # Unselected types - grey
                    self.buttons[btn_id].color = '0.85'  # Light grey
                    self.buttons[btn_id].hovercolor = '0.95'  # Very light grey
        
        plt.draw()