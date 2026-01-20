#!/usr/bin/env python3
"""
Package Delivery Planner - UI Design
Basic layout with map canvas and tabbed right panel
"""
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np


class DeliveryPlannerUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Package Delivery Planner")
        self.root.geometry("1400x800")

        # Create bottom panel first (at bottom)
        self.create_bottom_panel()

        # Create main content area (PanedWindow for resizable split)
        main_paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, sashwidth=5)
        main_paned.pack(fill=tk.BOTH, expand=True)

        # Create map panel (left side)
        self.map_container = tk.Frame(main_paned, bg='white', relief=tk.SUNKEN, borderwidth=2)
        main_paned.add(self.map_container, width=900)

        # Create right panel
        self.right_container = tk.Frame(main_paned, bg='lightgray', relief=tk.SUNKEN, borderwidth=2)
        main_paned.add(self.right_container, width=500)

        # Now populate the panels
        self.create_map_panel()
        self.create_right_panel()

        # Initialize with sample data
        self.init_sample_data()
        self.draw_initial_map()

    def create_map_panel(self):
        """Create left panel with matplotlib canvas"""
        # Create matplotlib figure
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title('Delivery Map View', fontsize=14, fontweight='bold')

        # Embed matplotlib canvas in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.map_container)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def create_right_panel(self):
        """Create right panel with 3 tabs"""
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.right_container)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Tab 1: By Address
        self.create_address_tab()

        # Tab 2: By Drone Trip
        self.create_trip_tab()

        # Tab 3: Fleet Status
        self.create_fleet_tab()

    def create_address_tab(self):
        """Tab 1: Delivery schedule by address"""
        tab1 = tk.Frame(self.notebook, bg='white')
        self.notebook.add(tab1, text='📍 By Address')

        # Title
        title = tk.Label(tab1, text='DELIVERY SCHEDULE', font=('Arial', 12, 'bold'), bg='white')
        title.pack(pady=10)

        # Create treeview for address table
        columns = ('Address', 'Requires', 'Receiving', 'Status')
        self.address_tree = ttk.Treeview(tab1, columns=columns, show='headings', height=20)

        # Define column headings
        self.address_tree.heading('Address', text='Addr')
        self.address_tree.heading('Requires', text='Requires')
        self.address_tree.heading('Receiving', text='Receiving')
        self.address_tree.heading('Status', text='Status')

        # Define column widths
        self.address_tree.column('Address', width=60)
        self.address_tree.column('Requires', width=100)
        self.address_tree.column('Receiving', width=150)
        self.address_tree.column('Status', width=80)

        # Add scrollbar
        scrollbar = ttk.Scrollbar(tab1, orient=tk.VERTICAL, command=self.address_tree.yview)
        self.address_tree.configure(yscroll=scrollbar.set)

        # Pack widgets
        self.address_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Add sample data
        self.populate_address_table()

    def create_trip_tab(self):
        """Tab 2: Trip details with cargo"""
        tab2 = tk.Frame(self.notebook, bg='white')
        self.notebook.add(tab2, text='🚁 By Drone Trip')

        # Title
        title = tk.Label(tab2, text='TRIP DETAILS', font=('Arial', 12, 'bold'), bg='white')
        title.pack(pady=10)

        # Create text widget for trip details
        self.trip_text = tk.Text(tab2, font=('Courier', 10), wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(tab2, orient=tk.VERTICAL, command=self.trip_text.yview)
        self.trip_text.configure(yscroll=scrollbar.set)

        self.trip_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Add sample data
        self.populate_trip_details()

    def create_fleet_tab(self):
        """Tab 3: Fleet status and inventory"""
        tab3 = tk.Frame(self.notebook, bg='white')
        self.notebook.add(tab3, text='📦 Fleet Status')

        # Title
        title = tk.Label(tab3, text='WAREHOUSE & FLEET', font=('Arial', 12, 'bold'), bg='white')
        title.pack(pady=10)

        # Create text widget for fleet status
        self.fleet_text = tk.Text(tab3, font=('Courier', 10), wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(tab3, orient=tk.VERTICAL, command=self.fleet_text.yview)
        self.fleet_text.configure(yscroll=scrollbar.set)

        self.fleet_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Add sample data
        self.populate_fleet_status()

    def create_bottom_panel(self):
        """Create bottom panel with controls and metrics"""
        bottom_frame = tk.Frame(self.root, bg='lightblue', relief=tk.SUNKEN, borderwidth=2, height=60)
        bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        # Control buttons
        button_frame = tk.Frame(bottom_frame, bg='lightblue')
        button_frame.pack(side=tk.LEFT, padx=10, pady=5)

        tk.Button(button_frame, text='📂 Load', width=10, command=self.load_scenario).pack(side=tk.LEFT, padx=2)
        tk.Button(button_frame, text='🧠 Solve', width=10, command=self.solve_delivery).pack(side=tk.LEFT, padx=2)
        tk.Button(button_frame, text='▶️ Animate', width=10, command=self.animate).pack(side=tk.LEFT, padx=2)
        tk.Button(button_frame, text='🔄 Reset', width=10, command=self.reset).pack(side=tk.LEFT, padx=2)
        tk.Button(button_frame, text='💾 Export', width=10, command=self.export_solution).pack(side=tk.LEFT, padx=2)

        # Metrics display
        metrics_frame = tk.Frame(bottom_frame, bg='lightblue')
        metrics_frame.pack(side=tk.RIGHT, padx=10, pady=5)

        self.fuel_label = tk.Label(metrics_frame, text='⛽ Fuel: 257', font=('Arial', 10, 'bold'), bg='lightblue')
        self.fuel_label.pack(side=tk.LEFT, padx=10)

        self.trips_label = tk.Label(metrics_frame, text='🔁 Trips: 5', font=('Arial', 10, 'bold'), bg='lightblue')
        self.trips_label.pack(side=tk.LEFT, padx=10)

        self.distance_label = tk.Label(metrics_frame, text='📏 Distance: 428km', font=('Arial', 10, 'bold'), bg='lightblue')
        self.distance_label.pack(side=tk.LEFT, padx=10)

        self.complete_label = tk.Label(metrics_frame, text='✅ Complete: 95/95', font=('Arial', 10, 'bold'), bg='lightgreen')
        self.complete_label.pack(side=tk.LEFT, padx=10)

    def init_sample_data(self):
        """Initialize sample data for visualization"""
        # Warehouse location
        self.warehouse = (50, 50)

        # Sample addresses with requirements
        self.addresses = [
            {'id': 'A1', 'pos': (20, 70), 'requires': {'H': 2, 'M': 3, 'S': 5}, 'status': 'partial'},
            {'id': 'A2', 'pos': (80, 80), 'requires': {'H': 1, 'M': 8, 'S': 0}, 'status': 'done'},
            {'id': 'A3', 'pos': (70, 30), 'requires': {'H': 0, 'M': 0, 'S': 12}, 'status': 'done'},
            {'id': 'A4', 'pos': (30, 20), 'requires': {'H': 1, 'M': 2, 'S': 4}, 'status': 'not_delivered'},
            {'id': 'A5', 'pos': (60, 65), 'requires': {'H': 0, 'M': 5, 'S': 8}, 'status': 'partial'},
            {'id': 'A6', 'pos': (85, 50), 'requires': {'H': 2, 'M': 0, 'S': 10}, 'status': 'not_delivered'},
            {'id': 'A7', 'pos': (40, 85), 'requires': {'H': 0, 'M': 8, 'S': 0}, 'status': 'done'},
            {'id': 'A8', 'pos': (15, 40), 'requires': {'H': 1, 'M': 3, 'S': 6}, 'status': 'partial'},
        ]

        # Sample no-fly zones
        self.no_fly_zones = [
            {'center': (50, 30), 'radius': 8},
            {'center': (65, 75), 'radius': 6},
        ]

        # Sample trips
        self.trips = [
            {'drone': 'Heavy', 'route': [self.warehouse, (20, 70), (60, 65), (40, 85), self.warehouse], 'color': 'blue'},
            {'drone': 'Medium', 'route': [self.warehouse, (70, 30), (80, 80), self.warehouse], 'color': 'green'},
            {'drone': 'Light', 'route': [self.warehouse, (30, 20), (15, 40), (85, 50), self.warehouse], 'color': 'orange'},
        ]

    def draw_initial_map(self):
        """Draw the initial map with warehouse, addresses, and no-fly zones"""
        self.ax.clear()
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)

        # Draw warehouse
        self.ax.plot(*self.warehouse, marker='s', markersize=15, color='green',
                     label='Warehouse', zorder=5)
        self.ax.text(self.warehouse[0], self.warehouse[1]-3, 'W',
                    ha='center', va='top', fontsize=10, fontweight='bold')

        # Draw addresses with status colors
        for addr in self.addresses:
            if addr['status'] == 'done':
                color = 'green'
            elif addr['status'] == 'partial':
                color = 'yellow'
            else:
                color = 'red'

            self.ax.plot(*addr['pos'], marker='o', markersize=10, color=color,
                        markeredgecolor='black', markeredgewidth=1, zorder=4)
            self.ax.text(addr['pos'][0], addr['pos'][1]+2, addr['id'],
                        ha='center', va='bottom', fontsize=9)

        # Draw no-fly zones
        for zone in self.no_fly_zones:
            circle = plt.Circle(zone['center'], zone['radius'], color='red',
                              alpha=0.2, zorder=1)
            self.ax.add_patch(circle)
            circle_outline = plt.Circle(zone['center'], zone['radius'], color='red',
                                       fill=False, linestyle='--', linewidth=2, zorder=2)
            self.ax.add_patch(circle_outline)

        # Draw sample trajectories
        for trip in self.trips:
            route = trip['route']
            xs = [p[0] for p in route]
            ys = [p[1] for p in route]
            self.ax.plot(xs, ys, color=trip['color'], linestyle='--',
                        linewidth=2, alpha=0.6, label=f"{trip['drone']} Drone")

        self.ax.legend(loc='upper left')
        self.ax.set_title('Delivery Map View', fontsize=14, fontweight='bold')
        self.canvas.draw()

    def populate_address_table(self):
        """Populate address table with sample data"""
        sample_data = [
            ('A1', '2H, 3M, 5S', 'Trip1(H): 2H,3M\nTrip3(L): 5S', 'Partial'),
            ('A2', '1H, 8M', 'Trip1(H): 1H,8M', '✓ Done'),
            ('A3', '12S', 'Trip2(M): 12S', '✓ Done'),
            ('A4', '1H, 2M, 4S', '-', 'Pending'),
            ('A5', '5M, 8S', 'Trip1(H): 5M', 'Partial'),
            ('A6', '2H, 10S', '-', 'Pending'),
            ('A7', '8M', 'Trip1(H): 8M', '✓ Done'),
            ('A8', '1H, 3M, 6S', 'Trip3(L): 6S', 'Partial'),
        ]

        for row in sample_data:
            self.address_tree.insert('', tk.END, values=row)

    def populate_trip_details(self):
        """Populate trip details text"""
        trip_info = """
╔════════════════════════════════════════════════╗
║  Heavy Drone - Trip 1 (Fuel: 72)             ║
╠════════════════════════════════════════════════╣
║  Route: W → A1 → A5 → A7 → W                 ║
║                                                ║
║  Loaded Compartments:                         ║
║    • Comp1: 2H                                ║
║    • Comp2: 8M (3M + 5M)                      ║
║    • Comp3: 8M                                ║
║                                                ║
║  Deliveries:                                  ║
║    📍 A1: 2H, 3M                              ║
║    📍 A5: 5M                                  ║
║    📍 A7: 8M                                  ║
╚════════════════════════════════════════════════╝

╔════════════════════════════════════════════════╗
║  Medium Drone - Trip 1 (Fuel: 48)            ║
╠════════════════════════════════════════════════╣
║  Route: W → A3 → A2 → W                      ║
║                                                ║
║  Loaded Compartments:                         ║
║    • Comp1-3: 12S (4S each)                   ║
║    • Comp4: 1M                                ║
║    • Comp5-12: Empty                          ║
║                                                ║
║  Deliveries:                                  ║
║    📍 A3: 12S                                 ║
║    📍 A2: 8M (needs 2nd trip)                 ║
╚════════════════════════════════════════════════╝

╔════════════════════════════════════════════════╗
║  Light Drone - Trip 1 (Fuel: 35)             ║
╠════════════════════════════════════════════════╣
║  Route: W → A1 → A8 → A4 → W                 ║
║                                                ║
║  Loaded Compartments:                         ║
║    • Comp1-5: 5S (A1)                         ║
║    • Comp6-7: 6S (A8)                         ║
║    • Comp8-9: 4S (A4)                         ║
║    • Comp10-27: Empty                         ║
║                                                ║
║  Deliveries:                                  ║
║    📍 A1: 5S                                  ║
║    📍 A8: 6S                                  ║
║    📍 A4: 4S                                  ║
╚════════════════════════════════════════════════╝
        """
        self.trip_text.insert('1.0', trip_info)
        self.trip_text.config(state=tk.DISABLED)

    def populate_fleet_status(self):
        """Populate fleet status text"""
        fleet_info = """
╔════════════════════════════════════════════════╗
║         WAREHOUSE INVENTORY                   ║
╠════════════════════════════════════════════════╣
║  Total Packages Available:                    ║
║    📦 Heavy (H):   15 packages                ║
║    📦 Medium (M):  32 packages                ║
║    📦 Small (S):   48 packages                ║
║                                                ║
║  Packages Delivered:                          ║
║    ✅ Heavy:   6 / 15  (40%)                  ║
║    ✅ Medium: 19 / 32  (59%)                  ║
║    ✅ Small:  27 / 48  (56%)                  ║
╚════════════════════════════════════════════════╝

╔════════════════════════════════════════════════╗
║         HEAVY DRONE STATUS                    ║
╠════════════════════════════════════════════════╣
║  Compartments: 3                              ║
║  Capacity: 2H or 8M per compartment           ║
║  Fuel Rate: 3 units/step                      ║
║                                                ║
║  Trips Completed:                             ║
║    Trip 1: [2H, 8M, 8M] → 72 fuel            ║
║    Trip 2: [1H, 16M, 0]  → 85 fuel           ║
║                                                ║
║  Total Fuel Used: 157 units                   ║
╚════════════════════════════════════════════════╝

╔════════════════════════════════════════════════╗
║         MEDIUM DRONE STATUS                   ║
╠════════════════════════════════════════════════╣
║  Compartments: 9                              ║
║  Capacity: 1M or 4S per compartment           ║
║  Fuel Rate: 2 units/step                      ║
║                                                ║
║  Trips Completed:                             ║
║    Trip 1: 12S + 1M → 48 fuel                ║
║                                                ║
║  Total Fuel Used: 48 units                    ║
╚════════════════════════════════════════════════╝

╔════════════════════════════════════════════════╗
║         LIGHT DRONE STATUS                    ║
╠════════════════════════════════════════════════╣
║  Compartments: 27                             ║
║  Capacity: 3S per compartment                 ║
║  Fuel Rate: 1 unit/step                       ║
║                                                ║
║  Trips Completed:                             ║
║    Trip 1: 15S → 35 fuel                     ║
║    Trip 2: 21S → 42 fuel                     ║
║                                                ║
║  Total Fuel Used: 77 units                    ║
╚════════════════════════════════════════════════╝

╔════════════════════════════════════════════════╗
║         SUMMARY                               ║
╠════════════════════════════════════════════════╣
║  Total Fuel Consumed: 282 units               ║
║  Total Trips: 5                               ║
║  Total Distance: 428 km                       ║
║  Delivery Progress: 52/95 packages (55%)      ║
╚════════════════════════════════════════════════╝
        """
        self.fleet_text.insert('1.0', fleet_info)
        self.fleet_text.config(state=tk.DISABLED)

    # Button callbacks (placeholders)
    def load_scenario(self):
        print("Load scenario clicked")

    def solve_delivery(self):
        print("Solve delivery clicked")

    def animate(self):
        print("Animate clicked")

    def reset(self):
        print("Reset clicked")
        self.draw_initial_map()

    def export_solution(self):
        print("Export solution clicked")


def main():
    root = tk.Tk()
    app = DeliveryPlannerUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
