"""
Generate ISR Agent Tools Documentation PDF.
Light theme version for better readability.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.backends.backend_pdf import PdfPages


def create_tools_documentation_pdf(output_path):
    """Generate a multi-page PDF documenting all ISR Agent tools."""

    with PdfPages(output_path) as pdf:
        # === PAGE 1: Title and Overview ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        # Title
        ax.text(5.5, 7.5, 'ISR Agent Tools Reference', fontsize=24, fontweight='bold',
                ha='center', va='center', color='#1a1a1a')
        ax.text(5.5, 6.8, 'Complete Tool Documentation', fontsize=18,
                ha='center', va='center', color='#666666')

        # Summary box
        summary = """
TOOLS OVERVIEW

The ISR Agent has 12 tools organized into two categories:

PLANNING TOOLS (9 tools)
  Used during mission planning to gather information, calculate routes,
  and validate solutions. These are called automatically by the agent.

OPTIMIZATION TOOLS (3 tools)
  Used to improve existing routes. Only called when user explicitly
  requests optimization (e.g., "optimize", "swap closer", etc.)


TOOL EXECUTION PATTERN

  1. Agent decides which tool(s) to call based on user query
  2. Tool receives parameters (drone_id, waypoints, routes dict, etc.)
  3. Tool accesses shared context (_current_env, _drone_configs, _distance_matrix)
  4. Tool returns formatted string with results
  5. Agent parses string and decides next action


SHARED CONTEXT (Global Variables)

  _current_env       Environment with airports, targets, SAMs
  _drone_configs     Per-drone configuration (fuel, airports, type access)
  _distance_matrix   Pre-computed SAM-aware distances between all waypoints
  _sam_paths         Pre-computed SAM-avoiding paths for visualization
        """
        ax.text(0.5, 6.2, summary, fontsize=9, family='monospace',
                va='top', ha='left', linespacing=1.3, color='#333333')

        ax.text(5.5, 0.5, 'Page 1 of 5', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 2: Planning Tools (Part 1) ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8, 'Planning Tools (1/2)', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        tools_p1 = [
            {
                'name': 'get_mission_overview',
                'desc': 'Get complete mission overview. CALL THIS FIRST.',
                'inputs': 'None',
                'outputs': '''- All airports with positions
- All targets grouped by type with priorities
- All enabled drones with constraints
- Current sequences if any
- Total possible points''',
                'color': '#2563eb'
            },
            {
                'name': 'get_drone_info',
                'desc': 'Get detailed constraints for a specific drone.',
                'inputs': 'drone_id: str ("1", "D1", etc.)',
                'outputs': '''- Fuel budget
- Start/end airports
- Accessible target types
- List of accessible targets with priorities
- Current assigned sequence''',
                'color': '#2563eb'
            },
            {
                'name': 'get_distance',
                'desc': 'Look up distance between two waypoints.',
                'inputs': '''from_point: str (e.g., "A1", "T3")
to_point: str''',
                'outputs': 'Distance in fuel units (SAM-aware)',
                'color': '#2563eb'
            },
            {
                'name': 'calculate_route_fuel',
                'desc': 'Calculate total fuel consumption for a route.',
                'inputs': '''drone_id: str
waypoints: List[str] (e.g., ["A1", "T3", "T7", "A1"])''',
                'outputs': '''- Each leg with distance
- Total fuel used
- Fuel budget
- Remaining fuel
- VALID/INVALID status''',
                'color': '#2563eb'
            },
        ]

        y = 7.5
        for tool in tools_p1:
            # Tool name box - light background
            box = FancyBboxPatch((0.3, y - 1.4), 10.4, 1.5, boxstyle="round,pad=0.05",
                                  facecolor='#f8fafc', edgecolor=tool['color'], linewidth=2)
            ax.add_patch(box)

            ax.text(0.5, y - 0.15, tool['name'], fontsize=11, fontweight='bold',
                    color=tool['color'], family='monospace')
            ax.text(3.5, y - 0.15, tool['desc'], fontsize=8, color='#4b5563')

            ax.text(0.5, y - 0.45, 'INPUTS:', fontsize=7, fontweight='bold', color='#b45309')
            ax.text(1.3, y - 0.45, tool['inputs'], fontsize=7, color='#1f2937', family='monospace')

            ax.text(5.5, y - 0.45, 'OUTPUTS:', fontsize=7, fontweight='bold', color='#047857')
            ax.text(6.5, y - 0.45, tool['outputs'].split('\n')[0], fontsize=7, color='#1f2937')

            # Additional output lines
            for i, line in enumerate(tool['outputs'].split('\n')[1:], 1):
                ax.text(6.5, y - 0.45 - (i * 0.18), line, fontsize=7, color='#1f2937')

            y -= 1.7

        ax.text(5.5, 0.5, 'Page 2 of 5', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 3: Planning Tools (Part 2) ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Planning Tools (2/2)', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        tools_p2 = [
            {
                'name': 'validate_drone_route',
                'desc': 'Validate a route against ALL drone constraints.',
                'inputs': 'drone_id: str, waypoints: List[str]',
                'outputs': 'Fuel/airport/type/duplicate checks, PASSED/FAILED status',
                'color': '#2563eb'
            },
            {
                'name': 'find_accessible_targets',
                'desc': 'Find targets drone can reach, sorted by efficiency.',
                'inputs': 'drone_id: str, from_point: str',
                'outputs': 'Table: Target | Pts | Type | Dist | Round | Eff',
                'color': '#2563eb'
            },
            {
                'name': 'suggest_drone_route',
                'desc': 'Generate a greedy baseline route for a drone.',
                'inputs': 'drone_id: str',
                'outputs': 'Route sequence, targets, points, fuel, ROUTE_Dx format',
                'color': '#2563eb'
            },
            {
                'name': 'check_target_conflicts',
                'desc': 'Check if targets are assigned to multiple drones.',
                'inputs': 'routes: Dict[str, str]  e.g., {"1": "A1,T3,A1"}',
                'outputs': 'Targets per drone, conflict list, OK/WARNING status',
                'color': '#2563eb'
            },
            {
                'name': 'get_mission_summary',
                'desc': 'Get summary stats for a multi-drone plan.',
                'inputs': 'routes: Dict[str, str]  e.g., {"1": "A1,T3,A1"}',
                'outputs': 'Per drone stats, totals, coverage %, unvisited list',
                'color': '#2563eb'
            },
        ]

        y = 7.6
        for tool in tools_p2:
            # Smaller boxes for 5 tools - light background
            box = FancyBboxPatch((0.3, y - 1.15), 10.4, 1.2, boxstyle="round,pad=0.05",
                                  facecolor='#f8fafc', edgecolor=tool['color'], linewidth=2)
            ax.add_patch(box)

            ax.text(0.5, y - 0.15, tool['name'], fontsize=10, fontweight='bold',
                    color=tool['color'], family='monospace')
            ax.text(4.2, y - 0.15, tool['desc'], fontsize=8, color='#4b5563')

            ax.text(0.5, y - 0.45, 'INPUTS:', fontsize=7, fontweight='bold', color='#b45309')
            ax.text(1.4, y - 0.45, tool['inputs'], fontsize=7, color='#1f2937', family='monospace')

            ax.text(0.5, y - 0.75, 'OUTPUTS:', fontsize=7, fontweight='bold', color='#047857')
            ax.text(1.6, y - 0.75, tool['outputs'], fontsize=7, color='#1f2937')

            y -= 1.4

        ax.text(5.5, 0.5, 'Page 3 of 5', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 4: Optimization Tools ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8, 'Optimization Tools', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')
        ax.text(5.5, 7.6, '(Only called when user explicitly requests optimization)',
                fontsize=10, ha='center', color='#6b7280')

        opt_tools = [
            {
                'name': 'optimize_assign_unvisited',
                'ui_name': 'UI Button: "Insert Missed"',
                'desc': 'Insert unvisited targets into routes with spare fuel capacity.',
                'inputs': '''routes: Dict[str, str]
  e.g., {"1": "A1,T3,A1", "2": "A2,T5,A2"}''',
                'outputs': '''- Updated routes with new targets inserted
- Points and fuel per drone
- ROUTE_Dx format for copy''',
                'algorithm': '''1. Find all unvisited targets
2. For each unvisited target:
   - Find drone with type access and spare fuel
   - Find optimal insertion position (min added distance)
   - Insert if within fuel budget
3. Return updated routes''',
                'color': '#059669'
            },
            {
                'name': 'optimize_reassign_targets',
                'ui_name': 'UI Button: "Swap Closer"',
                'desc': 'Move targets to drones whose trajectories pass closer.',
                'inputs': '''routes: Dict[str, str]
  e.g., {"1": "A1,T3,A1", "2": "A2,T5,A2"}''',
                'outputs': '''- Number of swaps made
- Swap details (target, from, to, savings)
- Updated routes
- ROUTE_Dx format for copy''',
                'algorithm': '''1. For each target in each route:
   - Calculate removal cost (fuel saved)
   - For each other drone with type access:
     - Calculate insertion cost at each segment
     - Check fuel budget constraint
   - If net savings > 0, perform swap
2. Repeat until no beneficial swaps''',
                'color': '#059669'
            },
            {
                'name': 'optimize_remove_crossings',
                'ui_name': 'UI Button: "Cross Remove"',
                'desc': 'Fix self-crossing trajectories using 2-opt algorithm.',
                'inputs': '''routes: Dict[str, str]
  e.g., {"1": "A1,T3,A1", "2": "A2,T5,A2"}''',
                'outputs': '''- Number of crossings fixed
- Fix details (drone, segment positions)
- Updated routes
- ROUTE_Dx format for copy''',
                'algorithm': '''1. For each drone route:
   - Check all pairs of non-adjacent segments
   - If segments cross (line intersection):
     - Reverse the middle portion
     - Check if distance improved
   - Repeat until no crossings remain
2. Return optimized routes''',
                'color': '#059669'
            },
        ]

        y = 7.0
        for tool in opt_tools:
            box = FancyBboxPatch((0.3, y - 2.0), 10.4, 2.1, boxstyle="round,pad=0.05",
                                  facecolor='#ecfdf5', edgecolor=tool['color'], linewidth=2)
            ax.add_patch(box)

            ax.text(0.5, y - 0.1, tool['name'], fontsize=11, fontweight='bold',
                    color=tool['color'], family='monospace')
            ax.text(5.0, y - 0.1, tool['ui_name'], fontsize=8, color='#b45309',
                    style='italic')
            ax.text(0.5, y - 0.35, tool['desc'], fontsize=8, color='#4b5563')

            # Left column: Inputs/Outputs
            ax.text(0.5, y - 0.6, 'INPUTS:', fontsize=7, fontweight='bold', color='#b45309')
            for i, line in enumerate(tool['inputs'].split('\n')):
                ax.text(1.3, y - 0.6 - (i * 0.15), line, fontsize=6.5, color='#1f2937', family='monospace')

            ax.text(0.5, y - 1.0, 'OUTPUTS:', fontsize=7, fontweight='bold', color='#047857')
            for i, line in enumerate(tool['outputs'].split('\n')):
                ax.text(1.3, y - 1.0 - (i * 0.15), line, fontsize=6.5, color='#1f2937')

            # Right column: Algorithm
            ax.text(5.3, y - 0.6, 'ALGORITHM:', fontsize=7, fontweight='bold', color='#7c3aed')
            for i, line in enumerate(tool['algorithm'].split('\n')[:7]):
                ax.text(5.3, y - 0.75 - (i * 0.14), line, fontsize=6, color='#5b21b6', family='monospace')

            y -= 2.3

        ax.text(5.5, 0.5, 'Page 4 of 5', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 5: Tool Usage Rules and Examples ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Tool Usage Rules & Examples', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Rules section (top left) - height 2.8, from y=5.0 to y=7.8
        rules_box = FancyBboxPatch((0.3, 5.0), 5, 2.8, boxstyle="round,pad=0.1",
                                    facecolor='#f5f3ff', edgecolor='#7c3aed', linewidth=2)
        ax.add_patch(rules_box)
        ax.text(2.8, 7.6, 'TOOL USAGE RULES', fontsize=11, fontweight='bold',
                ha='center', color='#5b21b6')

        rules = """1. ALWAYS call get_mission_overview first
2. Use planning tools freely during planning
3. Optimization tools ONLY when user asks:
   - "optimize" -> all three
   - "insert missed" -> optimize_assign_unvisited
   - "swap closer" -> optimize_reassign_targets
   - "remove crossings" -> optimize_remove_crossings
4. After ANY optimization, OUTPUT routes and STOP
5. optimize_reassign_targets: max 8 calls
6. Always validate routes before presenting"""
        ax.text(0.5, 7.3, rules, fontsize=7.5, va='top', color='#4c1d95',
                family='monospace', linespacing=1.2)

        # Workflow example (top right) - height 2.8, from y=5.0 to y=7.8
        workflow_box = FancyBboxPatch((5.5, 5.0), 5.2, 2.8, boxstyle="round,pad=0.1",
                                       facecolor='#ecfdf5', edgecolor='#059669', linewidth=2)
        ax.add_patch(workflow_box)
        ax.text(8.1, 7.6, 'TYPICAL WORKFLOW', fontsize=11, fontweight='bold',
                ha='center', color='#047857')

        workflow = """User: "Plan a mission for all drones"

Agent calls:
  1. get_mission_overview()
  2. suggest_drone_route("1")
  3. suggest_drone_route("2")
  4. check_target_conflicts(routes)
  5. validate_drone_route("1", route1)
  6. validate_drone_route("2", route2)
  7. get_mission_summary(routes)

Agent outputs:
  ROUTE_D1: A1,T3,T7,A1
  ROUTE_D2: A2,T5,T8,A2"""
        ax.text(5.7, 7.3, workflow, fontsize=7, va='top', color='#065f46',
                family='monospace', linespacing=1.1)

        # Input/Output format examples (bottom) - height 4.0, from y=0.7 to y=4.7
        format_box = FancyBboxPatch((0.3, 0.7), 10.4, 4.0, boxstyle="round,pad=0.1",
                                     facecolor='#f8fafc', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(format_box)
        ax.text(5.5, 4.5, 'INPUT/OUTPUT FORMAT EXAMPLES', fontsize=11, fontweight='bold',
                ha='center', color='#1d4ed8')

        # Left: Input examples
        ax.text(0.5, 4.1, 'Route Dictionary Input:', fontsize=9, fontweight='bold', color='#b45309')
        input_ex = '''routes = {
    "1": "A1,T3,T7,A1",
    "2": "A2,T5,T8,A2",
    "3": "A1,T1,T2,A1"
}'''
        ax.text(0.6, 3.85, input_ex, fontsize=8, color='#1f2937', family='monospace',
                va='top', linespacing=1.2)

        ax.text(0.5, 2.4, 'Waypoint List Input:', fontsize=9, fontweight='bold', color='#b45309')
        wp_ex = 'waypoints = ["A1", "T3", "T7", "A1"]'
        ax.text(0.6, 2.15, wp_ex, fontsize=8, color='#1f2937', family='monospace')

        # Right: Output examples
        ax.text(5.7, 4.1, 'Tool String Output:', fontsize=9, fontweight='bold', color='#047857')
        output_ex = '''=== VALIDATION PASSED for D1 ===

Route: A1 -> T3 -> T7 -> A1
Targets visited: 2
Total points: 15
Total fuel: 87.5 / 200

ROUTE_D1: A1,T3,T7,A1'''
        ax.text(5.8, 3.85, output_ex, fontsize=7.5, color='#065f46', family='monospace',
                va='top', linespacing=1.1)

        ax.text(5.5, 0.3, 'Page 5 of 5', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

    print(f"PDF generated: {output_path}")


if __name__ == "__main__":
    output = "/Users/kamalali/isr_projects/isr_web/docs/ISR_Agent_Tools.pdf"
    create_tools_documentation_pdf(output)
