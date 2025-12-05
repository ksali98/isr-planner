"""
Generate ISR Agent Architecture PDF documentation.
Light theme version for better readability.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle
from matplotlib.backends.backend_pdf import PdfPages


def create_agent_architecture_pdf(output_path):
    """Generate a multi-page PDF documenting the ISR Agent architecture."""

    with PdfPages(output_path) as pdf:
        # === PAGE 1: Title and Overview ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        # Title
        ax.text(5.5, 7.5, 'ISR Mission Planning Agent', fontsize=24, fontweight='bold',
                ha='center', va='center', color='#1a1a1a')
        ax.text(5.5, 6.8, 'Architecture Documentation', fontsize=18,
                ha='center', va='center', color='#666666')

        # Summary box
        summary = """
SYSTEM OVERVIEW

The ISR (Intelligence, Surveillance, Reconnaissance) Mission Planning system uses
a SINGLE LangGraph-based agent with 12 specialized tools to plan multi-drone
reconnaissance missions.

AGENT COUNT: 1 Main Agent

The system follows a ReAct (Reasoning + Acting) pattern:
  1. Agent receives user query + environment context
  2. Agent reasons about what tool to call
  3. Tool executes and returns result
  4. Agent reasons again (may call more tools)
  5. Agent produces final response with ROUTE_Dx outputs

KEY COMPONENTS:
  - 1 LangGraph StateGraph workflow
  - 1 LLM node (Claude claude-sonnet-4-20250514)
  - 1 ToolNode (with 12 tools)
  - Conditional edge for tool/end routing
        """
        ax.text(0.5, 6.2, summary, fontsize=10, family='monospace',
                va='top', ha='left', linespacing=1.4, color='#333333')

        ax.text(5.5, 0.5, 'Page 1 of 4', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 2: Architecture Diagram ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8, 'LangGraph Workflow Architecture', fontsize=16, fontweight='bold',
                ha='center', va='center', color='#1a1a1a')

        # Draw the workflow diagram
        # Entry point
        entry = Circle((2, 6), 0.3, fill=True, facecolor='#22c55e', edgecolor='#166534', linewidth=2)
        ax.add_patch(entry)
        ax.text(2, 6, 'START', fontsize=8, ha='center', va='center', color='white', fontweight='bold')

        # Agent Node
        agent_box = FancyBboxPatch((3.5, 5.2), 2.5, 1.6, boxstyle="round,pad=0.1",
                                    facecolor='#dbeafe', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(agent_box)
        ax.text(4.75, 6.3, 'AGENT NODE', fontsize=10, fontweight='bold', ha='center', color='#1d4ed8')
        ax.text(4.75, 5.9, 'Claude claude-sonnet-4-20250514', fontsize=7, ha='center', color='#3b82f6')
        ax.text(4.75, 5.5, '(with 12 tools bound)', fontsize=7, ha='center', color='#3b82f6')

        # Conditional edge (diamond)
        diamond_x, diamond_y = 7.5, 6
        diamond = plt.Polygon([(diamond_x, diamond_y+0.5), (diamond_x+0.5, diamond_y),
                               (diamond_x, diamond_y-0.5), (diamond_x-0.5, diamond_y)],
                              facecolor='#fef3c7', edgecolor='#d97706', linewidth=2)
        ax.add_patch(diamond)
        ax.text(diamond_x, diamond_y, 'has\ntools?', fontsize=7, ha='center', va='center', color='#92400e')

        # Tools Node
        tools_box = FancyBboxPatch((6.5, 2.5), 2, 1.5, boxstyle="round,pad=0.1",
                                    facecolor='#dcfce7', edgecolor='#16a34a', linewidth=2)
        ax.add_patch(tools_box)
        ax.text(7.5, 3.5, 'TOOLS NODE', fontsize=10, fontweight='bold', ha='center', color='#166534')
        ax.text(7.5, 3.0, '(ToolNode)', fontsize=8, ha='center', color='#22c55e')

        # End node
        end = Circle((9.5, 6), 0.3, fill=True, facecolor='#ef4444', edgecolor='#b91c1c', linewidth=2)
        ax.add_patch(end)
        ax.text(9.5, 6, 'END', fontsize=8, ha='center', va='center', color='white', fontweight='bold')

        # Arrows
        ax.annotate('', xy=(3.5, 6), xytext=(2.3, 6),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2))
        ax.annotate('', xy=(7, 6), xytext=(6, 6),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2))
        ax.annotate('', xy=(9.2, 6), xytext=(8, 6),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2))
        ax.text(8.6, 6.2, 'No', fontsize=8, color='#6b7280')

        ax.annotate('', xy=(7.5, 4), xytext=(7.5, 5.5),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2))
        ax.text(7.7, 4.8, 'Yes', fontsize=8, color='#6b7280')

        # Loop back arrow (curved)
        ax.annotate('', xy=(4.75, 5.2), xytext=(6.5, 3.25),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2,
                                   connectionstyle='arc3,rad=-0.3'))

        # Legend box
        legend_box = FancyBboxPatch((0.3, 0.8), 10.4, 1.5, boxstyle="round,pad=0.1",
                                     facecolor='#f8fafc', edgecolor='#94a3b8', linewidth=1)
        ax.add_patch(legend_box)
        ax.text(0.5, 2.0, 'Workflow Execution:', fontsize=10, fontweight='bold', color='#1e293b')
        legend_text = """1. User query enters at START  2. AGENT NODE calls LLM  3. Conditional edge checks tool_calls
4. YES -> TOOLS NODE executes  5. Loop back to AGENT  6. NO -> END (return response)"""
        ax.text(0.5, 1.7, legend_text, fontsize=8, family='monospace', va='top',
                linespacing=1.3, color='#475569')

        ax.text(5.5, 0.3, 'Page 2 of 4', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 3: Tools List ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8, 'Agent Tools (12 Total)', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Planning Tools Section
        ax.text(0.5, 7.4, 'PLANNING TOOLS (9)', fontsize=12, fontweight='bold', color='#1d4ed8')

        planning_tools = [
            ('get_mission_overview', 'Get all airports, targets, SAMs, drone configs'),
            ('get_drone_info', 'Get detailed constraints for a specific drone'),
            ('get_distance', 'Look up distance between two waypoints'),
            ('calculate_route_fuel', 'Calculate total fuel for a drone route'),
            ('validate_drone_route', 'Validate route against all constraints'),
            ('find_accessible_targets', 'Find targets drone can reach, by efficiency'),
            ('suggest_drone_route', 'Generate greedy baseline route for drone'),
            ('check_target_conflicts', 'Check for duplicate target assignments'),
            ('get_mission_summary', 'Get summary stats for multi-drone plan'),
        ]

        y = 7.0
        for name, desc in planning_tools:
            ax.text(0.7, y, f'  {name}', fontsize=9, fontweight='bold', family='monospace', color='#2563eb')
            ax.text(4.0, y, desc, fontsize=8, color='#4b5563')
            y -= 0.35

        # Optimization Tools Section
        ax.text(0.5, y - 0.2, 'OPTIMIZATION TOOLS (3)', fontsize=12, fontweight='bold', color='#059669')
        y -= 0.5

        opt_tools = [
            ('optimize_assign_unvisited', 'Insert unvisited targets into routes'),
            ('optimize_reassign_targets', 'Swap targets to closer drone trajectories'),
            ('optimize_remove_crossings', 'Fix self-crossing trajectories (2-opt)'),
        ]

        for name, desc in opt_tools:
            ax.text(0.7, y, f'  {name}', fontsize=9, fontweight='bold', family='monospace', color='#16a34a')
            ax.text(4.0, y, desc, fontsize=8, color='#4b5563')
            y -= 0.35

        # Tool I/O Box
        io_box = FancyBboxPatch((0.3, 0.8), 10.4, 2.5, boxstyle="round,pad=0.1",
                                 facecolor='#f8fafc', edgecolor='#94a3b8', linewidth=1)
        ax.add_patch(io_box)
        ax.text(0.5, 3.1, 'Tool Input/Output Pattern:', fontsize=10, fontweight='bold', color='#1e293b')
        io_text = """All tools follow a consistent pattern:
  - Input: Tool-specific parameters (drone_id, waypoints, routes dict, etc.)
  - Output: Formatted string with results

Tools access shared context via global variables:
  _current_env        Environment with airports, targets, SAMs
  _drone_configs      Per-drone configuration (fuel, airports, access)
  _distance_matrix    Pre-computed SAM-aware distances
  _sam_paths          Pre-computed SAM-avoiding paths"""
        ax.text(0.5, 2.85, io_text, fontsize=8, family='monospace', va='top',
                linespacing=1.25, color='#475569')

        ax.text(5.5, 0.3, 'Page 3 of 4', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 4: Data Flow and Integration ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8, 'System Integration & Data Flow', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Web UI Box
        ui_box = FancyBboxPatch((0.5, 5.5), 2.5, 2, boxstyle="round,pad=0.1",
                                 facecolor='#f1f5f9', edgecolor='#64748b', linewidth=2)
        ax.add_patch(ui_box)
        ax.text(1.75, 7.1, 'Web UI', fontsize=11, fontweight='bold', ha='center', color='#334155')
        ax.text(1.75, 6.6, 'index.html', fontsize=8, ha='center', color='#64748b')
        ax.text(1.75, 6.2, '- Canvas visualization', fontsize=7, ha='center', color='#475569')
        ax.text(1.75, 5.9, '- Route editing', fontsize=7, ha='center', color='#475569')

        # FastAPI Server Box
        api_box = FancyBboxPatch((4, 5.5), 3, 2, boxstyle="round,pad=0.1",
                                  facecolor='#fef2f2', edgecolor='#dc2626', linewidth=2)
        ax.add_patch(api_box)
        ax.text(5.5, 7.1, 'FastAPI Server', fontsize=11, fontweight='bold', ha='center', color='#991b1b')
        ax.text(5.5, 6.6, 'main.py', fontsize=8, ha='center', color='#b91c1c')
        ax.text(5.5, 6.2, '- /api/agent endpoint', fontsize=7, ha='center', color='#7f1d1d')
        ax.text(5.5, 5.9, '- Context preparation', fontsize=7, ha='center', color='#7f1d1d')

        # Agent Box
        agent_box2 = FancyBboxPatch((8, 5.5), 2.5, 2, boxstyle="round,pad=0.1",
                                     facecolor='#dbeafe', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(agent_box2)
        ax.text(9.25, 7.1, 'ISR Agent', fontsize=11, fontweight='bold', ha='center', color='#1d4ed8')
        ax.text(9.25, 6.6, 'isr_agent.py', fontsize=8, ha='center', color='#3b82f6')
        ax.text(9.25, 6.2, '- LangGraph workflow', fontsize=7, ha='center', color='#1e40af')
        ax.text(9.25, 5.9, '- 12 tools', fontsize=7, ha='center', color='#1e40af')

        # Arrows between components
        ax.annotate('', xy=(4, 6.5), xytext=(3, 6.5),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2))
        ax.text(3.5, 6.8, 'HTTP', fontsize=7, ha='center', color='#6b7280')

        ax.annotate('', xy=(8, 6.5), xytext=(7, 6.5),
                    arrowprops=dict(arrowstyle='->', color='#374151', lw=2))
        ax.text(7.5, 6.8, 'run_isr_agent()', fontsize=7, ha='center', color='#6b7280')

        # Lower components
        # Solver modules
        solver_box = FancyBboxPatch((0.5, 2.8), 4, 2, boxstyle="round,pad=0.1",
                                     facecolor='#dcfce7', edgecolor='#16a34a', linewidth=2)
        ax.add_patch(solver_box)
        ax.text(2.5, 4.4, 'Solver Modules', fontsize=11, fontweight='bold', ha='center', color='#166534')
        ax.text(2.5, 4.0, 'post_optimizer.py', fontsize=8, ha='center', color='#22c55e')
        ax.text(2.5, 3.6, 'sam_navigation.py', fontsize=8, ha='center', color='#22c55e')
        ax.text(2.5, 3.2, 'sam_distance_matrix.py', fontsize=8, ha='center', color='#22c55e')

        # Path Planning Core
        path_box = FancyBboxPatch((5.5, 2.8), 4, 2, boxstyle="round,pad=0.1",
                                   facecolor='#fef3c7', edgecolor='#d97706', linewidth=2)
        ax.add_patch(path_box)
        ax.text(7.5, 4.4, 'Path Planning Core', fontsize=11, fontweight='bold', ha='center', color='#92400e')
        ax.text(7.5, 4.0, 'boundary_navigation.py', fontsize=8, ha='center', color='#b45309')
        ax.text(7.5, 3.6, '- SAM avoidance', fontsize=7, ha='center', color='#a16207')
        ax.text(7.5, 3.2, '- Tangent-arc navigation', fontsize=7, ha='center', color='#a16207')

        # Connections
        ax.annotate('', xy=(2.5, 4.8), xytext=(5.5, 5.5),
                    arrowprops=dict(arrowstyle='<->', color='#9ca3af', lw=1.5, linestyle='--'))
        ax.annotate('', xy=(7.5, 4.8), xytext=(9.25, 5.5),
                    arrowprops=dict(arrowstyle='<->', color='#9ca3af', lw=1.5, linestyle='--'))
        ax.annotate('', xy=(4.5, 3.8), xytext=(5.5, 3.8),
                    arrowprops=dict(arrowstyle='<->', color='#9ca3af', lw=1.5, linestyle='--'))

        # Data flow description box
        flow_box = FancyBboxPatch((0.3, 0.4), 10.4, 2.2, boxstyle="round,pad=0.1",
                                   facecolor='#f8fafc', edgecolor='#94a3b8', linewidth=1)
        ax.add_patch(flow_box)
        ax.text(0.5, 2.4, 'Data Flow:', fontsize=10, fontweight='bold', color='#1e293b')
        flow_text = """1. User interacts with Web UI (draws, edits, clicks "Run Agent")
2. Web UI sends POST to /api/agent with environment + query
3. FastAPI prepares context and calls run_isr_agent()
4. Agent uses tools which access Solver modules for optimization
5. Solver uses Path Planning Core for SAM-aware distances
6. Agent returns ROUTE_Dx formatted routes -> Web UI renders on canvas"""
        ax.text(0.5, 2.15, flow_text, fontsize=8, family='monospace', va='top',
                linespacing=1.25, color='#475569')

        ax.text(5.5, 0.15, 'Page 4 of 4', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

    print(f"PDF generated: {output_path}")


if __name__ == "__main__":
    output = "/Users/kamalali/isr_projects/isr_web/docs/ISR_Agent_Architecture.pdf"
    create_agent_architecture_pdf(output)
