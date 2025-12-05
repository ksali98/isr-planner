"""
Generate ISR Agent System Prompt PDF documentation.
Includes the full system prompt and memory system documentation.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.backends.backend_pdf import PdfPages
import textwrap
from datetime import datetime


def wrap_text(text, width=95):
    """Wrap text to fit in the page."""
    lines = []
    for paragraph in text.split('\n'):
        if not paragraph.strip():
            lines.append('')
        else:
            wrapped = textwrap.wrap(paragraph, width=width)
            lines.extend(wrapped if wrapped else [''])
    return lines


def create_prompt_pdf(output_path):
    """Generate a multi-page PDF documenting the ISR Agent system prompt."""

    # The full system prompt (from isr_agent.py)
    system_prompt = """You are an expert ISR (Intelligence, Surveillance, Reconnaissance) multi-drone mission planner embedded in a web-based planning tool.

IMPORTANT: You are NOT a chatbot that needs to explain what you "would" do. You ARE the planner.
When asked to plan routes, you MUST actually use your tools and output the routes.
The UI will automatically parse your ROUTE_Dx: output and display it.

Your job is to help plan optimal routes for MULTIPLE DRONES that:
1. Maximize total priority points collected across all drones
2. Respect each drone's individual fuel budget (CRITICAL - exceeding means mission failure)
3. Respect each drone's start and end airport constraints
4. Respect each drone's target type access restrictions (some drones can only visit certain target types)
5. Avoid assigning the same target to multiple drones (each target should be visited at most once)

AVAILABLE TOOLS:
- get_mission_overview: Get complete overview (airports, targets, SAMs, ALL drone configs) - CALL THIS FIRST
- get_drone_info: Get detailed info for a specific drone
- get_distance: Look up distance between any two waypoints
- calculate_route_fuel: Calculate total fuel for a drone's route
- validate_drone_route: Validate a route against a drone's specific constraints
- find_accessible_targets: Find targets accessible to a specific drone (respects type restrictions)
- suggest_drone_route: Get a greedy baseline route for a specific drone
- check_target_conflicts: Check if any targets are assigned to multiple drones
- get_mission_summary: Get summary stats for a multi-drone plan

OPTIMIZATION TOOLS (use ONLY when user explicitly requests):
- optimize_assign_unvisited: Insert unvisited targets into routes (UI: "Insert Missed" button)
- optimize_reassign_targets: Swap targets to closer drone trajectories (UI: "Swap Closer" button)
- optimize_remove_crossings: Fix self-crossing trajectories using 2-opt (UI: "Cross Remove" button)

WORKFLOW:
1. FIRST call get_mission_overview to understand ALL drones and their constraints
2. Plan routes for each enabled drone, considering their individual constraints
3. Use check_target_conflicts to ensure no duplicate assignments
4. Validate each drone's route with validate_drone_route
5. Use get_mission_summary to verify the complete plan
6. OUTPUT the routes in ROUTE_Dx format and STOP

OPTIMIZATION RULES (CRITICAL - read carefully):
- Do NOT automatically call optimization tools during planning
- Only use them when user explicitly asks to "optimize", "insert missed", "swap closer", or "remove crossings"
- optimize_remove_crossings: Call ONCE, only if routes visually cross themselves
- optimize_reassign_targets: Call AT MOST 8 times total, stop if no swaps are made
- optimize_assign_unvisited: Call ONCE, only if there are unvisited targets
- After ANY optimization tool call, OUTPUT the new routes and STOP immediately

CRITICAL RULES:
- Each drone has its OWN fuel budget, start/end airports, and type restrictions
- NEVER exceed any drone's fuel budget
- Drones may have different starting airports (e.g., D1 starts at A1, D2 starts at A2)
- Drones may have different ending airports
- ANY ENDPOINT: If a drone's end_airport is "Any" or "-", it can end at ANY airport.
  The solver automatically chooses the optimal ending airport to maximize points.
  When you see "ANY (optimal)" as end airport, the route can end at any airport.
- Some drones can only visit certain target types (A, B, C, D, E)
- Each target should only be visited by ONE drone
- Always validate routes before presenting them
- DO NOT explain what you "would" do or ask for additional tools - USE the tools you have!

OUTPUT FORMAT - When presenting your final plan, you MUST include routes in this EXACT format:
ROUTE_D1: A1,T3,T7,A1
ROUTE_D2: A2,T5,T8,A2
ROUTE_D3: A1,T1,A1
(etc. for each enabled drone)

The UI automatically parses lines starting with "ROUTE_D" and loads them into the planner.
You do NOT need any special capabilities to update the UI - just output the routes in this format.

Include for each drone:
- The route sequence (in ROUTE_Dx format above)
- Points collected
- Fuel used

Then provide a brief summary of the total mission performance."""

    memory_system_doc = """
PERSISTENT MEMORY SYSTEM
========================

The ISR Agent supports a persistent memory system that allows storing instructions,
corrections, preferences, and facts that persist across sessions.

MEMORY CATEGORIES:
- correction: User corrected the agent's behavior
- instruction: User gave a standing instruction
- preference: User preference for how to do things
- fact: Important fact to remember

HOW MEMORIES ARE USED:
1. On each agent invocation, active memories are loaded from agent_memory.json
2. Memories are appended to the system prompt in a special section:

   ============================================================
   IMPORTANT MEMORIES & INSTRUCTIONS FROM PREVIOUS SESSIONS:
   ============================================================

   [CORRECTION] Always start D1 from A1, not A2
   [INSTRUCTION] Prioritize type-A targets over type-B
   [PREFERENCE] Use balanced allocation strategy

   ============================================================
   END OF MEMORIES - Follow these instructions carefully!
   ============================================================

3. The agent considers these memories when planning routes

MEMORY MANAGEMENT:
- Memories are stored in: isr_web/agent_memory.json
- Users can add/delete memories via the UI (Agent tab -> Memory section)
- API endpoints:
  - GET /api/agent/memory - List all memories
  - POST /api/agent/memory - Add a memory
  - DELETE /api/agent/memory/{id} - Delete specific memory
  - DELETE /api/agent/memory - Clear all memories

MEMORY FILE FORMAT (agent_memory.json):
[
  {
    "id": 1,
    "timestamp": "2024-12-05T10:30:00.000Z",
    "category": "instruction",
    "content": "Always prioritize high-priority targets",
    "active": true
  },
  ...
]
"""

    with PdfPages(output_path) as pdf:
        # === PAGE 1: Title ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 6.5, 'ISR Mission Planning Agent', fontsize=28, fontweight='bold',
                ha='center', va='center', color='#1a1a1a')
        ax.text(5.5, 5.5, 'System Prompt Documentation', fontsize=20,
                ha='center', va='center', color='#4b5563')
        ax.text(5.5, 4.5, '(Including Memory System)', fontsize=16,
                ha='center', va='center', color='#6b7280')

        ax.text(5.5, 2.5, f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}',
                fontsize=12, ha='center', va='center', color='#9ca3af')

        ax.text(5.5, 0.5, 'Page 1', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGES 2-4: System Prompt ===
        prompt_lines = wrap_text(system_prompt, width=100)
        lines_per_page = 45
        total_pages = (len(prompt_lines) + lines_per_page - 1) // lines_per_page

        for page_num in range(total_pages):
            fig, ax = plt.subplots(figsize=(11, 8.5))
            ax.set_xlim(0, 11)
            ax.set_ylim(0, 8.5)
            ax.axis('off')
            fig.patch.set_facecolor('white')

            ax.text(5.5, 8.2, f'System Prompt ({page_num + 1}/{total_pages})',
                    fontsize=14, fontweight='bold', ha='center', color='#1a1a1a')

            start_idx = page_num * lines_per_page
            end_idx = min(start_idx + lines_per_page, len(prompt_lines))
            page_lines = prompt_lines[start_idx:end_idx]

            y = 7.8
            for line in page_lines:
                # Color code different sections
                color = '#333333'
                fontweight = 'normal'
                if line.startswith('AVAILABLE TOOLS:') or line.startswith('OPTIMIZATION TOOLS') or \
                   line.startswith('WORKFLOW:') or line.startswith('CRITICAL RULES:') or \
                   line.startswith('OUTPUT FORMAT') or line.startswith('OPTIMIZATION RULES'):
                    color = '#1d4ed8'
                    fontweight = 'bold'
                elif line.startswith('- ') and ':' in line:
                    color = '#047857'
                elif line.startswith('IMPORTANT:') or line.startswith('FLEXIBLE ENDPOINT:'):
                    color = '#dc2626'
                    fontweight = 'bold'
                elif line.startswith('ROUTE_D'):
                    color = '#9333ea'

                ax.text(0.3, y, line, fontsize=7.5, family='monospace',
                        va='top', ha='left', color=color, fontweight=fontweight)
                y -= 0.165

            ax.text(5.5, 0.3, f'Page {page_num + 2}', fontsize=8, ha='center', color='#888888')
            pdf.savefig(fig, facecolor='white')
            plt.close()

        # === PAGE: Memory System Documentation ===
        memory_lines = wrap_text(memory_system_doc, width=100)
        lines_per_page = 45
        total_mem_pages = (len(memory_lines) + lines_per_page - 1) // lines_per_page

        for page_num in range(total_mem_pages):
            fig, ax = plt.subplots(figsize=(11, 8.5))
            ax.set_xlim(0, 11)
            ax.set_ylim(0, 8.5)
            ax.axis('off')
            fig.patch.set_facecolor('white')

            ax.text(5.5, 8.2, f'Memory System ({page_num + 1}/{total_mem_pages})',
                    fontsize=14, fontweight='bold', ha='center', color='#1a1a1a')

            start_idx = page_num * lines_per_page
            end_idx = min(start_idx + lines_per_page, len(memory_lines))
            page_lines = memory_lines[start_idx:end_idx]

            y = 7.8
            for line in page_lines:
                color = '#333333'
                fontweight = 'normal'
                if 'PERSISTENT MEMORY SYSTEM' in line or 'MEMORY CATEGORIES:' in line or \
                   'HOW MEMORIES ARE USED:' in line or 'MEMORY MANAGEMENT:' in line or \
                   'MEMORY FILE FORMAT' in line:
                    color = '#1d4ed8'
                    fontweight = 'bold'
                elif line.startswith('- '):
                    color = '#047857'
                elif line.startswith('[') and ']' in line:
                    color = '#9333ea'

                ax.text(0.3, y, line, fontsize=7.5, family='monospace',
                        va='top', ha='left', color=color, fontweight=fontweight)
                y -= 0.165

            current_page = total_pages + page_num + 2
            ax.text(5.5, 0.3, f'Page {current_page}', fontsize=8, ha='center', color='#888888')
            pdf.savefig(fig, facecolor='white')
            plt.close()

    print(f"PDF generated: {output_path}")


if __name__ == "__main__":
    output = "/Users/kamalali/isr_projects/isr_web/docs/ISR_Agent_Prompt_With_Memory.pdf"
    create_prompt_pdf(output)
