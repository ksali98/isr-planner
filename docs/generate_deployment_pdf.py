"""
Generate ISR Web Application Deployment Guide PDF.
Includes directory structure, Docker setup, and deployment options.
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


def create_deployment_pdf(output_path):
    """Generate a multi-page PDF documenting deployment setup."""

    with PdfPages(output_path) as pdf:
        # === PAGE 1: Title ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 6.5, 'ISR Mission Planning', fontsize=28, fontweight='bold',
                ha='center', va='center', color='#1a1a1a')
        ax.text(5.5, 5.5, 'Deployment & Setup Guide', fontsize=22,
                ha='center', va='center', color='#4b5563')

        ax.text(5.5, 3.5, f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}',
                fontsize=12, ha='center', va='center', color='#9ca3af')

        ax.text(5.5, 0.5, 'Page 1', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 2: Directory Structure ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Project Directory Structure', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Directory tree box - made taller
        tree_box = FancyBboxPatch((0.3, 3.2), 5.1, 4.6, boxstyle="round,pad=0.1",
                                   facecolor='#f8fafc', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(tree_box)
        ax.text(2.85, 7.6, 'CLEAN STRUCTURE', fontsize=11, fontweight='bold',
                ha='center', color='#1d4ed8')

        tree = """isr_web/
├── server/
│   ├── agents/
│   │   ├── isr_agent.py   # Claude agent
│   │   └── graph.py       # GPT agent
│   ├── solver/
│   │   ├── orienteering_solver.py
│   │   ├── solver_bridge.py
│   │   ├── trajectory_planner.py
│   │   ├── sam_distance_matrix.py
│   │   ├── target_allocator.py
│   │   └── post_optimizer.py
│   └── main.py            # FastAPI app
├── webapp/
│   ├── index.html
│   ├── isr.js
│   └── isr.css
├── docs/                  # PDF generators
├── requirements.txt
├── Dockerfile
├── docker-compose.yml
└── run_planner.sh"""
        ax.text(0.5, 7.35, tree, fontsize=7, va='top', color='#1f2937',
                family='monospace', linespacing=1.1)

        # Dependencies box
        deps_box = FancyBboxPatch((5.6, 3.2), 5.1, 4.6, boxstyle="round,pad=0.1",
                                   facecolor='#ecfdf5', edgecolor='#059669', linewidth=2)
        ax.add_patch(deps_box)
        ax.text(8.15, 7.6, 'SIBLING PACKAGES', fontsize=11, fontweight='bold',
                ha='center', color='#047857')

        deps = """isr_projects/          # Parent dir
├── isr_web/           # This app
├── path_planning_core/
│   ├── boundary_navigation.py
│   └── sam_wrapping.py
├── isr_editor/
│   ├── solver/
│   │   └── orienteering_interface.py
│   └── path_planning/
│       └── sam_navigator.py
├── isr_benchmark/
└── isr_agent/


NOTE: Docker build context must
be the parent isr_projects/
directory to include siblings."""
        ax.text(5.8, 7.35, deps, fontsize=7, va='top', color='#065f46',
                family='monospace', linespacing=1.1)

        # Files removed box
        removed_box = FancyBboxPatch((0.3, 0.5), 10.4, 2.5, boxstyle="round,pad=0.1",
                                      facecolor='#fef2f2', edgecolor='#dc2626', linewidth=2)
        ax.add_patch(removed_box)
        ax.text(5.5, 2.8, 'FILES MOVED TO _deprecated/ (Safe to Delete)',
                fontsize=11, fontweight='bold', ha='center', color='#991b1b')

        removed = """• server/main_v2.py, ui.py        - Legacy versions
• webapp/index_v2.html, isr_v2.js - Old UI files
• server/continuationChatGPT-1    - Debug artifact
• orienteering_with_matrix.py     - Duplicate solver
• src/                            - Empty directories
• webapp/editor/                  - Old Tkinter app"""
        ax.text(0.5, 2.5, removed, fontsize=7.5, va='top', color='#7f1d1d',
                family='monospace', linespacing=1.25)

        ax.text(5.5, 0.3, 'Page 2', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 3: Requirements & Environment ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Requirements & Environment Setup', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Requirements box - made taller
        req_box = FancyBboxPatch((0.3, 4.0), 5.1, 3.8, boxstyle="round,pad=0.1",
                                  facecolor='#fffbeb', edgecolor='#d97706', linewidth=2)
        ax.add_patch(req_box)
        ax.text(2.85, 7.6, 'requirements.txt', fontsize=11, fontweight='bold',
                ha='center', color='#b45309')

        requirements = """# Web Framework
fastapi>=0.122.0
uvicorn>=0.38.0
pydantic>=2.12.0
python-dotenv>=1.2.0

# LangGraph/LangChain
langgraph>=0.6.11
langchain>=0.3.27
langchain-anthropic>=0.3.22
langchain-core>=0.3.80

# OpenAI (GPT agent)
openai>=2.8.1

# Scientific
numpy>=2.0.0
matplotlib>=3.9.0

# Utilities
requests>=2.32.0"""
        ax.text(0.5, 7.35, requirements, fontsize=7, va='top', color='#92400e',
                family='monospace', linespacing=1.1)

        # Environment variables box
        env_box = FancyBboxPatch((5.6, 4.0), 5.1, 3.8, boxstyle="round,pad=0.1",
                                  facecolor='#f5f3ff', edgecolor='#7c3aed', linewidth=2)
        ax.add_patch(env_box)
        ax.text(8.15, 7.6, 'ENVIRONMENT VARIABLES', fontsize=10, fontweight='bold',
                ha='center', color='#5b21b6')

        env_vars = """Required:
────────────────────────────
ANTHROPIC_API_KEY=sk-ant-...
  For Claude-based ISR agent

Optional:
────────────────────────────
OPENAI_API_KEY=sk-...
  For GPT-based agent

PYTHONPATH=/path/to/isr_projects
  Required for sibling imports

Create .env file:
────────────────────────────
echo "ANTHROPIC_API_KEY=key" > .env
echo "OPENAI_API_KEY=key" >> .env"""
        ax.text(5.8, 7.35, env_vars, fontsize=7, va='top', color='#4c1d95',
                family='monospace', linespacing=1.1)

        # .gitignore box
        git_box = FancyBboxPatch((0.3, 0.5), 10.4, 3.2, boxstyle="round,pad=0.1",
                                  facecolor='#f8fafc', edgecolor='#475569', linewidth=2)
        ax.add_patch(git_box)
        ax.text(5.5, 3.5, '.gitignore Configuration', fontsize=11, fontweight='bold',
                ha='center', color='#334155')

        gitignore = """# Python                        # Runtime/Generated
venv/                           agent_memory.json
__pycache__/                    ai_solution.json
*.pyc                           mission_solution.json
*.pyo                           environment.json
*.egg-info/
                                # Generated PDFs
# OS                            docs/*.pdf
.DS_Store
                                # Deprecated files
# IDE                           _deprecated/
.vscode/
.idea/                          # Environment secrets
*.swp                           .env, .env.local"""
        ax.text(0.5, 3.2, gitignore, fontsize=7, va='top', color='#1f2937',
                family='monospace', linespacing=1.1)

        ax.text(5.5, 0.3, 'Page 3', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 4: Local Development ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Local Development Setup', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Quick start box
        quick_box = FancyBboxPatch((0.3, 4.8), 10.4, 3.0, boxstyle="round,pad=0.1",
                                    facecolor='#ecfdf5', edgecolor='#059669', linewidth=2)
        ax.add_patch(quick_box)
        ax.text(5.5, 7.6, 'QUICK START', fontsize=11, fontweight='bold',
                ha='center', color='#047857')

        quickstart = """# 1. Clone and setup                              # 3. Run the server
cd /path/to/isr_projects/isr_web                   python -m uvicorn server.main:app --reload --port 8893
python3 -m venv venv
source venv/bin/activate                           # 4. Open browser
pip install -r requirements.txt                    open http://localhost:8893

# 2. Set environment variables
export ANTHROPIC_API_KEY="your-api-key"
export PYTHONPATH=/path/to/isr_projects"""
        ax.text(0.5, 7.35, quickstart, fontsize=7, va='top', color='#065f46',
                family='monospace', linespacing=1.15)

        # run_planner.sh box
        script_box = FancyBboxPatch((0.3, 1.6), 5.1, 3.0, boxstyle="round,pad=0.1",
                                     facecolor='#f8fafc', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(script_box)
        ax.text(2.85, 4.4, 'run_planner.sh', fontsize=11, fontweight='bold',
                ha='center', color='#1d4ed8')

        script = """#!/usr/bin/env bash
PLANNER_PORT=8893

# Kill existing process on port
lsof -ti:${PLANNER_PORT} | xargs kill

cd /path/to/isr_projects/isr_web
source venv/bin/activate
export PYTHONPATH=/path/to/isr_projects

python -m uvicorn server.main:app \\
    --reload --port ${PLANNER_PORT}"""
        ax.text(0.5, 4.15, script, fontsize=7, va='top', color='#1f2937',
                family='monospace', linespacing=1.1)

        # Endpoints box
        endpoints_box = FancyBboxPatch((5.6, 1.6), 5.1, 3.0, boxstyle="round,pad=0.1",
                                        facecolor='#fffbeb', edgecolor='#d97706', linewidth=2)
        ax.add_patch(endpoints_box)
        ax.text(8.15, 4.4, 'API ENDPOINTS', fontsize=11, fontweight='bold',
                ha='center', color='#b45309')

        endpoints = """GET  /             → Web UI
GET  /api/health   → Health check

POST /api/environment
     → Load mission environment

POST /api/solve
     → Run solver for all drones

POST /api/agent
     → Send message to AI agent

GET/POST /api/agent/memory
     → Manage agent memories"""
        ax.text(5.8, 4.15, endpoints, fontsize=7, va='top', color='#92400e',
                family='monospace', linespacing=1.1)

        # Troubleshooting box
        trouble_box = FancyBboxPatch((0.3, 0.4), 10.4, 1.0, boxstyle="round,pad=0.1",
                                      facecolor='#fef2f2', edgecolor='#dc2626', linewidth=2)
        ax.add_patch(trouble_box)
        ax.text(5.5, 1.2, 'TROUBLESHOOTING', fontsize=10, fontweight='bold',
                ha='center', color='#991b1b')
        trouble = "Port in use: lsof -ti:8893 | xargs kill   |   Import error: Check PYTHONPATH   |   API key: Verify .env"
        ax.text(5.5, 0.75, trouble, fontsize=7, ha='center', color='#7f1d1d', family='monospace')

        ax.text(5.5, 0.15, 'Page 4', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 5: Docker Deployment ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Docker Deployment', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Dockerfile box
        docker_box = FancyBboxPatch((0.3, 3.8), 5.1, 4.0, boxstyle="round,pad=0.1",
                                     facecolor='#eff6ff', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(docker_box)
        ax.text(2.85, 7.6, 'Dockerfile', fontsize=11, fontweight='bold',
                ha='center', color='#1d4ed8')

        dockerfile = """FROM python:3.11-slim
WORKDIR /app

# Install dependencies
RUN apt-get update && apt-get install -y \\
    gcc && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy sibling packages
COPY path_planning_core/ /app/path_planning_core/
COPY isr_editor/ /app/isr_editor/

# Copy application
COPY server/ /app/isr_web/server/
COPY webapp/ /app/isr_web/webapp/

ENV PYTHONPATH=/app
EXPOSE 8893

CMD ["python", "-m", "uvicorn", \\
     "isr_web.server.main:app", \\
     "--host", "0.0.0.0", "--port", "8893"]"""
        ax.text(0.5, 7.35, dockerfile, fontsize=6.5, va='top', color='#1e40af',
                family='monospace', linespacing=1.05)

        # docker-compose box
        compose_box = FancyBboxPatch((5.6, 3.8), 5.1, 4.0, boxstyle="round,pad=0.1",
                                      facecolor='#f0fdf4', edgecolor='#16a34a', linewidth=2)
        ax.add_patch(compose_box)
        ax.text(8.15, 7.6, 'docker-compose.yml', fontsize=11, fontweight='bold',
                ha='center', color='#15803d')

        compose = """version: '3.8'

services:
  isr-web:
    build:
      context: ..  # isr_projects/
      dockerfile: isr_web/Dockerfile
    ports:
      - "8893:8893"
    environment:
      - ANTHROPIC_API_KEY=${ANTHROPIC_API_KEY}
      - OPENAI_API_KEY=${OPENAI_API_KEY:-}
    volumes:
      - ./agent_memory.json:/app/agent_memory.json
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f",
             "http://localhost:8893/"]
      interval: 30s
      timeout: 10s
      retries: 3"""
        ax.text(5.8, 7.35, compose, fontsize=6.5, va='top', color='#166534',
                family='monospace', linespacing=1.05)

        # Build commands box
        build_box = FancyBboxPatch((0.3, 0.5), 10.4, 3.0, boxstyle="round,pad=0.1",
                                    facecolor='#fefce8', edgecolor='#ca8a04', linewidth=2)
        ax.add_patch(build_box)
        ax.text(5.5, 3.3, 'BUILD & RUN COMMANDS', fontsize=11, fontweight='bold',
                ha='center', color='#a16207')

        commands = """# Option 1: Using docker-compose (recommended)
cd /path/to/isr_projects
export ANTHROPIC_API_KEY="your-key"
docker-compose -f isr_web/docker-compose.yml up --build

# Option 2: Manual docker build (from parent directory!)
cd /path/to/isr_projects
docker build -t isr-web -f isr_web/Dockerfile .
docker run -p 8893:8893 -e ANTHROPIC_API_KEY="your-key" isr-web

# Access: http://localhost:8893"""
        ax.text(0.5, 3.0, commands, fontsize=7.5, va='top', color='#854d0e',
                family='monospace', linespacing=1.15)

        ax.text(5.5, 0.3, 'Page 5', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

        # === PAGE 6: Cloud Deployment Options ===
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.set_xlim(0, 11)
        ax.set_ylim(0, 8.5)
        ax.axis('off')
        fig.patch.set_facecolor('white')

        ax.text(5.5, 8.1, 'Cloud Deployment Options', fontsize=16, fontweight='bold',
                ha='center', color='#1a1a1a')

        # Option A: Cloud VM
        vm_box = FancyBboxPatch((0.3, 5.3), 5.1, 2.5, boxstyle="round,pad=0.1",
                                 facecolor='#fff7ed', edgecolor='#ea580c', linewidth=2)
        ax.add_patch(vm_box)
        ax.text(2.85, 7.6, 'A: Cloud VM (AWS/GCP/Azure)', fontsize=10, fontweight='bold',
                ha='center', color='#c2410c')

        vm_text = """Cost: $5-20/month
Setup: Medium

1. Create VM (Ubuntu 22.04)
2. Install Docker
3. Clone repository
4. Set environment variables
5. Run docker-compose up -d

Pros: Full control, simple
Cons: Manual SSL, updates"""
        ax.text(0.5, 7.35, vm_text, fontsize=7, va='top', color='#9a3412',
                family='monospace', linespacing=1.1)

        # Option B: PaaS
        paas_box = FancyBboxPatch((5.6, 5.3), 5.1, 2.5, boxstyle="round,pad=0.1",
                                   facecolor='#f0fdf4', edgecolor='#16a34a', linewidth=2)
        ax.add_patch(paas_box)
        ax.text(8.15, 7.6, 'B: PaaS (Railway/Render/Fly)', fontsize=10, fontweight='bold',
                ha='center', color='#15803d')

        paas_text = """Cost: $5-25/month
Setup: Easy (git push deploys)

1. Connect GitHub repository
2. Set environment variables
3. Deploy automatically

Pros: Auto-deploy, HTTPS, scaling
Cons: Config for long agent calls"""
        ax.text(5.8, 7.35, paas_text, fontsize=7, va='top', color='#166534',
                family='monospace', linespacing=1.1)

        # Option C: Container Services
        container_box = FancyBboxPatch((0.3, 2.5), 5.1, 2.5, boxstyle="round,pad=0.1",
                                        facecolor='#eff6ff', edgecolor='#2563eb', linewidth=2)
        ax.add_patch(container_box)
        ax.text(2.85, 4.8, 'C: Container (ECS/Cloud Run)', fontsize=10, fontweight='bold',
                ha='center', color='#1d4ed8')

        container_text = """Cost: $10-50/month
Setup: Complex

1. Push image to registry
2. Configure service
3. Set up load balancer
4. Configure auto-scaling

Pros: Production-ready, scalable
Cons: More setup, higher cost"""
        ax.text(0.5, 4.55, container_text, fontsize=7, va='top', color='#1e40af',
                family='monospace', linespacing=1.1)

        # Option D: Internal
        internal_box = FancyBboxPatch((5.6, 2.5), 5.1, 2.5, boxstyle="round,pad=0.1",
                                       facecolor='#faf5ff', edgecolor='#7c3aed', linewidth=2)
        ax.add_patch(internal_box)
        ax.text(8.15, 4.8, 'D: Internal Server', fontsize=10, fontweight='bold',
                ha='center', color='#6d28d9')

        internal_text = """Cost: $0 (existing infra)
Setup: Depends on IT

1. Request server access
2. Install Docker
3. Deploy container
4. Configure internal DNS

Pros: Data stays internal
Cons: IT approval, maintenance"""
        ax.text(5.8, 4.55, internal_text, fontsize=7, va='top', color='#5b21b6',
                family='monospace', linespacing=1.1)

        # Recommendation box
        rec_box = FancyBboxPatch((0.3, 0.5), 10.4, 1.8, boxstyle="round,pad=0.1",
                                  facecolor='#fefce8', edgecolor='#ca8a04', linewidth=2)
        ax.add_patch(rec_box)
        ax.text(5.5, 2.1, 'RECOMMENDATION', fontsize=11, fontweight='bold',
                ha='center', color='#a16207')

        rec_text = """For team sharing:
• Quick start → Railway/Render (B): Easy setup, auto-deploy, HTTPS included
• More control → AWS EC2 (A): ~$10/mo, full access, bring your own SSL
• Sensitive data → Internal server (D): On-premises, requires IT coordination"""
        ax.text(0.5, 1.8, rec_text, fontsize=7.5, va='top', color='#854d0e',
                family='monospace', linespacing=1.2)

        ax.text(5.5, 0.3, 'Page 6', fontsize=8, ha='center', color='#888888')
        pdf.savefig(fig, facecolor='white')
        plt.close()

    print(f"PDF generated: {output_path}")


if __name__ == "__main__":
    output = "/Users/kamalali/isr_projects/isr_web/docs/ISR_Deployment_Guide.pdf"
    create_deployment_pdf(output)
