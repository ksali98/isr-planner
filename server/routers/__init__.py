"""
API Routers for ISR Planner.

This package contains modular FastAPI routers extracted from the monolithic main.py.

Routers:
    - solve: Solving and optimization endpoints
    - agent: AI agent chat and memory endpoints
    - environment: Environment management endpoints
"""

from .solve import router as solve_router
from .agent import router as agent_router
from .environment import router as environment_router

__all__ = ["solve_router", "agent_router", "environment_router"]
