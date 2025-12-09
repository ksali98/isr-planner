"""
ISR Multi-Agent Mission Planning System

This module exports the run_isr_agent function from the multi-agent v3 implementation.

Architecture (v3):
- Coordinator Agent: Understands user request, orchestrates workflow
- Allocator Agent: Distributes targets to drones optimally
- Router Agent: Computes optimal routes using Held-Karp algorithm
- Validator Agent: Checks routes against constraints

Key v3 improvements over v2:
1. Mandatory tool calls using tool_choice="required"
2. Explicit state transitions via phase enum (not text parsing)
3. Clear separation of concerns between agents
4. Designed for extension to delivery system
"""

# Import from v3 implementation
from .isr_agent_multi_v3 import run_isr_agent

# Export for use by other modules
__all__ = ["run_isr_agent"]
