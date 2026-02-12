"""
Algorithm Registry for ISR Planner

This module defines all available algorithms for target allocation, route optimization,
and solving. Each algorithm has a canonical ID, display name, description, and aliases
that can be used in natural language commands.

Used by:
- Constraint parser: To recognize algorithm commands from user messages
- Agents: To track which algorithms were used
- Responder: To explain algorithms when asked
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable
from enum import Enum


# ============================================================================
# Algorithm Execution Tracking
# ============================================================================

@dataclass
class AlgorithmExecution:
    """Record of an algorithm execution."""

    algorithm_id: str           # e.g., "swap_closer"
    algorithm_name: str         # e.g., "Swap Closer"
    agent: str                  # e.g., "Optimizer"

    # Execution details
    iterations: int = 1
    operations_performed: int = 0

    # Results summary
    result_summary: str = ""    # e.g., "5 swaps made, saved 42.3 fuel"

    # Timing (optional)
    duration_ms: float = 0.0

    # User-requested vs auto-selected
    user_requested: bool = False

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            "algorithm_id": self.algorithm_id,
            "algorithm_name": self.algorithm_name,
            "agent": self.agent,
            "iterations": self.iterations,
            "operations_performed": self.operations_performed,
            "result_summary": self.result_summary,
            "duration_ms": self.duration_ms,
            "user_requested": self.user_requested
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AlgorithmExecution":
        """Create from dictionary."""
        return cls(
            algorithm_id=data.get("algorithm_id", "unknown"),
            algorithm_name=data.get("algorithm_name", "Unknown"),
            agent=data.get("agent", "Unknown"),
            iterations=data.get("iterations", 1),
            operations_performed=data.get("operations_performed", 0),
            result_summary=data.get("result_summary", ""),
            duration_ms=data.get("duration_ms", 0.0),
            user_requested=data.get("user_requested", False)
        )


# ============================================================================
# Algorithm Registries
# ============================================================================

ALLOCATOR_ALGORITHMS = {
    "efficient": {
        "name": "Efficient (Auction-Based)",
        "description": "Maximizes priority-per-fuel-unit ratio using auction mechanism",
        "aliases": ["auction", "priority-per-fuel", "default"]
    },
    "greedy": {
        "name": "Greedy",
        "description": "Assigns highest-priority targets to nearest capable drone",
        "aliases": ["priority-first", "high-priority"]
    },
    "balanced": {
        "name": "Balanced",
        "description": "Distributes targets evenly by count across drones",
        "aliases": ["even", "equal-distribution"]
    },
    "geographic": {
        "name": "Geographic",
        "description": "Minimizes detours via spatial corridors",
        "aliases": ["spatial", "corridor", "regional"]
    },
    "exclusive": {
        "name": "Exclusive-First",
        "description": "Handles single-drone-only targets first",
        "aliases": ["exclusive-first", "constrained-first"]
    },
    "adaptive": {
        "name": "Adaptive Selection",
        "description": "Automatically selects best strategy based on environment analysis",
        "aliases": ["auto", "smart", "automatic"]
    }
}

OPTIMIZER_ALGORITHMS = {
    "insert_missed": {
        "name": "Insert Missed",
        "description": "Inserts unvisited targets into routes where they fit within fuel budget",
        "aliases": ["insert", "add-missed", "fill"]
    },
    "swap_closer": {
        "name": "Swap Closer",
        "description": "Moves targets to trajectories that pass closer to them",
        "aliases": ["swap", "reassign", "move-closer"]
    },
    "crossing_removal": {
        "name": "Crossing Removal (2-opt)",
        "description": "Removes route self-crossings using 2-opt reversal",
        "aliases": ["no-cross", "2-opt", "uncross"]
    },
    "unified": {
        "name": "Unified Optimizer",
        "description": "Interleaved Insert+Swap with greedy single-operation execution",
        "aliases": ["interleaved", "combined"]
    }
}

SOLVER_ALGORITHMS = {
    "held_karp": {
        "name": "Held-Karp Dynamic Programming",
        "description": "Optimal TSP/orienteering solver for small instances (up to ~15 targets per drone)",
        "aliases": ["dp", "dynamic-programming", "optimal"]
    },
    "greedy_tsp": {
        "name": "Greedy TSP",
        "description": "Fast nearest-neighbor heuristic for larger instances",
        "aliases": ["nearest-neighbor", "nn", "fast"]
    }
}


# ============================================================================
# Algorithm Resolution Functions
# ============================================================================

def resolve_algorithm_alias(alias: str, context: str) -> Optional[str]:
    """
    Resolve an alias to canonical algorithm ID.

    Args:
        alias: User-provided algorithm name or alias (e.g., "greedy", "2-opt", "dp")
        context: The type of algorithm ("allocation", "optimizer", "solver")

    Returns:
        Canonical algorithm ID or None if not found
    """
    alias_lower = alias.lower().strip()

    # Select registry based on context
    if context in ("allocation", "allocation_strategy", "allocator"):
        registry = ALLOCATOR_ALGORITHMS
    elif context in ("optimizer", "optimization", "optimizer_include", "optimizer_exclude"):
        registry = OPTIMIZER_ALGORITHMS
    elif context in ("solver", "solver_algorithm"):
        registry = SOLVER_ALGORITHMS
    else:
        # Try all registries
        for reg in [ALLOCATOR_ALGORITHMS, OPTIMIZER_ALGORITHMS, SOLVER_ALGORITHMS]:
            result = _search_registry(alias_lower, reg)
            if result:
                return result
        return None

    return _search_registry(alias_lower, registry)


def _search_registry(alias: str, registry: Dict[str, Dict]) -> Optional[str]:
    """Search a registry for matching algorithm."""
    for alg_id, alg_info in registry.items():
        # Check exact ID match
        if alias == alg_id:
            return alg_id
        # Check aliases
        if alias in [a.lower() for a in alg_info.get("aliases", [])]:
            return alg_id
        # Check name (partial match)
        if alias in alg_info.get("name", "").lower():
            return alg_id
    return None


def get_algorithm_info(algorithm_id: str) -> Optional[Dict[str, Any]]:
    """
    Get algorithm information by canonical ID.

    Args:
        algorithm_id: Canonical algorithm ID (e.g., "held_karp", "unified")

    Returns:
        Dict with name, description, aliases, and context (allocator/optimizer/solver)
    """
    # Check each registry
    if algorithm_id in ALLOCATOR_ALGORITHMS:
        info = ALLOCATOR_ALGORITHMS[algorithm_id].copy()
        info["context"] = "allocator"
        info["id"] = algorithm_id
        return info

    if algorithm_id in OPTIMIZER_ALGORITHMS:
        info = OPTIMIZER_ALGORITHMS[algorithm_id].copy()
        info["context"] = "optimizer"
        info["id"] = algorithm_id
        return info

    if algorithm_id in SOLVER_ALGORITHMS:
        info = SOLVER_ALGORITHMS[algorithm_id].copy()
        info["context"] = "solver"
        info["id"] = algorithm_id
        return info

    return None


def get_all_algorithms() -> Dict[str, List[Dict[str, Any]]]:
    """
    Get all available algorithms grouped by context.

    Returns:
        Dict with keys "allocator", "optimizer", "solver" containing lists of algorithm info
    """
    result = {
        "allocator": [],
        "optimizer": [],
        "solver": []
    }

    for alg_id, info in ALLOCATOR_ALGORITHMS.items():
        result["allocator"].append({
            "id": alg_id,
            "name": info["name"],
            "description": info["description"],
            "aliases": info.get("aliases", [])
        })

    for alg_id, info in OPTIMIZER_ALGORITHMS.items():
        result["optimizer"].append({
            "id": alg_id,
            "name": info["name"],
            "description": info["description"],
            "aliases": info.get("aliases", [])
        })

    for alg_id, info in SOLVER_ALGORITHMS.items():
        result["solver"].append({
            "id": alg_id,
            "name": info["name"],
            "description": info["description"],
            "aliases": info.get("aliases", [])
        })

    return result


# ============================================================================
# Algorithm Summary Generation
# ============================================================================

def generate_algorithm_summary(executions: List[AlgorithmExecution]) -> str:
    """
    Generate a human-readable summary of algorithms used.

    Args:
        executions: List of AlgorithmExecution records

    Returns:
        Formatted markdown string describing the algorithms used

    Example output:
        I used the following algorithms:
        1. **Efficient Allocation** - Assigned 12 targets across 4 drones
        2. **Held-Karp Solver** - Computed optimal routes for each drone
        3. **Unified Optimizer** - 3 iterations: 2 inserts, 1 swap, saved 15.2 fuel
        4. **Crossing Removal** - Fixed 1 crossing in D2's route
    """
    if not executions:
        return "No algorithms were executed."

    lines = ["I used the following algorithms:"]
    for i, exec in enumerate(executions, 1):
        line = f"{i}. **{exec.algorithm_name}**"
        if exec.result_summary:
            line += f" - {exec.result_summary}"
        lines.append(line)

    return "\n".join(lines)


def generate_algorithm_explanation(algorithm_id: str) -> str:
    """
    Generate a detailed explanation of an algorithm.

    Args:
        algorithm_id: Canonical algorithm ID

    Returns:
        Formatted explanation of how the algorithm works
    """
    info = get_algorithm_info(algorithm_id)
    if not info:
        return f"Unknown algorithm: {algorithm_id}"

    explanation = f"**{info['name']}**\n\n{info['description']}\n\n"

    # Add algorithm-specific details
    if algorithm_id == "efficient":
        explanation += (
            "This algorithm uses an auction-based mechanism where each target is assigned "
            "to the drone that can visit it with the best priority-per-fuel ratio. "
            "It ensures high-value targets are prioritized while minimizing fuel consumption."
        )
    elif algorithm_id == "unified":
        explanation += (
            "The Unified Optimizer interleaves Insert and Swap operations, evaluating ALL "
            "possible operations at each iteration and executing the single best one. This "
            "allows the optimization landscape to adapt dynamically.\n\n"
            "**Scoring Strategy (Hybrid):**\n"
            "- Insert score: `priority * 10 / cost` (maximize priority per fuel)\n"
            "- Swap score: `gain` (maximize fuel savings)\n\n"
            "Crossing removal runs as a final cleanup step."
        )
    elif algorithm_id == "held_karp":
        explanation += (
            "Held-Karp is a dynamic programming algorithm that finds the optimal solution "
            "to the Traveling Salesman Problem (TSP) and its variants like Orienteering. "
            "It has O(n^2 * 2^n) complexity, making it optimal for small instances "
            "(up to ~15 targets per drone)."
        )
    elif algorithm_id == "swap_closer":
        explanation += (
            "For each target, this algorithm calculates its distance to the current route "
            "segment and searches for closer segments across all trajectories. If a closer "
            "segment exists on a capable drone with sufficient fuel, the target is moved."
        )
    elif algorithm_id == "crossing_removal":
        explanation += (
            "This algorithm uses 2-opt reversal to eliminate route self-crossings. "
            "When two non-adjacent edges cross, reversing the segment between them "
            "removes the crossing and typically shortens the route."
        )

    return explanation


# ============================================================================
# Algorithm Command Detection
# ============================================================================

class AlgorithmCommandType(Enum):
    """Types of algorithm commands that can be detected."""
    ALLOCATION_STRATEGY = "allocation_strategy"
    OPTIMIZER_INCLUDE = "optimizer_include"
    OPTIMIZER_EXCLUDE = "optimizer_exclude"
    SOLVER_ALGORITHM = "solver_algorithm"


def get_algorithm_command_patterns() -> List[tuple]:
    """
    Get regex patterns for detecting algorithm commands.

    Returns:
        List of (pattern, command_type) tuples
    """
    import re

    return [
        # Allocation strategy commands
        (re.compile(r"use\s+(?:the\s+)?(\w+)\s+(?:allocation|strategy)", re.IGNORECASE),
         AlgorithmCommandType.ALLOCATION_STRATEGY),
        (re.compile(r"allocate\s+(?:using|with)\s+(\w+)", re.IGNORECASE),
         AlgorithmCommandType.ALLOCATION_STRATEGY),

        # Optimizer include commands
        (re.compile(r"(?:only\s+)?run\s+(\w+)\s+optim", re.IGNORECASE),
         AlgorithmCommandType.OPTIMIZER_INCLUDE),
        (re.compile(r"use\s+(?:the\s+)?(\w+)\s+optim", re.IGNORECASE),
         AlgorithmCommandType.OPTIMIZER_INCLUDE),

        # Optimizer exclude commands
        (re.compile(r"skip\s+(\w+)\s*(?:optimization|optimizer)?", re.IGNORECASE),
         AlgorithmCommandType.OPTIMIZER_EXCLUDE),
        (re.compile(r"no\s+(\w+)\s*(?:optimization|removal)?", re.IGNORECASE),
         AlgorithmCommandType.OPTIMIZER_EXCLUDE),
        (re.compile(r"don'?t\s+(?:use|run)\s+(\w+)", re.IGNORECASE),
         AlgorithmCommandType.OPTIMIZER_EXCLUDE),

        # Solver commands
        (re.compile(r"use\s+(?:the\s+)?(\w+)\s+solver", re.IGNORECASE),
         AlgorithmCommandType.SOLVER_ALGORITHM),
    ]
