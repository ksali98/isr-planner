"""
Constraint Program Schema and Compiler

Provides a DSL for expressing user constraints on mission planning:
- FORCE_VISIT: Ensure specific targets are visited
- MOVE: Assign a target to a specific drone
- REMOVE: Forbid visiting a target
- SWAP: Exchange targets between drones
- INSERT: Add unvisited target to existing route

The compiler transforms these high-level operations into solver patches
that the allocator and route optimizer can understand.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Union


# =============================================================================
# Constraint Operations
# =============================================================================

class ConstraintOp(str, Enum):
    """Supported constraint operations."""
    FORCE_VISIT = "FORCE_VISIT"
    MOVE = "MOVE"
    REMOVE = "REMOVE"
    SWAP = "SWAP"
    INSERT = "INSERT"


@dataclass
class ForceVisitConstraint:
    """Force specific targets to be visited."""
    op: str = "FORCE_VISIT"
    targets: List[str] = field(default_factory=list)
    scope: str = "global"  # "global" or "drone"
    drone_id: Optional[str] = None  # Required if scope="drone"


@dataclass
class MoveConstraint:
    """Move a target to a specific drone."""
    op: str = "MOVE"
    target: str = ""
    to_drone: str = ""
    lock: bool = True  # If True, prevent future re-optimization from moving it


@dataclass
class RemoveConstraint:
    """Forbid visiting a target."""
    op: str = "REMOVE"
    target: str = ""
    reason: Optional[str] = None


@dataclass
class SwapConstraint:
    """Swap targets between drones."""
    op: str = "SWAP"
    target_a: str = ""
    target_b: str = ""
    # If drone IDs are known, the swap is deterministic
    drone_a: Optional[str] = None
    drone_b: Optional[str] = None


@dataclass
class InsertConstraint:
    """Insert an unvisited target into a route."""
    op: str = "INSERT"
    target: str = ""
    drone_id: Optional[str] = None  # If None, find best drone
    position: Optional[int] = None  # If None, find best position


# Type alias for any constraint
Constraint = Union[
    ForceVisitConstraint,
    MoveConstraint,
    RemoveConstraint,
    SwapConstraint,
    InsertConstraint,
]


# =============================================================================
# Constraint Program
# =============================================================================

@dataclass
class ConstraintProgram:
    """
    A program of constraint operations to apply to a mission.

    The program is built from user input (natural language or explicit commands)
    and compiled into solver patches.

    Allocation operations (constraints list) are typed and compiled into patches.
    Sequencing hints are lightweight guidance for the Strategist agent to interpret
    when configuring solver parameters (start/end airports, mission segmentation).
    """
    constraints: List[Constraint] = field(default_factory=list)
    source: str = "user"  # "user", "system", "policy"
    original_text: Optional[str] = None  # Original user input
    parsed_at: Optional[str] = None
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    # Sequencing hints for Strategist interpretation
    # These are NOT compiled into patches - the Strategist agent reasons about how
    # to configure the solver based on these hints (e.g., setting start/end airports,
    # splitting into segmented solves, using pseudo-endpoints).
    #
    # Example structure:
    # {
    #     "start_with": {"priority_gte": 10},  # or {"targets": ["T5"]}
    #     "end_with": {"targets": ["T5", "T8"]},  # pick best of candidates
    #     "priority_order": "high_first",  # segment by priority
    #     "segment_by_priority": {"threshold": 7, "order": "high_first"},
    # }
    sequencing_hints: Optional[Dict[str, Any]] = None

    def add_force_visit(
        self,
        targets: List[str],
        scope: str = "global",
        drone_id: Optional[str] = None,
    ) -> "ConstraintProgram":
        """Add a FORCE_VISIT constraint."""
        self.constraints.append(ForceVisitConstraint(
            targets=targets,
            scope=scope,
            drone_id=drone_id,
        ))
        return self

    def add_move(
        self,
        target: str,
        to_drone: str,
        lock: bool = True,
    ) -> "ConstraintProgram":
        """Add a MOVE constraint."""
        self.constraints.append(MoveConstraint(
            target=target,
            to_drone=to_drone,
            lock=lock,
        ))
        return self

    def add_remove(
        self,
        target: str,
        reason: Optional[str] = None,
    ) -> "ConstraintProgram":
        """Add a REMOVE constraint."""
        self.constraints.append(RemoveConstraint(
            target=target,
            reason=reason,
        ))
        return self

    def add_swap(
        self,
        target_a: str,
        target_b: str,
        drone_a: Optional[str] = None,
        drone_b: Optional[str] = None,
    ) -> "ConstraintProgram":
        """Add a SWAP constraint."""
        self.constraints.append(SwapConstraint(
            target_a=target_a,
            target_b=target_b,
            drone_a=drone_a,
            drone_b=drone_b,
        ))
        return self

    def add_insert(
        self,
        target: str,
        drone_id: Optional[str] = None,
        position: Optional[int] = None,
    ) -> "ConstraintProgram":
        """Add an INSERT constraint."""
        self.constraints.append(InsertConstraint(
            target=target,
            drone_id=drone_id,
            position=position,
        ))
        return self

    def is_empty(self) -> bool:
        """Check if the program has no constraints."""
        return len(self.constraints) == 0

    def has_errors(self) -> bool:
        """Check if there are compilation errors."""
        return len(self.errors) > 0

    def set_sequencing_hints(self, hints: Dict[str, Any]) -> "ConstraintProgram":
        """Set sequencing hints for Strategist interpretation."""
        self.sequencing_hints = hints
        return self

    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dict for storage."""
        result = {
            "constraints": [
                {
                    "op": c.op,
                    **{k: v for k, v in vars(c).items() if k != "op" and v is not None}
                }
                for c in self.constraints
            ],
            "source": self.source,
            "original_text": self.original_text,
            "parsed_at": self.parsed_at,
            "errors": self.errors,
            "warnings": self.warnings,
        }
        if self.sequencing_hints:
            result["sequencing_hints"] = self.sequencing_hints
        return result

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ConstraintProgram":
        """Deserialize from dict."""
        program = cls(
            source=data.get("source", "user"),
            original_text=data.get("original_text"),
            parsed_at=data.get("parsed_at"),
            errors=data.get("errors", []),
            warnings=data.get("warnings", []),
            sequencing_hints=data.get("sequencing_hints"),
        )

        for c in data.get("constraints", []):
            op = c.get("op", "").upper()

            if op == "FORCE_VISIT":
                program.constraints.append(ForceVisitConstraint(
                    targets=c.get("targets", []),
                    scope=c.get("scope", "global"),
                    drone_id=c.get("drone_id"),
                ))
            elif op == "MOVE":
                program.constraints.append(MoveConstraint(
                    target=c.get("target", ""),
                    to_drone=c.get("to_drone", ""),
                    lock=c.get("lock", True),
                ))
            elif op == "REMOVE":
                program.constraints.append(RemoveConstraint(
                    target=c.get("target", ""),
                    reason=c.get("reason"),
                ))
            elif op == "SWAP":
                program.constraints.append(SwapConstraint(
                    target_a=c.get("target_a", ""),
                    target_b=c.get("target_b", ""),
                    drone_a=c.get("drone_a"),
                    drone_b=c.get("drone_b"),
                ))
            elif op == "INSERT":
                program.constraints.append(InsertConstraint(
                    target=c.get("target", ""),
                    drone_id=c.get("drone_id"),
                    position=c.get("position"),
                ))
            else:
                program.errors.append(f"Unknown operation: {op}")

        return program


# =============================================================================
# Compiler Result
# =============================================================================

@dataclass
class CompilerResult:
    """
    Result of compiling a constraint program into solver patches.

    Contains patches for different stages of the planning pipeline:
    - policy_patches: Changes to the coordinator policy
    - allocation_patches: Hard constraints for the allocator
    - solver_patches: Instructions for the route optimizer
    - post_opt_patches: Instructions for post-optimization tools
    """
    success: bool = True
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    # Patches for each stage
    policy_patches: Dict[str, Any] = field(default_factory=dict)
    allocation_patches: Dict[str, Any] = field(default_factory=dict)
    solver_patches: Dict[str, Any] = field(default_factory=dict)
    post_opt_patches: Dict[str, Any] = field(default_factory=dict)

    # For clarification loop
    need_clarification: bool = False
    clarification_questions: List[str] = field(default_factory=list)
    clarification_context: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dict for storage."""
        return {
            "success": self.success,
            "errors": self.errors,
            "warnings": self.warnings,
            "policy_patches": self.policy_patches,
            "allocation_patches": self.allocation_patches,
            "solver_patches": self.solver_patches,
            "post_opt_patches": self.post_opt_patches,
            "need_clarification": self.need_clarification,
            "clarification_questions": self.clarification_questions,
            "clarification_context": self.clarification_context,
        }


# =============================================================================
# Constraint Compiler
# =============================================================================

class ConstraintCompiler:
    """
    Compiles a ConstraintProgram into solver patches.

    The compiler validates constraints against the current mission state
    and generates patches that can be applied to the allocator and solver.
    """

    def __init__(
        self,
        drone_configs: Optional[Dict[str, Any]] = None,
        environment: Optional[Dict[str, Any]] = None,
        existing_solution: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize the compiler with mission context.

        Args:
            drone_configs: Drone configurations (for eligibility checks)
            environment: Environment data (targets, airports, SAMs)
            existing_solution: Current solution (for MOVE/SWAP validation)
        """
        self.drone_configs = drone_configs or {}
        self.environment = environment or {}
        self.existing_solution = existing_solution or {}

        # Build lookup tables
        self._target_ids = self._build_target_set()
        self._drone_ids = self._build_drone_set()
        self._eligibility = self._build_eligibility_map()
        self._current_allocation = self._build_current_allocation()

    def _build_target_set(self) -> set:
        """Build set of valid target IDs."""
        targets = self.environment.get("targets", [])
        return {str(t.get("id")) for t in targets}

    def _build_drone_set(self) -> set:
        """Build set of valid drone IDs."""
        return {str(did) for did in self.drone_configs.keys()}

    def _build_eligibility_map(self) -> Dict[str, List[str]]:
        """Build map of drone_id -> list of eligible target IDs."""
        eligibility = {}

        # Get target types
        target_types = {}
        for t in self.environment.get("targets", []):
            tid = str(t.get("id"))
            target_types[tid] = str(t.get("type", "")).upper()

        for did, cfg in self.drone_configs.items():
            did_str = str(did)
            accessible = cfg.get("accessibleTargets", cfg.get("accessible_targets", []))

            # Convert to set of types
            accessible_types = {str(t).upper() for t in accessible}

            # Find eligible targets
            eligible = []
            for tid, ttype in target_types.items():
                if "ALL" in accessible_types or ttype in accessible_types:
                    eligible.append(tid)

            eligibility[did_str] = eligible

        return eligibility

    def _build_current_allocation(self) -> Dict[str, str]:
        """Build map of target_id -> drone_id from current solution."""
        allocation = {}

        routes = self.existing_solution.get("routes", {})
        for did, route_data in routes.items():
            did_str = str(did)
            route = route_data.get("route", []) if isinstance(route_data, dict) else []
            for waypoint in route:
                wid = str(waypoint)
                if wid.startswith("T"):
                    allocation[wid] = did_str

        return allocation

    def _validate_target(self, target_id: str) -> Optional[str]:
        """Validate that a target ID exists. Returns error message if invalid."""
        if target_id not in self._target_ids:
            return f"Unknown target: {target_id}"
        return None

    def _validate_drone(self, drone_id: str) -> Optional[str]:
        """Validate that a drone ID exists. Returns error message if invalid."""
        # Normalize drone ID (handle "D1" vs "1")
        normalized = drone_id.lstrip("D")
        if normalized not in self._drone_ids and drone_id not in self._drone_ids:
            return f"Unknown drone: {drone_id}"
        return None

    def _check_eligibility(self, target_id: str, drone_id: str) -> Optional[str]:
        """Check if drone is eligible to visit target. Returns error if not."""
        normalized_drone = drone_id.lstrip("D")

        eligible = self._eligibility.get(
            normalized_drone,
            self._eligibility.get(drone_id, [])
        )

        if target_id not in eligible:
            return f"Drone {drone_id} is not eligible to visit target {target_id} (sensor type mismatch)"
        return None

    def compile(self, program: ConstraintProgram) -> CompilerResult:
        """
        Compile a constraint program into solver patches.

        Validates all constraints and generates patches for:
        - policy_patches: force_allocation, use_existing_allocation
        - allocation_patches: lock_assignment, forbidden_targets, required_targets
        - solver_patches: force_visit
        - post_opt_patches: insert_targets, swap_pairs
        """
        result = CompilerResult()

        # Initialize patch structures
        result.policy_patches = {
            "force_allocation": False,
            "use_existing_allocation": False,
        }

        result.allocation_patches = {
            "lock_assignment": {},  # target_id -> drone_id
            "forbidden_targets": [],
            "required_targets": [],
        }

        result.solver_patches = {
            "force_visit": [],
        }

        result.post_opt_patches = {
            "insert_targets": [],
            "swap_pairs": [],
        }

        # Process each constraint
        for constraint in program.constraints:
            if isinstance(constraint, ForceVisitConstraint):
                self._compile_force_visit(constraint, result)
            elif isinstance(constraint, MoveConstraint):
                self._compile_move(constraint, result)
            elif isinstance(constraint, RemoveConstraint):
                self._compile_remove(constraint, result)
            elif isinstance(constraint, SwapConstraint):
                self._compile_swap(constraint, result)
            elif isinstance(constraint, InsertConstraint):
                self._compile_insert(constraint, result)
            else:
                result.errors.append(f"Unknown constraint type: {type(constraint)}")

        # If any constraints modify allocation, force re-allocation
        if (
            result.allocation_patches["lock_assignment"]
            or result.allocation_patches["forbidden_targets"]
            or result.allocation_patches["required_targets"]
        ):
            result.policy_patches["force_allocation"] = True

        # Check for errors
        result.success = len(result.errors) == 0

        return result

    def _compile_force_visit(
        self,
        constraint: ForceVisitConstraint,
        result: CompilerResult,
    ) -> None:
        """Compile FORCE_VISIT constraint."""
        for target in constraint.targets:
            # Validate target
            error = self._validate_target(target)
            if error:
                result.errors.append(error)
                continue

            # Add to required targets
            if target not in result.allocation_patches["required_targets"]:
                result.allocation_patches["required_targets"].append(target)

            # Add to solver force_visit
            if target not in result.solver_patches["force_visit"]:
                result.solver_patches["force_visit"].append(target)

            # If scope is drone-specific, also lock assignment
            if constraint.scope == "drone" and constraint.drone_id:
                drone_id = constraint.drone_id.lstrip("D")

                # Validate drone
                error = self._validate_drone(drone_id)
                if error:
                    result.errors.append(error)
                    continue

                # Check eligibility
                error = self._check_eligibility(target, drone_id)
                if error:
                    result.errors.append(error)
                    continue

                result.allocation_patches["lock_assignment"][target] = drone_id

    def _compile_move(
        self,
        constraint: MoveConstraint,
        result: CompilerResult,
    ) -> None:
        """Compile MOVE constraint."""
        target = constraint.target
        to_drone = constraint.to_drone.lstrip("D")

        # Validate target
        error = self._validate_target(target)
        if error:
            result.errors.append(error)
            return

        # Validate drone
        error = self._validate_drone(to_drone)
        if error:
            result.errors.append(error)
            return

        # Check eligibility
        error = self._check_eligibility(target, to_drone)
        if error:
            result.errors.append(error)
            return

        # Check if target is already assigned to this drone
        current_drone = self._current_allocation.get(target)
        if current_drone == to_drone:
            if constraint.lock:
                # Lock it in place even if already there
                result.allocation_patches["lock_assignment"][target] = to_drone
                result.warnings.append(
                    f"Target {target} already assigned to drone {to_drone}, locking assignment"
                )
            else:
                result.warnings.append(
                    f"Target {target} already assigned to drone {to_drone} (no-op)"
                )
            return

        # Lock the assignment
        result.allocation_patches["lock_assignment"][target] = to_drone

        # Use existing allocation as base
        result.policy_patches["use_existing_allocation"] = True

    def _compile_remove(
        self,
        constraint: RemoveConstraint,
        result: CompilerResult,
    ) -> None:
        """Compile REMOVE constraint."""
        target = constraint.target

        # Validate target
        error = self._validate_target(target)
        if error:
            result.errors.append(error)
            return

        # Add to forbidden list
        if target not in result.allocation_patches["forbidden_targets"]:
            result.allocation_patches["forbidden_targets"].append(target)

    def _compile_swap(
        self,
        constraint: SwapConstraint,
        result: CompilerResult,
    ) -> None:
        """Compile SWAP constraint."""
        target_a = constraint.target_a
        target_b = constraint.target_b

        # Validate targets
        for target in [target_a, target_b]:
            error = self._validate_target(target)
            if error:
                result.errors.append(error)
                return

        # Get current assignments
        drone_a = constraint.drone_a or self._current_allocation.get(target_a)
        drone_b = constraint.drone_b or self._current_allocation.get(target_b)

        if not drone_a or not drone_b:
            # Need clarification - targets not currently assigned
            if not drone_a:
                result.need_clarification = True
                result.clarification_questions.append(
                    f"Target {target_a} is not currently assigned. Which drone should it be assigned to?"
                )
                result.clarification_context["target_a"] = target_a

            if not drone_b:
                result.need_clarification = True
                result.clarification_questions.append(
                    f"Target {target_b} is not currently assigned. Which drone should it be assigned to?"
                )
                result.clarification_context["target_b"] = target_b

            return

        # Normalize drone IDs
        drone_a = drone_a.lstrip("D")
        drone_b = drone_b.lstrip("D")

        # Check eligibility for swapped assignments
        error = self._check_eligibility(target_a, drone_b)
        if error:
            result.errors.append(f"Cannot swap: {error}")
            return

        error = self._check_eligibility(target_b, drone_a)
        if error:
            result.errors.append(f"Cannot swap: {error}")
            return

        # If drones are the same, it's a no-op
        if drone_a == drone_b:
            result.warnings.append(
                f"Targets {target_a} and {target_b} are both on drone {drone_a}, swap is no-op"
            )
            return

        # Add swap to post-opt patches (executed after allocation)
        result.post_opt_patches["swap_pairs"].append({
            "target_a": target_a,
            "target_b": target_b,
            "drone_a": drone_a,
            "drone_b": drone_b,
        })

        # Also lock the new assignments
        result.allocation_patches["lock_assignment"][target_a] = drone_b
        result.allocation_patches["lock_assignment"][target_b] = drone_a

        # Use existing allocation as base
        result.policy_patches["use_existing_allocation"] = True

    def _compile_insert(
        self,
        constraint: InsertConstraint,
        result: CompilerResult,
    ) -> None:
        """Compile INSERT constraint."""
        target = constraint.target

        # Validate target
        error = self._validate_target(target)
        if error:
            result.errors.append(error)
            return

        # Check if already assigned
        if target in self._current_allocation:
            result.warnings.append(
                f"Target {target} is already assigned to drone {self._current_allocation[target]}"
            )
            return

        insert_spec = {"target": target}

        # If drone specified, validate and check eligibility
        if constraint.drone_id:
            drone_id = constraint.drone_id.lstrip("D")

            error = self._validate_drone(drone_id)
            if error:
                result.errors.append(error)
                return

            error = self._check_eligibility(target, drone_id)
            if error:
                result.errors.append(error)
                return

            insert_spec["drone_id"] = drone_id

        if constraint.position is not None:
            insert_spec["position"] = constraint.position

        # Add to post-opt patches
        result.post_opt_patches["insert_targets"].append(insert_spec)

        # Also add to required targets so allocation knows
        if target not in result.allocation_patches["required_targets"]:
            result.allocation_patches["required_targets"].append(target)

        # Enable insert_unvisited post-opt
        result.policy_patches["insert_unvisited"] = True


def compile_constraints(
    program: ConstraintProgram,
    drone_configs: Optional[Dict[str, Any]] = None,
    environment: Optional[Dict[str, Any]] = None,
    existing_solution: Optional[Dict[str, Any]] = None,
) -> CompilerResult:
    """
    Convenience function to compile a constraint program.

    Args:
        program: The constraint program to compile
        drone_configs: Drone configurations
        environment: Environment data
        existing_solution: Current solution

    Returns:
        CompilerResult with patches for each pipeline stage
    """
    compiler = ConstraintCompiler(
        drone_configs=drone_configs,
        environment=environment,
        existing_solution=existing_solution,
    )
    return compiler.compile(program)
