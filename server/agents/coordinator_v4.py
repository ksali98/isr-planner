# server/agents/coordinator_v4.py
"""
Coordinator V4 - Deterministic pre-pass for the v4 multi-agent workflow.

This module implements a pure, deterministic policy/guardrail layer that:
1. Classifies user intent (plan/replan/explain/what_if/debug)
2. Validates environment, matrix, and synthetic starts
3. Selects allocation strategy and solver mode
4. Injects policy decisions into MissionState before graph invocation

The Coordinator does NOT wrap the workflow - it runs BEFORE workflow.invoke()
and injects values that strategist/allocator/route_optimizer nodes honor.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

# Intent classification tokens
INTENTS = ("plan", "replan", "explain", "what_if", "debug", "unknown")

DEBUG_TOKENS = (
    "error", "traceback", "crash", "undefined", "not defined",
    "bug", "wrong", "mismatch", "failed", "failing", "broken"
)
EXPLAIN_TOKENS = (
    "explain", "why", "how", "meaning", "walk me through",
    "what does", "tell me about", "describe"
)
WHATIF_TOKENS = (
    "what if", "compare", "instead", "different", "try strategy",
    "change fuel", "allocation strategy", "hypothetically"
)
REPLAN_TOKENS = (
    "cut", "checkpoint", "continue", "replan", "resume", "segment",
    "from current", "remaining", "_START"
)
PLAN_TOKENS = (
    "plan", "optimize", "solve", "compute", "generate", "create mission",
    "allocate", "route"
)

# Allocation modification tokens - user wants to change existing allocation
ALLOCATION_MOD_TOKENS = (
    "move", "reassign", "transfer", "shift", "switch",
    "give", "take", "from d", "to d", "from drone", "to drone",
)

# Include/force visit tokens - user wants specific targets included
INCLUDE_TOKENS = (
    "include", "must visit", "must include", "force visit", "make sure",
    "need to visit", "have to visit", "should visit", "ensure", "require",
    "add t", "visit t", "cover t",
)

import re

def parse_allocation_modifications(user_message: str) -> List[Dict[str, Any]]:
    """
    Parse allocation modification requests from user message.

    Detects patterns like:
    - "move T5 to D1"
    - "reassign T8 and T10 to drone 2"
    - "transfer T3 from D1 to D2"
    - "give T5, T6 to D1"

    Returns:
        List of modification dicts: [{"target": "T5", "to_drone": "1"}, ...]
    """
    modifications = []
    msg = user_message.upper()

    # Pattern 1: "move/reassign/transfer T<id> to D<id>/drone <id>"
    # Matches: "move T5 to D1", "reassign T8 to drone 2"
    pattern1 = r'(?:MOVE|REASSIGN|TRANSFER|SHIFT|GIVE)\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)\s+(?:TO|INTO)\s+(?:D|DRONE\s*)(\d+)'
    matches1 = re.findall(pattern1, msg)
    for targets_str, drone_id in matches1:
        # Extract all target IDs from the targets string
        target_ids = re.findall(r'T(\d+)', targets_str)
        for tid in target_ids:
            modifications.append({
                "target": f"T{tid}",
                "to_drone": drone_id,
            })

    # Pattern 2: "move T<id> from D<id> to D<id>"
    # Matches: "move T5 from D2 to D1"
    pattern2 = r'(?:MOVE|REASSIGN|TRANSFER)\s+(T\d+)\s+FROM\s+(?:D|DRONE\s*)(\d+)\s+TO\s+(?:D|DRONE\s*)(\d+)'
    matches2 = re.findall(pattern2, msg)
    for target_id, from_drone, to_drone in matches2:
        modifications.append({
            "target": target_id,
            "from_drone": from_drone,
            "to_drone": to_drone,
        })

    # Pattern 3: Simple pattern "T<id> to D<id>" anywhere in message
    # Matches: "put T5 to D1", "T5 should go to D1"
    pattern3 = r'(T\d+)\s+(?:TO|SHOULD\s+GO\s+TO|GOES\s+TO)\s+(?:D|DRONE\s*)(\d+)'
    matches3 = re.findall(pattern3, msg)
    for target_id, drone_id in matches3:
        # Avoid duplicates
        existing = [m for m in modifications if m.get("target") == target_id]
        if not existing:
            modifications.append({
                "target": target_id,
                "to_drone": drone_id,
            })

    return modifications


def parse_required_targets(user_message: str) -> List[str]:
    """
    Parse target inclusion requests from user message.

    Detects patterns like:
    - "include T2 and T3"
    - "must visit T5, T6, T7"
    - "make sure T2 is visited"
    - "you have to include T2 and T3 in your solution"
    - "ensure T8 and T10 are covered"

    Returns:
        List of target IDs: ["T2", "T3", "T5"]
    """
    required = []
    msg = user_message.upper()

    # Pattern 1: "include/must visit/ensure T<id> [and T<id>...]"
    # Matches: "include T2 and T3", "must visit T5, T6"
    pattern1 = r'(?:INCLUDE|MUST\s+VISIT|MUST\s+INCLUDE|ENSURE|MAKE\s+SURE|NEED\s+TO\s+VISIT|HAVE\s+TO\s+(?:VISIT|INCLUDE)|SHOULD\s+VISIT|REQUIRE|FORCE\s+VISIT|ADD|COVER)\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)'
    matches1 = re.findall(pattern1, msg)
    for targets_str in matches1:
        target_ids = re.findall(r'T(\d+)', targets_str)
        for tid in target_ids:
            target_id = f"T{tid}"
            if target_id not in required:
                required.append(target_id)

    # Pattern 2: "T<id> [and T<id>...] must be visited/included"
    # Matches: "T2 and T3 must be visited", "T5, T6 should be included"
    pattern2 = r'(T\d+(?:\s*(?:,|AND)\s*T\d+)*)\s+(?:MUST|SHOULD|HAVE\s+TO|NEED\s+TO)\s+BE\s+(?:VISITED|INCLUDED|COVERED|IN)'
    matches2 = re.findall(pattern2, msg)
    for targets_str in matches2:
        target_ids = re.findall(r'T(\d+)', targets_str)
        for tid in target_ids:
            target_id = f"T{tid}"
            if target_id not in required:
                required.append(target_id)

    # Pattern 3: "visit T<id>" or "cover T<id>" anywhere in message
    # Matches: "please visit T2", "make sure to cover T5"
    pattern3 = r'(?:VISIT|COVER|HIT)\s+(T\d+)'
    matches3 = re.findall(pattern3, msg)
    for tid in matches3:
        if tid not in required:
            required.append(tid)

    # Pattern 4: General "T<id> in your/the solution"
    # Matches: "include T2 in your solution", "T3 in the mission"
    pattern4 = r'(T\d+)\s+IN\s+(?:YOUR|THE|THIS)\s+(?:SOLUTION|MISSION|PLAN|ROUTE)'
    matches4 = re.findall(pattern4, msg)
    for tid in matches4:
        if tid not in required:
            required.append(tid)

    return required


@dataclass
class CoordinatorDecision:
    """
    Output of the Coordinator pre-pass.

    Canonical decision structure (frozen contract):
    - intent: plan | replan | reallocate | optimize | explain | what_if | debug
    - policy: solver/allocation control flags
    - constraints: moves, required/forbidden targets, fixed endpoints
    - explanation_only: if True, skip solver entirely
    - warnings/errors: validation results
    - trace_events: structured events for Agents Monitor tab

    Do NOT add new top-level keys. Extend under constraints or policy.
    """
    # Core decision
    intent: str  # plan | replan | reallocate | optimize | explain | what_if | debug
    confidence: float
    rules_hit: List[str]

    # Policy controls what downstream agents do
    policy: Dict[str, Any]
    # Expected keys:
    #   allow_solver: bool
    #   force_allocation: bool
    #   allocation_strategy: str | None  ("greedy"|"balanced"|"efficient"|"geographic"|"exclusive")
    #   post_opt: { crossing_removal, trajectory_swap, insert_unvisited }

    # Constraints extracted from user message
    constraints: Dict[str, Any] = field(default_factory=dict)
    # Expected keys:
    #   moves: List[{target, from_drone?, to_drone}]
    #   required_targets: List[str]
    #   forbidden_targets: List[str]
    #   fixed_endpoints: Dict[str, str]  # drone_id -> airport_id

    # Explanation-only mode (skip solver)
    explanation_only: bool = False

    # Validation results
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    # Structured trace events for Agents Monitor tab
    trace_events: List[Dict[str, Any]] = field(default_factory=list)

    # --- Legacy/internal fields (kept for backward compatibility) ---
    guardrails: Dict[str, Any] = field(default_factory=dict)
    drone_contracts: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    synthetic_starts: Dict[str, Dict[str, float]] = field(default_factory=dict)
    trace: Dict[str, Any] = field(default_factory=dict)

    @property
    def is_valid(self) -> bool:
        return len(self.errors) == 0

    def to_dict(self) -> Dict[str, Any]:
        """Export as canonical decision dict (for API/logging)."""
        return {
            "intent": self.intent,
            "policy": self.policy,
            "constraints": self.constraints,
            "explanation_only": self.explanation_only,
            "warnings": self.warnings,
            "errors": self.errors,
            "trace_events": self.trace_events,
        }


class CoordinatorV4:
    """
    Deterministic Coordinator for v4 multi-agent workflow.

    Usage:
        coordinator = CoordinatorV4()
        decision = coordinator.decide(
            user_message=user_message,
            environment=env,
            drone_configs=configs,
            sam_matrix=sam_matrix,
            ui_state=ui_state,
            preferences=preferences,
        )

        if not decision.is_valid:
            return error_response(decision.errors)

        # Inject into initial_state
        initial_state["intent"] = decision.intent
        initial_state["policy"] = decision.policy
        initial_state["guardrails"] = decision.guardrails
    """

    def __init__(self, debug: bool = False):
        self.debug = debug

    def _te(
        self,
        trace_events: List[Dict[str, Any]],
        stage: str,
        event: str,
        **details
    ) -> None:
        """
        Append a structured trace event.

        Args:
            trace_events: The list to append to
            stage: Pipeline stage (input, intent, constraints, policy, validate, final)
            event: Event name (e.g., "received", "intent_classified", "policy_selected")
            **details: Additional key-value pairs for the event
        """
        trace_events.append({
            "agent": "CoordinatorV4",
            "stage": stage,
            "event": event,
            "details": details or {},
        })

    def classify_intent(
        self,
        user_message: str,
        drone_configs: Dict[str, Any],
    ) -> Tuple[str, float, List[str]]:
        """
        Classify user intent based on message tokens and UI state.

        Returns:
            (intent, confidence, rules_hit)
        """
        msg_lower = user_message.lower()
        rules_hit: List[str] = []

        # Check for synthetic starts in configs (indicates replan)
        has_synthetic_start = any(
            cfg.get("start_airport", "").endswith("_START")
            for cfg in drone_configs.values()
            if isinstance(cfg, dict)
        )

        if has_synthetic_start:
            rules_hit.append("synthetic_start_detected")
            return ("replan", 0.95, rules_hit)

        # Token-based classification
        if any(tok in msg_lower for tok in DEBUG_TOKENS):
            rules_hit.append("debug_token_match")
            return ("debug", 0.85, rules_hit)

        if any(tok in msg_lower for tok in EXPLAIN_TOKENS):
            rules_hit.append("explain_token_match")
            return ("explain", 0.85, rules_hit)

        if any(tok in msg_lower for tok in WHATIF_TOKENS):
            rules_hit.append("whatif_token_match")
            return ("what_if", 0.80, rules_hit)

        if any(tok in msg_lower for tok in REPLAN_TOKENS):
            rules_hit.append("replan_token_match")
            return ("replan", 0.80, rules_hit)

        # Check for allocation modification requests (more specific than general plan)
        # This detects "move T5 to D1", "reassign T8 to drone 2", etc.
        if any(tok in msg_lower for tok in ALLOCATION_MOD_TOKENS):
            # Parse to verify it's actually an allocation modification
            mods = parse_allocation_modifications(user_message)
            if mods:
                rules_hit.append("allocation_modification_detected")
                rules_hit.append(f"modifications:{len(mods)}")
                return ("reallocate", 0.90, rules_hit)

        if any(tok in msg_lower for tok in PLAN_TOKENS):
            rules_hit.append("plan_token_match")
            return ("plan", 0.75, rules_hit)

        # Default to plan if no specific intent detected
        rules_hit.append("default_to_plan")
        return ("plan", 0.50, rules_hit)

    def validate_environment(
        self,
        environment: Dict[str, Any],
        drone_configs: Dict[str, Any],
        sam_matrix: Optional[Dict[str, Any]],
    ) -> Tuple[Dict[str, Any], List[str]]:
        """
        Validate environment, matrix labels, and synthetic starts.

        Returns:
            (guardrails_dict, errors_list)
        """
        errors: List[str] = []
        guardrails: Dict[str, Any] = {
            "env_valid": True,
            "matrix_valid": True,
            "synthetic_starts_valid": True,
            "checks": [],
        }

        airports = environment.get("airports", [])
        targets = environment.get("targets", [])

        # Basic environment validation
        if not airports:
            errors.append("No airports in environment")
            guardrails["env_valid"] = False

        if not targets:
            errors.append("No targets in environment")
            guardrails["env_valid"] = False

        # Build ID sets
        airport_ids: Set[str] = {a.get("id", a.get("label", "")) for a in airports}
        target_ids: Set[str] = {t.get("id", t.get("label", "")) for t in targets}

        guardrails["airport_ids"] = sorted(airport_ids)
        guardrails["target_ids"] = sorted(target_ids)

        # Matrix validation
        if sam_matrix:
            matrix_labels = sam_matrix.get("labels", [])
            guardrails["matrix_labels"] = matrix_labels

            # Check all airports and targets are in matrix
            missing_in_matrix = []
            for aid in airport_ids:
                if aid and aid not in matrix_labels:
                    missing_in_matrix.append(f"airport:{aid}")
            for tid in target_ids:
                if tid and tid not in matrix_labels:
                    missing_in_matrix.append(f"target:{tid}")

            if missing_in_matrix:
                guardrails["checks"].append(f"Missing from matrix: {missing_in_matrix}")
                # Not necessarily an error - could be excluded targets

        # Synthetic start validation
        synthetic_starts: Dict[str, Dict[str, float]] = {}
        for did, cfg in drone_configs.items():
            if not isinstance(cfg, dict):
                continue

            start_airport = cfg.get("start_airport", "")
            if start_airport and start_airport.endswith("_START"):
                # This is a synthetic start - verify it's in matrix
                if sam_matrix:
                    matrix_labels = sam_matrix.get("labels", [])
                    if start_airport not in matrix_labels:
                        errors.append(
                            f"D{did} synthetic start '{start_airport}' not in matrix_labels"
                        )
                        guardrails["synthetic_starts_valid"] = False

                # Try to find coordinates
                waypoints = sam_matrix.get("waypoints", []) if sam_matrix else []
                for wp in waypoints:
                    if wp.get("id") == start_airport:
                        synthetic_starts[start_airport] = {
                            "x": wp.get("x", 0),
                            "y": wp.get("y", 0),
                        }
                        break

        guardrails["synthetic_starts"] = synthetic_starts
        guardrails["checks"].append(f"Found {len(synthetic_starts)} synthetic starts")

        return guardrails, errors

    def _get_real_airport_ids(self, environment: Dict[str, Any]) -> Set[str]:
        """Get set of real airport IDs (excluding synthetic starts)."""
        real_ids: Set[str] = set()
        for a in environment.get("airports", []):
            if not a.get("is_synthetic", False):
                aid = a.get("id", a.get("label", ""))
                if aid:
                    real_ids.add(str(aid))
        return real_ids

    def select_policy(
        self,
        intent: str,
        drone_configs: Dict[str, Any],
        environment: Dict[str, Any],
        preferences: Optional[Dict[str, Any]] = None,
        ui_state: Optional[Dict[str, Any]] = None,
        user_message: str = "",
    ) -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]]]:
        """
        Select allocation strategy, solver mode, and build drone contracts.

        Returns:
            (policy dict, drone_contracts dict)
        """
        prefs = preferences or {}

        # Parse allocation modifications if this is a reallocate intent
        allocation_modifications = []
        if intent == "reallocate" and user_message:
            allocation_modifications = parse_allocation_modifications(user_message)

        # Base policy
        policy: Dict[str, Any] = {
            # Allocation control
            "allocation_strategy": prefs.get("allocation_strategy", "geographic"),
            "force_algorithmic_allocation": True,  # v4 agentic: tool-driven allocation

            # Solver control
            "solver_mode": prefs.get("solver_mode", None),  # None = infer per drone
            "tie_break": prefs.get("tie_break", "shorter_valid"),

            # Post-optimization toggles
            "post_opt": {
                "crossing_removal": prefs.get("crossing_removal", True),
                "trajectory_swap": prefs.get("trajectory_swap", True),
                "insert_unvisited": prefs.get("insert_unvisited", True),
            },

            # Intent-driven flags
            "explain_only": False,
            "skip_allocation": False,
            "skip_solver": False,
        }

        # Adjust based on intent
        if intent in ("explain", "debug"):
            policy["explain_only"] = True
            policy["skip_allocation"] = True
            policy["skip_solver"] = True

        if intent == "what_if":
            # What-if scenarios still run allocation/solver but may use different strategy
            policy["force_algorithmic_allocation"] = True
            policy["what_if_lightweight"] = True

        if intent == "reallocate" and allocation_modifications:
            # User wants to modify existing allocation, not compute new one
            # Tell the Allocator to use existing allocation and apply modifications
            policy["force_algorithmic_allocation"] = False  # Don't overwrite with algo
            policy["use_existing_allocation"] = True
            policy["allocation_modifications"] = allocation_modifications
            policy["skip_allocation"] = False  # Still run allocator to apply mods
            policy["skip_solver"] = False  # Re-route with new allocation

        # Get real airport IDs for home_airport fallback
        real_airport_ids = self._get_real_airport_ids(environment)
        default_real_airport = next(iter(sorted(real_airport_ids)), "A1")

        # Check for checkpoint_id from UI state (for replan scenarios)
        cut_state = (ui_state or {}).get("cut", {}) or {}
        checkpoint_id = cut_state.get("checkpoint_id") if intent == "replan" else None

        # Build per-drone start contracts
        drone_contracts: Dict[str, Dict[str, Any]] = {}
        for did, cfg in drone_configs.items():
            if not isinstance(cfg, dict):
                continue

            enabled = cfg.get("enabled", did == "1")

            cfg_start = cfg.get("start_airport", "") or cfg.get("start_id", "")
            cfg_home = cfg.get("home_airport", "")
            cfg_end = cfg.get("end_airport", "")

            # start_id determination:
            # 1. If checkpoint_id exists and intent is replan, use checkpoint_id
            # 2. Else use cfg_start if set
            # 3. Else use cfg_home
            # 4. Else use default real airport
            if checkpoint_id:
                start_id = str(checkpoint_id)
            elif cfg_start:
                start_id = str(cfg_start)
            elif cfg_home:
                start_id = str(cfg_home)
            else:
                start_id = default_real_airport

            # Determine if this is a synthetic start
            is_synthetic = start_id.endswith("_START")

            # home_airport MUST be a real airport (never synthetic)
            # Used for endpoint candidates in best_end mode
            if cfg_home and cfg_home in real_airport_ids:
                home_airport = str(cfg_home)
            elif start_id in real_airport_ids:
                home_airport = start_id
            else:
                home_airport = default_real_airport

            # end_airport handling
            flexible_endpoint = (cfg_end == "-")
            end_airport = cfg_end if cfg_end and cfg_end != "-" else None

            drone_contracts[str(did)] = {
                "enabled": bool(enabled),
                "start_id": start_id,
                "home_airport": home_airport,
                "end_airport": end_airport,
                "is_synthetic_start": is_synthetic,
                "flexible_endpoint": flexible_endpoint,
            }

        # Also store drone_contracts in policy for backwards compatibility
        policy["drone_contracts"] = drone_contracts

        return policy, drone_contracts

    def decide(
        self,
        user_message: str,
        environment: Dict[str, Any],
        drone_configs: Dict[str, Any],
        sam_matrix: Optional[Dict[str, Any]] = None,
        ui_state: Optional[Dict[str, Any]] = None,
        preferences: Optional[Dict[str, Any]] = None,
    ) -> CoordinatorDecision:
        """
        Main entry point: classify intent, validate, select policy.

        Returns:
            CoordinatorDecision with canonical structure (frozen contract).
        """
        # Initialize trace_events list and warnings
        trace_events: List[Dict[str, Any]] = []
        warnings: List[str] = []

        # ===================================================================
        # EVENT 1: Input Snapshot (sanitized)
        # ===================================================================
        enabled_drone_ids = [
            d for d, cfg in (drone_configs or {}).items()
            if isinstance(cfg, dict) and cfg.get("enabled")
        ]
        self._te(trace_events, "input", "received",
                 message=user_message[:400],
                 num_targets=len(environment.get("targets", [])),
                 num_sams=len(environment.get("sams", [])),
                 drones_enabled=enabled_drone_ids)

        # ===================================================================
        # EVENT 2: Intent Classification
        # ===================================================================
        intent, confidence, rules_hit = self.classify_intent(
            user_message, drone_configs
        )
        self._te(trace_events, "intent", "intent_classified",
                 intent=intent,
                 confidence=confidence,
                 rules_hit=rules_hit)

        # ===================================================================
        # EVENT 3: Constraint Extraction
        # ===================================================================
        # Extract moves from user message (e.g., "move T5 from D2 to D1")
        moves = parse_allocation_modifications(user_message) if user_message else []

        # Extract required targets from user message (e.g., "include T2 and T3")
        required_targets = parse_required_targets(user_message) if user_message else []

        # Build constraints dict (canonical structure)
        constraints: Dict[str, Any] = {
            "moves": moves,
            "required_targets": required_targets,
            "forbidden_targets": [],  # TODO: parse from user message if needed
            "fixed_endpoints": {},    # Populated from drone_contracts below
        }
        self._te(trace_events, "constraints", "constraints_extracted",
                 moves=moves,
                 num_moves=len(moves),
                 required_targets=required_targets,
                 num_required=len(required_targets))

        # ===================================================================
        # EVENT 4: Policy Selection
        # ===================================================================
        # Get raw policy and drone contracts from select_policy
        raw_policy, drone_contracts = self.select_policy(
            intent, drone_configs, environment, preferences, ui_state, user_message
        )

        # Build canonical policy structure
        policy: Dict[str, Any] = {
            "allow_solver": not raw_policy.get("skip_solver", False),
            "force_allocation": raw_policy.get("force_algorithmic_allocation", True),
            "allocation_strategy": raw_policy.get("allocation_strategy"),
            "post_opt": raw_policy.get("post_opt", {
                "crossing_removal": True,
                "trajectory_swap": True,
                "insert_unvisited": True,
            }),
            # Keep legacy keys for backward compatibility
            "solver_mode": raw_policy.get("solver_mode"),
            "skip_allocation": raw_policy.get("skip_allocation", False),
            "skip_solver": raw_policy.get("skip_solver", False),
            "use_existing_allocation": raw_policy.get("use_existing_allocation", False),
            "allocation_modifications": raw_policy.get("allocation_modifications", []),
            "drone_contracts": drone_contracts,
        }

        # Populate fixed_endpoints from drone_contracts
        for did, contract in drone_contracts.items():
            if contract.get("end_airport"):
                constraints["fixed_endpoints"][did] = contract["end_airport"]

        self._te(trace_events, "policy", "policy_selected",
                 allow_solver=policy["allow_solver"],
                 force_allocation=policy["force_allocation"],
                 allocation_strategy=policy["allocation_strategy"],
                 post_opt=policy["post_opt"])

        # ===================================================================
        # EVENT 5: Validation
        # ===================================================================
        guardrails, errors = self.validate_environment(
            environment, drone_configs, sam_matrix
        )

        # Add any guardrail issues as warnings
        if not guardrails.get("env_valid", True):
            warnings.append("Environment validation failed")
        if not guardrails.get("matrix_valid", True):
            warnings.append("SAM matrix validation failed")

        self._te(trace_events, "validate", "validated",
                 env_valid=guardrails.get("env_valid", False),
                 matrix_valid=guardrails.get("matrix_valid", False),
                 warnings=warnings,
                 errors=errors)

        # If errors, block solver
        explanation_only = raw_policy.get("explain_only", False)
        if errors:
            policy["allow_solver"] = False
            explanation_only = True
            self._te(trace_events, "policy", "solver_blocked_due_to_errors",
                     errors=errors)

        # ===================================================================
        # EVENT 6: Final Summary
        # ===================================================================
        synthetic_starts = guardrails.get("synthetic_starts", {})
        guardrails["synthetic_start_ids"] = list(synthetic_starts.keys())

        self._te(trace_events, "final", "decision_ready",
                 intent=intent,
                 allow_solver=policy["allow_solver"],
                 force_allocation=policy["force_allocation"],
                 allocation_strategy=policy["allocation_strategy"],
                 num_moves=len(moves),
                 num_errors=len(errors),
                 num_warnings=len(warnings))

        # Build legacy trace for backward compatibility
        trace = {
            "coordinator": {
                "version": "v4.3",
                "intent": intent,
                "confidence": confidence,
                "rules_hit": rules_hit,
                "policy": policy,
                "constraints": constraints,
                "guardrails_summary": {
                    "env_valid": guardrails.get("env_valid", False),
                    "matrix_valid": guardrails.get("matrix_valid", False),
                    "synthetic_starts": list(synthetic_starts.keys()),
                },
            }
        }

        if self.debug:
            print(f"[Coordinator] Intent: {intent} (confidence: {confidence:.2f})")
            print(f"[Coordinator] Rules hit: {rules_hit}")
            print(f"[Coordinator] Policy: {policy}")
            print(f"[Coordinator] Constraints: {constraints}")
            print(f"[Coordinator] Drone contracts: {drone_contracts}")
            print(f"[Coordinator] Errors: {errors}")
            print(f"[Coordinator] Warnings: {warnings}")

        return CoordinatorDecision(
            intent=intent,
            confidence=confidence,
            rules_hit=rules_hit,
            policy=policy,
            constraints=constraints,
            explanation_only=explanation_only,
            warnings=warnings,
            errors=errors,
            trace_events=trace_events,
            # Legacy fields
            guardrails=guardrails,
            drone_contracts=drone_contracts,
            synthetic_starts=synthetic_starts,
            trace=trace,
        )


# Convenience function for direct use
def run_coordinator(
    user_message: str,
    environment: Dict[str, Any],
    drone_configs: Dict[str, Any],
    sam_matrix: Optional[Dict[str, Any]] = None,
    ui_state: Optional[Dict[str, Any]] = None,
    preferences: Optional[Dict[str, Any]] = None,
    debug: bool = False,
) -> CoordinatorDecision:
    """
    Run the Coordinator pre-pass.

    Usage in run_multi_agent_v4():
        decision = run_coordinator(user_message, env, configs, sam_matrix)
        if not decision.is_valid:
            return {"error": decision.errors, ...}

        initial_state["intent"] = decision.intent
        initial_state["policy"] = decision.policy
        initial_state["guardrails"] = decision.guardrails
    """
    coordinator = CoordinatorV4(debug=debug)
    return coordinator.decide(
        user_message=user_message,
        environment=environment,
        drone_configs=drone_configs,
        sam_matrix=sam_matrix,
        ui_state=ui_state,
        preferences=preferences,
    )
