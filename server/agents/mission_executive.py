"""
Mission Executive Module

Persistent controller that manages the observe-decide-act loop for ISR missions.
"""

from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime

from ..database.mission_ledger import create_mission_run, log_event


class ExecutiveAction(str, Enum):
    """Actions the Executive can return to the UI."""
    CONTINUE = "CONTINUE"
    PAUSE = "PAUSE"
    CUT_AND_FREEZE = "CUT_AND_FREEZE"
    REPLAN_REMAINDER = "REPLAN_REMAINDER"
    DRAFT_READY = "DRAFT_READY"
    COMMIT_PLAN = "COMMIT_PLAN"
    REQUEST_APPROVAL = "REQUEST_APPROVAL"
    RESET = "RESET"


class MissionMode(str, Enum):
    """Current mission execution mode."""
    IDLE = "IDLE"
    READY = "READY"
    ANIMATING = "ANIMATING"
    PAUSED = "PAUSED"
    CHECKPOINT = "CHECKPOINT"
    DRAFT_READY = "DRAFT_READY"
    NEEDS_REPLAN = "NEEDS_REPLAN"


@dataclass
class DroneState:
    """Current state of a single drone."""
    drone_id: str
    position: List[float]  # [x, y]
    progress: float  # 0.0 to 1.0
    fuel_remaining: float
    visited_targets: List[str] = field(default_factory=list)


@dataclass
class TickRequest:
    """Incoming tick request from UI."""
    mission_id: Optional[str]
    ui_state: Dict[str, Any]  # {phase, progress_by_drone, positions_by_drone}
    events: List[Dict[str, Any]]  # [{type, ...}]
    env: Optional[Dict[str, Any]] = None
    drone_configs: Optional[Dict[str, Any]] = None


@dataclass
class TickResponse:
    """Response from Executive to UI."""
    action: ExecutiveAction
    mission_id: str
    draft_plan: Optional[Dict[str, Any]] = None
    joined_plan: Optional[Dict[str, Any]] = None
    visited_targets: List[str] = field(default_factory=list)
    markers: Dict[str, List[str]] = field(default_factory=dict)  # {green: [], red: []}
    message: str = ""


class MissionExecutive:
    """
    Mission Executive - adaptive controller for ISR missions.

    Responsibilities:
    - Observe current mission state
    - Detect events (human commands, env edits, checkpoints)
    - Decide next action using policy rules
    - Act by calling skills (planner, optimizer, critic)
    - Log everything to Supabase
    """

    def __init__(self):
        self._mission_states: Dict[str, Dict[str, Any]] = {}

    def tick(self, request: TickRequest) -> TickResponse:
        """
        Main entry point - process one tick of the executive loop.
        """
        # 1. Ensure mission_id exists
        mission_id = request.mission_id
        if not mission_id:
            mission_id = create_mission_run(
                system_version="executive-v1",
                mission_name="executive-mission"
            )
            if not mission_id:
                # Supabase not configured - use in-memory ID
                from uuid import uuid4
                mission_id = f"local_{uuid4().hex[:8]}"

        # 2. Log tick received
        log_event(mission_id, "EXEC_TICK_RECEIVED", {
            "ui_phase": request.ui_state.get("phase"),
            "events": [e.get("type") for e in request.events],
            "timestamp": datetime.utcnow().isoformat()
        })

        # 3. Process events and decide action
        action, message = self._decide_action(mission_id, request)

        # 4. Log decision
        log_event(mission_id, "EXEC_DECISION", {
            "action": action.value,
            "message": message,
            "timestamp": datetime.utcnow().isoformat()
        })

        # 5. Return response
        return TickResponse(
            action=action,
            mission_id=mission_id,
            message=message
        )

    def _decide_action(
        self,
        mission_id: str,
        request: TickRequest
    ) -> Tuple[ExecutiveAction, str]:
        """
        V1 Decision Policy - deterministic rules.
        """
        # Check for explicit human commands in events
        for event in request.events:
            event_type = event.get("type", "")

            if event_type == "HUMAN_COMMAND":
                command = event.get("command", "")
                if command == "CUT":
                    return ExecutiveAction.CUT_AND_FREEZE, "Cut command received"
                elif command == "PAUSE":
                    return ExecutiveAction.PAUSE, "Pause command received"
                elif command == "RESET":
                    return ExecutiveAction.RESET, "Reset command received"
                elif command == "ACCEPT":
                    return ExecutiveAction.COMMIT_PLAN, "Accept command received"
                elif command == "REJECT":
                    return ExecutiveAction.CONTINUE, "Reject - keeping current plan"

            elif event_type == "ENV_EDITS":
                return ExecutiveAction.REPLAN_REMAINDER, "Environment edited - replan needed"

        # Default: continue with current plan
        return ExecutiveAction.CONTINUE, "No action required"


# Singleton instance
_executive: Optional[MissionExecutive] = None


def get_executive() -> MissionExecutive:
    """Get or create the singleton executive instance."""
    global _executive
    if _executive is None:
        _executive = MissionExecutive()
    return _executive
