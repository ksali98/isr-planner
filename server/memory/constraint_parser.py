"""
LLM Constraint Parser

Parses natural language constraint requests into ConstraintProgram objects.
Uses a small LLM to extract:
1. Allocation operations (FORCE_VISIT, MOVE, REMOVE, SWAP, INSERT)
2. Sequencing hints for Strategist interpretation

The parser is designed to be called by the Coordinator or Strategist agent
when processing user requests that modify mission constraints.
"""

import json
import re
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from .constraints import (
    ConstraintProgram,
    ForceVisitConstraint,
    MoveConstraint,
    RemoveConstraint,
    SwapConstraint,
    InsertConstraint,
)


# =============================================================================
# Parser Prompt Templates
# =============================================================================

CONSTRAINT_EXTRACTION_PROMPT = """You are a constraint parser for an ISR (Intelligence, Surveillance, Reconnaissance) mission planning system.

Given a user's natural language request, extract:
1. **Allocation operations** - specific changes to target-drone assignments
2. **Sequencing hints** - guidance about visit order, priority, or timing

## Allocation Operations

Extract these operations when the user explicitly requests them:

- **FORCE_VISIT**: User wants specific targets to be visited
  Example: "Make sure we visit T5 and T8"

- **MOVE**: User wants to assign a target to a specific drone
  Example: "Move T5 to D1", "Assign T8 to drone 2"

- **REMOVE**: User wants to exclude a target from the mission
  Example: "Skip T3", "Don't visit T5"

- **SWAP**: User wants to exchange targets between drones
  Example: "Swap T5 and T8 between their drones"

- **INSERT**: User wants to add an unvisited target to a route
  Example: "Add T9 to the mission", "Insert T10 into D1's route"

## Sequencing Hints

Extract these hints when the user specifies visit order preferences:

- **start_with**: User wants to begin with specific targets or priority level
  Example: "Start with priority 10 targets", "Visit T5 first"

- **end_with**: User wants to end with specific targets before returning
  Example: "Make T5 and T8 the last targets before landing"

- **priority_order**: User wants targets visited in priority order
  Example: "Visit high priority targets first, then complete the mission"

- **segment_by_priority**: User wants mission split by priority threshold
  Example: "Complete all priority > 7 targets first, then handle the rest"

## Environment Context

Available targets: {target_ids}
Available drones: {drone_ids}
Target priorities: {target_priorities}

## User Request

{user_message}

## Output Format

Respond with a JSON object:
```json
{{
  "allocation_ops": [
    {{"op": "FORCE_VISIT", "targets": ["T5", "T8"]}},
    {{"op": "MOVE", "target": "T5", "to_drone": "1"}},
    {{"op": "REMOVE", "target": "T3", "reason": "user requested skip"}},
    {{"op": "SWAP", "target_a": "T5", "target_b": "T8"}},
    {{"op": "INSERT", "target": "T9", "drone_id": "1"}}
  ],
  "sequencing_hints": {{
    "start_with": {{"targets": ["T5"]}} or {{"priority_gte": 10}},
    "end_with": {{"targets": ["T5", "T8"]}},
    "priority_order": "high_first" or "low_first",
    "segment_by_priority": {{"threshold": 7, "order": "high_first"}}
  }},
  "confidence": 0.95,
  "ambiguities": ["unclear which drone for T9"]
}}
```

Only include operations/hints that are clearly requested. If nothing is requested, return empty arrays/objects.
If there are ambiguities that would benefit from clarification, list them."""


# =============================================================================
# Regex-based fallback parser
# =============================================================================

def _parse_target_ids(text: str) -> List[str]:
    """Extract target IDs (T1, T2, etc.) from text."""
    return re.findall(r'T\d+', text.upper())


def _parse_drone_ids(text: str) -> List[str]:
    """Extract drone IDs from text, normalizing to just the number."""
    # Match "D1", "D2", "drone 1", "drone 2", etc.
    matches = re.findall(r'(?:D|DRONE\s*)(\d+)', text.upper())
    return matches


def _parse_priority_threshold(text: str) -> Optional[int]:
    """Extract priority threshold from text like 'priority > 7' or 'priority >= 8'."""
    match = re.search(r'PRIORITY\s*[>>=]+\s*(\d+)', text.upper())
    if match:
        return int(match.group(1))
    return None


def regex_fallback_parse(
    user_message: str,
    target_ids: List[str],
    drone_ids: List[str],
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """
    Regex-based fallback parser for when LLM is unavailable.

    Returns:
        (allocation_ops, sequencing_hints)
    """
    msg = user_message.upper()
    allocation_ops: List[Dict[str, Any]] = []
    sequencing_hints: Dict[str, Any] = {}

    # --- Allocation operations ---

    # MOVE: "move T5 to D1", "assign T8 to drone 2"
    move_pattern = r'(?:MOVE|ASSIGN|REASSIGN|TRANSFER|GIVE)\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)\s+(?:TO|INTO)\s+(?:D|DRONE\s*)(\d+)'
    for match in re.finditer(move_pattern, msg):
        targets_str, drone_id = match.groups()
        for tid in _parse_target_ids(targets_str):
            allocation_ops.append({
                "op": "MOVE",
                "target": tid,
                "to_drone": drone_id,
            })

    # REMOVE: "skip T3", "don't visit T5", "exclude T8"
    remove_pattern = r'(?:SKIP|DON\'?T\s+VISIT|EXCLUDE|REMOVE|FORBID)\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)'
    for match in re.finditer(remove_pattern, msg):
        targets_str = match.group(1)
        for tid in _parse_target_ids(targets_str):
            allocation_ops.append({
                "op": "REMOVE",
                "target": tid,
                "reason": "user requested exclusion",
            })

    # FORCE_VISIT: "make sure we visit T5", "must visit T8", "visit T5 and T8"
    force_pattern = r'(?:MAKE\s+SURE|MUST|ENSURE|FORCE)\s+(?:WE\s+)?VISIT\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)'
    for match in re.finditer(force_pattern, msg):
        targets_str = match.group(1)
        targets = _parse_target_ids(targets_str)
        if targets:
            allocation_ops.append({
                "op": "FORCE_VISIT",
                "targets": targets,
            })

    # SWAP: "swap T5 and T8"
    swap_pattern = r'SWAP\s+(T\d+)\s+(?:AND|WITH)\s+(T\d+)'
    for match in re.finditer(swap_pattern, msg):
        target_a, target_b = match.groups()
        allocation_ops.append({
            "op": "SWAP",
            "target_a": target_a,
            "target_b": target_b,
        })

    # INSERT: "add T9 to the mission", "insert T10"
    insert_pattern = r'(?:ADD|INSERT)\s+(T\d+)(?:\s+(?:TO|INTO)\s+(?:D|DRONE\s*)(\d+))?'
    for match in re.finditer(insert_pattern, msg):
        target = match.group(1)
        drone_id = match.group(2)  # May be None
        op = {"op": "INSERT", "target": target}
        if drone_id:
            op["drone_id"] = drone_id
        allocation_ops.append(op)

    # --- Sequencing hints ---

    # Priority threshold segmentation: "priority > 7 first", "targets with priority > 7 first"
    priority_threshold = _parse_priority_threshold(user_message)
    if priority_threshold and "FIRST" in msg:
        sequencing_hints["segment_by_priority"] = {
            "threshold": priority_threshold,
            "order": "high_first",
        }
    # General priority ordering: "high priority first" (without specific threshold)
    elif "HIGH" in msg and "PRIORITY" in msg and "FIRST" in msg:
        sequencing_hints["priority_order"] = "high_first"

    # Start with: "start with T5", "begin with priority 10 targets"
    # Pattern for targets: "start with T5"
    start_target_pattern = r'(?:START|BEGIN)\s+WITH\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)'
    start_target_match = re.search(start_target_pattern, msg)
    if start_target_match:
        targets = _parse_target_ids(start_target_match.group(1))
        if targets:
            sequencing_hints["start_with"] = {"targets": targets}

    # Pattern for priority: "start with priority 10 targets"
    if "start_with" not in sequencing_hints:
        start_priority_pattern = r'(?:START|BEGIN)\s+WITH\s+(?:A\s+)?PRIORITY\s*(\d+)'
        start_priority_match = re.search(start_priority_pattern, msg)
        if start_priority_match:
            threshold = int(start_priority_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "visit priority X first" or "priority X target first"
    if "start_with" not in sequencing_hints:
        visit_priority_first_pattern = r'(?:VISIT|HIT|FLY\s+TO)?\s*(?:A\s+)?PRIORITY\s*(\d+)\s+(?:TARGET\s+)?FIRST'
        visit_first_match = re.search(visit_priority_first_pattern, msg)
        if visit_first_match:
            threshold = int(visit_first_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "every drone must visit a priority X target first"
    if "start_with" not in sequencing_hints:
        every_drone_pattern = r'(?:EVERY|EACH|ALL)\s+DRONE[S]?\s+(?:MUST|SHOULD|WILL|SHALL)\s+(?:FIRST\s+)?VISIT\s+(?:A\s+)?PRIORITY\s*(\d+)\s+(?:TARGET\s+)?(?:FIRST)?'
        every_drone_match = re.search(every_drone_pattern, msg)
        if every_drone_match:
            threshold = int(every_drone_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "first target must be priority X" or "first visit priority X"
    if "start_with" not in sequencing_hints:
        first_must_pattern = r'FIRST\s+(?:TARGET|VISIT|STOP)\s+(?:MUST\s+BE|SHOULD\s+BE|HAS\s+TO\s+BE)?\s*(?:A\s+)?PRIORITY\s*(\d+)'
        first_must_match = re.search(first_must_pattern, msg)
        if first_must_match:
            threshold = int(first_must_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "drones have to start by visiting a target of priority X"
    # Handles: "all drones have to start by directly visiting a target of priority 10"
    if "start_with" not in sequencing_hints:
        start_by_visiting_pattern = r'(?:DRONE[S]?\s+)?(?:HAVE\s+TO|NEED\s+TO|MUST|SHOULD)\s+START\s+(?:BY\s+)?(?:DIRECTLY\s+)?(?:VISIT(?:ING)?|FLY(?:ING)?\s+TO)\s+(?:A\s+)?TARGET\s+(?:OF\s+)?PRIORITY\s*(\d+)'
        start_by_match = re.search(start_by_visiting_pattern, msg)
        if start_by_match:
            threshold = int(start_by_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "target of priority X" anywhere with "first" or "start"
    # Handles: "a target of priority 10 first", "start with a target of priority 10"
    if "start_with" not in sequencing_hints:
        target_of_priority_pattern = r'(?:START|BEGIN|FIRST)\s+.*?TARGET\s+(?:OF\s+)?PRIORITY\s*(\d+)|TARGET\s+(?:OF\s+)?PRIORITY\s*(\d+)\s+.*?(?:FIRST|START)'
        target_of_match = re.search(target_of_priority_pattern, msg)
        if target_of_match:
            threshold = int(target_of_match.group(1) or target_of_match.group(2))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "priority X target" with "first" or "start" anywhere
    if "start_with" not in sequencing_hints:
        priority_x_first_pattern = r'PRIORITY\s*(\d+)\s+TARGET.*?(?:FIRST|START)|(?:FIRST|START).*?PRIORITY\s*(\d+)\s+TARGET'
        priority_first_match = re.search(priority_x_first_pattern, msg)
        if priority_first_match:
            threshold = int(priority_first_match.group(1) or priority_first_match.group(2))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "first target ... has to be ... priority X" or "first target ... be of priority X"
    # Handles: "the first target a drone visits has to be of priority 10"
    if "start_with" not in sequencing_hints:
        first_target_has_to_be_pattern = r'FIRST\s+TARGET\s+.*?(?:HAS\s+TO\s+BE|MUST\s+BE|SHOULD\s+BE|BE)\s+(?:OF\s+)?PRIORITY\s*(\d+)'
        first_target_match = re.search(first_target_has_to_be_pattern, msg)
        if first_target_match:
            threshold = int(first_target_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # Pattern for "visits has to be of priority X" or similar
    # Handles: "This time the first target a drone visits has to be of priority 10"
    if "start_with" not in sequencing_hints:
        visits_priority_pattern = r'(?:FIRST\s+TARGET|VISIT[S]?)\s+.*?(?:HAS\s+TO|MUST|SHOULD)\s+BE\s+(?:OF\s+)?PRIORITY\s*(\d+)'
        visits_match = re.search(visits_priority_pattern, msg)
        if visits_match:
            threshold = int(visits_match.group(1))
            sequencing_hints["start_with"] = {"priority_gte": threshold}

    # End with patterns - multiple approaches to catch various phrasings
    # Pattern 1: "make T5 and T8 the last targets"
    make_last_pattern = r'MAKE\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)\s+(?:THE\s+)?LAST'
    make_last_match = re.search(make_last_pattern, msg)
    if make_last_match:
        targets = _parse_target_ids(make_last_match.group(1))
        if targets:
            sequencing_hints["end_with"] = {"targets": targets}

    # Pattern 2: "T5 and T8 should be the last targets"
    if "end_with" not in sequencing_hints:
        should_last_pattern = r'(T\d+(?:\s*(?:,|AND)\s*T\d+)*)\s+(?:SHOULD\s+BE|ARE)\s+(?:THE\s+)?LAST'
        should_last_match = re.search(should_last_pattern, msg)
        if should_last_match:
            targets = _parse_target_ids(should_last_match.group(1))
            if targets:
                sequencing_hints["end_with"] = {"targets": targets}

    # Pattern 3: "end with T5 and T8"
    if "end_with" not in sequencing_hints:
        end_with_pattern = r'END\s+WITH\s+(T\d+(?:\s*(?:,|AND)\s*T\d+)*)'
        end_with_match = re.search(end_with_pattern, msg)
        if end_with_match:
            targets = _parse_target_ids(end_with_match.group(1))
            if targets:
                sequencing_hints["end_with"] = {"targets": targets}

    # Pattern 4: "last targets before landing/returning are T5 and T8"
    if "end_with" not in sequencing_hints:
        last_before_pattern = r'LAST\s+TARGETS?\s+(?:BEFORE\s+(?:LANDING|RETURN)(?:ING)?(?:\s+(?:ARE|SHOULD\s+BE))?\s*)?(T\d+(?:\s*(?:,|AND)\s*T\d+)*)'
        last_before_match = re.search(last_before_pattern, msg)
        if last_before_match:
            targets = _parse_target_ids(last_before_match.group(1))
            if targets:
                sequencing_hints["end_with"] = {"targets": targets}

    return allocation_ops, sequencing_hints


# =============================================================================
# Parser Result
# =============================================================================

@dataclass
class ParseResult:
    """Result of parsing a user constraint request."""
    program: ConstraintProgram
    confidence: float
    ambiguities: List[str]
    used_llm: bool
    raw_extraction: Optional[Dict[str, Any]] = None


# =============================================================================
# Main Parser Class
# =============================================================================

class ConstraintParser:
    """
    Parses natural language into ConstraintProgram objects.

    Can use either LLM-based extraction or regex fallback.
    """

    def __init__(
        self,
        llm_client: Optional[Any] = None,
        model: str = "claude-3-haiku-20240307",
        use_llm: bool = True,
    ):
        """
        Initialize the parser.

        Args:
            llm_client: Anthropic client for LLM-based parsing (optional)
            model: Model to use for LLM parsing
            use_llm: Whether to attempt LLM parsing (falls back to regex if unavailable)
        """
        self.llm_client = llm_client
        self.model = model
        self.use_llm = use_llm and llm_client is not None

    def _build_context(
        self,
        environment: Dict[str, Any],
        drone_configs: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Build context for the extraction prompt."""
        # Extract target info
        targets = environment.get("targets", [])
        target_ids = sorted([str(t.get("id", t.get("label", ""))) for t in targets])
        target_priorities = {
            str(t.get("id", t.get("label", ""))): t.get("priority", 5)
            for t in targets
        }

        # Extract drone info
        drone_ids = sorted([str(d) for d in drone_configs.keys()])

        return {
            "target_ids": target_ids,
            "drone_ids": drone_ids,
            "target_priorities": target_priorities,
        }

    def _llm_parse(
        self,
        user_message: str,
        context: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        """
        Use LLM to extract constraints from user message.

        Returns parsed JSON or None if LLM unavailable/failed.
        """
        if not self.llm_client:
            return None

        prompt = CONSTRAINT_EXTRACTION_PROMPT.format(
            target_ids=", ".join(context["target_ids"]),
            drone_ids=", ".join(f"D{d}" for d in context["drone_ids"]),
            target_priorities=json.dumps(context["target_priorities"]),
            user_message=user_message,
        )

        try:
            response = self.llm_client.messages.create(
                model=self.model,
                max_tokens=1024,
                messages=[{"role": "user", "content": prompt}],
            )

            # Extract JSON from response
            content = response.content[0].text
            # Try to find JSON block
            json_match = re.search(r'```(?:json)?\s*([\s\S]*?)\s*```', content)
            if json_match:
                json_str = json_match.group(1)
            else:
                # Assume entire response is JSON
                json_str = content

            return json.loads(json_str)

        except Exception as e:
            # Log but don't fail - fall back to regex
            return None

    def _build_program(
        self,
        allocation_ops: List[Dict[str, Any]],
        sequencing_hints: Dict[str, Any],
        original_text: str,
    ) -> ConstraintProgram:
        """Build a ConstraintProgram from extracted operations and hints."""
        program = ConstraintProgram(
            source="user",
            original_text=original_text,
            parsed_at=datetime.utcnow().isoformat(),
        )

        # Add allocation operations
        for op in allocation_ops:
            op_type = op.get("op", "").upper()

            if op_type == "FORCE_VISIT":
                program.add_force_visit(
                    targets=op.get("targets", []),
                    scope=op.get("scope", "global"),
                    drone_id=op.get("drone_id"),
                )
            elif op_type == "MOVE":
                program.add_move(
                    target=op.get("target", ""),
                    to_drone=op.get("to_drone", ""),
                    lock=op.get("lock", True),
                )
            elif op_type == "REMOVE":
                program.add_remove(
                    target=op.get("target", ""),
                    reason=op.get("reason"),
                )
            elif op_type == "SWAP":
                program.add_swap(
                    target_a=op.get("target_a", ""),
                    target_b=op.get("target_b", ""),
                    drone_a=op.get("drone_a"),
                    drone_b=op.get("drone_b"),
                )
            elif op_type == "INSERT":
                program.add_insert(
                    target=op.get("target", ""),
                    drone_id=op.get("drone_id"),
                    position=op.get("position"),
                )
            else:
                program.warnings.append(f"Unknown operation type: {op_type}")

        # Add sequencing hints
        if sequencing_hints:
            program.set_sequencing_hints(sequencing_hints)

        return program

    def parse(
        self,
        user_message: str,
        environment: Optional[Dict[str, Any]] = None,
        drone_configs: Optional[Dict[str, Any]] = None,
    ) -> ParseResult:
        """
        Parse a user message into a ConstraintProgram.

        Args:
            user_message: The natural language constraint request
            environment: Environment data (targets, airports, SAMs)
            drone_configs: Drone configurations

        Returns:
            ParseResult with the program, confidence, and any ambiguities
        """
        environment = environment or {}
        drone_configs = drone_configs or {}

        context = self._build_context(environment, drone_configs)
        used_llm = False
        raw_extraction = None
        confidence = 0.5
        ambiguities: List[str] = []

        # Try LLM parsing first
        if self.use_llm:
            llm_result = self._llm_parse(user_message, context)
            if llm_result:
                used_llm = True
                raw_extraction = llm_result
                allocation_ops = llm_result.get("allocation_ops", [])
                sequencing_hints = llm_result.get("sequencing_hints", {})
                confidence = llm_result.get("confidence", 0.8)
                ambiguities = llm_result.get("ambiguities", [])

                program = self._build_program(
                    allocation_ops, sequencing_hints, user_message
                )

                return ParseResult(
                    program=program,
                    confidence=confidence,
                    ambiguities=ambiguities,
                    used_llm=True,
                    raw_extraction=raw_extraction,
                )

        # Fall back to regex parsing
        allocation_ops, sequencing_hints = regex_fallback_parse(
            user_message,
            context["target_ids"],
            context["drone_ids"],
        )

        program = self._build_program(
            allocation_ops, sequencing_hints, user_message
        )

        # Regex parsing has lower confidence
        if allocation_ops or sequencing_hints:
            confidence = 0.6
        else:
            confidence = 0.3

        return ParseResult(
            program=program,
            confidence=confidence,
            ambiguities=ambiguities,
            used_llm=False,
            raw_extraction=None,
        )


# =============================================================================
# Convenience function
# =============================================================================

def parse_constraints(
    user_message: str,
    environment: Optional[Dict[str, Any]] = None,
    drone_configs: Optional[Dict[str, Any]] = None,
    llm_client: Optional[Any] = None,
) -> ParseResult:
    """
    Convenience function to parse constraints from a user message.

    Args:
        user_message: The natural language constraint request
        environment: Environment data
        drone_configs: Drone configurations
        llm_client: Optional Anthropic client for LLM parsing

    Returns:
        ParseResult with the ConstraintProgram
    """
    parser = ConstraintParser(llm_client=llm_client)
    return parser.parse(user_message, environment, drone_configs)
