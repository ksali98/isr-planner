"""
Unit tests for Pydantic API Schemas.

Tests schema validation, defaults, and serialization for:
- Solve request/response schemas
- Optimization schemas
- Agent schemas
- Executive schemas
"""

import pytest
from pydantic import ValidationError


class TestSolveSchemas:
    """Tests for solve endpoint schemas."""

    def test_solve_request_minimal(self):
        """Test SolveRequest with minimal required fields."""
        from server.schemas import SolveRequest

        req = SolveRequest(
            env={"airports": [], "targets": [], "sams": []},
            drone_configs={"1": {"enabled": True}}
        )

        assert req.allocation_strategy == "efficient"  # default
        assert req.mission_id is None  # optional

    def test_solve_request_full(self):
        """Test SolveRequest with all fields."""
        from server.schemas import SolveRequest

        req = SolveRequest(
            env={"airports": [{"id": "A1", "x": 0, "y": 0}], "targets": [], "sams": []},
            drone_configs={"1": {"enabled": True, "fuelBudget": 300}},
            allocation_strategy="greedy",
            mission_id="test-123"
        )

        assert req.allocation_strategy == "greedy"
        assert req.mission_id == "test-123"

    def test_solve_request_missing_required(self):
        """Test SolveRequest fails without required fields."""
        from server.schemas import SolveRequest

        with pytest.raises(ValidationError):
            SolveRequest(env={})  # missing drone_configs

    def test_solve_response(self):
        """Test SolveResponse schema."""
        from server.schemas import SolveResponse

        resp = SolveResponse(
            success=True,
            routes={"1": {"route": ["A1", "T1", "A1"]}},
            sequences={"1": "A1,T1,A1"}
        )

        assert resp.success == True
        assert resp.wrapped_polygons is None  # optional

    def test_solve_with_allocation_request(self):
        """Test SolveWithAllocationRequest schema."""
        from server.schemas import SolveWithAllocationRequest

        req = SolveWithAllocationRequest(
            env={"airports": [], "targets": [], "sams": []},
            drone_configs={"1": {"enabled": True}},
            visited_targets=["T1", "T2"],
            is_checkpoint_replan=True
        )

        assert req.post_optimize == True  # default
        assert req.visited_targets == ["T1", "T2"]
        assert req.is_checkpoint_replan == True


class TestApplySequenceSchemas:
    """Tests for apply sequence schemas."""

    def test_apply_sequence_request(self):
        """Test ApplySequenceRequest schema."""
        from server.schemas import ApplySequenceRequest

        req = ApplySequenceRequest(
            drone_id="1",
            sequence="A1,T1,T2,A1",
            env={"airports": [], "targets": [], "sams": []},
            fuel_budget=250.0
        )

        assert req.drone_id == "1"
        assert req.sequence == "A1,T1,T2,A1"
        assert req.fuel_budget == 250.0

    def test_apply_sequence_request_default_fuel(self):
        """Test ApplySequenceRequest default fuel budget."""
        from server.schemas import ApplySequenceRequest

        req = ApplySequenceRequest(
            drone_id="1",
            sequence="A1,T1,A1",
            env={}
        )

        assert req.fuel_budget == 300.0  # default

    def test_apply_sequence_response(self):
        """Test ApplySequenceResponse schema."""
        from server.schemas import ApplySequenceResponse

        resp = ApplySequenceResponse(
            success=True,
            route=["A1", "T1", "A1"],
            distance=100.0,
            points=10,
            trajectory=[[0, 0], [50, 50], [0, 0]]
        )

        assert resp.success == True
        assert resp.error is None


class TestOptimizationSchemas:
    """Tests for optimization endpoint schemas."""

    def test_trajectory_swap_request(self):
        """Test TrajectorySwapRequest schema."""
        from server.schemas import TrajectorySwapRequest

        req = TrajectorySwapRequest(
            solution={"routes": {}},
            env={},
            drone_configs={},
            visited_targets=["T1"],
            auto_iterate=True,
            debug=True
        )

        assert req.auto_iterate == True
        assert req.auto_regen == False  # default
        assert req.visited_targets == ["T1"]

    def test_insert_missed_request(self):
        """Test InsertMissedRequest schema."""
        from server.schemas import InsertMissedRequest

        req = InsertMissedRequest(
            solution={"routes": {}, "missed_targets": ["T3"]},
            env={"targets": []},
            drone_configs={"1": {}}
        )

        assert "missed_targets" in req.solution

    def test_coverage_stats_request(self):
        """Test CoverageStatsRequest schema."""
        from server.schemas import CoverageStatsRequest

        req = CoverageStatsRequest(
            solution={"routes": {}},
            env={"targets": []}
        )

        assert req.solution is not None

    def test_coverage_stats_response(self):
        """Test CoverageStatsResponse schema."""
        from server.schemas import CoverageStatsResponse

        resp = CoverageStatsResponse(
            targets_visited=3,
            targets_total=5,
            coverage_percent=60.0,
            points_collected=25,
            points_possible=40,
            points_percent=62.5,
            total_distance=350.0,
            unvisited_targets=["T4", "T5"]
        )

        assert resp.coverage_percent == 60.0
        assert len(resp.unvisited_targets) == 2


class TestAgentSchemas:
    """Tests for agent endpoint schemas."""

    def test_agent_chat_request_minimal(self):
        """Test AgentChatRequest with minimal fields."""
        from server.schemas import AgentChatRequest

        req = AgentChatRequest(
            message="Plan a mission",
            env={"airports": [], "targets": [], "sams": []}
        )

        assert req.message == "Plan a mission"
        assert req.sequences is None
        assert req.drone_configs is None

    def test_agent_chat_request_full(self):
        """Test AgentChatRequest with all fields."""
        from server.schemas import AgentChatRequest

        req = AgentChatRequest(
            message="Optimize the routes",
            env={"airports": [], "targets": [], "sams": []},
            sequences={"1": "A1,T1,A1"},
            drone_configs={"1": {"enabled": True}},
            mission_id="mission-456",
            existing_solution={"routes": {}}
        )

        assert req.mission_id == "mission-456"
        assert req.existing_solution is not None

    def test_agent_chat_response(self):
        """Test AgentChatResponse schema."""
        from server.schemas import AgentChatResponse

        resp = AgentChatResponse(
            reply="Mission planned successfully.",
            intent="solve",
            routes={"1": ["A1", "T1", "A1"]},
            allocations={"1": ["T1"]}
        )

        assert resp.reply == "Mission planned successfully."
        assert resp.intent == "solve"
        assert resp.valid is None  # optional

    def test_agent_trace_event(self):
        """Test AgentTraceEvent schema."""
        from server.schemas import AgentTraceEvent

        event = AgentTraceEvent(
            t="coordinator.intent",
            msg="Classified as solve request",
            data={"confidence": 0.95},
            ts_ms=1234567890
        )

        assert event.t == "coordinator.intent"
        assert event.data["confidence"] == 0.95

    def test_add_memory_request(self):
        """Test AddMemoryRequest schema."""
        from server.schemas import AddMemoryRequest

        req = AddMemoryRequest(
            content="Always prioritize high-value targets",
            category="instruction"
        )

        assert req.content == "Always prioritize high-value targets"
        assert req.category == "instruction"

    def test_add_memory_request_default_category(self):
        """Test AddMemoryRequest default category."""
        from server.schemas import AddMemoryRequest

        req = AddMemoryRequest(content="Fix this bug")

        assert req.category == "correction"  # default


class TestExecutiveSchemas:
    """Tests for executive endpoint schemas."""

    def test_executive_tick_request_minimal(self):
        """Test ExecutiveTickRequest with minimal fields."""
        from server.schemas import ExecutiveTickRequest

        req = ExecutiveTickRequest()

        assert req.mission_id is None
        assert req.ui_state == {}
        assert req.events == []

    def test_executive_tick_request_full(self):
        """Test ExecutiveTickRequest with all fields."""
        from server.schemas import ExecutiveTickRequest

        req = ExecutiveTickRequest(
            mission_id="exec-789",
            ui_state={"animation_playing": False},
            events=[{"type": "cut", "distance": 100}],
            env={"airports": []},
            drone_configs={"1": {}}
        )

        assert req.mission_id == "exec-789"
        assert len(req.events) == 1

    def test_executive_tick_response(self):
        """Test ExecutiveTickResponse schema."""
        from server.schemas import ExecutiveTickResponse

        resp = ExecutiveTickResponse(
            action="continue",
            mission_id="exec-789",
            visited_targets=["T1", "T2"],
            message="Mission in progress"
        )

        assert resp.action == "continue"
        assert resp.draft_plan is None
        assert len(resp.visited_targets) == 2
