"""
Solve Router - Solving and optimization endpoints.

Endpoints:
    POST /api/solve - Basic solving endpoint
    POST /api/solve_with_allocation - Solving with target allocation
    POST /api/prepare_matrix - Prepare distance matrix
    GET /api/matrix_status - Get matrix status
    POST /api/clear_matrix - Clear cached matrix
    POST /api/apply_sequence - Apply manual sequence
    POST /api/coverage_stats - Get coverage statistics
    POST /api/trajectory_swap_optimize - Swap closer optimization
    POST /api/crossing_removal_optimize - Remove route crossings
    POST /api/insert_missed_optimize - Insert missed targets
    GET /api/wrapped_polygons - Get SAM wrapped polygons
    POST /api/calculate_wrapping - Calculate SAM wrapping
"""

from fastapi import APIRouter

from ..schemas import (
    SolveRequest,
    SolveResponse,
    SolveWithAllocationRequest,
    ApplySequenceRequest,
    ApplySequenceResponse,
    PrepareMatrixRequest,
    PrepareMatrixResponse,
    CoverageStatsRequest,
    CoverageStatsResponse,
    TrajectorySwapRequest,
    InsertMissedRequest,
)

router = APIRouter(prefix="/api", tags=["solve"])

# =============================================================================
# NOTE: The endpoint implementations are currently in server/main.py
#
# During Phase 3 migration, these endpoints should be moved here from main.py:
#
# From main.py lines:
#   - solve():                  lines 1006-1158
#   - solve_with_allocation():  lines 1199-1377
#   - prepare_matrix():         lines 1379-1399
#   - matrix_status():          lines 1401-1421
#   - clear_matrix():           lines 1423-1430
#   - apply_sequence():         lines 1432-1516
#   - coverage_stats():         lines 1518-1538
#   - trajectory_swap_optimize(): lines 2067-2143
#   - crossing_removal_optimize(): lines 2145-2188
#   - insert_missed_optimize(): lines 2196-2280
#   - wrapped_polygons():       lines 2030-2051
#   - calculate_wrapping():     lines 2282-end
#
# For now, these endpoints remain in main.py and this router serves as
# documentation of the planned structure.
# =============================================================================

# Placeholder comments showing the router structure
# In a full migration, these decorators would have the actual implementations:

# @router.post("/solve", response_model=SolveResponse)
# async def solve(req: SolveRequest):
#     """Basic solving endpoint."""
#     pass

# @router.post("/solve_with_allocation", response_model=SolveResponse)
# async def solve_with_allocation(req: SolveWithAllocationRequest):
#     """Solve mission with target allocation."""
#     pass

# @router.post("/prepare_matrix", response_model=PrepareMatrixResponse)
# async def prepare_matrix(req: PrepareMatrixRequest):
#     """Prepare SAM-aware distance matrix."""
#     pass

# @router.get("/matrix_status")
# async def matrix_status():
#     """Get current distance matrix status."""
#     pass

# @router.post("/clear_matrix")
# async def clear_matrix():
#     """Clear cached distance matrix."""
#     pass

# @router.post("/apply_sequence", response_model=ApplySequenceResponse)
# async def apply_sequence(req: ApplySequenceRequest):
#     """Apply manual sequence to a drone."""
#     pass

# @router.post("/coverage_stats", response_model=CoverageStatsResponse)
# async def coverage_stats(req: CoverageStatsRequest):
#     """Get coverage statistics for a solution."""
#     pass

# @router.post("/trajectory_swap_optimize")
# async def trajectory_swap_optimize(req: TrajectorySwapRequest):
#     """Optimize by swapping targets to closer trajectories."""
#     pass

# @router.post("/crossing_removal_optimize")
# async def crossing_removal_optimize(req: TrajectorySwapRequest):
#     """Remove route crossings using 2-opt."""
#     pass

# @router.post("/insert_missed_optimize")
# async def insert_missed_optimize(req: InsertMissedRequest):
#     """Insert missed targets into routes."""
#     pass

# @router.get("/wrapped_polygons")
# async def wrapped_polygons():
#     """Get wrapped SAM polygon boundaries."""
#     pass

# @router.post("/calculate_wrapping")
# async def calculate_wrapping():
#     """Calculate SAM wrapping polygons."""
#     pass
