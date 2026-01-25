"""
Environment Router - Environment management endpoints.

Endpoints:
    POST /api/environment - Save environment
    GET /api/environment - Get current environment
    GET /api/export_environment - Export environment to file
"""

from fastapi import APIRouter

router = APIRouter(prefix="/api", tags=["environment"])

# =============================================================================
# NOTE: The endpoint implementations are currently in server/main.py
#
# During Phase 3 migration, these endpoints should be moved here from main.py:
#
# From main.py lines:
#   - save_environment():       lines 732-823
#   - get_environment():        lines 825-848
#   - export_environment():     lines 850-934
#
# For now, these endpoints remain in main.py and this router serves as
# documentation of the planned structure.
# =============================================================================

# Placeholder comments showing the router structure:

# @router.post("/environment")
# async def save_environment(request: Request):
#     """Save environment to file."""
#     pass

# @router.get("/environment")
# async def get_environment():
#     """Get current environment."""
#     pass

# @router.get("/export_environment")
# async def export_environment():
#     """Export environment to numbered JSON file."""
#     pass
