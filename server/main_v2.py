"""
ISR Mission Planner API v2 - Clean Rewrite
==========================================
Single-responsibility endpoints with consistent response formats.
"""

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pydantic import BaseModel
from typing import Dict, List, Any, Optional
from pathlib import Path
import sys

# Add project root to path
project_root = Path(__file__).parent.parent.parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# Import solver components
from .solver.solver_bridge import solve_mission_with_allocation
from .solver.trajectory_planner import ISRTrajectoryPlanner

# Import polygon wrapping
try:
    from path_planning_core.polygon_navigation import wrap_sams
except ImportError:
    wrap_sams = None

app = FastAPI(title="ISR Mission Planner v2")

# Static files
webapp_dir = Path(__file__).parent.parent / "webapp"
app.mount("/static", StaticFiles(directory=str(webapp_dir)), name="static")


# =============================================================================
# Pydantic Models - Clear Data Contracts
# =============================================================================

class Target(BaseModel):
    id: str
    x: float
    y: float
    type: str = "A"
    priority: int = 5


class Airport(BaseModel):
    id: str
    x: float
    y: float


class SAM(BaseModel):
    id: str
    x: float
    y: float
    range: float


class DroneConfig(BaseModel):
    enabled: bool = True
    fuel_budget: float = 300
    start_airport: str = "A1"
    end_airport: str = "A1"
    target_access: Dict[str, bool] = {"A": True, "B": True, "C": True, "D": True}


class Environment(BaseModel):
    targets: List[Target]
    airports: List[Airport]
    sams: List[SAM] = []


class SolveRequest(BaseModel):
    env: Environment
    drone_configs: Dict[str, DroneConfig]
    allocation_strategy: str = "efficient"


class DroneRoute(BaseModel):
    sequence: str
    route: List[str]
    trajectory: List[List[float]]
    distance: float
    points: int
    fuel_budget: float


class SolveResponse(BaseModel):
    success: bool
    error: Optional[str] = None
    routes: Dict[str, DroneRoute] = {}
    allocations: Dict[str, List[str]] = {}
    wrapped_polygons: List[List[List[float]]] = []


# =============================================================================
# API Endpoints
# =============================================================================

@app.get("/")
async def index():
    """Serve the main page."""
    return FileResponse(webapp_dir / "index_v2.html")


@app.post("/api/solve", response_model=SolveResponse)
async def solve(req: SolveRequest):
    """
    Solve the mission planning problem.

    Returns routes, allocations, trajectories, and wrapped polygons.
    """
    try:
        # Convert Pydantic models to dicts for solver
        env_dict = {
            "targets": [t.model_dump() for t in req.env.targets],
            "airports": [a.model_dump() for a in req.env.airports],
            "sams": [s.model_dump() for s in req.env.sams],
        }

        drone_configs_dict = {
            did: cfg.model_dump() for did, cfg in req.drone_configs.items()
        }

        print("\n" + "=" * 60)
        print("📥 SOLVE REQUEST")
        print(f"   Targets: {len(req.env.targets)}")
        print(f"   Airports: {len(req.env.airports)}")
        print(f"   SAMs: {len(req.env.sams)}")
        print(f"   Drones: {list(drone_configs_dict.keys())}")
        print(f"   Strategy: {req.allocation_strategy}")
        print("=" * 60)

        # Call solver
        result = solve_mission_with_allocation(
            env=env_dict,
            drone_configs=drone_configs_dict,
            allocation_strategy=req.allocation_strategy,
        )

        # Extract allocations
        allocations = result.get("allocations", {})

        # Log allocations
        print("\n🎯 ALLOCATIONS:")
        for did in sorted(allocations.keys(), key=lambda x: int(x)):
            targets = allocations[did]
            if targets:
                print(f"   D{did}: {targets}")

        # Build response routes with trajectories
        routes_response = {}
        solver_routes = result.get("routes", {})

        # Build waypoint positions for trajectory generation
        waypoint_positions = {}
        for t in req.env.targets:
            waypoint_positions[t.id] = [t.x, t.y]
        for a in req.env.airports:
            waypoint_positions[a.id] = [a.x, a.y]

        # Create trajectory planner
        sams_for_planner = [
            {"pos": [s.x, s.y], "range": s.range}
            for s in req.env.sams
        ]
        planner = ISRTrajectoryPlanner(sams_for_planner)

        for did, route_data in solver_routes.items():
            route_list = route_data.get("route", [])
            sequence = route_data.get("sequence", ",".join(route_list))

            # Generate SAM-avoiding trajectory
            trajectory = planner.generate_trajectory(
                route_list, waypoint_positions, drone_id=did
            )

            routes_response[did] = DroneRoute(
                sequence=sequence,
                route=route_list,
                trajectory=trajectory,
                distance=route_data.get("distance", 0),
                points=route_data.get("points", 0),
                fuel_budget=drone_configs_dict.get(did, {}).get("fuel_budget", 300),
            )

        # Calculate wrapped polygons
        wrapped_polygons = []
        if wrap_sams and req.env.sams:
            sams_for_wrapping = [
                {"pos": (s.x, s.y), "range": s.range}
                for s in req.env.sams
            ]
            try:
                polygon_arrays, _ = wrap_sams(sams_for_wrapping)
                wrapped_polygons = [poly.tolist() for poly in polygon_arrays]
            except Exception as e:
                print(f"⚠️ Wrapping failed: {e}")

        print("\n✅ SOLVE COMPLETE")
        print(f"   Routes generated: {len(routes_response)}")
        print(f"   Polygons: {len(wrapped_polygons)}")

        return SolveResponse(
            success=True,
            routes=routes_response,
            allocations=allocations,
            wrapped_polygons=wrapped_polygons,
        )

    except Exception as e:
        import traceback
        traceback.print_exc()
        return SolveResponse(
            success=False,
            error=str(e),
        )


@app.post("/api/calculate_wrapping")
async def calculate_wrapping(env: Environment):
    """Calculate wrapped SAM polygon boundaries."""
    try:
        if not wrap_sams or not env.sams:
            return {"success": True, "polygons": []}

        sams_for_wrapping = [
            {"pos": (s.x, s.y), "range": s.range}
            for s in env.sams
        ]

        polygon_arrays, _ = wrap_sams(sams_for_wrapping)
        polygons = [poly.tolist() for poly in polygon_arrays]

        return {"success": True, "polygons": polygons}

    except Exception as e:
        return {"success": False, "error": str(e), "polygons": []}


# =============================================================================
# Run with: uvicorn isr_web.server.main_v2:app --port 8894
# =============================================================================
