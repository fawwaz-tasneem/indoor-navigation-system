from typing import Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException, Query
from pydantic import BaseModel

from app.dependencies import get_navigation_service
from app.models import LocalisationResult
from app.services.navigation_service import NavigationService

"""
Indoor Navigation and Tracking API Router.

This module serves as the Controller Layer (Entry Point) for the indoor positioning 
application. It exposes REST endpoints via FastAPI to interface with client-side applications.

Key Software Engineering Design Decisions:
1. Separation of Concerns (Thin Controllers):
   - This router contains zero mathematical, filtering, or pathfinding logic. 
   - It strictly handles HTTP request parsing, data validation, and response serialization, 
     delegating all domain execution to the core `NavigationService`.

2. Dependency Injection (DI) via `Depends`:
   - Decouples the API layer from the initialization details of the `NavigationService`.
   - Enhances testability by allowing downstream services or configurations to be 
     easily swapped or mocked during automated unit testing.

3. Strict Request Validation & Case Harmonization:
   - Utilizes Pydantic input schemas (`LocaliseRequest`) to sanitize payloads at the boundary.
   - Enforces automatic, bidirectional transformation between Python's internal `snake_case` 
     and the frontend's web-standard `camelCase` using serialization aliases via `model_dump`.

4. Semantic HTTP Status Protocols:
   - Avoids generic server errors by responding with precise, actionable status codes 
     (e.g., 204 No Content for poor coverage, 404 Not Found for missing coordinates).
"""

router = APIRouter()


# ── request schema ────────────────────────────────────────────────────────────

class LocaliseRequest(BaseModel):
    rssi: Dict[str, float]   # { bssid: rssi_dbm }
    dt:   float = 1.0


# ── routes ────────────────────────────────────────────────────────────────────

@router.post("/localise")
async def localise(
    req: LocaliseRequest,
    svc: NavigationService = Depends(get_navigation_service),
):
    """
    POST /api/nav/localise
    Body: { "rssi": {"AA:BB:CC:DD:EE:01": -65, ...}, "dt": 1.0 }
    Returns both EKF (raw-range fusion) and KF (trilaterated-position fusion) estimates.
    """
    result = svc.localise(req.rssi, req.dt)
    if result.ekf is None and result.kf is None:
        raise HTTPException(status_code=204, detail="Not enough measurements")
    return result.model_dump(by_alias=True)


@router.get("/path")
async def get_path(
    from_: str = Query(alias="from"),
    to:    str = Query(),
    svc:   NavigationService = Depends(get_navigation_service),
):
    """GET /api/nav/path?from=entrance&to=lab1"""
    path = svc.navigate(from_, to)
    if not path:
        raise HTTPException(status_code=404, detail="No path found")
    return [n.model_dump(by_alias=True) for n in path]


@router.get("/nearest")
async def nearest_node(
    x:   float = Query(),
    y:   float = Query(),
    svc: NavigationService = Depends(get_navigation_service),
):
    """GET /api/nav/nearest?x=450&y=340"""
    node = svc.nearest_node(x, y)
    if node is None:
        raise HTTPException(status_code=404, detail="No nodes in map")
    return node.model_dump(by_alias=True)


@router.post("/reset")
async def reset_filters(svc: NavigationService = Depends(get_navigation_service)):
    """Reset both filters so localisation starts fresh."""
    svc.reset_filters()
    return {"status": "reset"}

