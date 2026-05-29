from typing import Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException, Query
from pydantic import BaseModel

from app.dependencies import get_navigation_service
from app.services.navigation_service import NavigationService

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
    """
    pos = svc.localise(req.rssi, req.dt)
    if pos is None:
        raise HTTPException(status_code=204, detail="Not enough measurements")
    return pos.model_dump(by_alias=True)


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
async def reset_ekf(svc: NavigationService = Depends(get_navigation_service)):
    """Reset the EKF so localisation starts fresh."""
    svc.reset_ekf()
    return {"status": "reset"}
