from fastapi import APIRouter, Depends

from app.dependencies import get_map_service
from app.services.map_service import MapService

router = APIRouter()


@router.get("")
async def get_map(svc: MapService = Depends(get_map_service)):
    """Return the full map config — nodes, edges, APs, canvas dimensions."""
    return svc.get_config().model_dump(by_alias=True)
