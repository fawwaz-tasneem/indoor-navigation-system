from functools import lru_cache

from app.services.map_service import MapService
from app.services.navigation_service import NavigationService


@lru_cache(maxsize=1)
def get_map_service() -> MapService:
    return MapService()


@lru_cache(maxsize=1)
def get_navigation_service() -> NavigationService:
    return NavigationService(get_map_service())
