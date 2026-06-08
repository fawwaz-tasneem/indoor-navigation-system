from functools import lru_cache
from typing import Dict

from app.services.map_service import MapService
from app.services.navigation_service import NavigationService


@lru_cache(maxsize=1)
def get_map_service() -> MapService:
    return MapService()


# Per-session filter registry.
# lru_cache is intentionally NOT used here: NavigationService owns the EKF
# and KF state, so a single shared instance would corrupt filter estimates
# across concurrent users.  Each session_id gets its own independent instance.
_service_registry: Dict[str, NavigationService] = {}


def get_navigation_service(session_id: str = "default") -> NavigationService:
    if session_id not in _service_registry:
        _service_registry[session_id] = NavigationService(get_map_service())
    return _service_registry[session_id]
