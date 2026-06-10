from functools import lru_cache
from typing import Dict
import time
from dataclasses import dataclass, field

from app.services.map_service import MapService
from app.services.navigation_service import NavigationService


@lru_cache(maxsize=1)
def get_map_service() -> MapService:
    return MapService()

@dataclass
class _Entry:
    service: NavigationService
    last_seen: float = field(default_factory=time.monotonic)


# Per-session filter registry.
# lru_cache is intentionally NOT used here: NavigationService owns the EKF
# and KF state, so a single shared instance would corrupt filter estimates
# across concurrent users.  Each session_id gets its own independent instance.
_service_registry: Dict[str, NavigationService] = {}
_SESSION_TTL = 600.0 # seconds

def get_navigation_service(session_id: str = "default") -> NavigationService:
    now = time.monotonic()
    # evict anything idle for more than TTL
    stale = [k for k, v in _service_registry.items() if now - v.last_seen > _SESSION_TTL]
    for k in stale:
        del _service_registry[k]

    if session_id not in _service_registry:
        _service_registry[session_id] = _Entry(NavigationService(get_map_service()))
        entry = _service_registry[session_id]
        entry.last_seen = now
    return entry.service
