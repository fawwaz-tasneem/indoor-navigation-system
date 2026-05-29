from __future__ import annotations

import math
from typing import Dict, List, Optional

from app.core.ekf import ExtendedKalmanFilter
from app.core.pathfinder import PathFinder
from app.core.rssi import rssi_to_distance
from app.core.trilateration import Measurement, solve as trilaterate
from app.models import NavNode, Position
from app.services.map_service import MapService


class NavigationService:
    def __init__(self, map_service: MapService) -> None:
        self._map = map_service
        self._ekf = ExtendedKalmanFilter(process_noise=1.0, measurement_noise=30.0)
        self._pathfinder: Optional[PathFinder] = None

    # ── localisation ──────────────────────────────────────────────────────────

    def localise(self, rssi_readings: Dict[str, float], dt: float = 1.0) -> Optional[Position]:
        """
        Accept { bssid: rssi_dbm } readings, run trilateration, feed the EKF.
        Returns the smoothed position estimate, or None if not yet reachable.
        """
        cfg = self._map.get_config()
        ap_by_bssid = {ap.bssid: ap for ap in cfg.access_points}

        measurements = [
            Measurement(ap=ap_by_bssid[bssid], distance_meters=rssi_to_distance(rssi, ap_by_bssid[bssid]))
            for bssid, rssi in rssi_readings.items()
            if bssid in ap_by_bssid
        ]

        if len(measurements) < 3:
            return self._ekf.position   # return last known estimate

        trilat = trilaterate(measurements, cfg.meters_per_pixel)
        if trilat is None:
            return self._ekf.position

        self._ekf.predict(dt)
        self._ekf.update(trilat.x, trilat.y)
        return self._ekf.position

    def reset_ekf(self) -> None:
        self._ekf.reset()

    # ── pathfinding ───────────────────────────────────────────────────────────

    def navigate(self, from_id: str, to_id: str) -> List[NavNode]:
        if self._pathfinder is None:
            self._pathfinder = PathFinder(self._map.get_config())
        return self._pathfinder.find_path(from_id, to_id)

    def nearest_node(self, canvas_x: float, canvas_y: float) -> Optional[NavNode]:
        nodes = self._map.get_config().nodes
        if not nodes:
            return None
        return min(nodes, key=lambda n: math.hypot(n.x - canvas_x, n.y - canvas_y))
