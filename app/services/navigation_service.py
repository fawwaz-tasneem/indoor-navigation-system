from __future__ import annotations

import math
from typing import Dict, List, Optional
import time

from app.core.kf import KalmanFilter
from app.core.ekf import ExtendedKalmanFilter
from app.core.pathfinder import PathFinder
from app.core.rssi import rssi_to_distance
from app.core.trilateration import Measurement, solve as trilaterate
from app.models import NavNode, Position, LocalisationResult
from app.services.map_service import MapService


class NavigationService:
    def __init__(self, map_service: MapService) -> None:
        self._map = map_service
        self._ekf = ExtendedKalmanFilter(process_noise=1.0, measurement_noise=2.0)      # measurement_noise in meters
        self._kf = KalmanFilter(process_noise=1.0, measurement_noise=30.0)              # measurement_noise in pixels
        self._pathfinder: Optional[PathFinder] = None
        self._last_time: Optional[float] = None                                         # time stamp of last localise call

    # ── localisation ──────────────────────────────────────────────────────────

    def localise(self, rssi_readings: Dict[str, float]) -> LocalisationResult:
        """
        Accept { bssid: rssi_dbm } readings and run both filters in parallel.

        SSID - Service Set Identifier - human readable, non unique
        BSSID - Basic SSID - globally unique MAC address

        EKF path: raw (AP, range_metres) pairs fed directly into the EKF.
                  The measurement model h_i(x) is nonlinear (Euclidean distance).
        KF  path: (AP, range_metres) pairs first collapsed to a single (x,y)
                  via trilateration, then fed into the linear KF.

        On the first call, the EKF is initialised from the trilateration result
        so it starts near the true position rather than converging from scratch.

        Returns LocalisationResult with both estimates (either may be None if
        the respective filter has not yet been initialised).

        dt is computed server-side from a monotonic clock so the filter's
        predict step reflects real elapsed time rather than a client-supplied value.
        A floor of 0.01 s prevents near-zero dt; a ceiling of 10 s prevents Q
        from blowing up when the client goes idle between calls
        """

        now = time.monotonic()
        if self._last_time is None:
            dt = 1.0   # first call: use a neutral default
        else:
            dt = now - self._last_time
            dt = min(max(dt, 0.01), 10.0)
        self._last_time = now

        # Fetch the active floor map configuration profile
        cfg = self._map.get_config()            
        # Pivot the access point list into a dictionary for O(1) high-speed BSSID lookups
        ap_by_bssid = {ap.bssid: ap for ap in cfg.access_points}

        # Map raw hardware signal strength inputs into physical ranges (meters)
        raw = [
            (ap_by_bssid[bssid], rssi_to_distance(rssi, ap_by_bssid[bssid]))
            for bssid, rssi in rssi_readings.items()
            if bssid in ap_by_bssid
        ]

        if len(raw) < 3:        # TODO: EKF doesn't need this hard constraint, it can predict with less than three
            return LocalisationResult(ekf=self._ekf.position, kf=self._kf.position)

        # Trilateration — used by the KF and to seed the EKF on first call
        tril_meas = [Measurement(ap=ap, distance_meters=r) for ap, r in raw]
        trilat    = trilaterate(tril_meas, cfg.meters_per_pixel)

        # ── EKF (primary) ──────────────────────────────────────────────────────
        if not self._ekf.is_initialised and trilat is not None:
            self._ekf.initialise(trilat.x, trilat.y)

        self._ekf.predict(dt)
        self._ekf.update(raw, cfg.meters_per_pixel)

        # ── KF (comparison baseline) ───────────────────────────────────────────
        self._kf.predict(dt)
        if trilat is not None:
            self._kf.update(trilat.x, trilat.y)

        return LocalisationResult(ekf=self._ekf.position, kf=self._kf.position)

    def reset_filters(self) -> None:
        self._ekf.reset()
        self._kf.reset()
        self._last_time = None 

    # ── pathfinding ───────────────────────────────────────────────────────────

    def navigate(self, from_id: str, to_id: str) -> List[NavNode]:
        """Calculates the shortest walkable pathway routing structure between two nodes."""
        if self._pathfinder is None:
            self._pathfinder = PathFinder(self._map.get_config())
        return self._pathfinder.find_path(from_id, to_id)

    def nearest_node(self, canvas_x: float, canvas_y: float) -> Optional[NavNode]:
        """Finds the closest structural waypoint node relative to a given pixel coordinate."""
        nodes = self._map.get_config().nodes
        if not nodes:
            return None
        return min(nodes, key=lambda n: math.hypot(n.x - canvas_x, n.y - canvas_y))