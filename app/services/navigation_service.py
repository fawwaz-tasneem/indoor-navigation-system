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
        # Both filters share q = 300 px²/s³ = 0.75 m²/s³ (PCWNA spectral density).
        # σ_position ≈ √(q·T³/3) ≈ 0.5 m after 1 s — matches pedestrian dynamics.
        #
        # EKF uses range-proportional R (heteroscedastic):
        #   σ_range(d) = d · ln(10)/(10·n) · σ_RSSI  (≈ ±21 % of the predicted range)
        #   At 10 m: σ ≈ 2.1 m; at 25 m: σ ≈ 5.3 m.
        #   R is rebuilt each update so far APs (noisy) don't inflate NIS.
        #
        # KF  measurement_noise = 100 px = 5 m (post-trilateration position σ).
        #   Larger than per-AP σ because the 4-corner AP geometry has poor DOP
        #   in the building interior, amplifying individual range errors.
        self._ekf = ExtendedKalmanFilter(process_noise=300.0, path_loss_n=2.7, rssi_sigma_dbm=2.5)
        self._kf  = KalmanFilter(process_noise=300.0, measurement_noise=100.0)
        self._pathfinder: Optional[PathFinder] = None
        self._last_time: Optional[float] = None                                         # time stamp of last localise call  
        self._ekf_consec_rejections: int    = 0

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
            return LocalisationResult(ekf=self._ekf.position, kf=self._kf.position)  # pre-gate early return — rare path

        # Trilateration — used by the KF and to seed the EKF on first call
        tril_meas = [Measurement(ap=ap, distance_meters=r) for ap, r in raw]
        trilat    = trilaterate(tril_meas, cfg.meters_per_pixel)

        # ── EKF (primary) ──────────────────────────────────────────────────────
        if not self._ekf.is_initialised and trilat is not None:
            self._ekf.initialise(trilat.x, trilat.y)

        self._ekf.predict(dt)
        accepted = self._ekf.update(raw, cfg.meters_per_pixel)
        if accepted:
            self._ekf_consec_rejections = 0
        else:
            self._ekf_consec_rejections += 1
            # Adaptive threshold: reinit only after 1.5 s of sustained rejection.
            # max(3, int(1.5/dt)) scales automatically with the update rate:
            #   10 Hz → 15 steps,  2 Hz → 3 steps,  1 Hz → 3 steps (floor).
            if self._ekf_consec_rejections >= max(3, int(1.5 / dt)) and trilat is not None:
                self._ekf.initialise(trilat.x, trilat.y)
                self._ekf_consec_rejections = 0


        # ── KF (comparison baseline) ───────────────────────────────────────────
        self._kf.predict(dt)
        if trilat is not None:
            self._kf.update(trilat.x, trilat.y)

        # Gate estimated positions to the canvas boundary.
        # The EKF/KF state is unconstrained; large RSSI spikes can push the
        # estimate outside the building footprint.  Clamping to canvas extent
        # prevents the UI dot from escaping the map entirely.
        W = float(cfg.canvas_width)
        H = float(cfg.canvas_height)

        def _gate(pos: Optional[Position]) -> Optional[Position]:
            if pos is None:
                return None
            return Position(
                x=max(0.0, min(W, pos.x)),
                y=max(0.0, min(H, pos.y)),
                uncertainty=pos.uncertainty,
            )

        return LocalisationResult(ekf=_gate(self._ekf.position), kf=_gate(self._kf.position))

    # new
    def reset_filters(self) -> None:
        self._ekf.reset()
        self._kf.reset()
        self._last_time             = None
        self._ekf_consec_rejections = 0


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