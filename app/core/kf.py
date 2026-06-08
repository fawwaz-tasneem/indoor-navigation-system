from __future__ import annotations
from typing import Optional
import numpy as np
from app.models import Position


class KalmanFilter:
    """
    2-D KF for indoor positioning.

    State:  x = [px, py, vx, vy]^T   (canvas pixels) is trivial to convert from metre to pixel and back

    Prediction (constant-velocity):
    F represents the motion model
        F = [[1, 0, dt, 0],
             [0, 1,  0, dt],
             [0, 0,  1,  0],
             [0, 0,  0,  1]]

    Measurement (position only from trilateration):
        H = [[1, 0, 0, 0],
             [0, 1, 0, 0]]

    Both motion model (F) and sensor model (H) are linear, so this is a standard Kalman
    filter. The full non-linear RSSI-to-position pipeline runs upstream; by the time the
    filter sees a measurement, it is already a 2D cartesian corrdinate (linear).

    This filter serves as a comparison against EKF in ekf.py, which fuses the raw ranges
    per AP directly.
    """

    def __init__(self, process_noise: float = 300.0, measurement_noise: float = 100.0):

        # measurement_noise: expected trilateration position error in pixels
        # (100 px × 0.05 m/px = 5 m).  Trilateration amplifies per-AP range errors
        # via the geometry's DOP; with 4 APs in corner positions and σ_range 2–5 m,
        # post-trilateration σ_position of 4–6 m is typical.

        self._pn = process_noise       # Q tuning: trust in constant-velocity model
        self._mn = measurement_noise   # R tuning: trilateration noise (pixels)
        self._x: Optional[np.ndarray] = None   # state matrix x (4,)
        self._P: Optional[np.ndarray] = None   # covariance matrix P(4, 4)

    # ── public ────────────────────────────────────────────────────────────────

    def initialise(self, px: float, py: float) -> None:
        self._x = np.array([px, py, 0.0, 0.0])
        self._P = np.eye(4) * 500.0   # high initial uncertainty

    def predict(self, dt: float) -> None:
        if self._x is None:
            return

        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=float)

        # PCWNA model: Q = q * [[dt⁴/4, dt³/2], [dt³/2, dt²]] per axis
        q = self._pn
        Q = q * np.array([
            [dt**4/4,       0, dt**3/2,       0],
            [      0, dt**4/4,       0, dt**3/2],
            [dt**3/2,       0,   dt**2,       0],
            [      0, dt**3/2,       0,   dt**2],
        ])


        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def update(self, mx: float, my: float) -> bool:
        """
        Update state from a trilaterated (x, y) position measurement.
        Returns True if accepted, False if rejected by the NIS gate.
        """
        if self._x is None:
            self.initialise(mx, my)
            return True

        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)
        R = np.eye(2) * (self._mn ** 2)

        z     = np.array([mx, my])
        y     = z - H @ self._x               # innovation
        S     = H @ self._P @ H.T + R         # innovation covariance

        # NIS outlier gate — measurement DOF is always 2 (x, y), so the
        # threshold is chi²(0.95, dof=2) = 5.99.  A trilaterated fix whose
        # NIS exceeds this is likely a corrupted RSSI scan; discard it.
        S_inv = np.linalg.inv(S)
        if float(y @ S_inv @ y) > 5.99:
            return False

        K         = self._P @ H.T @ S_inv     # Kalman gain (4×2)
        self._x   = self._x + K @ y

        # Joseph form: P = (I-KH) P (I-KH)^T + K R K^T
        # Keeps P symmetric positive-definite under floating-point accumulation,
        # unlike the simpler P = (I-KH) P which can drift asymmetric over time.
        I_KH    = np.eye(4) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T

        return True

    @property
    def position(self) -> Optional[Position]:
        if self._x is None:
            return None
        uncertainty = float(np.sqrt(self._P[0, 0] + self._P[1, 1]))
        return Position(x=float(self._x[0]), y=float(self._x[1]), uncertainty=uncertainty)

    @property
    def is_initialised(self) -> bool:
        return self._x is not None

    def reset(self) -> None:
        self._x = None
        self._P = None
