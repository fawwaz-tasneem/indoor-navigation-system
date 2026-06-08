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

    def __init__(self, process_noise: float = 1.0, measurement_noise: float = 30.0):

        # measurement_noise: expected trilateration error in pixels
        # (30 px × 0.05 m/px ≈ 1.5 m is a representative indoor WiFi error)

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

        # Using a piecewise constant white acceleration model (PCWNA)
        q = self._pn * dt
        Q = np.array([
            [q * dt*dt / 4,           0, q * dt / 2,          0],
            [           0, q * dt*dt / 4,          0, q * dt / 2],
            [  q * dt / 2,           0,          q,          0],
            [           0,  q * dt / 2,          0,          q],
        ])

        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def update(self, mx: float, my: float) -> None: # mx and my are the coords recieved from the trilateration
        if self._x is None:
            self.initialise(mx, my)
            return

        H = np.array([[1, 0, 0, 0],                 # Measurement Matrix H
                      [0, 1, 0, 0]], dtype=float)
        R = np.eye(2) * (self._mn ** 2)             # Measurement Noise Covariance (squaring the SD)

        z = np.array([mx, my])                  # measurment
        y = z - H @ self._x                    # innovation
        S = H @ self._P @ H.T + R              # innovation covariance
        K = self._P @ H.T @ np.linalg.inv(S)  # Kalman gain  (4×2)

        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ H) @ self._P     # Mathematically equivalent to P = P - K @ S @ K.T (less computations), but Joseph form better

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
