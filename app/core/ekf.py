from __future__ import annotations
from typing import List, Optional, Tuple
import numpy as np
from app.models import AccessPoint, Position

# 95th percentile chi-squared thresholds, indexed by degrees of freedom.
# Under the filter model, the NIS statistic for n measurements ~ chi^2(n).
# Measurements with NIS above this threshold are treated as outliers.
_CHI2_95: dict[int, float] = {
    1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49, 5: 11.07, 6: 12.59,
}

class ExtendedKalmanFilter:
    """
    2-D EKF for indoor positioning using raw per-AP range measurements.

    State:  x = [px, py, vx, vy]^T   (canvas pixels)

    Prediction (constant-velocity, linear — same structure as linear KF):
        F = [[1, 0, dt, 0],
             [0, 1,  0, dt],
             [0, 0,  1,  0],
             [0, 0,  0,  1]]

        Process noise Q — piecewise-constant white-noise-acceleration (PCWNA) model.
        For acceleration noise variance q (units: pixels²/s³ approximately):

            Q = q * [[dt⁴/4,  0,  dt³/2,  0    ],
                     [  0, dt⁴/4,  0,  dt³/2   ],
                     [dt³/2,  0,   dt²,  0      ],
                     [  0, dt³/2,  0,    dt²    ]]

    Measurement model (nonlinear):
        For AP i at canvas pixel position (ax_i, ay_i):

            h_i(x) = mpp × sqrt((px - ax_i)**2 + (py - ay_i)**2)

        where mpp = meters_per_pixel.  Output is the predicted range in metres.
        This is nonlinear because of the Euclidean distance (square root).

    Jacobian H (linearisation of h around the current state estimate):
        Row i: H_i = [ mpp·(px-ax_i)/r_i,  mpp·(py-ay_i)/r_i,  0,  0 ]
        where r_i = sqrt((px-ax_i)**2 + (py-ay_i)**2)  (pixel-space range).

        Derivation: ∂/∂px [mpp·sqrt(Δx²+Δy²)] = mpp·Δx / sqrt(Δx²+Δy²).
        Geometrically: the x-component of the unit vector from state to AP i,
        scaled by mpp.  Velocity terms are zero — range doesn't depend on vx, vy.
        An epsilon guard prevents ÷0 when the estimated position equals an AP.

    Outlier rejection (NIS gate):
        Innovation: y = z - h
        Innovation covariance: S = H P H^T + R
        NIS - Normalised Innovation Squared
        NIS = y^T S^{-1} y  ~  chi²(n) under the model
        Reject if NIS > chi²₀.₉₅(n).  Protects against multipath and
        reflected-signal outliers common in indoor WiFi.

    Covariance update — Joseph form for numerical stability:
        P⁺ = (I - KH) P⁻ (I - KH)^T + K R K^T
        Guarantees P stays positive semi-definite under floating-point errors,
        unlike the simpler P = (I - KH) P.
    """
    _EPS =1e-6      # epsilon prevents divde by zero in Jacobian when state position = AP position; replaces zero when required
    
    def __init__(self, process_noise: float = 1.0, measurement_noise: float = 2.0):
        # process_noise    : acceleration noise variance (approx. pixels²/s³)
        # measurement_noise: per-AP ranging std-dev in metres (typical WiFi uncertainty)
        self._pn = process_noise
        self._mn = measurement_noise
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None

    # ── public ────────────────────────────────────────────────────────────────

    def initialise(self, px: float, py: float) -> None:
        self._x = np.array([px, py, 0.0, 0.0])
        self._P = np.eye(4) * 500.0   # high initial uncertainty (pixels²)

    def predict(self, dt: float) -> None:
        if self._x is None:
            return

        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=float)

        q = self._pn            # process noise
        Q = q * np.array([
            [dt**4/4,       0, dt**3/2,       0],
            [      0, dt**4/4,       0, dt**3/2],
            [dt**3/2,       0,   dt**2,       0],
            [      0, dt**3/2,       0,   dt**2],
        ])

        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def update(
        self,
        measurements: List[Tuple[AccessPoint, float]],
        meters_per_pixel: float,
    ) -> bool:
        """
        Fuse raw per-AP range measurements into the state estimate.

        measurements     : list of (AccessPoint, measured_range_metres) pairs
        meters_per_pixel : scale factor converting pixel distances to metres

        Returns True if the update was accepted (NIS gate passed),
                False if the batch was rejected as an outlier.
        """
        if not measurements or self._x is None:
            return False

        n = len(measurements)
        px, py = self._x[0], self._x[1]

        h = np.zeros(n)
        H = np.zeros((n, 4))

        for i, (ap, _) in enumerate(measurements):
            dx   = px - ap.x
            dy   = py - ap.y
            r_px = max(np.hypot(dx, dy), self._EPS)   # pixel-space range

            h[i]    = meters_per_pixel * r_px          # predicted range (metres)
            H[i, 0] = meters_per_pixel * dx / r_px    # ∂h_i/∂px
            H[i, 1] = meters_per_pixel * dy / r_px    # ∂h_i/∂py
            # H[i, 2] = H[i, 3] = 0  (velocity independent of range)

        z     = np.array([r for _, r in measurements])
        y     = z - h                                  # innovation
        R     = np.eye(n) * (self._mn ** 2)
        S     = H @ self._P @ H.T + R                 # innovation covariance

        # NIS outlier gate
        S_inv     = np.linalg.inv(S)
        nis       = float(y @ S_inv @ y)
        threshold = _CHI2_95.get(n, 3.84 * n)
        if nis > threshold:
            return False   # likely multipath — discard

        K         = self._P @ H.T @ S_inv
        self._x   = self._x + K @ y

        # Joseph form
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