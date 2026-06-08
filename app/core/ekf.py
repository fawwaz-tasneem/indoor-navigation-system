from __future__ import annotations
from typing import List, Optional, Tuple
import numpy as np
from app.models import AccessPoint, Position

# 95th-percentile chi-squared thresholds, indexed by degrees of freedom.
# NIS statistic for n measurements ~ chi²(n) under the filter model.
_CHI2_95: dict[int, float] = {
    1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49, 5: 11.07, 6: 12.59,
}

# Used in RSSI → range error propagation (see _range_k below).
_LN10_OVER_10: float = float(np.log(10) / 10)   # ≈ 0.2303


class ExtendedKalmanFilter:
    """
    2-D EKF for indoor positioning using raw per-AP range measurements.

    State:  x = [px, py, vx, vy]^T   (canvas pixels)

    Prediction (constant-velocity, PCWNA process noise):
        F = [[1, 0, dt, 0],
             [0, 1,  0, dt],
             [0, 0,  1,  0],
             [0, 0,  0,  1]]

        Q = q · [[dt⁴/4,  0,  dt³/2,  0   ],
                 [  0, dt⁴/4,   0, dt³/2  ],
                 [dt³/2,  0,   dt²,  0    ],
                 [  0, dt³/2,   0,  dt²   ]]

        q = 300 px²/s³ = 0.75 m²/s³ → σ_position ≈ 0.5 m after 1 s free prediction,
        consistent with indoor pedestrian acceleration variability.

    Measurement model (nonlinear):
        h_i(x) = mpp × ‖pos − AP_i‖  (predicted range in metres)

    Jacobian:
        H_i = [ mpp·(px−ax_i)/r_i,  mpp·(py−ay_i)/r_i,  0,  0 ]

    Measurement noise R (range-proportional, heteroscedastic):
        The RSSI → range conversion d = 10^((TxPower−RSSI)/(10n)) implies:
            σ_range = d · ln(10)/(10·n) · σ_RSSI   [metres]

        R is therefore built fresh at each update step from the predicted range
        for each AP:
            R[i,i] = (r_i · k)²    where  k = ln(10)/(10·n) · σ_RSSI

        This prevents far APs (large d, large σ_range) from inflating the NIS
        statistic and triggering spurious outlier rejections.

    NIS gating:
        y = z − h,   S = H P Hᵀ + R,   NIS = yᵀ S⁻¹ y ~ χ²(n)
        Reject if NIS > χ²₀.₉₅(n).

    Covariance update — Joseph form:
        P⁺ = (I−KH) P⁻ (I−KH)ᵀ + K R Kᵀ
    """

    _EPS = 1e-6

    def __init__(
        self,
        process_noise:  float = 300.0,
        path_loss_n:    float = 2.7,
        rssi_sigma_dbm: float = 2.5,
    ) -> None:
        """
        process_noise  : PCWNA spectral density q  [px²/s³]
                         300 px²/s³ = 0.75 m²/s³ for mpp = 0.05
        path_loss_n    : path-loss exponent n  (same value used in rssi.py)
        rssi_sigma_dbm : expected RSSI measurement noise std-dev [dBm]
                         2.5 dBm is a realistic indoor best-case; real
                         environments are typically 5–15 dBm.
        """
        self._pn = process_noise
        # Pre-compute proportionality slope: σ_range = d · _range_k
        # k = ln(10)/(10·n) · σ_RSSI  ≈ 0.0853 · 2.5 = 0.213  (≈ ±21 % of distance)
        self._range_k: float = _LN10_OVER_10 / path_loss_n * rssi_sigma_dbm

        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None

    # ── public ───────────────────────────────────────────────────────────────

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

        q = self._pn
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
        meters_per_pixel : scale factor (metres per canvas pixel)

        Returns True if accepted (NIS gate passed), False if rejected.
        """
        if not measurements or self._x is None:
            return False

        n  = len(measurements)
        px, py = self._x[0], self._x[1]

        h = np.zeros(n)
        H = np.zeros((n, 4))
        R = np.zeros((n, n))

        for i, (ap, _) in enumerate(measurements):
            dx   = px - ap.x
            dy   = py - ap.y
            r_px = max(np.hypot(dx, dy), self._EPS)  # pixel-space range
            r_m  = meters_per_pixel * r_px             # predicted range (metres)

            h[i]    = r_m
            H[i, 0] = meters_per_pixel * dx / r_px    # ∂h_i/∂px  [m/px]
            H[i, 1] = meters_per_pixel * dy / r_px    # ∂h_i/∂py  [m/px]
            # H[i,2] = H[i,3] = 0  (range independent of velocity)

            # Range-proportional noise: σ_range(d) = d · k
            # Ensures far APs (noisy) contribute less relative weight than close APs.
            sigma_i = max(r_m * self._range_k, 0.5)   # 0.5 m floor
            R[i, i] = sigma_i ** 2

        z   = np.array([r for _, r in measurements])
        y   = z - h                                    # innovation
        S   = H @ self._P @ H.T + R                   # innovation covariance

        # NIS outlier gate
        S_inv     = np.linalg.inv(S)
        nis       = float(y @ S_inv @ y)
        threshold = _CHI2_95.get(n, 3.84 * n)
        if nis > threshold:
            return False

        K         = self._P @ H.T @ S_inv
        self._x   = self._x + K @ y

        # Joseph form covariance update
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
