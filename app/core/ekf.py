from __future__ import annotations
from typing import List, Optional, Tuple
import numpy as np
from app.models import AccessPoint, Position

# 95th-percentile chi-squared thresholds, indexed by degrees of freedom.
_CHI2_95: dict[int, float] = {
    1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49, 5: 11.07, 6: 12.59,
}

_LN10_OVER_10: float = float(np.log(10) / 10)   # ≈ 0.2303


class ExtendedKalmanFilter:
    """
    2-D EKF for indoor positioning using raw per-AP range measurements.

    State:  x = [px, py, vx, vy]^T   (canvas pixels)

    Prediction (PCWNA — piecewise-constant white-noise acceleration):
        F = [[1, 0, dt, 0],
             [0, 1,  0, dt],
             [0, 0,  1,  0],
             [0, 0,  0,  1]]

        Q = q · [[dt⁴/4,  0,  dt³/2,  0   ],
                 [  0, dt⁴/4,   0, dt³/2  ],
                 [dt³/2,  0,   dt²,  0    ],
                 [  0, dt³/2,   0,  dt²   ]]

        q = 300 px²/s³ = 0.75 m²/s³ →  σ_pos ≈ 0.5 m after 1 s free prediction.

    Measurement model (nonlinear):
        h_i(x) = mpp · ‖pos − AP_i‖  (predicted range in metres)

    Jacobian:
        H_i = [ mpp·(px−ax_i)/r_i,  mpp·(py−ay_i)/r_i,  0,  0 ]

    Measurement noise R (range-proportional, heteroscedastic):
        From error propagation of d = 10^((TxPower−RSSI)/(10n)):
            σ_range(d) = d · ln(10)/(10·n) · σ_RSSI
        R is diagonal, rebuilt each step with per-AP predicted range.
        This keeps the NIS (Normalised Innovation Squared) approximately χ²(n) regardless of AP distance,
        preventing far APs from inflating the statistic and causing spurious
        batch rejections.

    Outlier rejection — at-most-one-AP removal:
        If the batch NIS exceeds χ²₀.₉₅(n), the single worst AP (largest
        marginal standardised residual  y_i² / (H_i P H_i^T + R_ii))  is
        removed and the gate is re-evaluated once.  If the batch still fails,
        the update is skipped (returns False) and the adaptive-reinit counter
        increments toward a trilateration re-seed.

        Allowing only one removal prevents the filter from stripping multiple
        correct APs to maintain a coherent-but-wrong state after divergence.
        A single genuine multipath spike is still handled: removing one bad AP
        from a 4-AP batch almost always restores NIS consistency.

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
        process_noise  : PCWNA spectral density q [px²/s³]
                         300 px²/s³ = 0.75 m²/s³ for mpp = 0.05
        path_loss_n    : path-loss exponent (same value as in rssi.py)
        rssi_sigma_dbm : expected RSSI noise σ [dBm] used to compute per-AP R.
                         2.5 dBm is an optimistic indoor estimate; real
                         environments are typically 5–15 dBm.
        """
        self._pn = process_noise
        # Pre-compute proportionality slope: σ_range = d · _range_k
        # k = ln(10) / (10·n) · σ_RSSI  ≈ 0.0853 · 2.5 = 0.213  (≈ ±21% of range)
        self._range_k: float = _LN10_OVER_10 / path_loss_n * rssi_sigma_dbm

        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None

    # ── public ───────────────────────────────────────────────────────────────

    def initialise(self, px: float, py: float) -> None:
        self._x = np.array([px, py, 0.0, 0.0])
        self._P = np.eye(4) * 500.0

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
        Fuse raw per-AP range measurements, dropping individual outlier APs
        rather than rejecting the whole batch.

        measurements     : list of (AccessPoint, measured_range_metres) pairs
        meters_per_pixel : scale factor (metres per canvas pixel)

        Returns True if at least 2 APs survived outlier removal and the
        state was updated; False if the update was skipped entirely.
        """
        if not measurements or self._x is None:
            return False

        n  = len(measurements)
        px, py = self._x[0], self._x[1]

        # Build measured-range vector first — R is based on this, not predicted range.
        # Physical motivation: σ_range = d_true · (ln10/10n) · σ_RSSI.  The
        # measurement z_i is a noisy but unbiased estimate of d_true, so it gives
        # the correct heteroscedastic variance even when the state has drifted.
        # Using predicted range r_m would understate R for nearby-state APs when
        # the state is wrong, causing them to be wrongly flagged as outliers.
        z = np.array([r for _, r in measurements])

        h      = np.zeros(n)
        H      = np.zeros((n, 4))
        R_diag = np.zeros(n)   # diagonal of R (off-diagonal terms are zero)

        for i, (ap, _) in enumerate(measurements):
            dx   = px - ap.x
            dy   = py - ap.y
            r_px = max(np.hypot(dx, dy), self._EPS)
            r_m  = meters_per_pixel * r_px

            h[i]    = r_m
            H[i, 0] = meters_per_pixel * dx / r_px    # ∂h_i/∂px  [m/px]
            H[i, 1] = meters_per_pixel * dy / r_px    # ∂h_i/∂py  [m/px]

            sigma_i   = max(z[i] * self._range_k, 0.5)   # measured range; 0.5 m floor
            R_diag[i] = sigma_i ** 2

        y = z - h   # full innovation vector (all APs)

        # ── Greedy per-AP outlier removal (at most one AP) ───────────────────
        # Remove the single worst AP (largest marginal normalised residual) if
        # the batch NIS gate fails, then re-evaluate.  Allowing only one removal
        # handles the single-multipath-spike case without letting the filter strip
        # away multiple correct APs to maintain a consistent-but-wrong state.
        # If the batch still fails after one removal, the update is skipped and
        # the adaptive-reinitialisation counter increments toward a trilateration
        # reinit — the right recovery for genuine filter divergence.
        valid = list(range(n))

        for _pass in range(2):   # pass 0: try full batch; pass 1: try with 1 AP removed
            m = len(valid)
            if m < 2:
                return False

            Hv    = H[valid]
            Rv    = np.diag(R_diag[valid])
            yv    = y[valid]
            S     = Hv @ self._P @ Hv.T + Rv
            S_inv = np.linalg.inv(S)
            nis   = float(yv @ S_inv @ yv)

            threshold = _CHI2_95.get(m, 3.84 * m)
            if nis <= threshold:
                break   # batch is consistent — proceed to Kalman update

            if _pass == 0:
                # First failure: identify and remove the single worst AP.
                scores = [
                    y[i] ** 2 / max(H[i] @ self._P @ H[i] + R_diag[i], 1e-12)
                    for i in valid
                ]
                worst = valid[int(np.argmax(scores))]
                valid.remove(worst)
            else:
                # Second failure (after removing one AP): state is likely diverged.
                # Return False so the adaptive-reinit counter can fire.
                return False

        # ── Kalman update with surviving APs ─────────────────────────────────
        Hv   = H[valid]
        Rv   = np.diag(R_diag[valid])
        yv   = y[valid]
        S    = Hv @ self._P @ Hv.T + Rv
        S_inv = np.linalg.inv(S)

        K         = self._P @ Hv.T @ S_inv
        self._x   = self._x + K @ yv

        I_KH      = np.eye(4) - K @ Hv
        self._P   = I_KH @ self._P @ I_KH.T + K @ Rv @ K.T
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
