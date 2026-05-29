from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from app.models import AccessPoint, Position


@dataclass
class Measurement:
    ap: AccessPoint
    distance_meters: float


def solve(measurements: List[Measurement], meters_per_pixel: float) -> Optional[Position]:
    """
    Weighted least-squares trilateration from ≥3 AP measurements.

    Each AP gives one circle equation:
        (x - xi)^2 + (y - yi)^2 = di^2

    Subtracting the last equation from each of the others gives a linear system
    A * pos = b, solved via normal equations: pos = (A^T W A)^-1 A^T W b
    where W weights closer APs more (w_i = 1 / d_i^2).

    All computation is in metres; result is converted back to canvas pixels.
    """
    n = len(measurements)
    if n < 3:
        return None

    # AP positions in metres
    positions = np.array(
        [[m.ap.x * meters_per_pixel, m.ap.y * meters_per_pixel] for m in measurements],
        dtype=float,
    )
    distances = np.array([m.distance_meters for m in measurements], dtype=float)

    ref_pos  = positions[-1]
    ref_dist = distances[-1]

    # Linearised system (subtract reference row)
    A = 2.0 * (positions[:-1] - ref_pos)                         # (n-1, 2)
    b = (
        np.sum(positions[:-1] ** 2, axis=1)
        - np.sum(ref_pos ** 2)
        + ref_dist ** 2
        - distances[:-1] ** 2
    )                                                              # (n-1,)

    # Diagonal weight matrix
    W = np.diag(1.0 / np.maximum(distances[:-1] ** 2, 0.01))     # (n-1, n-1)

    AtWA = A.T @ W @ A   # (2, 2)
    AtWb = A.T @ W @ b   # (2,)

    try:
        pos_m = np.linalg.solve(AtWA, AtWb)   # position in metres
    except np.linalg.LinAlgError:
        return None

    # RMS residual → uncertainty in pixels
    diffs    = np.linalg.norm(positions - pos_m, axis=1) - distances
    residual = float(np.sqrt(np.mean(diffs ** 2)))

    return Position(
        x=float(pos_m[0] / meters_per_pixel),
        y=float(pos_m[1] / meters_per_pixel),
        uncertainty=residual / meters_per_pixel,
    )
