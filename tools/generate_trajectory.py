"""
generate_trajectory.py
======================
Generates a synthetic ground-truth trajectory for offline RMSE evaluation
of the indoor navigation filters (EKF and KF).

Usage
-----
    python tools/generate_trajectory.py

Output
------
    maps/sample_trajectory.json

How it works
------------
1.  A sequence of waypoints is defined manually — these represent named graph
    nodes the simulated pedestrian visits in order.

2.  The waypoints are interpolated at a fixed step size (STEP_PX pixels) to
    produce a dense list of (x, y) ground-truth positions.  The step size is
    chosen so that consecutive positions are STEP_PX / (canvas_px_per_metre)
    metres apart in the physical world.  At 1.5 m/s walking speed:

        dt  =  STEP_PX [px]  /  (walking_speed [m/s] / mpp [m/px])
            =  STEP_PX * mpp / walking_speed
            =  3 * 0.05 / 1.5  =  0.1 s  →  10 Hz update rate

3.  At each position, RSSI is simulated for every access point using the
    log-distance path-loss model:

        RSSI_clean  =  TxPower  −  10 · n · log10(d_m)

    where d_m is the 2-D Euclidean distance in metres and n is the path-loss
    exponent (set per AP in the map JSON).

4.  Independent Gaussian noise with zero mean and standard deviation
    SIGMA_RSSI_DBM is added to each RSSI value, then clamped to the
    detectable range [−90, −30] dBm to mimic hardware sensitivity limits.

5.  The output JSON has the format expected by the frontend trajectory player:

        {
          "steps": [
            {
              "t_ms":   0,
              "true_x": 800.0,
              "true_y": 560.0,
              "rssi": {
                "AA:BB:CC:DD:EE:01": -65.3,
                ...
              }
            },
            ...
          ]
        }

Parameters (edit this section to change the simulation)
---------------------------------------------------------
WAYPOINTS       list of (x, y) canvas pixel coordinates — the path nodes
STEP_PX         pixels between consecutive ground-truth positions (3 px = 15 cm)
SIGMA_RSSI_DBM  Gaussian RSSI noise sigma; 2.5 dBm is an optimistic indoor
                value (real environments: 5-15 dBm due to multipath / furniture)
RANDOM_SEED     fixed seed for reproducibility
"""

import json
import math
import os
import random
import sys
from pathlib import Path

# ── Parameters ────────────────────────────────────────────────────────────────

# Ground-truth waypoints (canvas pixel coordinates).
# These match the graph nodes in sample_map.json for the path:
#   entrance → v2 → v1 → corner → h4 → h3 → seminar
WAYPOINTS: list[tuple[float, float]] = [
    (800, 560),   # entrance
    (800, 480),   # v2  (vertical corridor, lower)
    (800, 360),   # v1  (vertical corridor, upper)
    (800, 220),   # corner (corridor junction)
    (590, 220),   # h4
    (420, 220),   # h3
    (420,  90),   # seminar hall
]

STEP_PX         = 3      # pixels per ground-truth step (15 cm at mpp = 0.05)
SIGMA_RSSI_DBM  = 2.5    # dBm — RSSI noise standard deviation
RANDOM_SEED     = 42     # reproducibility

# ── Paths ─────────────────────────────────────────────────────────────────────

REPO_ROOT   = Path(__file__).resolve().parent.parent
MAP_FILE    = REPO_ROOT / "maps" / "sample_map.json"
OUTPUT_FILE = REPO_ROOT / "maps" / "sample_trajectory.json"


# ── Helpers ───────────────────────────────────────────────────────────────────

def interpolate_path(waypoints: list[tuple[float, float]], step_px: float) -> list[tuple[float, float]]:
    """
    Linearly interpolate a list of waypoints at a fixed step size.

    Each segment is divided into N sub-steps where N = ceil(segment_length / step_px).
    The final waypoint is always included.
    """
    path: list[tuple[float, float]] = []

    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        n_steps = max(1, round(seg_len / step_px))

        for k in range(n_steps):
            t = k / n_steps
            path.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))

    path.append(waypoints[-1])  # include the final destination
    return path


def rssi_for_ap(tx: float, ty: float, ap: dict, mpp: float, sigma: float) -> float:
    """
    Simulate a noisy RSSI reading for one access point.

    Physics
    -------
        d_m         = Euclidean distance (metres) from (tx, ty) to the AP
        RSSI_clean  = TxPower − 10·n·log10(d_m)   [log-distance path-loss]
        RSSI_noisy  = RSSI_clean + N(0, sigma)
        RSSI_out    = clamp(RSSI_noisy, −90, −30)  [hardware sensitivity limits]
    """
    d_px = max(math.hypot(tx - ap["x"], ty - ap["y"]), 0.5)  # avoid log(0)
    d_m  = d_px * mpp
    rssi_clean = ap["txPower"] - 10.0 * ap["pathLossExponent"] * math.log10(d_m)
    rssi_noisy = rssi_clean + random.gauss(0.0, sigma)
    return round(max(-90.0, min(-30.0, rssi_noisy)), 1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    random.seed(RANDOM_SEED)

    # Load map for AP positions and scale factor
    with open(MAP_FILE) as f:
        cfg = json.load(f)

    mpp = cfg["metersPerPixel"]          # metres per canvas pixel
    aps = cfg["accessPoints"]            # list of AP dicts

    # Walking speed check: STEP_PX * mpp / dt = v_walk
    # With dt = STEP_PX * mpp / v_walk:
    v_walk_mps = 1.5                     # m/s  (typical indoor walking)
    dt_s       = STEP_PX * mpp / v_walk_mps
    dt_ms      = round(dt_s * 1000)     # ms between consecutive steps

    # Build dense ground-truth path
    path = interpolate_path(WAYPOINTS, STEP_PX)
    total_dist_m = sum(
        math.hypot(WAYPOINTS[i+1][0] - WAYPOINTS[i][0],
                   WAYPOINTS[i+1][1] - WAYPOINTS[i][1])
        for i in range(len(WAYPOINTS) - 1)
    ) * mpp

    # Build step records
    steps = []
    for k, (tx, ty) in enumerate(path):
        rssi = {ap["bssid"]: rssi_for_ap(tx, ty, ap, mpp, SIGMA_RSSI_DBM) for ap in aps}
        steps.append({
            "t_ms":   k * dt_ms,
            "true_x": round(tx, 1),
            "true_y": round(ty, 1),
            "rssi":   rssi,
        })

    trajectory = {
        "name":        "Entrance to Seminar Hall",
        "description": (
            f"Synthetic {1000//dt_ms} Hz trajectory along the main corridor. "
            f"Walking speed {v_walk_mps} m/s, {STEP_PX} px step, "
            f"sigma_RSSI={SIGMA_RSSI_DBM} dBm, seed={RANDOM_SEED}. "
            f"APs: {', '.join(ap['id'] + '(' + str(ap['x']) + ',' + str(ap['y']) + ')' for ap in aps)}."
        ),
        "interval_ms": dt_ms,
        "steps":       steps,
    }

    with open(OUTPUT_FILE, "w") as f:
        json.dump(trajectory, f, indent=2)

    print(f"Generated {len(steps)} steps  |  {len(steps)*dt_ms/1000:.1f} s  |  {total_dist_m:.1f} m  |  {dt_ms} ms interval")
    print(f"APs: {[(ap['id'], ap['x'], ap['y']) for ap in aps]}")
    print(f"Saved: {OUTPUT_FILE.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
