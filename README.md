# Indoor Navigation System

A WiFi-based indoor positioning and navigation system built as a 4th semester minor project. GPS is unreliable indoors due to signal attenuation and multipath — this system uses WiFi RSSI measurements from four fixed access points to localise a user and compute the shortest path to a destination.

---

## How It Works

### Full Pipeline

```
WiFi RSSI readings  { bssid: rssi_dbm, ... }
          │
          ▼
Log-distance path loss  →  (APᵢ, range_i [metres])  for each AP
          │
          ├─────────────────────────────────────┐
          │  EKF path (primary)                 │  KF path (baseline)
          ▼                                     ▼
Nonlinear measurement model         Weighted least-squares trilateration
h_i(x) = mpp · ‖pos − APᵢ‖         weights wᵢ = 1/dᵢ²  →  (x, y)
          │                                     │
          ▼                                     ▼
Range-proportional R                 Fixed measurement noise
σ_i = z[i] · 0.213  (heteroscedastic)  R = (100 px)²  = 5 m
          │                                     │
          ▼                                     ▼
EKF update  (Jacobian H, Joseph form)   KF update  (H = [I|0])
          │                                     │
          ▼                                     ▼
Smoothed position + uncertainty       Smoothed position + uncertainty
      (green dot)                           (blue dot)
```

### Step-by-step

1. **RSSI → Distance** — Each AP's received signal strength is converted to a range estimate using the log-distance path loss model:

   ```
   d = 10 ^ ((TxPower − RSSI) / (10 × n))
   ```

   `TxPower` is the calibrated transmit power at 1 m. `n = 2.7` is the path-loss exponent for a furnished indoor environment (free space = 2.0, heavy obstructions = 3.5). Both filters share this step.

2. **EKF (primary)** — A 4-state Extended Kalman Filter with state `[px, py, vx, vy]` fuses the raw per-AP range measurements directly. The measurement model is nonlinear:

   ```
   h_i(x) = mpp · √((px − ax_i)² + (py − ay_i)²)
   ```

   The EKF linearises this at each step via the Jacobian:

   ```
   H_i = [ mpp·(px−ax_i)/r_i,  mpp·(py−ay_i)/r_i,  0,  0 ]
   ```

   Measurement noise is **range-proportional** (heteroscedastic), derived from error propagation of the path-loss model:

   ```
   σ_range(i) = z[i] · ln(10)/(10·n) · σ_RSSI  ≈  z[i] · 0.213
   ```

   `z[i]` is the *measured* range, not the predicted range — this keeps R correct even when the filter state has drifted near an AP. Innovations are gated with a NIS chi-squared test (95th percentile threshold). If the batch fails, the single worst AP is removed and the gate is re-evaluated once. If it fails again, the update is skipped. After 1.5 s of consecutive skipped updates the filter reinitialises from the trilateration fallback.

3. **KF (baseline)** — A linear Kalman Filter first collapses the per-AP ranges to a single `(x, y)` position via weighted least-squares trilateration (`wᵢ = 1/dᵢ²`), then uses `H = [I | 0]` to update the same `[px, py, vx, vy]` state. This is the classical approach and serves as a quantitative comparison.

4. **Process model** — Both filters share a PCWNA (piecewise-constant white-noise acceleration) predict step:

   ```
   Q = q · [[dt⁴/4, dt³/2], [dt³/2, dt²]]   (per axis, block-diagonal)
   q = 300 px²/s³ = 0.75 m²/s³
   ```

   `q` gives σ_position ≈ 0.5 m after 1 second of free prediction — consistent with pedestrian dynamics at 1.5 m/s.

5. **Covariance update** — Both filters use the Joseph form:

   ```
   P⁺ = (I − KH) P⁻ (I − KH)ᵀ + K R Kᵀ
   ```

   This guarantees P remains positive-definite under floating-point accumulation, unlike the simpler `P = (I−KH)P` which can lose symmetry over long runs.

6. **Pathfinding** — The building is modelled as a weighted graph of nodes (rooms, corridor junctions) and edges. Dijkstra's algorithm finds the shortest walkable path between any two named nodes.

---

## Performance

Evaluated on a synthetic 285-step, 10 Hz trajectory (entrance → seminar hall, 28.5 s, 42.5 m) with σ_RSSI = 2.5 dBm Gaussian noise:

| Metric | EKF | KF |
|--------|-----|----|
| RMSE | **1.48 m** | 2.55 m |
| Mean error | **1.21 m** | 2.43 m |
| Max error | **4.09 m** | 4.39 m |
| Straight corridor (steps 0–114) | **1.83 m** | 2.58 m |
| After 90° turn (steps 115–284) | **1.18 m** | 2.53 m |

The EKF outperforms the KF by **42% overall**. The advantage comes from eliminating the trilateration step (which amplifies per-AP noise through geometric dilution of precision) and using range-proportional measurement noise (which correctly down-weights distant, uncertain APs).

---

## Project Structure

```
indoor-navigation-system/
├── main.py                        # FastAPI app + uvicorn entry point
├── requirements.txt
├── app/
│   ├── models.py                  # Pydantic models (camelCase JSON ↔ Python)
│   ├── dependencies.py            # Singleton services via lru_cache
│   ├── core/
│   │   ├── rssi.py                # Log-distance path loss model
│   │   ├── trilateration.py       # Weighted least-squares trilateration (NumPy)
│   │   ├── ekf.py                 # Extended Kalman Filter (range-proportional R,
│   │   │                          #   NIS gating, at-most-1-AP removal, adaptive reinit)
│   │   ├── kf.py                  # Linear Kalman Filter (trilateration input, NIS gate)
│   │   └── pathfinder.py          # Dijkstra's shortest-path
│   ├── services/
│   │   ├── map_service.py         # Loads and parses the map JSON
│   │   └── navigation_service.py  # Orchestrates localisation + navigation
│   └── routers/
│       ├── map_router.py          # GET  /api/map
│       └── nav_router.py          # POST /api/nav/localise · GET /path · /nearest · /reset
├── maps/
│   ├── sample_map.json            # Building layout — nodes, edges, APs, room rectangles
│   └── sample_trajectory.json    # Synthetic ground-truth trajectory for offline evaluation
├── tools/
│   └── generate_trajectory.py    # Standalone script to regenerate sample_trajectory.json
└── frontend/
    ├── index.html
    ├── css/style.css
    └── js/
        ├── api.js                 # Fetch wrapper for the backend API
        ├── map-renderer.js        # Canvas drawing (rooms, nodes, APs, position dots)
        └── navigation.js          # App logic, RSSI sliders, trajectory player, path display
```

---

## Getting Started

### Prerequisites

- Python 3.11+
- pip

### Installation

```bash
git clone <repo-url>
cd indoor-navigation-system
pip install -r requirements.txt
```

### Running

```bash
python main.py
```

Open **http://localhost:8000** in your browser. Interactive API docs at **http://localhost:8000/docs**.

### Regenerating the trajectory

```bash
python tools/generate_trajectory.py
```

Edit the `WAYPOINTS`, `SIGMA_RSSI_DBM`, or `RANDOM_SEED` constants at the top of the file to change the simulated path or noise level.

---

## Using the Web Interface

1. **Simulate position** — Adjust RSSI sliders (one per AP), or click anywhere on the map canvas to auto-compute RSSI from that position.
2. **Localise** — Click **Localise**. The system runs both filters and plots the EKF estimate (green) and KF estimate (blue) with uncertainty circles.
3. **Replay trajectory** — Load `sample_trajectory.json` with the trajectory player to watch both filters track the ground-truth path in real time.
4. **Navigate** — Select a destination from the dropdown and click **Get Directions**. Dijkstra's shortest path is highlighted on the map.

---

## API Reference

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/map` | Full map config (nodes, edges, APs, room rectangles, canvas size) |
| `POST` | `/api/nav/localise` | Run both filters on a new RSSI scan; returns EKF + KF positions |
| `GET` | `/api/nav/path?from=&to=` | Shortest path between two node IDs |
| `GET` | `/api/nav/nearest?x=&y=` | Nearest named node to a canvas coordinate |
| `POST` | `/api/nav/reset` | Reset both filters (start localisation fresh) |

**Localise request body:**
```json
{
  "rssi": {
    "AA:BB:CC:DD:EE:01": -65.0,
    "AA:BB:CC:DD:EE:02": -72.0,
    "AA:BB:CC:DD:EE:03": -58.0,
    "AA:BB:CC:DD:EE:04": -71.0
  },
  "session_id": "default"
}
```

`dt` is measured server-side with a monotonic clock — do not supply it from the client.

**Localise response:**
```json
{
  "ekf": { "x": 423.1, "y": 218.7, "uncertainty": 31.4 },
  "kf":  { "x": 401.6, "y": 225.3, "uncertainty": 48.2 }
}
```

`uncertainty` is √(P[0,0] + P[1,1]) in pixels (multiply by `metersPerPixel` for metres).

---

## Map Configuration

All map data lives in `maps/sample_map.json`. No code changes are needed to describe a different building.

```jsonc
{
  "buildingName": "My Building",
  "canvasWidth": 1000,
  "canvasHeight": 620,
  "metersPerPixel": 0.05,     // 1 pixel = 5 cm

  "nodes": [
    {
      "id":    "room101",
      "label": "Room 101",
      "type":  "CLASSROOM",   // CORRIDOR | CLASSROOM | OFFICE | LAB | SEMINAR | ENTRANCE | TOILET
      "x": 200, "y": 300      // canvas pixel coordinates
    }
  ],

  "edges": [
    { "from": "entrance", "to": "room101" }
    // distance defaults to Euclidean pixel distance; set explicitly when the
    // physical path bends around a wall and is longer than the straight line.
  ],

  "accessPoints": [
    {
      "id": "AP1", "ssid": "Building_WiFi_1",
      "bssid": "AA:BB:CC:DD:EE:01",
      "x": 441, "y": 5,           // canvas pixel position (mount on a wall)
      "txPower": -40,              // dBm at 1 m — calibrate per device
      "pathLossExponent": 2.7
    }
  ],

  "rooms": [
    { "id": "room101", "x": 140, "y": 5, "w": 68, "h": 170, "type": "CLASSROOM" }
  ]
}
```

**Placement tips:**
- Model corridors as a chain of junction nodes; rooms branch off to the side. This ensures Dijkstra routes through the corridor rather than cutting through walls.
- Place APs at the four corners of the floor for minimum dilution of precision.
- Mount APs on walls (not in the middle of rooms) so their pixel coordinates match the physical installation.
- Calibrate `txPower` by measuring average RSSI at exactly 1 m from each AP.

---

## Design Decisions and Trade-offs

| Decision | Choice | Alternative considered | Reason |
|----------|--------|------------------------|--------|
| Positioning | EKF on raw ranges | Fingerprinting | No offline survey; physics-based; explainable |
| Baseline | KF + trilateration | Second EKF variant | Quantifies gain from eliminating DOP step |
| Measurement noise R | Range-proportional (heteroscedastic) | Constant R | Correctly weights near APs over far APs |
| Range for R | Measured z[i] | Predicted r_m | Stable when state drifts near an AP |
| Outlier removal | At-most-1 AP removal | Unbounded removal | Prevents filter maintaining consistent-but-wrong state |
| Motion model | PCWNA | IMU-aided | No IMU; PCWNA matches pedestrian dynamics |
| Covariance update | Joseph form | Simple (I−KH)P | Numerically positive-definite over long runs |
| Pathfinding | Dijkstra | A* | Graph has ~25 nodes; complexity difference is irrelevant |
| Backend | FastAPI | Flask / Django | Auto Swagger docs; Pydantic validation; async-ready |
| Frontend | Vanilla JS + Canvas | React + WebGL | Zero build toolchain; direct pixel control |

---

## Limitations and Future Work

- **Simulated RSSI** — evaluation uses the same log-distance model as the filter; real RSSI includes correlated multipath noise not captured by the Gaussian model.
- **Static path-loss model** — does not account for specific wall attenuation or furniture. Real environments benefit from per-link calibration or fingerprinting.
- **No IMU fusion** — velocity is estimated solely from range measurements, which have low sensitivity to velocity. An accelerometer would give direct velocity observations and handle direction changes faster.
- **Single floor** — multi-floor navigation requires staircase/lift nodes and a floor-aware renderer.
- **Single session** — the backend holds one global filter. A production system would maintain per-device filters keyed by session token.
- **Android deployment** — the algorithms are lightweight enough to run entirely on-device. Replacing the Python backend with Kotlin and calling Android's `WifiManager` API would eliminate the network dependency.

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Backend | Python 3.11 · FastAPI · Uvicorn |
| Numerics | NumPy |
| Data validation | Pydantic v2 |
| Frontend | HTML · CSS · Vanilla JS · Canvas API |
