# Indoor Navigation System

A WiFi-based indoor positioning and navigation system built as a 4th semester minor project. GPS is unreliable indoors due to signal attenuation and multipath effects — this system uses WiFi RSSI measurements from multiple access points to localise a user and compute the shortest path to a destination.

---

## How It Works

### Localisation Pipeline

```
WiFi RSSI readings
      │
      ▼
Log-distance path loss model  →  distance estimates (metres)
      │
      ▼
Weighted least-squares trilateration  →  raw position (x, y)
      │
      ▼
Extended Kalman Filter  →  smoothed position + uncertainty
```

1. **RSSI → Distance** — Each access point's received signal strength is converted to an estimated distance using the log-distance path loss model:

   ```
   d = 10 ^ ((TxPower - RSSI) / (10 × n))
   ```

   where `TxPower` is the transmit power at 1 m (calibrated per AP) and `n` is the path-loss exponent (typically 2.5–3.5 indoors).

2. **Trilateration** — With distances to ≥ 3 APs of known position, the system solves a weighted least-squares system to find the (x, y) intersection. APs closer to the user are weighted more heavily since the path-loss model is more accurate at short range.

3. **Extended Kalman Filter** — A constant-velocity EKF with state `[px, py, vx, vy]` smooths the noisy trilateration output. The predict step advances the state by `dt` seconds; the update step corrects it when a new measurement arrives.

4. **Shortest Path** — The building is modelled as a weighted graph of nodes (rooms, corridor junctions) and edges. Dijkstra's algorithm finds the shortest path between any two nodes.

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
│   │   ├── ekf.py                 # Extended Kalman Filter (NumPy)
│   │   └── pathfinder.py          # Dijkstra's shortest-path
│   ├── services/
│   │   ├── map_service.py         # Loads and parses the map JSON
│   │   └── navigation_service.py  # Orchestrates localisation + navigation
│   └── routers/
│       ├── map_router.py          # GET  /api/map
│       └── nav_router.py          # POST /api/nav/localise · GET /path · /nearest
├── maps/
│   └── sample_map.json            # Building layout — edit this to change the map
└── frontend/
    ├── index.html
    ├── css/style.css
    └── js/
        ├── api.js                 # Fetch wrapper for the backend API
        ├── map-renderer.js        # Canvas drawing (nodes, edges, APs, position)
        └── navigation.js          # App logic, RSSI sliders, path display
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

Then open **http://localhost:8000** in your browser.

Interactive API docs are available at **http://localhost:8000/docs**.

---

## Using the Web Interface

1. **Simulate your position** — Adjust the RSSI sliders in the right panel (one per access point), or click anywhere on the map canvas to auto-compute RSSI values based on that position.
2. **Localise** — Click **Localise**. The system runs trilateration + EKF and places a white dot on the map showing your estimated position and uncertainty radius.
3. **Navigate** — Select a destination from the dropdown and click **Get Directions**. The shortest path is highlighted on the map.

---

## API Reference

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/map` | Full map config (nodes, edges, APs, canvas size) |
| `POST` | `/api/nav/localise` | Localise from RSSI readings, returns smoothed position |
| `GET` | `/api/nav/path?from=&to=` | Shortest path between two node IDs |
| `GET` | `/api/nav/nearest?x=&y=` | Nearest named node to a canvas coordinate |
| `POST` | `/api/nav/reset` | Reset the EKF (start localisation fresh) |

**Localise request body:**
```json
{
  "rssi": {
    "AA:BB:CC:DD:EE:01": -65,
    "AA:BB:CC:DD:EE:02": -72,
    "AA:BB:CC:DD:EE:03": -58
  },
  "dt": 1.0
}
```

---

## Configuring a Different Building

All map data lives in `maps/sample_map.json`. No code changes are needed — just replace the file (or point `nav.map.file` in `map_service.py` to a new one).

### Map JSON Format

```jsonc
{
  "buildingName": "My Building",
  "canvasWidth": 900,       // pixels — matches the frontend canvas
  "canvasHeight": 600,
  "metersPerPixel": 0.05,   // scale: 1 pixel = 5 cm

  "nodes": [
    {
      "id":    "room101",       // unique identifier used in edges and API calls
      "label": "Room 101",      // displayed on the map
      "type":  "CLASSROOM",     // CORRIDOR | CLASSROOM | OFFICE | LAB | SEMINAR | ENTRANCE | TOILET
      "x": 200,                 // canvas pixel position
      "y": 300
    }
  ],

  "edges": [
    {
      "from": "entrance",
      "to":   "room101",
      "distance": 150           // optional — defaults to Euclidean pixel distance
    }
  ],

  "accessPoints": [
    {
      "id":               "AP1",
      "ssid":             "Building_WiFi_1",
      "bssid":            "AA:BB:CC:DD:EE:01",  // used as the key in /localise requests
      "x": 200,
      "y": 150,
      "txPower":          -40,   // dBm at 1 m — calibrate per device
      "pathLossExponent":  2.7   // indoor: typically 2.5–3.5
    }
  ]
}
```

**Tips:**
- Model corridors as a chain of junction nodes connected by edges — rooms branch off to the side.
- Place at least 4 APs spread across the floor for good trilateration coverage.
- Calibrate `txPower` by measuring RSSI at exactly 1 m from each AP on the target device.
- Set `distance` on an edge explicitly when the physical path bends around a wall and is longer than the straight-line pixel distance.

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Backend | Python · FastAPI · Uvicorn |
| Math | NumPy |
| Data validation | Pydantic v2 |
| Frontend | HTML · CSS · Vanilla JS · Canvas API |

---

## Limitations & Future Work

- **Single floor only** — multi-floor navigation would require elevation data and staircase/elevator nodes.
- **Simulated RSSI** — on a real Android device, replace the sliders with live `WifiManager` scan results posted to `/api/nav/localise`.
- **Static path-loss model** — the log-distance model is a simplification; real environments benefit from a fingerprinting database or ray-tracing calibration.
- **Single EKF instance** — the backend holds one global filter. A production system would maintain a per-session filter keyed by a session token.
