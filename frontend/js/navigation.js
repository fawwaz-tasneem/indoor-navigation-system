/**
 * navigation.js — application controller.
 *
 * On load:
 *  1. Fetches map config from the backend.
 *  2. Initialises MapRenderer.
 *  3. Builds the destination dropdown and RSSI sliders.
 *  4. Wires up button handlers.
 *
 * Interactive mode:
 *  - User clicks the map canvas or adjusts RSSI sliders, then clicks "Localise".
 *  - If the position was set by a canvas click, the click coordinates are stored as
 *    ground truth and per-call error (metres) is shown in the Filter Comparison card.
 *
 * Trajectory replay mode:
 *  - Loads maps/sample_trajectory.json (served by FastAPI under /maps/).
 *  - Steps through each recorded RSSI snapshot at the trajectory's interval_ms cadence.
 *  - Displays a gold GT marker for the true position at each step.
 *  - Accumulates per-filter errors and computes running RMSE displayed live.
 */
(async () => {

  // ── Bootstrap ─────────────────────────────────────────────────────────────
  let mapConfig  = null;
  let fromNodeId = null;

  // Ground-truth for interactive mode (set when user clicks the canvas)
  let lastGtX = null, lastGtY = null;

  // Trajectory replay state
  let trajectoryData    = null;
  let trajectoryPlaying = false;
  let trajectoryTimer   = null;
  let trajEkfErrors     = [];
  let trajKfErrors      = [];

  try {
    mapConfig = await Api.getMap();
  } catch (err) {
    document.getElementById('building-name').textContent =
      'Cannot reach backend — start the server: python main.py (port 8000).';
    console.error(err);
    return;
  }

  document.getElementById('building-name').textContent = mapConfig.buildingName;
  document.title = mapConfig.buildingName + ' — Indoor Nav';

  const canvas = document.getElementById('map-canvas');
  MapRenderer.init(canvas, mapConfig);

  // ── Destination dropdown ──────────────────────────────────────────────────
  const destSelect     = document.getElementById('dest-select');
  const sliderContainer = document.getElementById('ap-sliders');
  const navigableTypes = new Set(['CLASSROOM', 'LAB', 'SEMINAR', 'OFFICE', 'ENTRANCE', 'TOILET']);

  for (const node of mapConfig.nodes) {
    if (!navigableTypes.has(node.type)) continue;
    const opt = document.createElement('option');
    opt.value       = node.id;
    opt.textContent = node.label;
    destSelect.appendChild(opt);
  }

  // ── RSSI sliders ──────────────────────────────────────────────────────────
  for (const ap of mapConfig.accessPoints) {
    const row = document.createElement('div');
    row.className = 'ap-slider-row';

    const lbl = document.createElement('label');
    lbl.textContent = ap.id;

    const slider = document.createElement('input');
    slider.type          = 'range';
    slider.min           = -90;
    slider.max           = -30;
    slider.value         = -70;
    slider.dataset.bssid = ap.bssid;

    const val = document.createElement('span');
    val.className   = 'rssi-val';
    val.textContent = slider.value + ' dBm';

    slider.addEventListener('input', () => {
      val.textContent = slider.value + ' dBm';
    });

    row.appendChild(lbl);
    row.appendChild(slider);
    row.appendChild(val);
    sliderContainer.appendChild(row);
  }

  // ── Canvas click → auto-compute RSSI + store ground truth ─────────────────
  MapRenderer.onCanvasClick((cx, cy) => {
    lastGtX = cx;
    lastGtY = cy;
    MapRenderer.setGroundTruth(cx, cy);

    for (const ap of mapConfig.accessPoints) {
      const dPx     = Math.hypot(ap.x - cx, ap.y - cy);
      const dM      = dPx * mapConfig.metersPerPixel;
      const rssi    = ap.txPower - 10 * ap.pathLossExponent * Math.log10(Math.max(dM, 0.1));
      const clamped = Math.max(-90, Math.min(-30, Math.round(rssi)));

      for (const s of sliderContainer.querySelectorAll('input[type="range"]')) {
        if (s.dataset.bssid === ap.bssid) {
          s.value = clamped;
          s.nextElementSibling.textContent = clamped + ' dBm';
        }
      }
    }
  });

  // ── Localise button ───────────────────────────────────────────────────────
  document.getElementById('btn-localise').addEventListener('click', async () => {
    const rssiMap = {};
    for (const s of sliderContainer.querySelectorAll('input[type="range"]')) {
      rssiMap[s.dataset.bssid] = parseFloat(s.value);
    }

    let result;
    try {
      result = await Api.localise(rssiMap);
    } catch (err) {
      alert('Localisation failed: ' + err.message);
      return;
    }

    if (!result || (!result.ekf && !result.kf)) {
      document.getElementById('current-location').textContent = 'Not enough APs';
      return;
    }

    const primaryPos = result.ekf || result.kf;
    MapRenderer.setPosition(result.ekf, result.kf);
    updatePositionDisplay(result);

    // Show per-call error only when ground truth is available (after a canvas click)
    if (lastGtX !== null) showInteractiveError(result);

    const nearest = await Api.nearestNode(primaryPos.x, primaryPos.y);
    if (nearest) {
      fromNodeId = nearest.id;
      document.getElementById('current-location').textContent = nearest.label;
    }

    const destId = destSelect.value;
    if (fromNodeId && destId) await showPath(fromNodeId, destId);
  });

  // ── Navigate button ───────────────────────────────────────────────────────
  document.getElementById('btn-navigate').addEventListener('click', async () => {
    const destId = destSelect.value;
    if (!destId) return;
    if (!fromNodeId) { alert('Localise yourself first.'); return; }
    await showPath(fromNodeId, destId);
  });

  // ── Trajectory Play / Stop ────────────────────────────────────────────────
  document.getElementById('btn-play').addEventListener('click', playTrajectory);
  document.getElementById('btn-stop').addEventListener('click', () => stopTrajectory(false));

  // ── Helpers ───────────────────────────────────────────────────────────────

  function updatePositionDisplay(result) {
    const mpp     = mapConfig.metersPerPixel;
    const ekfText = result.ekf
      ? `EKF (${result.ekf.x.toFixed(1)}, ${result.ekf.y.toFixed(1)}) ±${(result.ekf.uncertainty * mpp).toFixed(1)} m`
      : 'EKF: N/A';
    const kfText  = result.kf
      ? `KF (${result.kf.x.toFixed(1)}, ${result.kf.y.toFixed(1)}) ±${(result.kf.uncertainty * mpp).toFixed(1)} m`
      : 'KF: N/A';
    document.getElementById('position-coords').textContent = `${ekfText}   |   ${kfText}`;
  }

  function showInteractiveError(result) {
    const mpp = mapConfig.metersPerPixel;
    if (result.ekf) {
      const e = Math.hypot(result.ekf.x - lastGtX, result.ekf.y - lastGtY) * mpp;
      document.getElementById('ekf-error').textContent = e.toFixed(2) + ' m';
    }
    if (result.kf) {
      const e = Math.hypot(result.kf.x - lastGtX, result.kf.y - lastGtY) * mpp;
      document.getElementById('kf-error').textContent = e.toFixed(2) + ' m';
    }
  }

  async function playTrajectory() {
    if (trajectoryPlaying) return;

    if (!trajectoryData) {
      try {
        const res = await fetch('/maps/sample_trajectory.json');
        if (!res.ok) throw new Error(res.statusText);
        trajectoryData = await res.json();
      } catch (err) {
        alert('Could not load trajectory: ' + err.message);
        return;
      }
    }

    // Reset filter state and accumulators
    await fetch('/api/nav/reset', { method: 'POST' });
    lastGtX = null; lastGtY = null;
    trajEkfErrors = [];
    trajKfErrors  = [];
    document.getElementById('ekf-error').textContent = '—';
    document.getElementById('kf-error').textContent  = '—';
    document.getElementById('ekf-rmse').textContent  = '—';
    document.getElementById('kf-rmse').textContent   = '—';

    trajectoryPlaying = true;
    document.getElementById('btn-play').disabled = true;
    document.getElementById('btn-stop').disabled = false;
    document.getElementById('traj-progress').style.display = 'block';
    document.getElementById('traj-total').textContent = trajectoryData.steps.length;

    let i = 0;

    async function runStep() {
      if (!trajectoryPlaying || i >= trajectoryData.steps.length) {
        stopTrajectory(i >= trajectoryData.steps.length);
        return;
      }

      const s = trajectoryData.steps[i];
      document.getElementById('traj-step').textContent = i + 1;

      // Show true position on the map
      MapRenderer.setGroundTruth(s.true_x, s.true_y);

      const result = await Api.localise(s.rssi);
      if (result) {
        MapRenderer.setPosition(result.ekf, result.kf);
        updatePositionDisplay(result);

        const mpp = mapConfig.metersPerPixel;
        if (result.ekf) {
          const err = Math.hypot(result.ekf.x - s.true_x, result.ekf.y - s.true_y) * mpp;
          trajEkfErrors.push(err);
          document.getElementById('ekf-error').textContent = err.toFixed(2) + ' m';
        }
        if (result.kf) {
          const err = Math.hypot(result.kf.x - s.true_x, result.kf.y - s.true_y) * mpp;
          trajKfErrors.push(err);
          document.getElementById('kf-error').textContent = err.toFixed(2) + ' m';
        }
        updateRmseDisplay();
      }

      i++;
      trajectoryTimer = setTimeout(runStep, trajectoryData.interval_ms || 500);
    }

    runStep();
  }

  function stopTrajectory(finished) {
    trajectoryPlaying = false;
    clearTimeout(trajectoryTimer);
    document.getElementById('btn-play').disabled = false;
    document.getElementById('btn-stop').disabled = true;
    if (finished) {
      updateRmseDisplay();
      const ekfFinal = document.getElementById('ekf-rmse').textContent;
      const kfFinal  = document.getElementById('kf-rmse').textContent;
      document.getElementById('traj-step').textContent =
        `Done — RMSE: EKF ${ekfFinal}  KF ${kfFinal}`;
    }
  }

  function updateRmseDisplay() {
    if (trajEkfErrors.length > 0) {
      const rmse = Math.sqrt(trajEkfErrors.reduce((s, e) => s + e * e, 0) / trajEkfErrors.length);
      document.getElementById('ekf-rmse').textContent = rmse.toFixed(2) + ' m';
    }
    if (trajKfErrors.length > 0) {
      const rmse = Math.sqrt(trajKfErrors.reduce((s, e) => s + e * e, 0) / trajKfErrors.length);
      document.getElementById('kf-rmse').textContent = rmse.toFixed(2) + ' m';
    }
  }

  async function showPath(from, to) {
    let path;
    try {
      path = await Api.getPath(from, to);
    } catch (err) {
      alert('Path request failed: ' + err.message);
      return;
    }

    MapRenderer.setPath(path);

    const pathInfo  = document.getElementById('path-info');
    const pathSteps = document.getElementById('path-steps');

    if (!path || path.length === 0) {
      pathInfo.classList.remove('hidden');
      pathSteps.textContent = 'No path found.';
      return;
    }

    pathInfo.classList.remove('hidden');
    const namedStops = path.filter(n => n.type !== 'CORRIDOR');
    pathSteps.textContent = namedStops.map(n => n.label).join(' → ');
  }

})();
