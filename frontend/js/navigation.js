/**
 * navigation.js — application controller.
 *
 * On load:
 *  1. Fetches map config from the backend.
 *  2. Initialises MapRenderer.
 *  3. Builds the destination dropdown and RSSI sliders.
 *  4. Wires up button handlers.
 *
 * Simulation flow:
 *  - User adjusts RSSI sliders (or clicks the canvas to auto-compute RSSI
 *    from AP distances) then clicks "Localise".
 *  - The RSSI values are POSTed to /api/nav/localise.
 *  - The returned position is drawn on the canvas and the nearest named
 *    node is shown as the current location.
 */
(async () => {

  // ── Bootstrap ─────────────────────────────────────────────────────────────
  let mapConfig  = null;
  let fromNodeId = null;   // current estimated node (set after localise)

  try {
    mapConfig = await Api.getMap();
  } catch (err) {
    document.getElementById('building-name').textContent =
      'Cannot reach backend — start the Spring Boot server on port 8080.';
    console.error(err);
    return;
  }

  document.getElementById('building-name').textContent = mapConfig.buildingName;
  document.title = mapConfig.buildingName + ' — Indoor Nav';

  const canvas = document.getElementById('map-canvas');
  MapRenderer.init(canvas, mapConfig);

  // ── Destination dropdown ──────────────────────────────────────────────────
  const destSelect = document.getElementById('dest-select');
  const navigableTypes = new Set(['CLASSROOM', 'LAB', 'SEMINAR', 'OFFICE', 'ENTRANCE', 'TOILET']);

  for (const node of mapConfig.nodes) {
    if (!navigableTypes.has(node.type)) continue;
    const opt = document.createElement('option');
    opt.value       = node.id;
    opt.textContent = node.label;
    destSelect.appendChild(opt);
  }

  // ── RSSI sliders ──────────────────────────────────────────────────────────
  const sliderContainer = document.getElementById('ap-sliders');

  for (const ap of mapConfig.accessPoints) {
    const row = document.createElement('div');
    row.className = 'ap-slider-row';

    const lbl = document.createElement('label');
    lbl.textContent = ap.id;

    const slider = document.createElement('input');
    slider.type    = 'range';
    slider.min     = -90;
    slider.max     = -30;
    slider.value   = -70;
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

  // ── Canvas click → auto-compute RSSI from click position ─────────────────
  MapRenderer.onCanvasClick((cx, cy) => {
    for (const ap of mapConfig.accessPoints) {
      const dPx   = Math.hypot(ap.x - cx, ap.y - cy);
      const dM    = dPx * mapConfig.metersPerPixel;
      const rssi  = ap.txPower - 10 * ap.pathLossExponent * Math.log10(Math.max(dM, 0.1));
      const clamped = Math.max(-90, Math.min(-30, Math.round(rssi)));

      const sliders = sliderContainer.querySelectorAll('input[type="range"]');
      for (const s of sliders) {
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
    const sliders = sliderContainer.querySelectorAll('input[type="range"]');
    for (const s of sliders) rssiMap[s.dataset.bssid] = parseFloat(s.value);

    let pos;
    try {
      pos = await Api.localise(rssiMap, 1.0);
    } catch (err) {
      alert('Localisation failed: ' + err.message);
      return;
    }

    if (!pos) {
      document.getElementById('current-location').textContent = 'Not enough APs';
      return;
    }

    MapRenderer.setPosition(pos);

    document.getElementById('position-coords').textContent =
      `Canvas: (${pos.x.toFixed(1)}, ${pos.y.toFixed(1)})  ±${pos.uncertainty.toFixed(1)} px`;

    // Find nearest named node
    const nearest = await Api.nearestNode(pos.x, pos.y);
    if (nearest) {
      fromNodeId = nearest.id;
      document.getElementById('current-location').textContent = nearest.label;
    }

    // If a destination is already chosen, refresh the path
    const destId = destSelect.value;
    if (fromNodeId && destId) await showPath(fromNodeId, destId);
  });

  // ── Navigate button ───────────────────────────────────────────────────────
  document.getElementById('btn-navigate').addEventListener('click', async () => {
    const destId = destSelect.value;
    if (!destId) return;

    if (!fromNodeId) {
      alert('Localise yourself first by adjusting the RSSI sliders and clicking Localise.');
      return;
    }

    await showPath(fromNodeId, destId);
  });

  // ── Path display helper ───────────────────────────────────────────────────
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
