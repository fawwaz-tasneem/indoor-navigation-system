/**
 * MapRenderer — draws the building floor plan on a <canvas> element.
 *
 * Responsibilities:
 *  - Render room/corridor rectangles (floor plan background)
 *  - Render nodes (colour-coded by type), edges, access-point icons
 *  - Highlight a navigation path
 *  - Draw the EKF position (white/green dot) and KF position (blue dot), each labelled
 *  - Draw a ground-truth marker (gold cross) during trajectory evaluation
 *  - Draw a 10 m scale bar
 *  - Allow clicking on the canvas to simulate a position (used by navigation.js)
 */
const MapRenderer = (() => {

  // ── colour map (AMU palette) ─────────────────────────────────────────────
  const NODE_COLOURS = {
    CORRIDOR:  '#1C3A25',
    CLASSROOM: '#2D7248',
    LAB:       '#7A1515',
    LIBRARY:   '#4A6B3A',
    SEMINAR:   '#8B6914',
    OFFICE:    '#1E5C35',
    ENTRANCE:  '#8B1A1A',
    TOILET:    '#3A5445',
    STAIRCASE: '#7A5A10'
  };

  // Semi-transparent fills for room rectangles
  const ROOM_FILL = {
    CORRIDOR:  'rgba( 28,  58,  37, 0.55)',
    CLASSROOM: 'rgba( 45, 114,  72, 0.22)',
    LAB:       'rgba(122,  21,  21, 0.22)',
    LIBRARY:   'rgba( 74, 107,  58, 0.22)',
    SEMINAR:   'rgba(139, 105,  20, 0.22)',
    OFFICE:    'rgba( 30,  92,  53, 0.22)',
    ENTRANCE:  'rgba(139,  26,  26, 0.22)',
    TOILET:    'rgba( 58,  84,  69, 0.22)',
    STAIRCASE: 'rgba(122,  90,  16, 0.22)'
  };

  const NODE_RADIUS   = 10;
  const AP_RADIUS     = 8;
  const LABEL_OFFSET  = 14;

  let canvas, ctx, mapConfig;
  let currentPath          = [];
  let currentEkfPosition   = null;   // { x, y, uncertainty }
  let currentKfPosition    = null;   // { x, y, uncertainty }
  let currentGroundTruth   = null;   // { x, y } — true position for RMSE evaluation

  let clickCallback = null;

  // ── public ──────────────────────────────────────────────────────────────

  function init(canvasEl, config) {
    canvas    = canvasEl;
    mapConfig = config;
    canvas.width  = config.canvasWidth;
    canvas.height = config.canvasHeight;
    ctx = canvas.getContext('2d');

    canvas.addEventListener('click', e => {
      const rect   = canvas.getBoundingClientRect();
      const scaleX = canvas.width  / rect.width;
      const scaleY = canvas.height / rect.height;
      const cx = (e.clientX - rect.left) * scaleX;
      const cy = (e.clientY - rect.top)  * scaleY;
      if (clickCallback) clickCallback(cx, cy);
    });

    render();
  }

  function setPath(pathNodes) {
    currentPath = pathNodes || [];
    render();
  }

  function setPosition(ekfPos, kfPos = null) {
    currentEkfPosition = ekfPos;
    currentKfPosition  = kfPos;
    render();
  }

  function setGroundTruth(x, y) {
    currentGroundTruth = (x !== null && y !== null) ? { x, y } : null;
    render();
  }

  function onCanvasClick(fn) {
    clickCallback = fn;
  }

  // ── drawing ──────────────────────────────────────────────────────────────

  function render() {
    if (!ctx) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    drawRooms();          // filled room rectangles — background layer
    drawEdges();          // corridor / room-connection lines (pathfinding graph)
    drawPathEdges();      // highlighted active route
    drawNodes();
    drawAccessPoints();
    if (currentGroundTruth)  drawGroundTruth();
    if (currentKfPosition)   drawKfPosition();
    if (currentEkfPosition)  drawEkfPosition();
    drawScaleBar();       // always on top
  }

  // ── room rectangles ──────────────────────────────────────────────────────

  function drawRooms() {
    if (!mapConfig || !mapConfig.rooms || mapConfig.rooms.length === 0) return;
    for (const room of mapConfig.rooms) {
      ctx.fillStyle   = ROOM_FILL[room.type] || 'rgba(100,100,100,0.18)';
      ctx.strokeStyle = 'rgba(200,200,200,0.18)';
      ctx.lineWidth   = 1;
      ctx.fillRect(room.x, room.y, room.w, room.h);
      ctx.strokeRect(room.x, room.y, room.w, room.h);
    }
  }

  // ── scale bar ────────────────────────────────────────────────────────────

  function drawScaleBar() {
    if (!mapConfig) return;
    const barMetres = 10;
    const barPx     = barMetres / mapConfig.metersPerPixel;   // 10 / 0.05 = 200 px
    const x  = 14;
    const y  = canvas.height - 14;

    ctx.strokeStyle = '#EDE8DC';
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.moveTo(x,          y);  ctx.lineTo(x + barPx, y);      // main bar
    ctx.moveTo(x,          y - 4); ctx.lineTo(x,         y + 4); // left tick
    ctx.moveTo(x + barPx,  y - 4); ctx.lineTo(x + barPx, y + 4); // right tick
    ctx.stroke();

    ctx.font      = '11px Segoe UI, sans-serif';
    ctx.fillStyle = '#EDE8DC';
    ctx.textAlign = 'center';
    ctx.fillText(`${barMetres} m`, x + barPx / 2, y - 7);
  }

  // ── graph edges ──────────────────────────────────────────────────────────

  function drawEdges() {
    if (!mapConfig) return;
    const nodeMap = buildNodeMap();

    ctx.strokeStyle = 'rgba(45,114,72,0.45)';
    ctx.lineWidth   = 1.5;

    for (const edge of mapConfig.edges) {
      const a = nodeMap[edge.from];
      const b = nodeMap[edge.to];
      if (!a || !b) continue;
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
  }

  function drawPathEdges() {
    if (currentPath.length < 2) return;

    ctx.strokeStyle = '#C8961A';
    ctx.lineWidth   = 4;
    ctx.setLineDash([8, 4]);

    ctx.beginPath();
    ctx.moveTo(currentPath[0].x, currentPath[0].y);
    for (let i = 1; i < currentPath.length; i++) {
      ctx.lineTo(currentPath[i].x, currentPath[i].y);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // ── nodes & APs ─────────────────────────────────────────────────────────

  function drawNodes() {
    if (!mapConfig) return;
    const pathIds = new Set(currentPath.map(n => n.id));

    for (const node of mapConfig.nodes) {
      const colour = NODE_COLOURS[node.type] || '#888';
      const onPath = pathIds.has(node.id);

      if (onPath) {
        ctx.beginPath();
        ctx.arc(node.x, node.y, NODE_RADIUS + 5, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(200,150,26,0.22)';
        ctx.fill();
      }

      ctx.beginPath();
      ctx.arc(node.x, node.y, NODE_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle   = colour;
      ctx.strokeStyle = onPath ? '#C8961A' : '#060D07';
      ctx.lineWidth   = onPath ? 2.5 : 1.5;
      ctx.fill();
      ctx.stroke();

      if (node.type !== 'CORRIDOR') {
        ctx.font      = '11px Segoe UI, sans-serif';
        ctx.fillStyle = onPath ? '#C8961A' : '#EDE8DC';
        ctx.textAlign = 'center';
        ctx.fillText(node.label, node.x, node.y + NODE_RADIUS + LABEL_OFFSET);
      }
    }
  }

  function drawAccessPoints() {
    if (!mapConfig) return;
    for (const ap of mapConfig.accessPoints) {
      ctx.save();
      ctx.translate(ap.x, ap.y);
      ctx.rotate(Math.PI / 4);
      ctx.beginPath();
      ctx.rect(-AP_RADIUS, -AP_RADIUS, AP_RADIUS * 2, AP_RADIUS * 2);
      ctx.fillStyle   = '#C8961A';
      ctx.strokeStyle = '#060D07';
      ctx.lineWidth   = 1.5;
      ctx.fill();
      ctx.stroke();
      ctx.restore();

      ctx.font      = '10px Segoe UI, sans-serif';
      ctx.fillStyle = '#C8961A';
      ctx.textAlign = 'center';
      ctx.fillText(ap.id, ap.x, ap.y + AP_RADIUS + 14);
    }
  }

  // ── position markers ─────────────────────────────────────────────────────

  function drawEkfPosition() {
    const { x, y, uncertainty } = currentEkfPosition;

    if (uncertainty > 0) {
      ctx.beginPath();
      ctx.arc(x, y, uncertainty, 0, Math.PI * 2);
      ctx.fillStyle   = 'rgba(45,114,72,0.12)';
      ctx.strokeStyle = 'rgba(45,114,72,0.40)';
      ctx.lineWidth   = 1;
      ctx.fill();
      ctx.stroke();
    }

    // Outer ring
    ctx.beginPath();
    ctx.arc(x, y, 16, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(61,158,104,0.45)';
    ctx.lineWidth   = 1.5;
    ctx.stroke();

    // Dot
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, Math.PI * 2);
    ctx.fillStyle   = '#ffffff';
    ctx.strokeStyle = '#3D9E68';
    ctx.lineWidth   = 3;
    ctx.fill();
    ctx.stroke();

    // Label
    ctx.font      = 'bold 10px Segoe UI, sans-serif';
    ctx.fillStyle = '#3D9E68';
    ctx.textAlign = 'center';
    ctx.fillText('EKF', x, y - 20);
  }

  function drawKfPosition() {
    const { x, y, uncertainty } = currentKfPosition;

    if (uncertainty > 0) {
      ctx.beginPath();
      ctx.arc(x, y, uncertainty, 0, Math.PI * 2);
      ctx.fillStyle   = 'rgba(100,149,237,0.10)';
      ctx.strokeStyle = 'rgba(100,149,237,0.30)';
      ctx.lineWidth   = 1;
      ctx.fill();
      ctx.stroke();
    }

    // Dot
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, Math.PI * 2);
    ctx.fillStyle   = '#6495ED';
    ctx.strokeStyle = '#1a3a6e';
    ctx.lineWidth   = 2;
    ctx.fill();
    ctx.stroke();

    // Label
    ctx.font      = 'bold 10px Segoe UI, sans-serif';
    ctx.fillStyle = '#6495ED';
    ctx.textAlign = 'center';
    ctx.fillText('KF', x, y - 12);
  }

  function drawGroundTruth() {
    const { x, y } = currentGroundTruth;

    // Gold cross-hair marker
    ctx.strokeStyle = '#C8961A';
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(x - 9, y); ctx.lineTo(x + 9, y);
    ctx.moveTo(x, y - 9); ctx.lineTo(x, y + 9);
    ctx.stroke();

    ctx.font      = 'bold 9px Segoe UI, sans-serif';
    ctx.fillStyle = '#C8961A';
    ctx.textAlign = 'center';
    ctx.fillText('GT', x, y - 12);
  }

  // ── helpers ──────────────────────────────────────────────────────────────

  function buildNodeMap() {
    const m = {};
    if (mapConfig) for (const n of mapConfig.nodes) m[n.id] = n;
    return m;
  }

  return { init, setPath, setPosition, setGroundTruth, onCanvasClick, render };
})();
