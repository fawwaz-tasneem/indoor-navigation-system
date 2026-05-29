/**
 * MapRenderer — draws the building graph on a <canvas> element.
 *
 * Responsibilities:
 *  - Render nodes (colour-coded by type), edges, access-point icons
 *  - Highlight a navigation path
 *  - Draw the current estimated position + uncertainty circle
 *  - Allow clicking on the canvas to simulate a position (used by navigation.js)
 */
const MapRenderer = (() => {

  // ── colour map ──────────────────────────────────────────────────────────
  const NODE_COLOURS = {
    CORRIDOR:  '#4a5568',
    CLASSROOM: '#4f8ef7',
    LAB:       '#7c5cbf',
    SEMINAR:   '#f7c94f',
    OFFICE:    '#3ecf8e',
    ENTRANCE:  '#f76b4f',
    TOILET:    '#8892a4',
    STAIRCASE: '#e88c2a'
  };

  const NODE_RADIUS   = 10;
  const AP_RADIUS     = 8;
  const LABEL_OFFSET  = 14;

  let canvas, ctx, mapConfig;
  let currentPath     = [];   // array of NavNode
  let currentPosition = null; // { x, y, uncertainty }
  let clickCallback   = null;

  // ── public ──────────────────────────────────────────────────────────────

  function init(canvasEl, config) {
    canvas    = canvasEl;
    mapConfig = config;
    canvas.width  = config.canvasWidth;
    canvas.height = config.canvasHeight;
    ctx = canvas.getContext('2d');

    canvas.addEventListener('click', e => {
      const rect  = canvas.getBoundingClientRect();
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

  function setPosition(pos) {
    currentPosition = pos;
    render();
  }

  function onCanvasClick(fn) {
    clickCallback = fn;
  }

  // ── drawing ──────────────────────────────────────────────────────────────

  function render() {
    if (!ctx) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    drawEdges();
    drawPathEdges();
    drawNodes();
    drawAccessPoints();
    if (currentPosition) drawPosition();
  }

  function drawEdges() {
    if (!mapConfig) return;
    const nodeMap = buildNodeMap();

    ctx.strokeStyle = '#2e3450';
    ctx.lineWidth   = 2;

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

    ctx.strokeStyle = '#4f8ef7';
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

  function drawNodes() {
    if (!mapConfig) return;
    const pathIds = new Set(currentPath.map(n => n.id));

    for (const node of mapConfig.nodes) {
      const colour = NODE_COLOURS[node.type] || '#888';
      const onPath = pathIds.has(node.id);

      // Outer glow for path nodes
      if (onPath) {
        ctx.beginPath();
        ctx.arc(node.x, node.y, NODE_RADIUS + 5, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(79,142,247,0.25)';
        ctx.fill();
      }

      ctx.beginPath();
      ctx.arc(node.x, node.y, NODE_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle   = colour;
      ctx.strokeStyle = onPath ? '#4f8ef7' : '#0f1117';
      ctx.lineWidth   = onPath ? 2.5 : 1.5;
      ctx.fill();
      ctx.stroke();

      // Label — only for named rooms (not plain corridors)
      if (node.type !== 'CORRIDOR') {
        ctx.font      = '11px Segoe UI, sans-serif';
        ctx.fillStyle = '#e2e8f0';
        ctx.textAlign = 'center';
        ctx.fillText(node.label, node.x, node.y + NODE_RADIUS + LABEL_OFFSET);
      }
    }
  }

  function drawAccessPoints() {
    if (!mapConfig) return;
    for (const ap of mapConfig.accessPoints) {
      // Diamond shape for APs
      ctx.save();
      ctx.translate(ap.x, ap.y);
      ctx.rotate(Math.PI / 4);
      ctx.beginPath();
      ctx.rect(-AP_RADIUS, -AP_RADIUS, AP_RADIUS * 2, AP_RADIUS * 2);
      ctx.fillStyle   = '#f76b4f';
      ctx.strokeStyle = '#0f1117';
      ctx.lineWidth   = 1.5;
      ctx.fill();
      ctx.stroke();
      ctx.restore();

      // WiFi symbol label
      ctx.font      = '10px Segoe UI, sans-serif';
      ctx.fillStyle = '#f76b4f';
      ctx.textAlign = 'center';
      ctx.fillText(ap.id, ap.x, ap.y + AP_RADIUS + 14);
    }
  }

  function drawPosition() {
    const { x, y, uncertainty } = currentPosition;

    // Uncertainty circle
    if (uncertainty > 0) {
      ctx.beginPath();
      ctx.arc(x, y, uncertainty, 0, Math.PI * 2);
      ctx.fillStyle   = 'rgba(79,142,247,0.12)';
      ctx.strokeStyle = 'rgba(79,142,247,0.4)';
      ctx.lineWidth   = 1;
      ctx.fill();
      ctx.stroke();
    }

    // Position dot
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, Math.PI * 2);
    ctx.fillStyle   = '#ffffff';
    ctx.strokeStyle = '#4f8ef7';
    ctx.lineWidth   = 3;
    ctx.fill();
    ctx.stroke();

    // Pulse ring (static — animated version would use requestAnimationFrame)
    ctx.beginPath();
    ctx.arc(x, y, 16, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(79,142,247,0.5)';
    ctx.lineWidth   = 1.5;
    ctx.stroke();
  }

  // ── helpers ──────────────────────────────────────────────────────────────

  function buildNodeMap() {
    const m = {};
    if (mapConfig) for (const n of mapConfig.nodes) m[n.id] = n;
    return m;
  }

  return { init, setPath, setPosition, onCanvasClick, render };
})();
