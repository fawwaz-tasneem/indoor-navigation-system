/**
 * Thin wrapper around the FastAPI backend.
 * Empty string = same origin (served by uvicorn at localhost:8000).
 * If opening index.html directly as a file, change to 'http://localhost:8000'.
 */
const API_BASE = '';

const Api = {

  async getMap() {
    const res = await fetch(`${API_BASE}/api/map`);
    if (!res.ok) throw new Error('Failed to load map config');
    return res.json();
  },

  /**
   * @param {Object.<string,number>} rssiMap  { bssid: rssiDbm }
   * @param {number}                 dt       seconds since last call
   */
  async localise(rssiMap) {
    const res = await fetch(`${API_BASE}/api/nav/localise`, {
      method:  'POST',
      headers: { 'Content-Type': 'application/json' },
      body:    JSON.stringify({ rssi: rssiMap })
    });
    if (res.status === 422) return null;
    if (!res.ok) throw new Error('Localise request failed');
    return res.json();   // { ekf: { x, y, uncertainty }, kf: { x, y, uncertainty } }
  },

  /**
   * @param {string} fromId
   * @param {string} toId
   * @returns {Array} ordered list of NavNode objects
   */
  async getPath(fromId, toId) {
    const res = await fetch(`${API_BASE}/api/nav/path?from=${fromId}&to=${toId}`);
    if (res.status === 404) return [];
    if (!res.ok) throw new Error('Path request failed');
    return res.json();
  },

  async nearestNode(x, y) {
    const res = await fetch(`${API_BASE}/api/nav/nearest?x=${x}&y=${y}`);
    if (!res.ok) return null;
    return res.json();
  }
};
