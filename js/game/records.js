// Best results of the workshop games, kept in the browser (localStorage).
// Shape: { [engineId]: { [mode]: { score, time, stars } } }, mode = strip | build | hunt.
const KEY = 'motorilab-records';
export const MODES = ['strip', 'build', 'hunt'];

function read() {
  try {
    return JSON.parse(localStorage.getItem(KEY)) || {};
  } catch {
    return {};
  }
}

export function getRecord(engineId, mode) {
  return read()[engineId]?.[mode] || null;
}

// Saves the result when it beats the stored one. Returns true for a new record.
export function saveRecord(engineId, mode, result) {
  const all = read();
  const old = all[engineId]?.[mode];
  const better = !old || result.stars > old.stars || (result.stars === old.stars && result.score > old.score);
  if (!better) return false;
  (all[engineId] ||= {})[mode] = { score: result.score, time: Math.round(result.time), stars: result.stars };
  try {
    localStorage.setItem(KEY, JSON.stringify(all));
  } catch {}
  return true;
}

// Stars earned on one engine: the best over its game modes counts once per mode.
export function engineStars(engineId) {
  const r = read()[engineId] || {};
  return MODES.reduce((n, m) => n + (r[m]?.stars || 0), 0);
}

export function totalStars(engineIds) {
  return engineIds.reduce((n, id) => n + engineStars(id), 0);
}
