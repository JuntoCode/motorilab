// Dev-only sanity check for an engine module, runnable from the browser console on the site:
//   (await import('/js/dev/check.js')).checkEngine('diesel')
// Builds the engine with a mock ctx, runs update over two cycles at several throttles and
// explode values, and reports errors, NaN positions, triangle and particle counts.
import * as THREE from 'three';

export async function checkEngine(file) {
  const report = { file, errors: [], warnings: [] };
  let def;
  try {
    def = (await import(`../engines/${file}.js?t=${Date.now()}`)).default;
  } catch (e) {
    report.errors.push('import: ' + e.message);
    return report;
  }
  for (const k of ['id', 'name', 'short', 'family', 'vehicle', 'intro', 'specs', 'metrics', 'phases', 'quiz', 'camera', 'build']) {
    if (def[k] == null) report.errors.push(`missing field: ${k}`);
  }
  const partIds = [];
  const ctx = {
    part: (o, info) => {
      if (!info?.id || !info.name || !info.base || !info.expert) report.errors.push(`part incomplete: ${info?.id}`);
      partIds.push(info.id);
      const g = new THREE.Group();
      g.add(o);
      return g;
    },
    addFlow() {},
  };
  let eng;
  try {
    eng = def.build(ctx);
  } catch (e) {
    report.errors.push('build: ' + e.stack);
    return report;
  }
  for (const ph of def.phases || []) {
    for (const p of ph.parts || []) if (!partIds.includes(p)) report.errors.push(`phase ${ph.id} lists unknown part ${p}`);
  }
  (def.quiz || []).forEach((q, i) => {
    if (q.options?.length !== 4 || q.correct == null || !q.explain) report.errors.push(`quiz ${i} malformed`);
  });
  try {
    for (const explode of [0, 1]) {
      for (const th of [0, 0.5, 1]) {
        for (let i = 0; i <= 40; i++) {
          eng.update({ cycle: (i / 20) % 1, throttle: th, dt: i % 5 ? 0.016 : 0, time: i * 0.016, paused: false, explode });
        }
        eng.readout({ cycle: 0.3, throttle: th, dt: 0.016, time: 1, paused: false, explode });
      }
    }
  } catch (e) {
    report.errors.push('update/readout: ' + e.stack);
  }
  let tris = 0;
  let particles = 0;
  let nan = 0;
  eng.root.updateMatrixWorld(true);
  const box = new THREE.Box3();
  eng.root.traverse((o) => {
    const g = o.geometry;
    if (!g) return;
    const pos = g.attributes.position;
    if (o.isPoints) particles += pos.count;
    else if (o.isMesh) {
      tris += (g.index ? g.index.count : pos.count) / 3;
      box.expandByObject(o);
    }
    for (let i = 0; i < pos.array.length; i++) if (!Number.isFinite(pos.array[i])) { nan++; break; }
    if (!Number.isFinite(o.position.x + o.position.y + o.position.z + o.rotation.x + o.rotation.y + o.rotation.z)) nan++;
  });
  if (nan) report.errors.push(`${nan} objects with NaN/Infinity positions`);
  if (tris > 150000) report.warnings.push(`triangles ${Math.round(tris)} > 150k`);
  if (particles > 3000) report.warnings.push(`particles ${particles} > 3000`);
  const size = box.getSize(new THREE.Vector3());
  report.stats = {
    triangles: Math.round(tris),
    particles,
    parts: partIds.length,
    phases: def.phases?.length,
    bbox: [box.min, box.max].map((v) => v.toArray().map((n) => +n.toFixed(2))),
    size: size.toArray().map((n) => +n.toFixed(2)),
  };
  report.ok = report.errors.length === 0;
  return report;
}
