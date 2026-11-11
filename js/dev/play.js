// Dev-only: plays the "Smonta" workshop game on the open engine page with synthetic pointer events, the way a
// player would (hand = drag along the arrow, wrench = hold). Use it to check that every part of a
// `disassembly` can really be reached and taken off. From the console, on an engine page:
//   const { playStrip } = await import('/js/dev/play.js?t=' + Date.now());
//   await playStrip();            // -> { removed, total, unreachable: [...], errors, seconds }
// Works in a background tab too (it steps the viewer by hand). `unreachable` lists parts the pointer could
// not grab from any camera angle: usually parts buried inside something that is not in their `after` list.
import * as THREE from 'three';

// Background tabs clamp setTimeout to one second, which would make a run take minutes: yield through a
// MessageChannel instead (not throttled) and watch the real clock.
const yieldNow = () =>
  new Promise((r) => {
    const c = new MessageChannel();
    c.port1.onmessage = () => r();
    c.port2.postMessage(0);
  });
const sleep = async (ms) => {
  const end = performance.now() + ms;
  do await yieldNow();
  while (performance.now() < end);
};

export async function playStrip({ v = window.__viewer, ws = window.__ws, log = false } = {}) {
  if (!v || !ws) throw new Error('open an engine page first');
  const el = v.renderer.domElement;
  const wasManual = v.manual;
  v.manual = true;
  const frames = async (n = 2) => {
    for (let i = 0; i < n; i++) {
      v.step(0.05);
      await sleep(16);
    }
  };
  const pe = (type, x, y, buttons = type === 'pointerup' ? 0 : 1) =>
    el.dispatchEvent(new PointerEvent(type, { clientX: x, clientY: y, button: 0, buttons, pointerId: 7, bubbles: true }));
  const rect = () => el.getBoundingClientRect();
  const toScreen = (world) => {
    const r = rect();
    const p = world.clone().project(v.camera);
    return [r.left + (p.x * 0.5 + 0.5) * r.width, r.top + (-p.y * 0.5 + 0.5) * r.height];
  };
  // Points worth trying on a part: every instance of its instanced meshes, the centre of every mesh.
  const candidates = (part) => {
    const out = [];
    const M = new THREE.Matrix4();
    const box = new THREE.Box3();
    for (const m of part.meshes) {
      if (m.isInstancedMesh) {
        m.geometry.computeBoundingBox();
        for (let k = 0; k < m.count; k++) {
          const c = m.geometry.boundingBox.getCenter(new THREE.Vector3());
          // The head end of a screw (+Y) is the visible one.
          c.y = m.geometry.boundingBox.max.y;
          m.getMatrixAt(k, M);
          out.push(c.applyMatrix4(M).applyMatrix4(m.matrixWorld));
        }
      } else {
        box.setFromObject(m);
        if (!box.isEmpty()) out.push(box.getCenter(new THREE.Vector3()));
      }
    }
    // Rings, shells and plates have nothing at their centre: also try some of their vertices.
    const per = Math.max(4, Math.floor(120 / Math.max(1, part.meshes.length)));
    for (const m of part.meshes) {
      if (m.isInstancedMesh) continue;
      const pos = m.geometry.attributes.position;
      const stride = Math.max(1, Math.floor(pos.count / per));
      for (let i = 0; i < pos.count && out.length < 260; i += stride) out.push(new THREE.Vector3().fromBufferAttribute(pos, i).applyMatrix4(m.matrixWorld));
    }
    return out;
  };

  ws.dev = true; // no records from test runs
  if (ws.mode !== 'strip') ws.start('strip');
  await frames(3);
  const def = v.def;
  const order = def.disassembly.steps.flatMap((s) => s.parts);
  const home = { pos: v.camera.position.clone(), look: v.controls.target.clone() };
  const orbit = (az, el2 = 0) => {
    const off = home.pos.clone().sub(home.look);
    off.applyAxisAngle(new THREE.Vector3(0, 1, 0), az);
    if (el2) off.y += off.length() * el2;
    v.camera.position.copy(home.look).add(off);
    v.camTarget = null;
    v.controls.update();
  };
  const t0 = performance.now();
  const unreachable = [];
  const skipped = new Set();
  let guard = 0;
  while (guard++ < 200) {
    const next = order.map((id) => v.part(id)).find((p) => p && !p.out && !skipped.has(p.id) && v.visibleParts.includes(p) && !v.blockersOf(p.id).length);
    if (!next) break;
    ws.setTool(v.motionOf(next.id) === 'unscrew' ? 'wrench' : 'hand');
    let grabbed = null;
    search: for (const [az, up] of [[0, 0], [0, 0.6], [1.2, 0.2], [-1.2, 0.2], [2.4, 0.2], [-2.4, 0.2], [Math.PI, 0.5], [0, -0.5]]) {
      orbit(az, up);
      await frames(1);
      for (const c of candidates(next)) {
        const [x, y] = toScreen(c);
        const r = rect();
        if (x < r.left + 4 || y < r.top + 4 || x > r.right - 4 || y > r.bottom - 4) continue;
        pe('pointermove', x, y, 0); // hover
        // Only press where the pointer is really over the part, so a miss does not cost a mistake.
        if (v.hovered !== next) continue;
        pe('pointerdown', x, y);
        if (ws.grab?.part === next) {
          grabbed = [x, y];
          break search;
        }
        pe('pointerup', x, y);
      }
    }
    if (!grabbed) {
      unreachable.push(next.id);
      skipped.add(next.id);
      continue;
    }
    const [x, y] = grabbed;
    if (ws.tool === 'hand') {
      const a = v.pullAxis(next.id);
      const [ux, uy] = a.len < 28 ? [0, -1] : [a.x / a.len, a.y / a.len];
      for (let i = 1; i <= 12 && ws.grab; i++) {
        pe('pointermove', x + ux * i * 24, y + uy * i * 24);
        await frames(1);
      }
      pe('pointerup', x + ux * 290, y + uy * 290);
    } else {
      for (let i = 0; i < 120 && ws.grab; i++) await frames(1);
      pe('pointerup', x, y);
    }
    if (log) console.log('took off', next.id, 'out:', next.out);
    if (!next.out) {
      unreachable.push(next.id + ' (grabbed but did not come off)');
      skipped.add(next.id);
    }
    await frames(4);
  }
  orbit(0, 0);
  await frames(30);
  v.manual = wasManual;
  return {
    engine: def.id,
    removed: ws.targets.filter((p) => p.out).length,
    total: ws.total,
    unreachable,
    errors: ws.errors,
    score: ws.score,
    seconds: Math.round((performance.now() - t0) / 1000),
    finished: !!ws.ended,
  };
}
