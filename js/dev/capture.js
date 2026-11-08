// Dev-only screenshot tool. Works in background tabs (renders on demand, no animation loop needed).
// From the browser console on http://localhost:5173 :
//   const { shoot } = await import('/js/dev/capture.js?t=' + Date.now());
//   await shoot('diesel', { view: 'section', orbit: [-35, 20], zoom: 1.4, target: [0, 1, 0] });
// The PNG is written to .claude/shots/<name>.png (read it with an image viewer / the Read tool).
//
// Options:
//   name       output file name (default: file + view)
//   view       'section' | 'closed'                     (default 'section')
//   explode    0..1                                       (default 0)
//   throttle   0..1                                       (default 0.5)
//   cycle      0..1 animation position (cyclic engines)   (default 0.3)
//   phase      index: highlight that phase as in step mode
//   select     part id to highlight
//   option     { id: value } engine options (e.g. { timing: 'chain' })
//   orbit      [azimuthDeg, elevationDeg] around the target; default = the engine's own camera
//   zoom       >1 moves closer (default 1)
//   target     [x,y,z] look-at point override
//   width, height (default 1200 x 750), format 'png' | 'webp', path (full output path)
import * as THREE from 'three';
import { EngineViewer } from '../core/viewer.js';

export async function shoot(file, o = {}) {
  const def = (await import(`../engines/${file}.js?t=${Date.now()}`)).default;
  const width = o.width || 1200;
  const height = o.height || 750;
  const host = document.createElement('div');
  Object.assign(host.style, { position: 'fixed', left: '0', top: '0', width: width + 'px', height: height + 'px', opacity: '0', pointerEvents: 'none', zIndex: '-1' });
  document.body.appendChild(host);
  const v = new EngineViewer(host, { interactive: false });
  v.manual = true;
  v.renderer.setPixelRatio(1);
  try {
    v.load(def);
    v.resize();
    // Screenshots get the site's dark backdrop; thumbnails (webp) stay transparent for the card gradient.
    if ((o.format || 'png') === 'png' && !o.transparent) v.scene.background = new THREE.Color(0x121a2a);
    for (const [k, val] of Object.entries(o.option || {})) v.setOption(k, val);
    v.setCutaway((o.view || 'section') === 'section', true);
    v.setThrottle(o.throttle ?? 0.5);
    v.setExplode(o.explode ?? 0, true);
    if (o.phase != null) v.goToPhase(o.phase);
    v.state.cycle = o.cycle ?? (o.phase != null ? v.state.cycle : 0.3);
    if (o.select) v.selectPart(o.select);

    // Camera: engine default, optionally orbited / zoomed / retargeted.
    const cam = def.camera;
    const target = new THREE.Vector3(...(o.target || cam.target));
    const offset = new THREE.Vector3(...cam.position).sub(new THREE.Vector3(...cam.target));
    let dist = offset.length() / (o.zoom || 1);
    if (o.orbit) {
      const [az, el] = o.orbit.map((d) => (d * Math.PI) / 180);
      offset.set(Math.sin(az) * Math.cos(el), Math.sin(el), Math.cos(az) * Math.cos(el));
    }
    offset.setLength(dist * (1 + 0.5 * (o.explode ?? 0)));
    v.camera.position.copy(target).add(offset);
    v.controls.target.copy(target);
    v.camTarget = null;
    v.controls.update();

    // A few frames so flows, easing and the section caps settle.
    v.state.paused = o.phase != null;
    for (let i = 0; i < 12; i++) v.step(i < 11 ? 0.03 : 0);
    const format = o.format || 'png';
    const data = v.renderer.domElement.toDataURL(`image/${format}`, 0.88);
    const path = o.path || `.claude/shots/${o.name || `${file}-${o.view || 'section'}`}.${format}`;
    const res = await fetch(`/__save?path=${encodeURIComponent(path)}`, { method: 'POST', body: data });
    if (!res.ok) throw new Error(`save failed: ${res.status}`);
    return path;
  } finally {
    v.dispose();
    host.remove();
  }
}

// Card thumbnails for the home page: assets/thumbs/<id>.webp
export async function thumbs(files) {
  const out = [];
  for (const f of files) {
    const def = (await import(`../engines/${f}.js?t=${Date.now()}`)).default;
    const t = def.thumb || {};
    out.push(
      await shoot(f, { width: 800, height: 500, format: 'webp', path: `assets/thumbs/${def.id}.webp`, throttle: 0.6, ...t })
    );
  }
  return out;
}
