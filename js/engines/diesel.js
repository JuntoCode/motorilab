// Inline-4 common-rail turbodiesel, 4 valves per cylinder, central injector, glow plugs,
// turbocharger + intercooler at the timing end, DPF. Cycle 0..1 = 720° of crank for cylinder 1.
// Detailed castings (open-deck block, bulkheads, hollow head/cover/sump) for the section view;
// selectable timing drive (dry belt, wet belt, chain).
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, pipe, flow, ringX, clamp, lerp, smooth } from '../core/helpers.js';

const BORE_X = [-1.2, -0.4, 0.4, 1.2];
// Firing order 1-3-4-2: cycle offset of each cylinder relative to cylinder 1.
const OFFSET = [0, 0.75, 0.25, 0.5];
const R = 0.3; // crank radius
const L = 1.0; // conrod length
const CROWN = 0.2; // piston crown above the wrist pin (small clearance at TDC = high compression)
const HEAD_Y = 1.55; // flat fire deck of the head
const VALVE_Z = 0.15;
const VALVE_DX = 0.11; // two valves per side, 4 per cylinder
const CAM_Y = 2.18;
const INJ_TIP = HEAD_Y - 0.04;
const RAIL_Y = 2.64;
const RAIL_Z = -0.3;
// Turbocharger, axis along X beyond the timing/accessory drives
const TY = 1.25;
const CX = -2.54; // compressor wheel
const TX = -3.04; // turbine wheel
const PUMP = [-1.92, 0.95, -0.6]; // high-pressure pump drive centre (on the timing drive)
const DPF_X = -3.0;
const TIMING = ['dryBelt', 'wetBelt', 'chain']; // timing drive variants (option 'timing')

// ---------- engine kit: shared layout + geometry helpers (same code in diesel.js) ----------
const TAU = Math.PI * 2;
const UP = new THREE.Vector3(0, 1, 0);
const XA = new THREE.Vector3(1, 0, 0);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const BX = 1.8; // block half length
const BZ = 0.56; // block half width
const HX = 1.72; // head half length
const HZ = 0.46; // head half width
const GAL_Y = 1.84; // cam gallery floor
const HEAD_TOP = 2.06; // head / cam cover joint
const COVER_TOP = 2.42;
const XT = -1.92; // timing drive plane
const XACC = -2.12; // accessory belt plane
const R_CRANK_SPROCKET = 0.0725;
const CAP_X = [-1.58, -0.8, 0, 0.8, 1.58]; // cam bearing caps / towers

const wrap = (c) => ((c % 1) + 1) % 1;
const alongX = (g) => g.rotateZ(-Math.PI / 2); // axis Y -> X
const alongZ = (g) => g.rotateX(Math.PI / 2); // axis Y -> Z

// Smooth valve lift for an open window [a, b] in cycle units (may wrap around 0).
function lift(c, a, b) {
  const x = wrap(c - a);
  const w = b - a;
  return x < w ? Math.sin((Math.PI * x) / w) : 0;
}

function rod(a, b, r, mat, seg = 10) {
  const va = V(...a);
  const vb = V(...b);
  const dir = vb.clone().sub(va);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, dir.length(), seg), mat);
  m.position.copy(va).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(UP, dir.normalize());
  return m;
}
const mesh = (g, m, x = 0, y = 0, z = 0) => {
  const o = new THREE.Mesh(g, m);
  o.position.set(x, y, z);
  return o;
};
// Clone + cut every material in a group (small attachments that must vanish with the section).
function cutAll(group) {
  group.traverse((o) => {
    if (o.material) o.material = cut(o.material.clone());
  });
  return group;
}
const rbox = (w, h, d, r, m, x, y, z) => mesh(new RoundedBoxGeometry(w, h, d, 2, r), m, x, y, z);
const curve = (pts) => new THREE.CatmullRomCurve3(pts.map((p) => V(...p)), false, 'centripetal');
const tube = (c, r, m, seg = 48, rad = 12) => new THREE.Mesh(new THREE.TubeGeometry(c, seg, r, rad), m);

function convexHull(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [];
  for (const q of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], q) <= 0) lower.pop();
    lower.push(q);
  }
  const upper = [];
  for (const q of p.reverse()) {
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], q) <= 0) upper.pop();
    upper.push(q);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

// 2D outlines -> closed solids (closed solids get hatched section caps when cut).
function rrPath(p, x0, y0, x1, y1, r) {
  p.moveTo(x0 + r, y0);
  p.lineTo(x1 - r, y0);
  p.absarc(x1 - r, y0 + r, r, -Math.PI / 2, 0, false);
  p.lineTo(x1, y1 - r);
  p.absarc(x1 - r, y1 - r, r, 0, Math.PI / 2, false);
  p.lineTo(x0 + r, y1);
  p.absarc(x0 + r, y1 - r, r, Math.PI / 2, Math.PI, false);
  p.lineTo(x0, y0 + r);
  p.absarc(x0 + r, y0 + r, r, Math.PI, 1.5 * Math.PI, false);
  return p;
}
const rrShape = (...a) => rrPath(new THREE.Shape(), ...a);
const rrHole = (...a) => rrPath(new THREE.Path(), ...a);
const circleHole = (x, y, r) => new THREE.Path().absarc(x, y, r, 0, TAU, true);
function polyShape(pts) {
  const s = new THREE.Shape();
  pts.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
  s.closePath();
  return s;
}
function extrude(shape, depth, seg) {
  return new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: seg });
}
// Shape in the XZ plane (shape y = -z), solid from y0 to y1.
function extY(shape, y0, y1, seg = 24) {
  return extrude(shape, y1 - y0, seg).rotateX(-Math.PI / 2).translate(0, y0, 0);
}
// Shape in the XY plane, solid from z0 to z1.
function extZ(shape, z0, z1, seg = 24) {
  return extrude(shape, z1 - z0, seg).translate(0, 0, z0);
}
// Shape in the ZY plane given as (-z, y), solid from x0 to x1.
function extX(shape, x0, x1, seg = 24) {
  return extrude(shape, x1 - x0, seg).rotateY(Math.PI / 2).translate(x0, 0, 0);
}

// Rows of hex bolt heads as one InstancedMesh (axis: 'x' | 'y' | 'z').
const HEX = new THREE.CylinderGeometry(0.024, 0.024, 0.02, 6);
// Hex head + threaded shank of length `len` below it (local -Y), so a bolt backing out shows its shank.
const screw = (len, r = 0.024, h = 0.02) =>
  mergeGeometries([new THREE.CylinderGeometry(r, r, h, 6), new THREE.CylinderGeometry(r * 0.62, r * 0.62, len, 8).translate(0, -h / 2 - len / 2, 0)]);
function bolts(mat, pts, axis = 'y', geo = HEX) {
  const m = new THREE.InstancedMesh(geo, mat, pts.length);
  const q = new THREE.Quaternion();
  if (axis === 'z') q.setFromAxisAngle(XA, Math.PI / 2);
  if (axis === 'x') q.setFromAxisAngle(V(0, 0, 1), -Math.PI / 2);
  const mtx = new THREE.Matrix4();
  const one = V(1, 1, 1);
  pts.forEach((p, i) => m.setMatrixAt(i, mtx.compose(V(...p), q, one)));
  return m;
}

// Closed belt/chain loop around circles [y, z, r] in a YZ plane: arc-length sampler.
function loopPath(circles, n = 48) {
  const pts = [];
  for (const [y, z, r] of circles) for (let i = 0; i < n; i++) pts.push([y + Math.cos((i / n) * TAU) * r, z + Math.sin((i / n) * TAU) * r]);
  const hull = convexHull(pts); // counter-clockwise in (y, z)
  const cum = [0];
  for (let i = 1; i <= hull.length; i++) {
    const a = hull[i - 1];
    const b = hull[i % hull.length];
    cum.push(cum[i - 1] + Math.hypot(b[0] - a[0], b[1] - a[1]));
  }
  const total = cum[hull.length];
  function at(s, o) {
    s = ((s % total) + total) % total;
    let lo = 0;
    let hi = hull.length;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (cum[m] <= s) lo = m;
      else hi = m;
    }
    const a = hull[lo];
    const b = hull[(lo + 1) % hull.length];
    const len = cum[lo + 1] - cum[lo] || 1;
    const f = (s - cum[lo]) / len;
    o.y = a[0] + (b[0] - a[0]) * f;
    o.z = a[1] + (b[1] - a[1]) * f;
    o.ty = (b[0] - a[0]) / len;
    o.tz = (b[1] - a[1]) / len;
    o.ny = o.tz; // outward normal (right of a CCW path)
    o.nz = -o.ty;
    return o;
  }
  const spans = hull.map((a, i) => ({ a, b: hull[(i + 1) % hull.length], len: cum[i + 1] - cum[i] })).sort((p, q) => q.len - p.len);
  return { total, at, spans };
}

// Flat band (belt) following a loop at plane x: closed solid, inner face on the pitch line.
function ribbonGeo(loop, x, width, thick, samples = 260) {
  const pos = [];
  const idx = [];
  const o = {};
  for (let k = 0; k < samples; k++) {
    loop.at((k / samples) * loop.total, o);
    for (const r of [0, thick]) for (const dx of [-width / 2, width / 2]) pos.push(x + dx, o.y + o.ny * r, o.z + o.nz * r);
  }
  const v = (k, r, s) => (k % samples) * 4 + r * 2 + s;
  for (let k = 0; k < samples; k++) {
    const quad = (a, b, c, d) => idx.push(a, b, c, a, c, d);
    quad(v(k, 0, 0), v(k + 1, 0, 0), v(k + 1, 0, 1), v(k, 0, 1));
    quad(v(k, 1, 0), v(k, 1, 1), v(k + 1, 1, 1), v(k + 1, 1, 0));
    quad(v(k, 0, 0), v(k, 1, 0), v(k + 1, 1, 0), v(k + 1, 0, 0));
    quad(v(k, 0, 1), v(k + 1, 0, 1), v(k + 1, 1, 1), v(k, 1, 1));
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// Toothed wheel (sprocket / toothed pulley) with axis X.
function sprocketGeo(teeth, rTip, rRoot, width, holeR) {
  const s = new THREE.Shape();
  for (let k = 0; k < teeth; k++) {
    [[rRoot, -0.5], [rTip, -0.18], [rTip, 0.18], [rRoot, 0.5]].forEach(([r, f], j) => {
      const a = ((k + 0.5 + f * 0.9) / teeth) * TAU;
      if (k === 0 && j === 0) s.moveTo(r * Math.cos(a), r * Math.sin(a));
      else s.lineTo(r * Math.cos(a), r * Math.sin(a));
    });
  }
  s.closePath();
  s.holes.push(circleHole(0, 0, holeR));
  return extrude(s, width, 2).translate(0, 0, -width / 2).rotateY(Math.PI / 2);
}

// Camshaft lobe (base circle + flank), axis X.
function camLobeGeometry() {
  const s = new THREE.Shape();
  const N = 40;
  for (let i = 0; i <= N; i++) {
    const a = (i / N) * TAU;
    const r = 0.06 + 0.06 * Math.pow(Math.max(0, -Math.sin(a)), 3);
    i === 0 ? s.moveTo(r * Math.cos(a), r * Math.sin(a)) : s.lineTo(r * Math.cos(a), r * Math.sin(a));
  }
  return extrude(s, 0.09, 2).translate(0, 0, -0.045).rotateY(Math.PI / 2);
}

// Generic metal plate texture with stamped text (no real brands).
function plateTexture(lines) {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#b9c0c8';
  g.fillRect(0, 0, 256, 64);
  g.strokeStyle = '#59616b';
  g.lineWidth = 4;
  g.strokeRect(4, 4, 248, 56);
  g.fillStyle = '#1d232b';
  g.font = 'bold 26px sans-serif';
  g.textAlign = 'center';
  g.fillText(lines[0], 128, 32);
  g.font = '15px sans-serif';
  g.fillText(lines[1] || '', 128, 52);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// ---------- Timing drive: 'dryBelt' | 'wetBelt' | 'chain' ----------
// circles: [{ y, z, r, kind: 'crank'|'cam'|'pump'|'idler' }] in the plane x = XT.
function buildTiming(kind, circles, mats) {
  const chain = kind === 'chain';
  const pitchOff = chain ? 0.006 : 0.004;
  const loop = loopPath(circles.map((c) => [c.y, c.z, c.r + pitchOff]));
  const drive = new THREE.Group();
  const rotors = [];
  const darkHub = mats.dark;
  for (const c of circles) {
    const g = new THREE.Group();
    g.position.set(XT, c.y, c.z);
    if (c.kind === 'idler') {
      g.add(new THREE.Mesh(alongX(lathe([[0.02, -0.05], [c.r, -0.05], [c.r + 0.006, -0.045], [c.r + 0.006, 0.045], [c.r, 0.05], [0.02, 0.05]], 40)), mats.steel));
      g.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(c.r * 0.45, c.r * 0.45, 0.106, 24)), darkHub));
      // eccentric tensioner arm
      const arm = rbox(0.03, 0.05, c.r * 2.4, 0.01, mats.steel, 0.07, 0, -c.r * 0.6);
      g.add(arm);
    } else if (chain) {
      const n = Math.max(12, Math.round((TAU * c.r) / 0.03));
      g.add(new THREE.Mesh(sprocketGeo(n, c.r + 0.012, c.r - 0.012, 0.05, 0.03), mats.steel));
      g.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(c.r * 0.55, c.r * 0.55, 0.08, 24)), darkHub));
    } else {
      const n = Math.max(16, Math.round((TAU * c.r) / 0.0228));
      g.add(new THREE.Mesh(sprocketGeo(n, c.r, c.r - 0.009, 0.1, 0.03), kind === 'wetBelt' ? mats.steel : mats.darkSteel));
      const flange = alongX(new THREE.CylinderGeometry(c.r + 0.014, c.r + 0.014, 0.008, 40));
      g.add(mesh(flange, mats.steel, 0.054), mesh(flange, mats.steel, -0.054));
      g.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(c.r * 0.5, c.r * 0.5, 0.11, 24)), darkHub));
    }
    // lightening holes / bolt marks make the rotation readable
    if (c.kind !== 'idler' && c.r > 0.09) {
      for (let i = 0; i < 3; i++) {
        const a = (i / 3) * TAU;
        g.add(mesh(alongX(new THREE.CylinderGeometry(0.018, 0.018, chain ? 0.056 : 0.112, 10)), darkHub, 0, Math.cos(a) * c.r * 0.72, Math.sin(a) * c.r * 0.72));
      }
    }
    drive.add(g);
    rotors.push({ g, ratio: R_CRANK_SPROCKET / c.r });
  }

  // Belt band + teeth, or chain rollers + plates (instanced, moved along the loop).
  const o = {};
  const mtx = new THREE.Matrix4();
  const vx = V(1, 0, 0);
  const vn = new THREE.Vector3();
  const vt = new THREE.Vector3();
  let update;
  if (!chain) {
    const beltMat = kind === 'wetBelt' ? mats.wetBelt : mats.belt;
    const band = new THREE.Mesh(ribbonGeo(loop, XT, 0.09, 0.012), beltMat);
    drive.add(band);
    const n = Math.round(loop.total / 0.0228);
    const pitch = loop.total / n;
    const teeth = new THREE.InstancedMesh(new THREE.BoxGeometry(0.09, 0.012, 0.011), beltMat, n);
    drive.add(teeth);
    update = (s) => {
      for (let k = 0; k < n; k++) {
        loop.at(k * pitch + s, o);
        vn.set(0, o.ny, o.nz);
        vt.set(0, -o.nz, o.ny);
        mtx.makeBasis(vx, vn, vt).setPosition(XT, o.y - o.ny * 0.006, o.z - o.nz * 0.006);
        teeth.setMatrixAt(k, mtx);
      }
      teeth.instanceMatrix.needsUpdate = true;
    };
  } else {
    const n = Math.round(loop.total / 0.03);
    const pitch = loop.total / n;
    const rollers = new THREE.InstancedMesh(alongX(new THREE.CylinderGeometry(0.009, 0.009, 0.05, 8)), mats.steel, n);
    const plates = new THREE.InstancedMesh(new RoundedBoxGeometry(0.006, 0.024, pitch * 1.45, 1, 0.003), mats.darkSteel, n * 2);
    drive.add(rollers, plates);
    const py = new Float32Array(n);
    const pz = new Float32Array(n);
    update = (s) => {
      for (let k = 0; k < n; k++) {
        loop.at(k * pitch + s, o);
        py[k] = o.y;
        pz[k] = o.z;
        rollers.setMatrixAt(k, mtx.makeTranslation(XT, o.y, o.z));
      }
      for (let k = 0; k < n; k++) {
        const k1 = (k + 1) % n;
        let ty = py[k1] - py[k];
        let tz = pz[k1] - pz[k];
        const l = Math.hypot(ty, tz) || 1;
        ty /= l;
        tz /= l;
        vt.set(0, ty, tz);
        vn.set(0, tz, -ty);
        const off = k % 2 ? 0.022 : 0.03; // inner / outer link plates
        for (const sx of [-1, 1]) {
          mtx.makeBasis(vx, vn, vt).setPosition(XT + sx * off, (py[k] + py[k1]) / 2, (pz[k] + pz[k1]) / 2);
          plates.setMatrixAt(k * 2 + (sx > 0 ? 1 : 0), mtx);
        }
      }
      rollers.instanceMatrix.needsUpdate = true;
      plates.instanceMatrix.needsUpdate = true;
    };
    // Guides on the two long spans; hydraulic tensioner pushing the slack-side rail.
    loop.spans.slice(0, 2).forEach((sp, i) => {
      const dy = sp.b[0] - sp.a[0];
      const dz = sp.b[1] - sp.a[1];
      const len = Math.hypot(dy, dz);
      const ny = dz / len;
      const nz = -dy / len;
      const cy = (sp.a[0] + sp.b[0]) / 2 + ny * 0.026;
      const cz = (sp.a[1] + sp.b[1]) / 2 + nz * 0.026;
      const rail = rbox(0.07, 0.024, len * 0.78, 0.008, i ? mats.guideTens : mats.guide, XT, cy, cz);
      rail.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(vx, V(0, ny, nz), V(0, dy / len, dz / len)));
      drive.add(rail);
      const ends = [-0.36, 0.36].map((f) => [XT + 0.05, cy + (dy / len) * len * f, cz + (dz / len) * len * f]);
      drive.add(bolts(mats.steel, ends, 'x'));
      if (i === 1) {
        const by = cy + ny * 0.12;
        const bz = cz + nz * 0.12;
        const body = rod([XT, by, bz], [XT, by + ny * 0.1, bz + nz * 0.1], 0.03, mats.alu, 16);
        const plunger = rod([XT, cy + ny * 0.012, cz + nz * 0.012], [XT, by, bz], 0.012, mats.steel, 10);
        const bracket = rbox(0.05, 0.08, 0.08, 0.01, mats.alu, XT + 0.05, by + ny * 0.05, bz + nz * 0.05);
        drive.add(body, plunger, bracket);
      }
    });
  }
  update(0);

  // Cover / case: rim + front plate, outline = loop + margin. Shape coords are (-z, y).
  const outline = (m) => convexHull(circles.flatMap((c) => Array.from({ length: 32 }, (_, i) => [-(c.z + Math.sin((i / 32) * TAU) * (c.r + m)), c.y + Math.cos((i / 32) * TAU) * (c.r + m)])));
  const outer = outline(0.075);
  const inner = outline(0.05);
  const rimShape = polyShape(outer);
  rimShape.holes.push(polyShape(inner));
  const plateShape = polyShape(outer);
  const crank = circles.find((c) => c.kind === 'crank');
  plateShape.holes.push(circleHole(-crank.z, crank.y, 0.08));
  const cover = new THREE.Group();
  const caseMat = kind === 'dryBelt' ? mats.plastic : mats.case;
  cover.add(new THREE.Mesh(extX(rimShape, XT - 0.075, -BX), caseMat));
  cover.add(new THREE.Mesh(extX(plateShape, XT - 0.095, XT - 0.075), caseMat));
  if (kind === 'dryBelt') {
    // moulded ribs and the split line of the two-piece plastic cover
    const top = Math.max(...circles.map((c) => c.y + c.r));
    for (let y = 0.35; y < top - 0.2; y += 0.32) cover.add(rbox(0.02, 0.025, 0.36, 0.008, caseMat, XT - 0.105, y, 0));
    cover.add(mesh(new THREE.BoxGeometry(0.012, 0.012, 0.6), mats.dark, XT - 0.1, 1.05, 0));
    cover.add(bolts(mats.bolt, [[XT - 0.1, 1.9, 0.22], [XT - 0.1, 0.35, -0.22], [XT - 0.1, 1.9, -0.22]], 'x'));
  } else {
    const bp = [];
    const cu = outer.reduce((a, p) => a + p[0], 0) / outer.length;
    const cy = outer.reduce((a, p) => a + p[1], 0) / outer.length;
    for (let i = 0; i < outer.length; i += Math.max(1, Math.floor(outer.length / 14))) {
      const [u, y] = outer[i];
      const d = Math.hypot(u - cu, y - cy) || 1;
      bp.push([XT - 0.1, y - ((y - cy) / d) * 0.035, -(u - ((u - cu) / d) * 0.035)]);
    }
    cover.add(bolts(mats.bolt, bp, 'x'));
    if (kind === 'chain') cover.add(mesh(alongX(new THREE.CylinderGeometry(0.035, 0.035, 0.03, 6)), mats.bolt, XT - 0.11, 1.0, -0.2));
  }

  // Oil splash for oil-bathed drives (shown only when running).
  let splash = null;
  if (kind !== 'dryBelt') {
    const t = {};
    splash = flow({
      path: (u, out) => {
        loop.at(u * loop.total, t);
        return out.set(XT, t.y + t.ny * 0.03, t.z + t.nz * 0.03);
      },
      count: chain ? 90 : 140,
      size: 0.045,
      colors: [0xc9922e, 0x8a5a14],
      spread: 0.035,
      speed: 0.35,
      opacity: 0.75,
    });
  }
  return {
    drive,
    cover,
    splash,
    update(theta1) {
      for (const r of rotors) r.g.rotation.x = theta1 * r.ratio;
      update(theta1 * R_CRANK_SPROCKET);
    },
  };
}

const TIMING_TEXT = {
  dryBelt: {
    label: 'Cinghia a secco',
    base: 'Una cinghia di gomma dentata, rinforzata da fili, fuori dalla zona con l’olio e protetta solo da un coperchio di plastica. È silenziosa ed economica, ma invecchia: va sostituita a intervalli fissi, di solito ogni 5–10 anni o 90 000–240 000 km a seconda della casa. Se si rompe, in molti motori pistoni e valvole si scontrano.',
    expert:
      'Cinghia in HNBR con anima in fibra di vetro o aramidica e denti a profilo curvilineo; tendicinghia automatico eccentrico a molla. Pochi attriti e nessuna lubrificazione, ma la gomma subisce calore, ozono e contaminazioni: la sostituzione (spesso con pompa dell’acqua e rulli) va fatta a km o anni, il primo dei due. Nei motori “interferenti” la rottura piega le valvole.',
  },
  wetBelt: {
    label: 'Cinghia a bagno d’olio',
    base: 'La cinghia lavora dentro il motore, chiusa in un carter metallico e bagnata dall’olio. È più silenziosa e sulla carta dura di più, ma soffre se l’olio non è quello giusto o è vecchio: su alcuni dei primi motori di questo tipo la cinghia si è degradata e i suoi frammenti hanno intasato il pescante dell’olio.',
    expert:
      'Cinghia in elastomero specifico resistente all’olio, chiusa nel carter distribuzione e lubrificata. Riduce attriti e rumorosità rispetto a una catena e permette intervalli dichiarati lunghi (spesso oltre 150 000 km o 10 anni), ma è sensibile a specifica e qualità dell’olio, diluizione da carburante e fuliggine. In alcuni motori di prima generazione il deterioramento ha liberato detriti che hanno intasato la reticella del pescante, con cali di pressione dell’olio.',
  },
  chain: {
    label: 'Catena',
    base: 'Una catena metallica a rulli, come quella di una bici ma molto più robusta, che lavora nell’olio dentro un carter. È pensata per durare quanto il motore, ma col tempo si può allungare: un tendicatena idraulico e dei pattini la tengono in posizione e silenziosa.',
    expert:
      'Catena a rulli o a denti (silent chain) lubrificata dall’olio motore, con pattini guida in poliammide e tendicatena idraulico alimentato dal circuito di lubrificazione. Nessun intervallo di sostituzione programmato, ma usura di perni e boccole allunga la catena (errori di fase, rumore all’avviamento) e pattini e tenditore possono usurarsi: decisivi olio di qualità e cambi regolari. Più pesante e con più attriti di una cinghia.',
  },
};

// Radial turbo wheel facing +X: hub + backswept blades with a twisted inducer.
function turboWheel(rOut, len, count, bladeMat, hubMat) {
  const g = new THREE.Group();
  const hub = lathe([[0, -0.03], [rOut * 0.95, -0.03], [rOut * 0.92, 0], [rOut * 0.42, len * 0.5], [rOut * 0.24, len], [0, len]], 40);
  g.add(new THREE.Mesh(alongX(hub), hubMat));
  const s = new THREE.Shape();
  s.moveTo(-0.02, 0.02);
  s.lineTo(-0.02, rOut);
  s.lineTo(len * 0.2, rOut * 0.96);
  s.lineTo(len * 0.8, rOut * 0.66);
  s.lineTo(len, rOut * 0.62);
  s.lineTo(len, 0.02);
  s.closePath();
  const bg = new THREE.ExtrudeGeometry(s, { depth: 0.008, bevelEnabled: false });
  bg.translate(0, 0, -0.004);
  const p = bg.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const a = p.getX(i);
    const phi = 0.9 * Math.pow(clamp(a / len, 0, 1), 2); // inducer twist
    const y = p.getY(i);
    const z = p.getZ(i);
    p.setXYZ(i, a, y * Math.cos(phi) - z * Math.sin(phi), y * Math.sin(phi) + z * Math.cos(phi));
  }
  bg.computeVertexNormals();
  g.add(ringX(bg, bladeMat, count));
  return g;
}

// ---------- Engine-specific timing layout (crank, cams, high-pressure pump, tensioner) ----------
function timingCircles(kind) {
  const c = [
    { y: 0, z: 0, r: R_CRANK_SPROCKET, kind: 'crank' },
    { y: CAM_Y, z: -VALVE_Z, r: 0.145, kind: 'cam' },
    { y: CAM_Y, z: VALVE_Z, r: 0.145, kind: 'cam' },
    { y: PUMP[1], z: PUMP[2], r: 0.13, kind: 'pump' },
  ];
  if (kind !== 'chain') c.push({ y: 1.1, z: 0.3, r: 0.055, kind: 'idler' });
  return c;
}

export default {
  id: 'diesel',
  name: 'Motore diesel common rail',
  short: 'Diesel common rail',
  family: 'terra',
  vehicle: 'Auto, furgoni e camion',
  examples: 'Fiat Panda 1.3 Multijet, Volkswagen Golf TDI, Fiat Ducato, Iveco Daily e Stralis, trattori, quasi tutti i camion',
  tagline: 'Niente scintilla: l’aria viene compressa così tanto che il gasolio si accende da solo.',
  intro: {
    base:
      'Il diesel è un quattro tempi come il motore a benzina, ma <b>non ha candele</b>. Aspira solo aria e la schiaccia così tanto che diventa <b>rovente</b>; poi un iniettore spruzza il gasolio a pressione altissima e il carburante <b>si accende da solo</b>. Un turbocompressore, mosso dai gas di scarico, soffia dentro più aria. Risultato: consuma poco e ha tanta forza anche a bassi giri, per questo muove furgoni, camion e trattori.',
    expert:
      'Motore alternativo ad <b>accensione per compressione</b> (ciclo Diesel; il ciclo reale è più vicino al ciclo misto Sabathé). Rapporto di compressione 15–18:1, iniezione diretta <b>common rail</b> a 1 600–2 500 bar con iniettori piezo o a solenoide, sovralimentazione con turbocompressore e intercooler. Manca la farfalla: il carico si regola solo con la quantità di gasolio iniettata (miscela sempre magra, λ > 1), quindi le perdite di pompaggio sono piccole. Rendimento massimo 40–45% nelle auto, oltre il 45% nei motori da camion. Il rovescio della medaglia sono NOx e particolato, trattati con EGR, DPF e SCR con AdBlue.',
  },
  specs: [
    { label: 'Carburante', value: 'Gasolio (B7, HVO)' },
    { label: 'Ciclo', value: 'Diesel, 4 tempi, accensione per compressione' },
    { label: 'Cilindrata tipica', value: '1,2 – 3,0 l (auto/furgoni), 10 – 13 l (camion)' },
    { label: 'Rapporto di compressione', value: '15:1 – 18:1' },
    { label: 'Pressione di iniezione', value: '1 600 – 2 500 bar' },
    { label: 'Potenza tipica', value: '55 – 150 kW (auto), 300 – 450 kW (camion)' },
    { label: 'Distribuzione', value: 'Catena o cinghia (a secco o a bagno d’olio)' },
    { label: 'Rendimento massimo', value: '40 – 45 %' },
  ],
  metrics: { efficiency: 42, power: 110, powerDensity: 0.65 },
  variants: {
    columns: ['Motore', 'Cilindrata', 'Potenza', 'Distribuzione'],
    rows: [
      ['Fiat / Stellantis 1.3 Multijet', '1,25 l, 4 cilindri', '51 – 70 kW', 'Catena'],
      ['Renault 1.5 dCi (K9K)', '1,46 l, 4 cilindri', '50 – 85 kW', 'Cinghia a secco'],
      ['Volkswagen 2.0 TDI (EA288)', '1,97 l, 4 cilindri', '85 – 147 kW', 'Cinghia a secco'],
      ['Ford 2.0 EcoBlue', '2,0 l, 4 cilindri', '77 – 140 kW', 'Cinghia a bagno d’olio'],
      ['BMW 2.0 (B47)', '2,0 l, 4 cilindri', '85 – 170 kW', 'Catena (lato volano)'],
    ],
    note: 'Tutti common rail con turbocompressore; le potenze dipendono dalla versione e dall’anno.',
  },
  options: [
    {
      id: 'timing',
      label: 'Distribuzione',
      default: 'chain',
      choices: ['dryBelt', 'wetBelt', 'chain'].map((id) => ({ id, ...TIMING_TEXT[id] })),
    },
  ],
  thumb: { orbit: [-40, 20], zoom: 1.15, view: 'section' },
  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      range: [0, 0.25],
      parts: ['intakeValves', 'intakeManifold', 'turboRotor', 'intercooler', 'pistons'],
      base:
        'Il pistone scende e il cilindro si riempie di sola aria: nel diesel il carburante non entra da qui. Il turbo spinge l’aria dentro a pressione e l’intercooler la rinfresca, così nel cilindro ne entra di più, come quando gonfi un pallone con la pompa invece che col fiato.',
      expert:
        'Valvole di aspirazione aperte, pistone dal PMS al PMI (0–180°). Nessuna farfalla di regolazione del carico: il diesel riempie sempre il cilindro e dosa la potenza solo con il gasolio iniettato (λ ≈ 1,3–2 a pieno carico, molto più alto al minimo). Il compressore porta il collettore a 2–2,5 bar assoluti; l’intercooler raffredda l’aria compressa da ~150 °C a ~50 °C, aumentandone la densità. La valvola EGR può rimandare in aspirazione una parte dei gas di scarico raffreddati per abbassare gli NOx.',
    },
    {
      id: 'compression',
      name: 'Compressione',
      range: [0.25, 0.5],
      parts: ['pistons', 'cylinderHead', 'glowPlugs'],
      base:
        'Le valvole si chiudono e il pistone risale, schiacciando l’aria in uno spazio circa sedici volte più piccolo. L’aria si scalda moltissimo, come la pompa della bici che diventa calda mentre gonfi una gomma: alla fine è così rovente da accendere il gasolio da sola. A motore freddo aiutano le candelette.',
      expert:
        'Valvole chiuse, pistone dal PMI al PMS (180–360°). Con r ≈ 16:1 la compressione quasi adiabatica porta l’aria a ~700–900 °C e a 40–80 bar (di più con il turbo a pieno carico), ben sopra la temperatura di autoaccensione del gasolio (~250 °C). La camera di combustione è quasi tutta ricavata nella tazza del pistone. Poco prima del PMS arriva la pre-iniezione (pilota): pochi mm³ di gasolio che preparano la combustione e la rendono meno rumorosa.',
    },
    {
      id: 'power',
      name: 'Iniezione e combustione',
      range: [0.5, 0.75],
      parts: ['injectors', 'commonRail', 'pistons', 'conrods', 'crankshaft'],
      base:
        'Vicino al punto più alto l’iniettore spruzza il gasolio in getti finissimi, più sottili di un capello. Appena tocca l’aria rovente si accende da solo, senza scintilla: il gas caldissimo spinge giù il pistone con grande forza. Per questo il diesel ha tanta coppia anche a pochi giri.',
      expert:
        'Iniezione principale pochi gradi prima del PMS, con 6–8 getti da fori di ~0,1 mm a 1 600–2 500 bar. Dopo un ritardo d’accensione di ~0,5–1 ms la combustione è prima premiscelata e poi diffusiva, governata dalla legge di iniezione: per questo il ciclo ideale di riferimento è il Diesel (calore a pressione circa costante). Picco di pressione in camera 150–200 bar. Gli iniettori piezo eseguono fino a 8 iniezioni per ciclo (pilota, principale, post-iniezioni per il DPF).',
    },
    {
      id: 'exhaust',
      name: 'Scarico e turbo',
      range: [0.75, 1],
      parts: ['exhaustValves', 'exhaustManifold', 'turboRotor', 'turbocharger', 'dpf'],
      base:
        'Il pistone risale e spinge fuori i gas bruciati. Prima di uscire fanno girare la turbina del turbo, che sullo stesso albero fa girare il compressore dell’aria: l’energia che andrebbe persa viene riciclata. Poi il filtro antiparticolato trattiene la fuliggine.',
      expert:
        'Valvole di scarico aperte (540–720°). I gas a 300–700 °C espandono nella turbina, spesso a geometria variabile (VGT), che può superare i 200 000 giri/min. A valle: catalizzatore ossidante (DOC), filtro antiparticolato (DPF) che trattiene oltre il 95% del particolato e si rigenera bruciandolo a ~600 °C, e catalizzatore SCR che con l’AdBlue (urea al 32,5%) riduce gli NOx ad azoto e acqua.',
    },
  ],
  quiz: [
    {
      q: 'Che cosa accende il gasolio dentro il cilindro di un diesel?',
      options: ['Una candela con una scintilla', 'L’aria resa caldissima dalla compressione', 'Il turbo, che lo riscalda', 'Le candelette, sempre accese'],
      correct: 1,
      explain:
        'Il diesel comprime l’aria 15–18 volte: diventa così calda (700–900 °C) che il gasolio iniettato si accende da solo. Le candelette servono solo ad aiutare l’avviamento a freddo.',
    },
    {
      q: 'Come si regola la potenza di un diesel moderno?',
      options: [
        'Chiudendo una farfalla sull’aria, come nel benzina',
        'Cambiando il rapporto di compressione',
        'Variando la quantità di gasolio iniettata',
        'Accendendo più o meno candelette',
      ],
      correct: 2,
      explain:
        'Il diesel aspira sempre tutta l’aria che può e varia solo il gasolio: miscela sempre magra (λ > 1) e poche perdite di pompaggio, uno dei motivi del suo alto rendimento.',
    },
    {
      q: 'A quale pressione il common rail inietta il gasolio?',
      options: ['Circa 5 bar', 'Circa 50 bar', 'Tra 1 600 e 2 500 bar', 'Circa 20 000 bar'],
      correct: 2,
      explain:
        'La pompa ad alta pressione tiene il rail a 1 600–2 500 bar: così il gasolio esce da fori di un decimo di millimetro come nebbia finissima che si mescola bene con l’aria.',
    },
    {
      q: 'Che cosa fa girare la turbina del turbocompressore?',
      options: ['Una cinghia collegata all’albero motore', 'Un motorino elettrico', 'I gas di scarico', 'L’aria aspirata dal filtro'],
      correct: 2,
      explain:
        'I gas di scarico, ancora caldi e in pressione, fanno girare la turbina; sullo stesso albero c’è il compressore che spinge più aria nei cilindri. Energia recuperata invece che persa.',
    },
  ],
  // Officina (see CONTRACT.md): how each part comes off and what holds it; the guide suggests an order.
  // Simplified and educational, not a workshop manual.
  disassembly: {
    intro: 'Smontiamo il diesel dall’esterno verso l’interno: aria, scarico e turbo, impianto del gasolio, distribuzione e infine la testata, fino a vedere i pistoni con la loro tazza.',
    note: 'Sequenza semplificata a scopo didattico: in officina si segue il manuale del costruttore, con i suoi attrezzi e le sue coppie di serraggio.',
    parts: {
      airFilter: { dir: [-0.3, 0.6, -0.7], dist: 0.9 },
      intercooler: { dir: [-0.4, -0.1, -0.9], dist: 1.0 },
      dpf: { dir: [-1, -0.2, 0], dist: 0.9 },
      turbocharger: { after: ['dpf', 'airFilter', 'intercooler'], with: ['turboRotor'], dir: [-1, 0, 0], dist: 1.0 },
      exhaustNuts: { motion: 'unscrew', turns: 6, out: 0.08, dir: [0, 0, 1], dist: 0.4 },
      exhaustManifold: { after: ['exhaustNuts', 'turbocharger'], dir: [0, 0, 1], dist: 1.0 },
      commonRail: { dir: [0, 1, -0.3], dist: 0.9 },
      injectors: { after: ['commonRail'], dir: [0, 1, 0], dist: 1.1 },
      intakeManifold: { after: ['intercooler'], dir: [0, 0.1, -1], dist: 1.0 },
      glowPlugs: { after: ['intakeManifold'], motion: 'unscrew', turns: 6, out: 0.12, dir: [0, 0.72, -0.69], dist: 0.6 },
      camCoverBolts: { motion: 'unscrew', turns: 6, out: 0.1 },
      camCover: { after: ['camCoverBolts', 'injectors', 'commonRail'], dir: [0, 1, 0], dist: 0.8 },
      accessoryDrive: { after: ['turbocharger', 'dpf'], dir: [-1, 0, 0.4], dist: 0.9 },
      ...Object.fromEntries(
        TIMING.flatMap((k) => [
          [`timingCover_${k}`, { after: ['accessoryDrive'], dir: [-1, 0, 0], dist: 0.9 }],
          [`timing_${k}`, { after: [`timingCover_${k}`], dir: [-1, 0, 0], dist: 1.0 }],
        ])
      ),
      camshafts: { after: ['camCover', ...TIMING.map((k) => `timing_${k}`)], dir: [0, 1, 0], dist: 0.7 },
      intakeValves: { after: ['camshafts'], dir: [0, 1, 0], dist: 0.8 },
      exhaustValves: { after: ['camshafts'], dir: [0, 1, 0], dist: 0.8 },
      headBolts: { after: ['camCover'], motion: 'unscrew', turns: 8, out: 0.3, dist: 0.5, duration: 2.2 },
      cylinderHead: {
        after: ['headBolts', 'intakeManifold', 'exhaustManifold', ...TIMING.map((k) => `timing_${k}`)],
        with: ['intakeValves', 'exhaustValves', 'camshafts', 'glowPlugs'],
        dir: [0, 1, 0],
        dist: 1.0,
        duration: 1.2,
      },
      gaskets: { after: ['cylinderHead'], dir: [0, 1, 0], dist: 0.6 },
      block: { after: ['gaskets'] },
    },
    steps: [
      {
        id: 'air',
        name: 'Stacca filtro e intercooler',
        base: 'Si allentano le fascette e si tolgono la scatola del filtro dell’aria e l’intercooler con i suoi tubi: così il turbo resta libero.',
        expert: 'Manicotti e tubi della sovralimentazione sono tenuti da fascette o innesti rapidi. È buona norma tappare subito le bocche del turbo: un corpo estraneo aspirato dal compressore, che gira a oltre 100 000 giri/min, ne rovina le pale.',
        parts: ['airFilter', 'intercooler'],
      },
      {
        id: 'turbo',
        name: 'Togli DPF e turbo',
        base: 'Si stacca il tubo di scarico con il filtro antiparticolato, poi il turbocompressore con la sua girante: è fissato all’uscita del collettore di scarico.',
        expert: 'Prima si scollegano i tubicini del sensore di pressione differenziale e le sonde del DPF, poi le tubazioni di mandata e ritorno dell’olio del turbo: i suoi cuscinetti flottanti vivono dell’olio motore in pressione.',
        parts: ['dpf', 'turbocharger'],
      },
      {
        id: 'exhaust',
        name: 'Svita il collettore di scarico',
        base: 'Con la chiave si svitano i dadi che stringono il collettore alla testata, poi lo si sfila dai prigionieri.',
        expert: 'I dadi lavorano a centinaia di gradi e possono bloccarsi sui prigionieri: si allentano con cautela, per non spezzarli. La guarnizione metallica tra collettore e testata di norma si sostituisce.',
        parts: ['exhaustNuts', 'exhaustManifold'],
      },
      {
        id: 'fuel',
        name: 'Stacca rail e iniettori',
        base: 'Si toglie il rail con i tubi d’acciaio dell’alta pressione, poi si sfilano gli iniettori dai loro pozzetti insieme al cablaggio.',
        expert: 'Si lavora solo a motore fermo, quando la pressione nel rail si è scaricata. Le aperture dell’impianto vanno tappate subito: basta un granello di sporco a rovinare un iniettore. Ogni iniettore torna nel suo cilindro, perché la centralina ne conosce il codice di taratura.',
        parts: ['commonRail', 'injectors'],
      },
      {
        id: 'intake',
        name: 'Togli collettore e candelette',
        base: 'Si sfila il collettore di aspirazione dal fianco della testata; ora le candelette sono a portata di chiave e si svitano una per una.',
        expert: 'Con il collettore si stacca anche il condotto dell’EGR, di solito incrostato da un impasto di fuliggine e vapori d’olio. Le candelette hanno stelo lungo e filetto sottile: con gli anni i depositi carboniosi possono bloccarle nella testata, quindi si svitano con delicatezza.',
        parts: ['intakeManifold', 'glowPlugs'],
      },
      {
        id: 'cover',
        name: 'Togli il coperchio punterie',
        base: 'Si svitano le viti e si solleva il coperchio con la sua guarnizione: sotto compaiono i due alberi a camme.',
        expert: 'Le viti si allentano un po’ alla volta. La guarnizione in elastomero resta nella sua sede sul bordo del coperchio e va controllata o sostituita; nel coperchio c’è spesso anche il separatore dei vapori d’olio.',
        parts: ['camCoverBolts', 'camCover'],
      },
      {
        id: 'aux',
        name: 'Togli la cinghia servizi',
        base: 'Si allenta il tendicinghia e si toglie la cinghia esterna con l’alternatore e la puleggia: coprono il carter della distribuzione.',
        expert: 'Il tendicinghia automatico si ruota con una chiave per liberare la cinghia Poly-V; poi si smonta la puleggia smorzatrice sull’albero motore.',
        parts: ['accessoryDrive'],
      },
      {
        id: 'timing',
        name: 'Togli carter e distribuzione',
        base: 'Si toglie il carter laterale, poi si bloccano gli alberi nella posizione giusta e si sfila la catena (o la cinghia) con le sue ruote dentate.',
        expert: 'Con gli attrezzi di fasatura si bloccano albero motore e alberi a camme: al rimontaggio la fase deve tornare identica, perché nel diesel lo spazio tra valvole e pistone al punto morto superiore è minimo.',
        parts: [...TIMING.map((k) => `timingCover_${k}`), ...TIMING.map((k) => `timing_${k}`)],
      },
      {
        id: 'head',
        name: 'Solleva la testata',
        base: 'Si svitano le lunghe viti della testata e la si solleva tutta intera, con valvole e alberi a camme ancora al loro posto.',
        expert: 'Le viti si allentano poco per volta nell’ordine inverso a quello di serraggio, dall’esterno verso il centro, per non deformare la testata. In questo modello semplificato la testata esce completa di alberi a camme e valvole; in molti motori reali gli alberi a camme vanno tolti prima, perché coprono le viti.',
        parts: ['headBolts', 'cylinderHead'],
      },
      {
        id: 'gasket',
        name: 'Togli la guarnizione',
        base: 'Tolta la guarnizione della testata si vedono dall’alto i pistoni, ognuno con la sua tazza scavata: la camera di combustione del diesel.',
        expert: 'La guarnizione multistrato in acciaio non si riusa. Nel diesel se ne sceglie lo spessore misurando di quanto i pistoni sporgono dal piano del basamento: da quel valore dipende il rapporto di compressione.',
        parts: ['gaskets'],
      },
    ],
  },
  camera: { position: [-5.4, 3.0, 6.9], target: [0, 0.95, -0.1] },

  build(ctx) {
    const root = new THREE.Group();
    root.position.set(0.7, 0.2, 0); // centre the model (turbo end widens it on -X)

    // ---------- Materials ----------
    const castAlu = () => {
      const m = MAT.aluminum();
      m.color.set(0x9aa3ad);
      m.roughness = 0.62;
      m.metalness = 0.55;
      return m;
    };
    const ironMat = () => {
      const m = MAT.castIron();
      m.color.set(0x5f6670);
      return m;
    };
    const blockMat = cut(ironMat());
    const linerMat = cut(MAT.steel());
    const headMat = cut(castAlu());
    headMat.color.set(0x8e98a4);
    const gasketMat = cut(MAT.paint(0x2b2e33, 0.7));
    const boltMat = cut(MAT.steel());
    const steel = MAT.steel();
    const darkSteel = MAT.darkSteel();
    const rubber = MAT.rubber();
    const connMat = MAT.paint(0x3b3f45, 0.6);
    const kitMats = {
      steel,
      darkSteel,
      alu: MAT.aluminum(),
      dark: MAT.paint(0x22262b, 0.6),
      belt: (() => {
        const m = MAT.paint(0x141518, 0.9);
        m.metalness = 0;
        m.flatShading = true;
        m.side = THREE.DoubleSide;
        return m;
      })(),
      wetBelt: (() => {
        const m = MAT.paint(0x2a2320, 0.28);
        m.metalness = 0.15;
        m.flatShading = true;
        m.side = THREE.DoubleSide;
        return m;
      })(),
      guide: MAT.paint(0x2b2825, 0.6),
      guideTens: MAT.paint(0xb39566, 0.55),
      plastic: cut((() => {
        const m = MAT.paint(0x1a1d21, 0.75);
        m.metalness = 0.05;
        return m;
      })()),
      case: cut(castAlu()),
      bolt: boltMat,
    };

    // ---------- Block: deck floor, outer wall, siamese barrels (open-deck water jacket) ----------
    const block = new THREE.Group();
    const floor = rrShape(-BX, -BZ, BX, BZ, 0.1);
    BORE_X.forEach((x) => floor.holes.push(circleHole(x, 0, 0.348)));
    block.add(new THREE.Mesh(extY(floor, 0.5, 0.6, 40), blockMat));
    const wall = rrShape(-BX, -BZ, BX, BZ, 0.1);
    wall.holes.push(rrHole(-1.7, -0.46, 1.7, 0.46, 0.07));
    block.add(new THREE.Mesh(extY(wall, 0.6, HEAD_Y, 12), blockMat));
    const barrels = rrShape(-1.6, -0.4, 1.6, 0.4, 0.32);
    BORE_X.forEach((x) => barrels.holes.push(circleHole(x, 0, 0.348)));
    block.add(new THREE.Mesh(extY(barrels, 0.6, HEAD_Y, 40), blockMat));
    BORE_X.forEach((x) => block.add(mesh(pipe(0.348, 0.34, 1.05, 48), linerMat, x, 1.025, 0)));
    const coolant = rrShape(-1.695, -0.455, 1.695, 0.455, 0.07);
    coolant.holes.push(rrHole(-1.605, -0.405, 1.605, 0.405, 0.32));
    const coolantMesh = new THREE.Mesh(extY(coolant, 0.62, 1.5, 12), cut(MAT.gas(0x3fc9a8, 0.28, false), { cap: false }));
    coolantMesh.renderOrder = 2;
    block.add(coolantMesh);
    for (const x of [-0.8, 0, 0.8]) for (const s of [-1, 1]) block.add(rbox(0.06, 0.88, 0.05, 0.015, blockMat, x, 1.05, s * (BZ + 0.02)));
    const fittings = new THREE.Group();
    fittings.add(mesh(alongZ(new THREE.CylinderGeometry(0.06, 0.07, 0.05, 6)), darkSteel, 0.02, 1.0, BZ + 0.035));
    fittings.add(rod([-0.85, 0.35, BZ + 0.04], [-0.85, 1.4, BZ + 0.07], 0.016, steel, 8));
    fittings.add(mesh(new THREE.TorusGeometry(0.035, 0.012, 8, 20), MAT.paint(0xf2c230, 0.4), -0.85, 1.45, BZ + 0.07));
    fittings.add(mesh(alongZ(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 6)), steel, -0.85, 0.35, BZ + 0.02));
    // oil filter cartridge housing and oil cooler
    fittings.add(mesh(alongZ(new THREE.CylinderGeometry(0.12, 0.12, 0.05, 32)), castAlu(), 0.9, 0.1, BZ + 0.045));
    fittings.add(mesh(alongZ(lathe([[0, -0.13], [0.1, -0.13], [0.11, -0.11], [0.11, 0.11], [0.1, 0.13], [0, 0.13]], 32)), MAT.paint(0x1d4f8a, 0.4), 0.9, 0.1, BZ + 0.2));
    block.add(cutAll(fittings));
    root.add(
      ctx.part(block, {
        id: 'block',
        name: 'Basamento e cilindri',
        base: 'Il “corpo” del motore, con i quattro cilindri. Nel diesel è più robusto che nel benzina, perché dentro le pressioni sono quasi il doppio. Intorno ai cilindri circola il liquido di raffreddamento (in verde nella sezione).',
        expert:
          'Monoblocco in ghisa grigia o ghisa a grafite compatta (CGI) nei motori più sollecitati, oppure alluminio con canne in ghisa nelle auto. Deve reggere picchi di 180–200 bar in camera; le intercapedini dell’acqua circondano le canne. Sul fianco il supporto del filtro olio con lo scambiatore olio/acqua.',
        explode: [0, 0, 0],
      })
    );

    // ---------- Crankcase: side walls + main-bearing bulkheads ----------
    const crankcase = new THREE.Group();
    for (const s of [-1, 1]) crankcase.add(rbox(2 * BX, 1.0, 0.09, 0.02, blockMat, 0, 0, s * (BZ - 0.045)));
    const bulk = rrShape(-(BZ - 0.09), -0.5, BZ - 0.09, 0.5, 0.02);
    bulk.holes.push(circleHole(0, 0, 0.125));
    const mainsX = [-1.74, -0.8, 0, 0.8, 1.74];
    mainsX.forEach((x) => crankcase.add(new THREE.Mesh(extX(bulk, x - 0.06, x + 0.06, 32), blockMat)));
    const shellMat = MAT.paint(0xb08d57, 0.35);
    shellMat.metalness = 0.9;
    mainsX.forEach((x) => crankcase.add(mesh(alongX(pipe(0.125, 0.111, 0.1, 32)), shellMat, x, 0, 0)));
    for (const x of [-0.8, 0, 0.8]) for (const s of [-1, 1]) crankcase.add(rbox(0.06, 0.9, 0.05, 0.015, blockMat, x, 0, s * (BZ + 0.02)));
    const mounts = new THREE.Group();
    for (const x of [-1.25, 1.25]) {
      mounts.add(rbox(0.22, 0.2, 0.12, 0.02, blockMat, x, -0.15, BZ + 0.06));
      mounts.add(mesh(alongZ(new THREE.CylinderGeometry(0.07, 0.07, 0.1, 20)), rubber, x, -0.15, BZ + 0.17));
    }
    crankcase.add(cutAll(mounts));
    root.add(
      ctx.part(crankcase, {
        id: 'crankcase',
        name: 'Carter e supporti di banco',
        base: 'La parte bassa del motore che sostiene l’albero motore. Le pareti trasversali hanno un foro con un cuscinetto (in bronzo) in cui gira l’albero.',
        expert:
          'Paratie di banco robuste con cappelli o telaio integrale (bedplate): le forze di combustione del diesel sono molto più alte di quelle di un benzina di pari cilindrata. Bronzine a guscio sottile lubrificate a pressione.',
        explode: [0, -0.7, 0],
      })
    );

    // ---------- Oil pan (hollow) with oil, pickup and drain plug ----------
    const pan = new THREE.Group();
    const panMat = cut(MAT.darkSteel());
    const panWall = rrShape(-1.78, -0.54, 1.78, 0.54, 0.1);
    panWall.holes.push(rrHole(-1.71, -0.47, 1.71, 0.47, 0.06));
    pan.add(new THREE.Mesh(extY(panWall, -0.88, -0.53, 12), panMat));
    pan.add(new THREE.Mesh(extY(rrShape(-1.78, -0.54, 1.78, 0.54, 0.1), -0.94, -0.88, 12), panMat));
    const panFlange = rrShape(-1.84, -0.6, 1.84, 0.6, 0.12);
    panFlange.holes.push(rrHole(-1.71, -0.47, 1.71, 0.47, 0.06));
    pan.add(new THREE.Mesh(extY(panFlange, -0.53, -0.505, 12), panMat));
    const pb = [];
    for (let x = -1.6; x <= 1.61; x += 0.4) pb.push([x, -0.54, 0.57], [x, -0.54, -0.57]);
    pan.add(bolts(boltMat, pb, 'y'));
    pan.add(mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.03, 6), boltMat, 1.25, -0.955, 0.1));
    const oil = mesh(new THREE.BoxGeometry(3.38, 0.15, 0.92), cut(MAT.gas(0x5a3c0c, 0.6, false), { cap: false }), 0, -0.805, 0);
    oil.renderOrder = 2;
    pan.add(oil);
    pan.add(tube(curve([[0.25, -0.84, 0], [0.25, -0.6, 0], [0.12, -0.5, 0], [0.02, -0.46, 0]]), 0.022, steel, 20, 8));
    pan.add(mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.03, 24), darkSteel, 0.25, -0.85, 0));
    root.add(
      ctx.part(pan, {
        id: 'oilPan',
        name: 'Coppa dell’olio',
        base: 'La vaschetta sotto il motore che raccoglie l’olio che lubrifica tutte le parti in movimento. Nel diesel l’olio diventa scuro presto, per la fuliggine.',
        expert:
          'Coppa in lamiera o alluminio, 4–6 litri nelle auto. L’olio diesel (specifiche ACEA C) deve tenere in sospensione la fuliggine che passa oltre le fasce e avere poche ceneri per non intasare il DPF; nei motori con catena o cinghia in bagno d’olio la sua qualità protegge anche la distribuzione.',
        explode: [0, -1.5, 0],
      })
    );

    // ---------- Gaskets ----------
    const gaskets = new THREE.Group();
    const headGasket = rrShape(-BX, -BZ, BX, BZ, 0.1);
    BORE_X.forEach((x) => headGasket.holes.push(circleHole(x, 0, 0.352)));
    gaskets.add(new THREE.Mesh(extY(headGasket, HEAD_Y - 0.006, HEAD_Y + 0.006, 32), gasketMat));
    root.add(
      ctx.part(gaskets, {
        id: 'gaskets',
        name: 'Guarnizione della testata',
        base: 'Uno strato sottile tra testata e basamento che impedisce a gas, olio e acqua di uscire. Nel diesel lavora sotto pressioni enormi.',
        expert: 'Guarnizione multistrato in acciaio (MLS) con spessore scelto in base alla sporgenza dei pistoni, per ottenere il rapporto di compressione esatto.',
        explode: [0, 0.5, 0.2],
      })
    );

    // ---------- Cylinder head: flat deck, injector wells, jackets; skins; cam caps ----------
    const head = new THREE.Group();
    const corePts = [[-HX, HEAD_Y], [HX, HEAD_Y], [HX, HEAD_TOP], [HX - 0.1, HEAD_TOP], [HX - 0.1, GAL_Y]];
    [...BORE_X].reverse().forEach((c, i) => {
      // injector well: wide for the body, narrow bore for the nozzle down to the fire deck
      corePts.push([c + 0.06, GAL_Y], [c + 0.06, 1.7], [c + 0.03, 1.7], [c + 0.03, HEAD_Y + 0.012], [c - 0.03, HEAD_Y + 0.012], [c - 0.03, 1.7], [c - 0.06, 1.7], [c - 0.06, GAL_Y]);
      if (i < 3) {
        const t = c - 0.4;
        corePts.push([t + 0.05, GAL_Y], [t + 0.05, HEAD_TOP], [t - 0.05, HEAD_TOP], [t - 0.05, GAL_Y]);
      }
    });
    corePts.push([-HX + 0.1, GAL_Y], [-HX + 0.1, HEAD_TOP], [-HX, HEAD_TOP]);
    const core = polyShape(corePts);
    for (const c of BORE_X) for (const s of [-1, 1]) core.holes.push(rrHole(s > 0 ? c + 0.17 : c - 0.31, 1.6, s > 0 ? c + 0.31 : c - 0.17, 1.78, 0.03));
    head.add(new THREE.Mesh(extZ(core, -0.4, 0.4, 8), headMat));
    for (const s of [-1, 1]) head.add(rbox(2 * HX, HEAD_TOP - HEAD_Y, 0.06, 0.015, headMat, 0, (HEAD_Y + HEAD_TOP) / 2, s * (HZ - 0.03)));
    const capShape = rrShape(-0.3, HEAD_TOP, 0.3, 2.25, 0.03);
    for (const z of [-VALVE_Z, VALVE_Z]) capShape.holes.push(circleHole(-z, CAM_Y, 0.04));
    for (const x of CAP_X) head.add(new THREE.Mesh(extX(capShape, x - 0.045, x + 0.045, 16), headMat));
    const capBolts = [];
    for (const x of CAP_X) for (const z of [-0.26, 0, 0.26]) capBolts.push([x, 2.26, z]);
    head.add(bolts(boltMat, capBolts, 'y', new THREE.CylinderGeometry(0.03, 0.03, 0.025, 6)));
    // head bolts sit on raised bosses beside the cams, so they can be reached with the camshafts in place
    const HB_Y = 1.95;
    const headBolts = [];
    for (const c of BORE_X) for (const dx of [-0.3, 0.3]) for (const z of [-0.33, 0.33]) headBolts.push([c + dx, HB_Y + 0.015, z]);
    head.add(bolts(headMat, headBolts.map(([x, , z]) => [x, (GAL_Y + HB_Y) / 2, z]), 'y', new THREE.CylinderGeometry(0.052, 0.058, HB_Y - GAL_Y, 14)));
    // exhaust manifold studs (the nuts are their own part)
    const exStuds = BORE_X.flatMap((x) => [[x - 0.2, 1.83], [x + 0.2, 1.83]]);
    head.add(bolts(boltMat, exStuds.map(([x, y]) => [x, y, HZ + 0.035]), 'z', new THREE.CylinderGeometry(0.013, 0.013, 0.11, 8)));
    head.add(rbox(0.1, 0.16, 0.2, 0.03, headMat, HX + 0.05, 1.82, 0.12));
    root.add(
      ctx.part(head, {
        id: 'cylinderHead',
        name: 'Testata',
        base: 'Il “coperchio” dei cilindri: contiene le valvole, gli iniettori al centro e le candelette. Nel diesel il fondo è piatto, perché la camera di scoppio è scavata nel pistone. I fori nella sezione sono i passaggi dell’acqua.',
        expert:
          'Testata in lega di alluminio a 4 valvole per cilindro con iniettore centrale verticale (nel suo pozzetto) e condotti di aspirazione che imprimono all’aria un moto vorticoso (swirl). Piano di combustione piatto: la camera è la tazza del pistone. In alto la galleria degli alberi a camme con i supporti e i cappelli imbullonati.',
        explode: [0, 1.0, 0],
      }),
      ctx.part(bolts(boltMat, headBolts, 'y', screw(0.6, 0.04, 0.03)), {
        id: 'headBolts',
        name: 'Viti della testata',
        base: 'Sedici viti lunghe e robuste che stringono la testata al basamento, schiacciando la guarnizione in mezzo. Nel diesel devono reggere pressioni di combustione altissime.',
        expert: 'Viti ad alta resistenza che attraversano la testata e si avvitano nel basamento. Si serrano in più passate e in un ordine preciso, a spirale dal centro verso l’esterno, e si allentano nell’ordine inverso, per non deformare la testata. Spesso sono a snervamento (serraggio ad angolo) e vanno sostituite a ogni smontaggio.',
        explode: [0, 1.0, 0],
      })
    );

    // ---------- Cam cover: hollow U shell + end plates ----------
    const cover = new THREE.Group();
    const coverMat = cut(MAT.paint(0x1f2733, 0.55));
    const u = polyShape([
      [-1.7, HEAD_TOP], [-1.7, COVER_TOP - 0.05], [-1.65, COVER_TOP], [1.65, COVER_TOP], [1.7, COVER_TOP - 0.05], [1.7, HEAD_TOP],
      [1.64, HEAD_TOP], [1.64, COVER_TOP - 0.06], [-1.64, COVER_TOP - 0.06], [-1.64, HEAD_TOP],
    ]);
    cover.add(new THREE.Mesh(extZ(u, -0.36, 0.36, 4), coverMat));
    for (const s of [-1, 1]) cover.add(rbox(3.4, COVER_TOP - HEAD_TOP, 0.06, 0.02, coverMat, 0, (HEAD_TOP + COVER_TOP) / 2, s * 0.39));
    for (const z of [-0.26, 0.26]) cover.add(rbox(3.0, 0.03, 0.04, 0.01, coverMat, 0, COVER_TOP + 0.012, z));
    const cvb = [];
    for (let x = -1.5; x <= 1.51; x += 0.5) cvb.push([x, COVER_TOP + 0.01, -0.34], [x, COVER_TOP + 0.01, 0.34]);
    // elastomer gasket: it stays in its groove on the cover rim and comes off with it
    const coverGasket = rrShape(-1.7, -0.42, 1.7, 0.42, 0.06);
    coverGasket.holes.push(rrHole(-1.64, -0.36, 1.64, 0.36, 0.04));
    cover.add(new THREE.Mesh(extY(coverGasket, HEAD_TOP - 0.006, HEAD_TOP + 0.006, 8), gasketMat));
    cover.add(mesh(new THREE.CylinderGeometry(0.07, 0.075, 0.05, 24), MAT.paint(0x111317, 0.6), 1.5, COVER_TOP + 0.025, 0.17));
    const plate = mesh(new THREE.PlaneGeometry(0.5, 0.125).rotateX(-Math.PI / 2), cut(new THREE.MeshStandardMaterial({ map: plateTexture(['MOTORILAB 1.6 CRD', '16V · common rail']), metalness: 0.6, roughness: 0.4 }), { cap: false }), -0.8, COVER_TOP + 0.003, 0.17);
    cover.add(plate);
    root.add(
      ctx.part(cover, {
        id: 'camCover',
        name: 'Coperchio punterie',
        base: 'Chiude in alto il motore e tiene dentro l’olio che bagna gli alberi a camme. Gli iniettori lo attraversano.',
        expert: 'Coperchio in plastica o alluminio con guarnizione in elastomero sul bordo, separatore dei vapori d’olio (blow-by) e pozzetti a tenuta per gli iniettori.',
        explode: [0, 2.1, 0],
      }),
      ctx.part(bolts(boltMat, cvb, 'y', screw(0.38)), {
        id: 'camCoverBolts',
        name: 'Viti del coperchio',
        base: 'Le viti che fissano il coperchio punterie alla testata, tutto intorno al bordo.',
        expert: 'Si serrano poco e in modo uniforme: devono solo schiacciare la guarnizione in elastomero quanto basta per la tenuta dell’olio, senza deformare il coperchio.',
        explode: [0, 2.1, 0],
      })
    );

    // ---------- Crankshaft ----------
    const crankMat = MAT.steel();
    const crank = new THREE.Group();
    crank.add(mesh(alongX(new THREE.CylinderGeometry(0.11, 0.11, 3.5, 28)), crankMat));
    crank.add(mesh(alongX(new THREE.CylinderGeometry(0.065, 0.065, 0.44, 20)), crankMat, -1.96, 0, 0));
    crank.add(mesh(alongX(new THREE.CylinderGeometry(0.16, 0.16, 0.06, 32)), crankMat, 1.8, 0, 0));
    const pinGeo = alongX(new THREE.CylinderGeometry(0.1, 0.1, 0.26, 24));
    const webGeo = new THREE.BoxGeometry(0.08, 0.64, 0.34);
    const cwGeo = alongX(new THREE.CylinderGeometry(0.3, 0.3, 0.08, 32, 1, false, Math.PI / 2, Math.PI));
    BORE_X.forEach((x, k) => {
      const s = k === 0 || k === 3 ? 1 : -1;
      crank.add(mesh(pinGeo, crankMat, x, s * R, 0));
      for (const dx of [-0.165, 0.165]) {
        crank.add(mesh(webGeo, crankMat, x + dx, s * 0.13, 0));
        const cw = mesh(cwGeo, crankMat, x + dx, 0, 0);
        cw.rotation.x = s > 0 ? 0 : Math.PI;
        crank.add(cw);
      }
    });
    root.add(
      ctx.part(crank, {
        id: 'crankshaft',
        name: 'Albero motore',
        base: 'L’albero “a gomiti” che trasforma il su-e-giù dei pistoni in rotazione, come i pedali della bicicletta.',
        expert:
          'Albero in acciaio forgiato e temprato a induzione su 5 supporti di banco, manovelle a 180°. Nel diesel perni e cuscinetti sono più grandi per reggere picchi di combustione di 150–200 bar; spesso ci sono alberi di equilibratura per le forze del secondo ordine.',
        explode: [0, -1.1, 0.4],
      })
    );

    // ---------- Pistons with bowl, conrods ----------
    const pistonGeo = lathe(
      [
        [0, 0.12], [0.05, 0.1], [0.12, 0.088], [0.165, 0.13], [0.158, CROWN], [0.32, CROWN],
        [0.33, CROWN - 0.01], [0.33, -0.22], [0.31, -0.22], [0.31, 0.05], [0, 0.05],
      ],
      48
    );
    const ringGeo = new THREE.TorusGeometry(0.331, 0.008, 6, 48).rotateX(Math.PI / 2);
    const wristGeo = alongX(new THREE.CylinderGeometry(0.07, 0.07, 0.6, 16));
    const pistonMat = MAT.aluminum();
    const pistonGroup = new THREE.Group();
    const pistons = BORE_X.map((x) => {
      const p = new THREE.Group();
      p.add(new THREE.Mesh(pistonGeo, pistonMat));
      for (const y of [CROWN - 0.03, CROWN - 0.065, CROWN - 0.1]) p.add(mesh(ringGeo, darkSteel, 0, y, 0));
      p.add(new THREE.Mesh(wristGeo, steel));
      p.position.x = x;
      pistonGroup.add(p);
      return p;
    });
    root.add(
      ctx.part(pistonGroup, {
        id: 'pistons',
        name: 'Pistoni con camera a tazza',
        base: 'I “tappi mobili” dei cilindri. Sulla testa hanno una conca, la tazza: è lì che il gasolio spruzzato vortica con l’aria e brucia.',
        expert:
          'Pistoni in lega Al-Si (o acciaio nei camion) con camera di combustione a omega ricavata nel cielo, rientrante per esaltare la turbolenza (squish). Getti d’olio sotto il cielo e galleria di raffreddamento interna smaltiscono il calore; il primo anello di tenuta è in un inserto riportato.',
        explode: [0, 0.2, 1.5],
      })
    );

    const rodBeam = new THREE.BoxGeometry(0.06, L - 0.2, 0.1).translate(0, L / 2, 0);
    const bigEnd = new THREE.TorusGeometry(0.12, 0.038, 10, 28).rotateY(Math.PI / 2);
    const smallEnd = new THREE.TorusGeometry(0.085, 0.028, 10, 24).rotateY(Math.PI / 2).translate(0, L, 0);
    const rodGroup = new THREE.Group();
    const rods = BORE_X.map((x) => {
      const r = new THREE.Group();
      r.add(new THREE.Mesh(rodBeam, steel), new THREE.Mesh(bigEnd, steel), new THREE.Mesh(smallEnd, steel));
      r.position.x = x;
      rodGroup.add(r);
      return r;
    });
    root.add(
      ctx.part(rodGroup, {
        id: 'conrods',
        name: 'Bielle',
        base: 'Le “braccia” che collegano ogni pistone all’albero motore.',
        expert: 'Bielle in acciaio forgiato con testa divisa per frattura (cracking) e piede trapezoidale, per dare più appoggio allo spinotto che riceve la spinta della combustione.',
        explode: [0, -0.3, 1.2],
      })
    );

    // ---------- Valvetrain: 4 valves per cylinder ----------
    const valveGeo = lathe([[0, 0], [0.085, 0], [0.085, 0.012], [0.028, 0.05], [0.02, 0.09], [0.02, 0.5], [0, 0.5]], 24);
    const springGeo = new THREE.TubeGeometry(
      new (class extends THREE.Curve {
        getPoint(t, out = new THREE.Vector3()) {
          const a = t * TAU * 5;
          return out.set(Math.cos(a) * 0.04, t * 0.2, Math.sin(a) * 0.04);
        }
      })(),
      90,
      0.007,
      6
    );
    const bucketGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.06, 20);
    const bucketMat = MAT.chrome();
    const makeValves = (z, matValve) => {
      const g = new THREE.Group();
      const list = BORE_X.map((x) => {
        const pair = new THREE.Group();
        for (const dx of [-VALVE_DX, VALVE_DX]) {
          const v = new THREE.Group();
          v.add(new THREE.Mesh(valveGeo, matValve), mesh(springGeo, darkSteel, 0, 0.3, 0), mesh(bucketGeo, bucketMat, 0, 0.53, 0));
          v.position.x = dx;
          pair.add(v);
        }
        pair.position.set(x, HEAD_Y - 0.005, z);
        g.add(pair);
        return pair;
      });
      return { g, list };
    };
    const inV = makeValves(-VALVE_Z, MAT.steel());
    const exV = makeValves(VALVE_Z, MAT.titanium());
    root.add(
      ctx.part(inV.g, {
        id: 'intakeValves',
        name: 'Valvole di aspirazione',
        base: 'Due per cilindro: si aprono per far entrare l’aria (solo aria!) spinta dal turbo.',
        expert: 'Due valvole di aspirazione per cilindro; spesso uno dei due condotti è elicoidale e l’altro tangenziale (o parzializzabile) per regolare lo swirl a seconda di giri e carico.',
        explode: [0, 0.6, -0.5],
      }),
      ctx.part(exV.g, {
        id: 'exhaustValves',
        name: 'Valvole di scarico',
        base: 'Due per cilindro: si aprono per far uscire i gas bruciati verso il turbo.',
        expert: 'Valvole in acciaio austenitico resistente al calore, con sede riportata in stellite; i gas allo scarico di un diesel sono più freddi che in un benzina (300–700 °C).',
        explode: [0, 0.6, 0.5],
      })
    );

    const lobeGeo = camLobeGeometry();
    const camLen = 1.75 - XT + 0.03;
    const camShaftGeo = alongX(new THREE.CylinderGeometry(0.035, 0.035, camLen, 16));
    const makeCam = (z, peak) => {
      const g = new THREE.Group();
      g.add(mesh(camShaftGeo, steel, (1.75 + XT - 0.03) / 2, 0, 0));
      BORE_X.forEach((x, k) => {
        for (const dx of [-VALVE_DX, VALVE_DX]) {
          const lobe = mesh(lobeGeo, MAT.chrome(), x + dx, 0, 0);
          lobe.rotation.x = -TAU * (OFFSET[k] + peak);
          g.add(lobe);
        }
      });
      for (const x of CAP_X) g.add(mesh(alongX(new THREE.CylinderGeometry(0.04, 0.04, 0.08, 16)), steel, x, 0, 0));
      g.position.set(0, CAM_Y, z);
      return g;
    };
    const camIn = makeCam(-VALVE_Z, 0.125);
    const camEx = makeCam(VALVE_Z, 0.875);
    const cams = new THREE.Group();
    cams.add(camIn, camEx);
    root.add(
      ctx.part(cams, {
        id: 'camshafts',
        name: 'Alberi a camme',
        base: 'Alberi con delle “gobbe”: girando spingono giù le valvole al momento giusto. Girano a metà velocità dell’albero motore.',
        expert: 'Doppio albero a camme in testa (DOHC), 16 valvole. Rapporto 2:1 con l’albero motore; le camme comandano le valvole tramite punterie idrauliche o bilancieri a rullo.',
        explode: [0, 1.7, 0],
      })
    );

    // ---------- Injectors (central, vertical) ----------
    const injectors = new THREE.Group();
    const injBodyGeo = lathe([[0, 0], [0.012, 0], [0.02, 0.03], [0.02, 0.22], [0.04, 0.25], [0.04, 0.86], [0, 0.86]], 20);
    const injTopGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.22, 20);
    const injTopMat = MAT.paint(0x1c2026, 0.5);
    BORE_X.forEach((x) => {
      const inj = new THREE.Group();
      inj.add(new THREE.Mesh(injBodyGeo, steel), mesh(injTopGeo, injTopMat, 0, 0.97, 0), rbox(0.1, 0.06, 0.06, 0.012, injTopMat, 0.07, 1.02, 0));
      inj.position.set(x, INJ_TIP, 0);
      injectors.add(inj);
    });
    injectors.add(tube(curve([[-1.75, COVER_TOP + 0.2, 0.12], [-1.2, COVER_TOP + 0.24, 0.1], [0, COVER_TOP + 0.25, 0.1], [1.25, COVER_TOP + 0.24, 0.1]]), 0.018, rubber, 40, 8));
    BORE_X.forEach((x) => injectors.add(rod([x + 0.12, INJ_TIP + 1.02, 0], [x + 0.12, COVER_TOP + 0.24, 0.1], 0.01, rubber, 6)));
    root.add(
      ctx.part(injectors, {
        id: 'injectors',
        name: 'Iniettori',
        base: 'Spruzzatori comandati dalla centralina: al momento giusto aprono per un millesimo di secondo e nebulizzano il gasolio direttamente nel cilindro. I fili neri portano il comando elettrico.',
        expert:
          'Iniettori piezoelettrici o a solenoide con polverizzatore a 6–8 fori da ~0,1 mm, alloggiati in un pozzetto della testata. Si aprono in ~0,1 ms ed eseguono più iniezioni per ciclo: pilota (riduce rumore e NOx), principale, post-iniezioni (alzano la temperatura dei gas per rigenerare il DPF).',
        explode: [0, 1.9, 0.3],
      })
    );

    // ---------- Common rail + lines ----------
    const railGroup = new THREE.Group();
    const railMat = MAT.chrome();
    railGroup.add(mesh(alongX(new THREE.CylinderGeometry(0.055, 0.055, 2.95, 24)), railMat, -0.05, RAIL_Y, RAIL_Z));
    railGroup.add(mesh(alongX(new THREE.CylinderGeometry(0.035, 0.035, 0.12, 16)), injTopMat, 1.48, RAIL_Y, RAIL_Z));
    railGroup.add(rbox(0.12, 0.1, 0.1, 0.015, injTopMat, -1.58, RAIL_Y, RAIL_Z));
    for (const x of [-0.8, 0.8]) railGroup.add(rbox(0.06, 0.22, 0.08, 0.01, castAlu(), x, RAIL_Y - 0.13, RAIL_Z));
    BORE_X.forEach((x) => railGroup.add(rod([x, RAIL_Y, RAIL_Z], [x, INJ_TIP + 1.04, -0.04], 0.014, steel, 8)));
    const hpCurve = curve([
      [PUMP[0] + 0.3, PUMP[1] + 0.12, PUMP[2] + 0.02],
      [-1.62, 1.6, -0.64],
      [-1.62, 2.3, -0.56],
      [-1.6, RAIL_Y - 0.1, -0.36],
      [-1.5, RAIL_Y - 0.02, RAIL_Z],
    ]);
    railGroup.add(tube(hpCurve, 0.018, steel, 40, 8));
    root.add(
      ctx.part(railGroup, {
        id: 'commonRail',
        name: 'Rail (collettore comune) e tubi ad alta pressione',
        base: 'Un tubo d’acciaio robustissimo dove il gasolio aspetta sotto pressione enorme, pronto per tutti gli iniettori: da qui il nome “common rail”, binario comune.',
        expert:
          'Accumulatore forgiato che tiene il gasolio a pressione costante indipendentemente dai giri (1 600–2 500 bar). Il sensore di pressione e la valvola regolatrice permettono alla centralina di scegliere pressione, anticipo e numero di iniezioni in modo indipendente: è il grande vantaggio rispetto alle vecchie pompe meccaniche.',
        explode: [0, 2.6, -0.4],
      })
    );

    // ---------- High-pressure pump (its sprocket/pulley belongs to the timing drive) ----------
    const pump = new THREE.Group();
    pump.add(mesh(alongX(lathe([[0, -0.16], [0.11, -0.16], [0.12, -0.12], [0.12, 0.12], [0.1, 0.16], [0, 0.16]], 32)), MAT.aluminum(), PUMP[0] + 0.24, PUMP[1], PUMP[2]));
    pump.add(rbox(0.16, 0.12, 0.16, 0.02, MAT.aluminum(), PUMP[0] + 0.3, PUMP[1] + 0.09, PUMP[2]));
    pump.add(rbox(0.06, 0.05, 0.06, 0.01, injTopMat, PUMP[0] + 0.36, PUMP[1] + 0.02, PUMP[2] - 0.13));
    pump.add(bolts(steel, [[PUMP[0] + 0.1, PUMP[1] + 0.12, PUMP[2]], [PUMP[0] + 0.1, PUMP[1] - 0.12, PUMP[2]]], 'x'));
    root.add(
      ctx.part(pump, {
        id: 'hpPump',
        name: 'Pompa ad alta pressione',
        base: 'Mossa dalla distribuzione del motore, prende il gasolio dal serbatoio e lo pompa nel rail fino a oltre mille volte la pressione di una gomma.',
        expert:
          'Pompa a pistoncini radiali o in linea (es. Bosch CP1/CP4) mossa dalla catena o dalla cinghia della distribuzione; una valvola dosatrice sull’ingresso regola quanto gasolio comprimere per tenere nel rail la pressione chiesta dalla centralina. Il gasolio la lubrifica: per questo la qualità del carburante è critica.',
        explode: [-0.3, -0.3, -0.9],
      })
    );

    // ---------- Glow plugs + supply wire ----------
    const glowPlugs = new THREE.Group();
    const gpDir = V(0, 0.495, -0.475).normalize();
    const gpTipMat = MAT.darkSteel();
    gpTipMat.emissive.setHex(0xff5a14);
    gpTipMat.emissiveIntensity = 0;
    // Instanced (local +Y along the plug axis, out of the head) so the wrench can unscrew them.
    const gpQuat = new THREE.Quaternion().setFromUnitVectors(UP, gpDir);
    const gpInst = (geo, mat) => {
      const m = new THREE.InstancedMesh(geo, mat, BORE_X.length);
      const mtx = new THREE.Matrix4();
      BORE_X.forEach((x, i) => m.setMatrixAt(i, mtx.compose(V(x, HEAD_Y - 0.035, -0.075), gpQuat, V(1, 1, 1))));
      glowPlugs.add(m);
      return m;
    };
    const gpTips = [gpInst(new THREE.CylinderGeometry(0.012, 0.015, 0.12, 12).translate(0, 0.06, 0), gpTipMat)];
    gpInst(mergeGeometries([new THREE.CylinderGeometry(0.022, 0.022, 0.48, 12).translate(0, 0.36, 0), new THREE.CylinderGeometry(0.012, 0.012, 0.08, 8).translate(0, 0.66, 0)]), steel);
    gpInst(new THREE.CylinderGeometry(0.04, 0.04, 0.05, 6).translate(0, 0.6, 0), bucketMat);
    const gpEnd = V(0, HEAD_Y - 0.035, -0.075).addScaledVector(gpDir, 0.7);
    glowPlugs.add(tube(curve([[-1.55, gpEnd.y + 0.02, gpEnd.z - 0.04], [0, gpEnd.y + 0.03, gpEnd.z - 0.04], [1.3, gpEnd.y + 0.02, gpEnd.z - 0.04]]), 0.012, MAT.copper(), 40, 6));
    glowPlugs.add(rbox(0.1, 0.06, 0.08, 0.01, connMat, -1.6, gpEnd.y + 0.02, gpEnd.z - 0.04));
    root.add(
      ctx.part(glowPlugs, {
        id: 'glowPlugs',
        name: 'Candelette',
        base: 'Non sono candele! Sono piccole resistenze che diventano incandescenti per qualche secondo quando il motore è freddo, per aiutare la prima accensione. Le vedi accese solo al minimo, come in un avviamento a freddo.',
        expert:
          'Candelette ceramiche o metalliche a 11 V che raggiungono 1 000–1 300 °C in 2–5 s, alimentate da una centralina dedicata tramite la barretta di collegamento. Restano alimentate qualche minuto dopo l’avviamento (post-riscaldamento) per ridurre rumore, fumo bianco e HC. A motore caldo l’accensione è dovuta solo alla compressione.',
        explode: [0, 0.9, -1.1],
      })
    );

    // ---------- Gas charges and fuel sprays (effects, not parts) ----------
    const chargeGeo = new THREE.CylinderGeometry(0.335, 0.335, 1, 40);
    const charges = BORE_X.map((x) => {
      const m = mesh(chargeGeo, MAT.gas(0x5fd3ff, 0.2, false), x);
      m.renderOrder = 2;
      root.add(m);
      return m;
    });
    const jetGeo = new THREE.ConeGeometry(0.03, 0.17, 10, 1, true).translate(0, -0.085, 0); // apex at origin, opens toward -Y
    const sprays = BORE_X.map((x) => {
      const g = new THREE.Group();
      const mat = MAT.glow(0xffc04d, 2.4, 0.85);
      mat.side = THREE.DoubleSide;
      const jets = [];
      for (let j = 0; j < 7; j++) {
        const a = (j / 7) * TAU + 0.2;
        const m = new THREE.Mesh(jetGeo, mat);
        m.quaternion.setFromUnitVectors(V(0, -1, 0), V(Math.cos(a) * 0.91, -0.41, Math.sin(a) * 0.91).normalize());
        g.add(m);
        jets.push(m);
      }
      g.position.set(x, INJ_TIP, 0);
      g.visible = false;
      root.add(g);
      return { g, jets, mat };
    });
    const fireLights = BORE_X.map((x) => {
      const l = new THREE.PointLight(0xff8a3a, 0, 2.2, 2);
      l.position.set(x, HEAD_Y - 0.12, 0.15);
      root.add(l);
      return l;
    });

    // ---------- Intake manifold ----------
    const intake = new THREE.Group();
    const intakeMat = MAT.paint(0x2a3442, 0.5);
    intake.add(mesh(alongX(new THREE.CylinderGeometry(0.22, 0.22, 3.0, 32)), intakeMat, 0, 1.05, -1.15));
    for (const x of [-1.5, 1.5]) intake.add(mesh(alongX(new THREE.CylinderGeometry(0.23, 0.23, 0.04, 32)), intakeMat, x, 1.05, -1.15));
    intake.add(mesh(alongX(new THREE.CylinderGeometry(0.16, 0.16, 0.3, 32)), MAT.aluminum(), -1.65, 1.05, -1.15));
    intake.add(rbox(0.2, 0.16, 0.18, 0.03, MAT.paint(0x15191e, 0.5), -1.65, 1.27, -1.15));
    intake.add(rbox(0.1, 0.06, 0.08, 0.015, MAT.paint(0x15191e, 0.5), 0.6, 1.29, -1.15));
    intake.add(rbox(2.9, 0.16, 0.04, 0.02, intakeMat, 0, 1.74, -HZ - 0.02));
    const intakePaths = BORE_X.map((x) => {
      const pts = [[x, 1.05, -1.15], [x, 1.5, -1.05], [x, 1.8, -0.8], [x, 1.74, -0.5]];
      intake.add(tube(curve(pts), 0.1, intakeMat, 30, 14));
      return curve([...pts, [x, 1.7, -0.35], [x, HEAD_Y + 0.08, -VALVE_Z - 0.05]]);
    });
    root.add(
      ctx.part(intake, {
        id: 'intakeManifold',
        name: 'Collettore di aspirazione e valvola EGR',
        base: 'I tubi che distribuiscono ai cilindri l’aria compressa dal turbo. Il blocchetto nero è la valvola EGR, che rimescola un po’ di gas di scarico nell’aria per inquinare meno.',
        expert:
          'Plenum con condotti singoli; la farfalla non regola il carico ma serve a creare depressione per l’EGR e a spegnere il motore senza scossoni. L’EGR (fino al 30–40% ai carichi parziali) abbassa la temperatura di combustione e quindi gli NOx; il sensore MAP misura la pressione di sovralimentazione.',
        explode: [0, 0.3, -1.3],
      })
    );

    // ---------- Exhaust manifold ----------
    const exhaust = new THREE.Group();
    const exMat = cut(MAT.paint(0x6b4a3a, 0.55));
    exMat.metalness = 0.8;
    exhaust.add(rbox(2.9, 0.16, 0.04, 0.02, exMat, 0, 1.79, HZ + 0.02));
    const exhaustPaths = BORE_X.map((x) => {
      const pts = [[x, 1.79, HZ + 0.03], [x, 1.76, 0.58], [x - 0.12, 1.72, 0.66]];
      exhaust.add(tube(curve(pts), 0.075, exMat, 20, 12));
      return curve([[x, HEAD_Y + 0.08, VALVE_Z + 0.05], [x, 1.76, 0.33], ...pts, [-1.6, 1.72, 0.66]]);
    });
    const collectorPts = [[1.35, 1.72, 0.66], [-1.6, 1.72, 0.66], [-2.3, 1.7, 0.6], [-2.88, 1.62, 0.38], [TX, 1.5, 0.14]];
    exhaust.add(tube(curve(collectorPts), 0.09, exMat, 80, 14));
    root.add(
      ctx.part(exhaust, {
        id: 'exhaustManifold',
        name: 'Collettore di scarico',
        base: 'Raccoglie i gas bruciati dei quattro cilindri e li porta alla turbina del turbo.',
        expert: 'Collettore in ghisa al silicio-molibdeno o acciaio inox imbullonato alla testata con prigionieri, il più corto possibile per portare alla turbina gas caldi e pulsanti (sovralimentazione a impulsi).',
        explode: [0, 0.1, 1.6],
      }),
      // flanged nuts on the head studs, local +Y out of the flange (+Z)
      ctx.part(bolts(boltMat, exStuds.map(([x, y]) => [x, y, HZ + 0.063]), 'z', mergeGeometries([new THREE.CylinderGeometry(0.03, 0.03, 0.03, 6), new THREE.CylinderGeometry(0.036, 0.036, 0.008, 12).translate(0, -0.019, 0)])), {
        id: 'exhaustNuts',
        name: 'Dadi del collettore di scarico',
        base: 'I dadi che stringono il collettore di scarico contro la testata. Sono avvitati su perni filettati, i prigionieri, che restano piantati nella testata.',
        expert: 'Dadi flangiati, spesso ramati o autobloccanti, su prigionieri in acciaio: lavorano a centinaia di gradi e a ogni avviamento subiscono dilatazioni e ritiri, per questo col tempo tendono a bloccarsi sul filetto.',
        explode: [0, 0.1, 1.6],
      })
    );

    // ---------- Turbocharger: housings (cut) ----------
    const turbo = new THREE.Group();
    const compMat = cut(MAT.aluminum());
    const turbMat = cut(MAT.castIron());
    turbMat.color.set(0x5b4c48);
    const chMat = cut(MAT.darkSteel());
    turbo.add(mesh(new THREE.TorusGeometry(0.25, 0.085, 14, 56).rotateY(Math.PI / 2), compMat, CX, TY, 0));
    turbo.add(mesh(alongX(new THREE.CylinderGeometry(0.3, 0.3, 0.035, 48)), compMat, CX - 0.05, TY, 0));
    turbo.add(mesh(alongX(lathe([[0.175, 0], [0.19, 0], [0.19, 0.04], [0.125, 0.15], [0.11, 0.24], [0.11, 0.29], [0.095, 0.29], [0.095, 0.24], [0.112, 0.15], [0.175, 0.03]], 48)), compMat, CX - 0.02, TY, 0));
    turbo.add(mesh(alongX(new THREE.CylinderGeometry(0.1, 0.1, 0.36, 32)), chMat, (CX + TX) / 2 - 0.005, TY, 0));
    turbo.add(mesh(new THREE.TorusGeometry(0.23, 0.095, 14, 56).rotateY(Math.PI / 2), turbMat, TX, TY, 0));
    turbo.add(mesh(alongX(new THREE.CylinderGeometry(0.3, 0.3, 0.035, 48)), turbMat, TX + 0.055, TY, 0));
    turbo.add(mesh(alongX(lathe([[0.165, 0], [0.18, 0], [0.18, 0.04], [0.14, 0.16], [0.14, 0.21], [0.125, 0.21], [0.125, 0.16], [0.158, 0.03]], 48)).rotateY(Math.PI), turbMat, TX + 0.02, TY, 0));
    turbo.add(mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.1, 20), MAT.paint(0x2d3239, 0.5), TX + 0.05, TY - 0.05, -0.38));
    turbo.add(bolts(boltMat, Array.from({ length: 8 }, (_, i) => [CX - 0.07, TY + Math.cos((i / 8) * TAU) * 0.27, Math.sin((i / 8) * TAU) * 0.27]), 'x'));
    root.add(
      ctx.part(turbo, {
        id: 'turbocharger',
        name: 'Turbocompressore (chiocciole)',
        base: 'Due “chiocciole” unite: in quella scura passano i gas di scarico, in quella chiara l’aria fresca. In mezzo c’è il supporto dell’albero che le collega, lubrificato dall’olio del motore.',
        expert:
          'Chiocciola turbina in ghisa o acciaio fuso (fino a ~800 °C), chiocciola compressore in alluminio, corpo centrale con cuscinetti flottanti lubrificati e spesso raffreddati ad acqua. Il polmone pneumatico o l’attuatore elettrico comanda le palette della turbina a geometria variabile (VGT) per avere spinta già a 1 500 giri/min.',
        explode: [-0.9, 0.5, 0],
      })
    );

    // ---------- Turbo rotor (spins) ----------
    const turboRotor = new THREE.Group();
    const rotorSpin = new THREE.Group();
    rotorSpin.position.set(0, TY, 0);
    turboRotor.add(rotorSpin);
    const compWheel = turboWheel(0.17, 0.2, 12, MAT.aluminum(), MAT.aluminum());
    compWheel.position.x = CX;
    const turbWheel = turboWheel(0.16, 0.16, 11, MAT.titanium(), MAT.titanium());
    turbWheel.position.x = TX;
    turbWheel.rotation.y = Math.PI; // nose toward -X (turbine outlet)
    rotorSpin.add(mesh(alongX(new THREE.CylinderGeometry(0.025, 0.025, 0.56, 16)), steel, (CX + TX) / 2), compWheel, turbWheel);
    root.add(
      ctx.part(turboRotor, {
        id: 'turboRotor',
        name: 'Girante turbina e compressore',
        base: 'Due ventole sullo stesso alberino. I gas di scarico fanno girare la turbina (lato scuro) e lei trascina il compressore (lato chiaro), che soffia aria nel motore. Girano anche a 200 000 giri al minuto!',
        expert:
          'Turbina radiale centripeta in superlega di nichel (Inconel), saldata all’albero; compressore centrifugo in alluminio forgiato con pale a sbalzo e splitter. Velocità oltre 200 000 giri/min nei turbo da auto; rapporto di compressione 2–3:1. L’equilibrio tra potenza della turbina e del compressore fissa la pressione di sovralimentazione.',
        explode: [-0.9, 0.5, 1.1],
      })
    );

    // ---------- Air filter and inlet duct ----------
    const airBox = new THREE.Group();
    const hoseMat = MAT.paint(0x23282f, 0.75);
    airBox.add(tube(curve([[CX + 0.26, TY, 0], [-2.22, 1.42, 0], [-2.24, 1.75, -0.15], [-2.33, 1.95, -0.55], [-2.38, 2.0, -0.72]]), 0.09, hoseMat, 40, 14));
    airBox.add(mesh(alongZ(new THREE.CylinderGeometry(0.22, 0.22, 0.5, 32)), MAT.paint(0x2a3442, 0.55), -2.38, 2.0, -0.97));
    airBox.add(mesh(alongZ(new THREE.CylinderGeometry(0.23, 0.23, 0.04, 32)), MAT.paint(0x15191e, 0.6), -2.38, 2.0, -1.23));
    airBox.add(mesh(new THREE.TorusGeometry(0.095, 0.01, 8, 28).rotateY(Math.PI / 2), steel, CX + 0.29, TY, 0));
    root.add(
      ctx.part(airBox, {
        id: 'airFilter',
        name: 'Filtro dell’aria',
        base: 'Da qui entra l’aria fresca: il filtro trattiene polvere e sabbia prima che arrivino al compressore e ai cilindri.',
        expert:
          'Filtro a cartuccia in carta pieghettata; il debimetro (MAF) a valle misura la massa d’aria aspirata, dato con cui la centralina calcola EGR e quantità massima di gasolio senza fumo nero.',
        explode: [-0.5, 0.6, -0.6],
      })
    );

    // ---------- Intercooler and charge pipes ----------
    const ic = new THREE.Group();
    const icMat = MAT.aluminum();
    const icFinMat = MAT.paint(0x9aa3ad, 0.5);
    icFinMat.metalness = 0.7;
    ic.add(mesh(new THREE.BoxGeometry(0.12, 0.66, 0.6), icMat, -2.45, 0.32, -1.0));
    for (let i = 0; i < 14; i++) ic.add(mesh(new THREE.BoxGeometry(0.17, 0.012, 0.62), icFinMat, -2.45, 0.03 + i * 0.045, -1.0));
    for (const y of [0.72, -0.08]) ic.add(mesh(alongZ(new THREE.CylinderGeometry(0.075, 0.075, 0.64, 20)), icMat, -2.45, y, -1.0));
    const chargeInPts = [[CX, TY - 0.25, 0.06], [CX, TY - 0.3, -0.3], [-2.5, 0.86, -0.6], [-2.45, 0.72, -0.72]];
    const chargeOutPts = [[-2.45, -0.08, -1.32], [-2.3, -0.12, -1.4], [-2.02, 0.3, -1.3], [-1.92, 0.85, -1.18], [-1.8, 1.05, -1.15]];
    ic.add(tube(curve(chargeInPts), 0.07, icMat, 30, 12), tube(curve(chargeOutPts), 0.075, icMat, 36, 12));
    root.add(
      ctx.part(ic, {
        id: 'intercooler',
        name: 'Intercooler',
        base: 'Un radiatore per l’aria: comprimendola il turbo la scalda, e l’intercooler la raffredda prima che entri nel motore. L’aria fresca è più densa, quindi nel cilindro ne entra di più.',
        expert:
          'Scambiatore aria-aria (nella vettura sta davanti, dietro il paraurti) o aria-acqua. Porta la carica da ~150 °C a ~40–60 °C: più ossigeno per ciclo, temperature di combustione più basse, meno NOx. (Nel modello è avvicinato al motore per chiarezza.)',
        explode: [-0.5, -0.3, -0.7],
      })
    );

    // ---------- After-treatment: downpipe, DOC + DPF ----------
    const dpf = new THREE.Group();
    dpf.add(tube(curve([[TX - 0.19, TY, 0], [TX - 0.28, 1.1, 0], [TX - 0.16, 0.8, 0], [DPF_X - 0.02, 0.7, 0], [DPF_X, 0.64, 0]]), 0.08, exMat, 36, 12));
    dpf.add(mesh(lathe([[0, 0.68], [0.075, 0.68], [0.075, 0.6], [0.21, 0.45], [0.21, -0.42], [0.075, -0.57], [0.075, -0.64], [0, -0.64]], 48), cut(MAT.steel(), { cap: false }), DPF_X));
    dpf.add(mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.22, 40), MAT.paint(0x9a8f6a, 0.8), DPF_X, 0.3, 0));
    dpf.add(mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.55, 40), MAT.paint(0x4b4a48, 0.9), DPF_X, -0.13, 0));
    for (const y of [0.42, -0.05]) dpf.add(mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.08, 10), cut(MAT.chrome()), DPF_X - 0.24, y, 0.08).rotateZ(Math.PI / 2));
    dpf.add(tube(curve([[DPF_X, -0.62, 0], [DPF_X, -0.72, -0.15], [DPF_X, -0.72, -0.8]]), 0.07, exMat, 20, 12));
    root.add(
      ctx.part(dpf, {
        id: 'dpf',
        name: 'Catalizzatore e filtro antiparticolato (DPF)',
        base: 'Il “filtro” dello scarico: la parte chiara ossida i gas nocivi, quella scura è una spugna di ceramica che trattiene la fuliggine e ogni tanto la brucia per pulirsi. I due sensori misurano la pressione prima e dopo il filtro.',
        expert:
          'DOC (Pt/Pd) che ossida CO e HC e produce NO₂; DPF a nido d’ape in carburo di silicio o cordierite con canali chiusi alternativamente, efficienza > 95% in massa e numero di particelle. La centralina stima l’intasamento dalla pressione differenziale; rigenerazione passiva con NO₂ o attiva a ~600 °C con post-iniezioni. A valle i motori Euro 6 hanno l’SCR, che con l’AdBlue (urea 32,5%) riduce gli NOx a N₂ e H₂O.',
        explode: [-0.8, -0.5, 0.5],
      })
    );

    // ---------- Accessory drive: damper, alternator, tensioner, poly-V belt ----------
    const acc = new THREE.Group();
    const accCircles = [[0, 0, 0.17], [0.78, 0.8, 0.058], [0.62, 0.3, 0.045]];
    const damperProfile = [[0.07, -0.045]];
    for (let i = 0; i <= 8; i++) damperProfile.push([i % 2 ? 0.165 : 0.172, -0.04 + i * 0.01]);
    damperProfile.push([0.172, 0.045], [0.07, 0.045]);
    const damper = new THREE.Group();
    damper.position.set(XACC, 0, 0);
    damper.add(new THREE.Mesh(alongX(lathe(damperProfile, 48)), darkSteel), mesh(alongX(new THREE.CylinderGeometry(0.07, 0.07, 0.1, 24)), steel));
    for (let i = 0; i < 4; i++) damper.add(mesh(alongX(new THREE.CylinderGeometry(0.022, 0.022, 0.095, 10)), MAT.paint(0x22262b), 0, Math.cos((i / 4) * TAU) * 0.11, Math.sin((i / 4) * TAU) * 0.11));
    acc.add(damper);
    const altPulley = new THREE.Group();
    altPulley.position.set(XACC, 0.78, 0.8);
    altPulley.add(new THREE.Mesh(alongX(lathe([[0.02, -0.045], [0.058, -0.045], [0.052, 0], [0.058, 0.045], [0.02, 0.045]], 32)), steel));
    acc.add(altPulley);
    acc.add(mesh(alongX(lathe([[0, -0.24], [0.15, -0.24], [0.17, -0.2], [0.17, 0.18], [0.14, 0.24], [0, 0.24]], 40)), MAT.aluminum(), -1.78, 0.78, 0.8));
    for (let i = 0; i < 10; i++) acc.add(rbox(0.36, 0.012, 0.03, 0.004, darkSteel, -1.78, 0.78 + Math.cos((i / 10) * TAU) * 0.172, 0.8 + Math.sin((i / 10) * TAU) * 0.172));
    acc.add(rbox(0.16, 0.12, 0.2, 0.02, blockMat, -1.62, 0.62, 0.63));
    acc.add(mesh(alongX(new THREE.CylinderGeometry(0.045, 0.045, 0.1, 24)), steel, XACC, 0.62, 0.3));
    acc.add(rod([XACC + 0.06, 0.62, 0.3], [XACC + 0.06, 0.42, 0.5], 0.02, darkSteel, 8));
    acc.add(new THREE.Mesh(ribbonGeo(loopPath(accCircles), XACC, 0.08, 0.016), kitMats.belt));
    cutAll(acc);
    root.add(
      ctx.part(acc, {
        id: 'accessoryDrive',
        name: 'Alternatore e cinghia servizi',
        base: 'Una seconda cinghia, all’esterno, fa girare l’alternatore che ricarica la batteria (e spesso il compressore del clima). La puleggia grande sull’albero smorza anche le vibrazioni, forti nel diesel.',
        expert: 'Cinghia Poly-V con tendicinghia automatico; puleggia smorzatrice torsionale sull’albero motore, importante nel diesel per le forti irregolarità di coppia. Alternatore trifase da 14 V e 120–200 A (molti diesel hanno anche il riscaldatore elettrico ausiliario).',
        explode: [-1.2, -0.2, 0.7],
      })
    );

    // ---------- Timing drive variants ----------
    const timingParts = {};
    const timingInfo = {
      dryBelt: ['Cinghia di distribuzione (a secco)', 'Carter in plastica della cinghia'],
      wetBelt: ['Cinghia di distribuzione a bagno d’olio', 'Carter distribuzione in alluminio'],
      chain: ['Catena di distribuzione, pattini e tenditore', 'Carter distribuzione in alluminio'],
    };
    for (const kind of ['dryBelt', 'wetBelt', 'chain']) {
      const t = buildTiming(kind, timingCircles(kind), kitMats);
      const txt = TIMING_TEXT[kind];
      const a = ctx.part(t.drive, {
        id: `timing_${kind}`,
        name: timingInfo[kind][0],
        base: 'Collega l’albero motore agli alberi a camme (rapporto 2:1) e alla pompa ad alta pressione del gasolio. ' + txt.base,
        expert: txt.expert,
        explode: [-0.35, 0, 0.9],
      });
      const b = ctx.part(t.cover, {
        id: `timingCover_${kind}`,
        name: timingInfo[kind][1],
        base: kind === 'dryBelt' ? 'Un coperchio leggero in plastica: protegge la cinghia da sporco e acqua, ma non deve trattenere olio.' : 'Un carter metallico a tenuta: chiude la distribuzione dentro il motore, bagnata dall’olio.',
        expert: kind === 'dryBelt' ? 'Carter in materiale plastico, in due pezzi, fissato con viti; la tenuta dell’olio è affidata ai paraolio degli alberi.' : 'Carter in lega di alluminio con guarnizione, parte del circuito dell’olio: per ispezionare la distribuzione va smontato.',
        explode: [-0.45, 0, 1.7],
      });
      root.add(a, b);
      if (t.splash) root.add(t.splash.object);
      timingParts[kind] = { t, parts: [a, b] };
    }
    let timingKind = 'chain';
    const setTiming = (kind) => {
      timingKind = timingParts[kind] ? kind : 'chain';
      for (const [k, v] of Object.entries(timingParts)) {
        v.parts.forEach((p) => (p.visible = k === timingKind));
        if (v.t.splash) v.t.splash.object.visible = k === timingKind;
      }
    };
    setTiming('chain');

    // ---------- Flywheel ----------
    const flywheel = new THREE.Group();
    flywheel.add(mesh(alongX(new THREE.CylinderGeometry(0.55, 0.55, 0.06, 64)), darkSteel));
    flywheel.add(mesh(alongX(new THREE.CylinderGeometry(0.5, 0.5, 0.05, 64)), steel, 0.07));
    flywheel.add(mesh(new THREE.TorusGeometry(0.56, 0.03, 8, 90).rotateY(Math.PI / 2), steel));
    flywheel.add(mesh(new THREE.BoxGeometry(0.09, 0.3, 0.06), MAT.paint(0xff7a1a), 0.09, 0.3, 0));
    flywheel.add(bolts(steel, Array.from({ length: 6 }, (_, i) => [-0.04, Math.cos((i / 6) * TAU) * 0.1, Math.sin((i / 6) * TAU) * 0.1]), 'x'));
    flywheel.position.x = 1.9;
    root.add(
      ctx.part(flywheel, {
        id: 'flywheel',
        name: 'Volano a doppia massa',
        base: 'Un disco pesante che, come una trottola, porta avanti il motore tra una combustione e l’altra e assorbe gli scossoni prima che arrivino al cambio.',
        expert:
          'Volano bimassa: due masse collegate da molle ad arco filtrano le forti irregolarità di coppia del diesel (soprattutto ai bassi regimi) e proteggono cambio e trasmissione. La corona dentata serve al motorino di avviamento.',
        explode: [1.0, 0, 0],
      })
    );

    // ---------- Flows ----------
    const mk = (c, opts) => {
      const f = flow({ path: (t, out) => c.getPointAt(t, out), ...opts });
      root.add(f.object);
      ctx.addFlow(f);
      return f;
    };
    const intakeFlows = intakePaths.map((c) => mk(c, { count: 60, size: 0.07, colors: [0x5fd3ff, 0x8fe3ff], spread: 0.06, speed: 1 }));
    const exhaustFlows = exhaustPaths.map((c) => mk(c, { count: 60, size: 0.08, colors: [0xff5a1f, 0xb07a6a, 0x8c86a8], spread: 0.05, speed: 0.8 }));
    // Fresh air: filter -> compressor eye -> volute (exposed half) -> intercooler inlet.
    const vol = (a, r = 0.25) => [CX, TY + Math.cos(a) * r, Math.sin(a) * r];
    const airCurve = curve([
      [-2.38, 2.0, -1.05], [-2.38, 2.0, -0.72], [-2.33, 1.95, -0.55], [-2.24, 1.75, -0.15], [-2.22, 1.42, 0], [CX + 0.24, TY, 0],
      [CX + 0.05, TY, 0.02], vol(0.5, 0.2), vol(Math.PI / 2), vol(0.75 * Math.PI), vol(Math.PI, 0.25),
      [CX, TY - 0.3, -0.3], [-2.5, 0.86, -0.6], [-2.45, 0.7, -0.75], [-2.45, 0.6, -0.95],
    ]);
    const airFlow = mk(airCurve, { count: 220, size: 0.07, colors: [0x5fd3ff, 0x5fd3ff, 0x9fe8ff], spread: 0.05, speed: 0.35 });
    const boostCurve = curve([
      [-2.45, 0.6, -0.95], [-2.45, 0.1, -1.05], [-2.45, -0.08, -1.32], [-2.3, -0.12, -1.4], [-2.02, 0.3, -1.3],
      [-1.92, 0.85, -1.18], [-1.8, 1.05, -1.15], [-1.4, 1.05, -1.15], [1.4, 1.05, -1.15],
    ]);
    const boostFlow = mk(boostCurve, { count: 220, size: 0.07, colors: [0x5fd3ff], spread: 0.08, speed: 0.3 });
    // Exhaust: collector -> turbine (exposed half) -> downpipe -> DOC/DPF -> tailpipe.
    const exCurve = curve([
      [-1.6, 1.72, 0.66], [-2.3, 1.7, 0.6], [-2.88, 1.62, 0.38], [TX, 1.5, 0.14], [TX, TY + 0.2, 0.12], [TX - 0.05, TY, 0.04],
      [TX - 0.19, TY, 0], [TX - 0.28, 1.1, 0], [TX - 0.16, 0.8, 0], [DPF_X - 0.02, 0.7, 0], [DPF_X, 0.5, 0], [DPF_X, -0.6, 0],
      [DPF_X, -0.72, -0.15], [DPF_X, -0.72, -0.85],
    ]);
    const exFlow = mk(exCurve, { count: 260, size: 0.08, colors: [0xb07a6a, 0x8c86a8, 0x8c86a8, 0xb4b0cc], spread: 0.05, speed: 0.3 });
    const fuelFlow = mk(hpCurve, { count: 50, size: 0.05, colors: [0xffc04d], spread: 0.01, speed: 0.6 });

    // ---------- Animation ----------
    const cA = new THREE.Color();
    const C = {
      air: new THREE.Color(0x5fd3ff),
      hot: new THREE.Color(0xff9a3c),
      flash: new THREE.Color(0xfff0c0),
      fire: new THREE.Color(0xff5a1f),
      exh: new THREE.Color(0x8c86a8),
    };
    const rpmOf = (th) => 800 + th * 3700;
    const animRate = (th) => 0.12 + th * 0.8; // slow, readable motion (cycles/s)
    const torqueAt = (rpm) => 380 * (0.4 + 0.6 * smooth(clamp((rpm - 800) / 1000, 0, 1))) * (1 - 0.35 * clamp((rpm - 2800) / 1700, 0, 1));
    let turboAngle = 0;

    // Fuel spray length 0..1: pilot shot then main injection around TDC (local 0.5).
    function sprayAt(local) {
      if (local > 0.452 && local < 0.466) return 0.45 * Math.sin((Math.PI * (local - 0.452)) / 0.014);
      if (local > 0.478 && local < 0.548) return clamp((local - 0.478) / 0.012, 0, 1) * clamp((0.548 - local) / 0.012, 0, 1);
      return 0;
    }

    return {
      root,
      cycleRate: animRate,
      setOption(id, value) {
        if (id === 'timing') setTiming(value);
      },
      update(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const dt = state.dt || 0;
        const theta1 = state.cycle * Math.PI * 4;
        crank.rotation.x = theta1;
        flywheel.rotation.x = theta1;
        damper.rotation.x = theta1;
        altPulley.rotation.x = theta1 * (0.17 / 0.058);
        const camAngle = state.cycle * TAU;
        camIn.rotation.x = camAngle;
        camEx.rotation.x = camAngle;

        const rate = animRate(thr);
        const assembled = (state.explode || 0) < 0.05;
        const tp = timingParts[timingKind].t;
        tp.update(theta1);
        if (tp.splash) {
          tp.splash.object.visible = assembled;
          tp.splash.update(dt, rate * 1.5);
        }

        // Turbo spin: visual speed capped to stay readable (no wagon-wheel effect).
        turboAngle = (turboAngle + dt * (5 + 19 * thr)) % TAU;
        rotorSpin.rotation.x = turboAngle;

        const glow = clamp(1 - thr / 0.12, 0, 1); // glow plugs only near idle (cold-start idea)
        gpTips.forEach((m) => (m.material.emissiveIntensity = glow * (2.2 + 0.4 * Math.sin((state.time || 0) * 6))));

        BORE_X.forEach((x, k) => {
          const local = wrap(state.cycle - OFFSET[k]);
          const th = local * Math.PI * 4;
          const yc = R * Math.cos(th);
          const zc = R * Math.sin(th);
          const yp = yc + Math.sqrt(L * L - zc * zc);
          pistons[k].position.y = yp;
          rods[k].position.set(x, yc, zc);
          rods[k].rotation.x = Math.atan2(-zc, yp - yc);

          const li = lift(local, -0.03, 0.28);
          const le = lift(local, 0.72, 1.03);
          inV.list[k].position.y = HEAD_Y - 0.005 - li * 0.08;
          exV.list[k].position.y = HEAD_Y - 0.005 - le * 0.08;

          // Charge between piston crown and fire deck: air -> hot air -> flame -> exhaust.
          const top = yp + CROWN;
          const ch = charges[k];
          ch.position.y = (top + HEAD_Y) / 2;
          ch.scale.y = Math.max(0.001, HEAD_Y - top);
          let op;
          if (local < 0.25) {
            cA.copy(C.air);
            op = 0.12 + 0.08 * (local / 0.25);
          } else if (local < 0.487) {
            const f = (local - 0.25) / 0.237;
            cA.copy(C.air).lerp(C.hot, f * f);
            op = 0.2 + 0.32 * f;
          } else if (local < 0.75) {
            const f = (local - 0.487) / 0.263;
            if (f < 0.1) cA.copy(C.flash).lerp(C.fire, f / 0.1);
            else cA.copy(C.fire).lerp(C.exh, (f - 0.1) / 0.9);
            op = lerp(0.95, 0.35, f);
          } else {
            cA.copy(C.exh);
            op = lerp(0.35, 0.08, (local - 0.75) / 0.25);
          }
          ch.material.color.copy(cA);
          ch.material.opacity = op;
          ch.visible = assembled;

          const s = sprayAt(local);
          const sp = sprays[k];
          sp.g.visible = assembled && s > 0.01;
          if (s > 0.01) {
            const w = 0.5 + 0.5 * s;
            sp.jets.forEach((j) => j.scale.set(w, Math.max(0.05, s), w));
            sp.mat.opacity = 0.65 + 0.3 * Math.random();
          }
          const flash = local > 0.487 && local < 0.62 ? 1 - (local - 0.487) / 0.133 : 0;
          fireLights[k].intensity = assembled ? flash * (2 + thr * 4) : 0;

          const fi = intakeFlows[k];
          const fe = exhaustFlows[k];
          fi.object.visible = fe.object.visible = assembled;
          fi.material.opacity = clamp(li * 1.6, 0, 0.9);
          fi.update(dt, li * rate * 1.6);
          fe.material.opacity = clamp(le * 1.6, 0, 0.9);
          fe.update(dt, le * rate * 1.6);
        });

        const boost = 0.35 + 1.4 * thr;
        for (const f of [airFlow, boostFlow, exFlow]) {
          f.object.visible = assembled;
          f.update(dt, boost);
        }
        airFlow.material.opacity = boostFlow.material.opacity = 0.45 + 0.45 * thr;
        exFlow.material.opacity = 0.5 + 0.4 * thr;
        fuelFlow.object.visible = assembled;
        fuelFlow.update(dt, 0.4 + thr);
      },
      readout(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const rpm = rpmOf(thr);
        const rail = 250 + 1750 * thr;
        const boost = 0.05 + 1.35 * Math.pow(thr, 1.3);
        return [
          { label: 'Giri reali', value: Math.round(rpm / 10) * 10, unit: 'giri/min' },
          { label: 'Coppia', value: Math.round(torqueAt(rpm)), unit: 'Nm' },
          { label: 'Pressione rail', value: (Math.round(rail / 10) * 10).toLocaleString('it-IT'), unit: 'bar' },
          { label: 'Pressione turbo', value: boost.toFixed(1).replace('.', ','), unit: 'bar' },
        ];
      },
    };
  },
};
