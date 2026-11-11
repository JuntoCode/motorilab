// Inline-4 petrol engine, DOHC, port injection. Cycle 0..1 = 720° of crank for cylinder 1.
// Detailed castings (open-deck block with water jacket, bulkheads, hollow head/cover/sump) so the
// section view shows real internals; selectable timing drive (dry belt, wet belt, chain).
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, pipe, flow, clamp, lerp } from '../core/helpers.js';

const BORE_X = [-1.2, -0.4, 0.4, 1.2];
// Firing order 1-3-4-2: cycle offset of each cylinder relative to cylinder 1.
const OFFSET = [0, 0.75, 0.25, 0.5];
const R = 0.3; // crank radius (stroke 0.6)
const L = 1.0; // conrod length
const HEAD_Y = 1.55; // head deck (combustion chamber roofs are recessed above it)
const VALVE_Z = 0.15;
const CAM_Y = 2.18;

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
const TIMING = ['dryBelt', 'wetBelt', 'chain']; // timing drive variants (option 'timing')
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

// ---------- Engine-specific timing layout (crank, cams, water pump, tensioner) ----------
function timingCircles(kind) {
  const c = [
    { y: 0, z: 0, r: R_CRANK_SPROCKET, kind: 'crank' },
    { y: CAM_Y, z: -VALVE_Z, r: 0.145, kind: 'cam' },
    { y: CAM_Y, z: VALVE_Z, r: 0.145, kind: 'cam' },
  ];
  if (kind !== 'chain') c.push({ y: 0.75, z: -0.36, r: 0.08, kind: 'pump' }, { y: 1.1, z: 0.3, r: 0.055, kind: 'idler' });
  return c;
}

export default {
  id: 'petrol4t',
  name: 'Motore a benzina 4 tempi',
  short: 'Benzina 4T',
  family: 'terra',
  vehicle: 'Automobile',
  examples: 'Fiat Panda, Volkswagen Golf, Toyota Yaris, quasi tutte le utilitarie a benzina',
  tagline: 'Quattro tempi, migliaia di piccole esplosioni al minuto: il cuore dell’auto da oltre un secolo.',
  intro: {
    base:
      'Dentro ogni cilindro un pistone va su e giù. Aspira aria e benzina, la schiaccia, una scintilla la fa <b>esplodere</b> e la spinta del gas caldo spinge giù il pistone. La biella e l’albero motore trasformano questo su-e-giù in una <b>rotazione</b> che arriva alle ruote. Con quattro cilindri che “scoppiano” a turno il motore gira morbido.',
    expert:
      'Motore alternativo ad accensione comandata che segue il <b>ciclo Otto</b>: compressione adiabatica, combustione quasi a volume costante, espansione, scarico. Il rendimento ideale vale η = 1 − 1/r<sup>γ−1</sup> (r = rapporto di compressione): con r = 11 si arriva al 61% teorico, ma attriti, perdite di calore e di pompaggio portano il valore reale al 30–40%. Quattro cilindri in linea, ordine di accensione 1‑3‑4‑2, una fase utile ogni 180° di albero.',
  },
  specs: [
    { label: 'Carburante', value: 'Benzina (E10)' },
    { label: 'Ciclo', value: 'Otto, 4 tempi' },
    { label: 'Cilindrata tipica', value: '1,0 – 2,0 litri' },
    { label: 'Potenza tipica', value: '70 – 150 kW' },
    { label: 'Regime massimo', value: '≈ 6 500 giri/min' },
    { label: 'Rapporto di compressione', value: '10:1 – 13:1' },
    { label: 'Distribuzione', value: 'Cinghia (a secco o a bagno d’olio) o catena' },
    { label: 'Rendimento', value: '25 – 40 %' },
  ],
  metrics: { efficiency: 33, power: 110, powerDensity: 0.8 },
  variants: {
    columns: ['Motore', 'Cilindrata', 'Potenza', 'Distribuzione'],
    rows: [
      ['Fiat 1.2 FIRE 8v', '1,2 l, 4 cilindri', '51 kW', 'Cinghia a secco'],
      ['Volkswagen 1.5 TSI (EA211 evo)', '1,5 l, 4 cilindri, turbo', '96 – 110 kW', 'Cinghia a secco'],
      ['Ford 1.0 EcoBoost', '1,0 l, 3 cilindri, turbo', '74 – 92 kW', 'Cinghia a bagno d’olio'],
      ['Stellantis 1.2 PureTech (prime versioni)', '1,2 l, 3 cilindri', '55 – 96 kW', 'Cinghia a bagno d’olio'],
      ['Toyota 1.5 Dynamic Force (M15A)', '1,5 l, 3 cilindri', '68 – 92 kW', 'Catena'],
    ],
    note: 'Il modello 3D mostra un 4 cilindri aspirato a iniezione nel condotto; molti motori recenti sono turbo a iniezione diretta e alcuni hanno 3 cilindri.',
  },
  options: [
    {
      id: 'timing',
      label: 'Distribuzione',
      default: 'dryBelt',
      choices: ['dryBelt', 'wetBelt', 'chain'].map((id) => ({ id, ...TIMING_TEXT[id] })),
    },
  ],
  thumb: { orbit: [-38, 20], zoom: 1.15, view: 'section' },
  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      range: [0, 0.25],
      parts: ['intakeValves', 'intakeManifold', 'pistons', 'injectors'],
      base: 'Il pistone scende e “risucchia” aria dentro il cilindro, come una siringa che si riempie. L’iniettore spruzza la benzina nell’aria: ecco la miscela.',
      expert:
        'Valvola di aspirazione aperta, pistone dal PMS al PMI (0–180°). La depressione richiama la carica; la farfalla regola la massa d’aria (carico). Iniezione indiretta nel condotto con rapporto stechiometrico aria/benzina ≈ 14,7:1. Le valvole si aprono in anticipo e chiudono in ritardo per sfruttare l’inerzia del gas.',
    },
    {
      id: 'compression',
      name: 'Compressione',
      range: [0.25, 0.5],
      parts: ['pistons', 'cylinderHead'],
      base: 'Le valvole si chiudono e il pistone risale, schiacciando la miscela in uno spazio piccolissimo. Più la schiacci, più energia otterrai dall’esplosione.',
      expert:
        'Valvole chiuse, pistone dal PMI al PMS (180–360°). Compressione quasi adiabatica: con r ≈ 11 la pressione sale a 15–20 bar e la temperatura a ~400 °C. Il limite è la detonazione (battito in testa): per questo servono benzine ad alto numero di ottano.',
    },
    {
      id: 'power',
      name: 'Scoppio ed espansione',
      range: [0.5, 0.75],
      parts: ['sparkPlugs', 'ignitionCoils', 'pistons', 'crankshaft', 'conrods'],
      base: 'La candela fa una scintilla: la miscela brucia in un lampo, il gas caldissimo si espande e spinge giù il pistone con forza. È l’unico tempo che produce lavoro!',
      expert:
        'Accensione con anticipo di 10–35° prima del PMS, picco di pressione 50–80 bar poco dopo il PMS, temperatura di fiamma ~2 200 °C. L’espansione (360–540°) cede lavoro all’albero tramite la biella: la coppia istantanea è massima a ~90° dopo il PMS. Il volano accumula energia per gli altri tre tempi.',
    },
    {
      id: 'exhaust',
      name: 'Scarico',
      range: [0.75, 1],
      parts: ['exhaustValves', 'exhaustManifold', 'pistons'],
      base: 'Si apre la valvola di scarico e il pistone risale spingendo fuori i gas bruciati verso la marmitta. Il cilindro è pronto per ricominciare.',
      expert:
        'Valvola di scarico aperta già prima del PMI per ridurre la contropressione (540–720°). I gas a 700–900 °C passano nel collettore e nel catalizzatore trivalente, che converte CO, HC e NOx. A fine corsa c’è l’incrocio valvole: scarico e aspirazione aperte insieme per un breve tratto.',
    },
  ],
  quiz: [
    {
      q: 'Quale dei quattro tempi produce lavoro utile?',
      options: ['Aspirazione', 'Compressione', 'Scoppio ed espansione', 'Scarico'],
      correct: 2,
      explain: 'Solo durante l’espansione il gas spinge il pistone. Negli altri tre tempi il motore usa l’energia accumulata nel volano.',
    },
    {
      q: 'Quanti giri compie l’albero motore per un ciclo completo di un cilindro?',
      options: ['Mezzo giro', 'Un giro', 'Due giri', 'Quattro giri'],
      correct: 2,
      explain: 'Ogni tempo è mezza rotazione: 4 × 180° = 720°, cioè due giri. Per questo l’albero a camme gira a metà velocità.',
    },
    {
      q: 'A cosa serve la candela?',
      options: ['A iniettare benzina', 'A raffreddare il cilindro', 'A innescare la combustione con una scintilla', 'A chiudere le valvole'],
      correct: 2,
      explain: 'La candela genera una scintilla di decine di migliaia di volt che accende la miscela compressa.',
    },
    {
      q: 'Perché un rapporto di compressione più alto aumenta il rendimento?',
      options: [
        'Perché entra più benzina',
        'Perché il gas si espande di più e cede più lavoro',
        'Perché il motore gira più piano',
        'Perché il pistone è più leggero',
      ],
      correct: 1,
      explain: 'Nel ciclo Otto η = 1 − 1/r^(γ−1): più alta la compressione, più lunga l’espansione utile. Il limite pratico è la detonazione.',
    },
  ],
  // Officina (see CONTRACT.md): how each part comes off and what holds it; the guide suggests an order.
  // Simplified and educational, not a workshop manual.
  disassembly: {
    intro: 'Smontiamo la parte alta del motore fino a vedere i cilindri.',
    note: 'Sequenza semplificata a scopo didattico: in officina si segue il manuale del costruttore, con i suoi attrezzi e le sue coppie di serraggio.',
    parts: {
      ignitionCoils: { dir: [0, 1, 0], dist: 0.7 },
      camCoverBolts: { motion: 'unscrew', turns: 6, out: 0.1 },
      camCover: { after: ['camCoverBolts', 'ignitionCoils'], with: ['coverGasket'], dir: [0, 1, 0], dist: 0.8 },
      sparkPlugs: { after: ['ignitionCoils'], motion: 'unscrew', turns: 5, out: 0.12, dir: [0, 1, 0], dist: 0.8 },
      accessoryDrive: { dir: [-1, 0, 0.4], dist: 0.9 },
      ...Object.fromEntries(
        TIMING.flatMap((k) => [
          [`timingCover_${k}`, { after: ['accessoryDrive'], dir: [-1, 0, 0], dist: 0.9 }],
          [`timing_${k}`, { after: [`timingCover_${k}`], dir: [-1, 0, 0], dist: 1.0 }],
        ])
      ),
      camCapBolts: { after: ['camCover'], motion: 'unscrew', turns: 5, out: 0.1 },
      camCaps: { after: ['camCapBolts'], dir: [0, 1, 0], dist: 0.6 },
      camshafts: { after: ['camCaps', ...TIMING.map((k) => `timing_${k}`)], dir: [0, 1, 0], dist: 0.7 },
      intakeValves: { after: ['camshafts'], dir: [0, 1, 0], dist: 0.8 },
      exhaustValves: { after: ['camshafts'], dir: [0, 1, 0], dist: 0.8 },
      intakeManifold: { with: ['injectors'], dir: [0, 0.2, -1], dist: 0.9 },
      injectors: { dir: [0, 0.2, -1], dist: 0.9 },
      exhaustManifold: { dir: [0, 0, 1], dist: 0.9 },
      headBolts: { after: ['camshafts'], motion: 'unscrew', turns: 8, out: 0.3, dist: 0.5, duration: 2.2 },
      cylinderHead: {
        after: ['headBolts', 'intakeManifold', 'exhaustManifold', 'sparkPlugs'],
        with: ['intakeValves', 'exhaustValves'],
        dir: [0, 1, 0],
        dist: 1.0,
        duration: 1.2,
      },
      gaskets: { after: ['cylinderHead'], dir: [0, 1, 0], dist: 0.6 },
      block: { after: ['gaskets'] },
    },
    steps: [
      {
        id: 'coils',
        name: 'Stacca le bobine',
        base: 'Si scollega il cablaggio e si sfilano le bobine dall’alto: ognuna è infilata sopra una candela.',
        expert: 'Prima di tutto si stacca il polo negativo della batteria. Le bobine a matita escono tirandole dritte, dopo aver sganciato il connettore.',
        parts: ['ignitionCoils'],
      },
      {
        id: 'cover',
        name: 'Togli il coperchio',
        base: 'Si svitano le viti e si solleva il coperchio punterie: sotto ci sono gli alberi a camme.',
        expert: 'Le viti si allentano un po’ alla volta; la guarnizione di gomma resta sul bordo del coperchio e va controllata o sostituita.',
        parts: ['camCoverBolts', 'camCover'],
      },
      {
        id: 'plugs',
        name: 'Svita le candele',
        base: 'Con una chiave a tubo lunga si svitano le candele dal fondo dei loro pozzetti.',
        expert: 'Si svitano a motore freddo, per non rovinare il filetto nella testata di alluminio; il colore degli elettrodi racconta come lavora il motore.',
        parts: ['sparkPlugs'],
      },
      {
        id: 'aux',
        name: 'Togli la cinghia servizi',
        base: 'Si allenta il tendicinghia e si toglie la cinghia esterna con alternatore e puleggia: coprono la distribuzione.',
        expert: 'Il tendicinghia automatico si ruota con una chiave per liberare la cinghia Poly-V; poi si smonta la puleggia smorzatrice sull’albero motore.',
        parts: ['accessoryDrive'],
      },
      {
        id: 'timingCover',
        name: 'Togli il carter',
        base: 'Si svita il coperchio laterale che protegge la distribuzione.',
        expert: 'Con la cinghia a secco è un carter leggero in plastica; con cinghia a bagno d’olio o catena è un carter in alluminio a tenuta, con la sua guarnizione.',
        parts: TIMING.map((k) => `timingCover_${k}`),
      },
      {
        id: 'timing',
        name: 'Togli la distribuzione',
        base: 'Si bloccano gli alberi nella posizione giusta, si allenta il tenditore e si sfila la cinghia (o la catena).',
        expert: 'Con gli attrezzi di fasatura si bloccano albero motore e alberi a camme: al rimontaggio la fase deve tornare identica, altrimenti valvole e pistoni possono toccarsi.',
        parts: TIMING.map((k) => `timing_${k}`),
      },
      {
        id: 'cams',
        name: 'Togli gli alberi a camme',
        base: 'Si svitano i cappelli che li tengono fermi e si sollevano gli alberi a camme.',
        expert: 'Le viti dei cappelli si allentano poco per volta e in sequenza, perché le molle delle valvole spingono gli alberi verso l’alto. I cappelli sono numerati: ognuno torna al suo posto.',
        parts: ['camCapBolts', 'camCaps', 'camshafts'],
      },
      {
        id: 'manifolds',
        name: 'Stacca i collettori',
        base: 'Si staccano dai lati della testata il collettore di aspirazione, con gli iniettori, e quello di scarico.',
        expert: 'Sul lato aspirazione si scollegano anche il tubo del carburante e i connettori degli iniettori; i dadi dello scarico, cotti dal calore, sono spesso duri da svitare.',
        parts: ['intakeManifold', 'injectors', 'exhaustManifold'],
      },
      {
        id: 'head',
        name: 'Solleva la testata',
        base: 'Si svitano le lunghe viti della testata e la si solleva insieme alle valvole.',
        expert: 'Le viti si allentano nell’ordine inverso a quello di serraggio, dall’esterno verso il centro, per non deformare la testata. Le valvole restano al loro posto: per toglierle serve un attrezzo che comprime le molle.',
        parts: ['headBolts', 'cylinderHead'],
      },
      {
        id: 'gasket',
        name: 'Togli la guarnizione',
        base: 'Sotto la testata c’è la guarnizione: tolta quella, si vedono dall’alto i pistoni dentro i cilindri.',
        expert: 'La guarnizione multistrato non si riusa: al rimontaggio se ne mette una nuova, dopo aver controllato che i piani di testata e basamento siano perfettamente piatti.',
        parts: ['gaskets'],
      },
    ],
  },
  camera: { position: [-4.7, 2.7, 6.3], target: [0, 0.75, 0] },

  build(ctx) {
    const root = new THREE.Group();
    root.position.set(0.22, 0.2, 0);

    // ---------- Materials ----------
    const castAlu = () => {
      const m = MAT.aluminum();
      m.color.set(0x9aa3ad);
      m.roughness = 0.62;
      m.metalness = 0.55;
      return m;
    };
    const blockMat = cut(castAlu());
    const linerMat = cut(MAT.castIron());
    const headMat = cut(castAlu());
    headMat.color.set(0xa7b0ba);
    const gasketMat = cut(MAT.paint(0x2b2e33, 0.7));
    const boltMat = cut(MAT.steel());
    const steel = MAT.steel();
    const darkSteel = MAT.darkSteel();
    const rubber = MAT.rubber();
    const plastic = MAT.paint(0x1a1d21, 0.75);
    plastic.metalness = 0.05;
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
      plastic: cut(plastic),
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
    // coolant in the open-deck jacket (translucent, no hatch)
    const coolant = rrShape(-1.695, -0.455, 1.695, 0.455, 0.07);
    coolant.holes.push(rrHole(-1.605, -0.405, 1.605, 0.405, 0.32));
    const coolantMesh = new THREE.Mesh(extY(coolant, 0.62, 1.5, 12), cut(MAT.gas(0x3fc9a8, 0.28, false), { cap: false }));
    coolantMesh.renderOrder = 2;
    block.add(coolantMesh);
    // ribs, oil filter boss, knock sensor, dipstick
    for (const x of [-0.8, 0, 0.8]) for (const s of [-1, 1]) block.add(rbox(0.06, 0.88, 0.05, 0.015, blockMat, x, 1.05, s * (BZ + 0.02)));
    const fittings = new THREE.Group();
    fittings.add(mesh(alongZ(new THREE.CylinderGeometry(0.06, 0.07, 0.05, 6)), darkSteel, 0.02, 1.0, BZ + 0.035));
    fittings.add(rod([-0.85, 0.35, BZ + 0.04], [-0.85, 1.48, BZ + 0.07], 0.016, steel, 8));
    fittings.add(mesh(new THREE.TorusGeometry(0.035, 0.012, 8, 20), MAT.paint(0xf2c230, 0.4), -0.85, 1.53, BZ + 0.07));
    fittings.add(mesh(alongZ(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 6)), steel, -0.85, 0.35, BZ + 0.02));
    block.add(cutAll(fittings));
    root.add(
      ctx.part(block, {
        id: 'block',
        name: 'Basamento e cilindri',
        base: 'Il “corpo” del motore: un blocco di alluminio con quattro buchi tondi, i cilindri, dentro cui scorrono i pistoni. Intorno ai cilindri circola il liquido di raffreddamento (in verde nella sezione).',
        expert:
          'Monoblocco in lega di alluminio pressofuso, canne in ghisa riportate (o riporto al plasma). Qui in versione “open deck”: l’intercapedine dell’acqua è aperta verso la testata, le canne sono unite tra loro (siamesi). Le nervature esterne irrigidiscono le pareti e riducono le vibrazioni.',
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
    // engine mount brackets (front)
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
        base: 'La parte bassa del motore che sostiene l’albero motore. Le pareti trasversali (supporti di banco) hanno un foro con un cuscinetto in cui gira l’albero.',
        expert:
          'Semicarter con cinque paratie (bulkhead) e cappelli di banco; i cuscinetti sono bronzine a guscio sottile (in bronzo nella sezione) lubrificate a pressione. Spesso un telaio inferiore (ladder frame) lega i cappelli per aumentare la rigidezza.',
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
    const oil = mesh(new THREE.BoxGeometry(3.38, 0.15, 0.92), cut(MAT.gas(0x7a5410, 0.55, false), { cap: false }), 0, -0.805, 0);
    oil.renderOrder = 2;
    pan.add(oil);
    pan.add(tube(curve([[0.25, -0.84, 0], [0.25, -0.6, 0], [0.12, -0.5, 0], [0.02, -0.46, 0]]), 0.022, steel, 20, 8));
    pan.add(mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.03, 24), darkSteel, 0.25, -0.85, 0));
    root.add(
      ctx.part(pan, {
        id: 'oilPan',
        name: 'Coppa dell’olio',
        base: 'La vaschetta sotto il motore che raccoglie l’olio, il “sangue” che lubrifica tutte le parti in movimento. Dentro si vede il pescante, il tubo che lo aspira.',
        expert: 'Coppa in lamiera stampata, 3–5 litri di olio. La pompa lo aspira dal pescante con reticella e lo manda in pressione (2–5 bar) a cuscinetti, alberi a camme e getti sotto i pistoni; il tappo di scarico in basso serve al cambio olio.',
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
        base: 'Uno strato sottile tra testata e basamento che impedisce a gas, olio e acqua di uscire: deve reggere la pressione dello scoppio.',
        expert: 'Guarnizione di testata multistrato in acciaio (MLS) con bordini attorno alle canne che reggono i 50–80 bar della combustione. Va sostituita ogni volta che si smonta la testata.',
        explode: [0, 0.5, 0.2],
      })
    );
    const coverGasketShape = rrShape(-1.7, -0.42, 1.7, 0.42, 0.06);
    coverGasketShape.holes.push(rrHole(-1.64, -0.36, 1.64, 0.36, 0.04));
    root.add(
      ctx.part(new THREE.Mesh(extY(coverGasketShape, HEAD_TOP - 0.006, HEAD_TOP + 0.006, 8), gasketMat), {
        id: 'coverGasket',
        name: 'Guarnizione del coperchio',
        base: 'Un cordone di gomma tra testata e coperchio punterie: tiene l’olio dentro il motore.',
        expert: 'Guarnizione in elastomero sagomata, alloggiata in una gola del coperchio; di solito si cambia insieme al coperchio o quando trasuda olio.',
        explode: [0, 2.0, 0],
      })
    );

    // ---------- Cylinder head: core with chambers, wells, jackets; side skins; cam caps ----------
    const head = new THREE.Group();
    const corePts = [[-HX, HEAD_Y]];
    for (const c of BORE_X) corePts.push([c - 0.34, HEAD_Y], [c - 0.2, HEAD_Y + 0.075], [c + 0.2, HEAD_Y + 0.075], [c + 0.34, HEAD_Y]);
    corePts.push([HX, HEAD_Y], [HX, HEAD_TOP], [HX - 0.1, HEAD_TOP], [HX - 0.1, GAL_Y]);
    [...BORE_X].reverse().forEach((c, i) => {
      // spark-plug well down to the chamber roof
      corePts.push([c + 0.065, GAL_Y], [c + 0.065, 1.7], [c + 0.026, 1.7], [c + 0.026, HEAD_Y + 0.082], [c - 0.026, HEAD_Y + 0.082], [c - 0.026, 1.7], [c - 0.065, 1.7], [c - 0.065, GAL_Y]);
      if (i < 3) {
        const t = c - 0.4; // cam bearing tower between cylinders
        corePts.push([t + 0.05, GAL_Y], [t + 0.05, HEAD_TOP], [t - 0.05, HEAD_TOP], [t - 0.05, GAL_Y]);
      }
    });
    corePts.push([-HX + 0.1, GAL_Y], [-HX + 0.1, HEAD_TOP], [-HX, HEAD_TOP]);
    const core = polyShape(corePts);
    for (const c of BORE_X) for (const s of [-1, 1]) core.holes.push(rrHole(s > 0 ? c + 0.12 : c - 0.29, 1.665, s > 0 ? c + 0.29 : c - 0.12, 1.795, 0.03));
    head.add(new THREE.Mesh(extZ(core, -0.4, 0.4, 8), headMat));
    for (const s of [-1, 1]) head.add(rbox(2 * HX, HEAD_TOP - HEAD_Y, 0.06, 0.015, headMat, 0, (HEAD_Y + HEAD_TOP) / 2, s * (HZ - 0.03)));
    const capShape = rrShape(-0.3, HEAD_TOP, 0.3, 2.25, 0.03);
    for (const z of [-VALVE_Z, VALVE_Z]) capShape.holes.push(circleHole(-z, CAM_Y, 0.04));
    const camCaps = new THREE.Group();
    for (const x of CAP_X) camCaps.add(new THREE.Mesh(extX(capShape, x - 0.045, x + 0.045, 16), headMat));
    const capBolts = [];
    for (const x of CAP_X) for (const z of [-0.26, 0, 0.26]) capBolts.push([x, 2.26, z]);
    const headBolts = [];
    for (const c of BORE_X) for (const dx of [-0.3, 0.3]) for (const z of [-0.33, 0.33]) headBolts.push([c + dx, GAL_Y + 0.012, z]);
    // thermostat housing on the gearbox end
    head.add(rbox(0.1, 0.16, 0.2, 0.03, headMat, HX + 0.05, 1.82, 0.12));
    root.add(
      ctx.part(head, {
        id: 'cylinderHead',
        name: 'Testata',
        base: 'Il “coperchio” dei cilindri: qui ci sono le camere di scoppio a tetto, le valvole, le candele e i passaggi dell’acqua (i fori nella sezione) che la tengono fresca.',
        expert:
          'Testata in alluminio con camere di combustione a tetto (pent-roof), condotti di aspirazione e scarico, pozzetti delle candele e camicie d’acqua. In alto la galleria degli alberi a camme con i supporti (torri) e i cappelli imbullonati. Due valvole per cilindro in questo modello didattico (nella realtà spesso quattro).',
        explode: [0, 1.0, 0],
      })
    );
    root.add(
      ctx.part(bolts(boltMat, headBolts, 'y', screw(0.55, 0.04, 0.03)), {
        id: 'headBolts',
        name: 'Viti della testata',
        base: 'Sedici viti lunghe che stringono la testata al basamento, schiacciando la guarnizione in mezzo.',
        expert: 'Viti (o prigionieri) ad alta resistenza che attraversano la testata e si avvitano nel basamento; si serrano in un ordine preciso, a spirale dal centro verso l’esterno, e si allentano nell’ordine inverso, per non deformare la testata. Spesso sono a snervamento e vanno sostituite.',
        explode: [0, 1.0, 0],
      }),
      ctx.part(camCaps, {
        id: 'camCaps',
        name: 'Cappelli degli alberi a camme',
        base: 'I “coperchietti” che tengono fermi gli alberi a camme sui loro supporti, come le staffe che tengono un tubo.',
        expert: 'Cappelli lavorati insieme alla testata (non sono intercambiabili: sono numerati e orientati); con i supporti formano i cuscinetti di strisciamento degli alberi a camme, lubrificati in pressione.',
        explode: [0, 1.95, 0],
      }),
      ctx.part(bolts(boltMat, capBolts, 'y', screw(0.28, 0.03, 0.025)), {
        id: 'camCapBolts',
        name: 'Viti dei cappelli',
        base: 'Piccole viti che fissano i cappelli degli alberi a camme alla testata.',
        expert: 'Si allentano a poco a poco e in sequenza, così gli alberi a camme (spinti dalle molle delle valvole) non si piegano.',
        explode: [0, 1.95, 0],
      })
    );

    // ---------- Cam cover: hollow U shell + end plates, coils on top ----------
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
    const filler = mesh(new THREE.CylinderGeometry(0.07, 0.075, 0.05, 24), MAT.paint(0x111317, 0.6), 1.45, COVER_TOP + 0.025, 0.16);
    cover.add(filler);
    const plate = mesh(new THREE.PlaneGeometry(0.5, 0.125).rotateX(-Math.PI / 2), new THREE.MeshStandardMaterial({ map: plateTexture(['MOTORILAB 1.6', 'DOHC · 4 cil.']), metalness: 0.6, roughness: 0.4 }), -0.82, COVER_TOP + 0.003, 0.17);
    plate.material = cut(plate.material, { cap: false });
    cover.add(plate);
    root.add(
      ctx.part(cover, {
        id: 'camCover',
        name: 'Coperchio punterie',
        base: 'Chiude in alto il motore e tiene dentro l’olio che bagna gli alberi a camme. Dal tappo giallo-nero si rabbocca l’olio.',
        expert: 'Coperchio in plastica o magnesio con guarnizione; integra spesso il separatore dei vapori d’olio (blow-by) e il tappo di riempimento olio.',
        explode: [0, 2.1, 0],
      }),
      ctx.part(bolts(boltMat, cvb, 'y', screw(0.38)), {
        id: 'camCoverBolts',
        name: 'Viti del coperchio',
        base: 'Le viti che fissano il coperchio punterie alla testata.',
        expert: 'Spesso hanno distanziali e gommini: il coperchio “galleggia” sulla guarnizione e trasmette meno vibrazioni e rumore.',
        explode: [0, 2.1, 0],
      })
    );

    // ---------- Crankshaft ----------
    const crankMat = MAT.steel();
    const crank = new THREE.Group();
    crank.add(mesh(alongX(new THREE.CylinderGeometry(0.11, 0.11, 3.5, 28)), crankMat));
    crank.add(mesh(alongX(new THREE.CylinderGeometry(0.065, 0.065, 0.44, 20)), crankMat, -1.96, 0, 0));
    crank.add(mesh(alongX(new THREE.CylinderGeometry(0.16, 0.16, 0.06, 32)), crankMat, 1.8, 0, 0));
    const pinGeo = alongX(new THREE.CylinderGeometry(0.09, 0.09, 0.26, 24));
    const webGeo = new THREE.BoxGeometry(0.07, 0.62, 0.32);
    const cwGeo = alongX(new THREE.CylinderGeometry(0.3, 0.3, 0.07, 32, 1, false, Math.PI / 2, Math.PI));
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
        name: 'Albero motore (albero a gomiti)',
        base: 'L’albero “a zig-zag” che trasforma il su-e-giù dei pistoni in rotazione, come i pedali di una bicicletta trasformano la spinta delle gambe.',
        expert: 'Albero in acciaio forgiato su 5 supporti di banco, manovelle a 180° (configurazione piatta del 4 in linea). I contrappesi bilanciano le masse rotanti; le forze alterne del secondo ordine restano e si compensano, se serve, con alberi di equilibratura.',
        explode: [0, -1.1, 0.4],
      })
    );

    // ---------- Pistons, conrods ----------
    const pistonGeo = lathe([[0, 0.16], [0.32, 0.16], [0.33, 0.15], [0.33, -0.2], [0.31, -0.2], [0.31, 0.1], [0, 0.1]], 48);
    const ringGeo = new THREE.TorusGeometry(0.331, 0.008, 6, 48).rotateX(Math.PI / 2);
    const wristGeo = alongX(new THREE.CylinderGeometry(0.06, 0.06, 0.6, 16));
    const pistonMat = MAT.aluminum();
    const pistonGroup = new THREE.Group();
    const pistons = BORE_X.map((x) => {
      const p = new THREE.Group();
      p.add(new THREE.Mesh(pistonGeo, pistonMat));
      for (const y of [0.13, 0.095, 0.06]) p.add(mesh(ringGeo, darkSteel, 0, y, 0));
      p.add(new THREE.Mesh(wristGeo, steel));
      p.position.x = x;
      pistonGroup.add(p);
      return p;
    });
    root.add(
      ctx.part(pistonGroup, {
        id: 'pistons',
        name: 'Pistoni',
        base: 'I “tappi mobili” dentro i cilindri: ricevono la spinta dell’esplosione e la passano alla biella.',
        expert: 'Pistoni in lega Al-Si con tre fasce elastiche (due di tenuta, una raschiaolio). Velocità media a 6 500 giri con corsa 86 mm: ≈ 19 m/s; accelerazioni oltre 2 000 g al PMS.',
        explode: [0, 0.2, 1.5],
      })
    );

    const rodBeam = new THREE.BoxGeometry(0.05, L - 0.2, 0.09).translate(0, L / 2, 0);
    const bigEnd = new THREE.TorusGeometry(0.11, 0.035, 10, 28).rotateY(Math.PI / 2);
    const smallEnd = new THREE.TorusGeometry(0.075, 0.025, 10, 24).rotateY(Math.PI / 2).translate(0, L, 0);
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
        expert: 'Bielle in acciaio forgiato con sezione a doppio T, testa divisa (spesso cracked) con bronzine, piede con spinotto flottante. Rapporto biella/manovella λ = r/L ≈ 0,3.',
        explode: [0, -0.3, 1.2],
      })
    );

    // ---------- Valvetrain ----------
    const valveGeo = lathe([[0, 0], [0.1, 0], [0.1, 0.012], [0.03, 0.05], [0.022, 0.09], [0.022, 0.5], [0, 0.5]], 24);
    const springGeo = new THREE.TubeGeometry(
      new (class extends THREE.Curve {
        getPoint(t, out = new THREE.Vector3()) {
          const a = t * TAU * 5;
          return out.set(Math.cos(a) * 0.045, t * 0.2, Math.sin(a) * 0.045);
        }
      })(),
      90,
      0.008,
      6
    );
    const bucketGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.06, 20);
    const makeValves = (z, matValve) => {
      const g = new THREE.Group();
      const list = BORE_X.map((x) => {
        const v = new THREE.Group();
        v.add(new THREE.Mesh(valveGeo, matValve), mesh(springGeo, darkSteel, 0, 0.3, 0), mesh(bucketGeo, MAT.chrome(), 0, 0.53, 0));
        v.position.set(x, HEAD_Y - 0.005, z);
        g.add(v);
        return v;
      });
      return { g, list };
    };
    const inV = makeValves(-VALVE_Z, MAT.steel());
    const exV = makeValves(VALVE_Z, MAT.titanium());
    root.add(
      ctx.part(inV.g, {
        id: 'intakeValves',
        name: 'Valvole di aspirazione',
        base: 'Porte che si aprono per far entrare la miscela di aria e benzina nel cilindro.',
        expert: 'Valvole a fungo in acciaio, diametro maggiore di quelle di scarico per riempire meglio il cilindro. Alzata 8–10 mm, richiamate dalle molle e comandate dalle camme tramite punterie a bicchierino.',
        explode: [0, 0.6, -0.5],
      }),
      ctx.part(exV.g, {
        id: 'exhaustValves',
        name: 'Valvole di scarico',
        base: 'Porte che si aprono per far uscire i gas bruciati.',
        expert: 'Lavorano a 700–800 °C: acciaio austenitico o stelo cavo riempito di sodio per smaltire calore verso la sede.',
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
        const lobe = mesh(lobeGeo, MAT.chrome(), x, 0, 0);
        lobe.rotation.x = -TAU * (OFFSET[k] + peak);
        g.add(lobe);
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
        base: 'Alberi con delle “gobbe” (camme): girando, le gobbe spingono giù le valvole al momento giusto, come le punte di un carillon.',
        expert: 'Doppio albero a camme in testa (DOHC), girano a metà dei giri dell’albero motore, appoggiati ai supporti della testata. Il profilo della camma decide alzata, durata e fasatura; i variatori di fase (VVT) li ruotano di qualche grado in base a giri e carico.',
        explode: [0, 1.7, 0],
      })
    );

    // ---------- Spark plugs ----------
    const plugs = new THREE.Group();
    const sparks = [];
    const sparkLights = [];
    const sparkGeo = new THREE.SphereGeometry(0.06, 16, 12);
    BORE_X.forEach((x) => {
      const p = new THREE.Group();
      p.add(
        mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.45, 16), MAT.ceramic(), 0, 0.5, 0),
        mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.07, 6), MAT.chrome(), 0, 0.22, 0),
        mesh(new THREE.CylinderGeometry(0.024, 0.02, 0.2, 12), steel, 0, 0.09, 0)
      );
      p.position.set(x, HEAD_Y - 0.02, 0);
      plugs.add(p);
      const spark = mesh(sparkGeo, MAT.glow(0xbfe6ff, 6, 0.95), x, HEAD_Y - 0.03, 0);
      spark.visible = false;
      root.add(spark);
      sparks.push(spark);
      const light = new THREE.PointLight(0xffa040, 0, 2.2, 2);
      light.position.set(x, HEAD_Y - 0.15, 0.1);
      root.add(light);
      sparkLights.push(light);
    });
    root.add(
      ctx.part(plugs, {
        id: 'sparkPlugs',
        name: 'Candele',
        base: 'Fanno la scintilla che accende la miscela, proprio come l’accendino di un fornello.',
        expert: 'Elettrodi in iridio o platino, tensione d’accensione 20–40 kV prodotta da una bobina per cilindro (coil-on-plug). L’anticipo è calcolato dalla centralina in base a giri, carico e sensore di battito.',
        explode: [0, 1.5, 0.3],
      })
    );

    // ---------- Ignition coils + harness ----------
    const coils = new THREE.Group();
    const coilMat = MAT.paint(0x15171b, 0.45);
    const connMat = MAT.paint(0x3b3f45, 0.6);
    BORE_X.forEach((x) => {
      coils.add(mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.24, 16), coilMat, x, 2.3, 0));
      coils.add(rbox(0.14, 0.09, 0.24, 0.025, coilMat, x, COVER_TOP + 0.05, 0));
      coils.add(rbox(0.07, 0.05, 0.06, 0.01, connMat, x, COVER_TOP + 0.06, -0.15));
      coils.add(rod([x, COVER_TOP + 0.06, -0.18], [x, COVER_TOP + 0.04, -0.24], 0.012, rubber, 6));
    });
    coils.add(tube(curve([[-1.75, COVER_TOP + 0.03, -0.24], [-0.4, COVER_TOP + 0.04, -0.24], [1.3, COVER_TOP + 0.04, -0.24], [1.5, COVER_TOP + 0.01, -0.3]]), 0.02, rubber, 40, 8));
    coils.add(rbox(0.12, 0.06, 0.09, 0.01, connMat, -1.78, COVER_TOP + 0.03, -0.24));
    root.add(
      ctx.part(coils, {
        id: 'ignitionCoils',
        name: 'Bobine d’accensione',
        base: 'Una bobina per candela: trasforma i 12 volt della batteria nelle decine di migliaia di volt che servono per la scintilla. Il cablaggio le collega alla centralina.',
        expert: 'Bobine a matita (coil-on-plug) con primario a bassa tensione e secondario a migliaia di spire: l’interruzione della corrente primaria comandata dalla centralina induce 20–40 kV sul secondario. Il cablaggio porta anche i segnali di sensori (fase, battito, temperatura).',
        explode: [0, 2.6, 0.2],
      })
    );

    // ---------- Gas charge inside each cylinder ----------
    const chargeGeo = new THREE.CylinderGeometry(0.335, 0.335, 1, 40);
    const charges = BORE_X.map((x) => {
      const m = mesh(chargeGeo, MAT.gas(0x5fd3ff, 0.2, false), x);
      m.renderOrder = 2;
      root.add(m);
      return m;
    });

    // ---------- Intake: plenum, runners, throttle body, air box ----------
    const intake = new THREE.Group();
    const intakeMat = MAT.paint(0x2a3442, 0.5);
    intake.add(mesh(alongX(new THREE.CylinderGeometry(0.22, 0.22, 3.0, 32)), intakeMat, 0, 1.05, -1.15));
    for (const x of [-1.5, 1.5]) intake.add(mesh(alongX(new THREE.CylinderGeometry(0.23, 0.23, 0.04, 32)), intakeMat, x, 1.05, -1.15));
    intake.add(mesh(alongX(new THREE.CylinderGeometry(0.16, 0.16, 0.3, 32)), MAT.aluminum(), -1.65, 1.05, -1.15));
    intake.add(rbox(0.12, 0.1, 0.08, 0.02, MAT.paint(0x15171b, 0.5), -1.65, 1.25, -1.15));
    intake.add(rbox(0.1, 0.06, 0.08, 0.015, MAT.paint(0x15171b, 0.5), 0.6, 1.29, -1.15));
    intake.add(tube(curve([[-1.8, 1.05, -1.15], [-1.95, 1.07, -1.17], [-2.02, 1.18, -1.25], [-2.04, 1.4, -1.35]]), 0.14, rubber, 20, 16));
    intake.add(mesh(new THREE.TorusGeometry(0.145, 0.012, 8, 32).rotateY(Math.PI / 2), steel, -1.86, 1.05, -1.15));
    intake.add(rbox(2.9, 0.16, 0.04, 0.02, intakeMat, 0, 1.79, -HZ - 0.02));
    const intakePaths = BORE_X.map((x) => {
      const pts = [[x, 1.05, -1.15], [x, 1.55, -1.05], [x, 1.86, -0.78], [x, 1.8, -0.5]];
      intake.add(tube(curve(pts), 0.1, intakeMat, 30, 14));
      return curve([...pts, [x, 1.76, -0.35], [x, HEAD_Y + 0.05, -VALVE_Z]]);
    });
    root.add(
      ctx.part(intake, {
        id: 'intakeManifold',
        name: 'Collettore di aspirazione',
        base: 'I tubi che portano l’aria dal filtro ai cilindri. La farfalla, all’ingresso, è collegata al pedale dell’acceleratore.',
        expert: 'Plenum + condotti accordati: la lunghezza dei condotti sfrutta le onde di pressione per migliorare il riempimento a certi regimi. La farfalla elettronica (drive-by-wire) dosa l’aria; il sensore MAP sul plenum e il debimetro misurano l’aria aspirata.',
        explode: [0, 0.3, -1.3],
      })
    );

    const injectors = new THREE.Group();
    injectors.add(mesh(alongX(new THREE.CylinderGeometry(0.04, 0.04, 3.0, 12)), MAT.chrome(), 0, 2.1, -0.68));
    injectors.add(rod([1.5, 2.1, -0.68], [1.75, 1.9, -0.95], 0.014, steel, 6));
    BORE_X.forEach((x) => {
      const inj = mesh(new THREE.CylinderGeometry(0.035, 0.02, 0.24, 12), MAT.paint(0x222831), x, 1.99, -0.62);
      inj.rotation.x = 0.4;
      injectors.add(inj, rbox(0.05, 0.04, 0.05, 0.01, connMat, x + 0.05, 2.07, -0.66));
    });
    root.add(
      ctx.part(injectors, {
        id: 'injectors',
        name: 'Iniettori e flauto',
        base: 'Spruzzini comandati dal computer del motore: nebulizzano la benzina nell’aria che entra.',
        expert: 'Iniezione indiretta multipoint sequenziale a 3–5 bar; molti motori moderni usano l’iniezione diretta (GDI) a 200–350 bar direttamente in camera. La centralina regola la quantità in anello chiuso con la sonda lambda.',
        explode: [0, 1.0, -1.0],
      })
    );

    // ---------- Exhaust: flange, runners, collector, lambda, catalyst ----------
    const exhaust = new THREE.Group();
    const exMat = cut(MAT.paint(0x6b4a3a, 0.55));
    exMat.metalness = 0.8;
    exhaust.add(rbox(2.9, 0.16, 0.04, 0.02, exMat, 0, 1.79, HZ + 0.02));
    const studs = [];
    BORE_X.forEach((x) => studs.push([x - 0.17, 1.79, HZ + 0.05], [x + 0.17, 1.79, HZ + 0.05]));
    exhaust.add(bolts(boltMat, studs, 'z'));
    const exhaustPaths = BORE_X.map((x) => {
      const pts = [[x, 1.79, HZ + 0.03], [x, 1.78, 0.68], [x * 0.55, 1.3, 0.95], [0.1, 0.8, 1.05], [0.3, -0.05, 1.05]];
      exhaust.add(tube(curve(pts), 0.085, exMat, 50, 14));
      return curve([[x, HEAD_Y + 0.05, VALVE_Z], [x, 1.76, 0.33], ...pts, [0.3, -0.3, 1.05]]);
    });
    exhaust.add(mesh(new THREE.CylinderGeometry(0.035, 0.03, 0.12, 12), cut(MAT.chrome()), 0.38, 0.45, 1.13).rotateZ(-0.5));
    const catGeo = lathe([[0, -0.38], [0.1, -0.38], [0.21, -0.25], [0.21, 0.2], [0.1, 0.32], [0, 0.32]], 40);
    exhaust.add(mesh(catGeo, exMat, 0.3, -0.25, 1.05));
    for (const y of [0.07, -0.63]) exhaust.add(mesh(new THREE.CylinderGeometry(0.13, 0.13, 0.03, 24), exMat, 0.3, y, 1.05));
    root.add(
      ctx.part(exhaust, {
        id: 'exhaustManifold',
        name: 'Collettore di scarico e catalizzatore',
        base: 'I tubi che raccolgono i gas bruciati e li mandano al catalizzatore, che li rende meno inquinanti. Il sensore sul tubo (sonda lambda) “annusa” i gas.',
        expert: 'Collettore 4‑in‑1 in acciaio inox imbullonato alla testata con prigionieri; il catalizzatore trivalente (Pt, Pd, Rh) converte CO, HC e NOx con efficienza > 95% solo se la miscela è stechiometrica (λ = 1), mantenuta dalla sonda lambda.',
        explode: [0, 0.1, 1.6],
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
    damper.add(new THREE.Mesh(alongX(lathe(damperProfile, 48)), darkSteel));
    damper.add(mesh(alongX(new THREE.CylinderGeometry(0.07, 0.07, 0.1, 24)), steel));
    for (let i = 0; i < 4; i++) damper.add(mesh(alongX(new THREE.CylinderGeometry(0.022, 0.022, 0.095, 10)), MAT.paint(0x22262b), 0, Math.cos((i / 4) * TAU) * 0.11, Math.sin((i / 4) * TAU) * 0.11));
    acc.add(damper);
    const altPulley = new THREE.Group();
    altPulley.position.set(XACC, 0.78, 0.8);
    altPulley.add(new THREE.Mesh(alongX(lathe([[0.02, -0.045], [0.058, -0.045], [0.052, 0], [0.058, 0.045], [0.02, 0.045]], 32)), steel));
    acc.add(altPulley);
    const altBody = mesh(alongX(lathe([[0, -0.24], [0.15, -0.24], [0.17, -0.2], [0.17, 0.18], [0.14, 0.24], [0, 0.24]], 40)), MAT.aluminum(), -1.78, 0.78, 0.8);
    acc.add(altBody);
    for (let i = 0; i < 10; i++) acc.add(rbox(0.36, 0.012, 0.03, 0.004, darkSteel, -1.78, 0.78 + Math.cos((i / 10) * TAU) * 0.172, 0.8 + Math.sin((i / 10) * TAU) * 0.172));
    acc.add(rbox(0.16, 0.12, 0.2, 0.02, blockMat, -1.62, 0.62, 0.63));
    acc.add(mesh(alongX(new THREE.CylinderGeometry(0.045, 0.045, 0.1, 24)), steel, XACC, 0.62, 0.3));
    acc.add(rod([XACC + 0.06, 0.62, 0.3], [XACC + 0.06, 0.42, 0.5], 0.02, darkSteel, 8));
    const accLoop = loopPath(accCircles);
    acc.add(new THREE.Mesh(ribbonGeo(accLoop, XACC, 0.08, 0.016), kitMats.belt));
    cutAll(acc);
    root.add(
      ctx.part(acc, {
        id: 'accessoryDrive',
        name: 'Alternatore e cinghia servizi',
        base: 'Una seconda cinghia, all’esterno, fa girare l’alternatore che ricarica la batteria (e spesso il compressore del clima). La puleggia grande sull’albero smorza anche le vibrazioni.',
        expert: 'Cinghia Poly-V a più gole con tendicinghia automatico; puleggia smorzatrice torsionale (damper) sull’albero motore. L’alternatore trifase con regolatore fornisce 14 V e 100–180 A.',
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
        base: 'Collega l’albero motore agli alberi a camme: le valvole si aprono sempre sincronizzate con i pistoni (rapporto 2:1). ' + txt.base,
        expert: txt.expert,
        explode: [-0.8, 0, 0],
      });
      const b = ctx.part(t.cover, {
        id: `timingCover_${kind}`,
        name: timingInfo[kind][1],
        base: kind === 'dryBelt' ? 'Un coperchio leggero in plastica: protegge la cinghia da sporco e acqua, ma non deve trattenere olio.' : 'Un carter metallico a tenuta: chiude la distribuzione dentro il motore, bagnata dall’olio.',
        expert: kind === 'dryBelt' ? 'Carter in materiale plastico, in due pezzi, fissato con viti; la tenuta dell’olio è affidata ai paraolio degli alberi.' : 'Carter in lega di alluminio con guarnizione, parte del circuito dell’olio: per ispezionare la distribuzione va smontato.',
        explode: [-1.6, 0, 0],
      });
      root.add(a, b);
      if (t.splash) root.add(t.splash.object);
      timingParts[kind] = { t, parts: [a, b] };
    }
    let timingKind = 'dryBelt';
    const setTiming = (kind) => {
      timingKind = timingParts[kind] ? kind : 'dryBelt';
      for (const [k, v] of Object.entries(timingParts)) {
        v.parts.forEach((p) => (p.visible = k === timingKind));
        if (v.t.splash) v.t.splash.object.visible = k === timingKind;
      }
    };
    setTiming('dryBelt');

    // ---------- Flywheel ----------
    const flywheel = new THREE.Group();
    flywheel.add(mesh(alongX(new THREE.CylinderGeometry(0.55, 0.55, 0.08, 64)), darkSteel));
    flywheel.add(mesh(new THREE.TorusGeometry(0.56, 0.03, 8, 90).rotateY(Math.PI / 2), steel));
    flywheel.add(mesh(new THREE.BoxGeometry(0.09, 0.3, 0.06), MAT.paint(0xff7a1a), 0.02, 0.3, 0));
    flywheel.add(bolts(steel, Array.from({ length: 6 }, (_, i) => [-0.045, Math.cos((i / 6) * TAU) * 0.1, Math.sin((i / 6) * TAU) * 0.1]), 'x'));
    flywheel.position.x = 1.9;
    root.add(
      ctx.part(flywheel, {
        id: 'flywheel',
        name: 'Volano',
        base: 'Un disco pesante che, come una trottola, continua a girare e “porta” il motore tra uno scoppio e l’altro.',
        expert: 'Accumula energia cinetica per regolarizzare la coppia pulsante; spesso a doppia massa con molle torsionali. La corona dentata serve al motorino d’avviamento.',
        explode: [1.0, 0, 0],
      })
    );

    // ---------- Flows ----------
    const intakeFlows = intakePaths.map((c) => {
      const f = flow({ path: (t, out) => c.getPointAt(t, out), count: 70, size: 0.07, colors: [0x5fd3ff, 0x5fd3ff, 0xffc04d], spread: 0.06, speed: 1 });
      root.add(f.object);
      return f;
    });
    const exhaustFlows = exhaustPaths.map((c) => {
      const f = flow({ path: (t, out) => c.getPointAt(t, out), count: 80, size: 0.08, colors: [0xff5a1f, 0xb07a6a, 0x8c86a8], spread: 0.05, speed: 0.8 });
      root.add(f.object);
      return f;
    });

    // ---------- Animation ----------
    const cA = new THREE.Color();
    const C = {
      air: new THREE.Color(0x5fd3ff),
      mix: new THREE.Color(0x9ad8a0),
      fuel: new THREE.Color(0xffc04d),
      flash: new THREE.Color(0xfff2c4),
      fire: new THREE.Color(0xff5a1f),
      exh: new THREE.Color(0x8c86a8),
    };
    const rpmOf = (th) => 800 + th * 5700;
    const animRate = (th) => 0.12 + th * 0.9; // visible slow motion, cycles/s

    return {
      root,
      cycleRate: animRate,
      setOption(id, value) {
        if (id === 'timing') setTiming(value);
      },
      update(state) {
        const theta1 = state.cycle * Math.PI * 4;
        crank.rotation.x = theta1;
        flywheel.rotation.x = theta1;
        damper.rotation.x = theta1;
        altPulley.rotation.x = theta1 * (0.17 / 0.058);
        const camAngle = state.cycle * TAU;
        camIn.rotation.x = camAngle;
        camEx.rotation.x = camAngle;
        const rate = animRate(state.throttle);
        const assembled = (state.explode || 0) < 0.05;

        const tp = timingParts[timingKind].t;
        tp.update(theta1);
        if (tp.splash) {
          tp.splash.object.visible = assembled;
          tp.splash.update(state.dt, rate * 1.5);
        }

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
          inV.list[k].position.y = HEAD_Y - 0.005 - li * 0.09;
          exV.list[k].position.y = HEAD_Y - 0.005 - le * 0.09;

          // Gas charge between piston crown and the chamber roof.
          const top = yp + 0.16;
          const ch = charges[k];
          const roof = HEAD_Y + 0.04;
          ch.position.y = (top + roof) / 2;
          ch.scale.y = Math.max(0.001, roof - top);
          let op;
          if (local < 0.25) {
            const f = local / 0.25;
            cA.copy(C.air).lerp(C.mix, f);
            op = 0.12 + 0.12 * f;
          } else if (local < 0.5) {
            const f = (local - 0.25) / 0.25;
            cA.copy(C.mix).lerp(C.fuel, f);
            op = 0.25 + 0.3 * f;
          } else if (local < 0.75) {
            const f = (local - 0.5) / 0.25;
            if (f < 0.2) cA.copy(C.flash).lerp(C.fire, f / 0.2);
            else cA.copy(C.fire).lerp(C.exh, (f - 0.2) / 0.8);
            op = lerp(0.95, 0.35, f);
          } else {
            cA.copy(C.exh);
            op = lerp(0.35, 0.08, (local - 0.75) / 0.25);
          }
          ch.material.color.copy(cA);
          ch.material.opacity = op;
          ch.visible = assembled;

          const sparkOn = local > 0.485 && local < 0.53;
          sparks[k].visible = sparkOn && assembled;
          sparks[k].scale.setScalar(sparkOn ? 0.6 + Math.random() * 0.6 : 1);
          const flash = local > 0.49 && local < 0.62 ? 1 - (local - 0.49) / 0.13 : 0;
          sparkLights[k].intensity = assembled ? flash * (2 + state.throttle * 4) : 0;

          intakeFlows[k].object.visible = exhaustFlows[k].object.visible = assembled;
          intakeFlows[k].material.opacity = clamp(li * 1.6, 0, 0.9);
          intakeFlows[k].update(state.dt, li * rate * 1.6);
          exhaustFlows[k].material.opacity = clamp(le * 1.6, 0, 0.9);
          exhaustFlows[k].update(state.dt, le * rate * 1.6);
        });
      },
      readout(state) {
        const rpm = rpmOf(state.throttle);
        const torque = 140 + 60 * Math.sin(Math.PI * Math.min(1, state.throttle * 0.9 + 0.1));
        const kw = (torque * rpm * 2 * Math.PI) / 60 / 1000;
        return [
          { label: 'Giri reali', value: Math.round(rpm / 10) * 10, unit: 'giri/min' },
          { label: 'Coppia', value: Math.round(torque), unit: 'Nm' },
          { label: 'Potenza', value: Math.round(kw), unit: 'kW' },
          { label: 'Scoppi al secondo', value: Math.round((rpm / 60) * 2), unit: '/s' },
        ];
      },
    };
  },
};
