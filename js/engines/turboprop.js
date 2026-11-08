// Free-turbine turboprop (PW127 / PW150 class), technical cutaway.
// Axis along X: air enters the chin intake below the spinner (-X), flows aft through a two-spool
// gas generator with a reverse-flow combustor, drives a free power turbine and leaves through a
// rear duct bent upward. The propeller axis sits above the engine axis (offset reduction gearbox).
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, blade, ringX, flow, clamp, lerp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const DEG = Math.PI / 180;
const OX = 0.4; // model offset along X (keeps the model centred)

// ---------- Reduction gearbox layout (pitch radius = teeth * GM) ----------
const GM = 0.005;
const N_IN = 13; // input pinion on the power-turbine shaft
const N_LAY = 44; // first-stage gears on the two layshafts
const N_LAYP = 15; // second-stage pinions on the layshafts
const N_OUT = 66; // output gear on the propeller shaft
const LAY_Z = 0.25; // layshafts sit left/right of the centre plane
const LAY_A = Math.sqrt(((N_IN + N_LAY) * GM) ** 2 - LAY_Z ** 2);
const LAY_C = Math.sqrt(((N_LAYP + N_OUT) * GM) ** 2 - LAY_Z ** 2);
const YP = 0; // propeller axis height
const YE = YP - LAY_A - LAY_C; // engine axis height (offset below)
const YL = YE + LAY_A; // layshaft height
const RATIO = (N_LAY / N_IN) * (N_OUT / N_LAYP); // ~14.9:1
const XB = -2.2; // propeller blade plane
const X_G1 = -1.62; // first gear stage plane
const X_G2 = -1.8; // second gear stage plane
const PROP_VIS = 0.2; // propeller turns shown per second (real: 20 per second)

// ---------- Geometry helpers (local) ----------
// Lathe around X from [[radius, x], ...]; optional partial sweep.
function latheX(pts, seg = 72, phiStart = 0, phiLength = TAU) {
  const g = new THREE.LatheGeometry(pts.map(([r, x]) => new THREE.Vector2(r, x)), seg, phiStart, phiLength);
  g.rotateZ(-Math.PI / 2); // lathe Y axis -> X axis
  return g;
}

// Thin shell from a [radius, x] polyline, thickness t (negative = inward).
function wall(pts, t = 0.012, seg = 72, phiStart = 0, phiLength = TAU) {
  const back = pts.map(([r, x]) => [r + t, x]).reverse();
  return latheX([...pts, ...back, pts[0]], seg, phiStart, phiLength);
}

// Merge geometries (position + normal) into one indexed geometry.
function mergeGeos(list) {
  let nV = 0;
  let nI = 0;
  for (const g of list) {
    nV += g.attributes.position.count;
    nI += g.index ? g.index.count : g.attributes.position.count;
  }
  const pos = new Float32Array(nV * 3);
  const nor = new Float32Array(nV * 3);
  const idx = new Uint32Array(nI);
  let vo = 0;
  let io = 0;
  for (const g of list) {
    const n = g.attributes.position.count;
    pos.set(g.attributes.position.array, vo * 3);
    nor.set(g.attributes.normal.array, vo * 3);
    if (g.index) {
      const a = g.index.array;
      for (let i = 0; i < a.length; i++) idx[io + i] = a[i] + vo;
      io += a.length;
    } else {
      for (let i = 0; i < n; i++) idx[io + i] = vo + i;
      io += n;
    }
    vo += n;
    g.dispose();
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  out.setIndex(new THREE.BufferAttribute(idx, 1));
  return out;
}

// ringX() collapsed into a single mesh.
function mergeRing(geo, mat, count, radius) {
  const group = ringX(geo, mat, count, radius);
  const list = group.children.map((m) => {
    m.updateMatrix();
    return geo.clone().applyMatrix4(m.matrix);
  });
  geo.dispose();
  return new THREE.Mesh(mergeGeos(list), mat);
}

function rotorRow(x, rHub, rTip, count, chord, mat, { twist = 0.5, stagger = 0.55, thick = 0.012 } = {}) {
  const g = blade(rTip - rHub, chord, thick, twist);
  g.rotateY(-stagger);
  const m = mergeRing(g, mat, count, rHub);
  m.position.x = x;
  return m;
}

function statorRow(x, rIn, rOut, count, chord, mat, angle = 0.5, thick = 0.01) {
  const L = rOut - rIn;
  const g = new THREE.BoxGeometry(chord, L, thick);
  g.translate(0, L / 2, 0);
  g.rotateY(angle);
  const m = mergeRing(g, mat, count, rIn);
  m.position.x = x;
  return m;
}

// Disc carrying a blade row (hub web), centred at x.
function disc(x, rIn, rRim, w, mat) {
  return new THREE.Mesh(
    latheX([[rIn, x - w * 0.6], [rRim * 0.75, x - w * 0.35], [rRim, x - w / 2], [rRim, x + w / 2], [rRim * 0.75, x + w * 0.35], [rIn, x + w * 0.6], [rIn, x - w * 0.6]], 48),
    mat,
  );
}

// Toothed outline, tooth k centred at angle 2πk/N (inward = internal teeth).
function gearOutline(N, rp, inward = false) {
  const m = (2 * rp) / N;
  const ra = inward ? rp - m : rp + m;
  const rf = inward ? rp + 1.25 * m : rp - 1.25 * m;
  const step = TAU / N;
  const pts = [];
  for (let k = 0; k < N; k++) {
    const c = k * step;
    for (const [r, f] of [[rf, -0.5], [rf, -0.3], [ra, -0.13], [ra, 0.13], [rf, 0.3]]) {
      pts.push(new THREE.Vector2(r * Math.cos(c + f * step), r * Math.sin(c + f * step)));
    }
  }
  return pts;
}

// Spur gear with its axis along X. Tooth angle convention: atan2(y, z); rotation.x = -phase.
function spurX(N, rp, width, bore = 0) {
  const shape = new THREE.Shape(gearOutline(N, rp));
  if (bore > 0) {
    const h = new THREE.Path();
    h.absarc(0, 0, bore, 0, TAU, true);
    shape.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(shape, { depth: width, bevelEnabled: false, curveSegments: 24 });
  g.translate(0, 0, -width / 2);
  g.rotateY(-Math.PI / 2); // shape (s, t) -> (z = s, y = t)
  return g;
}

// Phase of gear B so its teeth mesh with gear A; alpha = direction from A to B.
function meshPhase(thA, nA, nB, alpha) {
  const d = (thA - alpha) % (TAU / nA);
  return alpha + Math.PI - (d * nA) / nB - Math.PI / nB;
}

// Cylinder between two points (unit-height cylinder is scaled).
const _up = new THREE.Vector3(0, 1, 0);
const _dir = new THREE.Vector3();
function placeBetween(mesh, a, b) {
  _dir.subVectors(b, a);
  const len = _dir.length() || 1e-6;
  mesh.position.addVectors(a, b).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(_up, _dir.multiplyScalar(1 / len));
  mesh.scale.set(1, len, 1);
}
function cylBetweenGeo(a, b, r, seg = 8) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 1, seg));
  placeBetween(m, a, b);
  m.updateMatrix();
  return m.geometry.applyMatrix4(m.matrix);
}

// Blade with airfoil section. r: 0..L radial, c: chord (leading edge at c < 0), t: thickness.
// twist(u) adds pitch; map(r, c, t) -> [x, y, z].
function airfoilGeo(L, chord, thick, { segL = 16, segC = 8, plan = () => 1, thk = () => 1, twist = () => 0, map }) {
  const g = new THREE.BoxGeometry(chord, L, thick, segC, segL, 1);
  const p = g.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const r = p.getY(i) + L / 2;
    const u = r / L;
    const w = p.getX(i) / chord + 0.5;
    const af = w < 0.3 ? Math.sqrt(Math.max(0, 1 - ((0.3 - w) / 0.3) ** 2)) : 1 - 0.88 * ((w - 0.3) / 0.7) ** 1.4;
    const c = p.getX(i) * plan(u);
    const t = p.getZ(i) * Math.max(0.12, af) * thk(u);
    const d = twist(u);
    const [x, y, z] = map(r, c * Math.cos(d) + t * Math.sin(d), -c * Math.sin(d) + t * Math.cos(d));
    p.setXYZ(i, x, y, z);
  }
  g.computeVertexNormals();
  return g;
}

// Tube lofted through sections in the XY plane: { p: [x, y], hw (along z), hh (in plane), e (superellipse) }.
function loft(sections, rings = 28, seg = 40) {
  const curve = new THREE.CatmullRomCurve3(sections.map((s) => new THREE.Vector3(s.p[0], s.p[1], 0)));
  const n = sections.length - 1;
  const pos = [];
  const tan = new THREE.Vector3();
  const c = new THREE.Vector3();
  for (let i = 0; i <= rings; i++) {
    const t = i / rings;
    curve.getPoint(t, c);
    curve.getTangent(t, tan);
    const f = t * n;
    const k = Math.min(Math.floor(f), n - 1);
    const s0 = sections[k];
    const s1 = sections[k + 1];
    const u = f - k;
    const hw = lerp(s0.hw, s1.hw, u);
    const hh = lerp(s0.hh, s1.hh, u);
    const e = lerp(s0.e ?? 2, s1.e ?? 2, u);
    const nx = -tan.y;
    const ny = tan.x;
    for (let j = 0; j <= seg; j++) {
      const a = (j / seg) * TAU;
      const ca = Math.cos(a);
      const sa = Math.sin(a);
      const su = Math.sign(ca) * Math.abs(ca) ** (2 / e);
      const sv = Math.sign(sa) * Math.abs(sa) ** (2 / e);
      pos.push(c.x + nx * hh * sv, c.y + ny * hh * sv, hw * su);
    }
  }
  const idx = [];
  for (let i = 0; i < rings; i++) {
    for (let j = 0; j < seg; j++) {
      const a = i * (seg + 1) + j;
      const b = a + seg + 1;
      idx.push(a, b, a + 1, b, b + 1, a + 1);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// Closed superellipse ring of one loft section (for rolled lips).
function lipGeo(s, r = 0.02) {
  const pts = [];
  const e = s.e ?? 2;
  for (let j = 0; j < 48; j++) {
    const a = (j / 48) * TAU;
    const ca = Math.cos(a);
    const sa = Math.sin(a);
    pts.push(new THREE.Vector3(s.p[0], s.p[1] + s.hh * Math.sign(sa) * Math.abs(sa) ** (2 / e), s.hw * Math.sign(ca) * Math.abs(ca) ** (2 / e)));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, true), 72, r, 8, true);
}

// Centrifugal impeller blades between hub and shroud curves ([r, x] lists), as one merged mesh.
function impeller(hub, shroud, count, mat, { wrap = 0.7, from = 0, n = 18 } = {}) {
  const ch = new THREE.CatmullRomCurve3(hub.map(([r, x]) => new THREE.Vector3(x, r, 0)));
  const cs = new THREE.CatmullRomCurve3(shroud.map(([r, x]) => new THREE.Vector3(x, r, 0)));
  const pos = [];
  const idx = [];
  const a = new THREE.Vector3();
  const b = new THREE.Vector3();
  for (let i = 0; i <= n; i++) {
    const u = from + ((1 - from) * i) / n;
    ch.getPoint(u, a);
    cs.getPoint(u, b);
    const th = wrap * (1 - u) ** 2; // inducer lean
    for (let j = 0; j <= 2; j++) {
      const x = lerp(a.x, b.x, j / 2);
      const r = lerp(a.y, b.y, j / 2);
      pos.push(x, r * Math.cos(th), r * Math.sin(th));
    }
  }
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < 2; j++) {
      const p = i * 3 + j;
      idx.push(p, p + 3, p + 1, p + 3, p + 4, p + 1);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return mergeRing(g, mat, count, 0);
}

// Arc-length sampled path through control points with colours: [[Vector3, hex], ...].
function colorPath(ctrl, n = 320) {
  const pts = ctrl.map(([v]) => v);
  const curve = new THREE.CatmullRomCurve3(pts);
  const s = curve.getSpacedPoints(n);
  const L = [0];
  for (let i = 1; i < pts.length; i++) L.push(L[i - 1] + pts[i].distanceTo(pts[i - 1]));
  const total = L[L.length - 1];
  const stops = ctrl.map(([, c], i) => [L[i] / total, new THREE.Color(c)]);
  const tmp = new THREE.Color();
  const colorAt = (t) => {
    for (let i = 0; i < stops.length - 1; i++) {
      const [t0, c0] = stops[i];
      const [t1, c1] = stops[i + 1];
      if (t <= t1) return tmp.copy(c0).lerp(c1, (t - t0) / (t1 - t0 || 1)).getHex();
    }
    return stops[stops.length - 1][1].getHex();
  };
  return {
    at(t, out) {
      const f = clamp(t, 0, 1) * n;
      const i = Math.min(Math.floor(f), n - 1);
      return out.lerpVectors(s[i], s[i + 1], f - i);
    },
    colors: (k = 40) => Array.from({ length: k + 1 }, (_, i) => colorAt(i / k)),
    tOf: (i) => L[i] / total,
  };
}

// Closed lofted tube with wall thickness t (outer surface follows the sections), so the section gets a hatch.
function loftWall(sections, t = 0.015, rings = 28, seg = 40) {
  const outer = loft(sections, rings, seg);
  const inner = loft(sections.map((s) => ({ ...s, hw: s.hw - t, hh: s.hh - t })), rings, seg);
  const po = outer.attributes.position.array;
  const pi = inner.attributes.position.array;
  const no = po.length / 3;
  const body = new THREE.BufferGeometry();
  const pos = new Float32Array(po.length * 2);
  pos.set(po, 0);
  pos.set(pi, po.length);
  const idx = Array.from(outer.index.array);
  const ii = inner.index.array;
  for (let k = 0; k < ii.length; k += 3) idx.push(ii[k] + no, ii[k + 2] + no, ii[k + 1] + no);
  body.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  body.setIndex(idx);
  body.computeVertexNormals();
  // end annuli (own vertices for crisp edges)
  const ends = [];
  for (const [ring, flip] of [[0, false], [rings, true]]) {
    const ep = [];
    const ei = [];
    for (let j = 0; j <= seg; j++) {
      const k = (ring * (seg + 1) + j) * 3;
      ep.push(po[k], po[k + 1], po[k + 2], pi[k], pi[k + 1], pi[k + 2]);
    }
    for (let j = 0; j < seg; j++) {
      const a = j * 2;
      const b = a + 2;
      if (flip) ei.push(a, a + 1, b, b, a + 1, b + 1);
      else ei.push(a, b, a + 1, b, b + 1, a + 1);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(ep, 3));
    g.setIndex(ei);
    g.computeVertexNormals();
    ends.push(g);
  }
  outer.dispose();
  inner.dispose();
  return mergeGeos([body, ...ends]);
}

// Closed ring (flange) around X between rIn and rOut, width w, centred at x.
function flangeX(x, rIn, rOut, w, seg = 72) {
  return latheX([[rIn, x - w / 2], [rOut, x - w / 2], [rOut, x + w / 2], [rIn, x + w / 2], [rIn, x - w / 2]], seg);
}

// Hex head with its shank of length `len` below it (local -Y), so a bolt backing out shows its shank.
function screwGeo(r, h, len) {
  return mergeGeos([new THREE.CylinderGeometry(r, r, h, 6), new THREE.CylinderGeometry(r * 0.6, r * 0.6, len, 8).translate(0, -h / 2 - len / 2, 0)]);
}

// Bolt heads as one InstancedMesh: add(pos, normal) then mesh(). Pass screwGeo() as `geo` for removable bolts.
function boltSet(mat, r = 0.011, h = 0.012, geo = new THREE.CylinderGeometry(r, r, h, 6)) {
  const list = [];
  const q = new THREE.Quaternion();
  const one = new THREE.Vector3(1, 1, 1);
  const n = new THREE.Vector3();
  const set = {
    add(p, normal) {
      q.setFromUnitVectors(_up, n.copy(normal).normalize());
      list.push(new THREE.Matrix4().compose(p.clone().addScaledVector(n, h / 2), q, one));
      return set;
    },
    // ring of n bolts around an axis parallel to X through (y0, z0), heads facing dir (+1 / -1 along X)
    ringX(x, r, count, dir = -1, y0 = 0, z0 = 0, phase = 0) {
      for (let i = 0; i < count; i++) {
        const a = phase + (i / count) * TAU;
        set.add(new THREE.Vector3(x, y0 + r * Math.cos(a), z0 + r * Math.sin(a)), new THREE.Vector3(dir, 0, 0));
      }
      return set;
    },
    // ring around an axis parallel to Y through (x0, z0), heads facing up/down
    ringY(y, r, count, dir = 1, x0 = 0, z0 = 0, phase = 0) {
      for (let i = 0; i < count; i++) {
        const a = phase + (i / count) * TAU;
        set.add(new THREE.Vector3(x0 + r * Math.cos(a), y, z0 + r * Math.sin(a)), new THREE.Vector3(0, dir, 0));
      }
      return set;
    },
    mesh() {
      const im = new THREE.InstancedMesh(geo, mat, Math.max(1, list.length));
      list.forEach((m, i) => im.setMatrixAt(i, m));
      im.count = list.length;
      im.instanceMatrix.needsUpdate = true;
      im.computeBoundingSphere();
      return im;
    },
  };
  return set;
}

// Small dark discs on a surface around X (dilution / cooling holes), one InstancedMesh.
function holesX(mat, rows, r = 0.011) {
  const geo = new THREE.CylinderGeometry(r, r, 0.006, 10);
  const mats = [];
  const q = new THREE.Quaternion();
  const n = new THREE.Vector3();
  const one = new THREE.Vector3(1, 1, 1);
  for (const [x, rad, count, phase = 0] of rows) {
    for (let i = 0; i < count; i++) {
      const a = phase + (i / count) * TAU;
      n.set(0, Math.cos(a), Math.sin(a));
      q.setFromUnitVectors(_up, n);
      mats.push(new THREE.Matrix4().compose(new THREE.Vector3(x, rad * n.y, rad * n.z), q, one));
    }
  }
  const im = new THREE.InstancedMesh(geo, mat, mats.length);
  mats.forEach((m, i) => im.setMatrixAt(i, m));
  im.instanceMatrix.needsUpdate = true;
  im.computeBoundingSphere();
  return im;
}

// Tube along a list of [x, y, z] points (pipes, hoses, harness); closed with end caps.
function pipeAlong(points, r = 0.012, seg = 48) {
  const curve = new THREE.CatmullRomCurve3(points.map((p) => new THREE.Vector3(...p)));
  return new THREE.TubeGeometry(curve, seg, r, 8, false);
}

// 2D convex hull (monotone chain) of [x, y] points.
function hull(points) {
  const p = points.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [];
  for (const q of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], q) <= 0) lower.pop();
    lower.push(q);
  }
  const upper = [];
  for (let i = p.length - 1; i >= 0; i--) {
    const q = p[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], q) <= 0) upper.pop();
    upper.push(q);
  }
  upper.pop();
  lower.pop();
  return lower.concat(upper);
}

const ds = (m) => {
  m.side = THREE.DoubleSide;
  return m;
};

// Flow colours
const C_AIR = 0x5fd3ff;
const C_AIR2 = 0x8fdcd8;
const C_WARM = 0xc9d890;
const C_HOT = 0xf2c26a;
const C_FIRE = 0xff5a1f;
const C_FIRE2 = 0xff6a28;
const C_TURB = 0xe0703a;
const C_EXH1 = 0xa8788a;
const C_EXH = 0x8c86a8;

export default {
  id: 'turboprop',
  name: 'Turboelica a turbina libera',
  short: 'Turboelica',
  family: 'aria',
  vehicle: 'Aereo regionale',
  throttleLabel: 'Manetta (potenza)',
  examples:
    'Pratt & Whitney Canada PW127 (ATR 42/72), PW150A (De Havilland Dash 8-400), Rolls-Royce AE 2100 (Lockheed C-130J), Europrop TP400-D6 (Airbus A400M), PT6A (Beechcraft King Air, Pilatus PC-12)',
  tagline: "Una turbina a gas che non spinge con il getto ma fa girare un'elica: la regina delle rotte brevi.",

  intro: {
    base:
      "Il turboelica è una turbina a gas, parente stretta del motore a getto, che però usa quasi tutta la sua energia per far girare un'<b>elica</b>. " +
      "L'aria entra da una bocca sotto l'ogiva, viene compressa, bruciata con il cherosene e fa girare delle turbine. L'ultima turbina, " +
      "<b>libera</b> e non collegata al compressore, muove l'elica attraverso un grande <b>riduttore</b> a ingranaggi. " +
      "Il trucco più bello: l'elica gira quasi sempre agli stessi giri, e per dare più o meno spinta le pale <b>ruotano su se stesse</b> e cambiano inclinazione (il passo).",
    expert:
      "Turboelica a <b>turbina libera</b> in stile Pratt & Whitney Canada PW100: generatore di gas <b>bialbero</b> (compressore di bassa pressione " +
      "assiale nel modello, come nel PW150; girante centrifuga di alta pressione), combustore anulare a <b>flusso invertito</b>, turbine HP e LP monostadio " +
      "e turbina di potenza a 2 stadi su un albero coassiale che attraversa il motore fino al <b>riduttore ad assi sfalsati</b> (rapporto ≈ 15–17:1). " +
      "L'elica a velocità costante (≈ 1.200 giri/min al decollo sull'ATR 72) assorbe la potenza variando il <b>passo</b>; l'elica fornisce circa il 90% della spinta, " +
      "il getto residuo il resto. Rendimento ottimo fino a ~650 km/h.",
  },

  specs: [
    { label: 'Carburante', value: 'Cherosene Jet A-1' },
    { label: 'Potenza all\'albero', value: '~2.000–2.500 kW (famiglia PW127, ATR 72)' },
    { label: 'Giri elica', value: '~1.200 giri/min al decollo, quasi costanti' },
    { label: 'Riduttore', value: '2 stadi ad assi sfalsati, ~15–17:1' },
    { label: 'Elica', value: '6 pale a passo variabile, Ø 3,93 m (ATR 72)' },
    { label: 'Peso', value: '~480 kg (PW127, motore secco)' },
    { label: 'Rendimento', value: '~30% all\'albero · elica ~85%' },
  ],

  metrics: {
    efficiency: 30,
    power: 2050,
    powerDensity: 4.3,
  },

  variants: {
    columns: ['Motore', 'Aereo', 'Potenza al decollo', 'Schema'],
    rows: [
      ['P&WC PW127M / PW127XT-M', 'ATR 72-600', '~2.050 kW (2.750 shp)', '2 compressori centrifughi, 3 alberi, riduttore sfalsato'],
      ['P&WC PW150A', 'De Havilland Dash 8-400', '~3.780 kW (5.071 shp)', 'Assiale + centrifugo, 3 alberi, elica a 6 pale'],
      ['P&WC PT6A (famiglia)', 'King Air, Pilatus PC-12, Cessna Caravan', '~430–1.450 kW', 'Flusso invertito, assiale + centrifugo, scarichi laterali'],
      ['Rolls-Royce AE 2100D3', 'Lockheed C-130J', '~3.460 kW (4.637 shp)', 'Assiale, elica Dowty a 6 pale'],
      ['Europrop TP400-D6', 'Airbus A400M', '~8.200 kW (11.000 shp)', '3 alberi, eliche a 8 pale controrotanti a coppie'],
    ],
    note: 'Il modello segue lo schema PW100 (PW127/PW150): turbina libera e riduttore ad assi sfalsati.',
  },

  thumb: { orbit: [-38, 16], zoom: 1.15, view: 'section' },

  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      parts: ['intake', 'nacelle'],
      base:
        "L'aria entra da una bocca a forma di \"mento\" sotto l'ogiva, subito dietro l'elica. Un condotto curvo la porta verso l'alto, in una camera ad anello " +
        "che la distribuisce tutt'intorno all'ingresso del compressore. È spostata in basso perché davanti, sull'asse del motore, c'è il riduttore.",
      expert:
        "Presa d'aria \"a mento\" (chin intake) con condotto a S verso il plenum anulare della cassa d'ingresso, attraversato dall'albero della turbina di potenza " +
        "dentro il tubo di coppia. La posizione è resa possibile dal riduttore ad assi sfalsati: l'asse dell'elica è più alto dell'asse del motore. " +
        "Il labbro è riscaldato o protetto contro il ghiaccio; un separatore inerziale (nei motori che lo hanno) devia corpi estranei e acqua.",
    },
    {
      id: 'compression',
      name: 'Compressione',
      parts: ['lpc', 'hpc'],
      base:
        "Prima alcune file di palette, come in un piccolo motore a getto, poi una girante centrifuga: una specie di ventola a chiocciola che lancia l'aria " +
        "verso l'esterno a velocità altissima. Alla fine l'aria è schiacciata circa 15 volte ed è già molto calda.",
      expert:
        "Compressore di bassa pressione (nel modello 2 stadi assiali, come i 3 del PW150; nel PW127 è una girante centrifuga) e compressore centrifugo di alta pressione " +
        "con diffusore radiale. I due rotori sono su alberi diversi (NL e NH, oltre 25.000–30.000 giri/min). Rapporto di compressione complessivo ≈ 15:1; " +
        "una girante centrifuga dà da sola ≈ 4–8:1 ed è robusta e compatta, ideale per portate d'aria piccole.",
    },
    {
      id: 'combustion',
      name: 'Combustione',
      parts: ['combustor', 'accessories'],
      base:
        "L'aria compressa scorre all'indietro attorno alla camera di combustione, entra dal fondo e ci brucia insieme al cherosene mentre torna in avanti; " +
        "poi fa un'inversione a U e va verso le turbine. Questo \"giro dell'oca\" serve a rendere il motore corto e leggero.",
      expert:
        "Combustore anulare a <b>flusso invertito</b>: gli iniettori sono sulla cupola posteriore, i gas scorrono in avanti e girano di 180° nel condotto d'uscita " +
        "verso il distributore HP. Il combustore avvolge le turbine HP/LP, accorciando gli alberi. Combustione a pressione quasi costante (ciclo di Brayton), " +
        "temperatura all'ingresso turbina ≈ 1.100–1.300 °C. Più manetta = più carburante, dosato dall'unità di controllo elettronica (EEC).",
    },
    {
      id: 'turbines',
      name: 'Turbine (generatore di gas + turbina libera)',
      parts: ['ggTurbine', 'powerTurbine', 'shafts', 'exhaust'],
      base:
        "I gas caldi fanno girare prima due piccole turbine che servono solo a muovere i compressori: insieme formano il \"generatore di gas\". " +
        "Poi incontrano la <b>turbina libera</b>, che non è collegata ai compressori: è la turbina che lavora per l'elica, tramite un albero che passa al centro del motore. " +
        "Infine i gas, ormai stanchi, escono da un condotto curvo.",
      expert:
        "Turbina HP (1 stadio) → compressore HP; turbina LP (1 stadio) → compressore LP; turbina di potenza a 2 stadi → albero coassiale interno fino al riduttore. " +
        "Tre alberi meccanicamente indipendenti: il generatore di gas varia la velocità con la potenza, mentre la turbina libera gira a velocità quasi costante imposta " +
        "dall'elica. All'uscita i gas hanno ancora ≈ 500–600 °C ma poca energia: il getto dà solo il 5–10% della spinta.",
    },
    {
      id: 'propeller',
      name: 'Riduttore ed elica',
      parts: ['gearbox', 'propeller', 'pitchControl'],
      base:
        "La turbina libera gira a quasi 20.000 giri al minuto, troppo per un'elica. Il riduttore, con ingranaggi sempre più grandi, rallenta la rotazione di circa 15 volte " +
        "e la rende molto più \"forte\". L'elica gira quindi quasi sempre a circa 1.200 giri/min: per avere più potenza le pale si girano di più contro l'aria " +
        "(passo più lungo), come una bicicletta in cui cambi rapporto invece di pedalare più veloce.",
      expert:
        "Riduttore a 2 stadi ad assi sfalsati: pignone d'ingresso → due alberi intermedi (ripartizione del carico) → ruota d'uscita, rapporto ≈ 15:1 (modello 14,9:1). " +
        "Elica a velocità costante: il regolatore (PVM/PEC) mantiene Np ≈ 100% (1.200 giri/min) variando il passo con un attuatore idraulico a giogo nel mozzo; " +
        "più potenza → passo maggiore → più spinta a giri costanti. Il passo può andare in bandiera (≈ 80–90°) a motore spento e in negativo (\"beta\") per frenare a terra.",
    },
  ],

  quiz: [
    {
      q: 'Come fa un turboelica a dare più spinta mentre l\'elica gira quasi agli stessi giri?',
      options: [
        'Allunga le pale con un meccanismo telescopico',
        'Cambia il rapporto del riduttore',
        'Aumenta il passo: le pale ruotano su se stesse e "mordono" più aria, mentre arriva più carburante',
        'Fa girare l\'elica al contrario',
      ],
      correct: 2,
      explain:
        'Con l\'elica a velocità costante, il regolatore aumenta l\'angolo delle pale (passo) quando il motore dà più potenza: ogni giro sposta più aria e la spinta cresce senza cambiare i giri.',
    },
    {
      q: 'A cosa serve il riduttore tra turbina libera ed elica?',
      options: [
        'A rallentare la rotazione di circa 15 volte e aumentare la coppia, perché la turbina gira a ~20.000 giri/min',
        'A far girare il compressore',
        'A cambiare il passo delle pale',
        'A raffreddare l\'olio del motore',
      ],
      correct: 0,
      explain:
        'Un\'elica di quasi 4 metri a 20.000 giri/min avrebbe le punte ben oltre la velocità del suono. Il riduttore porta i giri a ~1.200 e moltiplica la coppia.',
    },
    {
      q: 'Cosa significa che la turbina di potenza è "libera"?',
      options: [
        'Che può essere smontata senza attrezzi',
        'Che non ha palette',
        'Che gira solo a motore spento',
        'Che non è collegata ai compressori: è mossa solo dai gas e aziona soltanto l\'elica',
      ],
      correct: 3,
      explain:
        'Il generatore di gas (compressori + loro turbine) e la turbina libera sono su alberi separati: l\'elica può girare a velocità costante mentre il generatore di gas accelera o rallenta.',
    },
    {
      q: 'Perché gli aerei regionali come l\'ATR 72 usano turboeliche e non turbofan?',
      options: [
        'Perché i turboelica sono più veloci',
        'Perché sulle tratte brevi, sotto i ~650 km/h, l\'elica è molto efficiente e il consumo è fino al 30–40% più basso',
        'Perché non hanno bisogno di carburante',
        'Perché fanno meno rumore in cabina',
      ],
      correct: 1,
      explain:
        'Un\'elica accelera di poco una grande massa d\'aria: alle velocità moderate è il modo più efficiente di produrre spinta. Più veloce, le punte delle pale si avvicinano al suono e l\'efficienza crolla.',
    },
  ],

  // Officina (see CONTRACT.md): modular teardown from the outside in, along the engine axis.
  // The centrifugal compressor with its case stays as the base. Simplified and educational.
  disassembly: {
    intro: "Smontiamo il turboelica dall'esterno verso l'interno, un modulo alla volta, fino al compressore centrifugo.",
    note:
      "Sequenza semplificata a scopo didattico: su un motore vero si segue il manuale di manutenzione del costruttore, con attrezzature dedicate, " +
      "e ci lavorano solo tecnici abilitati.",
    parts: {
      nacelle: { dir: [0, 1, 0], dist: 1.8, duration: 1.2 },
      spinner: { dir: [-1, 0, 0], dist: 1.0 },
      pitchControl: { after: ['nacelle', 'spinner'], dir: [-1, 0.25, 0], dist: 1.0 },
      propBolts: { after: ['spinner', 'pitchControl'], motion: 'unscrew', turns: 6, out: 0.09, dir: [-1, 0, 0], dist: 0.4 },
      propeller: { after: ['propBolts', 'pitchControl'], dir: [-1, 0, 0], dist: 1.4, duration: 1.2 },
      accessories: { after: ['nacelle'], dir: [0, -0.25, -1], dist: 1.0 },
      gearboxBolts: { after: ['nacelle', 'accessories'], motion: 'unscrew', turns: 5, out: 0.05, dir: [-1, 0, 0], dist: 0.25 },
      gearbox: { after: ['gearboxBolts', 'propeller', 'pitchControl'], dir: [-1, 0, 0], dist: 1.4, duration: 1.2 },
      intake: { after: ['gearbox', 'accessories'], dir: [-1, 0, 0], dist: 1.4, duration: 1.1 },
      lpc: { after: ['intake'], dir: [-1, 0, 0], dist: 1.2 },
      exhaust: { after: ['nacelle'], dir: [1, 0.15, 0], dist: 1.3 },
      powerTurbine: { after: ['exhaust'], dir: [1, 0, 0], dist: 1.2 },
      caseBolts: { after: ['nacelle'], motion: 'unscrew', turns: 5, out: 0.06, dir: [-1, 0, 0], dist: 0.3, duration: 2 },
      combustor: { after: ['caseBolts', 'powerTurbine', 'accessories'], dir: [1, 0, 0], dist: 1.5, duration: 1.2 },
      ggTurbine: { after: ['combustor'], dir: [1, 0, 0], dist: 1.2 },
      shafts: { after: ['lpc', 'ggTurbine', 'powerTurbine'], dir: [1, 0, 0], dist: 1.8, duration: 1.2 },
    },
    steps: [
      {
        id: 'cowls',
        name: 'Apri le cofanature',
        base: "Si sganciano i ganci a leva e si tolgono le cofanature della gondola: sotto compare tutto il motore, dal riduttore allo scarico.",
        expert:
          "Sull'aereo le cofanature sono sportelli incernierati che si aprono per le ispezioni; qui le togliamo del tutto, insieme al traliccio degli attacchi, " +
          "come se il motore fosse su un cavalletto d'officina.",
        parts: ['nacelle'],
      },
      {
        id: 'spinner',
        name: "Scopri il mozzo dell'elica",
        base: "Si sfila in avanti l'ogiva, poi si stacca il meccanismo che cambia il passo delle pale, insieme al suo regolatore.",
        expert:
          "L'ogiva è solo una carenatura: il suo piatto posteriore resta sul mozzo. L'attuatore del passo è idraulico e riceve l'olio dal regolatore " +
          "attraverso un tubo che passa dentro l'albero dell'elica.",
        parts: ['spinner', 'pitchControl'],
      },
      {
        id: 'prop',
        name: "Smonta l'elica",
        base: "Con la chiave si svitano i bulloni del mozzo, poi l'elica si sfila in avanti dall'albero del riduttore.",
        expert:
          "Il mozzo è imbullonato alla flangia dell'albero dell'elica, che riceve tutta la coppia e la spinta. Un'elica di quasi 4 metri pesa ben più di cento chili: " +
          "nella realtà la si sostiene con un'imbracatura mentre si tolgono i bulloni.",
        parts: ['propBolts', 'propeller'],
      },
      {
        id: 'accessories',
        name: 'Stacca accessori e impianti',
        base: "Si scollegano tubi e cavi e si toglie la scatola accessori con le pompe, l'avviatore e il serbatoio dell'olio.",
        expert:
          "Tubazioni del carburante e dell'olio e cablaggi elettrici scavalcano i moduli del motore: vanno staccati prima di separarli. " +
          "Le aperture si chiudono con tappi, perché nei circuiti non deve entrare sporco.",
        parts: ['accessories'],
      },
      {
        id: 'gearbox',
        name: 'Togli il riduttore',
        base: "Si svita la corona di bulloni della flangia e il riduttore si sfila in avanti: il suo pignone esce dall'albero della turbina libera.",
        expert:
          "Il riduttore è un modulo a sé: si separa dal resto del motore alla flangia posteriore, dove l'albero di potenza si innesta nel pignone d'ingresso " +
          "con un accoppiamento scanalato. Così può essere revisionato senza aprire il generatore di gas.",
        parts: ['gearboxBolts', 'gearbox'],
      },
      {
        id: 'front',
        name: "Sfila presa d'aria e compressore LP",
        base: "Tolto il riduttore, la presa d'aria con la sua cassa ad anello esce in avanti lungo l'albero; dopo di lei si sfila il compressore di bassa pressione.",
        expert:
          "La cassa d'ingresso è attraversata al centro dall'albero di potenza: per questo può uscire solo lungo l'asse. Poi tocca al compressore LP, " +
          "con il rotore, gli statori e la sua cassa.",
        parts: ['intake', 'lpc'],
      },
      {
        id: 'rear',
        name: 'Togli scarico e turbina libera',
        base: "Dalla parte opposta si stacca il condotto di scarico: dietro appare la turbina libera, che si sfila all'indietro dal suo albero.",
        expert:
          "La turbina di potenza non ha legami meccanici con il generatore di gas: è collegata soltanto all'albero più interno. " +
          "Per questo forma un modulo separato, che esce da dietro senza smontare compressori e turbine HP e LP.",
        parts: ['exhaust', 'powerTurbine'],
      },
      {
        id: 'combustor',
        name: 'Sfila la camera di combustione',
        base: "Si svitano i bulloni della flangia più grande e la cassa con la camera di combustione scorre all'indietro, scoprendo le turbine che avvolgeva.",
        expert:
          "La cassa esterna è un recipiente in pressione: contiene l'aria che esce dal compressore, alla pressione più alta di tutto il motore. " +
          "Dentro porta il tubo di fiamma anulare con gli iniettori; essendo a flusso invertito, avvolge le turbine del generatore di gas.",
        parts: ['caseBolts', 'combustor'],
      },
      {
        id: 'core',
        name: 'Sfila turbine e alberi',
        base:
          "Restano le due turbine del generatore di gas e i tre alberi infilati uno nell'altro: escono all'indietro. " +
          "Sul banco rimane il compressore centrifugo con la sua cassa, il cuore del motore.",
        expert:
          "Ogni turbina è calettata sull'albero del suo compressore: HP all'esterno, LP in mezzo, potenza al centro. " +
          "I rotori sono equilibrati con grande precisione, perciò al rimontaggio ogni elemento deve tornare nella posizione di prima.",
        parts: ['ggTurbine', 'shafts'],
      },
    ],
  },

  camera: { position: [-4.7, 2.2, 6.1], target: [0, -0.1, 0] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.x = OX;
    root.add(model);
    const core = new THREE.Group(); // engine-axis frame
    core.position.y = YE;
    model.add(core);

    // ---------- Materials ----------
    const mCase = cut(MAT.steel());
    const mCaseDark = cut(MAT.darkSteel());
    const mInlet = cut(MAT.paint(0xc9cdd2, 0.45));
    const mRgb = cut(MAT.paint(0x8f979f, 0.5));
    const mStator = cut(MAT.darkSteel());
    const mSpinner = cut(MAT.paint(0x3a3f46, 0.32));
    const mBlade = MAT.paint(0x2b2e33, 0.55);
    const mBladeTip = MAT.paint(0xd8dce0, 0.4);
    const mCuff = MAT.carbon();
    const mHub = MAT.steel();
    const mTi = MAT.titanium();
    const mNi = MAT.steel();
    const mDrum = ds(MAT.darkSteel());
    const mGear = MAT.steel();
    const mGear2 = MAT.darkSteel();
    const mShaft = ds(MAT.chrome());
    const mShaft2 = ds(MAT.steel());
    const mHot = MAT.darkSteel();
    mHot.color.set(0x9a8c7c);
    mHot.emissive.set(0xff3a0a);
    mHot.emissiveIntensity = 0;
    const mHotStator = cut(MAT.darkSteel());
    mHotStator.color.set(0x8a7d70);
    mHotStator.emissive.set(0xff3a0a);
    mHotStator.emissiveIntensity = 0;
    const mPt = MAT.steel();
    mPt.color.set(0xa9a49c);
    const mLiner = cut(MAT.darkSteel());
    mLiner.color.set(0x9c8c7a);
    const mExhaust = cut(MAT.darkSteel());
    mExhaust.color.set(0x8a8178);
    const mFuel = MAT.copper();
    const mInjector = MAT.brass();
    const mActuator = MAT.aluminum();
    const mLink = MAT.chrome();
    const mBolt = cut(MAT.darkSteel(), { cap: false });
    const mGasket = cut(MAT.rubber());
    const mNacelle = cut(MAT.paint(0xd3d8dd, 0.42));
    const mNacelleDark = cut(MAT.paint(0x2a2e33, 0.6));
    const mLatch = cut(MAT.darkSteel());
    const mHole = cut(MAT.paint(0x1b1612, 0.9), { cap: false });
    const mPipe = cut(MAT.copper());
    const mHarness = cut(MAT.paint(0x3a3226, 0.8));
    const mAcc = cut(MAT.aluminum());
    const mAccDark = cut(MAT.castIron());

    const lpRot = [];
    const hpRot = [];
    const ptRot = [];
    const hotMeshes = [];

    // ---------- Propeller (6 variable-pitch blades) ----------
    const propPart = new THREE.Group();
    const propSpin = new THREE.Group();
    propSpin.position.set(XB, YP, 0);
    propPart.add(propSpin);

    const spinPts = [
      [0, -0.42], [0.06, -0.4], [0.13, -0.33], [0.19, -0.23], [0.232, -0.1], [0.25, 0.04], [0.252, 0.2],
    ];
    // spinner dome: its own part (it comes off first); the backplate stays on the hub
    const spinnerPart = new THREE.Group();
    const spinnerSpin = new THREE.Group();
    spinnerSpin.position.set(XB, YP, 0);
    spinnerPart.add(spinnerSpin);
    spinnerSpin.add(new THREE.Mesh(wall(spinPts, -0.01, 64), mSpinner));
    {
      const heads = boltSet(mBolt, 0.008, 0.004);
      for (let i = 0; i < 12; i++) {
        const a = ((i + 0.5) / 12) * TAU;
        heads.add(new THREE.Vector3(0.17, 0.2515 * Math.cos(a), 0.2515 * Math.sin(a)), new THREE.Vector3(0, Math.cos(a), Math.sin(a)));
      }
      spinnerSpin.add(heads.mesh());
    }
    propSpin.add(new THREE.Mesh(flangeX(0.195, 0.07, 0.241, 0.01, 64), mSpinner));
    propSpin.add(new THREE.Mesh(latheX([[0.05, -0.07], [0.1, -0.07], [0.11, -0.04], [0.11, 0.1], [0.07, 0.13], [0.05, 0.13]], 40), mHub));

    const BLADE_L = 1.38;
    const BLADE_R0 = 0.28;
    const TWIST = 0.55;
    const plan = (u) => (0.62 + 0.8 * u - 0.75 * u * u) * (u > 0.88 ? Math.sqrt(Math.max(0.06, 1 - ((u - 0.88) / 0.12) ** 2)) : 1);
    const bladeGeo = airfoilGeo(BLADE_L, 0.3, 0.06, {
      segL: 18,
      plan,
      thk: (u) => 1 - 0.72 * u,
      twist: (u) => TWIST * (0.75 - u),
      map: (r, c, t) => [c, r, t],
    });
    const tipGeo = airfoilGeo(0.16, 0.3, 0.06, {
      segL: 3,
      plan: (u) => plan(0.884 + u * 0.116),
      thk: (u) => 1 - 0.72 * (0.884 + u * 0.116),
      twist: (u) => TWIST * (0.75 - (0.884 + u * 0.116)),
      map: (r, c, t) => [c * 1.01, r, t * 1.15],
    });
    const cuffGeo = new THREE.CylinderGeometry(0.048, 0.056, 0.2, 18).translate(0, 0.18, 0);
    const collarGeo = new THREE.CylinderGeometry(0.066, 0.066, 0.025, 20).translate(0, 0.24, 0);
    const ARM = new THREE.Vector3(0.078, 0.17, 0.045); // crank pin, pitch-local
    const armGeo = new THREE.BoxGeometry(0.09, 0.018, 0.022).rotateY(-30 * DEG).translate(0.039, 0.17, 0.0225);
    const pinGeo = new THREE.SphereGeometry(0.016, 10, 8).translate(ARM.x, ARM.y, ARM.z);
    const pitchGroups = [];
    const NB = 6;
    for (let i = 0; i < NB; i++) {
      const az = new THREE.Group();
      az.rotation.x = (i / NB) * TAU;
      const pg = new THREE.Group();
      az.add(pg);
      const bl = new THREE.Mesh(bladeGeo, mBlade);
      bl.position.y = BLADE_R0;
      const tip = new THREE.Mesh(tipGeo, mBladeTip);
      tip.position.y = BLADE_R0 + BLADE_L * 0.884;
      pg.add(bl, tip, new THREE.Mesh(cuffGeo, mCuff), new THREE.Mesh(collarGeo, mHub));
      pg.add(new THREE.Mesh(armGeo, mHub), new THREE.Mesh(pinGeo, mLink));
      propSpin.add(az);
      pitchGroups.push(pg);
    }
    model.add(
      ctx.part(propPart, {
        id: 'propeller',
        name: 'Elica a passo variabile',
        base:
          "Sei pale in materiale composito montate su un mozzo. Ogni pala può <b>ruotare attorno al proprio asse</b>: più è girata, più aria \"morde\" a ogni giro. " +
          "Così l'elica può girare sempre alla stessa velocità e dare più o meno spinta. A motore spento le pale si mettono di taglio al vento (in bandiera) per non frenare l'aereo.",
        expert:
          "Elica Hamilton Sundstrand 568F dell'ATR 72: 6 pale in composito, Ø 3,93 m, velocità costante 1.200 giri/min al decollo (Np 100%), ~82% in crociera. " +
          "Il passo misurato al 75% del raggio va da pochi gradi (minimo in volo) a ~40° e oltre ad alta velocità, fino alla bandiera (~85°) e al passo negativo (\"beta\" e reverse a terra). " +
          "Le pale sono svergolate (più angolo alla radice che alla punta) perché la punta si muove più veloce. Fornisce ~90% della spinta.",
        explode: [-1.5, 0, 0],
      }),
      ctx.part(spinnerPart, {
        id: 'spinner',
        name: 'Ogiva',
        base:
          "La punta affusolata davanti all'elica. Copre il mozzo e il meccanismo del passo e accompagna l'aria in modo regolare verso le radici delle pale " +
          "e la presa d'aria che sta subito dietro.",
        expert:
          "Carenatura sottile fissata con una corona di viti a un piatto posteriore (bulkhead) solidale al mozzo: gira con l'elica. Non porta carichi, " +
          "ma deve essere centrata ed equilibrata con cura, perché ruota a ~1.200 giri/min. Riduce la resistenza della zona del mozzo, dove le pale hanno sezione tozza e spingono poco.",
        explode: [-1.85, 0, 0],
      }),
    );

    // hub bolts: hub front face to the propeller-shaft flange (rotate with the hub)
    const propBoltPart = new THREE.Group();
    const propBoltSpin = new THREE.Group();
    propBoltSpin.position.set(XB, YP, 0);
    propBoltPart.add(propBoltSpin);
    propBoltSpin.add(boltSet(mBolt, 0.012, 0.014, screwGeo(0.012, 0.014, 0.08)).ringX(-0.07, 0.078, 12, -1, 0, 0, TAU / 24).mesh());
    model.add(
      ctx.part(propBoltPart, {
        id: 'propBolts',
        name: "Bulloni del mozzo dell'elica",
        base:
          "Una corona di bulloni che stringe il mozzo dell'elica alla flangia dell'albero che esce dal riduttore. Tutta la forza dell'elica passa da qui.",
        expert:
          "Il giunto flangiato trasmette all'albero la coppia (oltre 15 kNm al decollo), la spinta e i momenti flettenti dell'elica. I bulloni lavorano a trazione: " +
          "è il loro precarico a tenere unite le due flange, mentre nei giunti di questo tipo la coppia passa per attrito e attraverso spine di trascinamento.",
        explode: [-1.5, 0, 0],
      }),
    );

    // ---------- Pitch-change mechanism (rotates with the hub) + governor ----------
    const pitchPart = new THREE.Group();
    const pitchSpin = new THREE.Group();
    pitchSpin.position.set(XB, YP, 0);
    pitchPart.add(pitchSpin);
    pitchSpin.add(new THREE.Mesh(latheX([[0, -0.36], [0.05, -0.355], [0.072, -0.33], [0.075, -0.2], [0.06, -0.18], [0.02, -0.18]], 32), mActuator));
    const yoke = new THREE.Group();
    pitchSpin.add(yoke);
    const ARM_Z = 0.088;
    const LINK = 0.12;
    {
      const list = [new THREE.CylinderGeometry(0.018, 0.018, 0.12, 10).rotateZ(Math.PI / 2).translate(-0.06, 0, 0)];
      list.push(new THREE.CylinderGeometry(0.06, 0.06, 0.03, 6).rotateZ(Math.PI / 2));
      for (let i = 0; i < NB; i++) {
        const a = (i / NB) * TAU;
        const end = new THREE.Vector3(0, 0.17 * Math.cos(a) - ARM_Z * Math.sin(a), 0.17 * Math.sin(a) + ARM_Z * Math.cos(a));
        list.push(cylBetweenGeo(new THREE.Vector3(0, 0, 0), end, 0.012, 6));
      }
      yoke.add(new THREE.Mesh(mergeGeos(list), mActuator));
    }
    const links = [];
    const linkGeo = new THREE.CylinderGeometry(0.008, 0.008, 1, 6);
    for (let i = 0; i < NB; i++) {
      const l = new THREE.Mesh(linkGeo, mLink);
      pitchSpin.add(l);
      links.push(l);
    }
    // propeller governor / pitch control unit on top of the gearbox (static)
    const governor = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.07, 0.13, 20), mActuator);
    governor.position.set(-1.62, YP + 0.48, 0);
    const govCap = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.05, 0.12), mActuator);
    govCap.position.set(-1.62, YP + 0.55, 0);
    pitchPart.add(governor, govCap);
    model.add(
      ctx.part(pitchPart, {
        id: 'pitchControl',
        name: 'Meccanismo del passo e regolatore',
        base:
          "Dentro l'ogiva c'è un pistone idraulico che sposta avanti e indietro un \"giogo\" a sei bracci. Ogni braccio tira o spinge un perno sulla radice di una pala " +
          "e la fa ruotare. Sopra il riduttore un regolatore controlla i giri: se l'elica accelera, aumenta il passo; se rallenta, lo riduce.",
        expert:
          "Attuatore idraulico a doppio effetto nel mozzo alimentato dall'olio tramite il tubo di trasferimento (beta tube) che passa dentro l'albero dell'elica. " +
          "Il giogo trasforma la corsa assiale del pistone in rotazione delle pale attraverso perni eccentrici (trunnion). Il regolatore (PVM con controllo elettronico PEC " +
          "sull'ATR) confronta Np con il valore scelto e dosa l'olio: sistema a velocità costante. Con perdita di pressione le pale vanno in bandiera grazie ai contrappesi e alle molle.",
        explode: [-0.85, 0.35, 0],
      }),
    );

    // ---------- Offset reduction gearbox ----------
    const rgb = new THREE.Group();
    {
      // hollow casting: side wall + front/rear covers (shape = hull of the gear envelopes)
      const outline = (off) => {
        const pts = [];
        for (const [s, t, r] of [[0, YP, 0.4], [LAY_Z, YL, 0.27], [-LAY_Z, YL, 0.27], [0, YE, 0.14]]) {
          for (let i = 0; i < 48; i++) pts.push([s + (r + off) * Math.cos((i / 48) * TAU), t + (r + off) * Math.sin((i / 48) * TAU)]);
        }
        return hull(pts).map(([s, t]) => new THREE.Vector2(s, t));
      };
      const toX = (g, xFront) => g.rotateY(-Math.PI / 2).translate(xFront, 0, 0); // depth d -> x = xFront - d
      const plate = (holes, depth, xBack) => {
        const sh = new THREE.Shape(outline(0));
        for (const [s, t, r] of holes) {
          const h = new THREE.Path();
          h.absarc(s, t, r, 0, TAU, true);
          sh.holes.push(h);
        }
        return toX(new THREE.ExtrudeGeometry(sh, { depth, bevelEnabled: false, curveSegments: 32 }), xBack);
      };
      const side = new THREE.Shape(outline(0));
      side.holes.push(new THREE.Path(outline(-0.03).reverse()));
      rgb.add(new THREE.Mesh(toX(new THREE.ExtrudeGeometry(side, { depth: 0.31, bevelEnabled: false }), -1.578), mRgb));
      rgb.add(new THREE.Mesh(plate([[0, YP, 0.09]], 0.035, -1.888), mRgb));
      rgb.add(new THREE.Mesh(plate([[0, YE, 0.095]], 0.035, -1.543), mRgb));
      for (const xb of [-1.886, -1.576]) {
        const g = new THREE.Shape(outline(0.005));
        g.holes.push(new THREE.Path(outline(-0.02).reverse()));
        rgb.add(new THREE.Mesh(toX(new THREE.ExtrudeGeometry(g, { depth: 0.005, bevelEnabled: false }), xb), mGasket));
      }
      // cover bolts along the perimeter
      const bolts = boltSet(mBolt, 0.011, 0.012);
      const ring = outline(-0.02);
      let acc = 0;
      for (let i = 0; i < ring.length; i++) {
        const p0 = ring[i];
        const p1 = ring[(i + 1) % ring.length];
        const L = p0.distanceTo(p1);
        while (acc < L) {
          const u = acc / L;
          const s0 = lerp(p0.x, p1.x, u);
          const t0 = lerp(p0.y, p1.y, u);
          bolts.add(new THREE.Vector3(-1.923, t0, s0), new THREE.Vector3(-1, 0, 0));
          bolts.add(new THREE.Vector3(-1.543, t0, s0), new THREE.Vector3(1, 0, 0));
          acc += 0.085;
        }
        acc -= L;
      }
      bolts.ringX(-1.94, 0.135, 10, -1, YP, 0, 0.3);
      rgb.add(bolts.mesh());
      // lifting eye, nose boss and rear torque tube
      const eye = new THREE.Mesh(new THREE.TorusGeometry(0.035, 0.011, 8, 20), mRgb);
      eye.position.set(-1.8, YP + 0.465, 0);
      const eyeBase = new THREE.Mesh(new RoundedBoxGeometry(0.1, 0.04, 0.05, 2, 0.01), mRgb);
      eyeBase.position.set(-1.8, YP + 0.42, 0);
      const boss = new THREE.Mesh(wall([[0.17, -1.92], [0.15, -1.99], [0.1, -2.02]], 0.012, 48), mRgb);
      boss.position.y = YP;
      const tube = new THREE.Mesh(wall([[0.15, -1.54], [0.09, -1.46], [0.085, -1.2], [0.1, -1.12]], 0.01, 40), mRgb);
      tube.position.y = YE;
      const tubeFlange = new THREE.Mesh(flangeX(-1.143, 0.095, 0.165, 0.016, 48), mRgb); // bolted to the inlet case
      tubeFlange.position.y = YE;
      rgb.add(eye, eyeBase, boss, tube, tubeFlange);
    }
    // gears (axis X); phase convention: Θ = -rotation.x
    const angTo = (y0, z0, y1, z1) => Math.atan2(y1 - y0, z1 - z0);
    const th = {};
    th.in = 0;
    th.layP = meshPhase(th.in, N_IN, N_LAY, angTo(YE, 0, YL, LAY_Z));
    th.layM = meshPhase(th.in, N_IN, N_LAY, angTo(YE, 0, YL, -LAY_Z));
    th.lpP = 0;
    th.out = meshPhase(th.lpP, N_LAYP, N_OUT, angTo(YL, LAY_Z, YP, 0));
    th.lpM = meshPhase(th.out, N_OUT, N_LAYP, angTo(YP, 0, YL, -LAY_Z));

    const gIn = new THREE.Mesh(spurX(N_IN, N_IN * GM, 0.07), mGear);
    gIn.position.set(X_G1, YE, 0);
    const gLay = [];
    for (const [z, key, keyP] of [[LAY_Z, 'layP', 'lpP'], [-LAY_Z, 'layM', 'lpM']]) {
      const shaft = new THREE.Group();
      shaft.position.set(0, YL, z);
      const big = new THREE.Mesh(spurX(N_LAY, N_LAY * GM, 0.05, 0.03), mGear);
      big.position.x = X_G1;
      const small = new THREE.Mesh(spurX(N_LAYP, N_LAYP * GM, 0.07), mGear2);
      small.position.x = X_G2;
      const axle = new THREE.Mesh(new THREE.CylinderGeometry(0.028, 0.028, 0.34, 16).rotateZ(Math.PI / 2), mShaft2);
      axle.position.x = (X_G1 + X_G2) / 2;
      const web = new THREE.Mesh(latheX([[0.03, X_G1 - 0.012], [0.19, X_G1 - 0.012], [0.19, X_G1 + 0.012], [0.03, X_G1 + 0.012]], 48), mGear2);
      shaft.add(big, small, axle, web);
      rgb.add(shaft);
      gLay.push({ big, small, th0: th[key], thp0: th[keyP] });
    }
    const outGroup = new THREE.Group();
    outGroup.position.set(X_G2, YP, 0);
    const gOut = new THREE.Mesh(spurX(N_OUT, N_OUT * GM, 0.06, 0.05), mGear);
    const outWeb = new THREE.Mesh(latheX([[0.06, -0.014], [0.29, -0.014], [0.29, 0.014], [0.06, 0.014]], 64), mGear2);
    const propShaft = new THREE.Mesh(latheX([[0.05, -0.3], [0.068, -0.3], [0.068, 0.08], [0.05, 0.08]], 32), mShaft);
    outGroup.add(gOut, outWeb, propShaft);
    rgb.add(gIn, outGroup);
    model.add(
      ctx.part(rgb, {
        id: 'gearbox',
        name: 'Riduttore ad assi sfalsati',
        base:
          "Una grande scatola di ingranaggi davanti al motore. Un ingranaggio piccolo, sull'albero della turbina libera, ne fa girare due grandi ai lati; " +
          "questi, con altri pignoni, fanno girare l'ingranaggio enorme dell'elica. Risultato: l'elica gira circa 15 volte più piano della turbina ma con molta più forza. " +
          "L'elica sta più in alto del motore, così sotto c'è posto per la presa d'aria.",
        expert:
          "Riduttore a due stadi con due alberi intermedi (twin layshaft): il carico si divide su due percorsi, riducendo dimensioni e peso degli ingranaggi. " +
          "Stadi da ≈ 3,4:1 e ≈ 4,4:1 nel modello (13/44 e 15/66 denti), totale ≈ 15:1; nei PW100 reali ≈ 15–17:1 per passare da ~20.000 giri/min a ~1.200. " +
          "Trasmette oltre 15 kNm di coppia all'albero dell'elica, su cui agiscono anche la spinta e i momenti giroscopici. Ospita il torsiometro e la pompa del regolatore.",
        explode: [-0.3, 0.95, 0],
      }),
      ctx.part(boltSet(mBolt, 0.013, 0.013, screwGeo(0.013, 0.013, 0.03)).ringX(-1.151, 0.135, 12, -1, YE).mesh(), {
        id: 'gearboxBolts',
        name: 'Bulloni della flangia del riduttore',
        base:
          "I bulloni che uniscono il riduttore al resto del motore, tutt'intorno al tubo in cui passa l'albero della turbina libera.",
        expert:
          "Flangia di accoppiamento tra il modulo riduttore e la cassa d'ingresso. Il giunto deve tenere perfettamente allineati l'albero di potenza e il pignone d'ingresso " +
          "e regge parte dei carichi dell'elica: spinta, coppia di reazione e momenti giroscopici.",
        explode: [-0.3, 0.95, 0],
      }),
    );

    // ---------- Intake: chin duct, inlet plenum ----------
    const intake = new THREE.Group();
    const duct = [
      { p: [-1.93, YE - 0.48], hw: 0.27, hh: 0.14, e: 3.2 },
      { p: [-1.55, YE - 0.5], hw: 0.27, hh: 0.15, e: 3.2 },
      { p: [-1.24, YE - 0.52], hw: 0.25, hh: 0.14, e: 3 },
      { p: [-1.08, YE - 0.47], hw: 0.23, hh: 0.13, e: 2.8 },
      { p: [-0.99, YE - 0.38], hw: 0.22, hh: 0.12, e: 2.6 },
    ];
    intake.add(new THREE.Mesh(loftWall(duct, 0.014, 30, 44), mInlet));
    intake.add(new THREE.Mesh(lipGeo({ ...duct[0], hw: duct[0].hw - 0.007, hh: duct[0].hh - 0.007 }, 0.016), cut(MAT.aluminum())));
    {
      const plenum = new THREE.Group();
      plenum.position.y = YE;
      const gap = 1.3;
      plenum.add(new THREE.Mesh(wall([[0.38, -1.12], [0.39, -1.0], [0.38, -0.92], [0.35, -0.875], [0.33, -0.86]], 0.012, 72, Math.PI / 2 + gap / 2, TAU - gap), mInlet));
      plenum.add(new THREE.Mesh(latheX([[0.1, -1.135], [0.392, -1.135], [0.392, -1.12], [0.1, -1.12], [0.1, -1.135]], 72), mInlet));
      plenum.add(new THREE.Mesh(wall([[0.1, -1.12], [0.105, -1.02], [0.12, -0.93], [0.13, -0.875]], -0.01, 48), mCaseDark));
      plenum.add(statorRow(-0.99, 0.11, 0.38, 6, 0.09, mStator, 0, 0.018).rotateX(Math.PI / 6));
      intake.add(plenum);
    }
    model.add(
      ctx.part(intake, {
        id: 'intake',
        name: 'Presa d\'aria e cassa d\'ingresso',
        base:
          "La bocca sotto l'ogiva e il condotto curvo che porta l'aria al motore. Prima del compressore l'aria entra in una camera ad anello, " +
          "così arriva uniforme da tutti i lati.",
        expert:
          "Presa \"a mento\" con condotto a S e plenum anulare (cassa d'ingresso) con montanti radiali che reggono il tubo di coppia dell'albero di potenza. " +
          "Il labbro è protetto dal ghiaccio (aria calda o resistenze elettriche); la distorsione del flusso dovuta alla curva è smorzata dal plenum prima dello stadio LP.",
        explode: [0, -0.95, 0],
      }),
    );

    // ---------- LP compressor (axial, LP spool) ----------
    const lpcHub = (x) => lerp(0.13, 0.15, (x + 0.88) / 0.38);
    const lpcTip = (x) => lerp(0.3, 0.27, (x + 0.88) / 0.38);
    const lpc = new THREE.Group();
    const lpcRotor = new THREE.Group();
    lpc.add(lpcRotor);
    lpRot.push(lpcRotor);
    lpcRotor.add(new THREE.Mesh(latheX([[0.06, -0.87], [0.13, -0.87], [0.15, -0.52], [0.13, -0.48], [0.06, -0.48], [0.06, -0.87]], 48), mDrum));
    for (const [x, n] of [[-0.79, 24], [-0.66, 28]]) {
      lpcRotor.add(rotorRow(x, lpcHub(x), lpcTip(x) - 0.004, n, 0.06, mTi, { twist: 0.5, stagger: 0.6, thick: 0.01 }));
    }
    for (const [x, a] of [[-0.845, 0.2], [-0.725, 0.45], [-0.6, 0.45]]) {
      lpc.add(statorRow(x, lpcHub(x) + 0.004, lpcTip(x) + 0.006, 30, 0.045, mStator, a, 0.008));
    }
    lpc.add(new THREE.Mesh(wall([[0.33, -0.86], [0.305, -0.84], [0.3, -0.75], [0.285, -0.6], [0.268, -0.5], [0.252, -0.47]], 0.012, 72), mCase));
    lpc.add(new THREE.Mesh(flangeX(-0.5, 0.27, 0.315, 0.022), mCase));
    lpc.add(boltSet(mBolt, 0.009, 0.01).ringX(-0.513, 0.298, 20, -1).mesh());
    core.add(
      ctx.part(lpc, {
        id: 'lpc',
        name: 'Compressore di bassa pressione',
        base:
          "Le prime file di palette: quelle che girano spingono l'aria, quelle ferme la raddrizzano. Danno una prima spremuta all'aria prima della girante centrifuga.",
        expert:
          "Stadio di bassa pressione sull'albero LP (NL), mosso dalla turbina LP. Nel modello 2 stadi assiali (il PW150 ne ha 3); nel PW127 è invece una girante centrifuga. " +
          "Rapporto di compressione dello stadio LP ≈ 3–4:1. La valvola di spillamento tra i due compressori evita lo stallo ai bassi regimi.",
        explode: [-0.55, 0, 0],
      }),
    );

    // ---------- HP compressor (centrifugal, HP spool) + diffuser ----------
    const hpc = new THREE.Group();
    const hpcRotor = new THREE.Group();
    hpc.add(hpcRotor);
    hpRot.push(hpcRotor);
    const impHub = [[0.12, -0.47], [0.135, -0.42], [0.17, -0.35], [0.24, -0.29], [0.33, -0.255], [0.4, -0.245]];
    const impShroud = [[0.235, -0.47], [0.24, -0.42], [0.27, -0.355], [0.325, -0.3], [0.4, -0.275]];
    hpcRotor.add(new THREE.Mesh(latheX([[0.07, -0.47], ...impHub, [0.402, -0.228], [0.3, -0.215], [0.12, -0.205], [0.07, -0.205], [0.07, -0.47]], 72), mDrum));
    hpcRotor.add(impeller(impHub, impShroud, 15, ds(MAT.titanium()), { wrap: 0.75 }));
    hpcRotor.add(impeller(impHub, impShroud, 15, ds(MAT.titanium()), { wrap: 0.75, from: 0.42, n: 10 }).rotateX(TAU / 30));
    hpc.add(new THREE.Mesh(wall([[0.25, -0.47], [0.255, -0.42], [0.283, -0.355], [0.335, -0.302], [0.41, -0.282], [0.5, -0.282], [0.55, -0.262], [0.568, -0.22], [0.568, -0.2]], 0.012, 72), mCase));
    hpc.add(new THREE.Mesh(wall([[0.41, -0.205], [0.48, -0.205], [0.5, -0.17], [0.505, -0.1]], 0.01, 72), mCase));
    {
      const vane = new THREE.BoxGeometry(0.07, 0.085, 0.008).translate(0, 0.0425, 0).rotateX(-0.75);
      const vanes = mergeRing(vane, mStator, 20, 0.415);
      vanes.position.x = -0.243;
      hpc.add(vanes);
    }
    core.add(
      ctx.part(hpc, {
        id: 'hpc',
        name: 'Compressore centrifugo di alta pressione',
        base:
          "Una girante a forma di campana con palette curve: prende l'aria dal centro e la lancia verso l'esterno, come l'acqua da una centrifuga. " +
          "Poi un anello di palette fisse (diffusore) la rallenta trasformando la velocità in pressione.",
        expert:
          "Girante centrifuga in titanio con pale intere e intercalari (splitter), sull'albero HP (NH ≈ 30.000+ giri/min), mossa dalla turbina HP. " +
          "Velocità periferica ≈ 500 m/s; il diffusore radiale a palette recupera pressione e devia il flusso all'indietro verso l'esterno del combustore. " +
          "Rapporto complessivo LP×HP ≈ 15:1 nel PW127, ≈ 18:1 nel PW150.",
        explode: [-0.2, 0, 0],
      }),
    );

    // ---------- Reverse-flow annular combustor ----------
    const comb = new THREE.Group();
    comb.add(new THREE.Mesh(wall([[0.568, -0.2], [0.568, 0.56], [0.552, 0.64], [0.5, 0.7], [0.42, 0.75], [0.35, 0.78]], 0.012, 72), mCase));
    comb.add(new THREE.Mesh(flangeX(-0.2, 0.575, 0.625, 0.03), mCase));
    comb.add(new THREE.Mesh(flangeX(0.3, 0.578, 0.6, 0.02), mCase));
    // igniter plugs (2) on the case
    for (const a of [200 * DEG, 340 * DEG]) {
      const plug = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.026, 0.09, 12), mLatch);
      plug.position.set(0.42, 0.6 * Math.cos(a), 0.6 * Math.sin(a));
      plug.rotation.x = a;
      comb.add(plug);
    }
    comb.add(
      new THREE.Mesh(
        wall([[0.465, 0.58], [0.465, 0.05], [0.45, -0.02], [0.41, -0.06], [0.34, -0.075], [0.26, -0.07], [0.19, -0.045], [0.15, -0.01], [0.135, 0.03], [0.13, 0.055]], 0.01, 72),
        mLiner,
      ),
    );
    comb.add(new THREE.Mesh(wall([[0.3, 0.58], [0.3, 0.1], [0.295, 0.05], [0.275, 0.022], [0.255, 0.028], [0.248, 0.055]], -0.01, 72), mLiner));
    comb.add(new THREE.Mesh(latheX([[0.3, 0.6], [0.475, 0.6], [0.475, 0.585], [0.3, 0.585], [0.3, 0.6]], 72), mLiner));
    {
      const manifold = new THREE.Mesh(new THREE.TorusGeometry(0.51, 0.012, 6, 72).rotateY(Math.PI / 2), mFuel);
      manifold.position.x = 0.8;
      comb.add(manifold);
      const stem = cylBetweenGeo(new THREE.Vector3(0.79, 0.51, 0), new THREE.Vector3(0.6, 0.385, 0), 0.009, 6);
      comb.add(mergeRing(stem, mInjector, 14, 0));
      const cup = new THREE.CylinderGeometry(0.022, 0.026, 0.03, 10).rotateZ(Math.PI / 2).translate(0.585, 0, 0);
      comb.add(mergeRing(cup, mInjector, 14, 0.38));
    }
    // flame kept thin and translucent so liner, swirlers and dilution holes stay readable
    const flamePts = [
      [0.38, 0.555], [0.405, 0.5], [0.415, 0.4], [0.41, 0.28], [0.395, 0.16], [0.375, 0.09], [0.36, 0.16], [0.35, 0.28], [0.345, 0.4], [0.355, 0.5], [0.38, 0.555],
    ];
    const flameCore = new THREE.Mesh(latheX(flamePts, 72), ds(MAT.glow(0xff7a2a, 1, 0.5)));
    const flameHalo = new THREE.Mesh(latheX(flamePts.map(([r, x]) => [0.38 + (r - 0.38) * 1.5, x]), 72), MAT.gas(0xff5a1f, 0.12));
    // dilution / cooling holes on the liner walls
    comb.add(holesX(mHole, [[0.42, 0.476, 28], [0.3, 0.476, 28, 0.11], [0.18, 0.476, 22], [0.4, 0.301, 22, 0.1], [0.24, 0.301, 22]], 0.012));
    const jetGeo = new THREE.ConeGeometry(0.028, 0.18, 8, 1, true).rotateZ(Math.PI / 2).translate(-0.09, 0, 0);
    const flameJets = mergeRing(jetGeo, MAT.gas(0xffa050, 0.25), 14, 0.38);
    flameJets.position.x = 0.565;
    comb.add(flameCore, flameHalo, flameJets);
    core.add(
      ctx.part(comb, {
        id: 'combustor',
        name: 'Camera di combustione a flusso invertito',
        base:
          "Un anello che avvolge le turbine. L'aria ci entra dal fondo, dove gli iniettori spruzzano il cherosene; la fiamma brucia di continuo mentre i gas tornano in avanti, " +
          "poi i gas fanno un'inversione a U verso le turbine. Più manetta = più carburante = fiamma più intensa.",
        expert:
          "Combustore anulare a flusso invertito con ~14 iniettori ad aria assistita sulla cupola posteriore e accenditori. Il ripiegamento del percorso permette di " +
          "sovrapporre combustore e turbine: alberi corti e rigidi, motore compatto. Rapporto aria/combustibile globale ≈ 50:1, zona primaria vicina allo stechiometrico; " +
          "l'aria di diluizione porta i gas a ≈ 1.100–1.300 °C all'ingresso turbina. Liner raffreddato a film.",
        explode: [0.15, 1.15, 0],
      }),
      // front case flange bolts (combustor case to compressor case), heads facing forward
      ctx.part(boltSet(mBolt, 0.012, 0.012, screwGeo(0.012, 0.012, 0.04)).ringX(-0.215, 0.602, 36, -1).mesh(), {
        id: 'caseBolts',
        name: 'Bulloni della cassa del combustore',
        base:
          "Decine di bulloni, uno accanto all'altro, sulla flangia più grande del motore: tengono unita la cassa della camera di combustione a quella del compressore.",
        expert:
          "Le casse di una turbina a gas sono unite da flange con molti bulloni ravvicinati: la giunzione deve restare a tenuta con l'aria alla pressione di mandata " +
          "del compressore (≈ 15 bar al decollo) e a qualche centinaio di gradi, seguendo le dilatazioni termiche senza deformarsi.",
        explode: [0.15, 1.15, 0],
      }),
    );

    // ---------- Gas-generator turbines (HP + LP, single stage each) ----------
    const ggt = new THREE.Group();
    const hptRotor = new THREE.Group();
    const lptRotor = new THREE.Group();
    ggt.add(hptRotor, lptRotor);
    hpRot.push(hptRotor);
    lpRot.push(lptRotor);
    {
      const hpRow = rotorRow(0.125, 0.135, 0.243, 40, 0.04, mHot, { twist: -0.25, stagger: -0.75, thick: 0.013 });
      hptRotor.add(hpRow, disc(0.125, 0.08, 0.135, 0.05, mDrum));
      hotMeshes.push(hpRow);
      const lpRow = rotorRow(0.29, 0.14, 0.255, 42, 0.042, mHot, { twist: -0.25, stagger: -0.7, thick: 0.012 });
      lptRotor.add(lpRow, disc(0.29, 0.055, 0.14, 0.05, mDrum));
      hotMeshes.push(lpRow);
      for (const [x, rh, rt] of [[0.07, 0.133, 0.247], [0.225, 0.138, 0.258]]) {
        const ngv = statorRow(x, rh, rt, 24, 0.045, mHotStator, 0.9, 0.013);
        ggt.add(ngv);
        hotMeshes.push(ngv);
      }
      ggt.add(new THREE.Mesh(wall([[0.252, 0.06], [0.258, 0.2], [0.265, 0.34], [0.278, 0.42]], 0.012, 72), mCaseDark));
    }
    core.add(
      ctx.part(ggt, {
        id: 'ggTurbine',
        name: 'Turbine del generatore di gas',
        base:
          "Due piccole turbine colpite per prime dai gas roventi. La prima fa girare il compressore centrifugo, la seconda le palette di bassa pressione. " +
          "Lavorano solo per tenere in moto il motore stesso: per questo, insieme ai compressori, si chiamano \"generatore di gas\".",
        expert:
          "Turbina HP monostadio (pale raffreddate in superlega monocristallina) → albero HP; turbina LP monostadio → albero LP, coassiale e interno. " +
          "Il generatore di gas bialbero dà margine allo stallo e risposta rapida; NH e NL variano con la potenza (~70% al minimo in volo, 100% al decollo). " +
          "Gas d'ingresso ≈ 1.100–1.300 °C, raffreddamento con aria spillata dal compressore HP.",
        explode: [0.2, -0.6, 0],
      }),
    );

    // ---------- Free power turbine (2 stages, PT shaft) ----------
    const pt = new THREE.Group();
    const ptRotor = new THREE.Group();
    pt.add(ptRotor);
    ptRot.push(ptRotor);
    for (const [x, rh, rt, n] of [[0.61, 0.15, 0.29, 44], [0.74, 0.158, 0.315, 46]]) {
      ptRotor.add(rotorRow(x, rh, rt - 0.004, n, 0.045, mPt, { twist: -0.3, stagger: -0.6, thick: 0.011 }));
      ptRotor.add(disc(x, 0.035, rh, 0.05, mDrum));
    }
    for (const [x, rh, rt] of [[0.55, 0.148, 0.284], [0.68, 0.154, 0.305]]) {
      pt.add(statorRow(x, rh, rt, 28, 0.045, mStator, 0.75, 0.011));
    }
    pt.add(new THREE.Mesh(wall([[0.278, 0.42], [0.288, 0.5], [0.295, 0.58], [0.312, 0.66], [0.33, 0.76], [0.345, 0.82]], 0.012, 72), mCase));
    pt.add(new THREE.Mesh(flangeX(0.81, 0.355, 0.4, 0.024), mCase));
    pt.add(boltSet(mBolt, 0.01, 0.011).ringX(0.797, 0.38, 24, -1).mesh());
    core.add(
      ctx.part(pt, {
        id: 'powerTurbine',
        name: 'Turbina libera (di potenza)',
        base:
          "Due file di palette più grandi, mosse dai gas che escono dal generatore di gas. Non sono collegate ai compressori: fanno girare soltanto l'elica, " +
          "attraverso l'albero più interno del motore. Per questo si chiama \"libera\".",
        expert:
          "Turbina di potenza a 2 stadi (PW127), circa 20.000 giri/min, quasi costanti perché legati all'elica a velocità costante tramite il riduttore. " +
          "Converte in lavoro all'albero gran parte dell'energia residua dei gas: ≈ 2 MW al decollo. Il disaccoppiamento dal generatore di gas semplifica l'avviamento " +
          "(l'avviatore muove solo NH) e permette di frenare l'elica a terra (\"hotel mode\" sull'ATR) con il motore acceso.",
        explode: [0.75, 0, 0],
      }),
    );

    // ---------- Shafts (PT innermost, LP, HP outermost) ----------
    const shafts = new THREE.Group();
    const ptShaft = new THREE.Mesh(latheX([[0.022, -1.66], [0.035, -1.66], [0.035, 0.76], [0.022, 0.76]], 24), mShaft);
    const lpShaft = new THREE.Mesh(latheX([[0.044, -0.87], [0.055, -0.87], [0.055, 0.31], [0.044, 0.31]], 28), mShaft2);
    const hpShaft = new THREE.Mesh(latheX([[0.064, -0.47], [0.078, -0.47], [0.078, 0.15], [0.064, 0.15]], 32), mShaft2);
    shafts.add(ptShaft, lpShaft, hpShaft);
    ptRot.push(ptShaft);
    lpRot.push(lpShaft);
    hpRot.push(hpShaft);
    core.add(
      ctx.part(shafts, {
        id: 'shafts',
        name: 'Alberi coassiali',
        base:
          "Tre alberi infilati uno dentro l'altro come tubi. Il più interno, lungo e sottile, porta la potenza della turbina libera fino al riduttore; " +
          "gli altri due collegano ciascun compressore alla sua turbina. Girano tutti a velocità diverse.",
        expert:
          "Architettura a tre alberi: albero di potenza interno (NP ≈ 20.000 giri/min), albero LP e albero HP esterno, sostenuti da cuscinetti a sfere e a rulli in camere d'olio. " +
          "L'albero di potenza attraversa tutto il generatore di gas e la cassa d'ingresso dentro il tubo di coppia fino al pignone del riduttore.",
        explode: [0, -0.35, 0],
      }),
    );

    // ---------- Exhaust duct ----------
    const exh = new THREE.Group();
    const exDuct = [
      { p: [0.82, 0], hw: 0.355, hh: 0.355 },
      { p: [1.05, 0.02], hw: 0.34, hh: 0.34 },
      { p: [1.3, 0.16], hw: 0.31, hh: 0.31 },
      { p: [1.48, 0.4], hw: 0.285, hh: 0.285 },
      { p: [1.6, 0.6], hw: 0.27, hh: 0.27 },
      { p: [1.68, 0.84], hw: 0.26, hh: 0.26 },
    ];
    exh.add(new THREE.Mesh(loftWall(exDuct, 0.014, 30, 48), mExhaust));
    exh.add(new THREE.Mesh(flangeX(0.835, 0.355, 0.4, 0.024), mExhaust));
    {
      // rolled lip at the stack exit, square to the duct
      const lip = new THREE.Mesh(new THREE.TorusGeometry(0.262, 0.014, 8, 48), mExhaust);
      const dir = new THREE.Vector3(0.07, 0.24, 0).normalize();
      lip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir);
      lip.position.set(1.68, 0.84, 0);
      exh.add(lip);
    }
    exh.add(new THREE.Mesh(latheX([[0.165, 0.79], [0.16, 0.88], [0.12, 0.98], [0.06, 1.06], [0, 1.09]], 48), ds(MAT.darkSteel())));
    exh.add(statorRow(0.85, 0.165, 0.35, 6, 0.08, mExhaust, 0, 0.018));
    core.add(
      ctx.part(exh, {
        id: 'exhaust',
        name: 'Condotto di scarico',
        base:
          "Il tubo curvo da cui escono i gas, ancora caldi ma ormai senza molta energia. È piegato per mandarli verso l'alto e indietro, lontano dall'ala e dalla fusoliera.",
        expert:
          "Condotto di scarico divergente con cono di coda e bracci del telaio posteriore. I gas escono a ≈ 500–600 °C e a bassa velocità: la spinta residua del getto è " +
          "il 5–10% del totale. Nell'ATR lo scarico sbocca sul lato della gondola sopra l'ala; nel PT6A ci sono due scarichi laterali vicino all'elica perché il flusso è invertito.",
        explode: [0.9, 0.35, 0],
      }),
    );

    // ---------- Nacelle (cowling) with intake face, latches, oil-cooler scoop, engine mounts ----------
    const nac = new THREE.Group();
    const nacSec = [
      { p: [-1.92, -0.42], hw: 0.4, hh: 0.76, e: 2.6 },
      { p: [-1.75, -0.3], hw: 0.56, hh: 0.9, e: 2.6 },
      { p: [-1.4, -0.28], hw: 0.63, hh: 0.95, e: 2.6 },
      { p: [-0.6, -0.3], hw: 0.67, hh: 0.93, e: 2.6 },
      { p: [0.6, -0.35], hw: 0.66, hh: 0.86, e: 2.6 },
      { p: [1.3, -0.42], hw: 0.55, hh: 0.68, e: 2.4 },
      { p: [1.8, -0.5], hw: 0.3, hh: 0.38, e: 2.2 },
      { p: [2.05, -0.55], hw: 0.06, hh: 0.07, e: 2 },
    ];
    nac.add(new THREE.Mesh(loftWall(nacSec, 0.018, 48, 64), mNacelle));
    const nacCurve = new THREE.CatmullRomCurve3(nacSec.map((q) => new THREE.Vector3(q.p[0], q.p[1], 0)));
    const nacAt = (x) => {
      const v = new THREE.Vector3();
      let best = 0;
      let bd = 1e9;
      for (let i = 0; i <= 400; i++) {
        nacCurve.getPoint(i / 400, v);
        if (Math.abs(v.x - x) < bd) {
          bd = Math.abs(v.x - x);
          best = i / 400;
        }
      }
      nacCurve.getPoint(best, v);
      const f = best * (nacSec.length - 1);
      const k = Math.min(Math.floor(f), nacSec.length - 2);
      const u = f - k;
      const a = nacSec[k];
      const b = nacSec[k + 1];
      return { p: [v.x, v.y], hw: lerp(a.hw, b.hw, u), hh: lerp(a.hh, b.hh, u), e: lerp(a.e, b.e, u) };
    };
    const surfZ = (q, y) => q.hw * Math.pow(Math.max(0, 1 - Math.abs(clamp((y - q.p[1]) / q.hh, -1, 1)) ** q.e), 1 / q.e);
    {
      // front face: closes the cowl around the spinner and the chin intake
      const sup = (q, cx, cy, n = 64) =>
        Array.from({ length: n }, (_, i) => {
          const a = (i / n) * TAU;
          const c = Math.cos(a);
          const sn = Math.sin(a);
          return new THREE.Vector2(cx + q.hw * Math.sign(c) * Math.abs(c) ** (2 / q.e), cy + q.hh * Math.sign(sn) * Math.abs(sn) ** (2 / q.e));
        });
      const face = new THREE.Shape(sup(nacSec[0], 0, nacSec[0].p[1]));
      const hs = new THREE.Path();
      hs.absarc(0, YP, 0.2, 0, TAU, true);
      face.holes.push(hs, new THREE.Path(sup(duct[0], 0, duct[0].p[1], 48).reverse()));
      nac.add(new THREE.Mesh(new THREE.ExtrudeGeometry(face, { depth: 0.02, bevelEnabled: false }).rotateY(-Math.PI / 2).translate(-1.92, 0, 0), mNacelle));
      // panel joints
      for (const x of [-1.7, -0.95, 0.5, 1.2]) {
        const q = nacAt(x);
        nac.add(new THREE.Mesh(lipGeo({ ...q, hw: q.hw + 0.002, hh: q.hh + 0.002 }, 0.006), mNacelleDark));
      }
      // cowl latches along both lower sides
      const latchGeo = new RoundedBoxGeometry(0.11, 0.035, 0.02, 2, 0.008);
      for (const x of [-1.45, -1.05, -0.45, 0.05, 0.85]) {
        const q = nacAt(x);
        const y = q.p[1] - 0.42 * q.hh;
        const z = surfZ(q, y) + 0.004;
        for (const sz of [1, -1]) {
          const l = new THREE.Mesh(latchGeo, mLatch);
          l.position.set(x, y, sz * z);
          nac.add(l);
        }
      }
      // oil-cooler scoop under the cowl
      const qb = nacAt(-0.6);
      const yb = qb.p[1] - qb.hh;
      const scoop = new THREE.Mesh(new RoundedBoxGeometry(0.6, 0.15, 0.34, 3, 0.05), cut(MAT.paint(0xd3d8dd, 0.42), { cap: false }));
      scoop.position.set(-0.55, yb - 0.03, 0);
      const mouth = new THREE.Mesh(new RoundedBoxGeometry(0.02, 0.085, 0.27, 2, 0.008), mNacelleDark);
      mouth.position.set(-0.85, yb - 0.04, 0);
      const exitFlap = new THREE.Mesh(new RoundedBoxGeometry(0.02, 0.06, 0.28, 2, 0.008), mNacelleDark);
      exitFlap.position.set(-0.255, yb - 0.03, 0);
      nac.add(scoop, mouth, exitFlap);
      // top hinge line and rear cooling louvers
      const hinge = [];
      for (let x = -1.7; x <= 1.21; x += 0.1) {
        const q = nacAt(x);
        hinge.push([x, q.p[1] + q.hh + 0.003, 0]);
      }
      nac.add(new THREE.Mesh(pipeAlong(hinge, 0.008, 60), mNacelleDark));
      const louver = new RoundedBoxGeometry(0.12, 0.016, 0.014, 2, 0.006);
      for (let i = 0; i < 6; i++) {
        const x = 0.7 + i * 0.065;
        const q = nacAt(x);
        const y = q.p[1] + 0.25 * q.hh;
        for (const sz of [1, -1]) {
          const l = new THREE.Mesh(louver, mNacelleDark);
          l.position.set(x, y, sz * (surfZ(q, y) + 0.002));
          nac.add(l);
        }
      }
      // engine mount struts: gearbox and combustor case to the nacelle structure
      for (const sz of [1, -1]) {
        for (const [a, b] of [
          [[-1.6, 0.25, 0.31], [-1.1, 0.4, 0.5]],
          [[-1.6, 0.25, 0.31], [-1.75, 0.45, 0.45]],
          [[0.3, 0.036, 0.344], [0.6, 0.3, 0.5]],
          [[0.3, 0.036, 0.344], [0.05, 0.32, 0.5]],
        ]) {
          nac.add(new THREE.Mesh(cylBetweenGeo(new THREE.Vector3(a[0], a[1], sz * a[2]), new THREE.Vector3(b[0], b[1], sz * b[2]), 0.017, 10), mLatch));
        }
      }
    }
    model.add(
      ctx.part(nac, {
        id: 'nacelle',
        name: 'Gondola motore (cofanature)',
        base:
          "Il guscio aerodinamico che avvolge motore e riduttore sotto l'ala. Si apre a sportelli, chiusi da ganci a leva, per la manutenzione. " +
          "Sotto c'è la presa d'aria del radiatore dell'olio; in coda il tubo di scarico sbuca dalla cofanatura.",
        expert:
          "Cofanature in composito e alluminio incernierate, con chiusure a leva; struttura di attacco motore a traliccio con supporti elastici " +
          "che isolano le vibrazioni. Presa \"a mento\" integrata nella parte anteriore, presa del radiatore olio-aria con sportello d'uscita regolabile, " +
          "pareti tagliafuoco e impianto antincendio nel vano motore.",
        explode: [0, 2.1, 0],
      }),
    );

    // ---------- Accessories: gearbox, fuel control, oil tank, pipes, harness ----------
    const acc = new THREE.Group();
    {
      const agb = new THREE.Mesh(new RoundedBoxGeometry(0.26, 0.15, 0.42, 3, 0.03), mAccDark);
      agb.position.set(-1.36, YE - 0.22, 0);
      const tower = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.08, 14), mAcc);
      tower.position.set(-1.36, YE - 0.12, 0);
      const hmu = new THREE.Mesh(new RoundedBoxGeometry(0.17, 0.12, 0.12, 3, 0.02), mAcc);
      hmu.position.set(-1.36, YE - 0.22, -0.27);
      const starter = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.18, 24).rotateX(Math.PI / 2), mAcc);
      starter.position.set(-1.36, YE - 0.22, 0.3);
      const tank = new THREE.Mesh(new RoundedBoxGeometry(0.28, 0.2, 0.16, 3, 0.04), mAcc);
      tank.position.set(-1.34, YE + 0.2, -0.36);
      const bracket = new THREE.Mesh(new RoundedBoxGeometry(0.07, 0.09, 0.08, 2, 0.01), mAccDark);
      bracket.position.set(-1.51, YE + 0.2, -0.36);
      const filler = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 14), mLatch);
      filler.position.set(-1.3, YE + 0.32, -0.36);
      acc.add(agb, tower, hmu, starter, tank, bracket, filler);
      acc.add(
        new THREE.Mesh(
          pipeAlong([[-1.3, YE - 0.22, -0.33], [-1.0, YE - 0.3, -0.47], [-0.4, YE - 0.35, -0.55], [0.2, YE - 0.36, -0.56], [0.65, YE - 0.3, -0.52], [0.8, YE - 0.22, -0.465]], 0.012, 64),
          mPipe,
        ),
      );
      acc.add(
        new THREE.Mesh(
          pipeAlong([[-1.3, YE - 0.22, -0.22], [-1.0, YE - 0.1, -0.45], [-0.5, YE - 0.2, -0.42], [-0.25, YE - 0.35, -0.55], [0.1, YE - 0.5, -0.42], [0.42, YE - 0.6, -0.22]], 0.014, 64),
          mHarness,
        ),
      );
      acc.add(new THREE.Mesh(pipeAlong([[-1.25, YE + 0.12, -0.36], [-1.2, YE - 0.05, -0.3], [-1.32, YE - 0.15, -0.21]], 0.014, 24), mHarness));
    }
    model.add(
      ctx.part(acc, {
        id: 'accessories',
        name: 'Scatola accessori e impianti',
        base:
          "Sotto il riduttore una piccola scatola di ingranaggi muove le pompe del carburante e dell'olio e il motorino di avviamento-generatore. " +
          "Il tubo di rame porta il cherosene agli iniettori, il cavo scuro porta la scintilla alle candele di accensione.",
        expert:
          "AGB trascinata dal generatore di gas; unità di dosaggio del carburante (FMU/HMU) comandata dall'EEC (controllo elettronico a doppio canale), " +
          "avviatore-generatore in corrente continua, serbatoio dell'olio integrato con pompe di mandata e recupero. Due candele ad alta energia servono solo " +
          "all'avviamento e in condizioni critiche (pioggia forte, ghiaccio).",
        explode: [-0.2, -0.8, -0.3],
      }),
    );

    // ---------- Flows ----------
    const flows = [];
    const addFlow = (f, kind) => {
      model.add(f.object);
      ctx.addFlow(f);
      flows.push({ f, kind });
    };
    const P = (x, r, a) => new THREE.Vector3(x, YE + r * Math.cos(a), r * Math.sin(a));
    const W = (x, y, z) => new THREE.Vector3(x, y, z);

    // core gas path, one lane per angle around the engine axis (visible half z <= 0)
    [180, 215, 250, 290, 330].forEach((deg, k) => {
      const a = deg * DEG;
      const zl = -0.05 - 0.04 * k;
      const ex = (x, y, rr) => W(x, YE + y + rr * Math.cos(a) * 0.7, rr * Math.sin(a));
      const path = colorPath([
        [W(-3.1, YE - 0.44, zl * 1.5), C_AIR],
        [W(-2.45, YE - 0.47, zl * 1.3), C_AIR],
        [W(-1.97, YE - 0.48, zl), C_AIR],
        [W(-1.55, YE - 0.5, zl), C_AIR],
        [W(-1.24, YE - 0.51, zl * 0.9), C_AIR],
        [W(-1.07, YE - 0.45, zl * 0.7), C_AIR],
        [W(-1.0, YE - 0.36, zl * 0.4), C_AIR],
        [P(-0.98, 0.3, lerp(Math.PI, a, 0.5)), C_AIR],
        [P(-0.93, 0.24, a), C_AIR],
        [P(-0.84, 0.215, a), C_AIR],
        [P(-0.72, 0.212, a), C_AIR2],
        [P(-0.58, 0.205, a), C_AIR2],
        [P(-0.47, 0.18, a), C_AIR2],
        [P(-0.4, 0.21, a), C_AIR2],
        [P(-0.33, 0.26, a), C_WARM],
        [P(-0.28, 0.33, a), C_WARM],
        [P(-0.255, 0.4, a), C_WARM],
        [P(-0.24, 0.47, a), C_WARM],
        [P(-0.17, 0.52, a), C_HOT],
        [P(0.1, 0.518, a), C_HOT],
        [P(0.45, 0.515, a), C_HOT],
        [P(0.61, 0.49, a), C_HOT],
        [P(0.635, 0.42, a), C_HOT],
        [P(0.565, 0.38, a), C_FIRE],
        [P(0.4, 0.38, a), C_FIRE],
        [P(0.2, 0.38, a), C_FIRE2],
        [P(0.04, 0.37, a), C_FIRE2],
        [P(-0.035, 0.31, a), C_FIRE2],
        [P(-0.035, 0.235, a), C_FIRE2],
        [P(0.01, 0.19, a), C_FIRE2],
        [P(0.07, 0.19, a), C_TURB],
        [P(0.2, 0.195, a), C_TURB],
        [P(0.36, 0.205, a), C_TURB],
        [P(0.5, 0.215, a), C_EXH1],
        [P(0.68, 0.23, a), C_EXH1],
        [P(0.82, 0.24, a), C_EXH],
        [ex(1.06, 0.03, 0.17), C_EXH],
        [ex(1.31, 0.17, 0.15), C_EXH],
        [ex(1.49, 0.41, 0.13), C_EXH],
        [ex(1.62, 0.62, 0.12), C_EXH],
        [ex(1.69, 0.86, 0.12), C_EXH],
        [ex(1.8, 1.15, 0.18), C_EXH],
        [ex(1.98, 1.55, 0.28), C_EXH],
      ]);
      const tFire = path.tOf(23);
      const tExit = path.tOf(40);
      addFlow(
        flow({
          path: path.at,
          count: 110,
          size: 0.045,
          speed: 0.1,
          colors: path.colors(48),
          spread: (t) => (t < 0.18 ? 0.06 : t < tFire ? 0.03 : t < tExit ? 0.035 : 0.05 + (t - tExit) * 0.6),
          opacity: 0.9,
        }),
        'air',
      );
    });

    // fuel: manifold -> injector -> spray
    for (const deg of [190, 235, 280, 325]) {
      const a = deg * DEG;
      const path = colorPath([
        [P(0.8, 0.51, a), 0xffc04d],
        [P(0.7, 0.448, a), 0xffc04d],
        [P(0.6, 0.385, a), 0xffb040],
        [P(0.5, 0.38, a), 0xff7a2a],
        [P(0.42, 0.38, a), 0xff7a2a],
      ]);
      addFlow(
        flow({ path: path.at, count: 22, size: 0.03, speed: 0.9, colors: path.colors(16), spread: (t) => (t < 0.6 ? 0.004 : 0.012 + (t - 0.6) * 0.08), opacity: 0.9 }),
        'fuel',
      );
    }

    // propeller slipstream (most of the thrust)
    for (const [deg, r0] of [[200, 1.25], [235, 1.38], [270, 1.2], [305, 1.38], [340, 1.25], [355, 1.12]]) {
      const a0 = deg * DEG;
      addFlow(
        flow({
          path: (t, out) => {
            const x = lerp(-3.5, 2.6, t);
            const r = x < -2.6 ? r0 : x < -1.6 ? lerp(r0, r0 * 0.86, x + 2.6) : r0 * 0.86;
            const a = a0 + (x > XB ? 0.28 * (x - XB) : 0);
            return out.set(x, YP + r * Math.cos(a), Math.min(0, r * Math.sin(a)));
          },
          count: 60,
          size: 0.05,
          speed: 0.24,
          colors: [C_AIR, C_AIR, 0x8fe4ff, 0x5fc4ee],
          spread: 0.09,
          opacity: 0.7,
        }),
        'prop',
      );
    }

    // ---------- State ----------
    let spool = null; // lagging power lever (engine inertia)
    let lpAng = 0;
    let hpAng = 0;
    let ptAng = 0; // power-turbine phase Θ (gear convention)
    const pin = new THREE.Vector3();
    const end = new THREE.Vector3();
    const np = () => 0.955 + 0.045 * spool;
    const betaDeg = () => lerp(12, 42, spool);

    return {
      root,
      cycleRate(throttle) {
        return 0.12 + 0.25 * throttle;
      },
      update(state) {
        const dt = state.dt || 0;
        const thr = clamp(state.throttle ?? 0, 0, 1);
        if (spool === null || dt === 0) spool = thr;
        else spool += (thr - spool) * (1 - Math.exp(-dt / 1.2));

        // spools: gas generator speeds follow power; PT/propeller almost constant
        lpAng -= dt * TAU * 1.3 * (0.55 + 0.45 * spool);
        hpAng -= dt * TAU * 2.1 * (0.65 + 0.35 * spool);
        ptAng += dt * TAU * PROP_VIS * RATIO * np();
        for (const o of lpRot) o.rotation.x = lpAng;
        for (const o of hpRot) o.rotation.x = hpAng;
        for (const o of ptRot) o.rotation.x = -ptAng;

        // gear train
        const lay = (ptAng * N_IN) / N_LAY;
        gIn.rotation.x = -(th.in + ptAng);
        for (const g of gLay) {
          g.big.rotation.x = -(g.th0 - lay);
          g.small.rotation.x = -(g.thp0 - lay);
        }
        const propAng = ptAng / RATIO;
        outGroup.rotation.x = -(th.out + propAng);
        propSpin.rotation.x = -propAng;
        spinnerSpin.rotation.x = -propAng;
        propBoltSpin.rotation.x = -propAng;
        pitchSpin.rotation.x = -propAng;

        // blade pitch about each blade axis + yoke and links
        const phi = betaDeg() * DEG - Math.PI / 2;
        const px = ARM.x * Math.cos(phi) + ARM.z * Math.sin(phi);
        const pz = -ARM.x * Math.sin(phi) + ARM.z * Math.cos(phi);
        for (const pg of pitchGroups) pg.rotation.y = phi;
        yoke.position.x = px - LINK;
        for (let i = 0; i < NB; i++) {
          const a = (i / NB) * TAU;
          const c = Math.cos(a);
          const s = Math.sin(a);
          pin.set(px, ARM.y * c - pz * s, ARM.y * s + pz * c);
          end.set(px - LINK, ARM.y * c - ARM_Z * s, ARM.y * s + ARM_Z * c);
          placeBetween(links[i], end, pin);
        }

        // flame and hot section follow fuel flow
        const time = state.time || 0;
        const flick = 0.9 + 0.1 * Math.sin(time * 31) * Math.sin(time * 17.3);
        flameCore.material.emissiveIntensity = (0.25 + 0.95 * spool) * flick;
        flameCore.material.opacity = 0.25 + 0.3 * spool;
        flameHalo.material.opacity = (0.03 + 0.1 * spool) * flick;
        flameJets.material.opacity = (0.1 + 0.25 * spool) * flick;
        flameJets.scale.set(0.4 + 0.6 * spool, 0.8, 0.8);
        for (const m of hotMeshes) {
          if (m.material.emissive) m.material.emissiveIntensity = 0.05 + 0.45 * spool * spool;
        }

        // particles
        const assembled = (state.explode || 0) < 0.05;
        for (const { f, kind } of flows) {
          f.object.visible = assembled;
          if (kind === 'fuel') {
            f.object.material.opacity = 0.3 + 0.6 * spool;
            f.update(dt, 0.3 + 0.9 * spool);
          } else if (kind === 'prop') {
            f.object.material.opacity = 0.35 + 0.5 * spool;
            f.update(dt, 0.5 + 0.9 * spool);
          } else {
            f.update(dt, 0.3 + 0.9 * spool);
          }
        }
      },
      readout() {
        const s = spool ?? 0;
        return [
          { label: 'Giri elica', value: String(Math.round(1200 * (0.955 + 0.045 * s))), unit: 'giri/min' },
          { label: 'Passo pale (75%)', value: lerp(12, 42, s).toFixed(0), unit: '°' },
          { label: 'Potenza all\'albero', value: String(Math.round(2050 * (0.06 + 0.94 * Math.pow(s, 1.25)))), unit: 'kW' },
          { label: 'NH (gen. di gas)', value: (70 + 30 * s).toFixed(0), unit: '%' },
        ];
      },
    };
  },
};
