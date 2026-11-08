// Helicopter free-turbine turboshaft (T700 / Arriel / PW210 class) with main gearbox, swashplate,
// rotor head and tail-rotor drive. Engine axis along X (air in at the front plenum, exhaust bent up
// at the rear); the output shaft runs forward to the main gearbox under the vertical rotor mast.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, blade, ringX, flow, clamp, lerp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const DEG = Math.PI / 180;
const OX = -0.05; // model offset along X

const XM = -1.2; // rotor mast x
const YI = -0.62; // engine / input shaft axis height
const YT = -1.18; // tail drive shaft height
const YS = -0.25; // planetary stage plane
const YH = 0.8; // rotor hub plane
// main bevel pair (input pinion -> crown wheel), tail take-off bevel pair
const NP_IN = 17;
const NW = 60;
const RW = 0.3;
const GW = Math.atan(NW / NP_IN);
const AW = RW / Math.sin(GW);
const NT = 36;
const NTP = 18;
const RT = 0.14;
const GT = Math.atan(NT / NTP);
const AT = RT / Math.sin(GT);
// planetary stage (sun on the crown-wheel shaft, fixed ring, carrier drives the mast)
const NS = 22;
const NPL = 24;
const NR = 70;
const PM = 0.004; // pitch radius per tooth
const RC = (NS + NPL) * PM; // carrier radius
const K_PLAN = 1 + NR / NS; // ~4.18:1
const K_MGB = (NW / NP_IN) * K_PLAN; // ~14.8:1 shown (real total ~80:1 with more stages)
// swashplate / pitch-horn geometry
const HR = 0.2; // horn radial station
const HL = 0.2; // horn arm towards the leading edge
const LK = 0.38; // pitch-link length
const NBL = 5; // main rotor blades
const ROTOR_VIS = 0.2; // mast turns shown per second

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

// Hex head with its shank below (local -Y): a bolt backing out shows its thread. +Y points out of the hole.
function screwGeo(r, h, len) {
  return mergeGeos([new THREE.CylinderGeometry(r, r, h, 6), new THREE.CylinderGeometry(r * 0.6, r * 0.6, len, 8).translate(0, -h / 2 - len / 2, 0)]);
}

// Retaining nut: washer face, hex body and domed cap. Origin 0.7 h above the seat, +Y out.
function capNutGeo(r, h) {
  return mergeGeos([
    new THREE.CylinderGeometry(r, r, h, 6),
    new THREE.CylinderGeometry(r * 1.2, r * 1.2, h * 0.2, 24).translate(0, -h * 0.6, 0),
    new THREE.SphereGeometry(r * 0.72, 16, 6, 0, TAU, 0, Math.PI / 2).scale(1, 0.6, 1).translate(0, h / 2, 0),
  ]);
}

// Bolt heads as one InstancedMesh: add(pos, normal) then mesh(). `geo` replaces the plain hex head
// (origin h / 2 above the surface, local +Y along the normal).
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

const ds = (m) => {
  m.side = THREE.DoubleSide;
  return m;
};

// Spur gear with its axis along Y. Tooth angle convention: atan2(-z, x); rotation.y = phase.
function spurY(N, rp, width, bore = 0) {
  const shape = new THREE.Shape(gearOutline(N, rp));
  if (bore > 0) {
    const h = new THREE.Path();
    h.absarc(0, 0, bore, 0, TAU, true);
    shape.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(shape, { depth: width, bevelEnabled: false, curveSegments: 24 });
  g.translate(0, 0, -width / 2);
  g.rotateX(-Math.PI / 2); // shape (s, t) -> (x = s, z = -t)
  return g;
}

// Internal (ring) gear with axis along Y.
function ringGearY(N, rp, rOut, width) {
  const shape = new THREE.Shape();
  shape.absarc(0, 0, rOut, 0, TAU, false);
  shape.holes.push(new THREE.Path(gearOutline(N, rp, true)));
  const g = new THREE.ExtrudeGeometry(shape, { depth: width, bevelEnabled: false, curveSegments: 64 });
  g.translate(0, 0, -width / 2);
  g.rotateX(-Math.PI / 2);
  return g;
}

// Bevel gear, axis +Y, pitch-cone apex at the origin, half-angle gamma, cone distance A, face width F.
// Teeth on the outer side of the cone, tooth k at azimuth 2πk/N (atan2(-z, x)).
function bevelGeo(N, A, F, gamma, hubR) {
  const sg = Math.sin(gamma);
  const cg = Math.cos(gamma);
  const m = (2 * A * sg) / N;
  const p1 = [(A - F) * sg, (A - F) * cg];
  const p2 = [A * sg, A * cg];
  const d = Math.max(0.03, F * 0.45);
  const q3 = [p2[0] - cg * d, p2[1] + sg * d];
  const list = [lathe([[hubR, p1[1]], p1, p2, q3, [hubR, q3[1]], [hubR, p1[1]]], 64)];
  const sm = A - F / 2;
  const w = 0.45 * ((TAU * sm * sg) / N);
  const tooth = new THREE.BoxGeometry(F, 2.1 * m, w).rotateZ(Math.PI / 2 - gamma).translate(sm * sg + cg * 0.3 * m, sm * cg - sg * 0.3 * m, 0);
  for (let k = 0; k < N; k++) list.push(tooth.clone().rotateY((k / N) * TAU));
  tooth.dispose();
  return mergeGeos(list);
}

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
  id: 'turboshaft',
  name: 'Turboalbero per elicottero',
  short: 'Turboalbero',
  family: 'aria',
  vehicle: 'Elicottero',
  throttleLabel: 'Collettivo',
  examples:
    'Safran Arriel 2E (Airbus H145), Pratt & Whitney Canada PT6C-67C (Leonardo AW139), GE T700 / RTM322 (Sikorsky UH-60, NH90), Safran Arrano (Airbus H160)',
  tagline: 'Una turbina che non spinge: fa girare il rotore sempre alla stessa velocità, e il pilota cambia solo l\'angolo delle pale.',

  intro: {
    base:
      "Il turboalbero è una turbina a gas che non produce un getto ma fa girare un <b>albero</b>. Nell'elicottero quell'albero entra nel <b>riduttore principale</b>, " +
      "che rallenta la rotazione di circa 80 volte e la gira in verticale verso il rotore. La cosa sorprendente: il rotore gira <b>sempre agli stessi giri</b>. " +
      "Quando il pilota alza la leva del <b>collettivo</b>, tutte le pale si inclinano di più, sollevano di più, e il motore riceve più carburante per non rallentare.",
    expert:
      "Turboalbero a <b>turbina libera</b>: generatore di gas monoalbero (compressore assiale + centrifugo, combustore anulare, turbina a 2 stadi, Ng ≈ 40.000–50.000 giri/min) " +
      "e turbina di potenza a 2 stadi su albero coassiale (≈ 20.000–30.000 giri/min) con uscita anteriore. Il riduttore principale (MGB) porta i giri a ~250–400 giri/min " +
      "(≈ 80:1 sul Black Hawk) con coppia conica e stadio epicicloidale. Il <b>FADEC</b> mantiene Nr ≈ 100%: la potenza segue il <b>passo collettivo</b>, " +
      "trasmesso dal piatto oscillante alle pale; il rotore di coda assorbe ~10–15% della potenza per compensare la coppia.",
  },

  specs: [
    { label: 'Carburante', value: 'Cherosene Jet A-1 / JP-8' },
    { label: 'Potenza per motore', value: '~600–1.500 kW (Arriel 2E ~700, T700 ~1.400)' },
    { label: 'Giri generatore di gas (Ng)', value: '~40.000–50.000 giri/min' },
    { label: 'Giri turbina libera', value: '~20.000–30.000 giri/min (T700: ~21.000)' },
    { label: 'Giri rotore principale', value: '~250–400 giri/min, costanti (Nr 100%)' },
    { label: 'Riduzione totale', value: '~80:1 in 3–4 stadi' },
    { label: 'Peso', value: '~200–250 kg (motore secco)' },
  ],

  metrics: {
    efficiency: 28,
    power: 1000,
    powerDensity: 5,
  },

  variants: {
    columns: ['Motore', 'Elicottero', 'Potenza al decollo', 'Schema'],
    rows: [
      ['Safran Arriel 2E', 'Airbus H145', '~700–800 kW', 'Assiale + centrifugo, combustore a flusso invertito, turbina libera monostadio'],
      ['Safran Arrano 1A', 'Airbus H160', '~1.000 kW', 'Compressore centrifugo a 2 stadi'],
      ['P&WC PW210', 'Leonardo AW169, Sikorsky S-76D', '~750 kW (classe 1.000 shp)', 'Compressore centrifugo a 2 stadi'],
      ['P&WC PT6C-67C', 'Leonardo AW139', '~1.100 kW', 'Assiale + centrifugo, turbina libera'],
      ['GE T700-701D', 'Sikorsky UH-60M, Boeing AH-64', '~1.500 kW (≈ 2.000 shp)', '5 assiali + 1 centrifugo, separatore di particelle integrato'],
      ['RTM322', 'NH90, AW101 Merlin', '~1.600–1.900 kW', 'Assiale + centrifugo, turbina libera'],
    ],
    note: 'Il modello segue lo schema del T700 (combustore diritto, uscita anteriore passante); quasi tutti gli elicotteri moderni sono bimotore.',
  },

  thumb: { orbit: [-35, 20], zoom: 1.15, view: 'section' },

  phases: [
    {
      id: 'gasgen',
      name: 'Turbina a gas',
      parts: ['intake', 'compressor', 'combustor', 'ggTurbine', 'accessories'],
      base:
        "L'aria entra da tutti i lati attraverso una griglia che ferma sassi e foglie, viene compressa da file di palette e da una girante centrifuga, " +
        "brucia con il cherosene in una camera ad anello e fa girare una turbina che tiene in moto il compressore. Questo blocco si chiama \"generatore di gas\": " +
        "il suo unico compito è produrre un fiume di gas caldissimi e veloci.",
      expert:
        "Generatore di gas monoalbero (schema T700: 5 stadi assiali + 1 centrifugo, qui semplificati in 3 + 1), rapporto di compressione ≈ 15–18:1, " +
        "combustore anulare diritto, turbina del generatore di gas a 2 stadi raffreddata. Ng ≈ 44.700 giri/min al 100% sul T700, ~52.000 sull'Arriel. " +
        "Con il collettivo il FADEC aumenta il carburante: Ng sale, T4 sale, cresce la portata di gas verso la turbina libera.",
    },
    {
      id: 'freeTurbine',
      name: 'Turbina libera',
      parts: ['powerTurbine', 'shafts', 'exhaust'],
      base:
        "Dietro il generatore di gas c'è una seconda turbina, non collegata al compressore: è la turbina \"libera\". I gas la fanno girare e lei, con un albero " +
        "che passa dentro tutto il motore, porta la potenza davanti, verso il riduttore. I gas ormai stanchi escono da un tubo piegato verso l'alto.",
      expert:
        "Turbina di potenza a 2 stadi, ≈ 20.900 giri/min (T700) quasi costanti, perché legata al rotore. L'albero di potenza attraversa coassialmente l'albero del " +
        "generatore di gas fino all'uscita anteriore. Il disaccoppiamento permette a Ng di variare con il carico mentre Np/Nr restano al 100%; " +
        "lo scarico, divergente, recupera pressione e produce spinta trascurabile.",
    },
    {
      id: 'mgb',
      name: 'Riduttore principale',
      parts: ['mainGearbox', 'driveShaft', 'airframe'],
      base:
        "Il riduttore principale è il \"cambio\" dell'elicottero, ma con un solo rapporto. Una coppia di ingranaggi conici gira la rotazione di 90°, da orizzontale a verticale; " +
        "poi un ingranaggio planetario, con piccoli satelliti che girano attorno a un sole dentro una corona fissa, rallenta ancora. " +
        "Così da decine di migliaia di giri si arriva a qualche centinaio, con una forza di torsione enorme sull'albero del rotore.",
      expert:
        "MGB con coppia conica spirale (qui 17/60 denti) e stadio epicicloidale a sole conduttore, corona fissa e portasatelliti sull'albero rotore (1 + 70/22 ≈ 4,2:1); " +
        "nel modello ≈ 15:1, nei riduttori reali un primo stadio ad alta velocità (spesso sul motore) porta il totale a ~80:1. Il riduttore somma la potenza dei due motori " +
        "attraverso <b>ruote libere</b> (in avaria o in autorotazione il rotore non trascina il motore), aziona pompe idrauliche e alternatori e regge il peso dell'elicottero. " +
        "Requisito di 30 minuti di funzionamento senza olio (\"dry run\").",
    },
    {
      id: 'swashplate',
      name: 'Piatto oscillante e passo collettivo',
      parts: ['swashplate', 'rotorHead'],
      base:
        "Come si cambia l'angolo di pale che girano? Con il <b>piatto oscillante</b>: due anelli uno sopra l'altro attorno all'albero. Quello sotto è fermo ed è spinto " +
        "da tre attuatori; quello sopra gira con il rotore e, tramite delle astine, tira le leve delle pale. Alzando il collettivo tutto il piatto sale e tutte le pale " +
        "aumentano l'inclinazione insieme: l'elicottero sale.",
      expert:
        "Piatto fisso mosso da 3 servoattuatori idraulici; piatto rotante su cuscinetto, trascinato dal compasso (scissors), collegato con le aste di comando del passo (pitch link) " +
        "alle squadrette (pitch horn) delle pale. Collettivo: traslazione verticale del piatto → stesso passo per tutte le pale (qui ~2°→16°). Ciclico: inclinazione del piatto → " +
        "passo che varia lungo il giro, per inclinare il disco e spostarsi. Nr costante, la portanza si regola solo con il passo.",
    },
    {
      id: 'tail',
      name: 'Rotore di coda (anti-coppia)',
      parts: ['tailDrive', 'tailRotor'],
      base:
        "Se il motore fa girare il rotore in un verso, la fusoliera tende a girare nel verso opposto, come quando ruoti un trapano molto potente. " +
        "Il rotore di coda, mosso da un lungo albero, spinge di lato la coda per impedirlo. Più potenza dai al rotore principale, più deve spingere il rotore di coda.",
      expert:
        "Coppia di reazione Q = P/Ω: con 1.500 kW a ~27 rad/s si superano 50 kNm. Il rotore di coda, a ~5 volte i giri del rotore principale, genera una spinta laterale " +
        "che con il braccio della trave di coda la bilancia; assorbe ~10–15% della potenza in hovering. Albero di trasmissione segmentato con giunti flessibili e cuscinetti " +
        "di supporto, scatola intermedia (cambio d'angolo) e scatola di coda a 90°. I pedali variano il passo delle pale di coda (qui segue il collettivo).",
    },
  ],

  quiz: [
    {
      q: 'Cosa succede quando il pilota alza la leva del collettivo?',
      options: [
        'Il rotore gira molto più veloce',
        'Tutte le pale aumentano l\'angolo insieme e il motore dà più potenza, ma i giri del rotore restano quasi costanti',
        'Si inclina il disco del rotore in avanti',
        'Il rotore di coda si ferma',
      ],
      correct: 1,
      explain:
        'Il rotore lavora a giri quasi costanti (~100% Nr). Il collettivo cambia il passo di tutte le pale; il FADEC aggiunge carburante per non far calare i giri.',
    },
    {
      q: 'A cosa serve il rotore di coda?',
      options: [
        'A dare più velocità in avanti',
        'A raffreddare il motore',
        'A contrastare la coppia di reazione che farebbe ruotare la fusoliera al contrario del rotore',
        'A frenare il rotore principale all\'atterraggio',
      ],
      correct: 2,
      explain:
        'Il motore fa girare il rotore in un verso e la fusoliera tenderebbe a ruotare nell\'altro. Il rotore di coda spinge lateralmente la coda e annulla questa coppia.',
    },
    {
      q: 'Qual è il compito del piatto oscillante?',
      options: [
        'Trasmettere i comandi dalle parti ferme alle pale che girano, cambiandone il passo',
        'Raffreddare l\'olio del riduttore',
        'Ridurre i giri del motore',
        'Avviare la turbina',
      ],
      correct: 0,
      explain:
        'Il piatto fisso riceve i comandi dagli attuatori; quello rotante gira con il rotore e, con le aste di comando, cambia l\'angolo di ogni pala (collettivo e ciclico).',
    },
    {
      q: 'Perché serve un riduttore principale così grande?',
      options: [
        'Per far girare il rotore più veloce del motore',
        'Per cambiare marcia durante il volo',
        'Per immagazzinare carburante',
        'Perché la turbina libera gira a ~20.000 giri/min e il rotore a ~300: serve una riduzione di circa 80 volte, con angolo di 90°',
      ],
      correct: 3,
      explain:
        'Il riduttore abbassa i giri e moltiplica la coppia, gira il moto in verticale con ingranaggi conici, e somma la potenza dei motori tramite ruote libere.',
    },
  ],

  // Officina (see CONTRACT.md): rotors first, then the engine modules slide off the rear along the shafts.
  // The roof structure, the main gearbox and the mast stay as the base. Simplified and educational.
  disassembly: {
    intro: "Smontiamo l'elicottero dall'esterno verso l'interno: prima i rotori, poi i moduli del motore, sfilati uno alla volta da dietro.",
    note:
      "Sequenza semplificata a scopo didattico: sugli elicotteri veri ogni intervento segue il manuale di manutenzione del costruttore, con i suoi attrezzi, controlli e serraggi.",
    parts: {
      tailRotorNut: { motion: 'unscrew', turns: 5, out: 0.08, dir: [0, 0, 1], dist: 0.3 },
      tailRotor: { after: ['tailRotorNut'], dir: [0, 0, 1], dist: 0.7 },
      tailDrive: { after: ['tailRotor'], dir: [0, 0, 1], dist: 0.9 },
      mastNut: { motion: 'unscrew', turns: 6, out: 0.12, dir: [0, 1, 0], dist: 0.35 },
      rotorHead: { after: ['mastNut'], dir: [0, 1, 0], dist: 1.0, duration: 1.2 },
      swashplate: { after: ['rotorHead'], dir: [0, 1, 0], dist: 1.0 },
      rotorMast: { after: ['rotorHead', 'swashplate'], dir: [0, 1, 0], dist: 1.2 },
      accessories: { dir: [0, 1, 0], dist: 0.7 },
      caseBolts: { motion: 'unscrew', turns: 6, out: 0.07, dir: [0, 0.3, 1], dist: 0.35, duration: 2 },
      exhaust: { after: ['caseBolts'], dir: [1, 0.3, 0], dist: 1.0 },
      powerTurbine: { after: ['caseBolts', 'exhaust'], dir: [1, 0, 0], dist: 1.0 },
      ggTurbine: { after: ['caseBolts', 'powerTurbine'], dir: [1, 0, 0], dist: 1.0 },
      combustor: { after: ['caseBolts', 'ggTurbine', 'accessories'], dir: [1, 0, 0], dist: 1.2 },
      compressor: { after: ['combustor', 'accessories'], dir: [1, 0, 0], dist: 1.2 },
      intake: { after: ['compressor'], dir: [1, 0, 0], dist: 1.0 },
      driveBolts: { motion: 'unscrew', turns: 5, out: 0.04, dir: [0, 0, 1], dist: 0.3 },
      shafts: { after: ['driveBolts', 'compressor', 'ggTurbine', 'powerTurbine'], dir: [1, 0, 0], dist: 1.4 },
      driveShaft: { after: ['driveBolts', 'shafts', 'intake'], dir: [1, 0, 0], dist: 0.9 },
      mainGearbox: { after: ['rotorMast', 'driveShaft', 'tailDrive'], dir: [0, 1, 0], dist: 1.0 },
    },
    steps: [
      {
        id: 'tailRotor',
        name: 'Togli il rotore di coda',
        base: "Si svita il dado al centro e si sfila di lato il rotore di coda dal suo alberino.",
        expert:
          "Prima si scollega il comando del passo, che arriva alle pale attraverso l'albero della scatola di coda; tolto il dado di ritegno, il mozzo si sfila dalle scanalature.",
        parts: ['tailRotorNut', 'tailRotor'],
      },
      {
        id: 'tailDrive',
        name: 'Stacca la trasmissione di coda',
        base: "Si libera il lungo albero che corre sotto il motore, con i suoi supporti e le due scatole di ingranaggi sulla deriva.",
        expert:
          "L'albero di coda è fatto a tronchi uniti da giunti flessibili: nella realtà si smonta un tronco alla volta, staccando i cuscinetti di supporto dalla struttura. " +
          "La scatola intermedia e quella di coda sono imbullonate alla deriva.",
        parts: ['tailDrive'],
      },
      {
        id: 'rotorHead',
        name: 'Togli la testa del rotore',
        base: "Si svita il grosso dado in cima all'albero e si solleva il mozzo con le pale: nella realtà serve una gru.",
        expert:
          "Prima si staccano le aste di comando del passo dalle squadrette (e di solito si smontano le pale, una alla volta); tolto il dado, il mozzo si sfila verso l'alto " +
          "dalle scanalature dell'albero rotore.",
        parts: ['mastNut', 'rotorHead'],
      },
      {
        id: 'swashplate',
        name: 'Sfila il piatto oscillante',
        base: "Senza il mozzo, il piatto oscillante con le sue astine e i tre attuatori si sfila verso l'alto lungo l'albero.",
        expert:
          "Piatto fisso, piatto rotante e snodo sferico formano un unico insieme che scorre lungo l'albero rotore, che invece resta nel riduttore. " +
          "Qui escono insieme anche i tre servoattuatori, che nella realtà sono fissati alla scatola del riduttore.",
        parts: ['swashplate'],
      },
      {
        id: 'accessories',
        name: 'Togli la scatola accessori',
        base: "Si staccano il tubo del carburante e il cavo di accensione, poi si solleva la scatola accessori con avviatore, alternatore e regolatore del carburante.",
        expert:
          "La scatola accessori è un modulo a sé, trascinato dal generatore di gas con un alberino radiale che si sfila insieme a lei. " +
          "Con lei vengono via l'unità idromeccanica (HMU), l'avviatore e l'alternatore del FADEC.",
        parts: ['accessories'],
      },
      {
        id: 'hotSection',
        name: 'Sfila scarico e turbine',
        base:
          "Svitate le corone di bulloni sulle flange, i moduli escono da dietro, uno dopo l'altro: lo scarico, la turbina libera e la turbina del generatore di gas.",
        expert:
          "Il motore è modulare: ogni modulo si separa sulla sua flangia e si sfila lungo l'asse. Tra la turbina libera e il generatore di gas non c'è alcun collegamento meccanico, " +
          "le unisce solo il flusso dei gas: per questo la turbina di potenza può essere un modulo separato.",
        parts: ['caseBolts', 'exhaust', 'powerTurbine', 'ggTurbine'],
      },
      {
        id: 'combustor',
        name: 'Sfila il combustore',
        base: "Tolte le turbine, la camera di combustione scorre all'indietro: dentro si vedono l'anello forato dove brucia la fiamma e gli iniettori.",
        expert:
          "La cassa esterna regge la pressione; il tubo di fiamma interno (liner) contiene la combustione ed è raffreddato dall'aria che entra dai suoi fori. " +
          "Con il combustore escono anche il collettore del carburante, gli iniettori e le candele di accensione.",
        parts: ['combustor'],
      },
      {
        id: 'coldSection',
        name: "Sfila compressore e presa d'aria",
        base:
          "Restano i pezzi \"freddi\": il compressore, con le sue palette e la girante, e la presa d'aria con la griglia. Anche loro si sfilano all'indietro, lungo gli alberi.",
        expert:
          "In molti motori la cassa del compressore assiale è divisa in due metà, che si aprono per ispezionare e sostituire le palette. " +
          "Qui, per semplicità, il compressore esce in un pezzo solo, con il suo rotore e il diffusore.",
        parts: ['compressor', 'intake'],
      },
      {
        id: 'shafts',
        name: 'Sfila gli alberi',
        base:
          "Si svitano i bulloni del giunto e si sfilano i due alberi coassiali del motore; per ultimo esce l'albero di trasmissione con la ruota libera. " +
          "Sul tetto restano la struttura e il riduttore principale con l'albero del rotore.",
        expert:
          "Il giunto flessibile separa l'albero di potenza del motore dall'albero ad alta velocità verso il riduttore. La ruota libera è su questo tratto: " +
          "trasmette la coppia solo dal motore al rotore, e in autorotazione lascia girare il rotore a motore fermo.",
        parts: ['driveBolts', 'shafts', 'driveShaft'],
      },
    ],
  },

  camera: { position: [-3.4, 2.7, 6.7], target: [0.05, -0.2, 0] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.x = OX;
    root.add(model);
    const core = new THREE.Group(); // engine-axis frame
    core.position.y = YI;
    model.add(core);

    // ---------- Materials ----------
    const mCase = cut(MAT.steel());
    const mCaseDark = cut(MAT.darkSteel());
    const mInlet = cut(MAT.paint(0xc9cdd2, 0.45));
    const mMgb = cut(MAT.paint(0x8a9198, 0.55));
    const mStator = cut(MAT.darkSteel());
    const mTi = MAT.titanium();
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
    const mScreen = MAT.darkSteel();
    const mBlade = MAT.paint(0x30343a, 0.5);
    const mCutFace = MAT.paint(0xc0392b, 0.6);
    const mHub = MAT.titanium();
    const mAct = MAT.aluminum();
    const mLink = MAT.chrome();
    const mTailBlade = MAT.paint(0xe8e8e4, 0.45);
    const mTailStripe = MAT.paint(0xc0392b, 0.5);
    const mCoupling = MAT.brass();
    const mBolt = cut(MAT.darkSteel(), { cap: false });
    const mHole = cut(MAT.paint(0x1b1612, 0.9), { cap: false });
    const mFrame = cut(MAT.paint(0x6f7b6c, 0.6));
    const mFin = cut(MAT.paint(0xd9dcd8, 0.45));
    const mPipe = cut(MAT.copper());
    const mHarness = cut(MAT.paint(0x3a3226, 0.8));
    const mAcc = cut(MAT.aluminum());
    const mAccDark = cut(MAT.castIron());
    const mBronze = MAT.brass();
    mBronze.color.set(0xb0793c);

    const ggRot = [];
    const ptRot = [];
    const hotMeshes = [];

    // ---------- Intake plenum + screen ----------
    const intake = new THREE.Group();
    intake.add(new THREE.Mesh(latheX([[0.07, -0.435], [0.45, -0.435], [0.45, -0.42], [0.07, -0.42], [0.07, -0.435]], 72), mInlet));
    intake.add(new THREE.Mesh(wall([[0.45, -0.2], [0.37, -0.175], [0.31, -0.13], [0.285, -0.07], [0.28, -0.04]], 0.012, 72), mInlet));
    intake.add(new THREE.Mesh(flangeX(-0.045, 0.285, 0.33, 0.02), mInlet));
    intake.add(boltSet(mBolt, 0.009, 0.01).ringX(-0.056, 0.31, 20, -1).mesh());
    intake.add(boltSet(mBolt, 0.009, 0.01).ringX(-0.437, 0.42, 30, -1).mesh());
    intake.add(new THREE.Mesh(wall([[0.07, -0.42], [0.085, -0.32], [0.1, -0.2], [0.11, -0.08]], -0.01, 48), mCaseDark));
    {
      const bar = new THREE.BoxGeometry(0.22, 0.005, 0.005).translate(-0.31, 0, 0);
      intake.add(mergeRing(bar, mScreen, 90, 0.45));
      for (const x of [-0.41, -0.31, -0.21]) {
        const hoop = new THREE.Mesh(new THREE.TorusGeometry(0.45, 0.005, 4, 90).rotateY(Math.PI / 2), mScreen);
        hoop.position.x = x;
        intake.add(hoop);
      }
    }
    core.add(
      ctx.part(intake, {
        id: 'intake',
        name: 'Presa d\'aria radiale con griglia',
        base:
          "L'aria entra da tutto il giro attraverso una griglia metallica, che ferma sassi, foglie e oggetti sollevati dal rotore, poi curva verso il compressore. " +
          "Negli elicotteri questo è importante: atterrano su prati, sabbia e neve.",
        expert:
          "Plenum d'ingresso radiale con schermo anti-FOD; molti elicotteri aggiungono filtri a barriera (IBF) o separatori inerziali di particelle (il T700 ne ha uno integrato) " +
          "per l'erosione da sabbia, e la protezione antighiaccio. Le perdite di pressione in ingresso costano direttamente potenza all'albero.",
        explode: [-0.45, 0.25, 0],
      }),
    );

    // ---------- Compressor: 3 axial stages + centrifugal (GG spool) ----------
    const axHub = (x) => lerp(0.11, 0.14, (x + 0.08) / 0.34);
    const axTip = (x) => lerp(0.265, 0.225, (x + 0.08) / 0.34);
    const comp = new THREE.Group();
    const compRotor = new THREE.Group();
    comp.add(compRotor);
    ggRot.push(compRotor);
    compRotor.add(new THREE.Mesh(latheX([[0.06, -0.07], [0.11, -0.07], [0.14, 0.26], [0.06, 0.26], [0.06, -0.07]], 48), mDrum));
    [-0.01, 0.09, 0.19].forEach((x, i) => {
      compRotor.add(rotorRow(x, axHub(x), axTip(x) - 0.004, 24 + 4 * i, 0.05, mTi, { twist: 0.5, stagger: 0.6, thick: 0.009 }));
    });
    for (const [x, a] of [[-0.055, 0.2], [0.04, 0.45], [0.14, 0.45], [0.235, 0.5]]) {
      comp.add(statorRow(x, axHub(x) + 0.004, axTip(x) + 0.006, 30, 0.04, mStator, a, 0.008));
    }
    comp.add(new THREE.Mesh(wall([[0.28, -0.05], [0.272, 0.05], [0.252, 0.2], [0.238, 0.26], [0.236, 0.27]], 0.012, 72), mCase));
    const impHub = [[0.14, 0.27], [0.15, 0.31], [0.18, 0.36], [0.24, 0.4], [0.31, 0.418], [0.36, 0.423]];
    const impShroud = [[0.222, 0.27], [0.226, 0.31], [0.25, 0.355], [0.3, 0.39], [0.36, 0.398]];
    compRotor.add(new THREE.Mesh(latheX([[0.06, 0.27], ...impHub, [0.362, 0.44], [0.25, 0.45], [0.12, 0.46], [0.06, 0.46], [0.06, 0.27]], 72), mDrum));
    compRotor.add(impeller(impHub, impShroud, 13, ds(MAT.titanium()), { wrap: 0.75 }));
    compRotor.add(impeller(impHub, impShroud, 13, ds(MAT.titanium()), { wrap: 0.75, from: 0.42, n: 10 }).rotateX(TAU / 26));
    comp.add(new THREE.Mesh(wall([[0.236, 0.27], [0.24, 0.31], [0.263, 0.352], [0.31, 0.384], [0.37, 0.392], [0.44, 0.392], [0.47, 0.41], [0.478, 0.45]], 0.012, 72), mCase));
    comp.add(new THREE.Mesh(wall([[0.37, 0.447], [0.42, 0.45], [0.432, 0.49]], 0.01, 72), mCase));
    comp.add(new THREE.Mesh(flangeX(0.437, 0.488, 0.527, 0.02), mCase)); // mates with the combustor flange
    {
      const vane = new THREE.BoxGeometry(0.05, 0.065, 0.007).translate(0, 0.0325, 0).rotateX(-0.75);
      const vanes = mergeRing(vane, mStator, 18, 0.372);
      vanes.position.x = 0.42;
      comp.add(vanes);
    }
    core.add(
      ctx.part(comp, {
        id: 'compressor',
        name: 'Compressore assiale-centrifugo',
        base:
          "Tre file di palette che girano (alternate a palette ferme) e poi una girante centrifuga a campana, che lancia l'aria verso l'esterno. " +
          "Insieme schiacciano l'aria più di 15 volte. Sono tutte sullo stesso albero e girano a oltre 40.000 giri al minuto.",
        expert:
          "Compressore combinato assiale + centrifugo su un unico albero (T700: 5 assiali + 1 centrifugo; Arriel: 1–2 assiali + 1 centrifugo), con statori a calettamento " +
          "variabile e valvole di spillamento antistallo. Rapporto ≈ 15–18:1; il centrifugo finale è compatto e tollera bene erosione e piccole portate.",
        explode: [-0.2, 0, 0],
      }),
    );

    // ---------- Straight annular combustor ----------
    const comb = new THREE.Group();
    comb.add(new THREE.Mesh(wall([[0.478, 0.45], [0.478, 0.94], [0.45, 0.99], [0.38, 1.02], [0.3, 1.03]], 0.012, 72), mCase));
    comb.add(new THREE.Mesh(flangeX(0.46, 0.488, 0.53, 0.026), mCase));
    comb.add(holesX(mHole, [[0.66, 0.399, 26], [0.78, 0.399, 26, 0.12], [0.7, 0.209, 16], [0.82, 0.209, 16, 0.2]], 0.01));
    for (const a of [300 * DEG, 60 * DEG]) {
      const plug = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.024, 0.08, 12), mAccDark);
      plug.position.set(0.7, 0.5 * Math.cos(a), 0.5 * Math.sin(a));
      plug.rotation.x = a;
      comb.add(plug);
    }
    comb.add(new THREE.Mesh(wall([[0.17, 0.48], [0.17, 0.9], [0.15, 0.95]], -0.01, 64), mCaseDark));
    comb.add(
      new THREE.Mesh(
        wall([[0.285, 0.965], [0.36, 0.92], [0.39, 0.85], [0.39, 0.62], [0.37, 0.57], [0.33, 0.55], [0.26, 0.55], [0.22, 0.57], [0.2, 0.62], [0.2, 0.85], [0.17, 0.92], [0.15, 0.965]], 0.008, 72),
        mLiner,
      ),
    );
    {
      const manifold = new THREE.Mesh(new THREE.TorusGeometry(0.52, 0.011, 6, 72).rotateY(Math.PI / 2), mFuel);
      manifold.position.x = 0.5;
      comb.add(manifold);
      const stem = cylBetweenGeo(new THREE.Vector3(0.5, 0.52, 0), new THREE.Vector3(0.535, 0.31, 0), 0.008, 6);
      comb.add(mergeRing(stem, mInjector, 12, 0));
      const cup = new THREE.CylinderGeometry(0.02, 0.024, 0.03, 10).rotateZ(Math.PI / 2).translate(0.548, 0, 0);
      comb.add(mergeRing(cup, mInjector, 12, 0.295));
    }
    // flame kept thin and translucent so liner, swirlers and dilution holes stay readable
    const flamePts = [
      [0.295, 0.57], [0.32, 0.61], [0.335, 0.7], [0.33, 0.8], [0.31, 0.88], [0.295, 0.9], [0.28, 0.88], [0.262, 0.8], [0.256, 0.7], [0.268, 0.61], [0.295, 0.57],
    ];
    const flameCore = new THREE.Mesh(latheX(flamePts, 72), ds(MAT.glow(0xff7a2a, 1, 0.5)));
    const flameHalo = new THREE.Mesh(latheX(flamePts.map(([r, x]) => [0.295 + (r - 0.295) * 1.5, x]), 72), MAT.gas(0xff5a1f, 0.12));
    const jetGeo = new THREE.ConeGeometry(0.026, 0.16, 8, 1, true).rotateZ(-Math.PI / 2).translate(0.08, 0, 0);
    const flameJets = mergeRing(jetGeo, MAT.gas(0xffa050, 0.25), 12, 0.295);
    flameJets.position.x = 0.565;
    comb.add(flameCore, flameHalo, flameJets);
    core.add(
      ctx.part(comb, {
        id: 'combustor',
        name: 'Camera di combustione anulare',
        base:
          "Un anello dove il cherosene, spruzzato dagli iniettori, brucia di continuo nell'aria compressa. Quando il pilota alza il collettivo, " +
          "il computer del motore manda più carburante e la fiamma si ravviva.",
        expert:
          "Combustore anulare diritto (T700) con 12 iniettori a swirler; l'Arriel usa invece un combustore a flusso invertito con iniezione centrifuga. " +
          "Combustione a pressione quasi costante, T4 ≈ 1.200–1.400 °C. Il FADEC dosa il carburante per tenere Np/Nr al 100% e anticipa il carico leggendo la posizione del collettivo.",
        explode: [0.1, 0.95, 0],
      }),
    );

    // ---------- Gas-generator turbine (2 stages) ----------
    const ggt = new THREE.Group();
    const ggtRotor = new THREE.Group();
    ggt.add(ggtRotor);
    ggRot.push(ggtRotor);
    for (const [x, rh, rt, n] of [[1.03, 0.152, 0.275, 40], [1.145, 0.155, 0.285, 42]]) {
      const row = rotorRow(x, rh, rt - 0.004, n, 0.04, mHot, { twist: -0.25, stagger: -0.75, thick: 0.012 });
      ggtRotor.add(row, disc(x, 0.07, rh, 0.045, mDrum));
      hotMeshes.push(row);
    }
    for (const [x, rh, rt] of [[0.975, 0.15, 0.274], [1.088, 0.153, 0.282]]) {
      const ngv = statorRow(x, rh, rt, 24, 0.042, mHotStator, 0.9, 0.012);
      ggt.add(ngv);
      hotMeshes.push(ngv);
    }
    ggt.add(new THREE.Mesh(wall([[0.282, 0.96], [0.287, 1.08], [0.296, 1.18]], 0.012, 72), mCaseDark));
    ggt.add(new THREE.Mesh(flangeX(1.175, 0.305, 0.345, 0.022), mCaseDark));
    core.add(
      ctx.part(ggt, {
        id: 'ggTurbine',
        name: 'Turbina del generatore di gas',
        base:
          "Due file di palette colpite dai gas roventi. Girano velocissime e, con un albero cavo, fanno girare il compressore: servono a tenere in moto il motore.",
        expert:
          "Turbina a 2 stadi raffreddata (pale in superlega monocristallina, barriera termica), mossa dai gas a ~1.200–1.400 °C, solidale al compressore (Ng ≈ 44.700 giri/min " +
          "sul T700). Il pilota la sorveglia con Ng e con la temperatura dei gas (TOT/MGT), i due limiti principali insieme alla coppia.",
        explode: [0.35, 0, 0],
      }),
    );

    // ---------- Free power turbine (2 stages) ----------
    const pt = new THREE.Group();
    const ptRotor = new THREE.Group();
    pt.add(ptRotor);
    ptRot.push(ptRotor);
    for (const [x, rh, rt, n] of [[1.265, 0.16, 0.3, 44], [1.38, 0.168, 0.325, 46]]) {
      ptRotor.add(rotorRow(x, rh, rt - 0.004, n, 0.042, mPt, { twist: -0.3, stagger: -0.6, thick: 0.011 }));
      ptRotor.add(disc(x, 0.035, rh, 0.045, mDrum));
    }
    for (const [x, rh, rt] of [[1.21, 0.158, 0.296], [1.325, 0.164, 0.316]]) {
      pt.add(statorRow(x, rh, rt, 28, 0.042, mStator, 0.75, 0.011));
    }
    pt.add(new THREE.Mesh(wall([[0.296, 1.18], [0.31, 1.27], [0.33, 1.38], [0.342, 1.44]], 0.012, 72), mCase));
    pt.add(new THREE.Mesh(flangeX(1.435, 0.352, 0.395, 0.022), mCase));
    core.add(
      ctx.part(pt, {
        id: 'powerTurbine',
        name: 'Turbina libera (di potenza)',
        base:
          "Due file di palette più grandi, mosse dai gas che escono dal generatore di gas ma non collegate al compressore. Fanno girare l'albero più interno, " +
          "che porta la potenza davanti, verso il riduttore e il rotore.",
        expert:
          "Turbina di potenza a 2 stadi a ≈ 20.900 giri/min (T700), mantenuta al 100% Np dal FADEC perché è collegata rigidamente al rotore attraverso il riduttore. " +
          "Eroga ~1.400 kW sul T700-701D; in caso di avaria di un motore l'altro sale ai regimi OEI (30 s, 2 min, continuo) per alcune decine di secondi o minuti.",
        explode: [0.7, 0, 0],
      }),
    );

    // ---------- Module flange bolts (their own part: the wrench takes them out) ----------
    core.add(
      ctx.part(
        boltSet(mBolt, 0.0095, 0.011, screwGeo(0.0095, 0.011, 0.058))
          .ringX(0.427, 0.508, 30, -1) // compressor / combustor
          .ringX(1.164, 0.327, 24, -1) // gas-generator turbine / power turbine
          .ringX(1.424, 0.375, 26, -1) // power turbine / exhaust
          .mesh(),
        {
          id: 'caseBolts',
          name: 'Bulloni delle flange del motore',
          base:
            "Tre corone di bulloni che tengono uniti i \"pezzi\" del motore: compressore e combustore, le due turbine, turbina libera e scarico. " +
            "Il motore è fatto a moduli, che si separano proprio su queste flange.",
          expert:
            "Flange circonferenziali imbullonate tra le casse dei moduli. La costruzione modulare (sul T700: modulo accessori, sezione fredda, sezione calda e turbina di potenza) " +
            "permette di sostituire un modulo senza smontare tutto il motore; nella sezione calda i bulloni sono in leghe resistenti al calore.",
          explode: [0.35, 0, 0],
        },
      ),
    );

    // ---------- Shafts (PT inside GG) ----------
    const shafts = new THREE.Group();
    const ptShaft = new THREE.Mesh(latheX([[0.022, -0.45], [0.035, -0.45], [0.035, 1.39], [0.022, 1.39]], 24), mShaft);
    const ggShaft = new THREE.Mesh(latheX([[0.055, -0.07], [0.068, -0.07], [0.068, 1.15], [0.055, 1.15]], 28), mShaft2);
    shafts.add(ptShaft, ggShaft);
    ptRot.push(ptShaft);
    ggRot.push(ggShaft);
    core.add(
      ctx.part(shafts, {
        id: 'shafts',
        name: 'Alberi coassiali',
        base:
          "Due alberi uno dentro l'altro. Quello esterno collega la turbina del generatore di gas al compressore; quello interno, più lungo, porta la potenza della turbina libera " +
          "fino all'uscita anteriore del motore.",
        expert:
          "Albero del generatore di gas cavo (Ng) e albero di potenza interno (Np) su cuscinetti a sfere e a rulli lubrificati. L'uscita anteriore \"passante\" (T700, PW210) " +
          "porta l'albero di potenza attraverso il compressore; l'Arriel ha invece un riduttore d'uscita integrato a ~6.000 giri/min.",
        explode: [0, -0.5, 0],
      }),
    );

    // ---------- Exhaust ----------
    const exh = new THREE.Group();
    exh.add(
      new THREE.Mesh(
        loftWall([
          { p: [1.44, 0], hw: 0.354, hh: 0.354 },
          { p: [1.65, 0.04], hw: 0.34, hh: 0.34 },
          { p: [1.82, 0.2], hw: 0.31, hh: 0.31 },
          { p: [1.93, 0.4], hw: 0.285, hh: 0.285 },
          { p: [2.0, 0.56], hw: 0.27, hh: 0.27 },
        ], 0.014, 30, 48),
        mExhaust,
      ),
    );
    exh.add(new THREE.Mesh(latheX([[0.17, 1.41], [0.16, 1.5], [0.1, 1.6], [0.0, 1.66]], 48), ds(MAT.darkSteel())));
    exh.add(statorRow(1.46, 0.17, 0.35, 6, 0.07, mExhaust, 0, 0.018));
    exh.add(new THREE.Mesh(flangeX(1.457, 0.352, 0.395, 0.022), mExhaust));
    {
      const lip = new THREE.Mesh(new THREE.TorusGeometry(0.262, 0.013, 8, 48), mExhaust);
      lip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(0.07, 0.16, 0).normalize());
      lip.position.set(2.0, 0.56, 0);
      exh.add(lip);
    }
    core.add(
      ctx.part(exh, {
        id: 'exhaust',
        name: 'Scarico',
        base: "Il tubo curvo da cui escono i gas, rivolto verso l'alto e indietro per non scaldare la fusoliera e non disturbare il rotore di coda.",
        expert:
          "Condotto di scarico divergente (diffusore) che recupera pressione e abbassa la contropressione sulla turbina di potenza. Spinta residua trascurabile; " +
          "sui militari si aggiungono soppressori infrarossi che miscelano i gas con aria fresca.",
        explode: [1.0, 0.35, 0],
      }),
    );

    // ---------- Drive shaft engine -> main gearbox ----------
    const drive = new THREE.Group();
    const driveSpin = new THREE.Group();
    driveSpin.position.y = YI;
    drive.add(driveSpin);
    const X_CPL = -0.482; // engine-side coupling
    {
      const x0 = XM + 0.36;
      const x1 = -0.45;
      driveSpin.add(new THREE.Mesh(latheX([[0.03, x0], [0.045, x0], [0.045, x1], [0.03, x1]], 24), mShaft));
      // flexible couplings: inside the gearbox nose and, wider, bolted to the engine output
      for (const [x, r] of [[x0 + 0.06, 0.1], [X_CPL, 0.125]]) {
        driveSpin.add(new THREE.Mesh(latheX([[0.045, x - 0.03], [r, x - 0.025], [r, x - 0.008], [r * 0.6, x], [r, x + 0.008], [r, x + 0.025], [0.045, x + 0.03]], 32), mCoupling));
      }
      // freewheel unit housing, half inside the gearbox nose
      driveSpin.add(new THREE.Mesh(latheX([[0.045, -0.62], [0.095, -0.6], [0.1, -0.565], [0.095, -0.54], [0.045, -0.525]], 32), mGear2));
    }
    const driveBolts = boltSet(mBolt, 0.012, 0.014, screwGeo(0.012, 0.014, 0.05)).ringX(X_CPL - 0.025, 0.103, 6, -1).mesh();
    driveBolts.position.y = YI;
    model.add(
      ctx.part(driveBolts, {
        id: 'driveBolts',
        name: 'Bulloni del giunto di trasmissione',
        base:
          "I bulloni che uniscono l'albero di trasmissione all'uscita del motore, attraverso un giunto flessibile che sopporta piccoli disallineamenti.",
        expert:
          "Bulloni del giunto flessibile (a lamelle o a membrana) tra l'uscita del motore e l'albero ad alta velocità verso il riduttore. Il giunto trasmette la coppia " +
          "a ≈ 20.000 giri/min e assorbe i piccoli disallineamenti dovuti alle dilatazioni termiche e ai movimenti del riduttore sui suoi attacchi.",
        explode: [-0.15, -0.5, 0],
      }),
    );
    model.add(
      ctx.part(drive, {
        id: 'driveShaft',
        name: 'Albero di trasmissione e ruota libera',
        base:
          "L'albero che porta la rotazione dal motore al riduttore. Ha giunti flessibili alle estremità, per assorbire piccoli spostamenti, e una ruota libera: " +
          "se il motore si ferma, il rotore può continuare a girare da solo (autorotazione) senza trascinarlo.",
        expert:
          "Albero ad alta velocità (≈ 20.000 giri/min) con giunti a lamelle o a membrana e ruota libera (sprag clutch) che trasmette coppia solo dal motore al rotore. " +
          "Permette l'autorotazione e lo spegnimento di un motore in volo sui bimotori; il torsiometro sull'uscita fornisce l'indicazione di coppia (TQ).",
        explode: [-0.15, -0.5, 0],
      }),
    );

    // ---------- Main gearbox ----------
    const mgb = new THREE.Group();
    // housing: sump, main case and top cover as hollow castings with bolted flanges
    const yLathe = (pts, mat = mMgb, seg = 72) => {
      const m = new THREE.Mesh(lathe(pts, seg), mat);
      m.position.x = XM;
      return m;
    };
    const shellY = (outer, t) => [...outer, ...outer.map(([r, y]) => [r - t, y]).reverse(), outer[0]];
    mgb.add(yLathe([[0.05, -1.29], [0.27, -1.29], [0.32, -1.22], [0.33, -0.975], [0.308, -0.975], [0.3, -1.21], [0.258, -1.268], [0.05, -1.268], [0.05, -1.29]]));
    mgb.add(yLathe(shellY([[0.33, -0.955], [0.47, -0.8], [0.49, -0.62], [0.47, -0.44], [0.4, -0.385]], 0.022)));
    mgb.add(yLathe(shellY([[0.4, -0.365], [0.385, -0.16], [0.3, -0.07], [0.15, -0.03], [0.1, 0.06]], 0.02)));
    mgb.add(yLathe([[0.3, -0.988], [0.372, -0.988], [0.372, -0.955], [0.3, -0.955], [0.3, -0.988]]));
    mgb.add(yLathe([[0.38, -0.392], [0.452, -0.392], [0.452, -0.364], [0.38, -0.364], [0.38, -0.392]]));
    {
      const bolts = boltSet(mBolt, 0.011, 0.012);
      bolts.ringY(-0.955, 0.352, 24, 1, XM, 0, 0.1);
      bolts.ringY(-0.364, 0.43, 28, 1, XM, 0, 0.05);
      // input nose (towards the engine) and tail take-off nose
      const nose = new THREE.Group();
      nose.position.y = YI;
      nose.add(new THREE.Mesh(wall([[0.13, XM + 0.42], [0.11, XM + 0.52], [0.1, XM + 0.62]], 0.016, 48), mMgb));
      nose.add(new THREE.Mesh(flangeX(XM + 0.62, 0.1, 0.15, 0.022), mMgb));
      nose.add(new THREE.Mesh(latheX([[0.036, XM + 0.38], [0.065, XM + 0.38], [0.065, XM + 0.43], [0.036, XM + 0.43], [0.036, XM + 0.38]], 24), mBronze));
      const tailNose = new THREE.Group();
      tailNose.position.y = YT;
      tailNose.add(new THREE.Mesh(wall([[0.09, XM + 0.27], [0.075, XM + 0.36], [0.07, XM + 0.42]], 0.014, 40), mMgb));
      tailNose.add(new THREE.Mesh(flangeX(XM + 0.42, 0.07, 0.11, 0.02), mMgb));
      mgb.add(nose, tailNose);
      bolts.ringX(XM + 0.633, 0.127, 10, 1, YI, 0, 0.3);
      bolts.ringX(XM + 0.432, 0.092, 8, 1, YT, 0, 0.2);
      mgb.add(bolts.mesh());
      // cast ribs around the main case
      const rib = new RoundedBoxGeometry(0.07, 0.36, 0.024, 2, 0.008);
      for (let i = 0; i < 8; i++) {
        const a = ((i + 0.5) / 8) * TAU;
        const r = new THREE.Mesh(rib, mMgb);
        r.position.set(XM + 0.47 * Math.cos(a), -0.63, -0.47 * Math.sin(a));
        r.rotation.y = a;
        mgb.add(r);
      }
      // oil sight glass, filler, chip detector, bearings
      const glass = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.02, 20).rotateX(Math.PI / 2), MAT.paint(0x7a4a12, 0.15));
      glass.position.set(XM, -1.12, -0.318);
      const filler = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.05, 16).rotateZ(Math.PI / 2), mAccDark);
      filler.position.set(XM - 0.47, -0.86, 0);
      const chip = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.05, 12), mAccDark);
      chip.position.set(XM - 0.12, -1.31, 0.08);
      mgb.add(glass, filler, chip);
      mgb.add(yLathe([[0.052, -0.01], [0.08, -0.01], [0.08, 0.045], [0.052, 0.045], [0.052, -0.01]], mBronze, 32));
      mgb.add(yLathe([[0.04, -1.265], [0.065, -1.265], [0.065, -1.22], [0.04, -1.22], [0.04, -1.265]], mBronze, 32));
    }
    // input pinion (axis X) and crown wheel (axis Y), apexes at (XM, YI)
    const pinion = new THREE.Mesh(bevelGeo(NP_IN, AW, 0.08, Math.PI / 2 - GW, 0.03).rotateZ(-Math.PI / 2), mGear);
    pinion.position.set(XM, YI, 0);
    const pinShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.16, 16).rotateZ(Math.PI / 2), mShaft);
    pinShaft.position.x = 0.36;
    pinion.add(pinShaft);
    const wheel = new THREE.Group();
    wheel.position.set(XM, 0, 0);
    {
      const crown = new THREE.Mesh(bevelGeo(NW, AW, 0.08, GW, 0.04), mGear);
      crown.position.y = YI;
      const tailGear = new THREE.Mesh(bevelGeo(NT, AT, 0.05, GT, 0.04), mGear2);
      tailGear.position.y = YT;
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, YS - YT + 0.04, 20), mShaft2);
      shaft.position.y = (YS + YT) / 2;
      wheel.add(crown, tailGear, shaft);
    }
    // planetary: sun on the wheel shaft, fixed ring, carrier -> mast
    const ang = (i) => (i / 4) * TAU;
    const planetPh = [0, 1, 2, 3].map((i) => {
      const step = TAU / NPL;
      const d = ((((Math.PI / NR - ang(i)) * NR) / NPL) % step + step) % step;
      return ang(i) + d;
    });
    const sunPh = meshPhase(planetPh[0], NPL, NS, Math.PI);
    const sun = new THREE.Mesh(spurY(NS, NS * PM, 0.06), mGear);
    sun.position.set(XM, YS, 0);
    const ring = new THREE.Mesh(ringGearY(NR, NR * PM, 0.32, 0.06), mGear2);
    ring.position.set(XM, YS, 0);
    const carrier = new THREE.Group();
    carrier.position.set(XM, YS, 0);
    const planets = [];
    {
      const plate = new THREE.Mesh(new THREE.CylinderGeometry(0.235, 0.235, 0.02, 48), mGear2);
      plate.position.y = 0.045;
      carrier.add(plate);
      const pGeo = spurY(NPL, NPL * PM, 0.055, 0.02);
      for (let i = 0; i < 4; i++) {
        const p = new THREE.Mesh(pGeo, mGear);
        p.position.set(RC * Math.cos(ang(i)), 0, -RC * Math.sin(ang(i)));
        const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.09, 12), mShaft2);
        pin.position.copy(p.position).setY(0.015);
        carrier.add(p, pin);
        planets.push(p);
      }
    }
    // tail take-off pinion (axis X)
    const tailPinion = new THREE.Mesh(bevelGeo(NTP, AT, 0.05, Math.PI / 2 - GT, 0.025).rotateZ(-Math.PI / 2), mGear);
    tailPinion.position.set(XM, YT, 0);
    mgb.add(pinion, wheel, sun, ring, carrier, tailPinion);
    model.add(
      ctx.part(mgb, {
        id: 'mainGearbox',
        name: 'Riduttore principale (MGB)',
        base:
          "Il grande blocco sotto il rotore. Dentro: un pignone conico che fa girare una corona conica (il moto passa da orizzontale a verticale) e poi un ingranaggio " +
          "planetario, con quattro satelliti che rotolano dentro una corona fissa. In basso un altro ingranaggio conico preleva il moto per il rotore di coda.",
        expert:
          "Coppia conica 17/60 (≈ 3,5:1) + epicicloidale a sole conduttore, 4 satelliti e corona fissa (22/24/70, ≈ 4,2:1): ≈ 15:1 nel modello. " +
          "Negli MGB reali uno stadio cilindrico d'ingresso per ciascun motore e talvolta un secondo epicicloidale portano il totale a ~80:1 (UH-60: da ~20.900 a ~258 giri/min). " +
          "L'albero rotore trasmette oltre 100 kNm e tutta la portanza; lubrificazione forzata con radiatore, scatola in lega di magnesio o alluminio.",
        explode: [-0.75, -0.15, 0],
      }),
    );

    // ---------- Swashplate (stationary + rotating) with servos and pitch links ----------
    const swash = new THREE.Group();
    swash.position.x = XM;
    const swStat = new THREE.Group();
    const swRot = new THREE.Group();
    swash.add(swStat, swRot);
    swStat.add(new THREE.Mesh(latheX([[0.075, -0.02], [0.23, -0.02], [0.235, 0], [0.23, 0.02], [0.075, 0.02]], 48).rotateZ(Math.PI / 2), mAct));
    swStat.add(new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.16, 24), mAct));
    swRot.add(new THREE.Mesh(latheX([[0.09, -0.015], [0.27, -0.015], [0.3, 0], [0.27, 0.015], [0.09, 0.015]], 48).rotateZ(Math.PI / 2), mHub));
    {
      const lug = new THREE.SphereGeometry(0.022, 10, 8);
      for (let i = 0; i < NBL; i++) {
        const a = (i / NBL) * TAU;
        const l = new THREE.Mesh(lug, mLink);
        l.position.set(HR * Math.cos(a) - HL * Math.sin(a), 0.015, -HR * Math.sin(a) - HL * Math.cos(a));
        swRot.add(l);
      }
    }
    const linkSpin = new THREE.Group();
    swash.add(linkSpin);
    const links = [];
    const linkGeo = new THREE.CylinderGeometry(0.01, 0.01, 1, 8);
    for (let i = 0; i < NBL; i++) {
      const l = new THREE.Mesh(linkGeo, mLink);
      linkSpin.add(l);
      links.push(l);
    }
    const servoRods = [];
    for (const deg of [90, 210, 330]) {
      const a = deg * DEG;
      const x = 0.2 * Math.cos(a);
      const z = -0.2 * Math.sin(a);
      const body = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.22, 16), mAct);
      body.position.set(x, 0.05, z);
      const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 1, 10), mLink);
      swash.add(body, rod);
      servoRods.push({ rod, x, z });
    }
    model.add(
      ctx.part(swash, {
        id: 'swashplate',
        name: 'Piatto oscillante e comandi',
        base:
          "Due anelli attorno all'albero del rotore: quello sotto non gira ed è mosso da tre attuatori; quello sopra gira con il rotore. Con il collettivo salgono insieme " +
          "e, attraverso le astine argentate, spingono in su le leve di tutte le pale, che aumentano l'angolo.",
        expert:
          "Piatto fisso su snodo sferico (uniball) scorrevole sul tubo guida, comandato da 3 servoattuatori idraulici in doppio corpo; piatto rotante su cuscinetto, " +
          "trascinato dal compasso. Le aste di comando (pitch link) collegano il piatto rotante alle squadrette di passo. Salita = collettivo, inclinazione = ciclico; " +
          "nel modello la corsa verticale è ≈ 5 cm per 14° di passo con una squadretta da 20 cm.",
        explode: [0, 0.6, 0],
      }),
    );

    // ---------- Rotor head: mast, hub, grips, horns, blade roots (cut) ----------
    const head = new THREE.Group();
    const mast = new THREE.Group();
    mast.position.x = XM;
    head.add(mast);
    mast.add(new THREE.Mesh(lathe([[0.05, YH - 0.04], [0.16, YH - 0.035], [0.17, YH], [0.16, YH + 0.035], [0.05, YH + 0.04]], 48), mHub));
    // mast (stays in the gearbox) with its threaded end, and the nut that holds the hub on it
    const mastShaft = new THREE.Mesh(lathe([[0.03, YS + 0.05], [0.052, YS + 0.05], [0.052, YH + 0.04], [0.036, YH + 0.045], [0.036, YH + 0.1], [0.03, YH + 0.1]], 32), mShaft2);
    mastShaft.position.x = XM;
    const mastNut = boltSet(mBolt, 0, 0.07, capNutGeo(0.07, 0.05)).add(new THREE.Vector3(0, YH + 0.04, 0), _up).mesh();
    mastNut.position.x = XM;
    model.add(
      ctx.part(mastShaft, {
        id: 'rotorMast',
        name: 'Albero del rotore (mast)',
        base:
          "L'albero verticale che esce dal riduttore e porta in cima il mozzo con le pale. Fa due lavori insieme: fa girare il rotore e regge l'elicottero, " +
          "che in volo resta letteralmente appeso a lui.",
        expert:
          "Albero rotore cavo in acciaio ad alta resistenza, trascinato dal portasatelliti dello stadio epicicloidale e sostenuto dai cuscinetti nel coperchio del riduttore. " +
          "Trasmette la coppia, la portanza e i momenti del mozzo; il mozzo è calettato su scanalature e bloccato in cima da un dado. Lungo il tratto esterno scorre il piatto oscillante.",
        explode: [0, 0.9, 0],
      }),
      ctx.part(mastNut, {
        id: 'mastNut',
        name: "Dado dell'albero rotore",
        base:
          "Il grosso dado in cima all'albero: tiene il mozzo, con tutte le pale, bloccato sull'albero del rotore. È un pezzo solo, ma è tra i più controllati di tutto l'elicottero.",
        expert:
          "Dado di ritegno del mozzo (mast nut): blocca assialmente il mozzo sulle scanalature dell'albero rotore ed è assicurato contro lo svitamento da un fermo meccanico. " +
          "Dove un unico dado trattiene tutto il rotore, i piloti lo chiamano scherzosamente \"Jesus nut\"; molti rotori moderni usano invece una corona di bulloni tra mozzo e albero.",
        explode: [0, 1.45, 0],
      }),
    );
    const BL = 1.25;
    const BR0 = 0.42;
    const RT_TWIST = 0.17;
    const bladeGeo = airfoilGeo(BL, 0.28, 0.045, {
      segL: 14,
      plan: (u) => (u < 0.08 ? 0.75 + u * 3.1 : 1),
      thk: (u) => (u < 0.08 ? 1.6 - u * 7.5 : 1),
      twist: (u) => RT_TWIST * (0.75 - u * 0.5),
      map: (r, c, t) => [r, t, c],
    });
    const capGeo = airfoilGeo(0.012, 0.28, 0.045, {
      segL: 1,
      plan: () => 1.01,
      thk: () => 1.08,
      twist: () => RT_TWIST * (0.75 - 0.5),
      map: (r, c, t) => [r + BL, t, c],
    });
    const gripGeo = new THREE.CylinderGeometry(0.042, 0.05, 0.32, 18).rotateZ(Math.PI / 2).translate(0.28, 0, 0);
    const hornGeo = new THREE.BoxGeometry(0.03, 0.022, HL).translate(HR, 0, -HL / 2);
    const hornBall = new THREE.SphereGeometry(0.018, 10, 8).translate(HR, 0, -HL);
    const armGeo = new THREE.BoxGeometry(0.16, 0.05, 0.08).translate(0.16, 0, 0);
    const pitchGroups = [];
    for (let i = 0; i < NBL; i++) {
      const az = new THREE.Group();
      az.position.y = YH;
      az.rotation.y = (i / NBL) * TAU;
      az.add(new THREE.Mesh(armGeo, mHub));
      const pg = new THREE.Group();
      az.add(pg);
      const bl = new THREE.Mesh(bladeGeo, mBlade);
      bl.position.x = BR0;
      const cap = new THREE.Mesh(capGeo, mCutFace);
      cap.position.x = BR0;
      pg.add(bl, cap, new THREE.Mesh(gripGeo, mHub), new THREE.Mesh(hornGeo, mHub), new THREE.Mesh(hornBall, mLink));
      mast.add(az);
      pitchGroups.push(pg);
    }
    model.add(
      ctx.part(head, {
        id: 'rotorHead',
        name: 'Testa del rotore e pale (tagliate)',
        base:
          "In cima all'albero il mozzo regge cinque pale. Qui sono tagliate (bordo rosso): in realtà sono lunghe 6–7 metri. Ogni pala può ruotare attorno al proprio asse, " +
          "grazie a un attacco snodato e a una leva (squadretta) mossa dal piatto oscillante. Il rotore gira sempre a circa 300 giri al minuto.",
        expert:
          "Rotore a 5 pale in composito (AW139: Ø 13,8 m) con mozzo a cuscinetti elastomerici che consentono passo, flappeggio e ritardo. Nr ≈ 100% costante " +
          "(~250–400 giri/min secondo il modello di elicottero) per tenere le punte intorno a 200–220 m/s. Passo collettivo ≈ 0–15°, svergolamento delle pale ≈ −8/−12°. " +
          "La coppia sull'albero rotore è decine di volte quella sull'albero del motore.",
        explode: [0, 1.2, 0],
      }),
    );

    // ---------- Tail drive: shaft, hanger bearings, intermediate and tail gearboxes ----------
    const tail = new THREE.Group();
    const tailSpin = new THREE.Group();
    tailSpin.position.y = YT;
    tail.add(tailSpin);
    const X_IGB = 2.28;
    const TGB = new THREE.Vector3(2.6, YT + 0.32, 0);
    const shaftLen = X_IGB - (XM + 0.17);
    tailSpin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, shaftLen, 16).rotateZ(Math.PI / 2).translate((X_IGB + XM + 0.17) / 2, 0, 0), mShaft));
    for (const x of [-0.6, 0.25, 1.1, 1.9]) {
      tailSpin.add(new THREE.Mesh(latheX([[0.03, x - 0.03], [0.07, x - 0.03], [0.07, x + 0.03], [0.03, x + 0.03]], 24), mCoupling));
    }
    for (const x of [-0.2, 0.7, 1.5]) {
      const hanger = new THREE.Mesh(new RoundedBoxGeometry(0.06, 0.1, 0.72, 2, 0.012), mFrame);
      hanger.position.set(x, YT - 0.07, 0);
      const brg = new THREE.Mesh(latheX([[0.03, x - 0.035], [0.06, x - 0.035], [0.06, x + 0.035], [0.03, x + 0.035], [0.03, x - 0.035]], 24), mBronze);
      brg.position.y = YT;
      tail.add(hanger, brg);
    }
    const slant = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.026, 1, 14), mShaft);
    placeBetween(slant, new THREE.Vector3(X_IGB, YT, 0), TGB);
    tail.add(slant);
    const igb = new THREE.Mesh(new THREE.SphereGeometry(0.085, 20, 14), mMgb);
    igb.position.set(X_IGB, YT, 0);
    const tgb = new THREE.Mesh(wall([[0.05, -0.02], [0.1, -0.02], [0.11, 0.06], [0.09, 0.18], [0.05, 0.2]], 0.01, 40).rotateY(-Math.PI / 2), mMgb);
    tgb.position.copy(TGB);
    tail.add(igb, tgb);
    for (const [x, y] of [[X_IGB, YT], [TGB.x, TGB.y]]) {
      const m = new THREE.Mesh(new RoundedBoxGeometry(0.1, 0.08, 0.06, 2, 0.01), mMgb);
      m.position.set(x, y, -0.07);
      tail.add(m);
    }
    model.add(
      ctx.part(tail, {
        id: 'tailDrive',
        name: 'Trasmissione al rotore di coda',
        base:
          "Un lungo albero sottile corre all'indietro sotto il motore fino alla coda (qui accorciato). Due scatole di ingranaggi cambiano direzione: una lo piega verso l'alto, " +
          "l'altra lo gira di lato verso il rotore di coda.",
        expert:
          "Albero di coda segmentato (≈ 4.000–6.000 giri/min) con giunti flessibili e cuscinetti di supporto (hanger bearing) lungo la trave, scatola intermedia (IGB) a ~45° " +
          "alla base della deriva e scatola di coda (TGB) a 90°. Prelevato dall'MGB a valle delle ruote libere, così il rotore di coda resta comandabile anche in autorotazione.",
        explode: [0, -0.5, 0],
      }),
    );

    // ---------- Tail rotor ----------
    const tailRotor = new THREE.Group();
    const trSpin = new THREE.Group();
    trSpin.position.set(TGB.x, TGB.y, 0.2);
    tailRotor.add(trSpin);
    trSpin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.08, 20).rotateX(Math.PI / 2), mHub));
    const trBlade = airfoilGeo(0.34, 0.09, 0.016, { segL: 8, segC: 6, twist: (u) => 0.12 * (0.75 - u), map: (r, c, t) => [c, r + 0.07, t] });
    const trTip = airfoilGeo(0.06, 0.09, 0.016, { segL: 2, segC: 6, twist: () => 0.12 * (0.75 - 1), map: (r, c, t) => [c * 1.01, r + 0.41, t * 1.1] });
    const trPitch = [];
    for (let i = 0; i < 4; i++) {
      const az = new THREE.Group();
      az.rotation.z = (i / 4) * TAU;
      const pg = new THREE.Group();
      az.add(pg);
      pg.add(new THREE.Mesh(trBlade, mTailBlade), new THREE.Mesh(trTip, mTailStripe));
      trSpin.add(az);
      trPitch.push(pg);
    }
    const trShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.22, 12).rotateX(Math.PI / 2), mShaft);
    trShaft.position.set(TGB.x, TGB.y, 0.1);
    tailRotor.add(trShaft);
    model.add(
      ctx.part(tailRotor, {
        id: 'tailRotor',
        name: 'Rotore di coda',
        base:
          "Una piccola elica laterale che spinge la coda di lato e impedisce alla fusoliera di girare su se stessa. Quando il pilota alza il collettivo e il rotore principale " +
          "chiede più potenza, anche le pale di coda devono inclinarsi di più (con i pedali).",
        expert:
          "Rotore di coda a 4 pale a passo variabile, ≈ 5 volte i giri del rotore principale, comandato dai pedali tramite un'asta che attraversa l'albero della TGB. " +
          "Assorbe ~10–15% della potenza in hovering. Alternative: fenestron intubato (Airbus) e NOTAR. Qui il passo di coda segue il collettivo per mostrare la compensazione di coppia.",
        explode: [0.6, 0.1, 0.3],
      }),
    );
    const trNut = boltSet(mBolt, 0, 0.035, capNutGeo(0.03, 0.025)).add(new THREE.Vector3(0, 0, 0.04), new THREE.Vector3(0, 0, 1)).mesh();
    trNut.position.copy(trSpin.position);
    model.add(
      ctx.part(trNut, {
        id: 'tailRotorNut',
        name: 'Dado del rotore di coda',
        base: "Il dado al centro del rotore di coda: blocca il mozzo con le quattro pale sull'alberino che esce dalla scatola di coda.",
        expert:
          "Dado di ritegno del mozzo del rotore di coda sull'albero di uscita della scatola a 90° (TGB), con rondella e fermo di sicurezza. " +
          "La coppia passa dalle scanalature tra albero e mozzo; il dado lo tiene soltanto in posizione lungo l'asse.",
        explode: [0.6, 0.1, 0.55],
      }),
    );

    // ---------- Airframe: cabin-roof beams, gearbox struts, engine mounts, tail fin ----------
    const frame = new THREE.Group();
    {
      for (const sz of [1, -1]) {
        const beam = new THREE.Mesh(new RoundedBoxGeometry(4.3, 0.08, 0.08, 2, 0.015), mFrame);
        beam.position.set(0.35, -1.34, sz * 0.32);
        frame.add(beam);
      }
      for (const x of [-1.2, -0.425, 0.9, 2.0]) {
        const cb = new THREE.Mesh(new RoundedBoxGeometry(0.08, 0.08, 0.72, 2, 0.015), mFrame);
        cb.position.set(x, -1.34, 0);
        frame.add(cb);
      }
      const v = (x, y, z) => new THREE.Vector3(x, y, z);
      const struts = [];
      for (const sz of [1, -1]) {
        struts.push([v(-0.428, -1.06, sz * 0.1), v(-0.428, -1.3, sz * 0.32)]);
        struts.push([v(0.9, -1.08, sz * 0.12), v(0.9, -1.3, sz * 0.32)]);
        for (const sx of [1, -1]) struts.push([v(XM + sx * 0.34, -0.82, sz * 0.34), v(XM + sx * 0.34, -1.3, sz * 0.32)]);
      }
      const lug = new RoundedBoxGeometry(0.07, 0.05, 0.07, 2, 0.01);
      for (const [a, b] of struts) {
        frame.add(new THREE.Mesh(cylBetweenGeo(a, b, 0.017, 10), mFrame));
        const l = new THREE.Mesh(lug, mFrame);
        l.position.copy(b).setY(-1.285);
        frame.add(l);
      }
      // vertical fin carrying the intermediate and tail gearboxes
      const fin = new THREE.Shape([
        new THREE.Vector2(2.1, -1.38), new THREE.Vector2(2.9, -1.38), new THREE.Vector2(2.9, -1.0),
        new THREE.Vector2(2.76, -0.55), new THREE.Vector2(2.48, -0.55), new THREE.Vector2(2.2, -1.2),
      ]);
      const finMesh = new THREE.Mesh(new THREE.ExtrudeGeometry(fin, { depth: 0.04, bevelEnabled: true, bevelThickness: 0.01, bevelSize: 0.012, bevelSegments: 2 }), mFin);
      finMesh.position.z = -0.14;
      frame.add(finMesh);
    }
    model.add(
      ctx.part(frame, {
        id: 'airframe',
        name: 'Struttura del tetto e attacchi',
        base:
          "Le travi del tetto della cabina su cui poggiano motore, riduttore e albero di coda. Il riduttore è appeso a quattro montanti: da lì passa tutto il peso " +
          "dell'elicottero in volo, perché il rotore \"tira su\" il riduttore e il riduttore tira su la fusoliera.",
        expert:
          "Il riduttore principale è fissato alla struttura con montanti (o una piastra) progettati per trasmettere portanza, coppia e vibrazioni del rotore; " +
          "spesso con isolatori antivibranti. Il motore ha attacchi anteriori e posteriori che ne consentono la dilatazione termica. Pavimento antincendio " +
          "e paratie separano il vano motori dalla cabina; la deriva porta scatola intermedia e di coda.",
        explode: [0, -0.7, 0],
      }),
    );

    // ---------- Engine accessories: gearbox, fuel control, starter, harness ----------
    const acc = new THREE.Group();
    {
      const agb = new THREE.Mesh(new RoundedBoxGeometry(0.36, 0.14, 0.34, 3, 0.03), mAccDark);
      agb.position.set(0.16, YI + 0.4, 0);
      const tower = new THREE.Mesh(cylBetweenGeo(new THREE.Vector3(0.118, YI + 0.07, 0), new THREE.Vector3(0.118, YI + 0.335, 0), 0.013, 12), mAcc);
      const starter = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.065, 0.15, 24).rotateX(Math.PI / 2), mAcc);
      starter.position.set(0.06, YI + 0.4, 0.24);
      const alt = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.12, 24).rotateX(Math.PI / 2), mAcc);
      alt.position.set(0.25, YI + 0.4, 0.23);
      const hmu = new THREE.Mesh(new RoundedBoxGeometry(0.2, 0.12, 0.12, 3, 0.02), mAcc);
      hmu.position.set(0.16, YI + 0.4, -0.23);
      const filter = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.08, 16), mAcc);
      filter.position.set(0.28, YI + 0.51, -0.08);
      acc.add(agb, tower, starter, alt, hmu, filter);
      acc.add(new THREE.Mesh(pipeAlong([[0.22, YI + 0.38, -0.29], [0.38, YI + 0.33, -0.42], [0.47, YI + 0.22, -0.48], [0.5, YI + 0.2, -0.483]], 0.011, 32), mPipe));
      acc.add(new THREE.Mesh(pipeAlong([[0.1, YI + 0.34, -0.17], [0.3, YI + 0.42, -0.36], [0.55, YI + 0.34, -0.47], [0.7, YI + 0.27, -0.47]], 0.013, 40), mHarness));
      const bolts = boltSet(mBolt, 0.008, 0.009);
      for (const x of [0.0, 0.08, 0.16, 0.24, 0.32]) bolts.add(new THREE.Vector3(x, YI + 0.47, 0.15), new THREE.Vector3(0, 1, 0)).add(new THREE.Vector3(x, YI + 0.47, -0.15), new THREE.Vector3(0, 1, 0));
      acc.add(bolts.mesh());
    }
    model.add(
      ctx.part(acc, {
        id: 'accessories',
        name: 'Scatola accessori e impianti',
        base:
          "Sopra il motore una scatola di ingranaggi, mossa dal generatore di gas, fa girare la pompa del carburante, il motorino di avviamento e l'alternatore. " +
          "Il tubo di rame porta il cherosene agli iniettori, il cavo scuro porta la scintilla alle candele.",
        expert:
          "AGB trascinata da un albero radiale dal generatore di gas; unità idromeccanica (HMU) comandata dal FADEC a doppio canale, avviatore elettrico, " +
          "alternatore dedicato al FADEC, pompe e filtro dell'olio. Le candele ad alta energia lavorano solo all'avviamento e per la riaccensione automatica.",
        explode: [0, 0.7, -0.2],
      }),
    );

    // ---------- Flows ----------
    const flows = [];
    const addFlow = (f, kind) => {
      model.add(f.object);
      ctx.addFlow(f);
      flows.push({ f, kind });
    };
    const P = (x, r, a) => new THREE.Vector3(x, YI + r * Math.cos(a), r * Math.sin(a));

    for (const deg of [200, 240, 280, 320, 355]) {
      const a = deg * DEG;
      const ex = (x, y, rr) => new THREE.Vector3(x, YI + y + rr * Math.cos(a) * 0.7, rr * Math.sin(a));
      const path = colorPath([
        [P(-0.31, 0.85, a), C_AIR],
        [P(-0.31, 0.62, a), C_AIR],
        [P(-0.31, 0.45, a), C_AIR],
        [P(-0.28, 0.34, a), C_AIR],
        [P(-0.19, 0.25, a), C_AIR],
        [P(-0.08, 0.19, a), C_AIR],
        [P(0.05, 0.185, a), C_AIR2],
        [P(0.2, 0.18, a), C_AIR2],
        [P(0.28, 0.185, a), C_AIR2],
        [P(0.33, 0.21, a), C_WARM],
        [P(0.37, 0.26, a), C_WARM],
        [P(0.395, 0.32, a), C_WARM],
        [P(0.41, 0.38, a), C_HOT],
        [P(0.43, 0.43, a), C_HOT],
        [P(0.49, 0.44, a), C_HOT],
        [P(0.53, 0.37, a), C_HOT],
        [P(0.565, 0.3, a), C_FIRE],
        [P(0.68, 0.295, a), C_FIRE],
        [P(0.82, 0.29, a), C_FIRE2],
        [P(0.93, 0.25, a), C_FIRE2],
        [P(0.975, 0.215, a), C_TURB],
        [P(1.09, 0.215, a), C_TURB],
        [P(1.2, 0.225, a), C_EXH1],
        [P(1.32, 0.235, a), C_EXH1],
        [P(1.44, 0.24, a), C_EXH],
        [ex(1.66, 0.05, 0.17), C_EXH],
        [ex(1.83, 0.21, 0.15), C_EXH],
        [ex(1.94, 0.41, 0.13), C_EXH],
        [ex(2.02, 0.58, 0.12), C_EXH],
        [ex(2.14, 0.82, 0.18), C_EXH],
        [ex(2.32, 1.15, 0.28), C_EXH],
      ]);
      const tFire = path.tOf(16);
      const tExit = path.tOf(28);
      addFlow(
        flow({
          path: path.at,
          count: 110,
          size: 0.042,
          speed: 0.13,
          colors: path.colors(48),
          spread: (t) => (t < path.tOf(3) ? 0.05 : t < tFire ? 0.025 : t < tExit ? 0.03 : 0.05 + (t - tExit) * 0.6),
          opacity: 0.9,
        }),
        'air',
      );
    }

    for (const deg of [200, 250, 300, 345]) {
      const a = deg * DEG;
      const path = colorPath([
        [P(0.5, 0.52, a), 0xffc04d],
        [P(0.515, 0.42, a), 0xffc04d],
        [P(0.535, 0.31, a), 0xffb040],
        [P(0.6, 0.296, a), 0xff7a2a],
        [P(0.68, 0.296, a), 0xff7a2a],
      ]);
      addFlow(
        flow({ path: path.at, count: 22, size: 0.03, speed: 0.9, colors: path.colors(16), spread: (t) => (t < 0.6 ? 0.004 : 0.012 + (t - 0.6) * 0.08), opacity: 0.9 }),
        'fuel',
      );
    }

    // rotor downwash
    for (const [deg, r0] of [[200, 1.3], [232, 0.95], [262, 1.45], [292, 1.0], [322, 1.35], [350, 0.85]]) {
      const a0 = deg * DEG;
      addFlow(
        flow({
          path: (t, out) => {
            const y = lerp(1.75, -0.95, t);
            const r = y > YH ? r0 * (1 + (y - YH) * 0.12) : r0 * lerp(1, 0.8, clamp((YH - y) / 1.2, 0, 1));
            const a = a0 + (y < YH ? 0.35 * (YH - y) : 0);
            return out.set(XM + r * Math.cos(a), y, Math.min(0, r * Math.sin(a)));
          },
          count: 55,
          size: 0.05,
          speed: 0.28,
          colors: [C_AIR, 0x8fe4ff, C_AIR, 0x5fc4ee],
          spread: 0.08,
          opacity: 0.7,
        }),
        'rotor',
      );
    }

    // ---------- State ----------
    let spool = null; // lagging collective/power (engine response)
    let collective = 0;
    let ggAng = 0;
    let rotorAng = 0; // mast / carrier angle
    const pinLow = new THREE.Vector3();
    const horn = new THREE.Vector3();
    const tmpA = new THREE.Vector3();
    const tmpB = new THREE.Vector3();
    const nr = () => clamp(100 - 2.5 * (collective - spool), 97, 101);

    return {
      root,
      cycleRate(throttle) {
        return 0.12 + 0.25 * throttle;
      },
      update(state) {
        const dt = state.dt || 0;
        const thr = clamp(state.throttle ?? 0, 0, 1);
        collective = thr;
        if (spool === null || dt === 0) spool = thr;
        else spool += (thr - spool) * (1 - Math.exp(-dt / 0.9));

        // rotor speed held ~100% by the FADEC; gas generator follows power
        rotorAng += dt * TAU * ROTOR_VIS * (nr() / 100);
        ggAng -= dt * TAU * (1.6 + 1.6 * spool);
        const wheelAng = rotorAng * K_PLAN;
        const ptAng = -(wheelAng * NW) / NP_IN; // input pinion / power turbine phase
        for (const o of ggRot) o.rotation.x = ggAng;
        for (const o of ptRot) o.rotation.x = ptAng;
        driveSpin.rotation.x = ptAng;
        driveBolts.rotation.x = ptAng;
        pinion.rotation.x = Math.PI / NP_IN - Math.PI + ptAng;
        wheel.rotation.y = wheelAng;
        tailPinion.rotation.x = Math.PI / NTP - Math.PI - (wheelAng * NT) / NTP;
        tailSpin.rotation.x = -(wheelAng * NT) / NTP;
        sun.rotation.y = sunPh + wheelAng;
        carrier.rotation.y = rotorAng;
        for (let i = 0; i < 4; i++) planets[i].rotation.y = planetPh[i] - (rotorAng * NR) / NPL;
        mast.rotation.y = rotorAng;
        mastShaft.rotation.y = rotorAng;
        mastNut.rotation.y = rotorAng;
        swRot.rotation.y = rotorAng;
        linkSpin.rotation.y = rotorAng;
        trSpin.rotation.z = rotorAng * 5;
        trNut.rotation.z = rotorAng * 5;

        // collective pitch -> swashplate height -> pitch links -> horns
        const th = lerp(2, 16, spool) * DEG;
        for (const pg of pitchGroups) pg.rotation.x = th;
        const ySw = YH + HL * Math.sin(th) - LK;
        swRot.position.y = ySw - 0.015;
        swStat.position.y = ySw - 0.05;
        for (let i = 0; i < NBL; i++) {
          const a = (i / NBL) * TAU;
          const c = Math.cos(a);
          const s = Math.sin(a);
          // azimuth frame (x radial, z towards the leading edge) -> mast frame
          const hz = -HL * Math.cos(th);
          horn.set(HR * c + hz * s, YH + HL * Math.sin(th), -HR * s + hz * c);
          pinLow.set(HR * c - HL * s, ySw, -HR * s - HL * c);
          placeBetween(links[i], pinLow, horn);
        }
        for (const { rod, x, z } of servoRods) {
          placeBetween(rod, tmpA.set(x, 0.16, z), tmpB.set(x, ySw - 0.07, z));
        }
        for (const pg of trPitch) pg.rotation.y = lerp(4, 16, spool) * DEG;

        // flame and hot section
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
          } else if (kind === 'rotor') {
            f.object.material.opacity = 0.25 + 0.55 * spool;
            f.update(dt, 0.4 + 0.9 * spool);
          } else {
            f.update(dt, 0.35 + 0.9 * spool);
          }
        }
      },
      readout() {
        const s = spool ?? 0;
        return [
          { label: 'Nr (rotore)', value: nr().toFixed(0), unit: '%' },
          { label: 'Ng (gen. di gas)', value: (70 + 30 * Math.pow(s, 0.9)).toFixed(0), unit: '%' },
          { label: 'Coppia (TQ)', value: (12 + 83 * Math.pow(s, 1.3)).toFixed(0), unit: '%' },
          { label: 'Passo collettivo', value: lerp(2, 16, s).toFixed(1), unit: '°' },
        ];
      },
    };
  },
};
