// Solid rocket motor (Avio P120C-like: Vega-C first stage, Ariane 6 booster). Continuous engine with no
// throttle: the slider is burn time (0..1). The star bore regresses outward, thrust follows the burning
// area computed from the grain geometry. state.cycle only drives the flame flicker.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, flow, clamp, lerp, smooth } from '../core/helpers.js';

// ---------- Stations (model space, nozzle points down -Y) ----------
const R_OUT = 0.62; // case outer radius
const R_CIN = 0.595; // case inner radius
const R_G = 0.563; // grain outer radius (inside the liner)
const Y_CT = 1.5; // cylinder top = forward dome equator
const Y_CB = -1.05; // cylinder bottom = aft dome equator
const H_F = 0.36; // forward dome height
const H_A = 0.38; // aft dome height
const R_FO = 0.15; // forward polar opening (igniter boss)
const R_AO = 0.3; // aft polar opening (nozzle)
const G_TOP = 1.46;
const G_BOT = -1.0;
const Y_NOSE = -1.08; // submerged nozzle nose
const R_NOSE = 0.22;
const Y_T = -1.3; // throat
const R_T = 0.1;
const Y_E = -2.5; // nozzle exit
const R_E = 0.4;
const PIV = -1.45; // flex seal pivot
const Y_TI0 = -1.22; // throat insert span
const Y_TI1 = -1.44;
const ROOT_Y = 1.1;
// P120C reference numbers for readouts
const BURN_S = 135;
const F_MAX = 4500;
const PROP_T = 142;
const PC_MAX = 100;

// ---------- Star grain burn-back table ----------
// The burning surface at web distance w is the offset of the initial star bore by w (uniform regression).
// Thrust ~ burning area^(1/(1-n)), burn rate ~ p^n with n = 0.35, so time is not linear in w.
const NS = 7; // star points
const R_B = 0.12; // inner radius of the grain arms
const R_TIP = 0.34; // slot tip radius
const TIP_HW = 0.035; // slot tip half width
const W0 = 0.012; // initial rounding of slot tips
const NB = 240; // bore samples around the axis
const NL = 64; // precomputed burn levels
const COS = new Float32Array(NB + 1);
const SIN = new Float32Array(NB + 1);
for (let i = 0; i <= NB; i++) {
  COS[i] = Math.cos((i / NB) * Math.PI * 2);
  SIN[i] = Math.sin((i / NB) * Math.PI * 2);
}

const BURN = (() => {
  const poly = [];
  for (let k = 0; k < NS; k++) {
    const a = (k / NS) * Math.PI * 2;
    const d = TIP_HW / R_TIP;
    poly.push([R_TIP * Math.cos(a - d), R_TIP * Math.sin(a - d)], [R_TIP * Math.cos(a + d), R_TIP * Math.sin(a + d)]);
    const b = a + Math.PI / NS;
    poly.push([R_B * Math.cos(b), R_B * Math.sin(b)]);
  }
  const edges = poly.map((p, i) => [p, poly[(i + 1) % poly.length]]);
  const dist = (x, z) => {
    let m = Infinity;
    for (const [a, b] of edges) {
      const ex = b[0] - a[0];
      const ez = b[1] - a[1];
      const s = clamp(((x - a[0]) * ex + (z - a[1]) * ez) / (ex * ex + ez * ez), 0, 1);
      const dx = a[0] + s * ex - x;
      const dz = a[1] + s * ez - z;
      m = Math.min(m, dx * dx + dz * dz);
    }
    return Math.sqrt(m);
  };
  // Radius of the initial bore along a ray (the star is star-shaped from the axis).
  const rPoly = (c, s) => {
    for (const [a, b] of edges) {
      const ex = b[0] - a[0];
      const ez = b[1] - a[1];
      const det = ex * s - c * ez;
      if (Math.abs(det) < 1e-12) continue;
      const t = (ex * a[1] - a[0] * ez) / det;
      const u = (c * a[1] - s * a[0]) / det;
      if (u >= -1e-9 && u <= 1 + 1e-9 && t > 0) return t;
    }
    return R_B;
  };
  let wMax = 0;
  for (let i = 0; i < NB; i++) wMax = Math.max(wMax, dist(R_G * COS[i], R_G * SIN[i]));
  wMax += 0.003;

  const r = new Float32Array(NL * NB);
  const per = new Float32Array(NL);
  const area = new Float32Array(NL);
  const dth = (Math.PI * 2) / NB;
  for (let k = 0; k < NL; k++) {
    const w = lerp(W0, wMax, k / (NL - 1));
    for (let i = 0; i < NB; i++) {
      let lo = rPoly(COS[i], SIN[i]);
      let hi = R_G + 0.01;
      for (let j = 0; j < 20; j++) {
        const m = (lo + hi) / 2;
        if (dist(m * COS[i], m * SIN[i]) <= w) lo = m;
        else hi = m;
      }
      r[k * NB + i] = Math.min(lo, R_G);
    }
    let p = 0;
    let A = 0;
    for (let i = 0; i < NB; i++) {
      const r0 = r[k * NB + i];
      const r1 = r[k * NB + ((i + 1) % NB)];
      A += 0.5 * r0 * r1 * Math.sin(dth);
      // only the part still in contact with propellant burns
      if (r0 < R_G - 1e-4 || r1 < R_G - 1e-4) p += Math.sqrt(r0 * r0 + r1 * r1 - 2 * r0 * r1 * Math.cos(dth));
    }
    per[k] = p;
    area[k] = A;
  }
  const pMax = Math.max(...per);
  const t = new Float32Array(NL);
  const F = new Float32Array(NL);
  const m = new Float32Array(NL);
  const full = Math.PI * R_G * R_G - area[0];
  for (let k = 0; k < NL; k++) {
    if (k > 0) t[k] = t[k - 1] + 1 / Math.max(0.85, Math.pow((per[k] + per[k - 1]) / 2 / pMax, 0.54));
    F[k] = Math.pow(per[k] / pMax, 1.54);
    m[k] = clamp(1 - (area[k] - area[0]) / full, 0, 1);
  }
  for (let k = 0; k < NL; k++) t[k] /= t[NL - 1];
  return { r, t, F, m };
})();

// Burn state at time fraction b. Fills `out` with bore radii (NB) when given.
function burnAt(b, out) {
  const { r, t, F, m } = BURN;
  let k = 0;
  while (k < NL - 2 && t[k + 1] < b) k++;
  const f = clamp((b - t[k]) / (t[k + 1] - t[k]), 0, 1);
  if (out) {
    const o0 = k * NB;
    for (let i = 0; i < NB; i++) out[i] = lerp(r[o0 + i], r[o0 + NB + i], f);
  }
  const ignite = smooth(clamp(b / 0.025, 0, 1));
  return { thrust: lerp(F[k], F[k + 1], f) * ignite, mass: lerp(m[k], m[k + 1], f) };
}

// ---------- Nozzle inner profile (submerged nose -> throat -> bell) ----------
function buildProfile() {
  const pts = [];
  for (let i = 0; i <= 12; i++) {
    const y = lerp(Y_NOSE, Y_T, i / 12);
    pts.push([R_T + (R_NOSE - R_T) * smooth((y - Y_T) / (Y_NOSE - Y_T)), y]);
  }
  const a0 = Math.tan((24 * Math.PI) / 180);
  const a1 = Math.tan((9 * Math.PI) / 180);
  const s = (R_E - a1 * (Y_T - Y_E) - R_T) / (a0 - a1);
  const c = [R_T + a0 * s, Y_T - s];
  for (let i = 1; i <= 36; i++) {
    const u = i / 36;
    const v = 1 - u;
    pts.push([v * v * R_T + 2 * v * u * c[0] + u * u * R_E, v * v * Y_T + 2 * v * u * c[1] + u * u * Y_E]);
  }
  return pts;
}
const PROF = buildProfile();

function rAt(y) {
  if (y >= PROF[0][1]) return PROF[0][0];
  for (let i = 1; i < PROF.length; i++) {
    const [r1, y1] = PROF[i];
    if (y >= y1) {
      const [r0, y0] = PROF[i - 1];
      return lerp(r0, r1, (y0 - y) / (y0 - y1));
    }
  }
  return PROF[PROF.length - 1][0];
}

function profSlice(yTop, yBot, dr = 0) {
  const pts = [[rAt(yTop) + dr, yTop]];
  for (const [r, y] of PROF) if (y < yTop && y > yBot) pts.push([r + dr, y]);
  pts.push([rAt(yBot) + dr, yBot]);
  return pts;
}

// ---------- Small builders ----------
const V = (x, y, z = 0) => new THREE.Vector3(x, y, z);

function ring(R, r, y, mat, segs = 72) {
  const m = new THREE.Mesh(new THREE.TorusGeometry(R, r, 8, segs), mat);
  m.rotation.x = Math.PI / 2;
  m.position.y = y;
  return m;
}

function band(rOut, rIn, y0, y1, mat, segs = 72) {
  return new THREE.Mesh(lathe([[rIn, y0], [rOut, y0], [rOut, y1], [rIn, y1], [rIn, y0]], segs), mat);
}

// Elliptical dome arc from the equator (R, yEq) to the polar opening rOpen; dir +1 up, -1 down.
function domeArc(yEq, R, H, rOpen, dir, n = 20) {
  const aMax = Math.acos(clamp(rOpen / R, 0, 1));
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const a = (i / n) * aMax;
    pts.push([R * Math.cos(a), yEq + dir * H * Math.sin(a)]);
  }
  return pts;
}
// Pressure-vessel shell (fwd dome + cylinder + aft dome) between an outer and an inner surface.
function shellGeo(o, i, segs = 96) {
  const path = (s) => [...domeArc(Y_CT, s.R, s.hf, R_FO, 1).reverse(), ...domeArc(Y_CB, s.R, s.ha, R_AO, -1)];
  const outer = path(o);
  const inner = path(i).reverse();
  return lathe([...outer, ...inner, outer[0]], segs);
}

// Grain with a regressing bore: bore surface + two end annuli, positions rewritten as it burns.
function grainGeometries() {
  const n = NB + 1;
  const bore = new THREE.BufferGeometry();
  bore.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 2 * 3), 3));
  const bi = [];
  for (let i = 0; i < NB; i++) {
    const t0 = i;
    const t1 = i + 1;
    const b0 = n + i;
    const b1 = n + i + 1;
    bi.push(b0, b1, t0, t0, b1, t1); // normals face the axis
  }
  bore.setIndex(bi);
  const ends = new THREE.BufferGeometry();
  const ePos = new Float32Array(n * 4 * 3);
  const eNor = new Float32Array(n * 4 * 3);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < 4; j++) {
      const v = j * n + i;
      eNor[v * 3 + 1] = j < 2 ? 1 : -1;
      if (j === 1 || j === 3) {
        ePos[v * 3] = R_G * COS[i];
        ePos[v * 3 + 1] = j === 1 ? G_TOP : G_BOT;
        ePos[v * 3 + 2] = R_G * SIN[i];
      }
    }
  }
  ends.setAttribute('position', new THREE.BufferAttribute(ePos, 3));
  ends.setAttribute('normal', new THREE.BufferAttribute(eNor, 3));
  const ei = [];
  for (let i = 0; i < NB; i++) {
    const [it, ot, ib, ob] = [0, n, 2 * n, 3 * n];
    ei.push(it + i, it + i + 1, ot + i, ot + i, it + i + 1, ot + i + 1); // top, +Y
    ei.push(ib + i, ob + i, ib + i + 1, ob + i, ob + i + 1, ib + i + 1); // bottom, -Y
  }
  ends.setIndex(ei);
  function set(rArr) {
    const bp = bore.attributes.position.array;
    const ep = ends.attributes.position.array;
    for (let i = 0; i <= NB; i++) {
      const r = rArr[i % NB];
      const x = r * COS[i];
      const z = r * SIN[i];
      bp.set([x, G_TOP, z], i * 3);
      bp.set([x, G_BOT, z], (n + i) * 3);
      ep.set([x, G_TOP, z], i * 3);
      ep.set([x, G_BOT, z], (2 * n + i) * 3);
    }
    bore.attributes.position.needsUpdate = true;
    ends.attributes.position.needsUpdate = true;
    bore.computeVertexNormals();
    bore.computeBoundingSphere();
    ends.computeBoundingSphere();
  }
  return { bore, ends, set };
}

// Hex head + shank of length `len` below it (local -Y): a screw backing out of its hole (+Y) shows its shank.
const screwGeo = (r, len) =>
  mergeGeometries([new THREE.CylinderGeometry(r, r, r * 0.9, 6), new THREE.CylinderGeometry(r * 0.55, r * 0.55, len, 8).translate(0, -r * 0.45 - len / 2, 0)]);

// Circle of axial screws, heads up (dir = 1) or down (dir = -1), seated on the face at height y (one InstancedMesh).
function screwCircle(R, y, n, mat, size, len, dir = 1) {
  const mesh = new THREE.InstancedMesh(screwGeo(size, len), mat, n);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), dir > 0 ? 0 : Math.PI);
  const s = new THREE.Vector3(1, 1, 1);
  for (let i = 0; i < n; i++) {
    const a = ((i + 0.5) / n) * Math.PI * 2;
    m.compose(new THREE.Vector3(Math.cos(a) * R, y + dir * size * 0.45, Math.sin(a) * R), q, s);
    mesh.setMatrixAt(i, m);
  }
  mesh.computeBoundingSphere();
  return mesh;
}

// Ring of hex bolt heads pointing radially outwards (one InstancedMesh). With `len` they are real screws.
function boltRing(R, y, n, mat, size = 0.011, phase = 0, len = 0) {
  const geo = len ? screwGeo(size, len) : new THREE.CylinderGeometry(size, size, size * 0.9, 6);
  const mesh = new THREE.InstancedMesh(geo, mat, n);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const s = new THREE.Vector3(1, 1, 1);
  for (let i = 0; i < n; i++) {
    const a = ((i + phase) / n) * Math.PI * 2;
    const d = V(Math.cos(a), 0, Math.sin(a));
    q.setFromUnitVectors(V(0, 1, 0), d);
    m.compose(d.clone().multiplyScalar(R + size * 0.45).setY(y), q, s);
    mesh.setMatrixAt(i, m);
  }
  mesh.computeBoundingSphere();
  return mesh;
}

// Filament-wound carbon look: crossing helical tows on a satin dark base.
function windingTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#1f2226';
  g.fillRect(0, 0, 256, 256);
  for (let i = -256; i < 512; i += 8) {
    g.strokeStyle = i % 16 ? 'rgba(255,255,255,0.05)' : 'rgba(0,0,0,0.25)';
    g.lineWidth = 3;
    g.beginPath();
    g.moveTo(i, 0);
    g.lineTo(i + 256, 256);
    g.stroke();
    g.strokeStyle = i % 16 ? 'rgba(255,255,255,0.035)' : 'rgba(0,0,0,0.18)';
    g.beginPath();
    g.moveTo(i + 256, 0);
    g.lineTo(i, 256);
    g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(10, 6);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

// Generic stage marking on the forward skirt (no real brand).
function markingTexture() {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = '#dfe3e8';
  g.fillRect(0, 0, 512, 128);
  g.fillStyle = '#1b2a4a';
  g.font = '700 54px system-ui, sans-serif';
  g.textAlign = 'center';
  g.fillText('SRM · STADIO 1', 256, 72);
  g.fillStyle = '#c4582c';
  g.fillRect(40, 92, 432, 10);
  g.fillStyle = '#5a6470';
  g.font = '500 20px system-ui, sans-serif';
  g.fillText('PROPELLENTE SOLIDO · NON APRIRE', 256, 122);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

// Thrust-time chart drawn once on a canvas (real P120C-like scale).
function chartTexture() {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 320;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(12,18,30,0.82)';
  g.strokeStyle = 'rgba(120,170,220,0.5)';
  g.lineWidth = 2;
  g.beginPath();
  g.roundRect(2, 2, 508, 316, 18);
  g.fill();
  g.stroke();
  const X0 = 70;
  const X1 = 490;
  const Y0 = 268;
  const Y1 = 40;
  g.font = '600 20px system-ui, sans-serif';
  g.fillStyle = '#cfe3ff';
  g.fillText('Spinta (kN)', 14, 28);
  g.textAlign = 'right';
  g.fillText('Tempo (s)', 496, 308);
  g.font = '16px system-ui, sans-serif';
  g.fillStyle = 'rgba(200,220,255,0.7)';
  g.strokeStyle = 'rgba(140,170,210,0.22)';
  g.lineWidth = 1;
  for (const kn of [0, 1000, 2000, 3000, 4000]) {
    const y = lerp(Y0, Y1, kn / F_MAX);
    g.beginPath();
    g.moveTo(X0, y);
    g.lineTo(X1, y);
    g.stroke();
    g.fillText(String(kn), X0 - 8, y + 5);
  }
  g.textAlign = 'center';
  for (const s of [0, 30, 60, 90, 120]) {
    const x = lerp(X0, X1, s / BURN_S);
    g.beginPath();
    g.moveTo(x, Y0);
    g.lineTo(x, Y1);
    g.stroke();
    g.fillText(String(s), x, Y0 + 20);
  }
  g.strokeStyle = '#ffb347';
  g.lineWidth = 4;
  g.lineJoin = 'round';
  g.beginPath();
  for (let i = 0; i <= 200; i++) {
    const b = i / 200;
    const x = lerp(X0, X1, b);
    const y = lerp(Y0, Y1, burnAt(b).thrust);
    if (i) g.lineTo(x, y);
    else g.moveTo(x, y);
  }
  g.stroke();
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  // plot area in plane units (u, v in 0..1)
  return { tex, plot: { u0: X0 / 512, u1: X1 / 512, v0: 1 - Y0 / 320, v1: 1 - Y1 / 320 } };
}

export default {
  id: 'solidRocket',
  name: 'Motore a razzo a propellente solido',
  short: 'Razzo a solido',
  family: 'spazio',
  vehicle: 'Booster e lanciatore Vega',
  examples: 'Avio P120C (Vega-C, Ariane 6), P80 (Vega), booster dello Space Shuttle e dello SLS',
  tagline: 'Una volta acceso non si spegne più: la spinta è scritta nella forma del suo “foro” interno.',
  throttleLabel: 'Tempo di combustione',
  intro: {
    base:
      'È il razzo più semplice che esista, parente stretto dei fuochi d’artificio: un enorme tubo pieno di un propellente solido, gommoso, che contiene già sia il combustibile sia l’ossigeno per bruciarlo. ' +
      'Al centro c’è un foro a forma di <b>stella</b>: quando lo si accende brucia tutta la superficie interna e il foro si allarga verso l’esterno. ' +
      'Non c’è un acceleratore: <b>una volta acceso brucia fino alla fine</b>. Usa il cursore “Tempo di combustione” per vedere come il grano si consuma.',
    expert:
      'Endoreattore a propellente solido composito (HTPB / perclorato d’ammonio / alluminio) con involucro monolitico in fibra di carbonio avvolta (filament winding), come l’<b>Avio P120C</b>, primo stadio del Vega-C e booster dell’Ariane 6, prodotto a <b>Colleferro</b> (Roma). ' +
      'La legge di spinta F(t) è “programmata” dalla geometria del grano: l’area di combustione Ab(t) fissa la pressione in camera (pc ∝ Ab^(1/(1−n)), con n ≈ 0,3–0,4) e quindi la spinta. ' +
      'Ugello sommerso con inserto di gola in carbonio-carbonio, giunto flessibile (flex seal) e attuatori elettromeccanici per il controllo vettoriale della spinta (TVC).',
  },
  specs: [
    { label: 'Propellente', value: 'HTPB + perclorato d’ammonio + alluminio' },
    { label: 'Spinta massima', value: '≈ 4500 kN (nel vuoto)' },
    { label: 'Massa di propellente', value: '≈ 142 t' },
    { label: 'Durata della combustione', value: '≈ 135 s' },
    { label: 'Impulso specifico', value: '≈ 279 s (vuoto)' },
    { label: 'Pressione in camera', value: '≈ 100 bar (massima)' },
    { label: 'Dimensioni', value: '≈ 13,5 m × 3,4 m' },
  ],
  variants: {
    columns: ['Motore', 'Impiego', 'Propellente', 'Spinta max', 'Combustione'],
    rows: [
      ['Avio P80', 'Vega, 1º stadio', '≈ 88 t', '≈ 3000 kN', '≈ 110 s'],
      ['Avio P120C', 'Vega-C 1º stadio, booster Ariane 6', '≈ 142 t', '≈ 4500 kN', '≈ 135 s'],
      ['Booster dello Space Shuttle (4 segmenti)', 'Shuttle, 2 per lancio (1981–2011)', '≈ 500 t', '≈ 12.500 kN', '≈ 124 s'],
      ['Booster SLS (5 segmenti)', 'SLS, missioni Artemis', '≈ 630 t', '≈ 16.000 kN', '≈ 126 s'],
    ],
    note:
      'I motori Avio, costruiti a Colleferro, hanno involucro monolitico in fibra di carbonio e propellente HTPB; i booster americani hanno involucro in acciaio diviso in segmenti e propellente PBAN.',
  },
  thumb: { orbit: [28, 22], zoom: 1.35, view: 'section' },
  metrics: {
    efficiency: 55, // approx. chemical energy -> jet kinetic energy
    power: 3.8e6, // kW, average jet power 0.5 * mdot * ve^2
    powerDensity: 350, // kW/kg of inert mass (~11 t)
  },
  phases: [
    {
      id: 'ignition',
      name: 'Accensione',
      parts: ['igniter', 'grain'],
      base:
        'In cima al motore c’è l’accenditore: un piccolo razzo a sua volta, che spara una fiammata lunga lungo tutto il foro centrale. ' +
        'In una frazione di secondo l’intera superficie interna del propellente prende fuoco e la pressione sale. Da quel momento non si torna indietro.',
      expert:
        'Accenditore pirogeno (un piccolo motore a solido) azionato da un dispositivo di sicurezza e armamento (S&A) che separa meccanicamente la carica dall’innesco fino al comando. ' +
        'Il getto caldo lambisce il grano e porta la superficie alla temperatura di accensione; il transitorio di salita in pressione dura qualche centinaio di millisecondi. ' +
        'Il “pressure spike” iniziale va controllato con cura, perché la camera è dimensionata sulla pressione massima (MEOP).',
    },
    {
      id: 'grain',
      name: 'Combustione del grano',
      parts: ['grain', 'liner', 'motorCase'],
      base:
        'Il propellente brucia solo sulla superficie esposta, strato dopo strato, di pochi millimetri al secondo, come una candela che si consuma dall’interno verso l’esterno. ' +
        'Più superficie brucia, più gas si produce e più il razzo spinge: per questo il foro ha la forma di una stella, studiata per dosare la spinta nel tempo.',
      expert:
        'Velocità di regressione r = a·pcⁿ (legge di Saint-Robert/Vieille), qualche mm/s a ~100 bar. Il grano si consuma per strati paralleli: ogni superficie avanza lungo la propria normale. ' +
        'Profilo <b>progressivo</b> (Ab cresce: es. foro cilindrico), <b>regressivo</b> (Ab cala: es. stella a punte profonde), <b>neutro</b> (Ab circa costante: stella ben proporzionata, finocyl). ' +
        'Nel modello la stella a 7 punte dà una fase regressiva, poi il foro diventa quasi circolare e la curva risale leggermente prima della coda finale. ' +
        'Il liner e l’isolamento in EPDM proteggono l’involucro dai gas a oltre 3000 °C.',
    },
    {
      id: 'nozzle',
      name: 'Ugello e spinta',
      parts: ['nozzle', 'throatInsert'],
      base:
        'I gas caldissimi si stringono nella gola dell’ugello, dove raggiungono la velocità del suono, poi si allargano nella campana e accelerano fino a circa 10.000 km/h. ' +
        'Lanciando indietro più di una tonnellata di gas al secondo, il motore riceve una spinta enorme in avanti: circa 450 tonnellate.',
      expert:
        'Ugello de Laval sommerso: la parte convergente sporge dentro il fondo posteriore della camera per accorciare lo stadio. ' +
        'La gola è un inserto in composito carbonio-carbonio, che resiste a erosione e temperature oltre i 3000 °C; il divergente è in carbonio-fenolico, che si consuma lentamente per ablazione portando via il calore. ' +
        'Portata media ~1 t/s, velocità di scarico efficace ~2,7 km/s (Isp ≈ 279 s nel vuoto). Il fumo bianco è ossido di alluminio (Al₂O₃) in particelle.',
    },
    {
      id: 'tvc',
      name: 'Controllo di direzione (TVC)',
      parts: ['flexSeal', 'tvc', 'nozzle'],
      base:
        'Per guidare il razzo si inclina l’intero ugello di pochi gradi, come si orienta il getto di una canna dell’acqua. ' +
        'L’ugello è attaccato al motore con un giunto flessibile fatto di strati di gomma e metallo, e due attuatori elettrici lo spingono e lo tirano.',
      expert:
        'Il flex seal è un giunto laminato di strati sferici alternati di elastomero e lamierini metallici: lavora a taglio e permette rotazioni di alcuni gradi (sul P120C circa ±6°) attorno a un punto vicino alla gola. ' +
        'Due attuatori elettromeccanici a 90° (EMA, alimentati a batteria) comandano beccheggio e imbardata; il rollio è affidato agli stadi superiori o a sistemi dedicati.',
    },
    {
      id: 'burnout',
      name: 'Esaurimento',
      parts: ['grain', 'motorCase'],
      base:
        'Quando la fiamma arriva alla parete, il propellente finisce: la spinta cala rapidamente e il getto si spegne. ' +
        'Lo stadio vuoto viene sganciato e cade, mentre il razzo prosegue con il motore successivo.',
      expert:
        'Il “web burnout” avviene quando il fronte di fiamma raggiunge l’isolamento; restano piccole schegge di propellente (sliver) tra le punte della stella che bruciano a pressione calante, da cui la coda (tail-off) della curva. ' +
        'Un solido non si può spegnere a comando né riaccendere: per terminare la spinta in emergenza si usano sistemi di neutralizzazione (apertura dell’involucro).',
    },
  ],
  quiz: [
    {
      q: 'Come si regola la spinta di un razzo a propellente solido durante il volo?',
      options: [
        'Con una valvola che dosa il propellente',
        'Non si regola a comando: la spinta nel tempo è decisa in anticipo dalla forma del grano',
        'Spegnendolo e riaccendendolo più volte',
        'Cambiando la velocità della turbopompa',
      ],
      correct: 1,
      explain:
        'Non ci sono valvole né pompe: una volta acceso brucia fino alla fine. La geometria del foro (stella, cilindro, finocyl) stabilisce quanta superficie brucia istante per istante, e quindi la spinta.',
    },
    {
      q: 'Perché il foro centrale del grano ha la forma di una stella?',
      options: [
        'Per alleggerire il motore',
        'Per far passare i cavi elettrici',
        'Per avere molta superficie che brucia fin dall’inizio e una spinta più costante',
        'Per raffreddare l’involucro',
      ],
      correct: 2,
      explain:
        'Le punte della stella aggiungono superficie di combustione all’inizio; man mano che si consumano, la superficie del foro che si allarga compensa. Un semplice foro cilindrico darebbe invece una spinta sempre crescente.',
    },
    {
      q: 'Che cosa contiene il propellente solido di un motore come il P120C?',
      options: [
        'Combustibile e ossidante già mescolati: gomma HTPB, perclorato d’ammonio e polvere di alluminio',
        'Solo combustibile: l’ossigeno arriva dall’aria',
        'Polvere da sparo nera',
        'Cherosene congelato',
      ],
      correct: 0,
      explain:
        'Il perclorato d’ammonio fornisce l’ossigeno, l’alluminio in polvere alza la temperatura e l’energia, la gomma HTPB fa da legante e da combustibile. Per questo funziona anche nel vuoto.',
    },
    {
      q: 'Rispetto a un motore a propellente liquido, qual è il principale svantaggio di un razzo a solido?',
      options: [
        'È molto più complicato da costruire',
        'Ha una spinta molto più piccola',
        'Non può essere conservato a lungo',
        'Non si può regolare né spegnere, e ha un impulso specifico più basso',
      ],
      correct: 3,
      explain:
        'I solidi sono semplici, economici, pronti al lancio e danno spinte enormi, ma non si possono parzializzare o spegnere e hanno un Isp più basso (~280 s contro 310–450 s dei liquidi).',
    },
  ],
  // Officina (see CONTRACT.md). A solid motor is cast, not assembled: the sequence is the final integration
  // run backwards, and the loaded case (case + insulation + grain) stays in one piece, as in reality.
  disassembly: {
    intro:
      'Un motore a solido non si smonta davvero: il propellente è colato e incollato dentro l’involucro. Qui ripercorriamo all’indietro il suo montaggio finale, fino a restare con il “motore caricato”.',
    note:
      'Sequenza semplificata a scopo didattico: è l’integrazione dello stadio vista al contrario, non una procedura reale. Involucro, isolamento e grano restano un pezzo unico: nella realtà non si possono separare, e su un motore carico lavora solo personale specializzato con procedure pirotecniche.',
    parts: {
      safeArm: { dir: [0, 1, 0], dist: 0.7 },
      igniterBolts: { after: ['safeArm'], motion: 'unscrew', turns: 6, out: 0.16, dir: [0, 1, 0], dist: 0.3 },
      igniter: { after: ['igniterBolts', 'safeArm'], dir: [0, 1, 0], dist: 1.3, duration: 1.2 },
      raceway: { dir: [0.32, 0, 0.95], dist: 0.7 },
      aftSkirtBolts: { motion: 'unscrew', turns: 5, out: 0.06, dir: [0, -1, 0], dist: 0.2 },
      aftSkirt: { after: ['aftSkirtBolts'], dir: [0, -1, 0], dist: 1.6, duration: 1.2 },
      tvcPins: { after: ['aftSkirt'], motion: 'unscrew', turns: 4, out: 0.11, dir: [0, -1, 0], dist: 0.25 },
      tvc: { after: ['tvcPins', 'aftSkirt'], dir: [-0.7, -0.5, -0.7], dist: 0.9 },
      nozzleBolts: { after: ['aftSkirt'], motion: 'unscrew', turns: 6, out: 0.08, dir: [0, -1, 0], dist: 0.3 },
      nozzle: { after: ['nozzleBolts', 'tvc', 'aftSkirt'], with: ['throatInsert', 'flexSeal'], dir: [0, -1, 0], dist: 1.7, duration: 1.3 },
      throatInsert: { dir: [0, -1, 0], dist: 1.6 },
      flexSeal: { after: ['nozzle'], dir: [0, -1, 0], dist: 1.0 },
      motorCase: { after: ['raceway', 'aftSkirt', 'nozzle', 'igniter'] },
    },
    steps: [
      {
        id: 'safe',
        name: 'Stacca il dispositivo di sicurezza',
        base: 'Per prima cosa si toglie il “lucchetto” dell’accensione con il suo cavo: senza di lui nessun comando può arrivare all’accenditore.',
        expert:
          'Nel montaggio reale il Safe & Arm e la linea di accensione sono tra le ultime cose a essere installate, proprio per sicurezza: andando all’indietro sono le prime a sparire. Con la catena pirotecnica interrotta il motore è inerte.',
        parts: ['safeArm'],
      },
      {
        id: 'igniter',
        name: 'Estrai l’accenditore',
        base: 'Si svitano i bulloni in cima al motore e si sfila l’accenditore dall’alto: è un tubo che pesca nel foro a stella del propellente.',
        expert:
          'L’accenditore pirogeno è imbullonato sul boss polare anteriore e sporge nel canale del grano. La tenuta è fatta con O-ring sulla flangia; tolto lui, la bocca anteriore dell’involucro resta aperta sul canale a stella.',
        parts: ['igniterBolts', 'igniter'],
      },
      {
        id: 'raceway',
        name: 'Togli la canalina dei cavi',
        base: 'Si stacca dal fianco del motore la canalina che porta i cavi dall’alto fino agli attuatori dell’ugello.',
        expert:
          'Scollegati i connettori alle due estremità, il tunnel dei sistemi si separa dai suoi supporti sull’involucro. I cablaggi del TVC non possono attraversare il motore: corrono per forza all’esterno.',
        parts: ['raceway'],
      },
      {
        id: 'skirt',
        name: 'Sfila la gonna posteriore',
        base: 'Si svita la corona di viti e la gonna bianca scende verso il basso, passando attorno all’ugello: ora si vedono il fondo del motore, il giunto e gli attuatori.',
        expert:
          'Tolte le viti radiali del giunto con l’involucro, la gonna si sfila assialmente. Nella realtà gli attuatori sono ancorati alla struttura posteriore e si raggiungono dagli sportelli; nel modello, per semplicità, i loro attacchi restano sul fondo dell’involucro.',
        parts: ['aftSkirtBolts', 'aftSkirt'],
      },
      {
        id: 'tvc',
        name: 'Stacca gli attuatori',
        base: 'Si sfilano i quattro perni e i due attuatori elettrici si liberano: l’ugello non è più comandato.',
        expert:
          'Ogni attuatore elettromeccanico è incernierato a due snodi sferici: tolti i perni e scollegati i cavi di potenza e dei sensori di posizione, esce intero. Senza attuatori il giunto flessibile tiene l’ugello circa al centro, ma niente lo guida più.',
        parts: ['tvcPins', 'tvc'],
      },
      {
        id: 'nozzle',
        name: 'Sfila l’ugello',
        base: 'Si svitano i bulloni della flangia e tutto l’ugello scende in un pezzo solo, con il giunto flessibile e l’inserto di gola: la sua parte alta era nascosta dentro il motore.',
        expert:
          'Ugello, inserto di gola e flex seal formano un unico gruppo, assemblato e collaudato a parte e poi imbullonato al boss polare posteriore. Essendo sommerso, va sfilato per un bel tratto prima che il naso esca dal fondo. ' +
          'Resta il motore caricato: involucro, isolamento e grano, che non si separano più.',
        parts: ['nozzleBolts', 'nozzle'],
      },
    ],
  },
  camera: { position: [5.41, 3.54, 10.44], target: [0.55, 0.2, 0.0] },

  build(ctx) {
    const root = new THREE.Group();
    root.position.y = ROOT_Y;
    // Gimballed nozzle: everything under `noz` uses model coordinates and pivots at PIV.
    const gimbal = new THREE.Group();
    gimbal.position.y = PIV;
    const noz = new THREE.Group();
    noz.position.y = -PIV;
    noz.updateMatrix();
    gimbal.add(noz);
    root.add(gimbal);

    const addPart = (parent, obj, info) => {
      const w = ctx.part(obj, info);
      parent.add(w);
      return w;
    };

    // ---------- Materials ----------
    const caseMat = cut(new THREE.MeshStandardMaterial({ map: windingTexture(), metalness: 0.3, roughness: 0.42 }));
    const caseBandMat = cut(new THREE.MeshStandardMaterial({ color: 0x2b2f35, metalness: 0.4, roughness: 0.5 }));
    const skirtMat = cut(MAT.paint(0xdfe3e8, 0.55));
    const linerMat = cut(MAT.paint(0x3a3230, 0.88));
    // Grain surfaces are open meshes; its cut face is modelled explicitly (propellant-coloured section planes).
    const grainMat = cut(new THREE.MeshStandardMaterial({ color: 0x9c8682, metalness: 0, roughness: 0.85 }), { cap: false });
    // Burning surface: emissive base colour, intensity animated in update (skipped while highlighted).
    const boreMat = cut(new THREE.MeshStandardMaterial({ color: 0x9c8682, metalness: 0, roughness: 0.8, emissive: 0xff5a1a, emissiveIntensity: 0.3 }), { cap: false });
    const sectionMat = new THREE.MeshStandardMaterial({ color: 0xa8928d, metalness: 0, roughness: 0.92, side: THREE.DoubleSide });
    const phenolic = cut(new THREE.MeshStandardMaterial({ color: 0x3d3029, metalness: 0.1, roughness: 0.72 }));
    const ccMat = cut(new THREE.MeshStandardMaterial({ color: 0x2a2a2e, metalness: 0.3, roughness: 0.45 }));
    const steelCut = cut(MAT.steel());
    const titanium = MAT.titanium();
    const steel = MAT.steel();
    const dark = MAT.darkSteel();
    const chrome = MAT.chrome();
    const rubber = MAT.rubber();
    const actMat = MAT.paint(0x3a424d, 0.5);
    const motorMat = MAT.paint(0x5a6470, 0.45);

    // ---------- Motor case (filament-wound carbon) + skirts ----------
    const caseG = new THREE.Group();
    caseG.add(
      new THREE.Mesh(shellGeo({ R: R_OUT, hf: H_F, ha: H_A }, { R: R_CIN, hf: H_F - 0.025, ha: H_A - 0.025 }), caseMat)
    );
    for (const y of [-0.55, 0.0, 0.55, 1.05]) caseG.add(ring(R_OUT + 0.002, 0.006, y, caseBandMat, 96));
    const RS = R_OUT + 0.016; // skirt outer radius
    caseG.add(new THREE.Mesh(lathe([[R_OUT + 0.002, 1.38], [RS, 1.38], [RS, 1.96], [R_OUT + 0.002, 1.96], [R_OUT + 0.002, 1.38]], 96), skirtMat));
    // The aft skirt, its joint screws and the cable raceway are parts of their own (they come off in the Officina).
    const aftG = new THREE.Group();
    aftG.add(
      new THREE.Mesh(
        lathe([[R_OUT + 0.002, -0.95], [RS, -0.95], [RS, -1.2], [0.682, -1.93], [0.664, -1.93], [R_OUT + 0.002, -1.2], [R_OUT + 0.002, -0.95]], 96),
        skirtMat
      )
    );
    // interface rings (machined aluminium) at the skirt ends
    caseG.add(band(RS + 0.006, R_OUT + 0.002, 1.95, 1.99, steelCut, 96));
    aftG.add(band(0.69, 0.66, -1.965, -1.925, steelCut, 96));
    // bolt rows: skirt flanges and skirt-to-case joints
    const boltMat = cut(MAT.steel());
    caseG.add(boltRing(RS + 0.006, 1.97, 64, boltMat), boltRing(RS, 1.43, 64, boltMat, 0.01, 0.5));
    aftG.add(boltRing(0.69, -1.945, 64, boltMat));
    const aftBolts = boltRing(RS, -0.99, 64, boltMat, 0.01, 0.5, 0.035);
    // systems tunnel (cable raceway) along the case, with clamps
    const PHI_R = 1.25;
    const raceG = new THREE.Group();
    const raceMat = cut(MAT.paint(0xc3c8ce, 0.5));
    const race = new THREE.Mesh(new RoundedBoxGeometry(0.1, 2.3, 0.05, 2, 0.018), raceMat);
    race.position.set(Math.cos(PHI_R) * (R_OUT + 0.024), 0.215, Math.sin(PHI_R) * (R_OUT + 0.024));
    race.rotation.y = Math.PI / 2 - PHI_R;
    raceG.add(race);
    const clamps = new THREE.InstancedMesh(new RoundedBoxGeometry(0.15, 0.025, 0.066, 1, 0.008), cut(MAT.darkSteel()), 7);
    const cm = new THREE.Matrix4();
    for (let i = 0; i < 7; i++) {
      const y = lerp(-0.8, 1.25, i / 6);
      cm.compose(V(Math.cos(PHI_R) * (R_OUT + 0.026), y, Math.sin(PHI_R) * (R_OUT + 0.026)), new THREE.Quaternion().setFromAxisAngle(V(0, 1, 0), race.rotation.y), V(1, 1, 1));
      clamps.setMatrixAt(i, cm);
    }
    clamps.computeBoundingSphere();
    raceG.add(clamps);
    // stage marking on the forward skirt and access doors on the aft skirt
    const mark = new THREE.Mesh(
      new THREE.CylinderGeometry(RS + 0.002, RS + 0.002, 0.15, 24, 1, true, 0.08, 0.7),
      cut(new THREE.MeshStandardMaterial({ map: markingTexture(), metalness: 0.2, roughness: 0.5 }), { cap: false })
    );
    mark.position.y = 1.68;
    caseG.add(mark);
    const doorMat = cut(MAT.paint(0xcfd4da, 0.45));
    for (const a of [0.35, 1.9]) {
      const door = new THREE.Mesh(new RoundedBoxGeometry(0.2, 0.24, 0.016, 2, 0.006), doorMat);
      const r = 0.664;
      door.position.set(Math.cos(a) * r, -1.55, Math.sin(a) * r);
      door.rotation.set(-0.05, Math.PI / 2 - a, 0, 'YXZ');
      aftG.add(door);
    }
    // polar bosses
    const yFo = Y_CT + H_F * Math.sin(Math.acos(R_FO / R_OUT));
    const yAo = Y_CB - H_A * Math.sin(Math.acos(R_AO / R_OUT));
    caseG.add(band(0.2, 0.08, yFo - 0.05, yFo + 0.03, steelCut, 48));
    caseG.add(band(0.36, 0.25, yAo - 0.045, yAo + 0.03, steelCut, 64));
    // TVC actuator brackets on the aft dome, just inside the aft skirt
    for (const [x, z] of [[-0.583, 0], [0, -0.583]]) {
      const b = new THREE.Mesh(new THREE.BoxGeometry(x ? 0.072 : 0.08, 0.1, x ? 0.08 : 0.072), dark);
      b.position.set(x, -1.2, z);
      caseG.add(b);
    }
    addPart(root, caseG, {
      id: 'motorCase',
      name: 'Involucro in fibra di carbonio',
      base:
        'È il “tubo” che contiene tutto: deve resistere a una pressione interna di circa 100 volte quella dell’aria, come una bombola gigante. ' +
        'Nel P120C è fatto di fibra di carbonio avvolta in un unico pezzo: è robustissimo ma pesa poco. Le “gonne” bianche in alto e in basso lo collegano al resto del razzo.',
      expert:
        'Involucro monolitico in composito carbonio/epossidico realizzato per avvolgimento di filamento (filament winding) con fondi ellittici e boss polari metallici; il P120C è circa 13,5 m × 3,4 m. ' +
        'Rispetto all’acciaio dei booster dello Shuttle (segmentati e recuperati) riduce molto la massa inerte: il rapporto di massa propellente/totale supera il 90%. ' +
        'Le gonne (skirt) anteriore e posteriore trasmettono la spinta allo stadio e ospitano avionica e attuatori.',
      explode: [-1.6, 0.15, 0],
    });
    addPart(root, aftG, {
      id: 'aftSkirt',
      name: 'Gonna posteriore',
      base:
        'L’anello bianco svasato in fondo al motore: ripara l’attacco dell’ugello e gli attuatori e collega il motore a ciò che gli sta sotto (la rampa di lancio o un altro stadio). Gli sportelli servono ai tecnici per arrivare a ciò che sta dentro.',
      expert:
        'Struttura cilindro-conica (skirt) fissata all’estremità dell’involucro con una corona di viti radiali: trasmette i carichi tra il motore e le strutture adiacenti e ospita attuatori TVC, cablaggi e protezioni termiche. ' +
        'È un pezzo separato dal recipiente in pressione: si monta a motore già caricato, durante l’integrazione dello stadio.',
      explode: [-1.6, -0.75, 0],
    });
    addPart(root, aftBolts, {
      id: 'aftSkirtBolts',
      name: 'Viti della gonna posteriore',
      base: 'Una corona di viti tutto intorno al motore: tengono la gonna posteriore attaccata all’involucro.',
      expert:
        'Viti radiali che attraversano la gonna e si avvitano in inserti metallici annegati nel bordo dell’involucro in composito: lavorano a taglio e distribuiscono il carico su tutta la circonferenza.',
      explode: [-1.6, -0.3, 0],
    });
    addPart(root, raceG, {
      id: 'raceway',
      name: 'Canalina dei cavi',
      base:
        'Un lungo coperchio che corre sul fianco del motore: dentro passano i cavi che portano corrente e comandi dalla parte alta del razzo fino agli attuatori dell’ugello. Non possono passare dall’interno, perché lì c’è il propellente.',
      expert:
        'Tunnel dei sistemi (raceway): carenatura esterna che protegge dal flusso aerodinamico e dal riscaldamento i cablaggi di potenza e di segnale del TVC e la linea pirotecnica. ' +
        'Di solito è fissata con supporti incollati, per non forare il recipiente in pressione in composito.',
      explode: [-1.3, 0.15, 0.7],
    });

    // ---------- Liner + thermal insulation ----------
    const linerG = new THREE.Group();
    linerG.add(
      new THREE.Mesh(
        shellGeo({ R: R_CIN - 0.001, hf: H_F - 0.026, ha: H_A - 0.026 }, { R: R_G, hf: H_F - 0.058, ha: H_A - 0.058 }),
        linerMat
      )
    );
    addPart(root, linerG, {
      id: 'liner',
      name: 'Isolamento termico e liner',
      base:
        'Uno strato di gomma speciale tra il propellente e l’involucro. Fa due cose: incolla il propellente alla parete e, alla fine, quando la fiamma arriva fin lì, protegge il carbonio dal calore.',
      expert:
        'Isolamento in EPDM caricato (fibre aramidiche/silice) che si carbonizza e si ablade lentamente; il liner è un sottile strato di legante che garantisce l’adesione del grano (case-bonded) e limita la migrazione dei plastificanti. ' +
        'Lo spessore è maggiore sui fondi, esposti più a lungo ai gas caldi. Agli estremi i “flap” di rilascio delle tensioni evitano che il grano si stacchi per le dilatazioni termiche.',
      explode: [-0.85, 0.1, -0.5],
    });

    // ---------- Propellant grain (star bore) ----------
    const grainG = new THREE.Group();
    const geo = grainGeometries();
    const boreMesh = new THREE.Mesh(geo.bore, boreMat);
    const endsMesh = new THREE.Mesh(geo.ends, grainMat);
    const outerMesh = new THREE.Mesh(new THREE.CylinderGeometry(R_G, R_G, G_TOP - G_BOT, 96, 1, true), grainMat);
    outerMesh.position.y = (G_TOP + G_BOT) / 2;
    // Section faces on the cut plane (z = 0) so the grain reads as solid in the cutaway.
    const gLen = G_TOP - G_BOT;
    const secR = new THREE.Mesh(new THREE.PlaneGeometry(1, gLen).translate(0.5, 0, 0), sectionMat);
    const secL = new THREE.Mesh(new THREE.PlaneGeometry(1, gLen).translate(-0.5, 0, 0), sectionMat);
    secR.position.y = secL.position.y = (G_TOP + G_BOT) / 2;
    grainG.add(boreMesh, endsMesh, outerMesh, secR, secL);
    addPart(root, grainG, {
      id: 'grain',
      name: 'Grano di propellente (foro a stella)',
      base:
        'È il propellente vero e proprio: una gomma solida, grigio-rosata, che contiene sia il combustibile sia l’ossigeno. Ne servono circa 142 tonnellate, colate come un budino e lasciate indurire nell’involucro. ' +
        'Il foro a stella al centro è la superficie che brucia: guarda come si allarga muovendo il cursore del tempo.',
      expert:
        'Propellente composito HTPB: ~69% perclorato d’ammonio (ossidante), ~19% alluminio in polvere, ~12% legante polibutadiene a terminazione ossidrilica con agenti di cura e additivi. Densità ~1,8 kg/L, temperatura di fiamma ~3300 °C. ' +
        'Il grano è colato sotto vuoto e polimerizzato in forno attorno a un mandrino che dà la forma del foro. Qui una stella a 7 punte: area di combustione alta all’inizio, fase regressiva, poi leggermente progressiva e coda finale. ' +
        'I booster dello Shuttle usavano una stella a 11 punte nel segmento anteriore per ridurre la spinta al passaggio della massima pressione dinamica.',
      explode: [0, 0, 0],
    });
    // Burning edge: thin glowing strips on the cut faces (own material, not highlighted).
    const stripMat = MAT.glow(0xff7a2a, 2.2, 0.95);
    const stripR = new THREE.Mesh(new THREE.PlaneGeometry(0.016, gLen), stripMat);
    const stripL = new THREE.Mesh(new THREE.PlaneGeometry(0.016, gLen), stripMat);
    stripR.position.set(0, (G_TOP + G_BOT) / 2, 0.002);
    stripL.position.copy(stripR.position);
    grainG.add(stripR, stripL);
    const sectionMeshes = [secR, secL];

    const boreR = new Float32Array(NB);
    let lastB = -1;
    let boreMean = R_B;
    function poseGrain(b) {
      if (b === lastB) return;
      lastB = b;
      burnAt(b, boreR);
      geo.set(boreR);
      let s = 0;
      for (let i = 0; i < NB; i++) s += boreR[i];
      boreMean = s / NB;
      const r0 = boreR[0];
      const rPi = boreR[NB / 2];
      secR.position.x = r0;
      secR.scale.x = Math.max(1e-3, R_G - r0);
      secL.position.x = -rPi;
      secL.scale.x = Math.max(1e-3, R_G - rPi);
      stripR.position.x = r0 + 0.006;
      stripL.position.x = -rPi - 0.006;
      const left = b < 0.999;
      boreMesh.visible = endsMesh.visible = outerMesh.visible = left;
      for (const m of sectionMeshes) m.visible = left;
    }

    // ---------- Igniter (pyrogen) + safe & arm device ----------
    const ign = new THREE.Group();
    ign.add(
      new THREE.Mesh(
        // closure flange seated on the forward polar boss (top of the boss: yFo + 0.03)
        lathe([[0, 1.99], [0.19, 1.99], [0.19, 1.9], [0.175, yFo + 0.03], [0.07, yFo + 0.03], [0.07, 1.52], [0.055, 1.45], [0.035, 1.4], [0.045, 1.37], [0.03, 1.37]], 32),
        steel
      )
    );
    const saG = new THREE.Group();
    const sa = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.08, 0.11), MAT.paint(0x6b7480, 0.5));
    sa.position.y = 2.03;
    const saCap = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.03, 16), MAT.paint(0xd2502a, 0.5));
    saCap.position.set(0.035, 2.085, 0);
    // firing line from the S&A to its connector on the forward skirt wall
    const cable = new THREE.Mesh(
      new THREE.TubeGeometry(new THREE.CatmullRomCurve3([V(-0.07, 2.03, 0), V(-0.25, 2.02, -0.05), V(-0.45, 1.97, -0.12), V(-0.57, 1.9, -0.21)]), 24, 0.012, 6),
      rubber
    );
    const plug = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.05), dark);
    plug.position.set(-0.565, 1.89, -0.205);
    plug.rotation.y = -0.35;
    saG.add(sa, saCap, cable, plug);
    addPart(root, saG, {
      id: 'safeArm',
      name: 'Dispositivo di sicurezza (Safe & Arm)',
      base:
        'Il “lucchetto” dell’accensione: finché non viene armato, poco prima del lancio, tiene interrotta la strada tra il comando elettrico e la carica dell’accenditore. Così una scintilla o un guasto non possono accendere il motore.',
      expert:
        'Safe & Arm Device: una barriera meccanica (un rotore) tiene disallineata la catena pirotecnica tra i detonatori e la carica di trasferimento; un attuatore la ruota in posizione “armato” solo nella sequenza finale di lancio. ' +
        'Lo stato sicuro/armato è segnalato a terra e spesso visibile anche a occhio.',
      explode: [0, 1.5, 0],
    });
    addPart(root, screwCircle(0.15, 1.99, 12, steel, 0.011, 0.14), {
      id: 'igniterBolts',
      name: 'Bulloni dell’accenditore',
      base: 'Dodici bulloni in cerchio che stringono l’accenditore sulla bocca anteriore del motore, a tenuta di gas.',
      expert:
        'Fissano la flangia dell’accenditore al boss polare anteriore (l’anello metallico annegato nel fondo dell’involucro); la tenuta è affidata a O-ring. ' +
        'Devono reggere la pressione di camera, circa 100 bar, che spinge l’accenditore verso l’esterno.',
      explode: [0, 1.22, 0],
    });
    addPart(root, ign, {
      id: 'igniter',
      name: 'Accenditore',
      base:
        'Un piccolo razzo dentro il razzo: quando arriva il comando spara una fiammata lungo il foro centrale e accende tutta la superficie del propellente in un attimo. ' +
        'Il blocchetto sopra è il dispositivo di sicurezza: finché non viene “armato”, l’accensione è fisicamente impossibile.',
      expert:
        'Accenditore pirogeno: un piccolo motore a solido con grano ad alta superficie, innescato da una catena pirotecnica (iniziatori, carica di rinforzo). ' +
        'Il Safe & Arm interrompe meccanicamente la catena fino all’armamento poco prima del lancio. Il getto deve riscaldare tutta la superficie del foro in poche centinaia di millisecondi per una salita in pressione regolare.',
      explode: [0, 1.0, 0],
    });
    const ignFlameMat = MAT.gas(0xffb347, 0.6);
    const ignFlame = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.9, 20, 1, true).translate(0, -0.45, 0), ignFlameMat);
    ignFlame.position.y = 1.37;
    root.add(ignFlame); // not inside the part: an invisible flame must not catch clicks

    // ---------- Submerged nozzle (carbon-phenolic) ----------
    const nozG = new THREE.Group();
    const TH = 0.05; // throat insert thickness
    const thick = (y) => lerp(0.085, 0.022, clamp((Y_TI1 - y) / (Y_TI1 - Y_E), 0, 1));
    const inner = [
      ...profSlice(Y_NOSE, Y_TI0),
      ...profSlice(Y_TI0, Y_TI1, TH),
      ...profSlice(Y_TI1, Y_E),
    ];
    const outer = [];
    for (let i = 0; i <= 24; i++) {
      const y = lerp(Y_E, -1.61, i / 24);
      outer.push([rAt(y) + thick(y), y]);
    }
    outer.push([0.31, -1.61], [0.31, -1.57], [0.215, -1.57], [0.215, -1.4], [0.25, -1.26], [0.272, -1.16], [0.258, -1.09]);
    nozG.add(new THREE.Mesh(lathe([...inner, ...outer, inner[0]], 72), phenolic));
    const yAct = -1.88;
    nozG.add(ring(rAt(yAct) + thick(yAct) + 0.02, 0.02, yAct, steelCut)); // actuation ring
    nozG.add(ring(R_E + 0.03, 0.014, Y_E + 0.01, steelCut));
    for (const y of [-2.12, -2.32]) nozG.add(ring(rAt(y) + thick(y) + 0.006, 0.009, y, steelCut, 72)); // exit cone stiffeners
    nozG.add(band(0.32, 0.2, -1.615, -1.575, steelCut, 64)); // nozzle flange to flex seal
    for (const [x, z] of [[-1, 0], [0, -1]]) {
      const r = rAt(yAct) + thick(yAct) + 0.05;
      const lug = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.06, 0.06), dark);
      lug.position.set(x * r, yAct, z * r);
      nozG.add(lug);
    }
    addPart(noz, nozG, {
      id: 'nozzle',
      name: 'Ugello sommerso',
      base:
        'L’ugello prima si stringe e poi si allarga a campana: i gas accelerano fino a circa 10.000 km/h e spingono il razzo in avanti. ' +
        'È “sommerso”: la sua parte iniziale entra dentro il motore, così il razzo è più corto. Il materiale scuro si consuma piano piano portando via il calore.',
      expert:
        'Ugello de Laval sommerso con convergente annidato nel fondo posteriore: riduce la lunghezza dello stadio ma espone la parte esterna del naso al flusso di ricircolo. ' +
        'Divergente in carbonio-fenolico ablativo (nessun raffreddamento attivo: la resina pirolizza e il carbonio si erode lentamente), struttura esterna in composito/metallo con l’anello degli attuatori. ' +
        'La portata di gas è circa 1 t/s; parte del getto è fatta di goccioline di Al₂O₃ fuso, che non si espandono come il gas (perdite bifase di qualche punto percentuale di Isp).',
      explode: [0, -0.95, 0],
    });

    // ---------- Throat insert (carbon-carbon) ----------
    const ti = new THREE.Group();
    const tiIn = profSlice(Y_TI0, Y_TI1);
    const tiOut = profSlice(Y_TI0, Y_TI1, TH).reverse();
    ti.add(new THREE.Mesh(lathe([...tiIn, ...tiOut, tiIn[0]], 64), ccMat));
    addPart(noz, ti, {
      id: 'throatInsert',
      name: 'Inserto di gola (carbonio-carbonio)',
      base:
        'La gola è il punto più stretto e più tormentato: qui passa tutto il gas, caldissimo e veloce. Per questo è fatta di un materiale speciale, il carbonio-carbonio, che resiste dove i metalli fonderebbero.',
      expert:
        'Inserto in composito C/C (fibre di carbonio in matrice di carbonio, densificato per CVI), resistente a shock termico ed erosione oltre i 3000 °C. ' +
        'L’erosione della gola durante la combustione aumenta l’area At e abbassa la pressione in camera: va prevista nel progetto della curva di spinta. In sezione di gola il flusso è sonico (M = 1).',
      explode: [0.9, -0.95, 0.3],
    });
    const throatGlowMat = MAT.glow(0xff8a3a, 1.2, 0.85);
    const throatGlow = new THREE.Mesh(lathe(profSlice(Y_TI0 - 0.02, Y_TI1 + 0.02, -0.004), 48), throatGlowMat);
    cut(throatGlowMat);
    ti.add(throatGlow);

    // ---------- Flex seal (static, on the aft boss) ----------
    const flex = new THREE.Group();
    for (let j = 0; j < 7; j++) {
      const y0 = -1.43 - j * 0.02;
      const r = 0.012 * Math.sin((j / 6) * Math.PI); // slight spherical bulge
      flex.add(band(0.29 + r, 0.218 + r * 0.5, y0 - 0.018, y0, j % 2 ? steel : rubber, 64));
    }
    flex.add(band(0.355, 0.216, -1.45, -1.428, steel, 64)); // mounting flange, bolted under the aft polar boss
    addPart(root, screwCircle(0.325, -1.45, 16, steel, 0.011, 0.06, -1), {
      id: 'nozzleBolts',
      name: 'Bulloni della flangia dell’ugello',
      base: 'Il cerchio di bulloni che tiene tutto l’ugello, con il suo giunto flessibile, attaccato al fondo del motore.',
      expert:
        'Collegano la flangia fissa del gruppo ugello (l’estremità anteriore del flex seal) al boss polare posteriore dell’involucro. ' +
        'Tutta la spinta che nasce sull’ugello e la forza della pressione sul giunto passano da questa corona di bulloni, con O-ring di tenuta tra le due facce.',
      explode: [0, -0.64, 0.45],
    });
    addPart(root, flex, {
      id: 'flexSeal',
      name: 'Giunto flessibile (flex seal)',
      base:
        'Un “sandwich” di anelli di gomma e di metallo che collega l’ugello al motore. È a tenuta di gas ma si può piegare un poco: così l’ugello può inclinarsi per guidare il razzo.',
      expert:
        'Giunto laminato a strati sferici concentrici alternati di elastomero e shim metallici (o compositi), centrati sul punto di rotazione dell’ugello. ' +
        'Sotto pressione la camera lo comprime; l’elastomero lavora a taglio permettendo deflessioni di alcuni gradi con una coppia di attuazione relativamente bassa, senza parti che strisciano.',
      explode: [0, -0.45, 0.45],
    });

    // ---------- TVC actuators (electromechanical) ----------
    const tvc = new THREE.Group();
    const actBodyGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.3, 20).translate(0, -0.18, 0);
    const rodGeo = new THREE.CylinderGeometry(0.016, 0.016, 1, 12).translate(0, -0.5, 0);
    const eyeGeo = new THREE.SphereGeometry(0.028, 12, 10);
    const actuators = [];
    const makeActuator = (anchor, attach) => {
      const g = new THREE.Group();
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.12, 16), motorMat);
      motor.position.set(0.065, -0.12, 0);
      const rod = new THREE.Mesh(rodGeo, chrome);
      rod.position.y = -0.24;
      const eye = new THREE.Mesh(eyeGeo, dark);
      g.add(new THREE.Mesh(actBodyGeo, actMat), new THREE.Mesh(eyeGeo, dark), motor, rod, eye);
      tvc.add(g);
      actuators.push({ g, rod, eye, anchor: V(...anchor), attach: V(...attach) });
    };
    const rLug = rAt(yAct) + thick(yAct) + 0.05;
    makeActuator([-0.57, -1.2, 0], [-rLug, yAct, 0]);
    makeActuator([0, -1.2, -0.57], [0, yAct, -rLug]);
    addPart(root, tvc, {
      id: 'tvc',
      name: 'Attuatori TVC',
      base:
        'Due “braccia” elettriche che inclinano l’ugello di pochi gradi, una per il su-giù e una per il destra-sinistra. Così il computer di bordo tiene il razzo sulla rotta giusta.',
      expert:
        'Attuatori elettromeccanici (EMA: motore elettrico brushless + vite a ricircolo di sfere) montati a 90° tra la gonna posteriore e l’anello dell’ugello; sul Vega sostituiscono i sistemi idraulici con batterie ad alta potenza. ' +
        'Deflessione di alcuni gradi e banda passante di qualche hertz, sufficienti a controllare beccheggio e imbardata durante il volo atmosferico.',
      explode: [-0.9, -0.7, 0],
    });
    // Hinge pins: instances 0-1 on the case brackets (fixed), 2-3 on the nozzle lugs (they follow the gimbal).
    const pins = new THREE.InstancedMesh(screwGeo(0.016, 0.085), steel, 4);
    const pinAxis = [V(0, 0, 1), V(1, 0, 0)]; // heads towards the viewer's side
    const pinQ = pinAxis.map((d) => new THREE.Quaternion().setFromUnitVectors(V(0, 1, 0), d));
    const pinM = new THREE.Matrix4();
    const ONE = V(1, 1, 1);
    const setPin = (i, k, p, off) => pins.setMatrixAt(i, pinM.compose(tmpD.copy(pinAxis[k]).multiplyScalar(off).add(p), pinQ[k], ONE));
    addPart(root, pins, {
      id: 'tvcPins',
      name: 'Perni degli attuatori',
      base: 'Quattro perni, come quelli di una cerniera: due fissano gli attuatori al motore e due all’ugello, lasciandoli liberi di ruotare mentre spingono.',
      expert:
        'Ogni attuatore è una biella a lunghezza variabile incernierata alle due estremità con snodi sferici e perni a taglio: così trasmette solo forze lungo il proprio asse e non si impunta quando l’ugello ruota anche sull’altro piano.',
      explode: [-0.75, -0.7, 0.25],
    });

    const tmpP = new THREE.Vector3();
    const tmpD = new THREE.Vector3();
    const DOWN = V(0, -1, 0);
    actuators.forEach((a, k) => setPin(k, k, a.anchor, 0.047));
    function poseActuators() {
      gimbal.updateMatrix();
      pins.instanceMatrix.needsUpdate = true;
      for (const [k, a] of actuators.entries()) {
        tmpP.copy(a.attach).applyMatrix4(noz.matrix).applyMatrix4(gimbal.matrix);
        setPin(k + 2, k, tmpP, 0.037);
        tmpD.subVectors(tmpP, a.anchor);
        const d = tmpD.length();
        tmpD.divideScalar(d);
        a.g.position.copy(a.anchor);
        a.g.quaternion.setFromUnitVectors(DOWN, tmpD);
        a.rod.scale.y = Math.max(0.01, d - 0.24);
        a.eye.position.y = -d;
      }
    }

    // ---------- Thrust-time chart ----------
    const chart = new THREE.Group();
    chart.position.set(1.95, 0.62, 0);
    const CW = 1.5;
    const CH = 0.94;
    const { tex, plot } = chartTexture();
    chart.add(
      new THREE.Mesh(
        new THREE.PlaneGeometry(CW, CH),
        new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false, side: THREE.DoubleSide })
      )
    );
    addPart(root, chart, {
      id: 'thrustCurve',
      name: 'Curva di spinta',
      base:
        'Il grafico mostra quanta spinta dà il motore secondo per secondo. Non la decide il pilota: è “scritta” nella forma del foro del propellente. Il puntino segue il cursore del tempo.',
      expert:
        'F(t) = ṁ·vₑ + (pₑ − pₐ)·Aₑ, con ṁ = ρₚ·r·Ab: la portata dipende dall’area di combustione Ab e dalla velocità di regressione r = a·pcⁿ. ' +
        'In questo modello la curva è calcolata dalla geometria del grano a stella (Ab elevata a 1/(1−n), n = 0,35), scalata ai valori del P120C: ~4500 kN di picco e ~135 s di combustione. ' +
        'Il profilo reale del P120C è studiato per ridurre la spinta attorno alla massima pressione dinamica (Max Q).',
      explode: [0.6, 0, 0],
    });
    const markMat = MAT.glow(0xfff1c4, 2.5);
    const marker = new THREE.Mesh(new THREE.SphereGeometry(0.025, 16, 12), markMat);
    const cursor = new THREE.Mesh(new THREE.PlaneGeometry(0.006, CH * (plot.v1 - plot.v0)), MAT.glow(0x7fd8ff, 1.5, 0.6));
    cursor.position.y = (plot.v0 + plot.v1 - 1) * 0.5 * CH;
    marker.position.z = cursor.position.z = 0.01;
    chart.add(marker, cursor);
    const chartX = (b) => (lerp(plot.u0, plot.u1, b) - 0.5) * CW;
    const chartY = (f) => (lerp(plot.v0, plot.v1, f) - 0.5) * CH;

    // ---------- Hot gas in the nozzle (not clickable) ----------
    const nozGasMat = cut(MAT.gas(0xff7a2a, 0.15));
    const nozGas = new THREE.Mesh(lathe([[0, Y_NOSE], ...profSlice(Y_NOSE, Y_E + 0.01, -0.012), [0, Y_E + 0.01]], 48), nozGasMat);
    noz.add(nozGas);

    // ---------- Exhaust plume (aluminised: bright, white smoke) ----------
    const plume = new THREE.Group();
    plume.position.y = Y_E;
    noz.add(plume);
    const plumeBody = new THREE.Group();
    plume.add(plumeBody);
    const fadeLathe = (pts, fade) => {
      const g = lathe(pts, 48);
      const pos = g.attributes.position;
      const col = new Float32Array(pos.count * 3);
      for (let i = 0; i < pos.count; i++) col.fill(clamp(fade(-pos.getY(i)), 0, 1), i * 3, i * 3 + 3);
      g.setAttribute('color', new THREE.BufferAttribute(col, 3));
      return g;
    };
    const gasMat = (color, opacity) => {
      const m = MAT.gas(color, opacity);
      m.vertexColors = true;
      return m;
    };
    const outerPts = [[0.38, 0], [0.35, -0.08], [0.34, -0.18], [0.37, -0.3], [0.45, -0.48], [0.56, -0.66], [0.68, -0.84], [0.8, -1.0]];
    const outerMat = gasMat(0xffa64a, 0.25);
    const haloMat = gasMat(0xff6a20, 0.08);
    const coreMat = gasMat(0xfff8e8, 0.8);
    plumeBody.add(
      new THREE.Mesh(fadeLathe(outerPts, (d) => Math.pow(1 - d, 1.2)), outerMat),
      new THREE.Mesh(fadeLathe(outerPts.map(([r, y]) => [r * 1.3 + 0.04, y * 1.08]), (d) => 1 - d), haloMat),
      new THREE.Mesh(fadeLathe([[0.37, 0], [0.33, -0.1], [0.25, -0.22], [0.14, -0.34], [0.05, -0.44], [0, -0.46]], (d) => 1 - d * 1.4), coreMat)
    );
    const diamondGeo = new THREE.SphereGeometry(1, 20, 14);
    const diamonds = [0, 1].map((i) => {
      const m = new THREE.Mesh(diamondGeo, MAT.gas(0xfff4dc, 0.35));
      m.scale.set(0.12 - 0.03 * i, 0.2, 0.12 - 0.03 * i);
      plume.add(m);
      return m;
    });
    const exitLight = new THREE.PointLight(0xffa860, 2, 6, 2);
    exitLight.position.set(0, Y_E - 0.5, 0.3);
    noz.add(exitLight);

    // ---------- Flows ----------
    // Combustion gas: down the bore, into the submerged nose, through the throat.
    const gasY = (t) =>
      t < 0.6 ? lerp(G_TOP - 0.05, G_BOT, t / 0.6) : t < 0.75 ? lerp(G_BOT, Y_T, (t - 0.6) / 0.15) : lerp(Y_T, Y_E, Math.pow((t - 0.75) / 0.25, 1.2));
    const gasFlow = flow({
      path: (t, out) => out.set(0, gasY(t), 0),
      count: 300,
      size: 0.055,
      colors: [0xffb347, 0xff8a2a, 0xff5a1f, 0xff7a30],
      spread: (t) =>
        t < 0.6 ? boreMean * 0.75 : t < 0.75 ? lerp(boreMean * 0.75, R_T * 0.8, (t - 0.6) / 0.15) : rAt(gasY(t)) * 0.82,
      speed: 0.42,
    });
    root.add(gasFlow.object);
    // Igniter jet down the bore.
    const ignFlow = flow({
      path: (t, out) => out.set(0, lerp(1.36, 0.1, t), 0),
      count: 140,
      size: 0.07,
      colors: [0xfff1c4, 0xffb347, 0xff5a1f],
      spread: (t) => 0.02 + 0.08 * t,
      speed: 1.2,
    });
    root.add(ignFlow.object);
    let plumeLen = 2;
    const plumeFlow = flow({
      path: (t, out) => out.set(0, Y_E - 0.03 - t * plumeLen, 0),
      count: 600,
      size: 0.11,
      colors: [0xfffaf0, 0xffd080, 0xff9a40, 0xa0502a],
      spread: (t) => 0.28 + 0.5 * t,
      speed: 1.4,
      opacity: 0.85,
    });
    noz.add(plumeFlow.object);
    // Al2O3 smoke: normal blending so the white cloud stays visible.
    const smokeFlow = flow({
      path: (t, out) => out.set(0.08 * t, Y_E - 0.9 - t * 1.6, 0.04 * t),
      count: 240,
      size: 0.28,
      colors: [0xf4f2f6, 0xd6d3dc, 0xa8a4b4],
      spread: (t) => 0.35 + 0.55 * t,
      speed: 0.35,
      opacity: 0.32,
    });
    smokeFlow.material.blending = THREE.NormalBlending;
    noz.add(smokeFlow.object);
    [gasFlow, ignFlow, plumeFlow, smokeFlow].forEach((f) => ctx.addFlow(f));

    function posePlume(T, fl) {
      plumeLen = 0.9 + 1.4 * T;
      const w = 0.85 + 0.2 * T;
      plumeBody.scale.set(w, plumeLen, w);
      outerMat.opacity = (0.12 + 0.2 * T) * fl;
      haloMat.opacity = (0.03 + 0.07 * T) * fl;
      coreMat.opacity = clamp((0.4 + 0.5 * T) * fl, 0, 1);
      diamonds.forEach((d, i) => {
        d.position.y = -(0.42 + i * 0.5) * (0.8 + 0.3 * T);
        d.material.opacity = (0.35 - 0.12 * i) * T * fl;
      });
    }

    poseGrain(0.35);
    posePlume(burnAt(0.35).thrust, 1);
    poseActuators();

    return {
      root,
      cycleRate() {
        return 0.5;
      },
      update(state) {
        const b = clamp(state.throttle, 0, 1);
        const { thrust: T } = burnAt(b);
        const { dt, time } = state;
        const c = state.cycle * Math.PI * 2;
        const fl = 1 + 0.05 * Math.sin(c * 7) + 0.035 * Math.sin(c * 13 + 1.7);
        const burning = b < 0.999 && T > 0.004;
        const ignite = 1 - smooth(clamp(b / 0.05, 0, 1)); // igniter jet fades after the first seconds
        const assembled = (state.explode || 0) < 0.05;

        poseGrain(b);
        // TVC: gentle steering while thrusting, centred after burnout and while the motor is taken apart.
        const k = burning && assembled ? 1 : 0;
        gimbal.rotation.set(0.04 * k * Math.sin(time * 0.6 + 1.0), 0, 0.05 * k * Math.sin(time * 0.8));
        poseActuators();

        // Glows
        const bm = boreMesh.material; // the part's clone
        if (bm.emissive.equals(bm.userData.baseEmissive ?? bm.emissive)) bm.emissiveIntensity = burning ? (0.08 + 0.35 * T) * fl : 0;
        stripMat.emissiveIntensity = burning ? (1.0 + 2.0 * T) * fl : 0;
        stripR.visible = stripL.visible = burning;
        throatGlowMat.emissiveIntensity = burning ? (0.6 + 1.6 * T) * fl : 0;
        throatGlow.visible = burning;
        nozGasMat.opacity = burning ? (0.06 + 0.12 * T) * fl : 0;
        ignFlameMat.opacity = 0.65 * ignite * fl;
        ignFlame.visible = ignite > 0.01 && assembled;
        ignFlame.scale.set(1, 0.6 + 0.4 * fl, 1);
        exitLight.intensity = burning ? (1.0 + 4 * T) * fl : 0;

        posePlume(T, fl);
        plumeFlow.material.opacity = 0.5 + 0.4 * T;
        gasFlow.update(dt, 0.3 + 0.7 * T);
        ignFlow.update(dt, 1);
        plumeFlow.update(dt, 0.6 + 0.4 * T);
        smokeFlow.update(dt, 0.6 + 0.4 * T);

        // Chart marker
        marker.position.set(chartX(b), chartY(T), 0.01);
        cursor.position.x = chartX(b);
        markMat.emissiveIntensity = 1.8 + 0.6 * fl * (burning ? 1 : 0);

        // Gas volumes and flows: only while burning and assembled.
        const live = burning && assembled;
        plume.visible = plumeFlow.object.visible = smokeFlow.object.visible = live;
        gasFlow.object.visible = nozGas.visible = live; // the gas volume must not stay behind when the nozzle leaves
        ignFlow.object.visible = ignite > 0.01 && assembled;
      },
      readout(state) {
        const b = clamp(state.throttle, 0, 1);
        const { thrust: T, mass } = burnAt(b);
        return [
          { label: 'Tempo', value: Math.round(b * BURN_S), unit: 's' },
          { label: 'Spinta', value: Math.round((F_MAX * T) / 10) * 10, unit: 'kN' },
          { label: 'Pressione camera', value: Math.round(PC_MAX * T), unit: 'bar' },
          { label: 'Propellente residuo', value: (PROP_T * mass).toFixed(1), unit: 't' },
        ];
      },
    };
  },
};
