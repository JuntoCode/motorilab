// Toyota-style power-split full hybrid (THS / e-CVT): Atkinson inline-4 -> planetary gear set
// (carrier = engine, sun = MG1, ring = counter drive to wheels) + MG2 on the counter shaft,
// power control unit (inverters + boost converter) and a small high-voltage battery.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, pipe, flow, clamp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const OFFSET = new THREE.Vector3(0.45, -0.15, 0); // centres the model (z must stay 0 for the cutaway)
const YA = 0.75;                 // main axis: crankshaft, MG1, planetary
// Planetary gear set (power-split device)
const S = 30, R = 78, P = (R - S) / 2;  // sun, ring, planet teeth (P = 24)
const MP = 0.0098;
const R_S = (MP * S) / 2;
const R_C = (MP * (S + P)) / 2;  // carrier (planet centre) radius
// Ring outer teeth = counter drive gear
const ZCD = 52, ZCN = 40, MC = 0.0178;
const YC = YA - (MC * (ZCD + ZCN)) / 2;      // counter shaft
// MG2 reduction (MG2 pinion -> reduction driven gear on the counter shaft)
const ZRD = 58, ZMP = 21, MR = 0.0165;
const YM2 = YC - (MR * (ZRD + ZMP)) / 2;     // MG2 axis
// Final drive
const ZFP = 18, ZFR = 58, MF = 0.0175;
const YD = YC - (MF * (ZFP + ZFR)) / 2;      // differential / half-shaft axis
const X_RD = -0.36;   // reduction gear plane
const X_FD = 0.36;    // final drive plane
const X_MG1 = -0.78;
const X_MG2 = -0.88;
const WHEEL_X = 1.75;
// Engine
const CYL_X = [-2.5, -2.15, -1.8, -1.45];
const FIRE = [0, 0.75, 0.25, 0.5]; // 1-3-4-2: cycle offset of each cylinder (720° units)
const CR = 0.11;   // crank radius
const CL = 0.36;   // conrod length
const DECK = YA + 0.6;
// Colours
const CURRENT = 0xd8ff4a;
const REGEN_C = 0x4ade80;
const FUEL = 0xffc04d;
const MECH = 0xff9a3c;
const FLAME = 0xff5a1f;

// ---------- small builders ----------
const alongX = (g) => g.rotateZ(-Math.PI / 2);

function box(w, h, d, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  return m;
}

function ribbedProfile(ri, ro, len, n, dOut, dIn) {
  const h = len / 2;
  const pts = [];
  for (let i = 0; i <= n; i++) pts.push([ro - (i % 2) * dOut, -h + (i / n) * len]);
  for (let i = n; i >= 0; i--) pts.push([ri + (i % 2) * dIn, -h + (i / n) * len]);
  pts.push([ro, -h]);
  return pts;
}

// Shape-space point at polar angle a in the YZ plane (after extrudeX: y = r cos a, z = r sin a).
const sp = (r, a) => [-r * Math.sin(a), r * Math.cos(a)];

function polyShape(pts, Cls = THREE.Shape) {
  const s = new Cls();
  pts.forEach(([u, v], i) => (i ? s.lineTo(u, v) : s.moveTo(u, v)));
  s.closePath();
  return s;
}

function circlePath(u, v, r) {
  const p = new THREE.Path();
  p.absarc(u, v, r, 0, TAU, true);
  return p;
}

function extrudeX(shapes, width, curveSegments = 16) {
  const g = new THREE.ExtrudeGeometry(shapes, { depth: width, bevelEnabled: false, curveSegments });
  g.translate(0, 0, -width / 2);
  g.rotateY(Math.PI / 2); // shape (u,v) -> (z=-u, y=v), extrusion along X
  return g;
}

// Tooth outline; tooth k centred at polar angle 2*PI*k/N. Internal teeth point inward.
function toothPts(teeth, m, internal = false) {
  const rp = (m * teeth) / 2;
  const tip = internal ? rp - m : rp + m;
  const root = internal ? rp + 1.25 * m : rp - 1.25 * m;
  const prof = [[root, -0.62], [tip, -0.3], [tip, 0.3], [root, 0.62]];
  const hp = Math.PI / teeth;
  const pts = [];
  for (let k = 0; k < teeth; k++) for (const [r, f] of prof) pts.push(sp(r, (TAU * k) / teeth + f * hp));
  return pts;
}

function gearGeo(teeth, m, width, bore, holes = 0, holeR = 0, holeRing = 0) {
  const s = polyShape(toothPts(teeth, m));
  if (bore) s.holes.push(circlePath(0, 0, bore));
  for (let i = 0; i < holes; i++) {
    const [u, v] = sp(holeRing, (TAU * i) / holes);
    s.holes.push(circlePath(u, v, holeR));
  }
  return extrudeX(s, width, 10);
}

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

function bellows(x0, x1, rMin, rMax, n = 6) {
  const pts = [];
  for (let i = 0; i <= n * 2; i++) pts.push([i % 2 ? rMax : rMin, x0 + ((x1 - x0) * i) / (n * 2)]);
  return alongX(lathe(pts, 20));
}

function polyPath(list) {
  const v = list.map((p) => new THREE.Vector3(...p));
  const acc = [0];
  for (let i = 1; i < v.length; i++) acc.push(acc[i - 1] + v[i].distanceTo(v[i - 1]));
  const total = acc[acc.length - 1];
  return (t, out) => {
    const d = clamp(t, 0, 1) * total;
    let i = 1;
    while (i < acc.length - 1 && acc[i] < d) i++;
    const f = (d - acc[i - 1]) / (acc[i] - acc[i - 1] || 1);
    return out.lerpVectors(v[i - 1], v[i], f);
  };
}

const curve = (pts) => new THREE.CatmullRomCurve3(pts.map((p) => new THREE.Vector3(...p)), false, 'centripetal');

function glowMat(base, color = CURRENT) {
  base.emissive.setHex(color);
  base.emissiveIntensity = 0;
  return base;
}

// Simplified permanent-magnet motor-generator around X, centred at the group origin.
function buildMG(len, housingMat) {
  const stator = new THREE.Group();
  stator.add(new THREE.Mesh(alongX(lathe(ribbedProfile(0.2, 0.33, len, 30, 0.005, 0), 64)), cut(MAT.darkSteel())));
  const slotMat = cut(MAT.copper());
  const slotGeo = new THREE.BoxGeometry(len + 0.02, 0.05, 0.022);
  for (let i = 0; i < 24; i++) {
    const a = (i / 24) * TAU;
    const s = new THREE.Mesh(slotGeo, slotMat);
    s.position.set(0, Math.cos(a) * 0.235, Math.sin(a) * 0.235);
    s.rotation.x = a;
    stator.add(s);
  }
  stator.add(new THREE.Mesh(alongX(pipe(0.4, 0.355, len + 0.18, 64)), housingMat));
  const wallGeo = alongX(pipe(0.4, 0.075, 0.03, 64));
  for (const e of [-1, 1]) {
    const w = new THREE.Mesh(wallGeo, housingMat);
    w.position.x = e * (len / 2 + 0.105);
    stator.add(w);
    stator.add(bolts(boltCircleX(e * (len / 2 + 0.128), 0, 0.37, 12, 0.13), cut(MAT.chrome(), { cap: false }), 0.011, 0.014));
  }
  const ARC = TAU / 6 - 0.06;
  const coilGeo = new THREE.TorusGeometry(0.262, 0.03, 8, 14, ARC);
  coilGeo.rotateY(Math.PI / 2);
  coilGeo.scale(1.4, 1, 1);
  const coils = [];
  for (const e of [-1, 1]) {
    for (let k = 0; k < 6; k++) {
      const a = (k * TAU) / 6;
      const m = new THREE.Mesh(coilGeo, glowMat(MAT.copper()));
      m.position.x = e * (len / 2 + 0.035);
      m.rotation.x = a + Math.PI / 2 - ARC / 2;
      stator.add(m);
      coils.push({ mesh: m, a });
    }
  }
  const rotor = new THREE.Group();
  rotor.add(new THREE.Mesh(alongX(lathe(ribbedProfile(0.065, 0.19, len, 30, 0.003, 0), 48)), MAT.steel()));
  const magN = MAT.paint(0xd2362f, 0.35);
  const magS = MAT.paint(0x2f62d0, 0.35);
  const magGeo = new THREE.BoxGeometry(len - 0.02, 0.012, 0.1);
  for (let k = 0; k < 8; k++) {
    const a = (k / 8) * TAU;
    const m = new THREE.Mesh(magGeo, k % 2 ? magS : magN);
    m.position.set(0, Math.cos(a) * 0.188, Math.sin(a) * 0.188);
    m.rotation.x = a;
    rotor.add(m);
  }
  return { stator, coils, rotor };
}

// Driving scenario from the throttle (visual speeds in rev/s, powers in kW).
function scenario(thr) {
  const ring = 0.12 + 0.55 * thr;
  if (thr < 0.03) return { id: 'regen', name: 'Frenata rigenerativa', ring: 0.35, carrier: 0, eng: 0, mg2: -30, mg1: 0, batt: -28 };
  if (thr < 0.3) {
    const k = (thr - 0.03) / 0.27;
    return { id: 'ev', name: 'Solo elettrico', ring, carrier: 0, eng: 0, mg2: 5 + 25 * k, mg1: 0, batt: 6 + 26 * k };
  }
  if (thr < 0.4) {
    const k = (thr - 0.3) / 0.1;
    return { id: 'start', name: 'Avvio motore', ring, carrier: 0.35 * k, eng: 10 * k, mg2: 30, mg1: -6 * (1 - k), batt: 36 - 10 * k };
  }
  if (thr < 0.78) {
    const k = (thr - 0.4) / 0.38;
    const eng = 20 + 70 * k;
    return { id: 'split', name: 'Divisione di potenza', ring, carrier: 0.35 + 0.25 * k, eng, mg2: 0.3 * eng, mg1: 0.32 * eng, batt: -4 };
  }
  const k = (thr - 0.78) / 0.22;
  return { id: 'boost', name: 'Spinta massima', ring, carrier: 0.6 + 0.15 * k, eng: 90 + 22 * k, mg2: 30 + 50 * k, mg1: 25, batt: 15 + 30 * k };
}

const fmt = (n) => (Math.round(n) < 0 ? '−' + Math.abs(Math.round(n)) : String(Math.round(n)));

// ---------- casting / fastener helpers ----------
const UP = new THREE.Vector3(0, 1, 0);
function rod(a, b, r, mat, seg = 10) {
  const va = new THREE.Vector3(...a);
  const vb = new THREE.Vector3(...b);
  const dir = vb.clone().sub(va);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, dir.length(), seg), mat);
  m.position.copy(va).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(UP, dir.normalize());
  return m;
}

function rrect(w, h, r, cu = 0, cv = 0, Cls = THREE.Shape) {
  const s = new Cls();
  const x0 = cu - w / 2;
  const y0 = cv - h / 2;
  s.moveTo(x0 + r, y0);
  s.lineTo(x0 + w - r, y0);
  s.quadraticCurveTo(x0 + w, y0, x0 + w, y0 + r);
  s.lineTo(x0 + w, y0 + h - r);
  s.quadraticCurveTo(x0 + w, y0 + h, x0 + w - r, y0 + h);
  s.lineTo(x0 + r, y0 + h);
  s.quadraticCurveTo(x0, y0 + h, x0, y0 + h - r);
  s.lineTo(x0, y0 + r);
  s.quadraticCurveTo(x0, y0, x0 + r, y0);
  return s;
}

// Extrude a shape in the XZ plane (u = x, v = -z) along +Y, from y0 to y0 + h.
function extrudeY(shape, h, y0 = 0, curveSegments = 8) {
  const g = new THREE.ExtrudeGeometry(shape, { depth: h, bevelEnabled: false, curveSegments });
  g.rotateX(-Math.PI / 2);
  g.translate(0, y0, 0);
  return g;
}

// Hollow rounded box with wall thickness t; top = false gives an open tray (oil pan).
function shellBox(w, h, d, t, r, mat, x = 0, y = 0, z = 0, top = true) {
  const g = new THREE.Group();
  const ring = rrect(w, d, r);
  ring.holes.push(rrect(w - 2 * t, d - 2 * t, Math.max(0.006, r - t), 0, 0, THREE.Path));
  g.add(new THREE.Mesh(extrudeY(ring, h - (top ? 2 : 1) * t, -h / 2 + t), mat));
  const plateG = extrudeY(rrect(w, d, r), t, 0);
  g.add(new THREE.Mesh(plateG, mat).translateY(-h / 2));
  if (top) g.add(new THREE.Mesh(plateG, mat).translateY(h / 2 - t));
  g.position.set(x, y, z);
  return g;
}

// One InstancedMesh of hex bolt heads; list of [x, y, z, axis].
function bolts(list, mat, r = 0.014, h = 0.014) {
  const mesh = new THREE.InstancedMesh(new THREE.CylinderGeometry(r, r, h, 6), mat, list.length);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const e = new THREE.Euler();
  const p = new THREE.Vector3();
  const s = new THREE.Vector3(1, 1, 1);
  list.forEach(([x, y, z, ax], i) => {
    e.set(ax === 'z' ? Math.PI / 2 : 0, 0, ax === 'x' ? Math.PI / 2 : 0);
    mesh.setMatrixAt(i, m.compose(p.set(x, y, z), q.setFromEuler(e), s));
  });
  return mesh;
}
const boltCircleX = (x, cy, r, n, a0 = 0) =>
  Array.from({ length: n }, (_, i) => [x, cy + Math.cos(a0 + (i * TAU) / n) * r, Math.sin(a0 + (i * TAU) / n) * r, 'x']);

// Officina fasteners: hex head with its shank below it (local -Y), so a screw backing out shows the shank.
const screwGeo = (len, r = 0.018, h = 0.018) =>
  mergeGeometries([new THREE.CylinderGeometry(r, r, h, 6), new THREE.CylinderGeometry(r * 0.6, r * 0.6, len, 8).translate(0, -h / 2 - len / 2, 0)]);
// One InstancedMesh of screws; list of [x, y, z, axis]: local +Y points out of the hole, along +axis ('x' | 'y').
function screws(list, mat, geo) {
  const mesh = new THREE.InstancedMesh(geo, mat, list.length);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const p = new THREE.Vector3();
  const s = new THREE.Vector3(1, 1, 1);
  list.forEach(([x, y, z, ax], i) => {
    q.setFromUnitVectors(UP, p.set(ax === 'x' ? 1 : 0, ax === 'x' ? 0 : 1, 0));
    mesh.setMatrixAt(i, m.compose(p.set(x, y, z), q, s));
  });
  return mesh;
}

// Generic label plate (canvas texture, no real brands).
function plate(lines, w, h, bg = '#e8c22a', fg = '#16181c') {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = Math.round((256 * h) / w);
  const g = c.getContext('2d');
  g.fillStyle = bg;
  g.fillRect(0, 0, c.width, c.height);
  g.strokeStyle = fg;
  g.lineWidth = 6;
  g.strokeRect(5, 5, c.width - 10, c.height - 10);
  g.fillStyle = fg;
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  lines.forEach((t, i) => {
    g.font = `bold ${i === 0 ? 30 : 22}px sans-serif`;
    g.fillText(t, c.width / 2, ((i + 0.5) * c.height) / lines.length);
  });
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.6 }));
}

// Smooth 0..1 valve lift inside the window [a, b] of the 720° cycle (u in 0..1).
function valveLift(u, a, b) {
  const x = (((u - a) % 1) + 1) % 1;
  return x < b - a ? Math.sin((Math.PI * x) / (b - a)) : 0;
}
// Cam lobe around X, nose at polar angle 0, half-width `half` (rad of cam).
function lobeGeo(base, bump, half, width) {
  const pts = [];
  for (let i = 0; i < 48; i++) {
    const a = (i / 48) * TAU - Math.PI;
    const k = Math.abs(a) < half ? Math.cos((a / half) * (Math.PI / 2)) ** 2 : 0;
    pts.push(sp(base + bump * k, a));
  }
  return extrudeX(polyShape(pts), width, 4);
}
// Intake closes late (Atkinson): window 0..0.36 of the cycle; exhaust 0.72..1.0.
const IN_WIN = [0.0, 0.36];
const EX_WIN = [0.72, 1.0];


export default {
  id: 'hybrid',
  name: 'Ibrido full hybrid a divisione di potenza (e-CVT)',
  short: 'Ibrido',
  family: 'terra',
  vehicle: 'Auto ibrida (full hybrid)',
  examples: 'Toyota Yaris/Corolla/C-HR Hybrid, Lexus, Ford Kuga FHEV',
  tagline: 'Un motore a benzina, due motori elettrici e un rotismo epicicloidale che li mette d\'accordo.',
  intro: {
    base:
      "L'ibrido «full» ha un motore a benzina e due macchine elettriche, collegati da un ingranaggio speciale chiamato <b>rotismo epicicloidale</b>, il «divisore di potenza». " +
      "In città può andare <b>solo in elettrico</b>; quando serve più forza si accende il motore a benzina, e una parte della sua energia passa dagli ingranaggi, un'altra viene trasformata in elettricità. " +
      "Non ci sono marce: il computer sceglie sempre i giri migliori. In frenata il motore elettrico <b>ricarica la piccola batteria</b>.",
    expert:
      'Sistema power-split (Toyota Hybrid System, e-CVT): motore a ciclo <b>Atkinson</b> collegato al portasatelliti, <b>MG1</b> al solare, la corona all\'uscita insieme a <b>MG2</b>. ' +
      'Il vincolo cinematico ω_s·Z_s + ω_r·Z_r = ω_c·(Z_s + Z_r) lascia un grado di libertà: variando la velocità di MG1 si sceglie il regime del motore indipendentemente dalla velocità dell\'auto (cambio continuo senza frizioni né marce). ' +
      'Il motore termico lavora vicino al suo punto di massimo rendimento (~40-41%); la potenza in eccesso o in difetto passa per il ramo elettrico MG1→MG2 o per la batteria (~1 kWh).',
  },
  specs: [
    { label: 'Motore termico', value: '2.0 benzina, ciclo Atkinson, ≈ 112 kW' },
    { label: 'Motori elettrici', value: 'MG1 generatore/avviatore, MG2 trazione ≈ 80 kW' },
    { label: 'Potenza di sistema', value: '≈ 145 kW (196 CV)' },
    { label: 'Trasmissione', value: 'e-CVT a rotismo epicicloidale (30/78 denti)' },
    { label: 'Batteria', value: 'NiMH o Li-ion ≈ 1 kWh, ~200 V (portati a ~600 V)' },
    { label: 'Rendimento max motore', value: '≈ 40-41 %' },
    { label: 'Consumo tipico', value: '≈ 4-5 l/100 km' },
    { label: 'Distribuzione', value: 'Catena, bialbero 16 valvole, fasatura variabile (VVT-iE / VVT-i)' },
  ],
  metrics: { efficiency: 40, power: 145, powerDensity: 0.7 },
  variants: {
    columns: ['Modello', 'Motore termico', 'Potenza di sistema', 'Sistema ibrido', 'Distribuzione'],
    rows: [
      ['Toyota Prius (2003-2009)', '1.5 1NZ-FXE, 4 cil.', '82 kW (110 CV)', 'THS II', 'Catena'],
      ['Toyota Prius (2016-2022)', '1.8 2ZR-FXE, 4 cil.', '90 kW (122 CV)', 'THS 4ª gen.', 'Catena'],
      ['Toyota Corolla 1.8 Hybrid (2019)', '1.8 2ZR-FXE, 4 cil.', '90 kW (122 CV)', 'THS 4ª gen.', 'Catena'],
      ['Toyota Yaris Hybrid (2020)', '1.5 M15A-FXE, 3 cil.', '85 kW (116 CV)', 'THS 4ª gen.', 'Catena'],
      ['Toyota Corolla 2.0 Hybrid (2019)', '2.0 M20A-FXS, 4 cil.', '135 kW (184 CV)', 'THS 4ª gen.', 'Catena'],
      ['Toyota Corolla 2.0 Hybrid (2023)', '2.0 M20A-FXS, 4 cil.', '144 kW (196 CV)', 'THS 5ª gen.', 'Catena'],
      ['Toyota RAV4 2.5 Hybrid (2019)', '2.5 A25A-FXS, 4 cil.', '160 kW (218 CV) 2WD', 'THS 4ª gen.', 'Catena'],
    ],
    note: 'Tutti a ciclo Atkinson con rotismo epicicloidale; le versioni AWD-i aggiungono un motore elettrico posteriore.',
  },
  thumb: { orbit: [38, 18], zoom: 1.2, view: 'section' },
  phases: [
    {
      id: 'ev',
      name: 'Solo elettrico',
      parts: ['batteria', 'caviHV', 'pcu', 'mg2', 'rotoreMg2', 'contralbero', 'ruota'],
      base:
        "Con poco acceleratore (fino a circa un terzo) il motore a benzina resta spento e fermo. La batteria manda corrente alla centralina di potenza, che fa girare MG2: è lui a spingere l'auto, in silenzio. " +
        'Guarda il rotismo: il portasatelliti (motore a benzina) è fermo, la corona gira con le ruote e il solare con MG1 gira all\'indietro «a vuoto».',
      expert:
        'Con ω_c = 0 il vincolo dà ω_s = −(Z_r/Z_s)·ω_r ≈ −2,6·ω_r: MG1 gira al contrario senza coppia. La marcia elettrica è limitata dalla piccola batteria (~1 kWh) e dalla potenza di MG2: ' +
        'pochi km a velocità urbana, oltre i ~50-60 km/h o con richiesta di coppia si avvia il termico. In città un full hybrid viaggia comunque in elettrico per il 40-60% del tempo.',
    },
    {
      id: 'avvio',
      name: 'Avvio motore termico',
      parts: ['mg1', 'rotoreMg1', 'solare', 'satelliti', 'portaSatelliti', 'alberoMotore', 'motoreTermico'],
      base:
        "Non c'è un motorino d'avviamento: è MG1 a far partire il motore. Spingendo sul solare, attraverso i satelliti fa girare il portasatelliti e quindi l'albero a gomiti, fino al regime giusto; " +
        "solo allora arriva la benzina e partono le scintille. Succede in una frazione di secondo e quasi non te ne accorgi.",
      expert:
        'MG1 fornisce coppia motrice sul solare: la reazione sulla corona è compensata da MG2, così l\'avvio non dà strappi alle ruote. Il motore viene portato a ~1000 rpm prima dell\'iniezione, ' +
        'poi MG1 passa a funzionare da generatore. Avviamenti e spegnimenti sono continui (anche centinaia per viaggio), per questo il termico ha catena di distribuzione e cuscinetti dimensionati allo scopo.',
    },
    {
      id: 'split',
      name: 'Divisione di potenza',
      parts: ['portaSatelliti', 'satelliti', 'solare', 'corona', 'mg1', 'mg2', 'pcu'],
      base:
        "A velocità di crociera il motore a benzina spinge il portasatelliti. I satelliti dividono la sua forza: una parte va direttamente alla corona e quindi alle ruote, un'altra fa girare il solare e MG1, che diventa una dinamo. " +
        "L'elettricità prodotta da MG1 va a MG2, che aiuta a spingere. Cambiando quanto «frena» MG1, il computer sceglie i giri del motore come un cambio continuo, senza marce.",
      expert:
        'Con S = 30 e R = 78 la coppia del motore si ripartisce in modo fisso: ~72% (R/(S+R)) alla corona, ~28% al solare. Il ramo elettrico MG1→inverter→MG2 trasporta tipicamente il 20-30% della potenza con rendimento ~80-85%; ' +
        'il resto viaggia per via meccanica (~97%). Il controllo tiene il termico lungo la linea di minimo consumo specifico (~210-220 g/kWh) a qualunque velocità del veicolo.',
    },
    {
      id: 'boost',
      name: 'Spinta massima (boost)',
      parts: ['motoreTermico', 'pistoni', 'batteria', 'pcu', 'mg2', 'corona', 'ruota'],
      base:
        'Acceleratore a fondo: il motore a benzina sale di giri e lavora a piena potenza, e in più la batteria manda tutta l\'energia che può a MG2. Le due spinte si sommano sulle ruote. ' +
        'Per questo un ibrido scatta meglio di quanto ci si aspetterebbe dal solo motore a benzina.',
      expert:
        'Potenza di sistema ≈ potenza del termico + potenza della batteria (non la somma delle potenze di targa: MG2 è alimentato anche da MG1). Esempio Toyota 2.0: 112 kW termico + ~35-40 kW di batteria ≈ 145 kW. ' +
        'Il convertitore boost porta la tensione da ~200 V a 600-650 V per ridurre le correnti nei motori. Al limite di giri di MG1 (~17.000 rpm) è vincolata la velocità massima.',
    },
    {
      id: 'regen',
      name: 'Frenata rigenerativa',
      parts: ['ruota', 'semiasse', 'differenziale', 'contralbero', 'rotoreMg2', 'mg2', 'pcu', 'batteria'],
      base:
        'Porta l\'acceleratore a zero: sono le ruote a far girare MG2, che diventa un generatore. Frena l\'auto e la corrente (in verde) torna indietro fino alla batteria. ' +
        'Il motore a benzina resta spento: invece di scaldare i freni, l\'energia del movimento viene messa da parte per la prossima ripartenza.',
      expert:
        'MG2 lavora in coppia negativa (fino a ~30-50 kW di recupero, limitati soprattutto dalla potenza di carica della batteria); la frenata è combinata con l\'impianto idraulico ("brake blending"). ' +
        'La batteria resta tra ~40% e ~80% di carica per durare decenni. Differenze: MHEV (12/48 V, aiuta solo il termico, niente marcia elettrica), HEV (questo), PHEV (batteria 10-25 kWh ricaricabile alla presa, 50-80 km elettrici).',
    },
  ],
  quiz: [
    {
      q: 'Nel rotismo epicicloidale di un ibrido Toyota, a cosa è collegato il motore a benzina?',
      options: ['Al solare', 'Al portasatelliti', 'Alla corona', 'Direttamente alla batteria'],
      correct: 1,
      explain:
        'Il termico spinge il portasatelliti; il solare è collegato a MG1 e la corona, insieme a MG2, va verso le ruote. Così la potenza si divide tra un ramo meccanico e uno elettrico.',
    },
    {
      q: 'Perché un full hybrid non ha marce da cambiare?',
      options: [
        'Perché ha solo la retromarcia',
        'Perché il motore a benzina gira sempre allo stesso regime',
        'Perché variando la velocità di MG1 il rotismo funziona come un cambio continuo (e-CVT)',
        'Perché le ruote sono collegate solo al motore elettrico',
      ],
      correct: 2,
      explain:
        'Il rotismo lega tre velocità; fissata quella delle ruote, la velocità di MG1 decide i giri del motore a benzina. Il risultato è un rapporto che varia con continuità, senza ingranaggi da innestare.',
    },
    {
      q: 'Che cosa fa il ciclo Atkinson nel motore a benzina di un ibrido?',
      options: [
        'Chiude tardi la valvola di aspirazione: l\'espansione è più lunga della compressione e il rendimento sale',
        'Aggiunge un turbocompressore per avere più potenza',
        'Fa funzionare il motore con il gasolio',
        'Spegne due cilindri in autostrada',
      ],
      correct: 0,
      explain:
        'Chiudendo in ritardo l\'aspirazione una parte della carica torna nel collettore: si comprime meno di quanto si espande. Si perde un po\' di potenza, che i motori elettrici compensano, ma il rendimento arriva al 40-41%.',
    },
    {
      q: 'Qual è la differenza principale tra un ibrido «full» (HEV) e un ibrido plug-in (PHEV)?',
      options: [
        'Il full hybrid non ha motore elettrico',
        'Il plug-in ha una batteria molto più grande che si ricarica alla presa e permette decine di km in elettrico',
        'Il plug-in non ha il motore a benzina',
        'Non c\'è differenza, è solo un nome commerciale',
      ],
      correct: 1,
      explain:
        'Il full hybrid ha una batteria di circa 1 kWh ricaricata solo da motore e frenate; il PHEV ne ha 10-25 kWh, si collega alla rete e fa 50-80 km in elettrico. Il mild hybrid (MHEV) a 48 V invece non viaggia mai in solo elettrico.',
    },
  ],
  // Officina (see CONTRACT.md): the strip-down is centred on the transaxle; petrol engine, battery, supports
  // and wheel stay as the base. Simplified and educational, not a workshop manual.
  disassembly: {
    intro: 'Smontiamo la trasmissione ibrida dall’esterno verso il cuore: cavi, centralina, scatola, ingranaggi e i due motori elettrici.',
    note: 'Sequenza semplificata a scopo didattico. Su un’auto vera l’impianto ad alta tensione va messo in sicurezza e toccato solo da tecnici abilitati, seguendo il manuale del costruttore.',
    parts: {
      vitiHV: { motion: 'unscrew', turns: 5, out: 0.05, dir: [0.3, 0.5, 1], dist: 0.3 },
      caviHV: { after: ['vitiHV'], dir: [0.2, 1, 0.5], dist: 0.8 },
      vitiPcu: { motion: 'unscrew', turns: 6, out: 0.08, dir: [0.5, 0, 1], dist: 0.3 },
      pcu: { after: ['vitiPcu', 'caviHV'], dir: [0, 1, 0], dist: 0.9 },
      mg2: { after: ['caviHV'], dir: [-1, 0, 0], dist: 0.9 },
      semiasse: { dir: [0, -0.2, 1], dist: 0.9 },
      vitiCassa: { motion: 'unscrew', turns: 6, out: 0.07, dir: [1, 0, 0.4], dist: 0.3, duration: 2 },
      cassa: { after: ['vitiCassa', 'pcu', 'mg2', 'semiasse'], dir: [0.3, 0, 1], dist: 1.3, duration: 1.2 },
      differenziale: { after: ['cassa', 'semiasse'], dir: [0, 0, 1], dist: 1.0 },
      contralbero: { after: ['cassa'], dir: [0, 0, 1], dist: 1.0 },
      rotoreMg2: { after: ['cassa', 'mg2'], dir: [-1, 0, 0], dist: 0.9 },
      portaSatelliti: { after: ['cassa'], dir: [1, 0, 0.5], dist: 0.7 },
      satelliti: { after: ['portaSatelliti'], dir: [1, 0, 0.5], dist: 0.7 },
      solare: { after: ['satelliti'], dir: [1, 0, 0.5], dist: 0.7 },
      corona: { after: ['satelliti'], dir: [0.5, 0, 1], dist: 0.9 },
      mg1: { after: ['solare', 'corona'], dir: [1, 0, 0], dist: 0.9 },
      rotoreMg1: { after: ['mg1'], dir: [1, 0, 0], dist: 0.7 },
      smorzatore: { after: ['rotoreMg1'], dir: [1, 0, 0], dist: 0.55 },
    },
    steps: [
      {
        id: 'cavi',
        name: 'Stacca i cavi arancioni',
        base: 'Prima di tutto si mette in sicurezza l’alta tensione: si toglie il sezionatore arancione sulla batteria e si aspetta che la centralina si scarichi. Poi si svitano i connettori e si tolgono i cavi arancioni.',
        expert:
          'Tolto il sezionatore di servizio il circuito della batteria è aperto, ma i condensatori dell’inverter restano carichi per qualche minuto: si attende e si verifica con uno strumento l’assenza di tensione. ' +
          'Solo allora si svitano i connettori: i cavi in continua verso la batteria e quelli trifase verso MG1 e MG2.',
        parts: ['vitiHV', 'caviHV'],
      },
      {
        id: 'pcu',
        name: 'Togli la centralina di potenza',
        base: 'Si svitano le viti dei piedini e si solleva la centralina: sotto restano la scatola del cambio e MG1.',
        expert: 'Prima di sollevarla si staccano anche i tubi del liquido di raffreddamento della piastra fredda (qui non disegnati). Dentro ci sono i due inverter e il convertitore boost, con i condensatori che abbiamo lasciato scaricare.',
        parts: ['vitiPcu', 'pcu'],
      },
      {
        id: 'mg2',
        name: 'Sfila lo statore di MG2',
        base: 'Il guscio di MG2, con gli avvolgimenti di rame, si sfila di lato: il rotore con le calamite resta sul suo albero.',
        expert: 'Lo statore è la parte fissa: lamierini e avvolgimenti trifase. I magneti permanenti del rotore lo attirano con forza anche a motore spento, quindi va sfilato ben dritto. Nei transaxle reali i due motori elettrici sono alloggiati dentro la scatola stessa.',
        parts: ['mg2'],
      },
      {
        id: 'semiasse',
        name: 'Sfila il semiasse',
        base: 'L’albero che porta il moto alla ruota è infilato nel differenziale: va sfilato prima di aprire la scatola.',
        expert: 'Il giunto interno (tripode) è innestato con un profilo scanalato nel differenziale; sfilandolo può uscire dell’olio, per questo di solito il cambio si svuota prima.',
        parts: ['semiasse'],
      },
      {
        id: 'cassa',
        name: 'Apri la scatola del cambio',
        base: 'Si svitano le viti lungo la flangia e si toglie la scatola di alluminio: finalmente si vedono il rotismo e gli ingranaggi.',
        expert: 'Le due metà sono unite da una corona di viti e sigillate da una guarnizione. Qui la scatola esce in un pezzo solo per semplicità; nella realtà si separano i due semigusci e gli alberi restano appoggiati ai cuscinetti di una metà.',
        parts: ['vitiCassa', 'cassa'],
      },
      {
        id: 'uscita',
        name: 'Togli gli ingranaggi di uscita',
        base: 'Si tolgono il differenziale, il contralbero con le sue tre ruote dentate e il rotore di MG2 con il suo pignone.',
        expert: 'È la catena di riduzione verso le ruote: corona→contralbero 52/40, pignone di MG2 21/58, coppia finale 18/58. Sono ingranaggi sempre in presa, senza sincronizzatori né frizioni: non c’è nulla da innestare.',
        parts: ['differenziale', 'contralbero', 'rotoreMg2'],
      },
      {
        id: 'rotismo',
        name: 'Smonta il rotismo epicicloidale',
        base: 'Il cuore dell’ibrido: prima il portasatelliti, poi i tre satelliti, il solare e infine la corona.',
        expert: 'Portasatelliti (motore termico), solare (MG1) e corona (uscita) sono coassiali: 30 e 78 denti, satelliti da 24. È tutto qui il «cambio» e-CVT: un solo rotismo, nessun rapporto da innestare.',
        parts: ['portaSatelliti', 'satelliti', 'solare', 'corona'],
      },
      {
        id: 'mg1',
        name: 'Sfila MG1',
        base: 'Ora che il rotismo non c’è più, lo statore di MG1 si sfila lungo l’albero; poi tocca al rotore, montato su un albero cavo.',
        expert: 'Il rotore di MG1 è calettato sull’albero cavo del solare; al suo interno passa, libero di girare, l’albero d’ingresso che viene dal motore termico.',
        parts: ['mg1', 'rotoreMg1'],
      },
      {
        id: 'smorzatore',
        name: 'Togli lo smorzatore',
        base: 'Resta il disco con le molle, fissato al volano del motore a benzina, con il lungo albero d’ingresso.',
        expert: 'Non c’è frizione: lo smorzatore torsionale collega in modo permanente albero motore e albero d’ingresso, filtrando le pulsazioni di coppia. Tolto questo, del gruppo ibrido restano il motore termico e la batteria.',
        parts: ['smorzatore'],
      },
    ],
  },
  camera: { position: [4.54, 2.52, 6.16], target: [0.12, -0.08, -0.2] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.copy(OFFSET);
    root.add(model);
    const add = (obj, opts) => model.add(ctx.part(obj, opts));

    // ================= Engine block =================
    const block = new THREE.Group();
    const alu = cut(MAT.aluminum());
    alu.roughness = 0.62; // cast finish
    const headMat = cut(MAT.aluminum());
    headMat.color.set(0x8e98a4);
    headMat.roughness = 0.6;
    const BX = (CYL_X[0] + CYL_X[3]) / 2;
    // Cylinder block: bores + water-jacket slots cut through (section shows webs and jacket).
    const blk = rrect(1.5, 0.44, 0.04, BX, 0);
    for (const x of CYL_X) {
      blk.holes.push(circlePath(x, 0, 0.15));
      for (const v of [-0.186, 0.186]) blk.holes.push(rrect(0.22, 0.026, 0.01, x, v, THREE.Path));
    }
    for (const x of [CYL_X[0] - 0.19, CYL_X[3] + 0.19]) blk.holes.push(rrect(0.026, 0.3, 0.01, x, 0, THREE.Path));
    block.add(new THREE.Mesh(extrudeY(blk, 0.42, YA + 0.18, 24), alu));
    const linerGeo = pipe(0.15, 0.136, 0.42, 40);
    const linerMat = cut(MAT.castIron());
    for (const x of CYL_X) block.add(new THREE.Mesh(linerGeo, linerMat).translateX(x).translateY(YA + 0.39));
    // Crankcase skirt + main-bearing bulkheads with bronze shells
    const skirt = rrect(1.5, 0.46, 0.04, BX, 0);
    skirt.holes.push(rrect(1.42, 0.38, 0.03, BX, 0, THREE.Path));
    block.add(new THREE.Mesh(extrudeY(skirt, 0.38, YA - 0.2), alu));
    const bulk = rrect(0.38, 0.38, 0.02, 0, YA - 0.01);
    bulk.holes.push(circlePath(0, YA, 0.052));
    const bulkGeo = extrudeX(bulk, 0.04, 16);
    const shellGeo = alongX(pipe(0.052, 0.045, 0.04, 24));
    const bronze = cut(MAT.brass());
    for (const x of [CYL_X[0] - 0.175, ...CYL_X.map((c) => c + 0.175)]) {
      block.add(new THREE.Mesh(bulkGeo, alu).translateX(x));
      block.add(new THREE.Mesh(shellGeo, bronze).translateX(x).translateY(YA));
    }
    // Oil pan (open tray) with flange bolts and drain plug
    const panMat = cut(MAT.darkSteel());
    block.add(shellBox(1.4, 0.18, 0.38, 0.015, 0.04, panMat, BX, YA - 0.29, 0, false));
    const pf = rrect(1.46, 0.44, 0.05, BX, 0);
    pf.holes.push(rrect(1.38, 0.36, 0.03, BX, 0, THREE.Path));
    block.add(new THREE.Mesh(extrudeY(pf, 0.012, YA - 0.212), panMat));
    const bMat = cut(MAT.chrome(), { cap: false });
    const pb = [];
    for (let i = 0; i < 8; i++) pb.push([BX - 0.66 + i * 0.188, YA - 0.2, 0.205, 'y'], [BX - 0.66 + i * 0.188, YA - 0.2, -0.205, 'y']);
    block.add(bolts(pb, bMat, 0.011, 0.012));
    block.add(new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.02, 6), MAT.darkSteel()).translateX(BX + 0.4).translateY(YA - 0.39));
    // Head: plug bores, valve/tappet bores, coolant passages; head gasket line
    const hd = rrect(1.5, 0.46, 0.03, BX, 0);
    CYL_X.forEach((x, k) => {
      hd.holes.push(circlePath(x, 0, 0.024));
      for (const dx of [-0.055, 0.055]) for (const v of [-0.07, 0.07]) hd.holes.push(circlePath(x + dx, v, 0.028));
      if (k < 3) hd.holes.push(rrect(0.06, 0.05, 0.01, x + 0.175, 0, THREE.Path));
    });
    block.add(new THREE.Mesh(extrudeY(hd, 0.2, YA + 0.6, 16), headMat));
    const hg = rrect(1.506, 0.466, 0.03, BX, 0);
    hg.holes.push(rrect(1.48, 0.44, 0.03, BX, 0, THREE.Path));
    block.add(new THREE.Mesh(extrudeY(hg, 0.008, YA + 0.597), cut(MAT.carbon())));
    // Cam cover (hollow, camshafts inside), oil filler cap, head bolts
    const coverMat = cut(MAT.paint(0x1f2733, 0.5));
    block.add(shellBox(1.46, 0.13, 0.42, 0.015, 0.05, coverMat, BX, YA + 0.865, 0));
    const cb = [];
    for (let i = 0; i < 7; i++) cb.push([BX - 0.66 + i * 0.22, YA + 0.94, 0.19, 'y'], [BX - 0.66 + i * 0.22, YA + 0.94, -0.19, 'y']);
    block.add(bolts(cb, bMat, 0.01, 0.012));
    block.add(new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.03, 20), MAT.paint(0xd9a520, 0.5)).translateX(-2.38).translateY(YA + 0.945).translateZ(-0.12));
    // Timing-chain cover (front), dipstick, oil filter (front, cut away in section)
    const tcHull = [];
    for (const [cz, cy, r] of [[0, YA, 0.075], [0.07, YA + 0.835, 0.1], [-0.07, YA + 0.835, 0.1]]) {
      for (let i = 0; i < 24; i++) tcHull.push([-cz + Math.cos((i / 24) * TAU) * r, cy + Math.sin((i / 24) * TAU) * r]);
    }
    const tcOut = convexHull(tcHull);
    const tcRing = polyShape(tcOut);
    const tcIn = convexHull(tcHull.map(([u, v]) => [u * 0.86, YA + (v - YA) * 0.97]));
    tcRing.holes.push(polyShape(tcIn.slice().reverse(), THREE.Path));
    const tcMat = cut(MAT.aluminum());
    tcMat.color.set(0x9aa2ab);
    block.add(new THREE.Mesh(extrudeX(tcRing, 0.13, 6), tcMat).translateX(-2.79));
    block.add(new THREE.Mesh(extrudeX(polyShape(tcOut), 0.014, 6), tcMat).translateX(-2.862));
    block.add(rod([-1.32, YA - 0.1, -0.24], [-1.33, YA + 0.86, -0.3], 0.008, MAT.steel()));
    block.add(new THREE.Mesh(new THREE.TorusGeometry(0.025, 0.007, 6, 16), MAT.paint(0xe8b416, 0.5)).translateX(-1.33).translateY(YA + 0.89).translateZ(-0.3));
    const filt = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.13, 20), cut(MAT.paint(0x2c4f8f, 0.45)));
    filt.position.set(-1.45, YA - 0.02, 0.29);
    filt.rotation.x = Math.PI / 2;
    block.add(filt);
    // exhaust manifold on the front (cut away with the casing) + close-coupled catalyst
    const exMat = cut(MAT.paint(0x6b4a3a, 0.55));
    exMat.metalness = 0.8;
    for (const x of CYL_X) {
      const c = curve([[x, YA + 0.68, 0.2], [x, YA + 0.68, 0.34], [x * 0.6 + BX * 0.4, YA + 0.45, 0.42], [BX, YA + 0.2, 0.44]]);
      block.add(new THREE.Mesh(new THREE.TubeGeometry(c, 24, 0.045, 10), exMat));
    }
    block.add(new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.34, 24), exMat).translateX(BX).translateY(YA + 0.0).translateZ(0.44));
    // spark plugs + coil-on-plug units with harness
    const plugMat = MAT.ceramic();
    const coilMat = MAT.paint(0x15181c, 0.5);
    for (const x of CYL_X) {
      block.add(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.3, 10), plugMat).translateX(x).translateY(YA + 0.78));
      block.add(new THREE.Mesh(new RoundedBoxGeometry(0.07, 0.08, 0.11, 2, 0.015), coilMat).translateX(x).translateY(YA + 0.97));
      block.add(new THREE.Mesh(new RoundedBoxGeometry(0.04, 0.03, 0.04, 2, 0.008), MAT.paint(0x3a3f46, 0.6)).translateX(x).translateY(YA + 0.99).translateZ(-0.07));
    }
    block.add(new THREE.Mesh(new THREE.TubeGeometry(curve([[-2.6, YA + 0.99, -0.1], [-1.9, YA + 0.995, -0.1], [-1.3, YA + 0.99, -0.1], [-1.22, YA + 0.85, -0.18], [-1.24, YA + 0.5, -0.24]]), 40, 0.012, 6), MAT.rubber()));
    const np = plate(['MOTORILAB', 'Ciclo Atkinson 2.0', '14:1'], 0.24, 0.12, '#c9cdd2', '#1c1f24');
    np.rotation.x = -Math.PI / 2;
    np.position.set(-2.0, YA + 0.931, -0.06);
    block.add(np);
    add(block, {
      id: 'motoreTermico',
      name: 'Motore a benzina (ciclo Atkinson)',
      base:
        "Un normale 4 cilindri a benzina, ma «ottimizzato per risparmiare»: lavora quasi sempre nella zona in cui consuma meno, e quando non serve si spegne del tutto. " +
        'Non ha bisogno di essere potentissimo, perché i motori elettrici lo aiutano nelle accelerazioni.',
      expert:
        'Toyota M20A-FXS 2.0 "Dynamic Force": ciclo Atkinson ottenuto con chiusura ritardata della valvola di aspirazione (VVT-iE), rapporto di compressione geometrico 14:1, ' +
        'corsa lunga, iniezione D-4S (diretta + indiretta), EGR raffreddato. Rendimento massimo ~40-41% (contro ~35-37% di un benzina tradizionale), 112 kW a 6.000 rpm.',
      explode: [-0.6, 0.25, -0.85],
    });

    // ================= Crankshaft =================
    const crank = new THREE.Group();
    crank.position.set(0, YA, 0);
    const crankSpin = new THREE.Group();
    const cMat = MAT.steel();
    crankSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.045, 0.045, 1.6, 20)), cMat).translateY(0).translateX(BX + 0.05));
    const semi = [];
    for (let i = 0; i <= 16; i++) semi.push(sp(0.13, Math.PI / 2 + (i / 16) * Math.PI));
    semi.push([0, 0]);
    const cwGeo = extrudeX(polyShape(semi), 0.03);
    const pinGeo = alongX(new THREE.CylinderGeometry(0.04, 0.04, 0.12, 16));
    CYL_X.forEach((x, k) => {
      const s = FIRE[k] === 0 || FIRE[k] === 0.5 ? 1 : -1; // pin on top (1) or bottom (-1)
      crankSpin.add(new THREE.Mesh(pinGeo, cMat).translateX(x).translateY(s * CR));
      for (const dx of [-0.075, 0.075]) {
        crankSpin.add(box(0.03, CR + 0.08, 0.1, cMat, x + dx, (s * CR) / 2, 0));
        const cw = new THREE.Mesh(cwGeo, cMat);
        cw.position.x = x + dx;
        cw.rotation.x = s > 0 ? 0 : Math.PI;
        crankSpin.add(cw);
      }
    });
    // nose: chain sprocket inside the timing cover, torsional damper hub outside
    crankSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.024, 0.024, 0.21, 16)), cMat).translateX(-2.83));
    crankSpin.add(new THREE.Mesh(gearGeo(11, 0.0057, 0.025, 0), MAT.darkSteel()).translateX(-2.79));
    const pulley = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.065, 0.065, 0.05, 32)), MAT.darkSteel());
    pulley.position.x = -2.91;
    crankSpin.add(pulley);
    crank.add(crankSpin);
    add(crank, {
      id: 'alberoMotore',
      name: 'Albero a gomiti',
      base: "L'albero «a zig-zag» che trasforma il su e giù dei pistoni in rotazione. Nell'ibrido è collegato direttamente al portasatelliti del rotismo, senza frizione.",
      expert:
        'Albero in acciaio forgiato, manovelle a 180° (1-3-4-2). Non c\'è frizione né convertitore di coppia: tra albero e portasatelliti c\'è solo uno smorzatore torsionale. Quando il motore è spento, l\'albero resta fermo anche con l\'auto in movimento.',
      explode: [-0.6, -0.55, 0.5],
    });

    // ================= Valvetrain: camshafts, valves, timing chain =================
    const vt = new THREE.Group();
    const CAM_Y = YA + 0.835;
    const lobe = lobeGeo(0.026, 0.022, Math.PI / 3, 0.035);
    const camMat = MAT.steel();
    const cams = [];
    for (const [z, win] of [[-0.07, IN_WIN], [0.07, EX_WIN]]) {
      const g = new THREE.Group();
      g.position.set(0, CAM_Y, z);
      g.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.017, 0.017, 1.6, 12)), camMat).translateX(BX - 0.05));
      const peak = (win[0] + win[1]) / 2;
      CYL_X.forEach((x, k) => {
        for (const dx of [-0.055, 0.055]) {
          const l = new THREE.Mesh(lobe, camMat);
          l.position.x = x + dx;
          l.rotation.x = Math.PI - TAU * (peak + FIRE[k]); // nose down at peak lift
          g.add(l);
        }
      });
      g.add(new THREE.Mesh(gearGeo(22, 0.0057, 0.025, 0.017), MAT.darkSteel()).translateX(-2.79));
      vt.add(g);
      cams.push(g);
    }
    const stemGeo = new THREE.CylinderGeometry(0.009, 0.009, 0.2, 8).translate(0, 0.1, 0);
    const discGeo = lathe([[0, 0], [0.04, 0], [0.04, 0.007], [0.012, 0.03], [0, 0.03]], 20);
    const bucketGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.028, 16).translate(0, 0.193, 0);
    const valves = [];
    CYL_X.forEach((x, k) => {
      for (const [z, win, mat] of [[-0.07, IN_WIN, MAT.steel()], [0.07, EX_WIN, MAT.titanium()]]) {
        for (const dx of [-0.055, 0.055]) {
          const v = new THREE.Group();
          v.add(new THREE.Mesh(stemGeo, mat), new THREE.Mesh(discGeo, mat), new THREE.Mesh(bucketGeo, MAT.chrome()));
          v.position.set(x + dx, YA + 0.6, z);
          vt.add(v);
          valves.push({ g: v, k, win });
        }
      }
    });
    // chain: links on the hull of the three sprockets, moved with the crank
    const chainPts = [];
    for (const [u, v, r] of [[0, YA, 0.037], [0.07, CAM_Y, 0.069], [-0.07, CAM_Y, 0.069]]) {
      for (let i = 0; i < 48; i++) chainPts.push([u + Math.cos((i / 48) * TAU) * r, v + Math.sin((i / 48) * TAU) * r]);
    }
    const loop = convexHull(chainPts).map(([u, v]) => new THREE.Vector2(v, -u)); // (y, z)
    const segL = [0];
    for (let i = 1; i <= loop.length; i++) segL.push(segL[i - 1] + loop[i % loop.length].distanceTo(loop[i - 1]));
    const LOOP = segL[segL.length - 1];
    const N_LINKS = Math.floor(LOOP / 0.019);
    const links = new THREE.InstancedMesh(new THREE.BoxGeometry(0.03, 0.008, 0.016), MAT.darkSteel(), N_LINKS);
    vt.add(links);
    const lm = new THREE.Matrix4();
    const lq = new THREE.Quaternion();
    const le = new THREE.Euler();
    const lp = new THREE.Vector3();
    const lsc = new THREE.Vector3(1, 1, 1);
    const placeLinks = (offset) => {
      let seg = 1;
      for (let i = 0; i < N_LINKS; i++) {
        const d = (((i * LOOP) / N_LINKS + offset) % LOOP + LOOP) % LOOP;
        seg = 1;
        while (seg < segL.length - 1 && segL[seg] < d) seg++;
        const a = loop[seg - 1];
        const b = loop[seg % loop.length];
        const f = (d - segL[seg - 1]) / (segL[seg] - segL[seg - 1] || 1);
        lp.set(-2.79, a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f);
        le.set(Math.atan2(-(b.x - a.x), b.y - a.y), 0, 0);
        links.setMatrixAt(i, lm.compose(lp, lq.setFromEuler(le), lsc));
      }
      links.instanceMatrix.needsUpdate = true;
    };
    placeLinks(0);
    add(vt, {
      id: 'distribuzione',
      name: 'Distribuzione a catena e valvole',
      base:
        'Una catena collega l’albero a gomiti ai due alberi a camme, che girano a metà velocità e aprono le valvole al momento giusto. ' +
        'Guarda le valvole di aspirazione (dietro): restano aperte più a lungo del solito, anche quando il pistone ha già cominciato a risalire. È il trucco del ciclo Atkinson.',
      expert:
        'Bialbero in testa a 16 valvole con catena silenziosa, punterie a bicchiere, fasatura variabile su entrambe le camme (VVT-iE elettrica in aspirazione sui Dynamic Force). ' +
        'La chiusura dell’aspirazione ritardata di ~60-80° dopo il PMI riporta parte della carica nel collettore: compressione effettiva ridotta (~10-11:1) con rapporto geometrico 13-14:1, quindi espansione più lunga della compressione.',
      explode: [-0.6, 1.0, 0.1],
    });

    // ================= Pistons + conrods =================
    const pistG = new THREE.Group();
    const pistonGeo = lathe([[0, 0.09], [0.128, 0.09], [0.13, 0.08], [0.13, -0.07], [0.12, -0.07], [0.12, 0.06], [0, 0.06]], 32);
    const ringGeo = new THREE.TorusGeometry(0.13, 0.005, 5, 32).rotateX(Math.PI / 2);
    const pMat = MAT.aluminum();
    const rMat = MAT.darkSteel();
    const rodMat = MAT.steel();
    const beamGeo = new THREE.BoxGeometry(0.03, CL - 0.08, 0.045).translate(0, CL / 2, 0);
    const bigEnd = new THREE.TorusGeometry(0.045, 0.014, 8, 20).rotateY(Math.PI / 2);
    const smallEnd = new THREE.TorusGeometry(0.028, 0.01, 8, 16).rotateY(Math.PI / 2).translate(0, CL, 0);
    const pistons = [];
    const rods = [];
    for (const x of CYL_X) {
      const p = new THREE.Group();
      p.add(new THREE.Mesh(pistonGeo, pMat));
      for (const y of [0.075, 0.05]) p.add(new THREE.Mesh(ringGeo, rMat).translateY(y));
      p.position.x = x;
      pistG.add(p);
      pistons.push(p);
      const r = new THREE.Group();
      r.add(new THREE.Mesh(beamGeo, rodMat), new THREE.Mesh(bigEnd, rodMat), new THREE.Mesh(smallEnd, rodMat));
      r.position.x = x;
      pistG.add(r);
      rods.push(r);
    }
    add(pistG, {
      id: 'pistoni',
      name: 'Pistoni e bielle',
      base: 'I pistoni ricevono la spinta della combustione e, tramite le bielle, fanno girare l\'albero a gomiti. Quando l\'auto va in elettrico restano immobili.',
      expert:
        'Corsa lunga (alesaggio 80,5 × corsa 97,6 mm sul 2.0 M20A) e pistoni a basso attrito: nel ciclo Atkinson la corsa di espansione utile è maggiore di quella di compressione, perché parte della carica viene respinta nel collettore prima che la valvola chiuda.',
      explode: [-0.6, 0.65, 0.6],
    });

    // Combustion glow (overlay, not a part)
    const flameMat = MAT.gas(FLAME, 0.7, true);
    const flames = CYL_X.map((x) => {
      const m = new THREE.Mesh(new THREE.CylinderGeometry(0.125, 0.125, 1, 24), flameMat);
      m.position.x = x;
      m.renderOrder = 2;
      model.add(m);
      return m;
    });

    // ================= Intake + fuel injection =================
    const intake = new THREE.Group();
    const inMat = MAT.paint(0x2a3442, 0.5);
    intake.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.09, 0.09, 1.25, 24)), inMat).translateX(-2.05).translateY(YA + 0.5).translateZ(-0.55));
    intake.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.075, 0.075, 0.16, 24)), MAT.aluminum()).translateX(-2.75).translateY(YA + 0.5).translateZ(-0.55));
    for (const x of CYL_X) {
      const c = curve([[x, YA + 0.5, -0.55], [x, YA + 0.72, -0.5], [x, YA + 0.76, -0.32], [x, YA + 0.68, -0.22]]);
      intake.add(new THREE.Mesh(new THREE.TubeGeometry(c, 20, 0.05, 10), inMat));
    }
    const fuelMat = MAT.brass();
    intake.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.02, 0.02, 1.25, 10)), MAT.chrome()).translateX(-2.0).translateY(YA + 0.62).translateZ(-0.3));
    for (const x of CYL_X) {
      const inj = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.014, 0.12, 10), MAT.paint(0x222831));
      inj.position.set(x, YA + 0.6, -0.25);
      inj.rotation.x = -0.6;
      intake.add(inj);
    }
    const fuelLine = curve([[-2.95, 0.25, -0.35], [-2.95, YA + 0.4, -0.33], [-2.8, YA + 0.62, -0.31], [-2.62, YA + 0.62, -0.3]]);
    intake.add(new THREE.Mesh(new THREE.TubeGeometry(fuelLine, 24, 0.014, 8), fuelMat));
    add(intake, {
      id: 'iniezione',
      name: 'Aspirazione e iniezione',
      base: 'Il collettore porta l\'aria ai cilindri e gli iniettori spruzzano la benzina. Quando il motore è spento non arriva neanche una goccia di carburante.',
      expert:
        'Doppia iniezione D-4S: diretta ad alta pressione (fino a ~350 bar) per raffreddare la carica e permettere la compressione alta, indiretta ai carichi bassi per una miscela più omogenea. ' +
        'La farfalla resta molto aperta: nel ciclo Atkinson il carico si regola anche con la fasatura, riducendo le perdite di pompaggio.',
      explode: [-0.6, 0.6, -1.25],
    });

    // ================= Torsional damper + input shaft =================
    const damper = new THREE.Group();
    damper.position.set(0, YA, 0);
    const damperSpin = new THREE.Group();
    damperSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.3, 0.3, 0.05, 48)), MAT.darkSteel()).translateX(-1.19));
    damperSpin.add(new THREE.Mesh(alongX(pipe(0.2, 0.08, 0.05, 32)), MAT.steel()).translateX(-1.135));
    const springGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.11, 10);
    for (let i = 0; i < 6; i++) {
      const a = (i / 6) * TAU;
      const sm = new THREE.Mesh(springGeo, MAT.chrome());
      sm.position.set(-1.135, Math.cos(a) * 0.15, Math.sin(a) * 0.15);
      sm.rotation.x = a + Math.PI / 2;
      damperSpin.add(sm);
    }
    damperSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.034, 0.034, 1.32, 16)), MAT.steel()).translateX(-0.5));
    damper.add(damperSpin);
    add(damper, {
      id: 'smorzatore',
      name: 'Smorzatore e albero d\'ingresso',
      base: 'Un disco con molle che assorbe le pulsazioni del motore a benzina, e un albero sottile che passa dentro MG1 e porta la rotazione al portasatelliti.',
      expert:
        'Smorzatore torsionale a molle elicoidali con limitatore di coppia: filtra le pulsazioni di coppia del 4 cilindri prima del rotismo. L\'albero d\'ingresso è coassiale all\'albero cavo del solare/MG1.',
      explode: [-0.45, 0, 0.35],
    });

    // ================= MG1 (generator / starter) =================
    const housingMat = cut(MAT.paint(0x56606b, 0.5));
    const mg1 = buildMG(0.42, housingMat);
    mg1.stator.position.set(X_MG1, YA, 0);
    add(mg1.stator, {
      id: 'mg1',
      name: 'MG1 (generatore e avviatore)',
      base: 'Il motore-generatore più piccolo. Di solito fa da dinamo, mosso dal motore a benzina attraverso il solare; all\'avvio fa da motorino di avviamento.',
      expert:
        'Macchina sincrona a magneti permanenti (~30-40 kW nelle versioni 1.8-2.0) collegata al solare. Controllandone coppia e velocità si regola il regime del termico: è il "variatore" dell\'e-CVT. Gira fino a ~17.000 rpm.',
      explode: [-0.25, 0.3, -0.95],
    });
    const mg1Rot = new THREE.Group();
    mg1Rot.position.set(0, YA, 0);
    const mg1Spin = new THREE.Group();
    mg1.rotor.position.x = X_MG1;
    mg1Spin.add(mg1.rotor);
    mg1Spin.add(new THREE.Mesh(alongX(pipe(0.06, 0.04, 0.96, 24)), MAT.steel()).translateX(-0.51));
    mg1Rot.add(mg1Spin);
    add(mg1Rot, {
      id: 'rotoreMg1',
      name: 'Rotore di MG1 e albero del solare',
      base: 'La parte che gira di MG1, con le sue calamite (rosse e blu). È montata su un albero cavo collegato al solare.',
      expert:
        'Rotore a magneti interni su albero cavo: dentro passa l\'albero d\'ingresso del motore termico. Velocità ω_s = [(S+R)·ω_c − R·ω_r]/S: positiva in divisione di potenza, negativa in marcia elettrica.',
      explode: [-0.25, 0, 0.5],
    });

    // ================= Planetary gear set =================
    const sun = new THREE.Group();
    sun.position.set(0, YA, 0);
    const sunMat = MAT.brass(); // distinct tones keep sun / planets / ring readable
    sunMat.metalness = 0.6;
    sunMat.roughness = 0.35;
    const sunMesh = new THREE.Mesh(gearGeo(S, MP, 0.11, 0.045), sunMat);
    sun.add(sunMesh);
    add(sun, {
      id: 'solare',
      name: 'Solare (collegato a MG1)',
      base: 'La ruota dentata al centro del rotismo, come il sole al centro dei pianeti. È collegata a MG1.',
      expert: 'Solare da 30 denti su albero cavo. Riceve circa il 28% della coppia del termico (S/(S+R)); la sua velocità decide il "rapporto" dell\'e-CVT.',
      explode: [0.15, 0, 0.2],
    });

    const planets = new THREE.Group();
    planets.position.set(0, YA, 0);
    const planetGeo = gearGeo(P, MP, 0.11, 0.03, 3, 0.018, 0.07);
    const planetMeshes = [0, 1, 2].map(() => {
      const m = new THREE.Mesh(planetGeo, MAT.aluminum());
      planets.add(m);
      return m;
    });
    add(planets, {
      id: 'satelliti',
      name: 'Satelliti (pignoni planetari)',
      base: 'Tre ingranaggi che ingranano sia con il solare sia con la corona. Girano su sé stessi e, insieme, girano intorno al solare come pianeti in orbita.',
      expert:
        'Tre satelliti da 24 denti ((R−S)/2), montati su perni a rullini del portasatelliti. Montaggio possibile perché (S+R)/3 = 36 è intero. Trasmettono le forze tra solare, corona e portasatelliti senza fissare un rapporto unico.',
      explode: [0.35, 0, 0.25],
    });

    const carrier = new THREE.Group();
    carrier.position.set(0, YA, 0);
    const carrierSpin = new THREE.Group();
    const cmat = MAT.darkSteel();
    for (let i = 0; i < 3; i++) {
      const a = (i / 3) * TAU;
      const arm = box(0.022, R_C + 0.02, 0.07, cmat, 0.078, (Math.cos(a) * (R_C + 0.02)) / 2, (Math.sin(a) * (R_C + 0.02)) / 2);
      arm.rotation.x = a;
      carrierSpin.add(arm);
      const pin = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.026, 0.026, 0.17, 12)), MAT.chrome());
      pin.position.set(0.01, Math.cos(a) * R_C, Math.sin(a) * R_C);
      carrierSpin.add(pin);
    }
    carrierSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.075, 0.075, 0.06, 24)), cmat).translateX(0.1));
    carrier.add(carrierSpin);
    add(carrier, {
      id: 'portaSatelliti',
      name: 'Portasatelliti (collegato al motore)',
      base: 'Il «braccio» a tre punte che regge i satelliti. È collegato all\'albero del motore a benzina: quando il motore è spento, sta fermo.',
      expert:
        'Il portasatelliti riceve la coppia del termico e la ripartisce: T_corona = T_c·R/(S+R), T_solare = T_c·S/(S+R). Con il termico fermo una frizione unidirezionale o la coppia di MG1/MG2 impediscono che giri all\'indietro.',
      explode: [0.6, 0, 0.3],
    });

    const ring = new THREE.Group();
    ring.position.set(0, YA, 0);
    const ringShape = polyShape(toothPts(ZCD, MC));
    ringShape.holes.push(polyShape(toothPts(R, MP, true), THREE.Path));
    const ringMesh = new THREE.Mesh(extrudeX(ringShape, 0.11, 8), MAT.steel());
    ring.add(ringMesh);
    add(ring, {
      id: 'corona',
      name: 'Corona (verso le ruote)',
      base: 'L\'anello esterno con i denti all\'interno, che ingranano con i satelliti, e all\'esterno, che portano il moto verso le ruote. Gira sempre insieme alle ruote.',
      expert:
        'Corona da 78 denti interni con dentatura esterna di rinvio (counter drive gear). È l\'uscita del rotismo: la sua velocità è proporzionale a quella del veicolo. Qui confluiscono la potenza meccanica del termico e, tramite il contralbero, quella di MG2.',
      explode: [0.05, 0.15, 0.8],
    });

    // ================= Counter shaft (counter driven, MG2 reduction, final drive pinion) =================
    const counter = new THREE.Group();
    counter.position.set(0, YC, 0);
    const cdMesh = new THREE.Mesh(gearGeo(ZCN, MC, 0.1, 0.04, 6, 0.04, 0.22), MAT.darkSteel());
    const rdMesh = new THREE.Mesh(gearGeo(ZRD, MR, 0.08, 0.04, 6, 0.06, 0.3), MAT.darkSteel());
    rdMesh.position.x = X_RD;
    const fpMesh = new THREE.Mesh(gearGeo(ZFP, MF, 0.09, 0.035), MAT.steel());
    fpMesh.position.x = X_FD;
    const cShaft = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.04, 0.04, 0.86, 16)), MAT.steel());
    counter.add(cdMesh, rdMesh, fpMesh, cShaft);
    add(counter, {
      id: 'contralbero',
      name: 'Contralbero e riduzione di MG2',
      base: 'Un albero con tre ruote dentate: una riceve il moto dalla corona, una da MG2, la terza lo passa al differenziale. È qui che la spinta del motore a benzina e quella elettrica si sommano.',
      expert:
        'Ingranaggi elicoidali: corona→contralbero 52/40, riduzione di MG2 21/58 (~2,8:1, così MG2 può essere piccolo e veloce), pignone finale 18/58 (~3,2:1). Il rapporto tra MG2 e ruote è fisso: MG2 gira sempre con l\'auto.',
      explode: [0.1, -0.2, 0.6],
    });

    // ================= MG2 (traction motor) =================
    const mg2 = buildMG(0.5, housingMat);
    mg2.stator.position.set(X_MG2, YM2, 0);
    add(mg2.stator, {
      id: 'mg2',
      name: 'MG2 (motore di trazione)',
      base: 'Il motore elettrico principale: spinge l\'auto da solo in marcia elettrica, aiuta il motore a benzina in accelerazione e in frenata diventa un generatore.',
      expert:
        'Sincrono a magneti permanenti interni, ~70-90 kW e ~200 Nm nelle versioni 2.0, fino a ~17.000 rpm con riduzione dedicata. Raffreddato a olio; in rigenerazione lavora in coppia negativa e ricarica la batteria.',
      explode: [-0.3, -0.35, -0.95],
    });
    const mg2Rot = new THREE.Group();
    mg2Rot.position.set(0, YM2, 0);
    const mg2Spin = new THREE.Group();
    mg2.rotor.position.x = X_MG2;
    mg2Spin.add(mg2.rotor);
    mg2Spin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.04, 0.04, 0.95, 16)), MAT.steel()).translateX(-0.77));
    mg2Rot.add(mg2Spin);
    const mpMesh = new THREE.Mesh(gearGeo(ZMP, MR, 0.08, 0.035), MAT.steel());
    mpMesh.position.x = X_RD;
    mg2Rot.add(mpMesh);
    add(mg2Rot, {
      id: 'rotoreMg2',
      name: 'Rotore di MG2 e pignone',
      base: 'Il rotore con le calamite di MG2 e il piccolo ingranaggio che lo collega al contralbero.',
      expert: 'Pignone da 21 denti che ingrana con la ruota da 58 del contralbero: MG2 gira ~2,8 volte più veloce del contralbero e ~9 volte più veloce delle ruote.',
      explode: [-0.3, -0.3, 0.5],
    });

    // ================= Differential + half-shaft + wheel =================
    const diff = new THREE.Group();
    diff.position.set(X_FD, YD, 0);
    const diffSpin = new THREE.Group();
    const frMesh = new THREE.Mesh(gearGeo(ZFR, MF, 0.09, 0.2, 8, 0.03, 0.3), MAT.darkSteel());
    diffSpin.add(frMesh);
    const caseProf = [
      [0.05, -0.2], [0.085, -0.2], [0.09, -0.15], [0.155, -0.09], [0.175, -0.04], [0.205, -0.04],
      [0.205, 0.0], [0.175, 0.0], [0.17, 0.05], [0.15, 0.1], [0.09, 0.16], [0.085, 0.2], [0.05, 0.2],
    ];
    diffSpin.add(new THREE.Mesh(alongX(lathe(caseProf, 40)), cut(MAT.castIron())));
    diffSpin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.27, 10), MAT.steel()));
    for (const s of [-1, 1]) {
      const pg = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.032, 0.05, 14), MAT.steel());
      pg.position.y = s * 0.085;
      if (s < 0) pg.rotation.x = Math.PI;
      diffSpin.add(pg);
    }
    diff.add(diffSpin);
    add(diff, {
      id: 'differenziale',
      name: 'Coppia finale e differenziale',
      base: 'La corona grande riceve il moto dal contralbero e, tramite il differenziale, lo divide tra le due ruote.',
      expert: 'Coppia finale 18/58 (~3,2:1) e differenziale a satelliti conici integrati nella scatola del cambio (transaxle).',
      explode: [0.45, -0.35, 0.5],
    });

    const hs = new THREE.Group();
    hs.position.set(0, YD, 0);
    const hsSpin = new THREE.Group();
    const steel = MAT.steel();
    const rubber = MAT.rubber();
    const cupGeo = (len, r) => alongX(lathe([[0.03, -len / 2], [r * 0.8, -len / 2], [r, -len / 4], [r, len / 4], [r * 0.6, len / 2], [0.03, len / 2]], 24));
    hsSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.14, 12)), steel).translateX(0.6));
    hsSpin.add(new THREE.Mesh(cupGeo(0.13, 0.095), steel).translateX(0.72));
    hsSpin.add(new THREE.Mesh(bellows(0.78, 0.95, 0.045, 0.08), rubber));
    hsSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.55, 12)), steel).translateX(1.18));
    hsSpin.add(new THREE.Mesh(bellows(1.3, 1.44, 0.045, 0.08), rubber));
    hsSpin.add(new THREE.Mesh(cupGeo(0.11, 0.1), steel).translateX(1.49));
    hs.add(hsSpin);
    add(hs, {
      id: 'semiasse',
      name: 'Semiasse',
      base: 'L\'albero che porta la rotazione dal differenziale alla ruota, con due giunti snodati protetti da cuffie di gomma.',
      expert: 'Semiasse con giunto tripode lato cambio e giunto Rzeppa lato ruota (omocinetici). L\'altro semiasse, verso la ruota sinistra, è nascosto dietro la scatola.',
      explode: [0.6, -0.35, 0.3],
    });

    const wheel = new THREE.Group();
    wheel.position.set(WHEEL_X, YD, 0);
    const wSpin = new THREE.Group();
    const tyreMat = cut(MAT.rubber());
    const TR = 0.62;
    const tyreProf = [
      [0.42, -0.15], [0.47, -0.18], [0.55, -0.185], [0.6, -0.17], [0.615, -0.13], [TR, -0.09],
      [TR, 0.09], [0.615, 0.13], [0.6, 0.17], [0.55, 0.185], [0.47, 0.18], [0.42, 0.15],
    ];
    wSpin.add(new THREE.Mesh(alongX(lathe(tyreProf, 64)), tyreMat));
    for (let i = 0; i < 36; i++) {
      const a = (i / 36) * TAU;
      const t = box(0.17, 0.016, 0.05, tyreMat, i % 2 ? 0.02 : -0.02, Math.cos(a) * (TR + 0.006), Math.sin(a) * (TR + 0.006));
      t.rotation.x = a;
      wSpin.add(t);
    }
    const rimMat = MAT.aluminum();
    wSpin.add(new THREE.Mesh(alongX(pipe(0.42, 0.395, 0.31, 56)), rimMat));
    wSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.1, 0.1, 0.07, 24)), rimMat).translateX(0.1));
    for (let i = 0; i < 10; i++) {
      const a = (i / 10) * TAU;
      const sk = box(0.035, 0.31, 0.04, rimMat, 0.1, Math.cos(a) * 0.245, Math.sin(a) * 0.245);
      sk.rotation.x = a;
      wSpin.add(sk);
    }
    wSpin.add(new THREE.Mesh(alongX(pipe(0.32, 0.14, 0.028, 56)), MAT.darkSteel()).translateX(-0.08));
    wSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.07, 0.07, 0.14, 20)), MAT.darkSteel()).translateX(-0.13));
    wheel.add(wSpin);
    const knMat = MAT.castIron();
    wheel.add(new THREE.Mesh(alongX(pipe(0.11, 0.075, 0.09, 28)), knMat).translateX(-0.17));
    wheel.add(new THREE.Mesh(new RoundedBoxGeometry(0.05, 0.6, 0.08, 2, 0.02), knMat).translateX(-0.17).translateY(-0.03).translateZ(-0.15));
    wheel.add(new THREE.Mesh(new THREE.SphereGeometry(0.035, 12, 10), MAT.steel()).translateX(-0.17).translateY(-0.33).translateZ(-0.15));
    wheel.add(box(0.09, 0.05, 0.1, knMat, -0.13, 0.2, -0.14));
    const caliper = box(0.1, 0.19, 0.11, MAT.paint(0x2b2f35, 0.5), -0.08, 0.2, -0.19);
    caliper.rotation.x = -0.76;
    wheel.add(caliper);
    add(wheel, {
      id: 'ruota',
      name: 'Ruota motrice',
      base: 'La ruota anteriore. La sua velocità è legata in modo fisso alla corona del rotismo e a MG2.',
      expert:
        'Ruota anteriore motrice (trazione anteriore; le versioni AWD-i aggiungono un MG posteriore senza albero di trasmissione). In frenata buona parte della decelerazione è affidata a MG2: i dischi lavorano meno e si consumano poco.',
      explode: [1.0, -0.2, 0],
    });

    // ================= Transaxle casing (hollow: walls + end plates with bearing bores) =================
    const caseHull = (dr, n = 36) => {
      const pts = [];
      for (const [y, r] of [[YA, 0.53], [YC, 0.53], [YD, 0.56], [YM2, 0.22]]) {
        for (let i = 0; i < n; i++) pts.push([Math.cos((i / n) * TAU) * (r + dr), y + Math.sin((i / n) * TAU) * (r + dr)]);
      }
      return convexHull(pts);
    };
    const caseMat = cut(MAT.aluminum());
    caseMat.color.set(0xb4bac1);
    caseMat.roughness = 0.6;
    const exX = (shape, depth, x0, mat = caseMat) => {
      const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: 16 });
      g.rotateY(Math.PI / 2);
      return new THREE.Mesh(g, mat).translateX(x0);
    };
    const hullRing = (outer, inner) => {
      const sh = polyShape(outer);
      sh.holes.push(polyShape(inner.slice().reverse(), THREE.Path));
      return sh;
    };
    const tcase = new THREE.Group();
    tcase.add(exX(hullRing(caseHull(0), caseHull(-0.035)), 0.96, -0.42));
    for (const [x0, bl] of [[-0.46, [[YA, 0.075], [YC, 0.045], [YM2, 0.05]]], [0.54, [[YC, 0.045], [YD, 0.1]]]]) {
      const pl = polyShape(caseHull(0));
      for (const [y, r] of bl) pl.holes.push(circlePath(0, y, r));
      tcase.add(exX(pl, 0.04, x0));
      for (const [y, r] of bl) tcase.add(new THREE.Mesh(alongX(pipe(r + 0.026, r, 0.04, 28)), cut(MAT.chrome())).translateX(x0 + 0.02).translateY(y));
    }
    // split flange with bolts, gasket line, ribs, plugs, breather
    tcase.add(exX(hullRing(caseHull(0.045), caseHull(0)), 0.04, 0.04));
    tcase.add(exX(hullRing(caseHull(0.006), caseHull(-0.035)), 0.006, 0.08, cut(MAT.rubber())));
    // flange screws: their own part (Officina), heads on the +X face of the flange
    const screwMat = cut(MAT.chrome(), { cap: false });
    const caseScrews = caseHull(0.023, 16).filter((_, i) => i % 2 === 0).map(([u, v]) => [0.089, v, -u, 'x']);
    for (const z of [-0.26, 0.26]) tcase.add(new THREE.Mesh(new RoundedBoxGeometry(0.03, YA - YD, 0.035, 2, 0.01), caseMat).translateX(0.59).translateY((YA + YD) / 2).translateZ(z));
    tcase.add(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.03, 6), MAT.darkSteel()).translateX(0.3).translateY(YD - 0.575));
    tcase.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.03, 6)), MAT.darkSteel()).translateX(0.595).translateY(YC).translateZ(-0.3));
    tcase.add(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.022, 0.06, 12), MAT.paint(0x1c1f24, 0.6)).translateX(0.25).translateY(YA + 0.55).translateZ(-0.12));
    add(tcase, {
      id: 'cassa',
      name: 'Scatola del cambio (transaxle)',
      base: 'La scatola di alluminio, in due metà imbullonate, che racchiude rotismo, ingranaggi e differenziale, immersi nell’olio. Ai suoi lati sono fissati MG1 e MG2.',
      expert:
        'Carter in lega di alluminio in due semigusci con guarnizione; un circuito d’olio lubrifica gli ingranaggi e raffredda i motori elettrici. Nessuna frizione, nessun innesto: solo ingranaggi sempre in presa, supportati da cuscinetti nelle pareti.',
      explode: [0.3, 0.1, -1.15],
    });
    add(screws(caseScrews, screwMat, screwGeo(0.034)), {
      id: 'vitiCassa',
      name: 'Viti della scatola del cambio',
      base: 'La corona di viti che stringe tra loro le due metà della scatola, lungo la flangia.',
      expert:
        'Viti disposte lungo tutta la flangia per schiacciare in modo uniforme la guarnizione: la scatola deve tenere l’olio che lubrifica gli ingranaggi e raffredda i motori elettrici.',
      explode: [0.3, 0.1, -1.15],
    });

    // ================= Power control unit =================
    const pcu = new THREE.Group();
    const pcuMat = cut(MAT.paint(0x9aa3ad, 0.45));
    pcu.add(shellBox(1.45, 0.32, 0.8, 0.02, 0.05, pcuMat, -0.28, 1.54, -0.05));
    const lid = rrect(1.49, 0.84, 0.07, -0.28, 0.05);
    lid.holes.push(rrect(1.41, 0.76, 0.04, -0.28, 0.05, THREE.Path));
    pcu.add(new THREE.Mesh(extrudeY(lid, 0.02, 1.64), pcuMat));
    const lb = [];
    for (let i = 0; i < 7; i++) lb.push([-0.92 + i * 0.213, 1.665, 0.37, 'y'], [-0.92 + i * 0.213, 1.665, -0.47, 'y']);
    pcu.add(bolts(lb, cut(MAT.chrome(), { cap: false }), 0.012, 0.016));
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.08, 0.27, 0.3, 2, 0.02), pcuMat).translateX(-0.85).translateY(1.265).translateZ(-0.1));
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.08, 0.14, 0.3, 2, 0.02), pcuMat).translateX(0.3).translateY(1.33).translateZ(-0.1));
    // mounting ears at the base of the two feet, screwed down to MG1 and to the transaxle case
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.17, 0.024, 0.28, 2, 0.008), pcuMat).translateX(-0.745).translateY(1.145).translateZ(-0.1));
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.2, 0.024, 0.28, 2, 0.008), pcuMat).translateX(0.42).translateY(1.272).translateZ(-0.1));
    const pcuScrews = [];
    for (const [x, y] of [[-0.72, 1.166], [0.48, 1.293]]) for (const z of [-0.2, 0]) pcuScrews.push([x, y, z, 'y']);
    pcu.add(box(1.3, 0.025, 0.68, cut(MAT.aluminum()), -0.28, 1.4, -0.05)); // cold plate
    const pcuSw = [];
    for (const [gx, n] of [[-0.75, 'mg1'], [-0.3, 'mg2']]) {
      for (let k = 0; k < 3; k++) {
        const m = box(0.11, 0.035, 0.16, glowMat(MAT.paint(0x23272d, 0.5)), gx + k * 0.13, 1.43, 0.17);
        pcu.add(m);
        pcuSw.push({ mesh: m, n, k });
      }
    }
    const coil = new THREE.Mesh(new THREE.TorusGeometry(0.075, 0.03, 10, 24).rotateY(Math.PI / 2), MAT.copper());
    coil.position.set(0.25, 1.5, 0.17);
    pcu.add(coil);
    pcu.add(box(0.9, 0.16, 0.25, MAT.paint(0xb8952f, 0.45), -0.35, 1.5, -0.25));
    pcu.add(box(1.2, 0.012, 0.3, MAT.paint(0x1d6b3a, 0.6), -0.3, 1.66, -0.22));
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.06, 0.14, 0.3, 2, 0.015), MAT.paint(0xf07a1a, 0.5)).translateX(0.47).translateY(1.52).translateZ(0.175));
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.12, 0.1, 0.06, 2, 0.015), MAT.paint(0xf07a1a, 0.5)).translateX(-0.95).translateY(1.47).translateZ(-0.47));
    pcu.add(new THREE.Mesh(new RoundedBoxGeometry(0.06, 0.07, 0.14, 2, 0.012), MAT.paint(0x1c1f24, 0.6)).translateX(0.47).translateY(1.6).translateZ(-0.25));
    const pl = plate(['ALTA TENSIONE', '650 V'], 0.26, 0.11);
    pl.rotation.x = -Math.PI / 2;
    pl.position.set(-0.55, 1.701, -0.25);
    pcu.add(pl);
    add(pcu, {
      id: 'pcu',
      name: 'Unità di controllo della potenza (PCU)',
      base:
        "Il «vigile urbano» dell'energia: decide istante per istante quanta corrente va dalla batteria ai motori, da MG1 a MG2 o dai motori alla batteria. Contiene due inverter (uno per motore) e un elevatore di tensione.",
      expert:
        'Due inverter trifase (IGBT o SiC) per MG1 e MG2 sullo stesso DC-link, più un convertitore boost (la bobina di rame) che eleva la tensione della batteria da ~200 V a 600-650 V. ' +
        'Raffreddamento a liquido su piastra fredda; la centralina ibrida coordina iniezione, MG1, MG2 e freni.',
      explode: [0, 1.0, 0],
    });
    add(screws(pcuScrews, screwMat, screwGeo(0.07, 0.02, 0.018)), {
      id: 'vitiPcu',
      name: 'Viti di fissaggio della PCU',
      base: 'Quattro viti che fissano i piedini della centralina di potenza sopra il cambio.',
      expert:
        'La centralina è montata direttamente sul transaxle per tenere corti i cavi trifase verso MG1 e MG2: meno peso, meno perdite e meno disturbi elettromagnetici.',
      explode: [0, 1.0, 0],
    });

    // ================= HV battery (on a support bracket bolted to the transaxle) =================
    const batt = new THREE.Group();
    const packMat = cut(MAT.paint(0x3b4049, 0.55));
    batt.add(shellBox(1.3, 0.4, 1.1, 0.02, 0.05, packMat, 1.4, 0.75, -0.3));
    const cellMat = MAT.paint(0x8a97a8, 0.35);
    const cellGeo = new THREE.BoxGeometry(0.085, 0.3, 0.46);
    const termGeo = new THREE.BoxGeometry(0.03, 0.02, 0.05);
    const busMat = MAT.copper();
    for (let i = 0; i < 12; i++) {
      for (const z of [-0.57, -0.06]) {
        const x = 0.84 + i * 0.1;
        batt.add(new THREE.Mesh(cellGeo, cellMat).translateX(x).translateY(0.74).translateZ(z));
        for (const dz of [-0.15, 0.15]) batt.add(new THREE.Mesh(termGeo, busMat).translateX(x).translateY(0.9).translateZ(z + dz));
      }
    }
    for (const z of [-0.72, -0.42, -0.21, 0.09]) batt.add(box(1.18, 0.008, 0.035, busMat, 1.39, 0.912, z));
    batt.add(box(0.1, 0.06, 0.16, MAT.paint(0xf07a1a, 0.5), 1.9, 0.98, -0.6)); // service plug
    // hold-down straps and the cooling blower (air-cooled pack)
    for (const x of [1.0, 1.8]) {
      batt.add(new THREE.Mesh(new RoundedBoxGeometry(0.06, 0.012, 1.14, 2, 0.005), MAT.darkSteel()).translateX(x).translateY(0.956).translateZ(-0.3));
      for (const z of [-0.872, 0.272]) batt.add(box(0.06, 0.42, 0.012, MAT.darkSteel(), x, 0.745, z));
    }
    batt.add(new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.09, 24).rotateX(Math.PI / 2), MAT.paint(0x22262c, 0.6)).translateX(1.0).translateY(0.75).translateZ(-0.9));
    batt.add(new THREE.Mesh(new RoundedBoxGeometry(0.3, 0.12, 0.08, 2, 0.02), MAT.paint(0x22262c, 0.6)).translateX(1.25).translateY(0.75).translateZ(-0.9));
    const bl = plate(['ALTA TENSIONE', '≈ 200 V c.c.'], 0.3, 0.13);
    bl.rotation.x = -Math.PI / 2;
    bl.position.set(1.4, 0.951, -0.62);
    batt.add(bl);
    add(batt, {
      id: 'batteria',
      name: 'Batteria ad alta tensione',
      base:
        "Una batteria piccola, circa 1 kWh (più o meno quanto 60 batterie di smartphone), che non si ricarica mai alla presa: la riempiono MG1 e le frenate. Fa da «polmone» per l'energia. " +
        "Nell'auto vera sta sotto il sedile posteriore; qui è montata su un supporto accanto al cambio per vederla insieme al resto.",
      expert:
        'NiMH (~1,3 kWh, 168 celle, ~200 V) o Li-ion (~0,9-1 kWh, ~180-220 V) a seconda della versione, raffreddata ad aria da un ventilatore. Lavora tra ~40 e ~80% di carica e scambia potenze di picco di ~25-40 kW. ' +
        'Molto più piccola di quella di un plug-in (10-25 kWh) o di un’elettrica (50-100 kWh).',
      explode: [0.6, 0.45, 0],
    });

    // ================= Supports: battery bracket and lower suspension arm =================
    const sup = new THREE.Group();
    const supMat = MAT.paint(0x2b3038, 0.55);
    const rbx = (w, h, d, x, y, z) => sup.add(new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 2, 0.012), supMat).translateX(x).translateY(y).translateZ(z));
    rbx(0.04, 0.26, 0.86, 0.6, 0.45, -0.3); // plate bolted to the transaxle
    for (const z of [-0.65, 0.05]) {
      rbx(1.5, 0.05, 0.06, 1.35, 0.525, z); // rails under the pack
      sup.add(rod([2.05, 0.5, z], [0.62, -0.02, z], 0.018, supMat)); // diagonal brace
      rbx(0.04, 0.1, 0.1, 0.6, -0.02, z); // brace foot on the case
    }
    const sb = [];
    for (const y of [0.37, 0.53]) for (const z of [-0.62, -0.3, 0.02]) sb.push([0.625, y, z, 'x']);
    sup.add(bolts(sb, MAT.chrome(), 0.013, 0.016));
    rbx(0.08, 0.08, 0.1, 0.62, YD - 0.32, -0.35); // arm bracket on the case
    sup.add(rod([0.64, YD - 0.32, -0.35], [WHEEL_X - 0.17, YD - 0.33, -0.15], 0.026, supMat));
    sup.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.045, 0.045, 0.07, 16)), MAT.rubber()).translateX(0.66).translateY(YD - 0.32).translateZ(-0.35));
    add(sup, {
      id: 'supporti',
      name: 'Supporti e braccio della sospensione',
      base: 'Le staffe d’acciaio che reggono la batteria e il braccio inferiore che tiene la ruota attaccata al resto dell’auto.',
      expert:
        'Schema semplificato: in vettura la batteria è fissata al pianale sotto il sedile posteriore e il braccio oscillante è incernierato alla culla anteriore (sospensione MacPherson). Qui staffe e braccio sono ancorati alla scatola del cambio per tenere tutto in una sola vista.',
      explode: [0.45, -0.3, -0.5],
    });

    // ================= HV cables =================
    const hv = new THREE.Group();
    const orange = MAT.paint(0xf07a1a, 0.5);
    const cBatt = [-1, 1].map((s) => curve([[0.85, 0.9, 0.12 + s * 0.05], [0.72, 1.15, 0.2 + s * 0.05], [0.58, 1.48, 0.2 + s * 0.05], [0.42, 1.52, 0.15 + s * 0.05]]));
    const cMG1 = curve([[-0.72, 1.39, 0.25], [-0.73, 1.26, 0.28], [-0.78, 1.12, 0.22]]);
    const cMG2 = curve([[-0.96, 1.45, -0.32], [-1.22, 1.25, -0.44], [-1.32, 0.4, -0.47], [-1.3, -0.25, -0.42], [-1.0, -0.42, -0.26]]);
    for (const c of [...cBatt, cMG1, cMG2]) hv.add(new THREE.Mesh(new THREE.TubeGeometry(c, 40, 0.022, 8, false), orange));
    // plugs at the cable ends on the battery, MG1 and MG2 (the PCU has its own sockets)
    const plugGeo = new RoundedBoxGeometry(0.16, 0.06, 0.12, 2, 0.015);
    hv.add(new THREE.Mesh(new RoundedBoxGeometry(0.14, 0.05, 0.2, 2, 0.015), orange).translateX(0.84).translateY(0.972).translateZ(0.13));
    hv.add(new THREE.Mesh(plugGeo, orange).translateX(-0.78).translateY(1.1).translateZ(0.22));
    hv.add(new THREE.Mesh(plugGeo, orange).translateX(-1.0).translateY(-0.395).translateZ(-0.26));
    add(hv, {
      id: 'caviHV',
      name: 'Cavi ad alta tensione',
      base: 'I cavi arancioni collegano batteria, centralina di potenza e i due motori elettrici. Arancione vuol dire alta tensione.',
      expert: 'Cavi schermati: in continua tra batteria e PCU (~200 V), trifase tra PCU e MG1/MG2 (fino a ~650 V). Correnti di picco di qualche centinaio di ampere.',
      explode: [0.2, 0.45, 0.45],
    });
    const hvScrews = [
      [0.507, 1.52, 0.06, 'x'], [0.507, 1.52, 0.29, 'x'],            // PCU socket
      [0.88, 1.004, 0.055, 'y'], [0.88, 1.004, 0.205, 'y'],          // battery plug
      [-0.835, 1.137, 0.22, 'y'], [-0.725, 1.137, 0.22, 'y'],        // MG1 plug
      [-1.05, -0.358, -0.235, 'y'], [-0.95, -0.358, -0.235, 'y'],    // MG2 plug
    ];
    add(screws(hvScrews, screwMat, screwGeo(0.035, 0.014, 0.014)), {
      id: 'vitiHV',
      name: 'Viti dei connettori ad alta tensione',
      base: 'Piccole viti che tengono fermi i connettori dei cavi arancioni su batteria, centralina e motori elettrici.',
      expert:
        'I connettori ad alta tensione sono avvitati e schermati; molti hanno anche un circuito di sicurezza (interlock) che fa aprire i contattori della batteria se un connettore viene staccato.',
      explode: [0.2, 0.45, 0.45],
    });

    // ================= Flows =================
    const shifted = (fn) => (t, out) => fn(t, out).add(OFFSET);
    const mkFlow = (path, count, size, speed) => {
      const dir = { reverse: false };
      const f = flow({
        path: shifted((t, out) => path(dir.reverse ? 1 - t : t, out)),
        count,
        size,
        colors: [0xffffff],
        spread: 0.016,
        speed,
        opacity: 0.9,
      });
      root.add(f.object);
      ctx.addFlow(f);
      f.dir = dir;
      return f;
    };
    const sampleCurve = (c, n, off) => {
      const out = [];
      for (let i = 0; i <= n; i++) out.push(c.getPointAt(i / n).add(off).toArray());
      return out;
    };
    const lift = new THREE.Vector3(0, 0.02, 0.025);
    const fBatt = mkFlow(polyPath([[1.95, 0.93, -0.06], [0.88, 0.93, 0.12], ...sampleCurve(cBatt[1], 16, lift), [0.2, 1.47, 0.1], [-0.3, 1.47, 0.1]]), 240, 0.05, 0.16);
    const fMG2 = mkFlow(polyPath([[-0.3, 1.47, 0.05], [-0.9, 1.47, -0.25], ...sampleCurve(cMG2, 24, lift), [-0.95, -0.55, -0.15]]), 220, 0.05, 0.13);
    const fMG1 = mkFlow(polyPath([[-0.78, 0.98, 0.15], ...sampleCurve(cMG1, 8, lift).reverse(), [-0.5, 1.47, 0.15], [-0.3, 1.47, 0.1]]), 110, 0.05, 0.35);
    const fFuel = mkFlow(polyPath([[-2.95, 0.25, -0.33], [-2.95, YA + 0.4, -0.31], [-2.8, YA + 0.64, -0.29], [-1.4, YA + 0.64, -0.29]]), 140, 0.045, 0.22);
    const fMechE = mkFlow(polyPath([[-2.6, YA, 0.09], [0.0, YA, 0.09], [0.0, YA - 0.46, 0.09], [0.0, YC + 0.34, 0.09]]), 200, 0.05, 0.14);
    const fMechW = mkFlow(polyPath([[X_RD, YM2 + 0.16, 0.1], [X_RD, YC, 0.1], [X_FD, YC, 0.1], [X_FD, YD, 0.1], [WHEEL_X - 0.2, YD, 0.1]]), 200, 0.05, 0.16);

    // ================= Animation =================
    let prevCycle = null;
    let thR = 0;   // ring angle (rad), from state.cycle
    let thC = 0;   // carrier / crank angle (rad)
    let soc = 0.6;
    const cycleRate = (thr) => scenario(thr).ring;

    const api = {
      root,
      cycleRate,
      update(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const dt = state.dt || 0;
        const sc = scenario(thr);
        const c = state.cycle || 0;
        if (prevCycle === null) prevCycle = c;
        let d = c - prevCycle;
        if (d < -0.5) d += 1;
        else if (d > 0.5) d -= 1;
        prevCycle = c;
        thR += d * TAU;
        thC += d * TAU * (sc.carrier / sc.ring);
        soc = clamp(soc - (sc.batt * dt) / 3600, 0.35, 0.85);

        // Planetary: S*ws + R*wr = (S+R)*wc holds on the angles, hence on the speeds.
        const thS = ((S + R) * thC - R * thR) / S;
        sunMesh.rotation.x = thS;
        mg1Spin.rotation.x = thS;
        ringMesh.rotation.x = thR;
        carrierSpin.rotation.x = thC;
        damperSpin.rotation.x = thC;
        for (let i = 0; i < 3; i++) {
          const psi = thC + (i * TAU) / 3;
          planetMeshes[i].position.set(0, Math.cos(psi) * R_C, Math.sin(psi) * R_C);
          planetMeshes[i].rotation.x = psi + (R * (thR - psi) - Math.PI) / P; // meshes with ring and sun
        }
        // Counter shaft, MG2, final drive (upper gear: PI + angle, lower gear: -PI/Z + angle).
        const thCS = (-thR * ZCD) / ZCN;
        const thM = (-thCS * ZRD) / ZMP;
        const thD = (-thCS * ZFP) / ZFR;
        cdMesh.rotation.x = -Math.PI / ZCN + thCS;
        rdMesh.rotation.x = Math.PI + thCS;
        fpMesh.rotation.x = Math.PI + thCS;
        cShaft.rotation.x = thCS;
        mg2Spin.rotation.x = thM;
        mpMesh.rotation.x = -Math.PI / ZMP + thM;
        diffSpin.rotation.x = -Math.PI / ZFR + thD;
        hsSpin.rotation.x = thD;
        wSpin.rotation.x = thD;
        api._angles = { thS, thR, thC, thCS, thM, thD }; // debug: lets dev checks verify the gear kinematics

        // Engine: crank = carrier.
        crankSpin.rotation.x = thC;
        for (const g of cams) g.rotation.x = thC / 2;
        for (const v of valves) {
          const u = (((thC / (2 * TAU) - FIRE[v.k]) % 1) + 1) % 1;
          v.g.position.y = YA + 0.6 - 0.022 * valveLift(u, v.win[0], v.win[1]);
        }
        placeLinks(thC * 0.031);
        const running = sc.eng > 0.5;
        CYL_X.forEach((x, k) => {
          const al = thC - FIRE[k] * 2 * TAU; // pin angle
          const py = CR * Math.cos(al);
          const pz = CR * Math.sin(al);
          const yPin = py + Math.sqrt(CL * CL - pz * pz);
          pistons[k].position.set(x, YA + yPin, 0);
          rods[k].position.set(x, YA + py, pz);
          rods[k].rotation.x = Math.atan2(-pz, yPin - py);
          // combustion in the power stroke (cycle 0.5..0.75 of 720°)
          const u = (((thC / (2 * TAU) - FIRE[k]) % 1) + 1) % 1;
          const crown = YA + yPin + 0.09;
          const on = running && u > 0.5 && u < 0.75;
          flames[k].visible = on;
          flames[k].scale.y = Math.max(0.01, DECK - crown);
          flames[k].position.y = (DECK + crown) / 2;
        });
        flameMat.opacity = 0.3 + 0.5 * clamp(sc.eng / 112, 0, 1);

        // Motor windings glow with |power| (green while regenerating).
        const glowMG = (mg, kw, ang, col) => {
          const act = clamp(Math.abs(kw) / 40, 0, 1);
          for (const cl of mg.coils) {
            const v = Math.cos(cl.a - ang * 2);
            cl.mesh.material.emissive.setHex(col);
            cl.mesh.material.emissiveIntensity = act * 1.1 * v * v;
          }
        };
        const regen = sc.id === 'regen';
        glowMG(mg1, sc.mg1, thS, CURRENT);
        glowMG(mg2, sc.mg2, thM, regen ? REGEN_C : CURRENT);
        const pwm = Math.sin((state.time || 0) * 80) > -0.4 ? 1 : 0.3;
        for (const sw of pcuSw) {
          const kw = sw.n === 'mg1' ? sc.mg1 : sc.mg2;
          const ang = sw.n === 'mg1' ? thS : thM;
          const i = Math.cos(ang * 2 - (sw.k * TAU) / 3);
          sw.mesh.material.emissive.setHex(regen && sw.n === 'mg2' ? REGEN_C : CURRENT);
          sw.mesh.material.emissiveIntensity = clamp(Math.abs(kw) / 40, 0, 1) * 1.4 * Math.abs(i) * pwm;
        }

        // Energy flows.
        const assembled = (state.explode || 0) < 0.05;
        const setFlow = (f, kw, scale, col, reverse) => {
          const on = assembled && Math.abs(kw) > 0.5;
          f.object.visible = on;
          f.dir.reverse = reverse;
          f.material.color.setHex(col);
          f.material.opacity = 0.35 + 0.6 * clamp(Math.abs(kw) / scale, 0, 1);
          f.update(dt, 0.4 + 1.2 * clamp(Math.abs(kw) / scale, 0, 1));
        };
        setFlow(fBatt, sc.batt, 40, sc.batt < 0 && regen ? REGEN_C : CURRENT, sc.batt < 0);
        setFlow(fMG2, sc.mg2, 60, regen ? REGEN_C : CURRENT, sc.mg2 < 0);
        setFlow(fMG1, sc.mg1, 30, CURRENT, sc.mg1 < 0);
        setFlow(fFuel, sc.eng, 112, FUEL, false);
        setFlow(fMechE, sc.eng, 112, MECH, false);
        setFlow(fMechW, regen ? 30 : sc.eng + sc.mg2, 140, regen ? REGEN_C : MECH, regen);
        for (const f of flames) f.visible = f.visible && assembled;
      },
      readout(state) {
        const sc = scenario(clamp(state.throttle || 0, 0, 1));
        return [
          { label: 'Modalità', value: sc.name, unit: '' },
          { label: 'Motore termico', value: fmt(sc.eng), unit: 'kW' },
          { label: 'Potenza elettrica', value: fmt(sc.mg2), unit: 'kW' },
          { label: 'Batteria', value: String(Math.round(soc * 100)), unit: '%' },
        ];
      },
    };
    return api;
  },
};
