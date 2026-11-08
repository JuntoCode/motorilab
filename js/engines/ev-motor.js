// Car electric drive unit (EDU): battery pack -> SiC inverter -> IPM permanent-magnet synchronous
// motor (hairpin stator, V-shaped magnets) -> 2-stage reduction (~9:1) + differential -> half-shafts -> wheel.
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, pipe, flow, clamp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const OFFSET = new THREE.Vector3(0.25, -0.5, 0); // centres the model (z must stay 0 for the cutaway)
// Motor
const XM = -0.5;          // motor centre along X
const LS = 0.8;           // lamination stack length
const R_ROT = 0.36;       // rotor radius
const R_SI = 0.375;       // stator bore
const R_SO = 0.55;        // stator outer radius
const PP = 3;             // pole pairs (6 poles, like Tesla Model 3 rear)
const SLOTS = 54;
// Gear train (all shaft axes along X in the z = 0 plane)
const Z1 = 17, Z2 = 53, Z3 = 19, Z4 = 55;
const M1 = 0.012, M2 = 0.0125;
const RATIO = (Z2 / Z1) * (Z4 / Z3); // ~9.0
const YD = 0;                               // differential / half-shaft axis
const YI = YD + (M2 * (Z3 + Z4)) / 2;       // intermediate shaft
const YM = YI + (M1 * (Z1 + Z2)) / 2;       // motor axis
const X_ST1 = 0.2;        // stage 1 gear plane
const X_ST2 = 0.44;       // stage 2 gear plane
const WHEEL_X = 1.98;
const TYRE_R = 0.75;
// Driving model
const REGEN = 0.03;       // throttle below this = lift-off -> regenerative braking
const MAX_RPM = 16000;
const P_MAX = 220;        // kW
const WHEEL_C = 2.136;    // m, tyre circumference (r ~ 0.34 m)
const REGEN_KMH = 90;
const CURRENT = 0xd8ff4a;
const FIELD = 0xff4ad8;
const REGEN_C = 0x4ade80;

// ---------- small builders ----------
const alongX = (g) => g.rotateZ(-Math.PI / 2); // lathe/cylinder axis Y -> X
const UP = new THREE.Vector3(0, 1, 0);

function box(w, h, d, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  return m;
}

function rod(a, b, r, mat, seg = 10) {
  const va = new THREE.Vector3(...a);
  const vb = new THREE.Vector3(...b);
  const dir = vb.clone().sub(va);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, dir.length(), seg), mat);
  m.position.copy(va).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(UP, dir.normalize());
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

// Extrude shapes along X, centred on x = 0.
function extrudeX(shapes, width, curveSegments = 16) {
  const g = new THREE.ExtrudeGeometry(shapes, { depth: width, bevelEnabled: false, curveSegments });
  g.translate(0, 0, -width / 2);
  g.rotateY(Math.PI / 2); // shape (u,v) -> (z=-u, y=v), extrusion along X
  return g;
}

// Spur gear around X; tooth k centred at polar angle 2*PI*k/N.
function toothPts(teeth, m) {
  const rp = (m * teeth) / 2;
  const prof = [[rp - 1.25 * m, -0.62], [rp + m, -0.3], [rp + m, 0.3], [rp - 1.25 * m, 0.62]];
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

// Rubber CV boot: zig-zag lathe between x0 and x1 (axis X).
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

function glowMat(base, color = CURRENT) {
  base.emissive.setHex(color);
  base.emissiveIntensity = 0;
  return base;
}

// Magnet slab between inner point a and outer point b (shape space), thickness th.
function slabPts(a, b, th, ext = 0) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const len = Math.hypot(dx, dy);
  const ux = dx / len, uy = dy / len;
  const nx = (-uy * th) / 2, ny = (ux * th) / 2;
  const a2 = [a[0] - ux * ext, a[1] - uy * ext];
  const b2 = [b[0] + ux * ext, b[1] + uy * ext];
  return [[a2[0] + nx, a2[1] + ny], [b2[0] + nx, b2[1] + ny], [b2[0] - nx, b2[1] - ny], [a2[0] - nx, a2[1] - ny]];
}

// Interior V magnets: pole k centred at 60°*k, two slabs per pole.
function vMagnets(th, ext) {
  const list = [];
  for (let k = 0; k < PP * 2; k++) {
    const phi = (TAU * k) / (PP * 2);
    for (const j of [-1, 1]) {
      const at = (rad, tan) => {
        const d = sp(1, phi);
        const t = sp(1, phi + Math.PI / 2);
        return [d[0] * rad + t[0] * tan * j, d[1] * rad + t[1] * tan * j];
      };
      list.push({ k, pts: slabPts(at(0.2, 0.03), at(0.305, 0.13), th, ext) });
    }
  }
  return list;
}

const fmt = (n) => Math.round(n).toLocaleString('it-IT');

// ---------- casting / fastener helpers ----------
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
function extrudeY(shape, h, y0 = 0, curveSegments = 6) {
  const g = new THREE.ExtrudeGeometry(shape, { depth: h, bevelEnabled: false, curveSegments });
  g.rotateX(-Math.PI / 2); // (u, v, w) -> (u, w, -v)
  g.translate(0, y0, 0);
  return g;
}

// Hollow rounded box with wall thickness t (closed solids, so the section shows the walls).
function shellBox(w, h, d, t, r, mat, x = 0, y = 0, z = 0) {
  const g = new THREE.Group();
  const ring = rrect(w, d, r);
  ring.holes.push(rrect(w - 2 * t, d - 2 * t, Math.max(0.006, r - t), 0, 0, THREE.Path));
  g.add(new THREE.Mesh(extrudeY(ring, h - 2 * t, -h / 2 + t), mat));
  const plate = extrudeY(rrect(w, d, r), t, 0);
  g.add(new THREE.Mesh(plate, mat).translateY(-h / 2));
  g.add(new THREE.Mesh(plate, mat).translateY(h / 2 - t));
  g.position.set(x, y, z);
  return g;
}

// One InstancedMesh of hex bolt heads; list of [x, y, z, axis] with axis 'x' | 'y' | 'z'.
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

// Generic warning / name plate (canvas texture, no real brands).
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

// Lamination outlines for the three rotor types (shape space, see sp()).
function eesmPoleShape() {
  const s = new THREE.Shape();
  const wb = 0.058;
  const rh = 0.14;
  const half = 0.4;
  const at = (phi, rad, tan) => {
    const d = sp(1, phi);
    const t = sp(1, phi + Math.PI / 2);
    return [d[0] * rad + t[0] * tan, d[1] * rad + t[1] * tan];
  };
  const pts = [];
  for (let k = 0; k < PP * 2; k++) {
    const phi = (TAU * k) / (PP * 2);
    const rIn = Math.sqrt(rh * rh - wb * wb);
    pts.push(at(phi, rIn, -wb), at(phi, 0.285, -wb), sp(0.285, phi - half));
    for (let i = 0; i <= 8; i++) pts.push(sp(0.355, phi - half + (i / 8) * 2 * half));
    pts.push(sp(0.285, phi + half), at(phi, 0.285, wb), at(phi, rIn, wb));
    const a0 = phi + Math.asin(wb / rh);
    const a1 = phi + TAU / (PP * 2) - Math.asin(wb / rh);
    for (let i = 1; i < 6; i++) pts.push(sp(rh, a0 + ((a1 - a0) * i) / 6));
  }
  pts.forEach(([u, v], i) => (i ? s.lineTo(u, v) : s.moveTo(u, v)));
  s.closePath();
  return s;
}

const CAGE_BARS = 36;
function cageSlot(j, w, h) {
  const a = (j / CAGE_BARS) * TAU;
  const d = sp(1, a);
  const t = sp(1, a + Math.PI / 2);
  const c = [d[0] * 0.322, d[1] * 0.322];
  const P = (rad, tan) => [c[0] + d[0] * rad + t[0] * tan, c[1] + d[1] * rad + t[1] * tan];
  return [P(-h / 2, -w / 2), P(-h / 2, w / 2), P(h / 2, w / 2), P(h / 2, -w / 2)];
}

export default {
  id: 'evMotor',
  name: 'Motore elettrico a magneti permanenti (auto elettrica)',
  short: 'Elettrico a magneti',
  family: 'terra',
  vehicle: 'Auto elettrica',
  examples: 'Tesla Model 3/Y, Volkswagen ID.3/ID.4 (APP310), Fiat 600e, Hyundai Ioniq 5',
  tagline: 'Batteria, elettronica e calamite: zero marce, coppia piena da fermo.',
  intro: {
    base:
      "Un'auto elettrica non ha pistoni né cambio: sotto il pianale c'è un grande <b>pacco batteria</b> fatto di migliaia di piccole celle. " +
      "Un <b>inverter</b> trasforma la sua corrente continua in tre correnti alternate che fanno girare un campo magnetico dentro il <b>motore</b>; " +
      "il rotore, pieno di <b>calamite</b>, gira agganciato a quel campo. Un semplice <b>riduttore</b> porta il moto alle ruote. " +
      "Quando alzi il piede, tutto funziona al contrario e il motore <b>ricarica la batteria</b> frenando l'auto.",
    expert:
      "Unità di trazione integrata (EDU): batteria Li-ion 400 V (o 800 V nelle piattaforme più recenti), inverter a MOSFET in <b>carburo di silicio (SiC)</b>, " +
      "<b>motore sincrono a magneti permanenti interni (IPM-SynRM)</b> con statore ad avvolgimento <b>hairpin</b> e rotore a magneti NdFeB disposti a V, " +
      "riduttore a due stadi a rapporto fisso (~9:1) con differenziale. Rendimento del motore 94-97% (picco), dalla batteria alla ruota ~85-90%; " +
      "coppia massima disponibile da 0 rpm e regime fino a 16.000-20.000 rpm.",
  },
  specs: [
    { label: 'Alimentazione', value: 'Batteria Li-ion 400 V / 800 V c.c.' },
    { label: 'Tipo di motore', value: 'Sincrono a magneti permanenti interni (IPM)' },
    { label: 'Potenza', value: '150-250 kW (tipico 220 kW)' },
    { label: 'Coppia', value: '≈ 300-550 Nm da 0 rpm' },
    { label: 'Giri max', value: '16.000-20.000 rpm' },
    { label: 'Rendimento', value: '94-97 % motore, 85-90 % batteria→ruota' },
    { label: 'Rapporto riduttore', value: '≈ 9:1, una sola marcia' },
  ],
  metrics: { efficiency: 90, power: 220, powerDensity: 2.5 },
  variants: {
    columns: ['Veicolo', 'Tipo di motore', 'Potenza', 'Note'],
    rows: [
      ['Tesla Model 3 (motore posteriore)', 'Sincrono a magneti permanenti interni', '—', 'Inverter SiC, riduttore ~9:1'],
      ['Tesla Model 3/Y Dual Motor (anteriore)', 'Asincrono a induzione', '—', 'Senza magneti: a vuoto non frena'],
      ['Tesla Model S/X (2012-2019)', 'Asincrono a induzione', '—', 'Rotore a gabbia in rame'],
      ['Volkswagen ID.3 / ID.4 (APP310)', 'Sincrono a magneti permanenti', '150 kW', '310 Nm, trazione posteriore'],
      ['Nissan Leaf (40 kWh)', 'Sincrono a magneti permanenti', '110 kW', '320 Nm'],
      ['Renault Mégane E-Tech Electric', 'Sincrono a rotore avvolto (EESM)', '160 kW', 'Senza terre rare'],
      ['BMW iX3 / i4 (eDrive 5ª gen.)', 'Sincrono a rotore avvolto (EESM)', '210-250 kW', 'Senza terre rare'],
    ],
    note: 'Le potenze dipendono dalla versione; dove il costruttore non le dichiara per il singolo motore sono omesse.',
  },
  options: [
    {
      id: 'motore',
      label: 'Rotore',
      default: 'pm',
      choices: [
        {
          id: 'pm',
          label: 'Magneti permanenti',
          base: 'Calamite al neodimio nascoste nel rotore: il rotore gira agganciato al campo. È il tipo più diffuso perché è il più efficiente e compatto.',
          expert: 'IPM-SynRM con magneti NdFeB a V: coppia di allineamento + riluttanza, rendimento di picco ~97%. Svantaggi: terre rare e perdite per controtensione ad alta velocità a vuoto.',
        },
        {
          id: 'eesm',
          label: 'Rotore avvolto (EESM)',
          base: 'Al posto delle calamite ci sono bobine di rame sul rotore, alimentate attraverso due anelli e delle spazzole: un «elettromagnete» che si può regolare. Niente terre rare.',
          expert: 'Macchina sincrona a eccitazione separata: la corrente di campo (alcune decine di A) arriva tramite anelli e spazzole (o trasferimento induttivo). Il flusso è regolabile, ottimo rendimento alle alte velocità; perdite nel rame del rotore a basso regime. Usato da Renault e BMW (eDrive 5ª generazione).',
        },
        {
          id: 'asm',
          label: 'Induzione (asincrono)',
          base: 'Rotore a gabbia di barre di rame, come nel treno: nessuna calamita e nessun contatto. Il rotore gira un po\' più lento del campo.',
          expert: 'Asincrono a gabbia di scoiattolo (rame pressofuso): robusto ed economico, senza magneti; a vuoto si può diseccitare del tutto, per questo Tesla lo usa sull\'asse anteriore dei Dual Motor. Rendimento un po\' inferiore per le perdite nel rotore (scorrimento 1-3%).',
        },
      ],
    },
  ],
  thumb: { orbit: [-38, 20], zoom: 1.25, view: 'section' },
  phases: [
    {
      id: 'batteria',
      name: 'Batteria (corrente continua)',
      parts: ['pacco', 'celle', 'caviHV'],
      base:
        "Il pacco batteria è una cassa piatta sotto il pavimento piena di celle, simili a grosse pile, collegate in serie e in parallelo. " +
        "Insieme danno una corrente continua a circa 400 volt (o 800 nelle auto più nuove), che viaggia nei cavi arancioni fino all'inverter. " +
        "Porta l'acceleratore a zero: la corrente diventa verde e torna indietro, perché in frenata il motore ricarica la batteria.",
      expert:
        'Celle cilindriche (2170/4680) o prismatiche/pouch, chimica NMC o LFP, raggruppate in moduli: ~96 gruppi in serie danno ~350-400 V nominali (~192 in serie per 800 V). ' +
        'Capacità tipica 60-80 kWh (densità di cella ~250-280 Wh/kg). Il BMS controlla tensione e temperatura di ogni gruppo; una piastra a liquido mantiene le celle a 20-40 °C. ' +
        'I contattori e il fusibile pirotecnico isolano il pacco a vettura spenta o in caso di urto. In rigenerazione il flusso si inverte: fino a 100-250 kW di ricarica in frenata.',
    },
    {
      id: 'inverter',
      name: 'Inverter (da continua a trifase)',
      parts: ['inverter', 'moduliSiC', 'condensatore', 'sbarre'],
      base:
        "Il motore ha bisogno di corrente alternata, la batteria dà corrente continua. L'inverter è un gruppo di interruttori elettronici che si aprono e chiudono decine di migliaia di volte al secondo: " +
        "ricompone così tre correnti alternate, una per ciascuna sbarra di rame, sfasate tra loro come tre pedalate. Più veloce è il ritmo, più veloce gira il motore.",
      expert:
        'Ponte trifase a 6 interruttori (3 rami) con MOSFET SiC da 750 V (sistemi 400 V) o 1200 V (800 V), modulazione PWM a 8-20 kHz e controllo vettoriale FOC delle correnti i_d/i_q. ' +
        'Il condensatore di DC-link a film livella la tensione; il SiC rispetto agli IGBT al silicio riduce le perdite di commutazione e porta il rendimento dell\'inverter a ~98-99%. ' +
        'In rigenerazione lo stesso ponte funziona da raddrizzatore attivo e rimanda energia alla batteria.',
    },
    {
      id: 'campo',
      name: 'Statore e campo rotante',
      parts: ['statore', 'hairpin', 'carcassa'],
      base:
        "Lo statore è la parte ferma: un pacco di lamierini d'acciaio con tante barrette di rame piegate a forcina (hairpin). " +
        "Le tre correnti accendono a turno i gruppi di barrette (si illuminano) e creano un campo magnetico che gira tutto intorno, il campo magenta, anche se nulla si muove davvero.",
      expert:
        'Statore a 54 cave con avvolgimento distribuito a barre rettangolari (hairpin) saldate al laser: fattore di riempimento ~70% contro ~45% del filo tondo, quindi meno perdite nel rame e miglior scambio termico. ' +
        'Con p = 3 coppie polari il campo ruota a n = 60·f/p: a 16.000 rpm la frequenza elettrica è 800 Hz. Lamierini sottili (0,2-0,3 mm) limitano le perdite nel ferro alle alte frequenze; raffreddamento a olio o con camicia ad acqua-glicole.',
    },
    {
      id: 'rotore',
      name: 'Rotore a magneti (sincrono)',
      parts: ['rotore', 'magneti', 'albero'],
      base:
        'Dentro il rotore ci sono delle calamite permanenti disposte a V (rosse = polo Nord, blu = polo Sud). Il campo rotante le attira come una calamita che ne trascina un\'altra: ' +
        'il rotore gira esattamente alla stessa velocità del campo, «agganciato». È la differenza con il motore del treno, dove il rotore resta sempre un po\' indietro. ' +
        'La spinta c\'è subito, anche da fermo: per questo le auto elettriche scattano così bene.',
      expert:
        'Macchina sincrona: velocità del rotore = velocità del campo, nessuno scorrimento e quindi nessuna perdita nel rotore (a differenza dell\'asincrono). ' +
        'I magneti NdFeB interni a V generano coppia di allineamento più coppia di riluttanza (L_q > L_d); l\'angolo di carico tra campo e rotore cresce con la coppia. ' +
        'Oltre la velocità base l\'inverter indebolisce il flusso (corrente i_d negativa) per salire di giri a potenza costante. ' +
        'Per ridurre le terre rare (Nd, Dy) si usano anche motori a rotore avvolto (EESM: Renault, BMW) o a induzione (Tesla anteriore, che può girare "a vuoto" senza perdite magnetiche).',
    },
    {
      id: 'riduttore',
      name: 'Riduttore e differenziale',
      parts: ['riduttore', 'cassaRiduttore', 'differenziale', 'semiassi', 'ruota'],
      base:
        "Il motore gira velocissimo (fino a 16.000 giri al minuto) ma la ruota deve girare molto più piano. Due coppie di ingranaggi rallentano la rotazione di circa 9 volte e moltiplicano la forza. " +
        "Non ci sono marce da cambiare: una sola riduzione va bene da 0 alla velocità massima. Il differenziale divide la spinta tra le due ruote e le lascia girare a velocità diverse in curva.",
      expert:
        'Riduttore a due stadi con ingranaggi elicoidali rettificati (qui 17/53 × 19/55, i ≈ 9,0), rendimento ~97-98%. Coppia alla ruota = coppia motore × 9 × η: con 420 Nm si superano 3.600 Nm. ' +
        'Il differenziale a satelliti conici ripartisce la coppia tra i semiassi con giunti omocinetici (tripode lato cambio, Rzeppa lato ruota). ' +
        'In rilascio la catena si inverte: la ruota trascina il motore, che lavora da generatore e frena l\'auto (fino a ~0,2-0,3 g) senza usare i freni a disco.',
    },
  ],
  quiz: [
    {
      q: 'Perché il motore di un\'auto elettrica si chiama «sincrono»?',
      options: [
        'Perché il rotore gira alla stessa velocità del campo magnetico rotante',
        'Perché funziona solo insieme a un motore a benzina',
        'Perché le ruote girano tutte alla stessa velocità',
        'Perché la batteria si ricarica mentre si guida',
      ],
      correct: 0,
      explain:
        'Le calamite del rotore restano «agganciate» al campo creato dallo statore e girano esattamente con lui. Nel motore asincrono (come nel treno) il rotore invece resta un po\' indietro.',
    },
    {
      q: 'A che cosa serve l\'inverter?',
      options: [
        'A cambiare marcia',
        'A raffreddare la batteria',
        'A trasformare la corrente continua della batteria in tre correnti alternate per il motore',
        'A trasformare la corrente alternata di casa in benzina',
      ],
      correct: 2,
      explain:
        'Con interruttori elettronici velocissimi (oggi in carburo di silicio) l\'inverter ricostruisce tre correnti alternate di frequenza variabile: così decide velocità e coppia del motore.',
    },
    {
      q: 'Perché un\'auto elettrica ha una sola marcia?',
      options: [
        'Perché le marce sono vietate sulle auto elettriche',
        'Perché il motore dà coppia già da fermo e può girare fino a 16-20 mila giri',
        'Perché la batteria non sopporterebbe il cambio',
        'Perché il differenziale fa da cambio automatico',
      ],
      correct: 1,
      explain:
        'Il motore elettrico spinge forte da 0 giri e ha un campo di velocità enorme: un unico rapporto fisso (~9:1) basta dall\'avvio alla velocità massima.',
    },
    {
      q: 'Che cosa succede quando alzi il piede dall\'acceleratore?',
      options: [
        'Il motore si scollega dalle ruote e l\'auto va in folle',
        'La batteria si scarica più in fretta',
        'Si accendono i freni a disco automaticamente',
        'Il motore diventa un generatore: frena l\'auto e rimanda energia alla batteria',
      ],
      correct: 3,
      explain:
        'In rigenerazione sono le ruote a far girare il motore: le calamite inducono corrente nello statore, l\'inverter la raddrizza e la batteria si ricarica. Si recupera una buona parte dell\'energia della frenata.',
    },
  ],
  camera: { position: [-4.0, 2.18, 5.59], target: [0.0, -0.22, -0.3] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.copy(OFFSET);
    root.add(model);
    const add = (obj, opts) => {
      const w = ctx.part(obj, opts);
      model.add(w);
      return w;
    };

    // ================= Battery pack =================
    const BX = [-2.75, -1.1];
    const BY = [-0.6, -0.27];
    const BZ = [-1.5, 0.75];
    const bcx = (BX[0] + BX[1]) / 2;
    const bcz = (BZ[0] + BZ[1]) / 2;
    const pack = new THREE.Group();
    const packMat = cut(MAT.paint(0x3b4049, 0.55));
    const PW = BX[1] - BX[0];
    const PD = BZ[1] - BZ[0];
    pack.add(shellBox(PW, BY[1] - BY[0], PD, 0.025, 0.07, packMat, bcx, (BY[0] + BY[1]) / 2, bcz));
    // perimeter flange (sill mounting) with bolts, gasket line under the lid
    const flange = rrect(PW + 0.12, PD + 0.12, 0.1, bcx, -bcz);
    flange.holes.push(rrect(PW - 0.01, PD - 0.01, 0.065, bcx, -bcz, THREE.Path));
    pack.add(new THREE.Mesh(extrudeY(flange, 0.03, BY[0] + 0.04), packMat));
    const gasket = rrect(PW + 0.006, PD + 0.006, 0.072, bcx, -bcz);
    gasket.holes.push(rrect(PW - 0.004, PD - 0.004, 0.068, bcx, -bcz, THREE.Path));
    pack.add(new THREE.Mesh(extrudeY(gasket, 0.012, BY[1] - 0.04), cut(MAT.rubber())));
    const fb = [];
    for (let i = 0; i < 9; i++) {
      const z = BZ[0] + 0.05 + (i * (PD - 0.1)) / 8;
      fb.push([BX[0] - 0.035, BY[0] + 0.077, z, 'y'], [BX[1] + 0.035, BY[0] + 0.077, z, 'y']);
    }
    for (let i = 0; i < 7; i++) {
      const x = BX[0] + 0.05 + (i * (PW - 0.1)) / 6;
      fb.push([x, BY[0] + 0.077, BZ[0] - 0.035, 'y'], [x, BY[0] + 0.077, BZ[1] + 0.035, 'y']);
    }
    pack.add(bolts(fb, cut(MAT.darkSteel(), { cap: false }), 0.016, 0.016));
    for (let i = 0; i < 5; i++) pack.add(new THREE.Mesh(new RoundedBoxGeometry(PW - 0.16, 0.02, 0.06, 2, 0.008), packMat).translateX(bcx).translateY(BY[1] + 0.008).translateZ(BZ[0] + 0.25 + i * 0.4));
    const warn = plate(['ALTA TENSIONE', '400 V c.c.'], 0.36, 0.16);
    warn.rotation.x = -Math.PI / 2;
    warn.position.set(bcx - 0.3, BY[1] + 0.002, -0.65);
    pack.add(warn);
    // coolant ports on the end wall
    const portGeo = alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.08, 14));
    for (const z of [-1.2, -1.0]) pack.add(new THREE.Mesh(portGeo, MAT.paint(0x2a5d9a, 0.5)).translateX(BX[0] - 0.04).translateY(-0.44).translateZ(z));
    add(pack, {
      id: 'pacco',
      name: 'Pacco batteria (involucro)',
      base: "La cassa piatta e robusta che contiene le celle. Sta sotto il pavimento dell'auto: abbassa il baricentro e protegge le celle dagli urti.",
      expert:
        'Involucro in alluminio estruso o acciaio, stagno (IP67), parte della struttura della scocca. Contiene moduli, piastra di raffreddamento a liquido, BMS e scatola di disconnessione; pesa in tutto 400-500 kg per 70-80 kWh.',
      explode: [-0.3, -0.7, 0],
    });

    const cellsG = new THREE.Group();
    const cellBase = new THREE.CylinderGeometry(0.04, 0.04, 0.22, 10);
    const cellGeos = [];
    const stripGeos = [];
    for (let j = 0; j < 23; j++) {
      if (j === 11) continue; // module wall
      const z = BZ[0] + 0.1 + j * 0.09;
      for (let i = 0; i < 17; i++) cellGeos.push(cellBase.clone().translate(BX[0] + 0.1 + i * 0.09, -0.44, z));
      stripGeos.push(new THREE.BoxGeometry(1.5, 0.006, 0.05).translate(bcx, -0.327, z));
    }
    cellsG.add(new THREE.Mesh(mergeGeometries(cellGeos), MAT.paint(0x2c5d8f, 0.4)));
    cellsG.add(new THREE.Mesh(mergeGeometries(stripGeos), MAT.aluminum()));
    cellsG.add(box(1.55, 0.02, 2.1, MAT.aluminum(), bcx, -0.57, bcz - 0.04)); // cooling plate
    cellsG.add(box(1.55, 0.24, 0.03, MAT.paint(0x1f2328, 0.6), bcx, -0.44, BZ[0] + 0.1 + 11 * 0.09));
    cellsG.add(box(0.03, 0.24, 2.05, MAT.paint(0x1f2328, 0.6), BX[1] - 0.04, -0.44, bcz - 0.04));
    add(cellsG, {
      id: 'celle',
      name: 'Celle e moduli',
      base:
        'Centinaia di celle, simili a grosse pile stilo, sono collegate tra loro da lamelle metalliche. Ogni cella dà poco più di 3,6 volt: mettendone tante in serie si arriva a 400 volt.',
      expert:
        'Celle Li-ion cilindriche 2170 (~3,6-3,7 V, ~5 Ah) o 4680, oppure prismatiche LFP. In serie per la tensione (~96s a 400 V), in parallelo per la capacità. ' +
        'Il BMS bilancia le celle e stima stato di carica (SoC) e salute (SoH); la piastra fredda sotto i moduli scambia calore con il circuito acqua-glicole.',
      explode: [-0.3, 0.15, 0.25],
    });

    // ================= HV cables + disconnect unit =================
    const hv = new THREE.Group();
    const orange = MAT.paint(0xf07a1a, 0.5);
    // Disconnect unit at the back corner: cables rise behind the motor end, not in front of it.
    hv.add(box(0.26, 0.1, 0.24, MAT.paint(0x24282e, 0.5), -1.3, -0.22, -0.95));
    const cableCurves = [-1, 1].map((s) => {
      const z0 = -0.95 + s * 0.05;
      return new THREE.CatmullRomCurve3(
        [
          [-1.3, -0.18, z0], [-1.33, 0.15, z0 + 0.05], [-1.28, 0.9, z0 + 0.15],
          [-1.2, 1.45, -0.45 + s * 0.05], [-1.1, 1.62, -0.3 + s * 0.05], [-0.98, 1.62, -0.3 + s * 0.05],
        ].map((p) => new THREE.Vector3(...p)),
        false,
        'centripetal',
      );
    });
    for (const c of cableCurves) hv.add(new THREE.Mesh(new THREE.TubeGeometry(c, 48, 0.024, 8, false), orange));
    add(hv, {
      id: 'caviHV',
      name: 'Cavi ad alta tensione e contattori',
      base: "I cavi arancioni portano l'energia dalla batteria all'inverter. Il colore arancione avvisa: alta tensione, non toccare! La scatolina sul pacco li stacca a vettura spenta.",
      expert:
        'Cavi schermati da 35-70 mm² (in rame o alluminio), arancioni per norma. La scatola di disconnessione (BDU) contiene contattori principali, circuito di precarica del DC-link e fusibile; ' +
        'correnti tipiche 300-600 A a 400 V. A 800 V, a pari potenza la corrente si dimezza: cavi più leggeri e ricarica più veloce.',
      explode: [-0.3, 0.25, 0.45],
    });

    // ================= Inverter =================
    const INV = { x: -0.35, y: 1.68, w: 1.4, h: 0.36, d: 0.84 };
    const invCase = new THREE.Group();
    const invMat = cut(MAT.paint(0x9aa3ad, 0.45));
    invCase.add(shellBox(INV.w, INV.h, INV.d, 0.025, 0.05, invMat, INV.x, INV.y, 0));
    // lid flange + gasket line + bolts
    const lidRing = rrect(INV.w + 0.04, INV.d + 0.04, 0.07);
    lidRing.holes.push(rrect(INV.w - 0.04, INV.d - 0.04, 0.03, 0, 0, THREE.Path));
    invCase.add(new THREE.Mesh(extrudeY(lidRing, 0.025, INV.y + INV.h / 2 - 0.07), invMat).translateX(INV.x));
    const gk = rrect(INV.w + 0.046, INV.d + 0.046, 0.07);
    gk.holes.push(rrect(INV.w + 0.02, INV.d + 0.02, 0.06, 0, 0, THREE.Path));
    invCase.add(new THREE.Mesh(extrudeY(gk, 0.006, INV.y + INV.h / 2 - 0.045), cut(MAT.rubber())).translateX(INV.x));
    const ib = [];
    for (let i = 0; i < 8; i++) {
      const x = INV.x - INV.w / 2 + 0.06 + (i * (INV.w - 0.12)) / 7;
      ib.push([x, INV.y + INV.h / 2 - 0.035, INV.d / 2 + 0.008, 'y'], [x, INV.y + INV.h / 2 - 0.035, -INV.d / 2 - 0.008, 'y']);
    }
    for (const z of [-0.25, 0, 0.25]) ib.push([INV.x - INV.w / 2 - 0.008, INV.y + INV.h / 2 - 0.035, z, 'y'], [INV.x + INV.w / 2 + 0.008, INV.y + INV.h / 2 - 0.035, z, 'y']);
    invCase.add(bolts(ib, cut(MAT.chrome(), { cap: false }), 0.013, 0.02));
    for (let i = 0; i < 8; i++) invCase.add(new THREE.Mesh(new RoundedBoxGeometry(0.03, 0.03, INV.d - 0.1, 2, 0.01), invMat).translateX(INV.x - 0.6 + i * 0.17).translateY(INV.y + INV.h / 2 + 0.012));
    // HV DC connector (cables enter here) and low-voltage signal connector
    invCase.add(new THREE.Mesh(new RoundedBoxGeometry(0.1, 0.16, 0.24, 2, 0.02), MAT.paint(0xf07a1a, 0.5)).translateX(INV.x - INV.w / 2 - 0.04).translateY(1.62).translateZ(-0.3));
    invCase.add(new THREE.Mesh(new RoundedBoxGeometry(0.08, 0.07, 0.14, 2, 0.015), MAT.paint(0x1c1f24, 0.6)).translateX(INV.x + INV.w / 2 + 0.035).translateY(1.75).translateZ(0.15));
    // coolant pipes with hoses down to the motor water jacket
    const pipeGeo = alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.12, 12));
    const hoseMat = MAT.rubber();
    const clampGeo = new THREE.TorusGeometry(0.031, 0.007, 6, 16);
    [[-0.25, -0.05], [-0.1, -0.3]].forEach(([z, xEnd]) => {
      invCase.add(new THREE.Mesh(pipeGeo, MAT.aluminum()).translateX(INV.x + INV.w / 2 + 0.06).translateY(1.58).translateZ(z));
      const c = new THREE.CatmullRomCurve3(
        [[0.47, 1.58, z], [0.56, 1.52, z], [0.5, 1.38, -0.4], [xEnd + 0.15, 1.36, -0.46], [xEnd, 1.33, -0.46]].map((p) => new THREE.Vector3(...p)),
      );
      invCase.add(new THREE.Mesh(new THREE.TubeGeometry(c, 30, 0.027, 8), hoseMat));
      for (const t of [0.02, 0.98]) {
        const cl = new THREE.Mesh(clampGeo, MAT.chrome());
        cl.position.copy(c.getPointAt(t));
        cl.lookAt(cl.position.clone().add(c.getTangentAt(t)));
        invCase.add(cl);
      }
    });
    add(invCase, {
      id: 'inverter',
      name: 'Inverter (involucro)',
      base: "La scatola sopra il motore che contiene l'elettronica di potenza. È raffreddata a liquido: i due tubi portano il refrigerante.",
      expert:
        'Inverter integrato nell\'unità di trazione (architettura "3 in 1": motore, inverter, riduttore), che elimina i cavi trifase esterni. Involucro in alluminio pressofuso con piastra fredda a liquido e schermatura EMC.',
      explode: [0, 1.25, 0],
    });

    const mods = new THREE.Group();
    mods.add(box(1.3, 0.03, 0.76, cut(MAT.aluminum()), INV.x, 1.54, 0)); // cold plate
    const MOD_X = [-0.85, -0.57, -0.29];
    const switches = [];
    for (const mx of MOD_X) {
      mods.add(box(0.24, 0.05, 0.26, MAT.paint(0x23272d, 0.5), mx, 1.575, 0.2));
      const leg = [];
      for (const z of [0.13, 0.27]) {
        const m = box(0.16, 0.014, 0.09, glowMat(MAT.paint(0x30343a, 0.4)), mx, 1.605, z);
        mods.add(m);
        leg.push(m);
      }
      switches.push(leg);
      mods.add(box(0.06, 0.02, 0.05, MAT.copper(), mx, 1.6, 0.35));
    }
    mods.add(box(1.25, 0.012, 0.3, MAT.paint(0x1d6b3a, 0.6), INV.x, 1.8, -0.2)); // gate-driver board
    add(mods, {
      id: 'moduliSiC',
      name: 'Moduli di potenza SiC',
      base:
        'Tre moduli neri, uno per fase, ciascuno con due interruttori elettronici: uno collega la fase al «più», l\'altro al «meno» della batteria. Si accendono e spengono a turno (li vedi illuminarsi) migliaia di volte al secondo.',
      expert:
        'Semiponti a MOSFET in carburo di silicio (SiC) sinterizzati su substrato ceramico e piastra fredda. Il SiC sopporta temperature di giunzione elevate (~175 °C) e commuta molto velocemente: ' +
        'perdite dimezzate rispetto agli IGBT al silicio, +3-5% di autonomia. La scheda verde è il gate driver, che pilota gli interruttori con isolamento galvanico.',
      explode: [0, 0.75, 0.45],
    });

    const capG = new THREE.Group();
    capG.add(box(1.0, 0.2, 0.28, MAT.paint(0xb8952f, 0.45), -0.5, 1.64, -0.21));
    capG.add(box(0.95, 0.008, 0.14, MAT.copper(), -0.55, 1.61, 0.03)); // laminated DC busbar
    add(capG, {
      id: 'condensatore',
      name: 'Condensatore del DC-link',
      base: 'Un «serbatoio» di energia elettrica sempre pronto: assorbe i colpi delle commutazioni velocissime e tiene stabile la tensione della batteria vista dall\'inverter.',
      expert:
        'Condensatore a film di polipropilene, ~400-800 µF a 450-900 V, collegato con sbarre laminate a bassa induttanza per limitare le sovratensioni di commutazione dei MOSFET.',
      explode: [0, 0.9, -0.35],
    });

    // ================= Phase busbars (inverter -> stator) =================
    const bars = new THREE.Group();
    const busCurves = [];
    const busMeshes = [];
    for (let k = 0; k < 3; k++) {
      const a = 0.25 + 0.14 * k;
      const ye = YM + 0.47 * Math.cos(a);
      const ze = 0.47 * Math.sin(a);
      const zr = 0.38 - 0.05 * k;
      const c = new THREE.CatmullRomCurve3(
        [
          [MOD_X[k], 1.6, 0.36], [MOD_X[k], 1.5, zr], [MOD_X[k] - 0.06, 1.465, zr],
          [-0.94, 1.465, zr], [-0.96, ye + 0.06, ze + 0.04], [-0.96, ye, ze],
        ].map((p) => new THREE.Vector3(...p)),
        false,
        'centripetal',
      );
      const m = new THREE.Mesh(new THREE.TubeGeometry(c, 40, 0.014, 6, false), glowMat(MAT.copper()));
      bars.add(m);
      busCurves.push(c);
      busMeshes.push(m);
    }
    add(bars, {
      id: 'sbarre',
      name: 'Sbarre trifase U-V-W',
      base: 'Tre sbarre di rame portano le tre correnti alternate dall\'inverter allo statore. Ognuna si illumina quando la sua corrente è più forte.',
      expert:
        'Terna trifase sfasata di 120° elettrici, con correnti di picco fino a 600-900 A. Nelle unità integrate le sbarre sono corte e rigide: meno induttanza parassita e meno peso rispetto ai cavi.',
      explode: [0, 0.5, 0.35],
    });

    // ================= Motor housing + end shield =================
    const housing = new THREE.Group();
    housing.position.set(XM, YM, 0);
    const hMat = cut(MAT.paint(0x3d454f, 0.5));
    // double wall with the water jacket in between (visible in section)
    housing.add(new THREE.Mesh(alongX(pipe(0.62, 0.597, 1.03, 72)), hMat));
    housing.add(new THREE.Mesh(alongX(pipe(0.578, 0.56, 1.03, 72)), hMat));
    for (const e of [-1, 1]) housing.add(new THREE.Mesh(alongX(pipe(0.62, 0.56, 0.03, 72)), hMat).translateX(e * 0.53));
    const ribGeo = alongX(pipe(0.645, 0.62, 0.03, 72));
    for (let i = 0; i < 6; i++) housing.add(new THREE.Mesh(ribGeo, hMat).translateX(-0.42 + i * 0.17));
    const shieldMat = cut(MAT.aluminum());
    const shieldProf = [[0.07, -0.545], [0.6, -0.545], [0.635, -0.56], [0.635, -0.595], [0.3, -0.6], [0.16, -0.62], [0.1, -0.62], [0.07, -0.6], [0.07, -0.545]];
    housing.add(new THREE.Mesh(alongX(lathe(shieldProf, 72)), shieldMat));
    for (let i = 0; i < 8; i++) {
      const a = (i / 8) * TAU + 0.2;
      const rib = box(0.03, 0.3, 0.025, shieldMat, -0.6, Math.cos(a) * 0.32, Math.sin(a) * 0.32);
      rib.rotation.x = a;
      housing.add(rib);
    }
    const capProf = [[0.0, -0.665], [0.115, -0.665], [0.13, -0.65], [0.13, -0.615], [0.12, -0.615], [0.12, -0.645], [0.11, -0.655], [0.0, -0.655]];
    housing.add(new THREE.Mesh(alongX(lathe([...capProf, capProf[0]], 40)), shieldMat));
    housing.add(bolts(boltCircleX(-0.6, 0, 0.6, 14, 0.1), cut(MAT.chrome(), { cap: false }), 0.014, 0.02));
    const np = plate(['MOTORILAB  EDU', '220 kW · 400 V', 'max 16.000 rpm'], 0.24, 0.13, '#c9cdd2', '#1c1f24');
    np.rotation.y = -Math.PI / 2;
    np.position.set(-0.61, 0.3, -0.3);
    housing.add(np);
    // coolant nipples (hoses from the inverter) and rubber-bushed mounting arms
    const nipGeo = new THREE.CylinderGeometry(0.03, 0.035, 0.07, 14);
    for (const lx of [0.45, 0.2]) {
      const n = new THREE.Mesh(nipGeo, MAT.aluminum());
      n.position.set(lx, Math.cos(-0.82) * 0.65, Math.sin(-0.82) * 0.65);
      n.rotation.x = -0.82;
      housing.add(n);
    }
    for (const sx of [-0.3, 0.3]) housing.add(new THREE.Mesh(new RoundedBoxGeometry(0.16, 0.14, 0.32, 2, 0.03), hMat).translateX(sx).translateY(-0.6).translateZ(-0.27));
    add(housing, {
      id: 'carcassa',
      name: 'Carcassa del motore',
      base: 'Il guscio di alluminio che tiene insieme statore e cuscinetti. Nelle sue pareti scorre il liquido di raffreddamento.',
      expert:
        'Carcassa pressofusa in alluminio con camicia di raffreddamento (acqua-glicole) o spruzzatori d\'olio diretti sulle teste delle hairpin. Il calore da smaltire è piccolo (~5% della potenza) ma concentrato.',
      explode: [-0.2, 0.3, -1.15],
    });

    // ================= Stator core + slot conductors =================
    const stator = new THREE.Group();
    stator.position.set(XM, YM, 0);
    stator.add(new THREE.Mesh(alongX(lathe(ribbedProfile(R_SI, R_SO, LS, 48, 0.005, 0), 72)), cut(MAT.darkSteel())));
    const slotGeos = [];
    const q = new THREE.Quaternion();
    const mtx = new THREE.Matrix4();
    const one = new THREE.Vector3(1, 1, 1);
    const pos = new THREE.Vector3();
    const eul = new THREE.Euler();
    for (let s = 0; s < SLOTS; s++) {
      const a = ((s + 0.5) / SLOTS) * TAU;
      pos.set(0, Math.cos(a) * 0.43, Math.sin(a) * 0.43);
      mtx.compose(pos, q.setFromEuler(eul.set(a, 0, 0)), one);
      slotGeos.push(new THREE.BoxGeometry(LS + 0.02, 0.08, 0.022).applyMatrix4(mtx));
    }
    stator.add(new THREE.Mesh(mergeGeometries(slotGeos), cut(MAT.copper())));
    add(stator, {
      id: 'statore',
      name: 'Statore (pacco lamellare)',
      base: "La parte ferma del motore: tanti dischi sottili d'acciaio impilati, con 54 scanalature (cave) dove passano le barrette di rame.",
      expert:
        'Lamierini di acciaio al silicio a grani non orientati (0,2-0,3 mm) isolati tra loro, per contenere isteresi e correnti parassite a frequenze fino a ~1 kHz. 54 cave, 6 poli, 3 cave per polo e per fase.',
      explode: [0, 0, 0],
    });

    // ================= Hairpin end windings (6 phase belts, glowing) =================
    const hair = new THREE.Group();
    hair.position.set(XM, YM, 0);
    const belts = [[], [], [], [], [], []];
    const barGeo = new THREE.BoxGeometry(0.13, 0.028, 0.017);
    const weldGeo = new THREE.BoxGeometry(0.03, 0.05, 0.02);
    for (let s = 0; s < SLOTS; s++) {
      const a = ((s + 0.5) / SLOTS) * TAU;
      const cls = Math.floor(s / 3) % 6;
      for (const e of [-1, 1]) {
        const x = e * (LS / 2 + 0.06);
        for (const [r, lean] of [[0.405, 0.55], [0.45, -0.55]]) {
          pos.set(x, Math.cos(a) * r, Math.sin(a) * r);
          mtx.compose(pos, q.setFromEuler(eul.set(a, lean * e, 0)), one);
          belts[cls].push(barGeo.clone().applyMatrix4(mtx));
        }
        if (e > 0) {
          pos.set(x + 0.075, Math.cos(a) * 0.427, Math.sin(a) * 0.427);
          mtx.compose(pos, q.setFromEuler(eul.set(a, 0, 0)), one);
          belts[cls].push(weldGeo.clone().applyMatrix4(mtx));
        }
      }
    }
    const beltMeshes = belts.map((list, c) => {
      const m = new THREE.Mesh(mergeGeometries(list), glowMat(MAT.copper()));
      hair.add(m);
      return { mesh: m, e: ((60 * c + 30) * Math.PI) / 180 }; // electrical position of the belt
    });
    add(hair, {
      id: 'hairpin',
      name: 'Avvolgimento a forcine (hairpin)',
      base:
        'Invece del classico filo avvolto, qui ci sono barrette di rame rettangolari piegate a forcina, infilate nelle cave e saldate tutte insieme. Si accendono a gruppi seguendo le tre correnti.',
      expert:
        'Conduttori rettangolari (~2×4 mm) inseriti assialmente e saldati al laser sul lato connessioni: riempimento della cava ~70%, resistenza e perdite I²R minori, teste di bobina corte e ben raffreddate. ' +
        'Limite: effetto pelle e correnti di prossimità ad alta frequenza, per cui si usano 4-8 strati di barre più sottili.',
      explode: [0, 0, 0.35],
    });

    // ================= Rotor: laminations with V pockets + magnets =================
    const rotor = new THREE.Group();
    rotor.position.set(XM, YM, 0);
    const lam = new THREE.Shape();
    lam.absarc(0, 0, R_ROT, 0, TAU, false);
    lam.holes.push(circlePath(0, 0, 0.062));
    for (const m of vMagnets(0.04, 0.008)) lam.holes.push(polyShape(m.pts, THREE.Path));
    const rotSpin = new THREE.Group();
    rotSpin.add(new THREE.Mesh(extrudeX(lam, LS, 48), cut(MAT.steel())));
    const endRingGeo = alongX(pipe(0.17, 0.062, 0.025, 40));
    for (const e of [-1, 1]) {
      const r = new THREE.Mesh(endRingGeo, MAT.brass());
      r.position.x = e * (LS / 2 + 0.013);
      rotSpin.add(r);
    }
    rotor.add(rotSpin);
    const wRotor = add(rotor, {
      id: 'rotore',
      name: 'Rotore (pacco lamellare)',
      base: 'La parte che gira. È fatta anch\'essa di lamierini d\'acciaio, con delle tasche a forma di V dove sono infilate le calamite.',
      expert:
        'Pacco rotorico con tasche a V e barriere di flusso: la geometria crea anisotropia magnetica (L_q > L_d) che aggiunge coppia di riluttanza. Ponticelli sottili (~1 mm) reggono la forza centrifuga a 16-20.000 rpm; spesso il rotore è diviso in segmenti sfalsati (skew) per ridurre ondulazione di coppia e rumore.',
      explode: [-1.3, 0.1, 0.5],
    });

    const magnets = new THREE.Group();
    magnets.position.set(XM, YM, 0);
    const magSpin = new THREE.Group();
    const mags = vMagnets(0.032, 0);
    const shapesN = mags.filter((m) => m.k % 2 === 0).map((m) => polyShape(m.pts));
    const shapesS = mags.filter((m) => m.k % 2 === 1).map((m) => polyShape(m.pts));
    magSpin.add(new THREE.Mesh(extrudeX(shapesN, LS - 0.004), MAT.paint(0xd2362f, 0.35)));
    magSpin.add(new THREE.Mesh(extrudeX(shapesS, LS - 0.004), MAT.paint(0x2f62d0, 0.35)));
    magnets.add(magSpin);
    const wMag = add(magnets, {
      id: 'magneti',
      name: 'Magneti permanenti (N rosso, S blu)',
      base:
        'Calamite potentissime al neodimio, due per ogni polo, a forma di V. Le rosse sono poli Nord, le blu poli Sud: in tutto 6 poli che vengono trascinati dal campo rotante.',
      expert:
        'Magneti sinterizzati NdFeB (rimanenza ~1,2-1,3 T), con disprosio/terbio per resistere alla smagnetizzazione ad alta temperatura; ~1-2 kg per motore. ' +
        'La disposizione a V concentra il flusso e protegge i magneti dalla centrifuga. Le terre rare sono il punto critico per costo e approvvigionamento: da qui i motori EESM (rotore avvolto, Renault/BMW) e a induzione.',
      explode: [-1.3, 0.1, 1.05],
    });

    // ================= Alternative rotor: wound field (EESM) =================
    const eesm = new THREE.Group();
    eesm.position.set(XM, YM, 0);
    const eesmSpin = new THREE.Group();
    const poleShape = eesmPoleShape();
    poleShape.holes.push(circlePath(0, 0, 0.062));
    eesmSpin.add(new THREE.Mesh(extrudeX(poleShape, LS, 12), cut(MAT.steel())));
    const fcGeos = [];
    for (let k = 0; k < PP * 2; k++) {
      const phi = (TAU * k) / (PP * 2);
      const place = (g, x, rad, tan) => {
        pos.set(x, rad * Math.cos(phi) - tan * Math.sin(phi), rad * Math.sin(phi) + tan * Math.cos(phi));
        mtx.compose(pos, q.setFromEuler(eul.set(phi, 0, 0)), one);
        fcGeos.push(g.applyMatrix4(mtx));
      };
      for (const j of [-1, 1]) place(new THREE.BoxGeometry(LS + 0.02, 0.13, 0.034), 0, 0.215, j * 0.078);
      for (const e of [-1, 1]) place(new THREE.BoxGeometry(0.035, 0.13, 0.19), e * (LS / 2 + 0.018), 0.215, 0);
    }
    const fieldCoils = new THREE.Mesh(mergeGeometries(fcGeos), glowMat(MAT.copper()));
    eesmSpin.add(fieldCoils);
    const slipGeo = alongX(pipe(0.068, 0.046, 0.02, 28));
    for (const x of [-0.47, -0.5]) eesmSpin.add(new THREE.Mesh(slipGeo, MAT.copper()).translateX(x));
    eesm.add(eesmSpin);
    for (const x of [-0.47, -0.5]) eesm.add(box(0.016, 0.05, 0.03, MAT.carbon(), x, 0.095, 0)); // brushes
    eesm.add(box(0.06, 0.04, 0.06, MAT.paint(0x2a2e34, 0.5), -0.485, 0.14, 0)); // brush holder
    eesm.add(rod([-0.485, 0.16, 0], [-0.535, 0.3, 0], 0.012, MAT.paint(0x2a2e34, 0.5)));
    const wEesm = add(eesm, {
      id: 'rotoreAvvolto',
      name: 'Rotore avvolto (EESM)',
      base: 'Un rotore senza calamite: ogni polo ha una bobina di rame. La corrente arriva da due anelli che girano sotto due spazzole di grafite; regolandola si regola la «forza» del magnete.',
      expert:
        'Rotore a poli salienti con avvolgimento di campo in c.c. (eccitazione separata) alimentato da anelli e spazzole. Il flusso regolabile permette di lavorare ad alta velocità senza deflussaggio forzato e di azzerarlo a vuoto; nessuna terra rara. Esempi: Renault (Zoe, Mégane E-Tech), BMW eDrive di 5ª generazione.',
      explode: [-1.3, 0.1, 0.5],
    });

    // ================= Alternative rotor: squirrel cage (induction) =================
    const cage = new THREE.Group();
    cage.position.set(XM, YM, 0);
    const cageSpin = new THREE.Group();
    const lamC = new THREE.Shape();
    lamC.absarc(0, 0, R_ROT, 0, TAU, false);
    lamC.holes.push(circlePath(0, 0, 0.062));
    for (let j = 0; j < CAGE_BARS; j++) lamC.holes.push(polyShape(cageSlot(j, 0.024, 0.05), THREE.Path));
    cageSpin.add(new THREE.Mesh(extrudeX(lamC, LS, 48), cut(MAT.steel())));
    const barClasses = [];
    for (let c = 0; c < 12; c++) {
      const shapes = [];
      for (let j = c; j < CAGE_BARS; j += 12) shapes.push(polyShape(cageSlot(j, 0.02, 0.046)));
      const m = new THREE.Mesh(extrudeX(shapes, LS + 0.05, 4), glowMat(MAT.copper()));
      cageSpin.add(m);
      barClasses.push({ mesh: m, e: (PP * c * TAU) / CAGE_BARS });
    }
    const endRing = alongX(pipe(0.352, 0.29, 0.03, 56));
    for (const e of [-1, 1]) cageSpin.add(new THREE.Mesh(endRing, cut(MAT.copper())).translateX(e * (LS / 2 + 0.025)));
    cage.add(cageSpin);
    const wCage = add(cage, {
      id: 'gabbia',
      name: 'Rotore a gabbia (induzione)',
      base: 'Barre di rame chiuse da due anelli: nessuna calamita e nessun filo collegato. Il campo che gira induce corrente nelle barre (si illuminano) e trascina il rotore, che resta un po’ indietro.',
      expert:
        'Gabbia in rame pressofuso su pacco lamellare; lo scorrimento (1-3% a carico, esagerato nell’animazione) genera le correnti rotoriche a frequenza s·f. Senza magneti non ci sono perdite a vuoto quando l’inverter lo spegne: per questo è usato come motore secondario (Tesla anteriore).',
      explode: [-1.3, 0.1, 0.5],
    });

    // ================= Shaft, bearings, resolver =================
    const shaft = new THREE.Group();
    shaft.position.set(0, YM, 0);
    const shaftSpin = new THREE.Group();
    shaftSpin.add(new THREE.Mesh(alongX(lathe([[0, -1.15], [0.045, -1.15], [0.045, -0.95], [0.06, -0.94], [0.06, -0.06], [0.045, -0.05], [0.045, 0.27], [0, 0.27]], 24)), MAT.steel()));
    const resRot = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.075, 0.075, 0.03, 6)), MAT.darkSteel());
    resRot.position.x = -1.12;
    shaftSpin.add(resRot);
    shaft.add(shaftSpin);
    const brgGeo = alongX(pipe(0.095, 0.045, 0.04, 32));
    for (const x of [-1.0, 0.0]) {
      const b = new THREE.Mesh(brgGeo, MAT.chrome());
      b.position.x = x;
      shaft.add(b);
    }
    const resStat = new THREE.Mesh(alongX(pipe(0.11, 0.08, 0.04, 32)), MAT.copper());
    resStat.position.x = -1.12;
    shaft.add(resStat);
    add(shaft, {
      id: 'albero',
      name: 'Albero, cuscinetti e resolver',
      base: "L'asse d'acciaio che porta il rotore. In fondo c'è un sensore (resolver) che dice all'inverter, istante per istante, dove si trovano le calamite.",
      expert:
        'Albero in acciaio cavo (spesso con olio di raffreddamento all\'interno), cuscinetti a sfere ad alta velocità con isolamento contro le correnti d\'albero. ' +
        'Il resolver misura l\'angolo del rotore con precisione di frazioni di grado: indispensabile per il controllo sincrono (FOC), che deve conoscere la posizione dei poli.',
      explode: [-1.3, 0.1, 0.5],
    });

    // ================= Reduction gears =================
    const red = new THREE.Group();
    const pin1 = new THREE.Mesh(gearGeo(Z1, M1, 0.07, 0.044), MAT.steel());
    pin1.position.set(X_ST1, YM, 0);
    const gear2 = new THREE.Mesh(gearGeo(Z2, M1, 0.07, 0.04, 5, 0.05, 0.19), MAT.darkSteel());
    gear2.position.set(X_ST1, YI, 0);
    const pin3 = new THREE.Mesh(gearGeo(Z3, M2, 0.09, 0.04), MAT.steel());
    pin3.position.set(X_ST2, YI, 0);
    const midShaft = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.04, 0.04, 0.46, 20)), MAT.steel());
    midShaft.position.set(0.33, YI, 0);
    red.add(pin1, gear2, pin3, midShaft);
    add(red, {
      id: 'riduttore',
      name: 'Riduttore a due stadi',
      base:
        'Il pignone piccolo sull\'albero del motore fa girare una ruota dentata grande; sullo stesso asse un altro pignone fa girare la corona del differenziale. In tutto la rotazione rallenta di circa 9 volte.',
      expert:
        'Due stadi a denti elicoidali: 17/53 (i₁ ≈ 3,12) e 19/55 (i₂ ≈ 2,89), i totale ≈ 9,0. Dentature rettificate e microgeometria ottimizzata contro il fischio, che senza il rumore di un motore termico si sentirebbe; lubrificazione a sbattimento o forzata.',
      explode: [0.35, 0, 0.55],
    });

    // Gearbox casing: hollow hull of the three gear circles (walls + end plates with bearing bores).
    const hull = (dr, n = 40) => {
      const pts = [];
      for (const [y, r] of [[YM, 0.16], [YI, 0.36], [YD, 0.4]]) {
        for (let i = 0; i < n; i++) {
          const a = (i / n) * TAU;
          pts.push([Math.cos(a) * (r + dr), y + Math.sin(a) * (r + dr)]);
        }
      }
      return convexHull(pts);
    };
    const caseMat = cut(MAT.aluminum());
    caseMat.color.set(0xb4bac1);
    caseMat.roughness = 0.6;
    const gcase = new THREE.Group();
    const wall = polyShape(hull(0));
    wall.holes.push(polyShape(hull(-0.035).reverse(), THREE.Path));
    const wallGeo = new THREE.ExtrudeGeometry(wall, { depth: 0.54, bevelEnabled: false });
    wallGeo.rotateY(Math.PI / 2);
    gcase.add(new THREE.Mesh(wallGeo, caseMat).translateX(0.085));
    const bores = [[YM, 0.05], [YI, 0.045], [YD, 0.1]];
    for (const [x0, bl] of [[0.05, [[YM, 0.05], [YI, 0.045], [YD, 0.035]]], [0.625, bores]]) {
      const pl = polyShape(hull(0));
      for (const [y, r] of bl) pl.holes.push(circlePath(0, y, r));
      const g = new THREE.ExtrudeGeometry(pl, { depth: 0.035, bevelEnabled: false, curveSegments: 16 });
      g.rotateY(Math.PI / 2);
      gcase.add(new THREE.Mesh(g, caseMat).translateX(x0));
      for (const [y, r] of bl) gcase.add(new THREE.Mesh(alongX(pipe(r + 0.028, r, 0.035, 28)), cut(MAT.chrome())).translateX(x0 + 0.0175).translateY(y));
    }
    // bolted flange towards the motor, gasket line, drain plug, breather
    const fl = polyShape(hull(0.035));
    fl.holes.push(polyShape(hull(0).reverse(), THREE.Path));
    const flGeo = new THREE.ExtrudeGeometry(fl, { depth: 0.035, bevelEnabled: false });
    flGeo.rotateY(Math.PI / 2);
    gcase.add(new THREE.Mesh(flGeo, caseMat).translateX(0.05));
    const gkHull = polyShape(hull(0.006));
    gkHull.holes.push(polyShape(hull(-0.035).reverse(), THREE.Path));
    const gkGeo = new THREE.ExtrudeGeometry(gkHull, { depth: 0.006, bevelEnabled: false });
    gkGeo.rotateY(Math.PI / 2);
    gcase.add(new THREE.Mesh(gkGeo, cut(MAT.rubber())).translateX(0.3));
    const hb = hull(0.018, 18).filter((_, i) => i % 3 === 0).filter(([u, v]) => Math.hypot(u, v - YM) > 0.64).map(([u, v]) => [0.045, v, -u, 'x']);
    gcase.add(bolts(hb, cut(MAT.chrome(), { cap: false }), 0.013, 0.02));
    gcase.add(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.03, 6), MAT.darkSteel()).translateX(0.4).translateY(YD - 0.405));
    gcase.add(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.025, 0.06, 12), MAT.paint(0x1c1f24, 0.6)).translateX(0.4).translateY(YM + 0.18).translateZ(-0.06));
    add(gcase, {
      id: 'cassaRiduttore',
      name: 'Scatola del riduttore',
      base: "La scatola di alluminio che contiene ingranaggi e differenziale, immersi nell'olio.",
      expert:
        'Carter pressofuso in alluminio, spesso in comune con il motore e con circuito d\'olio condiviso (olio ATF a bassa viscosità che lubrifica gli ingranaggi e raffredda le hairpin).',
      explode: [0.35, 0.1, -1.05],
    });

    // ================= Differential =================
    const diff = new THREE.Group();
    diff.position.set(X_ST2, YD, 0);
    const diffSpin = new THREE.Group();
    const ring4 = new THREE.Mesh(gearGeo(Z4, M2, 0.09, 0.2, 8, 0.022, 0.26), MAT.darkSteel());
    diffSpin.add(ring4);
    const caseProf = [
      [0.05, -0.2], [0.085, -0.2], [0.09, -0.15], [0.155, -0.09], [0.175, -0.04], [0.205, -0.04],
      [0.205, 0.0], [0.175, 0.0], [0.17, 0.05], [0.15, 0.1], [0.09, 0.16], [0.085, 0.2], [0.05, 0.2],
    ];
    diffSpin.add(new THREE.Mesh(alongX(lathe(caseProf, 40)), cut(MAT.castIron())));
    const spMat = MAT.steel();
    diffSpin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.27, 10), spMat));
    for (const s of [-1, 1]) {
      const pg = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.032, 0.05, 14), spMat);
      pg.position.y = s * 0.085;
      if (s < 0) pg.rotation.x = Math.PI;
      diffSpin.add(pg);
      const sg = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.07, 0.04, 0.05, 16)), spMat);
      sg.position.x = s * 0.075;
      sg.rotation.z = s > 0 ? Math.PI : 0;
      diffSpin.add(sg);
    }
    diff.add(diffSpin);
    add(diff, {
      id: 'differenziale',
      name: 'Differenziale',
      base:
        'La corona grande è fissata a una scatola che contiene quattro piccoli ingranaggi conici. In rettilineo girano tutti insieme; in curva lasciano girare la ruota esterna più veloce di quella interna.',
      expert:
        'Differenziale aperto a satelliti e planetari conici, corona da 55 denti. Ripartisce la coppia 50/50; il controllo di trazione frena la ruota che slitta, e con l\'elettrico la coppia si regola in pochi millisecondi direttamente dal motore.',
      explode: [0.6, -0.25, 0.45],
    });

    // ================= Half-shafts =================
    const shafts = new THREE.Group();
    shafts.position.y = YD;
    const hsSpin = new THREE.Group();
    const steel = MAT.steel();
    const rubber = MAT.rubber();
    const cupGeo = (len, r) => alongX(lathe([[0.03, -len / 2], [r * 0.8, -len / 2], [r, -len / 4], [r, len / 4], [r * 0.6, len / 2], [0.03, len / 2]], 24));
    const add1 = (geo, mat, x) => {
      const m = new THREE.Mesh(geo, mat);
      m.position.x = x;
      hsSpin.add(m);
    };
    // right side (to the wheel)
    add1(cupGeo(0.14, 0.1), steel, 0.75);
    hsSpin.add(new THREE.Mesh(bellows(0.82, 1.0, 0.045, 0.085), rubber));
    add1(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.8, 14)), steel, 1.24);
    hsSpin.add(new THREE.Mesh(bellows(1.5, 1.66, 0.045, 0.085), rubber));
    add1(cupGeo(0.12, 0.11), steel, 1.72);
    // left side (towards the other wheel)
    add1(cupGeo(0.14, 0.1), steel, -0.01);
    hsSpin.add(new THREE.Mesh(bellows(-0.28, -0.08, 0.045, 0.085), rubber));
    add1(alongX(new THREE.CylinderGeometry(0.03, 0.03, 1.0, 14)), steel, -0.73);
    hsSpin.add(new THREE.Mesh(bellows(-1.32, -1.18, 0.045, 0.085), rubber));
    add1(cupGeo(0.12, 0.11), steel, -1.38);
    add1(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.12, 12)), steel, 0.65);
    add1(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.24, 12)), steel, 0.13);
    shafts.add(hsSpin);
    add(shafts, {
      id: 'semiassi',
      name: 'Semiassi e giunti omocinetici',
      base: 'Due alberi portano la rotazione dal differenziale alle ruote. Le cuffie di gomma proteggono i giunti snodati, che permettono alla ruota di molleggiare mentre gira.',
      expert:
        'Semiassi in acciaio con giunto tripode scorrevole lato differenziale e giunto Rzeppa a sfere lato ruota: trasmettono velocità costante anche con angoli di lavoro di 10-20° (sospensione e sterzo).',
      explode: [0.5, -0.35, 0.2],
    });

    // ================= Wheel =================
    const wheel = new THREE.Group();
    wheel.position.set(WHEEL_X, YD, 0);
    const wSpin = new THREE.Group();
    const tyreMat = cut(MAT.rubber());
    const tyreProf = [
      [0.5, -0.17], [0.56, -0.2], [0.66, -0.205], [0.72, -0.19], [0.745, -0.15], [TYRE_R, -0.1],
      [TYRE_R, 0.1], [0.745, 0.15], [0.72, 0.19], [0.66, 0.205], [0.56, 0.2], [0.5, 0.17],
    ];
    wSpin.add(new THREE.Mesh(alongX(lathe(tyreProf, 72)), tyreMat));
    const treadGeos = [];
    for (let i = 0; i < 40; i++) {
      const a = (i / 40) * TAU;
      for (const xo of [-0.06, 0.06]) {
        pos.set(xo + (i % 2 ? 0.02 : -0.02), Math.cos(a) * (TYRE_R + 0.008), Math.sin(a) * (TYRE_R + 0.008));
        mtx.compose(pos, q.setFromEuler(eul.set(a, 0.35, 0)), one);
        treadGeos.push(new THREE.BoxGeometry(0.1, 0.02, 0.06).applyMatrix4(mtx));
      }
    }
    wSpin.add(new THREE.Mesh(mergeGeometries(treadGeos), tyreMat));
    const rimMat = MAT.aluminum();
    wSpin.add(new THREE.Mesh(alongX(pipe(0.5, 0.47, 0.36, 64)), rimMat));
    const hub = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.12, 0.12, 0.08, 24)), rimMat);
    hub.position.x = 0.12;
    wSpin.add(hub);
    const bearingHub = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.075, 0.075, 0.14, 20)), MAT.darkSteel());
    bearingHub.position.x = -0.13;
    wSpin.add(bearingHub);
    for (let i = 0; i < 5; i++) {
      const a = (i / 5) * TAU;
      const sk = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.37, 0.075), rimMat);
      sk.position.set(0.12, Math.cos(a) * 0.29, Math.sin(a) * 0.29);
      sk.rotation.x = a;
      wSpin.add(sk);
      const nut = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.014, 0.014, 0.03, 6)), MAT.chrome());
      nut.position.set(0.17, Math.cos(a + 0.6) * 0.075, Math.sin(a + 0.6) * 0.075);
      wSpin.add(nut);
    }
    const disc = new THREE.Mesh(alongX(pipe(0.38, 0.17, 0.032, 64)), MAT.darkSteel());
    disc.position.x = -0.1;
    wSpin.add(disc);
    const hat = new THREE.Mesh(alongX(pipe(0.17, 0.12, 0.12, 32)), MAT.darkSteel());
    hat.position.x = -0.04;
    wSpin.add(hat);
    wheel.add(wSpin);
    const knMat = MAT.castIron();
    wheel.add(new THREE.Mesh(alongX(pipe(0.13, 0.08, 0.1, 32)), knMat).translateX(-0.17));
    wheel.add(new THREE.Mesh(new RoundedBoxGeometry(0.06, 0.72, 0.09, 2, 0.02), knMat).translateX(-0.18).translateY(-0.03).translateZ(-0.16));
    wheel.add(new THREE.Mesh(new THREE.SphereGeometry(0.04, 12, 10), MAT.steel()).translateX(-0.18).translateY(-0.38).translateZ(-0.15));
    wheel.add(box(0.1, 0.05, 0.12, knMat, -0.14, 0.25, -0.16));
    const caliper = box(0.12, 0.22, 0.12, MAT.paint(0xb3261e, 0.4), -0.1, 0.25, -0.22);
    caliper.rotation.x = -0.72;
    wheel.add(caliper);
    add(wheel, {
      id: 'ruota',
      name: 'Ruota, pneumatico e freno',
      base:
        'La ruota motrice. Nota che in frenata normale i freni a disco lavorano poco: buona parte del rallentamento lo fa il motore elettrico, recuperando energia.',
      expert:
        'Pneumatico 235/45 R18 (raggio dinamico ~0,34 m). Con i ≈ 9 il motore a 16.000 rpm porta la ruota a ~1.770 rpm, cioè ~225 km/h. ' +
        'La frenata è "blended": la centralina usa prima la rigenerazione e aggiunge l\'idraulica solo quando serve, per cui le pastiglie durano molto di più.',
      explode: [0.9, 0, 0],
    });

    // ================= Subframe, mounts, lower arm (ties EDU, pack and wheel together) =================
    const sub = new THREE.Group();
    const subMat = MAT.paint(0x2b3038, 0.55);
    const rb = (w, h, d, x, y, z, mat = subMat) => sub.add(new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 2, 0.02), mat).translateX(x).translateY(y).translateZ(z));
    rb(1.32, 0.1, 0.12, -0.6, 0.1, -0.33);       // crossmember under the motor
    rb(0.1, 0.42, 0.12, -1.2, -0.06, -0.33);     // bracket bolted to the pack
    rb(0.18, 0.03, 0.18, -1.2, -0.255, -0.33);   // foot
    rb(0.1, 0.57, 0.12, -0.03, -0.135, -0.33);   // drop bracket for the lower arm
    const bush = new THREE.CylinderGeometry(0.045, 0.045, 0.06, 16);
    for (const x of [XM - 0.3, XM + 0.3]) sub.add(new THREE.Mesh(bush, MAT.rubber()).translateX(x).translateY(0.18).translateZ(-0.3));
    sub.add(bolts([[-1.26, -0.235, -0.27, 'y'], [-1.14, -0.235, -0.27, 'y'], [-1.26, -0.235, -0.39, 'y'], [-1.14, -0.235, -0.39, 'y']], MAT.chrome(), 0.014, 0.016));
    sub.add(rod([0.02, -0.4, -0.31], [WHEEL_X - 0.18, -0.38, -0.15], 0.03, subMat));
    sub.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.05, 0.05, 0.08, 16)), MAT.rubber()).translateX(0.04).translateY(-0.4).translateZ(-0.31));
    add(sub, {
      id: 'telaio',
      name: 'Telaio ausiliario e braccio',
      base: "La struttura d'acciaio che regge il gruppo motore con dei tamponi di gomma (per non trasmettere vibrazioni) ed è imbullonata alla scocca e al pacco batteria. Il braccio inferiore tiene la ruota.",
      expert:
        'Culla (subframe) in acciaio con supporti elastici del gruppo propulsore; il braccio oscillante inferiore e il montante formano la sospensione posteriore. In un’auto vera la culla è fissata alla scocca: qui è collegata direttamente al bordo del pacco per chiarezza.',
      explode: [0, -0.5, -0.4],
    });

    // ================= Rotating field overlay (not a part) =================
    const field = new THREE.Group();
    field.position.set(XM, YM, 0);
    model.add(field);
    const fieldMat = (opacity) =>
      new THREE.MeshBasicMaterial({
        color: FIELD,
        transparent: true,
        opacity,
        blending: THREE.AdditiveBlending,
        depthWrite: false, // depth-tested: only visible where the section opens the motor
        side: THREE.DoubleSide,
      });
    const lobeMat = fieldMat(0.35);
    const arrowMat = fieldMat(0.85);
    const LOBE = 0.55;
    const lobeGeos = [];
    const arrowGeos = [];
    for (let k = 0; k < PP * 2; k++) {
      const a = (k * TAU) / (PP * 2);
      lobeGeos.push(alongX(new THREE.CylinderGeometry(0.368, 0.368, LS - 0.1, 10, 1, true, a - Math.PI / 2 - LOBE / 2, LOBE)));
      // arrows on the open end: N (even) outward, S (odd) inward
      const out = k % 2 === 0;
      const stem = new THREE.CylinderGeometry(0.008, 0.008, 0.1, 6);
      const tip = new THREE.ConeGeometry(0.025, 0.05, 10).translate(0, 0.075, 0);
      for (const g of [stem, tip]) {
        if (!out) g.rotateX(Math.PI);
        g.translate(0, 0.25, 0).rotateX(a).translate(-LS / 2 - 0.16, 0, 0);
        arrowGeos.push(g);
      }
    }
    const lobes = new THREE.Mesh(mergeGeometries(lobeGeos), lobeMat);
    const arrows = new THREE.Mesh(mergeGeometries(arrowGeos), arrowMat);
    lobes.renderOrder = 30;
    arrows.renderOrder = 31;
    field.add(lobes, arrows);

    // ================= Current flows =================
    const shifted = (fn) => (t, out) => fn(t, out).add(OFFSET);
    // over the visible (front) cells, back to the disconnect unit, up the + cable, into the DC busbar
    const dcPts = [[-2.6, -0.31, 0.45], [-1.4, -0.31, 0.45], [-1.4, -0.31, -0.9], [-1.3, -0.2, -0.92]];
    for (let i = 0; i <= 24; i++) dcPts.push(cableCurves[1].getPointAt(i / 24).add(new THREE.Vector3(-0.02, 0, 0.03)).toArray());
    dcPts.push([-0.9, 1.625, 0.03], [-0.25, 1.625, 0.03]);
    const dcBase = polyPath(dcPts);
    let reverse = false;
    const dc = flow({
      path: shifted((t, out) => dcBase(reverse ? 1 - t : t, out)),
      count: 360,
      size: 0.05,
      colors: [0xffffff],
      spread: 0.018,
      speed: 0.12,
      opacity: 0.9,
    });
    root.add(dc.object);
    ctx.addFlow(dc);

    const lift = new THREE.Vector3(0, 0.012, 0.018);
    const acFlows = busCurves.map((c) => {
      const f = flow({
        path: shifted((t, out) => c.getPointAt(reverse ? 1 - t : t, out).add(lift)),
        count: 110,
        size: 0.04,
        colors: [0xffffff],
        spread: 0.008,
        speed: 0.5,
        opacity: 0.9,
      });
      root.add(f.object);
      ctx.addFlow(f);
      return f;
    });

    // ================= Animation =================
    let prevCycle = null;
    let rev = 0; // unwrapped field revolutions (from state.cycle)
    let slip = 0; // induction rotor: accumulated lag behind the field (revolutions)
    let motorType = 'pm';
    const setOption = (id, value) => {
      if (id !== 'motore') return;
      motorType = value;
      wRotor.visible = wMag.visible = value === 'pm';
      wEesm.visible = value === 'eesm';
      wCage.visible = value === 'asm';
    };
    setOption('motore', 'pm');
    const cycleRate = (thr) => (thr < REGEN ? 0.5 : 0.12 + thr * 1.3);

    return {
      root,
      cycleRate,
      setOption,
      update(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const time = state.time || 0;
        const regen = thr < REGEN;
        const c = state.cycle || 0;
        if (prevCycle === null) prevCycle = c;
        let d = c - prevCycle;
        if (d < -0.5) d += 1;
        else if (d > 0.5) d -= 1;
        rev += d;
        prevCycle = c;
        if (motorType === 'asm') slip += d * (regen ? -0.06 : 0.04 + 0.05 * thr); // exaggerated slip

        // Synchronous: rotor locked to the field, offset by the load angle
        // (field ahead when motoring, behind when generating).
        const thetaF = rev * TAU;
        const delta = (regen ? -0.3 : 0.15 + 0.4 * thr) / PP;
        // Synchronous types are locked to the field; the induction rotor slips behind it.
        const thetaR = motorType === 'asm' ? thetaF - slip * TAU : thetaF - delta;
        field.rotation.x = thetaF;
        rotSpin.rotation.x = thetaR;
        magSpin.rotation.x = thetaR;
        eesmSpin.rotation.x = thetaR;
        cageSpin.rotation.x = thetaR;
        shaftSpin.rotation.x = thetaR;

        // Gear train (upper gear: PI + angle, lower gear: -PI/Z + angle).
        const thI = (-thetaR * Z1) / Z2;
        const thD = (-thI * Z3) / Z4;
        pin1.rotation.x = Math.PI + thetaR;
        gear2.rotation.x = -Math.PI / Z2 + thI;
        pin3.rotation.x = Math.PI + thI;
        midShaft.rotation.x = thI;
        diffSpin.rotation.x = -Math.PI / Z4 + thD;
        hsSpin.rotation.x = thD;
        wSpin.rotation.x = thD;

        // Currents: belts glow with the electrical field position.
        const act = regen ? 0.75 : 0.3 + 0.7 * thr;
        const col = regen ? REGEN_C : CURRENT;
        const eAng = thetaF * PP;
        for (const b of beltMeshes) {
          const v = Math.cos(b.e - eAng);
          b.mesh.material.emissive.setHex(col);
          b.mesh.material.emissiveIntensity = act * 1.1 * v * v;
        }
        const pwm = Math.sin(time * 90) > -0.4 ? 1 : 0.3;
        for (let k = 0; k < 3; k++) {
          const i = Math.cos(eAng - (k * TAU) / 3);
          for (const sw of switches[k]) sw.material.emissive.setHex(col);
          switches[k][0].material.emissiveIntensity = act * 1.6 * Math.max(0, i) * pwm;
          switches[k][1].material.emissiveIntensity = act * 1.6 * Math.max(0, -i) * pwm;
          busMeshes[k].material.emissive.setHex(col);
          busMeshes[k].material.emissiveIntensity = act * 0.8 * Math.abs(i);
          acFlows[k].material.opacity = (0.15 + 0.8 * Math.abs(i)) * (0.45 + 0.55 * act);
        }
        fieldCoils.material.emissive.setHex(CURRENT);
        fieldCoils.material.emissiveIntensity = 0.12 + 0.2 * act; // DC field current
        for (const b of barClasses) {
          const v = Math.cos(b.e + PP * thetaR - eAng);
          b.mesh.material.emissive.setHex(col);
          b.mesh.material.emissiveIntensity = (0.15 + 0.85 * act) * v * v;
        }
        lobeMat.opacity = 0.05 + 0.13 * act; // faint, so the red/blue magnets stay readable
        arrowMat.opacity = 0.4 + 0.5 * act;

        // Flows: forward (yellow-green) when driving, reversed green when regenerating.
        reverse = regen;
        const dt = state.dt || 0;
        const rate = regen ? 1.2 : 0.25 + thr * 1.75;
        dc.material.color.setHex(col);
        dc.material.opacity = regen ? 0.85 : 0.35 + 0.55 * thr;
        dc.update(dt, rate);
        for (const f of acFlows) {
          f.material.color.setHex(col);
          f.update(dt, rate * 1.2);
        }
        const assembled = (state.explode || 0) < 0.05;
        field.visible = dc.object.visible = assembled;
        for (const f of acFlows) f.object.visible = assembled;
      },
      readout(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const kmhPerRpm = (WHEEL_C * 60) / 1000 / RATIO;
        if (thr < REGEN) {
          const rpm = REGEN_KMH / kmhPerRpm;
          return [
            { label: 'Giri motore', value: fmt(rpm), unit: 'rpm' },
            { label: 'Rigenerazione', value: '−45', unit: 'kW' },
            { label: 'Velocità', value: String(REGEN_KMH), unit: 'km/h' },
            { label: 'Frequenza statore', value: fmt((rpm / 60) * PP), unit: 'Hz' },
          ];
        }
        const rpm = thr * MAX_RPM;
        return [
          { label: 'Giri motore', value: fmt(rpm), unit: 'rpm' },
          { label: 'Potenza', value: fmt(thr * P_MAX), unit: 'kW' },
          { label: 'Velocità', value: fmt(rpm * kmhPerRpm), unit: 'km/h' },
          { label: 'Frequenza statore', value: fmt((rpm / 60) * PP * (motorType === 'asm' ? 1.02 : 1)), unit: 'Hz' },
        ];
      },
    };
  },
};
