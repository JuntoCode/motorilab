// Twin-rotor Wankel (Mazda 13B style: side intake ports in the intermediate housing, peripheral exhaust).
// Cycle 0..1 = one turn of the front rotor = three eccentric-shaft turns; phases follow chamber 1 of the front rotor.
// The front rotor sits on the section plane (z = 0); the rear rotor (z = Z2) is phased 180° on the shaft.
// Rotor geometry lives in a "local" housing frame (major axis along x, port waist at +y),
// turned by FRAME so the major axis is vertical and the ports face +x in the world.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, pipe, flow, clamp, lerp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const E = 0.2; // eccentricity
const RG = 1.4; // generating radius, R/e = 7
const HALF = 0.4; // half width of a rotor housing (z)
const RHALF = 0.35; // half width of a rotor
const OUTER = RG + 0.5; // outer contour of the housings
const Z2 = -1.05; // rear rotor centre
const FRONT = [HALF, HALF + 0.2]; // front side housing
const INTER = [Z2 + HALF, -HALF]; // intermediate housing (-0.65 .. -0.4)
const REAR = [Z2 - HALF - 0.2, Z2 - HALF]; // rear side housing
const INTER_MID = (INTER[0] + INTER[1]) / 2;
const FRAME = -Math.PI / 2; // local +y -> world +x
const EX_WIN = [0.86, 1.2]; // peripheral exhaust port window (housing parameter)
const IN_PORT = { r: 1.08, a: 2.06, w: 0.16, h: 0.09 }; // side intake port (local polar)
const PLUG_A = [-Math.PI / 2 + 0.2, -Math.PI / 2 - 0.22]; // leading, trailing
const BINS = 360; // rotor profile resolution
const NCH = 40; // samples per chamber edge

const wrap = (c) => ((c % 1) + 1) % 1;
// Epitrochoid: x = e·cos3a + R·cos a, y = e·sin3a + R·sin a
const troch = (a, R = RG) => [E * Math.cos(3 * a) + R * Math.cos(a), E * Math.sin(3 * a) + R * Math.sin(a)];
const toWorld = (x, y, z = 0) => new THREE.Vector3(y, -x, z);
const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const angDist = (a, b) => Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
const framed = (...objs) => {
  const g = new THREE.Group();
  g.rotation.z = FRAME;
  g.add(...objs);
  return g;
};
const rbox = (w, h, d, r = 0.02) => new RoundedBoxGeometry(w, h, d, 2, r);
const cylZ = (r, len, seg = 32) => new THREE.CylinderGeometry(r, r, len, seg).rotateX(Math.PI / 2);
const cylX = (r, len, seg = 32) => new THREE.CylinderGeometry(r, r, len, seg).rotateZ(Math.PI / 2);
const tube = (pts, r, mat, seg = 48, closed = false, radial = 12) =>
  new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, closed), seg, r, radial, closed), mat);

// Tension bolts and coolant passages (housing parameter angles), away from ports and plugs.
const BOLT_A = [];
for (let k = 0; k < 16; k++) {
  const a = (k / 16) * TAU + 0.2;
  if (angDist(a, (EX_WIN[0] + EX_WIN[1]) / 2) < 0.4 || PLUG_A.some((q) => angDist(a, q) < 0.3) || angDist(a, IN_PORT.a) < 0.35) continue;
  BOLT_A.push(a);
}
const COOL_A = [];
for (let k = 0; k < 14; k++) {
  const a = (k / 14) * TAU + 0.1;
  if (angDist(a, (EX_WIN[0] + EX_WIN[1]) / 2) < 0.45 || PLUG_A.some((q) => angDist(a, q) < 0.3)) continue;
  COOL_A.push(a);
}

// Rotor flank = inner envelope of the housing seen from the rotor frame (sampled min radius),
// with a little clearance and a combustion recess in the middle of each flank.
function rotorProfile() {
  const NA = 720;
  const NP = 720;
  const hx = new Float64Array(NA);
  const hy = new Float64Array(NA);
  for (let i = 0; i < NA; i++) [hx[i], hy[i]] = troch((i / NA) * TAU);
  const rmin = new Float64Array(BINS).fill(Infinity);
  for (let j = 0; j < NP; j++) {
    const phi = (j / NP) * TAU;
    const c = Math.cos(phi / 3);
    const s = Math.sin(phi / 3);
    const cx = E * Math.cos(phi);
    const cy = E * Math.sin(phi);
    for (let i = 0; i < NA; i++) {
      const dx = hx[i] - cx;
      const dy = hy[i] - cy;
      const qx = dx * c + dy * s;
      const qy = -dx * s + dy * c;
      let k = Math.round((Math.atan2(qy, qx) / TAU) * BINS) % BINS;
      if (k < 0) k += BINS;
      const r = Math.hypot(qx, qy);
      if (r < rmin[k]) rmin[k] = r;
    }
  }
  const third = BINS / 3;
  const prof = new Float64Array(BINS);
  for (let k = 0; k < BINS; k++) {
    let r = Math.min(rmin[k], rmin[(k + third) % BINS], rmin[(k + 2 * third) % BINS]) * 0.975;
    const b = (k / BINS) * TAU;
    // distance to the nearest flank middle (60°, 180°, 300°)
    const d = (((b - Math.PI / 3) % (TAU / 3)) + TAU / 3) % (TAU / 3);
    const dm = Math.min(d, TAU / 3 - d);
    if (dm < 0.32) r -= 0.06 * (0.5 + 0.5 * Math.cos((Math.PI * dm) / 0.32));
    prof[k] = r;
  }
  return prof;
}

// Gear outline: soft square-wave teeth around a pitch circle.
function toothed(rPitch, teeth, amp, phase, n) {
  const pts = [];
  const k = Math.tanh(3);
  for (let i = 0; i < n; i++) {
    const a = (i / n) * TAU;
    const r = rPitch + (amp * Math.tanh(3 * Math.cos(teeth * (a - phase)))) / k;
    pts.push(new THREE.Vector2(r * Math.cos(a), r * Math.sin(a)));
  }
  return pts;
}
const circle = (r, cx = 0, cy = 0) => {
  const p = new THREE.Path();
  p.absellipse(cx, cy, r, r, 0, TAU, true);
  return p;
};
const extrude = (shape, z0, z1, segs = 48) => {
  const g = new THREE.ExtrudeGeometry(shape, { depth: z1 - z0, bevelEnabled: false, curveSegments: segs });
  g.translate(0, 0, z0);
  return g;
};
// Cast plate with rounded (bevelled) edges.
const extrudeBevel = (shape, z0, z1, b = 0.02) => {
  const g = new THREE.ExtrudeGeometry(shape, {
    depth: z1 - z0 - 2 * b,
    bevelEnabled: true,
    bevelThickness: b,
    bevelSize: b,
    bevelSegments: 2,
    curveSegments: 24,
  });
  g.translate(0, 0, z0 + b);
  return g;
};
// Filled circular sector (counterweights), extruded along z.
function sector(r, center, half, z0, z1) {
  const s = new THREE.Shape();
  s.moveTo(0, 0);
  s.absarc(0, 0, r, center - half, center + half, false);
  s.lineTo(0, 0);
  return extrude(s, z0, z1, 24);
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
function instanced(geo, mat, items) {
  const m = new THREE.InstancedMesh(geo, mat, items.length);
  const q = new THREE.Quaternion();
  const one = v3(1, 1, 1);
  const mx = new THREE.Matrix4();
  items.forEach((it, i) => {
    q.setFromUnitVectors(v3(0, 1, 0), it.n.clone().normalize());
    m.setMatrixAt(i, mx.compose(it.p, q, one));
  });
  m.instanceMatrix.needsUpdate = true;
  return m;
}
// Hex head + shank of length `len` below it (local -Y), so a bolt backing out of its hole shows the shank.
const screw = (len, r, h) =>
  mergeGeometries([new THREE.CylinderGeometry(r, r, h, 6), new THREE.CylinderGeometry(r * 0.6, r * 0.6, len, 10).translate(0, -h / 2 - len / 2, 0)]);
// Replace every material under obj with a cut (no cap) clone, so it disappears in the section view.
function cutAll(obj) {
  obj.traverse((o) => {
    if (o.isMesh) o.material = cut(o.material.clone(), { cap: false });
  });
}
function nameplate() {
  const c = document.createElement('canvas');
  c.width = 320;
  c.height = 96;
  const g = c.getContext('2d');
  g.fillStyle = '#c9ccd0';
  g.fillRect(0, 0, 320, 96);
  g.strokeStyle = '#3a3f46';
  g.lineWidth = 4;
  g.strokeRect(4, 4, 312, 88);
  g.fillStyle = '#1d2229';
  g.font = 'bold 28px sans-serif';
  g.fillText('ROTATIVO 2 ROTORI', 20, 40);
  g.font = '20px sans-serif';
  g.fillText('2 × 654 cm³ · luci laterali', 20, 72);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return new THREE.MeshStandardMaterial({ map: tex, metalness: 0.5, roughness: 0.45 });
}

// Crescent-shaped gas volume between housing and rotor flank: fixed topology, positions per frame.
function chamberGeometry() {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(NCH * 4 * 3), 3));
  const idx = [];
  const quad = (a, b, c, d) => idx.push(a, b, c, b, d, c);
  const H0 = 0;
  const F0 = NCH;
  const H1 = 2 * NCH;
  const F1 = 3 * NCH;
  for (let k = 0; k < NCH - 1; k++) {
    quad(H0 + k, F0 + k, H0 + k + 1, F0 + k + 1);
    quad(H1 + k, F1 + k, H1 + k + 1, F1 + k + 1);
    quad(H0 + k, H1 + k, H0 + k + 1, H1 + k + 1);
    quad(F0 + k, F1 + k, F0 + k + 1, F1 + k + 1);
  }
  g.setIndex(idx);
  return g;
}

export default {
  id: 'wankel',
  name: 'Motore rotativo Wankel',
  short: 'Rotativo Wankel',
  family: 'terra',
  vehicle: 'Auto sportive e range extender',
  examples: 'Mazda RX-7 e RX-8, Mazda MX-30 R-EV (come generatore), NSU Ro 80, Mazda 787B',
  tagline: 'Niente pistoni che vanno su e giù: rotori triangolari girano in camere a forma di ovale strozzato.',
  intro: {
    base:
      'Al posto dei pistoni ci sono <b>rotori triangolari</b> che girano dentro camere a forma di ovale strozzato. Le tre punte di ogni rotore strisciano sempre sulla parete e dividono lo spazio in <b>tre camere</b> che, girando, si allargano e si stringono: così ognuna aspira, comprime, brucia e scarica, senza valvole e senza parti che vanno avanti e indietro. Questo modello ha <b>due rotori</b> in fila, come il Mazda 13B: in sezione vedi quello anteriore.',
    expert:
      'Motore a pistone rotante (Felix Wankel, NSU, 1957). Lo statore ha profilo <b>epitrocoide</b> a due lobi (qui R/e = 7); il rotore, con i fianchi ricavati dall’inviluppo interno della curva, è montato sull’eccentrico dell’albero e ingrana con un pignone fisso tramite una dentatura interna in rapporto 3:2, quindi <b>l’albero gira tre volte più veloce del rotore</b>. Ogni camera compie un ciclo Otto completo per giro di rotore: ogni rotore dà una fase utile a ogni giro d’albero. Birotore con eccentrici a 180° (13B): due fasi utili per giro, come un 4 cilindri 4 tempi. Distribuzione a luci, senza valvole: aspirazione laterale nella piastra intermedia, scarico periferico.',
  },
  specs: [
    { label: 'Carburante', value: 'Benzina (prove a idrogeno)' },
    { label: 'Ciclo', value: 'Otto, 4 fasi in camere rotanti' },
    { label: 'Cilindrata', value: '2 × 654 cm³ (13B)' },
    { label: 'Potenza tipica', value: '141–170 kW (RX-8)' },
    { label: 'Distribuzione', value: 'Nessuna valvola: luci laterali e periferiche' },
    { label: 'Regime massimo', value: '≈ 9 000 giri/min' },
    { label: 'Rapporto di compressione', value: '9:1 – 12:1' },
    { label: 'Rendimento', value: '20 – 28 %' },
  ],
  metrics: { efficiency: 25, power: 170, powerDensity: 1.4 },
  variants: {
    columns: ['Motore', 'Rotori × cilindrata', 'Potenza', 'Dove'],
    rows: [
      ['NSU KKM 612', '2 × 497,5 cm³', '85 kW (115 CV)', 'NSU Ro 80 (1967–1977)'],
      ['Mazda 13B-MSP Renesis', '2 × 654 cm³', '141–170 kW (192–231 CV)', 'Mazda RX-8 (2003–2012), luci tutte laterali'],
      ['Mazda R26B', '4 × 654 cm³', '≈ 515 kW (700 CV)', 'Mazda 787B, vincitrice a Le Mans 1991'],
      ['Mazda 8C', '1 × 830 cm³', '55 kW (75 CV)', 'MX-30 R-EV, solo generatore (dal 2023)'],
    ],
    note: 'La cilindrata di un rotore è il volume spostato da una delle sue tre camere.',
  },
  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      range: [0, 0.25],
      parts: ['intakeManifold', 'rotor', 'intermediateHousing'],
      base: 'La camera passa davanti al buco di aspirazione e si allarga: come un mantice che si apre, risucchia aria e benzina. Non ci sono valvole: è il fianco del rotore che scopre e copre il buco.',
      expert:
        'Luce di aspirazione laterale ricavata nella piastra intermedia, alimentata da un condotto interno: si apre quando il fianco del rotore la scopre, subito dopo il punto morto di incrocio. Il volume della camera passa dal minimo al massimo in 270° di albero (90° di rotore). Le luci laterali danno poco incrocio con lo scarico e un minimo regolare; le luci periferiche (motori da corsa come il R26B della 787B) danno più potenza ma un minimo irregolare.',
    },
    {
      id: 'compression',
      name: 'Compressione',
      range: [0.25, 0.5],
      parts: ['rotor', 'rotorHousing', 'apexSeals'],
      base: 'Il rotore continua a girare e la camera si restringe contro la “strozzatura” della pista: la miscela viene schiacciata, proprio come in un motore a pistoni.',
      expert:
        'Rapporto volumetrico 9–10:1 nei 13B (Renesis 10,0:1, 8C della MX-30 11,9:1); questo modello arriva a circa 9,2:1. Al punto morto superiore la camera è lunga e sottile: alto rapporto superficie/volume, quindi molte perdite di calore verso le pareti e fiamma lenta. La tenuta è affidata ai segmenti d’apice, angolari e laterali che strisciano sulla pista cromata.',
    },
    {
      id: 'power',
      name: 'Scoppio ed espansione',
      range: [0.5, 0.75],
      parts: ['sparkPlugs', 'rotor', 'eccentricShaft'],
      base: 'Due candele fanno la scintilla una dopo l’altra: la miscela brucia e il gas caldissimo spinge sul fianco del rotore e lo fa girare. L’eccentrico trasforma la spinta in rotazione dell’albero, senza bielle.',
      expert:
        'Doppia accensione: la candela leading (avanti nel senso di rotazione) scocca per prima, la trailing pochi gradi dopo per bruciare la miscela rimasta nella coda della camera. La pressione agisce sul fianco con braccio pari all’eccentricità e: coppia sull’albero senza forze alterne di biella. La zona di coda resta fredda e mal bruciata, fonte di idrocarburi incombusti.',
    },
    {
      id: 'exhaust',
      name: 'Scarico',
      range: [0.75, 1],
      parts: ['exhaustManifold', 'rotorHousing'],
      base: 'L’apice del rotore scopre il buco di scarico e, mentre la camera si stringe di nuovo, spinge fuori i gas bruciati. Poi la stessa camera torna ad aspirare: un giro di rotore, un ciclo completo per ognuna delle tre camere.',
      expert:
        'Luce di scarico periferica nello statore, come nel 13B; il Renesis dell’RX-8 l’ha spostata sulle piastre laterali per eliminare l’incrocio e ridurre gli HC. L’espansione è breve, quindi i gas escono molto caldi: bene per scaldare il catalizzatore, male per il rendimento.',
    },
  ],
  quiz: [
    {
      q: 'Quante volte gira l’albero eccentrico mentre il rotore fa un giro completo?',
      options: ['Una volta', 'Due volte', 'Tre volte', 'Sei volte'],
      correct: 2,
      explain: 'La dentatura interna del rotore e il pignone fisso stanno nel rapporto 3:2: l’albero fa tre giri per ogni giro del rotore.',
    },
    {
      q: 'Perché il Wankel non ha bisogno di valvole?',
      options: [
        'Perché non brucia carburante',
        'Perché è il rotore stesso a scoprire e coprire le luci di aspirazione e scarico',
        'Perché usa una turbina per aspirare',
        'Perché le camere restano sempre aperte',
      ],
      correct: 1,
      explain: 'Il fianco e gli apici del rotore passano sopra le luci e fanno da “porta”: niente valvole, alberi a camme o molle.',
    },
    {
      q: 'Qual è il punto debole classico del Wankel?',
      options: [
        'Vibra moltissimo',
        'È molto più pesante di un motore a pistoni',
        'Usura dei segmenti d’apice, consumo di olio e rendimento basso',
        'Non supera i 3 000 giri/min',
      ],
      correct: 2,
      explain: 'La camera lunga e sottile disperde calore (rendimento ~25%), i segmenti d’apice si consumano e serve iniettare olio per lubrificarli.',
    },
    {
      q: 'Perché Mazda usa un Wankel nella MX-30 R-EV?',
      options: [
        'Perché è compatto e leggero e, come generatore, lavora a regime costante',
        'Perché è il motore termico più efficiente',
        'Perché non consuma olio',
        'Perché muove direttamente le ruote',
      ],
      correct: 0,
      explain: 'Nella MX-30 R-EV un monorotore da 830 cm³ fa solo da generatore per la batteria: conta più l’ingombro ridotto che il rendimento di picco.',
    },
  ],
  // Officina (see CONTRACT.md): the engine is opened from the front, one slice of the sandwich at a time;
  // the rear side housing stays on the bench as the base. Simplified and educational, not a workshop manual.
  disassembly: {
    intro: 'Apriamo il Wankel da fuori verso dentro: prima gli accessori, poi il “panino” di piastre e statori, una fetta alla volta.',
    note: 'Sequenza semplificata a scopo didattico: in officina si segue il manuale del costruttore, con i suoi attrezzi e le sue coppie di serraggio. Nel 13B vero i tiranti si svitano dal lato del volano e il pacco si apre partendo da dietro; qui lo apriamo da davanti per vederlo meglio.',
    parts: {
      ignitionLeads: { dir: [-1, 0.25, 0], dist: 0.9 },
      sparkPlugs: { after: ['ignitionLeads'], motion: 'unscrew', turns: 6, out: 0.35, dir: [-1, 0, 0], dist: 0.5 },
      intakeManifold: { dir: [1, 0.6, 0], dist: 1.0 },
      exhaustManifold: { dir: [1, -0.4, 0], dist: 1.0 },
      accessoryDrive: { dir: [0, 0.15, 1], dist: 1.0 },
      frontCoverBolts: { after: ['accessoryDrive'], motion: 'unscrew', turns: 6, out: 0.12, dir: [0, 0, 1] },
      frontCover: { after: ['frontCoverBolts', 'accessoryDrive'], dir: [0, 0, 1], dist: 1.0 },
      frontCounterweight: { after: ['frontCover'], dir: [0, 0, 1], dist: 0.8 },
      oilPan: { dir: [0, -1, 0], dist: 0.9 },
      flywheelNut: { motion: 'unscrew', turns: 6, out: 0.15, dir: [0, 0, -1], dist: 0.4 },
      flywheel: { after: ['flywheelNut'], dir: [0, 0, -1], dist: 1.0 },
      tensionBolts: { after: ['accessoryDrive', 'frontCover', 'oilPan'], motion: 'unscrew', turns: 10, out: 0.6, dir: [0, 0, 1], dist: 1.2, duration: 2.4 },
      frontHousing: { after: ['tensionBolts', 'frontCounterweight'], with: ['phasingGears'], dir: [0, 0, 1], dist: 1.2 },
      phasingGears: { after: ['frontHousing'], dir: [0, 0, 1], dist: 0.9 },
      rotor: { after: ['frontHousing'], with: ['apexSeals'], dir: [0, 0, 1], dist: 1.2 },
      apexSeals: { after: ['frontHousing'], dir: [0, 0, 1], dist: 0.9 },
      rotorHousing: { after: ['frontHousing', 'sparkPlugs', 'exhaustManifold'], dir: [0, 0, 1], dist: 1.2 },
      intermediateHousing: { after: ['rotorHousing', 'rotor', 'intakeManifold', 'ignitionLeads'], dir: [0, 0, 1], dist: 1.3, duration: 1.1 },
      eccentricShaft: { after: ['intermediateHousing', 'flywheel'], dir: [0, 0, 1], dist: 1.7, duration: 1.2 },
      rearRotor: { after: ['eccentricShaft'], dir: [0, 0, 1], dist: 1.3 },
      rearRotorHousing: { after: ['intermediateHousing', 'sparkPlugs', 'exhaustManifold'], dir: [0, 0, 1], dist: 1.3 },
      rearHousing: { after: ['rearRotorHousing', 'rearRotor', 'flywheel'] },
    },
    steps: [
      {
        id: 'ignition',
        name: 'Stacca i cavi e svita le candele',
        base: 'Si sfilano i cappucci dei cavi e si tolgono le bobine; poi, con la chiave, si svitano le quattro candele dal fianco degli statori.',
        expert: 'Ogni cavo va segnato: scambiare leading e trailing, o un rotore con l’altro, fa scoccare la scintilla nel momento sbagliato. Leading e trailing hanno spesso gradi termici diversi, quindi anche le candele tornano ognuna nella sua sede.',
        parts: ['ignitionLeads', 'sparkPlugs'],
      },
      {
        id: 'manifolds',
        name: 'Stacca i collettori',
        base: 'Si staccano i due collettori dal fianco del motore: quello di aspirazione dalla piastra intermedia, quello di scarico dai due statori.',
        expert: 'Qui si vede la distribuzione a luci del Wankel: l’aspirazione entra di lato, attraverso la piastra intermedia, e lo scarico esce dalla periferia degli statori. I dadi dello scarico, cotti dal calore, sono spesso i più duri da svitare.',
        parts: ['intakeManifold', 'exhaustManifold'],
      },
      {
        id: 'belt',
        name: 'Togli la cinghia servizi',
        base: 'Si allenta la cinghia e si tolgono alternatore, pompa dell’acqua e la puleggia sulla punta dell’albero: stanno tutti davanti al coperchio.',
        expert: 'Di solito la cinghia si allenta spostando l’alternatore sul suo supporto. La puleggia è fissata sul naso dell’albero eccentrico; scollegati i manicotti, viene via anche la pompa dell’acqua.',
        parts: ['accessoryDrive'],
      },
      {
        id: 'cover',
        name: 'Togli il coperchio anteriore',
        base: 'Si svitano le viti lungo il bordo e si sfila il coperchio. Sotto, infilato sull’albero, c’è il contrappeso anteriore: si toglie anche quello.',
        expert: 'Nel motore vero dietro il coperchio ci sono anche la catena della pompa dell’olio e l’ingranaggio che comanda la pompa dosatrice; qui restano l’albero e il suo contrappeso, che si sfila in avanti.',
        parts: ['frontCoverBolts', 'frontCover', 'frontCounterweight'],
      },
      {
        id: 'pan',
        name: 'Togli la coppa dell’olio',
        base: 'Si scarica l’olio e si stacca la coppa da sotto il motore.',
        expert: 'La coppa è avvitata sotto tutte le fette del pacco: finché è al suo posto le tiene unite, quindi va tolta prima di separare piastre e statori.',
        parts: ['oilPan'],
      },
      {
        id: 'flywheel',
        name: 'Smonta il volano',
        base: 'Dietro al motore si svita il grosso dado centrale e si sfila il volano dall’albero.',
        expert: 'Per svitare il dado bisogna tenere fermo il volano, altrimenti gira tutto l’albero. Il volano è piantato su un cono: di solito serve un estrattore per staccarlo. Al rimontaggio la chiavetta lo riporta nella posizione giusta per l’equilibratura.',
        parts: ['flywheelNut', 'flywheel'],
      },
      {
        id: 'bolts',
        name: 'Svita i tiranti',
        base: 'Ora si svitano i lunghi tiranti che attraversano tutto il motore: sono loro a tenere chiuso il “panino” di piastre e statori.',
        expert: 'Si allentano poco per volta e in ordine incrociato, per non deformare il pacco. Sfilati i tiranti, le fette restano allineate solo dalle spine di centraggio tra una e l’altra.',
        parts: ['tensionBolts'],
      },
      {
        id: 'frontRotor',
        name: 'Apri il primo rotore',
        base: 'Si sfila la piastra anteriore con il suo pignone fisso: ecco il rotore. Poi escono il rotore, con i suoi segmenti, e lo statore.',
        expert: 'I segmenti d’apice, angolari e laterali si tolgono e si conservano in ordine, apice per apice: ognuno si è adattato alla sua sede. Sul fianco del rotore si vedono la dentatura interna e il cuscinetto che lavora sull’eccentrico.',
        parts: ['frontHousing', 'rotor', 'rotorHousing'],
      },
      {
        id: 'shaft',
        name: 'Sfila piastra intermedia e albero',
        base: 'La piastra intermedia si sfila in avanti, poi si tira fuori l’albero eccentrico: il secondo rotore resta appoggiato nel suo statore.',
        expert: 'Passaggio delicato: la piastra intermedia deve scavalcare l’eccentrico anteriore e per farlo si sposta un poco l’albero. Poi l’albero si sfila dal rotore posteriore e dal supporto nella piastra di fondo.',
        parts: ['intermediateHousing', 'eccentricShaft'],
      },
      {
        id: 'rearRotor',
        name: 'Apri il secondo rotore',
        base: 'Restano il rotore e lo statore posteriori: tolti quelli, sul banco c’è solo la piastra posteriore con il suo pignone fisso.',
        expert: 'A motore aperto si controllano l’usura della cromatura sulla pista degli statori, le facce delle piastre su cui strisciano i segmenti laterali e il gioco dei segmenti nelle loro sedi.',
        parts: ['rearRotor', 'rearRotorHousing'],
      },
    ],
  },
  camera: { position: [4.1, 2.02, 7.6], target: [0.15, -0.03, -0.45] },
  thumb: { orbit: [30, 16], zoom: 1.3, view: 'section' },

  build(ctx) {
    const root = new THREE.Group();
    const prof = rotorProfile();
    const rFlank = (b) => {
      const f = wrap(b / TAU) * BINS;
      const i = Math.floor(f);
      const t = f - i;
      return prof[i % BINS] * (1 - t) + prof[(i + 1) % BINS] * t;
    };
    const [p0, p1] = EX_WIN;
    const pm = (p0 + p1) / 2;
    const span = p0 + TAU - p1;
    const boltHoles = () => BOLT_A.map((a) => circle(0.04, ...troch(a, RG + 0.4)));
    const steelMat = MAT.steel();
    const hexGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.04, 6);
    const smallHex = new THREE.CylinderGeometry(0.03, 0.03, 0.03, 6);
    const boltHeadMat = cut(MAT.steel(), { cap: false });

    // ---------- Rotor housings (trochoid), with peripheral exhaust channel ----------
    const housingShape = new THREE.Shape();
    for (let i = 0; i <= 360; i++) {
      const [x, y] = troch(p1 + (span * i) / 360, OUTER);
      i === 0 ? housingShape.moveTo(x, y) : housingShape.lineTo(x, y);
    }
    for (let i = 0; i <= 360; i++) {
      const [x, y] = troch(p0 + TAU - (span * i) / 360, RG + 0.025);
      housingShape.lineTo(x, y);
    }
    for (const a of COOL_A) housingShape.holes.push(circle(0.075, ...troch(a, RG + 0.24)));
    housingShape.holes.push(...boltHoles());
    const housingGeo = extrude(housingShape, -HALF, HALF, 24);
    const liner = new THREE.Shape();
    for (let i = 0; i <= 360; i++) {
      const [x, y] = troch(p1 + (span * i) / 360, RG);
      i === 0 ? liner.moveTo(x, y) : liner.lineTo(x, y);
    }
    for (let i = 0; i <= 360; i++) liner.lineTo(...troch(p0 + TAU - (span * i) / 360, RG + 0.025));
    const linerGeo = extrude(liner, -HALF, HALF, 8);
    const gasketShape = new THREE.Shape();
    for (let i = 0; i <= 240; i++) {
      const [x, y] = troch((i / 240) * TAU, OUTER + 0.006);
      i === 0 ? gasketShape.moveTo(x, y) : gasketShape.lineTo(x, y);
    }
    const gHole = new THREE.Path();
    for (let i = 0; i <= 240; i++) {
      const [x, y] = troch((i / 240) * TAU, OUTER - 0.05);
      i === 0 ? gHole.moveTo(x, y) : gHole.lineTo(x, y);
    }
    gasketShape.holes.push(gHole);
    const gasketGeo = extrude(gasketShape, -0.006, 0.006, 8);
    const gasketMat = cut(MAT.paint(0x24272b, 0.8));
    const housingMat = cut(MAT.aluminum());
    housingMat.color.set(0x9aa3ad);
    housingMat.roughness = 0.55;
    const linerMat = cut(MAT.chrome());
    const makeHousing = (z) => {
      const g = new THREE.Group();
      g.add(new THREE.Mesh(housingGeo, housingMat), new THREE.Mesh(linerGeo, linerMat));
      for (const dz of [-HALF, HALF]) {
        const gk = new THREE.Mesh(gasketGeo, gasketMat);
        gk.position.z = dz;
        g.add(gk);
      }
      g.position.z = z;
      return g;
    };
    root.add(
      ctx.part(framed(makeHousing(0)), {
        id: 'rotorHousing',
        name: 'Statore anteriore (carter del rotore)',
        base: 'La “pista” a forma di ovale strozzato in cui gira il rotore. Nei buchi grandi che vedi nella sezione scorre il liquido di raffreddamento, in quelli piccoli passano i lunghi bulloni che tengono insieme il motore.',
        expert:
          'Profilo epitrocoide x = e·cos3θ + R·cosθ, y = e·sin3θ + R·sinθ (13B: R = 105 mm, e = 15 mm). Lega di alluminio con pista cromata; qui sono ricavate la luce di scarico periferica e le sedi delle candele. La zona candele resta sempre calda, quella di aspirazione sempre fredda: distorsioni termiche tipiche del Wankel. Il pacco di statori e piastre è serrato da tiranti passanti.',
        explode: [0, 0, 0],
      })
    );

    // ---------- Side housings ----------
    const outerShape = (notch) => {
      const s = new THREE.Shape();
      let inNotch = false;
      let first = true;
      const add = (x, y) => (first ? (s.moveTo(x, y), (first = false)) : s.lineTo(x, y));
      for (let i = 0; i < 360; i++) {
        const [x, y] = troch((i / 360) * TAU, OUTER);
        const w = notch && angDist(Math.atan2(y, x), IN_PORT.a) < 0.1;
        if (w && !inNotch) {
          // intake passage: radial slot from the outer edge to the port
          for (let k = 0; k <= 4; k++) {
            const a = IN_PORT.a - 0.1 + (0.2 * k) / 4;
            add(0.98 * Math.cos(a), 0.98 * Math.sin(a));
          }
        }
        inNotch = w;
        if (!w) add(x, y);
      }
      return s;
    };
    const portHole = () => {
      const h = new THREE.Path();
      h.absellipse(IN_PORT.r * Math.cos(IN_PORT.a), IN_PORT.r * Math.sin(IN_PORT.a), IN_PORT.w, IN_PORT.h, 0, TAU, true, IN_PORT.a + Math.PI / 2);
      return h;
    };
    const plateShape = ({ port = false, notch = false, shaft = 0.3 } = {}) => {
      const s = outerShape(notch);
      s.holes.push(circle(shaft), ...boltHoles());
      if (port) s.holes.push(portHole());
      return s;
    };
    const plateMat = cut(MAT.castIron());
    plateMat.color.set(0x5b636d);
    const frontPlate = new THREE.Group();
    frontPlate.add(new THREE.Mesh(extrudeBevel(plateShape(), FRONT[0], FRONT[1]), plateMat));
    root.add(
      ctx.part(framed(frontPlate), {
        id: 'frontHousing',
        name: 'Piastra laterale anteriore',
        base: 'Il “coperchio” davanti al rotore. Nella vista in sezione è tolto per farti vedere cosa succede dentro.',
        expert:
          'Piastra in ghisa con superficie nitrurata (nell’8C in alluminio con riporto ceramico) su cui strisciano i segmenti laterali. Porta il pignone fisso e il supporto di banco anteriore dell’albero eccentrico.',
        explode: [-1.38, 0.31, 1.53],
      })
    );

    // intermediate housing: front skin (port), middle layer (intake passage), rear skin (port)
    const inter = new THREE.Group();
    const skin = 0.06;
    inter.add(
      new THREE.Mesh(extrude(plateShape({ port: true }), INTER[1] - skin, INTER[1], 24), plateMat),
      new THREE.Mesh(extrude(plateShape({ notch: true }), INTER[0] + skin, INTER[1] - skin, 24), plateMat),
      new THREE.Mesh(extrude(plateShape({ port: true }), INTER[0], INTER[0] + skin, 24), plateMat)
    );
    // intermediate main bearing (bronze bush around the shaft)
    const interBush = new THREE.Mesh(pipe(0.3, 0.205, INTER[1] - INTER[0] - 0.01, 40).rotateX(Math.PI / 2), MAT.brass());
    interBush.position.z = INTER_MID;
    const interPart = new THREE.Group();
    interPart.add(framed(inter, interBush));
    // engine mounts on both sides (world coordinates)
    const mountMat = MAT.paint(0x2b2f36, 0.6);
    for (const sx of [-1, 1]) {
      const bracket = new THREE.Mesh(rbox(0.36, 0.3, 0.24, 0.03), mountMat);
      bracket.position.set(sx * 1.84, 0.05, INTER_MID);
      const rubber = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.12, 24), MAT.rubber());
      rubber.position.set(sx * 1.92, -0.16, INTER_MID);
      interPart.add(bracket, rubber);
    }
    root.add(
      ctx.part(interPart, {
        id: 'intermediateHousing',
        name: 'Piastra intermedia e luci di aspirazione',
        base: 'La parete tra i due rotori. Dentro ha un condotto che porta l’aria dal collettore alle due “bocche” a forma di fagiolo, una per lato: una per il rotore davanti e una per quello dietro.',
        expert:
          'Piastra intermedia in ghisa con condotti interni e luci di aspirazione laterali su entrambe le facce; la forma e la posizione delle luci decidono la fasatura al posto dell’albero a camme. Al centro il supporto di banco intermedio dell’albero eccentrico. Ai lati gli attacchi dei supporti motore.',
        explode: [0.53, -0.12, -0.59],
      })
    );

    const rearPlate = new THREE.Group();
    rearPlate.add(new THREE.Mesh(extrudeBevel(plateShape(), REAR[0], REAR[1]), plateMat));

    // ---------- Rotors ----------
    const rotorOutline = () => {
      const s = new THREE.Shape();
      for (let k = 0; k < BINS; k++) {
        const b = (k / BINS) * TAU;
        k === 0 ? s.moveTo(prof[k] * Math.cos(b), prof[k] * Math.sin(b)) : s.lineTo(prof[k] * Math.cos(b), prof[k] * Math.sin(b));
      }
      return s;
    };
    const rotorMat = MAT.castIron();
    rotorMat.color.set(0x7a7f86);
    rotorMat.metalness = 0.7;
    rotorMat.roughness = 0.45;
    const body = rotorOutline();
    body.holes.push(circle(0.47));
    const bodyGeo = extrude(body, -RHALF, 0.23);
    const lip = rotorOutline();
    lip.holes.push(circle(0.7));
    const lipGeo = extrude(lip, 0.23, RHALF);
    const bearingGeo = new THREE.CylinderGeometry(0.47, 0.47, 0.56, 48, 1, true).rotateX(Math.PI / 2).translate(0, 0, -0.06);
    // internal ring gear, 30 teeth, pitch radius 3e (gap at angle π meshes the pinion tooth)
    const ringShape = new THREE.Shape();
    ringShape.absellipse(0, 0, 0.695, 0.695, 0, TAU, false);
    const ringHole = new THREE.Path();
    ringHole.setFromPoints(toothed(3 * E, 30, 0.026, Math.PI, 600));
    ringShape.holes.push(ringHole);
    const ringGeo = extrude(ringShape, 0.23, 0.345, 64);
    const oRingGeo = new THREE.TorusGeometry(0.82, 0.012, 6, 64).translate(0, 0, RHALF + 0.002);
    const makeRotor = () => {
      const g = new THREE.Group();
      g.add(
        new THREE.Mesh(bodyGeo, rotorMat),
        new THREE.Mesh(lipGeo, rotorMat),
        new THREE.Mesh(bearingGeo, MAT.brass()),
        new THREE.Mesh(ringGeo, MAT.darkSteel()),
        new THREE.Mesh(oRingGeo, MAT.rubber())
      );
      return g;
    };
    const apexGeo = new THREE.BoxGeometry(0.07, 0.024, 2 * RHALF);
    const cornerGeo = new THREE.CylinderGeometry(0.032, 0.032, 2 * RHALF + 0.01, 16).rotateX(Math.PI / 2);
    const sideSealGeos = [0, 1, 2].map((i) => {
      const b0 = (i * TAU) / 3;
      const pts = [];
      for (let k = 0; k <= 30; k++) {
        const bb = b0 + 0.12 + ((TAU / 3 - 0.24) * k) / 30;
        const r = rFlank(bb) - 0.07;
        pts.push(v3(r * Math.cos(bb), r * Math.sin(bb), RHALF + 0.004));
      }
      return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 40, 0.009, 5);
    });
    const sealMat = MAT.chrome();
    const makeSeals = (tag) => {
      const g = new THREE.Group();
      for (let i = 0; i < 3; i++) {
        const b = (i * TAU) / 3;
        const apex = new THREE.Mesh(apexGeo, sealMat);
        apex.name = tag;
        apex.position.set((RG - 0.035) * Math.cos(b), (RG - 0.035) * Math.sin(b), 0);
        apex.rotation.z = b;
        const corner = new THREE.Mesh(cornerGeo, steelMat);
        corner.position.set((RG - 0.11) * Math.cos(b), (RG - 0.11) * Math.sin(b), 0);
        g.add(apex, corner, new THREE.Mesh(sideSealGeos[i], sealMat));
      }
      return g;
    };
    const rotor = makeRotor();
    root.add(
      ctx.part(framed(rotor), {
        id: 'rotor',
        name: 'Rotore anteriore',
        base: 'Il triangolo con i lati bombati: è il “pistone” del Wankel. Girando, ognuno dei suoi tre lati fa da parete mobile per una camera.',
        expert:
          'Rotore in ghisa, fianchi pari all’inviluppo interno dell’epitrocoide con una tasca di combustione al centro di ciascuno (ne fissa il rapporto di compressione). Al centro: cuscinetto a strisciamento sull’eccentrico, dentatura interna (30 denti) e anello di tenuta dell’olio. Il centro del rotore descrive un cerchio di raggio e mentre il rotore ruota di φ/3.',
        explode: [-0.61, 0.14, 0.68],
      })
    );
    const seals = makeSeals('apexSeal');
    root.add(
      ctx.part(framed(seals), {
        id: 'apexSeals',
        name: 'Segmenti d’apice e di tenuta',
        base: 'Lamelle sulle tre punte del rotore: strisciano sulla pista e tengono separate le camere, come le guarnizioni di una porta scorrevole.',
        expert:
          'Segmenti d’apice (2–3 mm, in ghisa o ceramica) spinti contro la pista da molle e dalla pressione del gas, più segmenti angolari e laterali. Velocità di strisciamento oltre 30 m/s: sono il punto critico per usura e tenuta. Vengono lubrificati iniettando olio nell’aspirazione (pompa dosatrice), da cui il consumo d’olio tipico del Wankel.',
        explode: [-0.88, 0.2, 0.98],
      })
    );

    // rear rotor: same parts flipped (ring gear on its rear face), phased 180° on the shaft
    const rotor2 = new THREE.Group();
    const flip = new THREE.Group();
    flip.rotation.x = Math.PI; // profile is mirror-symmetric about the apex axis
    flip.add(makeRotor(), makeSeals('apexSeal2'));
    rotor2.add(flip);
    root.add(
      ctx.part(framed(rotor2), {
        id: 'rearRotor',
        name: 'Rotore posteriore',
        base: 'Il secondo rotore, nascosto dietro la piastra intermedia. È girato di mezzo giro rispetto al primo: così le spinte arrivano alternate e il motore gira più regolare.',
        expert:
          'Eccentrico sfasato di 180° rispetto all’anteriore: due fasi utili per giro d’albero e masse rotanti bilanciate staticamente; la coppia residua è compensata dai contrappesi anteriore e sul volano. Pignone fisso montato sulla piastra posteriore.',
        explode: [1.03, -0.23, -1.15],
      })
    );
    root.add(
      ctx.part(framed(makeHousing(Z2)), {
        id: 'rearRotorHousing',
        name: 'Statore posteriore',
        base: 'La pista del secondo rotore, uguale a quella davanti.',
        expert: 'Statore identico all’anteriore, con propria luce di scarico periferica e due candele; tra i due statori la piastra intermedia.',
        explode: [1.03, -0.23, -1.15],
      })
    );

    // ---------- Phasing gears (fixed pinions on the side housings) ----------
    const pinionShape = new THREE.Shape(toothed(2 * E, 20, 0.024, Math.PI, 400));
    pinionShape.holes.push(circle(0.225));
    const pinionGeo = extrude(pinionShape, 0.245, 0.38, 64);
    const flangeGeo = new THREE.CylinderGeometry(0.52, 0.52, 0.03, 48).rotateX(Math.PI / 2).translate(0, 0, 0.39);
    const bushGeo = new THREE.CylinderGeometry(0.224, 0.224, 0.13, 32, 1, true).rotateX(Math.PI / 2).translate(0, 0, 0.31);
    const makePinion = () => {
      const g = new THREE.Group();
      g.add(new THREE.Mesh(pinionGeo, steelMat), new THREE.Mesh(flangeGeo, MAT.darkSteel()), new THREE.Mesh(bushGeo, MAT.brass()));
      return g;
    };
    root.add(
      ctx.part(framed(makePinion()), {
        id: 'phasingGears',
        name: 'Pignone fisso (ingranaggio di fase)',
        base: 'Una ruota dentata ferma, fissata alla piastra anteriore. I denti dentro il rotore le “rotolano” attorno e così il rotore gira sempre col giusto passo, senza mai sbattere contro la pista.',
        expert:
          'Pignone fisso a 20 denti (raggio primitivo 2e) e corona interna del rotore a 30 denti (3e): rapporto 3:2. Non trasmette coppia, solo la fase: il rotore ruota di φ/3 mentre il suo centro orbita di φ, così gli apici restano sull’epitrocoide. Dentro il pignone c’è la bronzina di banco dell’albero.',
        explode: [-1.11, 0.25, 1.23],
      })
    );
    const rearPinion = makePinion();
    rearPinion.rotation.x = Math.PI;
    rearPinion.position.z = Z2;
    rearPlate.add(rearPinion);
    root.add(
      ctx.part(framed(rearPlate), {
        id: 'rearHousing',
        name: 'Piastra laterale posteriore',
        base: 'La parete dietro al secondo rotore. Porta la seconda ruota dentata fissa e il supporto posteriore dell’albero.',
        expert: 'Piastra posteriore in ghisa con pignone fisso del rotore posteriore e supporto di banco; dietro c’è il volano.',
        explode: [1.49, -0.33, -1.66],
      })
    );

    // ---------- Eccentric shaft ----------
    const shaft = new THREE.Group();
    const journal = new THREE.Mesh(cylZ(0.2, 2.97, 32), steelMat);
    journal.position.z = -0.465; // z -1.95 .. 1.02
    shaft.add(journal);
    const lobeGeo = cylZ(0.44, 0.54, 48);
    const lobe1 = new THREE.Mesh(lobeGeo, MAT.chrome());
    lobe1.position.set(E, 0, -0.06);
    const lobe2 = new THREE.Mesh(lobeGeo, MAT.chrome());
    lobe2.position.set(-E, 0, Z2 + 0.06);
    shaft.add(lobe1, lobe2);
    const shaftThread = new THREE.Mesh(cylZ(0.09, 0.14, 20), steelMat); // threaded end for the flywheel nut
    shaftThread.position.z = -2.0;
    shaft.add(shaftThread);
    // front counterweight: its own part (it has to come off the shaft before the front housing can)
    const frontCw = new THREE.Group();
    frontCw.add(
      new THREE.Mesh(sector(0.42, Math.PI, Math.PI / 2, 0.65, 0.71), MAT.darkSteel()),
      new THREE.Mesh(pipe(0.27, 0.2, 0.06, 32).rotateX(Math.PI / 2).translate(0, 0, 0.68), MAT.darkSteel())
    );
    root.add(
      ctx.part(framed(frontCw), {
        id: 'frontCounterweight',
        name: 'Contrappeso anteriore',
        base: 'Un mezzo disco pesante infilato sull’albero, nascosto dentro il coperchio anteriore. Insieme a quello sul volano compensa il “dondolio” dei due rotori, che girano sfasati uno davanti all’altro.',
        expert:
          'I due rotori, sfasati di 180° e distanti lungo l’asse, si equilibrano come forze ma generano una coppia che tende a far oscillare l’albero. La compensano due masse opposte alle estremità: questo contrappeso davanti e la massa di squilibrio sul volano dietro.',
        explode: [-1.53, 0.34, 1.7],
      })
    );
    root.add(
      ctx.part(framed(shaft), {
        id: 'eccentricShaft',
        name: 'Albero eccentrico',
        base: 'L’albero del motore. Ha due “gomiti” rotondi (gli eccentrici), uno per rotore, girati di mezzo giro: quando un rotore spinge sul suo eccentrico, l’albero gira, tre volte più veloce dei rotori.',
        expert:
          'Albero in acciaio con due eccentrici a 180° (e = 15 mm nel 13B) su tre supporti di banco. Contrappeso anteriore e massa sul volano equilibrano la coppia dei due rotori: niente forze alterne, da cui la grande regolarità e i regimi fino a 9 000 giri/min. Davanti la puleggia della cinghia servizi.',
        explode: [0.19, -0.04, -0.21],
      })
    );

    // ---------- Flywheel ----------
    const flywheel = new THREE.Group();
    flywheel.add(new THREE.Mesh(cylZ(1.05, 0.08, 64), MAT.darkSteel()));
    const fwRing = new THREE.Shape(toothed(1.12, 96, 0.018, 0, 960));
    fwRing.holes.push(circle(1.04));
    flywheel.add(new THREE.Mesh(extrude(fwRing, -0.04, 0.04, 8), steelMat));
    flywheel.add(new THREE.Mesh(sector(0.85, 0, Math.PI * 0.3, 0.04, 0.1), steelMat)); // opposite the rear lobe
    flywheel.add(new THREE.Mesh(cylZ(0.3, 0.2, 32), steelMat));
    flywheel.position.z = -1.85;
    // one big nut on the shaft end (flanged hex, +Y pointing backwards out of the thread)
    const fwNut = new THREE.Group();
    const fwNutGeo = mergeGeometries([new THREE.CylinderGeometry(0.15, 0.15, 0.08, 6), new THREE.CylinderGeometry(0.2, 0.2, 0.016, 24).translate(0, -0.034, 0)]);
    fwNut.add(instanced(fwNutGeo, steelMat, [{ p: v3(0, 0, -1.992), n: v3(0, 0, -1) }]));
    root.add(
      ctx.part(framed(flywheel), {
        id: 'flywheel',
        name: 'Volano con contrappeso',
        base: 'Il disco pesante dietro al motore: tiene costante la rotazione e porta i denti per il motorino d’avviamento.',
        expert:
          'Volano con massa di squilibrio integrata, opposta all’eccentrico posteriore: insieme al contrappeso anteriore compensa la coppia dovuta ai due rotori sfasati. La corona dentata ingrana col motorino d’avviamento.',
        explode: [1.91, -0.42, -2.12],
      }),
      ctx.part(framed(fwNut), {
        id: 'flywheelNut',
        name: 'Dado del volano',
        base: 'Un solo grosso dado al centro tiene il volano stretto sulla punta dell’albero eccentrico.',
        expert:
          'Nel 13B il volano è calettato sull’estremità conica dell’albero, con una chiavetta che ne fissa la posizione angolare, ed è serrato da un unico grande dado (chiave da 54 mm). La posizione conta, perché il volano porta la massa di equilibratura posteriore.',
        explode: [2.1, -0.47, -2.33],
      })
    );

    // ---------- Tension bolts through the whole stack: heads on the front plate, threaded into the rear one ----------
    const tbolts = instanced(
      screw(FRONT[1] - REAR[0] - 0.01, 0.055, 0.05),
      cut(MAT.steel(), { cap: false }),
      BOLT_A.map((a) => {
        const [x, y] = troch(a, RG + 0.4);
        return { p: v3(x, y, FRONT[1] + 0.025), n: v3(0, 0, 1) };
      })
    );
    root.add(
      ctx.part(framed(tbolts), {
        id: 'tensionBolts',
        name: 'Tiranti del pacco',
        base: 'Lunghe viti che attraversano tutto il motore, da una piastra all’altra: stringono piastre e statori come le fette di un panino. Tolte quelle, il motore si apre fetta per fetta.',
        expert:
          'Tiranti passanti in acciaio (18 nel 13B, qui meno per semplicità) che serrano a pacco piastre laterali, statori e piastra intermedia. Il pacco alterna ghisa e alluminio, che si dilatano in modo diverso: per questo si serrano poco per volta e in ordine incrociato, così le tenute dell’acqua e dell’olio lavorano in modo uniforme.',
        explode: [-1.38, 0.31, 2.9], // still threaded through the front housing, pulled out forwards
      })
    );

    // ---------- Front cover (oil pump, front bearing), accessory drive ----------
    const coverMat = cut(MAT.aluminum());
    coverMat.color.set(0xb4b9bf);
    coverMat.roughness = 0.6;
    const coverShape = new THREE.Shape();
    coverShape.absellipse(0, -0.12, 1.05, 1.3, 0, TAU, false);
    coverShape.holes.push(circle(0.24));
    const cover = new THREE.Group();
    cover.add(new THREE.Mesh(extrudeBevel(coverShape, FRONT[1] - 0.01, FRONT[1] + 0.16, 0.03), coverMat));
    for (const x of [-0.6, 0.6]) {
      const rib = new THREE.Mesh(rbox(0.05, 1.6, 0.04, 0.015), coverMat);
      rib.position.set(x, -0.12, FRONT[1] + 0.16);
      cover.add(rib);
    }
    const coverBolts = [];
    for (let i = 0; i < 12; i++) {
      const a = (i / 12) * TAU;
      coverBolts.push({ p: v3(0.94 * Math.cos(a), -0.12 + 1.18 * Math.sin(a), FRONT[1] + 0.17), n: v3(0, 0, 1) });
    }
    const plate = new THREE.Mesh(new THREE.PlaneGeometry(0.62, 0.19), cut(nameplate(), { cap: false }));
    plate.position.set(0, -0.82, FRONT[1] + 0.162);
    cover.add(plate);
    const filler = new THREE.Mesh(cylZ(0.06, 0.08, 20), cut(MAT.paint(0xe0b23a, 0.4), { cap: false }));
    filler.position.set(-0.55, -0.85, FRONT[1] + 0.2);
    cover.add(filler);
    root.add(
      ctx.part(cover, {
        id: 'frontCover',
        name: 'Coperchio anteriore',
        base: 'Il coperchio davanti al motore: chiude gli ingranaggi della pompa dell’olio e regge l’albero. Sopra c’è il tappo per l’olio.',
        expert:
          'Coperchio in alluminio con pompa olio trascinata a catena dall’albero eccentrico e pompa dosatrice che inietta olio per i segmenti d’apice. L’olio raffredda anche l’interno dei rotori, che non sono toccati dal liquido di raffreddamento.',
        explode: [-1.68, 0.37, 1.87],
      }),
      ctx.part(instanced(screw(0.14, 0.034, 0.03), boltHeadMat, coverBolts), {
        id: 'frontCoverBolts',
        name: 'Viti del coperchio anteriore',
        base: 'Le viti lungo il bordo che fissano il coperchio anteriore alla prima piastra.',
        expert: 'Viti corte disposte lungo tutto il perimetro: premono il coperchio sulla sua guarnizione, perché dietro circola l’olio della pompa. Si allentano un po’ alla volta, in ordine incrociato.',
        explode: [-1.78, 0.39, 1.98],
      })
    );

    // crank pulley on the nose of the eccentric shaft: belongs to the accessory drive, turns with the shaft
    const pulley = new THREE.Mesh(
      new THREE.LatheGeometry(
        [[0.2, -0.04], [0.29, -0.04], [0.26, 0], [0.29, 0.04], [0.2, 0.04], [0.2, -0.04]].map(([r, y]) => new THREE.Vector2(r, y)),
        48
      ).rotateX(Math.PI / 2),
      MAT.darkSteel()
    );
    pulley.position.z = 0.97;
    const shaftNut = new THREE.Mesh(hexGeo, steelMat);
    shaftNut.rotation.x = Math.PI / 2;
    shaftNut.position.z = 1.03;
    const crankPulley = new THREE.Group();
    crankPulley.add(pulley, shaftNut);

    const acc = new THREE.Group();
    const wp = v3(-0.55, 1.15, 0);
    const alt = v3(1.15, 1.15, 0);
    const PZ = 0.97;
    const wpBody = new THREE.Mesh(cylZ(0.24, 0.26, 32), coverMat);
    wpBody.position.set(wp.x, wp.y, FRONT[1] + 0.13);
    const wpPulley = new THREE.Group();
    const wpHub = new THREE.Mesh(cylZ(0.05, 0.14, 16), steelMat);
    wpHub.position.z = -0.07;
    wpPulley.add(new THREE.Mesh(cylZ(0.2, 0.05, 40), MAT.darkSteel()), new THREE.Mesh(rbox(0.3, 0.05, 0.02, 0.01), steelMat), wpHub);
    wpPulley.position.set(wp.x, wp.y, PZ);
    const hose = tube([v3(wp.x, wp.y + 0.22, 0.78), v3(wp.x - 0.15, wp.y + 0.55, 0.45), v3(-0.95, 1.72, 0.05), v3(-1.02, 1.78, -0.15), v3(-1.03, 1.62, -0.2)], 0.055, MAT.rubber(), 32);
    const clampGeo = new THREE.TorusGeometry(0.06, 0.01, 6, 20);
    const hoseClamp = new THREE.Mesh(clampGeo, steelMat);
    hoseClamp.position.set(wp.x - 0.04, wp.y + 0.3, 0.72);
    hoseClamp.rotation.x = 1.0;
    const altBody = new THREE.Mesh(cylZ(0.2, 0.28, 32), MAT.aluminum());
    altBody.position.set(alt.x, alt.y, FRONT[1] + 0.18);
    const altFins = new THREE.Mesh(cylZ(0.205, 0.12, 32), MAT.darkSteel());
    altFins.position.set(alt.x, alt.y, FRONT[1] + 0.12);
    const altBracket = new THREE.Mesh(rbox(0.5, 0.12, 0.1, 0.02), mountMat);
    altBracket.position.set(alt.x - 0.3, alt.y - 0.12, FRONT[1] + 0.06);
    altBracket.rotation.z = 0.5;
    const altPulley = new THREE.Group();
    altPulley.add(new THREE.Mesh(cylZ(0.11, 0.05, 28), MAT.darkSteel()), new THREE.Mesh(rbox(0.18, 0.03, 0.02, 0.01), steelMat));
    altPulley.position.set(alt.x, alt.y, PZ);
    const pts = [];
    const ring = (cx, cy, r) => {
      for (let i = 0; i < 32; i++) pts.push([cx + r * Math.cos((i / 32) * TAU), cy + r * Math.sin((i / 32) * TAU)]);
    };
    ring(0, 0, 0.3);
    ring(wp.x, wp.y, 0.21);
    ring(alt.x, alt.y, 0.12);
    const hull = convexHull(pts).map(([x, y]) => v3(x, y, PZ));
    const belt = tube(hull, 0.018, MAT.rubber(), 160, true, 5);
    acc.add(crankPulley, wpBody, wpPulley, hose, hoseClamp, altBody, altFins, altBracket, altPulley, belt);
    cutAll(acc); // everything here is in front of the section plane
    root.add(
      ctx.part(acc, {
        id: 'accessoryDrive',
        name: 'Pompa dell’acqua, alternatore e cinghia',
        base: 'Una cinghia mossa dall’albero fa girare la pompa che spinge il liquido di raffreddamento negli statori e l’alternatore che ricarica la batteria.',
        expert:
          'Cinghia servizi dalla puleggia dell’albero eccentrico a pompa acqua e alternatore. Il liquido scorre nei passaggi assiali degli statori: la zona candele/scarico va raffreddata molto più di quella di aspirazione.',
        explode: [-2.06, 0.76, 2.29],
      })
    );

    // ---------- Oil pan and filter ----------
    const pan = new THREE.Group();
    const panMat = cut(MAT.paint(0x3a3f46, 0.55), { cap: false });
    const panZ = (FRONT[1] + REAR[0]) / 2;
    const panBox = new THREE.Mesh(new RoundedBoxGeometry(2.0, 0.5, FRONT[1] - REAR[0], 3, 0.08), panMat);
    panBox.position.set(0, -2.22, panZ);
    pan.add(panBox);
    for (let i = 0; i < 6; i++) {
      const rib = new THREE.Mesh(rbox(1.8, 0.05, 0.04, 0.015), panMat);
      rib.position.set(0, -2.48, panZ - 0.9 + i * 0.36);
      pan.add(rib);
    }
    const panRail = new THREE.Mesh(rbox(2.06, 0.05, FRONT[1] - REAR[0] + 0.04, 0.015), panMat);
    panRail.position.set(0, -1.98, panZ);
    pan.add(panRail);
    const oilFilter = new THREE.Mesh(cylX(0.13, 0.3, 32), MAT.paint(0x2d4a7a, 0.4));
    oilFilter.position.set(-1.17, -2.15, 0.05);
    const filterBase = new THREE.Mesh(cylX(0.1, 0.06, 24), MAT.aluminum());
    filterBase.position.set(-1.01, -2.15, 0.05);
    const drain = new THREE.Mesh(smallHex, steelMat);
    drain.position.set(0.5, -2.49, panZ);
    pan.add(oilFilter, filterBase, drain);
    root.add(
      ctx.part(pan, {
        id: 'oilPan',
        name: 'Coppa dell’olio e filtro',
        base: 'La vaschetta sotto il motore con l’olio. Nel Wankel l’olio lubrifica, raffredda l’interno dei rotori e un po’ viene anche bruciato apposta per lubrificare i segmenti.',
        expert:
          'Coppa in lamiera o alluminio con alette; il circuito raffredda i rotori dall’interno con getti d’olio. Una parte dell’olio viene dosata nella camera di combustione: è normale controllarne spesso il livello.',
        explode: [0, -1.1, 0],
      })
    );

    // ---------- Spark plugs, coils, leads ----------
    const plugs = []; // one group per plug, local +Y out of its bore (they unscrew along it)
    const boots = new THREE.Group(); // plug caps: they belong to the leads
    const sparks = [];
    const sparkLights = [];
    const fx = new THREE.Group(); // gas, sparks and lights, in the local frame
    fx.rotation.z = FRAME;
    root.add(fx);
    const plugBody = new THREE.CylinderGeometry(0.045, 0.045, 0.32, 16).translate(0, 0.72, 0);
    const plugHex = new THREE.CylinderGeometry(0.075, 0.075, 0.09, 6).translate(0, 0.52, 0);
    const plugTip = new THREE.CylinderGeometry(0.035, 0.03, 0.47, 12).translate(0, 0.235, 0);
    const bootGeo = new THREE.CylinderGeometry(0.06, 0.06, 0.2, 16).translate(0, 0.95, 0);
    const bootMats = [MAT.paint(0x2a2f38, 0.6), MAT.paint(0x8a2a22, 0.6)];
    const plugMats = [MAT.ceramic(), MAT.chrome(), steelMat];
    const coilPos = v3(-2.08, 1.35, INTER_MID);
    const leads = new THREE.Group();
    for (const z of [0, Z2]) {
      PLUG_A.forEach((a, i) => {
        const p = new THREE.Group();
        p.add(new THREE.Mesh(plugBody, plugMats[0]), new THREE.Mesh(plugHex, plugMats[1]), new THREE.Mesh(plugTip, plugMats[2]));
        const boot = new THREE.Mesh(bootGeo, bootMats[i]);
        const [x, y] = troch(a, RG + 0.02);
        for (const o of [p, boot]) {
          o.position.set(x, y, z);
          o.rotation.z = a - Math.PI / 2;
        }
        plugs.push(p);
        boots.add(boot);
        // lead from the boot to the coil pack (world coordinates)
        const tL = [x + 1.05 * Math.cos(a), y + 1.05 * Math.sin(a)];
        const tW = toWorld(tL[0], tL[1], z);
        const dirW = toWorld(Math.cos(a), Math.sin(a));
        leads.add(
          tube([tW, tW.clone().addScaledVector(dirW, 0.12), v3(-2.3, tW.y * 0.5 + 0.55, (z + INTER_MID) / 2), v3(coilPos.x - 0.1, coilPos.y - 0.25 + i * 0.12, coilPos.z + (z === 0 ? 0.2 : -0.2))], 0.018, bootMats[i], 32, false, 6)
        );
        if (z !== 0) return;
        const [sx, sy] = troch(a, RG - 0.04);
        const spark = new THREE.Mesh(new THREE.SphereGeometry(0.075, 16, 12), MAT.glow(0xbfe6ff, 6, 0.95));
        spark.position.set(sx, sy, 0.1);
        spark.visible = false;
        fx.add(spark);
        sparks.push(spark);
        const light = new THREE.PointLight(0xffa040, 0, 3, 2);
        light.position.set(sx * 0.85, sy * 0.85, 0.6);
        fx.add(light);
        sparkLights.push(light);
      });
    }
    const coil = new THREE.Mesh(rbox(0.24, 0.5, 0.7, 0.04), MAT.paint(0x1c1f24, 0.5));
    coil.position.copy(coilPos);
    const edge = toWorld(...troch(Math.PI * 235 / 180, OUTER), INTER_MID);
    const mid = edge.clone().add(coilPos).multiplyScalar(0.5);
    const coilBracket = new THREE.Mesh(rbox(edge.distanceTo(coilPos), 0.08, 0.2, 0.02), mountMat);
    coilBracket.position.copy(mid);
    coilBracket.rotation.z = Math.atan2(coilPos.y - edge.y, coilPos.x - edge.x);
    leads.add(coil, coilBracket);
    leads.add(framed(boots));
    root.add(
      ctx.part(framed(...plugs), {
        id: 'sparkPlugs',
        name: 'Candele leading e trailing',
        base: 'Due candele per rotore fanno la scintilla una dopo l’altra: la camera del Wankel è lunga e una sola scintilla non basterebbe a bruciare bene tutta la miscela.',
        expert:
          'Candela leading (avanti nel senso di rotazione) e trailing (indietro), con anticipi diversi: la trailing scocca qualche grado d’albero dopo per accendere la miscela spinta verso la coda della camera. Le candele lavorano sempre nella zona calda dello statore; le sedi sono piccole per non far “saltare” i segmenti d’apice.',
        explode: [-1.0, 0, 0],
      }),
      ctx.part(leads, {
        id: 'ignitionLeads',
        name: 'Bobine e cavi delle candele',
        base: 'Le bobine producono l’alta tensione e i cavi la portano alle quattro candele: ogni cavo finisce con un cappuccio infilato sulla sua candela.',
        expert:
          'Leading e trailing scoccano con anticipi diversi, quindi hanno circuiti separati. In molti 13B le due candele leading condividono una bobina a scintilla persa (i rotori sono sfasati di mezzo giro d’albero), mentre le trailing hanno bobine proprie; nel Renesis dell’RX-8 ogni candela ha la sua bobina.',
        explode: [-1.6, 0.15, 0],
      })
    );

    // ---------- Intake: plenum, throttle body, runner into the intermediate housing ----------
    const portL = [IN_PORT.r * Math.cos(IN_PORT.a), IN_PORT.r * Math.sin(IN_PORT.a)];
    const portW = toWorld(portL[0], portL[1]);
    let edgeR = 0;
    let best = Infinity;
    for (let i = 0; i < 2880; i++) {
      const [x, y] = troch((i / 2880) * TAU, OUTER);
      const d = angDist(Math.atan2(y, x), IN_PORT.a);
      if (d < best) [best, edgeR] = [d, Math.hypot(x, y)];
    }
    const edgeW = toWorld(edgeR * Math.cos(IN_PORT.a), edgeR * Math.sin(IN_PORT.a), INTER_MID);
    const outW = toWorld(Math.cos(IN_PORT.a), Math.sin(IN_PORT.a));
    const plenumC = v3(1.95, 1.78, -0.5);
    const intakeMat = cut(MAT.paint(0x2a3442, 0.5), { cap: false });
    const intake = new THREE.Group();
    const runnerCurve = new THREE.CatmullRomCurve3([
      edgeW.clone().addScaledVector(outW, -0.02),
      edgeW.clone().addScaledVector(outW, 0.25),
      v3(plenumC.x - 0.02, plenumC.y - 0.35, INTER_MID),
      v3(plenumC.x, plenumC.y - 0.1, INTER_MID),
    ]);
    intake.add(new THREE.Mesh(new THREE.TubeGeometry(runnerCurve, 32, 0.1, 16), intakeMat));
    const runnerFlange = new THREE.Mesh(rbox(0.06, 0.3, 0.26, 0.02), mountMat);
    runnerFlange.position.copy(edgeW).addScaledVector(outW, 0.02);
    runnerFlange.rotation.z = Math.atan2(outW.y, outW.x);
    const plenum = new THREE.Mesh(new THREE.CapsuleGeometry(0.26, 1.2, 8, 24).rotateX(Math.PI / 2), intakeMat);
    plenum.position.copy(plenumC);
    const tbody = new THREE.Mesh(cylZ(0.16, 0.18, 32), MAT.aluminum());
    tbody.position.set(plenumC.x, plenumC.y, 0.44);
    const duct = tube([v3(plenumC.x, plenumC.y, 0.52), v3(plenumC.x, plenumC.y + 0.05, 0.72), v3(plenumC.x, plenumC.y + 0.35, 0.82)], 0.14, MAT.rubber(), 16);
    const pod = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.17, 0.3, 32), MAT.paint(0xb8673a, 0.85));
    pod.position.set(plenumC.x, plenumC.y + 0.5, 0.84);
    const injGeo = new THREE.CylinderGeometry(0.035, 0.02, 0.24, 12);
    const injMat = MAT.paint(0x222831);
    const injectors = [0.35, 0.6].map((t) => {
      const p = runnerCurve.getPointAt(t);
      const inj = new THREE.Mesh(injGeo, injMat);
      inj.position.set(p.x - 0.08, p.y + 0.05, p.z + 0.12);
      inj.rotation.set(-0.6, 0, 0.5);
      return inj;
    });
    const rail = new THREE.Mesh(cylX(0.03, 0.4, 12), MAT.chrome());
    rail.position.copy(runnerCurve.getPointAt(0.48)).add(v3(-0.1, 0.12, 0.24));
    rail.rotation.z = 1.0;
    intake.add(runnerFlange, plenum, tbody, duct, pod, rail, ...injectors);
    root.add(
      ctx.part(intake, {
        id: 'intakeManifold',
        name: 'Collettore di aspirazione e iniettori',
        base: 'L’aria entra dal filtro, passa la farfalla del gas, si raccoglie in un “polmone” e scende nella piastra intermedia, che la porta ai due rotori. Gli iniettori spruzzano la benzina.',
        expert:
          'Plenum e condotti accordati; nel Renesis valvole secondarie aprono luci aggiuntive agli alti regimi. Iniezione indiretta nel condotto (diretta nell’8C della MX-30). La pompa dosatrice aggiunge una piccola quantità di olio per lubrificare i segmenti.',
        explode: [0.9, 0.7, 0],
      })
    );

    // ---------- Exhaust (peripheral port of each rotor housing, merged) ----------
    const exIn = troch(pm, RG + 0.03);
    const exOut = troch(pm, OUTER);
    const n = toWorld(Math.cos(pm), Math.sin(pm));
    const exOutW = toWorld(exOut[0], exOut[1]);
    const merge = exOutW.clone().addScaledVector(n, 0.5);
    const exhaustCurves = [0, Z2].map(
      (z) =>
        new THREE.CatmullRomCurve3([
          toWorld(exIn[0], exIn[1], z),
          exOutW.clone().addScaledVector(n, -0.05).setZ(z),
          exOutW.clone().addScaledVector(n, 0.3).setZ(z),
          v3(merge.x + 0.1, merge.y - 0.15, z * 0.4 + INTER_MID * 0.6),
          v3(merge.x + 0.45, merge.y - 0.6, INTER_MID),
          v3(merge.x + 0.5, -2.4, INTER_MID),
        ])
    );
    const exhaust = new THREE.Group();
    const exMat = cut(MAT.paint(0x6b4a3a, 0.55), { cap: false });
    exMat.metalness = 0.8;
    const flangeEx = new THREE.CylinderGeometry(0.2, 0.2, 0.05, 6);
    for (const [i, cv] of exhaustCurves.entries()) {
      exhaust.add(new THREE.Mesh(new THREE.TubeGeometry(cv, 64, 0.11, 18), exMat));
      const fl = new THREE.Mesh(flangeEx, MAT.darkSteel());
      fl.position.copy(exOutW).addScaledVector(n, 0.03).setZ(i === 0 ? 0 : Z2);
      fl.quaternion.setFromUnitVectors(v3(0, 1, 0), n);
      exhaust.add(fl);
    }
    const shield = new THREE.Mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.9, 24, 1, true), cut(MAT.paint(0x8d9196, 0.5), { cap: false }));
    shield.material.metalness = 0.7;
    shield.position.set(merge.x + 0.49, -1.75, INTER_MID);
    exhaust.add(shield);
    root.add(
      ctx.part(exhaust, {
        id: 'exhaustManifold',
        name: 'Luci e collettore di scarico',
        base: 'Da qui escono i gas bruciati dei due rotori. Sono molto caldi perché nel Wankel hanno poco tempo per espandersi e raffreddarsi.',
        expert:
          'Luci periferiche: apertura anticipata e grande sezione, quindi ottimo riempimento e scarico ma incrocio con l’aspirazione. Gas molto caldi: spesso un reattore termico o un catalizzatore vicino alle luci brucia gli HC residui.',
        explode: [1.0, -0.6, 0.3],
      })
    );

    // ---------- Gas volumes in the three chambers of the front rotor ----------
    const chambers = [0, 1, 2].map(() => {
      const m = new THREE.Mesh(chamberGeometry(), MAT.gas(0x5fd3ff, 0.25, false));
      m.frustumCulled = false;
      m.renderOrder = 2;
      fx.add(m);
      return m;
    });
    const poseChamber = (geo, i, phi) => {
      const psi = phi / 3;
      const c = Math.cos(psi);
      const s = Math.sin(psi);
      const cx = E * Math.cos(phi);
      const cy = E * Math.sin(phi);
      const pos = geo.attributes.position;
      const z = RHALF - 0.01;
      for (let k = 0; k < NCH; k++) {
        const b = ((i + k / (NCH - 1)) * TAU) / 3;
        const [hx, hy] = troch(psi + b);
        const r = rFlank(b);
        const fx0 = r * Math.cos(b);
        const fy0 = r * Math.sin(b);
        const fxw = cx + fx0 * c - fy0 * s;
        const fyw = cy + fx0 * s + fy0 * c;
        pos.setXYZ(k, hx, hy, z);
        pos.setXYZ(NCH + k, fxw, fyw, z);
        pos.setXYZ(2 * NCH + k, hx, hy, -z);
        pos.setXYZ(3 * NCH + k, fxw, fyw, -z);
      }
      pos.needsUpdate = true;
      geo.computeBoundingSphere();
    };

    // ---------- Flows ----------
    const intakeCurve = new THREE.CatmullRomCurve3([
      v3(plenumC.x, plenumC.y + 0.4, 0.8),
      v3(plenumC.x, plenumC.y, 0.45),
      v3(plenumC.x, plenumC.y, -0.2),
      v3(plenumC.x, plenumC.y - 0.1, INTER_MID),
      v3(plenumC.x - 0.02, plenumC.y - 0.35, INTER_MID),
      edgeW.clone().addScaledVector(outW, 0.25),
      edgeW,
      v3(portW.x, portW.y, INTER_MID),
      v3(portW.x, portW.y, -HALF),
      v3(portW.x * 1.04, portW.y * 1.04, -0.12),
    ]);
    const intakeFlow = flow({
      path: (t, out) => intakeCurve.getPointAt(t, out),
      count: 200,
      size: 0.09,
      colors: [0x5fd3ff, 0x5fd3ff, 0x9ad8a0, 0xffc04d],
      spread: 0.07,
      speed: 0.6,
    });
    root.add(intakeFlow.object);
    const exhaustFlows = exhaustCurves.map((cv) => {
      const f = flow({
        path: (t, out) => cv.getPointAt(t, out),
        count: 150,
        size: 0.1,
        colors: [0xff5a1f, 0xb07a6a, 0x8c86a8],
        spread: (t) => (t < 0.1 ? 0.18 : 0.07),
        speed: 0.6,
      });
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
    const exhaustAct = (cycle) => {
      let a = 0;
      for (let i = 0; i < 3; i++) {
        const local = wrap(cycle + i / 3);
        if (local >= 0.75) a += Math.sin((Math.PI * (local - 0.75)) / 0.25);
      }
      return a;
    };
    const rpmOf = (th) => 900 + th * 8100;
    const animRate = (th) => 0.1 + th * 0.35; // rotor turns per second (shaft ×3)

    return {
      root,
      cycleRate: animRate,
      update(state) {
        // shaft angle φ; chamber 1 (apexes 0-1) of the front rotor is at the port-side waist when cycle = 0
        const phi = Math.PI / 2 + state.cycle * 3 * TAU;
        for (const m of [rotor, seals]) {
          m.position.set(E * Math.cos(phi), E * Math.sin(phi), 0);
          m.rotation.z = phi / 3;
        }
        const phi2 = phi + Math.PI; // rear eccentric at 180°
        rotor2.position.set(E * Math.cos(phi2), E * Math.sin(phi2), Z2);
        rotor2.rotation.z = phi2 / 3;
        shaft.rotation.z = phi;
        flywheel.rotation.z = phi;
        fwNut.rotation.z = phi;
        frontCw.rotation.z = phi;
        crankPulley.rotation.z = phi;
        wpPulley.rotation.z = -phi * (0.3 / 0.2);
        altPulley.rotation.z = -phi * (0.3 / 0.11);

        const assembled = (state.explode || 0) < 0.05;
        const rate = animRate(state.throttle);
        let inAct = 0;
        let flash = 0;
        const sparkOn = [false, false];
        chambers.forEach((m, i) => {
          const local = wrap(state.cycle + i / 3);
          poseChamber(m.geometry, i, phi);
          let op;
          if (local < 0.25) {
            const f = local / 0.25;
            cA.copy(C.air).lerp(C.mix, f);
            op = 0.14 + 0.12 * f;
            inAct += Math.sin(Math.PI * f);
          } else if (local < 0.5) {
            const f = (local - 0.25) / 0.25;
            cA.copy(C.mix).lerp(C.fuel, f);
            op = 0.26 + 0.3 * f;
          } else if (local < 0.75) {
            const f = (local - 0.5) / 0.25;
            if (f < 0.2) cA.copy(C.flash).lerp(C.fire, f / 0.2);
            else cA.copy(C.fire).lerp(C.exh, (f - 0.2) / 0.8);
            op = lerp(0.9, 0.35, f);
          } else {
            const f = (local - 0.75) / 0.25;
            cA.copy(C.exh);
            op = lerp(0.35, 0.1, f);
          }
          m.material.color.copy(cA);
          m.material.opacity = op;
          m.visible = assembled;
          if (local > 0.47 && local < 0.51) sparkOn[0] = true; // leading first
          if (local > 0.49 && local < 0.53) sparkOn[1] = true; // trailing a few degrees later
          if (local > 0.48 && local < 0.6) flash = Math.max(flash, 1 - (local - 0.48) / 0.12);
        });
        sparks.forEach((s, k) => {
          s.visible = sparkOn[k] && assembled;
          s.scale.setScalar(sparkOn[k] ? 0.6 + Math.random() * 0.6 : 1);
        });
        sparkLights.forEach((l) => (l.intensity = assembled ? flash * (2 + state.throttle * 4) : 0));

        intakeFlow.object.visible = assembled;
        intakeFlow.material.opacity = clamp(inAct * 1.4, 0, 0.9);
        intakeFlow.update(state.dt, inAct * rate * 3);
        // rear rotor runs 1/6 of a cycle ahead (φ + 180° = half a shaft turn)
        [exhaustAct(state.cycle), exhaustAct(state.cycle + 1 / 6)].forEach((a, i) => {
          const f = exhaustFlows[i];
          f.object.visible = assembled;
          f.material.opacity = clamp(a * 1.4, 0, 0.9);
          f.update(state.dt, a * rate * 3);
        });
      },
      readout(state) {
        const rpm = rpmOf(state.throttle);
        const torque = 2 * (68 + 22 * Math.sin(Math.PI * Math.min(1, state.throttle * 0.85 + 0.15)));
        const kw = (torque * rpm * 2 * Math.PI) / 60 / 1000;
        return [
          { label: 'Giri albero', value: Math.round(rpm / 10) * 10, unit: 'giri/min' },
          { label: 'Giri rotori', value: Math.round(rpm / 30) * 10, unit: 'giri/min' },
          { label: 'Potenza (2 rotori)', value: Math.round(kw), unit: 'kW' },
          { label: 'Scoppi al secondo', value: Math.round((2 * rpm) / 60), unit: '/s' },
        ];
      },
    };
  },
};
