// Diesel-electric locomotive power chain: inline-6 diesel prime mover -> synchronous main
// alternator -> rectifier + DC link + inverter -> nose-suspended traction motors -> gears ->
// wheelsets of one bogie on a scrolling track. Roof resistor grids for dynamic braking.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, pipe, flow, ringX, clamp, smooth } from '../core/helpers.js';

const TAU = Math.PI * 2;
const OFFSET_MODEL = new THREE.Vector3(0.2, -1.35, 0); // centres the model (z stays 0 for the cutaway)
// Prime mover
const CY = 1.8; // crankshaft axis height
const CR = 0.12; // crank radius
const CL = 0.44; // conrod length
const CYL_X = [-2.3, -1.96, -1.62, -1.28, -0.94, -0.6];
const FIRE = [0, 4 / 6, 2 / 6, 5 / 6, 1 / 6, 3 / 6]; // firing order 1-5-3-6-2-4
const HEAD_Y = CY + CR + CL + 0.13;
const TB = [-2.82, 2.25]; // turbocharger centre (x, y), axis along Z
// Running gear
const WR = 0.42; // wheel tread radius = axle height above rail top (y = 0)
const WHEEL_Z = 0.6;
const GAUGE_Z = 0.62;
const AX = [-0.1, 1.9]; // axle positions
const BOGIE_C = (AX[0] + AX[1]) / 2;
const MO = 0.4; // motor-to-axle centre distance
const MX = [AX[0] + MO, AX[1] - MO]; // traction motors, between the axles
const MR = 0.27; // motor radius
const Z_PINION = 15;
const Z_GEAR = 63;
const RATIO = Z_GEAR / Z_PINION;
const R_GEAR = (MO * Z_GEAR) / (Z_GEAR + Z_PINION);
const R_PINION = MO - R_GEAR;
const MODULE = (2 * R_GEAR) / Z_GEAR;
const GEAR_Z = 0.44;
const DECK_Y = 1.2; // top of the underframe
const TRACK_X0 = -3.1;
const TRACK_X1 = 2.7;
const CURRENT = 0xd8ff4a;

const UP = new THREE.Vector3(0, 1, 0);
const V = (x, y, z) => new THREE.Vector3(x, y, z);
const wrap = (c) => ((c % 1) + 1) % 1;
const alongX = (g) => g.rotateZ(-Math.PI / 2); // axis Y -> X
const alongZ = (g) => g.rotateX(Math.PI / 2); // axis Y -> Z

function rod(a, b, r, mat, seg = 10) {
  const va = V(...a);
  const vb = V(...b);
  const dir = vb.clone().sub(va);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, dir.length(), seg), mat);
  m.position.copy(va).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(UP, dir.normalize());
  return m;
}

function box(w, h, d, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  return m;
}

function rbox(w, h, d, r, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 2, r), mat);
  m.position.set(x, y, z);
  return m;
}
const mesh = (g, m, x = 0, y = 0, z = 0) => {
  const o = new THREE.Mesh(g, m);
  o.position.set(x, y, z);
  return o;
};

// Rows of hex bolt heads as one InstancedMesh (axis: 'x' | 'y' | 'z').
const HEX = new THREE.CylinderGeometry(0.02, 0.02, 0.018, 6);
// Hex head + shank of length `len` below it (local -Y), so a bolt backing out shows its shank.
const screw = (len, r = 0.024, h = 0.02) =>
  mergeGeometries([new THREE.CylinderGeometry(r, r, h, 6), new THREE.CylinderGeometry(r * 0.62, r * 0.62, len, 8).translate(0, -h / 2 - len / 2, 0)]);
function bolts(mat, pts, axis = 'y', geo = HEX) {
  const m = new THREE.InstancedMesh(geo, mat, pts.length);
  const q = new THREE.Quaternion();
  if (axis === 'z') q.setFromAxisAngle(V(1, 0, 0), Math.PI / 2);
  if (axis === 'x') q.setFromAxisAngle(V(0, 0, 1), -Math.PI / 2);
  const mtx = new THREE.Matrix4();
  const one = V(1, 1, 1);
  pts.forEach((p, i) => m.setMatrixAt(i, mtx.compose(V(...p), q, one)));
  return m;
}

// Ring of bolt positions around an axis-X flange.
const boltCircle = (x, y, z, r, n) => Array.from({ length: n }, (_, i) => [x, y + Math.cos((i / n) * TAU) * r, z + Math.sin((i / n) * TAU) * r]);

// Clone + cut every material in a group (attachments that must vanish with the section).
function cutAll(group) {
  group.traverse((o) => {
    if (o.material) o.material = cut(o.material.clone());
  });
  return group;
}

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

// Generic warning label (no brands): yellow triangle with a lightning bolt.
function warnTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#f2c230';
  g.beginPath();
  g.moveTo(32, 4);
  g.lineTo(62, 58);
  g.lineTo(2, 58);
  g.closePath();
  g.fill();
  g.strokeStyle = '#111';
  g.lineWidth = 4;
  g.stroke();
  g.fillStyle = '#111';
  g.beginPath();
  g.moveTo(36, 18);
  g.lineTo(24, 38);
  g.lineTo(32, 38);
  g.lineTo(27, 52);
  g.lineTo(41, 31);
  g.lineTo(33, 31);
  g.closePath();
  g.fill();
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

const curve = (pts) => new THREE.CatmullRomCurve3(pts.map((p) => V(...p)), false, 'centripetal');
const tube = (c, r, mat, seg = 48, rad = 10) => new THREE.Mesh(new THREE.TubeGeometry(c, seg, r, rad), mat);

function glowMat(base, color = CURRENT) {
  base.emissive.setHex(color);
  base.emissiveIntensity = 0;
  return base;
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

// Spur gear in the XY plane, extruded along Z (axis = Z).
function gearGeo(teeth, rp, width) {
  const ra = rp + MODULE;
  const rf = rp - 1.25 * MODULE;
  const hp = Math.PI / teeth;
  const s = new THREE.Shape();
  const prof = [[rf, -0.62], [ra, -0.3], [ra, 0.3], [rf, 0.62]];
  for (let k = 0; k < teeth; k++) {
    const c = (TAU * k) / teeth;
    prof.forEach(([r, f], j) => {
      const a = c + f * hp;
      if (k === 0 && j === 0) s.moveTo(r * Math.cos(a), r * Math.sin(a));
      else s.lineTo(r * Math.cos(a), r * Math.sin(a));
    });
  }
  s.closePath();
  const g = new THREE.ExtrudeGeometry(s, { depth: width, bevelEnabled: false, curveSegments: 4 });
  g.translate(0, 0, -width / 2);
  return g;
}

// Gear case: hull of the gear and pinion circles (centres (0,0) and (dx,0)), extruded along Z.
function gearCaseGeo(dx, width) {
  const pts = [];
  for (const [cx, r] of [[0, R_GEAR + MODULE + 0.04], [dx, R_PINION + MODULE + 0.05]]) {
    for (let i = 0; i < 40; i++) {
      const a = (i / 40) * TAU;
      pts.push([cx + Math.cos(a) * r, Math.sin(a) * r]);
    }
  }
  const hull = convexHull(pts);
  const s = new THREE.Shape(hull.map(([x, y]) => new THREE.Vector2(x, y)));
  const g = new THREE.ExtrudeGeometry(s, { depth: width, bevelEnabled: false });
  g.translate(0, 0, -width / 2);
  return g;
}

function springGeo(r, h, turns, wire) {
  const c = new (class extends THREE.Curve {
    getPoint(t, out = new THREE.Vector3()) {
      const a = t * TAU * turns;
      return out.set(Math.cos(a) * r, t * h, Math.sin(a) * r);
    }
  })();
  return new THREE.TubeGeometry(c, turns * 14, wire, 6);
}

// Radial turbo wheel facing +X (hub + blades).
function turboWheel(rOut, len, count, mat) {
  const g = new THREE.Group();
  const hub = lathe([[0, -0.02], [rOut * 0.95, -0.02], [rOut * 0.92, 0], [rOut * 0.42, len * 0.5], [rOut * 0.24, len], [0, len]], 32);
  g.add(new THREE.Mesh(alongX(hub), mat));
  const s = new THREE.Shape();
  s.moveTo(-0.015, 0.015);
  s.lineTo(-0.015, rOut);
  s.lineTo(len * 0.2, rOut * 0.96);
  s.lineTo(len, rOut * 0.6);
  s.lineTo(len, 0.015);
  s.closePath();
  const bg = new THREE.ExtrudeGeometry(s, { depth: 0.006, bevelEnabled: false });
  bg.translate(0, 0, -0.003);
  g.add(ringX(bg, mat, count));
  return g;
}

// Wheel cross-section (r, axial), +axial = outer side, flange on the inner side (from electric-train).
const WHEEL_PROFILE = [
  [0.085, 0.09], [0.085, -0.1], [0.15, -0.1], [0.17, -0.07], [0.25, -0.03], [0.38, 0.0],
  [0.42, -0.02], [0.43, -0.065], [0.53, -0.065], [0.545, -0.045], [0.53, -0.025],
  [0.5, -0.015], [0.49, 0.07], [0.48, 0.075], [0.43, 0.075], [0.4, 0.05], [0.3, 0.04],
  [0.18, 0.06], [0.15, 0.09], [0.085, 0.09],
].map(([r, a]) => [r * 0.84, a * 0.84]);

export default {
  id: 'dieselElectric',
  name: 'Locomotiva diesel-elettrica',
  short: 'Diesel-elettrico',
  family: 'mare',
  vehicle: 'Locomotiva',
  examples: 'Class 66 (EMD), FS D.445 e D.345, Stadler Euro 4000, CZ Loko EffiShunter; navi da crociera e rompighiaccio diesel-elettrici',
  tagline: 'Una centrale elettrica su rotaie: il diesel produce corrente, i motori elettrici spingono le ruote.',
  intro: {
    base:
      'Una locomotiva diesel-elettrica è una piccola <b>centrale elettrica su rotaie</b>. Il grande motore diesel non è collegato alle ruote: fa girare un <b>generatore</b>, e la corrente prodotta arriva ai <b>motori elettrici</b> montati vicino agli assi. Così non servono un cambio gigantesco né una frizione: i motori elettrici spingono con tutta la forza già da fermi, ideale per smuovere treni merci da migliaia di tonnellate.',
    expert:
      'Trasmissione elettrica: motore primo diesel (2 000–3 300 kW a ~900–1 000 giri/min) → alternatore sincrono → raddrizzatore → circuito intermedio in continua → inverter VVVF → motori di trazione asincroni trifase (nelle macchine più vecchie, come la Class 66, motori in corrente continua). Rendimento termico del diesel ~40%, della catena elettrica ~85–90%: complessivo ~35% al cerchione. Il diesel lavora a giri e potenza fissati dal manipolatore mentre l’elettronica scambia forza e velocità lungo tutta la curva, come un cambio continuo. In frenatura i motori diventano generatori e l’energia si dissipa nei reostati sul tetto (frenatura dinamica). Lo stesso schema muove navi da crociera, rompighiaccio e traghetti ibridi.',
  },
  specs: [
    { label: 'Motore primo', value: 'Diesel V12–V16 o 6 in linea, 2 o 4 tempi, sovralimentato' },
    { label: 'Potenza diesel', value: '2 000 – 3 300 kW (Class 66: 2 460 kW)' },
    { label: 'Regime', value: '≈ 900 – 1 000 giri/min (minimo ≈ 300)' },
    { label: 'Trasmissione', value: 'Elettrica: alternatore → raddrizzatore → inverter' },
    { label: 'Motori di trazione', value: '4 – 6, asincroni trifase (c.c. nelle più vecchie)' },
    { label: 'Distribuzione (motore diesel)', value: 'Ingranaggi' },
    { label: 'Sforzo all’avviamento', value: '300 – 500 kN (Class 66: ≈ 409 kN)' },
    { label: 'Rendimento complessivo', value: '≈ 35 % (diesel ≈ 40 %)' },
  ],
  metrics: { efficiency: 35, power: 2500, powerDensity: 0.12 },
  variants: {
    columns: ['Locomotiva', 'Motore diesel', 'Potenza', 'Trazione'],
    rows: [
      ['Class 66 (EMD JT42CWR)', 'EMD 12-710, V12 2 tempi', '2 460 kW', 'Alternatore + raddrizzatore, motori c.c.'],
      ['Stadler Euro 4000', 'EMD 16-710, V16 2 tempi', '3 178 kW', 'Inverter, motori asincroni'],
      ['EMD SD70ACe', 'EMD 16-710, V16 2 tempi', '≈ 3 200 kW', 'Inverter, motori asincroni'],
      ['GE ES44AC (Evolution)', 'GEVO-12, V12 4 tempi', '≈ 3 280 kW', 'Inverter, motori asincroni'],
    ],
    note: 'Potenza nominale del motore diesel. Il modello 3D mostra un motore a 6 cilindri in linea per chiarezza.',
  },
  thumb: { orbit: [-28, 16], zoom: 1.25, view: 'section' },
  phases: [
    {
      id: 'diesel',
      name: 'Motore diesel',
      parts: ['primeMover', 'engineCrank', 'enginePistons', 'engineTurbo', 'fuelTank'],
      base:
        'Il cuore della locomotiva è un grande motore diesel, pesante come una decina di automobili. Funziona come quello di un camion, ma in grande: aria compressa e rovente, gasolio iniettato, combustione. Però non è collegato alle ruote: il suo unico lavoro è far girare il generatore, lentamente e sempre nel suo regime migliore.',
      expert:
        'Motore primo sovralimentato (EMD 710 a 2 tempi, GE/Wabtec, Caterpillar, MTU a 4 tempi), 2 000–3 300 kW a 900–1 000 giri/min, minimo ~300 giri/min, rendimento termico ~40%. Il macchinista agisce sul manipolatore (8 “tacche” nelle macchine di scuola americana): la centralina fissa giri e potenza del diesel, che resta così in pochi punti di funzionamento ottimali qualunque sia la velocità del treno. Serbatoio da 5 000–7 000 litri.',
    },
    {
      id: 'generatore',
      name: 'Generatore',
      parts: ['alternatorStator', 'alternatorRotor'],
      base:
        'Attaccato direttamente all’albero del diesel c’è un enorme generatore: una “dinamo” grande come un’auto. Il rotore con le sue bobine diventa una calamita che gira dentro gli avvolgimenti di rame fermi, e in quei fili nasce la corrente elettrica.',
      expert:
        'Alternatore sincrono trifase a poli salienti calettato sull’albero motore. La corrente di eccitazione (via anelli e spazzole o eccitatrice brushless) regola la tensione d’uscita, fino a ~1 500 V: così la centralina controlla la potenza senza cambiare i giri del diesel. Rendimento 95–97%. Un alternatore ausiliario alimenta ventilatori, compressore e servizi del treno.',
    },
    {
      id: 'elettronica',
      name: 'Elettronica di potenza',
      parts: ['powerElectronics', 'cables'],
      base:
        'La corrente del generatore va nell’armadio elettronico. Prima viene raddrizzata, cioè resa continua; poi l’inverter, un interruttore velocissimo, la ricompone in una corrente “su misura” per i motori: poca frequenza alla partenza, tanta ad alta velocità. È questo che sostituisce il cambio.',
      expert:
        'Raddrizzatore a diodi (o attivo) → circuito intermedio in continua con condensatori di filtro (DC link) → inverter VVVF a IGBT per ogni motore o carrello, con controllo vettoriale della coppia. La potenza del diesel resta costante e l’inverter trasforma lo stesso numero di kW in grande forza a bassa velocità o poca forza ad alta velocità, senza salti: una trasmissione a variazione continua.',
    },
    {
      id: 'motori',
      name: 'Motori di trazione',
      parts: ['tractionMotors', 'motorRotors', 'gears'],
      base:
        'Su ogni asse c’è un motore elettrico che, tramite un piccolo ingranaggio, fa girare una ruota dentata grande montata sull’asse. Un motore elettrico dà la sua forza massima già da fermo: niente frizione che slitta e niente marce da cambiare.',
      expert:
        'Motori “sospesi per il naso”: da un lato poggiano sull’asse con cuscinetti, dall’altro sulla traversa del carrello. Riduzione a ingranaggi cilindrici ~4–5:1 (qui 63/15). Motori asincroni trifase da 300–600 kW ciascuno, con coppia massima da velocità zero; nelle macchine più vecchie motori in serie a corrente continua, alimentati direttamente dal raddrizzatore.',
    },
    {
      id: 'ruote',
      name: 'Ruote e aderenza',
      parts: ['wheelsets', 'bogie', 'track', 'brakeResistors'],
      base:
        'Tutta la forza deve passare da un contatto acciaio su acciaio grande come un’unghia: se si esagera, le ruote slittano. Per questo l’elettronica dosa la spinta di continuo. In frenata i motori diventano generatori e l’energia del treno si trasforma in calore nelle griglie sul tetto: riduci il gas e le vedrai arrossarsi.',
      expert:
        'Lo sforzo di trazione è limitato dall’aderenza ruota-rotaia (μ ≈ 0,25–0,35 su rotaia asciutta, con sabbiatura e antislittamento elettronico fino a ~0,4): con 126 t sulle ruote la Class 66 parte con ~409 kN. Frenatura dinamica reostatica: i motori lavorano da generatori e le griglie di resistenze, raffreddate da ventilatori, dissipano 2–4 MW senza consumare ceppi e dischi. Sulle navi e sulle locomotive ibride l’energia può invece andare in batteria.',
    },
  ],
  quiz: [
    {
      q: 'Perché una locomotiva diesel-elettrica usa un generatore e motori elettrici invece di un cambio meccanico?',
      options: [
        'Perché il gasolio brucia meglio se c’è elettricità',
        'Perché i motori elettrici danno la forza massima già da fermi e non serve un enorme cambio con frizione',
        'Perché così il motore diesel può spegnersi in salita',
        'Perché le rotaie conducono la corrente',
      ],
      correct: 1,
      explain:
        'Un cambio con frizione capace di avviare migliaia di tonnellate sarebbe enorme e si consumerebbe. I motori elettrici hanno coppia massima da zero giri e l’elettronica regola forza e velocità in modo continuo.',
    },
    {
      q: 'Che cosa fa il motore diesel in una locomotiva diesel-elettrica?',
      options: ['Spinge direttamente le ruote', 'Comprime l’aria per i freni e basta', 'Fa girare un generatore che produce corrente', 'Ricarica la catenaria'],
      correct: 2,
      explain: 'Il diesel non tocca le ruote: fa girare l’alternatore. La corrente prodotta alimenta i motori di trazione sugli assi.',
    },
    {
      q: 'Dove finisce l’energia del treno nella frenatura dinamica reostatica?',
      options: ['Torna nel serbatoio come gasolio', 'Diventa calore nelle griglie di resistenze sul tetto', 'Va nella catenaria', 'Si perde nei ceppi dei freni'],
      correct: 1,
      explain:
        'I motori diventano generatori e frenano il treno; la corrente prodotta scalda le resistenze sul tetto, raffreddate da grandi ventilatori. Ceppi e dischi si consumano molto meno.',
    },
    {
      q: 'Circa quanta energia del gasolio arriva davvero alle ruote?',
      options: ['Circa il 10%', 'Circa il 35%', 'Circa il 70%', 'Circa il 95%'],
      correct: 1,
      explain:
        'Il diesel trasforma in lavoro ~40% dell’energia del gasolio; generatore, elettronica e motori perdono un altro 10–15%. Al cerchione arriva circa il 35%.',
    },
  ],
  camera: { position: [-2.6, 2.6, 7.8], target: [0, 0.2, 0] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.copy(OFFSET_MODEL);
    root.add(model);
    const add = (obj, opts) => model.add(ctx.part(obj, opts));

    // ================= Track =================
    const track = new THREE.Group();
    const railShape = new THREE.Shape();
    [
      [-0.065, 0], [0.065, 0], [0.065, 0.012], [0.015, 0.024], [0.011, 0.09], [0.034, 0.098],
      [0.035, 0.143], [-0.035, 0.143], [-0.034, 0.098], [-0.011, 0.09], [-0.015, 0.024], [-0.065, 0.012],
    ].forEach(([u, v], i) => (i ? railShape.lineTo(u, v) : railShape.moveTo(u, v)));
    railShape.closePath();
    const LEN = TRACK_X1 - TRACK_X0;
    const railGeo = new THREE.ExtrudeGeometry(railShape, { depth: LEN, bevelEnabled: false });
    railGeo.rotateY(Math.PI / 2); // extrusion along +X
    railGeo.translate(TRACK_X0, -0.143, 0);
    const railMat = MAT.darkSteel();
    const railHeadMat = MAT.chrome();
    for (const sz of [-1, 1]) {
      const r = new THREE.Mesh(railGeo, railMat);
      r.position.z = sz * GAUGE_Z;
      track.add(r, box(LEN, 0.004, 0.045, railHeadMat, (TRACK_X0 + TRACK_X1) / 2, 0.001, sz * GAUGE_Z));
    }
    track.add(box(LEN + 0.3, 0.12, 2.3, MAT.paint(0x5e5a53, 0.95), (TRACK_X0 + TRACK_X1) / 2, -0.31, 0));
    const sleepers = [];
    const slMat = MAT.paint(0x9b978e, 0.9);
    const clipMat = MAT.darkSteel();
    const SL_N = 10;
    const SL_P = 6.0; // wrap period (SL_N * 0.6)
    for (let i = 0; i < SL_N; i++) {
      const g = new THREE.Group();
      g.add(box(0.24, 0.12, 1.75, slMat, 0, -0.203, 0));
      for (const sz of [-1, 1]) for (const o of [-1, 1]) g.add(box(0.1, 0.03, 0.04, clipMat, 0, -0.138, sz * GAUGE_Z + o * 0.085));
      g.userData.x0 = i * 0.6;
      sleepers.push(g);
      track.add(g);
    }
    add(track, {
      id: 'track',
      name: 'Binario',
      base: 'Due rotaie d’acciaio fissate alle traversine. Acciaio su acciaio fa pochissimo attrito: un treno consuma poco, ma la spinta va dosata per non far slittare le ruote.',
      expert:
        'Rotaie 60 E1 su traversine in calcestruzzo ogni ~60 cm, scartamento 1 435 mm. Il contatto ruota-rotaia è un’ellisse di circa 1 cm² per ruota: tutta la forza di trazione passa da lì, ed è l’aderenza (μ) a limitarla.',
      explode: [0, -0.6, 0],
    });

    // ================= Wheelsets =================
    const wheelsets = new THREE.Group();
    const wheelLathe = lathe(WHEEL_PROFILE, 64);
    const wheelGeoPos = alongZ(wheelLathe.clone()); // outer side toward +Z
    const wheelGeoNeg = wheelLathe.clone().rotateX(-Math.PI / 2); // outer side toward -Z
    const wheelMat = MAT.steel();
    const axleMat = MAT.darkSteel();
    const spokeMat = MAT.paint(0xa33b2a, 0.5);
    const wheelSpins = AX.map((x) => {
      const spin = new THREE.Group();
      spin.position.set(x, WR, 0);
      spin.add(new THREE.Mesh(alongZ(new THREE.CylinderGeometry(0.075, 0.075, 1.62, 24)), axleMat));
      for (const sz of [-1, 1]) {
        const w = new THREE.Mesh(sz > 0 ? wheelGeoPos : wheelGeoNeg, wheelMat);
        w.position.z = sz * WHEEL_Z;
        spin.add(w);
        // painted web mark, makes the rotation readable
        const mark = box(0.05, 0.22, 0.012, spokeMat, 0, 0.2, sz * (WHEEL_Z + 0.06));
        spin.add(mark);
      }
      wheelsets.add(spin);
      return spin;
    });
    add(wheelsets, {
      id: 'wheelsets',
      name: 'Sale montate (assi e ruote)',
      base: 'Due ruote d’acciaio unite rigidamente dallo stesso asse. Il bordino all’interno della ruota la tiene sul binario.',
      expert:
        'Ruote monoblocco da ~1 m di diametro calettate a pressione sull’asse; profilo conico che autocentra la sala. Una locomotiva Co’Co’ come la Class 66 ha due carrelli a tre assi, tutti motorizzati; qui è rappresentato un carrello a due assi (Bo).',
      explode: [0, -0.35, 0.2],
    });

    // ================= Gears and gear cases =================
    const gears = new THREE.Group();
    const bigGearGeo = gearGeo(Z_GEAR, R_GEAR, 0.05);
    const pinionGeo = gearGeo(Z_PINION, R_PINION, 0.06);
    const gearMat = MAT.darkSteel();
    const pinionMat = MAT.steel();
    const caseMat = cut(MAT.paint(0x3c4550, 0.6));
    const bigGears = [];
    const pinions = [];
    AX.forEach((x, i) => {
      const g = new THREE.Mesh(bigGearGeo, gearMat);
      g.position.set(x, WR, GEAR_Z);
      const p = new THREE.Mesh(pinionGeo, pinionMat);
      p.position.set(MX[i], WR, GEAR_Z);
      const dx = MX[i] - x;
      const gc = new THREE.Mesh(gearCaseGeo(dx, 0.11), caseMat);
      gc.position.set(x, WR, GEAR_Z);
      gears.add(g, p, gc);
      bigGears.push(g);
      pinions.push(p);
    });
    add(gears, {
      id: 'gears',
      name: 'Ingranaggi di riduzione',
      base: 'Un ingranaggio piccolo (pignone) sul motore fa girare una ruota dentata grande sull’asse: la rotazione rallenta di circa quattro volte e la forza aumenta altrettanto.',
      expert:
        'Coppia di ingranaggi cilindrici in bagno d’olio, qui 15/63 denti (i = 4,2); nelle locomotive merci il rapporto va da ~4 a ~5,5 (più alto = più forza, meno velocità massima). Il carter è una scatola stagna fissata al motore.',
      explode: [0, 0, 1.6],
    });

    // ================= Traction motors (housings, stators) =================
    const motors = new THREE.Group();
    const motorMat = cut(MAT.paint(0x2f3a46, 0.55));
    const statorMat = cut(MAT.darkSteel());
    const windMat = cut(MAT.copper());
    const MZ0 = -0.45;
    const MZ1 = 0.36;
    const mLen = MZ1 - MZ0;
    const mzc = (MZ0 + MZ1) / 2;
    MX.forEach((x, i) => {
      const g = new THREE.Group();
      g.position.set(x, WR, 0);
      const shell = new THREE.Mesh(alongZ(pipe(MR, MR - 0.03, mLen, 48)), motorMat);
      shell.position.z = mzc;
      g.add(shell);
      for (const z of [MZ0, MZ1]) {
        const cap = new THREE.Mesh(alongZ(pipe(MR + 0.01, 0.05, 0.03, 48)), motorMat);
        cap.position.z = z;
        g.add(cap);
      }
      const stator = new THREE.Mesh(alongZ(pipe(MR - 0.03, 0.17, mLen - 0.2, 48)), statorMat);
      stator.position.z = mzc;
      g.add(stator);
      for (const z of [MZ0 + 0.07, MZ1 - 0.07]) {
        const ew = new THREE.Mesh(new THREE.TorusGeometry(0.2, 0.035, 8, 32), windMat);
        ew.position.z = z;
        g.add(ew);
      }
      g.add(box(0.14, 0.06, 0.14, motorMat, 0, MR + 0.02, -0.2)); // terminal box
      // nose lug toward the bogie transom, axle-suspension lugs toward the axle
      const toC = Math.sign(BOGIE_C - x);
      g.add(box(0.22, 0.08, 0.3, motorMat, toC * (MR + 0.08), 0.12, mzc));
      g.add(box(0.2, 0.1, 0.12, motorMat, -toC * (MR - 0.02), 0, -0.3));
      g.add(box(0.2, 0.1, 0.12, motorMat, -toC * (MR - 0.02), 0, 0.25));
      motors.add(g);
    });
    add(motors, {
      id: 'tractionMotors',
      name: 'Motori di trazione',
      base: 'I motori elettrici che fanno girare le ruote, uno per asse. Sono appesi da un lato all’asse e dall’altro al telaio del carrello. Da fermi danno già tutta la loro forza.',
      expert:
        'Motori asincroni trifase autoventilati o a ventilazione forzata, 300–600 kW ciascuno, sospesi “per il naso”: metà del peso grava sull’asse (massa non sospesa elevata, accettabile a 100–120 km/h). Nelle macchine più vecchie motori a corrente continua con eccitazione in serie, alimentati dal raddrizzatore.',
      explode: [0, -0.1, 1.2],
    });

    // ================= Traction motor rotors (spin) =================
    const motorRotors = new THREE.Group();
    const rotorCoreMat = MAT.steel();
    const barMat = glowMat(MAT.copper());
    const shaftMat = MAT.steel();
    const motorBars = [];
    const motorSpins = MX.map((x) => {
      const spin = new THREE.Group();
      spin.position.set(x, WR, 0);
      const core = new THREE.Mesh(alongZ(new THREE.CylinderGeometry(0.15, 0.15, mLen - 0.22, 32)), rotorCoreMat);
      core.position.z = mzc;
      spin.add(core);
      for (let k = 0; k < 14; k++) {
        const a = (k / 14) * TAU;
        const b = box(0.02, 0.018, mLen - 0.18, barMat, Math.cos(a) * 0.15, Math.sin(a) * 0.15, mzc);
        b.rotation.z = a;
        spin.add(b);
        motorBars.push(b);
      }
      for (const z of [MZ0 + 0.1, MZ1 - 0.1]) {
        const ring = new THREE.Mesh(alongZ(pipe(0.16, 0.1, 0.03, 32)), barMat);
        ring.position.z = z;
        spin.add(ring);
        motorBars.push(ring);
      }
      const shaft = new THREE.Mesh(alongZ(new THREE.CylinderGeometry(0.04, 0.04, GEAR_Z - MZ0 + 0.1, 16)), shaftMat);
      shaft.position.z = (GEAR_Z + MZ0) / 2;
      spin.add(shaft);
      motorRotors.add(spin);
      return spin;
    });
    add(motorRotors, {
      id: 'motorRotors',
      name: 'Rotori dei motori',
      base: 'La parte che gira dentro ogni motore. Il campo magnetico creato dalla corrente dell’inverter la trascina; il pignone sull’albero passa il movimento all’asse.',
      expert:
        'Rotore a gabbia di scoiattolo: barre di rame cortocircuitate da anelli, nessun contatto strisciante. Il motore gira circa 4–5 volte più veloce delle ruote; le barre si illuminano in proporzione alla corrente indotta.',
      explode: [0, -0.1, 2.0],
    });

    // ================= Bogie frame and springs (cut) =================
    const bogie = new THREE.Group();
    const frameMat = cut(MAT.paint(0x3a3f46, 0.55));
    const springMat = cut(MAT.paint(0x8a2d22, 0.45));
    const fx0 = AX[0] - 0.55;
    const fx1 = AX[1] + 0.55;
    const sprPrim = springGeo(0.045, 0.18, 4, 0.012);
    const sprSec = springGeo(0.09, 0.27, 5, 0.018);
    for (const sz of [-1, 1]) {
      const z = sz * 0.86;
      bogie.add(box(fx1 - fx0, 0.18, 0.08, frameMat, (fx0 + fx1) / 2, 0.71, z));
      for (const x of AX) {
        bogie.add(new THREE.Mesh(alongZ(new THREE.CylinderGeometry(0.11, 0.11, 0.14, 24)), frameMat).translateX(x).translateY(WR).translateZ(sz * 0.77));
        bogie.add(box(0.36, 0.05, 0.16, frameMat, x, WR + 0.03, sz * 0.8));
        for (const dx of [-0.15, 0.15]) {
          const s = new THREE.Mesh(sprPrim, springMat);
          s.position.set(x + dx, WR + 0.06, sz * 0.8);
          bogie.add(s);
        }
        bogie.add(box(0.06, 0.24, 0.1, frameMat, x - 0.2, 0.52, z), box(0.06, 0.24, 0.1, frameMat, x + 0.2, 0.52, z));
      }
      for (const dx of [-0.6, 0.6]) {
        const s = new THREE.Mesh(sprSec, springMat);
        s.position.set(BOGIE_C + dx, 0.8, z);
        bogie.add(s);
      }
    }
    bogie.add(box(0.3, 0.16, 1.66, frameMat, BOGIE_C, 0.7, 0)); // transom
    // tread brake units (cylinder + block on the outer side of each wheel), sandboxes with pipes
    const brakeBlockMat = cut(MAT.paint(0x2a2c2f, 0.8));
    AX.forEach((x, i) => {
      const out = i === 0 ? -1 : 1;
      for (const sz of [-1, 1]) {
        bogie.add(rbox(0.05, 0.2, 0.1, 0.01, brakeBlockMat, x + out * (WR + 0.035), WR, sz * WHEEL_Z));
        bogie.add(rbox(0.12, 0.12, 0.12, 0.02, frameMat, x + out * (WR + 0.13), WR + 0.04, sz * WHEEL_Z));
        bogie.add(rbox(0.14, 0.16, 0.14, 0.02, frameMat, x + out * (WR + 0.1), 0.88, sz * 0.86));
        bogie.add(rod([x + out * (WR + 0.1), 0.8, sz * 0.86], [x + out * (WR + 0.03), 0.06, sz * GAUGE_Z], 0.012, frameMat, 6));
      }
    });
    const boxBolts = [];
    for (const x of AX) for (const sz of [-1, 1]) for (let k = 0; k < 6; k++) boxBolts.push([x + Math.cos((k / 6) * TAU) * 0.075, WR + Math.sin((k / 6) * TAU) * 0.075, sz * 0.845]);
    bogie.add(bolts(cut(MAT.steel()), boxBolts, 'z'));
    for (const x of [fx0 + 0.06, fx1 - 0.06]) bogie.add(box(0.12, 0.14, 1.66, frameMat, x, 0.72, 0)); // end beams
    add(bogie, {
      id: 'bogie',
      name: 'Carrello',
      base: 'Il telaio con le ruote su cui poggia la locomotiva. Le molle rosse assorbono le irregolarità del binario; il carrello può ruotare per seguire le curve.',
      expert:
        'Telaio saldato con sospensione primaria (molle sulle boccole) e secondaria (molle flexicoil tra carrello e cassa). La forza di trazione passa alla cassa tramite un perno o barre di trazione; la sospensione secondaria permette la rotazione in curva.',
      explode: [0, -0.15, 0.7],
    });

    // ================= Underframe (deck, buffer beams) =================
    const frame = new THREE.Group();
    const deckMat = MAT.paint(0x39414b, 0.6);
    const deckX0 = -3.0;
    const deckX1 = 2.55;
    frame.add(box(deckX1 - deckX0, 0.15, 1.9, deckMat, (deckX0 + deckX1) / 2, DECK_Y - 0.075, 0));
    for (const sz of [-1, 1]) frame.add(box(deckX1 - deckX0, 0.1, 0.06, deckMat, (deckX0 + deckX1) / 2, DECK_Y - 0.2, sz * 0.92));
    const beamMat = MAT.paint(0xc8a22a, 0.5);
    const bufMat = MAT.darkSteel();
    for (const [x, s] of [[deckX0, -1], [deckX1, 1]]) {
      frame.add(box(0.1, 0.35, 1.9, beamMat, x + s * 0.05, DECK_Y - 0.1, 0));
      for (const sz of [-1, 1]) {
        const buf = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.06, 0.06, 0.16, 16)), bufMat);
        buf.position.set(x + s * 0.18, DECK_Y - 0.12, sz * 0.62);
        const plate = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.13, 0.13, 0.025, 24)), bufMat);
        plate.position.set(x + s * 0.27, DECK_Y - 0.12, sz * 0.62);
        frame.add(buf, plate);
      }
    }
    const rails = new THREE.Group();
    const railM = MAT.paint(0xd9b52a, 0.45);
    for (const sz of [-1, 1]) {
      const z = sz * 0.9;
      for (let x = deckX0 + 0.15; x <= deckX1 - 0.1; x += 0.62) rails.add(rod([x, DECK_Y, z], [x, DECK_Y + 0.85, z], 0.014, railM, 8));
      for (const y of [DECK_Y + 0.45, DECK_Y + 0.85]) rails.add(rod([deckX0 + 0.15, y, z], [deckX1 - 0.12, y, z], 0.013, railM, 8));
    }
    frame.add(cutAll(rails));
    add(frame, {
      id: 'underframe',
      name: 'Telaio della cassa',
      base: 'Il “pavimento” robusto della locomotiva: regge motore, generatore e armadi e trasmette la spinta ai vagoni attraverso i respingenti e il gancio.',
      expert:
        'Telaio in acciaio saldato con longheroni e traverse di testata che portano respingenti e organi di aggancio; deve resistere a compressioni longitudinali di oltre 2 000 kN. Poggia sui carrelli tramite la sospensione secondaria.',
      explode: [0, 0, 0],
    });

    // ================= Fuel tank =================
    const tank = new THREE.Group();
    const tankMat = cut(MAT.paint(0x2b3138, 0.6));
    const tx0 = -2.6;
    const tx1 = -0.78;
    const tankRing = rrPath(new THREE.Shape(), tx0, -0.7, tx1, 0.7, 0.08);
    tankRing.holes.push(rrPath(new THREE.Path(), tx0 + 0.04, -0.66, tx1 - 0.04, 0.66, 0.05));
    tank.add(new THREE.Mesh(new THREE.ExtrudeGeometry(tankRing, { depth: 0.5, bevelEnabled: false }).rotateX(-Math.PI / 2).translate(0, 0.48, 0), tankMat));
    for (const y of [0.46, 1.0]) tank.add(rbox(tx1 - tx0, 0.04, 1.4, 0.015, tankMat, (tx0 + tx1) / 2, y, 0));
    for (const x of [-2.1, -1.3]) tank.add(rbox(0.05, 0.62, 1.44, 0.015, tankMat, x, 0.73, 0)); // straps
    const tankFront = new THREE.Group();
    tankFront.add(mesh(alongZ(new THREE.CylinderGeometry(0.07, 0.07, 0.05, 20)), MAT.paint(0xc8a22a, 0.5), -2.35, 0.86, 0.72));
    tankFront.add(rbox(0.06, 0.34, 0.03, 0.01, MAT.chrome(), -1.0, 0.72, 0.715));
    tank.add(cutAll(tankFront));
    const fuelLine = curve([[-1.4, 1.02, -0.6], [-1.4, 1.3, -0.6], [-1.42, 1.75, -0.48], [-1.45, 2.05, -0.36]]);
    tank.add(tube(fuelLine, 0.018, MAT.steel(), 24, 8));
    add(tank, {
      id: 'fuelTank',
      name: 'Serbatoio del gasolio',
      base: 'Sotto il pavimento, tra i carrelli, c’è un serbatoio enorme: migliaia di litri di gasolio, abbastanza per viaggiare per giorni.',
      expert:
        'Serbatoio da 5 000–7 500 litri (Class 66: ~6 400 l). Consumo specifico del motore primo ~200–210 g/kWh a pieno carico; a 2,5 MW si bruciano circa 600 litri l’ora.',
      explode: [0, -0.25, 1.3],
    });
    const fuelVol = box(tx1 - tx0 - 0.1, 0.34, 1.3, cut(MAT.gas(0xffc04d, 0.22, false), { cap: false }), (tx0 + tx1) / 2, 0.65, 0);
    fuelVol.renderOrder = 2;
    model.add(fuelVol);

    // ================= Prime mover: block, crankcase, heads (cut) =================
    const engine = new THREE.Group();
    const ex0 = -2.48;
    const ex1 = -0.42;
    const blockShape = new THREE.Shape();
    blockShape.moveTo(ex0, -0.3);
    blockShape.lineTo(ex1, -0.3);
    blockShape.lineTo(ex1, 0.3);
    blockShape.lineTo(ex0, 0.3);
    blockShape.lineTo(ex0, -0.3);
    for (const x of CYL_X) {
      const h = new THREE.Path();
      h.absarc(x, 0, 0.136, 0, TAU, true);
      blockShape.holes.push(h);
    }
    const blockH = HEAD_Y - 1.98;
    const blockGeo = new THREE.ExtrudeGeometry(blockShape, { depth: blockH, bevelEnabled: false, curveSegments: 28 });
    blockGeo.rotateX(-Math.PI / 2);
    blockGeo.translate(0, 1.98, 0);
    const blockMat = cut(MAT.paint(0x4f6a5a, 0.55));
    engine.add(new THREE.Mesh(blockGeo, blockMat));
    const linerMat = cut(MAT.steel());
    for (const x of CYL_X) {
      const l = new THREE.Mesh(pipe(0.138, 0.132, blockH, 32), linerMat);
      l.position.set(x, 1.98 + blockH / 2, 0);
      engine.add(l);
    }
    const ccShape = new THREE.Shape();
    ccShape.moveTo(ex0 - 0.07, -0.36);
    ccShape.lineTo(ex1, -0.36);
    ccShape.lineTo(ex1, 0.36);
    ccShape.lineTo(ex0 - 0.07, 0.36);
    ccShape.lineTo(ex0 - 0.07, -0.36);
    const ccHole = new THREE.Path();
    ccHole.moveTo(ex0 - 0.02, -0.31);
    ccHole.lineTo(ex0 - 0.02, 0.31);
    ccHole.lineTo(ex1 - 0.05, 0.31);
    ccHole.lineTo(ex1 - 0.05, -0.31);
    ccHole.lineTo(ex0 - 0.02, -0.31);
    ccShape.holes.push(ccHole);
    const ccGeo = new THREE.ExtrudeGeometry(ccShape, { depth: 1.98 - DECK_Y - 0.1, bevelEnabled: false });
    ccGeo.rotateX(-Math.PI / 2);
    ccGeo.translate(0, DECK_Y + 0.1, 0);
    engine.add(new THREE.Mesh(ccGeo, blockMat));
    engine.add(box(ex1 - ex0, 0.1, 0.62, cut(MAT.darkSteel()), (ex0 + ex1) / 2, DECK_Y + 0.05, 0)); // oil sump
    const headMat = cut(MAT.paint(0x5d7a69, 0.5));
    const coverMat = cut(MAT.paint(0x26302b, 0.55));
    const eBolt = cut(MAT.steel());
    const topBolts = [];
    for (const x of CYL_X) {
      engine.add(rbox(0.31, 0.17, 0.62, 0.025, headMat, x, HEAD_Y + 0.085, 0));
      engine.add(rbox(0.27, 0.1, 0.5, 0.03, coverMat, x, HEAD_Y + 0.22, 0));
      engine.add(rbox(0.29, 0.012, 0.6, 0.004, cut(MAT.paint(0x1d2024, 0.8)), x, HEAD_Y + 0.172, 0)); // gasket line
      for (const dx of [-0.1, 0.1]) for (const dz of [-0.2, 0.2]) topBolts.push([x + dx, HEAD_Y + 0.275, dz]);
    }
    engine.add(bolts(eBolt, topBolts, 'y'));
    // main-bearing bulkheads between the throws (the section shows the bearing bores)
    const bulk = rrPath(new THREE.Shape(), -0.31, DECK_Y + 0.1, 0.31, 1.98, 0.02);
    bulk.holes.push(new THREE.Path().absarc(0, CY, 0.068, 0, TAU, true));
    const bulkGeo = new THREE.ExtrudeGeometry(bulk, { depth: 0.06, bevelEnabled: false, curveSegments: 24 }).rotateY(Math.PI / 2);
    const shellMat = MAT.paint(0xb08d57, 0.35);
    shellMat.metalness = 0.9;
    for (let k = 0; k < CYL_X.length - 1; k++) {
      const xm = (CYL_X[k] + CYL_X[k + 1]) / 2;
      engine.add(mesh(bulkGeo, blockMat, xm - 0.03, 0, 0));
      engine.add(mesh(alongX(pipe(0.068, 0.061, 0.06, 24)), shellMat, xm, CY, 0));
    }
    // crankcase inspection doors with bolts, ribs, mounting feet, injection lines
    const doors = new THREE.Group();
    const doorBolts = [];
    for (const x of CYL_X) {
      for (const sz of [-1, 1]) {
        doors.add(rbox(0.24, 0.32, 0.03, 0.03, coverMat, x, 1.64, sz * 0.375));
        for (const dx of [-0.09, 0.09]) for (const dy of [-0.13, 0, 0.13]) doorBolts.push([x + dx, 1.64 + dy, sz * 0.395]);
      }
    }
    doors.add(bolts(eBolt, doorBolts, 'z'));
    for (let k = 0; k < CYL_X.length - 1; k++) for (const sz of [-1, 1]) doors.add(rbox(0.04, 0.5, 0.05, 0.012, blockMat, (CYL_X[k] + CYL_X[k + 1]) / 2, 2.22, sz * 0.315));
    for (const x of [ex0 + 0.12, ex1 - 0.15]) for (const sz of [-1, 1]) doors.add(rbox(0.22, 0.12, 0.16, 0.02, blockMat, x, DECK_Y + 0.06, sz * 0.43));
    doors.add(rbox(ex1 - ex0 - 0.1, 0.12, 0.08, 0.03, blockMat, (ex0 + ex1) / 2, 2.3, -0.36));
    for (const x of CYL_X) doors.add(rod([x - 0.06, 2.36, -0.37], [x - 0.06, HEAD_Y + 0.1, -0.32], 0.01, MAT.steel(), 6));
    engine.add(cutAll(doors));
    // flywheel housing plate: the alternator's drive-end adapter bolts onto it
    engine.add(mesh(alongX(pipe(0.595, 0.31, 0.035, 64)), blockMat, ex1 - 0.0025, CY, 0));
    const mountBolts = [];
    for (const x of [ex0 + 0.12, ex1 - 0.15]) for (const dx of [-0.06, 0.06]) for (const sz of [-1, 1]) mountBolts.push([x + dx, DECK_Y + 0.1325, sz * 0.46]);
    add(bolts(eBolt, mountBolts, 'y', screw(0.2, 0.028, 0.025)), {
      id: 'engineMountBolts',
      name: 'Bulloni dei supporti del motore',
      base: 'Tengono i piedi del motore diesel fissati al telaio della locomotiva.',
      expert:
        'Il motore primo poggia sul telaio in pochi punti; spesso tra piede e telaio ci sono supporti elastici che filtrano le vibrazioni, e spessori che servono ad allineare il gruppo motore-alternatore.',
      explode: [0, 0.7, 0],
    });
    add(engine, {
      id: 'primeMover',
      name: 'Motore diesel (motore primo)',
      base: 'Il grande motore diesel della locomotiva, con sei cilindri in fila (nelle locomotive di linea spesso 12 o 16 a V). Funziona come il diesel di un camion, ma è molto più grande e gira più piano.',
      expert:
        'Motore primo sovralimentato: alesaggi di 170–250 mm e corse di 200–300 mm, cilindrate da 50 a oltre 150 litri. Una testata per cilindro per facilitare la manutenzione. Lavora a 900–1 000 giri/min con rendimento termico ~40%; regime e potenza sono fissati per tacche dal manipolatore.',
      explode: [0, 0.7, 0],
    });

    // ================= Crankshaft + coupling =================
    const crankGroup = new THREE.Group();
    const crank = new THREE.Group();
    crank.position.set(0, CY, 0);
    crankGroup.add(crank);
    const crankMat = MAT.steel();
    const mainJ = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.06, 0.06, ex1 - ex0 + 0.12, 20)), crankMat);
    mainJ.position.x = (ex0 + ex1) / 2;
    crank.add(mainJ);
    const pinGeo = alongX(new THREE.CylinderGeometry(0.05, 0.05, 0.12, 18));
    const webGeo = new THREE.BoxGeometry(0.035, 0.27, 0.15).translate(0, 0.045, 0);
    const cwGeo = alongX(new THREE.CylinderGeometry(0.14, 0.14, 0.035, 24, 1, false, Math.PI / 2, Math.PI));
    CYL_X.forEach((x, k) => {
      const phi = -FIRE[k] * Math.PI * 4; // pin angle in crank space
      const pin = new THREE.Mesh(pinGeo, crankMat);
      pin.position.set(x, CR * Math.cos(phi), CR * Math.sin(phi));
      crank.add(pin);
      for (const dx of [-0.075, 0.075]) {
        const web = new THREE.Mesh(webGeo, crankMat);
        web.position.x = x + dx;
        web.rotation.x = phi;
        const cw = new THREE.Mesh(cwGeo, crankMat);
        cw.position.x = x + dx;
        cw.rotation.x = phi;
        crank.add(web, cw);
      }
    });
    const coupling = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.3, 0.3, 0.06, 48)), MAT.darkSteel());
    coupling.position.x = -0.37;
    const couplingMark = box(0.07, 0.14, 0.05, MAT.paint(0xff7a1a), -0.37, 0.2, 0);
    crank.add(coupling, couplingMark);
    // coupling bolts (rotor to coupling disc): their own part, turning with the crank
    const couplingSpin = new THREE.Group();
    couplingSpin.position.set(0, CY, 0);
    const couplingPts = Array.from({ length: 8 }, (_, i) => [-0.33, Math.cos(((i + 0.5) / 8) * TAU) * 0.23, Math.sin(((i + 0.5) / 8) * TAU) * 0.23]);
    couplingSpin.add(bolts(MAT.steel(), couplingPts, 'x', screw(0.06, 0.022, 0.02)));
    add(new THREE.Group().add(couplingSpin), {
      id: 'couplingBolts',
      name: 'Bulloni del giunto',
      base: 'Fissano il rotore dell’alternatore al disco in fondo all’albero motore: tutta la potenza del diesel passa da qui.',
      expert:
        'Bulloni disposti in cerchio sul giunto: trasmettono al rotore tutta la coppia del diesel. Nei giunti a flangia la coppia passa per attrito tra le facce serrate e, con bulloni calibrati, anche a taglio.',
      explode: [0, -0.15, 1.1],
    });
    add(crankGroup, {
      id: 'engineCrank',
      name: 'Albero motore e giunto',
      base: 'L’albero a gomiti trasforma il su-e-giù dei pistoni in rotazione. Il disco in fondo lo collega direttamente al generatore: nessuna frizione, nessun cambio.',
      expert:
        'Albero forgiato con manovelle a 120° per il 6 in linea (ordine di accensione 1-5-3-6-2-4, perfettamente equilibrato); nei V16 a 2 tempi una combustione ogni 22,5°. Il giunto flessibile (o il volano) collega l’albero direttamente al rotore dell’alternatore, che gira agli stessi giri.',
      explode: [0, -0.15, 1.1],
    });

    // ================= Pistons and conrods =================
    const pg = new THREE.Group();
    const pistonGeo = lathe(
      [[0, 0.08], [0.06, 0.065], [0.09, 0.075], [0.095, 0.1], [0.12, 0.1], [0.125, 0.095], [0.125, -0.12], [0.115, -0.12], [0.115, 0.03], [0, 0.03]],
      32
    );
    const pistonMat = MAT.aluminum();
    const rodMat = MAT.steel();
    const rodGeo = new THREE.BoxGeometry(0.03, CL - 0.1, 0.05).translate(0, CL / 2, 0);
    const bigEndGeo = new THREE.TorusGeometry(0.055, 0.018, 8, 20).rotateY(Math.PI / 2);
    const smallEndGeo = new THREE.TorusGeometry(0.035, 0.012, 8, 16).rotateY(Math.PI / 2).translate(0, CL, 0);
    const pistons = [];
    const rods = [];
    for (const x of CYL_X) {
      const p = new THREE.Mesh(pistonGeo, pistonMat);
      p.position.x = x;
      const r = new THREE.Group();
      r.add(new THREE.Mesh(rodGeo, rodMat), new THREE.Mesh(bigEndGeo, rodMat), new THREE.Mesh(smallEndGeo, rodMat));
      r.position.x = x;
      pg.add(p, r);
      pistons.push(p);
      rods.push(r);
    }
    add(pg, {
      id: 'enginePistons',
      name: 'Pistoni e bielle',
      base: 'Sei pistoni che vanno su e giù a turno, spinti dalla combustione del gasolio. Le bielle portano la spinta all’albero motore.',
      expert:
        'Pistoni articolati o in acciaio con camera a tazza nel cielo, raffreddati da getti d’olio; bielle forgiate. Ciclo diesel ad accensione per compressione con iniezione diretta ad alta pressione (iniettori-pompa o common rail).',
      explode: [0, 0.35, 1.6],
    });
    const chargeGeo = new THREE.CylinderGeometry(0.128, 0.128, 1, 24);
    const charges = CYL_X.map((x) => {
      const m = new THREE.Mesh(chargeGeo, MAT.gas(0x5fd3ff, 0.2, false));
      m.position.x = x;
      m.renderOrder = 2;
      model.add(m);
      return m;
    });

    // ================= Turbocharger, manifolds, stack =================
    const turbo = new THREE.Group();
    const exMat = cut(MAT.paint(0x6b4a3a, 0.55));
    exMat.metalness = 0.8;
    const inMat = cut(MAT.paint(0x56606b, 0.5));
    const [tbx, tby] = TB;
    const exPts = [[-0.5, HEAD_Y + 0.09, -0.44], [-2.45, HEAD_Y + 0.09, -0.44], [-2.7, HEAD_Y + 0.03, -0.36], [tbx, tby + 0.17, -0.24]];
    turbo.add(tube(curve(exPts), 0.07, exMat, 60, 12));
    const inPts = [[tbx, tby - 0.17, 0.2], [tbx + 0.08, 2.02, 0.36], [-2.62, 2.3, 0.44], [-2.45, HEAD_Y + 0.08, 0.44], [-0.5, HEAD_Y + 0.08, 0.44]];
    turbo.add(tube(curve(inPts), 0.07, inMat, 60, 12));
    for (const x of CYL_X) {
      turbo.add(rod([x, HEAD_Y + 0.09, -0.3], [x, HEAD_Y + 0.09, -0.44], 0.045, exMat, 12));
      turbo.add(rod([x, HEAD_Y + 0.08, 0.3], [x, HEAD_Y + 0.08, 0.44], 0.045, inMat, 12));
    }
    const turbHousing = new THREE.Mesh(new THREE.TorusGeometry(0.15, 0.065, 12, 40), cut(MAT.castIron()));
    turbHousing.position.set(tbx, tby, -0.2);
    const compHousing = new THREE.Mesh(new THREE.TorusGeometry(0.15, 0.06, 12, 40), cut(MAT.aluminum()));
    compHousing.position.set(tbx, tby, 0.2);
    const compInlet = new THREE.Mesh(alongZ(pipe(0.11, 0.095, 0.14, 32)), cut(MAT.aluminum()));
    compInlet.position.set(tbx, tby, 0.32);
    const tbCenter = new THREE.Mesh(alongZ(new THREE.CylinderGeometry(0.07, 0.07, 0.3, 24)), cut(MAT.darkSteel()));
    tbCenter.position.set(tbx, tby, 0);
    const stackPts = [[tbx, tby, -0.28], [tbx, tby + 0.03, -0.5], [tbx, 2.62, -0.58], [tbx, 3.05, -0.58]];
    turbo.add(turbHousing, compHousing, compInlet, tbCenter, tube(curve(stackPts), 0.09, exMat, 30, 14));
    const silencer = box(0.3, 0.3, 0.3, MAT.darkSteel(), tbx, 2.82, -0.58);
    const stackLip = new THREE.Mesh(pipe(0.11, 0.09, 0.05, 24), MAT.darkSteel());
    stackLip.position.set(tbx, 3.06, -0.58);
    turbo.add(silencer, stackLip);
    const tbRotor = new THREE.Group();
    tbRotor.position.set(tbx, tby, 0);
    const tbShaft = new THREE.Mesh(alongZ(new THREE.CylinderGeometry(0.018, 0.018, 0.42, 10)), MAT.steel());
    const compW = turboWheel(0.12, 0.12, 11, MAT.aluminum());
    compW.position.z = 0.2;
    compW.rotation.y = -Math.PI / 2; // nose toward +Z
    const turbW = turboWheel(0.11, 0.1, 10, MAT.titanium());
    turbW.position.z = -0.2;
    turbW.rotation.y = Math.PI / 2; // nose toward -Z
    tbRotor.add(tbShaft, compW, turbW);
    turbo.add(tbRotor);
    add(turbo, {
      id: 'engineTurbo',
      name: 'Turbocompressore, collettori e camino',
      base: 'I gas di scarico (collettore scuro dietro) fanno girare la turbina del turbo, che soffia aria compressa nel collettore chiaro davanti. Poi escono dal camino sul tetto.',
      expert:
        'Turbocompressore (nei 2 tempi EMD trascinato anche meccanicamente all’avviamento, perché il motore ha bisogno di aria in pressione per il lavaggio) con aftercooler sull’aria di carica. Pressione di sovralimentazione 2,5–4 bar assoluti; le norme Stage V/Tier 4 aggiungono EGR, SCR e filtri antiparticolato.',
      explode: [-0.6, 0.6, 0.2],
    });

    // ================= Main alternator =================
    const AL0 = -0.3;
    const AL1 = 0.72;
    const alc = (AL0 + AL1) / 2;
    const alLen = AL1 - AL0;
    const alt = new THREE.Group();
    const altMat = cut(MAT.paint(0x35506a, 0.5));
    const shellA = new THREE.Mesh(alongX(pipe(0.52, 0.48, alLen, 64)), altMat);
    shellA.position.set(alc, CY, 0);
    alt.add(shellA);
    for (const x of [AL0, AL1]) {
      const bell = new THREE.Mesh(alongX(pipe(0.53, 0.09, 0.04, 64)), altMat);
      bell.position.set(x, CY, 0);
      alt.add(bell);
    }
    for (const x of [AL0 + 0.15, AL1 - 0.15]) alt.add(box(0.14, CY - 0.45 - DECK_Y, 1.0, altMat, x, (CY - 0.45 + DECK_Y) / 2, 0)); // feet
    alt.add(bolts(cut(MAT.steel()), boltCircle(AL1 + 0.025, CY, 0, 0.5, 16), 'x'));
    // drive-end adapter with its flange: encloses the coupling and bolts to the engine's flywheel housing
    alt.add(mesh(alongX(pipe(0.53, 0.5, 0.06, 64)), altMat, AL0 - 0.05, CY, 0));
    alt.add(mesh(alongX(pipe(0.595, 0.5, 0.025, 64)), altMat, AL0 - 0.0925, CY, 0));
    add(bolts(cut(MAT.steel()), boltCircle(AL0 - 0.066, CY, 0, 0.563, 12), 'x', screw(0.07, 0.026, 0.028)), {
      id: 'alternatorBolts',
      name: 'Bulloni della flangia dell’alternatore',
      base: 'Una corona di bulloni unisce la carcassa dell’alternatore al diesel: così i due formano un blocco unico e i loro alberi restano allineati.',
      expert:
        'La flangia della carcassa si centra sulla campana del volano del diesel: l’accoppiamento rigido delle due carcasse garantisce l’allineamento degli alberi e un traferro costante, indispensabile quando il rotore ha un solo cuscinetto.',
      explode: [0, 1.0, 0],
    });
    const vents = new THREE.Group();
    for (let i = 0; i < 7; i++) {
      const a = ((i - 3) / 16) * Math.PI;
      const v = rbox(0.6, 0.015, 0.05, 0.006, MAT.paint(0x1d2630, 0.6), alc, CY + Math.cos(a) * 0.525, Math.sin(a) * 0.525);
      v.rotation.x = a;
      vents.add(v);
    }
    alt.add(cutAll(vents));
    const coreProfile = [];
    const nR = 36;
    for (let i = 0; i <= nR; i++) coreProfile.push([0.47 - (i % 2) * 0.005, -0.38 + (i / nR) * 0.76]);
    for (let i = nR; i >= 0; i--) coreProfile.push([0.34, -0.38 + (i / nR) * 0.76]);
    coreProfile.push([0.47, -0.38]);
    const statorCore = new THREE.Mesh(alongX(lathe(coreProfile, 64)), cut(MAT.darkSteel()));
    statorCore.position.set(alc, CY, 0);
    alt.add(statorCore);
    const slotMat = cut(MAT.copper());
    const slotGeo = new THREE.BoxGeometry(0.82, 0.03, 0.025);
    for (let i = 0; i < 36; i++) {
      const a = (i / 36) * TAU;
      const s = new THREE.Mesh(slotGeo, slotMat);
      s.position.set(alc, CY + Math.cos(a) * 0.36, Math.sin(a) * 0.36);
      s.rotation.x = a;
      alt.add(s);
    }
    for (const x of [alc - 0.42, alc + 0.42]) {
      const ew = new THREE.Mesh(new THREE.TorusGeometry(0.4, 0.04, 8, 48).rotateY(Math.PI / 2), slotMat);
      ew.position.set(x, CY, 0);
      alt.add(ew);
    }
    const tBox = box(0.36, 0.12, 0.3, altMat, 0.42, CY + 0.55, -0.2);
    alt.add(tBox);
    const bushGeo = new THREE.CylinderGeometry(0.025, 0.03, 0.06, 10);
    const bushMat = MAT.ceramic();
    for (let k = 0; k < 3; k++) {
      const b = new THREE.Mesh(bushGeo, bushMat);
      b.position.set(0.32 + k * 0.1, CY + 0.64, -0.2);
      alt.add(b);
    }
    add(alt, {
      id: 'alternatorStator',
      name: 'Alternatore principale (statore)',
      base: 'Il generatore: la parte ferma è un grande anello di lamierini d’acciaio con dentro gli avvolgimenti di rame. Quando la calamita del rotore gli gira dentro, nel rame nasce la corrente.',
      expert:
        'Statore trifase (spesso doppio avvolgimento sfasato per ridurre l’ondulazione dopo il raddrizzamento) in pacco lamellare di acciaio al silicio. Alternatori da 2–3,5 MW, tensione fino a ~1 500 V, frequenza proporzionale ai giri del diesel (decine di Hz).',
      explode: [0, 1.0, 0],
    });

    const rotorGroup = new THREE.Group();
    const altRotor = new THREE.Group();
    altRotor.position.set(0, CY, 0);
    rotorGroup.add(altRotor);
    const arShaft = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.07, 0.07, AL1 - AL0 + 0.36, 20)), MAT.steel());
    arShaft.position.x = alc + 0.05;
    altRotor.add(arShaft);
    const arCore = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.14, 0.14, 0.78, 24)), MAT.darkSteel());
    arCore.position.x = alc;
    altRotor.add(arCore);
    const poleMat = MAT.steel();
    const fieldMat = glowMat(MAT.copper());
    const NP = 10;
    const fieldCoils = [];
    for (let i = 0; i < NP; i++) {
      const a = (i / NP) * TAU;
      const pole = new THREE.Group();
      pole.rotation.x = a;
      pole.position.x = alc;
      const coil = box(0.7, 0.08, 0.16, fieldMat, 0, 0.22, 0);
      pole.add(box(0.74, 0.14, 0.1, poleMat, 0, 0.2, 0), box(0.76, 0.035, 0.2, poleMat, 0, 0.31, 0), coil);
      altRotor.add(pole);
      fieldCoils.push(coil);
    }
    for (const dx of [0, 0.06]) {
      const ring = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.09, 0.09, 0.03, 24)), MAT.copper());
      ring.position.x = AL1 + 0.14 + dx;
      altRotor.add(ring);
    }
    const fan = new THREE.Group();
    fan.position.x = AL0 + 0.06;
    const fanBlade = new THREE.BoxGeometry(0.04, 0.2, 0.012).translate(0, 0.22, 0);
    for (let i = 0; i < 12; i++) {
      const b = new THREE.Mesh(fanBlade, MAT.aluminum());
      b.rotation.x = (i / 12) * TAU;
      fan.add(b);
    }
    altRotor.add(fan);
    add(rotorGroup, {
      id: 'alternatorRotor',
      name: 'Rotore dell’alternatore',
      base: 'La parte che gira, attaccata all’albero del diesel. Le bobine sui “denti” (poli) sono percorse da corrente e diventano calamite: più corrente, più energia produce il generatore.',
      expert:
        'Rotore a 10 poli salienti con avvolgimenti di campo alimentati in continua tramite anelli e spazzole o eccitatrice rotante. Regolando la corrente di eccitazione la centralina controlla tensione e potenza erogata a giri costanti del diesel. Il ventilatore calettato sull’albero raffredda la macchina.',
      explode: [0, 0.15, 1.5],
    });

    // ================= Power electronics cabinet =================
    const cab = new THREE.Group();
    const cabMat = cut(MAT.paint(0x56636f, 0.55));
    const CB = { x0: 1.0, x1: 2.45, y0: DECK_Y, y1: 2.5, z0: -0.75, z1: 0.3 };
    const cw = CB.x1 - CB.x0;
    const ch = CB.y1 - CB.y0;
    const cd = CB.z1 - CB.z0;
    const ccx = (CB.x0 + CB.x1) / 2;
    const ccy = (CB.y0 + CB.y1) / 2;
    const ccz = (CB.z0 + CB.z1) / 2;
    cab.add(
      box(cw, ch, 0.03, cabMat, ccx, ccy, CB.z0 + 0.015),
      box(cw, ch, 0.03, cabMat, ccx, ccy, CB.z1 - 0.015),
      box(cw, 0.03, cd, cabMat, ccx, CB.y1 - 0.015, ccz),
      box(0.03, ch, cd, cabMat, CB.x0 + 0.015, ccy, ccz),
      box(0.03, ch, cd, cabMat, CB.x1 - 0.015, ccy, ccz)
    );
    const cabFront = new THREE.Group();
    for (const side of [-1, 1]) {
      const x = ccx + (side * cw) / 4;
      cabFront.add(rbox(cw / 2 - 0.04, ch - 0.08, 0.02, 0.008, cabMat, x, ccy, CB.z1 + 0.01));
      cabFront.add(rbox(0.03, 0.18, 0.03, 0.01, MAT.chrome(), x - side * (cw / 4 - 0.08), ccy, CB.z1 + 0.035));
      for (const y of [CB.y0 + 0.25, CB.y1 - 0.25]) cabFront.add(rbox(0.04, 0.1, 0.03, 0.008, MAT.darkSteel(), x + side * (cw / 4 - 0.03), y, CB.z1 + 0.03));
    }
    cabFront.add(mesh(new THREE.PlaneGeometry(0.16, 0.16), new THREE.MeshStandardMaterial({ map: warnTexture(), roughness: 0.6 }), ccx - cw / 4, CB.y1 - 0.35, CB.z1 + 0.022));
    cab.add(cutAll(cabFront));
    for (let i = 0; i < 8; i++) cab.add(rbox(0.02, 0.025, cd - 0.25, 0.006, MAT.paint(0x3c4753, 0.6), CB.x1 + 0.005, CB.y0 + 0.35 + i * 0.09, ccz)); // louvres
    const sinkMat = MAT.aluminum();
    const darkMod = MAT.paint(0x1c2128, 0.5);
    // rectifier: 3 diode legs
    const diodes = [];
    for (let k = 0; k < 3; k++) {
      const x = 1.2 + k * 0.15;
      cab.add(box(0.12, 0.62, 0.1, sinkMat, x, 1.85, -0.32));
      for (const y of [2.02, 1.68]) {
        const d = box(0.1, 0.1, 0.05, glowMat(MAT.paint(0x1c2128, 0.5)), x, y, -0.25);
        cab.add(d);
        diodes.push(d);
      }
    }
    // DC link capacitors
    const capMat = MAT.paint(0x2c5d8a, 0.4);
    for (const x of [1.67, 1.83]) {
      for (const z of [-0.25, -0.45]) {
        const c = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.065, 0.34, 20), capMat);
        c.position.set(x, 1.45, z);
        cab.add(c);
      }
    }
    // inverter: 3 legs x 2 IGBT modules on a heat sink
    cab.add(box(0.52, 0.5, 0.06, sinkMat, 2.17, 1.88, -0.32));
    const igbt = [];
    for (let k = 0; k < 3; k++) {
      const leg = [];
      for (const y of [2.0, 1.76]) {
        const m = box(0.13, 0.1, 0.04, glowMat(MAT.paint(0x1c2128, 0.5)), 2.0 + k * 0.17, y, -0.27);
        cab.add(m);
        leg.push(m);
      }
      igbt.push(leg);
    }
    const busMat = MAT.copper();
    cab.add(box(1.25, 0.03, 0.02, busMat, 1.75, 2.2, -0.2), box(1.25, 0.03, 0.02, busMat, 1.75, 1.3, -0.2));
    cab.add(box(0.03, 0.9, 0.02, busMat, 1.55, 1.75, -0.2), box(0.03, 0.9, 0.02, busMat, 1.92, 1.75, -0.2));
    cab.add(box(0.26, 0.2, 0.08, darkMod, 2.3, 1.42, -0.3)); // control unit
    // mounting flanges along the base, bolted to the deck
    const cabBolts = [];
    for (const z of [CB.z0 - 0.04, CB.z1 + 0.04]) {
      cab.add(box(cw, 0.03, 0.08, cabMat, ccx, CB.y0 + 0.015, z));
      for (let k = 0; k < 4; k++) cabBolts.push([CB.x0 + 0.12 + (k * (cw - 0.24)) / 3, CB.y0 + 0.04, z + Math.sign(z) * 0.008]);
    }
    add(bolts(cut(MAT.steel()), cabBolts, 'y', screw(0.14)), {
      id: 'cabinetBolts',
      name: 'Bulloni dell’armadio di potenza',
      base: 'I bulloni che fissano la base dell’armadio elettronico al telaio della locomotiva.',
      expert:
        'L’armadio è un modulo sostituibile: fissato al telaio con pochi bulloni lungo la base, si estrae intero dall’alto. I fissaggi devono reggere urti e vibrazioni di marcia senza allentarsi.',
      explode: [0.35, 0.25, 0.8],
    });
    add(cab, {
      id: 'powerElectronics',
      name: 'Armadio di potenza (raddrizzatore e inverter)',
      base: 'Un armadio pieno di elettronica: a sinistra il raddrizzatore rende continua la corrente del generatore, al centro i condensatori la tengono stabile, a destra l’inverter la trasforma nella corrente giusta per i motori. I moduli si illuminano quando conducono.',
      expert:
        'Raddrizzatore trifase a ponte di diodi (o attivo) → DC link ~1 500–2 800 V con condensatori a film → inverter VVVF a IGBT (spesso uno per carrello o per asse, per un controllo dell’aderenza asse per asse). Raffreddamento ad aria forzata o a liquido; la centralina coordina eccitazione dell’alternatore, inverter e antislittamento.',
      explode: [0.35, 0.25, 0.8],
    });

    // ================= Brake resistor grids (roof) =================
    const res = new THREE.Group();
    const resFrame = MAT.paint(0x4a525c, 0.55);
    const RY = CB.y1;
    for (const x of [1.08, 2.37]) for (const z of [-0.68, 0.22]) res.add(box(0.05, 0.36, 0.05, resFrame, x, RY + 0.18, z));
    res.add(box(1.36, 0.03, 0.95, resFrame, ccx, RY + 0.36, ccz));
    const gridMat = MAT.paint(0x6f6a64, 0.6);
    gridMat.emissive.setHex(0xff4a14);
    gridMat.emissiveIntensity = 0;
    const gridGeo = new THREE.BoxGeometry(0.012, 0.22, 0.1);
    const grids = new THREE.Group();
    for (let r = 0; r < 5; r++) {
      for (let k = 0; k < 18; k++) {
        const m = new THREE.Mesh(gridGeo, gridMat);
        m.position.set(1.18 + k * 0.065, RY + 0.17, -0.6 + r * 0.19);
        m.rotation.y = k % 2 ? 0.35 : -0.35; // zig-zag ribbons
        grids.add(m);
      }
    }
    res.add(grids);
    const shroud = new THREE.Mesh(pipe(0.29, 0.27, 0.1, 40), resFrame);
    shroud.position.set(ccx, RY + 0.42, ccz);
    res.add(shroud);
    const resFan = new THREE.Group();
    resFan.position.set(ccx, RY + 0.42, ccz);
    const rfb = new THREE.BoxGeometry(0.24, 0.012, 0.07).translate(0.13, 0, 0);
    for (let i = 0; i < 6; i++) {
      const b = new THREE.Mesh(rfb, MAT.darkSteel());
      b.rotation.y = (i / 6) * TAU;
      b.rotation.x = 0.3;
      resFan.add(b);
    }
    res.add(resFan);
    add(res, {
      id: 'brakeResistors',
      name: 'Reostati di frenatura dinamica',
      base: 'Griglie di nastri metallici sul tetto. Quando il macchinista frena, i motori diventano generatori e la loro corrente scalda queste griglie fino a farle arrossare: l’energia del treno se ne va in calore. Riduci di colpo il gas per vederle accendersi.',
      expert:
        'Frenatura dinamica reostatica: l’inverter fa lavorare i motori da generatori e un chopper scarica il DC link sulle resistenze, raffreddate da ventilatori. Potenze di 2–4 MW, utilissima in discesa perché risparmia ceppi e dischi. Senza catenaria l’energia non può tornare in linea; nelle macchine ibride va in parte in batteria.',
      explode: [0.3, 1.0, 0],
    });

    // ================= Cables =================
    const cables = new THREE.Group();
    const cableMat = glowMat(MAT.paint(0x2a2d31, 0.6));
    const altCab = [-0.14, -0.2, -0.26].map((z, i) =>
      curve([[0.32 + i * 0.1, CY + 0.66, z], [0.45 + i * 0.1, 2.68, z], [0.82, 2.55, z], [CB.x0 + 0.02, 2.15 - i * 0.04, z]])
    );
    const cabM1 = curve([[1.4, DECK_Y + 0.02, -0.5], [1.3, 0.98, -0.55], [0.62, 0.92, -0.5], [MX[0], 0.84, -0.3], [MX[0], WR + MR + 0.05, -0.2]]);
    const cabM2 = curve([[1.62, DECK_Y + 0.02, -0.5], [1.62, 0.98, -0.5], [MX[1], 0.84, -0.3], [MX[1], WR + MR + 0.05, -0.2]]);
    const cableMeshes = [...altCab, cabM1, cabM2].map((c) => tube(c, 0.022, cableMat, 40, 8));
    cables.add(...cableMeshes);
    add(cables, {
      id: 'cables',
      name: 'Cavi di potenza',
      base: 'Grossi cavi di rame portano la corrente dal generatore all’armadio elettronico e da lì ai motori sui carrelli. Si illuminano in base a quanta corrente passa.',
      expert:
        'Cavi in rame flessibile di grande sezione (spesso più conduttori in parallelo): le correnti di trazione all’avviamento arrivano a diversi kA per motore. Il tratto verso i carrelli deve seguire i movimenti della sospensione e la rotazione in curva.',
      explode: [0, 0.25, 0.9],
    });

    // ================= Flows =================
    const mk = (c, opts, lift = null) => {
      const f = flow({
        path: (t, out) => {
          c.getPointAt(t, out);
          if (lift) out.add(lift);
          return out.add(OFFSET_MODEL);
        },
        ...opts,
      });
      root.add(f.object);
      ctx.addFlow(f);
      return f;
    };
    const exFlowCurve = curve([...exPts, [tbx, tby, -0.2], ...stackPts.slice(1), [tbx + 0.05, 3.4, -0.58], [tbx + 0.2, 3.75, -0.58]]);
    const exFlow = mk(exFlowCurve, {
      count: 260,
      size: 0.09,
      colors: [0xb07a6a, 0x8c86a8, 0x8c86a8, 0x8c86a8],
      spread: (t) => (t > 0.85 ? 0.05 + (t - 0.85) * 1.2 : 0.04),
      speed: 0.22,
    });
    const airFlowCurve = curve([[tbx, tby, 0.7], [tbx, tby, 0.3], [tbx, tby + 0.1, 0.2], [tbx + 0.1, tby - 0.1, 0.2], ...inPts]);
    const airFlow = mk(airFlowCurve, { count: 160, size: 0.07, colors: [0x5fd3ff], spread: 0.04, speed: 0.25 });
    const fuelFlow = mk(fuelLine, { count: 40, size: 0.05, colors: [0xffc04d], spread: 0.01, speed: 0.5 });
    const wireLift = V(0, 0.022, 0.01);
    const curAlt = mk(altCab[1], { count: 120, size: 0.06, colors: [CURRENT, 0xf4ffb0, CURRENT], spread: 0.05, speed: 0.5 }, wireLift);
    const cabCurve = curve([
      [CB.x0 + 0.03, 2.1, -0.2], [1.2, 2.06, -0.18], [1.35, 1.85, -0.18], [1.5, 1.68, -0.18], [1.75, 1.62, -0.15],
      [2.0, 2.0, -0.2], [2.17, 1.88, -0.2], [2.34, 1.76, -0.2], [2.2, 1.35, -0.2], [1.6, 1.28, -0.4], [1.5, DECK_Y + 0.03, -0.5],
    ]);
    const curCab = mk(cabCurve, { count: 140, size: 0.06, colors: [CURRENT, 0xf4ffb0, CURRENT], spread: 0.04, speed: 0.35 });
    const curM = [cabM1, cabM2].map((c) =>
      mk(c, { count: 110, size: 0.06, colors: [CURRENT, 0xf4ffb0, CURRENT], spread: 0.02, speed: 0.55 }, wireLift)
    );
    const currentFlows = [curAlt, curCab, ...curM];

    // ================= Animation =================
    const cA = new THREE.Color();
    const C = {
      air: new THREE.Color(0x5fd3ff),
      hot: new THREE.Color(0xff9a3c),
      fire: new THREE.Color(0xff5a1f),
      exh: new THREE.Color(0x8c86a8),
    };
    const cycleRate = (th) => 0.2 + th * 0.6; // engine cycles/s (2 crank turns each)
    let speed = 0; // train speed 0..1, lags the throttle (inertia)
    let wheelAng = 0;
    let scroll = 0;
    let turboAng = 0;
    let fanAng = 0;
    let braking = 0;
    const kmhOf = (v) => 120 * Math.pow(v, 1.6);

    return {
      root,
      cycleRate,
      update(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const dt = state.dt || 0;
        const time = state.time || 0;
        const assembled = (state.explode || 0) < 0.05;

        // Diesel: crank, pistons, rods, alternator rotor (direct coupling).
        const theta1 = state.cycle * Math.PI * 4;
        crank.rotation.x = theta1;
        couplingSpin.rotation.x = theta1;
        altRotor.rotation.x = theta1;
        CYL_X.forEach((x, k) => {
          const local = wrap(state.cycle - FIRE[k]);
          const th = local * Math.PI * 4;
          const yc = CR * Math.cos(th);
          const zc = CR * Math.sin(th);
          const yp = yc + Math.sqrt(CL * CL - zc * zc);
          pistons[k].position.y = CY + yp;
          rods[k].position.set(x, CY + yc, zc);
          rods[k].rotation.x = Math.atan2(-zc, yp - yc);
          const top = CY + yp + 0.1;
          const ch = charges[k];
          ch.position.y = (top + HEAD_Y) / 2;
          ch.scale.y = Math.max(0.001, HEAD_Y - top);
          let op;
          if (local < 0.25) {
            cA.copy(C.air);
            op = 0.12;
          } else if (local < 0.49) {
            const f = (local - 0.25) / 0.24;
            cA.copy(C.air).lerp(C.hot, f * f);
            op = 0.15 + 0.3 * f;
          } else if (local < 0.75) {
            const f = (local - 0.49) / 0.26;
            cA.copy(C.fire).lerp(C.exh, smooth(f));
            op = 0.9 - 0.55 * f;
          } else {
            cA.copy(C.exh);
            op = 0.3 - 0.2 * ((local - 0.75) / 0.25);
          }
          ch.material.color.copy(cA);
          ch.material.opacity = op * (0.6 + 0.4 * thr);
          ch.visible = assembled;
        });

        // Train speed follows the throttle with inertia; slowing down = dynamic braking.
        if (dt > 0) speed += (thr - speed) * Math.min(1, dt * 0.5);
        braking = clamp((speed - thr) * 5, 0, 1);
        const omega = speed * 2.6; // wheel rad/s (visual)
        wheelAng = (wheelAng + dt * omega) % (TAU * Z_PINION);
        scroll += dt * omega * WR;
        wheelSpins.forEach((s) => (s.rotation.z = wheelAng));
        bigGears.forEach((g) => (g.rotation.z = wheelAng));
        const motorAng = -wheelAng * RATIO;
        motorSpins.forEach((s) => (s.rotation.z = motorAng));
        pinions.forEach((p) => (p.rotation.z = motorAng + Math.PI / Z_PINION));
        for (const s of sleepers) {
          const x = TRACK_X0 - 0.15 + wrap((s.userData.x0 + scroll) / SL_P) * SL_P;
          const d = Math.min(x - TRACK_X0, TRACK_X1 - x);
          const e = clamp((0.45 - d) / 0.45, 0, 1);
          s.position.set(x, -0.2 * smooth(e), 0);
        }

        turboAng = (turboAng + dt * (3 + 20 * thr)) % TAU;
        tbRotor.rotation.z = turboAng;
        fanAng = (fanAng + dt * (0.4 + 9 * braking)) % TAU;
        resFan.rotation.y = fanAng;

        // Electrical glow: excitation, diodes, IGBT PWM, motor bars, cables; grids when braking.
        // (write to mesh.material: ctx.part cloned the materials)
        const load = thr;
        fieldCoils.forEach((m) => (m.material.emissiveIntensity = 0.15 + 0.9 * load));
        const pwm = Math.sin(time * 70) > -0.3 ? 1 : 0.35;
        diodes.forEach((d, i) => (d.material.emissiveIntensity = load * 1.2 * Math.max(0, Math.cos(theta1 * 5 - (i * TAU) / 6))));
        igbt.forEach((leg, k) => {
          const ph = Math.cos(time * (1 + 6 * speed) - (k * TAU) / 3);
          leg[0].material.emissiveIntensity = load * 1.5 * Math.max(0, ph) * pwm;
          leg[1].material.emissiveIntensity = load * 1.5 * Math.max(0, -ph) * pwm;
        });
        const barGlow = 0.1 + 0.9 * Math.max(load, braking);
        motorBars.forEach((m) => (m.material.emissiveIntensity = barGlow));
        const cableGlow = 0.7 * Math.max(load, braking * 0.6);
        cableMeshes.forEach((m) => (m.material.emissiveIntensity = cableGlow));
        const gridGlow = braking * (1.6 + 0.3 * Math.sin(time * 9));
        grids.children.forEach((m) => (m.material.emissiveIntensity = gridGlow));

        // Flows.
        exFlow.material.opacity = 0.45 + 0.45 * thr;
        exFlow.update(dt, 0.4 + 1.6 * thr);
        airFlow.material.opacity = 0.35 + 0.5 * thr;
        airFlow.update(dt, 0.4 + 1.6 * thr);
        fuelFlow.update(dt, 0.3 + thr);
        for (const f of currentFlows) {
          f.material.opacity = 0.9 * clamp(load * 3, 0, 1);
          f.update(dt, 0.2 + 1.8 * load);
        }
        for (const f of [exFlow, airFlow, fuelFlow, ...currentFlows]) f.object.visible = assembled;
        fuelVol.visible = assembled;
      },
      readout(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const rpm = 320 + 630 * thr;
        const brakingNow = braking > 0.02;
        const kw = brakingNow ? 0 : 2600 * thr;
        const kmh = kmhOf(speed);
        const v = kmh / 3.6;
        let kn = 0;
        if (brakingNow) kn = -Math.round(250 * braking); // dynamic braking
        else if (kw > 1) kn = Math.round(Math.min(410, kw / Math.max(v, 0.1))); // adhesion-limited at low speed
        return [
          { label: 'Giri motore diesel', value: Math.round(rpm / 10) * 10, unit: 'giri/min' },
          { label: 'Potenza di trazione', value: Math.round(kw / 10) * 10, unit: 'kW' },
          { label: 'Velocità', value: Math.round(kmh), unit: 'km/h' },
          { label: 'Sforzo alle ruote', value: kn, unit: 'kN' },
        ];
      },
    };
  },
};
