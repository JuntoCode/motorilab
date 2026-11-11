// Low-speed two-stroke crosshead marine diesel (WinGD X92 / MAN B&W G95ME-C class), 3 of its cylinders.
// Uniflow scavenging: air in through liner ports, exhaust out through one hydraulic valve.
// Scale ≈ 0.3 units per metre. Cycle 0..1 = one revolution of cylinder 1, starting at exhaust valve closing.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, pipe, flow, blade, ringX, clamp, lerp, smooth } from '../core/helpers.js';

const CYL_X = [-0.48, 0, 0.48];
const OFFSET = [0, 1 / 3, 2 / 3]; // throws at 120°
const R = 0.52; // crank radius (stroke ≈ 3.5 m)
const L = 1.25; // conrod
const ROD = 1.3; // crosshead pin -> piston base
const CROWN = 0.22; // crown edge above piston base
const BORE = 0.14; // bore radius (≈ 0.92 m bore)
const LINER_BOT = 1.98;
const COVER_Y = 3.36;
const VALVE_Y = COVER_Y + 0.02;
const A0 = 260; // crank angle (deg after TDC) at cycle 0 = exhaust valve closing
const EVO = 110; // exhaust valve opens
const SPO = 136; // scavenge ports open (close at 360 - SPO)
const INJ = 357; // injection start
const DEG = Math.PI / 180;

const xheadY = (deg) => {
  const t = deg * DEG;
  const zc = R * Math.sin(t);
  return R * Math.cos(t) + Math.sqrt(L * L - zc * zc);
};
const PORT_BOT = xheadY(180) + ROD + CROWN - 0.01;
const PORT_TOP = xheadY(SPO) + ROD + CROWN;
const PORT_MID = (PORT_BOT + PORT_TOP) / 2;
const angleOf = (c) => (((c * 360 + A0) % 360) + 360) % 360;
const valveLift = (a) => smooth(clamp((a - EVO) / 25, 0, 1)) * smooth(clamp((A0 - a) / 30, 0, 1));
const portOpen = (a) => clamp((PORT_TOP - (xheadY(a) + ROD + CROWN)) / (PORT_TOP - PORT_BOT), 0, 1);

const v3 = (x, y, z = 0) => new THREE.Vector3(x, y, z);
const curve = (pts) => new THREE.CatmullRomCurve3(pts, false, 'centripetal');
const tube = (pts, r, mat, seg = 48) => new THREE.Mesh(new THREE.TubeGeometry(curve(pts), seg, r, 16), mat);

// Rectangular frame (hollow box walls) extruded vertically from y0 to y1.
function boxRing(hx, hz, wall, y0, y1, mat) {
  const s = new THREE.Shape();
  s.moveTo(-hx, -hz);
  s.lineTo(hx, -hz);
  s.lineTo(hx, hz);
  s.lineTo(-hx, hz);
  s.lineTo(-hx, -hz);
  const h = new THREE.Path();
  h.moveTo(-hx + wall, -hz + wall);
  h.lineTo(-hx + wall, hz - wall);
  h.lineTo(hx - wall, hz - wall);
  h.lineTo(hx - wall, -hz + wall);
  h.lineTo(-hx + wall, -hz + wall);
  s.holes.push(h);
  const g = new THREE.ExtrudeGeometry(s, { depth: y1 - y0, bevelEnabled: false });
  g.rotateX(-Math.PI / 2);
  g.translate(0, y0, 0);
  return new THREE.Mesh(g, mat);
}
// Horizontal plate with round holes at the cylinder positions.
function holedPlate(hx, hz, holeR, y, thick, mat) {
  const s = new THREE.Shape();
  s.moveTo(-hx, -hz);
  s.lineTo(hx, -hz);
  s.lineTo(hx, hz);
  s.lineTo(-hx, hz);
  s.lineTo(-hx, -hz);
  for (const x of CYL_X) {
    const h = new THREE.Path();
    h.absarc(x, 0, holeR, 0, Math.PI * 2, true);
    s.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: thick, bevelEnabled: false, curveSegments: 32 });
  g.rotateX(-Math.PI / 2);
  g.translate(0, y, 0);
  return new THREE.Mesh(g, mat);
}

// Flips a non-indexed geometry so its faces point outward (positive signed volume).
function orientOutward(g) {
  const p = g.attributes.position.array;
  let vol = 0;
  for (let i = 0; i < p.length; i += 9) {
    vol +=
      p[i] * (p[i + 4] * p[i + 8] - p[i + 5] * p[i + 7]) -
      p[i + 1] * (p[i + 3] * p[i + 8] - p[i + 5] * p[i + 6]) +
      p[i + 2] * (p[i + 3] * p[i + 7] - p[i + 4] * p[i + 6]);
  }
  if (vol >= 0) return g;
  for (const name of Object.keys(g.attributes)) {
    const a = g.attributes[name].array;
    const s = g.attributes[name].itemSize;
    for (let t = 0; t < a.length; t += 3 * s) {
      for (let k = 0; k < s; k++) {
        const tmp = a[t + s + k];
        a[t + s + k] = a[t + 2 * s + k];
        a[t + 2 * s + k] = tmp;
      }
    }
  }
  const n = g.attributes.normal.array;
  for (let i = 0; i < n.length; i++) n[i] = -n[i];
  return g;
}
// Lathe of a closed (r, y) loop: a watertight solid that gets a section cap.
function solidLathe(pts, segs = 48) {
  const g = lathe([...pts, pts[0]], segs).toNonIndexed();
  g.deleteAttribute('uv');
  return orientOutward(g);
}
// Hollow body of revolution: outer profile plus the same profile offset inward by `wall`.
function shellLathe(outer, wall, segs = 48) {
  const inner = outer.map(([r, y]) => [Math.max(0.01, r - wall), y]).reverse();
  return solidLathe([...outer, ...inner], segs);
}
// Hex bolt heads / nuts as one InstancedMesh. items: [[x,y,z], [nx,ny,nz]].
const HEX = new THREE.CylinderGeometry(1, 1, 1, 6);
function bolts(items, mat, r = 0.022, h = 0.025) {
  const m = new THREE.InstancedMesh(HEX, mat, items.length);
  const up = v3(0, 1, 0);
  const q = new THREE.Quaternion();
  const N = new THREE.Vector3();
  const P = new THREE.Vector3();
  const M = new THREE.Matrix4();
  const S = v3(r, h, r);
  items.forEach(([p, n], i) => {
    N.set(...n).normalize();
    q.setFromUnitVectors(up, N);
    P.set(...p).addScaledVector(N, h / 2);
    m.setMatrixAt(i, M.compose(P, q, S));
  });
  m.instanceMatrix.needsUpdate = true;
  return m;
}
// Bolt row along X on a face with normal n.
const boltRow = (x0, x1, n, y, z, normal) =>
  Array.from({ length: n }, (_, i) => [[lerp(x0, x1, n > 1 ? i / (n - 1) : 0.5), y, z], normal]);
// Ring of bolts on a plane facing `normal` ('x' | 'y' | 'z' with sign).
function boltCircle(cx, cy, cz, r, n, axis, sign = 1) {
  const out = [];
  for (let i = 0; i < n; i++) {
    const a = ((i + 0.5) / n) * Math.PI * 2;
    const u = Math.cos(a) * r;
    const v = Math.sin(a) * r;
    if (axis === 'z') out.push([[cx + u, cy + v, cz], [0, 0, sign]]);
    else if (axis === 'y') out.push([[cx + u, cy, cz + v], [0, sign, 0]]);
    else out.push([[cx, cy + u, cz + v], [sign, 0, 0]]);
  }
  return out;
}
// Fasteners that are their own part (Officina, see CONTRACT.md): one InstancedMesh of `geo`, seat at the
// item position, local +Y out of the hole. items: [[x,y,z], [nx,ny,nz]].
function fasteners(geo, items, mat) {
  const m = new THREE.InstancedMesh(geo, mat, items.length);
  const up = v3(0, 1, 0);
  const q = new THREE.Quaternion();
  const N = new THREE.Vector3();
  const M = new THREE.Matrix4();
  const one = v3(1, 1, 1);
  items.forEach(([p, n], i) => m.setMatrixAt(i, M.compose(v3(...p), q.setFromUnitVectors(up, N.set(...n).normalize()), one)));
  m.instanceMatrix.needsUpdate = true;
  return m;
}
// Nut on a stud: hex body on a round collar, seat at y = 0 (the stud itself stays on the engine).
const studNut = (r, h) =>
  mergeGeometries([
    new THREE.CylinderGeometry(r, r, h * 0.82, 6).translate(0, h * 0.59, 0),
    new THREE.CylinderGeometry(r * 1.12, r * 1.12, h * 0.18, 16).translate(0, h * 0.09, 0),
  ]);
// Screwed-in plug: hex head above the seat, threaded shank of length `len` below it.
const plugGeo = (r, h, len) =>
  mergeGeometries([new THREE.CylinderGeometry(r, r, h, 6).translate(0, h / 2, 0), new THREE.CylinderGeometry(r * 0.55, r * 0.55, len, 8).translate(0, -len / 2, 0)]);
const rbox = (w, h, d, r = 0.02) => new RoundedBoxGeometry(w, h, d, 2, Math.min(r, w / 2, h / 2, d / 2) * 0.99);

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
// Crank web: hull of journal and pin circles, extruded along X, pin toward +Y.
function webGeometry(thick) {
  const pts = [];
  for (let i = 0; i < 32; i++) {
    const a = (i / 32) * Math.PI * 2;
    pts.push([Math.cos(a) * 0.21, Math.sin(a) * 0.21]);
    pts.push([Math.cos(a) * 0.16, R + Math.sin(a) * 0.16]);
  }
  const hull = convexHull(pts);
  const s = new THREE.Shape();
  hull.forEach(([x, y], i) => (i ? s.lineTo(x, y) : s.moveTo(x, y)));
  const g = new THREE.ExtrudeGeometry(s, { depth: thick, bevelEnabled: false });
  g.translate(0, 0, -thick / 2);
  g.rotateY(Math.PI / 2);
  return g;
}

export default {
  id: 'marineDiesel',
  name: 'Motore diesel navale lento a 2 tempi',
  short: 'Diesel navale 2 tempi',
  family: 'mare',
  vehicle: 'Nave portacontainer',
  examples: 'WinGD X92, MAN B&W G95ME‑C, Wärtsilä RT‑flex96C (Emma Mærsk): grandi portacontainer, petroliere e portarinfuse',
  tagline: 'Alto come un palazzo di cinque piani, gira a 80 giri al minuto ed è il motore termico più efficiente al mondo.',
  intro: {
    base:
      'È il motore delle grandi navi portacontainer: ogni cilindro è largo quasi un metro e il pistone percorre più di tre metri. Gira lentissimo, poco più di un giro al secondo, ed è collegato <b>direttamente all’elica</b>, senza cambio. Funziona a <b>due tempi</b>: a ogni giro ogni cilindro dà una spinta. Guarda la persona in basso per capire quanto è grande!',
    expert:
      'Diesel lento a 2 tempi a <b>testa a croce</b> con <b>lavaggio unidirezionale</b> (uniflow): l’aria entra dalle luci alla base della camicia, i gas escono dalla grande valvola di scarico in testa comandata idraulicamente. Sovralimentazione a pressione costante (≈ 4–4,5 bar assoluti), iniezione elettronica a ≈ 1 000 bar, pressione massima ≈ 200 bar. Con una corsa lunghissima (fino a ≈ 4 volte l’alesaggio) e un regime bassissimo supera il <b>50% di rendimento</b>: consumo specifico ≈ 160 g/kWh.',
  },
  specs: [
    { label: 'Carburante', value: 'Olio pesante (HFO), gasolio marino, GNL, metanolo' },
    { label: 'Ciclo', value: 'Diesel 2 tempi, lavaggio unidirezionale' },
    { label: 'Distribuzione', value: 'Luci di lavaggio + valvola di scarico idraulica; comando elettronico (ME, X) o ad albero a camme a catena (MC, RTA)' },
    { label: 'Alesaggio × corsa', value: '0,90–0,96 m × 2,5–3,5 m' },
    { label: 'Potenza', value: '6–14 cilindri in linea, ≈ 6 000 kW per cilindro, fino a ≈ 80 000 kW' },
    { label: 'Regime', value: '70 – 100 giri/min' },
    { label: 'Rendimento', value: '50 – 54 %' },
  ],
  variants: {
    columns: ['Motore', 'Alesaggio × corsa', 'Potenza', 'Comando iniezione e valvole'],
    rows: [
      ['MAN B&W 12G95ME‑C', '950 × 3 460 mm', '≈ 6 900 kW per cilindro', 'Elettro-idraulico, senza albero a camme'],
      ['WinGD 12X92', '920 × 3 468 mm', '≈ 6 450 kW per cilindro', 'Common rail elettronico, senza albero a camme'],
      ['Wärtsilä 14RT‑flex96C (Emma Mærsk)', '960 × 2 500 mm', '≈ 80 000 kW (14 cilindri)', 'Common rail elettronico'],
      ['Wärtsilä‑Sulzer 14RTA96C', '960 × 2 500 mm', '≈ 80 000 kW (14 cilindri)', 'Albero a camme'],
    ],
    note: 'I motori MAN B&W MC e Sulzer RTA hanno un albero a camme mosso da catena o ingranaggi che aziona pompe del combustibile e valvole; i ME, RT‑flex e X fanno tutto con elettronica e olio idraulico.',
  },
  options: [
    {
      id: 'control',
      label: 'Comando',
      default: 'me',
      choices: [
        {
          id: 'me',
          label: 'Elettronico (ME / X)',
          base: 'Niente albero a camme: un computer decide, cilindro per cilindro e giro per giro, quando iniettare il combustibile e quando aprire la valvola di scarico, usando olio ad alta pressione. Il motore si adatta al carico e consuma meno.',
          expert:
            'MAN B&W ME: per ogni cilindro un’unità idraulica (HCU) alimentata con olio a ≈ 200–300 bar da pompe idrauliche; valvole elettroniche comandano l’amplificatore di pressione del combustibile e l’attuatore della valvola di scarico. WinGD X e Wärtsilä RT‑flex usano un common rail del combustibile e uno dell’olio di comando. Fasatura, pressione e forma dell’iniezione si cambiano in marcia.',
        },
        {
          id: 'mc',
          label: 'Albero a camme (MC / RTA)',
          base: 'Una grande catena, in coda al motore, fa girare un albero a camme alla stessa velocità dell’albero motore. Ogni camma spinge una pompa del combustibile e una pompa che apre la valvola di scarico.',
          expert:
            'Nel 2 tempi l’albero a camme gira a rapporto 1:1 con l’albero motore. Pompe d’iniezione a stantuffo per cilindro e pompe idrauliche che aprono la valvola di scarico attraverso un tubo d’olio. La fasatura è fissata dal profilo delle camme (con una regolazione limitata dell’anticipo); per invertire la marcia un meccanismo cambia la fasatura delle pompe.',
        },
      ],
    },
  ],
  thumb: { orbit: [30, 12], zoom: 1.0, view: 'section' },
  metrics: { efficiency: 52, power: 60000, powerDensity: 0.035 },
  phases: [
    {
      id: 'compression',
      name: 'Compressione',
      range: [0, 0.278],
      parts: ['pistons', 'liners', 'exhaustValves', 'crossheads'],
      base: 'La grande valvola in alto si è chiusa e le finestre in basso sono coperte dal pistone. Il pistone sale e schiaccia l’aria fresca, che diventa caldissima: più di 500 gradi, abbastanza per accendere il combustibile senza candela.',
      expert:
        'Dalla chiusura della valvola di scarico (≈ 100° prima del PMS) al PMS. Il momento di chiusura, regolato elettronicamente, fissa il rapporto di compressione effettivo; a fine corsa si raggiungono ≈ 150–170 bar e oltre 500 °C. La testa a croce assorbe la spinta laterale della biella: pistone e camicia lavorano quasi senza forze trasversali.',
    },
    {
      id: 'combustion',
      name: 'Iniezione, combustione ed espansione',
      range: [0.278, 0.583],
      parts: ['injectors', 'pistons', 'pistonRods', 'crankshaft', 'cylinderCovers'],
      base: 'Gli iniettori spruzzano il combustibile in goccioline finissime nell’aria rovente: si accende da solo e brucia. Il gas spinge giù il pistone, l’asta, la testa a croce e la biella, che fanno girare l’albero e quindi l’elica.',
      expert:
        'Due o tre iniettori per cilindro, comandati elettronicamente (common rail o amplificatori idraulici) a ≈ 800–1 000 bar, da pochi gradi prima a ≈ 15–20° dopo il PMS. Pressione massima ≈ 180–200 bar, combustione quasi a pressione costante (ciclo Diesel/Sabathé). L’olio pesante va riscaldato a 130–150 °C per avere la viscosità giusta all’iniezione.',
    },
    {
      id: 'exhaust',
      name: 'Apertura dello scarico',
      range: [0.583, 0.667],
      parts: ['exhaustValves', 'exhaustReceiver', 'turbocharger'],
      base: 'Prima che il pistone arrivi in fondo, un pistoncino spinto dall’olio apre la grande valvola di scarico. I gas bruciati, ancora in pressione, corrono nel collettore e fanno girare la turbina del turbocompressore.',
      expert:
        'Valvola di scarico aperta ≈ 110° dopo il PMS da un attuatore idraulico e richiusa da una molla pneumatica. Lo scarico spontaneo (blowdown) porta la pressione nel cilindro sotto quella dell’aria di lavaggio prima che le luci si aprano. Il collettore a pressione costante smorza le pulsazioni e alimenta la turbina con gas a ≈ 400 °C.',
    },
    {
      id: 'scavenge',
      name: 'Lavaggio unidirezionale',
      range: [0.667, 0.889],
      parts: ['liners', 'scavengeReceiver', 'turbocharger', 'exhaustValves'],
      base: 'Il pistone scopre l’anello di finestre in fondo alla camicia: l’aria compressa dal turbo entra girando a vortice e sale, spingendo fuori dalla valvola i gas bruciati. Tutto il gas va in una sola direzione, dal basso verso l’alto: per questo si chiama lavaggio unidirezionale.',
      expert:
        'Luci di lavaggio aperte ≈ 136°–224°, inclinate tangenzialmente per imprimere uno swirl che stabilizza il fronte di lavaggio e aiuta la miscelazione dell’iniezione successiva. Il lavaggio uniflow lascia pochissimi gas residui e, soprattutto, non perde combustibile allo scarico: nell’aria di lavaggio non c’è carburante, al contrario del 2 tempi a carter pompa.',
    },
    {
      id: 'charge',
      name: 'Chiusura delle luci e carica',
      range: [0.889, 1],
      parts: ['exhaustValves', 'scavengeReceiver', 'liners'],
      base: 'Il pistone risale e copre di nuovo le finestre. La valvola di scarico resta aperta ancora un poco per finire di svuotare il cilindro, poi si chiude: il cilindro è pieno di aria fresca, pronto per un nuovo giro.',
      expert:
        'Luci chiuse a ≈ 224°, valvola di scarico chiusa a ≈ 260°. L’aria di lavaggio, scaldata a ≈ 200 °C dalla compressione nel turbo, passa per il refrigerante (fino a ≈ 40 °C) e il separatore di condensa: più fredda è, più massa d’aria entra nel cilindro e più bassi sono gli NOx.',
    },
  ],
  quiz: [
    {
      q: 'A cosa serve la testa a croce?',
      options: [
        'A raffreddare il pistone',
        'A scaricare la spinta laterale della biella sulle guide, invece che sulla camicia',
        'A far girare il turbocompressore',
        'A iniettare il combustibile',
      ],
      correct: 1,
      explain: 'La biella inclinata spinge di lato: con la testa a croce questa forza va su pattini e guide nel basamento, così pistone e camicia si consumano poco e il carter resta separato dal cilindro.',
    },
    {
      q: 'Da dove entra l’aria fresca nel cilindro?',
      options: [
        'Da una valvola di aspirazione in testa',
        'Dalle finestre (luci) alla base della camicia, scoperte dal pistone',
        'Dal carter, come in uno scooter',
        'Dagli iniettori',
      ],
      correct: 1,
      explain: 'L’aria del turbo entra dalle luci in basso quando il pistone è vicino al punto morto inferiore ed esce, insieme ai gas combusti, dall’unica valvola in alto: lavaggio unidirezionale.',
    },
    {
      q: 'Perché questo motore non ha bisogno del cambio?',
      options: [
        'Perché gira già lentamente, alla velocità giusta per l’elica, e può invertire il senso di rotazione',
        'Perché ha un solo cilindro',
        'Perché è elettrico',
        'Perché l’elica è ferma',
      ],
      correct: 0,
      explain: 'A 70–100 giri/min l’albero si collega direttamente all’elica, che rende al meglio proprio a bassa velocità. Per la retromarcia il motore si ferma e riparte nel verso opposto.',
    },
    {
      q: 'Quale rendimento raggiunge un grande diesel navale moderno?',
      options: ['Circa il 20%', 'Circa il 35%', 'Oltre il 50%', 'Circa il 90%'],
      correct: 2,
      explain: 'Con circa 160 g di combustibile per kWh supera il 50%: corsa lunghissima, regime basso, pressioni altissime e poche perdite di calore lo rendono il motore a combustione più efficiente.',
    },
  ],
  // Officina (see CONTRACT.md): the piston overhaul from the top, with the engine-room crane.
  // Simplified and educational, not a maintenance manual.
  disassembly: {
    intro: 'Revisione dall’alto: con il carroponte della sala macchine si apre la testa dei cilindri e si sfilano pistoni e camicie.',
    note: 'Sequenza semplificata a scopo didattico: a bordo si segue il manuale del costruttore, con attrezzi idraulici e di sollevamento dedicati, e si lavora su un cilindro alla volta.',
    parts: {
      // The two alternatives of the `control` option: the one that is hidden counts as off.
      hcu: { dir: [0, 0.5, 1], dist: 1.2 },
      camshaft: { dir: [0.6, 0.2, 1], dist: 1.4 },
      injectors: { after: ['hcu', 'camshaft'], dir: [0, 1, 0], dist: 0.9 },
      exhaustValveNuts: { motion: 'unscrew', turns: 5, out: 0.07, dir: [0, 1, 0], dist: 0.4 },
      exhaustValves: { after: ['exhaustValveNuts', 'hcu', 'camshaft'], dir: [0, 1, 0], dist: 1.2 },
      coverNuts: { motion: 'unscrew', turns: 6, out: 0.09, dir: [0, 1, 0], dist: 0.5, duration: 2.0 },
      cylinderCovers: { after: ['coverNuts', 'exhaustValves', 'injectors', 'hcu', 'camshaft'], dir: [0, 1, 0], dist: 1.3, duration: 1.2 },
      // Piston, rod and stuffing box leave together, lifted out through the liner.
      pistons: { after: ['cylinderCovers'], with: ['pistonRods', 'stuffingBox'], dir: [0, 1, 0], dist: 2.6, duration: 1.6 },
      pistonRods: { after: ['cylinderCovers'], with: ['pistons', 'stuffingBox'], dir: [0, 1, 0], dist: 2.6, duration: 1.6 },
      stuffingBox: { after: ['pistonRods'], dir: [0, 1, 0], dist: 2.6 },
      lubeQuills: { motion: 'unscrew', turns: 5, out: 0.1, dir: [0, 0.3, 1], dist: 0.4 },
      liners: { after: ['pistons', 'pistonRods', 'lubeQuills'], dir: [0, 1, 0], dist: 2.0, duration: 1.3 },
      cylinderFrame: { after: ['liners', 'cylinderCovers'] },
    },
    steps: [
      {
        id: 'control',
        name: 'Scollega il comando',
        base: 'Si staccano i tubi ad alta pressione che portano l’olio di comando alle valvole di scarico e il combustibile agli iniettori. Nel modello, per semplicità, si sposta tutto il gruppo di comando.',
        expert:
          'Prima si scarica la pressione dai circuiti idraulici e del combustibile, poi si scollegano i tubi ad alta pressione dalle teste dei cilindri. Sul motore vero le unità idrauliche (HCU) o l’albero a camme, con pompe e catena, restano al loro posto.',
        parts: ['hcu', 'camshaft'],
      },
      {
        id: 'injectors',
        name: 'Sfila gli iniettori',
        base: 'Si sfilano dall’alto gli iniettori, con il loro tubo del combustibile: sono infilati nel coperchio, attorno alla valvola di scarico.',
        expert:
          'Gli iniettori si revisionano a intervalli regolari: su un banco prova si controllano la pressione di apertura e la forma del getto. L’olio pesante si tiene caldo e in circolazione anche a motore fermo, altrimenti si addensa nei tubi.',
        parts: ['injectors'],
      },
      {
        id: 'valves',
        name: 'Togli le valvole di scarico',
        base: 'Si svitano i quattro dadi di ogni valvola e il carroponte la solleva tutta intera, con il suo attuatore.',
        expert:
          'La valvola esce completa del suo corpo e viene sostituita con una già revisionata: sede e fungo si rettificano in officina. È uno dei lavori più frequenti, perché la valvola lavora immersa in gas a centinaia di gradi.',
        parts: ['exhaustValveNuts', 'exhaustValves'],
      },
      {
        id: 'covers',
        name: 'Solleva i coperchi',
        base: 'Si allentano i grandi dadi con l’attrezzo idraulico e il carroponte solleva i coperchi: ora si vede dall’alto il cielo dei pistoni.',
        expert:
          'I dadi si allentano tutti insieme, con martinetti idraulici collegati alla stessa pompa: così il coperchio si scarica in modo uniforme. Ogni coperchio pesa diverse tonnellate; sul motore vero si stacca prima anche il tratto di tubo che porta i gas dalla valvola al collettore di scarico.',
        parts: ['coverNuts', 'cylinderCovers'],
      },
      {
        id: 'pistons',
        name: 'Sfila i pistoni',
        base: 'Il carroponte aggancia il pistone e lo tira su attraverso la camicia, insieme alla sua lunga asta e al premistoppa.',
        expert:
          'Prima, dalle porte del basamento, si svita il piede dell’asta dalla testa a croce e si libera la flangia del premistoppa; poi si porta il pistone verso il punto morto superiore, si monta l’attrezzo di sollevamento sul cielo e si sfila tutto dall’alto. Fuori si controllano fasce, cave e usura della camicia.',
        parts: ['pistons'],
      },
      {
        id: 'liners',
        name: 'Estrai le camicie',
        base: 'Tolti gli ugelli dell’olio, la camicia si sfila verso l’alto dal blocco cilindri, come un grande tubo.',
        expert:
          'È il coperchio a stringere la camicia nella sua sede sul blocco cilindri. Si scollegano ugelli di lubrificazione e acqua di raffreddamento e la si solleva con una traversa. Si sostituisce quando l’usura dell’alesaggio supera il limite del costruttore, di solito dopo decine di migliaia di ore.',
        parts: ['lubeQuills', 'liners'],
      },
    ],
  },
  camera: { position: [4.39, 3.02, 8.38], target: [0.25, 1.6, 0.0] },

  build(ctx) {
    const root = new THREE.Group();
    root.position.y = -0.2;

    // ---------- Bedplate with main bearing girders ----------
    const bedMat = cut(MAT.paint(0x56606b, 0.6));
    const bed = new THREE.Group();
    bed.add(boxRing(0.82, 0.78, 0.05, -0.85, -0.1, bedMat));
    const floor = new THREE.Mesh(new THREE.BoxGeometry(1.64, 0.05, 1.56), bedMat);
    floor.position.y = -0.83;
    bed.add(floor);
    const girder = new THREE.Shape();
    girder.moveTo(-0.72, -0.83);
    girder.lineTo(0.72, -0.83);
    girder.lineTo(0.72, 0);
    girder.lineTo(0.15, 0);
    girder.absarc(0, 0, 0.15, 0, Math.PI, true);
    girder.lineTo(-0.72, 0);
    girder.lineTo(-0.72, -0.83);
    const girderGeo = new THREE.ExtrudeGeometry(girder, { depth: 0.06, bevelEnabled: false, curveSegments: 24 });
    girderGeo.translate(0, 0, -0.03);
    girderGeo.rotateY(Math.PI / 2);
    const shellGeo = pipe(0.15, 0.12, 0.07, 32).rotateZ(Math.PI / 2);
    for (const x of [-0.72, -0.24, 0.24, 0.72]) {
      const g = new THREE.Mesh(girderGeo, bedMat);
      g.position.x = x;
      const shell = new THREE.Mesh(shellGeo, cut(MAT.brass()));
      shell.position.x = x;
      bed.add(g, shell);
    }
    // Thrust block at the aft end.
    const thrust = new THREE.Mesh(rbox(0.2, 0.85, 0.8, 0.04), bedMat);
    thrust.position.set(0.92, -0.38, 0);
    bed.add(thrust);
    // Top flange to the frame box, foot flange with holding-down bolts and chocks.
    bed.add(boxRing(0.86, 0.82, 0.1, -0.17, -0.1, bedMat));
    bed.add(boxRing(0.92, 0.88, 0.14, -0.85, -0.79, bedMat));
    const hwBolt = cut(MAT.darkSteel());
    const bedBolts = [];
    for (const z of [-0.85, 0.85]) bedBolts.push(...boltRow(-0.84, 0.84, 13, -0.1, z, [0, 1, 0]), ...boltRow(-0.84, 0.84, 9, -0.79, z, [0, 1, 0]));
    bed.add(bolts(bedBolts, hwBolt, 0.02, 0.022));
    const chockGeo = rbox(0.1, 0.04, 0.12, 0.01);
    for (const x of [-0.7, -0.35, 0, 0.35, 0.7]) {
      for (const z of [-0.86, 0.86]) {
        const c = new THREE.Mesh(chockGeo, cut(MAT.paint(0x3a2f26, 0.8)));
        c.position.set(x, -0.87, z);
        bed.add(c);
      }
    }
    root.add(
      ctx.part(bed, {
        id: 'bedplate',
        name: 'Basamento e cuscinetto reggispinta',
        base: 'La base d’acciaio su cui poggia tutto il motore, fissata al fondo della nave. Sostiene l’albero motore e, in coda, il cuscinetto che trasmette la spinta dell’elica allo scafo.',
        expert:
          'Basamento in lamiere d’acciaio saldate con traverse che portano i cuscinetti di banco (gusci con metallo bianco). Il cuscinetto reggispinta integrato a poppavia scarica sullo scafo la spinta assiale dell’elica, che per una grande portacontainer vale diverse centinaia di tonnellate.',
        explode: [0, -0.8, 0],
      })
    );

    // ---------- Frame box (A-frames) with crosshead guides ----------
    const frameMat = cut(MAT.paint(0x6a7480, 0.6));
    const frame = new THREE.Group();
    frame.add(boxRing(0.8, 0.7, 0.05, -0.1, 1.95, frameMat));
    const wallGeo = new THREE.BoxGeometry(0.05, 2.05, 1.36);
    for (const x of [-0.72, -0.24, 0.24, 0.72]) {
      const w = new THREE.Mesh(wallGeo, frameMat);
      w.position.set(x, 0.925, 0);
      frame.add(w);
    }
    const guideMat = cut(MAT.steel());
    const guideGeo = new THREE.BoxGeometry(0.03, 1.4, 0.025);
    for (const x of CYL_X) {
      for (const dx of [-0.195, 0.195]) {
        for (const z of [-0.0675, 0.0675]) {
          const g = new THREE.Mesh(guideGeo, guideMat);
          g.position.set(x + dx, 1.27, z);
          frame.add(g);
        }
      }
    }
    // Top flange, crankcase doors with relief valves, fasteners.
    frame.add(boxRing(0.84, 0.74, 0.08, 1.85, 1.91, frameMat));
    const doorGeo = rbox(0.3, 0.55, 0.035, 0.05);
    const reliefGeo = new THREE.CylinderGeometry(0.055, 0.06, 0.05, 20).rotateX(Math.PI / 2);
    const frameBolts = [];
    for (const x of CYL_X) {
      const d = new THREE.Mesh(doorGeo, frameMat);
      d.position.set(x, 0.8, 0.715);
      const rv = new THREE.Mesh(reliefGeo, cut(MAT.paint(0xb8432f, 0.5)));
      rv.position.set(x, 0.62, 0.755);
      frame.add(d, rv);
      for (let i = 0; i < 4; i++) {
        frameBolts.push([[x - 0.12, 0.58 + i * 0.147, 0.733], [0, 0, 1]], [[x + 0.12, 0.58 + i * 0.147, 0.733], [0, 0, 1]]);
      }
    }
    for (const z of [-0.84, 0.84]) frameBolts.push(...boltRow(-0.8, 0.8, 13, 1.91, z * 0.94, [0, 1, 0]));
    frame.add(bolts(frameBolts, cut(MAT.darkSteel()), 0.018, 0.02));
    root.add(
      ctx.part(frame, {
        id: 'frame',
        name: 'Montanti e guide della testa a croce',
        base: 'La “gabbia” d’acciaio sopra il basamento. Dentro scorrono su e giù le teste a croce, guidate da binari come un ascensore.',
        expert:
          'Telaio a montanti (A-frame) in lamiera saldata; tiranti lunghi oltre 10 m precaricano insieme basamento, montanti e blocco cilindri, così la struttura lavora a compressione. Le guide piane della testa a croce ricevono la componente laterale della forza di biella.',
        explode: [0, 0.2, -1.0],
      })
    );

    // ---------- Cylinder frame: scavenge air box, diaphragm, cooling jackets ----------
    const cylFrameMat = cut(MAT.paint(0x7b8590, 0.55));
    const cylFrame = new THREE.Group();
    cylFrame.add(boxRing(0.8, 0.55, 0.04, 1.95, 2.62, cylFrameMat));
    cylFrame.add(holedPlate(0.8, 0.7, 0.045, 1.91, 0.04, cylFrameMat));
    cylFrame.add(holedPlate(0.8, 0.55, 0.2, 2.62, 0.05, cylFrameMat));
    const hw = cut(MAT.darkSteel());
    const cfBolts = [];
    const doorGeo2 = new THREE.CylinderGeometry(0.1, 0.1, 0.03, 32).rotateX(Math.PI / 2);
    const lubGeo = rbox(0.1, 0.08, 0.05, 0.015);
    const studFlangeGeo = pipe(0.238, 0.213, 0.04, 40);
    const coverStuds = [];
    for (const x of CYL_X) {
      const j = new THREE.Mesh(pipe(0.215, 0.2, 0.69, 40), cylFrameMat);
      j.position.set(x, 3.015, 0);
      // Flange under the cover with the eight cover studs.
      const fl = new THREE.Mesh(studFlangeGeo, cylFrameMat);
      fl.position.set(x, 3.34, 0);
      cylFrame.add(j, fl);
      coverStuds.push(...boltCircle(x, 3.34, 0, 0.215, 8, 'y'));
      // Scavenge box inspection cover and cylinder lubricator.
      const d = new THREE.Mesh(doorGeo2, cylFrameMat);
      d.position.set(x, 2.28, 0.565);
      const lub = new THREE.Mesh(lubGeo, cut(MAT.paint(0x2f5f8a, 0.45)));
      lub.position.set(x + 0.1, 2.95, 0.21);
      cylFrame.add(d, lub);
      cfBolts.push(...boltCircle(x, 2.28, 0.58, 0.085, 8, 'z'));
    }
    // Tie-bolt nuts on the top deck.
    const nutGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.07, 6);
    const domeGeo = new THREE.SphereGeometry(0.035, 12, 6, 0, Math.PI * 2, 0, Math.PI / 2);
    for (const x of [-0.72, -0.24, 0.24, 0.72]) {
      for (const z of [-0.45, 0.45]) {
        const n = new THREE.Mesh(nutGeo, hw);
        n.position.set(x, 2.705, z);
        const dm = new THREE.Mesh(domeGeo, hw);
        dm.position.set(x, 2.74, z);
        cylFrame.add(n, dm);
      }
    }
    cylFrame.add(bolts(cfBolts, hw, 0.012, 0.015), bolts(coverStuds, hw, 0.011, 0.222));
    root.add(
      ctx.part(cylFrame, {
        id: 'cylinderFrame',
        name: 'Blocco cilindri e camera di lavaggio',
        base: 'Il cassone che circonda la parte bassa dei cilindri: è pieno di aria compressa che aspetta di entrare dalle finestre. Più in alto, l’acqua di raffreddamento circola intorno alle camicie.',
        expert:
          'Il blocco cilindri contiene lo spazio dell’aria di lavaggio a ≈ 3–3,5 bar di sovrapressione, separato dal basamento da un diaframma attraversato dalle aste dei pistoni. Le camicie sono raffreddate ad acqua dolce a ≈ 80–90 °C nella parte alta, la più calda.',
        explode: [0, 1.0, 0],
      })
    );

    // ---------- Liners with scavenge ports ----------
    const linerMat = cut(MAT.castIron());
    linerMat.color.set(0x7d858e);
    const liners = new THREE.Group();
    const webGeo = new THREE.BoxGeometry(0.022, PORT_TOP - PORT_BOT, 0.03);
    const quillMat = cut(MAT.brass());
    const quills = [];
    for (const x of CYL_X) {
      const lo = new THREE.Mesh(pipe(0.16, 0.142, PORT_BOT - LINER_BOT, 40), linerMat);
      lo.position.set(x, (LINER_BOT + PORT_BOT) / 2, 0);
      const hi = new THREE.Mesh(pipe(0.17, 0.142, COVER_Y - PORT_TOP, 40), linerMat);
      hi.position.set(x, (PORT_TOP + COVER_Y) / 2, 0);
      liners.add(lo, hi);
      // Port bars, angled tangentially to give the air its swirl.
      for (let i = 0; i < 16; i++) {
        const a = (i / 16) * Math.PI * 2;
        const w = new THREE.Mesh(webGeo, linerMat);
        w.position.set(x + Math.sin(a) * 0.152, PORT_MID, Math.cos(a) * 0.152);
        w.rotation.y = a + 0.45;
        liners.add(w);
      }
      // Cylinder oil quills: screwed into the liner through the cooling jacket (their own part, below).
      for (let i = 0; i < 8; i++) {
        const a = ((i + 0.5) / 8) * Math.PI * 2;
        quills.push([[x + Math.sin(a) * 0.215, 2.95, Math.cos(a) * 0.215], [Math.sin(a), 0, Math.cos(a)]]);
      }
    }
    root.add(
      ctx.part(liners, {
        id: 'liners',
        name: 'Camicie con luci di lavaggio',
        base: 'I tubi dentro cui scorrono i pistoni. In basso hanno un anello di finestre inclinate: quando il pistone scende fin lì, l’aria entra girando come in un vortice.',
        expert:
          'Camicie in ghisa legata, alesaggio ≈ 0,92–0,96 m, con circa 20–30 luci di lavaggio tangenziali alla base. La lubrificazione dei cilindri è separata da quella del motore: un lubrificatore elettronico inietta olio cilindri alcalino (per neutralizzare l’acido solforico dello zolfo) da 6–10 ugelli a ogni giro, ≈ 0,6–1 g/kWh.',
        explode: [0, 1.5, 0],
      }),
      ctx.part(fasteners(plugGeo(0.014, 0.03, 0.055), quills, quillMat), {
        id: 'lubeQuills',
        name: 'Ugelli dell’olio cilindri',
        base: 'Piccoli ugelli avvitati nella camicia, tutto intorno al cilindro: da qui entra l’olio che lubrifica le fasce del pistone.',
        expert:
          'Valvole di non ritorno (in gergo “quills”) che attraversano la camicia d’acqua e si avvitano nella camicia del cilindro: il lubrificatore vi manda una piccola dose di olio cilindri a ogni giro, o ogni pochi giri. Finché sono avvitate, la camicia non si può sfilare.',
        explode: [0, 1.5, 0],
      })
    );

    // ---------- Cylinder covers and exhaust valve cages ----------
    const coverMat = cut(MAT.darkSteel());
    const covers = new THREE.Group();
    const coverGeo = solidLathe([[0.085, 3.4], [0.142, COVER_Y], [0.25, COVER_Y], [0.25, 3.52], [0.085, 3.52]], 48);
    const coverNuts = [];
    const valveStuds = [];
    const cageGeo = pipe(0.11, 0.085, 0.34, 36);
    for (const x of CYL_X) {
      const c = new THREE.Mesh(coverGeo, coverMat);
      c.position.x = x;
      const cage = new THREE.Mesh(cageGeo, coverMat);
      cage.position.set(x, 3.69, 0);
      covers.add(c, cage);
      coverNuts.push(...boltCircle(x, 3.52, 0, 0.215, 8, 'y'));
      valveStuds.push(...boltCircle(x, 3.85, 0, 0.098, 4, 'y'));
      // Starting air valve and its branch to the manifold.
      const sav = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.035, 0.1, 16), coverMat);
      sav.position.set(x, 3.57, 0.17);
      covers.add(sav, tube([v3(x, 3.62, 0.17), v3(x, 3.7, 0.22), v3(x, 3.74, 0.3)], 0.018, coverMat, 12));
    }
    // Studs of the exhaust valve housings (the cover nuts and the valve nuts are their own parts).
    covers.add(bolts(valveStuds, coverMat, 0.008, 0.066));
    const startAir = new THREE.Mesh(solidLathe([[0.04, -0.78], [0.045, -0.78], [0.045, 0.78], [0.04, 0.78], [0.0, 0.78], [0, -0.78]], 20).rotateZ(Math.PI / 2), coverMat);
    startAir.position.set(0, 3.76, 0.3);
    covers.add(startAir);
    root.add(
      ctx.part(covers, {
        id: 'cylinderCovers',
        name: 'Coperchi dei cilindri e gabbie valvola',
        base: 'I pesanti “tappi” d’acciaio che chiudono in alto ogni cilindro. Al centro ospitano la valvola di scarico, intorno gli iniettori.',
        expert:
          'Coperchi in acciaio forgiato, raffreddati con fori trivellati, fissati da prigionieri idraulici. Devono reggere ≈ 200 bar su un diametro di quasi un metro: una forza di oltre 1 300 tonnellate a ogni giro.',
        explode: [0, 2.0, 0],
      }),
      ctx.part(fasteners(studNut(0.025, 0.034), coverNuts, cut(MAT.chrome())), {
        id: 'coverNuts',
        name: 'Dadi dei coperchi',
        base: 'Otto grandi dadi per cilindro stringono ogni coperchio sui suoi prigionieri. Sono così grossi che nessuna chiave a mano basterebbe: per allentarli si usa la pressione dell’olio.',
        expert:
          'Dadi serrati idraulicamente: un martinetto ad anello, alimentato con olio ad altissima pressione, tira il prigioniero e lo allunga elasticamente; a quel punto il dado gira a mano e, tolta la pressione, il prigioniero resta teso. Così tutti i prigionieri hanno lo stesso precarico, senza attrito e senza torsione.',
        explode: [0, 2.0, 0],
      })
    );

    // ---------- Exhaust valves (hydraulically opened) ----------
    const valveGeo = lathe([[0, 0], [0.075, 0], [0.075, 0.012], [0.03, 0.04], [0.014, 0.08], [0.014, 0.48], [0, 0.48]], 32);
    const valveMat = MAT.titanium();
    const exValves = new THREE.Group();
    const valveNuts = [];
    const valveFlangeGeo = new THREE.CylinderGeometry(0.125, 0.125, 0.024, 28);
    const valves = CYL_X.map((x) => {
      const v = new THREE.Mesh(valveGeo, valveMat);
      v.position.set(x, VALVE_Y, 0);
      exValves.add(v);
      const act = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.08, 0.2, 24), MAT.paint(0x2b3440, 0.5));
      act.position.set(x, 3.96, 0);
      // Housing flange sitting on the cage, held by four nuts.
      const fl = new THREE.Mesh(valveFlangeGeo, MAT.paint(0x2b3440, 0.5));
      fl.position.set(x, 3.872, 0);
      exValves.add(fl);
      valveNuts.push(...boltCircle(x, 3.884, 0, 0.098, 4, 'y'));
      const pipeH = tube([v3(x + 0.06, 3.98, 0), v3(x + 0.14, 4.02, -0.1), v3(x + 0.16, 3.9, -0.3)], 0.012, MAT.chrome(), 16);
      exValves.add(act, pipeH);
      return v;
    });
    root.add(
      ctx.part(exValves, {
        id: 'exhaustValves',
        name: 'Valvole di scarico idrauliche',
        base: 'Una sola grande valvola per cilindro, larga quasi mezzo metro. Non c’è albero a camme: la apre un pistoncino spinto da olio ad alta pressione e la richiude un cuscino d’aria.',
        expert:
          'Valvola a fungo in Nimonic o con riporto in Stellite, diametro ≈ 0,45 m, alzata ≈ 120 mm. Attuatore idraulico a controllo elettronico (motori ME/RT-flex) e molla pneumatica di richiamo; alette sullo stelo la fanno ruotare a ogni apertura per distribuire usura e calore sulla sede.',
        explode: [0, 2.5, 0],
      }),
      ctx.part(fasteners(studNut(0.018, 0.026), valveNuts, MAT.chrome()), {
        id: 'exhaustValveNuts',
        name: 'Dadi delle valvole di scarico',
        base: 'Quattro dadi tengono ogni valvola di scarico, con il suo attuatore, sopra il coperchio del cilindro.',
        expert:
          'La valvola di scarico completa (corpo, sede, fungo, attuatore idraulico e molla pneumatica) è un blocco unico fissato al coperchio da quattro prigionieri con dadi serrati idraulicamente: si smonta intera e si revisiona in officina, mentre al suo posto se ne monta una di scorta.',
        explode: [0, 2.5, 0],
      })
    );

    // ---------- Injectors and fuel rail ----------
    const injectors = new THREE.Group();
    const injMat = MAT.paint(0x30363e, 0.45);
    const injBody = new THREE.CylinderGeometry(0.018, 0.014, 0.2, 12).translate(0, 0.1, 0);
    const sprays = [];
    const sprayGeo = new THREE.ConeGeometry(0.035, 0.12, 14, 1, true).translate(0, -0.06, 0);
    const down = new THREE.Vector3(0, -1, 0);
    CYL_X.forEach((x, k) => {
      sprays[k] = [];
      for (const s of [-1, 1]) {
        const tip = v3(x + s * 0.11, 3.385, -0.05);
        const inj = new THREE.Mesh(injBody, injMat);
        inj.position.copy(tip);
        inj.rotation.z = -s * 0.35; // lean outward, away from the valve cage
        injectors.add(inj);
        injectors.add(tube([v3(x + s * 0.175, 3.56, -0.06), v3(x + s * 0.2, 3.55, -0.2), v3(x + s * 0.18, 3.51, -0.3)], 0.01, MAT.chrome(), 12));
        const spray = new THREE.Mesh(sprayGeo, MAT.gas(0xffc04d, 0.7, true));
        spray.position.copy(tip);
        const dir = v3(x - s * 0.02, 3.3, 0.04).sub(tip).normalize();
        spray.quaternion.setFromUnitVectors(down, dir);
        spray.visible = false;
        root.add(spray);
        sprays[k].push(spray);
      }
    });
    const rail = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 1.5, 16).rotateZ(Math.PI / 2), MAT.chrome());
    rail.position.set(0, 3.5, -0.32);
    injectors.add(rail);
    root.add(
      ctx.part(injectors, {
        id: 'injectors',
        name: 'Iniettori e rail del combustibile',
        base: 'Due iniettori per cilindro spruzzano il combustibile nell’aria rovente, come un nebulizzatore potentissimo. Nessuna candela: il combustibile si accende da solo per il calore.',
        expert:
          'Due o tre iniettori disposti intorno alla valvola di scarico, con getti tangenziali che seguono lo swirl. Pressione ≈ 800–1 000 bar generata da amplificatori idraulici o common rail, fasatura e quantità decise dalla centralina a ogni giro. Nei dual-fuel ad alta pressione (ME‑GI) il gas naturale è iniettato a ≈ 300 bar dopo una piccola iniezione pilota di gasolio.',
        explode: [0, 2.2, 0.5],
      })
    );

    // ---------- Pistons, piston rods, stuffing boxes ----------
    const pistonGeo = lathe(
      [[0, 0.16], [0.06, 0.17], [0.12, 0.21], [0.137, CROWN], [0.139, 0.2], [0.139, 0], [0.12, 0], [0.12, 0.13], [0, 0.13]],
      40
    );
    const pRingGeo = new THREE.TorusGeometry(0.14, 0.004, 4, 40).rotateX(Math.PI / 2);
    const pistonGroup = new THREE.Group();
    const pistons = CYL_X.map((x) => {
      const p = new THREE.Group();
      p.add(new THREE.Mesh(pistonGeo, MAT.steel()));
      for (const y of [0.19, 0.17, 0.15, 0.13]) {
        const r = new THREE.Mesh(pRingGeo, MAT.darkSteel());
        r.position.y = y;
        p.add(r);
      }
      p.position.x = x;
      pistonGroup.add(p);
      return p;
    });
    root.add(
      ctx.part(pistonGroup, {
        id: 'pistons',
        name: 'Pistoni',
        base: 'Dischi d’acciaio larghi quasi un metro: ricevono la spinta della combustione. Sono raffreddati dall’interno con olio, perché il fuoco sopra di loro è caldissimo.',
        expert:
          'Cielo in acciaio al cromo-molibdeno, leggermente concavo, raffreddato a olio tramite l’asta; mantello corto perché le spinte laterali le prende la testa a croce. Quattro o cinque fasce elastiche. Velocità media del pistone ≈ 8–9 m/s, simile a quella di un motore d’auto.',
        explode: [0, 1.3, 1.3],
      })
    );

    const prodGroup = new THREE.Group();
    const prodGeo = new THREE.CylinderGeometry(0.04, 0.04, ROD, 20).translate(0, ROD / 2, 0);
    const prods = CYL_X.map((x) => {
      const r = new THREE.Group();
      r.add(new THREE.Mesh(prodGeo, MAT.chrome()));
      const foot = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.04, 24), MAT.steel());
      foot.position.y = 0.09;
      r.add(foot);
      r.position.x = x;
      prodGroup.add(r);
      return r;
    });
    root.add(
      ctx.part(prodGroup, {
        id: 'pistonRods',
        name: 'Aste dei pistoni',
        base: 'Lunghe aste dritte che collegano ogni pistone alla testa a croce. Si muovono solo su e giù, senza mai inclinarsi.',
        expert:
          'Asta in acciaio forgiato lunga ≈ 4 m, cava per il passaggio dell’olio di raffreddamento del pistone. Restando sempre verticale può attraversare a tenuta il diaframma tra camera di lavaggio e basamento.',
        explode: [0, 0.7, 1.5],
      })
    );

    const stuffing = new THREE.Group();
    for (const x of CYL_X) {
      const s = new THREE.Mesh(pipe(0.075, 0.042, 0.14, 28), cut(MAT.brass()));
      s.position.set(x, 1.86, 0);
      stuffing.add(s);
    }
    root.add(
      ctx.part(stuffing, {
        id: 'stuffingBox',
        name: 'Premistoppa delle aste',
        base: 'Una guarnizione speciale attorno a ogni asta: tiene separati l’olio pulito del basamento e lo sporco della zona dei cilindri.',
        expert:
          'Premistoppa con anelli di tenuta e raschiaolio: impedisce all’aria di lavaggio e ai residui acidi della combustione di contaminare l’olio di sistema, e all’olio di sistema di risalire. È ciò che permette di usare un olio cilindri dedicato e un olio motore separato.',
        explode: [0, 0.4, 1.1],
      })
    );

    // ---------- Crossheads ----------
    const xhGroup = new THREE.Group();
    const xhPinGeo = new THREE.CylinderGeometry(0.075, 0.075, 0.36, 24).rotateZ(Math.PI / 2);
    const shoeGeo = new THREE.BoxGeometry(0.04, 0.22, 0.1);
    const crossheads = CYL_X.map((x) => {
      const c = new THREE.Group();
      c.add(new THREE.Mesh(xhPinGeo, MAT.chrome()));
      for (const dx of [-0.19, 0.19]) {
        const sh = new THREE.Mesh(shoeGeo, MAT.brass());
        sh.position.x = dx;
        c.add(sh);
      }
      c.position.x = x;
      xhGroup.add(c);
      return c;
    });
    root.add(
      ctx.part(xhGroup, {
        id: 'crossheads',
        name: 'Testa a croce e pattini',
        base: 'Il “carrello” che corre su e giù sulle guide, tra l’asta del pistone e la biella. Prende lui le spinte di lato della biella, così il pistone scivola dritto e non consuma la camicia.',
        expert:
          'Perno della testa a croce con pattini in metallo bianco che scorrono su guide lubrificate. La biella si collega qui e non al pistone: la forza laterale (fino a ≈ 15–20% della forza sul pistone) va sulle guide, mai sulla camicia. È la soluzione che rende possibili corse di 3 m e oltre.',
        explode: [0, 0.3, 1.7],
      })
    );

    // ---------- Conrods ----------
    const rodMat = MAT.steel();
    const rodGroup = new THREE.Group();
    const rodBeam = new THREE.BoxGeometry(0.09, L - 0.3, 0.1).translate(0, L / 2, 0);
    const bigEnd = new THREE.TorusGeometry(0.15, 0.045, 10, 32).rotateY(Math.PI / 2);
    const topEnd = new THREE.CylinderGeometry(0.1, 0.1, 0.16, 24).rotateZ(Math.PI / 2).translate(0, L, 0);
    const rods = CYL_X.map((x) => {
      const r = new THREE.Group();
      r.add(new THREE.Mesh(rodBeam, rodMat), new THREE.Mesh(bigEnd, rodMat), new THREE.Mesh(topEnd, rodMat));
      r.position.x = x;
      rodGroup.add(r);
      return r;
    });
    root.add(
      ctx.part(rodGroup, {
        id: 'conrods',
        name: 'Bielle',
        base: 'Le braccia che collegano la testa a croce all’albero motore: trasformano il su-e-giù in rotazione.',
        expert:
          'Bielle in acciaio forgiato lunghe ≈ 4 m, con cuscinetto di testa (sul perno di manovella) e di piede (sulla testa a croce) in metallo bianco. Rapporto manovella/biella ≈ 0,4, più alto che nei motori veloci.',
        explode: [0, -0.1, 2.0],
      })
    );

    // ---------- Crankshaft ----------
    const crank = new THREE.Group();
    const crankMat = MAT.steel();
    const journalGeo = new THREE.CylinderGeometry(0.12, 0.12, 1, 32).rotateZ(Math.PI / 2);
    for (const [x0, x1] of [[-0.8, -0.655], [-0.305, -0.175], [0.175, 0.305], [0.655, 0.82]]) {
      const j = new THREE.Mesh(journalGeo, crankMat);
      j.scale.x = x1 - x0;
      j.position.x = (x0 + x1) / 2;
      crank.add(j);
    }
    const webG = webGeometry(0.07);
    const pinGeo = new THREE.CylinderGeometry(0.11, 0.11, 0.21, 28).rotateZ(Math.PI / 2);
    CYL_X.forEach((x, k) => {
      const throwG = new THREE.Group();
      for (const dx of [-0.14, 0.14]) {
        const w = new THREE.Mesh(webG, crankMat);
        w.position.x = dx;
        throwG.add(w);
      }
      const pin = new THREE.Mesh(pinGeo, MAT.chrome());
      pin.position.y = R;
      throwG.add(pin);
      throwG.position.x = x;
      throwG.rotation.x = -2 * Math.PI * OFFSET[k];
      crank.add(throwG);
    });
    root.add(
      ctx.part(crank, {
        id: 'crankshaft',
        name: 'Albero motore',
        base: 'L’albero a gomiti che gira con l’elica. Per un motore da 12 cilindri è lungo come un autobus e pesa centinaia di tonnellate.',
        expert:
          'Albero semi-composito: perni di banco e manovelle forgiati, montati a caldo (calettamento per interferenza). Per 12–14 cilindri supera i 25 m di lunghezza e le 300 t. Le manovelle sono sfasate per avere una spinta regolare; le vibrazioni torsionali si controllano con smorzatori e scelta dell’ordine di accensione.',
        explode: [0, -0.6, 2.1],
      })
    );

    // ---------- Exhaust receiver ----------
    const exMat = cut(MAT.paint(0x8a7a6a, 0.55));
    exMat.metalness = 0.6;
    const exRec = new THREE.Group();
    const recv = new THREE.Mesh(shellLathe([[0.06, -0.9], [0.18, -0.88], [0.24, -0.82], [0.24, 0.82], [0.18, 0.88], [0.06, 0.9]], 0.02, 40).rotateZ(Math.PI / 2), exMat);
    recv.position.set(-0.12, 3.72, -0.62);
    exRec.add(recv);
    const bandMat = cut(MAT.steel());
    for (let x = -0.85; x <= 0.65; x += 0.3) {
      const b = new THREE.Mesh(new THREE.TorusGeometry(0.243, 0.01, 6, 40).rotateY(Math.PI / 2), bandMat);
      b.position.set(x, 3.72, -0.62);
      exRec.add(b);
    }
    // Supports standing on the scavenge air receiver.
    const supGeo = rbox(0.06, 1.0, 0.08, 0.015);
    for (const x of [-0.72, 0.0, 0.6]) {
      const s = new THREE.Mesh(supGeo, cut(MAT.paint(0x56606b, 0.6)));
      s.position.set(x, 2.98, -0.66);
      exRec.add(s);
    }
    const bellowGeo = new THREE.TorusGeometry(0.083, 0.012, 6, 24);
    for (const x of CYL_X) {
      exRec.add(tube([v3(x, 3.62, -0.06), v3(x, 3.66, -0.25), v3(x, 3.72, -0.5)], 0.075, exMat, 24));
      for (let i = 0; i < 3; i++) {
        const b = new THREE.Mesh(bellowGeo, bandMat);
        b.position.set(x, 3.685 + i * 0.006, -0.32 - i * 0.03);
        b.rotation.x = -0.12;
        exRec.add(b);
      }
    }
    exRec.add(tube([v3(-0.95, 3.72, -0.62), v3(-1.12, 3.7, -0.45), v3(-1.25, 3.62, -0.22), v3(-1.3, 3.6, -0.08)], 0.12, exMat, 32));
    root.add(
      ctx.part(exRec, {
        id: 'exhaustReceiver',
        name: 'Collettore di scarico',
        base: 'Un grande tubo che raccoglie i gas di tutti i cilindri e li porta al turbocompressore. Fa da “polmone”: i gas arrivano a sbuffi, ma ne escono con un flusso regolare.',
        expert:
          'Collettore a pressione costante, coibentato, con volume di molte volte la cilindrata di un cilindro: smorza le pulsazioni e alimenta la turbina in modo stazionario (≈ 3,5–4 bar, ≈ 400 °C a pieno carico). Dopo la turbina i gas a ≈ 230–260 °C passano spesso in una caldaia a recupero per produrre vapore.',
        explode: [0, 1.8, -0.9],
      })
    );

    // ---------- Turbocharger ----------
    const tcMat = cut(MAT.castIron());
    tcMat.color.set(0x6b7079);
    const turbo = new THREE.Group();
    const housing = (rIn, rOut, w) =>
      shellLathe([[rIn, -w], [rOut, -w * 0.8], [rOut + 0.01, 0], [rOut, w * 0.8], [rIn, w]], 0.018, 40).rotateZ(-Math.PI / 2);
    const scrollMat = cut(MAT.castIron(), { cap: false });
    scrollMat.color.set(0x6b7079);
    const turbH = new THREE.Mesh(housing(0.1, 0.25, 0.12), tcMat);
    turbH.position.set(-1.3, 3.6, 0);
    const tcPedestal = new THREE.Mesh(rbox(0.34, 0.62, 0.26, 0.04), tcMat);
    tcPedestal.position.set(-1.56, 3.16, -0.12);
    const outFlange = new THREE.Mesh(pipe(0.2, 0.135, 0.04, 32), tcMat);
    outFlange.position.set(-1.3, 4.33, 0);
    turbo.add(tcPedestal, outFlange);
    turbo.add(
      bolts(
        [...boltCircle(-1.3, 4.35, 0, 0.175, 10, 'y'), ...boltCircle(-1.43, 3.6, 0, 0.27, 12, 'x', 1), ...boltCircle(-1.69, 3.6, 0, 0.3, 12, 'x', -1)],
        cut(MAT.darkSteel()),
        0.016,
        0.018
      )
    );
    const turbScroll = new THREE.Mesh(new THREE.TorusGeometry(0.24, 0.07, 12, 40).rotateY(Math.PI / 2), scrollMat);
    turbScroll.position.set(-1.3, 3.6, 0);
    const brgH = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.13, 0.32, 28).rotateZ(Math.PI / 2), tcMat);
    brgH.position.set(-1.56, 3.6, 0);
    const compH = new THREE.Mesh(housing(0.12, 0.28, 0.11), tcMat);
    compH.position.set(-1.82, 3.6, 0);
    const compScroll = new THREE.Mesh(new THREE.TorusGeometry(0.28, 0.08, 12, 40).rotateY(Math.PI / 2), scrollMat);
    compScroll.position.set(-1.82, 3.6, 0);
    const silencer = new THREE.Mesh(pipe(0.32, 0.3, 0.22, 40).rotateZ(Math.PI / 2), cut(MAT.paint(0x3c444d, 0.7)));
    silencer.position.set(-2.06, 3.6, 0);
    const outlet = new THREE.Mesh(pipe(0.15, 0.135, 0.5, 32), tcMat);
    outlet.position.set(-1.3, 4.1, 0);
    turbo.add(turbH, turbScroll, brgH, compH, compScroll, silencer, outlet);
    const rotor = new THREE.Group();
    rotor.add(new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.62, 12).rotateZ(Math.PI / 2), MAT.chrome()));
    const tWheel = ringX(blade(0.12, 0.07, 0.012, 0.6), MAT.titanium(), 22, 0.07);
    tWheel.position.x = 0.26;
    const cWheel = ringX(blade(0.15, 0.09, 0.01, 0.9), MAT.aluminum(), 16, 0.06);
    cWheel.position.x = -0.26;
    const hubGeo = new THREE.CylinderGeometry(0.07, 0.05, 0.1, 20).rotateZ(Math.PI / 2);
    for (const x of [-0.26, 0.26]) {
      const h = new THREE.Mesh(hubGeo, MAT.steel());
      h.position.x = x;
      rotor.add(h);
    }
    rotor.add(tWheel, cWheel);
    rotor.position.set(-1.56, 3.6, 0);
    turbo.add(rotor);
    root.add(
      ctx.part(turbo, {
        id: 'turbocharger',
        name: 'Turbocompressore',
        base: 'Una girandola doppia: i gas di scarico fanno girare la turbina e, sullo stesso albero, un ventilatore (il compressore) spinge aria nel motore. Così riusa energia che altrimenti andrebbe persa dal camino.',
        expert:
          'Turbocompressore a flusso radiale/assiale, uno ogni 4–6 cilindri, rapporto di compressione ≈ 4,5–5, rendimento complessivo oltre il 70%. Gira a ≈ 10 000–15 000 giri/min. A bassi carichi un soffiante elettrico ausiliario garantisce l’aria di lavaggio, perché il 2 tempi non può aspirare da solo.',
        explode: [-0.9, 1.6, 0],
      })
    );

    // ---------- Air cooler and scavenge air receiver ----------
    const scMat = cut(MAT.paint(0x4d6b80, 0.55));
    const scav = new THREE.Group();
    scav.add(tube([v3(-1.8, 3.32, -0.05), v3(-1.76, 3.1, -0.25), v3(-1.62, 2.88, -0.35)], 0.1, scMat, 24));
    const cooler = new THREE.Mesh(rbox(0.5, 0.6, 0.5, 0.05), cut(MAT.paint(0x4d6b80, 0.55), { cap: false }));
    cooler.position.set(-1.55, 2.55, -0.35);
    scav.add(cooler);
    // Cooling water pipes down to the floor, with flanges.
    const waterMat = cut(MAT.paint(0x3f7a4a, 0.5));
    const flGeo = pipe(0.075, 0.04, 0.03, 24);
    for (const x of [-1.68, -1.42]) {
      const p = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 3.1, 16), waterMat);
      p.position.set(x, 0.7, -0.45);
      scav.add(p);
      for (const y of [2.24, 1.0, -0.84]) {
        const f = new THREE.Mesh(flGeo, waterMat);
        f.position.set(x, y, -0.45);
        scav.add(f);
      }
    }
    for (let i = 0; i < 7; i++) {
      const f = new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.52, 0.008), MAT.copper());
      f.position.set(-1.55, 2.55, -0.56 + i * 0.06);
      scav.add(f);
    }
    const sRecv = new THREE.Mesh(lathe([[0, -1.04], [0.18, -1.02], [0.18, 1.02], [0, 1.04]], 36).rotateZ(Math.PI / 2), scMat);
    sRecv.position.set(-0.27, 2.3, -0.72);
    scav.add(sRecv);
    root.add(
      ctx.part(scav, {
        id: 'scavengeReceiver',
        name: 'Refrigerante e collettore dell’aria di lavaggio',
        base: 'L’aria che esce dal turbo è calda: passa in un radiatore ad acqua che la raffredda, poi si raccoglie in un grande tubo che la distribuisce a tutti i cilindri.',
        expert:
          'Refrigerante a fasci tubieri alettati raffreddato ad acqua (da ≈ 200 °C a ≈ 40 °C) con separatore di condensa: in un motore da 80 MW si separano tonnellate d’acqua al giorno nei climi umidi. Il collettore alimenta le camere di lavaggio attraverso valvole di non ritorno.',
        explode: [-0.4, 0.8, -1.0],
      })
    );

    // ---------- Propeller shaft and propeller fragment ----------
    const bronze = MAT.brass();
    bronze.color.set(0xa8814f);
    bronze.roughness = 0.38;
    const shaftLine = new THREE.Group();
    const spin = new THREE.Group();
    spin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 1.65, 24).rotateZ(Math.PI / 2).translate(1.6, 0, 0), MAT.steel()));
    const collar = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.06, 36).rotateZ(Math.PI / 2), MAT.chrome());
    collar.position.x = 0.92;
    const turning = new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.42, 0.07, 56).rotateZ(Math.PI / 2), MAT.darkSteel());
    turning.position.x = 1.1;
    spin.add(collar, turning);
    const toothGeo = new THREE.BoxGeometry(0.06, 0.035, 0.03);
    for (let i = 0; i < 48; i++) {
      const a = (i / 48) * Math.PI * 2;
      const t = new THREE.Mesh(toothGeo, MAT.darkSteel());
      t.position.set(1.1, Math.cos(a) * 0.43, Math.sin(a) * 0.43);
      t.rotation.x = a;
      spin.add(t);
    }
    const mark = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.24, 0.05), MAT.paint(0xff7a1a));
    mark.position.set(1.1, 0.24, 0);
    spin.add(mark);
    const hub = new THREE.Mesh(lathe([[0.08, -0.16], [0.17, -0.12], [0.18, 0.08], [0.1, 0.22], [0, 0.26]], 32).rotateZ(-Math.PI / 2), bronze);
    hub.position.x = 2.45;
    const bladeGeo = blade(0.8, 0.34, 0.035, 0.7).rotateY(0.8);
    const blades = ringX(bladeGeo, bronze, 5, 0.14);
    blades.position.x = 2.45;
    spin.add(hub, blades);
    shaftLine.add(spin);
    const pedestal = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.7, 0.36), cut(MAT.paint(0x56606b, 0.6)));
    pedestal.position.set(1.65, -0.42, 0);
    const brgRing = new THREE.Mesh(pipe(0.16, 0.095, 0.14, 32).rotateZ(Math.PI / 2), cut(MAT.paint(0x56606b, 0.6)));
    brgRing.position.x = 1.65;
    const sternTube = new THREE.Mesh(pipe(0.16, 0.1, 0.3, 32).rotateZ(Math.PI / 2), cut(MAT.castIron()));
    sternTube.position.x = 2.12;
    shaftLine.add(pedestal, brgRing, sternTube);
    root.add(
      ctx.part(shaftLine, {
        id: 'propellerShaft',
        name: 'Linea d’asse ed elica',
        base: 'L’albero motore prosegue direttamente nell’asse dell’elica: niente cambio, niente frizione. L’elica gira alla stessa velocità del motore. Qui ne vedi solo il mozzo e l’attacco delle pale: quella vera è larga quasi 10 metri!',
        expert:
          'Accoppiamento diretto: il motore lento gira al regime ottimale dell’elica a passo fisso (diametro 9–10 m, ≈ 100 t in bronzo al nichel-alluminio). Per la retromarcia il motore si arresta e viene riavviato al contrario con aria compressa a 30 bar. Il volano di viraggio (turning wheel) permette di ruotare lentamente il motore per la manutenzione.',
        explode: [1.1, 0, 0],
      })
    );

    // ---------- Human figure for scale (1.8 m ≈ 0.54 units) ----------
    const human = new THREE.Group();
    const suit = MAT.paint(0xff7a1a, 0.7);
    const dark = MAT.paint(0x2a2f36, 0.8);
    const skin = MAT.paint(0xd9a982, 0.8);
    for (const s of [-1, 1]) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.25, 0.06), suit);
      leg.position.set(s * 0.033, 0.135, 0);
      const boot = new THREE.Mesh(new THREE.BoxGeometry(0.055, 0.025, 0.08), dark);
      boot.position.set(s * 0.033, 0.0125, 0.01);
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.034, 0.21, 0.04), suit);
      arm.position.set(s * 0.098, 0.335, 0);
      arm.rotation.z = s * 0.08;
      human.add(leg, boot, arm);
    }
    const torso = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.21, 0.08), suit);
    torso.position.y = 0.36;
    const headM = new THREE.Mesh(new THREE.SphereGeometry(0.042, 12, 8), skin);
    headM.position.y = 0.5;
    const helmet = new THREE.Mesh(new THREE.SphereGeometry(0.047, 12, 6, 0, Math.PI * 2, 0, Math.PI / 2), MAT.paint(0xf2f2ee, 0.4));
    helmet.position.y = 0.51;
    human.add(torso, headM, helmet);
    human.position.set(1.4, -0.85, 0.65);
    human.rotation.y = -0.5;
    root.add(
      ctx.part(human, {
        id: 'human',
        name: 'Persona (per la scala)',
        base: 'Un tecnico alto 1,80 m, disegnato alla stessa scala del motore. Il motore intero è alto come un palazzo di cinque piani: i macchinisti lo ispezionano camminando su passerelle a più livelli.',
        expert:
          'Altezza del motore dal basamento alla sommità ≈ 15–16 m, lunghezza ≈ 25–30 m per 12–14 cilindri, massa ≈ 2 000 t. Il carter è ispezionabile dall’interno: si entra fisicamente nel basamento per controllare cuscinetti e teste a croce.',
        explode: [0.8, 0, 0.8],
      })
    );

    // ---------- Platforms with gratings, handrails and a ladder (manoeuvring side) ----------
    const platMat = cut(MAT.paint(0x7f8891, 0.75));
    const railMat = cut(MAT.paint(0xd9b23a, 0.5));
    const platforms = new THREE.Group();
    const postGeo = new THREE.CylinderGeometry(0.012, 0.012, 0.3, 8);
    const railGeo = new THREE.CylinderGeometry(0.01, 0.01, 1.6, 8).rotateZ(Math.PI / 2);
    const slotMat = cut(MAT.paint(0x4c535b, 0.8));
    for (const [y, wallZ, z0, z1] of [[1.3, 0.7, 0.72, 1.06], [2.64, 0.55, 0.78, 1.12]]) {
      const w = z1 - z0;
      const plate = new THREE.Mesh(rbox(1.6, 0.025, w, 0.008), platMat);
      plate.position.set(0, y, (z0 + z1) / 2);
      platforms.add(plate);
      for (let i = 1; i < 8; i++) {
        const slot = new THREE.Mesh(new THREE.BoxGeometry(1.56, 0.004, 0.006), slotMat);
        slot.position.set(0, y + 0.013, z0 + (w * i) / 8);
        platforms.add(slot);
      }
      for (let i = 0; i < 5; i++) {
        const p = new THREE.Mesh(postGeo, railMat);
        p.position.set(-0.78 + i * 0.39, y + 0.15, z1 - 0.02);
        platforms.add(p);
      }
      for (const dy of [0.29, 0.15]) {
        const r = new THREE.Mesh(railGeo, railMat);
        r.position.set(0, y + dy, z1 - 0.02);
        platforms.add(r);
      }
      for (const x of [-0.72, -0.24, 0.24, 0.72]) {
        const b = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.1, z1 - wallZ), platMat);
        b.position.set(x, y - 0.06, (z1 + wallZ) / 2);
        platforms.add(b);
      }
    }
    const ladRail = new THREE.CylinderGeometry(0.012, 0.012, 2.45, 8);
    for (const x of [-0.72, -0.58]) {
      const r = new THREE.Mesh(ladRail, railMat);
      r.position.set(x, 0.38, 1.09);
      platforms.add(r);
    }
    const rungs = [];
    for (let y = -0.75; y < 1.3; y += 0.1) rungs.push([[-0.72, y, 1.09], [1, 0, 0]]);
    platforms.add(bolts(rungs, railMat, 0.008, 0.14));
    root.add(
      ctx.part(platforms, {
        id: 'platforms',
        name: 'Passerelle e scale',
        base: 'Il motore è così alto che per controllarlo i macchinisti salgono su passerelle a più piani, con ringhiere e scale, come su un ponteggio.',
        expert:
          'Piattaforme in grigliato a livello delle teste a croce, del blocco cilindri e dei coperchi: da qui si accede alle porte del basamento, alle camere di lavaggio e alle unità di comando. Le porte del basamento hanno valvole di sicurezza contro le esplosioni di nebbia d’olio.',
        explode: [0, 0, 1.0],
      })
    );

    // ---------- Control option ME: hydraulic cylinder units, no camshaft ----------
    const hcuMat = cut(MAT.paint(0x2e4a6b, 0.45));
    const hydMat = cut(MAT.chrome());
    const me = new THREE.Group();
    for (const x of CYL_X) {
      const u = new THREE.Mesh(rbox(0.2, 0.3, 0.16, 0.03), hcuMat);
      u.position.set(x, 2.82, 0.66);
      const acc = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.16, 16), hcuMat);
      acc.position.set(x - 0.06, 3.04, 0.66);
      const valveBlock = new THREE.Mesh(rbox(0.08, 0.06, 0.08, 0.01), cut(MAT.aluminum()));
      valveBlock.position.set(x + 0.05, 3.0, 0.66);
      me.add(u, acc, valveBlock);
      me.add(tube([v3(x + 0.05, 3.03, 0.64), v3(x + 0.07, 3.45, 0.42), v3(x + 0.07, 3.9, 0.18), v3(x + 0.05, 3.97, 0.07)], 0.012, hydMat, 24));
    }
    me.add(tube([v3(0.95, -0.45, 0.7), v3(0.95, 2.45, 0.7), v3(0.9, 2.62, 0.67), v3(0.6, 2.635, 0.66), v3(-0.75, 2.635, 0.66)], 0.025, hydMat, 64));
    const hps = new THREE.Mesh(rbox(0.22, 0.4, 0.3, 0.03), hcuMat);
    hps.position.set(0.95, -0.65, 0.7);
    me.add(hps);
    for (const z of [0.62, 0.78]) {
      const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.16, 20), cut(MAT.paint(0x3b6e8f, 0.5)));
      motor.position.set(0.95, -0.37, z);
      me.add(motor);
    }
    const meWrap = ctx.part(me, {
      id: 'hcu',
      name: 'Unità idrauliche di comando (HCU)',
      base: 'Al posto dell’albero a camme, una centralina e delle unità idrauliche (una per cilindro) decidono quando spruzzare il combustibile e quando aprire la valvola di scarico.',
      expert:
        'Ogni HCU riceve olio a ≈ 200–300 bar dalla stazione idraulica (in basso a poppa) e, con valvole elettroniche, comanda l’amplificatore d’iniezione e l’attuatore della valvola di scarico. Fasatura e profilo d’iniezione si adattano in tempo reale; consente anche il funzionamento a carico molto ridotto (slow steaming).',
      explode: [0, 0.5, 1.2],
    });
    root.add(meWrap);

    // ---------- Control option MC: chain-driven camshaft with fuel and valve pumps ----------
    const camMat = cut(MAT.paint(0x6a7480, 0.6));
    const mc = new THREE.Group();
    const chainCase = new THREE.Mesh(rbox(0.17, 3.4, 1.36, 0.05), cut(MAT.paint(0x6a7480, 0.6), { cap: false }));
    chainCase.position.set(0.885, 1.45, 0.32);
    const camHouse = new THREE.Mesh(rbox(1.66, 0.22, 0.22, 0.03), camMat);
    camHouse.position.set(0.03, 2.78, 0.66);
    mc.add(chainCase, camHouse);
    for (const x of CYL_X) {
      const fp = new THREE.Mesh(rbox(0.15, 0.3, 0.15, 0.025), camMat);
      fp.position.set(x - 0.06, 3.04, 0.66);
      const ap = new THREE.Mesh(rbox(0.1, 0.22, 0.12, 0.02), camMat);
      ap.position.set(x + 0.12, 3.0, 0.66);
      mc.add(fp, ap);
      for (const s of [-1, 1]) {
        mc.add(tube([v3(x - 0.06, 3.19, 0.64), v3(x + 0.1 * s, 3.5, 0.38), v3(x + 0.2 * s, 3.62, 0.08), v3(x + 0.175 * s, 3.57, -0.04)], 0.009, hydMat, 24));
      }
      mc.add(tube([v3(x + 0.12, 3.11, 0.64), v3(x + 0.1, 3.6, 0.36), v3(x + 0.07, 3.92, 0.15), v3(x + 0.05, 3.97, 0.07)], 0.012, hydMat, 24));
    }
    const camSpin = new THREE.Group();
    camSpin.add(new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 1.75, 16).rotateZ(Math.PI / 2).translate(0.075, 0, 0), cut(MAT.steel())));
    const lobeGeo = new THREE.CylinderGeometry(0.075, 0.075, 0.05, 24).rotateZ(Math.PI / 2).translate(0, 0.025, 0);
    CYL_X.forEach((x, k) => {
      for (const dx of [-0.06, 0.12]) {
        const lobe = new THREE.Mesh(lobeGeo, cut(MAT.chrome()));
        lobe.position.x = x + dx;
        lobe.rotation.x = -2 * Math.PI * OFFSET[k] + (dx > 0 ? 2.2 : 0);
        camSpin.add(lobe);
      }
    });
    const sprocketGeo = new THREE.CylinderGeometry(0.3, 0.3, 0.05, 40).rotateZ(Math.PI / 2);
    const camSprocket = new THREE.Mesh(sprocketGeo, cut(MAT.darkSteel()));
    camSprocket.position.x = 0.885;
    camSpin.add(camSprocket);
    camSpin.position.set(0, 2.78, 0.66);
    const crankSprocket = new THREE.Mesh(sprocketGeo, cut(MAT.darkSteel()));
    crankSprocket.position.set(0.885, 0, 0);
    const chainPts = [];
    for (let i = 0; i < 24; i++) {
      const a = (i / 24) * Math.PI * 2;
      chainPts.push([Math.cos(a) * 0.315, Math.sin(a) * 0.315], [2.78 + Math.cos(a) * 0.315, 0.66 + Math.sin(a) * 0.315]);
    }
    const chain = new THREE.Mesh(
      new THREE.TubeGeometry(new THREE.CatmullRomCurve3(convexHull(chainPts).map(([y, z]) => v3(0.885, y, z)), true), 160, 0.018, 6, true),
      cut(MAT.darkSteel())
    );
    mc.add(camSpin, crankSprocket, chain);
    const mcWrap = ctx.part(mc, {
      id: 'camshaft',
      name: 'Albero a camme e catena (motori MC)',
      base: 'Nei motori più vecchi una grande catena fa girare un albero a camme alla stessa velocità del motore. Le camme spingono le pompe che iniettano il combustibile e quelle che aprono le valvole di scarico.',
      expert:
        'Catena a rulli doppia in coda al motore, rapporto 1:1. Per ogni cilindro una pompa d’iniezione a stantuffo e una pompa idraulica di comando della valvola di scarico. La fasatura dipende dal profilo delle camme, con regolazione limitata dell’anticipo: meno flessibile del comando elettronico, ma semplice e collaudatissimo.',
      explode: [0.7, 0.3, 1.2],
    });
    root.add(mcWrap);

    // ---------- Gas volumes ----------
    const chargeGeo = new THREE.CylinderGeometry(0.138, 0.138, 1, 32);
    const charges = CYL_X.map((x) => {
      const m = new THREE.Mesh(chargeGeo, MAT.gas(0x5fd3ff, 0.2, false));
      m.position.x = x;
      m.renderOrder = 2;
      root.add(m);
      return m;
    });
    const scavGas = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.6, 0.48), MAT.gas(0x5fd3ff, 0.1, false));
    scavGas.position.set(0, 2.28, -0.27);
    scavGas.renderOrder = 1;
    root.add(scavGas);
    const lights = CYL_X.map((x) => {
      const l = new THREE.PointLight(0xffa040, 0, 1.6, 2);
      l.position.set(x, 3.2, 0.25);
      root.add(l);
      return l;
    });

    // ---------- Flows ----------
    const mkFlow = (pts, opts) => {
      const c = curve(pts);
      const f = flow({ path: (t, out) => c.getPointAt(t, out), ...opts });
      root.add(f.object);
      return f;
    };
    const airFlow = mkFlow(
      [v3(-2.35, 3.6), v3(-2.05, 3.6), v3(-1.82, 3.62), v3(-1.8, 3.34, -0.05), v3(-1.74, 3.05, -0.27), v3(-1.6, 2.75, -0.35), v3(-1.45, 2.42, -0.5), v3(-1.2, 2.3, -0.72), v3(0.7, 2.3, -0.72)],
      { count: 260, size: 0.07, colors: [0x5fd3ff, 0xffb37a, 0xffb37a, 0x5fd3ff, 0x5fd3ff], spread: (t) => (t < 0.2 ? 0.14 : 0.08), speed: 0.35 }
    );
    const recvFlow = mkFlow(
      [v3(0.7, 3.72, -0.62), v3(-0.95, 3.72, -0.62), v3(-1.12, 3.7, -0.45), v3(-1.25, 3.62, -0.22), v3(-1.3, 3.6, 0), v3(-1.3, 3.95), v3(-1.3, 4.5)],
      { count: 200, size: 0.08, colors: [0xb07a6a, 0x8c86a8, 0x8c86a8, 0x8c86a8], spread: (t) => (t < 0.5 ? 0.13 : 0.07), speed: 0.3 }
    );
    // Uniflow: from the receiver through the ports, swirling up to the exhaust valve.
    const scavFlows = CYL_X.map((x) => {
      const start = v3(x, 2.3, -0.62);
      const entry = v3(x, PORT_MID, -0.17);
      const f = flow({
        path: (t, out) => {
          if (t < 0.18) return out.lerpVectors(start, entry, t / 0.18);
          const u = (t - 0.18) / 0.82;
          const ang = Math.PI + u * Math.PI * 5;
          const rad = 0.125 * (1 - 0.85 * u);
          return out.set(x + Math.sin(ang) * rad, lerp(PORT_MID, VALVE_Y + 0.02, Math.pow(u, 0.9)), Math.cos(ang) * rad);
        },
        count: 140,
        size: 0.05,
        colors: [0x5fd3ff, 0x5fd3ff, 0x8fbfe0, 0x8c86a8],
        spread: 0.018,
        speed: 0.5,
      });
      root.add(f.object);
      return f;
    });
    const exhFlows = CYL_X.map((x) =>
      mkFlow([v3(x, 3.28), v3(x, 3.45), v3(x, 3.62), v3(x, 3.67, -0.22), v3(x, 3.72, -0.5), v3(x * 0.8 - 0.1, 3.72, -0.62)], {
        count: 80,
        size: 0.07,
        colors: [0xff5a1f, 0xb07a6a, 0x8c86a8],
        spread: 0.04,
        speed: 0.7,
      })
    );

    // ---------- Animation ----------
    const C = {
      air: new THREE.Color(0x5fd3ff),
      hot: new THREE.Color(0xffd7a8),
      flash: new THREE.Color(0xfff2c4),
      fire: new THREE.Color(0xff5a1f),
      exh: new THREE.Color(0x8c86a8),
    };
    const cA = new THREE.Color();
    const rpmOf = (th) => 30 + th * 50;
    const animRate = (th) => 0.1 + th * 0.5;

    return {
      root,
      cycleRate: animRate,
      setOption(id, value) {
        if (id !== 'control') return;
        meWrap.visible = value === 'me';
        mcWrap.visible = value === 'mc';
      },
      update(state) {
        const th1 = angleOf(state.cycle) * DEG;
        crank.rotation.x = th1;
        spin.rotation.x = th1;
        camSpin.rotation.x = th1; // two-stroke: camshaft at crank speed
        crankSprocket.rotation.x = th1;
        rotor.rotation.x += state.dt * (6 + 22 * state.throttle);
        const assembled = (state.explode || 0) < 0.05;
        const rate = animRate(state.throttle);

        CYL_X.forEach((x, k) => {
          const a = angleOf(state.cycle - OFFSET[k]);
          const t = a * DEG;
          const yc = R * Math.cos(t);
          const zc = R * Math.sin(t);
          const yh = yc + Math.sqrt(L * L - zc * zc);
          crossheads[k].position.y = yh;
          prods[k].position.y = yh;
          pistons[k].position.y = yh + ROD;
          rods[k].position.set(x, yc, zc);
          rods[k].rotation.x = Math.atan2(-zc, yh - yc);
          const lift = valveLift(a);
          valves[k].position.y = VALVE_Y - lift * 0.06;
          const sp = portOpen(a);

          // Charge between crown and cover.
          const bottom = yh + ROD + CROWN - 0.02;
          const top = COVER_Y + 0.03;
          const ch = charges[k];
          ch.position.y = (top + bottom) / 2;
          ch.scale.y = Math.max(0.001, top - bottom);
          const ai = (a - INJ + 360) % 360; // degrees since injection start
          let op;
          if (ai < 18) {
            cA.copy(C.flash).lerp(C.fire, ai / 18);
            op = 0.95;
          } else if (a < EVO && ai < 200) {
            const f = (a - 15) / (EVO - 15);
            cA.copy(C.fire).lerp(C.exh, clamp(f, 0, 1));
            op = lerp(0.9, 0.35, clamp(f, 0, 1));
          } else if (a < SPO) {
            cA.copy(C.exh);
            op = lerp(0.35, 0.2, (a - EVO) / (SPO - EVO));
          } else if (a < 360 - SPO) {
            cA.copy(C.exh).lerp(C.air, (a - SPO) / (360 - 2 * SPO));
            op = 0.2 - 0.05 * ((a - SPO) / (360 - 2 * SPO));
          } else if (a < A0) {
            cA.copy(C.air);
            op = 0.15;
          } else {
            const f = (a - A0) / (INJ - A0);
            cA.copy(C.air).lerp(C.hot, f);
            op = 0.15 + 0.25 * f;
          }
          ch.material.color.copy(cA);
          ch.material.opacity = op;
          ch.visible = assembled;

          // Fuel sprays and combustion glow.
          const inj = ai < 23;
          for (const s of sprays[k]) {
            s.visible = inj && assembled;
            s.scale.set(1, inj ? 0.7 + 0.5 * Math.sin((Math.PI * ai) / 23) : 1, 1);
          }
          const flash = ai > 2 && ai < 50 ? 1 - (ai - 2) / 48 : 0;
          lights[k].intensity = assembled ? flash * (1.5 + state.throttle * 3) : 0;

          const sf = scavFlows[k];
          sf.object.visible = assembled;
          sf.material.opacity = clamp(sp * 1.6, 0, 0.9);
          sf.update(state.dt, sp * rate * 1.6);
          const blow = a > EVO && a < SPO + 10 ? 2 : 1;
          const ef = exhFlows[k];
          ef.object.visible = assembled;
          ef.material.opacity = clamp(lift * 1.4, 0, 0.9);
          ef.update(state.dt, lift * rate * 1.4 * blow);
        });

        // Turbocharger circuits run steadily (constant-pressure receivers).
        const steady = 0.35 + 0.65 * state.throttle;
        for (const f of [airFlow, recvFlow]) {
          f.object.visible = assembled;
          f.material.opacity = 0.5 + 0.35 * state.throttle;
          f.update(state.dt, steady * (0.6 + rate));
        }
        scavGas.visible = assembled;
        scavGas.material.opacity = 0.06 + 0.1 * state.throttle;
      },
      readout(state) {
        const rpm = rpmOf(state.throttle);
        const load = Math.pow(rpm / 80, 3); // propeller law
        return [
          { label: 'Giri reali', value: Math.round(rpm), unit: 'giri/min' },
          { label: 'Potenza (motore da 12 cilindri)', value: (77 * load).toFixed(1).replace('.', ','), unit: 'MW' },
          { label: 'Pressione aria di lavaggio', value: (1 + 3.3 * Math.pow(rpm / 80, 2)).toFixed(1).replace('.', ','), unit: 'bar' },
          { label: 'Velocità della nave', value: Math.round((23 * rpm) / 80), unit: 'nodi' },
        ];
      },
    };
  },
};
