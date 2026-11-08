// Motorcycle inline-four 600 cc supersport engine with the 6-speed cassette gearbox and wet clutch
// in the same crankcase. Seen from the right side: crank along Z, bike front = +X, cylinders tilted
// 30° forward. Cycle 0..1 = 720° of crank for cylinder 1, flat-plane crank, firing order 1-2-4-3.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, pipe, flow, clamp, lerp, smooth } from '../core/helpers.js';

const TAU = Math.PI * 2;
const TILT = Math.PI / 6;              // cylinder bank tilted forward
const CS = Math.cos(TILT), SN = Math.sin(TILT);
const CYL_Z = [-1.2, -0.4, 0.4, 1.2];  // cylinder centres (local z); root shifted so cyl 3 sits on the cut plane
const ROOT_Z = -0.4;
const OFFSET = [0, 0.25, 0.75, 0.5];   // firing 1-2-4-3: cycle delay of each cylinder
const R = 0.2125;                      // crank radius (stroke 42.5 mm, 1 unit = 100 mm)
const L = 0.95;                        // conrod length
const BORE_R = 0.335;                  // bore 67 mm
const DECK = 1.35;                     // head face (cylinder frame y')
const CAM_Y = 2.08;
const CAM_X = [-0.26, 0.25];           // intake, exhaust camshaft (cylinder frame x')
const VTILT = 0.19;                    // valve tilt (≈11°)
const VALVE_DZ = 0.14;
const CHAIN_Z = 1.7;                   // cam chain plane
// Gearbox (root frame, side view)
const MAIN = new THREE.Vector2(-0.98, 0.3);
const CNTR = new THREE.Vector2(-1.6, -0.12);
const DRUM = new THREE.Vector2(-0.9, -0.48);
const GEAR_D = MAIN.distanceTo(CNTR);
const PRI_D = MAIN.length();
const NP = 25, NB = 50;                // primary drive / clutch basket teeth (2:1)
// teeth [main, counter] for gears 1..6
const GEARS = [null, [12, 31], [16, 32], [18, 30], [18, 26], [21, 27], [23, 27]];
const PAIR_Z = { 1: -0.4, 5: -0.25, 3: -0.1, 4: 0.05, 6: 0.2, 2: 0.35 };
const GW = 0.085;                      // gear width
const SHIFT = 0.035;                   // slider travel
const FINAL = 43 / 16;                 // final drive (chain)
const WHEEL_C = 1.98;                  // rear tyre circumference, m
// Shift drum: detent order 1-N-2-3-4-5-6, slider states [C5, C6, M3/4]
const DRUM_ORDER = [1, 0, 2, 3, 4, 5, 6];
const TRACK = [[-1, 0, 0], [0, 0, 0], [0, 1, 0], [1, 0, 0], [0, -1, 0], [0, 0, -1], [0, 0, 1]];
const STEP = (50 * Math.PI) / 180;
const N_IDX = 360 / 50;

const wrap = (c) => ((c % 1) + 1) % 1;
function lift(c, a, b) {
  const x = wrap(c - a);
  const w = b - a;
  return x < w ? Math.sin((Math.PI * x) / w) : 0;
}
// cylinder frame (x', y') -> root (x, y)
const fr = (x, y) => [CS * x + SN * y, -SN * x + CS * y];
const frV = (x, y, z) => new THREE.Vector3(...fr(x, y), z);

function convexHull(pts) {
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [];
  for (const q of p) {
    while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop();
    lo.push(q);
  }
  const up = [];
  for (const q of p.reverse()) {
    while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop();
    up.push(q);
  }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
const circlePts = (cx, cy, r, n = 28) =>
  Array.from({ length: n }, (_, i) => [cx + r * Math.cos((i / n) * TAU), cy + r * Math.sin((i / n) * TAU)]);

function poly(pts, Cls = THREE.Shape) {
  const s = new Cls();
  pts.forEach(([u, v], i) => (i ? s.lineTo(u, v) : s.moveTo(u, v)));
  s.closePath();
  return s;
}
function circ(u, v, r) {
  const p = new THREE.Path();
  p.absarc(u, v, r, 0, TAU, true);
  return p;
}
// Shape(s) in the XY plane extruded from z0 to z1.
function extZ(shapes, z0, z1, curveSegments = 18, bevel = 0) {
  const g = new THREE.ExtrudeGeometry(shapes, {
    depth: z1 - z0 - 2 * bevel, curveSegments,
    bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2,
  });
  g.translate(0, 0, z0 + bevel);
  return g;
}
// Shape in (x', -z) extruded along y' from y0 to y1 (cylinder frame).
function extY(shapes, y0, y1, curveSegments = 32) {
  const g = new THREE.ExtrudeGeometry(shapes, { depth: y1 - y0, bevelEnabled: false, curveSegments });
  g.rotateX(-Math.PI / 2);
  g.translate(0, y0, 0);
  return g;
}
// Spur gear, tooth 0 centred on +X, axis Z, centred on z = 0.
function gearGeo(N, r, w, holeR = 0.06, holes = 0) {
  const m = (2 * r) / N, ra = r + m, rf = r - 1.25 * m, p = TAU / N;
  const pts = [];
  for (let i = 0; i < N; i++) {
    const a = i * p;
    for (const [rr, k] of [[rf, -0.5], [rf, -0.27], [ra, -0.12], [ra, 0.12], [rf, 0.27]])
      pts.push([rr * Math.cos(a + k * p), rr * Math.sin(a + k * p)]);
  }
  const s = poly(pts);
  s.holes.push(circ(0, 0, holeR));
  for (let k = 0; k < holes; k++) {
    const a = (k / holes) * TAU + 0.3;
    const rr = (rf + holeR) / 2;
    s.holes.push(circ(rr * Math.cos(a), rr * Math.sin(a), (rf - holeR) * 0.28));
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: w, bevelEnabled: false, curveSegments: 10 });
  g.translate(0, 0, -w / 2);
  g.userData.rf = rf;
  return g;
}
// Angle of gear 2 meshing with gear 1 (angle a1), centres c1 -> c2.
function meshAngle(a1, n1, n2, c1, c2) {
  const b = Math.atan2(c2.y - c1.y, c2.x - c1.x);
  return -(n1 / n2) * (a1 - b) + b + Math.PI + Math.PI / n2;
}
const zAxis = (g) => g.rotateX(Math.PI / 2); // cylinder along Y -> along Z
const cylZ = (r, h, seg = 24) => zAxis(new THREE.CylinderGeometry(r, r, h, seg));

// Instanced hex bolt heads. axis: 'x' | 'y' | 'z'.
function bolts(list, axis, mat, r = 0.028, h = 0.028) {
  const g = new THREE.CylinderGeometry(r, r, h, 6);
  if (axis === 'z') zAxis(g);
  if (axis === 'x') g.rotateZ(Math.PI / 2);
  const m = new THREE.InstancedMesh(g, mat, list.length);
  const d = new THREE.Object3D();
  list.forEach((p, i) => {
    d.position.set(p[0], p[1], p[2]);
    d.updateMatrix();
    m.setMatrixAt(i, d.matrix);
  });
  return m;
}
function tube(points, r, mat, seg = 40, radial = 10, closed = false) {
  const c = new THREE.CatmullRomCurve3(points.map((p) => (p.isVector3 ? p : new THREE.Vector3(...p))), closed);
  const m = new THREE.Mesh(new THREE.TubeGeometry(c, seg, r, radial, closed), mat);
  m.userData.curve = c;
  return m;
}
function mesh(geo, mat, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  return m;
}
function rbox(w, h, d, r, mat, x = 0, y = 0, z = 0) {
  return mesh(new RoundedBoxGeometry(w, h, d, 2, r), mat, x, y, z);
}

// Gear selection model: throttle 0..1 sweeps an acceleration run 1st -> 6th.
const ratioOf = (g) => GEARS[g][1] / GEARS[g][0];
const SHIFT_RPM = 14500;
function driveState(th) {
  if (th < 0.04) return { gear: 0, rpm: 1300 + th * 6000, kmh: 0 };
  const band = (th - 0.04) / 0.96 * 6;
  const gear = Math.min(6, Math.floor(band) + 1);
  const f = clamp(band - (gear - 1), 0, 1);
  const r0 = gear === 1 ? 4000 : (SHIFT_RPM * ratioOf(gear)) / ratioOf(gear - 1);
  const r1 = gear === 6 ? 14000 : SHIFT_RPM;
  const rpm = lerp(r0, r1, f);
  const kmh = (rpm / (2 * ratioOf(gear) * FINAL)) * WHEEL_C * 0.06;
  return { gear, rpm, kmh };
}
function torqueAt(rpm) {
  if (rpm < 11500) return 66 * (0.55 + 0.45 * Math.sin((Math.PI / 2) * clamp((rpm - 1500) / 10000, 0, 1)));
  return 66 - 0.95 * ((rpm - 11500) / 1000) ** 2;
}

export default {
  id: 'moto4t',
  name: 'Motore moto 4 tempi (4 cilindri in linea)',
  short: 'Moto 4 tempi',
  family: 'terra',
  vehicle: 'Moto sportiva e stradale',
  examples: 'Supersportive 600 a quattro cilindri, naked e stradali sportive giapponesi ed europee',
  tagline: 'Quindicimila giri al minuto e il cambio chiuso nello stesso carter: un motore che è anche una trasmissione.',
  intro: {
    base:
      'Funziona come il motore di un’auto: aspirazione, compressione, scoppio e scarico. Ma ha pistoni piccoli e leggeri che fanno una <b>corsa cortissima</b>, così può girare fino a 15 000 giri al minuto. La grande differenza è dietro l’albero motore: <b>il cambio sta dentro lo stesso carter</b> e usa lo stesso olio. La frizione a dischi, a destra, collega il motore al cambio a 6 marce; il pedale fa ruotare un <b>tamburo</b> che sposta le forcelle e innesta una marcia alla volta, in sequenza. Alla fine un pignone e una catena portano la spinta alla ruota posteriore.',
    expert:
      'Quattro cilindri in linea da ≈ 600 cm³ (alesaggio 67 × corsa 42,5 mm, rapporto alesaggio/corsa ≈ 1,6), bialbero in testa a 16 valvole comandato da una <b>catena laterale</b> con tenditore idraulico. Albero a gomiti piatto (manovelle a 180°), ordine di accensione 1‑2‑4‑3, un’accensione ogni 180°. Il gruppo è “in blocco”: albero motore, primaria a ingranaggi (≈ 2:1), frizione multidisco a bagno d’olio, cambio a cassetta a 6 marce sempre in presa con innesti frontali (dog) e tamburo selettore, tutti lubrificati dallo stesso olio della coppa. Potenza ≈ 85–90 kW a 13 000–14 000 giri, velocità media del pistone ≈ 21 m/s a 15 000 giri.',
  },
  specs: [
    { label: 'Architettura', value: '4 cilindri in linea, inclinati in avanti' },
    { label: 'Cilindrata', value: '≈ 600 cm³ (67 × 42,5 mm)' },
    { label: 'Distribuzione', value: 'Bialbero in testa (DOHC), 16 valvole, catena laterale' },
    { label: 'Regime massimo', value: '≈ 15 000 giri/min' },
    { label: 'Potenza tipica', value: '85 – 90 kW (≈ 120 CV)' },
    { label: 'Cambio', value: '6 marce sequenziale, in blocco col motore' },
    { label: 'Frizione e raffreddamento', value: 'Multidisco a bagno d’olio · a liquido' },
  ],
  metrics: { efficiency: 30, power: 88, powerDensity: 1.5 },
  variants: {
    columns: ['Moto', 'Architettura', 'Cilindrata', 'Distribuzione'],
    rows: [
      ['Honda CBR600RR', '4 cilindri in linea', '599 cm³', 'Bialbero, catena'],
      ['Yamaha YZF‑R1', '4 in linea, albero “crossplane”', '998 cm³', 'Bialbero, catena'],
      ['Ducati Monster (937)', 'Bicilindrico a L, Testastretta', '937 cm³', 'Desmodromica, cinghie dentate'],
      ['BMW R 1300 GS', 'Bicilindrico boxer', '1 300 cm³', 'Bialbero, catena'],
      ['KTM 390 Duke', 'Monocilindrico', '373 – 399 cm³', 'Bialbero, catena'],
      ['Moto Guzzi V7', 'Bicilindrico a V trasversale', '853 cm³', 'Aste e bilancieri'],
    ],
    note: 'Cilindrate e potenze cambiano tra una generazione e l’altra. In quasi tutte le moto il cambio è in blocco con il motore.',
  },
  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      range: [0, 0.25],
      parts: ['intakeValves', 'throttleBodies', 'injectors', 'airbox', 'pistons'],
      base: 'Il pistone scende e risucchia aria dalla scatola filtro attraverso i corpi farfallati. L’iniettore spruzza la benzina: ogni cilindro ha il suo.',
      expert:
        'Valvole di aspirazione aperte (2 per cilindro), pistone dal PMS al PMI. Un corpo farfallato per cilindro con trombetta corta dentro l’airbox: condotti brevi e diritti favoriscono il riempimento agli alti regimi, aiutati dalla presa d’aria dinamica (ram‑air) che a 250 km/h aggiunge qualche decina di millibar. Iniezione indiretta sequenziale, farfalle spesso comandate elettronicamente (ride‑by‑wire).',
    },
    {
      id: 'compression',
      name: 'Compressione',
      range: [0.25, 0.5],
      parts: ['pistons', 'cylinderHead'],
      base: 'Le valvole si chiudono e il pistone risale schiacciando la miscela. La camera di scoppio è piccola e compatta: la miscela viene compressa più di 12 volte.',
      expert:
        'Valvole chiuse, pistone dal PMI al PMS. Rapporto di compressione ≈ 12–13:1 grazie alla camera a tetto (pent‑roof) compatta con candela centrale e alla benzina a 95–98 ottani. Con alesaggio grande e corsa corta le valvole possono essere grandi rispetto al cilindro: è il segreto del riempimento ad alto regime.',
    },
    {
      id: 'power',
      name: 'Scoppio ed espansione',
      range: [0.5, 0.75],
      parts: ['sparkPlugs', 'pistons', 'conrods', 'crankshaft'],
      base: 'La candela accende la miscela e il gas caldissimo spinge giù il pistone. Con quattro cilindri c’è uno scoppio ogni mezzo giro: a 15 000 giri sono 500 scoppi al secondo!',
      expert:
        'Accensione con bobina sulla candela (stick‑coil) e anticipo mappato dalla centralina. Albero piatto: accensioni regolari ogni 180° (ordine 1‑2‑4‑3). La Yamaha R1 usa invece un albero “crossplane” con manovelle a 90° e intervalli irregolari 270‑180‑90‑180°: la coppia arriva alla ruota più “leggibile”, simile a un V4. Corsa di 42,5 mm: a 15 000 giri la velocità media del pistone è ≈ 21 m/s.',
    },
    {
      id: 'exhaust',
      name: 'Scarico',
      range: [0.75, 1],
      parts: ['exhaustValves', 'exhaustHeaders', 'pistons'],
      base: 'Si aprono le valvole di scarico e il pistone spinge fuori i gas bruciati nei collettori, che si uniscono a coppie e poi in un solo tubo.',
      expert:
        'Collettori 4‑2‑1 in acciaio inox o titanio: si uniscono prima 1‑4 e 2‑3 (cilindri che scoppiano a 360° di distanza) così l’onda di depressione di un cilindro aiuta a svuotare l’altro durante l’incrocio valvole. Lunghezze e diametri sono accordati per un regime preciso; molte moto hanno anche una valvola allo scarico comandata dalla centralina.',
    },
    {
      id: 'transmission',
      name: 'Frizione e cambio',
      parts: ['primaryGear', 'clutchBasket', 'clutchPlates', 'pressurePlate', 'mainShaft', 'countershaft', 'shiftDrum', 'sprocket'],
      base: 'L’albero motore fa girare il cestello della frizione con un ingranaggio. Quando la frizione è chiusa, i dischi stretti tra loro portano la rotazione all’albero primario del cambio. Gli ingranaggi delle 6 marce sono sempre accoppiati; quando il pedale ruota il tamburo, una forcella sposta un ingranaggio scorrevole che “aggancia” con i suoi denti frontali la marcia scelta. L’albero secondario fa girare il pignone e la catena. Alza l’acceleratore e guarda le cambiate da 1ª a 6ª.',
      expert:
        'Primaria ≈ 2:1 (25/50 denti) sul cestello; frizione multidisco a bagno d’olio con molle elicoidali e spesso antisaltellamento (slipper). Cambio a cassetta sempre in presa: gli ingranaggi folli girano liberi sul loro albero finché un ingranaggio scorrevole scanalato non li vincola con gli innesti frontali. Tre forcelle guidate dalle piste del tamburo selettore, che ruota di uno scatto per ogni azionamento del pedale: sequenza 1‑N‑2‑3‑4‑5‑6. Molte sportive hanno il cambio elettronico (quickshifter) che taglia l’accensione per pochi millisecondi e permette di salire di marcia senza frizione.',
    },
  ],
  quiz: [
    {
      q: 'Dove si trova il cambio in una moto come questa?',
      options: [
        'In una scatola separata dietro al motore, con un suo olio',
        'Dentro lo stesso carter del motore, e usa lo stesso olio',
        'Nel mozzo della ruota posteriore',
        'Non c’è: la moto ha una sola marcia',
      ],
      correct: 1,
      explain: 'Il cambio “in blocco” sta dietro l’albero motore nello stesso carter: il motore è più corto e compatto, e un unico olio lubrifica motore, frizione e cambio.',
    },
    {
      q: 'Perché il cambio della moto si chiama “sequenziale”?',
      options: [
        'Perché le marce si scelgono con una leva a H',
        'Perché il tamburo ruota di uno scatto alla volta: si passa solo alla marcia vicina',
        'Perché cambia da solo come un automatico',
        'Perché ha una frizione per ogni marcia',
      ],
      correct: 1,
      explain: 'Ogni colpo di pedale fa ruotare il tamburo selettore di un passo: le piste spostano le forcelle e si sale o si scende di una marcia per volta (1‑N‑2‑3‑4‑5‑6).',
    },
    {
      q: 'Che cosa permette a un 600 sportivo di girare a 15 000 giri al minuto?',
      options: [
        'Pistoni grandi e una corsa molto lunga',
        'Un turbocompressore',
        'Corsa corta, pistoni leggeri e valvole grandi',
        'Il raffreddamento ad aria',
      ],
      correct: 2,
      explain: 'Con una corsa di soli 42,5 mm il pistone percorre poca strada a ogni giro: a 15 000 giri la sua velocità media resta ≈ 21 m/s, sopportabile. L’alesaggio grande lascia spazio a valvole grandi per respirare.',
    },
    {
      q: 'A cosa serve la frizione multidisco “a bagno d’olio”?',
      options: [
        'A raffreddare l’olio del cambio',
        'A staccare il motore dal cambio per partire e cambiare marcia, con dischi lubrificati e raffreddati dall’olio',
        'A cambiare il rapporto di compressione',
        'A tendere la catena della distribuzione',
      ],
      correct: 1,
      explain: 'Tirando la leva si separano i dischi d’attrito da quelli d’acciaio e il motore gira libero. Molti dischi piccoli trasmettono tanta coppia in poco spazio; l’olio li raffredda e rende l’innesto dolce.',
    },
  ],
  camera: { position: [-4.83, 3.35, 8.04], target: [-0.35, 0.65, -0.35] },
  thumb: { orbit: [-28, 16], zoom: 1.2, view: 'section' },

  build(ctx) {
    const root = new THREE.Group();
    root.position.set(0.3, 0.1, ROOT_Z);
    const F = new THREE.Group(); // cylinder frame: y' along the bore axis
    F.rotation.z = -TILT;
    root.add(F);
    const P = (parent, obj, id, name, base, expert, explode = [0, 0, 0]) => {
      const w = ctx.part(obj, { id, name, base, expert, explode });
      parent.add(w);
      return w;
    };

    // ---------- Materials ----------
    const caseMat = cut(MAT.aluminum());
    caseMat.color.set(0x8f969e);
    caseMat.roughness = 0.62;
    const caseLowMat = cut(MAT.aluminum());
    caseLowMat.color.set(0x80878f);
    caseLowMat.roughness = 0.66;
    const headMat = cut(MAT.aluminum());
    headMat.color.set(0xaab1b9);
    headMat.roughness = 0.42;
    const coverMat = cut(MAT.paint(0x2f3339, 0.36));
    coverMat.metalness = 0.6;
    const camCoverMat = cut(MAT.paint(0x8c1d1d, 0.3));
    camCoverMat.metalness = 0.45;
    const gasketMat = cut(MAT.paint(0x101215, 0.85));
    const boltMat = cut(MAT.chrome());
    boltMat.roughness = 0.28;
    const linerMat = cut(MAT.steel());
    const rubberCut = cut(MAT.rubber());
    const blackCut = cut(MAT.paint(0x1a1c20, 0.6));
    const steel = MAT.steel();
    const darkSteel = MAT.darkSteel();
    const bronze = MAT.brass();
    bronze.color.set(0xb07a3a);

    // ---------- Crankcase (split horizontally at the crank axis) ----------
    const notch = (r, top) => {
      const pts = [];
      for (let i = 0; i <= 10; i++) {
        const a = Math.PI - (i / 10) * Math.PI;
        pts.push([r * Math.cos(a), (top ? 1 : -1) * r * Math.sin(a)]);
      }
      return pts;
    };
    const UP_OUT = [[0.72, 0], [0.72, 0.1], [0.73, 0.225], fr(0.52, 0.62), [-0.55, 0.86], [-1.3, 0.86], [-1.75, 0.78], [-2.1, 0.58], [-2.3, 0.3], [-2.32, 0]];
    const LOW_OUT = [[-2.32, 0], [-2.31, -0.45], [-2.12, -0.84], [-1.9, -0.98], [0.2, -0.98], [0.5, -0.8], [0.72, -0.45], [0.72, 0]];
    const upperWall = () => {
      const s = poly([[-2.32, 0], ...notch(0.19, true).map(([x, y]) => [x, y]), ...UP_OUT.slice(0, -1)]);
      s.holes.push(circ(MAIN.x, MAIN.y, 0.09));
      return s;
    };
    const lowerWall = () => {
      const s = poly([...LOW_OUT, ...notch(0.19, false).reverse()]);
      for (const [x, y, r] of [[CNTR.x, CNTR.y, 0.075], [DRUM.x, DRUM.y, 0.05], [-0.45, -0.75, 0.04], [0.15, -0.8, 0.05]]) s.holes.push(circ(x, y, r));
      return s;
    };
    const upperCase = new THREE.Group();
    const frontPiece = poly([[0.65, 0], [0.72, 0], [0.72, 0.1], [0.73, 0.225], fr(0.52, 0.62), fr(0.37, 0.6), [0.65, 0.12]]);
    const rearPiece = poly([[-2.25, 0], [-2.32, 0], [-2.3, 0.3], [-2.1, 0.58], [-1.75, 0.78], [-1.3, 0.86], [-0.55, 0.86], fr(-0.52, 0.62), fr(-0.37, 0.6), [-0.4, 0.74], [-0.6, 0.79], [-1.3, 0.79], [-1.73, 0.71], [-2.06, 0.51], [-2.24, 0.27]]);
    upperCase.add(mesh(extZ([frontPiece, rearPiece], -1.66, 1.76, 12, 0.03), caseMat));
    upperCase.add(mesh(extZ(upperWall(), -1.74, -1.66, 12, 0.02), caseMat), mesh(extZ(upperWall(), 1.76, 1.84, 12, 0.02), caseMat));
    const bulkUp = poly([[-0.5, 0], [-0.19, 0], ...notch(0.19, true).slice(1, -1), [0.19, 0], [0.66, 0], [0.66, 0.12], fr(0.4, 0.5), fr(-0.4, 0.5), [-0.4, 0.7], [-0.5, 0.55]]);
    const bulkLow = poly([[-0.5, 0], [-0.5, -0.65], [0.4, -0.65], [0.66, -0.43], [0.66, 0], [0.19, 0], ...notch(0.19, false).reverse().slice(1, -1), [-0.19, 0]]);
    const JOURNALS = [-1.6, -0.8, 0, 0.8, 1.6];
    for (const z of JOURNALS) upperCase.add(mesh(extZ(bulkUp, z - 0.05, z + 0.05), caseMat));
    // engine mounts, lifting boss, gasket line, bolts
    for (const [x, y] of [[-2.08, 0.6]]) for (const z of [-1.86, 1.96]) {
      const m = mesh(zAxis(pipe(0.1, 0.045, 0.16, 20)), caseMat, x, y, z - Math.sign(z) * 0.05);
      upperCase.add(m);
    }
    upperCase.add(mesh(new THREE.BoxGeometry(0.014, 0.02, 3.56), gasketMat, 0.726, 0.0, 0.05));
    upperCase.add(mesh(new THREE.BoxGeometry(0.014, 0.02, 3.56), gasketMat, -2.326, 0.0, 0.05));
    upperCase.add(mesh(new THREE.BoxGeometry(3.0, 0.02, 0.014), gasketMat, -0.8, 0.0, -1.726));
    const caseBolts = [];
    for (const x of [-0.75, -1.15, -1.55, -1.95]) for (const z of [-1.45, -0.6, 0.3, 1.2]) caseBolts.push([x, x < -1.7 ? 0.76 : 0.875, z]);
    for (const x of [-2.1, -1.5, -0.9, 0.45]) caseBolts.push([x, -0.08, -1.735]);
    upperCase.add(bolts(caseBolts.slice(0, 16), 'y', boltMat), bolts(caseBolts.slice(16), 'z', boltMat));
    P(root, upperCase, 'crankcaseUpper', 'Semicarter superiore',
      'La metà alta della “scatola” del motore. Sostiene l’albero motore e, dietro, gli alberi del cambio. Nelle moto il carter contiene motore e cambio insieme.',
      'Fusione in lega di alluminio con i cilindri integrati (monoblocco). Il carter è diviso orizzontalmente sull’asse dell’albero motore e su quello del primario: i supporti di banco e i cuscinetti del cambio sono serrati tra i due semicarter. Paratie interne con i semicuscinetti di banco, attacchi al telaio (il motore è spesso parte portante del telaio).',
      [0, 0.15, -1.1]);

    const lowerCase = new THREE.Group();
    const lowU = poly([...LOW_OUT, [0.65, 0], [0.65, -0.43], [0.45, -0.73], [0.18, -0.92], [-1.86, -0.92], [-2.06, -0.79], [-2.24, -0.43], [-2.25, 0]]);
    lowerCase.add(mesh(extZ(lowU, -1.66, 1.76, 12, 0.03), caseLowMat));
    lowerCase.add(mesh(extZ(lowerWall(), -1.74, -1.66, 12, 0.02), caseLowMat), mesh(extZ(lowerWall(), 1.76, 1.84, 12, 0.02), caseLowMat));
    for (const z of JOURNALS) lowerCase.add(mesh(extZ(bulkLow, z - 0.05, z + 0.05), caseLowMat));
    for (const z of [-1.86, 1.96]) lowerCase.add(mesh(zAxis(pipe(0.1, 0.045, 0.16, 20)), caseLowMat, -2.2, -0.6, z - Math.sign(z) * 0.05));
    // sump ribs and drain plug
    for (const x of [-1.5, -1.0, -0.5]) lowerCase.add(mesh(new THREE.BoxGeometry(0.05, 0.05, 3.4), caseLowMat, x, -1.0, 0.05));
    lowerCase.add(mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.05, 6), boltMat, -0.8, -1.005, 0.6));
    // bearing retainer and ribs on the right wall, behind the clutch cover
    lowerCase.add(mesh(cylZ(0.2, 0.04, 32), caseLowMat, CNTR.x, CNTR.y, 1.86));
    lowerCase.add(bolts([0, 1, 2, 3].map((i) => [CNTR.x + 0.15 * Math.cos(i * 1.57 + 0.6), CNTR.y + 0.15 * Math.sin(i * 1.57 + 0.6), 1.89]), 'z', boltMat, 0.02, 0.02));
    for (const [x0, y0, x1, y1] of [[-2.25, -0.3, -1.85, -0.3], [-2.0, -0.85, -1.75, -0.5], [-0.3, -0.92, 0.15, -0.6]]) {
      const rib = mesh(new THREE.BoxGeometry(Math.hypot(x1 - x0, y1 - y0), 0.04, 0.04), caseLowMat, (x0 + x1) / 2, (y0 + y1) / 2, 1.86);
      rib.rotation.z = Math.atan2(y1 - y0, x1 - x0);
      lowerCase.add(rib);
    }
    for (const [x0, y0, x1, y1] of [[-2.25, -0.35, -1.95, -0.35], [-0.5, -0.92, -0.1, -0.55], [-1.2, -0.92, -0.9, -0.62]]) {
      const rib = mesh(new THREE.BoxGeometry(Math.hypot(x1 - x0, y1 - y0), 0.04, 0.04), caseLowMat, (x0 + x1) / 2, (y0 + y1) / 2, -1.76);
      rib.rotation.z = Math.atan2(y1 - y0, x1 - x0);
      lowerCase.add(rib);
    }
    const lowBolts = [];
    for (const x of [-2.1, -1.5, -0.9, -0.3, 0.45]) lowBolts.push([x, -0.25, -1.735], [x, -0.25, 1.835]);
    lowerCase.add(bolts(lowBolts, 'z', boltMat));
    P(root, lowerCase, 'crankcaseLower', 'Semicarter inferiore e coppa',
      'La metà bassa del carter. In fondo raccoglie l’olio, che qui serve sia al motore sia al cambio.',
      'Semicarter inferiore con i cappelli di banco integrati e la coppa dell’olio (carter umido, ≈ 3,5 litri). Le nervature irrigidiscono la fusione e aiutano a smaltire calore; il tappo di scarico è magnetico per trattenere le particelle metalliche del cambio.',
      [0, -1.1, 0]);

    // Main bearing shells
    const shells = new THREE.Group();
    for (const z of JOURNALS) shells.add(mesh(zAxis(pipe(0.19, 0.16, 0.1, 32)), bronze, 0, 0, z));
    root.add(shells);

    // ---------- Cylinder block (cylinder frame) ----------
    const roundRect = (x0, y0, x1, y1, r) => {
      const pts = [];
      for (const [cx, cy, a0] of [[x1 - r, y0 + r, -Math.PI / 2], [x1 - r, y1 - r, 0], [x0 + r, y1 - r, Math.PI / 2], [x0 + r, y0 + r, Math.PI]])
        for (let i = 0; i <= 4; i++) pts.push([cx + r * Math.cos(a0 + (i / 4) * (Math.PI / 2)), cy + r * Math.sin(a0 + (i / 4) * (Math.PI / 2))]);
      return pts;
    };
    const blockShape = () => {
      const s = poly(roundRect(-0.52, -1.86, 0.52, 1.64, 0.07));
      for (const c of CYL_Z) s.holes.push(circ(0, -c, 0.345));
      s.holes.push(poly([[-0.45, -1.8], [0.45, -1.8], [0.45, -1.62], [-0.45, -1.62]], THREE.Path));
      return s;
    };
    // open-deck block: outer shell + siamesed liner wall, coolant jacket in between
    const shellShape = () => {
      const s = poly(roundRect(-0.52, -1.86, 0.52, 1.64, 0.07));
      s.holes.push(poly(roundRect(-0.47, -1.585, 0.47, 1.585, 0.12), THREE.Path));
      s.holes.push(poly([[-0.45, -1.8], [0.45, -1.8], [0.45, -1.62], [-0.45, -1.62]], THREE.Path));
      return s;
    };
    const wallShape = () => {
      const s = poly(roundRect(-0.405, -1.55, 0.405, 1.55, 0.3));
      for (const c of CYL_Z) s.holes.push(circ(0, -c, 0.345));
      return s;
    };
    const block = new THREE.Group();
    block.add(mesh(extY(shellShape(), 0.48, DECK), caseMat), mesh(extY(wallShape(), 0.48, DECK - 0.03), caseMat));
    const floor = poly(roundRect(-0.47, -1.585, 0.47, 1.585, 0.12));
    floor.holes.push(poly(roundRect(-0.405, -1.55, 0.405, 1.55, 0.3), THREE.Path));
    block.add(mesh(extY(floor, 0.48, 0.56), caseMat));
    // stiffening ribs on front and back between the bores
    for (const z of [-1.62, -0.8, 0, 0.8]) for (const x of [-0.54, 0.54]) block.add(rbox(0.05, 0.8, 0.06, 0.02, caseMat, x, 0.9, z));
    for (const c of CYL_Z) block.add(mesh(pipe(0.345, 0.335, 0.87, 48), linerMat, 0, 0.915, c));
    // coolant inlet boss on the back of the block
    block.add(mesh(new THREE.CylinderGeometry(0.06, 0.07, 0.12, 16).rotateZ(Math.PI / 2), caseMat, -0.57, 0.85, -1.45));
    P(F, block, 'cylinderBlock', 'Cilindri',
      'Quattro tubi affiancati in cui scorrono i pistoni. Sono inclinati in avanti per fare spazio alla scatola filtro e portare il peso verso la ruota anteriore.',
      'Canne riportate o con riporto al nichel‑carburo di silicio, alesaggio 67 mm. Attorno alle canne c’è l’intercapedine del liquido di raffreddamento (visibile in sezione). A destra il tunnel della catena di distribuzione. Interasse tra i cilindri ridotto al minimo per tenere il motore stretto.',
      [0, 0.55, 0]);

    // ---------- Cylinder head (slices along z so the section shows ports) ----------
    const saddle = (cx, n = 8) => {
      const pts = [];
      const a0 = Math.asin(0.04 / 0.095);
      for (let i = 0; i <= n; i++) {
        const aa = a0 - (i / n) * (Math.PI + 2 * a0);
        pts.push([cx + 0.095 * Math.cos(aa), CAM_Y + 0.095 * Math.sin(aa)]);
      }
      return pts;
    };
    const cylSlab = () => {
      const s = poly([[-0.58, DECK], [-BORE_R, DECK], [0, 1.43], [BORE_R, DECK], [0.58, DECK], [0.6, 1.5], [0.58, 2.12], [0.48, 2.12], [0.48, 1.74], [-0.48, 1.74], [-0.48, 2.12], [-0.58, 2.12], [-0.6, 1.5]]);
      s.holes.push(poly([[-0.56, 1.5], [-0.34, 1.47], [-0.2, 1.425], [-0.1, 1.45], [-0.18, 1.52], [-0.36, 1.64], [-0.56, 1.66]], THREE.Path));
      s.holes.push(poly([[0.56, 1.48], [0.56, 1.62], [0.36, 1.6], [0.2, 1.5], [0.11, 1.44], [0.2, 1.42], [0.34, 1.45]], THREE.Path));
      s.holes.push(poly([[-0.045, 1.47], [0.045, 1.47], [0.045, 1.72], [-0.045, 1.72]], THREE.Path));
      s.holes.push(poly([[-0.3, 1.68], [-0.12, 1.62], [-0.08, 1.62], [-0.08, 1.71], [-0.3, 1.71]], THREE.Path));
      s.holes.push(poly([[0.3, 1.68], [0.3, 1.71], [0.08, 1.71], [0.08, 1.62], [0.12, 1.62]], THREE.Path));
      return s;
    };
    const bridgeSlab = () => {
      const s = poly([[-0.58, DECK], [0.58, DECK], [0.6, 1.5], [0.58, 2.12], ...saddle(CAM_X[1]), ...saddle(CAM_X[0]), [-0.58, 2.12], [-0.6, 1.5]]);
      s.holes.push(circ(0, 1.95, 0.03), circ(-0.32, 1.55, 0.05), circ(0.32, 1.55, 0.05));
      return s;
    };
    const tunnelWalls = () => [
      poly([[-0.6, DECK], [-0.5, DECK], [-0.5, 2.12], [-0.58, 2.12], [-0.6, 1.5]]),
      poly([[0.5, DECK], [0.6, DECK], [0.6, 1.5], [0.58, 2.12], [0.5, 2.12]]),
    ];
    const head = new THREE.Group();
    const bridgeZ = [[-1.64, -1.5], [-0.9, -0.7], [-0.1, 0.1], [0.7, 0.9], [1.5, 1.6], [1.8, 1.86]];
    const bridgeGeo = (a, b) => extZ(bridgeSlab(), a, b, 10);
    for (const [a, b] of bridgeZ) head.add(mesh(bridgeGeo(a, b), headMat));
    const cylGeo = extZ(cylSlab(), -0.3, 0.3, 10);
    for (const c of CYL_Z) head.add(mesh(cylGeo, headMat, 0, 0, c));
    head.add(mesh(extZ(tunnelWalls(), 1.6, 1.8), headMat));
    // cam bearing caps
    const capGeo = new RoundedBoxGeometry(0.2, 0.08, 0.1, 2, 0.02);
    const capBolts = [];
    for (const [a, b] of bridgeZ.slice(0, 5)) for (const cx of CAM_X) {
      const z = (a + b) / 2;
      head.add(mesh(capGeo, headMat, cx, 2.16, z));
      capBolts.push([cx - 0.075, 2.21, z], [cx + 0.075, 2.21, z]);
    }
    head.add(bolts(capBolts, 'y', boltMat, 0.02, 0.02));
    // head gasket (block outline with bore holes)
    head.add(mesh(extY(blockShape(), DECK - 0.006, DECK + 0.006), gasketMat));
    // casting ribs and coolant gallery bosses along the head faces
    for (const x of [-0.6, 0.6]) {
      head.add(rbox(0.05, 0.05, 3.44, 0.02, headMat, x * 1.02, 1.95, 0.11));
      head.add(rbox(0.05, 0.04, 3.44, 0.015, headMat, x * 1.01, 1.38, 0.11));
    }
    for (const z of [-1.66, 1.88]) for (const cx of CAM_X) head.add(mesh(cylZ(0.11, 0.05, 24), headMat, cx, CAM_Y, z));
    // exhaust stud bosses on the front face
    const studs = [];
    for (const c of CYL_Z) studs.push([0.61, 1.47, c - 0.12], [0.61, 1.64, c + 0.12]);
    head.add(bolts(studs, 'x', boltMat, 0.022, 0.03));
    P(F, head, 'cylinderHead', 'Testata',
      'Il “tetto” dei cilindri: qui ci sono le camere di scoppio, i condotti dove entra l’aria e escono i gas, 16 valvole e le candele.',
      'Testata in lega di alluminio con camere a tetto (pent‑roof), 4 valvole per cilindro con angolo tra le valvole stretto (≈ 22°) per una camera compatta. Condotti di aspirazione quasi diritti e ripidi verso le valvole, condotti di scarico corti; sedi e guide valvola riportate, punterie a bicchierino e alberi a camme su supporti con cappelli lavorati in linea.',
      [0, 1.15, 0]);

    // Cam cover
    const ccOut = [[-0.6, 2.12], [-0.6, 2.3], [-0.54, 2.39], [-0.44, 2.43], [0.44, 2.43], [0.54, 2.39], [0.6, 2.3], [0.6, 2.12]];
    const ccIn = [[0.55, 2.12], [0.55, 2.29], [0.5, 2.36], [0.42, 2.38], [-0.42, 2.38], [-0.5, 2.36], [-0.55, 2.29], [-0.55, 2.12]];
    const camCover = new THREE.Group();
    camCover.add(mesh(extZ(poly([...ccOut, ...ccIn]), -1.66, 1.88, 12, 0.012), camCoverMat));
    camCover.add(mesh(extZ(poly(ccOut), -1.71, -1.65, 8, 0.015), camCoverMat), mesh(extZ(poly(ccOut), 1.87, 1.93, 8, 0.015), camCoverMat));
    for (const x of [-0.605, 0.605]) camCover.add(mesh(new THREE.BoxGeometry(0.014, 0.014, 3.62), gasketMat, x, 2.125, 0.11));
    const ccBolts = [];
    for (const z of [-1.5, -0.8, 0, 0.8, 1.55]) ccBolts.push([-0.3, 2.44, z], [0.3, 2.44, z]);
    camCover.add(bolts(ccBolts, 'y', boltMat, 0.024, 0.025));
    // breather nipple
    camCover.add(mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.1, 12).rotateZ(Math.PI / 2), blackCut, -0.64, 2.3, -1.2));
    P(F, camCover, 'camCover', 'Coperchio punterie',
      'Il coperchio che chiude in alto la testata e tiene dentro l’olio che bagna gli alberi a camme.',
      'Coperchio in magnesio o alluminio pressofuso con guarnizione in gomma; le bobine a candela passano attraverso i suoi pozzetti. Integra lo sfiato dei vapori d’olio (blow‑by), rimandati all’airbox.',
      [0, 2.1, 0]);

    // ---------- Crankshaft (cylinder frame, axis z) ----------
    const crank = new THREE.Group();
    const webShape = (s) => {
      const pts = [...circlePts(0, s * R, 0.235, 24), ...circlePts(0, 0, 0.2, 20)];
      for (let i = 0; i <= 12; i++) {
        const a = -Math.PI / 2 + (i / 12 - 0.5) * 2.4;
        pts.push([0.43 * Math.cos(a), s * 0.43 * Math.sin(a)]);
      }
      return poly(convexHull(pts));
    };
    const webGeo = [webShape(1), webShape(-1)].map((sh) => extZ(sh, -0.11, 0.11, 8));
    for (const z of JOURNALS) crank.add(mesh(cylZ(0.155, 0.14, 28), steel, 0, 0, z));
    CYL_Z.forEach((c, k) => {
      const s = k === 0 || k === 3 ? 1 : -1;
      crank.add(mesh(cylZ(0.165, 0.22, 28), steel, 0, s * R, c));
      for (const dz of [-0.22, 0.22]) crank.add(mesh(webGeo[s > 0 ? 0 : 1], steel, 0, 0, c + dz));
    });
    crank.add(mesh(cylZ(0.12, 0.72, 24), steel, 0, 0, 2.03));
    crank.add(mesh(cylZ(0.12, 0.45, 24), steel, 0, 0, -1.9));
    crank.add(mesh(gearGeo(16, 0.09, 0.06, 0.05), darkSteel, 0, 0, CHAIN_Z));
    const pulse = mesh(cylZ(0.2, 0.03, 36), darkSteel, 0, 0, 2.34);
    pulse.add(mesh(new THREE.BoxGeometry(0.06, 0.04, 0.03), darkSteel, 0, 0.21, 0));
    crank.add(pulse);
    P(F, crank, 'crankshaft', 'Albero motore',
      'L’albero “a gomiti” che trasforma il su‑e‑giù dei pistoni in rotazione. È corto e leggero: in moto deve poter accelerare e rallentare in un attimo.',
      'Albero forgiato in acciaio su 5 supporti di banco con bronzine, manovelle a 180° (albero piatto) e contrappesi integrati nelle maschette. A destra porta il pignone della catena di distribuzione, l’ingranaggio della primaria e la ruota foronica per il sensore di giri; a sinistra il rotore dell’alternatore. Momento d’inerzia basso per una risposta pronta del gas.',
      [0, -0.9, 0]);

    // ---------- Pistons, conrods ----------
    const pistonGeo = lathe([[0, 0.17], [0.31, 0.17], [0.33, 0.155], [0.33, -0.02], [0.32, -0.1], [0.29, -0.1], [0.29, 0.12], [0, 0.12]], 40);
    const ringGeo = new THREE.TorusGeometry(0.331, 0.007, 5, 40).rotateX(Math.PI / 2);
    const pistonMat = MAT.aluminum();
    const pistonGroup = new THREE.Group();
    const pistons = CYL_Z.map((c) => {
      const p = new THREE.Group();
      p.add(new THREE.Mesh(pistonGeo, pistonMat));
      for (const y of [0.145, 0.12, 0.09]) p.add(mesh(ringGeo, darkSteel, 0, y, 0));
      p.add(mesh(cylZ(0.07, 0.52, 16), steel));
      p.position.z = c;
      pistonGroup.add(p);
      return p;
    });
    P(F, pistonGroup, 'pistons', 'Pistoni',
      'Piccoli e leggerissimi: pesano meno di un etto e mezzo l’uno e salgono e scendono fino a 250 volte al secondo.',
      'Pistoni forgiati con mantello corto (slipper), due fasce di tenuta e una raschiaolio sottili, spinotto flottante. A 15 000 giri con corsa 42,5 mm la velocità media è ≈ 21 m/s e l’accelerazione al PMS supera 6 000 g: ogni grammo risparmiato conta. Getti d’olio sotto il cielo li raffreddano.',
      [0, 0.35, 0]);

    const rodGroup = new THREE.Group();
    const rodBeam = new THREE.BoxGeometry(0.1, L - 0.34, 0.07).translate(0, L / 2, 0);
    const rodFlange = new THREE.BoxGeometry(0.03, L - 0.36, 0.11).translate(0, L / 2, 0);
    const bigEnd = zAxis(pipe(0.235, 0.165, 0.1, 28));
    const smallEnd = zAxis(pipe(0.105, 0.07, 0.1, 20)).translate(0, L, 0);
    const rodMat = MAT.steel();
    const rods = CYL_Z.map((c) => {
      const r = new THREE.Group();
      r.add(new THREE.Mesh(rodBeam, rodMat), new THREE.Mesh(bigEnd, rodMat), new THREE.Mesh(smallEnd, rodMat));
      for (const x of [-0.06, 0.06]) r.add(mesh(rodFlange, rodMat, x, 0, 0));
      r.add(mesh(new THREE.BoxGeometry(0.5, 0.05, 0.1), rodMat, 0, -0.2, 0));
      for (const x of [-0.2, 0.2]) r.add(mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.06, 6), MAT.chrome(), x, -0.235, 0));
      r.position.z = c;
      rodGroup.add(r);
      return r;
    });
    P(F, rodGroup, 'conrods', 'Bielle',
      'Le “braccia” che collegano ogni pistone all’albero motore, trasformando la spinta in rotazione.',
      'Bielle in acciaio forgiato (titanio su alcune moto da corsa) con sezione a doppio T, testa divisa con cappello e bulloni, bronzine sul perno di manovella. Lunghe circa 2,2 volte la corsa per contenere l’inclinazione e l’attrito del pistone sul cilindro.',
      [0, -0.3, 0]);

    // Gas charge in each cylinder
    const chargeGeo = new THREE.CylinderGeometry(0.332, 0.332, 1, 36);
    const charges = CYL_Z.map((c) => {
      const m = mesh(chargeGeo, MAT.gas(0x5fd3ff, 0.2, false), 0, 1, c);
      m.renderOrder = 2;
      F.add(m);
      return m;
    });

    // ---------- Valvetrain ----------
    const valveGeo = (r) => lathe([[0, 0], [r, 0], [r, 0.012], [0.035, 0.06], [0.022, 0.1], [0.022, 0.55], [0, 0.55]], 20);
    const springCurve = new (class extends THREE.Curve {
      getPoint(t, out = new THREE.Vector3()) {
        const a = t * TAU * 4.5;
        return out.set(Math.cos(a) * 0.06, t * 0.2, Math.sin(a) * 0.06);
      }
    })();
    const springGeo = new THREE.TubeGeometry(springCurve, 54, 0.009, 4);
    const bucketGeo = new THREE.CylinderGeometry(0.1, 0.1, 0.1, 20);
    const retGeo = new THREE.CylinderGeometry(0.07, 0.04, 0.03, 16);
    const makeValves = (x0, tilt, r, mat) => {
      const g = new THREE.Group();
      const list = [];
      CYL_Z.forEach((c) => {
        for (const dz of [-VALVE_DZ, VALVE_DZ]) {
          const v = new THREE.Group();
          const mov = new THREE.Group();
          mov.add(new THREE.Mesh(valveGeo(r), mat), mesh(retGeo, steel, 0, 0.475, 0), mesh(bucketGeo, MAT.chrome(), 0, 0.55, 0));
          const spring = mesh(springGeo, darkSteel, 0, 0.27, 0);
          v.add(mov, spring);
          v.position.set(x0, 1.395, c + dz);
          v.rotation.z = tilt;
          g.add(v);
          list.push({ mov, spring });
        }
      });
      return { g, list };
    };
    const inV = makeValves(-0.125, VTILT, 0.115, steel);
    const exV = makeValves(0.12, -VTILT, 0.1, MAT.titanium());
    P(F, inV.g, 'intakeValves', 'Valvole di aspirazione',
      'Due per cilindro: si aprono per far entrare l’aria e la benzina. Sono richiamate da molle molto rigide per seguire le camme anche a 15 000 giri.',
      'Valvole a fungo da ≈ 27 mm (titanio su alcuni modelli), alzata ≈ 8–9 mm, comandate direttamente dalle camme tramite punterie a bicchierino con pastiglie di registro. A questi regimi il limite è il “galleggiamento” della valvola: servono masse piccole e molle rigide (o il richiamo desmodromico di Ducati, o pneumatico in MotoGP).',
      [-0.3, 0.75, 0]);
    P(F, exV.g, 'exhaustValves', 'Valvole di scarico',
      'Due per cilindro, più piccole: si aprono per far uscire i gas bruciati.',
      'Diametro ≈ 22–23 mm, in acciaio resistente al calore o titanio; lavorano oltre 700 °C e cedono calore alla sede raffreddata dal liquido. Si aprono prima del PMI per sfruttare la pressione residua (scarico spontaneo).',
      [0.3, 0.75, 0]);

    // Camshafts
    const lobeShape = poly(Array.from({ length: 40 }, (_, i) => {
      const a = (i / 40) * TAU;
      const r = 0.1 + 0.085 * Math.pow(Math.max(0, Math.cos(a)), 2.2);
      return [r * Math.cos(a), r * Math.sin(a)];
    }));
    const lobeGeo = extZ(lobeShape, -0.035, 0.035, 4);
    const camSprocketGeo = gearGeo(32, 0.18, 0.05, 0.05, 5);
    const makeCam = (x, peak, target) => {
      const g = new THREE.Group();
      g.add(mesh(cylZ(0.055, 3.47, 16), steel, 0, 0, 0.1));
      for (const [a, b] of bridgeZ.slice(0, 5)) g.add(mesh(cylZ(0.085, b - a - 0.01, 24), MAT.chrome(), 0, 0, (a + b) / 2));
      CYL_Z.forEach((c, k) => {
        for (const dz of [-VALVE_DZ, VALVE_DZ]) {
          const lobe = mesh(lobeGeo, MAT.chrome(), 0, 0, c + dz);
          lobe.rotation.z = target + (OFFSET[k] + peak) * TAU;
          g.add(lobe);
        }
      });
      g.add(mesh(camSprocketGeo, darkSteel, 0, 0, CHAIN_Z));
      g.add(mesh(cylZ(0.07, 0.06, 6), steel, 0, 0, CHAIN_Z + 0.05));
      g.position.set(x, CAM_Y, 0);
      return g;
    };
    const camIn = makeCam(CAM_X[0], 0.125, VTILT - Math.PI / 2);
    const camEx = makeCam(CAM_X[1], 0.875, -Math.PI / 2 - VTILT);
    const cams = new THREE.Group();
    cams.add(camIn, camEx);
    P(F, cams, 'camshafts', 'Alberi a camme',
      'Due alberi con delle “gobbe”: girando premono le valvole al momento giusto. Girano a metà dei giri dell’albero motore.',
      'Doppio albero a camme in testa (DOHC) in ghisa o acciaio, 8 camme ciascuno, su supporti con cappelli; profili con alzata e durata spinte (aspirazione ≈ 260–290° di durata) per riempire bene ad alto regime. Comandati dalla catena con rapporto 2:1 (pignone 16 denti, corone 32).',
      [0, 1.75, 0]);

    // ---------- Cam chain, tensioner and guides ----------
    const hull = convexHull([...circlePts(0, 0, 0.1, 24), ...circlePts(CAM_X[0], CAM_Y, 0.19, 30), ...circlePts(CAM_X[1], CAM_Y, 0.19, 30)]);
    const chainCurve = new THREE.CatmullRomCurve3(hull.map(([x, y]) => new THREE.Vector3(x, y, CHAIN_Z)), true, 'centripetal');
    const chainLen = chainCurve.getLength();
    const NLINK = Math.round(chainLen / 0.034);
    const camChainLinks = new THREE.InstancedMesh(new THREE.BoxGeometry(0.036, 0.016, 0.055), darkSteel, NLINK);
    camChainLinks.frustumCulled = false;
    const camChain = new THREE.Group();
    camChain.add(camChainLinks);
    const tens = new THREE.Group();
    const shoe = (a, b, out, t0, t1, bow) => {
      const A = new THREE.Vector2(...a), B = new THREE.Vector2(...b);
      const d = B.clone().sub(A).normalize();
      const n = new THREE.Vector2(-d.y, d.x).multiplyScalar(out);
      const pts = [], back = [];
      for (let i = 0; i <= 10; i++) {
        const t = lerp(t0, t1, i / 10);
        const p = A.clone().lerp(B, t).addScaledVector(n, 0.03 + bow * Math.sin(Math.PI * (i / 10)));
        pts.push([p.x, p.y]);
        back.push([p.x + n.x * 0.045, p.y + n.y * 0.045]);
      }
      return extZ(poly([...pts, ...back.reverse()]), CHAIN_Z - 0.035, CHAIN_Z + 0.035, 4);
    };
    const shoeMat = MAT.paint(0x3b2a20, 0.55);
    tens.add(new THREE.Mesh(shoe([-0.1, 0], [CAM_X[0] - 0.19, CAM_Y], 1, 0.12, 0.8, -0.012), shoeMat));
    tens.add(new THREE.Mesh(shoe([0.1, 0], [CAM_X[1] + 0.19, CAM_Y], -1, 0.12, 0.85, 0), shoeMat));
    tens.add(mesh(new THREE.BoxGeometry(0.36, 0.035, 0.06), shoeMat, (CAM_X[0] + CAM_X[1]) / 2, CAM_Y + 0.235, CHAIN_Z));
    // hydraulic tensioner on the back of the cylinder
    const tensBody = mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.22, 16).rotateZ(Math.PI / 2), cut(MAT.aluminum()), -0.58, 1.0, CHAIN_Z);
    tens.add(tensBody, mesh(new RoundedBoxGeometry(0.04, 0.2, 0.16, 2, 0.015), tensBody.material, -0.53, 1.0, CHAIN_Z));
    tens.add(bolts([[-0.555, 1.07, CHAIN_Z], [-0.555, 0.93, CHAIN_Z]], 'x', boltMat, 0.02, 0.02));
    tens.add(mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.15, 10).rotateZ(Math.PI / 2), steel, -0.42, 1.0, CHAIN_Z));
    P(F, camChain, 'camChain', 'Catena della distribuzione',
      'Una catena come quella della bicicletta, sul lato destro del motore: collega l’albero motore ai due alberi a camme, che girano a metà velocità.',
      'Catena silenziosa (a denti) o a rulli, rapporto 2:1. Laterale per tenere stretto il motore; la catena è più robusta di una cinghia e non richiede sostituzioni periodiche, ma si allunga con l’usura: per questo serve il tenditore. Nei motori interferenti un salto di denti fa toccare valvole e pistoni.',
      [0, 0, 1.0]);
    P(F, tens, 'chainTensioner', 'Tenditore e pattini della catena',
      'Pattini in plastica guidano la catena; dietro al cilindro un pistoncino a molla spinge il pattino mobile per tenerla sempre tesa.',
      'Tenditore automatico a molla con cricchetto antiritorno o idraulico (alimentato dall’olio motore) sul lato lasco, che qui è quello posteriore perché l’albero motore tira la catena dal lato anteriore. Pattino fisso anteriore e guida superiore tra le corone; i pattini sono in poliammide su anima in acciaio o alluminio.',
      [-0.5, 0, 1.0]);

    // ---------- Spark plugs and stick coils ----------
    const plugs = new THREE.Group();
    const coilMat = cut(MAT.paint(0x15171a, 0.5));
    const sparks = [];
    const sparkLights = [];
    CYL_Z.forEach((c) => {
      const p = new THREE.Group();
      p.add(mesh(new THREE.CylinderGeometry(0.022, 0.016, 0.1, 10), steel, 0, 0.05, 0));
      p.add(mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.05, 6), MAT.chrome(), 0, 0.13, 0));
      p.add(mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.16, 12), MAT.ceramic(), 0, 0.23, 0));
      p.position.set(0, 1.42, c);
      plugs.add(p);
      plugs.add(mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.75, 12), coilMat, 0, 2.05, c));
      plugs.add(rbox(0.13, 0.1, 0.15, 0.02, coilMat, 0, 2.48, c));
      plugs.add(rbox(0.08, 0.05, 0.07, 0.01, cut(MAT.paint(0x4a4f57, 0.5)), -0.08, 2.5, c));
      const spark = mesh(new THREE.SphereGeometry(0.05, 12, 8), MAT.glow(0xbfe6ff, 6, 0.95), 0, 1.415, c);
      spark.visible = false;
      F.add(spark);
      sparks.push(spark);
      const light = new THREE.PointLight(0xffa040, 0, 1.6, 2);
      light.position.set(0.1, 1.3, c + 0.3);
      F.add(light);
      sparkLights.push(light);
    });
    P(F, plugs, 'sparkPlugs', 'Candele e bobine',
      'Ogni cilindro ha la sua candela e, sopra, la sua bobina: insieme fanno la scintilla che accende la miscela.',
      'Bobine a candela (stick‑coil) da 30–40 kV inserite nei pozzetti del coperchio; candele a elettrodo sottile in iridio, filettatura 10 mm per lasciare spazio a valvole grandi. La centralina gestisce anticipo e taglio d’accensione (limitatore, controllo di trazione, quickshifter).',
      [0, 1.4, 0]);

    // ---------- Intake: throttle bodies, injectors, airbox ----------
    const TB_B = new THREE.Vector2(0.17, 1.83);
    const TB_PHI = 0.32;
    const tbDir = new THREE.Vector2(-Math.sin(TB_PHI), Math.cos(TB_PHI));
    const tbAt = (u, v) => TB_B.clone().add(new THREE.Vector2(u * Math.cos(TB_PHI) - v * Math.sin(TB_PHI), u * Math.sin(TB_PHI) + v * Math.cos(TB_PHI)));
    const tbMat = cut(MAT.aluminum());
    tbMat.color.set(0x9da4ab);
    const throttleBodies = new THREE.Group();
    const butterflies = [];
    const stackGeo = lathe([[0.085, 0], [0.105, 0], [0.11, 0.12], [0.16, 0.24], [0.16, 0.26], [0.145, 0.26], [0.095, 0.13], [0.085, 0]], 32);
    const tbGeo = pipe(0.108, 0.085, 0.3, 32);
    const portA = fr(-0.6, 1.57);
    CYL_Z.forEach((c) => {
      const g = new THREE.Group();
      g.position.set(TB_B.x, TB_B.y, c);
      g.rotation.z = TB_PHI;
      g.add(mesh(tbGeo, tbMat, 0, 0.15, 0));
      g.add(mesh(stackGeo, tbMat, 0, 0.3, 0));
      g.add(rbox(0.1, 0.12, 0.2, 0.02, tbMat, 0.1, 0.13, 0));
      const bf = mesh(new THREE.CylinderGeometry(0.084, 0.084, 0.01, 24), cut(MAT.steel()), 0, 0.14, 0);
      g.add(bf);
      butterflies.push(bf);
      throttleBodies.add(g);
      throttleBodies.add(tube([[portA[0], portA[1], c], [portA[0] - 0.06, portA[1] + 0.07, c], [TB_B.x + 0.01, TB_B.y - 0.03, c]], 0.098, rubberCut, 8, 20));
      for (const dy of [0.05, 0.13]) throttleBodies.add(mesh(new THREE.TorusGeometry(0.104, 0.012, 6, 28).rotateX(Math.PI / 2).rotateZ(-0.6), boltMat, portA[0] - 0.05 * (dy + 0.4), portA[1] + dy, c));
    });
    const shaftP = tbAt(0, 0.14);
    throttleBodies.add(mesh(cylZ(0.014, 3.2, 8), steel, shaftP.x, shaftP.y, 0));
    const pulleyP = tbAt(0.0, 0.14);
    throttleBodies.add(mesh(cylZ(0.07, 0.04, 24), tbMat, pulleyP.x, pulleyP.y, -1.62));
    throttleBodies.add(rbox(0.12, 0.14, 0.08, 0.02, blackCut, pulleyP.x, pulleyP.y, 1.6));
    P(root, throttleBodies, 'throttleBodies', 'Corpi farfallati',
      'Quattro “rubinetti” dell’aria, uno per cilindro: la farfalla dentro ruota quando giri la manopola del gas.',
      'Corpi farfallati da ≈ 40–44 mm, uno per cilindro, sincronizzati su un unico alberino; condotti corti e diritti per la massima portata ad alto regime. Il sensore di posizione farfalla (TPS) e, sulle moto recenti, un motorino ride‑by‑wire permettono mappe di potenza e controllo di trazione. Collegati alla testata da manicotti in gomma.',
      [-0.6, 0.6, 0]);

    const injectors = new THREE.Group();
    const injMat = cut(MAT.paint(0x2b2f36, 0.45));
    CYL_Z.forEach((c) => {
      const p = tbAt(-0.15, 0.1);
      const inj = mesh(new THREE.CylinderGeometry(0.03, 0.022, 0.18, 12), injMat, p.x, p.y, c);
      inj.rotation.z = TB_PHI + 0.75;
      injectors.add(inj);
      const q = tbAt(-0.21, 0.17);
      injectors.add(rbox(0.05, 0.05, 0.08, 0.012, cut(MAT.paint(0x5a5f68, 0.5)), q.x, q.y, c + 0.06));
    });
    const railP = tbAt(-0.24, 0.16);
    injectors.add(mesh(cylZ(0.03, 3.1, 12), cut(MAT.chrome()), railP.x, railP.y, 0));
    injectors.add(tube([[railP.x, railP.y, -1.55], [railP.x - 0.1, railP.y + 0.05, -1.75], [railP.x - 0.4, railP.y - 0.1, -1.8]], 0.022, rubberCut, 12, 8));
    P(root, injectors, 'injectors', 'Iniettori e flauto',
      'Spruzzatori comandati dalla centralina: nebulizzano la benzina nell’aria appena prima della valvola.',
      'Iniezione indiretta sequenziale a ≈ 3–4 bar, un iniettore per cilindro (molte sportive ne aggiungono un secondo sopra le trombette per gli alti regimi). A 15 000 giri c’è meno di un centesimo di secondo per ciclo: gli iniettori sono multiforo per nebulizzare in fretta.',
      [-0.9, 0.9, 0]);

    const airbox = new THREE.Group();
    const abMat = cut(MAT.paint(0x1b1d21, 0.65), { cap: false });
    abMat.metalness = 0.1;
    airbox.add(rbox(1.3, 0.48, 3.3, 0.12, abMat, -0.3, 2.6, 0.1));
    airbox.add(mesh(new THREE.BoxGeometry(1.0, 0.05, 2.9), cut(MAT.paint(0xc9c0a0, 0.9)), -0.3, 2.74, 0.1));
    airbox.add(tube([[0.3, 2.64, 0.1], [0.8, 2.72, 0.1], [1.3, 2.78, 0.1]], 0.13, abMat, 12, 20));
    airbox.add(mesh(new THREE.TorusGeometry(0.135, 0.02, 6, 28).rotateY(Math.PI / 2), abMat, 1.3, 2.78, 0.1));
    const abBolts = [];
    for (const z of [-1.4, -0.5, 0.5, 1.5]) abBolts.push([-0.95, 2.6, z], [0.35, 2.6, z]);
    airbox.add(bolts(abBolts, 'x', boltMat, 0.022, 0.02));
    P(root, airbox, 'airbox', 'Scatola filtro (airbox)',
      'Una scatola sopra il motore che contiene il filtro dell’aria. Davanti un condotto prende aria fresca dal muso della moto.',
      'Airbox in plastica con filtro a pannello e presa d’aria dinamica (ram‑air): con la moto veloce l’aria entra in leggera pressione e aumenta il riempimento. Il volume dell’airbox smorza le pulsazioni e il rumore di aspirazione; dentro sfociano le trombette dei corpi farfallati.',
      [-0.2, 1.3, 0]);

    // ---------- Exhaust 4-2-1 ----------
    const exMat = cut(MAT.paint(0xb3966a, 0.32));
    exMat.metalness = 0.85;
    const exhaust = new THREE.Group();
    const P0 = fr(0.6, 1.55), P1 = fr(0.86, 1.5);
    const headerPts = CYL_Z.map((c, k) => {
      const outer = k === 0 || k === 3;
      return outer
        ? [[P0[0], P0[1], c], [P1[0], P1[1], c], [1.64, 0.3, c], [1.6, -0.62, c * 0.92], [1.3, -1.12, c * 0.55], [0.85, -1.22, c * 0.18], [0.45, -1.24, -0.12]]
        : [[P0[0], P0[1], c], [P1[0], P1[1], c], [1.64, 0.3, c], [1.55, -0.55, c], [1.25, -0.9, c * 0.65], [0.85, -0.98, c * 0.3], [0.5, -1.0, 0.12]];
    });
    headerPts.forEach((pts, k) => {
      exhaust.add(tube(pts, 0.065, exMat, 60, 12));
      const fl = rbox(0.04, 0.24, 0.32, 0.015, exMat, ...fr(0.625, 1.55), CYL_Z[k]);
      fl.rotation.z = -TILT;
      exhaust.add(fl);
    });
    const S14 = [[0.45, -1.24, -0.12], [0.0, -1.22, -0.08], [-0.4, -1.18, 0]];
    const S23 = [[0.5, -1.0, 0.12], [0.15, -1.1, 0.1], [-0.4, -1.16, 0.02]];
    const COLL = [[-0.4, -1.17, 0], [-1.2, -1.2, 0], [-2.2, -1.18, 0]];
    exhaust.add(tube(S14, 0.085, exMat, 20, 12), tube(S23, 0.085, exMat, 20, 12), tube(COLL, 0.11, exMat, 24, 16));
    exhaust.add(mesh(new THREE.TorusGeometry(0.115, 0.025, 8, 28).rotateY(Math.PI / 2), exMat, -2.2, -1.18, 0));
    exhaust.add(mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.14, 8), cut(MAT.chrome()), -0.9, -1.06, 0.05));
    P(root, exhaust, 'exhaustHeaders', 'Collettori di scarico 4‑2‑1',
      'Quattro tubi, uno per cilindro, che si uniscono a due a due e poi in un solo tubo che va al silenziatore.',
      'Collettori in acciaio inox o titanio, unione 1‑4 e 2‑3 e poi collettore unico (4‑2‑1): lunghezze accordate per sfruttare le onde di pressione a un regime scelto. Sul tubo finale la sonda lambda; più a valle catalizzatore e silenziatore (qui non rappresentati).',
      [1.0, -0.3, 0]);

    // ---------- Cooling ----------
    const cooling = new THREE.Group();
    const pumpMat = cut(MAT.aluminum());
    cooling.add(mesh(cylZ(0.17, 0.2, 28), pumpMat, 0.15, -0.8, -1.82));
    cooling.add(mesh(cylZ(0.12, 0.06, 28), pumpMat, 0.15, -0.8, -1.95));
    cooling.add(bolts([[0.3, -0.8, -1.98], [0.0, -0.8, -1.98], [0.15, -0.66, -1.98], [0.15, -0.94, -1.98]], 'z', boltMat, 0.02, 0.02));
    const hoseLow = tube([[0.3, -0.72, -1.9], [0.9, -0.6, -1.85], [1.35, -0.2, -1.7]], 0.075, rubberCut, 20, 12);
    const hoseBlock = tube([[0.02, -0.7, -1.95], [-0.55, -0.5, -1.98], [-0.66, 0.45, -1.92], [-0.4, 0.97, -1.62], ...[fr(-0.63, 0.85)].map(([x, y]) => [x, y, -1.45])], 0.06, rubberCut, 40, 12);
    const thermo = new THREE.Vector3(...fr(0.68, 1.92), -1.5);
    const hoseUp = tube([[thermo.x + 0.05, thermo.y + 0.03, -1.5], [1.85, 1.5, -1.55], [2.05, 1.5, -1.6]], 0.07, rubberCut, 16, 12);
    cooling.add(hoseLow, hoseBlock, hoseUp);
    for (const [x, y, z] of [[1.3, -0.24, -1.71], [0.36, -0.71, -1.9], [-0.1, 1.04, -1.47], [2.0, 1.5, -1.6]])
      cooling.add(mesh(new THREE.TorusGeometry(0.08, 0.012, 6, 24).rotateY(Math.PI / 2), boltMat, x, y, z));
    const thHouse = rbox(0.14, 0.16, 0.2, 0.04, pumpMat, thermo.x, thermo.y, thermo.z);
    thHouse.rotation.z = -TILT;
    cooling.add(thHouse);
    for (const end of [[1.35, -0.2, -1.7], [2.05, 1.5, -1.6]]) cooling.add(mesh(new THREE.TorusGeometry(0.075, 0.018, 6, 24).rotateY(Math.PI / 2), MAT.paint(0x5fd3ff, 0.4), ...end));
    P(root, cooling, 'waterPump', 'Pompa dell’acqua e manicotti',
      'Una pompa fa circolare il liquido di raffreddamento attorno ai cilindri e nella testata, poi verso il radiatore davanti alla moto (qui solo l’attacco dei tubi).',
      'Pompa centrifuga sul lato sinistro, sullo stesso alberino della pompa dell’olio. Il liquido entra nel basamento, sale nella testata e torna al radiatore attraverso il termostato (apertura ≈ 80 °C); con il motore freddo il termostato chiude il circuito del radiatore per scaldare in fretta. La ventola elettrica interviene nel traffico.',
      [0, -0.4, -0.9]);

    // ---------- Oil: shared by engine, clutch and gearbox ----------
    const oil = new THREE.Group();
    const oilMat = cut(MAT.gas(0xd9a030, 0.5, false), { cap: false });
    oil.add(mesh(extZ(poly([[-1.86, -0.915], [0.17, -0.915], [0.31, -0.82], [0.34, -0.79], [-2.04, -0.79]]), -1.64, 1.74, 4), oilMat));
    const pump = new THREE.Group();
    pump.add(rbox(0.26, 0.22, 0.3, 0.04, MAT.aluminum(), 0.15, -0.78, -1.45));
    pump.add(tube([[0.05, -0.85, -1.4], [-0.25, -0.88, -1.2], [-0.45, -0.89, -1.0]], 0.025, MAT.aluminum(), 10, 8));
    pump.add(rbox(0.28, 0.03, 0.22, 0.01, MAT.darkSteel(), -0.5, -0.89, -0.95));
    oil.add(pump);
    const filt = mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.3, 28).rotateZ(Math.PI / 2), cut(MAT.paint(0x1d2d52, 0.4)), 0.9, -0.42, 0);
    oil.add(filt, mesh(new THREE.CylinderGeometry(0.12, 0.13, 0.06, 28).rotateZ(Math.PI / 2), caseLowMat, 0.75, -0.42, 0));
    P(root, oil, 'engineOil', 'Olio, pompa e filtro',
      'Lo stesso olio lubrifica il motore, la frizione e il cambio. Si raccoglie in basso, la pompa lo aspira e lo manda ovunque, passando dal filtro.',
      'Carter umido con ≈ 3,5 litri: la pompa trocoidale pesca dalla coppa attraverso la succheruola e manda olio filtrato in pressione a banco, bielle, alberi a camme e getti sotto i pistoni; il cambio è lubrificato a sbattimento e da getti. Per questo l’olio moto deve essere adatto alla frizione in bagno d’olio (norma JASO MA): additivi troppo “scivolosi” la farebbero slittare.',
      [0, -0.9, 0.6]);

    // ---------- Starter motor ----------
    const starter = new THREE.Group();
    starter.add(mesh(cylZ(0.13, 0.62, 28), cut(MAT.paint(0x22252a, 0.45)), -0.66, 1.0, -1.2));
    starter.add(mesh(cylZ(0.135, 0.08, 28), cut(MAT.aluminum()), -0.66, 1.0, -1.55));
    starter.add(mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.08, 8), cut(MAT.copper()), -0.66, 1.15, -1.0));
    P(root, starter, 'starter', 'Motorino d’avviamento',
      'Un piccolo motore elettrico che fa girare l’albero motore per avviare il motore quando premi il pulsante.',
      'Motorino a magneti permanenti da ≈ 0,5–0,8 kW alimentato dalla batteria a 12 V; agisce sull’albero motore tramite ingranaggi di rinvio e una ruota libera (frizione d’avviamento) sul rotore dell’alternatore, che si sgancia appena il motore parte.',
      [-0.5, 0.6, -0.6]);

    // ---------- Alternator (left) ----------
    const alt = new THREE.Group();
    const altRotor = new THREE.Group();
    altRotor.add(mesh(zAxis(pipe(0.42, 0.37, 0.22, 40)), darkSteel, 0, 0, -1.92));
    altRotor.add(mesh(cylZ(0.42, 0.03, 40), darkSteel, 0, 0, -2.04));
    alt.add(altRotor);
    const stator = new THREE.Group();
    for (let i = 0; i < 12; i++) {
      const a = (i / 12) * TAU;
      const coil = mesh(new THREE.BoxGeometry(0.11, 0.08, 0.14), MAT.copper(), 0.27 * Math.cos(a), 0.27 * Math.sin(a), -1.92);
      coil.rotation.z = a;
      stator.add(coil);
    }
    stator.add(mesh(cylZ(0.15, 0.16, 24), darkSteel, 0, 0, -1.92));
    alt.add(stator);
    const altCover = lathe([[0.2, 0], [0.5, 0], [0.5, 0.25], [0.44, 0.4], [0.3, 0.48], [0.2, 0.5], [0.2, 0.45], [0.28, 0.43], [0.4, 0.36], [0.45, 0.24], [0.45, 0], [0.2, 0]], 48);
    altCover.rotateX(-Math.PI / 2);
    const altC = new THREE.Group();
    altC.add(mesh(altCover, coverMat, 0, 0, -1.72));
    altC.add(mesh(cylZ(0.2, 0.05, 40), coverMat, 0, 0, -2.2));
    const acBolts = [];
    for (let i = 0; i < 8; i++) acBolts.push([0.47 * Math.cos((i / 8) * TAU + 0.2), 0.47 * Math.sin((i / 8) * TAU + 0.2), -1.75]);
    altC.add(bolts(acBolts, 'z', boltMat, 0.025, 0.03));
    P(root, alt, 'alternator', 'Alternatore',
      'Un generatore sull’albero motore che produce l’elettricità per candele, iniettori, luci e per ricaricare la batteria.',
      'Rotore a magneti permanenti calettato sull’albero motore che gira attorno a uno statore con 12–18 bobine trifase; regolatore/raddrizzatore esterno. Potenza ≈ 350–450 W. Il rotore fa anche da piccolo volano.',
      [0, 0, -0.9]);
    P(root, altC, 'alternatorCover', 'Coperchio alternatore',
      'Il coperchio sul lato sinistro che protegge l’alternatore.',
      'Coperchio in magnesio o alluminio; lo statore è fissato al suo interno e l’olio del carter raffredda le bobine.',
      [0, 0, -1.6]);

    // ---------- Wiring harness and sensors ----------
    const wireMat = cut(MAT.paint(0x111214, 0.7));
    const wiring = new THREE.Group();
    const coilRun = fr(-0.2, 2.47);
    wiring.add(tube([[-0.35, 2.06, 1.6], [-0.35, 2.06, -1.6], [-0.45, 1.9, -1.85], [-0.7, 1.3, -1.9], [-1.4, 1.0, -1.9]], 0.03, wireMat, 30, 8));
    wiring.add(tube([[coilRun[0], coilRun[1], 1.35], [coilRun[0], coilRun[1], -1.5], [0.7, 2.35, -1.85], [-0.2, 2.15, -1.85], [-0.38, 2.0, -1.75]], 0.018, wireMat, 30, 6));
    CYL_Z.forEach((c) => {
      const q = tbAt(-0.21, 0.17);
      wiring.add(tube([[q.x, q.y + 0.02, c + 0.1], [q.x - 0.15, q.y + 0.1, c + 0.15], [-0.35, 2.06, c + 0.2]], 0.012, wireMat, 8, 5));
    });
    P(root, wiring, 'wiring', 'Cablaggio',
      'Il fascio di fili che collega candele, iniettori e sensori alla centralina elettronica.',
      'Cablaggio con connettori stagni verso la centralina (ECU), che a ogni giro legge giri, posizione farfalla, pressione e temperatura dell’aria, temperatura del liquido, marcia inserita, e decide quantità di benzina e anticipo per ogni cilindro.',
      [-0.6, 1.0, 0]);

    const sensors = new THREE.Group();
    const sensMat = cut(MAT.paint(0x3b3f46, 0.5));
    sensors.add(rbox(0.16, 0.12, 0.07, 0.02, sensMat, DRUM.x, DRUM.y, -1.76));
    sensors.add(tube([[DRUM.x, DRUM.y + 0.06, -1.8], [DRUM.x - 0.4, DRUM.y + 0.4, -1.86], [-1.4, 1.0, -1.9]], 0.014, wireMat, 12, 6));
    const ect = mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.1, 8), MAT.brass(), thermo.x - 0.07, thermo.y + 0.08, thermo.z + 0.1);
    sensors.add(ect);
    sensors.add(rbox(0.07, 0.07, 0.1, 0.015, sensMat, 0.25, 0.3, 2.47));
    sensors.add(tube([[0.25, 0.34, 2.5], [-0.1, 0.55, 2.49], [-0.45, 0.95, 2.32], [-0.62, 1.06, 1.9], [-0.55, 1.6, 1.72], [-0.35, 2.06, 1.65]], 0.014, wireMat, 24, 6));
    P(root, sensors, 'sensors', 'Sensori',
      'Piccoli “occhi” del motore: dicono alla centralina quanti giri fa, che marcia è inserita, quanto è caldo il motore.',
      'Sensore di posizione albero (pick‑up induttivo sulla ruota foronica, coperchio accensione), sensore di marcia inserita sul tamburo selettore (serve anche al quickshifter e alle mappe per marcia), sensore temperatura liquido sul termostato. Il sensore marcia permette di mostrare il numero sul cruscotto.',
      [0, 0, 0.6]);

    // ---------- Gearbox: main shaft, countershaft, shift drum ----------
    const gearMat = MAT.steel();
    const STEEL_HEX = gearMat.color.getHex();
    const gm = {}; // gear id -> meshes (for engaged highlight)
    const reg = (id, m) => ((gm[id] ||= []).push(m), m);
    const rM = (g) => (GEAR_D * GEARS[g][0]) / (GEARS[g][0] + GEARS[g][1]);
    const rC = (g) => (GEAR_D * GEARS[g][1]) / (GEARS[g][0] + GEARS[g][1]);
    const rootR = (r, n) => r - 1.25 * ((2 * r) / n);
    const dogs = (r, z, dir, mat) => {
      const g = new THREE.Group();
      for (let i = 0; i < 4; i++) {
        const a = (i / 4) * TAU + 0.4;
        const d = mesh(new THREE.BoxGeometry(0.06, 0.05, 0.032), mat, r * Math.cos(a), r * Math.sin(a), z + dir * 0.016);
        d.rotation.z = a;
        g.add(d);
      }
      return g;
    };
    // Slider gear: two toothed halves + a central groove for the fork.
    const slider = (id, n, r, withDogs) => {
      const g = new THREE.Group();
      const half = gearGeo(n, r, 0.03, 0.055);
      reg(id, g.add(mesh(half, gearMat, 0, 0, -0.0275)).children.at(-1));
      reg(id, g.add(mesh(half, gearMat, 0, 0, 0.0275)).children.at(-1));
      const rf = rootR(r, n);
      g.add(mesh(cylZ(rf - 0.06, 0.03, 32), darkSteel));
      if (withDogs) g.add(dogs(rf * 0.62, -GW / 2, -1, darkSteel), dogs(rf * 0.62, GW / 2, 1, darkSteel));
      return g;
    };

    const mainPart = new THREE.Group();
    mainPart.position.set(MAIN.x, MAIN.y, 0);
    const mainRot = new THREE.Group();
    mainPart.add(mainRot);
    mainRot.add(mesh(cylZ(0.055, 4.1, 16), steel, 0, 0, 0.3));
    mainRot.add(reg('M1', mesh(gearGeo(GEARS[1][0], rM(1), GW, 0.05), gearMat, 0, 0, PAIR_Z[1])));
    mainRot.add(reg('M2', mesh(gearGeo(GEARS[2][0], rM(2), GW, 0.05), gearMat, 0, 0, PAIR_Z[2])));
    const mSlider = new THREE.Group();
    const M_MID = (PAIR_Z[3] + PAIR_Z[4]) / 2;
    mSlider.add(reg('M3', mesh(gearGeo(GEARS[3][0], rM(3), GW, 0.05), gearMat, 0, 0, PAIR_Z[3] - M_MID)));
    mSlider.add(reg('M4', mesh(gearGeo(GEARS[4][0], rM(4), GW, 0.05), gearMat, 0, 0, PAIR_Z[4] - M_MID)));
    mSlider.add(mesh(cylZ(0.17, 0.07, 28), darkSteel));
    mSlider.add(dogs(0.16, PAIR_Z[3] - M_MID - GW / 2, -1, darkSteel), dogs(0.18, PAIR_Z[4] - M_MID + GW / 2, 1, darkSteel));
    mainRot.add(mSlider);
    const freeM = {};
    for (const g of [5, 6]) {
      freeM[g] = reg('M' + g, mesh(gearGeo(GEARS[g][0], rM(g), GW, 0.06), gearMat, 0, 0, PAIR_Z[g]));
      mainPart.add(freeM[g]);
    }
    for (const z of [-1.69, 1.79]) mainPart.add(mesh(zAxis(pipe(0.11, 0.056, 0.06, 24)), bronze, 0, 0, z));
    P(root, mainPart, 'mainShaft', 'Albero primario del cambio',
      'L’albero che riceve il moto dalla frizione. Porta sei ingranaggi, uno per marcia: alcuni sono fissi, altri girano liberi o scorrono di lato.',
      'Albero d’entrata del cambio a cassetta: ingranaggi di 1ª e 2ª solidali, 3ª‑4ª in un unico ingranaggio scorrevole scanalato (sposta per innestare 5ª o 6ª), 5ª e 6ª folli su boccole. Tutti gli ingranaggi sono sempre in presa con quelli del secondario; denti dritti per ridurre gli attriti e le spinte assiali.',
      [0, 0.35, 1.3]);

    const cntrPart = new THREE.Group();
    cntrPart.position.set(CNTR.x, CNTR.y, 0);
    const cRot = new THREE.Group();
    cntrPart.add(cRot);
    cRot.add(mesh(cylZ(0.06, 2.0, 16), steel, 0, 0, -0.9));
    const c5 = slider('C5', GEARS[5][1], rC(5), true);
    const c6 = slider('C6', GEARS[6][1], rC(6), true);
    cRot.add(c5, c6);
    const freeC = {};
    for (const g of [1, 3, 4, 2]) {
      freeC[g] = reg('C' + g, mesh(gearGeo(GEARS[g][1], rC(g), GW, 0.07, g === 1 ? 6 : 5), gearMat, 0, 0, PAIR_Z[g]));
      cntrPart.add(freeC[g]);
    }
    for (const z of [-1.69, 0.12]) cntrPart.add(mesh(zAxis(pipe(0.12, 0.061, 0.06, 24)), bronze, 0, 0, z));
    P(root, cntrPart, 'countershaft', 'Albero secondario del cambio',
      'Il secondo albero del cambio: i suoi ingranaggi sono accoppiati a quelli del primario. Quello agganciato decide la marcia e fa girare il pignone della catena.',
      'Albero d’uscita: ingranaggi folli di 1ª, 2ª, 3ª e 4ª e due scorrevoli (5ª e 6ª) con innesti frontali (dog) da entrambi i lati. Rapporti tipici di una 600: 1ª ≈ 2,6:1 … 6ª ≈ 1,2:1, ravvicinati per restare sempre nella zona di potenza (12 000–15 000 giri). L’ingranaggio colorato è la coppia in presa.',
      [-0.2, -0.3, 1.6]);

    // Shift drum, forks and shift shaft
    const drumPart = new THREE.Group();
    const drumG = new THREE.Group();
    drumG.position.set(DRUM.x, DRUM.y, 0);
    drumPart.add(drumG);
    drumG.add(mesh(cylZ(0.13, 1.0, 32), MAT.aluminum(), 0, 0, -0.05));
    drumG.add(mesh(cylZ(0.03, 1.2, 10), steel, 0, 0, -1.15));
    const starPts = [];
    for (let i = 0; i < 14; i++) {
      const a = (i / 14) * TAU;
      const r = i % 2 ? 0.1 : 0.17;
      starPts.push([r * Math.cos(a), r * Math.sin(a)]);
    }
    drumG.add(mesh(extZ(poly(starPts), 0.46, 0.5, 4), darkSteel));
    const trackOffset = (idx, f) => {
      idx = ((idx % N_IDX) + N_IDX) % N_IDX;
      if (idx >= 6) return lerp(TRACK[6][f], TRACK[0][f], smooth(clamp((idx - 6 - 0.25) / 0.7, 0, 1)));
      const i = Math.floor(idx);
      return lerp(TRACK[i][f], TRACK[i + 1][f], smooth(clamp((idx - i - 0.25) / 0.5, 0, 1)));
    };
    // forks: [track index, shaft centre, neutral z, tip inner/outer radius]
    const FORKS = [
      [0, CNTR, PAIR_Z[5], rootR(rC(5), GEARS[5][1]) - 0.055, rootR(rC(5), GEARS[5][1]) - 0.008],
      [1, CNTR, PAIR_Z[6], rootR(rC(6), GEARS[6][1]) - 0.055, rootR(rC(6), GEARS[6][1]) - 0.008],
      [2, MAIN, M_MID, 0.176, 0.225],
    ];
    const forkMat = MAT.brass();
    forkMat.color.set(0xa98a4a);
    const forks = FORKS.map(([f, S, z0, ri, ro]) => {
      const g = new THREE.Group();
      const toD = new THREE.Vector2().subVectors(DRUM, S);
      const ang = Math.atan2(toD.y, toD.x);
      const tip = new THREE.Shape();
      tip.absarc(S.x, S.y, ro, ang - Math.PI / 2, ang + Math.PI / 2, false);
      tip.absarc(S.x, S.y, ri, ang + Math.PI / 2, ang - Math.PI / 2, true);
      g.add(mesh(extZ(tip, -0.011, 0.011, 16), forkMat));
      const a = S.clone().addScaledVector(toD.clone().normalize(), ro - 0.02);
      const b = DRUM.clone().addScaledVector(toD.clone().normalize(), -0.16);
      const len = a.distanceTo(b);
      const arm = mesh(new THREE.BoxGeometry(len, 0.06, 0.022), forkMat, (a.x + b.x) / 2, (a.y + b.y) / 2, 0);
      arm.rotation.z = ang;
      g.add(arm);
      g.add(mesh(zAxis(pipe(0.18, 0.133, 0.07, 28)), forkMat, DRUM.x, DRUM.y, 0));
      g.userData = { f, z0 };
      drumPart.add(g);
      // track on the drum surface, followed by the fork pin
      const psi = Math.atan2(S.y - DRUM.y, S.x - DRUM.x);
      const pts = [];
      for (let i = 0; i < 90; i++) {
        const al = (i / 90) * TAU;
        const idx = (psi - al) / STEP;
        pts.push(new THREE.Vector3(0.132 * Math.cos(al), 0.132 * Math.sin(al), z0 + SHIFT * trackOffset(idx, f)));
      }
      drumG.add(new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, true), 120, 0.016, 5, true), darkSteel));
      return g;
    });
    const shiftShaft = new THREE.Group();
    shiftShaft.position.set(-0.45, -0.75, 0);
    shiftShaft.add(mesh(cylZ(0.035, 2.6, 12), steel, 0, 0, -0.75));
    const toStar = new THREE.Vector2(DRUM.x + 0.45, DRUM.y + 0.75);
    const sArm = mesh(new THREE.BoxGeometry(toStar.length(), 0.05, 0.03), darkSteel, toStar.x / 2, toStar.y / 2, 0.52);
    sArm.rotation.z = Math.atan2(toStar.y, toStar.x);
    shiftShaft.add(sArm);
    const lever = mesh(new THREE.BoxGeometry(0.3, 0.05, 0.035), darkSteel, -0.13, -0.08, -2.02);
    lever.rotation.z = 0.5;
    shiftShaft.add(lever);
    drumPart.add(shiftShaft);
    P(root, drumPart, 'shiftDrum', 'Tamburo selettore e forcelle',
      'Un cilindro con tre piste a zig‑zag. Ogni colpo di pedale lo fa ruotare di uno scatto: le piste spostano le forcelle, che fanno scorrere un ingranaggio e innestano la marcia.',
      'Tamburo con tre piste (camme cilindriche) che guidano i perni di tre forcelle: una sul primario (5ª/6ª), due sul secondario (1ª/3ª e 2ª/4ª). Una stella di arresto con leva a molla lo tiene fermo su 7 posizioni (1‑N‑2‑3‑4‑5‑6); l’albero del selettore, mosso dal pedale, lo fa avanzare di un passo con un nottolino. Per questo il cambio è sequenziale: non si può saltare una marcia.',
      [0, -0.9, 1.0]);

    // ---------- Primary drive and wet clutch ----------
    const rP = (PRI_D * NP) / (NP + NB), rB = (PRI_D * NB) / (NP + NB);
    const primary = new THREE.Group();
    const primGear = mesh(gearGeo(NP, rP, 0.08, 0.12, 0), steel, 0, 0, 1.91);
    primary.add(primGear);
    P(root, primary, 'primaryGear', 'Trasmissione primaria',
      'Un ingranaggio sull’albero motore che fa girare il cestello della frizione, a metà velocità.',
      'Coppia di ingranaggi a denti dritti (o elicoidali) con rapporto ≈ 2:1: riduce i giri e moltiplica la coppia prima della frizione, così frizione e cambio lavorano a regimi più bassi. Spesso l’ingranaggio condotto è “sdoppiato” con molle per annullare il gioco e il rumore.',
      [0.4, -0.4, 1.0]);

    const basket = new THREE.Group();
    basket.position.set(MAIN.x, MAIN.y, 0);
    const basketRot = new THREE.Group();
    basket.add(basketRot);
    basketRot.add(mesh(gearGeo(NB, rB, 0.08, 0.2, 6), darkSteel, 0, 0, 1.91));
    basketRot.add(mesh(cylZ(0.62, 0.03, 48), MAT.aluminum(), 0, 0, 1.965));
    for (let i = 0; i < 8; i++) {
      const a = (i / 8) * TAU;
      const f = mesh(new THREE.BoxGeometry(0.04, 0.13, 0.3), MAT.aluminum(), 0.6 * Math.cos(a), 0.6 * Math.sin(a), 2.12);
      f.rotation.z = a;
      basketRot.add(f);
    }
    for (let i = 0; i < 6; i++) basketRot.add(mesh(cylZ(0.035, 0.03, 10), steel, 0.4 * Math.cos(i), 0.4 * Math.sin(i), 1.94));
    P(root, basket, 'clutchBasket', 'Campana della frizione',
      'Il “cestello” della frizione: ha attorno la corona dentata mossa dal motore e delle dita in cui si incastrano i dischi.',
      'Campana in alluminio con la corona della primaria e parastrappi in gomma o molle; gira folle sull’albero primario. Le linguette dei dischi d’attrito si impegnano nelle sue finestre. Dietro, spesso, il pignone della catena che aziona la pompa dell’olio.',
      [0, 0, 1.0]);

    const fricShape = (() => {
      const pts = [];
      for (let i = 0; i < 8; i++) {
        const a0 = (i / 8) * TAU + TAU / 16;
        for (let j = 0; j <= 6; j++) {
          const a = a0 - TAU / 16 + (j / 6) * (TAU / 8);
          const tab = Math.abs(a - a0) < 0.12 ? 0.585 : 0.555;
          pts.push([tab * Math.cos(a), tab * Math.sin(a)]);
        }
      }
      const s = poly(pts);
      s.holes.push(circ(0, 0, 0.4));
      return s;
    })();
    const fricGeo = extZ(fricShape, -0.008, 0.008, 24);
    const steelPlateGeo = (() => {
      const s = poly(circlePts(0, 0, 0.545, 48));
      s.holes.push(circ(0, 0, 0.33));
      return extZ(s, -0.006, 0.006, 24);
    })();
    const plates = new THREE.Group();
    plates.position.set(MAIN.x, MAIN.y, 0);
    const platesRot = new THREE.Group();
    plates.add(platesRot);
    const plateList = [];
    const fricMat = MAT.paint(0x7a4d26, 0.85);
    let pz = 1.995;
    for (let i = 0; i < 15; i++) {
      const fr_ = i % 2 === 0;
      const t = fr_ ? 0.016 : 0.012;
      const m = mesh(fr_ ? fricGeo : steelPlateGeo, fr_ ? fricMat : steel, 0, 0, pz + t / 2);
      platesRot.add(m);
      plateList.push({ m, z: pz + t / 2, i });
      pz += t + 0.003;
    }
    P(root, plates, 'clutchPlates', 'Dischi della frizione',
      'Una pila di dischi: quelli marroni (d’attrito) girano con il cestello, quelli d’acciaio con l’albero del cambio. Stretti insieme trascinano il cambio; separati lo lasciano libero.',
      'Pacco di 8 dischi d’attrito e 7 dischi d’acciaio, immersi nell’olio che li raffredda e rende l’innesto progressivo. Molti dischi di diametro piccolo trasmettono oltre 60 Nm di coppia motore moltiplicata dalla primaria; la frizione antisaltellamento (slipper) li fa slittare in scalata per non bloccare la ruota posteriore.',
      [0, 0, 1.6]);

    const pressure = new THREE.Group();
    pressure.position.set(MAIN.x, MAIN.y, 0);
    const pressureRot = new THREE.Group();
    pressure.add(pressureRot);
    pressureRot.add(mesh(cylZ(0.31, 0.27, 32), MAT.aluminum(), 0, 0, 2.12));
    const pPlate = new THREE.Group();
    pPlate.add(mesh(cylZ(0.53, 0.035, 48), MAT.aluminum(), 0, 0, 2.29));
    for (let i = 0; i < 5; i++) {
      const a = (i / 5) * TAU;
      pPlate.add(mesh(cylZ(0.06, 0.08, 16), MAT.aluminum(), 0.22 * Math.cos(a), 0.22 * Math.sin(a), 2.34));
      pPlate.add(mesh(cylZ(0.035, 0.03, 6), MAT.chrome(), 0.22 * Math.cos(a), 0.22 * Math.sin(a), 2.39));
    }
    pPlate.add(mesh(cylZ(0.07, 0.06, 6), MAT.chrome(), 0, 0, 2.34));
    pressureRot.add(pPlate);
    P(root, pressure, 'pressurePlate', 'Spingidisco e molle della frizione',
      'Il disco esterno che, spinto dalle molle, stringe tutti i dischi. Quando tiri la leva, si allontana e la frizione “stacca”.',
      'Spingidisco con 5 molle elicoidali (o una molla a tazza) e mozzo interno scanalato sul primario. La leva al manubrio, via cavo o comando idraulico, spinge un’astina che passa dentro l’albero primario e solleva lo spingidisco di 1–2 mm. Nell’animazione si solleva a ogni cambiata.',
      [0, 0, 2.1]);

    // Clutch cover with ignition cover
    const coverOut = convexHull([...circlePts(MAIN.x, MAIN.y, 0.8, 40), ...circlePts(0, 0, 0.46, 30)]);
    const coverIn = convexHull([...circlePts(MAIN.x, MAIN.y, 0.75, 40), ...circlePts(0, 0, 0.41, 30)]);
    const clutchCover = new THREE.Group();
    const rimShape = poly(coverOut);
    rimShape.holes.push(poly(coverIn, THREE.Path));
    clutchCover.add(mesh(extZ(rimShape, 1.82, 2.4, 4), coverMat));
    clutchCover.add(mesh(extZ(poly(coverOut), 2.4, 2.47, 4, 0.025), coverMat));
    clutchCover.add(mesh(new THREE.TorusGeometry(0.56, 0.025, 8, 64), coverMat, MAIN.x, MAIN.y, 2.47));
    clutchCover.add(mesh(cylZ(0.12, 0.05, 32), coverMat, MAIN.x, MAIN.y, 2.49));
    const ccb = [];
    const per = coverOut.length;
    for (let i = 0; i < per; i += Math.max(1, Math.floor(per / 14))) {
      const [x, y] = coverOut[i];
      const cx = x * 0.94 + (x > -0.4 ? 0 : MAIN.x) * 0.06, cy = y * 0.94 + (x > -0.4 ? 0 : MAIN.y) * 0.06;
      ccb.push([cx, cy, 2.48]);
    }
    clutchCover.add(bolts(ccb, 'z', boltMat, 0.026, 0.03));
    clutchCover.add(mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.07, 16), cut(MAT.paint(0x15171a, 0.5)), MAIN.x + 0.15, MAIN.y + 0.82, 2.12));
    clutchCover.add(mesh(cylZ(0.1, 0.02, 24), cut(MAT.chrome()), MAIN.x + 0.3, MAIN.y - 0.5, 2.48));
    clutchCover.add(mesh(cylZ(0.075, 0.025, 24), cut(MAT.gas(0xd9a030, 0.85, false)), MAIN.x + 0.3, MAIN.y - 0.5, 2.485));
    // generic nameplate
    const cnv = document.createElement('canvas');
    cnv.width = 256;
    cnv.height = 96;
    const g2 = cnv.getContext('2d');
    g2.fillStyle = '#2a2d33';
    g2.fillRect(0, 0, 256, 96);
    g2.strokeStyle = '#c9a24a';
    g2.lineWidth = 6;
    g2.strokeRect(6, 6, 244, 84);
    g2.fillStyle = '#e6e9ee';
    g2.font = 'bold 40px sans-serif';
    g2.textAlign = 'center';
    g2.fillText('600 · 16V', 128, 52);
    g2.font = '20px sans-serif';
    g2.fillText('DOHC  6‑SPEED', 128, 80);
    const tex = new THREE.CanvasTexture(cnv);
    tex.colorSpace = THREE.SRGBColorSpace;
    const plate = mesh(new THREE.PlaneGeometry(0.42, 0.16), cut(new THREE.MeshStandardMaterial({ map: tex, metalness: 0.4, roughness: 0.4 }), { cap: false }), MAIN.x, MAIN.y + 0.22, 2.522);
    clutchCover.add(plate);
    P(root, clutchCover, 'clutchCover', 'Coperchio frizione',
      'Il grande coperchio sul lato destro: tolto questo si vedono la frizione e gli ingranaggi della primaria. In alto il tappo per l’olio, in basso l’oblò per controllarne il livello.',
      'Coperchio in alluminio o magnesio con guarnizione, tappo di riempimento e spia di livello dell’olio. Integra il comando di disinnesto della frizione e, davanti, il coperchio del sensore di giri. Smontabile senza aprire il carter: frizione e primaria sono accessibili per la manutenzione.',
      [0, 0, 2.6]);

    const ign = new THREE.Group();
    ign.add(mesh(lathe([[0, 0], [0.28, 0], [0.28, 0.04], [0.22, 0.08], [0, 0.085]], 40).rotateX(Math.PI / 2), coverMat, 0, 0, 2.47));
    const ib = [];
    for (let i = 0; i < 5; i++) ib.push([0.245 * Math.cos(i * 1.257 + 0.3), 0.245 * Math.sin(i * 1.257 + 0.3), 2.53]);
    ign.add(bolts(ib, 'z', boltMat, 0.02, 0.02));
    P(root, ign, 'ignitionCover', 'Coperchio del sensore di giri',
      'Un piccolo coperchio sull’estremità dell’albero motore: sotto c’è la ruota dentata che il sensore “conta” per sapere giri e posizione dei pistoni.',
      'Sotto il coperchio la ruota foronica (rotore pick‑up) calettata sull’albero motore; il sensore induttivo genera un impulso a ogni dente e la centralina ricava giri e angolo di manovella per fasare accensione e iniezione.',
      [0.3, 0, 2.2]);

    // ---------- Output sprocket and final drive chain ----------
    const sprocketPart = new THREE.Group();
    sprocketPart.position.set(CNTR.x, CNTR.y, 0);
    const sprocketRot = new THREE.Group();
    sprocketPart.add(sprocketRot);
    sprocketRot.add(mesh(gearGeo(16, 0.405, 0.055, 0.08, 4), darkSteel, 0, 0, -1.86));
    sprocketRot.add(mesh(cylZ(0.13, 0.02, 24), steel, 0, 0, -1.9));
    sprocketRot.add(bolts([[0.08, 0, -1.92], [-0.08, 0, -1.92]], 'z', MAT.chrome(), 0.022, 0.02));
    const RS = 0.405;
    const chainPath = (s, out) => {
      const straight = 1.6;
      const arc = Math.PI * RS;
      const total = 2 * straight + arc;
      s = ((s % total) + total) % total;
      if (s < straight) return out.set(-straight + s, RS, 0);
      if (s < straight + arc) {
        const a = Math.PI / 2 - (s - straight) / RS;
        return out.set(RS * Math.cos(a), RS * Math.sin(a), a - Math.PI / 2);
      }
      return out.set(-(s - straight - arc), -RS, Math.PI);
    };
    const CHAIN_TOTAL = 3.2 + Math.PI * RS;
    const DPITCH = 0.159;
    const NDL = Math.floor(CHAIN_TOTAL / DPITCH);
    const driveLinks = new THREE.InstancedMesh(new THREE.BoxGeometry(0.15, 0.06, 0.09), darkSteel, NDL);
    const driveRollers = new THREE.InstancedMesh(cylZ(0.032, 0.1, 10), MAT.chrome(), NDL);
    for (const m of [driveLinks, driveRollers]) {
      m.frustumCulled = false;
      m.position.z = -1.86;
      sprocketPart.add(m);
    }
    P(root, sprocketPart, 'sprocket', 'Pignone e catena di trasmissione',
      'Il pignone dentato all’uscita del cambio, sul lato sinistro, tira la catena che fa girare la ruota posteriore (qui ne vedi solo un pezzo).',
      'Pignone da ≈ 15–16 denti sull’albero secondario, catena O‑ring o X‑ring passo 520–525 e corona posteriore da ≈ 40–45 denti: rapporto finale ≈ 2,7:1. Cambiando di un dente il pignone si modifica sensibilmente la spinta o la velocità massima.',
      [0, 0, -0.9]);

    // ---------- Flows ----------
    const V = (p) => (p.isVector3 ? p : new THREE.Vector3(...p));
    const curveOf = (pts) => new THREE.CatmullRomCurve3(pts.map(V), false, 'centripetal');
    const intakeFlows = CYL_Z.map((c) => {
      const st = tbAt(0, 0.55), tb = tbAt(0, 0.12);
      const crv = curveOf([[1.3, 2.78, 0.1], [0.6, 2.7, 0.1 + c * 0.3], [-0.15, 2.56, c * 0.8], [st.x, st.y, c], [tb.x, tb.y, c], [TB_B.x, TB_B.y - 0.02, c], [portA[0], portA[1], c], [...fr(-0.38, 1.6), c], [...fr(-0.18, 1.48), c], [...fr(-0.12, 1.41), c]]);
      const f = flow({ path: (t, o) => crv.getPointAt(t, o), count: 55, size: 0.07, colors: [0x5fd3ff, 0x5fd3ff, 0x5fd3ff, 0xffc04d], spread: 0.045, speed: 0.8 });
      root.add(f.object);
      return f;
    });
    const exhaustFlows = CYL_Z.map((c, k) => {
      const outer = k === 0 || k === 3;
      const crv = curveOf([[...fr(0.12, 1.41), c], [...fr(0.3, 1.52), c], ...headerPts[k], ...(outer ? S14 : S23).slice(1), COLL[1], COLL[2]]);
      const f = flow({ path: (t, o) => crv.getPointAt(t, o), count: 70, size: 0.08, colors: [0xff5a1f, 0xb07a6a, 0x8c86a8], spread: 0.04, speed: 0.55 });
      root.add(f.object);
      return f;
    });
    const coolCurve = curveOf([[1.35, -0.2, -1.7], [0.9, -0.6, -1.85], [0.2, -0.75, -1.9], [-0.55, -0.5, -1.98], [-0.66, 0.45, -1.92], [-0.4, 0.97, -1.62], [...fr(-0.63, 0.85), -1.45], [...fr(-0.43, 1.0), -0.9], [...fr(-0.43, 1.2), 0.9], [...fr(0.3, 1.58), 0.9], [...fr(0.32, 1.58), -1.3], [thermo.x, thermo.y, -1.5], [1.85, 1.5, -1.55], [2.05, 1.5, -1.6]]);
    const coolFlow = flow({ path: (t, o) => coolCurve.getPointAt(t, o), count: 110, size: 0.06, colors: [0x5fd3ff, 0x5fd3ff, 0xff9a6a], spread: 0.035, speed: 0.12 });
    root.add(coolFlow.object);

    // ---------- Animation state ----------
    const C = {
      air: new THREE.Color(0x5fd3ff), mix: new THREE.Color(0x9ad8a0), fuel: new THREE.Color(0xffc04d),
      flash: new THREE.Color(0xfff2c4), fire: new THREE.Color(0xff5a1f), exh: new THREE.Color(0x8c86a8),
    };
    const cA = new THREE.Color();
    const O2 = new THREE.Vector2(0, 0);
    const dummy = new THREE.Object3D();
    const tmp = new THREE.Vector3();
    const tan = new THREE.Vector3();
    const animRate = (th) => 0.12 + th * 0.85;
    const sim = { prevCycle: null, crankAcc: 0, drumIdx: null, lift: 0, thc: 0, lastGear: -1, phaseK: 0, vel: 0, prevThm: 0 };
    const gearOf = (idx) => DRUM_ORDER[clamp(Math.round(idx), 0, 6)];

    return {
      root,
      cycleRate: animRate,
      update(state) {
        const dt = state.dt || 0;
        const th = state.throttle;
        const assembled = (state.explode || 0) < 0.05;
        // continuous crank angle (cycle wraps every 720°)
        if (sim.prevCycle === null) {
          sim.prevCycle = state.cycle;
          sim.crankAcc = state.cycle * 2 * TAU;
        }
        let dc = state.cycle - sim.prevCycle;
        if (dc > 0.5) dc -= 1;
        if (dc < -0.5) dc += 1;
        sim.prevCycle = state.cycle;
        sim.crankAcc += dc * 2 * TAU;
        const th1 = state.cycle * 2 * TAU;
        crank.rotation.z = -th1;
        camIn.rotation.z = camEx.rotation.z = -state.cycle * TAU;
        altRotor.rotation.z = -th1;

        CYL_Z.forEach((c, k) => {
          const local = wrap(state.cycle - OFFSET[k]);
          const a = local * 2 * TAU;
          const xs = R * Math.sin(a), yc = R * Math.cos(a);
          const yp = yc + Math.sqrt(L * L - xs * xs);
          pistons[k].position.y = yp;
          rods[k].position.set(xs, yc, c);
          rods[k].rotation.z = Math.atan2(xs, yp - yc);
          const li = lift(local, -0.03, 0.28);
          const le = lift(local, 0.72, 1.03);
          for (const j of [0, 1]) {
            const vi = inV.list[k * 2 + j], ve = exV.list[k * 2 + j];
            vi.mov.position.y = -li * 0.085;
            vi.spring.scale.y = (0.2 - li * 0.085) / 0.2;
            ve.mov.position.y = -le * 0.085;
            ve.spring.scale.y = (0.2 - le * 0.085) / 0.2;
          }
          const top = yp + 0.17;
          const roof = 1.39;
          const ch = charges[k];
          ch.position.y = (top + roof) / 2;
          ch.scale.y = Math.max(0.001, roof - top);
          let op;
          if (local < 0.25) { const f = local / 0.25; cA.copy(C.air).lerp(C.mix, f); op = 0.12 + 0.12 * f; }
          else if (local < 0.5) { const f = (local - 0.25) / 0.25; cA.copy(C.mix).lerp(C.fuel, f); op = 0.25 + 0.3 * f; }
          else if (local < 0.75) {
            const f = (local - 0.5) / 0.25;
            if (f < 0.2) cA.copy(C.flash).lerp(C.fire, f / 0.2);
            else cA.copy(C.fire).lerp(C.exh, (f - 0.2) / 0.8);
            op = lerp(0.95, 0.35, f);
          } else { const f = (local - 0.75) / 0.25; cA.copy(C.exh); op = lerp(0.35, 0.08, f); }
          ch.material.color.copy(cA);
          ch.material.opacity = op;
          ch.visible = assembled;
          const sparkOn = local > 0.485 && local < 0.53;
          sparks[k].visible = sparkOn && assembled;
          sparks[k].scale.setScalar(sparkOn ? 0.6 + Math.random() * 0.6 : 1);
          const flash = local > 0.49 && local < 0.62 ? 1 - (local - 0.49) / 0.13 : 0;
          sparkLights[k].intensity = assembled ? flash * (1.5 + th * 3) : 0;
          const rate = animRate(th);
          intakeFlows[k].object.visible = exhaustFlows[k].object.visible = assembled;
          intakeFlows[k].material.opacity = clamp(li * 1.6, 0, 0.9);
          intakeFlows[k].update(dt, li * rate * 1.6);
          exhaustFlows[k].material.opacity = clamp(le * 1.6, 0, 0.9);
          exhaustFlows[k].update(dt, le * rate * 1.6);
        });
        coolFlow.object.visible = assembled;
        coolFlow.update(dt, 0.4 + th * 1.5);
        butterflies.forEach((b) => (b.rotation.z = 0.08 + th * 1.35));

        // cam chain
        for (let i = 0; i < NLINK; i++) {
          const u = wrap(i / NLINK - (sim.crankAcc * 0.1) / chainLen);
          chainCurve.getPointAt(u, tmp);
          chainCurve.getTangentAt(u, tan);
          dummy.position.copy(tmp);
          dummy.rotation.set(0, 0, Math.atan2(tan.y, tan.x));
          dummy.updateMatrix();
          camChainLinks.setMatrixAt(i, dummy.matrix);
        }
        camChainLinks.instanceMatrix.needsUpdate = true;

        // primary drive and main shaft
        const thp = -TILT - sim.crankAcc;
        primGear.rotation.z = thp;
        const thm = meshAngle(thp, NP, NB, O2, MAIN);
        const dthm = thm - sim.prevThm;
        sim.prevThm = thm;
        mainRot.rotation.z = basketRot.rotation.z = platesRot.rotation.z = pressureRot.rotation.z = thm;

        // sequential shift: the drum walks one detent at a time towards the target gear
        const ds = driveState(th);
        const tIdx = DRUM_ORDER.indexOf(ds.gear);
        if (sim.drumIdx === null || dt === 0) sim.drumIdx = tIdx;
        else {
          const d = tIdx - sim.drumIdx;
          sim.drumIdx += Math.sign(d) * Math.min(Math.abs(d), dt * 3.2);
        }
        const moving = Math.abs(tIdx - sim.drumIdx) > 1e-3;
        sim.lift = dt === 0 ? 0 : lerp(sim.lift, moving ? 1 : 0, Math.min(1, dt * 14));
        const near = Math.round(sim.drumIdx);
        const engaged = Math.abs(sim.drumIdx - near) < 0.12 ? DRUM_ORDER[near] : 0;
        drumG.rotation.z = sim.drumIdx * STEP;
        const o = [0, 1, 2].map((f) => trackOffset(sim.drumIdx, f));
        c5.position.z = PAIR_Z[5] + SHIFT * o[0];
        c6.position.z = PAIR_Z[6] + SHIFT * o[1];
        mSlider.position.z = M_MID + SHIFT * o[2];
        forks.forEach((fk) => (fk.position.z = SHIFT * o[fk.userData.f]));
        const frac = sim.drumIdx - Math.floor(sim.drumIdx);
        shiftShaft.rotation.z = moving ? 0.35 * Math.sin(Math.PI * frac) * Math.sign(tIdx - sim.drumIdx) : 0;

        // countershaft: locked to the engaged pair, coasting while shifting, still in neutral
        if (engaged > 0) {
          const [nm, nc] = GEARS[engaged];
          const base = meshAngle(thm, nm, nc, MAIN, CNTR);
          if (sim.lastGear !== engaged) sim.phaseK = Math.round((sim.thc - base) / (TAU / nc));
          sim.thc = base + (sim.phaseK * TAU) / nc;
          sim.vel = -nm / nc;
        } else if (ds.gear === 0 && !moving) sim.vel = 0;
        else sim.thc += sim.vel * dthm;
        sim.lastGear = engaged;
        cRot.rotation.z = sim.thc;
        sprocketRot.rotation.z = sim.thc;
        for (const g of [1, 3, 4, 2]) freeC[g].rotation.z = meshAngle(thm, GEARS[g][0], GEARS[g][1], MAIN, CNTR);
        for (const g of [5, 6]) freeM[g].rotation.z = meshAngle(sim.thc, GEARS[g][1], GEARS[g][0], CNTR, MAIN);
        for (const id in gm) {
          const on = engaged > 0 && (id === 'M' + engaged || id === 'C' + engaged);
          for (const m of gm[id]) m.material.color.setHex(on ? 0xffb347 : STEEL_HEX);
        }
        // clutch lift while shifting
        pPlate.position.z = sim.lift * 0.05;
        plateList.forEach((p) => (p.m.position.z = p.z + sim.lift * 0.0025 * p.i));
        // final drive chain
        for (let i = 0; i < NDL; i++) {
          chainPath(i * DPITCH - sim.thc * RS, tmp);
          dummy.position.set(tmp.x, tmp.y, 0);
          dummy.rotation.set(0, 0, tmp.z);
          dummy.updateMatrix();
          driveLinks.setMatrixAt(i, dummy.matrix);
          chainPath(i * DPITCH + DPITCH / 2 - sim.thc * RS, tmp);
          dummy.position.set(tmp.x, tmp.y, 0);
          dummy.updateMatrix();
          driveRollers.setMatrixAt(i, dummy.matrix);
        }
        driveLinks.instanceMatrix.needsUpdate = true;
        driveRollers.instanceMatrix.needsUpdate = true;
      },
      readout(state) {
        const ds = driveState(state.throttle);
        const g = sim.drumIdx === null ? ds.gear : gearOf(sim.drumIdx);
        const kw = ds.gear === 0 ? 1.5 + state.throttle * 40 : (torqueAt(ds.rpm) * ds.rpm * TAU) / 60 / 1000;
        return [
          { label: 'Giri', value: Math.round(ds.rpm / 50) * 50, unit: 'giri/min' },
          { label: 'Marcia', value: g === 0 ? 'N' : String(g), unit: '' },
          { label: 'Velocità', value: Math.round(ds.kmh), unit: 'km/h' },
          { label: 'Potenza', value: Math.round(kw), unit: 'kW' },
        ];
      },
    };
  },
};
