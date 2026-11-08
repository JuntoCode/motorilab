// Air-cooled flat-four aviation engine (Lycoming O-360 class) with direct-drive propeller.
// Crankshaft along X (+x = front, propeller). Right bank (cyl 1, 3) at z > 0, left bank (2, 4) at z < 0.
// Lycoming layout: gear-driven camshaft ABOVE the crankshaft, hydraulic tappets, pushrods in shroud
// tubes above the cylinders, rockers under covers on top of the heads; intake and exhaust ports underneath.
// Scale ≈ 2.3 units per metre. Cycle 0..1 = 720° of crank for cylinder 1; firing order 1-3-2-4.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, pipe, flow, clamp, lerp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const CR = 0.128; // crank radius (stroke 111 mm)
const ROD = 0.4; // conrod length
const BORE = 0.15; // bore radius (130 mm)
const DECK = 0.3; // crankcase side faces (|z|)
const HEAD_Z = 0.64; // combustion chamber roof (|z|)
const CROWN = 0.075; // wrist pin to piston crown
const CAM_Y = 0.3; // camshaft, tappets and pushrods height (above the crank)
const ROCK_Y = 0.15; // rocker pivot height
const ROCK_Z = 0.885; // rocker pivot (|z|)
const CASE_Y = [-0.36, 0.52];
const PROP_X = 0.91;
const PROP_R = 2.15; // 1.88 m propeller
const PITCH = 3.4; // geometric pitch (≈ 1.5 m)
// s = bank side (+1 right, -1 left); off = cycle offset for firing order 1-3-2-4
const CYL = [
  { x: 0.38, s: 1, off: 0 },
  { x: 0.14, s: -1, off: 0.5 },
  { x: -0.1, s: 1, off: 0.25 },
  { x: -0.34, s: -1, off: 0.75 },
];
// crank throw angle of each cylinder (right bank TDC with pin at +z, left bank with pin at -z)
CYL.forEach((c) => (c.throw = (((-c.off * 2 * TAU + (c.s < 0 ? Math.PI : 0)) % TAU) + TAU) % TAU));
const IN_WIN = [-0.03, 0.28];
const EX_WIN = [0.72, 1.03];

const wrap = (c) => ((c % 1) + 1) % 1;
function lift(c, a, b) {
  const x = wrap(c - a);
  const w = b - a;
  return x < w ? Math.sin((Math.PI * x) / w) : 0;
}
const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
const tube = (pts, r, mat, seg = 48, closed = false, radial = 10) =>
  new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts, closed), seg, r, radial, closed), mat);
const cylX = (r, len, seg = 32) => new THREE.CylinderGeometry(r, r, len, seg).rotateZ(Math.PI / 2);
const cylZ = (r, len, seg = 32) => new THREE.CylinderGeometry(r, r, len, seg).rotateX(Math.PI / 2);
const rbox = (w, h, d, r = 0.015) => new RoundedBoxGeometry(w, h, d, 2, r);
const Y_UP = v3(0, 1, 0);

// Many small identical parts (bolts, nuts) as one InstancedMesh; items = [{ p, n }] position + axis.
function instanced(geo, mat, items) {
  const m = new THREE.InstancedMesh(geo, mat, items.length);
  const q = new THREE.Quaternion();
  const one = v3(1, 1, 1);
  const mx = new THREE.Matrix4();
  items.forEach((it, i) => {
    q.setFromUnitVectors(Y_UP, it.n.clone().normalize());
    m.setMatrixAt(i, mx.compose(it.p, q, one));
  });
  m.instanceMatrix.needsUpdate = true;
  return m;
}

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
// Spur gear in the Y-Z plane (axis X), centred on its mid-thickness. Shape angle a -> (y = sin a, z = -cos a).
function gearX(rPitch, teeth, amp, phase, width, bore) {
  const s = new THREE.Shape(toothed(rPitch, teeth, amp, phase, teeth * 10));
  const h = new THREE.Path();
  h.absellipse(0, 0, bore, bore, 0, TAU, true);
  s.holes.push(h);
  const g = new THREE.ExtrudeGeometry(s, { depth: width, bevelEnabled: false });
  g.translate(0, 0, -width / 2);
  g.rotateY(Math.PI / 2);
  return g;
}
function roundedRect(x0, y0, x1, y1, r) {
  const s = new THREE.Shape();
  s.moveTo(x0 + r, y0);
  s.lineTo(x1 - r, y0);
  s.quadraticCurveTo(x1, y0, x1, y0 + r);
  s.lineTo(x1, y1 - r);
  s.quadraticCurveTo(x1, y1, x1 - r, y1);
  s.lineTo(x0 + r, y1);
  s.quadraticCurveTo(x0, y1, x0, y1 - r);
  s.lineTo(x0, y0 + r);
  s.quadraticCurveTo(x0, y0, x0 + r, y0);
  return s;
}
const hole = (x, y, r) => {
  const h = new THREE.Path();
  h.absellipse(x, y, r, r, 0, TAU, true);
  return h;
};

// Cam lobe in the Y-Z plane; nose along +Z at rotation 0.
function camLobeGeometry() {
  const s = new THREE.Shape();
  const N = 48;
  for (let i = 0; i <= N; i++) {
    const a = (i / N) * TAU;
    const r = 0.045 + 0.03 * Math.pow(Math.max(0, -Math.cos(a)), 3);
    i === 0 ? s.moveTo(r * Math.cos(a), r * Math.sin(a)) : s.lineTo(r * Math.cos(a), r * Math.sin(a));
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: 0.05, bevelEnabled: false });
  g.translate(0, 0, -0.025);
  g.rotateY(Math.PI / 2); // u -> -Z, so the nose (u < 0) points +Z
  return g;
}

// Twisted, tapered propeller blade along +Y between radii r0..r1 (pitch from geometric pitch).
function bladeGeometry(r0, r1, stations) {
  const M = 16;
  const pos = [];
  const idx = [];
  const chordAt = (f) => {
    let c = f < 0.35 ? lerp(0.15, 0.27, f / 0.35) : lerp(0.27, 0.15, (f - 0.35) / 0.65);
    if (f > 0.93) c *= Math.sqrt(Math.max(0.02, 1 - Math.pow((f - 0.93) / 0.07, 2)));
    return c;
  };
  for (let j = 0; j <= stations; j++) {
    const r = r0 + ((r1 - r0) * j) / stations;
    const f = clamp((r - 0.12) / (PROP_R - 0.12), 0, 1);
    const c = chordAt(f);
    const t = lerp(0.07, 0.016, f) * (c / 0.2);
    const beta = Math.min(1.1, Math.atan(PITCH / (TAU * r)));
    const cb = Math.cos(beta);
    const sb = Math.sin(beta);
    for (let i = 0; i < M; i++) {
      const a = (i / M) * TAU;
      const u = (c / 2) * Math.cos(a);
      const v = (t / 2) * Math.sin(a) + 0.035 * c * (1 - Math.pow((2 * u) / c, 2)); // cambered lens
      pos.push(u * sb + v * cb, r, u * cb - v * sb);
    }
  }
  for (let j = 0; j < stations; j++) {
    for (let i = 0; i < M; i++) {
      const a = j * M + i;
      const b = j * M + ((i + 1) % M);
      idx.push(a, a + M, b, b, a + M, b + M);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// Generic data plate (no real brand).
function nameplate() {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 96;
  const g = c.getContext('2d');
  g.fillStyle = '#c9ccd0';
  g.fillRect(0, 0, 256, 96);
  g.strokeStyle = '#3a3f46';
  g.lineWidth = 4;
  g.strokeRect(4, 4, 248, 88);
  g.fillStyle = '#1d2229';
  g.font = 'bold 26px sans-serif';
  g.fillText('MOTORE AVIO', 22, 38);
  g.font = '18px sans-serif';
  g.fillText('4 CIL. OPPOSTI · 5,9 L', 22, 62);
  g.fillText('134 kW · 2700 giri/min', 22, 84);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return new THREE.MeshStandardMaterial({ map: tex, metalness: 0.5, roughness: 0.45 });
}

export default {
  id: 'aeroPiston',
  name: 'Motore a pistoni aeronautico',
  short: 'Pistoni aeronautico',
  family: 'aria',
  vehicle: 'Aereo da turismo',
  examples: 'Cessna 172, Piper PA-28, Robin DR400 (Lycoming O-360); Tecnam e ultraleggeri (Rotax 912)',
  tagline: 'Quattro cilindri sdraiati, raffreddati dall’aria dell’elica, con tutto doppio per non fermarsi mai in volo.',
  intro: {
    base:
      'È un parente del motore dell’auto, ma pensato per volare: quattro cilindri <b>sdraiati</b> e contrapposti (boxer), due per lato, che girano lenti e fanno ruotare <b>direttamente l’elica</b>. Non c’è radiatore: l’aria spinta dall’elica passa tra le <b>alette</b> dei cilindri e li raffredda. Ogni cilindro ha <b>due candele</b> con due impianti separati: se uno si guasta, il motore continua a girare.',
    expert:
      'Motore alternativo a ciclo Otto, 4 cilindri orizzontali contrapposti, raffreddato ad aria (es. Lycoming O-360: alesaggio 130 mm, corsa 111 mm, 5,9 L, 134 kW a 2 700 giri/min). Distribuzione ad <b>aste e bilancieri</b>: albero a camme sopra l’albero motore, comandato da ingranaggi, punterie idrauliche, aste nei tubi di protezione sopra i cilindri e bilancieri sotto i coperchi delle testate. Accensione doppia a magneti, indipendente dall’impianto elettrico. Presa diretta dell’elica per tenere le punte delle pale sotto Mach ~0,8; ordine di accensione 1‑3‑2‑4.',
  },
  specs: [
    { label: 'Carburante', value: 'Avgas 100LL (con piombo)' },
    { label: 'Ciclo', value: 'Otto, 4 tempi, boxer 4 cilindri' },
    { label: 'Cilindrata', value: '5,9 litri (O-360)' },
    { label: 'Potenza', value: '≈ 134 kW (180 CV) a 2 700 giri/min' },
    { label: 'Distribuzione', value: 'Ingranaggi + aste e bilancieri (OHV)' },
    { label: 'Raffreddamento', value: 'Ad aria, con deflettori' },
    { label: 'Accensione', value: 'Doppia: 2 magneti, 8 candele' },
    { label: 'Rendimento', value: '25 – 30 %' },
  ],
  metrics: { efficiency: 28, power: 134, powerDensity: 1.0 },
  variants: {
    columns: ['Motore', 'Cilindrata', 'Potenza', 'Distribuzione'],
    rows: [
      ['Lycoming O-320', '5,2 l', '112–119 kW (150–160 CV)', 'Ingranaggi + aste e bilancieri'],
      ['Lycoming O-360 (questo modello)', '5,9 l', '134 kW (180 CV)', 'Ingranaggi + aste e bilancieri'],
      ['Continental IO-360', '5,9 l', '≈ 150 kW (200–210 CV)', 'Ingranaggi + aste e bilancieri, iniezione'],
      ['Rotax 912 ULS', '1,35 l', '73,5 kW (100 CV) a 5 800 giri/min', 'Aste e bilancieri, riduttore 2,43:1'],
    ],
    note: 'Il Rotax 912 ha cilindri raffreddati ad aria e teste a liquido, gira più veloce e muove l’elica tramite un riduttore; può usare benzina auto senza piombo oltre all’avgas.',
  },
  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      range: [0, 0.25],
      parts: ['intakeSystem', 'valves', 'pistons', 'oilSump'],
      base: 'Il pistone si allontana dalla testa e risucchia la miscela preparata dal carburatore, che sta sotto il motore. I tubi passano dentro la coppa dell’olio caldo, così la benzina evapora meglio.',
      expert:
        'Valvola di aspirazione aperta (0–180°). Carburatore a galleggiante montato sotto la coppa: i condotti attraversano la coppa, che preriscalda la miscela, ed entrano nelle testate dal basso. Il pilota regola manetta e miscela; le versioni a iniezione (IO-360) dosano la benzina nei condotti, senza rischio di ghiaccio nel venturi.',
    },
    {
      id: 'compression',
      name: 'Compressione',
      range: [0.25, 0.5],
      parts: ['pistons', 'cylinderHeads', 'crankshaft'],
      base: 'Le valvole si chiudono e il pistone torna verso la testa, schiacciando la miscela. Il pistone di fronte, dall’altra parte del motore, fa lo stesso movimento in direzione opposta: le spinte si annullano e il motore vibra poco.',
      expert:
        'Rapporto di compressione 8,5:1 (O-360-A), basso per evitare la detonazione a pieno carico con teste molto calde (limite tipico 260 °C, ideale sotto 200 °C). Nel boxer i pistoni opposti hanno manovelle a 180°: le forze alterne del primo ordine si bilanciano, restano piccole coppie dovute allo sfalsamento dei cilindri lungo l’albero.',
    },
    {
      id: 'power',
      name: 'Scoppio (doppia accensione)',
      range: [0.5, 0.75],
      parts: ['sparkPlugs', 'magnetos', 'crankshaft', 'propeller'],
      base: 'Due candele fanno la scintilla insieme: la fiamma parte da due punti e brucia più in fretta. Ogni candela è collegata a un magnete diverso, che produce corrente da solo girando: anche senza batteria e con un magnete guasto, il motore continua a funzionare. Il gas spinge il pistone e fa girare l’elica.',
      expert:
        'Due magneti indipendenti, anticipo fisso ~25° prima del PMS. Ogni magnete accende una candela in ogni cilindro (superiori da un lato, inferiori dall’altro). Prima del decollo il pilota prova i magneti uno alla volta: il calo ammesso è di circa 125–175 giri/min. Due fronti di fiamma accorciano la combustione nel grande alesaggio da 130 mm.',
    },
    {
      id: 'exhaust',
      name: 'Scarico',
      range: [0.75, 1],
      parts: ['exhaust', 'valves', 'camshaft'],
      base: 'Si apre la valvola di scarico e il pistone spinge fuori i gas caldissimi verso la marmitta sotto il motore. Attorno alla marmitta un “manicotto” scalda l’aria per la cabina e per il carburatore.',
      expert:
        'Gas di scarico a 650–750 °C: il pilota usa l’EGT per smagrire la miscela fino al picco di temperatura o poco più ricco, a potenze di crociera ≤ 75%. Lo scambiatore sulla marmitta fornisce il riscaldamento cabina e il “carb heat”, aria calda contro il ghiaccio che si forma nel venturi anche con 20 °C e aria umida.',
    },
  ],
  quiz: [
    {
      q: 'Perché ogni cilindro ha due candele alimentate da due magneti diversi?',
      options: [
        'Per avere più potenza al decollo',
        'Per ridondanza: se un impianto si guasta, il motore continua a funzionare',
        'Per consumare meno olio',
        'Perché la benzina avio non si accende con una sola scintilla',
      ],
      correct: 1,
      explain: 'In volo non ci si può fermare a bordo strada: ogni parte vitale è doppia. In più due fiamme bruciano la miscela più in fretta.',
    },
    {
      q: 'Perché l’elica è collegata direttamente all’albero e il motore gira solo a ~2 700 giri/min?',
      options: [
        'Per tenere le punte delle pale sotto la velocità del suono',
        'Perché un motore a pistoni non può girare più veloce',
        'Per non disturbare la radio',
        'Per risparmiare olio',
      ],
      correct: 0,
      explain: 'Con un’elica di 1,9 m a 2 700 giri/min le punte viaggiano a ~265 m/s, circa Mach 0,8. Più veloci diventerebbero rumorose e inefficienti.',
    },
    {
      q: 'Come viene raffreddato questo motore?',
      options: [
        'Con un radiatore ad acqua come in auto',
        'Con l’aria dell’elica e del volo, guidata dai deflettori tra le alette dei cilindri',
        'Con l’olio del cambio',
        'Con il carburante che passa nei cilindri',
      ],
      correct: 1,
      explain: 'L’aria entra dietro l’elica, i deflettori la costringono a passare tra le alette e poi esce sotto il motore: niente radiatore né pompa dell’acqua.',
    },
    {
      q: 'Cosa fa il pilota con la leva rossa della miscela salendo di quota?',
      options: [
        'La porta tutta avanti per avere più benzina',
        'La tira indietro (smagrisce) perché l’aria è meno densa',
        'Non la tocca mai in volo',
        'La usa per avviare il motorino',
      ],
      correct: 1,
      explain: 'In quota l’aria pesa meno: con la stessa benzina la miscela diventa troppo ricca. Smagrire riporta il giusto rapporto aria/benzina e riduce i consumi.',
    },
  ],
  camera: { position: [-2.87, 1.98, 5.71], target: [0.15, 0.08, 0] },
  thumb: { orbit: [-24, 18], zoom: 1.5, view: 'section' },

  build(ctx) {
    const root = new THREE.Group();
    const [cy0, cy1] = CASE_Y;
    const valvesList = []; // { c, x, intake }
    for (const c of CYL) valvesList.push({ c, x: c.x - 0.07, intake: true }, { c, x: c.x + 0.07, intake: false });
    const boltGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.02, 6);
    const nutGeo = new THREE.CylinderGeometry(0.014, 0.014, 0.018, 6);

    // ---------- Crankcase (vertically split, aluminium) ----------
    const caseMat = cut(MAT.aluminum());
    caseMat.color.set(0xa3a9b0);
    caseMat.roughness = 0.6;
    const crankcase = new THREE.Group();
    for (const s of [1, -1]) {
      const wall = roundedRect(-0.66, cy0, 0.66, cy1, 0.1);
      for (const c of CYL.filter((q) => q.s === s)) {
        wall.holes.push(hole(c.x, 0, 0.16));
        for (const dx of [-0.07, 0.07]) wall.holes.push(hole(c.x + dx, CAM_Y, 0.024)); // pushrod tubes
      }
      const g = new THREE.ExtrudeGeometry(wall, { depth: 0.04, bevelEnabled: false, curveSegments: 24 });
      g.translate(0, 0, s > 0 ? DECK - 0.04 : -DECK);
      crankcase.add(new THREE.Mesh(g, caseMat));
    }
    const chh = cy1 - cy0;
    const top = new THREE.Mesh(rbox(1.2, 0.05, 2 * DECK), caseMat);
    top.position.y = cy1 - 0.025;
    const bottom = new THREE.Mesh(rbox(1.2, 0.05, 2 * DECK), caseMat);
    bottom.position.y = cy0 + 0.025;
    const front = new THREE.Mesh(rbox(0.05, chh - 0.12, 2 * DECK), caseMat);
    front.position.set(0.66 - 0.025, (cy0 + cy1) / 2, 0);
    const rear = front.clone();
    rear.position.x = -0.66 + 0.025;
    crankcase.add(top, bottom, front, rear);
    // parting flanges along the split line (z = 0), top and bottom
    for (const y of [cy1 + 0.02, cy0 - 0.02]) {
      const fl = new THREE.Mesh(rbox(1.26, 0.04, 0.09, 0.012), caseMat);
      fl.position.y = y;
      crankcase.add(fl);
    }
    // thrust-bearing nose (closed ring profile)
    const nose = new THREE.Mesh(
      lathe([[0.065, 0], [0.22, 0], [0.2, 0.04], [0.16, 0.08], [0.065, 0.08], [0.065, 0]], 40).rotateZ(-Math.PI / 2),
      caseMat
    );
    nose.position.x = 0.66;
    crankcase.add(nose);
    // accessory housing at the rear
    const accessory = new THREE.Mesh(rbox(0.09, 0.8, 0.6, 0.025), caseMat);
    accessory.position.set(-0.705, 0.06, 0);
    crankcase.add(accessory);
    const oilFilter = new THREE.Group();
    const filt = new THREE.Mesh(cylX(0.075, 0.2), MAT.paint(0x2d4a7a, 0.4));
    filt.position.x = -0.86;
    const filtBase = new THREE.Mesh(cylX(0.06, 0.05), MAT.aluminum());
    filtBase.position.x = -0.735;
    oilFilter.add(filt, filtBase);
    oilFilter.position.set(0, -0.12, 0);
    crankcase.add(oilFilter);
    // engine-driven fuel pump and tach drive on the accessory case
    const fuelPump = new THREE.Mesh(rbox(0.1, 0.12, 0.12, 0.02), MAT.paint(0x3b4048, 0.5));
    fuelPump.position.set(-0.8, -0.18, 0.22);
    const tach = new THREE.Mesh(cylX(0.03, 0.08, 16), MAT.aluminum());
    tach.position.set(-0.79, 0.08, 0);
    crankcase.add(fuelPump, tach);
    // nameplate
    const plate = new THREE.Mesh(new THREE.PlaneGeometry(0.22, 0.085).rotateY(-Math.PI / 2), nameplate());
    plate.position.set(-0.752, 0.08, -0.18);
    crankcase.add(plate);
    // rubber engine mounts on the rear corners
    for (const y of [0.4, -0.25]) {
      for (const z of [-0.36, 0.36]) {
        const m = new THREE.Mesh(cylX(0.055, 0.07, 20), MAT.rubber());
        m.position.set(-0.7, y, z);
        const boss = new THREE.Mesh(rbox(0.08, 0.1, 0.1, 0.02), caseMat);
        boss.position.set(-0.66, y, z * 0.88);
        crankcase.add(m, boss);
      }
    }
    // oil filler neck + dipstick cap
    const filler = tube([v3(-0.6, cy0 + 0.02, -0.25), v3(-0.6, -0.1, -0.36), v3(-0.6, 0.35, -0.4)], 0.022, MAT.aluminum(), 24);
    const fillerCap = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.035, 16), MAT.paint(0xe0b23a, 0.4));
    fillerCap.position.set(-0.6, 0.37, -0.4);
    crankcase.add(filler, fillerCap);
    // case bolts along the parting line
    const caseBolts = [];
    for (let x = -0.56; x <= 0.57; x += 0.16) {
      caseBolts.push({ p: v3(x, cy1 + 0.05, 0), n: v3(0, 1, 0) });
    }
    crankcase.add(instanced(boltGeo, cut(MAT.steel(), { cap: false }), caseBolts));
    root.add(
      ctx.part(crankcase, {
        id: 'crankcase',
        name: 'Basamento',
        base: 'La scatola centrale del motore: tiene l’albero motore e, sopra, l’albero a camme, e regge i quattro cilindri, due per lato. È fatta di due metà unite al centro da una fila di bulloni.',
        expert:
          'Basamento in lega di alluminio diviso verticalmente lungo l’asse dell’albero, con tre supporti di banco e i supporti dell’albero a camme sopra l’albero motore. Dietro c’è la scatola accessori (magneti, pompe olio e carburante, filtro, presa contagiri); davanti il cuscinetto reggispinta che trasmette la trazione dell’elica. Supporti elastici in gomma verso il castello motore.',
        explode: [0, 0, 0],
      })
    );

    const sumpMat = cut(MAT.aluminum());
    sumpMat.color.set(0x9aa0a7);
    sumpMat.roughness = 0.65;
    const sumpGroup = new THREE.Group();
    const sumpGeo = new THREE.ExtrudeGeometry(roundedRect(-0.6, -0.58, 0.3, cy0 - 0.005, 0.06), {
      depth: 0.44,
      bevelEnabled: true,
      bevelThickness: 0.02,
      bevelSize: 0.02,
      bevelSegments: 2,
    }).translate(0, 0, -0.22);
    sumpGroup.add(new THREE.Mesh(sumpGeo, sumpMat));
    for (let x = -0.5; x <= 0.25; x += 0.125) {
      const rib = new THREE.Mesh(rbox(0.02, 0.06, 0.4, 0.008), sumpMat);
      rib.position.set(x, -0.61, 0);
      sumpGroup.add(rib);
    }
    const drain = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.03, 6), MAT.steel());
    drain.position.set(-0.5, -0.615, 0.12);
    sumpGroup.add(drain);
    root.add(
      ctx.part(sumpGroup, {
        id: 'oilSump',
        name: 'Coppa dell’olio',
        base: 'La vaschetta sotto il motore con circa 7–8 litri di olio. Dentro passano anche i tubi della miscela verso i cilindri.',
        expert:
          'Coppa a carter umido da circa 8 quarti (≈ 7,5 L) per l’O-360. L’olio lubrifica e raffredda pistoni e cuscinetti e smaltisce una parte importante del calore, spesso con un radiatore dell’olio dedicato. I condotti di aspirazione la attraversano e ne prendono calore.',
        explode: [0, -1.3, 0],
      })
    );

    // ---------- Cylinders (barrels with fins), pushrod shroud tubes ----------
    const barrelMat = cut(MAT.paint(0x3e454e, 0.55));
    barrelMat.metalness = 0.6;
    const cylinders = new THREE.Group();
    const barrelLen = HEAD_Z - DECK + 0.03;
    const barrelGeo = pipe(0.168, BORE + 0.002, barrelLen, 40).rotateX(Math.PI / 2);
    const finGeo = pipe(0.232, 0.168, 0.009, 36).rotateX(Math.PI / 2);
    const flangeGeo = pipe(0.205, 0.168, 0.025, 36).rotateX(Math.PI / 2);
    const shroudGeo = pipe(0.022, 0.015, 0.54, 12).rotateX(Math.PI / 2);
    const shroudMat = cut(MAT.paint(0x9aa1a8, 0.45));
    const baseNuts = [];
    for (const c of CYL) {
      const b = new THREE.Mesh(barrelGeo, barrelMat);
      b.position.set(c.x, 0, c.s * (DECK - 0.03 + barrelLen / 2));
      cylinders.add(b);
      const fl = new THREE.Mesh(flangeGeo, barrelMat);
      fl.position.set(c.x, 0, c.s * (DECK + 0.0125));
      cylinders.add(fl);
      for (let z = 0.34; z < HEAD_Z - 0.01; z += 0.024) {
        const f = new THREE.Mesh(finGeo, barrelMat);
        f.position.set(c.x, 0, c.s * z);
        cylinders.add(f);
      }
      for (const dx of [-0.07, 0.07]) {
        const sh = new THREE.Mesh(shroudGeo, shroudMat);
        sh.position.set(c.x + dx, CAM_Y, c.s * (DECK + 0.25));
        cylinders.add(sh);
      }
      for (let i = 0; i < 8; i++) {
        const a = ((i + 0.5) / 8) * TAU;
        baseNuts.push({ p: v3(c.x + 0.188 * Math.cos(a), 0.188 * Math.sin(a), c.s * (DECK + 0.033)), n: v3(0, 0, c.s) });
      }
    }
    cylinders.add(instanced(nutGeo, cut(MAT.steel(), { cap: false }), baseNuts));
    root.add(
      ctx.part(cylinders, {
        id: 'cylinders',
        name: 'Cilindri alettati',
        base: 'I quattro tubi in cui scorrono i pistoni. Le sottili alette tutto intorno funzionano come quelle di un termosifone al contrario: aumentano la superficie e l’aria porta via il calore. Sopra corrono i tubi che proteggono le aste delle valvole.',
        expert:
          'Canne in acciaio (spesso nitrurate o cromate) con alettatura integrale, avvitate a testate in alluminio e fissate al basamento con otto prigionieri. Cilindri sfalsati lungo l’albero perché le bielle opposte stiano una accanto all’altra. Sopra i cilindri le guaine delle aste, a tenuta d’olio. Nella vista in sezione la bancata destra (cilindri 1 e 3) è tolta.',
        explode: [0, 0.75, 0],
      })
    );

    // ---------- Cylinder heads with fins and rocker covers (on top) ----------
    const headMat = cut(MAT.paint(0x8f979f, 0.5));
    headMat.metalness = 0.6;
    const coverMat = cut(MAT.paint(0x5f666e, 0.55));
    coverMat.metalness = 0.5;
    const heads = new THREE.Group();
    const headCore = cylZ(0.2, 0.19, 40);
    const headFin = pipe(0.235, 0.2, 0.009, 36).rotateX(Math.PI / 2);
    const portBoss = new THREE.CylinderGeometry(0.048, 0.048, 0.12, 20);
    const coverGeo = rbox(0.3, 0.46, 0.13, 0.04);
    const coverRibGeo = rbox(0.26, 0.02, 0.02, 0.008);
    const coverScrews = [];
    for (const c of CYL) {
      const h = new THREE.Mesh(headCore, headMat);
      h.position.set(c.x, 0, c.s * (HEAD_Z - 0.02 + 0.095));
      heads.add(h);
      for (let z = HEAD_Z + 0.015; z < HEAD_Z + 0.17; z += 0.022) {
        const f = new THREE.Mesh(headFin, headMat);
        f.position.set(c.x, 0, c.s * z);
        heads.add(f);
      }
      const cover = new THREE.Mesh(coverGeo, coverMat);
      cover.position.set(c.x, ROCK_Y, c.s * 0.885);
      heads.add(cover);
      for (const dy of [-0.1, 0, 0.1]) {
        const rib = new THREE.Mesh(coverRibGeo, coverMat);
        rib.position.set(c.x, ROCK_Y + dy, c.s * 0.952);
        heads.add(rib);
      }
      for (const dx of [-0.12, 0.12]) {
        for (const dy of [-0.19, 0, 0.19]) coverScrews.push({ p: v3(c.x + dx, ROCK_Y + dy, c.s * 0.952), n: v3(0, 0, c.s) });
      }
      for (const dx of [-0.14, 0.14]) {
        const p = new THREE.Mesh(portBoss, headMat);
        p.position.set(c.x + dx, -0.19, c.s * 0.72);
        heads.add(p);
      }
    }
    heads.add(instanced(nutGeo, cut(MAT.steel(), { cap: false }), coverScrews));
    root.add(
      ctx.part(heads, {
        id: 'cylinderHeads',
        name: 'Testate e coperchi bilancieri',
        base: 'Le “teste” dei cilindri, in alluminio: qui ci sono le valvole e le candele. Sono la parte più calda, per questo hanno alette più fitte. Sopra ogni testa un coperchio chiude i bilancieri.',
        expert:
          'Testate in lega di alluminio fuse con alettatura fitta, sedi valvola riportate e due fori candela (sopra e sotto). Condotti di aspirazione e scarico escono dal basso; i bilancieri stanno in alto sotto il coperchio, dove arrivano le aste. Il pilota tiene d’occhio la CHT: limite tipico 260 °C, meglio sotto 200 °C per la durata.',
        explode: [0, 1.2, 0],
      })
    );

    // ---------- Crankshaft ----------
    const crankMat = MAT.steel();
    const crank = new THREE.Group();
    // journals between the throws (throws sorted rear to front)
    const throwXs = CYL.map((c) => c.x).sort((a, b) => a - b);
    const allX = [-0.6, ...throwXs.flatMap((x) => [x - 0.075, x + 0.075]), 0.84];
    for (let i = 0; i < allX.length; i += 2) {
      const len = allX[i + 1] - allX[i];
      const j = new THREE.Mesh(cylX(0.06, len, 24), crankMat);
      j.position.x = (allX[i] + allX[i + 1]) / 2;
      crank.add(j);
    }
    const pinGeo = cylX(0.05, 0.09, 24);
    const webGeo = new THREE.BoxGeometry(0.03, 0.12, CR + 0.1);
    const cwGeo = new THREE.CylinderGeometry(0.15, 0.15, 0.03, 32, 1, false, Math.PI / 2, Math.PI).rotateZ(Math.PI / 2);
    for (const c of CYL) {
      const d = c.throw;
      const pin = new THREE.Mesh(pinGeo, crankMat);
      pin.position.set(c.x, -CR * Math.sin(d), CR * Math.cos(d));
      crank.add(pin);
      for (const dx of [-0.06, 0.06]) {
        const web = new THREE.Mesh(webGeo, crankMat);
        web.position.set(c.x + dx, (-CR / 2) * Math.sin(d), (CR / 2) * Math.cos(d));
        web.rotation.x = d;
        crank.add(web);
        const cw = new THREE.Mesh(cwGeo, crankMat);
        cw.position.x = c.x + dx;
        cw.rotation.x = d;
        crank.add(cw);
      }
    }
    const propFlange = new THREE.Mesh(cylX(0.17, 0.03, 40), crankMat);
    propFlange.position.x = 0.83;
    crank.add(propFlange);
    const crankGear = new THREE.Mesh(gearX(0.1, 16, 0.012, Math.PI / 2, 0.035, 0.05), MAT.darkSteel());
    crankGear.position.x = -0.6;
    crank.add(crankGear);
    root.add(
      ctx.part(crank, {
        id: 'crankshaft',
        name: 'Albero motore',
        base: 'L’albero a gomiti che trasforma l’andirivieni dei pistoni in rotazione. Davanti esce dal motore ed è avvitato direttamente all’elica; dietro un ingranaggio fa girare l’albero a camme.',
        expert:
          'Albero in acciaio forgiato a quattro manovelle (a coppie a 180°) su tre supporti di banco, spesso con smorzatori dinamici a pendolo sui contrappesi contro le vibrazioni torsionali. La flangia anteriore porta l’elica senza riduttore: niente volano, è l’elica stessa a fare da volano. Sul retro l’ingranaggio della distribuzione.',
        explode: [0, -0.45, 0],
      })
    );

    // ---------- Pistons and conrods ----------
    const pistonGeo = lathe(
      [[0, CROWN], [0.146, CROWN], [0.148, CROWN - 0.01], [0.148, -0.08], [0.138, -0.08], [0.138, CROWN - 0.03], [0, CROWN - 0.03]],
      40
    );
    const ringGeo = new THREE.TorusGeometry(0.149, 0.005, 6, 40).rotateX(Math.PI / 2);
    const pistonMat = MAT.aluminum();
    const ringMat = MAT.darkSteel();
    const pinMat = MAT.steel();
    const pistonGroup = new THREE.Group();
    const pistons = CYL.map((c) => {
      const outer = new THREE.Group();
      const p = new THREE.Group();
      p.add(new THREE.Mesh(pistonGeo, pistonMat));
      for (const y of [0.055, 0.035, 0.015]) {
        const r = new THREE.Mesh(ringGeo, ringMat);
        r.position.y = y;
        p.add(r);
      }
      p.add(new THREE.Mesh(cylX(0.035, 0.27, 16), pinMat));
      p.rotation.x = c.s * (Math.PI / 2); // crown points away from the crank
      outer.add(p);
      outer.position.x = c.x;
      pistonGroup.add(outer);
      return outer;
    });
    root.add(
      ctx.part(pistonGroup, {
        id: 'pistons',
        name: 'Pistoni',
        base: 'I tappi mobili spinti dal gas. Guarda i due di fronte: si allontanano e si avvicinano insieme, come due pugili che si scambiano colpi, per questo si chiama “boxer”.',
        expert:
          'Pistoni in lega di alluminio da 130 mm con tre o quattro fasce. Corsa 111 mm: a 2 700 giri/min la velocità media è ~10 m/s, circa la metà di un motore d’auto sportivo, a tutto vantaggio della durata (TBO ~2 000 ore).',
        explode: [0, 0.35, 0],
      })
    );

    const rodMat = MAT.steel();
    const rodBeam = new THREE.BoxGeometry(0.04, 0.06, ROD - 0.1).translate(0, 0, ROD / 2);
    const bigEnd = new THREE.TorusGeometry(0.062, 0.02, 10, 24).rotateY(Math.PI / 2);
    const smallEnd = new THREE.TorusGeometry(0.045, 0.016, 10, 20).rotateY(Math.PI / 2).translate(0, 0, ROD);
    const rodGroup = new THREE.Group();
    const rods = CYL.map((c) => {
      const r = new THREE.Group();
      r.add(new THREE.Mesh(rodBeam, rodMat), new THREE.Mesh(bigEnd, rodMat), new THREE.Mesh(smallEnd, rodMat));
      r.position.x = c.x;
      rodGroup.add(r);
      return r;
    });
    root.add(
      ctx.part(rodGroup, {
        id: 'conrods',
        name: 'Bielle',
        base: 'Le braccia che collegano i pistoni all’albero motore. Ogni biella ha la sua manovella, perché i cilindri sono leggermente sfalsati.',
        expert: 'Bielle in acciaio forgiato a doppio T con cuscinetti a guscio; i cilindri sfalsati permettono una manovella per biella senza allungare troppo il motore.',
        explode: [0, -0.2, 0],
      })
    );

    // ---------- Camshaft (above the crank), hydraulic tappets, pushrods ----------
    const camGroup = new THREE.Group();
    const cam = new THREE.Group();
    cam.position.y = CAM_Y;
    cam.add(new THREE.Mesh(cylX(0.028, 1.12, 16), MAT.steel()).translateX(-0.07));
    const lobeGeo = camLobeGeometry();
    const lobeMat = MAT.chrome();
    for (const v of valvesList) {
      const peak = v.intake ? (IN_WIN[0] + IN_WIN[1]) / 2 : (EX_WIN[0] + EX_WIN[1]) / 2;
      const lobe = new THREE.Mesh(lobeGeo, lobeMat);
      lobe.position.x = v.x;
      // cam turns at -½ crank speed: nose meets the follower (±Z) at the lift peak
      lobe.rotation.x = TAU * (v.c.off + peak) + (v.c.s < 0 ? Math.PI : 0);
      cam.add(lobe);
    }
    // cam gear: twice the crank gear, gap facing down toward the crank gear
    const camGear = new THREE.Mesh(gearX(0.2, 32, 0.012, -Math.PI / 2 + Math.PI / 32, 0.035, 0.03), MAT.darkSteel());
    camGear.position.x = -0.6;
    cam.add(camGear);
    camGroup.add(cam);
    const tappetGeo = cylZ(0.028, 0.1, 16);
    const pushGeo = cylZ(0.011, 0.725, 8).translate(0, 0, 0.3625);
    const tappetMat = MAT.chrome();
    const pushMat = MAT.steel();
    const tappets = [];
    const pushrods = [];
    for (const v of valvesList) {
      const t = new THREE.Mesh(tappetGeo, tappetMat);
      t.position.set(v.x, CAM_Y, 0);
      camGroup.add(t);
      tappets.push(t);
      const p = new THREE.Mesh(pushGeo, pushMat);
      p.position.set(v.x, CAM_Y, 0);
      if (v.c.s < 0) p.rotation.y = Math.PI;
      camGroup.add(p);
      pushrods.push(p);
    }
    root.add(
      ctx.part(camGroup, {
        id: 'camshaft',
        name: 'Albero a camme, punterie e aste',
        base: 'Un albero con otto “gobbe” sopra l’albero motore, che gira a metà velocità grazie a due ingranaggi. Le gobbe spingono le punterie e lunghe aste che corrono sopra i cilindri fino alla testa.',
        expert:
          'Albero a camme unico nel basamento, sopra l’albero motore, comandato da ingranaggi 2:1 sul retro; una camma per valvola, condivisa da una sola punteria. Punterie idrauliche (recupero automatico del gioco) e aste in tubo d’acciaio dentro guaine a tenuta d’olio. Con l’albero in alto, dopo lunghe soste l’olio scola via dalle camme: la corrosione delle camme è un problema noto di questi motori se volano poco.',
        explode: [0, 0.4, -1.2],
      })
    );

    // ---------- Valves and rockers ----------
    const valveGeo = lathe([[0, 0], [0.055, 0], [0.055, 0.01], [0.016, 0.045], [0.011, 0.07], [0.011, 0.235], [0, 0.235]], 20);
    const springGeo = new THREE.TubeGeometry(
      new (class extends THREE.Curve {
        getPoint(t, out = new THREE.Vector3()) {
          const a = t * TAU * 5;
          return out.set(Math.cos(a) * 0.028, 0.09 + t * 0.12, Math.sin(a) * 0.028);
        }
      })(),
      90,
      0.005,
      5
    );
    const rockerGeo = new THREE.BoxGeometry(0.03, 0.34, 0.035);
    const valveObjs = [];
    const rockers = [];
    const valvesPart = new THREE.Group();
    const inValveMat = MAT.steel();
    const exValveMat = MAT.titanium();
    const springMat = MAT.darkSteel();
    const rockerMat = MAT.chrome();
    for (const v of valvesList) {
      const s = v.c.s;
      const g = new THREE.Group();
      const inner = new THREE.Group();
      inner.add(new THREE.Mesh(valveGeo, v.intake ? inValveMat : exValveMat));
      inner.add(new THREE.Mesh(springGeo, springMat));
      inner.rotation.x = s * (Math.PI / 2);
      g.add(inner);
      g.position.set(v.x, 0, s * HEAD_Z);
      valvesPart.add(g);
      valveObjs.push(g);
      const r = new THREE.Mesh(rockerGeo, rockerMat);
      r.position.set(v.x, ROCK_Y, s * ROCK_Z);
      valvesPart.add(r);
      rockers.push(r);
    }
    for (const c of CYL) {
      const sh = new THREE.Mesh(cylX(0.012, 0.22, 10), springMat);
      sh.position.set(c.x, ROCK_Y, c.s * ROCK_Z);
      valvesPart.add(sh);
    }
    root.add(
      ctx.part(valvesPart, {
        id: 'valves',
        name: 'Valvole e bilancieri',
        base: 'Due porte per cilindro: una fa entrare la miscela, l’altra fa uscire i gas. Il bilanciere è un’altalena: l’asta spinge da una parte e l’altra parte spinge dentro la valvola.',
        expert:
          'Due valvole per cilindro; quella di scarico ha spesso lo stelo cavo riempito di sodio per smaltire il calore. Bilancieri in acciaio su perni nella testata, sotto il coperchio. Le punterie idrauliche annullano il gioco, che cambia molto tra motore freddo e caldo in un cilindro raffreddato ad aria.',
        explode: [0, 1.65, 0],
      })
    );

    // ---------- Spark plugs, sparks ----------
    const plugs = new THREE.Group();
    const plugGeos = [
      [new THREE.CylinderGeometry(0.022, 0.018, 0.08, 12).translate(0, 0.04, 0), MAT.steel()],
      [new THREE.CylinderGeometry(0.04, 0.04, 0.04, 6).translate(0, 0.1, 0), MAT.chrome()],
      [new THREE.CylinderGeometry(0.024, 0.024, 0.09, 12).translate(0, 0.165, 0), MAT.ceramic()],
      [new THREE.CylinderGeometry(0.03, 0.03, 0.06, 12).translate(0, 0.235, 0), MAT.paint(0x8a2a22, 0.6)],
    ];
    const plugTerm = []; // [cyl][top/bottom] -> terminal position
    const sparks = [];
    const sparkLights = [];
    const sparkGeo = new THREE.SphereGeometry(0.045, 14, 10);
    CYL.forEach((c, k) => {
      plugTerm.push([]);
      sparks.push([]);
      for (const up of [1, -1]) {
        const dir = v3(0, 0.8 * up, 0.6 * c.s).normalize();
        const tip = v3(c.x, 0.11 * up, c.s * 0.625);
        const p = new THREE.Group();
        for (const [g, m] of plugGeos) p.add(new THREE.Mesh(g, m));
        p.position.copy(tip);
        p.quaternion.setFromUnitVectors(Y_UP, dir);
        plugs.add(p);
        plugTerm[k].push(tip.clone().addScaledVector(dir, 0.265));
        const sp = new THREE.Mesh(sparkGeo, MAT.glow(0xbfe6ff, 6, 0.95));
        sp.position.set(c.x, 0.1 * up, c.s * 0.615);
        sp.visible = false;
        root.add(sp);
        sparks[k].push(sp);
      }
      const light = new THREE.PointLight(0xffa040, 0, 1.6, 2);
      light.position.set(c.x, 0.05, c.s * 0.75);
      root.add(light);
      sparkLights.push(light);
    });
    root.add(
      ctx.part(plugs, {
        id: 'sparkPlugs',
        name: 'Candele (due per cilindro)',
        base: 'Otto candele, due per cilindro, una in alto e una in basso. Scoccano insieme: se una smette di funzionare, l’altra basta a far girare il motore.',
        expert:
          'Candele aeronautiche schermate (contro i disturbi radio) a elettrodi massicci o in platino/iridio. Il piombo della 100LL tende a sporcarle: per questo a terra si smagrisce e si tengono i giri sopra il minimo.',
        explode: [0, 1.95, 0],
      })
    );

    // ---------- Magnetos and ignition leads ----------
    const mags = new THREE.Group();
    const magMat = MAT.paint(0x1c1f24, 0.5);
    const leadMat = [MAT.paint(0x4d6fb3, 0.6), MAT.paint(0xb35a4d, 0.6)];
    const magFlangeMat = MAT.aluminum();
    const magCapMat = MAT.paint(0x6a6d72, 0.5);
    const clampMat = MAT.steel();
    [-1, 1].forEach((zs, mi) => {
      const body = new THREE.Mesh(cylX(0.075, 0.2, 28), magMat);
      body.position.set(-0.85, 0.32, zs * 0.18);
      const cap = new THREE.Mesh(cylX(0.065, 0.05, 24), magCapMat);
      cap.position.set(-0.975, 0.32, zs * 0.18);
      const flangeM = new THREE.Mesh(cylX(0.095, 0.02, 28), magFlangeMat);
      flangeM.position.set(-0.75, 0.32, zs * 0.18);
      const pLead = tube([v3(-0.9, 0.39, zs * 0.18), v3(-0.86, 0.45, zs * 0.1), v3(-0.75, 0.4, zs * 0.05)], 0.006, MAT.paint(0x222222, 0.6), 12);
      mags.add(body, cap, flangeM, pLead);
      const capPt = v3(-1.0, 0.32, zs * 0.18);
      // left magneto (z<0): top plugs right bank + bottom plugs left bank; right magneto the opposite
      CYL.forEach((c, k) => {
        const useTop = mi === 0 ? c.s > 0 : c.s < 0;
        const term = plugTerm[k][useTop ? 0 : 1];
        // top leads run over the case, bottom leads down the rear and under the heads
        const pts = useTop
          ? [capPt, v3(-0.95, 0.6, zs * 0.22), v3(c.x - 0.12, 0.62, c.s * 0.4), v3(c.x, 0.46, c.s * 0.74), term]
          : [capPt, v3(-1.0, 0.05, (zs + c.s) * 0.2), v3(-0.76, -0.42, c.s * 0.74), v3(c.x - 0.06, -0.45, c.s * 0.87), v3(c.x, -0.39, c.s * 0.83), term];
        mags.add(tube(pts, 0.011, leadMat[mi], 64, false, 6));
      });
      // harness clamp on top of the case
      const clamp = new THREE.Mesh(rbox(0.06, 0.03, 0.08, 0.01), clampMat);
      clamp.position.set(-0.45, 0.6, zs * 0.18);
      mags.add(clamp);
    });
    root.add(
      ctx.part(mags, {
        id: 'magnetos',
        name: 'Magneti e cavi di accensione',
        base: 'Due piccoli generatori dietro al motore: girando producono da soli la corrente per le candele, senza bisogno della batteria. I cavi blu vengono da uno, quelli rossi dall’altro.',
        expert:
          'Magneti a induzione con magnete permanente rotante, trasformatore e ruttore (o elettronica), trascinati dalla scatola accessori. Ogni magnete alimenta una candela per cilindro: con l’interruttore a chiave si sceglie L, R o BOTH. Un cavo di massa (P-lead) staccato lascia il magnete “caldo”: girare l’elica a mano è pericoloso anche a motore spento.',
        explode: [-0.8, 0.9, 0],
      })
    );

    // ---------- Intake: carburettor, air box, intake pipes, fuel line ----------
    const intakeMat = cut(MAT.aluminum(), { cap: false });
    const intake = new THREE.Group();
    const carbMat = cut(MAT.paint(0x9a8f7a, 0.5));
    const carb = new THREE.Mesh(rbox(0.16, 0.16, 0.2, 0.02), carbMat);
    carb.position.set(-0.12, -0.67, 0);
    const bowl = new THREE.Mesh(cylZ(0.07, 0.16, 24), carbMat);
    bowl.position.set(-0.02, -0.69, 0);
    const airbox = new THREE.Mesh(rbox(0.32, 0.12, 0.36, 0.03), cut(MAT.paint(0x2b2f36, 0.7), { cap: false }));
    airbox.position.set(-0.12, -0.81, 0);
    const filterEl = new THREE.Mesh(rbox(0.26, 0.03, 0.3, 0.01), MAT.paint(0xb8673a, 0.8));
    filterEl.position.set(-0.12, -0.885, 0);
    const mixLever = new THREE.Mesh(rbox(0.1, 0.02, 0.02, 0.006), MAT.paint(0xc8352b, 0.5));
    mixLever.position.set(-0.22, -0.64, 0.11);
    const thrLever = new THREE.Mesh(rbox(0.1, 0.02, 0.02, 0.006), MAT.paint(0x1c1f24, 0.5));
    thrLever.position.set(-0.22, -0.7, -0.11);
    const fuelLine = tube([v3(-0.8, -0.24, 0.22), v3(-0.62, -0.5, 0.2), v3(-0.3, -0.66, 0.14), v3(-0.02, -0.66, 0.08)], 0.012, MAT.paint(0x6b5a3a, 0.7), 32);
    intake.add(carb, bowl, airbox, filterEl, mixLever, thrLever, fuelLine);
    const hoseClampGeo = new THREE.TorusGeometry(0.04, 0.007, 6, 16);
    const intakePaths = CYL.map((c) => {
      const xi = c.x - 0.14;
      intake.add(
        tube([v3(xi, -0.48, c.s * 0.2), v3(xi, -0.48, c.s * 0.46), v3(xi, -0.36, c.s * 0.66), v3(xi, -0.25, c.s * 0.72)], 0.034, intakeMat, 32)
      );
      const cl = new THREE.Mesh(hoseClampGeo, clampMat);
      cl.position.set(xi, -0.48, c.s * 0.34);
      cl.rotation.y = Math.PI / 2;
      intake.add(cl);
      return new THREE.CatmullRomCurve3([
        v3(-0.12, -0.9, 0),
        v3(-0.12, -0.66, 0),
        v3(-0.12, -0.5, 0),
        v3(xi, -0.48, c.s * 0.2),
        v3(xi, -0.48, c.s * 0.46),
        v3(xi, -0.36, c.s * 0.66),
        v3(xi, -0.2, c.s * 0.72),
        v3(c.x - 0.07, -0.02, c.s * 0.63),
      ]);
    });
    root.add(
      ctx.part(intake, {
        id: 'intakeSystem',
        name: 'Carburatore e aspirazione',
        base: 'Sotto il motore c’è il carburatore: mescola aria e benzina e la manda ai cilindri. Dalla cabina il pilota usa due leve: il gas (nera) e la miscela (rossa).',
        expert:
          'Carburatore a galleggiante con venturi, montato sotto la coppa (aspirazione dal basso). L’espansione dell’aria nel venturi e l’evaporazione della benzina raffreddano molto: ghiaccio anche con aria esterna mite e umida, contrastato con il “carb heat”. Il comando miscela permette di smagrire in quota, dove la densità dell’aria cala (~3% ogni 300 m). La pompa carburante meccanica è sulla scatola accessori.',
        explode: [0, -1.85, 0],
      })
    );

    // ---------- Exhaust ----------
    const exMat = cut(MAT.paint(0x8a7f74, 0.45), { cap: false });
    exMat.metalness = 0.85;
    const exhaust = new THREE.Group();
    const exFlangeGeo = rbox(0.11, 0.025, 0.11, 0.01);
    const exhaustPaths = CYL.map((c) => {
      const xe = c.x + 0.14;
      const pts = [
        v3(xe, -0.25, c.s * 0.72),
        v3(xe, -0.44, c.s * 0.7),
        v3(xe - 0.05, -0.56, c.s * 0.56),
        v3(-0.55, -0.68, c.s * 0.42),
        v3(-0.8, -0.78, c.s * 0.2),
      ];
      exhaust.add(tube(pts, 0.035, exMat, 48));
      const fl = new THREE.Mesh(exFlangeGeo, MAT.darkSteel());
      fl.position.set(xe, -0.26, c.s * 0.72);
      exhaust.add(fl);
      return new THREE.CatmullRomCurve3([
        v3(c.x + 0.07, -0.02, c.s * 0.63),
        v3(xe, -0.2, c.s * 0.72),
        ...pts.slice(1),
        v3(-0.86, -0.84, c.s * 0.06),
        v3(-1.0, -0.98, 0),
        v3(-1.15, -1.08, 0),
      ]);
    });
    const muffler = new THREE.Mesh(cylZ(0.11, 0.62, 28), exMat);
    muffler.position.set(-0.86, -0.8, 0);
    const muff = new THREE.Mesh(cylZ(0.13, 0.4, 28), cut(MAT.aluminum(), { cap: false }));
    muff.position.copy(muffler.position);
    const heatHose = tube([v3(-0.86, -0.67, 0.12), v3(-0.7, -0.6, 0.12), v3(-0.3, -0.84, 0.12), v3(-0.18, -0.84, 0.12)], 0.025, MAT.paint(0x8c8f93, 0.8), 24);
    exhaust.add(muffler, muff, heatHose, tube([v3(-0.86, -0.86, 0), v3(-1.0, -0.98, 0), v3(-1.15, -1.08, 0)], 0.04, exMat, 16));
    root.add(
      ctx.part(exhaust, {
        id: 'exhaust',
        name: 'Scarico e marmitta',
        base: 'Quattro tubi portano i gas caldi alla marmitta sotto il motore. Il manicotto attorno alla marmitta scalda l’aria per la cabina e, con un tubo, per il carburatore.',
        expert:
          'Collettori in acciaio inox verso marmitta con scambiatore di calore per cabina e carb heat. Una crepa nello scambiatore può portare monossido di carbonio in cabina: per questo si usano rilevatori di CO. Sonde EGT su ogni cilindro per la regolazione della miscela.',
        explode: [-0.9, -1.0, 0],
      })
    );

    // ---------- Propeller ----------
    const prop = new THREE.Group();
    const bladeMat = MAT.aluminum();
    bladeMat.roughness = 0.3;
    const tipMat = MAT.paint(0xf2c230, 0.4);
    const mainGeo = bladeGeometry(0.12, PROP_R - 0.13, 30);
    const tipGeo = bladeGeometry(PROP_R - 0.13, PROP_R, 6);
    for (const a of [0, Math.PI]) {
      const b = new THREE.Group();
      b.add(new THREE.Mesh(mainGeo, bladeMat), new THREE.Mesh(tipGeo, tipMat));
      b.rotation.x = a;
      prop.add(b);
    }
    const hub = new THREE.Mesh(cylX(0.13, 0.15, 32), MAT.aluminum());
    prop.add(hub);
    const spinner = new THREE.Mesh(
      lathe([[0, 0], [0.19, 0], [0.185, 0.1], [0.15, 0.24], [0.08, 0.36], [0, 0.42]], 40).rotateZ(-Math.PI / 2),
      MAT.paint(0xd23a2a, 0.3)
    );
    spinner.position.x = 0.07;
    prop.add(spinner);
    const propBolts = [];
    for (let i = 0; i < 6; i++) {
      const a = ((i + 0.5) / 6) * TAU;
      propBolts.push({ p: v3(-0.08, 0.1 * Math.cos(a), 0.1 * Math.sin(a)), n: v3(-1, 0, 0) });
    }
    prop.add(instanced(boltGeo, MAT.steel(), propBolts));
    const blurMat = new THREE.MeshBasicMaterial({ color: 0x9aa4b0, transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide });
    const blur = new THREE.Mesh(new THREE.RingGeometry(0.22, PROP_R, 72, 1).rotateY(Math.PI / 2), blurMat);
    blur.renderOrder = 3;
    const propWrap = new THREE.Group();
    propWrap.position.x = PROP_X;
    propWrap.add(prop);
    root.add(
      ctx.part(propWrap, {
        id: 'propeller',
        name: 'Elica bipala a passo fisso',
        base: 'Due pale lunghe quasi un metro ciascuna: sono ali che girano e “avvitano” l’aereo nell’aria. Sono storte (svergolate) perché la punta va molto più veloce della radice.',
        expert:
          'Elica in lega di alluminio, diametro ~1,88 m (74 pollici) e passo ~1,5 m, in presa diretta: a 2 700 giri/min le punte girano a ~265 m/s (Mach ~0,78). Lo svergolamento mantiene un angolo d’attacco simile lungo la pala. Punte gialle per renderla visibile a terra. Sui motori più potenti l’elica a passo variabile (giri costanti) ottimizza decollo e crociera.',
        explode: [1.5, 0, 0],
      })
    );
    blur.position.x = PROP_X - 0.01;
    root.add(blur);

    // ---------- Starter ring gear and starter motor ----------
    const starterPart = new THREE.Group();
    const ringGear = new THREE.Mesh(gearX(0.42, 90, 0.012, 0, 0.035, 0.3), MAT.steel());
    ringGear.position.x = 0.79;
    const ringDisc = new THREE.Mesh(cylX(0.3, 0.02, 40), MAT.darkSteel());
    ringDisc.position.x = 0.79;
    const pulleyMain = new THREE.Mesh(
      lathe([[0.1, -0.025], [0.28, -0.025], [0.26, 0], [0.28, 0.025], [0.1, 0.025], [0.1, -0.025]], 48).rotateZ(Math.PI / 2),
      MAT.aluminum()
    );
    pulleyMain.position.x = 0.745;
    const ringSpin = new THREE.Group();
    ringSpin.add(ringGear, ringDisc, pulleyMain);
    starterPart.add(ringSpin);
    const stPos = v3(0.62, -0.3, -0.33);
    const stBody = new THREE.Mesh(cylX(0.075, 0.24, 28), MAT.paint(0x1c1f24, 0.5));
    stBody.position.copy(stPos);
    const stSol = new THREE.Mesh(cylX(0.035, 0.16, 16), MAT.paint(0x1c1f24, 0.5));
    stSol.position.set(0.6, -0.39, -0.39);
    const stPinion = new THREE.Mesh(cylX(0.035, 0.04, 12), MAT.steel());
    stPinion.position.set(0.79, stPos.y, stPos.z);
    const stBracket = new THREE.Mesh(rbox(0.04, 0.18, 0.16, 0.01), caseMat);
    stBracket.position.set(0.66, -0.25, -0.26);
    const stCable = tube([v3(0.53, -0.39, -0.39), v3(0.45, -0.45, -0.42), v3(0.3, -0.42, -0.34)], 0.012, MAT.rubber(), 16);
    starterPart.add(stBody, stSol, stPinion, stBracket, stCable);
    root.add(
      ctx.part(starterPart, {
        id: 'starter',
        name: 'Corona dentata e motorino d’avviamento',
        base: 'La ruota dentata dietro l’elica: all’avviamento il motorino elettrico la fa girare per mettere in moto. Una volta partito, il motorino si stacca.',
        expert:
          'Corona dentata sul mozzo dell’elica, ingranata all’avviamento dal pignone del motorino a 12 o 24 V. Nella stessa flangia c’è la puleggia della cinghia dell’alternatore. Dopo l’avviamento il pignone si disinnesta: se resta ingranato, l’avviso “starter engaged” chiede di spegnere.',
        explode: [0.9, -0.3, -0.5],
      })
    );

    // ---------- Alternator with belt ----------
    const altPart = new THREE.Group();
    const altPos = v3(0.6, -0.42, 0.3);
    const altBody = new THREE.Mesh(cylX(0.09, 0.18, 28), MAT.aluminum());
    altBody.position.copy(altPos);
    const altFins = new THREE.Mesh(pipe(0.095, 0.085, 0.12, 28).rotateZ(Math.PI / 2), MAT.darkSteel());
    altFins.position.copy(altPos);
    const altBracket = new THREE.Mesh(rbox(0.04, 0.14, 0.16, 0.01), caseMat);
    altBracket.position.set(0.66, -0.3, 0.24);
    const altPulley = new THREE.Group();
    altPulley.add(new THREE.Mesh(cylX(0.05, 0.04, 20), MAT.darkSteel()));
    const altFan = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.15, 0.02), MAT.steel());
    altFan.position.x = -0.03;
    altPulley.add(altFan);
    altPulley.position.set(0.745, altPos.y, altPos.z);
    const altWire = tube([v3(0.52, -0.42, 0.36), v3(0.42, -0.4, 0.42), v3(0.3, -0.33, 0.33)], 0.01, MAT.paint(0x8a2a22, 0.6), 16);
    // belt: hull of the two pulleys in the Y-Z plane
    const beltPts = [];
    const dy = altPos.y;
    const dz = altPos.z;
    const dl = Math.hypot(dy, dz);
    const base = Math.atan2(dz, dy);
    const gam = Math.acos((0.28 - 0.05) / dl);
    for (let i = 0; i <= 28; i++) {
      const a = base + gam + ((TAU - 2 * gam) * i) / 28;
      beltPts.push(v3(0.745, 0.285 * Math.cos(a), 0.285 * Math.sin(a)));
    }
    for (let i = 0; i <= 10; i++) {
      const a = base - gam + (2 * gam * i) / 10;
      beltPts.push(v3(0.745, dy + 0.055 * Math.cos(a), dz + 0.055 * Math.sin(a)));
    }
    altPart.add(altBody, altFins, altBracket, altPulley, altWire, tube(beltPts, 0.012, MAT.rubber(), 120, true, 6));
    root.add(
      ctx.part(altPart, {
        id: 'alternator',
        name: 'Alternatore',
        base: 'Una piccola dinamo mossa da una cinghia: ricarica la batteria e alimenta radio e strumenti. Al motore non serve per girare, grazie ai magneti.',
        expert:
          'Alternatore 14 o 28 V, 60–70 A, trascinato a cinghia dalla puleggia sul mozzo dell’elica. Alimenta avionica, luci e pompa elettrica; se si guasta il motore continua a funzionare perché l’accensione è a magneti.',
        explode: [0.9, -0.5, 0.6],
      })
    );

    // ---------- Cooling baffles ----------
    const baffleMat = cut(MAT.paint(0xbfc5cb, 0.45));
    baffleMat.metalness = 0.5;
    const sealMat = MAT.paint(0xc8552b, 0.7);
    const baffles = new THREE.Group();
    const underGeo = new THREE.CylinderGeometry(0.25, 0.25, 0.28, 24, 1, true, Math.PI / 2, Math.PI).rotateX(-Math.PI / 2);
    const underMat = cut(MAT.paint(0xbfc5cb, 0.45), { cap: false });
    for (const s of [1, -1]) {
      const rearB = new THREE.Mesh(rbox(0.016, 0.5, 0.58, 0.006), baffleMat);
      rearB.position.set(-0.65, 0.13, s * 0.68);
      const rearSeal = new THREE.Mesh(rbox(0.04, 0.03, 0.58, 0.01), sealMat);
      rearSeal.position.set(-0.65, 0.39, s * 0.68);
      const frontB = new THREE.Mesh(rbox(0.016, 0.16, 0.62, 0.006), baffleMat);
      frontB.position.set(0.67, 0.36, s * 0.66);
      const frontSeal = new THREE.Mesh(rbox(0.04, 0.03, 0.62, 0.01), sealMat);
      frontSeal.position.set(0.67, 0.455, s * 0.66);
      baffles.add(rearB, rearSeal, frontB, frontSeal);
    }
    for (const c of CYL) {
      const u = new THREE.Mesh(underGeo, underMat);
      u.position.set(c.x, 0, c.s * 0.48);
      baffles.add(u);
    }
    root.add(
      ctx.part(baffles, {
        id: 'baffles',
        name: 'Deflettori di raffreddamento',
        base: 'Lamiere che fanno da “corridoio” per l’aria: la costringono a passare proprio in mezzo alle alette dei cilindri invece di girarci intorno.',
        expert:
          'Raffreddamento a pressione: l’aria entra dalle prese dietro l’elica nel plenum sopra i cilindri, i deflettori con le guarnizioni in silicone (arancioni) sigillano contro la cappottatura e la obbligano ad attraversare le alette verso il basso; i deflettori curvi sotto i cilindri la tengono aderente alle alette. Un deflettore rotto fa salire subito la CHT di un cilindro.',
        explode: [-0.4, 0.95, 0],
      })
    );

    // ---------- Gas charges ----------
    const chargeGeo = cylZ(0.147, 1, 32);
    const charges = CYL.map((c) => {
      const m = new THREE.Mesh(chargeGeo, MAT.gas(0x5fd3ff, 0.2, false));
      m.position.x = c.x;
      m.renderOrder = 2;
      root.add(m);
      return m;
    });

    // ---------- Flows ----------
    const intakeFlows = intakePaths.map((cv) => {
      const f = flow({
        path: (t, out) => cv.getPointAt(t, out),
        count: 60,
        size: 0.05,
        colors: [0x5fd3ff, 0xffc04d, 0xc8d070, 0x9ad8a0],
        spread: 0.025,
        speed: 1,
      });
      root.add(f.object);
      return f;
    });
    const exhaustFlows = exhaustPaths.map((cv) => {
      const f = flow({
        path: (t, out) => cv.getPointAt(t, out),
        count: 70,
        size: 0.06,
        colors: [0xff5a1f, 0xb07a6a, 0x8c86a8],
        spread: 0.025,
        speed: 0.8,
      });
      root.add(f.object);
      return f;
    });
    const coolFlows = CYL.map((c) => {
      const cv = new THREE.CatmullRomCurve3([
        v3(1.75, 0.55, c.s * 0.6),
        v3(1.05, 0.52, c.s * 0.6),
        v3(c.x + 0.12, 0.46, c.s * 0.58),
        v3(c.x, 0.22, c.s * 0.56),
        v3(c.x, -0.1, c.s * 0.56),
        v3(c.x - 0.05, -0.42, c.s * 0.62),
        v3(-0.4, -0.95, c.s * 0.6),
        v3(-1.1, -1.05, c.s * 0.5),
      ]);
      const f = flow({
        path: (t, out) => cv.getPointAt(t, out),
        count: 130,
        size: 0.06,
        colors: [0x5fd3ff, 0x5fd3ff, 0x5fd3ff, 0xffb38a, 0xc9a08a],
        spread: (t) => (t > 0.35 && t < 0.62 ? 0.17 : 0.1),
        speed: 0.35,
        opacity: 0.75,
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
    const rpmOf = (th) => 700 + th * 2000;
    const animRate = (th) => 0.12 + th * 0.6;

    return {
      root,
      cycleRate: animRate,
      update(state) {
        const theta = state.cycle * 2 * TAU; // crank angle, 2 turns per cycle
        crank.rotation.x = theta;
        prop.rotation.x = theta;
        ringSpin.rotation.x = theta;
        altPulley.rotation.x = theta * (0.28 / 0.05);
        cam.rotation.x = -state.cycle * TAU; // gear-driven: opposite sense, half speed

        const assembled = (state.explode || 0) < 0.05;
        const rate = animRate(state.throttle);
        const vRate = rate * 2;

        CYL.forEach((c, k) => {
          const local = wrap(state.cycle - c.off);
          const a = theta + c.throw;
          const py = -CR * Math.sin(a);
          const pz = CR * Math.cos(a);
          const zp = pz + c.s * Math.sqrt(ROD * ROD - py * py);
          pistons[k].position.set(c.x, 0, zp);
          rods[k].position.set(c.x, py, pz);
          rods[k].rotation.x = Math.atan2(py, zp - pz);

          // valvetrain: cam lift -> tappet -> pushrod (above) -> rocker -> valve
          const li = lift(local, IN_WIN[0], IN_WIN[1]);
          const le = lift(local, EX_WIN[0], EX_WIN[1]);
          [li, le].forEach((l, j) => {
            const vi = k * 2 + j;
            tappets[vi].position.z = c.s * (0.095 + 0.03 * l);
            pushrods[vi].position.z = c.s * (0.145 + 0.03 * l);
            rockers[vi].rotation.x = c.s * 0.2 * l;
            valveObjs[vi].position.z = c.s * (HEAD_Z - 0.035 * l);
          });

          // gas charge between crown and head
          const crown = Math.abs(zp) + CROWN;
          const ch = charges[k];
          ch.position.z = (c.s * (crown + HEAD_Z)) / 2;
          ch.scale.z = Math.max(0.001, HEAD_Z - crown);
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
            const f = (local - 0.75) / 0.25;
            cA.copy(C.exh);
            op = lerp(0.35, 0.08, f);
          }
          ch.material.color.copy(cA);
          ch.material.opacity = op;
          ch.visible = assembled;

          // dual ignition: both plugs fire together ~25° before TDC
          const sparkOn = assembled && local > 0.465 && local < 0.51;
          for (const sp of sparks[k]) {
            sp.visible = sparkOn;
            sp.scale.setScalar(sparkOn ? 0.6 + Math.random() * 0.6 : 1);
          }
          const flash = local > 0.49 && local < 0.62 ? 1 - (local - 0.49) / 0.13 : 0;
          sparkLights[k].intensity = assembled ? flash * (1.5 + state.throttle * 3) : 0;

          intakeFlows[k].object.visible = exhaustFlows[k].object.visible = assembled;
          intakeFlows[k].material.opacity = clamp(li * 1.6, 0, 0.9);
          intakeFlows[k].update(state.dt, li * vRate);
          exhaustFlows[k].material.opacity = clamp(le * 1.6, 0, 0.9);
          exhaustFlows[k].update(state.dt, le * vRate);
          coolFlows[k].object.visible = assembled;
          coolFlows[k].update(state.dt, 0.4 + state.throttle * 1.2);
        });

        blur.visible = assembled;
        blur.material.opacity = Math.max(0, state.throttle - 0.5) * 0.08; // faint disc only at high rpm
      },
      readout(state) {
        const rpm = rpmOf(state.throttle);
        const kw = 134 * Math.pow(rpm / 2700, 3); // fixed-pitch propeller load
        const tip = (Math.PI * 1.88 * rpm) / 60;
        return [
          { label: 'Giri motore = elica', value: Math.round(rpm / 10) * 10, unit: 'giri/min' },
          { label: 'Potenza assorbita', value: Math.round(kw), unit: 'kW' },
          { label: 'Velocità punta pala', value: Math.round(tip), unit: 'm/s' },
          { label: 'Mach punta pala', value: (tip / 340).toFixed(2).replace('.', ','), unit: '' },
        ];
      },
    };
  },
};
