// Electric train traction drive: catenary -> pantograph -> converter -> VVVF inverter ->
// 3-phase asynchronous (induction) motor -> gearbox -> wheelset in a bogie, on rails.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, pipe, flow, clamp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const OFFSET = new THREE.Vector3(0.45, -0.95, 0); // centres the model (z must stay 0 for the cutaway)
const X0 = -0.06;          // motor centre along X
const YM = 0.515;          // motor axis height (wheelset axle at y = 0, z = 0)
const Z_S = -1.25;         // depth of the carbody equipment (behind the motor)
const Z_PINION = 17;
const Z_GEAR = 60;
const RATIO = Z_GEAR / Z_PINION;                     // ~3.53
const R_GEAR = (YM * Z_GEAR) / (Z_GEAR + Z_PINION);  // pitch radii from centre distance
const R_PINION = YM - R_GEAR;
const MODULE = (2 * R_GEAR) / Z_GEAR;
const GEAR_X = 0.64;
const GAUGE_X = 0.88;      // wheel / rail centre
const BOX_X = 1.14;        // axle boxes and bogie side frames
const WHEEL_R = 0.5;
const RAIL_TOP = -0.5;
const TX = -0.25;          // transformer / pantograph / contact wire centre
const IX = 0.85;           // inverter centre
const FLOOR_Y = 1.27;      // top of the carbody floor
const WIRE_Y = 3.0;        // contact wire height
const MESS_Y = 3.32;       // messenger wire height at the supports
const MAST_X = -2.05;
const MAST_Z = [-2.35, 0.95];
const MAX_RPM = 6000;
const POLE_PAIRS = 2;
const WHEEL_D = 0.92;      // m, real wheel diameter for the readout
const CURRENT = 0xd8ff4a;
const FIELD = 0xff4ad8;

// ---------- small builders ----------
const alongX = (g) => g.rotateZ(-Math.PI / 2); // lathe/cylinder axis Y -> X
const UP = new THREE.Vector3(0, 1, 0);
const V = (p) => new THREE.Vector3(...p);

function rod(a, b, r, mat, seg = 10) {
  const va = V(a);
  const dir = V(b).sub(va);
  const len = dir.length();
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, seg), mat);
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

function tube(points, r, mat, seg = 32) {
  const curve = new THREE.CatmullRomCurve3(points.map(V), false, 'centripetal');
  return new THREE.Mesh(new THREE.TubeGeometry(curve, seg, r, 6, false), mat);
}

// Instanced hex bolt heads. items: [[x,y,z],[nx,ny,nz]] (position, outward normal).
const HEX = new THREE.CylinderGeometry(0.011, 0.011, 0.01, 6);
function bolts(mat, items, geo = HEX) {
  const im = new THREE.InstancedMesh(geo, mat, items.length);
  const m4 = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const one = new THREE.Vector3(1, 1, 1);
  items.forEach(([p, n], i) => {
    q.setFromUnitVectors(UP, V(n).normalize());
    m4.compose(V(p), q, one);
    im.setMatrixAt(i, m4);
  });
  im.instanceMatrix.needsUpdate = true;
  return im;
}
const boltRingX = (x, r, n, sx, a0 = 0) =>
  Array.from({ length: n }, (_, i) => {
    const a = a0 + (i / n) * TAU;
    return [[x, Math.cos(a) * r, Math.sin(a) * r], [sx, 0, 0]];
  });

// Closed lathe profile of a laminated ring: zig-zag outer (and inner) surface = stacked sheets.
function ribbedProfile(ri, ro, len, n, dOut, dIn) {
  const h = len / 2;
  const pts = [];
  for (let i = 0; i <= n; i++) pts.push([ro - (i % 2) * dOut, -h + (i / n) * len]);
  for (let i = n; i >= 0; i--) pts.push([ri + (i % 2) * dIn, -h + (i / n) * len]);
  pts.push([ro, -h]);
  return pts;
}

// Shape in (u,v) -> extrusion along X centred on 0, with v -> y and u -> -z.
function extrudeX(shape, depth, opts = {}) {
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: 12, ...opts });
  g.translate(0, 0, -depth / 2);
  g.rotateY(Math.PI / 2);
  return g;
}

// Spur gear around the X axis; tooth k sits at polar angle 2*PI*k/N in the YZ plane.
function gearGeo(teeth, rp, width, holeR = 0, holeCount = 0, holeRing = 0) {
  const ra = rp + MODULE;
  const rf = rp - 1.25 * MODULE;
  const hp = Math.PI / teeth;
  const s = new THREE.Shape();
  const prof = [[rf, -0.62], [ra, -0.3], [ra, 0.3], [rf, 0.62]];
  for (let k = 0; k < teeth; k++) {
    const c = (TAU * k) / teeth + Math.PI / 2; // +PI/2 compensates rotateY in extrudeX
    prof.forEach(([r, f], j) => {
      const a = c + f * hp;
      if (k === 0 && j === 0) s.moveTo(r * Math.cos(a), r * Math.sin(a));
      else s.lineTo(r * Math.cos(a), r * Math.sin(a));
    });
  }
  s.closePath();
  for (let i = 0; i < holeCount; i++) {
    const a = (TAU * i) / holeCount;
    s.holes.push(new THREE.Path().absarc(Math.cos(a) * holeRing, Math.sin(a) * holeRing, holeR, 0, TAU, true));
  }
  return extrudeX(s, width);
}

// Hull of the gear circle (centre 0,0) and the pinion circle (centre 0,YM).
function drawHull(p, r1, r2) {
  const b = Math.asin((r1 - r2) / YM);
  p.moveTo(r2 * Math.cos(b), YM + r2 * Math.sin(b));
  p.absarc(0, YM, r2, b, Math.PI - b, false);
  p.absarc(0, 0, r1, Math.PI - b, TAU + b, false);
  p.closePath();
  return p;
}

// Ribbed ceramic insulator standing on y = 0.
function insulatorGeo(h = 0.12, r = 0.032, sheds = 3) {
  const pts = [[0, 0], [r * 0.7, 0]];
  for (let i = 0; i < sheds; i++) {
    const y0 = (h / sheds) * i;
    const s = h / sheds;
    pts.push([r * 0.7, y0 + 0.2 * s], [r * 1.45, y0 + 0.45 * s], [r * 0.7, y0 + s]);
  }
  pts.push([0, h]);
  return lathe(pts, 16);
}
function insulatorBetween(a, b, r, mat) {
  const va = V(a);
  const dir = V(b).sub(va);
  const m = new THREE.Mesh(insulatorGeo(dir.length(), r, 5), mat);
  m.position.copy(va);
  m.quaternion.setFromUnitVectors(UP, dir.normalize());
  return m;
}

// Helical coil spring along Y, base at y = 0.
function springGeo(radius, wire, height, turns) {
  const pts = [];
  const n = turns * 24;
  for (let i = 0; i <= n; i++) {
    const a = (i / 24) * TAU;
    pts.push(new THREE.Vector3(Math.cos(a) * radius, wire + (i / n) * (height - 2 * wire), Math.sin(a) * radius));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), n * 2, wire, 6, false);
}

// Polyline path with uniform speed, t in [0,1].
function polyPath(list) {
  const v = list.map(V);
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

// Flow points are added to the (identity) root, so paths include the model offset.
const shifted = (fn) => (t, out) => fn(t, out).add(OFFSET);

function glowMat(base) {
  base.emissive.setHex(CURRENT);
  base.emissiveIntensity = 0;
  return base;
}

// Contact wire zig-zag (stagger) between the two masts, messenger sag.
const wireX = (z) => TX + 0.1 * ((2 * (z - MAST_Z[0])) / (MAST_Z[1] - MAST_Z[0]) - 1);
const messY = (z) => {
  const u = (z - (MAST_Z[0] + MAST_Z[1]) / 2) / ((MAST_Z[1] - MAST_Z[0]) / 2);
  return MESS_Y - 0.12 * (1 - u * u);
};

// Generic motor rating plate (no brand).
function plateTexture() {
  const c = document.createElement('canvas');
  c.width = 256;
  c.height = 112;
  const g = c.getContext('2d');
  g.fillStyle = '#c9ccd0';
  g.fillRect(0, 0, 256, 112);
  g.strokeStyle = '#3b4148';
  g.lineWidth = 4;
  g.strokeRect(5, 5, 246, 102);
  g.fillStyle = '#1d2228';
  g.font = 'bold 17px sans-serif';
  g.fillText('MOTORE DI TRAZIONE 3~', 16, 32);
  g.font = '15px sans-serif';
  g.fillText('Asincrono · 4 poli · 610 kW', 16, 58);
  g.fillText('6000 giri/min · 2,2 kV', 16, 80);
  g.fillText('Isolamento cl. 200 · IP20', 16, 100);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// Wheel cross-section (r, axial), +axial = outer side, flange on the inner side.
const WHEEL_PROFILE = [
  [0.085, 0.09], [0.085, -0.1], [0.15, -0.1], [0.17, -0.07], [0.25, -0.03], [0.38, 0.0],
  [0.42, -0.02], [0.43, -0.065], [0.53, -0.065], [0.545, -0.045], [0.53, -0.025],
  [0.5, -0.015], [0.49, 0.07], [0.48, 0.075], [0.43, 0.075], [0.4, 0.05], [0.3, 0.04],
  [0.18, 0.06], [0.15, 0.09], [0.085, 0.09],
];

// End shield cross-section (r, axial), axial 0 = housing end, + = outward.
const SHIELD_PROFILE = [
  [0.09, -0.01], [0.14, -0.01], [0.32, 0.0], [0.37, 0.0], [0.37, 0.035], [0.33, 0.045],
  [0.17, 0.08], [0.125, 0.115], [0.09, 0.115], [0.09, -0.01],
];

export default {
  id: 'electricTrain',
  name: 'Motore elettrico di trazione ferroviaria',
  short: 'Treno elettrico',
  family: 'mare',
  vehicle: 'Treno elettrico',
  examples: 'Frecciarossa 1000, Italo, Caravaggio/Rock regionali',
  tagline: 'Un campo magnetico che gira, senza toccare nulla, spinge un treno a 300 km/h.',
  intro: {
    base:
      "Il treno elettrico non si porta dietro il carburante: prende l'energia da un filo teso sopra i binari, la <b>catenaria</b>, attraverso il <b>pantografo</b>. " +
      "A bordo l'elettronica di potenza trasforma questa energia in tre correnti alternate che fanno <b>girare un campo magnetico</b> dentro il motore. " +
      'Il rotore viene trascinato da questo campo invisibile, come una foglia presa in un vortice, e tramite gli ingranaggi fa girare le ruote. ' +
      "In frenata il motore diventa una dinamo e <b>restituisce energia</b> alla linea.",
    expert:
      'Motore <b>asincrono trifase a gabbia di scoiattolo</b> alimentato da un <b>inverter VVVF</b> (tensione e frequenza variabili) a IGBT o SiC. ' +
      "In Italia la linea fornisce 3 kV c.c. sulla rete tradizionale e 25 kV 50 Hz c.a. sull'Alta Velocità: trasformatore e raddrizzatore 4QS alimentano un circuito intermedio in continua (DC link) da cui l'inverter sintetizza la terna trifase in PWM. " +
      'Il campo statorico ruota alla velocità di sincronismo n<sub>s</sub> = 60·f/p e il rotore lo insegue con uno scorrimento del 2-4%. ' +
      'Rendimento del motore ~95-96%; in frenatura a recupero il flusso di energia si inverte verso la linea.',
  },
  specs: [
    { label: 'Alimentazione', value: '3 kV c.c. / 25 kV 50 Hz c.a.' },
    { label: 'Potenza per motore', value: '≈ 610 kW (ETR 1000: 16 motori, 9,8 MW)' },
    { label: 'Giri max', value: '≈ 6000 rpm' },
    { label: 'Rendimento', value: '95-96 % (motore)' },
    { label: 'Peso motore', value: 'circa 600-700 kg' },
    { label: 'Raffreddamento', value: 'Aria, ventilazione forzata' },
    { label: 'Frenatura', value: 'Elettrica a recupero + dischi' },
  ],
  metrics: { efficiency: 95, power: 610, powerDensity: 0.95 },
  variants: {
    columns: ['Treno / locomotiva', 'Motori di trazione', 'Potenza totale'],
    rows: [
      ['Frecciarossa 1000 (ETR 400)', 'Asincroni trifase, 16 motori', '9,8 MW'],
      ['Pendolino ETR 600 / ETR 610', 'Asincroni trifase', '5,5 MW'],
      ['Siemens Vectron AC (locomotiva)', 'Asincroni trifase, 4 motori', '6,4 MW'],
      ['Siemens Vectron DC (locomotiva)', 'Asincroni trifase, 4 motori', '5,2 MW'],
      ['Stadler FLIRT (regionale)', 'Asincroni trifase', 'varia con la versione'],
    ],
    note: 'Alcuni treni, come l’AGV di Italo, usano invece motori sincroni a magneti permanenti.',
  },
  thumb: { orbit: [20, 10], zoom: 1.35, view: 'section' },
  phases: [
    {
      id: 'captazione',
      name: 'Captazione energia',
      parts: ['catenaria', 'pantografo'],
      base:
        "Sopra i binari corre un filo di rame sempre sotto tensione: la catenaria. Il pantografo, il braccio snodato sul tetto, lo tocca con una striscia di carbone spinta verso l'alto da una molla ad aria: così il treno «beve» energia anche correndo a 300 km/h. " +
        'La corrente poi torna indietro passando per le ruote e le rotaie.',
      expert:
        'Linee tradizionali italiane: 3 kV in corrente continua; linee AV/AC: 25 kV 50 Hz in corrente alternata (i treni AV sono politensione). ' +
        'Il filo di contatto è sospeso alla fune portante tramite pendini e, grazie ai bracci di poligonazione sulle mensole, corre a zig-zag per consumare in modo uniforme gli striscianti in grafite. ' +
        "Un attuatore pneumatico mantiene la forza di contatto (circa 70-120 N) stabile; il circuito si chiude attraverso ruote, rotaie e sottostazioni elettriche.",
    },
    {
      id: 'conversione',
      name: 'Conversione di potenza',
      parts: ['trasformatore', 'inverter', 'cavi'],
      base:
        "L'energia della linea non è adatta al motore così com'è. Prima viene «addomesticata» (abbassata e resa continua) dal trasformatore e dal convertitore; " +
        "poi l'inverter, un interruttore elettronico velocissimo, la accende e spegne migliaia di volte al secondo e ricompone tre correnti alternate, una per cavo, in ritardo l'una sull'altra. " +
        'Cambiando quanto velocemente si alternano, il macchinista decide la velocità del treno.',
      expert:
        "A 25 kV c.a. il trasformatore principale abbassa la tensione e un raddrizzatore attivo 4QS alimenta il DC link (~3-3,6 kV); a 3 kV c.c. la linea alimenta il DC link tramite filtro LC. " +
        "L'inverter (moduli IGBT 3,3-6,5 kV, oggi anche SiC) genera la terna trifase con PWM a tensione e frequenza variabili (VVVF): fino alla velocità base tiene U/f ≈ costante (flusso e coppia costanti), " +
        'oltre lavora a tensione massima in deflussaggio (potenza costante). Frequenza di uscita da pochi Hz a ~200 Hz.',
    },
    {
      id: 'campo',
      name: 'Campo magnetico rotante',
      parts: ['statore', 'avvolgimenti'],
      base:
        "Nello statore, la parte ferma del motore, ci sono tre gruppi di bobine di rame, uno per ogni cavo. Le tre correnti arrivano una dopo l'altra, come tre persone che spingono a turno un'altalena: " +
        "le bobine si «accendono» in sequenza (in verde) e creano un campo magnetico che gira intorno all'asse, anche se nulla si muove davvero. In vista Sezione è il campo magenta.",
      expert:
        'Tre avvolgimenti sfasati di 120° nello spazio, percorsi da correnti sfasate di 120° nel tempo, producono una forza magnetomotrice di ampiezza costante che ruota alla velocità di sincronismo n_s = 60·f/p ' +
        '(p = coppie polari: con p = 2 e f = 200 Hz, n_s = 6000 rpm). Il nucleo è un pacco di lamierini di acciaio al silicio isolati, per ridurre le perdite per correnti parassite e isteresi. ' +
        "(Nell'animazione il campo è semplificato a 2 poli.)",
    },
    {
      id: 'induzione',
      name: 'Induzione e coppia',
      parts: ['rotore', 'albero'],
      base:
        'Il rotore non ha fili collegati: è una «gabbia di scoiattolo» di barre di rame chiuse da due anelli. Il campo che gira «striscia» sulle barre e vi fa nascere delle correnti (le barre si illuminano); ' +
        'queste correnti, immerse nel campo, vengono spinte e il rotore si mette a inseguire il campo. Non lo raggiunge mai del tutto: se lo raggiungesse, non ci sarebbe più nulla che striscia e la spinta sparirebbe.',
      expert:
        'Lo scorrimento s = (n_s − n)/n_s, tipicamente 2-4% a carico, fa sì che le barre vedano un campo variabile a frequenza s·f: per Faraday-Lenz si induce una f.e.m. e quindi una corrente nelle barre, ' +
        "e la forza di Lorentz (F = I·L×B) produce la coppia. La coppia cresce con lo scorrimento fino alla coppia massima; l'inverter regola direttamente coppia e flusso (controllo vettoriale o DTC). " +
        'Niente spazzole né collettore: grande robustezza e manutenzione minima.',
    },
    {
      id: 'trasmissione',
      name: 'Trasmissione alla ruota',
      parts: ['riduttore', 'cassaRiduttore', 'sala', 'carrello', 'binario'],
      base:
        "Il motore gira molto veloce ma con poca forza; il riduttore, un ingranaggio piccolo che spinge una ruota dentata grande, rallenta la rotazione di circa 3,5 volte e moltiplica la forza. " +
        "La ruota dentata grande è montata sull'asse della sala, cioè due ruote d'acciaio unite rigidamente, guidata dal telaio del carrello. Acciaio su acciaio fa pochissimo attrito: il treno consuma poco, ma la spinta va dosata per non far slittare le ruote.",
      expert:
        'Rapporto di riduzione ~1:3,5 (es. 17/60 denti, ingranaggi elicoidali in bagno d\'olio, rendimento ~98%): a 6000 rpm del motore la sala gira a ~1700 rpm e con ruote da 920 mm il treno viaggia a ~300 km/h. ' +
        "Lo sforzo di trazione è limitato dall'aderenza ruota-rotaia (μ ≈ 0,25-0,35 su rotaia asciutta): l'antislittamento corregge la coppia dell'inverter in pochi millisecondi. " +
        'In frenatura a recupero il motore lavora da generatore (scorrimento negativo) e restituisce energia alla linea o ai servizi di bordo.',
    },
  ],
  quiz: [
    {
      q: 'Che cosa fa girare il rotore di un motore asincrono?',
      options: [
        'Delle spazzole che gli portano la corrente',
        'Un campo magnetico rotante che induce correnti nelle sue barre',
        'Dei magneti permanenti incollati sul rotore',
        "Il flusso dell'aria di raffreddamento",
      ],
      correct: 1,
      explain:
        'Lo statore crea un campo magnetico che gira: «strisciando» sulle barre della gabbia vi induce correnti, e la forza tra correnti e campo trascina il rotore. Nessun contatto elettrico con il rotore.',
    },
    {
      q: 'Perché questo motore si chiama «asincrono»?',
      options: [
        'Perché il rotore gira un po\' più lento del campo magnetico (scorrimento)',
        'Perché le tre fasi non sono sincronizzate tra loro',
        'Perché funziona solo in corrente continua',
        'Perché si avvia sempre con qualche secondo di ritardo',
      ],
      correct: 0,
      explain:
        'Il rotore resta sempre leggermente indietro rispetto al campo (2-4%): proprio questa differenza fa nascere le correnti nelle barre e quindi la coppia. Se fosse «sincrono», la coppia sparirebbe.',
    },
    {
      q: "A che cosa serve l'inverter su un treno elettrico?",
      options: [
        'A raffreddare il motore di trazione',
        'A cambiare marcia nel riduttore',
        'A creare tre correnti alternate di frequenza e tensione variabili per regolare la velocità del motore',
        'A sollevare e abbassare il pantografo',
      ],
      correct: 2,
      explain:
        "Il motore asincrono gira a una velocità legata alla frequenza delle correnti: l'inverter VVVF varia frequenza e tensione e così controlla velocità e coppia, dall'avvio fino a 300 km/h.",
    },
    {
      q: 'Che cosa succede nella frenatura a recupero?',
      options: [
        'Il motore si spegne e frenano solo i dischi',
        'Il pantografo si abbassa per evitare scintille',
        'Il riduttore si sgancia dalle ruote',
        'Il motore lavora da generatore e restituisce energia alla linea',
      ],
      correct: 3,
      explain:
        "L'inverter porta la frequenza del campo sotto la velocità del rotore (scorrimento negativo): il motore diventa un generatore, frena il treno e rimanda energia in catenaria per altri treni.",
    },
  ],
  camera: { position: [5.0, 2.7, 6.4], target: [-0.1, 0.35, -0.45] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.copy(OFFSET);
    root.add(model);
    const add = (obj, opts) => model.add(ctx.part(obj, opts));

    // shared materials
    const boltMat = MAT.darkSteel();
    const boltCut = cut(MAT.darkSteel(), { cap: false });
    const rubber = MAT.rubber();
    const ceramic = MAT.ceramic();
    const frameMat = MAT.paint(0x30353b, 0.55);   // bogie
    const alu = MAT.aluminum();

    // ================= Track =================
    const track = new THREE.Group();
    const railShape = new THREE.Shape();
    [
      [-0.065, 0], [0.065, 0], [0.065, 0.012], [0.015, 0.024], [0.011, 0.09], [0.034, 0.098],
      [0.035, 0.143], [-0.035, 0.143], [-0.034, 0.098], [-0.011, 0.09], [-0.015, 0.024], [-0.065, 0.012],
    ].forEach(([u, v], i) => (i ? railShape.lineTo(u, v) : railShape.moveTo(u, v)));
    railShape.closePath();
    const railGeo = new THREE.ExtrudeGeometry(railShape, { depth: 3.4, bevelEnabled: false });
    railGeo.translate(0, RAIL_TOP - 0.143, -1.7);
    const railMat = MAT.darkSteel();
    const headMat = MAT.chrome();
    for (const sx of [-1, 1]) {
      const r = new THREE.Mesh(railGeo, railMat);
      r.position.x = sx * GAUGE_X;
      track.add(r);
      track.add(box(0.045, 0.004, 3.4, headMat, sx * GAUGE_X, RAIL_TOP + 0.001, 0));
    }
    track.add(box(2.9, 0.1, 3.9, MAT.paint(0x5e5a53, 0.95), 0, -0.8, 0));                 // ballast
    track.add(box(4.9, 0.06, 3.9, MAT.paint(0x4a463f, 0.95), -0.35, -0.88, 0));           // formation
    const concrete = MAT.paint(0x9b978e, 0.9);
    for (const mz of MAST_Z) track.add(rbox(0.36, 0.2, 0.36, 0.03, concrete, MAST_X, -0.75, mz)); // mast foundations
    const sleepers = [];
    const clipMat = MAT.darkSteel();
    for (let i = 0; i < 6; i++) {
      const g = new THREE.Group();
      g.add(box(2.5, 0.12, 0.24, concrete, 0, RAIL_TOP - 0.143 - 0.06, 0));
      for (const sx of [-1, 1]) {
        for (const o of [-1, 1]) {
          g.add(box(0.04, 0.03, 0.1, clipMat, sx * GAUGE_X + o * 0.085, RAIL_TOP - 0.138, 0));
        }
      }
      g.userData.z0 = -1.5 + i * 0.6;
      sleepers.push(g);
      track.add(g);
    }
    add(track, {
      id: 'binario',
      name: 'Binario',
      base: "Due rotaie d'acciaio fissate alle traversine. Acciaio contro acciaio scivola poco: il treno consuma poco, ma la spinta va dosata per non slittare.",
      expert:
        'Rotaia 60 E1 (60 kg/m) su traversine in calcestruzzo ogni ~60 cm, scartamento 1435 mm. Il contatto ruota-rotaia è grande circa come un\'unghia (~1 cm²): l\'aderenza disponibile (μ ≈ 0,25-0,35) limita lo sforzo di trazione.',
      explode: [0, -0.45, 0],
    });

    // ================= Wheelset (axle, wheels, discs, axle boxes) =================
    const sala = new THREE.Group();
    const salaSpin = new THREE.Group();
    sala.add(salaSpin);
    salaSpin.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.085, 0.085, 2.3, 32)), MAT.darkSteel()));
    const wheelGeo = alongX(lathe(WHEEL_PROFILE, 72));
    const wheelMat = MAT.steel();
    const discGeo = alongX(pipe(0.37, 0.2, 0.035, 48));
    const discMat = MAT.darkSteel();
    const chrome = MAT.chrome();
    const discBolts = boltRingX(0.1, 0.285, 10, 1);
    for (const sx of [-1, 1]) {
      const w = new THREE.Group();
      w.add(new THREE.Mesh(wheelGeo, wheelMat));
      const disc = new THREE.Mesh(discGeo, discMat);
      disc.position.x = 0.078;
      w.add(disc, bolts(chrome, discBolts));
      w.position.x = sx * GAUGE_X;
      if (sx < 0) w.rotation.y = Math.PI; // mirror by rotation (keeps winding)
      salaSpin.add(w);
    }
    const boxMat = MAT.castIron();
    const coverMat = MAT.paint(0x2b2f35, 0.6);
    for (const sx of [-1, 1]) {
      const ab = new THREE.Group();
      ab.position.x = sx * BOX_X;
      ab.add(new THREE.Mesh(alongX(lathe([[0.0, -0.08], [0.125, -0.08], [0.135, -0.06], [0.135, 0.06], [0.11, 0.08], [0.0, 0.08]], 40)), boxMat));
      const cover = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.1, 0.105, 0.03, 32)), coverMat);
      cover.position.x = sx * 0.095;
      ab.add(cover);
      ab.add(bolts(boltMat, boltRingX(sx * 0.112, 0.08, 6, sx)));
      ab.add(rbox(0.13, 0.05, 0.6, 0.015, boxMat, 0, 0.1, 0));      // spring wings
      ab.add(rbox(0.06, 0.09, 0.08, 0.012, boxMat, 0, 0.04, 0.32)); // damper lug
      for (const z of [-0.2, 0.2]) ab.add(new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.012, 20), coverMat).translateY(0.131).translateZ(z));
      // earth-return brush housing with cable
      const brush = rbox(0.05, 0.07, 0.07, 0.01, coverMat, sx * 0.125, 0.0, 0);
      ab.add(brush);
      sala.add(ab);
    }
    add(sala, {
      id: 'sala',
      name: 'Sala montata (asse, ruote, boccole)',
      base:
        "Due ruote d'acciaio fissate rigidamente sullo stesso asse. Il bordino, il rilievo sul lato interno della ruota, la tiene sul binario; la superficie leggermente conica la aiuta a centrarsi e a seguire le curve. Alle estremità, le boccole contengono i cuscinetti dell'asse.",
      expert:
        'Ruote monoblocco calettate a pressione, diametro a nuovo ~920 mm, profilo conico che garantisce l\'autocentraggio. Sul velo delle ruote sono imbullonati i dischi freno; le boccole con cuscinetti a rulli poggiano sulle molle della sospensione primaria e portano il dispositivo di ritorno della corrente a terra.',
      explode: [0, -0.2, 0],
    });

    // ================= Bogie frame, suspension, brakes, motor mounting =================
    const bogie = new THREE.Group();
    const sideShape = new THREE.Shape();
    [[0.75, 0.6], [-0.35, 0.6], [-0.62, 0.46], [-1.6, 0.46], [-1.6, 0.12], [-0.55, 0.12], [-0.3, 0.42], [0.6, 0.42], [0.75, 0.5]]
      .forEach(([z, y], i) => (i ? sideShape.lineTo(-z, y) : sideShape.moveTo(-z, y)));
    sideShape.closePath();
    const sideGeo = extrudeX(sideShape, 0.11, { bevelEnabled: true, bevelSize: 0.015, bevelThickness: 0.015, bevelSegments: 2 });
    for (const sx of [-1, 1]) {
      const sf = new THREE.Mesh(sideGeo, frameMat);
      sf.position.x = sx * BOX_X;
      bogie.add(sf);
    }
    // transverse beam: two tubes
    const beamGeo = alongX(new THREE.CylinderGeometry(0.1, 0.1, 2.2, 32));
    for (const z of [-0.93, -1.17]) bogie.add(new THREE.Mesh(beamGeo, frameMat).translateY(0.3).translateZ(z));
    // primary springs and dampers
    const sprGeo = springGeo(0.055, 0.012, 0.29, 5);
    const sprMat = MAT.paint(0x8a2a22, 0.5);
    const dampMat = MAT.paint(0x1f2a36, 0.45);
    for (const sx of [-1, 1]) {
      for (const z of [-0.2, 0.2]) {
        const s = new THREE.Mesh(sprGeo, sprMat);
        s.position.set(sx * BOX_X, 0.13, z);
        bogie.add(s);
      }
      bogie.add(rod([sx * BOX_X, 0.07, 0.32], [sx * BOX_X, 0.26, 0.37], 0.035, dampMat, 16));
      bogie.add(rod([sx * BOX_X, 0.26, 0.37], [sx * BOX_X, 0.42, 0.41], 0.013, chrome, 10));
      bogie.add(rbox(0.07, 0.04, 0.08, 0.01, frameMat, sx * BOX_X, 0.41, 0.41));
    }
    // brake calipers on the wheel discs (rear, horizontal), hung from the side frames
    const calMat = MAT.paint(0x56606a, 0.5);
    const padMat = MAT.carbon();
    for (const sx of [-1, 1]) {
      const g = new THREE.Group();
      g.add(rbox(0.08, 0.17, 0.11, 0.02, calMat, 1.015, 0.0, -0.29));
      g.add(box(0.012, 0.13, 0.075, padMat, 0.981, 0.0, -0.29));
      g.add(rbox(0.06, 0.36, 0.06, 0.012, calMat, 1.03, 0.25, -0.29));
      g.add(rbox(0.12, 0.04, 0.09, 0.01, calMat, 1.07, 0.41, -0.29));
      g.add(new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 0.11, 20).rotateX(Math.PI / 2), calMat).translateX(1.015).translateZ(-0.4));
      g.add(tube([[1.015, 0.03, -0.46], [1.02, 0.1, -0.55], [1.06, 0.2, -0.6], [1.08, 0.25, -0.62]], 0.011, rubber, 16));
      g.scale.x = sx; // mirrored for the left side (renderer handles negative scale)
      bogie.add(g);
    }
    // motor suspension arms from the motor lugs to the transverse beam
    for (const dx of [-0.3, 0.3]) {
      bogie.add(rbox(0.1, 0.12, 0.42, 0.02, frameMat, X0 + dx, 0.41, -0.62));
      bogie.add(new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.12, 20).rotateZ(Math.PI / 2), rubber).translateX(X0 + dx).translateY(0.41).translateZ(-0.45));
    }
    // gearbox torque-reaction link with rubber-bushed eyes
    bogie.add(rod([GEAR_X, 0.12, -0.42], [GEAR_X, 0.24, -0.84], 0.018, chrome, 10));
    for (const [y, z] of [[0.12, -0.42], [0.24, -0.84]]) {
      bogie.add(new THREE.Mesh(alongX(pipe(0.04, 0.02, 0.05, 20)), rubber).translateX(GEAR_X).translateY(y).translateZ(z));
    }
    add(bogie, {
      id: 'carrello',
      name: 'Carrello: telaio, sospensioni e freni',
      base:
        "Il telaio che tiene insieme ruote e motore. Le molle sopra le boccole assorbono le irregolarità del binario, gli ammortizzatori smorzano i sobbalzi e le pinze freno stringono i dischi montati sulle ruote. Il motore è appeso alla trave del carrello con due bracci.",
      expert:
        'Telaio saldato a longheroni e traversa tubolare; sospensione primaria a molle elicoidali con smorzatori verticali, sospensione secondaria pneumatica verso la cassa. Motore sospeso al telaio (massa non sospesa ridotta), riduttore vincolato con una biella di reazione su silent-block. Freno a dischi sulle ruote con pinze pneumatiche, usato soprattutto a bassa velocità e in emergenza.',
      explode: [0, 0.25, -0.6],
    });

    // ================= Gearbox =================
    const gearbox = new THREE.Group();
    const pinion = new THREE.Mesh(gearGeo(Z_PINION, R_PINION, 0.1), MAT.steel());
    pinion.position.set(GEAR_X, YM, 0);
    const gear = new THREE.Mesh(gearGeo(Z_GEAR, R_GEAR, 0.08, 0.055, 6, 0.25), MAT.darkSteel());
    gear.position.set(GEAR_X, 0, 0);
    gear.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.15, 0.15, 0.16, 32)), MAT.darkSteel()));
    gearbox.add(pinion, gear);
    add(gearbox, {
      id: 'riduttore',
      name: 'Riduttore (pignone e corona)',
      base:
        "Un piccolo ingranaggio (pignone) sull'albero del motore fa girare una grande ruota dentata (corona) sull'asse delle ruote: la rotazione rallenta circa 3,5 volte e la forza aumenta altrettanto.",
      expert:
        'Ingranaggi cilindrici a denti elicoidali, qui 17/60 denti (i ≈ 3,53), rendimento ~98%. Coppia alla sala = coppia motore × i × η; la corona gira in verso opposto al pignone.',
      explode: [0, 0, 0.35],
    });

    // Gearbox casing: hollow shell (rim + two side plates) so the section shows wall thickness.
    const casing = new THREE.Group();
    casing.position.x = GEAR_X;
    const casMat = cut(MAT.paint(0x4a5562, 0.6));
    const r1 = R_GEAR + MODULE + 0.045;
    const r2 = R_PINION + MODULE + 0.06;
    const rimShape = drawHull(new THREE.Shape(), r1, r2);
    rimShape.holes.push(drawHull(new THREE.Path(), r1 - 0.03, r2 - 0.03));
    casing.add(new THREE.Mesh(extrudeX(rimShape, 0.18, { curveSegments: 40 }), casMat));
    const plateShape = drawHull(new THREE.Shape(), r1, r2);
    plateShape.holes.push(new THREE.Path().absarc(0, YM, 0.065, 0, TAU, true));
    plateShape.holes.push(new THREE.Path().absarc(0, 0, 0.105, 0, TAU, true));
    const plateGeo = extrudeX(plateShape, 0.02, { curveSegments: 40 });
    for (const sx of [-1, 1]) {
      casing.add(new THREE.Mesh(plateGeo, casMat).translateX(sx * 0.1));
      const sealA = new THREE.Mesh(alongX(pipe(0.105, 0.085, 0.024, 32)), cut(MAT.rubber()));
      sealA.position.set(sx * 0.1, 0, 0);
      const sealP = new THREE.Mesh(alongX(pipe(0.065, 0.045, 0.024, 24)), cut(MAT.rubber()));
      sealP.position.set(sx * 0.1, YM, 0);
      casing.add(sealA, sealP);
    }
    const boltPts = drawHull(new THREE.Shape(), r1 - 0.016, r2 - 0.016).getSpacedPoints(28).slice(0, 28);
    casing.add(bolts(boltCut, boltPts.map((p) => [[0.115, p.y, -p.x], [1, 0, 0]])));
    casing.add(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, 0.06, 12), casMat).translateY(YM + r2 + 0.02).translateZ(-0.05)); // breather
    casing.add(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.03, 6), casMat).translateY(-r1 - 0.005).translateZ(-0.08));   // drain plug
    const glass = new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.03, 0.03, 0.012, 20)), MAT.glow(0xd29a2a, 0.35));
    glass.position.set(0.116, -0.22, -0.2);
    casing.add(glass);
    casing.add(rbox(0.08, 0.08, 0.1, 0.015, casMat, 0, 0.12, -0.39)); // reaction-link lug
    add(casing, {
      id: 'cassaRiduttore',
      name: 'Scatola del riduttore',
      base: "La scatola chiusa che contiene gli ingranaggi e l'olio che li lubrifica. La spia permette di controllare il livello dell'olio.",
      expert:
        "Carter in ghisa o alluminio in due semigusci imbullonati, lubrificazione a sbattimento d'olio, tenute a labirinto sull'asse e sul pignone, sfiato in alto e tappo di scarico in basso. Una biella di reazione sul carrello impedisce al carter di ruotare attorno all'asse.",
      explode: [0.25, 0, 0.95],
    });

    // ================= Motor: housing =================
    const housing = new THREE.Group();
    housing.position.set(X0, YM, 0);
    const hMat = cut(MAT.paint(0x34404d, 0.55));
    const housingShell = new THREE.Mesh(alongX(pipe(0.345, 0.305, 0.9, 64)), hMat);
    housing.add(housingShell);
    const flangeGeo = alongX(pipe(0.37, 0.3, 0.035, 64));
    const gasketGeo = alongX(pipe(0.374, 0.355, 0.008, 64));
    const gasketMat = cut(MAT.rubber());
    for (const sx of [-1, 1]) {
      housing.add(new THREE.Mesh(flangeGeo, hMat).translateX(sx * 0.432));
      housing.add(new THREE.Mesh(gasketGeo, gasketMat).translateX(sx * 0.452));
    }
    const finGeo = new THREE.BoxGeometry(0.8, 0.06, 0.014);
    for (let i = 0; i < 36; i++) {
      const a = (i / 36) * TAU;
      const f = new THREE.Mesh(finGeo, hMat);
      f.position.set(0, Math.cos(a) * 0.372, Math.sin(a) * 0.372);
      f.rotation.x = a;
      housing.add(f);
    }
    // terminal box with lid, bolts, cable glands and rating plate
    const tbMat = MAT.paint(0x2a333d, 0.5);
    housing.add(rbox(0.28, 0.14, 0.24, 0.02, tbMat, 0.2, 0.43, -0.12));
    housing.add(rbox(0.3, 0.02, 0.26, 0.008, tbMat, 0.2, 0.505, -0.12));
    housing.add(bolts(boltMat, [[0.07, -0.23], [0.33, -0.23], [0.07, -0.01], [0.33, -0.01]].map(([x, z]) => [[x, 0.518, z], [0, 1, 0]])));
    const glandGeo = new THREE.CylinderGeometry(0.022, 0.026, 0.04, 12);
    const glandMat = MAT.brass();
    for (let k = 0; k < 3; k++) housing.add(new THREE.Mesh(glandGeo, glandMat).translateX(0.12 + k * 0.08).translateY(0.53).translateZ(-0.12));
    const plate = new THREE.Mesh(new THREE.PlaneGeometry(0.2, 0.0875), new THREE.MeshStandardMaterial({ map: plateTexture(), metalness: 0.4, roughness: 0.45 }));
    plate.position.set(0.2, 0.43, 0.002);
    housing.add(plate);
    // forced-ventilation air inlet with grille
    housing.add(rbox(0.2, 0.2, 0.2, 0.02, tbMat, -0.2, 0.43, -0.17));
    housing.add(rbox(0.24, 0.03, 0.24, 0.01, tbMat, -0.2, 0.545, -0.17));
    for (let i = 0; i < 6; i++) housing.add(box(0.012, 0.012, 0.2, alu, -0.29 + i * 0.036, 0.565, -0.17));
    // lifting eyes on the flanges
    const eyeGeo = new THREE.TorusGeometry(0.03, 0.009, 8, 20);
    for (const sx of [-1, 1]) housing.add(new THREE.Mesh(eyeGeo, hMat).translateX(sx * 0.4).translateY(0.41).translateZ(-0.05).rotateY(Math.PI / 2));
    // suspension lugs (back side, bolted to the bogie arms)
    for (const sx of [-1, 1]) housing.add(rbox(0.12, 0.14, 0.18, 0.02, hMat, sx * 0.3, -0.1, -0.36));
    add(housing, {
      id: 'carcassa',
      name: 'Carcassa, alette e morsettiera',
      base:
        "Il guscio esterno del motore. Le alette aumentano la superficie per smaltire il calore; in alto ci sono la morsettiera con la targhetta, dove entrano i tre cavi, e la presa d'aria del raffreddamento.",
      expert:
        "Carcassa in acciaio saldato, appesa al telaio del carrello con due attacchi. Raffreddamento a ventilazione forzata: un ventilatore in cassa soffia aria attraverso il motore; isolamento in classe 200 °C. Massa tipica ~600-700 kg per 600 kW (≈ 1 kW/kg).",
      explode: [0, 1.0, 0],
    });

    // ================= End shields =================
    const shieldMat = cut(MAT.castIron());
    const shieldGeo = alongX(lathe(SHIELD_PROFILE, 56));
    const shieldBolts = boltRingX(0.04, 0.355, 16, 1, 0.1);
    const shieldDE = new THREE.Group();
    shieldDE.position.set(X0 + 0.45, YM, 0);
    shieldDE.add(new THREE.Mesh(shieldGeo, shieldMat), bolts(boltCut, shieldBolts));
    shieldDE.add(new THREE.Mesh(alongX(pipe(0.11, 0.05, 0.02, 32)), shieldMat).translateX(0.125));
    add(shieldDE, {
      id: 'scudoDE',
      name: 'Scudo lato trasmissione',
      base: 'Il coperchio dal lato degli ingranaggi: chiude il motore e regge il cuscinetto dell\'albero.',
      expert:
        'Scudo in ghisa con sede del cuscinetto (spesso a rulli cilindrici lato trasmissione, per reggere il carico radiale del pignone) e tenute contro polvere e olio.',
      explode: [0.1, 0, 0.8],
    });

    const shieldNDE = new THREE.Group();
    shieldNDE.position.set(X0 - 0.45, YM, 0);
    shieldNDE.rotation.y = Math.PI; // mirrored: +local axial points to -X
    shieldNDE.add(new THREE.Mesh(shieldGeo, shieldMat), bolts(boltCut, shieldBolts));
    const encMat = MAT.paint(0x1f2328, 0.5);
    shieldNDE.add(new THREE.Mesh(alongX(new THREE.CylinderGeometry(0.07, 0.07, 0.05, 24)), encMat).translateX(0.14));
    shieldNDE.add(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.04, 10).rotateX(Math.PI / 2), encMat).translateX(0.15).translateZ(0.075));
    shieldNDE.add(tube([[0.15, 0.0, 0.09], [0.18, -0.04, 0.25], [0.17, -0.18, 0.6], [0.14, -0.215, 0.83]], 0.009, rubber, 24)); // to bogie beam
    add(shieldNDE, {
      id: 'scudoNDE',
      name: 'Scudo lato opposto e sensore di giri',
      base: 'Il coperchio dal lato opposto. Porta il secondo cuscinetto e il sensore che conta i giri del motore, collegato con un cavo.',
      expert:
        "Ospita un cuscinetto isolato e il sensore di velocità (ruota fonica/encoder), indispensabile all'inverter per il controllo vettoriale della coppia e per l'antislittamento.",
      explode: [-0.5, 0, 0],
    });

    // ================= Stator =================
    const stator = new THREE.Group();
    stator.position.set(X0, YM, 0);
    stator.add(new THREE.Mesh(alongX(lathe(ribbedProfile(0.2, 0.302, 0.66, 44, 0.006, 0.003), 72)), cut(MAT.darkSteel())));
    const slotMat = cut(MAT.copper());
    const slotGeo = new THREE.BoxGeometry(0.7, 0.032, 0.02);
    for (let i = 0; i < 36; i++) {
      const a = (i / 36) * TAU;
      const s = new THREE.Mesh(slotGeo, slotMat);
      s.position.set(0, Math.cos(a) * 0.228, Math.sin(a) * 0.228);
      s.rotation.x = a;
      stator.add(s);
    }
    add(stator, {
      id: 'statore',
      name: 'Statore (pacco lamellare)',
      base:
        "La parte ferma del motore: un pacco di sottili dischi d'acciaio con, nelle scanalature (cave), i conduttori di rame. È qui che nasce il campo magnetico.",
      expert:
        'Pacco di lamierini di acciaio al silicio (0,35-0,5 mm) isolati tra loro per limitare le correnti parassite. Le cave ospitano un avvolgimento trifase distribuito, tipicamente a 4 poli (p = 2).',
      explode: [0, 0.55, 0],
    });

    // ================= Windings (end turns, 6 coil groups per side) =================
    const windings = new THREE.Group();
    windings.position.set(X0, YM, 0);
    const ARC = TAU / 6 - 0.05;
    const coilGeo = new THREE.TorusGeometry(0.235, 0.034, 10, 18, ARC);
    coilGeo.rotateY(Math.PI / 2);
    coilGeo.scale(1.5, 1, 1);
    const coils = [];
    for (const sx of [-1, 1]) {
      for (let k = 0; k < 6; k++) {
        const mat = glowMat(MAT.copper());
        mat.side = THREE.DoubleSide;
        const m = new THREE.Mesh(coilGeo, mat);
        const a = (k * TAU) / 6; // belts U+, W-, V+, U-, W+, V-
        m.position.x = sx * 0.385;
        m.rotation.x = a + Math.PI / 2 - ARC / 2;
        windings.add(m);
        coils.push({ mesh: m, a });
      }
    }
    add(windings, {
      id: 'avvolgimenti',
      name: 'Avvolgimenti di rame',
      base:
        "Le bobine di rame: tre gruppi, uno per fase, ripetuti intorno al motore. Si «accendono» a turno (in verde) e così fanno girare il campo magnetico.",
      expert:
        'Teste di bobina dell\'avvolgimento statorico in piattina di rame isolata con nastro mica-vetro e impregnata sottovuoto (VPI). Ogni gruppo occupa una fascia di 60°: la sequenza U+, W−, V+, U−, W+, V− produce il campo rotante.',
      explode: [0, 0.55, 0],
    });

    // ================= Rotor (squirrel cage) =================
    const rotor = new THREE.Group();
    rotor.position.set(X0, YM, 0);
    rotor.add(new THREE.Mesh(alongX(lathe(ribbedProfile(0.06, 0.184, 0.66, 44, 0.004, 0), 64)), MAT.steel()));
    const barGeo = new THREE.BoxGeometry(0.74, 0.02, 0.017);
    const bars = [];
    for (let i = 0; i < 28; i++) {
      const a = (i / 28) * TAU;
      const m = new THREE.Mesh(barGeo, glowMat(MAT.copper()));
      m.position.set(0, Math.cos(a) * 0.177, Math.sin(a) * 0.177);
      m.rotation.x = a;
      rotor.add(m);
      bars.push({ mesh: m, a });
    }
    const ringGeo = alongX(pipe(0.19, 0.125, 0.035, 48));
    const ringMat = MAT.copper();
    for (const sx of [-1, 1]) rotor.add(new THREE.Mesh(ringGeo, ringMat).translateX(sx * 0.355));
    // internal cooling fan (also makes the rotation readable)
    rotor.add(new THREE.Mesh(alongX(pipe(0.17, 0.06, 0.008, 40)), alu).translateX(-0.39));
    const bladeGeo = new THREE.BoxGeometry(0.04, 0.09, 0.008);
    for (let i = 0; i < 10; i++) {
      const a = (i / 10) * TAU;
      const b = new THREE.Mesh(bladeGeo, alu);
      b.position.set(-0.414, Math.cos(a) * 0.12, Math.sin(a) * 0.12);
      b.rotation.x = a;
      rotor.add(b);
    }
    add(rotor, {
      id: 'rotore',
      name: 'Rotore a gabbia di scoiattolo',
      base:
        'La parte che gira. Non ha fili collegati: è una gabbia di barre di rame chiusa da due anelli, simile alla ruota di un criceto. Il campo che gira induce corrente nelle barre (si illuminano) e le trascina con sé.',
      expert:
        'Barre e anelli di cortocircuito in rame brasati su un pacco lamellare calettato sull\'albero. Senza spazzole né collettore: robusto e quasi senza manutenzione. Le correnti rotoriche hanno frequenza s·f, cioè pochi Hz.',
      explode: [-0.75, 0, 0.6],
    });

    // ================= Shaft =================
    const shaft = new THREE.Group();
    shaft.position.set(X0, YM, 0);
    const sA = -0.56;
    const sB = 0.7 - X0;
    shaft.add(
      new THREE.Mesh(
        alongX(lathe([[0, sA], [0.045, sA], [0.045, -0.36], [0.06, -0.35], [0.06, 0.35], [0.045, 0.36], [0.045, sB], [0, sB]], 28)),
        MAT.steel(),
      ),
    );
    add(shaft, {
      id: 'albero',
      name: 'Albero motore',
      base: "L'asse d'acciaio che porta il rotore e passa la rotazione al pignone del riduttore.",
      expert:
        'Albero in acciaio legato bonificato; il pignone è calettato a caldo. Superando i 6000 rpm, rotore e albero sono equilibrati dinamicamente con grande precisione.',
      explode: [-0.75, 0, 0.6],
    });

    // ================= Bearings =================
    const bearings = new THREE.Group();
    bearings.position.set(X0, YM, 0);
    const innerGeo = alongX(pipe(0.062, 0.045, 0.035, 32));
    const outerGeo = alongX(pipe(0.09, 0.076, 0.035, 32));
    const ballGeo = new THREE.SphereGeometry(0.0125, 10, 8);
    const ballSets = [];
    for (const sx of [-1, 1]) {
      bearings.add(new THREE.Mesh(innerGeo, chrome).translateX(sx * 0.5), new THREE.Mesh(outerGeo, chrome).translateX(sx * 0.5));
      const balls = new THREE.Group();
      balls.position.x = sx * 0.5;
      for (let i = 0; i < 10; i++) {
        const a = (i / 10) * TAU;
        balls.add(new THREE.Mesh(ballGeo, chrome).translateY(Math.cos(a) * 0.069).translateZ(Math.sin(a) * 0.069));
      }
      bearings.add(balls);
      ballSets.push(balls);
    }
    add(bearings, {
      id: 'cuscinetti',
      name: 'Cuscinetti',
      base: "Anelli con sfere d'acciaio che permettono all'albero di girare velocissimo con pochissimo attrito.",
      expert:
        "Cuscinetti volventi lubrificati a grasso; uno è isolato (anelli ceramizzati o sfere ceramiche) per bloccare le correnti d'albero causate dalla commutazione PWM, che altrimenti erodono le piste.",
      explode: [-0.75, 0, 0.6],
    });

    // ================= Carbody floor + secondary suspension =================
    const body = new THREE.Group();
    const floorMat = MAT.paint(0x59606a, 0.6);
    body.add(rbox(2.7, 0.06, 0.95, 0.015, floorMat, 0.05, FLOOR_Y - 0.03, -1.225));
    for (const x of [-1.2, 1.3]) body.add(rbox(0.1, 0.12, 0.95, 0.015, floorMat, x, FLOOR_Y - 0.12, -1.225)); // sole bars
    const bellowsGeo = lathe([[0, 0], [0.11, 0], [0.14, 0.04], [0.15, 0.09], [0.14, 0.14], [0.11, 0.18], [0, 0.18]], 32);
    for (const sx of [-1, 1]) {
      body.add(new THREE.Mesh(bellowsGeo, rubber).translateX(sx * BOX_X).translateY(0.46).translateZ(-1.05));
      body.add(new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.02, 32), frameMat).translateX(sx * BOX_X).translateY(0.65).translateZ(-1.05));
      body.add(rbox(0.2, 0.56, 0.24, 0.02, floorMat, sx * BOX_X, 0.93, -1.05));
    }
    add(body, {
      id: 'cassa',
      name: 'Cassa e sospensione secondaria',
      base:
        'Il pavimento della carrozza (qui solo un tratto) poggia sul carrello tramite molle ad aria, che rendono il viaggio morbido. Sotto e sopra la cassa sono montate le apparecchiature elettriche.',
      expert:
        'Sospensione secondaria pneumatica: i soffietti in gomma mantengono costante l\'altezza della cassa al variare del carico. Nei treni a trazione distribuita come il Frecciarossa 1000 trasformatori e convertitori sono sotto il pavimento; qui sono disegnati sopra per chiarezza.',
      explode: [0, 0.15, -0.7],
    });

    // ================= Catenary: masts, cantilevers, wires =================
    const cat = new THREE.Group();
    const mastMat = MAT.paint(0x8d949b, 0.6);
    const tubeMat = MAT.aluminum();
    const brass = MAT.brass();
    const wireMat = MAT.copper();
    const Z0 = -2.75;
    const Z1 = 1.3;
    cat.add(rod([wireX(Z0), WIRE_Y, Z0], [wireX(Z1), WIRE_Y, Z1], 0.012, wireMat, 12));
    const messPts = [];
    for (let z = Z0; z <= Z1 + 1e-6; z += (Z1 - Z0) / 24) messPts.push([TX, messY(z), z]);
    cat.add(tube(messPts, 0.009, brass, 96));
    const clampGeo = new THREE.BoxGeometry(0.02, 0.018, 0.035);
    for (let z = MAST_Z[0] + 0.3; z < MAST_Z[1] - 0.15; z += 0.42) {
      cat.add(rod([wireX(z), WIRE_Y + 0.01, z], [TX, messY(z), z], 0.0035, brass, 6));
      cat.add(new THREE.Mesh(clampGeo, brass).translateX(wireX(z)).translateY(WIRE_Y + 0.012).translateZ(z));
    }
    const mastH = 4.25;
    for (const mz of MAST_Z) {
      // H-section mast on its foundation
      cat.add(box(0.02, mastH, 0.16, mastMat, MAST_X - 0.06, -0.65 + mastH / 2, mz));
      cat.add(box(0.02, mastH, 0.16, mastMat, MAST_X + 0.06, -0.65 + mastH / 2, mz));
      cat.add(box(0.12, mastH, 0.02, mastMat, MAST_X, -0.65 + mastH / 2, mz));
      cat.add(box(0.2, 0.03, 0.2, mastMat, MAST_X, 3.6, mz));
      // cantilever: top tube + stay tube, each with an insulator at the mast
      const top = [[MAST_X + 0.07, 3.4, mz], [MAST_X + 0.37, 3.4, mz]];
      cat.add(insulatorBetween(top[0], top[1], 0.026, ceramic));
      cat.add(rod(top[1], [TX + 0.12, 3.4, mz], 0.018, tubeMat));
      cat.add(rod([TX, 3.4, mz], [TX, messY(mz) + 0.005, mz], 0.008, tubeMat, 8)); // messenger support
      cat.add(new THREE.Mesh(clampGeo, brass).translateX(TX).translateY(messY(mz)).translateZ(mz));
      const L0 = [MAST_X + 0.07, 2.82, mz];
      const D2 = [TX - 0.3, 3.4, mz];
      const dir = V(D2).sub(V(L0)).normalize();
      const L1 = V(L0).addScaledVector(dir, 0.3).toArray();
      cat.add(insulatorBetween(L0, L1, 0.026, ceramic));
      cat.add(rod(L1, D2, 0.018, tubeMat));
      // registration arm + steady arm holding the contact wire in zig-zag
      const tReg = (3.1 - L0[1]) / (D2[1] - L0[1]);
      const xReg = L0[0] + tReg * (D2[0] - L0[0]);
      const xw = wireX(mz);
      cat.add(rod([xReg, 3.1, mz], [xw - 0.3, 3.1, mz], 0.013, tubeMat));
      cat.add(rod([xw - 0.3, 3.1, mz], [xw, WIRE_Y + 0.012, mz], 0.007, tubeMat, 8));
      cat.add(new THREE.Mesh(clampGeo, brass).translateX(xw).translateY(WIRE_Y + 0.012).translateZ(mz));
      // mast brackets
      for (const y of [3.4, 2.82]) cat.add(box(0.06, 0.06, 0.08, mastMat, MAST_X + 0.08, y, mz));
    }
    add(cat, {
      id: 'catenaria',
      name: 'Catenaria (linea aerea)',
      base: 'Il filo di rame teso sopra i binari, sempre sotto tensione: una «presa di corrente» lunga centinaia di chilometri, appesa a pali con mensole e isolatori, da cui il treno prende energia.',
      expert:
        'Filo di contatto in rame (100-150 mm²) sospeso alla fune portante con pendini; le mensole sui pali portano isolatori e bracci di poligonazione che tengono il filo a zig-zag rispetto all\'asse del binario. In Italia: 3 kV c.c. sulla rete tradizionale, 25 kV 50 Hz c.a. sulle linee AV/AC. Il ritorno della corrente avviene attraverso ruote e rotaie.',
      explode: [0, 0.6, 0],
    });

    // ================= Pantograph (arm in the track plane, head across the track) =================
    const pan = new THREE.Group();
    const pFrame = MAT.paint(0x8f99a4, 0.45);
    const armMat = MAT.paint(0xc5cbd2, 0.4);
    const insGeo = insulatorGeo();
    pan.add(rbox(0.6, 0.04, 0.44, 0.01, pFrame, TX, 2.03, Z_S));
    for (const dx of [-0.21, 0.21]) {
      for (const dz of [-0.16, 0.16]) pan.add(new THREE.Mesh(insGeo, ceramic).translateX(TX + dx).translateY(1.89).translateZ(Z_S + dz));
    }
    const H = [2.07, Z_S - 0.2];
    const K = [2.5, Z_S + 0.42];
    const P = [2.93, Z_S];
    pan.add(rod([TX - 0.17, H[0], H[1]], [TX + 0.17, H[0], H[1]], 0.022, pFrame));
    for (const s of [-1, 1]) {
      pan.add(rbox(0.04, 0.06, 0.08, 0.008, pFrame, TX + s * 0.17, 2.07, H[1]));          // hinge brackets
      pan.add(rod([TX + s * 0.15, H[0], H[1]], [TX + s * 0.035, K[0], K[1]], 0.018, armMat));
    }
    pan.add(rod([TX - 0.05, K[0], K[1]], [TX + 0.05, K[0], K[1]], 0.026, pFrame));
    pan.add(rod([TX, K[0], K[1]], [TX, P[0], P[1]], 0.016, armMat));
    pan.add(rod([TX + 0.06, 2.08, Z_S - 0.12], [TX + 0.06, 2.44, Z_S + 0.33], 0.008, pFrame)); // control rod
    pan.add(rod([TX, 2.05, Z_S + 0.05], [TX, 2.27, Z_S + 0.08], 0.035, rubber));                // air actuator
    pan.add(rbox(0.06, 0.05, 0.05, 0.01, pFrame, TX, 2.92, P[1]));
    pan.add(rod([TX - 0.45, 2.945, P[1]], [TX + 0.45, 2.945, P[1]], 0.014, armMat));
    const carbon = MAT.carbon();
    for (const dz of [-0.05, 0.05]) {
      pan.add(box(0.74, 0.012, 0.05, pFrame, TX, 2.962, P[1] + dz));
      pan.add(box(0.72, 0.02, 0.035, carbon, TX, 2.978, P[1] + dz));
    }
    for (const s of [-1, 1]) {
      pan.add(rod([TX + s * 0.36, 2.965, P[1]], [TX + s * 0.5, 2.94, P[1]], 0.011, armMat));
      pan.add(rod([TX + s * 0.5, 2.94, P[1]], [TX + s * 0.58, 2.86, P[1]], 0.011, armMat));
    }
    add(pan, {
      id: 'pantografo',
      name: 'Pantografo',
      base: 'Il braccio snodato sul tetto: una molla ad aria lo spinge contro il filo e una striscia di carbone scorre sulla catenaria senza rovinarla.',
      expert:
        'Pantografo a braccio singolo su isolatori, con striscianti in grafite e attuatore pneumatico che mantiene la forza di contatto quasi costante anche ad alta velocità; i treni politensione hanno pantografi dedicati a 3 kV c.c. e 25 kV c.a.',
      explode: [0, 0.3, 0.1],
    });

    // ================= Transformer / converter =================
    const trafo = new THREE.Group();
    const tMat = MAT.paint(0x5d6f80, 0.5);
    trafo.add(rbox(0.75, 0.62, 0.55, 0.03, tMat, TX, FLOOR_Y + 0.31, Z_S));
    const tFin = MAT.paint(0x4b5b6b, 0.5);
    for (let i = 0; i < 8; i++) trafo.add(box(0.014, 0.48, 0.05, tFin, TX - 0.27 + i * 0.07, FLOOR_Y + 0.3, Z_S + 0.3));
    trafo.add(new THREE.Mesh(insGeo, ceramic).translateX(TX + 0.3).translateY(1.89).translateZ(Z_S + 0.18)); // HV bushing
    trafo.add(rod([TX + 0.375, 1.55, Z_S + 0.12], [IX - 0.4, 1.55, Z_S + 0.12], 0.022, MAT.paint(0xd0611f, 0.5))); // DC link
    for (const x of [TX + 0.38, IX - 0.405]) {
      trafo.add(new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.03, 16).rotateZ(Math.PI / 2), MAT.brass()).translateX(x).translateY(1.55).translateZ(Z_S + 0.12));
    }
    add(trafo, {
      id: 'trasformatore',
      name: 'Trasformatore e convertitore',
      base: "Adatta l'energia della linea: abbassa la tensione e la rende continua, pronta per l'inverter. Il cavo arancione è il collegamento in continua (DC link).",
      expert:
        'In c.a.: trasformatore principale raffreddato a olio + raddrizzatore attivo a 4 quadranti (4QS) che alimenta il DC link con fattore di potenza ≈ 1. In c.c.: filtro LC d\'ingresso. In frenatura a recupero il flusso di energia si inverte verso la linea.',
      explode: [-0.4, 0.25, 0.3],
    });

    // ================= Inverter =================
    const inv = new THREE.Group();
    inv.add(rbox(0.8, 0.42, 0.5, 0.03, MAT.paint(0x2f3946, 0.5), IX, FLOOR_Y + 0.21, Z_S));
    for (let i = 0; i < 14; i++) inv.add(box(0.012, 0.1, 0.44, alu, IX - 0.36 + i * 0.055, FLOOR_Y + 0.47, Z_S));
    const igbt = [];
    for (let k = 0; k < 3; k++) {
      const leg = [];
      for (const y of [1.6, 1.47]) {
        const m = rbox(0.15, 0.09, 0.03, 0.008, glowMat(MAT.paint(0x1c2128, 0.5)), IX - 0.22 + k * 0.22, y, Z_S + 0.26);
        inv.add(m);
        leg.push(m);
      }
      igbt.push(leg);
    }
    const termGeo = new THREE.CylinderGeometry(0.028, 0.032, 0.05, 12).rotateX(Math.PI / 2);
    for (let k = 0; k < 3; k++) inv.add(new THREE.Mesh(termGeo, ceramic).translateX(IX - 0.1 + k * 0.1).translateY(1.33).translateZ(Z_S + 0.27));
    add(inv, {
      id: 'inverter',
      name: 'Inverter di trazione (VVVF)',
      base:
        "Il «cervello di potenza»: con interruttori elettronici velocissimi trasforma la corrente continua in tre correnti alternate e ne regola il ritmo, decidendo quanto veloce gira il motore. I moduli che si accendono sono gli interruttori in conduzione.",
      expert:
        'Inverter a tensione impressa (VSI) con 3 rami da 2 interruttori IGBT (3,3-6,5 kV) o SiC; modulazione PWM e controllo vettoriale della coppia. Tensione e frequenza variabili (VVVF), da pochi Hz all\'avvio fino a ~200 Hz.',
      explode: [0.4, 0.25, 0.3],
    });

    // ================= 3-phase cables =================
    const cables = new THREE.Group();
    const cableCurves = [];
    const cableMeshes = [];
    const cableColors = [0x7a4a25, 0x1e1f22, 0x8e949b]; // U brown, V black, W grey
    for (let k = 0; k < 3; k++) {
      const xs = IX - 0.1 + k * 0.1;
      const xe = X0 + 0.12 + k * 0.08;
      const curve = new THREE.CatmullRomCurve3(
        [
          [xs, 1.33, Z_S + 0.29], [xs, 1.34, -0.82], [(xs + xe) / 2 + 0.05, 1.31, -0.58],
          [xe, 1.2, -0.28], [xe, 1.1, -0.12], [xe, 1.055, -0.12],
        ].map(V),
        false,
        'centripetal',
      );
      const m = new THREE.Mesh(new THREE.TubeGeometry(curve, 64, 0.018, 8, false), glowMat(MAT.paint(cableColors[k], 0.6)));
      cables.add(m);
      cableCurves.push(curve);
      cableMeshes.push(m);
    }
    for (const t of [0.35, 0.62]) {
      const p = cableCurves[1].getPointAt(t);
      cables.add(rbox(0.24, 0.035, 0.035, 0.01, MAT.paint(0x15181b, 0.6), p.x, p.y, p.z)); // cable cleats
    }
    add(cables, {
      id: 'cavi',
      name: 'Cavi trifase U-V-W',
      base: "Tre cavi, uno per fase. Portano tre correnti uguali ma sfasate: ognuna raggiunge il massimo un terzo di giro dopo l'altra.",
      expert:
        'Terna trifase sfasata di 120° elettrici: i_U = I·cos ωt, i_V = I·cos(ωt − 120°), i_W = I·cos(ωt − 240°); colori marrone, nero, grigio secondo la convenzione europea. Nell\'animazione la luminosità di ogni cavo segue la sua corrente.',
      explode: [0, 0.15, 0.25],
    });

    // ================= Rotating magnetic field (overlay, visible in section view) =================
    const field = new THREE.Group();
    field.position.set(X0, YM, 0);
    model.add(field);
    const lobeMat = new THREE.MeshBasicMaterial({
      color: FIELD, transparent: true, opacity: 0.4, blending: THREE.AdditiveBlending,
      depthWrite: false, depthTest: false, side: THREE.DoubleSide,
    });
    const arrowMat = new THREE.MeshBasicMaterial({ color: FIELD, transparent: true, opacity: 0.9, depthWrite: false, depthTest: false });
    const LOBE = 1.2;
    const lobeGeo = alongX(new THREE.CylinderGeometry(0.194, 0.194, 0.64, 20, 1, true, -Math.PI / 2 - LOBE / 2, LOBE));
    for (const rot of [0, Math.PI]) {
      const l = new THREE.Mesh(lobeGeo, lobeMat);
      l.rotation.x = rot;
      l.renderOrder = 30;
      field.add(l);
    }
    const arrowShaft = new THREE.CylinderGeometry(0.01, 0.01, 0.3, 8);
    const arrowTip = new THREE.ConeGeometry(0.03, 0.07, 12);
    for (const x of [-0.22, 0, 0.22]) {
      const s = new THREE.Mesh(arrowShaft, arrowMat);
      s.position.set(x, -0.03, 0);
      const t = new THREE.Mesh(arrowTip, arrowMat);
      t.position.set(x, 0.155, 0);
      s.renderOrder = t.renderOrder = 31;
      field.add(s, t);
    }

    // ================= Current flows =================
    const main = flow({
      path: shifted(
        polyPath([
          [wireX(-2.6) + 0.02, WIRE_Y - 0.014, -2.6], [wireX(Z_S) + 0.02, WIRE_Y - 0.014, Z_S], [TX + 0.04, 2.95, Z_S],
          [TX + 0.04, K[0], K[1]], [TX + 0.04, H[0] + 0.01, H[1]], [TX + 0.3, 2.07, Z_S + 0.2], [TX + 0.3, 1.95, Z_S + 0.24],
          [TX + 0.395, 1.87, Z_S + 0.29], [TX + 0.395, 1.6, Z_S + 0.29], [TX + 0.42, 1.55, Z_S + 0.16],
          [IX - 0.42, 1.55, Z_S + 0.16], [IX - 0.38, 1.4, Z_S + 0.28], [IX, 1.39, Z_S + 0.28],
        ]),
      ),
      count: 480,
      size: 0.055,
      colors: [CURRENT, 0xf4ffb0, CURRENT],
      spread: 0.016,
      speed: 0.09,
      opacity: 0.9,
    });
    root.add(main.object);
    ctx.addFlow(main);

    const lift = new THREE.Vector3(0, 0.026, 0.012); // ride on the cable surface, not inside it
    const phaseFlows = cableCurves.map((curve) => {
      const f = flow({
        path: shifted((t, out) => curve.getPointAt(t, out).add(lift)),
        count: 150,
        size: 0.045,
        colors: [CURRENT],
        spread: 0.01,
        speed: 0.45,
        opacity: 0.9,
      });
      root.add(f.object);
      ctx.addFlow(f);
      return f;
    });

    // ================= Animation =================
    let rotAcc = 0;   // rotor angle (rad)
    let scroll = 0;   // track scroll (units)
    const cycleRate = (thr) => 0.08 + thr * 1.3; // field revolutions per second (visual)

    return {
      root,
      cycleRate,
      update(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const dt = state.dt || 0;
        const time = state.time || 0;
        const act = 0.3 + 0.7 * thr;
        const assembled = (state.explode || 0) < 0.05;

        // How far the section plane has opened the motor (0 closed .. 1 open).
        const plane = housingShell.material.clippingPlanes && housingShell.material.clippingPlanes[0];
        const c = plane ? plane.constant : 1e6;
        const open = c > 50 ? 0 : clamp((0.5 - c) / 0.5, 0, 1);

        // Field angle from the cycle; rotor lags behind (visual slip exaggerated to be visible).
        const theta = (state.cycle || 0) * TAU;
        const fieldRate = cycleRate(thr);
        const rotorRate = Math.max(0, fieldRate - (0.07 + 0.05 * thr));
        rotAcc = (rotAcc + dt * rotorRate * TAU) % (TAU * Z_GEAR);
        field.rotation.x = theta;
        field.visible = assembled && open > 0.01;
        lobeMat.opacity = (0.16 + 0.3 * act) * open;
        arrowMat.opacity = (0.45 + 0.45 * act) * open;

        rotor.rotation.x = rotAcc;
        shaft.rotation.x = rotAcc;
        for (const b of ballSets) b.rotation.x = rotAcc * 0.4;
        pinion.rotation.x = Math.PI + rotAcc;
        const wheelAng = -Math.PI / Z_GEAR - rotAcc / RATIO; // meshes with the pinion, opposite sense
        gear.rotation.x = wheelAng;
        salaSpin.rotation.x = wheelAng;

        // Track slides under the (fixed) wheelset at the rolling speed.
        scroll += dt * (rotorRate / RATIO) * TAU * WHEEL_R;
        for (const s of sleepers) {
          const z = ((((s.userData.z0 + scroll + 1.8) % 3.6) + 3.6) % 3.6) - 1.8;
          const e = clamp((Math.abs(z) - 1.3) / 0.5, 0, 1);
          s.position.set(0, -0.22 * e * e * (3 - 2 * e), z);
        }

        // Phase currents (120° apart); coil belt k at angle k*60° glows with its current.
        const iPh = [Math.cos(theta), Math.cos(theta - TAU / 3), Math.cos(theta - (2 * TAU) / 3)];
        for (const co of coils) {
          const v = Math.cos(co.a - theta);
          co.mesh.material.emissiveIntensity = act * v * v;
        }
        for (const b of bars) {
          const v = Math.cos(b.a + rotAcc - theta);
          b.mesh.material.emissiveIntensity = (0.15 + 0.85 * thr) * v * v;
        }
        const pwm = Math.sin(time * 70) > -0.3 ? 1 : 0.35;
        for (let k = 0; k < 3; k++) {
          const i = iPh[k];
          cableMeshes[k].material.emissiveIntensity = act * 0.7 * Math.abs(i);
          igbt[k][0].material.emissiveIntensity = act * 1.5 * Math.max(0, i) * pwm;
          igbt[k][1].material.emissiveIntensity = act * 1.5 * Math.max(0, -i) * pwm;
          phaseFlows[k].material.opacity = (0.15 + 0.8 * Math.abs(i)) * (0.45 + 0.55 * thr);
          phaseFlows[k].object.visible = assembled;
        }

        const rate = 0.25 + thr * 1.75;
        main.object.visible = assembled;
        main.material.opacity = 0.35 + 0.55 * thr;
        main.update(dt, rate);
        for (const f of phaseFlows) f.update(dt, rate * 1.2);
      },
      readout(state) {
        const thr = clamp(state.throttle || 0, 0, 1);
        const rpm = thr * MAX_RPM;
        const fSlip = 0.5 + 3.5 * Math.min(1, thr * 5);
        const f = (rpm / 60) * POLE_PAIRS + fSlip;
        const kw = 610 * Math.min(1, thr / 0.4);
        const kmh = ((rpm / RATIO) * Math.PI * WHEEL_D * 60) / 1000;
        return [
          { label: 'Giri motore', value: Math.round(rpm).toLocaleString('it-IT'), unit: 'rpm' },
          { label: 'Frequenza inverter', value: f < 10 ? f.toFixed(1).replace('.', ',') : String(Math.round(f)), unit: 'Hz' },
          { label: 'Potenza', value: String(Math.round(kw)), unit: 'kW' },
          { label: 'Velocità treno', value: String(Math.round(kmh)), unit: 'km/h' },
        ];
      },
    };
  },
};
