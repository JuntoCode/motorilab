// Liquid rocket engine, gas-generator cycle (Merlin 1D-like, RP-1 + LOX). Continuous engine:
// state.cycle only drives the turbopump rotor angle; everything else follows throttle and time.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, pipe, flow, clamp, lerp, smooth } from '../core/helpers.js';

// ---------- Stations (model space, nozzle points down -Y) ----------
const PIV = 1.02; // gimbal pivot
const Y_INJ = 0.8; // injector face
const R_C = 0.19; // chamber inner radius
const Y_BAR = 0.42; // end of cylindrical barrel
const Y_T = 0.14; // throat
const R_T = 0.112; // throat radius
const Y_E = -1.63; // nozzle exit
const R_E = 0.45; // exit radius (area ratio ~16)
const Y_EXT = -0.35; // copper section / dark extension joint
const LINER = 0.014; // regen wall: copper liner, coolant gap, outer jacket
const GAP = 0.022;
const JACKET = 0.012;
const R_OUT = LINER + GAP + JACKET; // outer jacket offset from the hot-gas wall
const TP_X = 0.78; // turbopump axis
const TANK_R = 1.0;
const DOME_H = 0.4;
const RP_APEX = 1.36; // RP-1 tank bottom dome apex
const LOX_APEX = 1.9; // LOX tank bottom dome (common bulkhead) apex
const TANK_TOP = 2.42;
const ROTOR_TURNS = 4; // visual rotor turns per cycle

const thrustOf = (th) => 0.4 + 0.6 * clamp(th, 0, 1); // deep throttle 40% .. 100%

// ---------- Small builders ----------
const V = (x, y, z = 0) => new THREE.Vector3(x, y, z);
const crv = (pts) => new THREE.CatmullRomCurve3(pts.map((p) => V(...p)), false, 'centripetal');
const tube = (curve, r, mat, seg = 64, rad = 12) =>
  new THREE.Mesh(new THREE.TubeGeometry(curve, seg, r, rad, false), mat);

function ring(R, r, y, mat, x = 0, z = 0) {
  const m = new THREE.Mesh(new THREE.TorusGeometry(R, r, 10, 64), mat);
  m.rotation.x = Math.PI / 2;
  m.position.set(x, y, z);
  return m;
}

function strut(a, b, r, mat) {
  const d = new THREE.Vector3().subVectors(b, a);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, d.length(), 10), mat);
  m.position.copy(a).add(b).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(V(0, 1, 0), d.normalize());
  return m;
}

function rbox(w, h, d, r, mat, pos) {
  const m = new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 3, r), mat);
  if (pos) m.position.set(...pos);
  return m;
}

// Offset a lathe profile by t along its right-hand normal (inside, for a profile drawn top->bottom).
function offsetPts(pts, t) {
  return pts.map((p, i) => {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(pts.length - 1, i + 1)];
    let tx = b[0] - a[0];
    let ty = b[1] - a[1];
    const l = Math.hypot(tx, ty) || 1;
    tx /= l;
    ty /= l;
    return [Math.max(0, p[0] + ty * t), p[1] - tx * t];
  });
}
// Closed solid shell of thickness t from an open profile (gets a hatched section cap).
const shell = (pts, t, segs = 48) => lathe([...pts, ...offsetPts(pts, t).reverse(), pts[0]], segs);

// Closed lathe with internal passages: holes must wind opposite to the outer loop.
function latheLoops(outer, holes, segs = 48) {
  const pts = [...outer, outer[0]];
  for (const h of holes) pts.push(...h, h[0], outer[0]);
  return lathe(pts, segs);
}

// Hollow torus (manifold / volute): wall between radii ro and ri around a circle of radius R.
function hollowTorus(R, ro, ri, n = 14, segs = 48) {
  const o = [];
  const i = [];
  for (let k = 0; k <= n; k++) {
    const a = (k / n) * Math.PI * 2;
    o.push([R + ro * Math.cos(a), ro * Math.sin(a)]);
    i.push([R + ri * Math.cos(a), ri * Math.sin(a)]);
  }
  return lathe([...o, ...i.reverse(), o[0]], segs);
}

// Repeated small parts as one InstancedMesh. items: [{ p: [x,y,z], r: [rx,ry,rz] }]
const _m4 = new THREE.Matrix4();
const _q = new THREE.Quaternion();
const _e = new THREE.Euler();
const _p = new THREE.Vector3();
const _s = new THREE.Vector3(1, 1, 1);
function instanced(geo, mat, items) {
  const im = new THREE.InstancedMesh(geo, mat, items.length);
  items.forEach((t, k) => {
    _q.setFromEuler(_e.set(...(t.r || [0, 0, 0])));
    im.setMatrixAt(k, _m4.compose(_p.set(...t.p), _q, _s));
  });
  im.instanceMatrix.needsUpdate = true;
  im.computeBoundingSphere();
  return im;
}
const circle = (r, y, n, x = 0, z = 0, a0 = 0) =>
  Array.from({ length: n }, (_, k) => {
    const a = a0 + (k / n) * Math.PI * 2;
    return { p: [x + Math.cos(a) * r, y, z + Math.sin(a) * r] };
  });
const BOLT = new THREE.CylinderGeometry(0.0085, 0.0085, 0.009, 6).translate(0, 0.0045, 0);

// Flexible bellows on a line, at arc fraction u.
function bellows(curve, u, r, mat) {
  const g = new THREE.Group();
  g.position.copy(curve.getPointAt(u));
  g.quaternion.setFromUnitVectors(V(0, 0, 1), curve.getTangentAt(u));
  const geo = new THREE.TorusGeometry(r, r * 0.2, 8, 32);
  for (let i = 0; i < 5; i++) {
    const m = new THREE.Mesh(geo, mat);
    m.position.z = (i - 2) * r * 0.42;
    g.add(m);
  }
  return g;
}

// Elliptical dome, apex down: [[r, y], ...] from axis to equator.
function domePts(apex, R, H, k = 1, n = 22) {
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const a = (i / n) * (Math.PI / 2);
    pts.push([R * Math.sin(a) * k, apex + H * (1 - Math.cos(a))]);
  }
  return pts;
}

function liquid(color, opacity) {
  return cut(
    new THREE.MeshStandardMaterial({ color, metalness: 0, roughness: 0.15, transparent: true, opacity, depthWrite: false })
  );
}

// ---------- Chamber + nozzle inner profile ----------
function buildProfile() {
  const pts = [[R_C, Y_INJ]];
  for (let i = 0; i <= 14; i++) {
    const y = lerp(Y_BAR, Y_T, i / 14);
    pts.push([R_T + (R_C - R_T) * smooth((y - Y_T) / (Y_BAR - Y_T)), y]);
  }
  // Bell: quadratic Bezier, 30° after the throat, 8° at the exit (Rao-like).
  const a0 = Math.tan((30 * Math.PI) / 180);
  const a1 = Math.tan((8 * Math.PI) / 180);
  const s = (R_E - a1 * (Y_T - Y_E) - R_T) / (a0 - a1);
  const c = [R_T + a0 * s, Y_T - s];
  for (let i = 1; i <= 40; i++) {
    const u = i / 40;
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

function profSlice(yTop, yBot, k = 1) {
  const pts = [[rAt(yTop) * k, yTop]];
  for (const [r, y] of PROF) if (y < yTop && y > yBot) pts.push([r * k, y]);
  pts.push([rAt(yBot) * k, yBot]);
  return pts;
}

// Closed wall section between two heights, from offset a to offset b off the hot-gas wall.
function wallGeo(yTop, yBot, a, b, segs = 64) {
  const base = profSlice(yTop, yBot);
  const inner = base.map(([r, y]) => [r + a, y]);
  const outer = base
    .slice()
    .reverse()
    .map(([r, y]) => [r + b, y]);
  return lathe([...inner, ...outer, inner[0]], segs);
}

// Coolant-channel land (rib) following the wall, in the gap between liner and jacket.
function ribGeo(yTop, yBot) {
  const pts = [];
  for (let i = 0; i <= 16; i++) {
    const y = lerp(yTop, yBot, i / 16);
    pts.push(V(rAt(y) + LINER + GAP / 2, y, 0));
  }
  return new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 24, GAP / 2, 5, false);
}
const RIBS = 40;
const ribItems = () => Array.from({ length: RIBS }, (_, k) => ({ p: [0, 0, 0], r: [0, ((k + 0.5) / RIBS) * Math.PI * 2, 0] }));

// ---------- Feed line geometry (shared by meshes and flows) ----------
const LOX_DOWN = [
  [0.5, 2.05, 0],
  [0.5, 1.7, 0],
  [0.5, 1.33, 0],
  [0.52, 1.2, 0],
  [0.62, 1.09, 0],
  [0.74, 1.03, 0],
  [0.78, 0.96, 0],
  [0.78, 0.9, 0],
];
const LOX_DIS = [
  [0.61, 0.76, 0],
  [0.52, 0.82, 0],
  [0.42, 0.9, 0],
  [0.3, 0.95, 0],
  [0.18, 0.94, 0],
];
const FUEL_FEED = [
  [0.8, 1.66, -0.3],
  [0.8, 1.45, -0.3],
  [0.86, 1.2, -0.3],
  [0.93, 0.86, -0.3],
  [0.88, 0.7, -0.22],
  [0.81, 0.63, -0.13],
  [0.8, 0.6, -0.11],
];
const R_MAN = rAt(Y_EXT) + R_OUT + 0.036; // fuel inlet manifold on the nozzle
const FUEL_DIS = [
  [0.62, 0.47, 0],
  [0.5, 0.39, 0],
  [0.43, 0.24, 0],
  [0.38, 0.02, 0],
  [0.35, -0.2, 0],
  [R_MAN, Y_EXT + 0.045, 0],
];
const DUCT = [
  [TP_X, 0.11, 0],
  [0.8, -0.05, 0],
  [0.86, -0.35, 0],
  [0.93, -0.85, 0],
  [0.98, -1.25, 0],
  [1.0, -1.48, 0],
];
const GG_POS = [1.2, 0.4, -0.22];

export default {
  id: 'rocket',
  name: 'Motore a razzo a propellente liquido',
  short: 'Razzo a liquido',
  family: 'spazio',
  vehicle: 'Lanciatore spaziale',
  examples: 'Falcon 9 (Merlin), Vega-C, Ariane 6 (Vulcain 2.1)',
  tagline: 'Brucia una tonnellata di propellente ogni tre secondi e la lancia indietro a 10.000 km/h.',
  intro: {
    base:
      'Un razzo non ha bisogno dell’aria: porta con sé sia il combustibile (un cherosene speciale) sia l’<b>ossigeno</b>, reso liquido a −183 °C. ' +
      'Una turbopompa potentissima li spinge nella camera di combustione, dove bruciano a circa 3400 °C. ' +
      'I gas escono dall’ugello a circa 10.000 km/h e, come un palloncino che si sgonfia, spingono il razzo nella direzione opposta.',
    expert:
      'Endoreattore bipropellente RP-1/LOX a <b>ciclo a generatore di gas</b> (ciclo aperto), come il Merlin 1D del Falcon 9: pochi punti percentuali della portata bruciano in un generatore ricco di combustibile che aziona la turbina e vengono poi scaricati fuori bordo. ' +
      'Camera a ~97 bar e ~3400 °C con raffreddamento rigenerativo, O/F ≈ 2,36, ugello de Laval con velocità di scarico ~3 km/s (Isp 282 s al mare, 311 s nel vuoto). ' +
      'Circa il 65% dell’energia chimica diventa energia cinetica del getto (stima indicativa). ' +
      'Il Raptor di SpaceX usa invece metano e un ciclo a <b>combustione stadiata a flusso completo</b> (~300 bar), senza gas scaricato.',
  },
  specs: [
    { label: 'Propellenti', value: 'RP-1 (cherosene) + ossigeno liquido' },
    { label: 'Spinta al livello del mare', value: '≈ 845 kN (≈ 86 t)' },
    { label: 'Impulso specifico', value: '≈ 282 s (mare) · 311 s (vuoto)' },
    { label: 'Pressione in camera', value: '≈ 97 bar' },
    { label: 'Temperatura in camera', value: '≈ 3400 °C' },
    { label: 'Massa', value: '≈ 470 kg' },
    { label: 'Rapporto spinta/peso', value: '≈ 180' },
  ],
  metrics: {
    efficiency: 65, // approx. chemical energy -> jet kinetic energy
    power: 1.2e6, // kW, jet power 0.5 * mdot * ve^2
    powerDensity: 2500, // kW/kg
  },
  phases: [
    {
      id: 'feed',
      name: 'Alimentazione dei propellenti',
      parts: ['loxTank', 'rp1Tank', 'valves'],
      base:
        'I serbatoi contengono ossigeno liquido e cherosene, leggermente in pressione grazie a un po’ di elio. ' +
        'I propellenti scendono lungo grossi tubi verso il motore e le valvole principali decidono quando e quanto ne passa, come rubinetti molto veloci.',
      expert:
        'La pressione dei serbatoi (pochi bar) più la colonna di liquido sotto l’accelerazione di volo garantiscono il NPSH richiesto dalle pompe, evitando la cavitazione. ' +
        'Il LOX scende nel downcomer che attraversa il serbatoio RP-1; soffietti flessibili assorbono i movimenti del gimbal. ' +
        'MOV e MFV, a valle delle pompe, gestiscono la sequenza di avvio (leggermente ricca di combustibile) e lo spegnimento.',
    },
    {
      id: 'turbopump',
      name: 'Turbopompa',
      parts: ['turbopump', 'gasGenerator', 'exhaustDuct'],
      base:
        'Per spingere i propellenti in una camera dove la pressione è altissima servono pompe enormemente potenti. ' +
        'Le fa girare una turbina, mossa dai gas di un piccolo bruciatore: il generatore di gas. Dopo la turbina quei gas vengono buttati fuori dal tubo di scarico.',
      expert:
        'Ciclo a generatore di gas (aperto): pochi % della portata bruciano nel GG con miscela molto ricca di combustibile, per tenere i gas sotto il limite termico delle palette. ' +
        'La turbina ad azione muove le pompe centrifughe LOX e RP-1 sullo stesso albero (oltre 36.000 giri/min nel Merlin 1D). Lo scarico fuori bordo costa qualche punto di Isp. ' +
        'Nel Raptor (combustione stadiata a flusso completo) due precamere, una ricca di ossigeno e una ricca di metano, alimentano due turbine e tutto il gas finisce in camera, a ~300 bar.',
    },
    {
      id: 'combustion',
      name: 'Iniezione e combustione',
      parts: ['injector', 'chamber'],
      base:
        'L’iniettore spruzza ossigeno e cherosene in una nebbia finissima che si mescola e brucia all’istante. ' +
        'La temperatura sale a circa 3400 °C e la pressione arriva a quasi cento volte quella dell’aria che respiriamo.',
      expert:
        'Iniettore a pintle: un propellente esce radialmente dallo spinotto centrale e incrocia la lamina anulare dell’altro; combustione stabile e regolabile. ' +
        'Pc ≈ 97 bar, T ≈ 3400 °C, O/F ≈ 2,36: più ricca di combustibile dello stechiometrico (≈ 3,4), così i gas hanno massa molare più bassa e corrono più veloci, e la parete soffre meno. ' +
        'Prodotti principali: CO₂, H₂O, CO, H₂. Accensione con TEA-TEB, il lampo verde dei lanci.',
    },
    {
      id: 'cooling',
      name: 'Raffreddamento rigenerativo',
      parts: ['chamber', 'nozzle'],
      base:
        'Nessun metallo resisterebbe a 3400 °C. Il trucco: prima di entrare in camera, il cherosene ancora freddo corre in centinaia di canalini scavati nella parete di rame, ' +
        'si porta via il calore e la mantiene a poche centinaia di gradi. Poi, già riscaldato, viene bruciato.',
      expert:
        'Il combustibile entra dal collettore sull’ugello e risale i canali del liner in lega di rame fino all’iniettore: il calore asportato rientra nel ciclo, da cui il nome “rigenerativo”. ' +
        'Il punto critico è la gola, dove il flusso termico raggiunge decine di MW/m². Limite tipico del cherosene: il coking, depositi carboniosi nei canali se la parete si scalda troppo.',
    },
    {
      id: 'thrust',
      name: 'Espansione nell’ugello e spinta',
      parts: ['nozzle', 'gimbal'],
      base:
        'I gas caldi passano dalla gola, il punto più stretto, dove raggiungono la velocità del suono; poi si allargano nella campana e accelerano ancora, fino a circa 10.000 km/h. ' +
        'Lanciando indietro centinaia di kg di gas al secondo, il motore riceve una spinta uguale e contraria: è la terza legge di Newton. Gli attuatori inclinano il motore per guidare il razzo.',
      expert:
        'F = ṁ·vₑ + (pₑ − pₐ)·Aₑ: con ṁ ≈ 300 kg/s e vₑ ≈ 2,8 km/s si ottengono ~845 kN al mare. Conservazione della quantità di moto: l’impulso ceduto al getto è uguale e opposto a quello del razzo. ' +
        'Ugello de Laval: nel tratto convergente il flusso subsonico accelera, in gola M = 1, nel divergente il flusso supersonico accelera ancora perché la sezione cresce; pressione e temperatura calano. ' +
        'Isp ≈ 282 s al mare, ≈ 311 s nel vuoto (niente contropressione). Le losanghe luminose nel getto sono i “diamanti d’urto” (Mach diamonds).',
    },
  ],
  quiz: [
    {
      q: 'Perché un motore a razzo funziona anche nel vuoto dello spazio?',
      options: [
        'Perché spinge contro l’aria rimasta dentro l’ugello',
        'Perché porta con sé l’ossidante e lancia indietro i gas: per la terza legge di Newton riceve una spinta in avanti',
        'Perché nel vuoto la gravità non agisce',
        'Perché l’ugello funziona come un’elica',
      ],
      correct: 1,
      explain:
        'Il razzo non “si appoggia” a nulla: espelle massa ad altissima velocità e riceve un impulso uguale e contrario. Nel vuoto rende anzi di più: l’Isp sale da 282 a 311 s.',
    },
    {
      q: 'A cosa serve il generatore di gas in un motore come il Merlin?',
      options: [
        'A produrre la fiamma principale',
        'A pressurizzare i serbatoi',
        'A far girare la turbina che aziona le pompe dei propellenti',
        'A raffreddare la camera di combustione',
      ],
      correct: 2,
      explain:
        'Brucia una piccola parte dei propellenti solo per muovere la turbina della turbopompa; poi quel gas viene scaricato fuori bordo (ciclo aperto).',
    },
    {
      q: 'Perché il cherosene scorre nelle pareti della camera prima di essere bruciato?',
      options: [
        'Per raffreddare la parete, che altrimenti fonderebbe',
        'Per renderlo più denso',
        'Per lubrificare la turbopompa',
        'Per rallentarlo prima dell’iniezione',
      ],
      correct: 0,
      explain:
        'È il raffreddamento rigenerativo: il combustibile freddo assorbe il calore della parete in rame e lo riporta in camera, senza sprecarlo.',
    },
    {
      q: 'Cosa succede ai gas nella parte dell’ugello che si allarga (divergente)?',
      options: [
        'Rallentano e si riscaldano',
        'Si fermano e tornano verso la camera',
        'Si raffreddano fino a diventare liquidi',
        'Accelerano oltre la velocità del suono mentre pressione e temperatura calano',
      ],
      correct: 3,
      explain:
        'In un ugello de Laval, dopo la gola (dove M = 1) il flusso è supersonico e l’aumento di sezione lo accelera ancora, trasformando pressione e calore in velocità.',
    },
  ],
  variants: {
    columns: ['Motore', 'Dove vola', 'Propellenti', 'Ciclo', 'Spinta'],
    rows: [
      ['Merlin 1D (SpaceX)', 'Falcon 9, Falcon Heavy', 'RP-1 + LOX', 'Generatore di gas', '≈ 0,85 MN (mare)'],
      ['Raptor 2 (SpaceX)', 'Starship', 'Metano + LOX', 'Combustione stadiata a flusso completo', '≈ 2,3 MN (mare)'],
      ['RS-25 (Aerojet Rocketdyne)', 'Space Shuttle, SLS', 'Idrogeno + LOX', 'Combustione stadiata', '≈ 1,9 MN (mare)'],
      ['Vulcain 2.1 (ArianeGroup)', 'Ariane 6, stadio centrale', 'Idrogeno + LOX', 'Generatore di gas', '≈ 1,4 MN (vuoto)'],
      ['Vinci (ArianeGroup)', 'Ariane 6, stadio superiore', 'Idrogeno + LOX', 'Ciclo a espansione', '≈ 180 kN (vuoto)'],
      ['M10 (Avio)', 'Vega-E, stadio superiore (in sviluppo)', 'Metano + LOX', 'Ciclo a espansione', '≈ 100 kN (vuoto)'],
    ],
    note: 'Valori indicativi. “Mare” = spinta al livello del mare, “vuoto” = nello spazio, dove lo stesso motore spinge di più.',
  },
  thumb: { orbit: [-25, 10], zoom: 1.1, view: 'section' },
  camera: { position: [2.4, 0.3, 6.6], target: [0.25, -0.4, 0] },

  build(ctx) {
    const root = new THREE.Group();
    root.position.y = -0.55;
    // Gimballed engine: everything under `eng` uses model coordinates and pivots at PIV.
    const gimbal = new THREE.Group();
    gimbal.position.y = PIV;
    const eng = new THREE.Group();
    eng.position.y = -PIV;
    eng.updateMatrix();
    gimbal.add(eng);
    root.add(gimbal);

    const addPart = (parent, obj, info) => {
      const w = ctx.part(obj, info);
      parent.add(w);
      return w;
    };

    // ---------- Materials ----------
    const copper = cut(MAT.copper());
    const jacketMat = cut(MAT.copper());
    jacketMat.color.set(0xa8602f);
    jacketMat.roughness = 0.42;
    const ribMat = cut(MAT.copper());
    ribMat.color.set(0x8f4c26);
    const steelCut = cut(MAT.steel());
    const darkCut = cut(MAT.darkSteel());
    const alCut = cut(MAT.aluminum());
    const frost = cut(MAT.paint(0xdbe9f4, 0.88));
    const nozzleExt = cut(MAT.carbon());
    const ductMat = cut(MAT.castIron());
    const boltMat = cut(MAT.darkSteel());
    const steel = MAT.steel();
    const dark = MAT.darkSteel();
    const chrome = MAT.chrome();
    const chromeCut = cut(MAT.chrome());
    const brass = MAT.brass();
    const bronze = cut(MAT.brass());
    const rubber = MAT.rubber();
    const loxLine = cut(MAT.paint(0xe6eef5, 0.6));
    const lineSteel = cut(MAT.steel()); // lines are cut so the flow inside shows
    const leverMat = MAT.paint(0xd2502a, 0.5);
    const actMat = MAT.paint(0x3a424d, 0.5);

    // ---------- LOX tank (bottom dome = common bulkhead) ----------
    const loxTank = new THREE.Group();
    loxTank.add(new THREE.Mesh(shell([...domePts(LOX_APEX, TANK_R, DOME_H), [TANK_R, TANK_TOP]], 0.014, 72), frost));
    const loxFill = domePts(LOX_APEX + 0.004, TANK_R, DOME_H, 0.99);
    loxFill.push([TANK_R * 0.99, 2.2], [0, 2.2]);
    loxTank.add(new THREE.Mesh(lathe(loxFill, 64), liquid(0x8fd4ff, 0.22)));
    const cLoxDown = crv(LOX_DOWN);
    loxTank.add(tube(cLoxDown, 0.045, loxLine, 72));
    loxTank.add(ring(0.06, 0.014, LOX_APEX + 0.054, steelCut, 0.5, 0)); // downcomer outlet flange
    loxTank.add(bellows(cLoxDown, 0.93, 0.052, lineSteel));
    addPart(root, loxTank, {
      id: 'loxTank',
      name: 'Serbatoio ossigeno liquido (LOX)',
      base:
        'Contiene l’ossigeno, raffreddato fino a diventare liquido (−183 °C): così ne entra tantissimo in poco spazio. È lui che permette di bruciare il cherosene anche dove non c’è aria. Il tubo che scende (downcomer) lo porta al motore attraversando il serbatoio del cherosene.',
      expert:
        'Sul Falcon 9 il LOX è “densificato” a circa −207 °C per aumentarne la densità (~1,2 kg/L) e caricarne di più. Pressurizzazione a elio di pochi bar contro la cavitazione delle pompe; il fondo a cupola ellittica è un setto comune con il serbatoio RP-1 e regge la pressione con pareti sottili in lega Al-Li.',
      explode: [0, 0.95, 0],
    });

    // ---------- RP-1 tank ----------
    const rpTank = new THREE.Group();
    rpTank.add(
      new THREE.Mesh(shell([...domePts(RP_APEX, TANK_R, DOME_H), [TANK_R, LOX_APEX + DOME_H]], 0.014, 72), alCut)
    );
    rpTank.add(ring(TANK_R + 0.014, 0.02, RP_APEX + DOME_H, alCut), ring(TANK_R + 0.014, 0.02, LOX_APEX + DOME_H, alCut));
    const rpFill = domePts(RP_APEX + 0.004, TANK_R, DOME_H, 0.99);
    rpFill.push([TANK_R * 0.99, 1.7], [0, 1.7]);
    rpTank.add(new THREE.Mesh(lathe(rpFill, 64), liquid(0xd9982b, 0.22)));
    const cFuelFeed = crv(FUEL_FEED);
    rpTank.add(tube(cFuelFeed, 0.04, lineSteel, 72));
    rpTank.add(bellows(cFuelFeed, 0.9, 0.047, lineSteel));
    addPart(root, rpTank, {
      id: 'rp1Tank',
      name: 'Serbatoio RP-1 (cherosene)',
      base:
        'Contiene il combustibile: un cherosene molto raffinato, parente di quello degli aerei. Dal fondo parte il tubo che lo porta alla pompa. È più piccolo del serbatoio dell’ossigeno perché per bruciarlo serve più del doppio di ossigeno.',
      expert:
        'RP-1: cherosene a tagli stretti, quasi privo di zolfo e aromatici, densità ~0,81 kg/L (il Falcon 9 lo raffredda per densificarlo). Rapporto di miscela O/F ≈ 2,36 in massa: per ogni kg di RP-1 servono ~2,4 kg di LOX. Il condotto di uscita ha un soffietto per seguire il gimbal.',
      explode: [0, 0.55, 0],
    });

    // ---------- Thrust structure (static, not clickable) ----------
    const thrustMat = cut(MAT.darkSteel());
    root.add(new THREE.Mesh(shell([[0.3, 1.385], [0.12, 1.1]], 0.02, 40), thrustMat));
    root.add(rbox(0.66, 0.06, 0.08, 0.012, thrustMat, [-0.45, 1.16, 0]));
    root.add(rbox(0.08, 0.06, 0.66, 0.012, thrustMat, [0, 1.16, -0.45]));
    root.add(strut(V(-0.76, 1.16, 0), V(-0.85, 1.56, 0), 0.025, thrustMat));
    root.add(strut(V(0, 1.16, -0.76), V(0, 1.56, -0.85), 0.025, thrustMat));
    root.add(instanced(BOLT, boltMat, [...circle(0.27, 1.355, 10, 0, 0, 0.3), ...circle(0.14, 1.12, 8)]));
    // Avionics / harness junction box on the thrust structure.
    const jbox = rbox(0.12, 0.08, 0.09, 0.012, MAT.paint(0x2e3540, 0.6), [-0.5, 1.11, -0.09]);
    root.add(jbox);

    // ---------- Turbopump ----------
    const tp = new THREE.Group();
    tp.position.x = TP_X;
    const pumpMat = cut(MAT.aluminum());
    const turbMat = cut(MAT.darkSteel());
    const pumpProfile = [
      [0.075, 0.13],
      [0.08, 0.085],
      [0.13, 0.06],
      [0.17, 0.03],
      [0.178, 0],
      [0.17, -0.035],
      [0.12, -0.065],
      [0.065, -0.08],
      [0.06, -0.095],
    ];
    const pumpGeo = shell(pumpProfile, 0.012);
    const voluteGeo = hollowTorus(0.175, 0.048, 0.036);
    const tpBolts = [];
    for (const y of [0.76, 0.48]) {
      const h = new THREE.Mesh(pumpGeo, pumpMat);
      h.position.y = y;
      const vol = new THREE.Mesh(voluteGeo, pumpMat);
      vol.position.y = y;
      tp.add(h, vol);
      tpBolts.push(...circle(0.14, y + 0.052, 12, 0, 0, 0.13));
    }
    tp.add(ring(0.082, 0.012, 0.89, pumpMat)); // LOX inlet flange
    tp.add(
      new THREE.Mesh(
        shell(
          [
            [0.07, 0.345],
            [0.15, 0.315],
            [0.205, 0.28],
            [0.212, 0.24],
            [0.205, 0.2],
            [0.15, 0.165],
            [0.095, 0.14],
            [0.085, 0.1],
          ],
          0.014
        ),
        turbMat
      )
    );
    const manifold = new THREE.Mesh(hollowTorus(0.235, 0.04, 0.03), turbMat); // hot-gas inlet manifold
    manifold.position.y = 0.24;
    tp.add(manifold);
    tpBolts.push(...circle(0.16, 0.31, 14));
    for (const y of [0.365, 0.64]) {
      const c = new THREE.Mesh(pipe(0.07, 0.056, 0.06, 32), turbMat);
      c.position.y = y;
      const brg = new THREE.Mesh(pipe(0.034, 0.024, 0.03, 24), bronze); // bearing / seal package
      brg.position.y = y;
      tp.add(c, brg);
    }
    tp.add(instanced(BOLT, boltMat, tpBolts));
    // Rotor: shaft + LOX impeller + fuel impeller + turbine wheel.
    const rotor = new THREE.Group();
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.86, 16), MAT.steel());
    shaft.position.y = 0.53;
    rotor.add(shaft);
    const vaneGeo = new THREE.BoxGeometry(0.1, 0.038, 0.009);
    const makeImpeller = (mat) => {
      const g = new THREE.Group();
      const disk = new THREE.Mesh(new THREE.CylinderGeometry(0.135, 0.135, 0.012, 40), mat);
      disk.position.y = -0.025;
      const hub = new THREE.Mesh(new THREE.ConeGeometry(0.045, 0.075, 20), mat);
      hub.position.y = 0.02;
      g.add(disk, hub);
      for (let i = 0; i < 7; i++) {
        const piv = new THREE.Group();
        piv.rotation.y = (i / 7) * Math.PI * 2;
        const v = new THREE.Mesh(vaneGeo, mat);
        v.position.x = 0.075;
        v.rotation.y = 0.55; // backswept
        piv.add(v);
        g.add(piv);
      }
      return g;
    };
    const loxImp = makeImpeller(MAT.aluminum());
    loxImp.position.y = 0.76;
    const fuelImp = makeImpeller(MAT.steel());
    fuelImp.position.y = 0.48;
    const turbine = new THREE.Group();
    const turbWheelMat = MAT.titanium();
    turbine.add(new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.03, 48), turbWheelMat));
    const bucketGeo = new THREE.BoxGeometry(0.055, 0.042, 0.012);
    for (let i = 0; i < 44; i++) {
      const piv = new THREE.Group();
      piv.rotation.y = (i / 44) * Math.PI * 2;
      const b = new THREE.Mesh(bucketGeo, turbWheelMat);
      b.position.x = 0.172;
      b.rotation.x = 0.55;
      piv.add(b);
      turbine.add(piv);
    }
    turbine.position.y = 0.24;
    rotor.add(loxImp, fuelImp, turbine);
    tp.add(rotor);
    // Mounting struts to the chamber jacket.
    tp.add(strut(V(0.225 - TP_X, 0.56, -0.07), V(-0.13, 0.56, -0.07), 0.016, dark));
    tp.add(strut(V(0.17 - TP_X, 0.3, -0.08), V(-0.19, 0.3, -0.08), 0.016, dark));
    // Speed pickup with its cable.
    tp.add(rbox(0.03, 0.03, 0.04, 0.006, brass, [0, 0.64, -0.085]));
    addPart(eng, tp, {
      id: 'turbopump',
      name: 'Turbopompa',
      base:
        'Il cuore nascosto del motore: una turbina fa girare due pompe montate sullo stesso albero, a circa 36.000 giri al minuto. Insieme spingono in camera circa 300 kg di propellenti al secondo: svuoterebbero una vasca da bagno in mezzo secondo.',
      expert:
        'Pompe centrifughe monostadio per LOX (in alto) e RP-1, turbina ad azione in basso, tutto su un unico albero. La pressione di mandata deve superare i ~97 bar della camera più le perdite di canali e iniettore. Tra la pompa LOX e quella del cherosene una tenuta spurgata con elio impedisce che i due propellenti si incontrino.',
      explode: [0.9, 0.15, 0.2],
    });

    // ---------- Gas generator ----------
    const ggPart = new THREE.Group();
    const gg = new THREE.Group();
    gg.position.set(...GG_POS);
    const ggMat = cut(MAT.titanium());
    gg.add(
      new THREE.Mesh(
        shell(
          [
            [0.0, 0.15],
            [0.045, 0.145],
            [0.062, 0.12],
            [0.065, 0.1],
            [0.065, -0.08],
            [0.045, -0.1],
            [0.035, -0.12],
          ],
          0.01,
          32
        ),
        ggMat
      )
    );
    const ggHead = new THREE.Mesh(pipe(0.072, 0.0, 0.022, 32), steelCut);
    ggHead.position.y = 0.16;
    gg.add(ggHead, instanced(BOLT, boltMat, circle(0.06, 0.171, 8)));
    ggPart.add(gg);
    ggPart.add(
      tube(
        crv([
          [GG_POS[0], 0.29, GG_POS[2]],
          [1.15, 0.25, -0.19],
          [1.06, 0.24, -0.15],
          [0.99, 0.24, -0.115],
        ]),
        0.035,
        ggMat,
        24,
        10
      )
    );
    ggPart.add(
      tube(crv([[0.92, 0.74, -0.09], [1.06, 0.68, -0.17], [1.17, 0.6, -0.22], [1.2, 0.57, -0.22]]), 0.013, loxLine, 32, 8),
      tube(crv([[0.92, 0.48, -0.09], [1.04, 0.53, -0.16], [1.14, 0.59, -0.21], [1.18, 0.57, -0.22]]), 0.013, brass, 32, 8)
    );
    // GG control valve on the fuel tap.
    ggPart.add(rbox(0.04, 0.04, 0.05, 0.008, actMat, [1.06, 0.535, -0.16]));
    addPart(eng, ggPart, {
      id: 'gasGenerator',
      name: 'Generatore di gas',
      base:
        'Un piccolo “motorino a razzo” che brucia una minima parte dei propellenti solo per far girare la turbina. Il suo gas poi viene buttato fuori: è uno spreco, ma rende il motore più semplice e affidabile.',
      expert:
        'Combustione molto ricca di combustibile (O/F ben sotto 1) per tenere la temperatura entro il limite delle palette della turbina. Due piccole linee prelevano LOX e RP-1 dalle mandate delle pompe; regolando la portata del GG si cambiano i giri della turbopompa e quindi la spinta (fino a circa il 40%).',
      explode: [1.4, 0.2, 0],
    });
    // Flame added after registration: keeps its own material for animation.
    const ggFlameMat = MAT.glow(0xff7a2a, 2, 0.9);
    const ggFlame = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.04, 0.16, 16), ggFlameMat);
    ggFlame.position.set(GG_POS[0], GG_POS[1] + 0.01, GG_POS[2]);
    ggPart.add(ggFlame);

    // ---------- Turbine exhaust duct ----------
    const duct = new THREE.Group();
    const cDuct = crv(DUCT);
    duct.add(tube(cDuct, 0.065, ductMat, 64, 16));
    const flare = new THREE.Mesh(shell([[0.068, 0.02], [0.075, -0.03], [0.1, -0.08]], 0.008, 32), ductMat);
    flare.position.set(...DUCT[DUCT.length - 1]);
    duct.add(flare);
    duct.add(ring(0.07, 0.012, 0.09, ductMat, TP_X, 0)); // flange to the turbine casing
    // Clamp band + strut to the nozzle extension.
    duct.add(ring(0.07, 0.01, -1.0, ductMat, 0.951, 0));
    duct.add(strut(V(rAt(-1.0) + 0.012, -1.0, -0.04), V(0.89, -1.0, -0.04), 0.014, dark));
    addPart(eng, duct, {
      id: 'exhaustDuct',
      name: 'Scarico della turbina',
      base:
        'Il gas che ha fatto girare la turbina esce da questo tubo accanto all’ugello. È il getto scuro e fumoso che si vede nei lanci del Falcon 9: è ricco di cherosene non bruciato.',
      expert:
        'Gas a bassa pressione e temperatura, ricco di fuliggine per la combustione fuel-rich del GG: produce una piccola spinta, con Isp molto inferiore a quello della camera. È la “tassa” del ciclo aperto rispetto ai cicli stadiati, dove questo gas viene iniettato in camera.',
      explode: [1.2, -0.4, 0],
    });

    // ---------- Main valves + discharge lines ----------
    const valves = new THREE.Group();
    const cLoxDis = crv(LOX_DIS);
    const cFuelDis = crv(FUEL_DIS);
    valves.add(tube(cLoxDis, 0.04, loxLine, 48), tube(cFuelDis, 0.034, lineSteel, 64));
    const levers = [];
    const valveBodyGeo = new THREE.SphereGeometry(0.058, 24, 16);
    const flangeGeo = new THREE.CylinderGeometry(0.056, 0.056, 0.018, 24);
    const valveBoltItems = circle(0.046, 0, 6, 0, 0, 0.5);
    const makeValve = (curve, u) => {
      const g = new THREE.Group();
      g.position.copy(curve.getPointAt(u));
      g.quaternion.setFromUnitVectors(V(0, 1, 0), curve.getTangentAt(u));
      const bodyMat = MAT.aluminum();
      g.add(new THREE.Mesh(valveBodyGeo, bodyMat));
      for (const s of [-1, 1]) {
        const f = new THREE.Mesh(flangeGeo, bodyMat);
        f.position.y = s * 0.062;
        const b = instanced(BOLT, dark, valveBoltItems);
        b.position.y = s * 0.062 + (s > 0 ? 0.009 : -0.018);
        g.add(f, b);
      }
      const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.05, 10), steel);
      stem.rotation.x = Math.PI / 2;
      stem.position.z = 0.065;
      const act = rbox(0.075, 0.08, 0.06, 0.01, actMat, [0, 0, 0.115]);
      const lever = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.07, 0.01).translate(0, 0.028, 0), leverMat);
      lever.position.z = 0.15;
      const plug = new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.03, 10), rubber); // connector
      plug.position.set(0.045, 0, 0.115);
      plug.rotation.z = Math.PI / 2;
      g.add(stem, act, lever, plug);
      levers.push(lever);
      return g;
    };
    valves.add(makeValve(cLoxDis, 0.45), makeValve(cFuelDis, 0.4));
    addPart(eng, valves, {
      id: 'valves',
      name: 'Valvole principali e linee di mandata',
      base:
        'Come rubinetti velocissimi, aprono e chiudono il passaggio dei propellenti dalle pompe verso la camera. Servono per accendere, spegnere e regolare il motore.',
      expert:
        'MOV (Main Oxidizer Valve) sulla linea LOX verso la cupola dell’iniettore e MFV (Main Fuel Valve) sulla linea RP-1 verso il collettore di raffreddamento. La loro sequenza di apertura evita picchi di pressione e ambienti caldi ricchi di ossigeno, che corroderebbero il metallo.',
      explode: [-0.3, 0.15, 0.7],
    });

    // ---------- Injector ----------
    const inj = new THREE.Group();
    const PT = Y_INJ + 0.035; // plate top
    const RF = R_C + R_OUT + 0.012; // flange radius
    // Plate with the fuel distribution annulus and the central LOX bore (pintle).
    inj.add(
      new THREE.Mesh(
        latheLoops(
          [
            [0.03, Y_INJ],
            [RF, Y_INJ],
            [RF, PT],
            [0.03, PT],
          ],
          [
            [
              [0.15, Y_INJ + 0.008],
              [0.15, PT - 0.008],
              [0.178, PT - 0.008],
              [0.178, Y_INJ + 0.008],
            ],
          ]
        ),
        steelCut
      )
    );
    // LOX dome (closed shell).
    inj.add(
      new THREE.Mesh(
        shell(
          [
            [0.2, PT],
            [0.197, 0.87],
            [0.17, 0.925],
            [0.11, 0.962],
            [0.04, 0.978],
            [0, 0.982],
          ],
          0.012
        ),
        steelCut
      )
    );
    inj.add(instanced(BOLT, boltMat, circle(RF - 0.012, PT, 24, 0, 0, 0.07)));
    // Hollow pintle: LOX runs down its bore and sprays out radially at the tip.
    inj.add(
      new THREE.Mesh(
        lathe([
          [0.036, PT],
          [0.036, Y_INJ - 0.09],
          [0.052, Y_INJ - 0.102],
          [0.05, Y_INJ - 0.12],
          [0.018, Y_INJ - 0.12],
          [0.018, PT],
          [0.036, PT],
        ], 32),
        chromeCut
      )
    );
    // Igniter (TEA-TEB) line and boss.
    const boss = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, 0.03, 16), steel);
    boss.position.set(-0.085, 0.968, -0.04);
    boss.rotation.z = 0.5;
    inj.add(boss, tube(crv([[-0.09, 0.975, -0.04], [-0.2, 1.0, -0.12], [-0.32, 1.0, -0.2], [-0.46, 1.08, -0.12]]), 0.012, brass, 32, 8));
    addPart(eng, inj, {
      id: 'injector',
      name: 'Piastra iniettori (pintle)',
      base:
        'Come il soffione di una doccia, spruzza ossigeno e cherosene in goccioline finissime e li mescola, così bruciano in pochi millesimi di secondo. Sopra c’è la cupola che distribuisce l’ossigeno.',
      expert:
        'Il Merlin usa un iniettore a pintle: uno spinotto centrale da cui un propellente esce in getti radiali che incrociano la lamina anulare dell’altro. È robusto contro le instabilità di combustione e si presta alla regolazione della spinta. L’accensione avviene con TEA-TEB, che si infiamma da solo a contatto con l’ossigeno (lampo verde).',
      explode: [0, 0.5, 0],
    });

    // ---------- Combustion chamber: copper liner + coolant channels + jacket ----------
    const chamber = new THREE.Group();
    chamber.add(new THREE.Mesh(wallGeo(Y_INJ, Y_BAR, 0, LINER), copper));
    chamber.add(new THREE.Mesh(wallGeo(Y_INJ, Y_BAR, LINER + GAP, R_OUT), jacketMat));
    chamber.add(instanced(ribGeo(Y_INJ, Y_BAR), ribMat, ribItems()));
    for (const y of [0.74, 0.6, 0.46]) chamber.add(ring(R_C + R_OUT + 0.008, 0.01, y, steelCut));
    for (const p of [[-0.245, 0.56, 0], [0, 0.56, -0.245]]) chamber.add(rbox(0.05, 0.06, 0.05, 0.01, dark, p));
    // Pressure transducers with their harness to the junction box.
    const sensorGeo = new THREE.CylinderGeometry(0.014, 0.014, 0.045, 12);
    for (const [a, y] of [[Math.PI * 0.8, 0.7], [Math.PI * 1.25, 0.52]]) {
      const r = R_C + R_OUT + 0.02;
      const s = new THREE.Mesh(sensorGeo, steel);
      s.position.set(Math.cos(a) * r, y, Math.sin(a) * r);
      s.lookAt(0, y, 0);
      s.rotateX(Math.PI / 2);
      chamber.add(s);
    }
    chamber.add(
      tube(
        crv([
          [Math.cos(Math.PI * 1.25) * 0.27, 0.52, Math.sin(Math.PI * 1.25) * 0.27],
          [-0.26, 0.64, -0.13],
          [Math.cos(Math.PI * 0.8) * 0.27, 0.7, Math.sin(Math.PI * 0.8) * 0.27],
          [-0.3, 0.85, 0.02],
          [-0.42, 0.98, -0.06],
          [-0.47, 1.07, -0.09],
        ]),
        0.009,
        rubber,
        48,
        6
      )
    );
    addPart(eng, chamber, {
      id: 'chamber',
      name: 'Camera di combustione',
      base:
        'Qui ossigeno e cherosene bruciano a circa 3400 °C, più del punto di fusione di quasi tutti i metalli. La parete in lega di rame non fonde perché il cherosene, ancora freddo, le scorre dentro in tanti canalini prima di essere bruciato: in sezione si vede il canale tra il rivestimento interno e la camicia esterna.',
      expert:
        'Pc ≈ 97 bar, T ≈ 3400 °C, O/F ≈ 2,36. Liner in lega di rame ad alta conducibilità con canali di raffreddamento e camicia strutturale esterna: raffreddamento rigenerativo, con il calore che torna in camera insieme al combustibile. Il tempo di permanenza dei gas in camera è di pochi millisecondi.',
      explode: [0, 0, 0],
    });

    // ---------- Throat + bell nozzle ----------
    const nozzle = new THREE.Group();
    nozzle.add(new THREE.Mesh(wallGeo(Y_BAR, Y_EXT, 0, LINER), copper));
    nozzle.add(new THREE.Mesh(wallGeo(Y_BAR, Y_EXT, LINER + GAP, R_OUT), jacketMat));
    nozzle.add(instanced(ribGeo(Y_BAR, Y_EXT), ribMat, ribItems()));
    nozzle.add(new THREE.Mesh(wallGeo(Y_EXT, Y_E, 0, 0.014, 72), nozzleExt));
    // Bolted joint between the regen section and the extension.
    const rj = rAt(Y_EXT);
    const joint = new THREE.Mesh(pipe(rj + 0.075, rj + 0.006, 0.022, 64), steelCut);
    joint.position.y = Y_EXT - 0.011;
    nozzle.add(joint, instanced(BOLT, boltMat, circle(rj + 0.062, Y_EXT, 28, 0, 0, 0.05)));
    const man = new THREE.Mesh(hollowTorus(R_MAN, 0.034, 0.024), steelCut); // fuel inlet manifold
    man.position.y = Y_EXT + 0.045;
    nozzle.add(man);
    for (const y of [-0.75, -1.15]) nozzle.add(ring(rAt(y) + 0.02, 0.012, y, darkCut)); // hat bands
    nozzle.add(ring(R_E + 0.02, 0.016, Y_E + 0.008, darkCut));
    addPart(eng, nozzle, {
      id: 'nozzle',
      name: 'Gola e ugello',
      base:
        'L’ugello prima si stringe (la gola) e poi si allarga a campana. Nel punto più stretto i gas raggiungono la velocità del suono; nella campana continuano ad accelerare, raffreddandosi, fino a circa 10.000 km/h. È questa velocità che crea la spinta.',
      expert:
        'Ugello de Laval: nel convergente il flusso subsonico accelera, in gola M = 1, nel divergente il flusso supersonico accelera perché la sezione aumenta. Rapporto d’area ~16 sul Merlin 1D al mare, profilo a campana (Rao). Velocità efficace di scarico ~2,8 km/s al mare e ~3,05 km/s nel vuoto. La parte alta in rame è raffreddata dal combustibile, che entra dal collettore anulare.',
      explode: [0, -0.8, 0],
    });

    // ---------- Gimbal actuators ----------
    const gimbalPart = new THREE.Group();
    const block = rbox(0.13, 0.09, 0.13, 0.015, dark, [0, PIV + 0.03, 0]);
    const pinX = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.2, 12), chrome);
    pinX.rotation.z = Math.PI / 2;
    pinX.position.y = PIV + 0.03;
    const pinZ = pinX.clone();
    pinZ.rotation.set(Math.PI / 2, 0, 0);
    gimbalPart.add(block, pinX, pinZ);
    const actBodyGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.32, 20).translate(0, -0.19, 0);
    const capGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.03, 20);
    const rodGeo = new THREE.CylinderGeometry(0.018, 0.018, 1, 12).translate(0, -0.5, 0);
    const eyeGeo = new THREE.SphereGeometry(0.03, 12, 10);
    const actuators = [];
    const makeActuator = (anchor, attach) => {
      const clevis = rbox(0.07, 0.05, 0.07, 0.01, dark, [anchor[0], anchor[1] + 0.03, anchor[2]]);
      const g = new THREE.Group();
      const port = rbox(0.03, 0.05, 0.03, 0.006, brass, [0.05, -0.1, 0]);
      const rod = new THREE.Mesh(rodGeo, chrome);
      rod.position.y = -0.25;
      const eye = new THREE.Mesh(eyeGeo, dark);
      const c1 = new THREE.Mesh(capGeo, dark);
      c1.position.y = -0.04;
      const c2 = new THREE.Mesh(capGeo, dark);
      c2.position.y = -0.34;
      g.add(new THREE.Mesh(actBodyGeo, actMat), new THREE.Mesh(eyeGeo, dark), c1, c2, port, rod, eye);
      gimbalPart.add(clevis, g);
      actuators.push({ g, rod, eye, port, anchor: V(...anchor), attach: V(...attach) });
    };
    makeActuator([-0.62, 1.11, 0], [-0.27, 0.56, 0]);
    makeActuator([0, 1.11, -0.62], [0, 0.56, -0.27]);

    const tmpP = new THREE.Vector3();
    const tmpD = new THREE.Vector3();
    const DOWN = V(0, -1, 0);
    function poseActuators() {
      gimbal.updateMatrix();
      for (const a of actuators) {
        tmpP.copy(a.attach).applyMatrix4(eng.matrix).applyMatrix4(gimbal.matrix);
        tmpD.subVectors(tmpP, a.anchor);
        const d = tmpD.length();
        tmpD.divideScalar(d);
        a.g.position.copy(a.anchor);
        a.g.quaternion.setFromUnitVectors(DOWN, tmpD);
        a.rod.scale.y = Math.max(0.01, d - 0.25);
        a.eye.position.y = -d;
      }
    }
    poseActuators();
    // Hydraulic hoses from the thrust structure to each actuator port (posed at rest).
    for (const a of actuators) {
      const p = a.port.position.clone().applyQuaternion(a.g.quaternion).add(a.g.position);
      const up = a.anchor.clone().add(V(a.anchor.x ? 0.1 : 0.06, 0.05, a.anchor.z ? 0.1 : 0.06));
      gimbalPart.add(tube(new THREE.CatmullRomCurve3([up, p.clone().lerp(up, 0.5).add(V(0.04, -0.03, 0.04)), p]), 0.01, rubber, 24, 6));
    }
    addPart(root, gimbalPart, {
      id: 'gimbal',
      name: 'Attuatori di gimbal',
      base:
        'Due pistoni idraulici inclinano tutto il motore di pochi gradi, come quando si orienta il getto di una canna dell’acqua: così il razzo sterza e resta dritto.',
      expert:
        'Il motore è appeso a un giunto cardanico sotto la struttura di spinta; due attuatori a 90° comandano beccheggio e imbardata (qualche grado). Sul Falcon 9 il rollio si ottiene orientando in modo differenziale i nove motori, e gli attuatori usano come fluido idraulico lo stesso RP-1 ad alta pressione.',
      explode: [-1.0, 0.5, 0],
    });

    // ---------- Hot gas inside chamber and nozzle (not clickable) ----------
    const chamberGasMat = cut(MAT.gas(0xffa040, 0.15));
    const bellGasMat = cut(MAT.gas(0xff6a2a, 0.18));
    const gasVolumes = [
      new THREE.Mesh(lathe([[0, Y_INJ - 0.01], ...profSlice(Y_INJ - 0.01, Y_T - 0.1, 0.95), [0, Y_T - 0.1]], 48), chamberGasMat),
      new THREE.Mesh(lathe([[0, Y_T - 0.1], ...profSlice(Y_T - 0.1, Y_E + 0.01, 0.96), [0, Y_E + 0.01]], 48), bellGasMat),
    ];
    eng.add(...gasVolumes);

    // ---------- Exhaust plume ----------
    const plume = new THREE.Group();
    plume.position.y = Y_E;
    eng.add(plume);
    const plumeBody = new THREE.Group();
    plume.add(plumeBody);
    // Additive gas: darker vertex colour = more transparent, so the plume fades with distance.
    const fadeLathe = (pts, fade) => {
      const geo = lathe(pts, 48);
      const pos = geo.attributes.position;
      const col = new Float32Array(pos.count * 3);
      for (let i = 0; i < pos.count; i++) col.fill(clamp(fade(-pos.getY(i)), 0, 1), i * 3, i * 3 + 3);
      geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
      return geo;
    };
    const gasMat = (color, opacity) => {
      const m = MAT.gas(color, opacity);
      m.vertexColors = true;
      return m;
    };
    const outerPts = [
      [0.43, 0],
      [0.39, -0.08],
      [0.37, -0.18],
      [0.39, -0.3],
      [0.46, -0.48],
      [0.56, -0.66],
      [0.68, -0.84],
      [0.8, -1.0],
    ];
    const outerMat = gasMat(0xff8a3a, 0.22);
    const haloMat = gasMat(0xff5a1f, 0.07);
    const coreMat = gasMat(0xfff3d8, 0.7);
    plumeBody.add(
      new THREE.Mesh(fadeLathe(outerPts, (d) => Math.pow(1 - d, 1.3)), outerMat),
      new THREE.Mesh(
        fadeLathe(outerPts.map(([r, y]) => [r * 1.3 + 0.04, y * 1.08]), (d) => 1 - d),
        haloMat
      ),
      new THREE.Mesh(
        fadeLathe([[0.41, 0], [0.36, -0.08], [0.27, -0.18], [0.15, -0.28], [0.05, -0.36], [0, -0.38]], (d) => 1 - d * 1.6),
        coreMat
      )
    );
    // Mach diamonds.
    const diamondGeo = new THREE.SphereGeometry(1, 20, 14);
    const diamonds = [0, 1, 2].map((i) => {
      const m = new THREE.Mesh(diamondGeo, MAT.gas(0xfff0c8, 0.5));
      m.scale.set(0.13 - 0.025 * i, 0.2, 0.13 - 0.025 * i);
      plume.add(m);
      return m;
    });
    const exitLight = new THREE.PointLight(0xff9a50, 2, 6, 2);
    exitLight.position.set(0, Y_E - 0.5, 0.2);
    eng.add(exitLight);

    // ---------- Flows ----------
    // Feed lines (static frame): LOX tank -> pump -> injector dome -> pintle.
    const cLoxFlow = crv([
      [0.5, 2.15, 0],
      ...LOX_DOWN,
      [0.78, 0.82, 0],
      [0.7, 0.765, 0],
      ...LOX_DIS,
      [0.08, 0.9, 0],
      [0.0, 0.86, 0],
      [0, 0.7, 0],
    ]);
    const loxFlow = flow({
      path: (t, out) => cLoxFlow.getPointAt(t, out),
      count: 360,
      size: 0.05,
      colors: [0x5fd3ff, 0x5fd3ff, 0x9fe6ff],
      spread: (t) => (t < 0.05 ? 0.1 : 0.02),
      speed: 0.16,
    });
    root.add(loxFlow.object);
    // RP-1 tank -> fuel pump -> MFV -> cooling manifold.
    const cFuelFlow = crv([
      [0.8, 1.68, -0.3],
      ...FUEL_FEED,
      [0.78, 0.52, -0.04],
      [0.7, 0.48, 0],
      ...FUEL_DIS,
    ]);
    const fuelFlow = flow({
      path: (t, out) => cFuelFlow.getPointAt(t, out),
      count: 300,
      size: 0.05,
      colors: [0xffc04d],
      spread: (t) => (t < 0.05 ? 0.06 : 0.02),
      speed: 0.17,
    });
    root.add(fuelFlow.object);
    // Regenerative cooling: fuel rising in the channels seen at both cut edges of the wall.
    const STRAND_ANG = [Math.PI + 0.03, Math.PI * 2 - 0.03];
    const coolFlow = flow({
      path: (t, out) => {
        const f = t * STRAND_ANG.length;
        const k = Math.min(Math.floor(f), STRAND_ANG.length - 1);
        const y = lerp(Y_EXT + 0.02, Y_INJ - 0.005, f - k);
        const r = rAt(y) + LINER + GAP / 2;
        out.set(Math.cos(STRAND_ANG[k]) * r, y, Math.sin(STRAND_ANG[k]) * r);
      },
      count: 200,
      size: 0.035,
      colors: [0xffc04d],
      spread: 0.004,
      speed: 0.11,
    });
    eng.add(coolFlow.object);
    // Combustion: pintle spray -> chamber -> throat -> fast through the bell.
    const combY = (t) =>
      t < 0.12
        ? lerp(Y_INJ - 0.11, Y_INJ - 0.18, t / 0.12)
        : t < 0.55
          ? lerp(Y_INJ - 0.18, Y_T, (t - 0.12) / 0.43)
          : lerp(Y_T, Y_E, Math.pow((t - 0.55) / 0.45, 1.25));
    const combFlow = flow({
      path: (t, out) => out.set(0, combY(t), 0),
      count: 420,
      size: 0.07,
      colors: [0xfff1c4, 0xffb347, 0xff5a1f, 0xff7a30],
      spread: (t) => (t < 0.12 ? lerp(0.03, 0.14, t / 0.12) : rAt(combY(t)) * 0.82),
      speed: 0.5,
    });
    eng.add(combFlow.object);
    // Gas generator -> turbine -> exhaust duct.
    const cHot = crv([
      [GG_POS[0], 0.46, GG_POS[2]],
      [GG_POS[0], 0.3, GG_POS[2]],
      [1.13, 0.255, -0.18],
      [0.99, 0.24, -0.115],
      [0.9, 0.24, -0.05],
      [0.8, 0.17, 0],
      ...DUCT,
    ]);
    const hotFlow = flow({
      path: (t, out) => cHot.getPointAt(t, out),
      count: 160,
      size: 0.05,
      colors: [0xff5a1f, 0xc07060, 0x8c86a8],
      spread: 0.022,
      speed: 0.3,
    });
    eng.add(hotFlow.object);
    // Sooty turbine exhaust: normal blending so dark particles stay visible.
    const cSmoke = crv([
      [1.0, -1.52, 0],
      [1.05, -1.95, 0],
      [1.16, -2.5, 0.05],
      [1.32, -3.1, 0.1],
      [1.5, -3.6, 0.15],
    ]);
    const smokeFlow = flow({
      path: (t, out) => cSmoke.getPointAt(t, out),
      count: 220,
      size: 0.16,
      colors: [0x8c86a8, 0x4a4656, 0x2c2a33],
      spread: (t) => 0.05 + 0.4 * t,
      speed: 0.45,
      opacity: 0.5,
    });
    smokeFlow.material.blending = THREE.NormalBlending;
    eng.add(smokeFlow.object);
    // Main plume particles; length follows thrust.
    let plumeLen = 1.6;
    const plumeFlow = flow({
      path: (t, out) => out.set(0, Y_E - 0.03 - t * plumeLen, 0),
      count: 650,
      size: 0.1,
      colors: [0xfff4d6, 0xffb050, 0xff6a20, 0x6a2410],
      spread: (t) => 0.3 + 0.5 * t,
      speed: 1.4,
      opacity: 0.8,
    });
    eng.add(plumeFlow.object);

    const internalFlows = [loxFlow, fuelFlow, coolFlow, combFlow, hotFlow];
    [...internalFlows, smokeFlow, plumeFlow].forEach((f) => ctx.addFlow(f));

    function posePlume(T, fl) {
      plumeLen = 0.9 + 1.1 * T;
      const w = 0.9 + 0.15 * T;
      plumeBody.scale.set(w, plumeLen, w);
      outerMat.opacity = (0.1 + 0.18 * T) * fl;
      haloMat.opacity = (0.03 + 0.06 * T) * fl;
      coreMat.opacity = clamp((0.25 + 0.4 * T) * fl, 0, 1);
      const spacing = 0.75 + 0.35 * T;
      diamonds.forEach((d, i) => {
        d.position.y = -(0.34 + i * 0.44) * spacing;
        d.material.opacity = (0.55 - 0.14 * i) * (0.3 + 0.7 * T) * fl;
      });
    }
    posePlume(thrustOf(0.35), 1);

    return {
      root,
      cycleRate(throttle) {
        return 0.25 + 0.35 * throttle;
      },
      update(state) {
        const T = thrustOf(state.throttle);
        const { dt, time } = state;
        const fl = 1 + 0.05 * Math.sin(time * 41) + 0.04 * Math.sin(time * 27.3 + 1.7);

        rotor.rotation.y = state.cycle * Math.PI * 2 * ROTOR_TURNS;
        gimbal.rotation.set(0.04 * Math.sin(time * 0.5 + 1.0), 0, 0.05 * Math.sin(time * 0.7));
        poseActuators();
        for (const l of levers) l.rotation.z = -0.3 - 1.0 * T;

        posePlume(T, fl);
        chamberGasMat.opacity = (0.06 + 0.1 * T) * fl;
        bellGasMat.opacity = (0.04 + 0.08 * T) * fl;
        ggFlameMat.emissiveIntensity = (1.4 + 2.4 * T) * fl;
        exitLight.intensity = (1.2 + 3.5 * T) * fl;
        plumeFlow.material.opacity = 0.5 + 0.4 * T;

        for (const f of internalFlows) f.update(dt, T);
        smokeFlow.update(dt, 0.6 + 0.4 * T);
        plumeFlow.update(dt, 0.6 + 0.4 * T);
        // Flows, gas and plume would float in mid-air once exploded.
        const assembled = (state.explode || 0) < 0.05;
        plume.visible = smokeFlow.object.visible = plumeFlow.object.visible = assembled;
        ggFlame.visible = exitLight.visible = assembled;
        for (const g of gasVolumes) g.visible = assembled;
        for (const f of internalFlows) f.object.visible = assembled;
      },
      readout(state) {
        const T = thrustOf(state.throttle);
        return [
          { label: 'Spinta', value: Math.round(845 * T), unit: 'kN' },
          { label: 'Pressione camera', value: Math.round(97 * T), unit: 'bar' },
          { label: 'Giri turbopompa', value: Math.round((36000 * Math.sqrt(T)) / 100) * 100, unit: 'giri/min' },
          { label: 'Consumo propellenti', value: Math.round(305 * T), unit: 'kg/s' },
        ];
      },
    };
  },
};
