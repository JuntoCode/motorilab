// Single-cylinder crankcase-scavenged two-stroke: reed valve, loop scavenging, expansion chamber.
// Crank axis along Z so the full-circle crank wheels face the camera; the piston itself opens the ports.
// Cycle 0..1 = one crank revolution, starting when the exhaust port closes (266° after TDC).
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { MAT, cut, lathe, pipe, flow, clamp, lerp, windowFn } from '../core/helpers.js';

const R = 0.36; // crank radius (stroke 0.72)
const L = 1.3; // conrod length
const BORE = 0.42; // bore radius
const CROWN = 0.3; // crown edge above wrist pin
const SKIRT = 0.42; // skirt below wrist pin
const HEAD_Y = 2.0; // squish face of the head
const LINER_BOT = 0.62; // bottom of the spigot inside the crankcase
const DECK_Y = 0.88; // crankcase / barrel joint
const B_BOT = 0.95; // top of the barrel base flange
const CASE_Z = 0.52; // half depth of the crankcase cavity
const R_SK = 0.47; // transfer channel: inner skin outer radius
const R_CH = 0.555; // transfer channel: outer radius
const R_OUT = 0.6; // barrel outer radius
const A0 = 266; // crank angle (deg after TDC) at cycle 0 = exhaust port closing
const EVO = 97; // exhaust port opens (closes at 360 - EVO)
const TPO = 122; // transfer ports open
const IGN = 342; // ignition, 18° before TDC
const DEG = Math.PI / 180;

const pinY = (deg) => {
  const t = deg * DEG;
  const xc = R * Math.sin(t);
  return R * Math.cos(t) + Math.sqrt(L * L - xc * xc);
};
// Port heights follow from the timing: the crown edge uncovers them at EVO / TPO.
const PORT_BOT = pinY(180) + CROWN - 0.01;
const EX_TOP = pinY(EVO) + CROWN;
const TR_TOP = pinY(TPO) + CROWN;
const angleOf = (c) => (((c * 360 + A0) % 360) + 360) % 360;

// Port windows. a = angle around Y with x = r sin a, z = r cos a (Three's cylinder/lathe convention).
const WINDOWS = {
  ex: { a: Math.PI / 2, w: 1.2, y0: PORT_BOT, y1: EX_TOP },
  trB: { a: Math.PI + 0.55, w: 0.8, y0: PORT_BOT, y1: TR_TOP },
  boost: { a: Math.PI * 1.5, w: 0.45, y0: PORT_BOT, y1: TR_TOP },
  trF: { a: Math.PI * 2 - 0.55, w: 0.8, y0: PORT_BOT, y1: TR_TOP },
};

// Exhaust duct + expansion chamber centreline (z = 0 plane, so the cutaway halves it).
const EXH_PTS = [
  [0.43, 1.37], [0.75, 1.36], [1.05, 1.28], [1.3, 1.08], [1.48, 0.75], [1.6, 0.3], [1.64, -0.2],
  [1.55, -0.65], [1.3, -1.0], [0.9, -1.2], [0.4, -1.28], [-0.1, -1.3], [-0.5, -1.3],
];
// Radius along the chamber (arc-length fraction): header, diffuser, belly, baffle cone, stinger.
const SEAMS = [0.22, 0.42, 0.56, 0.76];
function chamberR(u) {
  if (u < 0.04) return lerp(0.15, 0.13, u / 0.04);
  if (u < SEAMS[0]) return 0.13;
  if (u < SEAMS[1]) return lerp(0.13, 0.36, (u - SEAMS[0]) / 0.2);
  if (u < SEAMS[2]) return 0.36;
  if (u < SEAMS[3]) return lerp(0.36, 0.085, (u - SEAMS[2]) / 0.2);
  return 0.075;
}

const v3 = (x, y, z = 0) => new THREE.Vector3(x, y, z);
const curve = (pts) => new THREE.CatmullRomCurve3(pts, false, 'centripetal');

// ---------- Closed-solid geometry helpers (section caps need watertight meshes) ----------
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
  const n = g.attributes.normal?.array;
  if (n) for (let i = 0; i < n.length; i++) n[i] = -n[i];
  return g;
}
// Solid annular sector: radii r0..r1, heights y0..y1, angles a0..a1 (closed, outward).
function sectorSolid(r0, r1, y0, y1, a0, a1, segs) {
  const P = [];
  const N = [];
  const full = a1 - a0 > Math.PI * 2 - 1e-6;
  const pt = (r, a, y) => [r * Math.sin(a), y, r * Math.cos(a)];
  const tri = (A, B, C, nA, nB, nC) => {
    P.push(...A, ...B, ...C);
    N.push(...nA, ...nB, ...nC);
  };
  const quad = (A, B, C, D, nA, nB = nA, nC = nA, nD = nA) => {
    tri(A, B, C, nA, nB, nC);
    tri(A, C, D, nA, nC, nD);
  };
  for (let i = 0; i < segs; i++) {
    const u0 = a0 + ((a1 - a0) * i) / segs;
    const u1 = a0 + ((a1 - a0) * (i + 1)) / segs;
    const o0 = [Math.sin(u0), 0, Math.cos(u0)];
    const o1 = [Math.sin(u1), 0, Math.cos(u1)];
    const i0 = o0.map((v) => -v);
    const i1 = o1.map((v) => -v);
    quad(pt(r1, u0, y0), pt(r1, u1, y0), pt(r1, u1, y1), pt(r1, u0, y1), o0, o1, o1, o0);
    quad(pt(r0, u0, y0), pt(r0, u0, y1), pt(r0, u1, y1), pt(r0, u1, y0), i0, i0, i1, i1);
    quad(pt(r0, u0, y1), pt(r1, u0, y1), pt(r1, u1, y1), pt(r0, u1, y1), [0, 1, 0]);
    quad(pt(r0, u0, y0), pt(r0, u1, y0), pt(r1, u1, y0), pt(r1, u0, y0), [0, -1, 0]);
  }
  if (!full) {
    const t0 = [-Math.cos(a0), 0, Math.sin(a0)];
    const t1 = [Math.cos(a1), 0, -Math.sin(a1)];
    quad(pt(r0, a0, y0), pt(r1, a0, y0), pt(r1, a0, y1), pt(r0, a0, y1), t0);
    quad(pt(r0, a1, y0), pt(r0, a1, y1), pt(r1, a1, y1), pt(r1, a1, y0), t1);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(P, 3));
  g.setAttribute('normal', new THREE.Float32BufferAttribute(N, 3));
  return orientOutward(g);
}
// Lathe of a closed (r, y) loop, made watertight and outward.
function solidLathe(pts, segs = 48) {
  const g = lathe([...pts, pts[0]], segs).toNonIndexed();
  g.deleteAttribute('uv');
  return orientOutward(g);
}
// Thick-walled tube along a curve with variable outer radius (expansion chamber).
function sweepSolid(c, radiusAt, wall, tubular = 150, radial = 28) {
  const frames = c.computeFrenetFrames(tubular, false);
  const centres = [];
  for (let i = 0; i <= tubular; i++) centres.push(c.getPointAt(i / tubular));
  // Closed profile loop: inner start -> outer along -> inner back.
  const loop = [[0, radiusAt(0) - wall]];
  for (let i = 0; i <= tubular; i++) loop.push([i, radiusAt(i / tubular)]);
  for (let i = tubular; i >= 1; i--) loop.push([i, radiusAt(i / tubular) - wall]);
  const pos = [];
  for (let j = 0; j <= radial; j++) {
    const v = (j / radial) * Math.PI * 2;
    const s = Math.sin(v);
    const k = -Math.cos(v);
    for (const [i, r] of loop) {
      const n = frames.normals[i];
      const b = frames.binormals[i];
      const P = centres[i];
      pos.push(P.x + r * (k * n.x + s * b.x), P.y + r * (k * n.y + s * b.y), P.z + r * (k * n.z + s * b.z));
    }
  }
  const M = loop.length;
  const idx = [];
  for (let j = 0; j < radial; j++) {
    for (let q = 0; q < M; q++) {
      const a = j * M + q;
      const b = j * M + ((q + 1) % M);
      const c2 = (j + 1) * M + q;
      const d = (j + 1) * M + ((q + 1) % M);
      idx.push(a, c2, b, b, c2, d);
    }
  }
  let g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  g = g.toNonIndexed();
  return orientOutward(g);
}
// Extrude a shape along Z (depth from z0) as a closed solid.
const extrude = (shape, depth, curveSegments = 32) =>
  new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments });

// Thin tube outlining a port window on the liner.
function windowFrame(r, w, mat) {
  const P = (a, y) => v3(r * Math.sin(a), y, r * Math.cos(a));
  const a0 = w.a - w.w / 2;
  const a1 = w.a + w.w / 2;
  const pts = [];
  for (let i = 0; i <= 8; i++) pts.push(P(lerp(a0, a1, i / 8), w.y0));
  for (let i = 0; i <= 8; i++) pts.push(P(lerp(a1, a0, i / 8), w.y1));
  const c = new THREE.CatmullRomCurve3(pts, true, 'catmullrom', 0);
  return new THREE.Mesh(new THREE.TubeGeometry(c, 90, 0.013, 5, true), mat);
}

function ringShape(r0, r1) {
  const s = new THREE.Shape();
  s.absarc(0, 0, r1, 0, Math.PI * 2, false);
  const h = new THREE.Path();
  h.absarc(0, 0, r0, 0, Math.PI * 2, true);
  s.holes.push(h);
  return s;
}
function arcShape(r0, r1, a0, a1, seg = 40) {
  const s = new THREE.Shape();
  for (let i = 0; i <= seg; i++) {
    const a = lerp(a0, a1, i / seg);
    i ? s.lineTo(r1 * Math.cos(a), r1 * Math.sin(a)) : s.moveTo(r1 * Math.cos(a), r1 * Math.sin(a));
  }
  for (let i = seg; i >= 0; i--) {
    const a = lerp(a0, a1, i / seg);
    s.lineTo(r0 * Math.cos(a), r0 * Math.sin(a));
  }
  return s;
}
// Fin with a straight notch around +X where the exhaust boss sits (half-width hz).
function notchedFinShape(r0, r1, hz) {
  const b1 = Math.asin(hz / r1);
  const b0 = Math.asin(hz / r0);
  const s = new THREE.Shape();
  const n = 56;
  for (let i = 0; i <= n; i++) {
    const a = lerp(b1, Math.PI * 2 - b1, i / n);
    i ? s.lineTo(r1 * Math.cos(a), r1 * Math.sin(a)) : s.moveTo(r1 * Math.cos(a), r1 * Math.sin(a));
  }
  for (let i = n; i >= 0; i--) {
    const a = lerp(b0, Math.PI * 2 - b0, i / n);
    s.lineTo(r0 * Math.cos(a), r0 * Math.sin(a));
  }
  return s;
}
// Horizontal plate at height y (shape drawn in XZ: shape x = world x, shape y = -world z).
function plateGeo(shape, y, thick = 0.032, curveSegments = 40) {
  const g = extrude(shape, thick, curveSegments);
  g.rotateX(-Math.PI / 2);
  g.translate(0, y, 0);
  return g;
}
// Annular-sector hole in plate (shape) coordinates.
function sectorPath(r0, r1, a0, a1, n = 12) {
  const p = new THREE.Path();
  const at = (r, a) => [r * Math.sin(a), -r * Math.cos(a)];
  for (let i = 0; i <= n; i++) {
    const [x, y] = at(r1, lerp(a0, a1, i / n));
    i ? p.lineTo(x, y) : p.moveTo(x, y);
  }
  for (let i = n; i >= 0; i--) p.lineTo(...at(r0, lerp(a0, a1, i / n)));
  return p;
}

// Hex bolt heads as one InstancedMesh. items: [[x,y,z], [nx,ny,nz]] (head sits on the surface along n).
const HEX = new THREE.CylinderGeometry(1, 1, 1, 6);
function bolts(items, mat, r = 0.028, h = 0.03) {
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
// Coil spring between two points.
function spring(a, b, coils, rad, wire, mat) {
  const d = b.clone().sub(a);
  const len = d.length();
  const c = new (class extends THREE.Curve {
    getPoint(t, out = new THREE.Vector3()) {
      const ang = t * Math.PI * 2 * coils;
      return out.set(Math.cos(ang) * rad, t * len, Math.sin(ang) * rad);
    }
  })();
  const m = new THREE.Mesh(new THREE.TubeGeometry(c, coils * 12, wire, 5), mat);
  m.position.copy(a);
  m.quaternion.setFromUnitVectors(v3(0, 1, 0), d.normalize());
  return m;
}
// Box strut between two points in the XY plane.
function strut(x0, y0, x1, y1, w, d, mat, z = 0) {
  const len = Math.hypot(x1 - x0, y1 - y0);
  const m = new THREE.Mesh(new RoundedBoxGeometry(w, len, d, 2, Math.min(w, d) * 0.3), mat);
  m.position.set((x0 + x1) / 2, (y0 + y1) / 2, z);
  m.rotation.z = -Math.atan2(x1 - x0, y1 - y0);
  return m;
}

// Port openings (0..1) and reed lift for a crank angle in degrees.
function portState(a) {
  const crown = pinY(a) + CROWN;
  const ef = clamp((EX_TOP - crown) / (EX_TOP - PORT_BOT), 0, 1);
  const tf = clamp((TR_TOP - crown) / (TR_TOP - PORT_BOT), 0, 1);
  const ai = (a - 245 + 360) % 360; // reed open from ~245° to ~50° after TDC
  const reed = ai < 165 ? Math.sin((Math.PI * ai) / 165) : 0;
  return { crown, ef, tf, reed };
}

export default {
  id: 'twoStroke',
  name: 'Motore a 2 tempi a miscela',
  short: 'Moto 2 tempi',
  family: 'terra',
  vehicle: 'Scooter, moto da cross, motoseghe',
  examples: 'Moto da cross ed enduro, vecchi scooter 50 cm³, Vespa PX, motoseghe, decespugliatori, fuoribordo',
  tagline: 'Niente valvole: è il pistone stesso ad aprire e chiudere le finestre del cilindro. Uno scoppio a ogni giro.',
  intro: {
    base:
      'Un motore senza valvole: il pistone, salendo e scendendo, scopre e copre delle <b>finestre</b> (le luci) nella parete del cilindro. Sotto il pistone il <b>carter</b> fa da pompa: aspira la miscela di benzina e olio e la spinge nel cilindro attraverso i canali di travaso. Così c’è uno scoppio a <b>ogni giro</b> dell’albero, il doppio di un 4 tempi: tanta potenza in un motore piccolo e leggero.',
    expert:
      'Motore ad accensione comandata con <b>lavaggio a pompa carter</b>: il ciclo si chiude in 360° di albero. Sopra il pistone avvengono compressione ed espansione, sotto il pistone aspirazione (valvola lamellare) e precompressione nel carter (rapporto ≈ 1,3–1,5). Lavaggio a correnti tangenziali (Schnürle) e camera di espansione accordata che sfrutta le onde di pressione. Potenze specifiche di 200–250 kW/l nei 125 da cross, ma rendimento del 20–25% ed emissioni di HC elevate per la miscela che fugge allo scarico.',
  },
  specs: [
    { label: 'Carburante', value: 'Benzina + olio al 2 % (50:1)' },
    { label: 'Ciclo', value: '2 tempi a pompa carter, 1 fase utile a giro' },
    { label: 'Distribuzione', value: 'Nessuna valvola: luci nel cilindro, aspirazione lamellare o a disco rotante' },
    { label: 'Cilindrata tipica', value: '50 – 300 cm³' },
    { label: 'Potenza tipica', value: '3 – 40 kW' },
    { label: 'Regime massimo', value: '9 000 – 13 000 giri/min' },
    { label: 'Rendimento', value: '20 – 25 %' },
  ],
  variants: {
    columns: ['Motore', 'Cilindrata', 'Alimentazione', 'Aspirazione'],
    rows: [
      ['KTM 300 EXC TPI (enduro)', '293 cm³', 'Iniezione nei travasi (TPI)', 'Lamellare nel carter'],
      ['Yamaha YZ125 (cross)', '125 cm³', 'Carburatore', 'Lamellare nel carter'],
      ['Vespa PX 125 / 150', '123 / 150 cm³', 'Carburatore', 'Disco rotante (spalla dell’albero)'],
      ['Motoseghe professionali (es. Stihl MS 261)', '≈ 50 cm³', 'Carburatore a membrana', 'Luce comandata dal pistone'],
      ['Fuoribordo Evinrude E‑TEC', 'varie', 'Iniezione diretta', 'Lamellare nel carter'],
    ],
    note: 'Nessun 2 tempi ha valvole in testa: le luci nel cilindro fanno da distribuzione. Cambia solo il modo in cui si apre e si chiude l’aspirazione nel carter.',
  },
  metrics: { efficiency: 22, power: 30, powerDensity: 1.6 },
  phases: [
    {
      id: 'compression',
      name: 'Compressione e aspirazione nel carter',
      range: [0, 0.261],
      parts: ['piston', 'reedValve', 'carburetor', 'crankcase', 'sparkPlug'],
      base: 'Tutte le finestre sono coperte: il pistone sale e schiaccia la miscela nel cilindro. Intanto sotto di lui, nel carter, si crea un risucchio: le lamelle si aprono come una porta a spinta e la miscela fresca entra dal carburatore. Un solo movimento, due lavori!',
      expert:
        'Dalla chiusura della luce di scarico (≈ 94° prima del PMS) al PMS. Il rapporto di compressione effettivo, contato dalla chiusura dello scarico, è ≈ 7–9:1. Nel carter la depressione apre il pacco lamellare e richiama la miscela aria‑benzina‑olio. L’accensione elettronica (CDI) fa scoccare la scintilla con 15–25° di anticipo, qui a 18° prima del PMS.',
    },
    {
      id: 'power',
      name: 'Scoppio ed espansione',
      range: [0.261, 0.531],
      parts: ['sparkPlug', 'piston', 'conrod', 'crankshaft', 'head'],
      base: 'La scintilla ha acceso la miscela: il gas caldissimo spinge giù il pistone. Scendendo, il pistone schiaccia anche la miscela fresca che si trova nel carter, preparandola per il prossimo giro.',
      expert:
        'Dal PMS all’apertura della luce di scarico (≈ 97° dopo il PMS). Picco di pressione 50–70 bar poco dopo il PMS. Il lato inferiore del pistone precomprime la carica nel carter (le lamelle si richiudono). L’espansione utile è corta rispetto a un 4T perché lo scarico si apre presto: è una delle ragioni del rendimento modesto.',
    },
    {
      id: 'exhaust',
      name: 'Scarico (si apre la luce)',
      range: [0.531, 0.6],
      parts: ['ports', 'exhaustPipe', 'piston'],
      base: 'Scendendo, il bordo del pistone scopre la finestra di scarico: i gas bruciati, ancora in pressione, escono di colpo nella marmitta a pancia. È questo “sbuffo” a dare al 2 tempi il suo suono acuto.',
      expert:
        'Scarico spontaneo (blowdown) tra apertura dello scarico e apertura dei travasi, ≈ 25° di albero. L’onda di pressione corre nel collettore a ~500 m/s; nel cono divergente della camera di espansione si riflette come onda di depressione che aiuta a svuotare il cilindro. Molti motori hanno una valvola parzializzatrice (power valve) che abbassa la luce ai bassi regimi.',
    },
    {
      id: 'transfer',
      name: 'Travaso e lavaggio',
      range: [0.6, 0.922],
      parts: ['transferDucts', 'ports', 'crankcase', 'piston'],
      base: 'Ora il pistone scopre anche le finestre di travaso: la miscela compressa nel carter sale dai canali ed entra nel cilindro, spazzando via i gas bruciati verso lo scarico. Una parte di miscela fresca però scappa dritta nella marmitta: è il grande difetto del 2 tempi.',
      expert:
        'Travasi aperti ≈ 122°–238° (≈ 116° di durata). Lavaggio a correnti tangenziali (Schnürle): due travasi laterali e un travaso di boost opposto allo scarico dirigono il getto verso la parete e la testa, formando un anello che spinge il gas combusto verso la luce di scarico. Con lo scarico aperto si perde in corto circuito il 15–30% della carica nei motori a carburatore: benzina incombusta (HC) e olio allo scarico.',
    },
    {
      id: 'returnWave',
      name: 'Onda di ritorno e chiusura',
      range: [0.922, 1],
      parts: ['exhaustPipe', 'ports', 'piston'],
      base: 'Il pistone risale e chiude i travasi, ma lo scarico è ancora aperto. Proprio ora dalla marmitta a pancia torna indietro un’“onda” di pressione che rispinge nel cilindro la miscela scappata. Poi il pistone chiude lo scarico e si ricomincia.',
      expert:
        'Tra chiusura dei travasi (238°) e chiusura dello scarico (266°). Il cono convergente della camera di espansione riflette un’onda positiva che, se arriva in fase, tappa la luce e riporta nel cilindro la carica fuggita: una sovralimentazione dinamica. Funziona in una banda di giri ristretta, da cui l’erogazione “a scoppio” dei 2T da competizione.',
    },
  ],
  quiz: [
    {
      q: 'Quante volte scoppia la miscela in un 2 tempi a ogni giro dell’albero motore?',
      options: ['Mai', 'Una volta', 'Due volte', 'Una volta ogni due giri'],
      correct: 1,
      explain: 'Il ciclo completo dura un solo giro (360°): c’è una fase utile a ogni giro, il doppio di un 4 tempi allo stesso regime.',
    },
    {
      q: 'Chi apre e chiude le luci di scarico e di travaso?',
      options: ['Le valvole a fungo', 'L’albero a camme', 'Il pistone stesso, scorrendo davanti alle finestre', 'La candela'],
      correct: 2,
      explain: 'Nel 2 tempi non ci sono valvole in testa: il bordo del pistone scopre e copre le luci. La fasatura dipende solo dall’altezza delle finestre.',
    },
    {
      q: 'Perché nel serbatoio di un 2 tempi classico si mette olio insieme alla benzina?',
      options: [
        'Per aumentare il numero di ottano',
        'Perché la miscela passa dal carter e deve lubrificare albero, biella e pistone',
        'Per raffreddare la marmitta',
        'Per far partire il motore a freddo',
      ],
      correct: 1,
      explain: 'Il carter è pieno di miscela fresca, non di olio: l’olio (circa il 2%) deve viaggiare con il carburante. Bruciando, produce il tipico fumo azzurrognolo.',
    },
    {
      q: 'Qual è il principale difetto ambientale del 2 tempi a carburatore?',
      options: [
        'È troppo silenzioso',
        'Parte della miscela fresca esce dallo scarico senza bruciare',
        'Consuma l’olio del cambio',
        'Non ha bisogno della candela',
      ],
      correct: 1,
      explain: 'Durante il lavaggio scarico e travasi sono aperti insieme e una parte della carica fugge in corto circuito. Per questo con l’Euro 5 quasi tutti i 2T stradali sono spariti; restano nel fuoristrada, nelle motoseghe e nelle versioni a iniezione (es. TPI).',
    },
  ],
  camera: { position: [2.15, 1.45, 6.1], target: [0, 0.28, 0] },
  thumb: { orbit: [28, 14], zoom: 1.15, view: 'section' },

  build(ctx) {
    const root = new THREE.Group();
    root.position.y = -0.45;
    // Static hardware on the casings is cut too, so nothing floats in the section view.
    const boltMat = cut(MAT.darkSteel());
    const brightBolt = cut(MAT.chrome());
    const hwSteel = cut(MAT.steel());

    // ---------- Crankcase (pump) ----------
    const caseMat = cut(MAT.aluminum());
    caseMat.color.set(0x9ba4ae);
    caseMat.roughness = 0.55;
    const crankcase = new THREE.Group();
    // Drum around the wheels, open at the top (cylinder) and at the rear (reed valve).
    for (const [a0, a1] of [[140, 161], [199, 400]]) {
      const g = extrude(arcShape(0.74, 0.82, a0 * DEG, a1 * DEG, 48), CASE_Z * 2);
      g.translate(0, 0, -CASE_Z);
      crankcase.add(new THREE.Mesh(g, caseMat));
    }
    // Side walls with the main-bearing housings.
    const sideShape = new THREE.Shape();
    sideShape.absarc(0, 0, 0.82, 0, Math.PI * 2, false);
    const shaftHole = new THREE.Path();
    shaftHole.absarc(0, 0, 0.1, 0, Math.PI * 2, true);
    sideShape.holes.push(shaftHole);
    const sideGeo = extrude(sideShape, 0.05, 48);
    for (const z of [-CASE_Z - 0.05, CASE_Z]) {
      const w = new THREE.Mesh(sideGeo, caseMat);
      w.position.z = z;
      crankcase.add(w);
    }
    // Shoulders and front/back plates joining the drum to the cylinder deck.
    for (const s of [-1, 1]) {
      const sh = new THREE.Shape();
      sh.moveTo(s * 0.56, DECK_Y);
      sh.lineTo(s * 0.72, DECK_Y);
      sh.lineTo(s * 0.72, 0.3);
      const aStart = Math.atan2(0.3, Math.sqrt(0.76 * 0.76 - 0.09));
      const aEnd = Math.atan2(Math.sqrt(0.76 * 0.76 - 0.56 * 0.56), 0.56);
      for (let i = 0; i <= 12; i++) {
        const a = lerp(aStart, aEnd, i / 12);
        sh.lineTo(s * 0.76 * Math.cos(a), 0.76 * Math.sin(a));
      }
      sh.lineTo(s * 0.56, DECK_Y);
      const g = extrude(sh, CASE_Z * 2 + 0.1);
      g.translate(0, 0, -CASE_Z - 0.05);
      crankcase.add(new THREE.Mesh(g, caseMat));
      const plate = new THREE.Mesh(new THREE.BoxGeometry(1.12, DECK_Y - 0.45, 0.05), caseMat);
      plate.position.set(0, (DECK_Y + 0.45) / 2, s * (CASE_Z + 0.025));
      crankcase.add(plate);
    }
    // Ball bearings in the housings.
    const bearingGeo = pipe(0.2, 0.1, 0.1, 32).rotateX(Math.PI / 2);
    for (const s of [-1, 1]) {
      const brg = new THREE.Mesh(bearingGeo, MAT.steel());
      brg.position.z = s * (CASE_Z - 0.03);
      crankcase.add(brg);
    }
    // Engine mounts with rubber bushes and through-bolts.
    const lugGeo = new RoundedBoxGeometry(0.22, 0.24, 0.5, 2, 0.04);
    const bushGeo = pipe(0.075, 0.035, 0.56, 24).rotateX(Math.PI / 2);
    const mountBoltGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.66, 12).rotateX(Math.PI / 2);
    for (const ang of [-55, 235]) {
      const a = ang * DEG;
      const lug = new THREE.Mesh(lugGeo, caseMat);
      lug.position.set(0.86 * Math.cos(a), 0.86 * Math.sin(a), 0);
      lug.rotation.z = a - Math.PI / 2;
      const bush = new THREE.Mesh(bushGeo, cut(MAT.rubber()));
      bush.position.set(0.96 * Math.cos(a), 0.96 * Math.sin(a), 0);
      const mb = new THREE.Mesh(mountBoltGeo, hwSteel);
      mb.position.copy(bush.position);
      crankcase.add(lug, bush, mb);
    }
    // Clutch cover (front) and magneto cover (rear), domed and bolted.
    const frontCover = new THREE.Mesh(
      solidLathe([[0, 0.88], [0.41, 0.87], [0.5, 0.8], [0.51, 0.57], [0.6, 0.57], [0.6, 0.6], [0.56, 0.62], [0.53, 0.82], [0.42, 0.9], [0, 0.92]], 56).rotateX(Math.PI / 2),
      caseMat
    );
    const rearCover = new THREE.Mesh(
      solidLathe([[0, 1.0], [0.46, 0.99], [0.52, 0.9], [0.53, 0.57], [0.62, 0.57], [0.62, 0.6], [0.58, 0.62], [0.56, 0.92], [0.48, 1.02], [0, 1.04]], 56).rotateX(-Math.PI / 2),
      caseMat
    );
    crankcase.add(frontCover, rearCover);
    const filler = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.05, 16).rotateX(Math.PI / 2), cut(MAT.paint(0x1f2328, 0.5)));
    filler.position.set(0.28, 0.25, 0.905);
    crankcase.add(filler);
    // Fasteners: drum side walls, covers, drain plug.
    const caseBolts = [];
    for (let i = 0; i < 14; i++) {
      const a = ((i + 0.5) / 14) * Math.PI * 2;
      if (Math.abs(Math.sin(a)) < 0.35 && Math.cos(a) < 0) continue; // skip the intake opening
      for (const s of [-1, 1]) caseBolts.push([[0.73 * Math.cos(a), 0.73 * Math.sin(a), s * (CASE_Z + 0.05)], [0, 0, s]]);
    }
    for (let i = 0; i < 10; i++) {
      const a = (i / 10) * Math.PI * 2;
      caseBolts.push([[0.56 * Math.cos(a), 0.56 * Math.sin(a), 0.6], [0, 0, 1]]);
      caseBolts.push([[0.58 * Math.cos(a), 0.58 * Math.sin(a), -0.6], [0, 0, -1]]);
    }
    crankcase.add(bolts(caseBolts, brightBolt, 0.026, 0.025));
    crankcase.add(bolts([[[0, -0.82, 0], [0, -1, 0]]], boltMat, 0.06, 0.04));
    root.add(
      ctx.part(crankcase, {
        id: 'crankcase',
        name: 'Carter pompa',
        base: 'La “pancia” del motore che racchiude l’albero. Nel 2 tempi è chiusa a tenuta e fa da pompa: il pistone, scendendo, ci schiaccia la miscela fresca e la spinge su verso il cilindro. Davanti c’è il coperchio della frizione, dietro quello del volano.',
        expert:
          'Due semicarter in lega di alluminio pressofusa con paraoli sui perni di banco per la tenuta. Il volume è ridotto al minimo per alzare il rapporto di precompressione (≈ 1,3–1,5). I cuscinetti di banco, a sfere o a rulli, sono lubrificati soltanto dall’olio contenuto nella miscela; il cambio, separato, ha un suo olio.',
        explode: [0, -0.5, -0.4],
      })
    );

    // ---------- Cylinder: barrel with ports and transfer channels, fins, exhaust boss ----------
    const barrelMat = cut(MAT.aluminum());
    barrelMat.color.set(0xb4bcc6);
    barrelMat.roughness = 0.5;
    const cylinder = new THREE.Group();
    const barrelGeos = [];
    const ductGeos = [];
    const wins = Object.values(WINDOWS);
    const breaks = [0, Math.PI * 2, ...wins.flatMap((w) => [w.a - w.w / 2, w.a + w.w / 2])].sort((p, q) => p - q);
    for (let i = 0; i < breaks.length - 1; i++) {
      const a = breaks[i];
      const b = breaks[i + 1];
      if (b - a < 1e-4) continue;
      const segs = Math.max(2, Math.ceil(((b - a) / (Math.PI * 2)) * 72));
      const win = wins.find((w) => Math.abs((a + b) / 2 - w.a) < w.w / 2);
      if (!win) barrelGeos.push(sectorSolid(BORE, R_OUT, B_BOT, HEAD_Y, a, b, segs));
      else if (win === WINDOWS.ex) {
        barrelGeos.push(sectorSolid(BORE, R_OUT, B_BOT, win.y0, a, b, segs), sectorSolid(BORE, R_OUT, win.y1, HEAD_Y, a, b, segs));
      } else {
        // Transfer: inner skin up to the window, open channel, outer skin, solid roof above.
        ductGeos.push(sectorSolid(BORE, R_SK, LINER_BOT, win.y0, a, b, segs), sectorSolid(R_CH, R_OUT, B_BOT, win.y1, a, b, segs));
        barrelGeos.push(sectorSolid(BORE, R_OUT, win.y1, HEAD_Y, a, b, segs));
      }
      if (!win || win === WINDOWS.ex) barrelGeos.push(sectorSolid(BORE, R_SK, LINER_BOT, B_BOT, a, b, segs)); // spigot
    }
    cylinder.add(new THREE.Mesh(mergeGeometries(barrelGeos), barrelMat));
    // Base flange: holes for the bore and the transfer channels.
    const flangeShape = new THREE.Shape();
    flangeShape.moveTo(-0.66, -0.55);
    flangeShape.lineTo(0.66, -0.55);
    flangeShape.lineTo(0.66, 0.55);
    flangeShape.lineTo(-0.66, 0.55);
    flangeShape.lineTo(-0.66, -0.55);
    const boreHole = new THREE.Path();
    boreHole.absarc(0, 0, R_SK, 0, Math.PI * 2, true);
    flangeShape.holes.push(boreHole);
    for (const w of [WINDOWS.trB, WINDOWS.boost, WINDOWS.trF]) flangeShape.holes.push(sectorPath(R_SK + 0.004, R_CH, w.a - w.w / 2, w.a + w.w / 2));
    cylinder.add(new THREE.Mesh(plateGeo(flangeShape, DECK_Y, B_BOT - DECK_Y), barrelMat));
    // Cooling fins, notched around the exhaust boss.
    const fullFin = ringShape(R_OUT - 0.005, 0.88);
    const notchFin = notchedFinShape(R_OUT - 0.005, 0.88, 0.37);
    for (let i = 0; i < 11; i++) {
      const y = 1.0 + i * 0.093;
      const notched = y > 1.06 && y < 1.66;
      cylinder.add(new THREE.Mesh(plateGeo(notched ? notchFin : fullFin, y), barrelMat));
    }
    // Exhaust boss with the round duct, and spring hooks.
    const boss = new THREE.Shape();
    boss.moveTo(-0.36, 1.1);
    boss.lineTo(0.36, 1.1);
    boss.lineTo(0.36, 1.64);
    boss.lineTo(-0.36, 1.64);
    boss.lineTo(-0.36, 1.1);
    const bossHole = new THREE.Path();
    bossHole.absarc(0, 1.37, 0.15, 0, Math.PI * 2, true);
    boss.holes.push(bossHole);
    const bossGeo = extrude(boss, 0.36, 40);
    bossGeo.rotateY(Math.PI / 2);
    bossGeo.translate(0.5, 0, 0);
    cylinder.add(new THREE.Mesh(bossGeo, barrelMat));
    const hookGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.08, 8).rotateZ(Math.PI / 2);
    for (const z of [-0.26, 0.26]) {
      const h = new THREE.Mesh(hookGeo, hwSteel);
      h.position.set(0.89, 1.37, z);
      cylinder.add(h);
    }
    // Base nuts and base gasket.
    cylinder.add(
      bolts([[-0.58, -0.47], [0.58, -0.47], [-0.58, 0.47], [0.58, 0.47]].map(([x, z]) => [[x, B_BOT, z], [0, 1, 0]]), brightBolt, 0.04, 0.035)
    );
    const gasketMat = cut(MAT.paint(0x2c2620, 0.8));
    const baseGasket = flangeShape.clone();
    cylinder.add(new THREE.Mesh(plateGeo(baseGasket, DECK_Y - 0.006, 0.012), gasketMat));
    root.add(
      ctx.part(cylinder, {
        id: 'cylinder',
        name: 'Cilindro alettato',
        base: 'Il tubo in cui scorre il pistone. Le alette all’esterno funzionano come un radiatore: l’aria che passa porta via il calore. Nella parete ci sono le finestre e i canali da cui passano i gas.',
        expert:
          'Cilindro in lega di alluminio con canna riportata al Nikasil (nichel + carburo di silicio). Raffreddamento ad aria nelle motoseghe e negli scooter; i 2T da cross moderni sono raffreddati a liquido. Le luci e i condotti sono ricavati di fusione e rifiniti: altezza e larghezza decidono fasatura e carattere del motore.',
        explode: [0, 0.85, 0],
      })
    );

    // Glowing outlines of the port windows (they brighten while uncovered).
    const ports = new THREE.Group();
    const exFrameMat = cut(MAT.glow(0xff7a3a, 0.6));
    const trFrameMat = cut(MAT.glow(0x5fd3ff, 0.6));
    const exFrame = windowFrame(BORE - 0.004, WINDOWS.ex, exFrameMat);
    const trFrames = [WINDOWS.trB, WINDOWS.boost, WINDOWS.trF].map((w) => windowFrame(BORE - 0.004, w, trFrameMat));
    ports.add(exFrame, ...trFrames);
    root.add(
      ctx.part(ports, {
        id: 'ports',
        name: 'Luci (finestre) nel cilindro',
        base: 'Le finestre nella parete del cilindro, evidenziate dai bordi luminosi. Non ci sono valvole: è il pistone, passandoci davanti, a scoprirle e coprirle. La più alta (arancione) è lo scarico, le più basse (azzurre) sono i travasi.',
        expert:
          'Fasatura simmetrica rispetto al PMI decisa solo dalla geometria: scarico aperto ≈ 97°–266° (≈ 170°, fino a 190–200° nei motori da gara), travasi ≈ 122°–238°. La luce di scarico può essere larga fino al 70% dell’alesaggio; le fasce elastiche hanno un perno anti-rotazione perché le loro estremità non si impiglino nelle luci.',
        explode: [0, 0.85, 0],
      })
    );

    // Transfer channel walls (cast into the barrel; a separate part so they can be highlighted).
    const ductMat = cut(MAT.aluminum());
    ductMat.color.set(0xa3adb9);
    ductMat.roughness = 0.5;
    const ducts = new THREE.Group();
    ducts.add(new THREE.Mesh(mergeGeometries(ductGeos), ductMat));
    root.add(
      ctx.part(ducts, {
        id: 'transferDucts',
        name: 'Condotti di travaso',
        base: 'Canali ricavati nello spessore del cilindro che collegano il carter alle finestre di travaso: come scivoli al contrario, guidano la miscela fresca dal basso verso l’alto. Nella sezione si vede il canale aperto in basso verso il carter.',
        expert:
          'Due travasi principali laterali più un travaso di boost di fronte allo scarico (nei motori moderni le luci di travaso sono 5). Gli angoli d’uscita, rivolti verso la parete opposta e verso l’alto, creano il vortice di lavaggio a corrente tangenziale (Schnürle) che limita il corto circuito verso lo scarico.',
        explode: [-0.5, 0.85, 0],
      })
    );

    // ---------- Head ----------
    const headMat = cut(MAT.aluminum());
    headMat.color.set(0xa7b0bb);
    headMat.roughness = 0.5;
    const head = new THREE.Group();
    head.add(
      new THREE.Mesh(
        solidLathe([[0.06, 2.125], [0.14, 2.11], [0.24, 2.065], [0.3, 2.025], [0.43, 2.0], [0.62, 2.0], [0.62, 2.38], [0.3, 2.39], [0.14, 2.42], [0.06, 2.42]], 64),
        headMat
      )
    );
    for (let i = 0; i < 4; i++) head.add(new THREE.Mesh(plateGeo(ringShape(0.6, 0.9 - i * 0.03), 2.05 + i * 0.085), headMat));
    head.add(new THREE.Mesh(pipe(0.62, 0.43, 0.012, 56).translate(0, HEAD_Y, 0), cut(MAT.copper())));
    head.add(
      bolts(
        [0, 1, 2, 3].map((k) => {
          const a = Math.PI / 4 + (k * Math.PI) / 2;
          return [[0.5 * Math.sin(a), 2.385, 0.5 * Math.cos(a)], [0, 1, 0]];
        }),
        brightBolt,
        0.045,
        0.05
      )
    );
    root.add(
      ctx.part(head, {
        id: 'head',
        name: 'Testata',
        base: 'Il coperchio del cilindro, anch’esso alettato e fissato con quattro dadi. Al centro c’è la piccola cupola dove la miscela viene schiacciata e brucia. Niente valvole, niente alberi a camme: è semplicissima.',
        expert:
          'Testata in alluminio con camera emisferica centrale e banda di squish periferica (pochi decimi di millimetro dal pistone) che spinge la carica verso la candela e accelera la combustione. Guarnizione in rame. Nei 2T da cross è raffreddata a liquido e spesso ha un inserto intercambiabile per variare il rapporto di compressione.',
        explode: [0, 1.5, 0],
      })
    );

    // ---------- Spark plug, cap, HT lead and ignition coil ----------
    const plug = new THREE.Group();
    const plugTip = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.03, 0.26, 16), MAT.steel());
    plugTip.position.y = 2.25;
    const plugHex = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.08, 6), MAT.chrome());
    plugHex.position.y = 2.46;
    const plugBody = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.05, 0.2, 16), MAT.ceramic());
    plugBody.position.y = 2.6;
    const plugCap = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.065, 0.18, 16), MAT.rubber());
    plugCap.position.y = 2.74;
    const lead = new THREE.Mesh(
      new THREE.TubeGeometry(curve([v3(0, 2.82), v3(0, 2.92, -0.2), v3(0.05, 2.7, -0.75), v3(0.08, 2.0, -1.0), v3(0.08, 1.1, -1.0), v3(0.05, 0.8, -0.9)]), 60, 0.02, 8),
      MAT.rubber()
    );
    const coil = new THREE.Mesh(new RoundedBoxGeometry(0.3, 0.16, 0.2, 2, 0.04), MAT.paint(0x1d2024, 0.6));
    coil.position.set(0.05, 0.72, -0.8);
    plug.add(plugTip, plugHex, plugBody, plugCap);
    const ignition = new THREE.Group();
    ignition.add(lead, coil);
    root.add(
      ctx.part(ignition, {
        id: 'ignitionCoil',
        name: 'Bobina e cavo candela',
        base: 'La bobina trasforma la corrente del volano magnete in un impulso ad altissima tensione, che il cavo porta alla candela.',
        expert: 'Bobina d’accensione comandata dalla centralina CDI: il condensatore si scarica nel primario e il secondario genera 20–30 kV. Cavo con cappuccio antidisturbo (resistenza ≈ 5 kΩ) per non disturbare l’elettronica.',
        explode: [0, 0.3, -1.1],
      })
    );
    root.add(
      ctx.part(plug, {
        id: 'sparkPlug',
        name: 'Candela',
        base: 'Fa la scintilla che accende la miscela, come l’accendino di un fornello. Qui lavora il doppio che in un 4 tempi: una scintilla a ogni giro.',
        expert:
          'Candela con grado termico scelto per lo scoppio a ogni giro e per non sporcarsi con l’olio della miscela. Accensione elettronica a scarica capacitiva (CDI) alimentata dal volano magnete, con anticipo variabile col regime (≈ 15–25° prima del PMS); la bobina eleva la tensione a 20–30 kV.',
        explode: [0, 2.1, 0],
      })
    );
    const spark = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 12), MAT.glow(0xbfe6ff, 6, 0.95));
    spark.position.set(0, 2.1, 0);
    spark.visible = false;
    root.add(spark);
    const flashLight = new THREE.PointLight(0xffa040, 0, 2.6, 2);
    flashLight.position.set(0, 1.85, 0.3);
    root.add(flashLight);

    // ---------- Crankshaft with full-circle wheels ----------
    const crank = new THREE.Group();
    const wheelMat = MAT.steel();
    const heavyMat = MAT.darkSteel();
    const wheelGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.12, 72).rotateX(Math.PI / 2);
    const plugGeo = new THREE.CylinderGeometry(0.075, 0.075, 0.125, 20).rotateX(Math.PI / 2);
    const rimGeo = new THREE.TorusGeometry(0.6, 0.012, 6, 72);
    for (const z of [-0.15, 0.15]) {
      const w = new THREE.Mesh(wheelGeo, wheelMat);
      w.position.z = z;
      crank.add(w);
      for (const s of [-1, 1]) {
        const r = new THREE.Mesh(rimGeo, MAT.chrome());
        r.position.z = z + s * 0.06;
        crank.add(r);
      }
      // Heavy-metal balance inserts opposite the crank pin.
      for (const s of [-1, 1]) {
        const p = new THREE.Mesh(plugGeo, heavyMat);
        p.position.set(s * 0.42 * Math.sin(0.5), -0.42 * Math.cos(0.5), z);
        crank.add(p);
      }
    }
    const crankPin = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.42, 24).rotateX(Math.PI / 2), MAT.chrome());
    crankPin.position.y = R;
    crank.add(crankPin);
    const mainGeo = new THREE.CylinderGeometry(0.095, 0.095, 1, 24).rotateX(Math.PI / 2);
    for (const [z0, z1] of [[0.21, 0.82], [-0.21, -0.93]]) {
      const m = new THREE.Mesh(mainGeo, wheelMat);
      m.scale.z = Math.abs(z1 - z0);
      m.position.z = (z0 + z1) / 2;
      crank.add(m);
    }
    // Primary drive pinion (to the clutch), inside the clutch cover.
    const gear = new THREE.Group();
    gear.add(new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.08, 40).rotateX(Math.PI / 2), MAT.darkSteel()));
    const toothGeo = new THREE.BoxGeometry(0.035, 0.05, 0.08);
    for (let i = 0; i < 24; i++) {
      const a = (i / 24) * Math.PI * 2;
      const t = new THREE.Mesh(toothGeo, MAT.darkSteel());
      t.position.set(Math.sin(a) * 0.235, Math.cos(a) * 0.235, 0);
      t.rotation.z = -a;
      gear.add(t);
    }
    gear.position.z = 0.72;
    crank.add(gear);
    root.add(
      ctx.part(crank, {
        id: 'crankshaft',
        name: 'Albero motore a volani pieni',
        base: 'Due grossi dischi pieni uniti da un perno: girando trasformano il su-e-giù del pistone in rotazione. Sono pieni anche per occupare spazio nel carter, così la “pompa” spinge meglio la miscela.',
        expert:
          'Albero composito: due volani pieni uniti dal perno di manovella piantato a pressa. L’equilibratura si ottiene con fori di alleggerimento o inserti di metallo pesante opposti al perno. I volani riducono il volume morto del carter (precompressione più alta); il pignone primario porta il moto alla frizione.',
        explode: [0, -0.4, 1.7],
      })
    );

    // Magneto flywheel inside the rear cover.
    const flywheel = new THREE.Group();
    const fwCup = new THREE.Mesh(lathe([[0.1, 0], [0.46, 0], [0.48, 0.02], [0.48, 0.16], [0.45, 0.16], [0.45, 0.025], [0.1, 0.025]], 48), MAT.darkSteel());
    fwCup.geometry.rotateX(Math.PI / 2);
    for (let i = 0; i < 6; i++) {
      const mag = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.035, 0.12), MAT.paint(0x8a2a2a));
      const a = (i / 6) * Math.PI * 2;
      mag.position.set(Math.sin(a) * 0.43, Math.cos(a) * 0.43, 0.09);
      mag.rotation.z = -a;
      fwCup.add(mag);
    }
    flywheel.add(fwCup);
    flywheel.position.z = -0.93;
    root.add(
      ctx.part(flywheel, {
        id: 'flywheel',
        name: 'Volano magnete',
        base: 'Un disco con magneti fissato all’albero: girando davanti alle bobine produce l’elettricità per la scintilla, come la dinamo di una bicicletta.',
        expert:
          'Rotore a magneti permanenti con statore di bobine: alimenta l’accensione CDI ed eventuali luci. Nelle moto da cross è leggero: poca inerzia significa motore pronto a salire di giri, al prezzo di un minimo meno regolare.',
        explode: [0, 0, -1.3],
      })
    );

    // ---------- Piston ----------
    const piston = new THREE.Group();
    const pistonMat = MAT.aluminum();
    piston.add(
      new THREE.Mesh(
        lathe(
          [[0, CROWN + 0.03], [0.2, CROWN + 0.018], [0.405, CROWN - 0.004], [0.415, CROWN - 0.02], [0.415, -SKIRT], [0.39, -SKIRT], [0.39, CROWN - 0.07], [0, CROWN - 0.07]],
          56
        ),
        pistonMat
      )
    );
    const ringGeo = new THREE.TorusGeometry(0.416, 0.007, 6, 56).rotateX(Math.PI / 2);
    for (const y of [CROWN - 0.045, CROWN - 0.09]) {
      const r = new THREE.Mesh(ringGeo, MAT.darkSteel());
      r.position.y = y;
      piston.add(r);
    }
    piston.add(new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.065, 0.76, 20).rotateX(Math.PI / 2), MAT.chrome()));
    root.add(
      ctx.part(piston, {
        id: 'piston',
        name: 'Pistone',
        base: 'Fa tre lavori insieme: riceve la spinta dello scoppio, fa da “porta” per le finestre e, con la sua faccia inferiore, pompa la miscela nel carter.',
        expert:
          'Pistone in lega Al‑Si, una o due fasce con spina di arresto. Il mantello deve coprire la luce di scarico al PMS, altrimenti carter e scarico comunicherebbero. Il cielo riceve uno scoppio a ogni giro: con miscela troppo magra o poco olio il rischio è il grippaggio.',
        explode: [0, 0.6, 1.2],
      })
    );

    // ---------- Conrod ----------
    const rod = new THREE.Group();
    const rodMat = MAT.steel();
    rod.add(
      new THREE.Mesh(new THREE.BoxGeometry(0.12, L - 0.3, 0.07).translate(0, L / 2, 0), rodMat),
      new THREE.Mesh(new THREE.TorusGeometry(0.13, 0.045, 10, 32), rodMat),
      new THREE.Mesh(new THREE.TorusGeometry(0.088, 0.03, 10, 28).translate(0, L, 0), rodMat),
      new THREE.Mesh(new THREE.TorusGeometry(0.095, 0.012, 6, 28), MAT.brass())
    );
    root.add(
      ctx.part(rod, {
        id: 'conrod',
        name: 'Biella',
        base: 'Il braccio che collega il pistone al perno dei volani: trasforma la spinta in rotazione.',
        expert:
          'Biella in acciaio forgiato in un solo pezzo, con gabbie a rullini alla testa e al piede: senza olio in pressione le bronzine non sopravvivrebbero. Non essendo divisa, si monta prima di piantare a pressa i volani sul perno.',
        explode: [0.9, 0.2, 1.4],
      })
    );

    // ---------- Reed valve in its housing ----------
    const reed = new THREE.Group();
    const cageMat = cut(MAT.rubber());
    const petalMat = MAT.paint(0x9e3b2a, 0.5);
    const faceLen = Math.hypot(0.22, 0.22);
    const petals = [];
    for (const s of [1, -1]) {
      const ang = -s * (Math.PI / 4);
      const face = new THREE.Mesh(new THREE.BoxGeometry(faceLen, 0.02, 0.34), cageMat);
      face.position.set(-0.89, s * 0.11, 0);
      face.rotation.z = ang;
      reed.add(face);
      const pivot = new THREE.Group();
      pivot.position.set(-1.0, s * 0.22, 0);
      pivot.rotation.z = ang;
      pivot.add(new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.012, 0.3).translate(0.15, s * 0.017, 0), petalMat));
      reed.add(pivot);
      petals.push({ pivot, ang, s });
    }
    const sideTri = new THREE.Shape();
    sideTri.moveTo(-1.02, 0.23);
    sideTri.lineTo(-0.77, 0);
    sideTri.lineTo(-1.02, -0.23);
    sideTri.lineTo(-1.02, 0.23);
    const sideTriGeo = extrude(sideTri, 0.02);
    for (const z of [-0.19, 0.17]) {
      const m = new THREE.Mesh(sideTriGeo, cageMat);
      m.position.z = z;
      reed.add(m);
    }
    // Housing bolted into the crankcase opening.
    const housing = new THREE.Shape();
    housing.moveTo(-0.26, -0.3);
    housing.lineTo(0.26, -0.3);
    housing.lineTo(0.26, 0.3);
    housing.lineTo(-0.26, 0.3);
    housing.lineTo(-0.26, -0.3);
    const hh = new THREE.Path();
    hh.moveTo(-0.195, -0.235);
    hh.lineTo(-0.195, 0.235);
    hh.lineTo(0.195, 0.235);
    hh.lineTo(0.195, -0.235);
    hh.lineTo(-0.195, -0.235);
    housing.holes.push(hh);
    const housingGeo = extrude(housing, 0.27);
    housingGeo.rotateY(Math.PI / 2);
    housingGeo.translate(-1.06, 0, 0);
    reed.add(new THREE.Mesh(housingGeo, caseMat));
    reed.add(bolts([[0.27, 0.22], [0.27, -0.22], [-0.27, 0.22], [-0.27, -0.22]].map(([y, z]) => [[-1.06, y, z], [-1, 0, 0]]), brightBolt, 0.026, 0.025));
    root.add(
      ctx.part(reed, {
        id: 'reedValve',
        name: 'Valvola lamellare',
        base: 'Una porticina a senso unico fatta di lamelle elastiche: si apre quando nel carter c’è risucchio e si richiude da sola quando la miscela viene spinta, così non può tornare indietro.',
        expert:
          'Pacco lamellare con lamelle in fibra di carbonio o vetroresina su gabbia rivestita in gomma. Ha sostituito l’aspirazione comandata dal mantello del pistone o dal disco rotante: la durata dell’aspirazione si adatta da sola al regime, a vantaggio di coppia e consumi.',
        explode: [-0.6, 0, 0.6],
      })
    );

    // ---------- Carburettor, intake boot and air filter ----------
    const carb = new THREE.Group();
    const carbMat = cut(MAT.aluminum());
    carbMat.color.set(0xc2c6ca);
    const bootMat = cut(MAT.rubber());
    const boot = new THREE.Mesh(solidLathe([[0.15, -0.11], [0.19, -0.11], [0.2, -0.05], [0.18, 0], [0.2, 0.05], [0.19, 0.11], [0.15, 0.11]], 40).rotateZ(Math.PI / 2), bootMat);
    boot.position.x = -1.17;
    const body = new THREE.Mesh(pipe(0.155, 0.13, 0.46, 40).rotateZ(Math.PI / 2), carbMat);
    body.position.x = -1.5;
    const tower = new THREE.Mesh(pipe(0.135, 0.118, 0.34, 32), carbMat);
    tower.position.set(-1.5, 0.29, 0);
    const towerCap = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.05, 32), carbMat);
    towerCap.position.set(-1.5, 0.48, 0);
    const adjuster = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.1, 10), brightBolt);
    adjuster.position.set(-1.5, 0.55, 0);
    const bowl = new THREE.Mesh(new RoundedBoxGeometry(0.28, 0.2, 0.28, 3, 0.06), carbMat);
    bowl.position.set(-1.5, -0.23, 0);
    const drain = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 6), cut(MAT.brass()));
    drain.position.set(-1.5, -0.35, 0);
    const idle = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.1, 12).rotateX(Math.PI / 2), cut(MAT.brass()));
    idle.position.set(-1.43, 0.02, 0.17);
    const bell = new THREE.Mesh(lathe([[0.13, 0], [0.135, 0.05], [0.16, 0.1], [0.2, 0.13]], 40).rotateZ(Math.PI / 2), carbMat);
    bell.position.x = -1.73;
    const filter = new THREE.Mesh(
      solidLathe([[0, 0.42], [0.18, 0.4], [0.22, 0.32], [0.24, 0.12], [0.2, 0.02], [0.17, 0.0], [0, 0.0]], 40).rotateZ(Math.PI / 2),
      cut(MAT.paint(0x8a2b2b, 0.95), { cap: false })
    );
    filter.position.x = -1.76;
    const clampRing = (r) => new THREE.TorusGeometry(r, 0.013, 6, 36).rotateY(Math.PI / 2);
    for (const [x, r] of [[-1.09, 0.195], [-1.25, 0.195], [-1.8, 0.205]]) {
      const c = new THREE.Mesh(clampRing(r), hwSteel);
      c.position.x = x;
      carb.add(c);
    }
    const slide = new THREE.Group();
    slide.add(new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.3, 0.2), MAT.chrome()));
    const needle = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.004, 0.22, 10), MAT.brass());
    needle.position.y = -0.26;
    slide.add(needle);
    slide.position.set(-1.5, 0.1, 0);
    carb.add(boot, body, tower, towerCap, adjuster, bowl, drain, idle, bell, filter, slide);
    root.add(
      ctx.part(carb, {
        id: 'carburetor',
        name: 'Carburatore e filtro',
        base: 'Dosa la benzina nell’aria che entra dal filtro. La saracinesca al centro (valvola a ghigliottina) è collegata alla manopola del gas: più si alza, più miscela passa. Qui si alza con l’acceleratore.',
        expert:
          'Carburatore a valvola a ghigliottina con spillo conico e getti di minimo e massimo, vaschetta a galleggiante sotto. La dosatura imprecisa e le perdite in corto circuito sono incompatibili con l’Euro 5: i 2T moderni usano l’iniezione nei travasi (KTM/Husqvarna TPI) o diretta in camera (es. Evinrude E‑TEC), con l’olio dosato da una pompa separata.',
        explode: [-1.2, 0.3, 0],
      })
    );

    // ---------- Exhaust: expansion chamber + silencer ----------
    const exhCurve = curve(EXH_PTS.map(([x, y]) => v3(x, y)));
    const exhMat = cut(MAT.paint(0x3a3e46, 0.32));
    exhMat.metalness = 0.85;
    const exhaust = new THREE.Group();
    exhaust.add(new THREE.Mesh(sweepSolid(exhCurve, chamberR, 0.014), exhMat));
    // Weld seams between the cones.
    const zAxis = v3(0, 0, 1);
    for (const u of [0.1, ...SEAMS]) {
      const seam = new THREE.Mesh(new THREE.TorusGeometry(chamberR(u) + 0.004, 0.01, 6, 40), boltMat);
      exhCurve.getPointAt(u, seam.position);
      seam.quaternion.setFromUnitVectors(zAxis, exhCurve.getTangentAt(u));
      exhaust.add(seam);
    }
    // Header springs to the hooks on the boss.
    const springMat = hwSteel;
    for (const z of [-0.26, 0.26]) exhaust.add(spring(v3(0.92, 1.37, z), v3(1.12, 1.27, z * 0.55), 7, 0.018, 0.006, springMat));
    // Silencer with clamps, and the support struts to the crankcase.
    const silencer = new THREE.Mesh(
      solidLathe([[0.058, -0.42], [0.07, -0.44], [0.17, -0.37], [0.17, 0.37], [0.06, 0.43], [0.048, 0.41], [0.158, 0.36], [0.158, -0.36]], 40).rotateZ(Math.PI / 2),
      exhMat
    );
    silencer.position.set(-0.9, -1.3, 0);
    exhaust.add(silencer);
    for (const x of [-0.62, -1.18]) {
      const c = new THREE.Mesh(clampRing(0.176), hwSteel);
      c.position.set(x, -1.3, 0);
      exhaust.add(c);
    }
    const tabMat = boltMat;
    const belly = exhCurve.getPointAt(0.49);
    exhaust.add(strut(belly.x - 0.35, belly.y, 0.8, belly.y - 0.1, 0.05, 0.08, tabMat));
    exhaust.add(strut(-0.88, -1.14, -0.58, -0.62, 0.05, 0.08, tabMat));
    root.add(
      ctx.part(exhaust, {
        id: 'exhaustPipe',
        name: 'Camera di espansione (marmitta a pancia)',
        base: 'La marmitta panciuta tipica dei 2 tempi. Non è solo un tubo: la sua forma fa rimbalzare le onde di pressione dei gas, che prima aiutano a svuotare il cilindro e poi rispingono dentro la miscela scappata.',
        expert:
          'Collettore, cono divergente, ventre, cono convergente e codino (stinger) che fa da strozzatura, poi il silenziatore; lamiera calandrata e saldata a settori, tenuta al cilindro da due molle. Il cono divergente rimanda un’onda di depressione che aspira i gas combusti, quello convergente un’onda di pressione che torna alla luce prima che si chiuda. In un 125 da cross la camera è lunga circa un metro.',
        explode: [1.1, 0, -0.5],
      })
    );

    // ---------- Gas volumes ----------
    const charge = new THREE.Mesh(new THREE.CylinderGeometry(0.412, 0.412, 1, 40), MAT.gas(0x5fd3ff, 0.2, false));
    charge.renderOrder = 2;
    root.add(charge);
    const caseGas = new THREE.Mesh(new THREE.CylinderGeometry(0.73, 0.73, 0.57, 40).rotateX(Math.PI / 2), MAT.gas(0x9ad8a0, 0.12, false));
    caseGas.position.z = -0.235;
    caseGas.renderOrder = 1;
    root.add(caseGas);

    // Pressure wave travelling in the expansion chamber.
    const wave = new THREE.Mesh(new THREE.TorusGeometry(1, 0.06, 8, 40), MAT.glow(0xffb07a, 1.6, 0.6));
    root.add(wave);

    // ---------- Flows ----------
    const mkFlow = (pts, opts) => {
      const c = curve(pts);
      const f = flow({ path: (t, out) => c.getPointAt(t, out), ...opts });
      root.add(f.object);
      return f;
    };
    const FRESH = [0x5fd3ff, 0x9ad8a0, 0xffc04d];
    const intakeFlows = [1, -1].map((s) =>
      mkFlow(
        [v3(-2.0, 0), v3(-1.5, 0), v3(-1.1, s * 0.02), v3(-0.93, s * 0.12), v3(-0.76, s * 0.25), v3(-0.52, s * 0.32), v3(-0.3, s * 0.22), v3(-0.12, s * -0.1)],
        { count: 90, size: 0.065, colors: FRESH, spread: (t) => 0.04 + t * 0.08, speed: 0.9 }
      )
    );
    const chMid = (R_SK + R_CH) / 2;
    const ductProfile = [[0.3, 0.45], [chMid, 0.62], [chMid, 0.95], [chMid, 1.2], [0.48, 1.29], [0.41, 1.31]];
    const dB = [Math.sin(WINDOWS.trB.a), Math.cos(WINDOWS.trB.a)];
    const boostPts = ductProfile.map(([rho, y]) => v3(-rho, y, 0));
    const backPts = ductProfile.map(([rho, y]) => v3(dB[0] * rho, y, dB[1] * rho));
    const transferFlows = [
      [...boostPts, v3(-0.25, 1.5), v3(-0.12, 1.8), v3(0.1, 1.9), v3(0.28, 1.7), v3(0.33, 1.45)],
      [...backPts, v3(dB[0] * 0.25, 1.5, dB[1] * 0.25), v3(-0.02, 1.8, -0.08), v3(0.15, 1.88, 0), v3(0.3, 1.65, 0), v3(0.34, 1.42, 0)],
    ].map((pts) => mkFlow(pts, { count: 120, size: 0.065, colors: [0x9ad8a0, 0xffc04d, 0xffc04d, 0xb0a890], spread: 0.035, speed: 0.9 }));
    // Exhaust: cylinder -> port -> chamber -> silencer.
    const exhFlowPts = [v3(0, 1.62), v3(0.25, 1.45), ...EXH_PTS.map(([x, y]) => v3(x, y)), v3(-0.9, -1.3), v3(-1.35, -1.3)];
    const exhFlowCurve = curve(exhFlowPts);
    const flowLen = exhFlowCurve.getLength();
    const chamberLen = exhCurve.getLength();
    const leadLen = exhFlowPts[0].distanceTo(exhFlowPts[1]) + exhFlowPts[1].distanceTo(exhFlowPts[2]);
    const exhaustFlow = flow({
      path: (t, out) => exhFlowCurve.getPointAt(t, out),
      count: 420,
      size: 0.08,
      colors: [0xff5a1f, 0xd07a5a, 0x8c86a8, 0x8c86a8],
      spread: (t) => {
        const u = (t * flowLen - leadLen) / chamberLen;
        return u < 0 ? 0.1 : u > 1 ? 0.1 : chamberR(u) * 0.6;
      },
      speed: 0.55,
    });
    root.add(exhaustFlow.object);
    // Fresh charge lost straight into the exhaust (short-circuit), and pushed back by the return wave.
    const shortFlow = mkFlow(
      [v3(dB[0] * 0.4, 1.31, dB[1] * 0.4), v3(0, 1.42, -0.2), v3(0.3, 1.38, -0.05), v3(0.45, 1.36), v3(0.75, 1.36), v3(1.05, 1.28), v3(1.3, 1.08), v3(1.48, 0.75)],
      { count: 90, size: 0.07, colors: [0x5fd3ff, 0xffc04d, 0xffc04d], spread: 0.05, speed: 0.8 }
    );
    const pushFlow = mkFlow([v3(1.3, 1.08), v3(1.05, 1.28), v3(0.75, 1.36), v3(0.45, 1.36), v3(0.2, 1.45), v3(0, 1.6)], {
      count: 60,
      size: 0.07,
      colors: [0xffc04d, 0x9ad8a0],
      spread: 0.06,
      speed: 1.4,
    });

    // ---------- Animation ----------
    const C = {
      mix: new THREE.Color(0x9ad8a0),
      fuel: new THREE.Color(0xffc04d),
      flash: new THREE.Color(0xfff2c4),
      fire: new THREE.Color(0xff5a1f),
      exh: new THREE.Color(0x8c86a8),
    };
    const cA = new THREE.Color();
    const tan = new THREE.Vector3();
    const rpmOf = (th) => 2000 + th * 9500;
    const animRate = (th) => 0.12 + th * 0.8;

    return {
      root,
      cycleRate: animRate,
      update(state) {
        const a = angleOf(state.cycle);
        const th = a * DEG;
        const xc = R * Math.sin(th);
        const yc = R * Math.cos(th);
        const yp = yc + Math.sqrt(L * L - xc * xc);
        crank.rotation.z = -th;
        fwCup.rotation.z = -th;
        piston.position.y = yp;
        rod.position.set(xc, yc, 0);
        rod.rotation.z = Math.atan2(xc, yp - yc);

        const { crown, ef, tf, reed: reedLift } = portState(a);
        const reedOpen = reedLift * (0.45 + 0.55 * state.throttle);
        for (const p of petals) p.pivot.rotation.z = p.ang + p.s * reedOpen * 0.32;
        slide.position.y = lerp(0.1, 0.27, state.throttle);

        exFrame.material.emissiveIntensity = 0.5 + ef * 3;
        for (const f of trFrames) f.material.emissiveIntensity = 0.5 + tf * 3;

        const assembled = (state.explode || 0) < 0.05;
        const rate = animRate(state.throttle);

        // Cylinder charge between crown and head.
        const top = HEAD_Y + 0.06;
        const bottom = crown + 0.01;
        charge.position.y = (top + bottom) / 2;
        charge.scale.y = Math.max(0.001, top - bottom);
        let op;
        if (a < 15) {
          cA.copy(C.flash).lerp(C.fire, a / 15);
          op = 0.95;
        } else if (a < EVO) {
          cA.copy(C.fire).lerp(C.exh, (a - 15) / (EVO - 15));
          op = lerp(0.9, 0.4, (a - 15) / (EVO - 15));
        } else if (a < TPO) {
          cA.copy(C.exh);
          op = lerp(0.4, 0.25, (a - EVO) / (TPO - EVO));
        } else if (a < 360 - TPO) {
          cA.copy(C.exh).lerp(C.mix, (a - TPO) / (360 - 2 * TPO));
          op = 0.25 + 0.05 * ((a - TPO) / (360 - 2 * TPO));
        } else if (a < A0) {
          cA.copy(C.mix);
          op = 0.3;
        } else {
          const f = (a - A0) / (360 - A0);
          cA.copy(C.mix).lerp(C.fuel, f);
          op = 0.3 + 0.3 * f;
        }
        charge.material.color.copy(cA);
        charge.material.opacity = op;
        charge.visible = assembled;

        // Crankcase pressure: rises while the piston descends, released by the transfers.
        let press = 0;
        if (a > 50 && a <= TPO) press = (a - 50) / (TPO - 50);
        else if (a > TPO && a < 200) press = 1 - (a - TPO) / (200 - TPO);
        caseGas.material.opacity = 0.08 + 0.22 * press;
        caseGas.material.color.copy(C.mix).lerp(C.fuel, press * 0.6);
        caseGas.visible = assembled;

        // Spark and combustion flash.
        const ai = (a - IGN + 360) % 360;
        const sparkOn = ai < 9;
        spark.visible = sparkOn && assembled;
        spark.scale.setScalar(sparkOn ? 0.6 + Math.random() * 0.6 : 1);
        const flash = ai > 4 && ai < 50 ? 1 - (ai - 4) / 46 : 0;
        flashLight.intensity = assembled ? flash * (2 + state.throttle * 4) : 0;

        // Pressure wave: out to the baffle cone, reflected back to the port.
        let wu = -1;
        if (a >= EVO && a < 190) wu = lerp(0.03, 0.74, (a - EVO) / (190 - EVO));
        else if (a >= 190 && a < 255) wu = lerp(0.74, 0.05, (a - 190) / 65);
        wave.visible = assembled && wu >= 0;
        if (wu >= 0) {
          exhCurve.getPointAt(wu, wave.position);
          exhCurve.getTangentAt(wu, tan);
          wave.quaternion.setFromUnitVectors(zAxis, tan);
          wave.scale.setScalar(chamberR(wu) * 0.9);
          wave.material.opacity = (0.25 + 0.45 * state.throttle) * Math.min(1, wu * 12);
        }

        // Flows.
        const blow = a > EVO && a < 145 ? 1.8 : 1;
        const setFlow = (f, k, speedK = 1) => {
          f.object.visible = assembled;
          f.material.opacity = clamp(k * 1.5, 0, 0.9);
          f.update(state.dt, k * rate * speedK);
        };
        intakeFlows.forEach((f) => setFlow(f, reedOpen, 1.5));
        transferFlows.forEach((f) => setFlow(f, tf, 1.6));
        setFlow(exhaustFlow, ef, 1.4 * blow);
        setFlow(shortFlow, tf * 0.6, 1.6);
        setFlow(pushFlow, Math.min(1, windowFn(a, 226, 268, 10) * ef * 4) * (0.4 + 0.6 * state.throttle), 1.5);
      },
      readout(state) {
        const rpm = rpmOf(state.throttle);
        const torque = 13 + 12 * state.throttle * state.throttle * (3 - 2 * state.throttle);
        const kw = (torque * rpm * 2 * Math.PI) / 60 / 1000;
        const { ef } = portState(angleOf(state.cycle));
        return [
          { label: 'Giri reali', value: Math.round(rpm / 10) * 10, unit: 'giri/min' },
          { label: 'Potenza', value: Math.round(kw), unit: 'kW' },
          { label: 'Scoppi al secondo', value: Math.round(rpm / 60), unit: '/s' },
          { label: 'Luce di scarico aperta', value: Math.round(ef * 100), unit: '%' },
        ];
      },
    };
  },
};
