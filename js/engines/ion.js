// Gridded electrostatic ion thruster (NEXT-C / T6-like) on a spacecraft bus with xenon tank, PPU and
// solar array. Continuous engine: the slider is electrical power; state.cycle only drives a faint flicker.
// The thruster is built along local +Y (lathe-friendly) and turned so the beam points to world +X.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, flow, clamp, lerp } from '../core/helpers.js';

// ---------- Thruster stations (local frame: axis +Y, beam towards +Y) ----------
const R_CH = 0.95; // discharge chamber radius
const Y_BACK = -0.72; // back plate
const R_BACK = 0.3; // back plate radius (cone start)
const Y_CONE = -0.38; // cone -> cylinder
const Y_FRONT = 0.3; // chamber front (grid mount)
const RS = 3.0; // grid dish sphere radius
const A_GRID = 0.92; // grid dish radius
const SAG = RS - Math.sqrt(RS * RS - A_GRID * A_GRID);
const Y_SCREEN = Y_FRONT + SAG; // screen grid apex
const Y_ACCEL = Y_FRONT + 0.04 + SAG; // accel grid apex
const BEAM_L = 3.3;
const BEAM_DIV = Math.tan((12 * Math.PI) / 180);
const NEUT = [-1.2, 0.42]; // neutralizer base (local x, y)
const NEUT_DIR = new THREE.Vector2(0.5, 0.86).normalize();

// ---------- Operating envelope (NEXT-C-like) ----------
const P_MIN = 0.6;
const P_MAX = 6.9;
const op = (th) => {
  const t = clamp(th, 0, 1);
  const power = lerp(P_MIN, P_MAX, t); // kW
  const vBeam = lerp(275, 1800, t); // V
  const thrust = lerp(25, 236, t); // mN
  const isp = 4190 * Math.sqrt(vBeam / 1800); // s
  const mdot = (thrust / 1000 / (isp * 9.81)) * 1e6; // mg/s
  return { t, power, vBeam, thrust, isp, mdot };
};

// ---------- Small builders ----------
const V = (x, y, z = 0) => new THREE.Vector3(x, y, z);
const L2W = (x, y, z = 0) => V(y, -x, z); // thruster local -> world (rotation z = -90°)
const crv = (pts) => new THREE.CatmullRomCurve3(pts.map((p) => (p.isVector3 ? p : V(...p))), false, 'centripetal');
const tube = (curve, r, mat, seg = 48, rad = 8) => new THREE.Mesh(new THREE.TubeGeometry(curve, seg, r, rad, false), mat);

function ring(R, r, y, mat, segs = 64) {
  const m = new THREE.Mesh(new THREE.TorusGeometry(R, r, 10, segs), mat);
  m.rotation.x = Math.PI / 2;
  m.position.y = y;
  return m;
}

function strut(a, b, r, mat) {
  const d = new THREE.Vector3().subVectors(b, a);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, d.length(), 10), mat);
  m.position.copy(a).add(b).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(V(0, 1, 0), d.normalize());
  return m;
}

// Closed wall from a profile [[r, y], ...] with thickness inward.
function shell(pts, t, segs = 72) {
  const inner = pts.map(([r, y]) => [Math.max(0.001, r - t), y]).reverse();
  return lathe([...pts, ...inner, pts[0]], segs);
}

// Chamber wall radius at height y (inner surface).
function wallR(y) {
  if (y >= Y_CONE) return R_CH;
  return lerp(R_BACK, R_CH, clamp((y - Y_BACK) / (Y_CONE - Y_BACK), 0, 1));
}

// Dished grid: spherical cap, apex at yApex, convex towards the beam.
function gridCap(yApex) {
  const g = new THREE.SphereGeometry(RS, 96, 6, 0, Math.PI * 2, 0, Math.asin(A_GRID / RS));
  g.translate(0, yApex - RS, 0);
  return g;
}

// Hexagonal pattern of apertures on a grid cap (instanced dark discs facing out).
function apertures(yApex, rHole, pitch, mat) {
  const pts = [];
  const rMax = A_GRID - 0.08;
  for (let j = -20; j <= 20; j++) {
    for (let i = -20; i <= 20; i++) {
      const u = (i + (j % 2 ? 0.5 : 0)) * pitch;
      const v = j * pitch * 0.866;
      if (u * u + v * v < rMax * rMax) pts.push([u, v]);
    }
  }
  const geo = new THREE.CircleGeometry(rHole, 10);
  const mesh = new THREE.InstancedMesh(geo, mat, pts.length);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const n = new THREE.Vector3();
  const s = new THREE.Vector3(1, 1, 1);
  const cy = yApex - RS;
  pts.forEach(([u, v], k) => {
    const y = cy + Math.sqrt(RS * RS - u * u - v * v);
    n.set(u, y - cy, v).normalize();
    q.setFromUnitVectors(V(0, 0, 1), n);
    m.compose(V(u, y, v).addScaledVector(n, 0.004), q, s);
    mesh.setMatrixAt(k, m);
  });
  mesh.instanceMatrix.needsUpdate = true;
  mesh.computeBoundingSphere();
  return mesh;
}

// Crinkled gold MLI blanket.
function mliTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const g = c.getContext('2d');
  g.fillStyle = '#c99a38';
  g.fillRect(0, 0, 256, 256);
  for (let i = 0; i < 260; i++) {
    const x = Math.random() * 256;
    const y = Math.random() * 256;
    const a = Math.random() * Math.PI;
    const l = 10 + Math.random() * 40;
    g.strokeStyle = Math.random() < 0.5 ? 'rgba(255,236,170,0.35)' : 'rgba(90,60,10,0.3)';
    g.lineWidth = 1 + Math.random() * 3;
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(x + Math.cos(a) * l, y + Math.sin(a) * l);
    g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(2, 2);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// Perforated plasma-screen sheet: grey with a fine hole pattern.
function perfTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#80868e';
  g.fillRect(0, 0, 64, 64);
  g.fillStyle = '#2a2e34';
  for (const [x, y] of [[16, 16], [48, 16], [0, 48], [32, 48], [64, 48]]) {
    g.beginPath();
    g.arc(x, y, 7, 0, Math.PI * 2);
    g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(60, 32);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

// Solar cell texture (drawn once).
function cellTexture() {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 296;
  const g = c.getContext('2d');
  g.fillStyle = '#c9ccd2';
  g.fillRect(0, 0, 512, 296);
  const nx = 16;
  const ny = 9;
  const w = 512 / nx;
  const h = 296 / ny;
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const grd = g.createLinearGradient(i * w, j * h, (i + 1) * w, (j + 1) * h);
      grd.addColorStop(0, '#1d2d6b');
      grd.addColorStop(1, '#13204f');
      g.fillStyle = grd;
      g.fillRect(i * w + 2, j * h + 2, w - 4, h - 4);
      g.strokeStyle = 'rgba(190,200,230,0.25)';
      g.lineWidth = 1;
      for (let k = 1; k < 4; k++) {
        g.beginPath();
        g.moveTo(i * w + 2, j * h + (k * h) / 4);
        g.lineTo((i + 1) * w - 2, j * h + (k * h) / 4);
        g.stroke();
      }
    }
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.anisotropy = 4;
  return tex;
}

// ---------- World-frame plumbing (shared by meshes and flows) ----------
const TANK_C = V(-2.2, 0.3, -0.25);
const TANK_R = 0.55;
const XFC = V(-1.33, 0.62, -0.45);
const PPU_C = V(-1.62, -0.8, -0.2);
const BUS = { x0: -3.0, x1: -1.15, y: 1.0, z: 1.0, t: 0.04 }; // hollow bus box (panel thickness t)
const NEUT_BASE = L2W(NEUT[0], NEUT[1]);
const FEED_MAIN = [
  V(-2.2, 0.88, -0.25),
  V(-2.15, 1.0, -0.3),
  V(-1.85, 0.95, -0.42),
  V(-1.6, 0.72, -0.45),
  XFC,
  V(-1.1, 0.45, -0.5),
  V(-0.85, 0.22, -0.5),
  L2W(0, -0.6, -0.5),
];
const FEED_CATH = [XFC, V(-1.12, 0.4, -0.3), V(-1.07, 0.15, -0.1), V(-1.04, 0.02, -0.01), L2W(0, -1.0, 0)];
const FEED_NEUT = [XFC, V(-1.1, 0.85, -0.45), V(-0.75, 1.17, -0.35), V(-0.1, 1.3, -0.2), V(0.25, 1.24, -0.06), NEUT_BASE];
const POWER = [
  V(-2.0, 1.62, -0.4),
  V(-2.0, 1.1, -0.4),
  V(-2.0, 0.9, -0.88),
  V(-1.9, -0.2, -0.88),
  V(-1.68, -0.68, -0.62),
  V(-1.62, -0.78, -0.52),
  PPU_C,
  V(-1.3, -0.78, -0.2),
  V(-1.14, -0.6, -0.22),
  V(-1.0, -0.35, -0.21),
  V(-0.935, -0.22, -0.2),
];

export default {
  id: 'ion',
  name: 'Propulsore ionico a griglie',
  short: 'Propulsore ionico',
  family: 'spazio',
  vehicle: 'Sonde e satelliti',
  examples: 'NASA NSTAR (Deep Space 1, Dawn), NEXT-C (DART), QinetiQ T6 (BepiColombo)',
  tagline: 'Spinge quanto il peso di qualche foglio di carta, ma non si ferma per anni: e alla fine va più veloce di qualsiasi razzo chimico.',
  throttleLabel: 'Potenza elettrica',
  intro: {
    base:
      'Invece di bruciare qualcosa, questo motore usa l’<b>elettricità</b> dei pannelli solari. Prende un gas pesante, lo <b>xeno</b>, gli strappa un elettrone trasformandolo in ioni (atomi con carica elettrica) e li spara fuori a circa 40 km/s con due griglie piene di buchi, a cui è applicata una tensione di oltre mille volt. ' +
      'La spinta è piccolissima, come il peso di qualche foglio di carta, ma dura mesi o anni: goccia dopo goccia la sonda raggiunge velocità impossibili per un razzo chimico con la stessa quantità di propellente.',
    expert:
      'Propulsore elettrostatico a griglie con camera di scarica a <b>ring-cusp</b>: un catodo cavo emette elettroni che ionizzano lo xeno per urto (energia di prima ionizzazione 12,1 eV), il plasma è confinato da anelli di magneti permanenti e gli ioni vengono estratti e accelerati da un sistema ottico a due griglie (screen ~+1800 V, accel ~−200 V). ' +
      'Un catodo neutralizzatore emette altrettanti elettroni nel fascio per mantenere neutra la sonda. ' +
      'NEXT-C: 0,6–6,9 kW, 25–236 mN, Isp fino a ~4200 s, efficienza totale ~70%. Il limite non è l’energia del propellente ma la potenza elettrica disponibile.',
  },
  specs: [
    { label: 'Propellente', value: 'Xeno (gas nobile)' },
    { label: 'Potenza elettrica', value: '0,6–6,9 kW' },
    { label: 'Spinta', value: '25–236 mN' },
    { label: 'Impulso specifico', value: '≈ 1400–4200 s' },
    { label: 'Velocità degli ioni', value: '≈ 30–40 km/s' },
    { label: 'Tensione di accelerazione', value: '≈ 300–1800 V' },
    { label: 'Diametro del fascio', value: '≈ 36 cm (NEXT) · 22 cm (T6)' },
  ],
  variants: {
    columns: ['Propulsore', 'Tipo', 'Missioni', 'Potenza', 'Spinta', 'Isp'],
    rows: [
      ['NASA NSTAR', 'Ionico a griglie, xeno', 'Deep Space 1, Dawn', '0,5–2,3 kW', '19–92 mN', '1900–3100 s'],
      ['NASA NEXT-C', 'Ionico a griglie, xeno', 'DART', 'fino a ≈ 6,9 kW', 'fino a ≈ 236 mN', 'fino a ≈ 4100 s'],
      ['QinetiQ T6', 'Ionico a griglie (Kaufman), xeno', 'BepiColombo (ESA/JAXA)', '≈ 4,5 kW', 'fino a ≈ 145 mN', '≈ 4000 s'],
      ['SPT-100', 'Effetto Hall, xeno', 'Satelliti geostazionari', '≈ 1,35 kW', '≈ 83 mN', '≈ 1600 s'],
      ['Starlink V2 mini', 'Effetto Hall, argon', 'Costellazione Starlink', '≈ 4,2 kW', '≈ 170 mN', '≈ 2500 s'],
    ],
    note: 'I propulsori a effetto Hall non hanno griglie: danno più spinta per kW ma un impulso specifico più basso.',
  },
  thumb: { orbit: [38, 20], zoom: 1.25, view: 'section' },
  metrics: {
    efficiency: 65, // electrical power -> jet kinetic power
    power: 5, // kW, typical operating point
    powerDensity: 0.4, // kW/kg, thruster alone (~13 kg)
  },
  phases: [
    {
      id: 'feed',
      name: 'Alimentazione xeno',
      parts: ['xenonTank', 'feedSystem'],
      base:
        'Lo xeno è conservato compresso in un serbatoio sferico. Un sistema di valvole e regolatori ne fa uscire pochissimo, pochi milligrammi al secondo, e lo divide fra la camera, il catodo e il neutralizzatore.',
      expert:
        'Xeno stoccato in condizioni supercritiche (~100–150 bar) in un serbatoio COPV con liner in titanio e sovravvolgimento in fibra di carbonio: densità ~1,2–1,6 kg/L. ' +
        'Il controllore di flusso (XFC) riduce la pressione con regolatori e fissa le portate con restrizioni calibrate: qualche mg/s nella camera e frazioni di mg/s ai due catodi. Lo xeno è scelto perché pesante (131 u), inerte e facile da ionizzare.',
    },
    {
      id: 'ionization',
      name: 'Ionizzazione',
      parts: ['cathode', 'chamber', 'magnets'],
      base:
        'Dal catodo cavo, sul fondo della camera, escono elettroni velocissimi. Urtando gli atomi di xeno ne staccano un elettrone e li trasformano in ioni positivi: la camera si riempie di un gas luminoso e carico, il plasma. ' +
        'Gli anelli di magneti fanno girare gli elettroni a spirale, così prima di essere persi urtano molti più atomi.',
      expert:
        'Scarica in corrente continua a ~25 V tra catodo cavo e parete (anodo). Il catodo cavo ha un inserto emissivo (tungsteno impregnato di ossido di bario o LaB₆) riscaldato e un elettrodo “keeper”. ' +
        'Le cuspidi del campo magnetico (anelli a polarità alternata) confinano gli elettroni primari aumentandone il cammino e quindi la probabilità di ionizzazione: costo tipico ~150–200 W per ampere di fascio.',
    },
    {
      id: 'acceleration',
      name: 'Accelerazione tra le griglie',
      parts: ['grids', 'ppu'],
      base:
        'Davanti alla camera ci sono due griglie parallele piene di migliaia di forellini. Fra le due c’è una tensione di oltre mille volt: gli ioni che arrivano a un buco vengono “risucchiati” e sparati fuori a circa 40 km/s, più di 140.000 km/h. ' +
        'L’unità di potenza (PPU) trasforma l’elettricità dei pannelli nelle tensioni che servono.',
      expert:
        'Ottica ionica a due griglie in molibdeno, bombate per resistere alle dilatazioni: screen grid a ~+1800 V (fissa il potenziale del plasma), accel grid a ~−200 V per impedire agli elettroni del fascio di risalire (backstreaming). ' +
        'Ogni foro forma un “beamlet”; la corrente estraibile è limitata dalla carica spaziale (legge di Child–Langmuir, j ∝ V^1,5/d²). ' +
        'La PPU converte il bus a 100 V circa in alimentazioni per fascio, accel, scarica, keeper e riscaldatori, con rendimento ~95%.',
    },
    {
      id: 'neutralization',
      name: 'Neutralizzazione',
      parts: ['neutralizer'],
      base:
        'Se la sonda sparasse fuori solo cariche positive diventerebbe sempre più negativa e finirebbe per richiamare indietro gli ioni. ' +
        'Per questo un secondo piccolo catodo, fuori dal motore, spruzza nel fascio tanti elettroni quanti sono gli ioni: il getto resta neutro.',
      expert:
        'Il neutralizzatore è un catodo cavo a bassa portata che emette una corrente di elettroni pari alla corrente di fascio (qualche ampere). ' +
        'Il potenziale della sonda rispetto al plasma del fascio si assesta a qualche decina di volt (coupling voltage). Gli elettroni non si ricombinano davvero con gli ioni: viaggiano con loro formando un plasma quasi neutro.',
    },
    {
      id: 'thrust',
      name: 'Spinta piccolissima ma continua',
      parts: ['grids', 'solarArray'],
      base:
        'La spinta è minuscola: al massimo circa il peso di un mandarino, al minimo meno di quello di un foglio di carta. Sulla Terra non solleverebbe nemmeno sé stesso! ' +
        'Ma nello spazio non c’è attrito: accesa per anni, accumula velocità. La sonda Dawn ha guadagnato così circa 11 km/s, più di qualsiasi altra sonda con il proprio motore.',
      expert:
        'F = ṁ·vₑ con ṁ di pochi mg/s e vₑ ~ 30–40 km/s: 20–250 mN. Isp 3000–4000 s, circa 10 volte quello di un razzo chimico, quindi per l’equazione di Tsiolkovsky serve molta meno massa di propellente a parità di Δv. ' +
        'Dawn (3 NSTAR) ha accumulato ~11,5 km/s di Δv in quasi 6 anni di accensioni con ~425 kg di xeno. ' +
        'Il cugino più diffuso è il propulsore a effetto Hall (senza griglie, campo magnetico radiale): spinta per kW più alta, Isp 1500–2500 s; i satelliti Starlink usano Hall a kripton e argon.',
    },
  ],
  quiz: [
    {
      q: 'Che cosa accelera gli ioni di xeno fuori dal propulsore?',
      options: [
        'Una combustione ad alta temperatura',
        'Un’elica elettrica',
        'La differenza di tensione elettrica tra due griglie forate',
        'La pressione del serbatoio',
      ],
      correct: 2,
      explain:
        'Gli ioni sono carichi positivamente: tra la griglia screen (più di +1000 V) e la griglia accel (negativa) un forte campo elettrico li spinge fuori a 30–40 km/s. Non c’è nessuna combustione.',
    },
    {
      q: 'A cosa serve il neutralizzatore all’esterno del motore?',
      options: [
        'A emettere elettroni nel fascio, così la sonda non si carica elettricamente',
        'A raffreddare le griglie',
        'A misurare la velocità della sonda',
        'A riaccendere il motore se si spegne',
      ],
      correct: 0,
      explain:
        'Il motore espelle cariche positive: senza neutralizzatore la sonda accumulerebbe carica negativa e richiamerebbe indietro gli ioni. Gli elettroni emessi rendono il getto neutro.',
    },
    {
      q: 'Rispetto a un motore chimico, un propulsore ionico ha…',
      options: [
        'Più spinta e meno efficienza',
        'Spinta piccolissima ma impulso specifico circa 10 volte più alto',
        'La stessa spinta ma consuma più propellente',
        'Bisogno di ossigeno per funzionare',
      ],
      correct: 1,
      explain:
        'Spinge con decine o centinaia di millinewton, ma lancia ogni grammo di xeno a 30–40 km/s: con pochi kg di propellente ottiene Δv enormi, purché resti acceso a lungo.',
    },
    {
      q: 'Perché nella camera di scarica ci sono anelli di magneti?',
      options: [
        'Per attirare il serbatoio di xeno',
        'Per orientare la sonda verso il Sole',
        'Per generare l’elettricità del motore',
        'Per intrappolare gli elettroni più a lungo, così ionizzano più atomi di xeno',
      ],
      correct: 3,
      explain:
        'Il campo a cuspidi fa percorrere agli elettroni lunghe traiettorie a spirale prima di raggiungere la parete: più urti con gli atomi, più ioni con la stessa potenza.',
    },
  ],
  camera: { position: [4.9, 2.5, 8.0], target: [0.35, 0.35, 0] },

  build(ctx) {
    const root = new THREE.Group();
    // Thruster frame: local +Y -> world +X.
    const thr = new THREE.Group();
    thr.rotation.z = -Math.PI / 2;
    root.add(thr);

    const addPart = (parent, obj, info) => {
      const w = ctx.part(obj, info);
      parent.add(w);
      return w;
    };

    // ---------- Materials ----------
    const wallMat = cut(new THREE.MeshStandardMaterial({ color: 0x8a9097, metalness: 0.6, roughness: 0.62 })); // roughened stainless anode
    const housingMat = cut(new THREE.MeshStandardMaterial({ map: perfTexture(), metalness: 0.6, roughness: 0.5 }));
    const ringMat = cut(MAT.paint(0x6b717a, 0.5));
    const magN = cut(MAT.paint(0xc0392b, 0.45));
    const magS = cut(MAT.paint(0x2e5bd8, 0.45));
    const moly = cut(new THREE.MeshStandardMaterial({ color: 0xaaa59b, metalness: 0.6, roughness: 0.45 }));
    const molyDark = cut(new THREE.MeshStandardMaterial({ color: 0x86827b, metalness: 0.6, roughness: 0.5 }));
    const steel = MAT.steel();
    const dark = MAT.darkSteel();
    const copper = MAT.copper();
    const ceramic = MAT.ceramic();
    const ceramicCut = cut(MAT.ceramic());
    const goldMLI = cut(new THREE.MeshStandardMaterial({ map: mliTexture(), metalness: 0.7, roughness: 0.4 }));
    const tankMat = cut(new THREE.MeshStandardMaterial({ color: 0x2a2d33, metalness: 0.4, roughness: 0.5 }));
    const lineMat = cut(MAT.steel());
    const cableMat = MAT.paint(0x2b2f36, 0.7);
    const cableRed = MAT.paint(0xa8322a, 0.6);
    const ppuMat = MAT.paint(0x4b5563, 0.5);
    const finMat = MAT.aluminum();

    // ---------- Spacecraft bus (context, not clickable) ----------
    // Hollow box of MLI-covered panels; the +X side is the aluminium thruster mounting panel.
    const { x0, x1, y: BY, z: BZ, t: BT } = BUS;
    const bw = x1 - x0;
    const bcx = (x0 + x1) / 2;
    const addBox = (geo, mat, x, y, z) => {
      const m = new THREE.Mesh(geo, mat);
      m.position.set(x, y, z);
      root.add(m);
      return m;
    };
    const B = (w, h, d) => new THREE.BoxGeometry(w, h, d);
    addBox(B(bw, 2 * BY, BT), goldMLI, bcx, 0, -BZ + BT / 2);
    addBox(B(bw, 2 * BY, BT), goldMLI, bcx, 0, BZ - BT / 2);
    addBox(B(bw, BT, 2 * BZ - 2 * BT), goldMLI, bcx, BY - BT / 2, 0);
    addBox(B(bw, BT, 2 * BZ - 2 * BT), goldMLI, bcx, -BY + BT / 2, 0);
    addBox(B(BT, 2 * BY - 2 * BT, 2 * BZ - 2 * BT), goldMLI, x0 + BT / 2, 0, 0);
    addBox(B(BT, 2 * BY - 2 * BT, 2 * BZ - 2 * BT), cut(MAT.aluminum()), x1 - BT / 2, 0, 0);
    // edge rails
    const railMat = cut(MAT.aluminum());
    const railX = new RoundedBoxGeometry(bw + 0.05, 0.05, 0.05, 1, 0.012);
    const railY = new RoundedBoxGeometry(0.05, 2 * BY + 0.05, 0.05, 1, 0.012);
    const railZ = new RoundedBoxGeometry(0.05, 0.05, 2 * BZ + 0.05, 1, 0.012);
    for (const a of [-1, 1])
      for (const b of [-1, 1]) {
        addBox(railX, railMat, bcx, a * BY, b * BZ);
        addBox(railY, railMat, a < 0 ? x0 : x1, 0, b * BZ);
        addBox(railZ, railMat, a < 0 ? x0 : x1, b * BY, 0);
      }
    // thruster mount: three struts from the panel to the mounting ring on the housing
    for (const [a, b] of [
      [V(x1 + 0.01, 0.62, -0.42), L2W(-0.57, -0.6, -0.56)],
      [V(x1 + 0.01, -0.62, -0.42), L2W(0.57, -0.6, -0.56)],
      [V(x1 + 0.01, 0, -0.8), L2W(0, -0.6, -0.8)],
    ])
      root.add(strut(a, b, 0.025, dark));

    // ---------- Xenon tank ----------
    const tank = new THREE.Group();
    const half = (r) => Array.from({ length: 25 }, (_, i) => [r * Math.sin((i / 24) * Math.PI), -r * Math.cos((i / 24) * Math.PI)]);
    const tankShell = new THREE.Mesh(lathe([...half(TANK_R), ...half(TANK_R - 0.025).reverse(), [0, -TANK_R]], 48), tankMat);
    // support skirt down to the bottom panel
    const skirtY0 = -BUS.y + BUS.t - TANK_C.y;
    const skirtY1 = -Math.sqrt(TANK_R * TANK_R - 0.22 * 0.22) + 0.01;
    tank.add(new THREE.Mesh(lathe([[0.2, skirtY0], [0.22, skirtY0], [0.22, skirtY1], [0.2, skirtY1], [0.2, skirtY0]], 40), cut(MAT.aluminum())));
    const xeMat = cut(new THREE.MeshStandardMaterial({ color: 0xb9b4ff, emissive: 0x3a2f80, emissiveIntensity: 0.4, transparent: true, opacity: 0.35, depthWrite: false, roughness: 0.2 }));
    const xe = new THREE.Mesh(new THREE.SphereGeometry(TANK_R - 0.03, 40, 24), xeMat);
    tank.add(tankShell, xe);
    for (const s of [1, -1]) {
      const boss = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.08, 20), MAT.titanium());
      boss.position.y = s * (TANK_R + 0.02);
      tank.add(boss);
    }
    tank.add(ring(TANK_R + 0.005, 0.012, 0, cut(MAT.steel())));
    tank.position.copy(TANK_C);
    addPart(root, tank, {
      id: 'xenonTank',
      name: 'Serbatoio di xeno',
      base:
        'Una sfera robusta che contiene lo xeno, un gas raro e pesante, compresso così tanto da essere denso quasi come un liquido. Su una sonda come Dawn ce n’erano circa 425 kg, che sono bastati per quasi dieci anni di missione.',
      expert:
        'Serbatoio COPV: liner in titanio sovravvolto in fibra di carbonio, xeno a ~100–150 bar in stato supercritico (sopra 16,6 °C e 58 bar), densità ~1,2–1,6 kg/L. ' +
        'Lo xeno costa caro (è estratto dall’aria in quantità minime): per questo i sistemi più recenti, soprattutto Hall, passano a kripton o argon.',
      explode: [0, 0, 0],
    });

    // ---------- Feed system (XFC + lines) ----------
    const feed = new THREE.Group();
    const xfc = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.2, 0.24), MAT.paint(0x8a929c, 0.45));
    xfc.position.copy(XFC);
    feed.add(xfc);
    for (const [dx, dz] of [[-0.07, 0.13], [0.04, 0.13], [0.15, 0.13]]) {
      const v = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.05, 12), MAT.brass());
      v.rotation.x = Math.PI / 2;
      v.position.set(XFC.x + dx - 0.04, XFC.y + 0.03, XFC.z + dz);
      feed.add(v);
    }
    const cFeed = crv(FEED_MAIN);
    const cCath = crv(FEED_CATH);
    const cNeut = crv(FEED_NEUT);
    feed.add(tube(cFeed, 0.018, lineMat, 72), tube(cCath, 0.012, lineMat, 32), tube(cNeut, 0.012, lineMat, 64));
    // propellant isolator (ceramic break) on the main line near the thruster
    const iso = new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.09, 16), ceramic);
    iso.position.copy(cFeed.getPointAt(0.88));
    iso.quaternion.setFromUnitVectors(V(0, 1, 0), cFeed.getTangentAt(0.88));
    feed.add(iso);
    addPart(root, feed, {
      id: 'feedSystem',
      name: 'Sistema di alimentazione dello xeno',
      base:
        'Valvole, regolatori e tubicini sottili che dosano lo xeno con precisione estrema: pochi milligrammi al secondo, come una goccia d’acqua ogni dieci secondi. Un tubo va alla camera, uno al catodo e uno al neutralizzatore.',
      expert:
        'Lo XFC (Xenon Flow Controller) abbassa la pressione da ~100 bar a pochi bar con regolatori meccanici o elettronici e fissa le portate con restrizioni a flusso termico controllato. ' +
        'Prima del propulsore un isolatore ceramico interrompe elettricamente la linea: la camera si trova a +1800 V rispetto alla sonda.',
      explode: [0, 0.35, -0.25],
    });

    // ---------- PPU + cables ----------
    const ppu = new THREE.Group();
    const box = new THREE.Mesh(new RoundedBoxGeometry(0.55, 0.32, 0.6, 2, 0.025), ppuMat);
    box.position.copy(PPU_C);
    ppu.add(box);
    for (const [dy, dz] of [[0, 0], [0.04, 0.03], [-0.04, 0.03]]) {
      const conn = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.05, 14), MAT.aluminum());
      conn.rotation.z = Math.PI / 2;
      conn.position.set(PPU_C.x + 0.29, PPU_C.y + dy, PPU_C.z + dz);
      ppu.add(conn);
    }
    const finBase = new THREE.Mesh(new THREE.BoxGeometry(0.52, 0.02, 0.56), finMat);
    finBase.position.set(PPU_C.x, PPU_C.y + 0.165, PPU_C.z);
    ppu.add(finBase);
    for (let i = 0; i < 7; i++) {
      const fin = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.08, 0.56), finMat);
      fin.position.set(PPU_C.x - 0.24 + i * 0.08, PPU_C.y + 0.2, PPU_C.z);
      ppu.add(fin);
    }
    const cPower = crv(POWER);
    ppu.add(tube(crv(POWER.slice(0, 6)), 0.02, cableRed, 48, 6));
    for (const [dy, dz] of [[0, 0], [0.04, 0.03], [-0.04, 0.03]]) {
      const pts = POWER.slice(6).map((p, i, a) => p.clone().add(V(0, dy * (1 - i / (a.length - 1)) + dy * 0.3, dz)));
      ppu.add(tube(crv(pts), 0.016, cableMat, 40, 6));
    }
    addPart(root, ppu, {
      id: 'ppu',
      name: 'Unità di potenza (PPU)',
      base:
        'Il “trasformatore” del motore: prende l’elettricità dei pannelli solari e la trasforma nelle tensioni giuste per ogni parte, da poche decine fino a quasi duemila volt. Le alette in alto smaltiscono il calore.',
      expert:
        'Power Processing Unit: convertitori DC/DC ad alta tensione per fascio (fino a ~1800 V, alcuni A), accel (~−200 V), scarica (~25 V, fino a ~25 A), keeper e riscaldatori dei catodi, più la logica di controllo e di gestione dei corti tra le griglie (recycle). ' +
        'Rendimento ~95%: alcune centinaia di watt diventano calore da irraggiare nello spazio. Pesa spesso più del propulsore.',
      explode: [0, -0.8, 0],
    });

    // ---------- Solar array stub ----------
    const solar = new THREE.Group();
    const boom = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.7, 12), steel);
    boom.position.set(-2.0, 1.33, -0.4);
    const yoke = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.05, 0.05), steel);
    yoke.position.set(-2.0, 1.66, -0.4);
    const drive = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.1, 20), dark);
    drive.position.set(-2.0, 1.03, -0.4);
    const edge = MAT.paint(0x9aa0a8, 0.5);
    const cells = new THREE.MeshStandardMaterial({ map: cellTexture(), metalness: 0.3, roughness: 0.35 });
    const panel = new THREE.Mesh(new THREE.BoxGeometry(2.2, 1.25, 0.035), [edge, edge, edge, edge, cells, edge]);
    panel.position.set(-2.0, 2.31, -0.4);
    solar.add(boom, yoke, drive, panel);
    addPart(root, solar, {
      id: 'solarArray',
      name: 'Pannello solare (tratto)',
      base:
        'L’energia del motore viene dal Sole. Qui ne vedi solo un pezzo: le ali vere sono molto più grandi. Più il Sole è lontano, meno energia arriva e più il motore deve “abbassare il gas”.',
      expert:
        'Pannelli a celle a tripla giunzione (GaInP/GaAs/Ge, efficienza ~30%): a 1 UA producono ~300–350 W/m². DART aveva i ROSA arrotolabili (~3,5 kW a 1 UA per ala), Dawn ~10 kW a 1 UA che scendevano a ~1,3 kW vicino a Cerere. ' +
        'Nella propulsione solare-elettrica la potenza disponibile, che cala con il quadrato della distanza dal Sole, fissa il punto di funzionamento del motore.',
      explode: [0, 0.8, 0],
    });

    // ---------- Discharge chamber + plasma screen housing ----------
    const chamber = new THREE.Group();
    const wallPts = [[0.095, Y_BACK], [R_BACK, Y_BACK], [R_CH, Y_CONE], [R_CH, Y_FRONT]];
    chamber.add(new THREE.Mesh(shell(wallPts, -0.015), wallMat));
    const housePts = [[0.1, Y_BACK - 0.1], [0.38, Y_BACK - 0.1], [R_CH + 0.11, Y_CONE - 0.06], [R_CH + 0.11, Y_FRONT + 0.04]];
    chamber.add(new THREE.Mesh(shell(housePts, -0.012), housingMat));
    // propellant distributor ring at the back
    chamber.add(ring(0.42, 0.02, -0.66, cut(MAT.steel())));
    // mounting ring on the housing (struts land here) and HV junction box
    chamber.add(ring(0.8, 0.03, -0.6, ringMat));
    const jbox = new THREE.Mesh(new RoundedBoxGeometry(0.22, 0.1, 0.26, 2, 0.02), cut(MAT.paint(0x4b5563, 0.5)));
    jbox.position.set(0.22, Y_BACK - 0.16, -0.2);
    chamber.add(jbox);
    addPart(thr, chamber, {
      id: 'chamber',
      name: 'Camera di scarica',
      base:
        'È la “pentola” dove nasce il plasma: lo xeno entra dal fondo, gli elettroni del catodo lo colpiscono e la camera si riempie di un gas carico e luminoso. La parete metallica fa da elettrodo positivo e raccoglie gli elettroni.',
      expert:
        'Camera cilindro-conica che funge da anodo della scarica (~25 V); l’alimentazione principale entra da un anello distributore sul fondo. ' +
        'Il rendimento di utilizzo del propellente (frazione di xeno che esce ionizzata) supera il 90%. Attorno, uno schermo forato (plasma screen) a potenziale della sonda evita scariche verso l’esterno.',
      explode: [0, -0.15, -1.2],
    });

    // ---------- Ring-cusp magnets ----------
    const magnets = new THREE.Group();
    const cusps = [
      [R_CH + 0.035, 0.16, magN],
      [R_CH + 0.035, -0.2, magS],
      [wallR(-0.55) + 0.05, -0.55, magN],
      [R_BACK + 0.04, Y_BACK - 0.035, magS],
    ];
    for (const [R, y, m] of cusps) magnets.add(ring(R, 0.032, y, m, 72));
    addPart(thr, magnets, {
      id: 'magnets',
      name: 'Magneti ad anello (ring-cusp)',
      base:
        'Anelli di magneti permanenti, alternati nord e sud, montati attorno alla camera. Il loro campo fa girare gli elettroni a spirale vicino alle pareti: restano intrappolati più a lungo e colpiscono molti più atomi di xeno.',
      expert:
        'Magneti in samario-cobalto (stabili ad alta temperatura) a polarità alternata: il campo forma cuspidi sulle pareti e lascia il centro quasi privo di campo, dove il plasma è uniforme. ' +
        'Gli elettroni primari, che seguono le linee di campo, vengono riflessi dalle cuspidi (specchio magnetico) e raggiungono l’anodo solo in zone strette: perdite ridotte e meno W per ione prodotto.',
      explode: [0, 0, 0],
    });
    // Cusp field lines (magenta), between adjacent rings on the inner wall.
    const fieldMat = cut(MAT.glow(0xff4ad8, 1.2, 0.55));
    const wallPt = [
      [R_CH, 0.16],
      [R_CH, -0.2],
      [wallR(-0.55), -0.55],
      [R_BACK, Y_BACK],
    ];
    for (let k = 0; k < wallPt.length - 1; k++) {
      const [r1, y1] = wallPt[k];
      const [r2, y2] = wallPt[k + 1];
      const dx = r2 - r1;
      const dy = y2 - y1;
      const len = Math.hypot(dx, dy);
      // inward normal (towards the axis)
      let nx = -dy / len;
      let ny = dx / len;
      if (nx > 0) [nx, ny] = [-nx, -ny];
      for (const bulge of [0.09, 0.16]) {
        const prof = [];
        for (let i = 0; i <= 10; i++) {
          const u = i / 10;
          const s = Math.sin(u * Math.PI) * bulge * (len / 0.36);
          prof.push([lerp(r1, r2, u) + nx * s, lerp(y1, y2, u) + ny * s]);
        }
        for (let a = 0; a < 12; a++) {
          const phi = (a / 12) * Math.PI * 2 + 0.13;
          const c = crv(prof.map(([r, y]) => [r * Math.cos(phi), y, r * Math.sin(phi)]));
          magnets.add(tube(c, 0.005, fieldMat, 16, 4));
        }
      }
    }

    // ---------- Hollow cathode ----------
    const cath = new THREE.Group();
    cath.add(new THREE.Mesh(lathe([[0, -1.0], [0.045, -1.0], [0.045, -0.6], [0.03, -0.575], [0, -0.575]], 24), MAT.titanium()));
    cath.add(
      new THREE.Mesh(
        lathe([[0.062, -0.82], [0.085, -0.82], [0.085, -0.53], [0.022, -0.505], [0.022, -0.52], [0.062, -0.54], [0.062, -0.82]], 32),
        MAT.paint(0x8f9399, 0.4)
      )
    );
    for (let i = 0; i < 5; i++) {
      const c = ring(0.052, 0.008, -0.97 + i * 0.03, copper, 20);
      cath.add(c);
    }
    const cathFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.11, 0.025, 24), steel);
    cathFlange.position.y = Y_BACK - 0.02;
    cath.add(cathFlange);
    addPart(thr, cath, {
      id: 'cathode',
      name: 'Catodo cavo',
      base:
        'Un piccolo tubo che, scaldato, “sputa” elettroni. Dentro c’è un materiale speciale che a circa mille gradi libera elettroni con facilità; un po’ di xeno che lo attraversa li aiuta a uscire. Sono loro che accendono il plasma.',
      expert:
        'Catodo cavo con inserto in tungsteno poroso impregnato di ossidi di bario (o LaB₆) a ~1000–1100 °C: emissione termoionica favorita dalla bassa funzione di lavoro, con un plasma interno denso che neutralizza la carica spaziale. ' +
        'Il riscaldatore serve solo all’avvio; poi il catodo si autosostiene. Il keeper protegge l’orifizio dall’erosione degli ioni e mantiene la scarica. Durata qualificata di decine di migliaia di ore.',
      explode: [0, 0.2, 1.3],
    });

    // ---------- Grids (screen + accel) ----------
    const grids = new THREE.Group();
    grids.add(new THREE.Mesh(gridCap(Y_SCREEN), moly), new THREE.Mesh(gridCap(Y_ACCEL), molyDark));
    grids.add(new THREE.Mesh(shell([[A_GRID + 0.13, Y_FRONT - 0.01], [A_GRID + 0.13, Y_FRONT + 0.08]], 0.12), housingMat));
    const gb = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.014, 0.014, 0.012, 6), cut(MAT.steel()), 36);
    const gm = new THREE.Matrix4();
    for (let i = 0; i < 36; i++) {
      const a = ((i + 0.5) / 36) * Math.PI * 2;
      gm.makeTranslation(Math.cos(a) * (A_GRID + 0.1), Y_FRONT + 0.086, Math.sin(a) * (A_GRID + 0.1));
      gb.setMatrixAt(i, gm);
    }
    gb.computeBoundingSphere();
    grids.add(gb);
    for (let i = 0; i < 12; i++) {
      const a = (i / 12) * Math.PI * 2;
      const b = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.1, 8), ceramicCut);
      b.position.set(Math.cos(a) * (A_GRID + 0.07), Y_FRONT + 0.06, Math.sin(a) * (A_GRID + 0.07));
      grids.add(b);
    }
    addPart(thr, grids, {
      id: 'grids',
      name: 'Griglie di accelerazione',
      base:
        'Due dischi sottili, uno davanti all’altro a meno di un millimetro, bucherellati con migliaia di forellini allineati. Tra di loro c’è una tensione di più di mille volt: gli ioni che arrivano a un buco vengono sparati fuori come proiettili. ' +
        'Ogni forellino produce un sottilissimo getto; tutti insieme formano il fascio blu-violetto.',
      expert:
        'Sistema ottico a due griglie in molibdeno (alcune versioni in carbonio-carbonio o grafite pirolitica), bombate e con migliaia di aperture allineate (screen più grandi e sottili, accel più piccole e spesse). ' +
        'Distanza tra le griglie ~0,5–1 mm; tensione totale ~1,1–2 kV. L’usura per erosione da scambio di carica (ioni lenti che colpiscono la griglia accel) è il principale limite di vita: NEXT ha superato 50.000 ore di prova a terra.',
      explode: [0, 0.9, 0],
    });
    // Aperture discs: their own (non-highlighted) material so they can glow with the beamlets.
    const holeMat = cut(new THREE.MeshStandardMaterial({ color: 0x07070b, emissive: 0x8a7cff, emissiveIntensity: 0.5, roughness: 0.6 }));
    grids.add(apertures(Y_ACCEL, 0.019, 0.07, holeMat), apertures(Y_SCREEN, 0.026, 0.07, holeMat));

    // ---------- Neutralizer ----------
    const neutPart = new THREE.Group();
    const neut = new THREE.Group();
    neut.position.set(NEUT[0], NEUT[1], 0);
    neut.quaternion.setFromUnitVectors(V(0, 1, 0), V(NEUT_DIR.x, NEUT_DIR.y, 0));
    neut.add(new THREE.Mesh(lathe([[0, 0], [0.04, 0], [0.04, 0.24], [0.058, 0.24], [0.058, 0.3], [0.014, 0.315], [0, 0.315]], 24), MAT.paint(0x8f9399, 0.4)));
    for (let i = 0; i < 3; i++) neut.add(ring(0.045, 0.007, 0.05 + i * 0.03, copper, 16));
    neutPart.add(neut);
    neutPart.add(strut(V(-(A_GRID + 0.13), Y_FRONT + 0.03, 0), V(NEUT[0] + 0.02, NEUT[1] + 0.05, 0), 0.018, dark));
    addPart(thr, neutPart, {
      id: 'neutralizer',
      name: 'Neutralizzatore',
      base:
        'Un piccolo catodo fuori dal motore, puntato verso il fascio. Emette elettroni (i puntini giallo-verdi) che si mescolano agli ioni: così il getto è elettricamente neutro e la sonda non si carica.',
      expert:
        'Catodo cavo a bassa portata (decimi di mg/s di xeno) che emette una corrente elettronica pari alla corrente di fascio. ' +
        'Senza neutralizzazione la sonda si porterebbe a un potenziale fortemente negativo fino a richiamare il fascio e annullare la spinta. Il suo funzionamento in “plume mode” o “spot mode” influenza erosione e rumore del plasma.',
      explode: [-0.7, 0.5, 0],
    });

    // ---------- Plasma, glows and beam (not clickable) ----------
    const plasmaMat = cut(MAT.gas(0x8f74ff, 0.12));
    const plasma = new THREE.Mesh(
      lathe([[0, Y_BACK + 0.02], [R_BACK - 0.03, Y_BACK + 0.02], [R_CH - 0.03, Y_CONE], [R_CH - 0.03, Y_FRONT + SAG * 0.8], [0, Y_SCREEN - 0.01]], 48),
      plasmaMat
    );
    thr.add(plasma);
    const cathGlowMat = MAT.glow(0xcfe0ff, 3, 0.9);
    const cathGlow = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 12), cathGlowMat);
    cathGlow.position.y = -0.49;
    const neutGlowMat = MAT.glow(0xe8ff9a, 3, 0.9);
    const neutGlow = new THREE.Mesh(new THREE.SphereGeometry(0.03, 12, 10), neutGlowMat);
    neutGlow.position.set(NEUT[0] + NEUT_DIR.x * 0.33, NEUT[1] + NEUT_DIR.y * 0.33, 0);
    thr.add(cathGlow, neutGlow);

    const beam = new THREE.Group();
    beam.position.y = Y_ACCEL;
    thr.add(beam);
    const fadeLathe = (pts, fade) => {
      const g = lathe(pts, 48);
      const pos = g.attributes.position;
      const col = new Float32Array(pos.count * 3);
      for (let i = 0; i < pos.count; i++) col.fill(clamp(fade(pos.getY(i) / BEAM_L), 0, 1), i * 3, i * 3 + 3);
      g.setAttribute('color', new THREE.BufferAttribute(col, 3));
      return g;
    };
    const beamMat = (color, opacity) => {
      const m = MAT.gas(color, opacity);
      m.vertexColors = true;
      return m;
    };
    const R0 = A_GRID - 0.08;
    const beamPts = [];
    for (let i = 0; i <= 10; i++) {
      const y = (i / 10) * BEAM_L;
      beamPts.push([R0 + y * BEAM_DIV, y]);
    }
    const outerBeamMat = beamMat(0x7f9bff, 0.22);
    const coreBeamMat = beamMat(0xb084ff, 0.3);
    beam.add(
      new THREE.Mesh(fadeLathe(beamPts, (d) => clamp(d / 0.1, 0, 1) * Math.pow(1 - d, 1.4)), outerBeamMat),
      new THREE.Mesh(fadeLathe(beamPts.map(([r, y]) => [r * 0.62, y]), (d) => clamp(d / 0.1, 0, 1) * Math.pow(1 - d, 1.1)), coreBeamMat)
    );
    const beamLight = new THREE.PointLight(0x8f8cff, 1.5, 5, 2);
    beamLight.position.set(0, 1.8, 0.6);
    beam.add(beamLight);

    // ---------- Flows ----------
    // Xenon feed: tank -> XFC -> chamber (world frame).
    const feedFlow = flow({
      path: (t, out) => cFeed.getPointAt(t, out),
      count: 130,
      size: 0.045,
      colors: [0xc2c6ef, 0xa9b3ef],
      spread: (t) => (t < 0.05 ? 0.12 : 0.014),
      speed: 0.12,
    });
    root.add(feedFlow.object);
    // Electric power: panel -> PPU -> thruster.
    const powerFlow = flow({
      path: (t, out) => cPower.getPointAt(t, out),
      count: 160,
      size: 0.045,
      colors: [0xd8ff4a],
      spread: 0.012,
      speed: 0.18,
    });
    root.add(powerFlow.object);
    // Chamber: neutral atoms (grey-blue) drift to the grids and become ions (violet).
    const chamberFlow = flow({
      path: (t, out) => out.set(0, lerp(-0.62, Y_SCREEN - 0.06, t), 0),
      count: 280,
      size: 0.05,
      colors: [0xaebbd0, 0xaebbd0, 0xb084ff, 0x9f8bff],
      spread: (t) => wallR(lerp(-0.62, Y_SCREEN - 0.06, t)) * 0.82,
      speed: 0.12,
    });
    thr.add(chamberFlow.object);
    // Primary electrons from the hollow cathode, spiralling outwards.
    const elecFlow = flow({
      path: (t, out) => {
        const r = 0.03 + 0.72 * Math.pow(t, 0.8);
        const a = t * Math.PI * 5;
        out.set(r * Math.cos(a), lerp(-0.5, 0.15, t), r * Math.sin(a));
      },
      count: 220,
      size: 0.035,
      colors: [0xf2ffb0, 0xd8ff4a],
      spread: (t) => 0.03 + 0.18 * t,
      speed: 0.35,
    });
    thr.add(elecFlow.object);
    // Ion beam: accelerated between the grids, then straight out with a slight divergence.
    const beamFlow = flow({
      path: (t, out) => out.set(0, Y_SCREEN - 0.02 + Math.pow(t, 0.9) * (BEAM_L + 0.05), 0),
      count: 900,
      size: 0.05,
      colors: [0xe6dcff, 0xb084ff, 0x7f9bff, 0x4a5cc8],
      spread: (t) => R0 * 0.9 + t * BEAM_L * BEAM_DIV,
      speed: 0.9,
      opacity: 0.85,
    });
    thr.add(beamFlow.object);
    // Neutralizer electrons join the beam.
    const nTip = V(NEUT[0] + NEUT_DIR.x * 0.33, NEUT[1] + NEUT_DIR.y * 0.33, 0);
    const cNeutE = crv([nTip, V(-0.8, 0.95, 0), V(-0.4, 1.4, 0), V(-0.15, 2.2, 0), V(-0.05, Y_ACCEL + BEAM_L, 0)]);
    const neutFlow = flow({
      path: (t, out) => cNeutE.getPointAt(t, out),
      count: 220,
      size: 0.035,
      colors: [0xf2ffb0, 0xd8ff4a, 0xb8e040],
      spread: (t) => 0.02 + 0.75 * Math.pow(t, 0.7),
      speed: 0.55,
    });
    thr.add(neutFlow.object);
    const allFlows = [feedFlow, powerFlow, chamberFlow, elecFlow, beamFlow, neutFlow];
    allFlows.forEach((f) => ctx.addFlow(f));

    return {
      root,
      cycleRate(throttle) {
        return 0.4 + 0.3 * throttle;
      },
      update(state) {
        const o = op(state.throttle);
        const k = o.t; // 0..1 power level
        const { dt } = state;
        const c = state.cycle * Math.PI * 2;
        const fl = 1 + 0.04 * Math.sin(c * 5) + 0.025 * Math.sin(c * 11 + 0.7);
        const lvl = 0.25 + 0.75 * k;

        plasmaMat.opacity = (0.04 + 0.08 * k) * fl;
        cathGlowMat.emissiveIntensity = (1.2 + 1.2 * k) * fl;
        neutGlowMat.emissiveIntensity = (1.6 + 1.6 * k) * fl;
        holeMat.emissiveIntensity = (0.25 + 0.9 * k) * fl;
        outerBeamMat.opacity = (0.06 + 0.16 * k) * fl;
        coreBeamMat.opacity = (0.07 + 0.2 * k) * fl;
        beamLight.intensity = (0.4 + 1.4 * k) * fl;
        beamFlow.material.opacity = 0.45 + 0.45 * k;
        chamberFlow.material.opacity = 0.6 + 0.3 * k;
        fieldMat.opacity = 0.4 + 0.1 * fl;

        feedFlow.update(dt, lvl);
        powerFlow.update(dt, lvl);
        chamberFlow.update(dt, lvl);
        elecFlow.update(dt, lvl);
        beamFlow.update(dt, lvl);
        neutFlow.update(dt, lvl);

        const assembled = (state.explode || 0) < 0.05;
        for (const f of allFlows) f.object.visible = assembled;
        beam.visible = plasma.visible = cathGlow.visible = neutGlow.visible = assembled;
      },
      readout(state) {
        const o = op(state.throttle);
        return [
          { label: 'Potenza', value: o.power.toFixed(1), unit: 'kW' },
          { label: 'Spinta', value: Math.round(o.thrust), unit: 'mN' },
          { label: 'Impulso specifico', value: Math.round(o.isp / 10) * 10, unit: 's' },
          { label: 'Consumo xeno', value: o.mdot.toFixed(1), unit: 'mg/s' },
        ];
      },
    };
  },
};
