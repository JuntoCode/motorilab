// High-bypass two-spool turbofan (CFM LEAP / GE9X class), technical cutaway.
// Axis along X: air enters at -X (fan), exhaust leaves at +X.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { MAT, cut, lathe, blade, ringX, flow, clamp, lerp } from '../core/helpers.js';

const TAU = Math.PI * 2;
const OX = -0.12; // model offset along X (keeps the engine centred)
const DEG = Math.PI / 180;

// ---------- Geometry helpers (local) ----------
// Lathe around X from [[radius, x], ...].
function latheX(pts, seg = 72) {
  const g = lathe(pts, seg);
  g.rotateZ(-Math.PI / 2); // lathe Y axis -> X axis
  return g;
}

// Thin closed shell from a [radius, x] polyline, thickness t (negative = inward).
function wall(pts, t = 0.012, seg = 72) {
  const back = pts.map(([r, x]) => [r + t, x]).reverse();
  return latheX([...pts, ...back, pts[0]], seg);
}

// Closed skin: polyline [r, x] offset by t along its right-hand normal (in x-r plane).
function skin(pts, t, seg = 72) {
  const n = pts.length;
  const off = pts.map(([r, x], i) => {
    const [r0, x0] = pts[Math.max(i - 1, 0)];
    const [r1, x1] = pts[Math.min(i + 1, n - 1)];
    const dx = x1 - x0;
    const dr = r1 - r0;
    const L = Math.hypot(dx, dr) || 1;
    return [Math.max(0, r - (dx / L) * t), x + (dr / L) * t];
  });
  return latheX([...pts, ...off.reverse(), pts[0]], seg);
}

// Closed rectangular ring (flange, seal, bulkhead) at x.
function ring(x, rIn, rOut, w, seg = 72) {
  const h = w / 2;
  return latheX([[rIn, x - h], [rOut, x - h], [rOut, x + h], [rIn, x + h], [rIn, x - h]], seg);
}

// Samples a sorted [r, x] profile between x0 and x1 (n segments), in travel order x0 -> x1.
function sampleR(profile, x0, x1, n) {
  return Array.from({ length: n + 1 }, (_, i) => {
    const x = lerp(x0, x1, i / n);
    return [radiusAt(profile, x), x];
  });
}

// Merge a list of geometries (position + normal) into one indexed geometry.
function mergeGeos(list) {
  let nV = 0;
  let nI = 0;
  for (const g of list) {
    nV += g.attributes.position.count;
    nI += g.index ? g.index.count : g.attributes.position.count;
  }
  const pos = new Float32Array(nV * 3);
  const nor = new Float32Array(nV * 3);
  const idx = new Uint32Array(nI);
  let vo = 0;
  let io = 0;
  for (const g of list) {
    const n = g.attributes.position.count;
    pos.set(g.attributes.position.array, vo * 3);
    nor.set(g.attributes.normal.array, vo * 3);
    if (g.index) {
      const a = g.index.array;
      for (let i = 0; i < a.length; i++) idx[io + i] = a[i] + vo;
      io += a.length;
    } else {
      for (let i = 0; i < n; i++) idx[io + i] = vo + i;
      io += n;
    }
    vo += n;
    g.dispose();
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  out.setIndex(new THREE.BufferAttribute(idx, 1));
  return out;
}

// ringX() collapsed into a single mesh (one draw call per blade row).
function mergeRing(geo, mat, count, radius) {
  const group = ringX(geo, mat, count, radius);
  const list = group.children.map((m) => {
    m.updateMatrix();
    return geo.clone().applyMatrix4(m.matrix);
  });
  geo.dispose();
  return new THREE.Mesh(mergeGeos(list), mat);
}

// Rotating blade row (twisted blades) centred at x.
function rotorRow(x, rHub, rTip, count, chord, mat, { twist = 0.5, stagger = 0.55, thick = 0.012 } = {}) {
  const g = blade(rTip - rHub, chord, thick, twist);
  g.rotateY(-stagger);
  const m = mergeRing(g, mat, count, rHub);
  m.position.x = x;
  return m;
}

// Static vane row (flat vanes) centred at x.
function statorRow(x, rIn, rOut, count, chord, mat, angle = 0.5, thick = 0.01) {
  const L = rOut - rIn;
  const g = new THREE.BoxGeometry(chord, L, thick);
  g.translate(0, L / 2, 0);
  g.rotateY(angle);
  const m = mergeRing(g, mat, count, rIn);
  m.position.x = x;
  return m;
}

// Hex bolt heads on flanges: rows [[x, r, count], ...] -> one InstancedMesh (heads face X).
const boltGeo = new THREE.CylinderGeometry(0.0075, 0.0075, 0.008, 6).rotateZ(Math.PI / 2);
function bolts(rows, mat) {
  const total = rows.reduce((s, r) => s + r[2], 0);
  const im = new THREE.InstancedMesh(boltGeo, mat, total);
  const d = new THREE.Object3D();
  let k = 0;
  for (const [x, r, n] of rows) {
    for (let i = 0; i < n; i++) {
      const a = ((i + 0.5) / n) * TAU;
      d.position.set(x, r * Math.cos(a), r * Math.sin(a));
      d.rotation.set(a, 0, 0);
      d.updateMatrix();
      im.setMatrixAt(k++, d.matrix);
    }
  }
  im.instanceMatrix.needsUpdate = true;
  return im;
}

// Officina fasteners: hex head + shank below it (local -Y), so +Y points out of the hole (see CONTRACT.md).
function screwGeo(r, h, len) {
  return mergeGeos([
    new THREE.CylinderGeometry(r, r, h, 6),
    new THREE.CylinderGeometry(r * 0.6, r * 0.6, len, 8).translate(0, -(h + len) / 2, 0),
  ]);
}

// Screws as one InstancedMesh; place(d, i) poses the i-th one (local +Y out of the hole).
function screwRing(geo, mat, count, place) {
  const im = new THREE.InstancedMesh(geo, mat, count);
  const d = new THREE.Object3D();
  for (let i = 0; i < count; i++) {
    place(d, i);
    d.updateMatrix();
    im.setMatrixAt(i, d.matrix);
  }
  im.instanceMatrix.needsUpdate = true;
  return im;
}

// Place an object tangent to a surface of revolution: local Y = radial, at angle aDeg around X.
function onSurface(obj, x, r, aDeg, slope = 0) {
  const a = aDeg * DEG;
  obj.position.set(x, r * Math.cos(a), r * Math.sin(a));
  obj.rotation.set(a, 0, 0);
  if (slope) obj.rotateZ(Math.atan(slope)); // follow a conical surface (dr/dx)
  return obj;
}

// Spur gear (external teeth) or ring gear (internal teeth), extruded along X and centred at x = 0.
function gearGeo(rp, teeth, m, width, { bore = 0, internal = false, rOut = 0 } = {}) {
  const p = TAU / teeth;
  const rTip = internal ? rp - m : rp + m;
  const rRoot = internal ? rp + 1.25 * m : rp - 1.25 * m;
  const outline = (path) => {
    for (let k = 0; k < teeth; k++) {
      const a0 = k * p;
      const pts = [[rRoot, a0], [rRoot, a0 + 0.2 * p], [rTip, a0 + 0.35 * p], [rTip, a0 + 0.65 * p], [rRoot, a0 + 0.8 * p]];
      pts.forEach(([r, a], i) => {
        const fn = k === 0 && i === 0 ? 'moveTo' : 'lineTo';
        path[fn](r * Math.cos(a), r * Math.sin(a));
      });
    }
    path.closePath();
  };
  let shape;
  if (internal) {
    shape = new THREE.Shape();
    shape.absarc(0, 0, rOut, 0, TAU, false);
    const hole = new THREE.Path();
    outline(hole);
    shape.holes.push(hole);
  } else {
    shape = new THREE.Shape();
    outline(shape);
    if (bore > 0) {
      const hole = new THREE.Path();
      hole.absarc(0, 0, bore, 0, TAU, true);
      shape.holes.push(hole);
    }
  }
  const g = new THREE.ExtrudeGeometry(shape, { depth: width, bevelEnabled: false, curveSegments: 10 });
  g.translate(0, 0, -width / 2);
  g.rotateY(Math.PI / 2); // extrusion Z -> X
  return g;
}

// Arc-length sampled meridional path [x, r]; place at any angle around X.
function meridian(pts, n = 240) {
  const curve = new THREE.CatmullRomCurve3(pts.map(([x, r]) => new THREE.Vector3(x, r, 0)));
  const s = curve.getSpacedPoints(n);
  const sample = (t) => {
    const f = clamp(t, 0, 1) * n;
    const i = Math.min(Math.floor(f), n - 1);
    return [i, f - i];
  };
  return {
    at(t, angle, out) {
      const [i, u] = sample(t);
      const x = lerp(s[i].x, s[i + 1].x, u);
      const r = lerp(s[i].y, s[i + 1].y, u);
      return out.set(x + OX, r * Math.cos(angle), r * Math.sin(angle));
    },
    x(t) {
      const [i, u] = sample(t);
      return lerp(s[i].x, s[i + 1].x, u);
    },
  };
}

// Piecewise colour gradient keyed by axial position x.
function gradientByX(stops) {
  const cols = stops.map(([x, c]) => [x, new THREE.Color(c)]);
  const tmp = new THREE.Color();
  return (x) => {
    if (x <= cols[0][0]) return cols[0][1].getHex();
    for (let i = 0; i < cols.length - 1; i++) {
      const [x0, c0] = cols[i];
      const [x1, c1] = cols[i + 1];
      if (x <= x1) return tmp.copy(c0).lerp(c1, (x - x0) / (x1 - x0)).getHex();
    }
    return cols[cols.length - 1][1].getHex();
  };
}

// Interpolate radius from a [r, x] profile sorted by x.
function radiusAt(pts, x) {
  if (x <= pts[0][1]) return pts[0][0];
  for (let i = 0; i < pts.length - 1; i++) {
    const [r0, x0] = pts[i];
    const [r1, x1] = pts[i + 1];
    if (x >= x0 && x <= x1) return lerp(r0, r1, (x - x0) / (x1 - x0 || 1));
  }
  return pts[pts.length - 1][0];
}

const tint = (m, color) => {
  m.color.set(color);
  return m;
};

// Core flow-path radii (hub / tip) per stage.
const boostHub = (x) => lerp(0.335, 0.312, (x + 1.16) / 0.46);
const boostTip = (x) => lerp(0.525, 0.48, (x + 1.16) / 0.46);
const hpcHub = (x) => lerp(0.225, 0.292, (x + 0.55) / 0.92);
const hpcTip = (x) => lerp(0.395, 0.338, (x + 0.55) / 0.92);
const lptHub = (x) => lerp(0.31, 0.345, (x - 1.07) / 0.63);
const lptTip = (x) => lerp(0.445, 0.545, (x - 1.07) / 0.63);

// Nacelle surfaces [r, x], sorted by x.
const OUTER_R = [[1.215, -1.95], [1.23, -1.5], [1.23, -0.9], [1.19, -0.25], [1.1, 0.35], [0.97, 0.78], [0.93, 0.8]];
const INNER_R = [[1.0, -2.0], [1.02, -1.9], [1.045, -1.75], [1.045, -1.0], [1.03, -0.6], [0.97, 0.2], [0.93, 0.8]];

// Star gearbox (GTF option): tooth counts.
const Z_SUN = 16;
const Z_PLANET = 19;
const Z_RING = 54;
const GEAR_M = 0.00875;

export default {
  id: 'turbofan',
  name: 'Turbofan ad alto rapporto di diluizione',
  short: 'Turbofan',
  family: 'aria',
  vehicle: 'Aereo di linea',
  examples: 'CFM LEAP (Airbus A320neo, Boeing 737 MAX), GE9X (Boeing 777X), Rolls-Royce Trent XWB (Airbus A350)',
  tagline: 'Un ventilatore gigante mosso da una turbina a gas: così volano gli aerei di linea.',

  intro: {
    base:
      'Il turbofan è il motore degli aerei di linea. Davanti c\'è un enorme ventilatore, il <b>fan</b>, che aspira moltissima aria: ' +
      'la maggior parte gli passa attorno e viene spinta all\'indietro, come da un\'elica chiusa in un tubo. ' +
      'Una piccola parte entra invece nel <b>nucleo</b>, dove viene schiacciata, bruciata con il cherosene e usata per far girare ' +
      'le turbine, che a loro volta fanno girare il fan. In pratica è una piccola "centrale" che alimenta un ventilatore gigante.',
    expert:
      'Turbofan <b>bialbero</b> a flussi separati e alto rapporto di diluizione (BPR ≈ 11:1 sul CFM LEAP-1A, ≈ 10:1 sul GE9X). ' +
      'Il generatore di gas (compressore HP, combustore anulare, turbina HP) realizza un <b>ciclo di Brayton</b> con rapporto di compressione ' +
      'complessivo di 40–60; la turbina di bassa pressione converte l\'energia residua dei gas in lavoro all\'albero che muove fan e booster. ' +
      'Il fan fornisce circa l\'80–90% della spinta accelerando molta aria di poco: il <b>rendimento propulsivo</b> (≈ 75–80%) moltiplicato ' +
      'per quello termico (≈ 50%) dà un rendimento globale intorno al 40%.',
  },

  specs: [
    { label: 'Carburante', value: 'Cherosene Jet A-1' },
    { label: 'Spinta al decollo', value: '110–145 kN (LEAP-1A) · ~470 kN (GE9X)' },
    { label: 'Rapporto di diluizione', value: '~11:1 (LEAP) · ~10:1 (GE9X)' },
    { label: 'Rapporto di compressione', value: '~40:1 (LEAP) · ~60:1 (GE9X)' },
    { label: 'Temperatura ingresso turbina', value: '~1500–1700 °C' },
    { label: 'Peso', value: '~3 t (LEAP-1A)' },
    { label: 'Rendimento', value: '~40% globale (termico ~50%)' },
  ],

  metrics: {
    efficiency: 40,
    power: 24000,
    powerDensity: 8,
  },

  variants: {
    columns: ['Motore', 'Aereo', 'Spinta al decollo', 'Diluizione', 'Architettura'],
    rows: [
      ['CFM LEAP-1A', 'Airbus A320neo', '~110–145 kN', '~11:1', 'Bialbero, fan diretto'],
      ['CFM LEAP-1B', 'Boeing 737 MAX', '~100–130 kN', '~9:1', 'Bialbero, fan diretto'],
      ['Pratt & Whitney PW1100G-JM', 'Airbus A320neo', '~110–150 kN', '~12:1', 'Bialbero, fan con riduttore (GTF)'],
      ['Rolls-Royce Trent XWB', 'Airbus A350', '~375–430 kN', '~9:1', 'Trialbero, fan diretto'],
      ['GE Aerospace GE9X', 'Boeing 777X', '~470 kN', '~10:1', 'Bialbero, fan diretto da 3,4 m'],
    ],
    note: 'Spinte indicative: ogni motore è certificato in più livelli di spinta a seconda della versione dell\'aereo.',
  },

  options: [
    {
      id: 'fan',
      label: 'Trasmissione al fan',
      default: 'direct',
      choices: [
        {
          id: 'direct',
          label: 'Fan diretto',
          base:
            'Il fan è montato direttamente sull\'albero di bassa pressione: fan, booster e turbina di bassa pressione girano insieme, ' +
            'alla stessa velocità. È la soluzione di CFM LEAP, GE9X e Rolls-Royce Trent.',
          expert:
            'Compromesso di velocità: il fan grande deve girare piano per non avere punte troppo supersoniche, quindi anche la turbina LP ' +
            'gira lenta e servono più stadi per estrarre la potenza (7 sul LEAP-1A, 6 sul GE9X).',
        },
        {
          id: 'gtf',
          label: 'Fan con riduttore (GTF)',
          base:
            'Tra l\'albero e il fan c\'è un riduttore a ingranaggi epicicloidali: turbina e booster girano veloci, il fan circa tre volte ' +
            'più piano. Ogni parte lavora alla sua velocità ideale. È la soluzione del Pratt & Whitney PW1000G ("Geared Turbofan").',
          expert:
            'Riduttore epicicloidale con rapporto ≈ 3:1 tra albero LP e fan: la turbina LP veloce si accontenta di 3 stadi e il booster lavora ' +
            'a velocità periferiche più efficienti; il fan può essere grande e lento (BPR ≈ 12:1 sul PW1100G-JM). Il riduttore trasmette ' +
            'decine di MW e ha un proprio circuito di lubrificazione e raffreddamento dell\'olio.',
        },
      ],
    },
  ],

  thumb: { orbit: [-40, 18], zoom: 1.15, view: 'section' },

  phases: [
    {
      id: 'intake',
      name: 'Aspirazione',
      parts: ['nacelle', 'fan'],
      base:
        'La presa d\'aria arrotondata della gondola guida l\'aria verso il grande fan senza scossoni. Il fan funziona come un ventilatore ' +
        'intubato: al decollo aspira centinaia di chili d\'aria ogni secondo (oltre una tonnellata nei motori più grandi). ' +
        'Subito dietro, un bordo affilato divide l\'aria in due strade: quella esterna e quella che entra nel nucleo.',
      expert:
        'La presa d\'aria è un diffusore subsonico: in crociera (Mach 0,78–0,85) rallenta il flusso a circa Mach 0,5–0,6 sulla faccia del fan, ' +
        'recuperando pressione. Il fan del LEAP ha 18 pale in composito di carbonio tessuto 3D con bordo d\'attacco in titanio (16 pale e 3,4 m ' +
        'di diametro sul GE9X); rapporto di compressione del fan ≈ 1,4–1,5, punte transoniche (≈ 400 m/s al decollo). ' +
        'Lo splitter separa il flusso: circa 11 parti nel condotto secondario per 1 nel nucleo.',
    },
    {
      id: 'compression',
      name: 'Compressione',
      parts: ['booster', 'hpc'],
      base:
        'L\'aria che entra nel nucleo attraversa tante file di palette che girano, alternate a palette ferme. Ogni fila la spinge e la ' +
        'schiaccia un po\' di più, come una pompa da bici azionata moltissime volte di seguito. Alla fine l\'aria occupa uno spazio ' +
        'circa 40 volte più piccolo ed è già caldissima, oltre 600 °C, senza aver ancora bruciato nulla.',
      expert:
        'Booster a 3 stadi sull\'albero di bassa pressione, poi compressore assiale di alta pressione a 10 stadi (LEAP) o 11 (GE9X) ' +
        'sull\'albero HP, con statori a calettamento variabile nei primi stadi per evitare lo stallo. Rapporto di compressione complessivo ' +
        '(OPR) ≈ 40:1 sul LEAP e ≈ 60:1 sul GE9X. La sezione di passaggio si restringe stadio dopo stadio perché l\'aria diventa più densa; ' +
        'la compressione è quasi adiabatica e T₃ in uscita arriva a ~650–750 °C. Un OPR più alto aumenta il rendimento termico del ciclo.',
    },
    {
      id: 'combustion',
      name: 'Combustione',
      parts: ['combustor'],
      base:
        'Nella camera di combustione, a forma di anello, gli iniettori spruzzano cherosene in goccioline finissime nell\'aria compressa e una ' +
        'fiamma sempre accesa lo brucia senza sosta. Non ci sono scoppi come nel motore di un\'auto: è come la fiamma di un fornello, ' +
        'ma enormemente più potente. Più si spinge la manetta, più carburante entra e più la fiamma è intensa.',
      expert:
        'Combustore anulare con circa venti iniettori (sul LEAP il sistema TAPS a doppio swirler e premiscelazione, che riduce gli NOx). ' +
        'Il calore viene aggiunto a pressione quasi costante (riscaldamento isobaro del ciclo di Brayton). Solo una parte dell\'aria ' +
        'partecipa alla zona primaria; il resto diluisce i gas e raffredda le pareti. La temperatura d\'ingresso turbina (T₄) raggiunge ' +
        '~1500–1700 °C, oltre il punto di fusione delle superleghe: servono film di raffreddamento e materiali ceramici (CMC nel GE9X).',
    },
    {
      id: 'expansion',
      name: 'Espansione',
      parts: ['hpt', 'lpt', 'shafts'],
      base:
        'I gas caldissimi corrono verso il retro e fanno girare le turbine, come il vento fa girare un mulino. La prima turbina, piccola e ' +
        'velocissima, fa girare il compressore; quelle successive, più grandi e lente, fanno girare il fan davanti tramite un albero che passa ' +
        'dentro l\'altro. Così il motore si mantiene in moto da solo.',
      expert:
        'La turbina HP (2 stadi) aziona il compressore HP; la turbina LP (7 stadi sul LEAP-1A, 6 sul GE9X) aziona fan e booster tramite ' +
        'l\'albero LP, coassiale e interno all\'albero HP. I due alberi sono meccanicamente indipendenti: N1 resta su qualche migliaio di ' +
        'giri/min per limitare la velocità di punta del grande fan, N2 supera i 15 000 giri/min. Le pale HPT, monocristalline in superlega ' +
        'di nichel con canali di raffreddamento e barriera termica ceramica, lavorano in gas più caldi del loro punto di fusione. ' +
        'All\'uscita la temperatura dei gas (EGT) è ~850–950 °C al decollo.',
    },
    {
      id: 'exhaust',
      name: 'Scarico e spinta',
      parts: ['nozzle', 'bypass', 'reverser'],
      base:
        'Dal retro escono due getti: quello caldo e veloce del nucleo e quello, molto più grande, di aria fredda spinta dal fan. È soprattutto ' +
        'quest\'ultimo a far volare l\'aereo: muovere tanta aria con calma consuma meno che sparare poca aria a velocità altissima. ' +
        'Spingendo l\'aria all\'indietro, il motore viene spinto in avanti, come quando si rema.',
      expert:
        'Spinta netta F ≈ ṁ_bypass·(v₁₉ − v₀) + ṁ_core·(v₉ − v₀): con BPR 11:1 il flusso secondario dà circa l\'80–90% della spinta. ' +
        'A parità di spinta un getto più lento e massiccio spreca meno energia cinetica (η_prop ≈ 2 / (1 + v_j/v₀)). Entrambi gli ugelli sono ' +
        'convergenti; il cono di scarico guida il getto caldo. Un LEAP-1A dà 110–145 kN al decollo con un consumo specifico (TSFC) ' +
        'di crociera di circa 15 g/(kN·s), ~15% in meno del CFM56 che sostituisce.',
    },
  ],

  quiz: [
    {
      q: 'Da dove viene la maggior parte della spinta in un turbofan ad alto rapporto di diluizione?',
      options: [
        'Dal getto caldo che esce dal nucleo',
        'Dall\'aria spinta dal fan nel condotto secondario',
        'Dal postbruciatore',
        'Dalla rotazione del cono di scarico',
      ],
      correct: 1,
      explain:
        'Circa l\'80–90% della spinta viene dal flusso freddo del fan: accelerare di poco tanta aria è molto più efficiente che accelerare tantissimo poca aria.',
    },
    {
      q: 'Cosa indica un rapporto di diluizione di 11:1?',
      options: [
        'Il motore brucia 11 kg d\'aria per ogni kg di cherosene',
        'Il fan gira 11 volte più piano della turbina di alta pressione',
        'Per ogni kg d\'aria che entra nel nucleo, 11 kg passano nel condotto esterno',
        'L\'aria viene compressa 11 volte',
      ],
      correct: 2,
      explain:
        'Il rapporto di diluizione (bypass ratio) confronta la portata d\'aria che aggira il nucleo con quella che lo attraversa. Più è alto, più il motore è efficiente e silenzioso.',
    },
    {
      q: 'Perché il fan e il compressore di alta pressione girano a velocità diverse?',
      options: [
        'Sono su due alberi coassiali indipendenti, ciascuno mosso dalla sua turbina',
        'Per risparmiare olio lubrificante',
        'Il fan è mosso da un motore elettrico separato',
        'In realtà girano alla stessa velocità',
      ],
      correct: 0,
      explain:
        'Il fan è grande: se girasse veloce come il compressore HP le punte delle pale andrebbero ben oltre la velocità del suono. Con due alberi (bassa e alta pressione) ciascuna parte gira alla velocità migliore.',
    },
    {
      q: 'Qual è il compito principale delle turbine in un turbofan?',
      options: [
        'Raffreddare i gas di scarico prima che escano',
        'Iniettare il carburante nella camera di combustione',
        'Aumentare la pressione dell\'aria in ingresso',
        'Estrarre energia dai gas caldi per far girare compressori e fan',
      ],
      correct: 3,
      explain:
        'Le turbine "raccolgono" parte dell\'energia dei gas caldi e la trasmettono, tramite gli alberi, al compressore (turbina HP) e al fan (turbina LP).',
    },
  ],

  // Officina (see CONTRACT.md): a simplified modular strip-down, from the nacelle in to the core.
  // The pylon stays as the base: the LP turbine hangs from its aft mount, so it comes off last.
  disassembly: {
    intro: 'Smontiamo il turbofan un modulo alla volta, dall\'esterno verso il cuore: gondola, fan, poi il nucleo.',
    note:
      'Sequenza semplificata a scopo didattico: un turbofan vero si smonta per moduli in officine specializzate, ' +
      'seguendo il manuale del costruttore e con attrezzature dedicate.',
    parts: {
      nacelle: { dir: [-1, 0, 0], dist: 2.0 },
      reverser: { dir: [1, 0, 0], dist: 1.8 },
      fanBolts: { motion: 'unscrew', turns: 5, out: 0.06, dir: [-1, 0, 0], dist: 0.5 },
      fan: { after: ['fanBolts'], dir: [-1, 0, 0], dist: 1.8 },
      reduction: { after: ['fan'], dir: [-1, 0, 0], dist: 1.4 },
      gearbox: { after: ['nacelle'], dir: [0, -1, 0], dist: 1.2 },
      fanCaseBolts: { after: ['nacelle', 'gearbox'], motion: 'unscrew', turns: 5, out: 0.07, dir: [-1, 0, 0], dist: 0.4, duration: 1.8 },
      fancase: { after: ['fanCaseBolts', 'nacelle', 'gearbox', 'fan'], dir: [-1, 0, 0], dist: 2.2, duration: 1.1 },
      nozzleBolts: { motion: 'unscrew', turns: 5, out: 0.06, dir: [1, 0, 0], dist: 0.4 },
      nozzle: { after: ['nozzleBolts'], dir: [1, 0, 0], dist: 1.6 },
      bypass: { after: ['reverser', 'fancase'], dir: [-1, 0, 0], dist: 2.4, duration: 1.2 },
      boosterBolts: { after: ['bypass'], motion: 'unscrew', turns: 5, out: 0.06, dir: [-1, 0, 0], dist: 0.4 },
      booster: { after: ['boosterBolts', 'bypass', 'reduction'], dir: [-1, 0, 0], dist: 1.5 },
      hpc: { after: ['booster'], dir: [-1, 0, 0], dist: 1.8 },
      combustor: { after: ['hpc'], dir: [-1, 0, 0], dist: 1.8 },
      hpt: { after: ['combustor'], dir: [-1, 0, 0], dist: 1.8 },
      shafts: { after: ['booster', 'hpc', 'hpt', 'nozzle'], dir: [-1, 0, 0], dist: 2.4, duration: 1.2 },
      lpt: { after: ['hpt', 'shafts', 'nozzle'], dir: [0, -1, 0], dist: 1.2 },
    },
    steps: [
      {
        id: 'nacelle',
        name: 'Togli gondola e invertitore',
        base:
          'Si sfila in avanti la gondola con la presa d\'aria e si toglie da dietro l\'invertitore di spinta: ' +
          'sotto compaiono la cassa del fan e la carenatura del nucleo.',
        expert:
          'Sull\'aereo gli sportelli del fan e le due metà dell\'invertitore sono incernierati al pilone e si aprono verso l\'alto ' +
          'come un cofano; la presa d\'aria è imbullonata alla flangia anteriore della cassa del fan. Qui li togliamo interi per semplicità.',
        parts: ['nacelle', 'reverser'],
      },
      {
        id: 'fan',
        name: 'Smonta il fan',
        base:
          'Si svitano le viti alla base dell\'ogiva e si sfila in avanti il fan con il suo disco. ' +
          'Nei motori con riduttore, subito dietro c\'è il gruppo di ingranaggi.',
        expert:
          'Nella realtà si toglie prima l\'ogiva; le pale sono infilate una a una in cave a coda di rondine del disco e si possono ' +
          'sostituire singolarmente, anche con il motore sotto l\'ala. Qui ogiva, pale e disco escono insieme.',
        parts: ['fanBolts', 'fan', 'reduction'],
      },
      {
        id: 'agb',
        name: 'Stacca la scatola accessori',
        base:
          'Si scollegano tubi e cavi e si abbassa la scatola accessori, appesa sotto la cassa del fan con pompe, generatore e avviatore.',
        expert:
          'Pompe, generatore e starter sono unità sostituibili singolarmente (LRU), fissate ai loro attacchi sulla scatola. ' +
          'Insieme alla scatola si sfila l\'albero radiale che la collega all\'albero di alta pressione.',
        parts: ['gearbox'],
      },
      {
        id: 'fancase',
        name: 'Sfila la cassa del fan',
        base: 'Si svitano le viti che uniscono la cassa alle palette raddrizzatrici, poi si sfila in avanti il grande anello.',
        expert:
          'Le palette raddrizzatrici (OGV) collegano la cassa del fan al nucleo: in molti motori moderni, oltre a raddrizzare il flusso, ' +
          'fanno da struttura portante. Nei motori veri fan, cassa e booster formano spesso un unico grande modulo.',
        parts: ['fanCaseBolts', 'fancase'],
      },
      {
        id: 'nozzle',
        name: 'Togli l\'ugello di scarico',
        base: 'Si svita la corona di viti in coda e si sfilano all\'indietro l\'ugello e il cono di scarico.',
        expert:
          'Ugello e cono sono gusci leggeri in lega resistente al calore, imbullonati al telaio posteriore della turbina: la struttura ' +
          'a bracci che sostiene l\'ultimo cuscinetto. In questo modello il telaio esce insieme all\'ugello.',
        parts: ['nozzleBolts', 'nozzle'],
      },
      {
        id: 'corecowl',
        name: 'Scopri il nucleo',
        base:
          'Si sfila la carenatura che avvolge il nucleo, insieme alle palette raddrizzatrici: ora il generatore di gas è tutto in vista.',
        expert:
          'Sull\'aereo la carenatura del nucleo si apre in due metà, senza smontarla: sotto ci sono tubazioni, cablaggi, il collettore ' +
          'del carburante e i tappi per l\'ispezione endoscopica (boroscopio) degli stadi interni.',
        parts: ['bypass'],
      },
      {
        id: 'compressors',
        name: 'Sfila i compressori',
        base:
          'Si svita la flangia tra i due compressori e li si sfila in avanti: prima il booster, poi il compressore di alta pressione.',
        expert:
          'Le casse dei moduli sono unite da flange imbullonate, i rotori da accoppiamenti scanalati o flange. I rotori sono equilibrati ' +
          'con grande cura: a oltre 15 000 giri/min basta uno squilibrio minimo per creare forti vibrazioni.',
        parts: ['boosterBolts', 'booster', 'hpc'],
      },
      {
        id: 'hot',
        name: 'Smonta la parte calda',
        base:
          'Tocca alla parte calda: la camera di combustione con i suoi iniettori e, subito dietro, la turbina di alta pressione.',
        expert:
          'Compressore HP, combustore e turbina HP formano insieme il modulo "core", il generatore di gas. Le parti calde sono le più ' +
          'sollecitate: liner, distributori e palette si ispezionano alla ricerca di cricche, bruciature e perdita del rivestimento ceramico.',
        parts: ['combustor', 'hpt'],
      },
      {
        id: 'lp',
        name: 'Sfila gli alberi e l\'ultima turbina',
        base:
          'Si sfilano in avanti i due alberi coassiali e per ultima si abbassa la turbina di bassa pressione, ' +
          'rimasta appesa all\'attacco posteriore del pilone.',
        expert:
          'Nei motori veri l\'albero di bassa pressione esce di solito da dietro insieme al modulo della turbina LP, a cui è unito, ' +
          'e l\'albero HP fa parte del rotore del nucleo: qui li sfiliamo da soli per mostrarli. Resta il pilone, che appartiene all\'aereo e non al motore.',
        parts: ['shafts', 'lpt'],
      },
    ],
  },

  camera: { position: [-5.0, 2.3, 6.3], target: [0.1, -0.05, 0] },

  build(ctx) {
    const root = new THREE.Group();
    const model = new THREE.Group();
    model.position.x = OX;
    root.add(model);

    // ---------- Materials ----------
    const mNacelle = cut(MAT.paint(0xdfe3e8, 0.38));
    const mPanel = cut(MAT.paint(0xcdd2d8, 0.5));
    const mSeal = cut(MAT.rubber());
    const mLiner = cut(MAT.paint(0x7f868e, 0.62)); // acoustic liner
    const mLip = cut(MAT.aluminum());
    const mStructure = cut(tint(MAT.aluminum(), 0x9ba2aa));
    const mFanCase = cut(tint(MAT.aluminum(), 0xaab0b6));
    const mKevlar = cut(tint(MAT.carbon(), 0x2c2a26));
    const mRub = cut(MAT.paint(0xb8b2a4, 0.8));
    const mCascade = cut(tint(MAT.darkSteel(), 0x4c5259));
    const mPylon = cut(MAT.paint(0xe9ecef, 0.42));
    const mCowl = cut(MAT.paint(0xc6cbd1, 0.42));
    const mCase = cut(MAT.steel());
    const mCastCase = cut(tint(MAT.castIron(), 0x6a6f76));
    const mStator = cut(MAT.darkSteel());
    const mOGV = cut(MAT.aluminum());
    const mBolt = cut(MAT.steel());
    const mBronze = cut(MAT.brass());
    const mFan = MAT.titanium();
    const mSpinner = cut(MAT.paint(0x3a3f46, 0.3));
    const mSpiral = cut(MAT.paint(0xf4f4f2, 0.5));
    const mTi = MAT.titanium();
    const mNi = MAT.steel();
    const mDrum = cut(MAT.darkSteel());
    const mHot = tint(MAT.darkSteel(), 0x9a8c7c);
    mHot.emissive.set(0xff3a0a);
    mHot.emissiveIntensity = 0;
    const mHotStator = cut(tint(MAT.darkSteel(), 0x8a7d70));
    const mLpt = tint(MAT.steel(), 0xa9a49c);
    const mCombLiner = cut(tint(MAT.darkSteel(), 0x9c8c7a));
    const mNozzle = cut(tint(MAT.darkSteel(), 0x8a8178));
    const mPlug = cut(tint(MAT.darkSteel(), 0x857c72));
    const mFuel = cut(MAT.copper());
    const mInjector = cut(MAT.brass());
    const mTube = cut(tint(MAT.steel(), 0xc9ced4));
    const mHarness = cut(tint(MAT.rubber(), 0x3b3428));
    const mShaftLP = cut(MAT.chrome());
    const mShaftHP = cut(MAT.steel());
    const mGear = cut(tint(MAT.steel(), 0xc8ccd0));
    const mGearbox = tint(MAT.castIron(), 0x6b7078);
    const mAccessory = MAT.aluminum();
    const mAccDark = MAT.paint(0x30343a, 0.55);
    const mDark = cut(MAT.paint(0x2a2e33, 0.6));

    const fanRotors = []; // always at fan speed
    const lpRotors = []; // LP shaft speed (= fan speed unless GTF)
    const hpRotors = [];
    const hotMeshes = [];

    // =============== Nacelle: inlet + fan cowl doors ===============
    const nacelle = new THREE.Group();
    const lipPts = [
      [1.0, -2.0], [1.01, -2.12], [1.04, -2.21], [1.09, -2.25],
      [1.15, -2.24], [1.19, -2.17], [1.21, -2.05], [1.216, -1.95],
    ];
    nacelle.add(new THREE.Mesh(skin(lipPts, 0.014), mLip));
    nacelle.add(new THREE.Mesh(skin(sampleR(OUTER_R, -1.95, -1.62, 4), 0.012), mNacelle));
    nacelle.add(new THREE.Mesh(skin(sampleR(INNER_R, -1.765, -2.0, 4), 0.018), mLiner));
    nacelle.add(new THREE.Mesh(ring(-2.1, 1.03, 1.19, 0.012), mStructure)); // anti-ice D-duct bulkhead
    nacelle.add(new THREE.Mesh(ring(-1.71, 1.08, 1.206, 0.014), mStructure)); // aft bulkhead
    nacelle.add(new THREE.Mesh(ring(-1.95, 1.2, 1.2185, 0.006, 96), mSeal));
    // fan cowl doors (two halves hinged on the pylon)
    nacelle.add(new THREE.Mesh(skin(sampleR(OUTER_R, -1.6, -0.54, 8), 0.012), mNacelle));
    nacelle.add(new THREE.Mesh(ring(-1.61, 1.2, 1.222, 0.02, 96), mSeal));
    for (const x of [-1.5, -1.07, -0.65]) {
      nacelle.add(new THREE.Mesh(ring(x, 1.196, 1.217, 0.012, 72), mStructure)); // cowl frames
    }
    {
      const strip = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.004, 0.006), mSeal);
      strip.position.set(-1.25, -1.2305, 0);
      nacelle.add(strip);
      const hingeGeo = new RoundedBoxGeometry(0.07, 0.03, 0.06, 2, 0.008);
      for (const x of [-1.45, -1.2]) nacelle.add(onSurface(new THREE.Mesh(hingeGeo, mDark), x, 1.24, 0));
      const latchGeo = new RoundedBoxGeometry(0.1, 0.012, 0.03, 2, 0.005);
      for (const x of [-1.35, -1.05, -0.75]) nacelle.add(onSurface(new THREE.Mesh(latchGeo, mDark), x, 1.233, 180));
      // access panel with dark outline, anti-ice exhaust grille on the inlet
      nacelle.add(onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.29, 0.004, 0.2, 1, 0.0015), mSeal), -1.15, 1.2312, 62));
      nacelle.add(onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.275, 0.006, 0.185, 1, 0.0025), mPanel), -1.15, 1.2322, 62));
      nacelle.add(onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.11, 0.006, 0.07, 1, 0.0025), mDark), -1.8, 1.221, 48));
      const slat = new THREE.BoxGeometry(0.09, 0.004, 0.006);
      for (let i = -1; i <= 1; i++) {
        const s = onSurface(new THREE.Mesh(slat, mLip), -1.8, 1.225, 48);
        s.translateZ(i * 0.018);
        nacelle.add(s);
      }
      // flush screws along the inlet/lip joint
      const screw = new THREE.CylinderGeometry(0.0045, 0.0045, 0.003, 6);
      const ims = new THREE.InstancedMesh(screw, mBolt, 96);
      const d = new THREE.Object3D();
      for (let i = 0; i < 96; i++) {
        onSurface(d, -1.935, 1.2175, (i / 96) * 360);
        d.updateMatrix();
        ims.setMatrixAt(i, d.matrix);
      }
      nacelle.add(ims);
    }
    model.add(
      ctx.part(nacelle, {
        id: 'nacelle',
        name: 'Gondola e presa d\'aria',
        base:
          'Il guscio aerodinamico che avvolge il motore. Il bordo arrotondato davanti (labbro) guida l\'aria dolcemente verso il fan; ' +
          'dentro è rivestito di pannelli fonoassorbenti. I due grandi sportelli laterali si aprono verso l\'alto, come un cofano, per la manutenzione.',
        expert:
          'Presa d\'aria con labbro in alluminio antighiaccio (riscaldato con aria spillata dal compressore in un condotto a "D") e pannelli acustici ' +
          'a nido d\'ape. Le fan cowl sono due semigusci incernierati al pilone e chiusi da ganci (latch) sulla linea inferiore; una volta aperti ' +
          'danno accesso alla cassa del fan, alla scatola accessori e all\'elettronica FADEC.',
        explode: [0, 1.9, 0],
      }),
    );

    // =============== Thrust reverser (translating sleeve + cascades) ===============
    const reverser = new THREE.Group();
    reverser.add(new THREE.Mesh(skin(sampleR(INNER_R, -0.545, 0.765, 8), -0.014), mLiner)); // bypass duct outer wall
    reverser.add(new THREE.Mesh(skin(sampleR(OUTER_R, -0.52, 0.765, 8), 0.012), mNacelle)); // sleeve outer skin
    reverser.add(new THREE.Mesh(latheX([[0.935, 0.76], [0.976, 0.76], [0.97, 0.802], [0.93, 0.802], [0.935, 0.76]], 96), mNacelle));
    reverser.add(new THREE.Mesh(ring(-0.535, 1.045, 1.2, 0.036), mStructure)); // torque box
    reverser.add(new THREE.Mesh(ring(-0.53, 1.19, 1.205, 0.016, 96), mSeal));
    for (let k = 0; k < 8; k++) {
      const x = -0.47 + k * 0.052;
      reverser.add(new THREE.Mesh(latheX([[1.065, x], [1.15, x - 0.026], [1.15, x - 0.016], [1.065, x + 0.01], [1.065, x]], 72), mCascade));
    }
    {
      const sb = new THREE.BoxGeometry(0.42, 0.085, 0.008).translate(0, 0.0425, 0);
      const strongbacks = mergeRing(sb, mCascade, 36, 1.065);
      strongbacks.position.x = -0.28;
      reverser.add(strongbacks);
      const rp = radiusAt(OUTER_R, 0.0);
      reverser.add(onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.3, 0.004, 0.22, 1, 0.0015), mSeal), 0.0, rp + 0.0012, 66, -0.15));
      reverser.add(onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.285, 0.006, 0.205, 1, 0.0025), mPanel), 0.0, rp + 0.0022, 66, -0.15));
    }
    model.add(
      ctx.part(reverser, {
        id: 'reverser',
        name: 'Invertitore di spinta',
        base:
          'Dopo l\'atterraggio la parte posteriore della gondola scorre all\'indietro e scopre delle griglie: l\'aria del fan viene deviata ' +
          'in avanti e aiuta a frenare l\'aereo, risparmiando i freni delle ruote.',
        expert:
          'Invertitore a cascate con manicotto traslante (translating sleeve): attuatori spostano il manicotto all\'indietro, le blocker door ' +
          'chiudono il condotto secondario e il flusso del fan esce attraverso le schiere di palette (cascades) orientate in avanti. ' +
          'Agisce solo sul flusso freddo; la parte interna del manicotto forma la parete esterna del condotto di bypass.',
        explode: [0.9, 1.9, 0],
      }),
    );

    // =============== Pylon stub + engine mounts ===============
    const pylon = new THREE.Group();
    {
      const sh = new THREE.Shape();
      const bottom = sampleR(OUTER_R, -0.92, 0.7, 8).map(([r, x]) => [x, r - 0.012]);
      sh.moveTo(bottom[0][0], bottom[0][1]);
      for (const [x, y] of bottom.slice(1)) sh.lineTo(x, y);
      for (const [x, y] of [[0.86, 0.85], [1.05, 0.7], [1.4, 0.615], [1.78, 0.6], [1.98, 0.72], [1.3, 1.45], [1.0, 1.74], [-0.45, 1.74]]) sh.lineTo(x, y);
      sh.closePath();
      // box-beam bays: the section shows spars and ribs, not a solid slab
      const yb = (x) => (x <= 0.7 ? radiusAt(OUTER_R, x) - 0.012 : lerp(0.85, 0.7, (x - 0.86) / 0.19));
      for (const [x0, x1, top0, top1] of [[-0.38, 0.12, 1.69, 1.69], [0.2, 0.7, 1.69, 1.69], [0.92, 1.04, 1.68, 1.62]]) {
        const h = new THREE.Path();
        h.moveTo(x0, yb(x0) + 0.05);
        h.lineTo(x1, yb(x1) + 0.05);
        h.lineTo(x1, top1);
        h.lineTo(x0, top0);
        h.closePath();
        sh.holes.push(h);
        // side skins over the bay, slightly recessed (the front one is cut away in the section view)
        const skinShape = new THREE.Shape();
        skinShape.moveTo(x0 - 0.016, yb(x0) + 0.034);
        skinShape.lineTo(x1 + 0.016, yb(x1) + 0.034);
        skinShape.lineTo(x1 + 0.016, top1 + 0.016);
        skinShape.lineTo(x0 - 0.016, top0 + 0.016);
        skinShape.closePath();
        for (const z of [-0.076, 0.066]) {
          pylon.add(new THREE.Mesh(new THREE.ExtrudeGeometry(skinShape, { depth: 0.01, bevelEnabled: false }).translate(0, 0, z), mPanel));
        }
      }
      const g = new THREE.ExtrudeGeometry(sh, { depth: 0.13, bevelEnabled: true, bevelThickness: 0.014, bevelSize: 0.014, bevelSegments: 2, curveSegments: 4 });
      g.translate(0, 0, -0.065);
      pylon.add(new THREE.Mesh(g, mPylon));
      // cut-off top (wing attachment) and mounts to the fan case / turbine case
      const top = new THREE.Mesh(new RoundedBoxGeometry(1.5, 0.03, 0.2, 2, 0.01), mStructure);
      top.position.set(0.27, 1.755, 0);
      pylon.add(top);
      const fwd = new THREE.Mesh(new RoundedBoxGeometry(0.16, 0.14, 0.12, 2, 0.02), mCastCase);
      fwd.position.set(-0.75, 1.14, 0);
      pylon.add(fwd);
      const aft = new THREE.Mesh(new RoundedBoxGeometry(0.14, 0.07, 0.1, 2, 0.015), mCastCase);
      aft.position.set(1.47, 0.57, 0);
      pylon.add(aft);
    }
    model.add(
      ctx.part(pylon, {
        id: 'pylon',
        name: 'Pilone (attacco all\'ala)',
        base:
          'La "trave" che appende il motore sotto l\'ala. Dentro passano i tubi del carburante, dell\'aria e i cavi elettrici che collegano motore e aereo.',
        expert:
          'Pilone a cassone in titanio e acciaio: trasmette spinta e peso all\'ala tramite l\'attacco anteriore (sulla cassa del fan) e quello ' +
          'posteriore (sul telaio della turbina). Ospita condotti dell\'aria spillata (bleed), linee carburante e idrauliche, cablaggi; ' +
          'la carenatura posteriore separa i getti caldo e freddo.',
        explode: [0, 2.6, 0],
      }),
    );

    // =============== Fan case + containment ===============
    const fanCase = new THREE.Group();
    fanCase.add(new THREE.Mesh(skin(sampleR(INNER_R, -1.765, -0.565, 8), -0.03), mFanCase));
    fanCase.add(new THREE.Mesh(ring(-1.757, 1.045, 1.125, 0.016), mFanCase));
    fanCase.add(new THREE.Mesh(ring(-0.565, 1.03, 1.1, 0.016), mFanCase));
    fanCase.add(new THREE.Mesh(latheX([[1.074, -1.67], [1.13, -1.63], [1.13, -1.24], [1.074, -1.2], [1.074, -1.67]], 96), mKevlar));
    fanCase.add(new THREE.Mesh(latheX([[1.036, -1.6], [1.046, -1.6], [1.046, -1.3], [1.036, -1.3], [1.036, -1.6]], 96), mRub));
    fanCase.add(bolts([[-1.744, 1.1, 72], [-0.552, 1.075, 60]], mBolt));
    fanCase.add(new THREE.Mesh(ring(-0.85, 1.062, 1.078, 0.15), mFanCase)); // OGV attachment band
    model.add(
      ctx.part(
        // two rows of radial screws through the band into the outer ends of the OGVs
        screwRing(screwGeo(0.015, 0.012, 0.04), mBolt, 72, (d, i) =>
          onSurface(d, i < 36 ? -0.885 : -0.815, 1.084, (i % 36) * 10 + 5),
        ),
        {
          id: 'fanCaseBolts',
          name: 'Viti della cassa del fan',
          base:
            'Due file di viti tutto intorno alla cassa del fan: la uniscono alle palette raddrizzatrici, che a loro volta la collegano al nucleo.',
          expert:
            'Le estremità esterne delle palette raddrizzatrici (OGV) sono imbullonate alla cassa attraverso una fascia di rinforzo. ' +
            'È una giunzione strutturale: attraverso le OGV i carichi del fan e della presa d\'aria passano al resto del motore.',
          explode: [0, 0, -2.4],
        },
      ),
    );
    model.add(
      ctx.part(fanCase, {
        id: 'fancase',
        name: 'Cassa del fan e anello di contenimento',
        base:
          'Il robusto anello che circonda il fan. Se una pala si rompesse, deve trattenerla all\'interno del motore: per questo è avvolto ' +
          'da strati di fibra resistentissima.',
        expert:
          'Cassa in lega leggera o composito con fascia di contenimento avvolta in fibra aramidica (Kevlar) o composito tessuto: deve trattenere ' +
          'una pala rotta alla massima velocità (prova di "blade-out" obbligatoria per la certificazione). Sulla superficie interna, uno strato ' +
          'abradabile sopra le punte delle pale riduce il gioco senza danneggiarle. Porta le flange della presa d\'aria e le OGV.',
        explode: [0, 0, -2.4],
      }),
    );

    // =============== Fan + spinner ===============
    const fan = new THREE.Group();
    const fanRotor = new THREE.Group();
    fan.add(fanRotor);
    fanRotors.push(fanRotor);

    const FAN_L = 0.7;
    const fanGeo = blade(FAN_L, 0.36, 0.03, 0.85);
    {
      // wide-chord scimitar shape: narrow root, swept tip
      const p = fanGeo.attributes.position;
      for (let i = 0; i < p.count; i++) {
        const u = p.getY(i) / FAN_L;
        const f = 0.7 + 0.75 * u - 0.5 * u * u;
        p.setXYZ(i, p.getX(i) * f + 0.06 * u * u, p.getY(i), p.getZ(i) * f);
      }
      fanGeo.computeVertexNormals();
      fanGeo.rotateY(-0.32);
    }
    const fanBlades = ringX(fanGeo, mFan, 18, 0.3);
    fanBlades.position.x = -1.45;
    fanRotor.add(fanBlades);

    const spinPts = [
      [0, -2.06], [0.05, -2.03], [0.12, -1.95], [0.19, -1.84], [0.25, -1.72],
      [0.29, -1.6], [0.305, -1.52], [0.305, -1.24],
    ];
    fanRotor.add(new THREE.Mesh(skin(spinPts, 0.014, 64), mSpinner));
    // fan disk (rim under blade roots) and web down to the shaft
    fanRotor.add(new THREE.Mesh(latheX([[0.235, -1.6], [0.291, -1.6], [0.291, -1.3], [0.235, -1.3], [0.235, -1.6]], 64), mDrum));
    fanRotor.add(new THREE.Mesh(latheX([[0.055, -1.31], [0.24, -1.36], [0.24, -1.32], [0.055, -1.28], [0.055, -1.31]], 64), mDrum));
    {
      // spiral marking on the spinner (makes rotation readable)
      const pts = [];
      for (let i = 0; i <= 40; i++) {
        const u = 0.06 + (i / 40) * 0.86;
        const x = lerp(-2.06, -1.56, u);
        const r = radiusAt(spinPts, x) + 0.004;
        const a = u * TAU * 1.1;
        pts.push(new THREE.Vector3(x, r * Math.cos(a), r * Math.sin(a)));
      }
      const tube = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 80, 0.011, 5, false);
      fanRotor.add(new THREE.Mesh(tube, mSpiral));
    }
    model.add(
      ctx.part(fan, {
        id: 'fan',
        name: 'Fan e ogiva',
        base:
          'Il grande ventilatore anteriore, con pale larghe e svergolate. Spinge all\'indietro la maggior parte dell\'aria e produce quasi tutta ' +
          'la spinta. La spirale bianca sull\'ogiva serve al personale di terra per vedere se il fan sta girando.',
        expert:
          'Fan monostadio a pale "wide-chord" a freccia (18 sul LEAP, in composito RTM tessuto 3D con bordo d\'attacco in titanio). ' +
          'Rapporto di compressione ≈ 1,4–1,5, velocità di punta transonica. È mosso dalla turbina di bassa pressione, direttamente ' +
          'tramite l\'albero LP oppure attraverso un riduttore nei turbofan "geared".',
        explode: [-1.7, 0, 0],
      }),
    );
    // spinner retaining screws (their own part for the Officina wrench); they turn with the fan
    const fanBolts = new THREE.Group();
    fanBolts.add(
      screwRing(screwGeo(0.014, 0.012, 0.035), mBolt, 12, (d, i) => onSurface(d, -1.66, 0.27, i * 30 + 15, 1 / 3).translateY(0.005)),
    );
    fanRotors.push(fanBolts);
    model.add(
      ctx.part(fanBolts, {
        id: 'fanBolts',
        name: 'Viti dell\'ogiva',
        base: 'Una corona di viti alla base dell\'ogiva: la tiene fissata al disco del fan e gira insieme a lui.',
        expert:
          'L\'ogiva (spinner) è avvitata al disco del fan con una corona di viti. Spesso in questa zona si montano anche i pesetti ' +
          'di equilibratura, con cui si correggono le vibrazioni del fan senza smontarlo (trim balance).',
        explode: [-1.7, 0, 0],
      }),
    );

    // =============== Star reduction gearbox (GTF option) ===============
    const reduction = new THREE.Group();
    const ringRot = new THREE.Group(); // ring gear + coupling to fan (fan speed)
    reduction.add(ringRot);
    const rSun = (Z_SUN * GEAR_M) / 2;
    const rPlanet = (Z_PLANET * GEAR_M) / 2;
    const rRing = (Z_RING * GEAR_M) / 2;
    const GX = -1.24;
    const ringGear = new THREE.Mesh(gearGeo(rRing, Z_RING, GEAR_M, 0.05, { internal: true, rOut: rRing + 0.03 }), mGear);
    ringGear.position.x = GX;
    ringRot.add(ringGear);
    ringRot.add(new THREE.Mesh(latheX([[rRing + 0.012, -1.3], [rRing + 0.03, -1.3], [rRing + 0.03, -1.265], [rRing + 0.012, -1.265], [rRing + 0.012, -1.3]], 64), mGear));
    const sunGear = new THREE.Mesh(gearGeo(rSun, Z_SUN, GEAR_M, 0.05, { bore: 0.05 }), mGear);
    sunGear.position.x = GX;
    reduction.add(sunGear);
    lpRotors.push(sunGear);
    const planets = [];
    const planetGeo = gearGeo(rPlanet, Z_PLANET, GEAR_M, 0.046, { bore: 0.012 });
    const orbit = rSun + rPlanet;
    for (const aDeg of [180, 300, 60]) {
      const p = new THREE.Mesh(planetGeo, mGear);
      const a = aDeg * DEG;
      p.position.set(GX, orbit * Math.cos(a), orbit * Math.sin(a));
      reduction.add(p);
      planets.push(p);
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.075, 12).rotateZ(Math.PI / 2), mCase);
      pin.position.set(GX + 0.01, p.position.y, p.position.z);
      reduction.add(pin);
    }
    // fixed carrier: rear plate + torque tube to the static diaphragm
    reduction.add(new THREE.Mesh(ring(-1.198, 0.085, 0.19, 0.014, 64), mCastCase));
    reduction.add(new THREE.Mesh(ring(-1.175, 0.12, 0.14, 0.035, 64), mCastCase));
    const wReduction = ctx.part(reduction, {
      id: 'reduction',
      name: 'Riduttore epicicloidale (GTF)',
      base:
        'Un gruppo di ingranaggi tra albero e fan: un ingranaggio centrale veloce fa girare alcuni ingranaggi intermedi, che a loro volta ' +
        'fanno girare una corona dentata collegata al fan, circa tre volte più piano.',
      expert:
        'Riduttore epicicloidale a stella: solare sull\'albero LP, satelliti su portatreno fisso, corona collegata al fan. Rapporto ≈ 3:1 ' +
        'sul PW1000G. Con il portatreno fisso la corona gira in verso opposto al solare. Ingranaggi a denti dritti o a doppia elica, ' +
        'lubrificazione forzata e scambiatore olio dedicato.',
      explode: [-1.35, 0, 0],
    });
    model.add(wReduction);

    // =============== Booster / LPC (LP spool) ===============
    const booster = new THREE.Group();
    const boostRotor = new THREE.Group();
    booster.add(boostRotor);
    lpRotors.push(boostRotor);
    boostRotor.add(new THREE.Mesh(latheX([[0.31, -1.09], [0.332, -1.09], [0.312, -0.7], [0.06, -0.7], [0.06, -0.73], [0.29, -0.73], [0.31, -1.09]], 64), mDrum));
    for (const x of [-1.06, -0.93, -0.8]) {
      boostRotor.add(rotorRow(x, boostHub(x), boostTip(x) - 0.004, 36, 0.06, mTi, { twist: 0.55, stagger: 0.6 }));
      boostRotor.add(new THREE.Mesh(ring(x, 0.2, boostHub(x) - 0.015, 0.022, 48), mDrum));
    }
    for (const x of [-1.13, -0.995, -0.865, -0.735]) {
      booster.add(statorRow(x, boostHub(x) + 0.004, boostTip(x) + 0.006, 40, 0.045, mStator, 0.45));
    }
    booster.add(new THREE.Mesh(wall([[0.53, -1.16], [0.485, -0.7]], 0.014, 72), mCase));
    booster.add(new THREE.Mesh(ring(-0.708, 0.495, 0.53, 0.014), mCase));
    // (the bolt circle of this flange is its own part: boosterBolts, below)
    // static IGV inner shroud + bearing diaphragm (also carries the GTF gear carrier)
    booster.add(new THREE.Mesh(ring(-1.135, 0.3, 0.338, 0.07, 64), mCastCase));
    booster.add(new THREE.Mesh(ring(-1.15, 0.075, 0.3, 0.02, 64), mCastCase));
    model.add(
      ctx.part(booster, {
        id: 'booster',
        name: 'Compressore di bassa pressione (booster)',
        base:
          'Poche file di palette subito dietro il fan, sulla parte d\'aria che entra nel nucleo. Danno una prima "strizzata" all\'aria ' +
          'e girano con l\'albero di bassa pressione.',
        expert:
          'Booster assiale a 3 stadi sull\'albero LP: porta il rapporto di compressione del flusso primario a circa 2–3 prima del compressore HP. ' +
          'Nel fan diretto gira a N1, con velocità periferiche basse; nel GTF gira con la turbina LP, circa tre volte più veloce del fan.',
        explode: [-1.0, 0, 0],
      }),
    );
    model.add(
      ctx.part(
        // axial screws through the booster / HPC flange pair, heads on the front face
        screwRing(screwGeo(0.009, 0.01, 0.03), mBolt, 32, (d, i) => {
          const a = ((i + 0.5) / 32) * TAU;
          d.position.set(-0.72, 0.517 * Math.cos(a), 0.517 * Math.sin(a));
          d.rotation.set(a, 0, Math.PI / 2); // local +Y -> -X
        }),
        {
          id: 'boosterBolts',
          name: 'Viti della flangia dei compressori',
          base: 'Il cerchio di viti che unisce la cassa del booster a quella del compressore di alta pressione.',
          expert:
            'Le casse dei moduli si uniscono con flange circolari serrate da decine di bulloni ravvicinati: la giunzione deve restare ' +
            'a tenuta e trasmettere spinta e flessione da un modulo all\'altro.',
          explode: [-1.0, 0, 0],
        },
      ),
    );

    // =============== High-pressure compressor (HP spool) ===============
    const hpc = new THREE.Group();
    const hpcRotor = new THREE.Group();
    hpc.add(hpcRotor);
    hpRotors.push(hpcRotor);
    hpcRotor.add(new THREE.Mesh(skin([[0.222, -0.56], [0.292, 0.37]], 0.014, 64), mDrum)); // drum
    hpcRotor.add(new THREE.Mesh(ring(-0.55, 0.115, 0.215, 0.02, 48), mDrum)); // front web
    hpcRotor.add(new THREE.Mesh(latheX([[0.115, 0.38], [0.27, 0.355], [0.28, 0.375], [0.115, 0.4], [0.115, 0.38]], 48), mDrum)); // rear cone
    for (let i = 0; i < 8; i++) {
      const xr = -0.5 + i * 0.11;
      const xs = xr + 0.055;
      const chord = lerp(0.065, 0.035, i / 7);
      hpcRotor.add(rotorRow(xr, hpcHub(xr), hpcTip(xr) - 0.004, 26 + 3 * i, chord, i < 4 ? mTi : mNi, { twist: 0.45, stagger: 0.65, thick: 0.009 }));
      hpcRotor.add(new THREE.Mesh(latheX([[0.13, xr - 0.01], [hpcHub(xr) - 0.012, xr - 0.016], [hpcHub(xr) - 0.012, xr + 0.016], [0.13, xr + 0.01], [0.13, xr - 0.01]], 48), mDrum));
      hpc.add(statorRow(xs, hpcHub(xs) + 0.004, hpcTip(xs) + 0.006, 30 + 3 * i, chord * 0.8, mStator, 0.5, 0.008));
    }
    // inlet guide vane, transition duct (swan neck), casing, flanges
    hpc.add(statorRow(-0.57, 0.228, 0.402, 30, 0.05, mStator, 0.2, 0.009));
    hpc.add(new THREE.Mesh(wall([[0.485, -0.7], [0.455, -0.66], [0.415, -0.58], [0.399, -0.55], [0.342, 0.37]], 0.014, 72), mCase));
    hpc.add(new THREE.Mesh(wall([[0.31, -0.7], [0.28, -0.66], [0.24, -0.6], [0.226, -0.57]], -0.012, 64), mCase));
    hpc.add(new THREE.Mesh(ring(-0.612, 0.132, 0.25, 0.016, 48), mCastCase)); // #3 bearing support
    hpc.add(new THREE.Mesh(ring(-0.694, 0.487, 0.528, 0.014), mCase)); // front flange, mates with the booster's
    hpc.add(new THREE.Mesh(ring(0.362, 0.35, 0.39, 0.014), mCase));
    hpc.add(bolts([[0.352, 0.375, 40]], mBolt));
    {
      // variable stator vane (VSV) actuation: unison rings, levers, actuator
      for (let i = 0; i < 4; i++) {
        const xs = -0.445 + i * 0.11;
        const rc = hpcTip(xs) + 0.035;
        const ur = new THREE.Mesh(new THREE.TorusGeometry(rc, 0.005, 6, 72).rotateY(Math.PI / 2), mTube);
        ur.position.x = xs - 0.012;
        hpc.add(ur);
        const lev = mergeRing(new THREE.BoxGeometry(0.03, 0.03, 0.006).translate(-0.006, 0.015, 0), mStator, 30, hpcTip(xs) + 0.012);
        lev.position.x = xs;
        hpc.add(lev);
      }
      const act = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.26, 16).rotateZ(Math.PI / 2), mCase);
      onSurface(act, -0.33, 0.43, 225);
      hpc.add(act);
      const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.16, 8).rotateZ(Math.PI / 2), mTube);
      onSurface(rod, -0.52, 0.43, 225);
      hpc.add(rod);
      // bleed air duct from a mid-stage port, running aft along the casing
      const a = 300 * DEG;
      const P = (x, r) => new THREE.Vector3(x, r * Math.cos(a), r * Math.sin(a));
      const duct = new THREE.TubeGeometry(
        new THREE.CatmullRomCurve3([P(-0.12, 0.39), P(-0.05, 0.47), P(0.15, 0.555), P(0.6, 0.565), P(0.92, 0.53), P(0.98, 0.47)]),
        48, 0.016, 10, false,
      );
      hpc.add(new THREE.Mesh(duct, mTube));
      const boss = new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.03, 0.02, 16), mCase);
      onSurface(boss, -0.12, 0.392, 300);
      hpc.add(boss);
    }
    model.add(
      ctx.part(hpc, {
        id: 'hpc',
        name: 'Compressore di alta pressione',
        base:
          'Una lunga serie di file di palette sempre più piccole: ogni fila comprime ancora un po\' l\'aria. Gira molto più veloce del fan, ' +
          'perché è collegato a un albero diverso. Fuori dalla cassa, anelli e leve orientano le prime file di palette fisse.',
        expert:
          'Compressore assiale a 10 stadi (LEAP) / 11 (GE9X) sull\'albero HP, con IGV e primi statori a calettamento variabile (VSV) mossi da ' +
          'anelli di sincronizzazione e attuatori comandati dal FADEC. Rapporto di compressione del solo HPC ≈ 20–27; OPR complessivo 40–60. ' +
          'Dalle porte di spillamento si preleva aria per pressurizzazione cabina, antighiaccio e raffreddamento. Primi stadi in titanio, ' +
          'ultimi in superleghe di nichel (T₃ ≈ 700 °C).',
        explode: [-0.35, 0, 0],
      }),
    );

    // =============== Combustor (static) ===============
    const comb = new THREE.Group();
    comb.add(new THREE.Mesh(wall([[0.342, 0.37], [0.4, 0.4], [0.47, 0.45], [0.49, 0.55], [0.49, 0.74], [0.455, 0.85]], 0.012, 72), mCase));
    comb.add(new THREE.Mesh(wall([[0.27, 0.4], [0.255, 0.5], [0.255, 0.72], [0.285, 0.85]], -0.01, 64), mCase));
    comb.add(
      new THREE.Mesh(
        skin(
          [
            [0.3, 0.84], [0.285, 0.72], [0.278, 0.56], [0.287, 0.46], [0.31, 0.425], [0.355, 0.41],
            [0.4, 0.425], [0.423, 0.46], [0.433, 0.56], [0.428, 0.72], [0.42, 0.84],
          ],
          -0.008,
          72,
        ),
        mCombLiner,
      ),
    );
    comb.add(new THREE.Mesh(ring(0.852, 0.455, 0.5, 0.014), mCase));
    comb.add(bolts([[0.842, 0.485, 44]], mBolt));
    {
      // fuel manifold, 18 injector stems, swirler cups, igniter plugs
      const manifold = new THREE.Mesh(new THREE.TorusGeometry(0.535, 0.011, 6, 72).rotateY(Math.PI / 2), mFuel);
      manifold.position.x = 0.405;
      comb.add(manifold);
      const stem = new THREE.CylinderGeometry(0.008, 0.008, 0.17, 6).translate(0, 0.085, 0);
      const stems = mergeRing(stem, mInjector, 18, 0.36);
      stems.position.x = 0.405;
      comb.add(stems);
      const flangeGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.012, 8).translate(0, 0.006, 0);
      const flanges = mergeRing(flangeGeo, mInjector, 18, 0.497);
      flanges.position.x = 0.405;
      comb.add(flanges);
      const cup = new THREE.CylinderGeometry(0.02, 0.024, 0.03, 10).rotateZ(Math.PI / 2);
      const cups = mergeRing(cup, mInjector, 18, 0.355);
      cups.position.x = 0.418;
      comb.add(cups);
      for (const aDeg of [215, 145]) {
        const ign = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.12, 10).translate(0, 0.06, 0), mCase);
        onSurface(ign, 0.56, 0.44, aDeg);
        comb.add(ign);
      }
    }
    // flame: glowing torus core + additive halo + one jet per injector
    const flamePts = [
      [0.355, 0.44], [0.385, 0.48], [0.405, 0.57], [0.4, 0.68], [0.385, 0.78], [0.36, 0.83],
      [0.335, 0.78], [0.315, 0.68], [0.31, 0.57], [0.325, 0.48], [0.355, 0.44],
    ];
    const flameCore = new THREE.Mesh(latheX(flamePts, 72), cut(MAT.glow(0xff8a2a, 2, 0.85)));
    flameCore.material.side = THREE.DoubleSide;
    const flameHalo = new THREE.Mesh(
      latheX(flamePts.map(([r, x]) => [0.357 + (r - 0.357) * 1.3, x]), 72),
      cut(MAT.gas(0xff5a1f, 0.3)),
    );
    const jetGeo = new THREE.ConeGeometry(0.028, 0.16, 8, 1, true).rotateZ(-Math.PI / 2).translate(0.08, 0, 0);
    const flameJets = mergeRing(jetGeo, cut(MAT.gas(0xff8a3a, 0.4)), 18, 0.355);
    flameJets.position.x = 0.43;
    comb.add(flameCore, flameHalo, flameJets);
    model.add(
      ctx.part(comb, {
        id: 'combustor',
        name: 'Camera di combustione',
        base:
          'Un anello dove il cherosene, spruzzato dagli iniettori, brucia di continuo nell\'aria compressa. La fiamma diventa più ' +
          'intensa quando si dà più manetta. Due candelette la accendono all\'avviamento, poi si spengono.',
        expert:
          'Combustore anulare con ~18–20 iniettori e swirler (TAPS sul LEAP: premiscelazione magra, NOx ridotti), alimentati da un collettore ' +
          'carburante esterno. Combustione a pressione quasi costante, rapporto aria/combustibile globale ≈ 40–50:1 ma ~15:1 in zona primaria; ' +
          'l\'aria di diluizione abbassa la temperatura a T₄ ≈ 1500–1700 °C prima della turbina. Pareti (liner) raffreddate a film.',
        explode: [0.25, 0, 0],
      }),
    );

    // =============== High-pressure turbine (HP spool) ===============
    const hpt = new THREE.Group();
    const hptRotor = new THREE.Group();
    hpt.add(hptRotor);
    hpRotors.push(hptRotor);
    for (const [x, rh, rt, n] of [[0.925, 0.3, 0.425, 44], [1.03, 0.305, 0.435, 46]]) {
      const row = rotorRow(x, rh, rt, n, 0.042, mHot, { twist: -0.25, stagger: -0.75, thick: 0.014 });
      hptRotor.add(row);
      hotMeshes.push(row);
      hptRotor.add(
        new THREE.Mesh(
          latheX([[0.115, x - 0.028], [0.24, x - 0.022], [rh, x - 0.026], [rh, x + 0.026], [0.24, x + 0.022], [0.115, x + 0.028], [0.115, x - 0.028]], 64),
          mDrum,
        ),
      );
    }
    hptRotor.add(new THREE.Mesh(ring(0.978, 0.2, 0.24, 0.08, 48), mDrum)); // inter-disk spacer
    for (const [x, rh, rt] of [[0.875, 0.3, 0.432], [0.978, 0.302, 0.44]]) {
      const ngv = statorRow(x, rh, rt, 32, 0.045, mHotStator, 0.9, 0.014);
      hpt.add(ngv);
      hotMeshes.push(ngv);
    }
    hpt.add(new THREE.Mesh(wall([[0.437, 0.85], [0.447, 1.07]], 0.014, 72), mCase));
    hpt.add(new THREE.Mesh(ring(1.072, 0.447, 0.5, 0.014), mCase));
    hpt.add(bolts([[1.062, 0.485, 44]], mBolt));
    {
      const acc = new THREE.Mesh(new THREE.TorusGeometry(0.472, 0.008, 6, 72).rotateY(Math.PI / 2), mTube);
      acc.position.x = 0.96;
      hpt.add(acc);
    }
    model.add(
      ctx.part(hpt, {
        id: 'hpt',
        name: 'Turbina di alta pressione',
        base:
          'Le prime palette colpite dai gas roventi. Girano velocissime e, tramite un albero, fanno girare il compressore di alta pressione. ' +
          'Lavorano in un calore che fonderebbe quasi ogni metallo.',
        expert:
          'HPT a 2 stadi: distributori (NGV) e palette monocristalline in superlega di nichel con raffreddamento interno, fori a film ' +
          'e barriera termica (TBC). Gas in ingresso a ~1500–1700 °C. Estrae il lavoro necessario a trascinare il compressore HP a N2 ' +
          '(oltre 15 000 giri/min). Un anello di aria di raffreddamento sulla cassa (controllo attivo dei giochi) ne regola il diametro.',
        explode: [0.75, 0, 0],
      }),
    );

    // =============== Low-pressure turbine (LP spool) ===============
    const lpt = new THREE.Group();
    const lptRotor = new THREE.Group();
    lpt.add(lptRotor);
    lpRotors.push(lptRotor);
    lptRotor.add(new THREE.Mesh(skin([[0.31, 1.09], [0.346, 1.67]], 0.01, 64), mDrum));
    for (let i = 0; i < 6; i++) {
      const xr = 1.13 + i * 0.1;
      const xs = xr - 0.045;
      const hub = lptHub(xr);
      lptRotor.add(rotorRow(xr, hub, lptTip(xr) - 0.004, 36 + 3 * i, 0.045, mLpt, { twist: -0.3, stagger: -0.6, thick: 0.011 }));
      lptRotor.add(
        new THREE.Mesh(
          latheX([[0.052, xr - 0.012], [hub - 0.01, xr - 0.02], [hub - 0.01, xr + 0.02], [0.052, xr + 0.012], [0.052, xr - 0.012]], 48),
          mDrum,
        ),
      );
      lpt.add(statorRow(xs, lptHub(xs) + 0.004, lptTip(xs) + 0.006, 34 + 3 * i, 0.04, mStator, 0.7, 0.01));
    }
    lpt.add(new THREE.Mesh(wall([[0.45, 1.07], [0.553, 1.69]], 0.014, 72), mCase));
    lpt.add(new THREE.Mesh(ring(1.692, 0.553, 0.578, 0.012), mCase));
    lpt.add(bolts([[1.682, 0.568, 48]], mBolt));
    {
      // active clearance control manifolds wrapped around the LPT case
      for (const x of [1.25, 1.4, 1.55]) {
        const t = new THREE.Mesh(new THREE.TorusGeometry(lptTip(x) + 0.034, 0.007, 6, 72).rotateY(Math.PI / 2), mTube);
        t.position.x = x;
        lpt.add(t);
      }
      const a = 300 * DEG;
      const feed = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.36, 10).rotateZ(Math.PI / 2), mTube);
      const rf = lptTip(1.4) + 0.05;
      feed.position.set(1.4, rf * Math.cos(a), rf * Math.sin(a));
      lpt.add(feed);
    }
    model.add(
      ctx.part(lpt, {
        id: 'lpt',
        name: 'Turbina di bassa pressione',
        base:
          'Più file di palette, sempre più grandi verso il fondo. Raccolgono l\'energia rimasta nei gas e la usano per far girare il fan ' +
          'e il booster, tramite un albero lungo che passa al centro del motore.',
        expert:
          'LPT a 7 stadi (LEAP-1A) / 6 (GE9X), coassiale all\'albero LP; nel GTF bastano 3 stadi perché gira più veloce. Fornisce la potenza ' +
          'all\'albero del fan, decine di MW al decollo. L\'altezza delle palette cresce perché pressione e densità del gas calano. ' +
          'Pale sottili in superleghe di nichel o alluminuro di titanio (TiAl) sugli ultimi stadi; i collettori attorno alla cassa la ' +
          'raffreddano con aria del fan per stringere i giochi in crociera.',
        explode: [1.25, 0, 0],
      }),
    );

    // =============== Shafts (LP inside HP) + bearings ===============
    const shafts = new THREE.Group();
    const lpShaft = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 3.11, 24).rotateZ(Math.PI / 2), mShaftLP);
    lpShaft.position.x = 0.245;
    const hpShaft = new THREE.Mesh(latheX([[0.095, -0.67], [0.115, -0.67], [0.115, 1.07], [0.095, 1.07], [0.095, -0.67]], 32), mShaftHP);
    shafts.add(lpShaft, hpShaft);
    lpRotors.push(lpShaft);
    hpRotors.push(hpShaft);
    // bearings: #1 (fan), #3 (HP front), #4 (intershaft), #5 (turbine rear frame)
    for (const [x, rIn, rOut] of [[-1.15, 0.05, 0.075], [-0.612, 0.115, 0.132], [1.05, 0.05, 0.095], [1.78, 0.05, 0.075]]) {
      shafts.add(new THREE.Mesh(ring(x, rIn, rOut, 0.026, 40), mBronze));
    }
    model.add(
      ctx.part(shafts, {
        id: 'shafts',
        name: 'Alberi e cuscinetti',
        base:
          'Due alberi uno dentro l\'altro, come due tubi infilati. Quello interno collega la turbina di bassa pressione al fan; ' +
          'quello esterno collega la turbina di alta pressione al compressore. Girano a velocità diverse, sostenuti da cuscinetti.',
        expert:
          'Architettura bialbero: albero LP (N1) interno e albero HP (N2) esterno, sostenuti da cuscinetti a sfere (spinta) e a rulli ' +
          'in "sump" lubrificati e sigillati; tra i due alberi lavora un cuscinetto intershaft. Spool indipendenti: ciascuno trova la velocità ' +
          'ottimale; nel turbofan con riduttore (Pratt & Whitney GTF) un ingranaggio epicicloidale separa ulteriormente fan e turbina LP.',
        explode: [0, 0, 0],
      }),
    );

    // =============== Core cowl / bypass duct inner wall + outlet guide vanes ===============
    const bypass = new THREE.Group();
    bypass.add(
      new THREE.Mesh(
        latheX(
          [
            [0.575, -1.16], [0.62, -1.05], [0.655, -0.8], [0.665, -0.5], [0.645, 0.0], [0.615, 0.5],
            [0.605, 1.0], [0.605, 1.4], [0.6, 1.7], [0.58, 1.7], [0.585, 1.0], [0.59, 0.5],
            [0.62, 0.0], [0.635, -0.5], [0.6, -0.8], [0.55, -1.05], [0.575, -1.16],
          ],
          96,
        ),
        mCowl,
      ),
    );
    bypass.add(statorRow(-0.85, 0.64, 1.04, 36, 0.13, mOGV, 0.25, 0.015));
    bypass.add(new THREE.Mesh(ring(-0.25, 0.645, 0.657, 0.012, 96), mSeal)); // core cowl split line
    {
      const latchGeo = new RoundedBoxGeometry(0.08, 0.01, 0.026, 2, 0.004);
      for (const x of [0.2, 0.6, 1.0]) bypass.add(onSurface(new THREE.Mesh(latchGeo, mDark), x, radiusAt([[0.645, 0], [0.615, 0.5], [0.605, 1.0]], x) + 0.004, 180));
    }
    model.add(
      ctx.part(bypass, {
        id: 'bypass',
        name: 'Condotto secondario (bypass)',
        base:
          'Il corridoio ad anello tra la gondola e il nucleo, dove passa la maggior parte dell\'aria del fan senza bruciare. ' +
          'Le palette fisse raddrizzano l\'aria che esce dal fan, che altrimenti girerebbe a vortice.',
        expert:
          'Flusso secondario con BPR ≈ 10–11:1. Le OGV (outlet guide vanes) eliminano la componente tangenziale (swirl) e recuperano ' +
          'pressione; l\'ugello secondario convergente espande l\'aria a ~300 m/s al decollo. È qui che nasce l\'80–90% della spinta, ' +
          'e il flusso freddo che avvolge il getto caldo riduce anche il rumore. La carenatura del nucleo (core cowl) si apre in due metà.',
        explode: [0, -1.6, 0],
      }),
    );

    // =============== Core exhaust nozzle + plug + turbine rear frame ===============
    const nozzle = new THREE.Group();
    nozzle.add(
      new THREE.Mesh(
        latheX([[0.555, 1.7], [0.6, 1.7], [0.565, 1.95], [0.52, 2.18], [0.5, 2.18], [0.53, 1.95], [0.555, 1.7]], 72),
        mNozzle,
      ),
    );
    nozzle.add(
      new THREE.Mesh(skin([[0.33, 1.69], [0.33, 1.72], [0.3, 1.92], [0.24, 2.15], [0.15, 2.38], [0.06, 2.55], [0, 2.62]], 0.012, 64), mPlug),
    );
    nozzle.add(new THREE.Mesh(latheX([[0.596, 1.705], [0.606, 1.705], [0.6, 1.765], [0.588, 1.765], [0.596, 1.705]], 72), mNozzle)); // attachment flange band
    nozzle.add(statorRow(1.72, 0.33, 0.556, 12, 0.08, mNozzle, 0, 0.02));
    nozzle.add(new THREE.Mesh(latheX([[0.075, 1.765], [0.3, 1.7], [0.33, 1.7], [0.33, 1.73], [0.095, 1.795], [0.075, 1.795], [0.075, 1.765]], 48), mCastCase));
    model.add(
      ctx.part(nozzle, {
        id: 'nozzle',
        name: 'Ugello e cono di scarico',
        base:
          'L\'uscita del getto caldo. Il condotto si stringe un po\' per far uscire i gas più veloci, e il cono centrale li accompagna ' +
          'senza creare vortici.',
        expert:
          'Ugello primario convergente con plug (cono di scarico) e telaio posteriore della turbina (TRF) con bracci che sostengono ' +
          'il cuscinetto posteriore. Il getto primario esce a ~400–500 m/s e ~500–600 °C al decollo; contribuisce solo al 10–20% della spinta.',
        explode: [1.9, 0, 0],
      }),
    );
    model.add(
      ctx.part(
        // radial screws around the nozzle's front ring (none under the pylon)
        screwRing(screwGeo(0.015, 0.012, 0.035), mBolt, 20, (d, i) => onSurface(d, 1.735, 0.603, i * 18 + 9, -0.1).translateY(0.005)),
        {
          id: 'nozzleBolts',
          name: 'Viti dell\'ugello',
          base: 'Una corona di viti che fissa l\'ugello di scarico in coda al motore.',
          expert:
            'L\'ugello primario è fissato alla flangia del telaio posteriore della turbina. In zona calda si usa bulloneria in leghe ' +
            'resistenti al calore, montata con pasta antigrippaggio perché non si blocchi dopo tanti cicli termici.',
          explode: [1.9, 0, 0],
        },
      ),
    );

    // =============== Accessory gearbox on the fan case + equipment ===============
    const gearbox = new THREE.Group();
    {
      // crescent-shaped AGB hugging the bottom of the fan case
      const R0 = 1.088;
      const R1 = 1.185;
      const th0 = -118 * DEG;
      const th1 = -62 * DEG;
      const sh = new THREE.Shape();
      const N = 16;
      for (let i = 0; i <= N; i++) {
        const t = lerp(th0, th1, i / N);
        const fn = i === 0 ? 'moveTo' : 'lineTo';
        sh[fn](R1 * Math.cos(t), R1 * Math.sin(t));
      }
      for (let i = N; i >= 0; i--) {
        const t = lerp(th0, th1, i / N);
        sh.lineTo(R0 * Math.cos(t), R0 * Math.sin(t));
      }
      sh.closePath();
      const g = new THREE.ExtrudeGeometry(sh, { depth: 0.3, bevelEnabled: true, bevelThickness: 0.01, bevelSize: 0.008, bevelSegments: 2, curveSegments: 4 });
      g.rotateY(Math.PI / 2);
      g.translate(-1.16, 0, 0);
      gearbox.add(new THREE.Mesh(g, mGearbox));
      // accessories on the aft face: [angle θ, radius, length, label]
      const P = (th, r) => [r * Math.sin(th * DEG), -r * Math.cos(th * DEG)];
      const accs = [
        [-116, 1.135, 0.11, 0.032, mAccessory], // hydraulic pump
        [-103, 1.14, 0.18, 0.05, mAccessory], // generator (IDG/VFG)
        [-90, 1.135, 0.14, 0.04, mAccDark], // air turbine starter
        [-78, 1.145, 0.13, 0.04, mAccessory], // fuel pump + FMU
        [-66, 1.135, 0.1, 0.03, mAccDark], // lube & scavenge pump
      ];
      const padRows = [];
      for (const [th, r, len, rad, mat] of accs) {
        const [y, z] = P(th, r);
        const body = new THREE.Mesh(new THREE.CylinderGeometry(rad, rad, len, 20).rotateZ(Math.PI / 2), mat);
        body.position.set(-0.85 + 0.012 + len / 2, y, z);
        gearbox.add(body);
        const pad = new THREE.Mesh(new THREE.CylinderGeometry(rad + 0.012, rad + 0.012, 0.012, 20).rotateZ(Math.PI / 2), mGearbox);
        pad.position.set(-0.85 + 0.006, y, z);
        gearbox.add(pad);
        const cap = new THREE.Mesh(new THREE.SphereGeometry(rad * 0.85, 14, 8, 0, TAU, 0, Math.PI / 2).rotateZ(-Math.PI / 2), mat);
        cap.position.set(-0.85 + 0.012 + len, y, z);
        gearbox.add(cap);
        padRows.push([y, z, rad + 0.006]);
      }
      // bolts around each mounting pad
      const im = new THREE.InstancedMesh(boltGeo, mBolt, padRows.length * 6);
      const d = new THREE.Object3D();
      let k = 0;
      for (const [y, z, rb] of padRows) {
        for (let i = 0; i < 6; i++) {
          const a = (i / 6) * TAU;
          d.position.set(-0.835, y + rb * Math.cos(a), z + rb * Math.sin(a));
          d.rotation.set(a, 0, 0);
          d.updateMatrix();
          im.setMatrixAt(k++, d.matrix);
        }
      }
      gearbox.add(im);
      // radial drive shaft from the HP spool through a fairing in the bypass duct, transfer gearbox
      const a = new THREE.Vector3(-0.65, -0.16, 0);
      const b = new THREE.Vector3(-0.87, -1.06, 0);
      const dir = b.clone().sub(a);
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, dir.length(), 10), mShaftHP);
      shaft.position.copy(a).add(b).multiplyScalar(0.5);
      shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
      gearbox.add(shaft);
      const fair = new THREE.Mesh(new RoundedBoxGeometry(0.12, 0.42, 0.045, 2, 0.02), mOGV);
      fair.position.copy(a.clone().lerp(b, 0.79));
      fair.quaternion.copy(shaft.quaternion);
      gearbox.add(fair);
      const tgb = new THREE.Mesh(new RoundedBoxGeometry(0.09, 0.05, 0.08, 2, 0.012), mGearbox);
      tgb.position.set(-0.86, -1.075, 0);
      gearbox.add(tgb);
      const bevel = new THREE.Mesh(latheX([[0.1, -0.665], [0.16, -0.665], [0.13, -0.64], [0.1, -0.64], [0.1, -0.665]], 40), mShaftHP);
      gearbox.add(bevel);
      hpRotors.push(bevel);
      // oil tank with sight glass and filler cap, FADEC box, wiring harness
      const tank = onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.3, 0.07, 0.16, 3, 0.025), mAccessory), -0.92, 1.135, 216);
      gearbox.add(tank);
      const glass = onSurface(new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.012, 16), MAT.paint(0x9c7a2a, 0.15)), -0.86, 1.172, 216);
      glass.translateZ(0.04);
      gearbox.add(glass);
      const filler = onSurface(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.02, 12), mAccDark), -1.02, 1.178, 216);
      gearbox.add(filler);
      for (const x of [-1.03, -0.81]) gearbox.add(onSurface(new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.1), mGearbox), x, 1.09, 216));
      const fadec = onSurface(new THREE.Mesh(new RoundedBoxGeometry(0.18, 0.055, 0.12, 2, 0.012), mAccDark), -1.36, 1.16, 250);
      gearbox.add(fadec);
      for (const dz of [-0.03, 0.03]) {
        const con = onSurface(new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.03, 12), mAccessory), -1.3, 1.195, 250);
        con.translateZ(dz);
        gearbox.add(con);
      }
      const hp = [];
      for (let i = 0; i <= 16; i++) {
        const aa = lerp(212, 336, i / 16) * DEG;
        hp.push(new THREE.Vector3(-0.7, 1.075 * Math.cos(aa), 1.075 * Math.sin(aa)));
      }
      gearbox.add(new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(hp), 48, 0.009, 6, false), MAT.paint(0x3b3428, 0.7)));
      for (const aa of [240, 280, 320]) gearbox.add(onSurface(new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.024, 0.03), mGearbox), -0.7, 1.072, aa));
    }
    model.add(
      ctx.part(gearbox, {
        id: 'gearbox',
        name: 'Scatola accessori e equipaggiamenti',
        base:
          'Una scatola di ingranaggi a mezzaluna sotto la cassa del fan: preleva un po\' di rotazione dal motore per azionare le pompe del ' +
          'carburante e dell\'olio, il generatore elettrico e la pompa idraulica. Lì vicino ci sono il serbatoio dell\'olio e il "cervello" ' +
          'elettronico del motore.',
        expert:
          'AGB (accessory gearbox) montata sulla cassa del fan, trascinata dall\'albero HP tramite coppia conica, albero radiale (in una carenatura ' +
          'attraverso il condotto di bypass) e scatola di rinvio. Aziona pompa carburante e FMU, pompe olio, generatore (IDG/VFG) e pompa ' +
          'idraulica; all\'avviamento lo starter pneumatico fa il percorso inverso e mette in rotazione N2. La centralina FADEC (a doppio canale) ' +
          'controlla carburante, VSV, valvole di spillamento e giochi in turbina.',
        explode: [0, -2.5, 0],
      }),
    );

    // =============== Flows (lanes in the visible z <= 0 half) ===============
    const flows = [];
    const addLanes = (path, angles, opts) => {
      for (const deg of angles) {
        const a = deg * DEG;
        const f = flow({ ...opts, path: (t, out) => path.at(t, a, out) });
        root.add(f.object);
        ctx.addFlow(f);
        flows.push({ f, kind: opts.kind });
      }
    };

    const bypassPath = meridian([
      [-3.0, 0.78], [-2.2, 0.8], [-1.75, 0.8], [-1.45, 0.8], [-1.1, 0.83], [-0.85, 0.85], [-0.3, 0.84],
      [0.3, 0.8], [0.85, 0.76], [1.5, 0.72], [2.3, 0.7], [3.0, 0.7],
    ]);
    // lanes avoid the pylon (top) and the radial drive fairing (bottom)
    addLanes(bypassPath, Array.from({ length: 7 }, (_, i) => 192 + i * 26), {
      kind: 'air',
      count: 120,
      size: 0.05,
      speed: 0.2,
      colors: [0x5fd3ff, 0x5fd3ff, 0x8fe4ff, 0x5fc4ee],
      spread: (t) => {
        const x = bypassPath.x(t);
        return x < -1.75 ? 0.2 : x < 0.85 ? 0.11 : 0.14;
      },
      opacity: 0.8,
    });

    const corePath = meridian([
      [-3.0, 0.3], [-2.2, 0.33], [-1.75, 0.38], [-1.45, 0.42], [-1.15, 0.43], [-0.93, 0.41], [-0.72, 0.39],
      [-0.62, 0.34], [-0.52, 0.31], [-0.1, 0.305], [0.33, 0.312], [0.45, 0.355], [0.62, 0.357], [0.8, 0.36],
      [0.95, 0.365], [1.3, 0.395], [1.65, 0.435], [1.9, 0.42], [2.2, 0.39], [2.7, 0.34], [3.3, 0.3],
    ]);
    const coreColorAt = gradientByX([
      [-1.2, 0x5fd3ff], [-0.6, 0x8fdcd8], [0.0, 0xc9d890], [0.36, 0xf2c26a], [0.5, 0xff5a1f],
      [0.85, 0xff6a28], [1.2, 0xe0703a], [1.7, 0xa8788a], [2.1, 0x8c86a8],
    ]);
    addLanes(corePath, [180, 210, 240, 270, 300, 330, 360], {
      kind: 'air',
      count: 105,
      size: 0.05,
      speed: 0.19,
      colors: Array.from({ length: 33 }, (_, i) => coreColorAt(corePath.x(i / 32))),
      spread: (t) => {
        const x = corePath.x(t);
        if (x < -1.8) return 0.12;
        if (x < -0.6) return 0.06;
        if (x < 1.0) return 0.035;
        if (x < 1.75) return 0.055;
        return 0.06 + (x - 1.75) * 0.12;
      },
      opacity: 0.9,
    });

    const fuelPath = meridian([
      [0.405, 0.55], [0.405, 0.47], [0.405, 0.39], [0.42, 0.358], [0.47, 0.355], [0.56, 0.356], [0.64, 0.36],
    ], 80);
    addLanes(fuelPath, [180, 220, 260, 300, 340, 360], {
      kind: 'fuel',
      count: 24,
      size: 0.03,
      speed: 1.1,
      colors: [0xffc04d, 0xffc04d, 0xffb040, 0xff7a2a],
      spread: (t) => (t < 0.55 ? 0.005 : 0.01 + (t - 0.55) * 0.08),
      opacity: 0.9,
    });

    // ---------- State ----------
    let spool = null; // lagging throttle (spool-up inertia)
    let fanAngle = 0;
    let lpFast = 0; // LP shaft angle in GTF mode (sun gear)
    let hpAngle = 0;
    let gtf = false;
    const RATIO = Z_RING / Z_SUN; // star gear: ring (fan) turns opposite to the sun, RATIO times slower

    const setOption = (id, value) => {
      if (id !== 'fan') return;
      gtf = value === 'gtf';
      wReduction.visible = gtf;
    };
    setOption('fan', 'direct');

    return {
      root,
      setOption,
      cycleRate(throttle) {
        return 0.12 + 0.25 * throttle;
      },
      update(state) {
        const dt = state.dt || 0;
        const thr = clamp(state.throttle ?? 0, 0, 1);
        if (spool === null || dt === 0) spool = thr;
        else spool += (thr - spool) * (1 - Math.exp(-dt / 1.4));
        const n1 = 0.2 + 0.8 * spool;
        const n2 = 0.6 + 0.4 * spool;

        // spools: fan (N1), LP shaft (= fan, or RATIO x faster and reversed with GTF), HP (N2)
        fanAngle -= dt * TAU * 1.15 * n1;
        lpFast += dt * TAU * 1.15 * n1 * RATIO;
        hpAngle -= dt * TAU * 2.6 * n2;
        const lpAngle = gtf ? lpFast : fanAngle;
        for (const o of fanRotors) o.rotation.x = fanAngle;
        for (const o of lpRotors) o.rotation.x = lpAngle;
        for (const o of hpRotors) o.rotation.x = hpAngle;
        ringRot.rotation.x = -lpFast / RATIO;
        const planetAngle = (-lpFast * Z_SUN) / Z_PLANET;
        for (const p of planets) p.rotation.x = planetAngle;

        // flame and hot section follow fuel flow
        const time = state.time || 0;
        const flick = 0.9 + 0.1 * Math.sin(time * 31) * Math.sin(time * 17.3);
        flameCore.material.emissiveIntensity = (0.4 + 1.6 * spool) * flick;
        flameCore.material.opacity = 0.5 + 0.4 * spool;
        flameHalo.material.opacity = (0.1 + 0.35 * spool) * flick;
        flameJets.material.opacity = (0.12 + 0.3 * spool) * flick;
        flameJets.scale.set(0.5 + 0.8 * spool, 1, 1);
        for (const m of hotMeshes) {
          if (m.material.emissive) m.material.emissiveIntensity = 0.05 + 0.45 * spool * spool;
        }

        // particle rates
        const airRate = 0.25 + 1.0 * spool;
        const fuelRate = 0.3 + 0.9 * spool;
        const assembled = (state.explode || 0) < 0.05;
        flameCore.visible = flameHalo.visible = flameJets.visible = assembled;
        for (const { f, kind } of flows) {
          f.object.visible = assembled;
          if (kind === 'fuel') {
            f.object.material.opacity = 0.3 + 0.6 * spool;
            f.update(dt, fuelRate);
          } else {
            f.update(dt, airRate);
          }
        }
      },
      readout() {
        const s = spool ?? 0;
        const N1 = 0.2 + 0.8 * s;
        const N2 = 0.6 + 0.4 * s;
        return [
          { label: gtf ? 'N1 (albero LP)' : 'N1 (fan)', value: (N1 * 100).toFixed(0), unit: '%' },
          { label: 'N2 (alta pressione)', value: (N2 * 100).toFixed(0), unit: '%' },
          { label: 'Spinta', value: (130 * N1 * N1).toFixed(0), unit: 'kN' },
          { label: 'EGT', value: String(Math.round(470 + 430 * Math.pow(s, 1.2))), unit: '°C' },
        ];
      },
    };
  },
};
