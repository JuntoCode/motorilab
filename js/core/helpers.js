// Shared building blocks for engine modules: materials, parts, flows.
// See js/engines/CONTRACT.md for how an engine module uses them.
import * as THREE from 'three';

// ---------- Materials ----------
const std = (color, metalness, roughness, extra = {}) =>
  new THREE.MeshStandardMaterial({ color, metalness, roughness, ...extra });

export const MAT = {
  steel: () => std(0xb9c0c8, 0.9, 0.3),
  darkSteel: () => std(0x5d636b, 0.85, 0.4),
  aluminum: () => std(0xd5d9de, 0.75, 0.42),
  castIron: () => std(0x4a4f56, 0.55, 0.75),
  titanium: () => std(0x9a9a92, 0.85, 0.35),
  copper: () => std(0xc8733a, 1.0, 0.33),
  brass: () => std(0xc9a34a, 1.0, 0.3),
  chrome: () => std(0xeef2f6, 1.0, 0.12),
  ceramic: () => std(0xf1efe8, 0.0, 0.55),
  rubber: () => std(0x1d1f22, 0.0, 0.9),
  carbon: () => std(0x222428, 0.4, 0.5),
  paint: (color, roughness = 0.45) => std(color, 0.25, roughness),
  // Self-lit material for flames, sparks, hot zones.
  glow: (color, intensity = 2, opacity = 1) =>
    new THREE.MeshStandardMaterial({
      color: 0x000000,
      emissive: color,
      emissiveIntensity: intensity,
      transparent: opacity < 1,
      opacity,
      depthWrite: opacity >= 1,
    }),
  // Translucent volume (gas charge, plume core).
  // additive = true glows (plumes, flames); false keeps the colour readable (gas charges).
  gas: (color, opacity = 0.35, additive = true) =>
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity,
      depthWrite: false,
      blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending,
      side: THREE.DoubleSide,
    }),
};

// Marks a material as a "shell": the cutaway plane slices it open.
// Closed meshes get a hatched section cap on the cut face; pass { cap: false } for hollow volumes
// modelled as closed solids (tanks full of propellant, gas volumes) that should stay open.
export function cut(material, { cap = true } = {}) {
  material.userData.cut = true;
  material.userData.cap = cap;
  material.side = THREE.DoubleSide;
  return material;
}

// ---------- Geometry helpers ----------
// Lathe profile from [[radius, y], ...] points, revolved around Y.
export function lathe(points, segments = 64) {
  return new THREE.LatheGeometry(points.map(([r, y]) => new THREE.Vector2(r, y)), segments);
}

// Hollow tube (pipe) between two radii along Y, centered at origin.
export function pipe(rOuter, rInner, height, segments = 48) {
  const h = height / 2;
  return lathe([[rInner, -h], [rOuter, -h], [rOuter, h], [rInner, h], [rInner, -h]], segments);
}

// Blade/vane shape: thin twisted box. Good enough for fans and turbines.
export function blade(length, chord, thickness, twist = 0.6) {
  const g = new THREE.BoxGeometry(chord, length, thickness, 1, 6, 1);
  const pos = g.attributes.position;
  const v = new THREE.Vector3();
  for (let i = 0; i < pos.count; i++) {
    v.fromBufferAttribute(pos, i);
    const k = (v.y / length + 0.5) * twist; // twist grows toward tip
    const x = v.x * Math.cos(k) - v.z * Math.sin(k);
    const z = v.x * Math.sin(k) + v.z * Math.cos(k);
    pos.setXYZ(i, x, v.y + length / 2, z);
  }
  g.computeVertexNormals();
  return g;
}

// Ring of N copies of a mesh around the X axis (for fans/turbines facing X).
export function ringX(geometry, material, count, radiusOffset = 0) {
  const group = new THREE.Group();
  for (let i = 0; i < count; i++) {
    const m = new THREE.Mesh(geometry, material);
    const a = (i / count) * Math.PI * 2;
    m.rotation.x = a;
    m.position.set(0, Math.cos(a) * radiusOffset, Math.sin(a) * radiusOffset);
    group.add(m);
  }
  return group;
}

// ---------- Flow particles ----------
let dotTexture;
function getDot() {
  if (dotTexture) return dotTexture;
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grd.addColorStop(0, 'rgba(255,255,255,1)');
  grd.addColorStop(0.4, 'rgba(255,255,255,0.6)');
  grd.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grd;
  g.fillRect(0, 0, 64, 64);
  dotTexture = new THREE.CanvasTexture(c);
  return dotTexture;
}

/**
 * Particles travelling along a path. Use for air, fuel, exhaust, electric current.
 * opts.path(t, outVec3)       position at t in [0,1] (required)
 * opts.count, opts.size
 * opts.colors [c0, c1, ...]   gradient along t (hex numbers)
 * opts.spread(t) -> radius    random offset radius at t (number or function)
 * opts.speed                  path fraction per second at rate 1
 * opts.opacity
 * Returns { object, update(dt, rate) }. rate 0 freezes the flow.
 */
export function flow(opts) {
  const {
    path,
    count = 200,
    size = 0.06,
    colors = [0x66ccff, 0xff6633],
    spread = 0.1,
    speed = 0.3,
    opacity = 0.9,
  } = opts;
  const geo = new THREE.BufferGeometry();
  const positions = new Float32Array(count * 3);
  const cols = new Float32Array(count * 3);
  geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(cols, 3));
  const mat = new THREE.PointsMaterial({
    size,
    map: getDot(),
    vertexColors: true,
    transparent: true,
    opacity,
    depthWrite: false,
    blending: THREE.AdditiveBlending,
    sizeAttenuation: true,
  });
  const points = new THREE.Points(geo, mat);
  points.frustumCulled = false;

  const ts = new Float32Array(count);
  const offs = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    ts[i] = Math.random();
    // random direction inside unit sphere
    let x, y, z;
    do {
      x = Math.random() * 2 - 1;
      y = Math.random() * 2 - 1;
      z = Math.random() * 2 - 1;
    } while (x * x + y * y + z * z > 1);
    offs[i * 3] = x;
    offs[i * 3 + 1] = y;
    offs[i * 3 + 2] = z;
  }
  const stops = colors.map((c) => new THREE.Color(c));
  const tmp = new THREE.Vector3();
  const col = new THREE.Color();
  const spreadAt = typeof spread === 'function' ? spread : () => spread;

  function colorAt(t) {
    if (stops.length === 1) return col.copy(stops[0]);
    const f = t * (stops.length - 1);
    const i = Math.min(Math.floor(f), stops.length - 2);
    return col.copy(stops[i]).lerp(stops[i + 1], f - i);
  }

  function update(dt, rate = 1) {
    for (let i = 0; i < count; i++) {
      ts[i] = (ts[i] + dt * speed * rate) % 1;
      const t = ts[i];
      path(t, tmp);
      const r = spreadAt(t);
      positions[i * 3] = tmp.x + offs[i * 3] * r;
      positions[i * 3 + 1] = tmp.y + offs[i * 3 + 1] * r;
      positions[i * 3 + 2] = tmp.z + offs[i * 3 + 2] * r;
      colorAt(t);
      cols[i * 3] = col.r;
      cols[i * 3 + 1] = col.g;
      cols[i * 3 + 2] = col.b;
    }
    geo.attributes.position.needsUpdate = true;
    geo.attributes.color.needsUpdate = true;
  }
  update(0, 0);
  return { object: points, update, material: mat };
}

// ---------- Math ----------
export const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
export const lerp = (a, b, t) => a + (b - a) * t;
export const smooth = (t) => t * t * (3 - 2 * t);
// 0..1 window: 1 inside [a,b] with soft edges of width w.
export const windowFn = (x, a, b, w = 0.02) =>
  clamp((x - a) / w, 0, 1) * clamp((b - x) / w, 0, 1);
