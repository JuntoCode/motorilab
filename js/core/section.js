// Section caps: fills the faces cut by the cutaway plane with a hatched "technical drawing" surface.
// Classic stencil technique: for every closed cut mesh, back faces increment and front faces decrement
// the stencil; where the count is non-zero the plane lies inside solid material, so a cap is drawn.
// Open meshes (single surfaces, tubes) are skipped: they stay thin shells.
import * as THREE from 'three';

// True when every edge is shared by an even number of triangles (welded by position).
export function isClosed(geometry) {
  if (geometry.userData.closed !== undefined) return geometry.userData.closed;
  const pos = geometry.attributes.position;
  const index = geometry.index;
  const ids = new Map();
  const canon = new Int32Array(pos.count);
  for (let i = 0; i < pos.count; i++) {
    const k = `${Math.round(pos.getX(i) * 1e4)},${Math.round(pos.getY(i) * 1e4)},${Math.round(pos.getZ(i) * 1e4)}`;
    let id = ids.get(k);
    if (id === undefined) ids.set(k, (id = ids.size));
    canon[i] = id;
  }
  const edges = new Map();
  const tri = index ? index.count / 3 : pos.count / 3;
  const v = (n) => canon[index ? index.getX(n) : n];
  const add = (a, b) => {
    const k = a < b ? a * 4194304 + b : b * 4194304 + a;
    edges.set(k, (edges.get(k) || 0) + 1);
  };
  for (let t = 0; t < tri; t++) {
    const a = v(t * 3), b = v(t * 3 + 1), c = v(t * 3 + 2);
    if (a === b || b === c || a === c) continue; // degenerate (poles, collapsed seams)
    add(a, b);
    add(b, c);
    add(c, a);
  }
  let closed = edges.size > 0;
  for (const n of edges.values()) if (n % 2) { closed = false; break; }
  geometry.userData.closed = closed;
  return closed;
}

function hatchTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#c4582c';
  g.fillRect(0, 0, 64, 64);
  g.strokeStyle = '#7d2c12';
  g.lineWidth = 7;
  for (let i = -64; i <= 128; i += 32) {
    g.beginPath();
    g.moveTo(i, 64);
    g.lineTo(i + 64, 0);
    g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

const noRaycast = () => {};

export class SectionCaps {
  constructor(scene, clipPlanes) {
    this.clipPlanes = clipPlanes;
    const stencilBase = {
      depthWrite: false,
      depthTest: false,
      colorWrite: false,
      stencilWrite: true,
      stencilFunc: THREE.AlwaysStencilFunc,
      clippingPlanes: clipPlanes,
    };
    this.backMat = new THREE.MeshBasicMaterial({
      ...stencilBase,
      side: THREE.BackSide,
      stencilFail: THREE.IncrementWrapStencilOp,
      stencilZFail: THREE.IncrementWrapStencilOp,
      stencilZPass: THREE.IncrementWrapStencilOp,
    });
    this.frontMat = new THREE.MeshBasicMaterial({
      ...stencilBase,
      side: THREE.FrontSide,
      stencilFail: THREE.DecrementWrapStencilOp,
      stencilZFail: THREE.DecrementWrapStencilOp,
      stencilZPass: THREE.DecrementWrapStencilOp,
    });
    const tex = hatchTexture();
    tex.repeat.set(220, 220);
    this.cap = new THREE.Mesh(
      new THREE.PlaneGeometry(40, 40),
      new THREE.MeshStandardMaterial({
        map: tex,
        metalness: 0.35,
        roughness: 0.55,
        stencilWrite: true,
        stencilRef: 0,
        stencilFunc: THREE.NotEqualStencilFunc,
        stencilFail: THREE.ReplaceStencilOp,
        stencilZFail: THREE.ReplaceStencilOp,
        stencilZPass: THREE.ReplaceStencilOp,
        polygonOffset: true,
        polygonOffsetFactor: -1,
      })
    );
    this.cap.renderOrder = 11;
    this.cap.raycast = noRaycast;
    this.cap.visible = false;
    scene.add(this.cap);
    this.helpers = [];
  }

  // Adds stencil helpers to every closed mesh whose material is a cut shell.
  attach(root) {
    this.helpers = [];
    const targets = [];
    root.traverse((o) => {
      if (!o.isMesh || o.isInstancedMesh || o.userData.sectionHelper) return;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      const capped = mats.some((m) => m?.userData.cut) && mats.every((m) => m && !m.transparent && m.userData.cap !== false);
      if (capped && isClosed(o.geometry)) targets.push(o);
    });
    for (const o of targets) {
      for (const [mat, order] of [[this.backMat, 10], [this.frontMat, 10]]) {
        const h = new THREE.Mesh(o.geometry, mat);
        h.renderOrder = order;
        h.raycast = noRaycast;
        h.userData.sectionHelper = true;
        h.castShadow = h.receiveShadow = false;
        o.add(h);
        this.helpers.push(h);
      }
    }
    return targets.length;
  }

  update(constant) {
    const on = constant < 50;
    this.cap.visible = on && this.helpers.length > 0;
    for (const h of this.helpers) h.visible = on;
    this.cap.position.z = constant;
  }

  dispose() {
    this.cap.geometry.dispose();
    this.cap.material.map.dispose();
    this.cap.material.dispose();
    this.backMat.dispose();
    this.frontMat.dispose();
  }
}
