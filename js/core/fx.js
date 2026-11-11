// Rendering extras shared by every engine: studio lighting environment, backdrop, and the post-processing
// chain (ambient occlusion that respects the cutaway, outline of picked parts, bloom on flames and sparks).
import * as THREE from 'three';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';
import { GTAOPass } from 'three/addons/postprocessing/GTAOPass.js';
import { Pass, FullScreenQuad } from 'three/addons/postprocessing/Pass.js';

// ---------- Studio environment: a dark room with a few softboxes, for crisp reflections on metal ----------
export function studioEnvironment(renderer) {
  const s = new THREE.Scene();
  const add = (w, h, pos, rot, rgb) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ color: new THREE.Color(...rgb), side: THREE.DoubleSide }));
    m.position.set(...pos);
    m.rotation.set(...rot);
    s.add(m);
  };
  s.add(new THREE.Mesh(new THREE.BoxGeometry(24, 14, 24), new THREE.MeshBasicMaterial({ color: 0x10141c, side: THREE.BackSide })));
  add(9, 7, [0, 6.9, 1], [Math.PI / 2, 0, 0], [7, 7.2, 7.6]); // big top softbox
  add(3, 9, [-11.5, 1, 3], [0, Math.PI / 2, 0], [5, 6.5, 9]); // cool strip, left
  add(2.2, 8, [11.5, 1.5, -2], [0, -Math.PI / 2, 0], [10, 7, 4.5]); // warm strip, right
  add(10, 1.2, [0, 3.5, -11.5], [0, 0, 0], [5, 5.5, 6.5]); // rim bar, back
  add(6, 3, [3, 2, 11.5], [0, Math.PI, 0], [2.2, 2.2, 2.4]); // soft fill, front
  add(24, 24, [0, -6.9, 0], [-Math.PI / 2, 0, 0], [0.22, 0.24, 0.3]); // floor bounce
  const pmrem = new THREE.PMREMGenerator(renderer);
  const tex = pmrem.fromScene(s, 0.035).texture;
  pmrem.dispose();
  s.traverse((o) => (o.geometry?.dispose(), o.material?.dispose()));
  return tex;
}

// Screen-fixed backdrop: soft spotlight on graphite, darker toward the corners.
export function backdropTexture() {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 512;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(256, 215, 0, 256, 256, 400);
  grd.addColorStop(0, '#2b3547');
  grd.addColorStop(0.45, '#181f2b');
  grd.addColorStop(1, '#090b10');
  g.fillStyle = grd;
  g.fillRect(0, 0, 512, 512);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// Workbench floor: faint blueprint rings and ticks under the model.
export function floorTexture() {
  const S = 1024;
  const c = document.createElement('canvas');
  c.width = c.height = S;
  const g = c.getContext('2d');
  const h = S / 2;
  const glow = g.createRadialGradient(h, h, 0, h, h, h);
  glow.addColorStop(0, 'rgba(120,170,220,0.13)');
  glow.addColorStop(0.55, 'rgba(120,170,220,0.05)');
  glow.addColorStop(1, 'rgba(120,170,220,0)');
  g.fillStyle = glow;
  g.fillRect(0, 0, S, S);
  g.strokeStyle = 'rgba(160,200,240,0.085)';
  for (const [r, w] of [[0.3, 1.5], [0.52, 3], [0.74, 1.5]]) {
    g.lineWidth = w;
    g.beginPath();
    g.arc(h, h, h * r, 0, Math.PI * 2);
    g.stroke();
  }
  g.lineWidth = 2;
  for (let i = 0; i < 72; i++) {
    const a = (i / 72) * Math.PI * 2;
    const r0 = h * (i % 6 ? 0.535 : 0.52);
    const r1 = h * (i % 6 ? 0.555 : 0.58);
    g.beginPath();
    g.moveTo(h + Math.cos(a) * r0, h + Math.sin(a) * r0);
    g.lineTo(h + Math.cos(a) * r1, h + Math.sin(a) * r1);
    g.stroke();
  }
  // Fade everything out toward the rim.
  g.globalCompositeOperation = 'destination-in';
  const fade = g.createRadialGradient(h, h, h * 0.35, h, h, h);
  fade.addColorStop(0, 'rgba(0,0,0,1)');
  fade.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = fade;
  g.fillRect(0, 0, S, S);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

// ---------- Depth + normal pre-pass that honours the cutaway (clipped shells, hatched caps) ----------
class GBufferPass extends Pass {
  constructor(fx) {
    super();
    this.fx = fx;
    this.needsSwap = false;
    // Renders into the AO pass's own normal + depth(stencil) target, in place of its unclipped pre-pass.
    this.target = fx.ao.normalRenderTarget;
    fx.ao._renderGBuffer = false;
    const clip = fx.viewer.clipPlanes;
    this.plain = new THREE.MeshNormalMaterial({ side: THREE.DoubleSide, blending: THREE.NoBlending });
    this.clipped = new THREE.MeshNormalMaterial({ side: THREE.DoubleSide, blending: THREE.NoBlending, clippingPlanes: clip });
    const cap = fx.viewer.section.cap.material;
    this.capMat = new THREE.MeshNormalMaterial({
      blending: THREE.NoBlending,
      stencilWrite: true,
      stencilRef: 0,
      stencilFunc: cap.stencilFunc,
      stencilFail: cap.stencilFail,
      stencilZFail: cap.stencilZFail,
      stencilZPass: cap.stencilZPass,
      polygonOffset: true,
      polygonOffsetFactor: -1,
    });
    this.clear = new THREE.Color(0x7f7fff);
    this._old = new THREE.Color();
  }
  setSize() {}
  render(renderer) {
    const { fx } = this;
    const { scene, camera } = fx;
    const swapped = [];
    const hidden = [];
    for (const o of fx.extras) if (o.visible) (o.visible = false), hidden.push(o);
    for (const t of fx.solids) {
      const m = t.mesh.material;
      const first = Array.isArray(m) ? m[0] : m;
      if (first.transparent || first.opacity < 1) {
        if (t.mesh.visible) (t.mesh.visible = false), hidden.push(t.mesh);
        continue;
      }
      swapped.push(t.mesh, m);
      t.mesh.material = first.userData.cut ? this.clipped : this.plain;
    }
    for (const o of fx.ghosts) if (o.visible) (o.visible = false), hidden.push(o);
    const cap = fx.viewer.section.cap;
    const capMat = cap.material;
    cap.material = this.capMat;
    const bg = scene.background;
    scene.background = null;
    renderer.getClearColor(this._old);
    const oldAlpha = renderer.getClearAlpha();
    const oldAuto = renderer.autoClear;
    renderer.autoClear = true;
    renderer.setClearColor(this.clear, 1);
    renderer.setRenderTarget(this.target);
    renderer.render(scene, camera);
    renderer.setClearColor(this._old, oldAlpha);
    renderer.autoClear = oldAuto;
    scene.background = bg;
    cap.material = capMat;
    for (let i = 0; i < swapped.length; i += 2) swapped[i].material = swapped[i + 1];
    for (const o of hidden) o.visible = true;
  }
  dispose() {
    this.plain.dispose();
    this.clipped.dispose();
    this.capMat.dispose();
  }
}

// ---------- Outline of hovered / selected / suggested parts (drawn through other parts, like an x-ray) ----------
const MASK_LAYER = 7;
const OutlineShader = {
  uniforms: {
    tDiffuse: { value: null },
    tMask: { value: null },
    texel: { value: new THREE.Vector2() },
    cSelect: { value: new THREE.Color(0x38d6ff) },
    cHint: { value: new THREE.Color(0xff8a1f) },
    cHover: { value: new THREE.Color(0xbfefff) },
    pulse: { value: 1 },
    width: { value: 2 },
  },
  vertexShader: `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
  fragmentShader: `
    uniform sampler2D tDiffuse; uniform sampler2D tMask; uniform vec2 texel;
    uniform vec3 cSelect; uniform vec3 cHint; uniform vec3 cHover; uniform float pulse; uniform float width;
    varying vec2 vUv;
    void main(){
      vec4 base = texture2D(tDiffuse, vUv);
      vec3 m = texture2D(tMask, vUv).rgb;
      vec3 near = vec3(0.0); vec3 far = vec3(0.0);
      for (int i = 0; i < 8; i++) {
        float a = float(i) * 0.7853982;
        vec2 d = vec2(cos(a), sin(a)) * texel;
        near = max(near, texture2D(tMask, vUv + d * width).rgb);
        far = max(far, texture2D(tMask, vUv + d * width * 2.6).rgb);
      }
      vec3 line = clamp(near - m, 0.0, 1.0);
      vec3 halo = clamp(far - m, 0.0, 1.0) * 0.35;
      vec3 k = line + halo;
      vec3 col = base.rgb;
      col += cSelect * (k.r * 1.9 + m.r * 0.05);
      col += cHint * (k.g * 1.9 * pulse + m.g * 0.10 * pulse);
      col += cHover * (k.b * 1.1 + m.b * 0.03);
      gl_FragColor = vec4(col, base.a);
    }`,
};

class OutlinePass extends Pass {
  constructor(fx) {
    super();
    this.fx = fx;
    this.mask = new THREE.WebGLRenderTarget(1, 1, { depthBuffer: true });
    this.material = new THREE.ShaderMaterial({
      uniforms: THREE.UniformsUtils.clone(OutlineShader.uniforms),
      vertexShader: OutlineShader.vertexShader,
      fragmentShader: OutlineShader.fragmentShader,
      depthTest: false,
      depthWrite: false,
    });
    this.quad = new FullScreenQuad(this.material);
    const clip = fx.viewer.clipPlanes;
    const mk = (hex, clipped) => new THREE.MeshBasicMaterial({ color: hex, side: THREE.DoubleSide, clippingPlanes: clipped ? clip : null, toneMapped: false });
    // red = selected, green = suggested, blue = hovered
    this.mats = { select: [mk(0xff0000), mk(0xff0000, true)], hint: [mk(0x00ff00), mk(0x00ff00, true)], hover: [mk(0x0000ff), mk(0x0000ff, true)] };
    this.items = []; // { mesh, kind }
    this.black = new THREE.Color(0);
    this._old = new THREE.Color();
    this.layers = new THREE.Layers();
    this.layers.set(MASK_LAYER);
  }
  setSize(w, h) {
    this.mask.setSize(w, h);
    this.material.uniforms.texel.value.set(1 / w, 1 / h);
    this.material.uniforms.width.value = Math.max(1.5, h / 520);
  }
  render(renderer, writeBuffer, readBuffer) {
    const { scene, camera } = this.fx;
    const swapped = [];
    for (const it of this.items) {
      const m = it.mesh.material;
      const first = Array.isArray(m) ? m[0] : m;
      swapped.push(it.mesh, m);
      it.mesh.material = this.mats[it.kind][first.userData.cut ? 1 : 0];
      it.mesh.layers.enable(MASK_LAYER);
    }
    const camMask = camera.layers.mask;
    camera.layers.mask = this.layers.mask;
    const bg = scene.background;
    scene.background = null;
    renderer.getClearColor(this._old);
    const oldAlpha = renderer.getClearAlpha();
    const oldAuto = renderer.autoClear;
    renderer.autoClear = true;
    renderer.setClearColor(this.black, 1);
    renderer.setRenderTarget(this.mask);
    renderer.render(scene, camera);
    renderer.setClearColor(this._old, oldAlpha);
    renderer.autoClear = oldAuto;
    scene.background = bg;
    camera.layers.mask = camMask;
    for (let i = 0; i < swapped.length; i += 2) {
      swapped[i].material = swapped[i + 1];
      swapped[i].layers.disable(MASK_LAYER);
    }
    const u = this.material.uniforms;
    u.tDiffuse.value = readBuffer.texture;
    u.tMask.value = this.mask.texture;
    u.pulse.value = 0.55 + 0.45 * Math.sin(performance.now() / 170);
    renderer.setRenderTarget(this.renderToScreen ? null : writeBuffer);
    this.quad.render(renderer);
  }
  dispose() {
    this.mask.dispose();
    this.material.dispose();
    this.quad.dispose();
    Object.values(this.mats).forEach((p) => p.forEach((m) => m.dispose()));
  }
}

// ---------- Scene pass: the only multisampled target of the chain ----------
// The scene is drawn once into its own 4x MSAA target (colour only is resolved: resolving depth and stencil
// too costs several milliseconds a frame) and copied into the composer's plain buffers, so the passes that
// follow never trigger another multisample resolve.
class ScenePass extends Pass {
  constructor(fx) {
    super();
    this.fx = fx;
    this.target = new THREE.WebGLRenderTarget(1, 1, {
      type: THREE.HalfFloatType,
      samples: 4,
      stencilBuffer: true,
      resolveDepthBuffer: false,
      resolveStencilBuffer: false,
    });
    this.material = new THREE.ShaderMaterial({
      uniforms: { tDiffuse: { value: this.target.texture } },
      vertexShader: `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
      fragmentShader: `uniform sampler2D tDiffuse; varying vec2 vUv; void main(){ gl_FragColor = texture2D(tDiffuse, vUv); }`,
      depthTest: false,
      depthWrite: false,
      blending: THREE.NoBlending,
    });
    this.quad = new FullScreenQuad(this.material);
  }
  setSize(w, h) {
    this.target.setSize(w, h);
  }
  render(renderer, writeBuffer) {
    const oldAuto = renderer.autoClear;
    renderer.autoClear = false;
    renderer.setRenderTarget(this.target);
    renderer.clear();
    renderer.render(this.fx.scene, this.fx.camera);
    renderer.setRenderTarget(this.renderToScreen ? null : writeBuffer);
    this.quad.render(renderer);
    renderer.autoClear = oldAuto;
  }
  dispose() {
    this.target.dispose();
    this.material.dispose();
    this.quad.dispose();
  }
}

// ---------- The chain ----------
// level 2: ambient occlusion + outline + bloom, level 1: outline + bloom, level 0: plain render.
export class Fx {
  constructor(viewer, { level = 2 } = {}) {
    this.viewer = viewer;
    this.renderer = viewer.renderer;
    this.scene = viewer.scene;
    this.camera = viewer.camera;
    this.level = level;
    this.solids = []; // opaque meshes of the engine: { mesh }
    this.extras = []; // things that must not write depth in the pre-pass (flows, gases, gizmos, floor)
    this.ghosts = [];
    // Ambient occlusion is soft by nature: half resolution is a quarter of the work and looks the same.
    this.aoScale = 0.5;

    const target = new THREE.WebGLRenderTarget(1, 1, { type: THREE.HalfFloatType, depthBuffer: false });
    this.composer = new EffectComposer(this.renderer, target);
    this.renderPass = new ScenePass(this);
    this.ao = new GTAOPass(this.scene, this.camera, 1, 1);
    this.gbuffer = new GBufferPass(this);
    this.ao.updateGtaoMaterial({ radius: 0.32, distanceExponent: 1.4, thickness: 1.2, scale: 1.15, samples: 8, distanceFallOff: 1, screenSpaceRadius: false });
    this.ao.updatePdMaterial({ lumaPhi: 10, depthPhi: 2, normalPhi: 3, radius: 5, radiusExponent: 1, rings: 2, samples: 8 });
    this.ao.blendIntensity = 0.9;
    const aoSetSize = this.ao.setSize.bind(this.ao);
    this.ao.setSize = (w, h) => aoSetSize(Math.max(1, Math.round(w * this.aoScale)), Math.max(1, Math.round(h * this.aoScale)));
    this.outline = new OutlinePass(this);
    this.outline.enabled = false;
    this.bloom = new UnrealBloomPass(new THREE.Vector2(256, 256), 0.3, 0.5, 1.3);
    // Threshold on the brightest channel: saturated flames (orange, blue) bloom as much as white sparks.
    const hp = this.bloom.materialHighPassFilter;
    // A NaN or runaway pixel (any shader can produce one) must not enter the blur, or it spreads over the
    // whole picture as a black block: such pixels are dropped here.
    hp.fragmentShader = hp.fragmentShader.replace(
      'float v = luminance( texel.xyz );',
      'float v = max( texel.r, max( texel.g, texel.b ) ); if ( ! ( v >= 0.0 && v < 1.0e4 ) ) { texel = vec4( 0.0 ); v = 0.0; }'
    );
    hp.needsUpdate = true;
    this.output = new OutputPass();
    for (const p of [this.renderPass, this.gbuffer, this.ao, this.outline, this.bloom, this.output]) this.composer.addPass(p);
    this.setLevel(level);
  }

  setLevel(level) {
    this.level = level;
    this.gbuffer.enabled = this.ao.enabled = level >= 2;
  }

  // Called after an engine is loaded: sort its objects into solids and everything else.
  attach(root, extras = []) {
    this.solids = [];
    this.extras = [...extras];
    root.traverse((o) => {
      if (o.userData.sectionHelper) return;
      if (o.isPoints || o.isLine || o.isSprite) return this.extras.push(o);
      if (!o.isMesh) return;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      const solid = mats.every((m) => m && !m.transparent && m.depthWrite !== false && (m.isMeshStandardMaterial || m.isMeshPhongMaterial || m.isMeshLambertMaterial));
      if (solid) this.solids.push({ mesh: o });
      else this.extras.push(o);
    });
  }

  // parts: { select, hover, hint[] } -> meshes drawn in the outline mask.
  setOutline({ select, hover, hint = [] }) {
    const items = [];
    const seen = new Set();
    const push = (p, kind) => {
      if (!p || seen.has(p)) return;
      seen.add(p);
      for (const mesh of p.meshes) items.push({ mesh, kind });
    };
    push(select, 'select');
    push(hover, 'hover');
    hint.forEach((p) => push(p, 'hint'));
    this.outline.items = items;
    this.outline.enabled = items.length > 0;
  }

  setSize(w, h, pixelRatio) {
    this.composer.setPixelRatio(pixelRatio);
    this.composer.setSize(w, h);
  }

  render() {
    if (this.level <= 0) return this.renderer.render(this.scene, this.camera);
    // Meshes hidden by an ancestor are skipped by three; parts that are fading out must not occlude.
    this.composer.render();
  }

  dispose() {
    this.renderPass.dispose();
    this.gbuffer.dispose();
    this.outline.dispose();
    this.ao.dispose();
    this.bloom.dispose();
    this.output.dispose();
    this.composer.dispose();
  }
}
