// EngineViewer: one Three.js canvas that hosts one engine module.
// Handles camera, lights, cutaway, explode, picking, phase highlight and the animation clock.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { clamp, lerp } from './helpers.js';
import { SectionCaps } from './section.js';

const HOVER = new THREE.Color(0x38d6ff);
const SELECT = new THREE.Color(0x38d6ff);
const PHASE = new THREE.Color(0xff7a1a);

export class EngineViewer {
  constructor(container, { interactive = true, autoRotate = false, shadows = true } = {}) {
    this.container = container;
    this.interactive = interactive;
    this.shadows = shadows;

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, stencil: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 0.95;
    this.renderer.localClippingEnabled = true;
    this.renderer.shadowMap.enabled = shadows;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this.scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();
    this.scene.environmentIntensity = 0.75;

    this.camera = new THREE.PerspectiveCamera(40, 1, 0.05, 200);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = 1.5;
    this.controls.maxDistance = 25;
    this.controls.autoRotate = autoRotate;
    this.controls.autoRotateSpeed = 0.6;
    this.controls.enabled = interactive;
    this.controls.addEventListener('start', () => (this.userMoved = true));

    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(5, 8, 6);
    key.castShadow = shadows;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.bias = -0.0004;
    key.shadow.normalBias = 0.02;
    this.key = key;
    this.scene.add(key.target);
    const rim = new THREE.DirectionalLight(0x6fb8ff, 0.9);
    rim.position.set(-6, 3, -5);
    this.scene.add(key, rim, new THREE.HemisphereLight(0xbfd8ff, 0x1a1d24, 0.35));
    this.scene.add(this.#makeFloor());
    // Soft ground shadow under the model.
    this.shadowFloor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), new THREE.ShadowMaterial({ opacity: 0.32 }));
    this.shadowFloor.rotation.x = -Math.PI / 2;
    this.shadowFloor.receiveShadow = true;
    this.shadowFloor.visible = shadows;
    this.scene.add(this.shadowFloor);

    // Cutaway: a plane that clips z > 0. Disabled by pushing it far away.
    this.clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 1e6);
    this.clipPlanes = [this.clipPlane];
    this.section = new SectionCaps(this.scene, this.clipPlanes);

    this.state = { cycle: 0, throttle: 0.35, dt: 0, time: 0, paused: false, explode: 0 };
    this.explodeTarget = 0;
    this.explode = 0;
    this.cutaway = false;
    this.stepMode = false;
    this.phaseIndex = -1;
    this.parts = [];
    this.hovered = null;
    this.selected = null;
    this.listeners = {};

    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    if (interactive) this.#bindPointer();

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resizeObserver.observe(this.renderer.domElement);
    this.visible = true;
    this.intersection = new IntersectionObserver(([e]) => (this.visible = e.isIntersecting));
    this.intersection.observe(container);

    this.clock = new THREE.Clock();
    this.running = true;
    this.renderer.setAnimationLoop(() => this.#tick());
    this.resize();
  }

  on(event, fn) {
    (this.listeners[event] ||= []).push(fn);
  }
  #emit(event, data) {
    (this.listeners[event] || []).forEach((fn) => fn(data));
  }

  #makeFloor() {
    const c = document.createElement('canvas');
    c.width = c.height = 256;
    const g = c.getContext('2d');
    const grd = g.createRadialGradient(128, 128, 0, 128, 128, 128);
    grd.addColorStop(0, 'rgba(56,214,255,0.18)');
    grd.addColorStop(0.5, 'rgba(56,214,255,0.05)');
    grd.addColorStop(1, 'rgba(56,214,255,0)');
    g.fillStyle = grd;
    g.fillRect(0, 0, 256, 256);
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(14, 14),
      new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(c), transparent: true, depthWrite: false })
    );
    floor.rotation.x = -Math.PI / 2;
    this.floor = floor;
    return floor;
  }

  // ---------- Loading ----------
  load(def) {
    this.unload();
    this.def = def;
    this.parts = [];
    const ctx = {
      part: (object, info) => this.#registerPart(object, info),
      addFlow: () => {},
    };
    this.engine = def.build(ctx);
    this.root = this.engine.root;
    this.scene.add(this.root);

    // Shell materials get the shared clip plane array.
    this.root.traverse((o) => {
      if (!o.material) return;
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      mats.forEach((m) => {
        if (m.userData.cut) {
          m.clippingPlanes = this.clipPlanes;
          m.clipShadows = true;
        }
      });
      // Solid, opaque surfaces cast and receive shadows; glows, gases and particles don't.
      const solid = !o.isPoints && mats.every((m) => !m.transparent && m.isMeshStandardMaterial && !(m.emissiveIntensity > 1.5));
      o.castShadow = o.receiveShadow = this.shadows && solid;
    });
    this.section.attach(this.root);

    // Sit the floor under the model and fit the shadow camera around it.
    const box = new THREE.Box3().setFromObject(this.root);
    this.floor.position.y = box.min.y - 0.02;
    this.shadowFloor.position.y = box.min.y - 0.015;
    const centre = box.getCenter(new THREE.Vector3());
    const radius = box.getSize(new THREE.Vector3()).length() / 2 + 0.5;
    this.key.target.position.copy(centre);
    this.key.position.copy(centre).add(new THREE.Vector3(4, 8, 5).normalize().multiplyScalar(radius * 2.5));
    const sc = this.key.shadow.camera;
    Object.assign(sc, { left: -radius, right: radius, top: radius, bottom: -radius, near: 0.1, far: radius * 5 });
    sc.updateProjectionMatrix();

    // Engine options (e.g. timing drive type) start at their defaults.
    this.options = {};
    for (const opt of def.options || []) this.setOption(opt.id, opt.default ?? opt.choices[0].id);

    this.state.cycle = 0;
    this.phaseIndex = -1;
    this.explode = this.explodeTarget = 0;
    this.selected = this.hovered = null;
    this.userMoved = false;
    this.resetCamera(true);
    this.#updateEngine(0);
  }

  unload() {
    if (!this.root) return;
    this.scene.remove(this.root);
    this.root.traverse((o) => {
      o.geometry?.dispose();
      const mats = Array.isArray(o.material) ? o.material : o.material ? [o.material] : [];
      mats.forEach((m) => m.dispose());
    });
    this.root = null;
    this.engine = null;
  }

  #registerPart(object, info) {
    const wrapper = new THREE.Group();
    wrapper.name = info.id;
    wrapper.add(object);
    // Own materials per part so highlight does not leak to other parts.
    const meshes = [];
    wrapper.traverse((o) => {
      if (!o.isMesh || !o.material) return;
      const clone = (m) => {
        const c = m.clone();
        c.userData.baseEmissive = c.emissive ? c.emissive.clone() : null;
        c.userData.baseEmissiveIntensity = c.emissiveIntensity ?? 1;
        return c;
      };
      o.material = Array.isArray(o.material) ? o.material.map(clone) : clone(o.material);
      o.userData.partId = info.id;
      meshes.push(o);
    });
    const part = {
      ...info,
      wrapper,
      meshes,
      explodeVec: new THREE.Vector3(...(info.explode || [0, 0, 0])),
      basePos: null,
    };
    wrapper.userData.part = part;
    this.parts.push(part);
    return wrapper;
  }

  // ---------- Public controls ----------
  setThrottle(v) {
    this.state.throttle = clamp(v, 0, 1);
  }
  setExplode(v, instant = false) {
    this.explodeTarget = clamp(v, 0, 1);
    if (instant) this.explode = this.explodeTarget;
  }
  setCutaway(on, instant = false) {
    this.cutaway = on;
    if (instant) this.clipPlane.constant = on ? 0 : 1e6;
  }
  setOption(id, value) {
    this.options[id] = value;
    this.engine?.setOption?.(id, value);
    if (this.selected && !isShown(this.selected.wrapper)) this.selectPart(null);
    this.#emit('option', { id, value });
  }
  // Parts currently shown (options can hide alternative parts).
  get visibleParts() {
    return this.parts.filter((p) => isShown(p.wrapper));
  }
  setPaused(p) {
    this.state.paused = p;
  }
  togglePaused() {
    this.state.paused = !this.state.paused;
    return this.state.paused;
  }

  // Space covered by overlaid UI (px). The model is centred and fitted in the free area.
  setInsets({ top = 0, bottom = 0, right = 0 } = {}) {
    const i = this.insets || {};
    if (i.top === top && i.bottom === bottom && i.right === right) return;
    this.insets = { top, bottom, right };
    this.resize();
    if (this.def && !this.userMoved) this.resetCamera();
  }
  setBottomInset(px) {
    this.setInsets({ ...this.insets, bottom: px });
  }

  resetCamera(instant = false) {
    const cam = this.def?.camera || { position: [5, 3, 7], target: [0, 0, 0] };
    const look = new THREE.Vector3(...cam.target);
    // Keep the explode dolly factor so un-exploding later lands on the default framing.
    // Narrow (portrait) canvases need the camera further back to fit wide models.
    // Overlaid UI shrinks the free area: fit the model to that area, not the whole canvas.
    const el = this.renderer.domElement;
    const W = el.clientWidth || 1;
    const H = el.clientHeight || 1;
    const { top = 0, bottom = 0, right = 0 } = this.insets || {};
    const freeW = Math.max(W * 0.4, W - right);
    const freeH = Math.max(H * 0.4, H - top - bottom);
    const fit = Math.pow(Math.max(1, 1.3 / (freeW / freeH)), 0.9) * Math.pow(H / freeH, 0.4);
    const pos = new THREE.Vector3(...cam.position).sub(look).multiplyScalar((1 + 0.5 * this.explode) * fit).add(look);
    this.camTarget = { pos, look };
    if (instant) {
      this.camera.position.copy(this.camTarget.pos);
      this.controls.target.copy(this.camTarget.look);
      this.camTarget = null;
    }
  }

  get phases() {
    return this.def?.phases || [];
  }

  // Jump to a phase. Cyclic phases move the animation there and pause it.
  goToPhase(i) {
    const n = this.phases.length;
    if (!n) return;
    i = ((i % n) + n) % n;
    const ph = this.phases[i];
    this.stepMode = true;
    if (ph.range) {
      const mid = (ph.range[0] + ph.range[1]) / 2;
      this.state.cycle = mid;
      this.state.paused = true;
    }
    this.#setPhase(i);
  }

  exitStepMode() {
    this.stepMode = false;
    this.state.paused = false;
    this.#setPhase(this.#phaseFromCycle());
  }

  selectPart(id) {
    const p = this.parts.find((x) => x.id === id) || null;
    this.selected = p;
    this.#refreshGlow();
    this.#emit('select', p);
  }

  // ---------- Internals ----------
  #phaseFromCycle() {
    const c = this.state.cycle % 1;
    const i = this.phases.findIndex((p) => p.range && c >= p.range[0] && c < p.range[1]);
    return i;
  }

  #setPhase(i) {
    if (i === this.phaseIndex) return;
    this.phaseIndex = i;
    this.#refreshGlow();
    this.#emit('phase', i);
  }

  #refreshGlow() {
    const phase = this.phases[this.phaseIndex];
    const phaseParts = this.stepMode && phase ? phase.parts || [] : [];
    for (const p of this.parts) {
      let color = null;
      let k = 0;
      if (p === this.selected) [color, k] = [SELECT, 0.55];
      else if (p === this.hovered) [color, k] = [HOVER, 0.3];
      else if (phaseParts.includes(p.id)) [color, k] = [PHASE, 0.45];
      for (const mesh of p.meshes) {
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        for (const m of mats) {
          if (!m.emissive) continue;
          if (color) {
            m.emissive.copy(color);
            m.emissiveIntensity = k;
          } else {
            m.emissive.copy(m.userData.baseEmissive);
            m.emissiveIntensity = m.userData.baseEmissiveIntensity;
          }
        }
      }
    }
  }

  #bindPointer() {
    const el = this.renderer.domElement;
    let down = null;
    const pick = (e) => {
      const r = el.getBoundingClientRect();
      this.pointer.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      this.raycaster.setFromCamera(this.pointer, this.camera);
      if (!this.root) return null;
      const hits = this.raycaster.intersectObject(this.root, true);
      for (const h of hits) {
        if (!h.object.isMesh) continue;
        // Skip the half hidden by the cutaway.
        if (this.cutaway && h.point.z > this.clipPlane.constant + 0.001 && this.#isCutMesh(h.object)) continue;
        if (!isShown(h.object)) continue;
        let o = h.object;
        while (o && !o.userData.part) o = o.parent;
        if (o) return o.userData.part;
      }
      return null;
    };
    el.addEventListener('pointermove', (e) => {
      if (e.buttons) return;
      const p = pick(e);
      if (p !== this.hovered) {
        this.hovered = p;
        el.style.cursor = p ? 'pointer' : 'grab';
        this.#refreshGlow();
        this.#emit('hover', p);
      }
    });
    el.addEventListener('pointerleave', () => {
      this.hovered = null;
      this.#refreshGlow();
      this.#emit('hover', null);
    });
    el.addEventListener('pointerdown', (e) => (down = [e.clientX, e.clientY]));
    el.addEventListener('pointerup', (e) => {
      if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
      const p = pick(e);
      this.selected = p;
      this.#refreshGlow();
      this.#emit('select', p);
    });
  }

  #isCutMesh(mesh) {
    const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    return mats.some((m) => m.userData.cut);
  }

  resize() {
    // Measure the canvas itself: on phones CSS gives it a fixed height below which the dock sits.
    const el = this.renderer.domElement;
    if (!el.style.width) Object.assign(el.style, { width: '100%', height: '100%' });
    const w = el.clientWidth;
    const h = el.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    // Shift the view so the model is centred in the space left free by overlaid controls.
    const { top = 0, bottom = 0, right = 0 } = this.insets || {};
    if (top || bottom || right) this.camera.setViewOffset(w, h, right / 2, (bottom - top) / 2, w, h);
    else this.camera.clearViewOffset();
    const prevAspect = this.camera.aspect;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    // Re-frame if the shape changed a lot and the user hasn't started orbiting yet.
    if (this.def && !this.userMoved && Math.abs(Math.log(prevAspect / this.camera.aspect)) > 0.15) this.resetCamera(true);
  }

  #updateEngine(dt) {
    if (!this.engine) return;
    const s = this.state;
    s.dt = s.paused ? 0 : dt;
    if (!s.paused) {
      s.time += dt;
      s.cycle = (s.cycle + dt * this.engine.cycleRate(s.throttle)) % 1;
    }
    this.engine.update(s);
  }

  #tick() {
    const dt = Math.min(this.clock.getDelta(), 0.05);
    if (!this.visible || document.hidden || !this.engine || this.manual) return;
    this.step(dt);
  }

  // Advances the animation by dt and renders one frame (also used by the dev capture tool).
  step(dt) {
    if (!this.engine) return;
    this.#updateEngine(dt);

    // Explode easing; base position captured lazily after the first build pose.
    const prevF = 1 + 0.5 * this.explode;
    this.explode = lerp(this.explode, this.explodeTarget, 1 - Math.exp(-dt * 6));
    this.state.explode = this.explode;
    // Dolly the camera out while exploding so the separated parts stay in view.
    const ratio = (1 + 0.5 * this.explode) / prevF;
    if (Math.abs(ratio - 1) > 1e-5) {
      const off = this.camera.position.clone().sub(this.controls.target).multiplyScalar(ratio);
      this.camera.position.copy(this.controls.target).add(off);
      if (this.camTarget) this.camTarget.pos.sub(this.camTarget.look).multiplyScalar(ratio).add(this.camTarget.look);
    }
    for (const p of this.parts) {
      if (!p.basePos) p.basePos = p.wrapper.position.clone();
      p.wrapper.position.copy(p.basePos).addScaledVector(p.explodeVec, this.explode);
    }

    // Cutaway plane slides in/out.
    const targetC = this.cutaway ? 0 : 8;
    const c = this.clipPlane.constant > 50 ? 8 : this.clipPlane.constant;
    const next = lerp(c, targetC, 1 - Math.exp(-dt * 5));
    this.clipPlane.constant = !this.cutaway && next > 7.9 ? 1e6 : next;
    this.section.update(this.clipPlane.constant);

    if (!this.stepMode) this.#setPhase(this.#phaseFromCycle());

    if (this.camTarget) {
      const k = 1 - Math.exp(-dt * 4);
      this.camera.position.lerp(this.camTarget.pos, k);
      this.controls.target.lerp(this.camTarget.look, k);
      if (this.camera.position.distanceTo(this.camTarget.pos) < 0.01) this.camTarget = null;
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.#emit('frame', this.state);
  }

  // World-space position of a part centre projected to container pixels.
  projectPart(part) {
    if (!part.localCentre) {
      // Centre in wrapper space, measured once at the assembled pose.
      const box = new THREE.Box3().setFromObject(part.wrapper);
      part.localCentre = part.wrapper.worldToLocal(box.getCenter(new THREE.Vector3()));
    }
    const v = part.wrapper.localToWorld(part.localCentre.clone()).project(this.camera);
    // Pixels relative to the container, measured on the canvas (which may not fill the container).
    const el = this.renderer.domElement;
    const w = el.clientWidth;
    const h = el.clientHeight;
    return { x: el.offsetLeft + (v.x * 0.5 + 0.5) * w, y: el.offsetTop + (-v.y * 0.5 + 0.5) * h, behind: v.z > 1 };
  }

  dispose() {
    this.renderer.setAnimationLoop(null);
    this.resizeObserver.disconnect();
    this.intersection.disconnect();
    this.unload();
    this.section.dispose();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}

function isShown(o) {
  for (; o; o = o.parent) if (!o.visible) return false;
  return true;
}
