// EngineViewer: one Three.js canvas that hosts one engine module.
// Handles camera, lights, cutaway, explode, picking, phase highlight and the animation clock.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { clamp, lerp } from './helpers.js';
import { SectionCaps } from './section.js';
import { Fx, studioEnvironment, backdropTexture, floorTexture } from './fx.js';

const HOVER = new THREE.Color(0x38d6ff);
const SELECT = new THREE.Color(0x38d6ff);
const PHASE = new THREE.Color(0xff7a1a);
const Y = new THREE.Vector3(0, 1, 0);
const _v = new THREE.Vector3();
const _w = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _m = new THREE.Matrix4();
const _r = new THREE.Matrix4();
const THUMB_LAYER = 6;

export class EngineViewer {
  // fx: post-processing (ambient occlusion, outlines, bloom) and an opaque studio backdrop.
  constructor(container, { interactive = true, autoRotate = false, shadows = true, fx = interactive } = {}) {
    this.container = container;
    this.interactive = interactive;
    this.shadows = shadows;

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, stencil: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.06;
    this.renderer.localClippingEnabled = true;
    this.renderer.shadowMap.enabled = shadows;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.environment = studioEnvironment(this.renderer);
    this.scene.environmentIntensity = 0.62;
    if (fx) this.scene.background = backdropTexture();

    this.camera = new THREE.PerspectiveCamera(40, 1, 0.05, 200);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = 1.5;
    this.controls.maxDistance = 25;
    this.controls.autoRotate = autoRotate;
    this.controls.autoRotateSpeed = 0.6;
    this.controls.enabled = interactive;
    this.controls.addEventListener('start', () => (this.userMoved = true));

    const key = new THREE.DirectionalLight(0xfff4e6, 1.75);
    key.position.set(5, 8, 6);
    key.castShadow = shadows;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.bias = -0.0004;
    key.shadow.normalBias = 0.02;
    this.key = key;
    this.scene.add(key.target);
    const rim = new THREE.DirectionalLight(0x6fb8ff, 1.1);
    rim.position.set(-6, 3, -5);
    const hemi = new THREE.HemisphereLight(0xbfd8ff, 0x1a1d24, 0.3);
    this.lights = [key, rim, hemi];
    this.lights.forEach((l) => l.layers.enable(THUMB_LAYER));
    this.scene.add(key, rim, hemi);
    this.scene.add(this.#makeFloor());
    // Soft ground shadow under the model.
    this.shadowFloor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), new THREE.ShadowMaterial({ opacity: 0.42 }));
    this.shadowFloor.rotation.x = -Math.PI / 2;
    this.shadowFloor.receiveShadow = true;
    this.shadowFloor.visible = shadows;
    this.scene.add(this.shadowFloor);

    // Cutaway: a plane that clips z > 0. Disabled by pushing it far away.
    this.clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 1e6);
    this.clipPlanes = [this.clipPlane];
    this.section = new SectionCaps(this.scene, this.clipPlanes);
    this.arrow = this.#makeArrow();
    this.scene.add(this.arrow);
    // Phones and tablets skip the ambient occlusion pass; slow machines step down by themselves (see #tick).
    this.fx = fx ? new Fx(this, { level: matchMedia('(hover: none)').matches ? 1 : 2 }) : null;

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
    this.history = [];
    this.queue = [];
    this.hint = [];

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
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.MeshBasicMaterial({ map: floorTexture(), transparent: true, depthWrite: false, toneMapped: false })
    );
    floor.rotation.x = -Math.PI / 2;
    this.floor = floor;
    return floor;
  }

  // Arrow shown over a part to say which way it comes off (workshop game).
  #makeArrow() {
    const mat = new THREE.MeshBasicMaterial({ color: 0xffc233, depthTest: false, depthWrite: false, transparent: true, opacity: 0.95, toneMapped: false });
    const g = new THREE.Group();
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.035, 0.62, 12).translate(0, 0.31, 0), mat);
    const head = new THREE.Mesh(new THREE.ConeGeometry(0.11, 0.3, 16).translate(0, 0.77, 0), mat);
    g.add(shaft, head);
    g.traverse((o) => ((o.renderOrder = 999), (o.raycast = () => {})));
    g.visible = false;
    return g;
  }

  // ---------- Loading ----------
  load(def) {
    this.unload();
    this.def = def;
    this.parts = [];
    this.history = []; // groups of parts taken off, last on top
    this.queue = []; // { parts, d: 1 take off | -1 put back, fast }
    this.active = null;
    this.autoplay = 0; // 1 taking everything off, -1 putting everything back
    this.hint = []; // parts glowing orange: guide suggestions or what holds a part
    this.guide = null;
    this.game = false; // the workshop game drives hints and take-offs itself
    this.pulling = null;
    this.ghost = null;
    this.arrow.visible = false;
    this.arrowPart = null;
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
    this.fx?.attach(this.root, [this.floor, this.shadowFloor, this.arrow]);

    // Sit the floor under the model and fit the shadow camera around it.
    const box = new THREE.Box3().setFromObject(this.root);
    this.floor.position.set(centre0(box).x, box.min.y - 0.02, centre0(box).z);
    this.floor.scale.setScalar(Math.max(9, box.getSize(new THREE.Vector3()).length() * 2.4));
    this.shadowFloor.position.y = box.min.y - 0.015;
    const centre = box.getCenter(new THREE.Vector3());
    const radius = box.getSize(new THREE.Vector3()).length() / 2 + 0.5;
    this.modelRadius = radius - 0.5;
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
    if (this.interactive && !this.manual) {
      // Arrive with a short sweep around the model.
      const look = this.controls.target;
      this.camera.position.sub(look).applyAxisAngle(Y, -0.7).multiplyScalar(1.3).add(look);
      this.camera.position.y += this.modelRadius * 0.35;
      this.resetCamera();
    }
    this.#updateEngine(0);
    this.root.updateMatrixWorld(true);
    for (const p of this.parts) this.#measure(p);
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
    // The holder sits between the wrapper (which engines may show/hide for options) and the object:
    // only the viewer touches it, to take the part off and put it back.
    const holder = new THREE.Group();
    holder.userData.holder = true;
    wrapper.add(holder);
    holder.add(object);
    // Own materials per part so highlight does not leak to other parts.
    const meshes = [];
    wrapper.traverse((o) => {
      if (!o.isMesh || !o.material) return;
      const clone = (m) => {
        const c = m.clone();
        c.onBeforeCompile = m.onBeforeCompile; // not copied by clone(): keeps the grain shader hook
        c.customProgramCacheKey = m.customProgramCacheKey;
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
      holder,
      meshes,
      removeT: 0, // 0 = in place, 1 = taken off
      removeTarget: 0,
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
    if (this.history?.length || this.active) {
      this.#resetAllRemovals();
      this.#changed();
    }
    this.options[id] = value;
    this.engine?.setOption?.(id, value);
    if (this.selected && !isShown(this.selected.wrapper)) this.selectPart(null);
    this.#emit('option', { id, value });
  }
  // Parts currently shown (options can hide alternative parts; removed parts are gone).
  get visibleParts() {
    return this.parts.filter((p) => isPresent(p.holder));
  }
  get removedParts() {
    return this.parts.filter((p) => p.out);
  }

  // ---------- Taking parts off ----------
  // Free exploration: any part can come off, but with `def.disassembly.parts` rules some parts first need
  // the parts in their `after` list (bolts, covers…). Removals and restores run one at a time from a queue.
  #find(id) {
    return this.parts.find((x) => x.id === id) || null;
  }
  #rule(p) {
    return this.def?.disassembly?.parts?.[p.id] || null;
  }
  // Off the engine: taken off, or hidden by an option (e.g. the chain when the belt is chosen).
  #gone(p) {
    return p.out || !isShown(p.wrapper);
  }
  // Parts that must come off before `id` (empty when it can come off now).
  blockersOf(id) {
    const p = this.#find(id);
    const after = (p && this.#rule(p)?.after) || [];
    return after.map((x) => this.#find(x)).filter((b) => b && !this.#gone(b));
  }
  // 'unscrew' for bolts and plugs, otherwise 'lift'.
  motionOf(id) {
    const p = this.#find(id);
    return (p && this.#rule(p)?.motion) || 'lift';
  }
  get busy() {
    return !!(this.active || this.queue.length || this.autoplay);
  }

  // Returns false (and emits 'blocked' with the parts to take off first) when the part is still held.
  removePart(id) {
    const p = this.#find(id);
    if (!p || p.out || !isShown(p.wrapper)) return false;
    const blockers = this.blockersOf(id);
    if (blockers.length) {
      this.#blocked(p, blockers);
      return false;
    }
    const group = this.#groupOf(p);
    this.#takeOff(group);
    this.queue.push({ parts: group, d: 1 });
    this.#emit('select', this.selected);
    this.#changed();
    return true;
  }

  #blocked(p, blockers, mount = false) {
    if (this.game) {
      this.flashParts(blockers, 1.4);
      this.shakePart(p.id);
    } else {
      this.hint = blockers;
      this.hintBlocked = true;
      this.#refreshGlow();
    }
    this.#emit('blocked', { part: p, blockers, mount });
  }

  // The part plus what comes off with it (gasket with its cover, valves with the head).
  #groupOf(p) {
    const rule = this.#rule(p) || {};
    return [p, ...(rule.with || []).map((x) => this.#find(x)).filter((q) => q && q !== p && !this.#gone(q))];
  }

  #takeOff(group) {
    const rule = this.#rule(group[0]) || {};
    for (const q of group) {
      q.out = true;
      q.motion = rule; // parts that come off "with" it move the same way
    }
    if (group.includes(this.selected)) this.selected = null;
    if (group.includes(this.hovered)) this.hovered = null;
    this.history.push(group);
  }

  // ---------- Direct manipulation (workshop game) ----------
  // The page drives the take-off with a gesture: startPull, setPull(0..1) while dragging or holding the
  // wrench, endPull(true) to let go of the part for good or endPull(false) to let it settle back.
  startPull(id) {
    const p = this.#find(id);
    if (!p || p.out || !isShown(p.wrapper) || this.blockersOf(id).length) return false;
    const group = this.#groupOf(p);
    const rule = this.#rule(p) || {};
    for (const q of group) {
      q.motion = rule;
      q.pull = 0;
    }
    this.pulling = { part: p, group };
    return true;
  }
  setPull(t) {
    if (!this.pulling) return;
    for (const q of this.pulling.group) q.pull = clamp(t, 0, 1) * 0.6;
  }
  endPull(commit) {
    const g = this.pulling;
    if (!g) return;
    this.pulling = null;
    for (const q of g.group) {
      q.pull = null;
      q.removeTarget = commit ? 1 : 0;
      q.speed = commit ? 1.5 : 2.5;
    }
    if (commit) {
      this.#takeOff(g.group);
      this.#emit('select', this.selected);
    }
    this.#changed();
  }
  // Screen-space vector (px) of a full pull, to compare with the pointer drag.
  pullAxis(id) {
    const p = this.#find(id);
    if (!p) return { x: 0, y: -1, len: 0 };
    const a = this.projectPart(p);
    this.#removalDir(p, _v);
    const c = p.wrapper.localToWorld(p.localCentre.clone());
    _w.copy(_v).transformDirection(p.wrapper.parent.matrixWorld).multiplyScalar(this.#removalDist(p)).add(c).project(this.camera);
    const el = this.renderer.domElement;
    const bx = el.offsetLeft + (_w.x * 0.5 + 0.5) * el.clientWidth;
    const by = el.offsetTop + (-_w.y * 0.5 + 0.5) * el.clientHeight;
    const x = bx - a.x;
    const y = by - a.y;
    return { x, y, len: Math.hypot(x, y) };
  }

  // Parts that must go back on before `id` can (the reverse of blockersOf).
  mountBlockersOf(id) {
    const p = this.#find(id);
    const group = this.history.find((g) => g.includes(p));
    if (!group) return [];
    const res = new Set();
    for (const a of group)
      for (const q of this.parts)
        if (q.out && !group.includes(q) && this.#rule(q)?.after?.includes(a.id)) res.add(q);
    return [...res];
  }
  // Put one specific part (with its group) back on. False (and 'blocked') when something must go on first.
  mountPart(id) {
    const p = this.#find(id);
    const group = this.history.find((g) => g.includes(p));
    if (!p || !group) return false;
    const blockers = this.mountBlockersOf(id);
    if (blockers.length) {
      this.#emit('blocked', { part: p, blockers, mount: true });
      return false;
    }
    this.history.splice(this.history.indexOf(group), 1);
    if (this.ghost && group.includes(this.ghost)) this.previewPart(null);
    for (const q of group) {
      q.out = false;
      q.removeTarget = 0;
      q.speed = 1.4;
    }
    this.#changed();
    return true;
  }
  // Show a taken-off part as a ghost in its place (null to clear).
  previewPart(id) {
    const p = id ? this.#find(id) : null;
    const set = (q, on) => {
      const group = this.history.find((g) => g.includes(q)) || [q];
      for (const x of group) {
        x.ghost = on;
        if (!on && x.out && x.removeT >= 1) x.holder.visible = false;
      }
    };
    if (this.ghost) set(this.ghost, false);
    this.ghost = p && p.out ? p : null;
    if (this.ghost) set(this.ghost, true);
    this.#refreshGlow();
  }

  // Short feedback: wobble a part that will not come off, light up parts for a moment.
  shakePart(id) {
    const p = this.#find(id);
    if (p) p.shake = 0.45;
  }
  flashParts(parts, seconds = 1.2) {
    for (const p of parts) p.flash = seconds;
    this.#refreshGlow();
  }
  // Parts glowing orange until changed (game hints). Pass [] to clear.
  setHint(parts) {
    this.hint = parts.map((x) => (typeof x === 'string' ? this.#find(x) : x)).filter(Boolean);
    this.hintBlocked = false;
    this.#refreshGlow();
  }
  setGame(on) {
    this.game = on;
    this.hint = [];
    this.showArrow(null);
    this.previewPart(null);
    this.#changed();
  }
  // Arrow over a part pointing the way it comes off (null to hide).
  showArrow(id) {
    this.arrowPart = id ? this.#find(id) : null;
    this.arrow.visible = !!this.arrowPart;
  }
  part(id) {
    return this.#find(id);
  }

  // Put back the last part taken off.
  undo() {
    this.autoplay = 0;
    this.#dropPending();
    const group = this.history.pop();
    if (!group) return;
    group.forEach((q) => (q.out = false));
    this.queue.push({ parts: group, d: -1 });
    this.#changed();
  }

  // Put everything back in reverse order (quick, overlapping rewind).
  restoreAll() {
    this.#dropPending();
    this.autoplay = this.history.length ? -1 : 0;
    while (this.history.length) {
      const group = this.history.pop();
      group.forEach((q) => (q.out = false));
      this.queue.push({ parts: group, d: -1, fast: true });
    }
    this.#changed();
  }

  // Take everything off in the guide's order, one part at a time.
  autoDisassemble() {
    this.#dropPending();
    this.autoplay = 1;
    this.#changed();
  }

  stopAuto() {
    this.autoplay = 0;
    this.#dropPending();
    this.#changed();
  }

  // ---------- Officina guide: def.disassembly.steps suggest an order and explain each step ----------
  setGuide(on) {
    const d = this.def?.disassembly;
    if (!d?.steps || on === !!this.guide) return;
    if (on) {
      this.guide = { wasPaused: this.state.paused };
      this.state.paused = true;
    } else {
      this.state.paused = this.guide.wasPaused;
      this.guide = null;
    }
    this.#changed();
    this.#emit('guide', this.guide);
  }

  // Current step = first step with something still on the engine; `todo` = its parts still on.
  get guideStatus() {
    const steps = this.def?.disassembly?.steps || [];
    const live = steps.map((st) => st.parts.map((id) => this.#find(id)).filter((p) => p && isShown(p.wrapper)));
    let index = live.findIndex((ps) => ps.some((p) => !p.out));
    if (index < 0) index = steps.length;
    return { steps, index, todo: index < steps.length ? live[index].filter((p) => !p.out) : [] };
  }

  // "Show me": take off the next part the guide suggests.
  guideDemoNext() {
    const p = this.#nextGuidePart();
    if (p) this.removePart(p.id);
    return !!p;
  }

  #nextGuidePart() {
    const { steps } = this.guideStatus;
    for (const st of steps)
      for (const id of st.parts) {
        const p = this.#find(id);
        if (p && !this.#gone(p) && !this.blockersOf(id).length) return p;
      }
    return null;
  }

  // Something was taken off or put back: refresh the guide highlight and tell the page.
  #changed() {
    this.#guideHint();
    this.#emit('parts', this.removedParts);
  }
  #guideHint() {
    this.hintBlocked = false;
    if (!this.game) this.hint = this.guide ? this.guideStatus.todo.filter((p) => !this.blockersOf(p.id).length) : [];
    this.#refreshGlow();
  }

  // Queued items not started yet are cancelled, keeping `out` and the history consistent.
  #dropPending() {
    for (let i = this.queue.length - 1; i >= 0; i--) {
      const it = this.queue[i];
      if (it.d > 0) {
        it.parts.forEach((q) => (q.out = false));
        this.history.splice(this.history.lastIndexOf(it.parts), 1);
      } else {
        it.parts.forEach((q) => (q.out = true));
        this.history.push(it.parts);
      }
    }
    this.queue = [];
  }

  #runQueue() {
    const a = this.active;
    if (a) {
      const target = a.d > 0 ? 1 : 0;
      if (a.parts.some((p) => Math.abs(p.removeT - target) > (a.fast ? 0.45 : 0))) return;
      this.active = null;
    }
    if (!this.queue.length && this.autoplay > 0) {
      const next = this.#nextGuidePart();
      if (next) this.removePart(next.id);
    }
    const it = this.queue.shift();
    if (!it) {
      if (this.autoplay) {
        this.autoplay = 0;
        this.#changed();
      }
      return;
    }
    this.active = it;
    for (const p of it.parts) {
      p.removeTarget = it.d > 0 ? 1 : 0;
      p.speed = it.fast ? 2.2 : 1;
    }
  }

  #resetRemoval(p) {
    p.removeT = p.removeTarget = 0;
    p.out = false;
    p.pull = null;
    p.ghost = false;
    p.holder.visible = true;
    setFade(p.holder, 1);
    if (p.screws) spin(p.screws, 0, 0);
    p.motion = null;
  }

  #resetAllRemovals() {
    this.parts.forEach((p) => this.#resetRemoval(p));
    this.history = [];
    this.queue = [];
    this.active = null;
    this.autoplay = 0;
    this.pulling = null;
    this.ghost = null;
  }
  // Everything in the guide's list off at once, in a valid order (start of the "put it back" game).
  stripAll() {
    this.#dropPending();
    this.autoplay = 0;
    let p;
    while ((p = this.#nextGuidePart())) {
      const group = this.#groupOf(p);
      this.#takeOff(group);
      for (const q of group) {
        q.removeT = q.removeTarget = 1;
        q.holder.visible = false;
      }
    }
    this.#changed();
  }
  // Everything back in place at once (used when a game starts or ends).
  resetParts() {
    this.#resetAllRemovals();
    this.#changed();
  }

  setPaused(p) {
    this.state.paused = p;
  }
  togglePaused() {
    this.state.paused = !this.state.paused;
    return this.state.paused;
  }

  // Space covered by overlaid UI (px). The model is centred and fitted in the free area.
  setInsets({ top = 0, bottom = 0, right = 0, left = 0 } = {}) {
    const i = this.insets || {};
    if (i.top === top && i.bottom === bottom && i.right === right && i.left === left) return;
    this.insets = { top, bottom, right, left };
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
    const { top = 0, bottom = 0, right = 0, left = 0 } = this.insets || {};
    const freeW = Math.max(W * 0.4, W - right - left);
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
    if (this.hintBlocked) this.#guideHint();
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
    // With post-processing the outline does most of the work, so the tint can stay light.
    const soft = this.fx && this.fx.level > 0 ? 0.55 : 1;
    const lit = [];
    for (const p of this.parts) {
      let color = null;
      let k = 0;
      if (p.flash > 0) ([color, k] = [PHASE, 0.75]), lit.push(p);
      else if (p === this.selected || p.ghost) [color, k] = [SELECT, 0.55 * soft];
      else if (p === this.hovered) [color, k] = [HOVER, 0.3 * soft];
      else if (this.hint.includes(p)) ([color, k] = [PHASE, 0.3 + 0.3 * Math.sin(performance.now() / 160) ** 2]), lit.push(p);
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
    this.fx?.setOutline({ select: this.ghost || this.selected, hover: this.hovered, hint: lit });
  }

  #bindPointer() {
    const el = this.renderer.domElement;
    let down = null;
    // Top-most part under a point in normalised device coordinates: { part, distance } or null.
    const cast = (x, y, root) => {
      this.pointer.set(x, y);
      this.raycaster.setFromCamera(this.pointer, this.camera);
      for (const h of this.raycaster.intersectObject(root, true)) {
        if (!h.object.isMesh) continue;
        // Skip the half hidden by the cutaway.
        if (this.cutaway && h.point.z > this.clipPlane.constant + 0.001 && this.#isCutMesh(h.object)) continue;
        if (!isShown(h.object)) continue;
        let o = h.object;
        while (o && !o.userData.part) o = o.parent;
        if (o && !o.userData.part.out) return { part: o.userData.part, distance: h.distance };
      }
      return null;
    };
    const pick = (e) => {
      if (!this.root) return null;
      const r = el.getBoundingClientRect();
      const x = ((e.clientX - r.left) / r.width) * 2 - 1;
      const y = -((e.clientY - r.top) / r.height) * 2 + 1;
      const hit = cast(x, y, this.root);
      // Bolts and plugs are a few pixels wide: a near miss still picks them (when nothing nearer is in the way).
      const small = this.parts.filter((p) => this.#rule(p)?.motion === 'unscrew' && !p.out && isShown(p.wrapper));
      if (small.length && !small.includes(hit?.part)) {
        const limit = (hit ? hit.distance : Infinity) + this.modelRadius * 0.06;
        for (const px of [5, 10, 15])
          for (let a = 0; a < 8; a++) {
            const dx = (Math.cos((a * Math.PI) / 4) * px * 2) / r.width;
            const dy = (Math.sin((a * Math.PI) / 4) * px * 2) / r.height;
            for (const p of small) {
              const near = cast(x + dx, y + dy, p.wrapper);
              if (near && near.distance <= limit) return near.part;
            }
          }
      }
      return hit ? hit.part : null;
    };
    // Workshop game: the page may grab a part on pointer down (onGrab returns true); the drag then moves
    // the part instead of the camera. Registered in the capture phase to run before OrbitControls.
    const drag = (phase, e) => {
      const d = this.drag;
      this.#emit('drag', { phase, part: d.part, x: e.clientX, y: e.clientY, dx: e.clientX - d.x, dy: e.clientY - d.y });
    };
    el.addEventListener(
      'pointerdown',
      (e) => {
        if (e.button !== 0 || !this.onGrab) return;
        const p = pick(e);
        if (!p || !this.onGrab(p, e)) return;
        this.drag = { part: p, x: e.clientX, y: e.clientY, id: e.pointerId };
        this.controls.enabled = false;
        try {
          el.setPointerCapture(e.pointerId);
        } catch {}
        drag('start', e);
      },
      { capture: true }
    );
    const endDrag = (e) => {
      if (!this.drag || e.pointerId !== this.drag.id) return;
      drag(e.type === 'pointercancel' ? 'cancel' : 'end', e);
      this.drag = null;
      this.controls.enabled = this.interactive;
      down = null;
    };
    el.addEventListener('pointercancel', endDrag);
    el.addEventListener('pointermove', (e) => {
      if (this.drag) return drag('move', e);
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
      if (this.drag) return endDrag(e);
      if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
      const p = pick(e);
      if (this.hintBlocked) this.#guideHint();
      this.selected = p;
      this.#refreshGlow();
      this.#emit('select', p);
      // Right click without dragging (right drag pans): ask the page for a context menu.
      if (e.button === 2) this.#emit('context', { part: p, x: e.clientX, y: e.clientY });
    });
  }

  // Take-off animation (and its reverse when restoring). Runs after the explode offset is applied.
  // motion 'lift' (default): move along dir, fading out. 'unscrew': each screw spins and backs out
  // along its own axis, then the part moves away and fades.
  #animateRemoval(p, dt) {
    if (p.removeTarget && p.removeT >= 1) {
      if (p.ghost) this.#poseRemoval(p);
      return; // gone: nothing to animate
    }
    const m = p.motion || {};
    const dur = m.duration ?? (m.motion === 'unscrew' ? 1.6 : 0.9);
    p.removeT = clamp(p.removeT + ((p.removeTarget ? dt : -dt) * (p.speed || 1)) / dur, 0, 1);
    this.#poseRemoval(p);
  }

  #removalDir(p, out) {
    const m = p.motion || this.#rule(p) || {};
    if (m.dir) out.set(...m.dir).normalize();
    else if (p.explodeVec.lengthSq() > 1e-6) out.copy(p.explodeVec).normalize();
    else out.copy(Y);
    return out;
  }
  #removalDist(p) {
    const m = p.motion || this.#rule(p) || {};
    return m.dist ?? (m.motion === 'unscrew' ? 0.3 : Math.max(0.4, this.modelRadius * 0.2));
  }

  // Pose of a part from its removeT (0 in place .. 1 gone).
  #poseRemoval(p) {
    const t = p.removeT;
    const m = p.motion || {};
    const screw = m.motion === 'unscrew';
    if (screw) p.screws ||= findScrews(p);
    if (p.ghost && t >= 1) {
      p.holder.visible = true;
      if (p.screws) spin(p.screws, 0, 0);
      setFade(p.holder, 0.3);
      return;
    }
    p.holder.visible = t < 1;
    let lift, fade;
    if (screw) {
      const k = smooth(0, 0.6, t);
      spin(p.screws, k * (m.turns ?? 5) * Math.PI * 2, k * (m.out ?? 0.1));
      lift = smooth(0.6, 1, t);
      fade = 1 - smooth(0.75, 1, t);
    } else {
      lift = smooth(0, 0.8, t);
      fade = 1 - smooth(0.4, 1, t);
    }
    p.wrapper.position.addScaledVector(this.#removalDir(p, _v), lift * this.#removalDist(p));
    setFade(p.holder, fade);
  }

  // Centre and size of a part in its wrapper's frame, measured at the assembled pose.
  #measure(p) {
    const box = new THREE.Box3().setFromObject(p.wrapper);
    if (box.isEmpty()) box.setFromCenterAndSize(p.wrapper.getWorldPosition(new THREE.Vector3()), new THREE.Vector3(0.1, 0.1, 0.1));
    p.localCentre = p.wrapper.worldToLocal(box.getCenter(new THREE.Vector3()));
    p.radius = box.getSize(new THREE.Vector3()).length() / 2;
  }

  // Small pictures of single parts (for the bench list of the workshop game): { id: dataURL }.
  partThumbs(ids, size = 144) {
    const out = {};
    const rt = new THREE.WebGLRenderTarget(size, size, { samples: 4 });
    rt.texture.colorSpace = THREE.SRGBColorSpace;
    const cam = new THREE.PerspectiveCamera(30, 1, 0.01, 200);
    cam.layers.set(THUMB_LAYER);
    const buf = new Uint8Array(size * size * 4);
    const canvas = document.createElement('canvas');
    canvas.width = canvas.height = size;
    const g = canvas.getContext('2d');
    const img = g.createImageData(size, size);
    const r = this.renderer;
    const keep = { bg: this.scene.background, clip: this.clipPlane.constant, auto: r.shadowMap.autoUpdate, target: r.getRenderTarget(), alpha: r.getClearAlpha() };
    this.scene.background = null;
    this.clipPlane.constant = 1e6;
    r.shadowMap.autoUpdate = false;
    r.setClearAlpha(0);
    const envI = this.scene.environmentIntensity;
    this.scene.environmentIntensity = 1.5;
    const dir = new THREE.Vector3(...(this.def.camera?.position || [4, 3, 6])).sub(new THREE.Vector3(...(this.def.camera?.target || [0, 0, 0]))).normalize();
    for (const id of ids) {
      const p = this.#find(id);
      if (!p || !isShown(p.wrapper)) continue;
      const c = p.wrapper.localToWorld(p.localCentre.clone());
      // Start outside the bounding sphere, then move in until the box corners fill the frame.
      let dist = (p.radius / Math.sin((cam.fov * Math.PI) / 360)) * 1.05;
      const box = new THREE.Box3().setFromObject(p.wrapper);
      const tan = Math.tan((cam.fov * Math.PI) / 360) * 0.86;
      for (let pass = 0; pass < 3 && !box.isEmpty(); pass++) {
        cam.position.copy(c).addScaledVector(dir, dist);
        cam.lookAt(c);
        cam.updateMatrixWorld();
        let fill = 0;
        for (let k = 0; k < 8; k++) {
          _w.set(k & 1 ? box.max.x : box.min.x, k & 2 ? box.max.y : box.min.y, k & 4 ? box.max.z : box.min.z).applyMatrix4(cam.matrixWorldInverse);
          fill = Math.max(fill, Math.abs(_w.x) / -_w.z, Math.abs(_w.y) / -_w.z);
        }
        if (!(fill > 0) || !Number.isFinite(fill)) break;
        dist = Math.max(p.radius * 1.02, dist * (fill / tan));
      }
      cam.position.copy(c).addScaledVector(dir, dist);
      cam.lookAt(c);
      cam.updateMatrixWorld();
      p.meshes.forEach((m) => m.layers.enable(THUMB_LAYER));
      r.setRenderTarget(rt);
      r.clear();
      r.render(this.scene, cam);
      p.meshes.forEach((m) => m.layers.disable(THUMB_LAYER));
      r.readRenderTargetPixels(rt, 0, 0, size, size, buf);
      for (let y = 0; y < size; y++) img.data.set(buf.subarray((size - 1 - y) * size * 4, (size - y) * size * 4), y * size * 4);
      g.putImageData(img, 0, 0);
      out[id] = canvas.toDataURL('image/webp', 0.85);
    }
    r.setRenderTarget(keep.target);
    r.setClearAlpha(keep.alpha);
    r.shadowMap.autoUpdate = keep.auto;
    r.shadowMap.needsUpdate = true;
    this.scene.environmentIntensity = envI;
    this.scene.background = keep.bg;
    this.clipPlane.constant = keep.clip;
    rt.dispose();
    return out;
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
    this.fx?.setSize(w, h, this.renderer.getPixelRatio());
    // Shift the view so the model is centred in the space left free by overlaid controls.
    const { top = 0, bottom = 0, right = 0, left = 0 } = this.insets || {};
    if (top || bottom || right || left) this.camera.setViewOffset(w, h, (right - left) / 2, (bottom - top) / 2, w, h);
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
    const raw = this.clock.getDelta();
    const dt = Math.min(raw, 0.05);
    if (!this.visible || document.hidden || !this.engine || this.manual) return;
    // Slow machine: step the effects down (first the ambient occlusion, then everything).
    if (this.fx?.level > 0 && raw < 0.25) {
      this.frameAvg = lerp(this.frameAvg ?? raw, raw, 0.04);
      this.frameCount = (this.frameCount || 0) + 1;
      if (this.frameCount > 100 && this.frameAvg > 0.037) {
        this.fx.setLevel(this.fx.level - 1);
        this.frameCount = 0;
        this.frameAvg = 0.016;
      }
    }
    this.step(dt);
  }

  // Advances the animation by dt and renders one frame (also used by the dev capture tool).
  step(dt) {
    if (!this.engine) return;
    this.#updateEngine(dt);

    // Explode easing; base position captured lazily after the first build pose.
    const prevF = 1 + 0.5 * this.explode;
    this.explode = lerp(this.explode, this.explodeTarget, 1 - Math.exp(-dt * 6));
    // Engines hide flows and gas when exploded; in the Officina guide the engine is open, so the same applies.
    this.state.explode = this.guide || this.game ? Math.max(1, this.explode) : this.explode;
    this.#runQueue();
    let flashing = false;
    for (const p of this.parts)
      if (p.flash > 0) {
        p.flash -= dt;
        flashing = true;
      }
    if (this.hint.length || flashing) this.#refreshGlow(); // pulse
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
      if (p.pull != null) {
        p.removeT = lerp(p.removeT, p.pull, 1 - Math.exp(-dt * 20));
        this.#poseRemoval(p);
      } else if (p.removeT || p.removeTarget) this.#animateRemoval(p, dt);
      if (p.shake > 0) {
        p.shake -= dt;
        const a = Math.sin(p.shake * 70) * Math.min(1, p.shake * 4) * 0.012 * this.modelRadius;
        p.wrapper.position.x += a;
        p.wrapper.position.z += a * 0.6;
      }
    }
    if (this.arrowPart) this.#poseArrow();

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
    if (this.fx) this.fx.render();
    else this.renderer.render(this.scene, this.camera);
    this.#emit('frame', this.state);
  }

  #poseArrow() {
    const p = this.arrowPart;
    if (p.out && !this.pulling) return void (this.arrow.visible = false);
    const c = p.wrapper.localToWorld(p.localCentre.clone());
    this.#removalDir(p, _v).transformDirection(p.wrapper.parent.matrixWorld);
    const s = clamp(this.modelRadius * 0.28, 0.5, 1.6);
    const bob = 0.08 * s * (1 + Math.sin(performance.now() / 180));
    this.arrow.position.copy(c).addScaledVector(_v, p.radius * 0.35 + bob);
    this.arrow.quaternion.setFromUnitVectors(Y, _v);
    this.arrow.scale.setScalar(s);
    this.arrow.visible = true;
  }

  // World-space position of a part centre projected to container pixels.
  projectPart(part) {
    if (!part.localCentre) this.#measure(part);
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
    this.fx?.dispose();
    this.scene.environment?.dispose();
    this.scene.background?.dispose?.();
    this.controls.dispose();
    this.renderer.dispose();
    // Give the GPU memory back now: browsing many engines in a row must not pile up dead WebGL contexts.
    this.renderer.forceContextLoss();
    this.renderer.domElement.remove();
  }
}

function smooth(a, b, x) {
  const t = clamp((x - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
}

// Fades every material under `root` to factor f (1 = as built). Section-cap helpers are muted while
// fading, otherwise a solid hatched cap would show through the ghost.
function setFade(root, f) {
  root.traverse((o) => {
    if (o.userData.sectionHelper) {
      o.userData.muted = f < 1;
      return;
    }
    if (!o.material) return;
    for (const m of Array.isArray(o.material) ? o.material : [o.material]) {
      const u = m.userData;
      if (u.fadeBase === undefined) {
        if (f >= 1) continue;
        u.fadeBase = { opacity: m.opacity, transparent: m.transparent };
      }
      const transparent = u.fadeBase.transparent || f < 1;
      if (m.transparent !== transparent) {
        m.transparent = transparent;
        m.needsUpdate = true;
      }
      m.opacity = u.fadeBase.opacity * f;
      if (f >= 1) delete u.fadeBase;
    }
  });
}

// Like isShown, but a part that is coming back counts as present while it is still animating in.
function isPresent(o) {
  for (; o; o = o.parent) {
    if (o.userData.holder) {
      if (o.parent.userData.part.out) return false;
    } else if (!o.visible) return false;
  }
  return true;
}

// Screws of a part: every instance of its InstancedMeshes, or else the children of its object.
// Each one turns around and backs out along its own local Y axis.
function findScrews(p) {
  const list = [];
  p.holder.traverse((o) => o.isInstancedMesh && list.push({ mesh: o, base: o.instanceMatrix.array.slice() }));
  if (!list.length) for (const c of p.holder.children[0].children) list.push({ obj: c, pos: c.position.clone(), quat: c.quaternion.clone() });
  return list;
}

function spin(screws, angle, out) {
  _r.makeRotationY(angle).setPosition(0, out, 0);
  for (const s of screws) {
    if (s.mesh) {
      const a = s.mesh.instanceMatrix.array;
      if (!angle && !out) a.set(s.base);
      else for (let i = 0; i < s.mesh.count; i++) _m.fromArray(s.base, i * 16).multiply(_r).toArray(a, i * 16);
      s.mesh.instanceMatrix.needsUpdate = true;
    } else {
      s.obj.quaternion.copy(s.quat).multiply(_q.setFromAxisAngle(Y, angle));
      s.obj.position.copy(s.pos).add(_w.set(0, out, 0).applyQuaternion(s.quat));
    }
  }
}

function isShown(o) {
  for (; o; o = o.parent) if (!o.visible) return false;
  return true;
}

function centre0(box) {
  return box.getCenter(new THREE.Vector3());
}
