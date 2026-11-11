# Engine module contract

Each engine is one ES module in `js/engines/<id>.js` with a default export.
Three.js is imported as `import * as THREE from 'three'`; helpers from `../core/helpers.js`.
All UI text is Italian. Two text levels: `base` (curious beginner, no formulas) and
`expert` (correct terms, numbers, thermodynamic cycle, efficiencies).

```js
export default {
  id: 'petrol4t',
  name: 'Motore a benzina 4 tempi',
  short: 'Benzina 4T',               // short label for chips/selects
  family: 'terra',                    // terra | aria | spazio | mare
  vehicle: 'Automobile',              // where you find it today
  examples: 'Fiat Panda, VW Golf…',   // real-world examples
  tagline: 'One-line hook',
  intro: { base: '...', expert: '...' },   // 2-4 sentences each, may contain <b>
  specs: [ { label: 'Carburante', value: 'Benzina' }, ... ],  // 5-7 rows, shown in panel + compare
  metrics: {                           // numbers for compare bars (typical real example)
    efficiency: 35,     // % fuel/energy -> useful work
    power: 110,         // kW, typical unit
    powerDensity: 0.8,  // kW per kg of engine
  },
  phases: [
    // Cyclic engine: `range` is a slice of the 0..1 cycle; the UI follows the animation.
    { id: 'intake', name: 'Aspirazione', range: [0, 0.25], parts: ['intakeValve'],
      base: '...', expert: '...' },
    // Continuous engine (turbine, rocket, electric): omit `range`, list `parts` to highlight.
  ],
  quiz: [ { q: '...', options: ['a','b','c','d'], correct: 1, explain: '...' } ],  // 4 questions
  camera: { position: [4, 2.5, 6], target: [0, 0.5, 0] },
  throttleLabel: 'Acceleratore',     // optional: label of the 0..1 slider (e.g. 'Collettivo', 'Tempo di combustione')

  build(ctx) {
    // ctx.part(object3D, { id, name, base, expert, explode: [x,y,z] }) -> wrapper Group
    //   Every clickable component goes through ctx.part. Animate the INNER object,
    //   add the WRAPPER to your scene graph (explode moves the wrapper by explode*amount).
    // ctx.addFlow(flowObject) registers a helpers.flow() result (viewer calls nothing on it;
    //   you call flow.update(dt, rate) yourself in update()).
    const root = new THREE.Group();
    return {
      root,
      // cycles per second at a given throttle 0..1 (drives state.cycle)
      cycleRate(throttle) { return 0.3 + throttle * 2; },
      // state: { cycle (0..1, wraps), throttle 0..1, dt, time, paused, explode 0..1 }
      // Hide gas volumes/flows that would float in mid-air when explode > 0.05.
      update(state) { ... },
      // live gauges, 2-4 entries
      readout(state) { return [{ label: 'Giri', value: '3200', unit: 'rpm' }]; },
    };
  },
};
```

## Scene conventions
- Y is up. The model fits roughly in a 5-unit box centred near the origin.
- The camera looks from +Z. The cutaway removes everything with z > 0 for materials
  marked with `cut(material)` (casings, housings, nacelles, tanks). Inner moving parts
  are NOT cut, so the cutaway reveals them.
- Use `MAT.*` factories (one call per distinct look; sharing a material between meshes is fine,
  the viewer clones per part for highlighting).
- Flows: cold air/fluid in cyan-blue (0x5fd3ff), fuel in amber (0xffc04d), combustion in
  orange-red (0xff5a1f), exhaust in grey-violet (0x8c86a8), electric current in yellow-green
  (0xd8ff4a), magnetic field in magenta (0xff4ad8).
- Keep it light: < 150k triangles, < 3000 particles total.
- When `state.dt` is 0 (paused) update must still pose parts from `state.cycle`,
  so phase stepping works.

## Lessons from the first four engines (read before building a new one)
- Look at `petrol4t.js` (cyclic engine) and `turbofan.js` (continuous engine) as references.
- `ctx.part` clones the materials of everything inside the part. If you animate emissive/opacity of a
  part's mesh, write to `mesh.material` (the clone) inside `update`, never to the original material variable.
- Hide flows, gas volumes, flames and plumes when `state.explode > 0.05`, otherwise they float in mid-air.
- On desktop the control dock covers the bottom ~140 px; the viewer already shifts the view up to compensate.
  Frame the whole model (including plumes, propellers, rails) in `camera`, then check it in the preview.
- When `state.dt === 0` (paused / phase stepping) the pose must still come from `state.cycle`.
- Cyclic engines (pistons, rotary): phases use `range` and the UI shows "cilindro 1"-style progress.
  `cycleRate` should give a slow, readable motion (0.1–1 cycles/s), with real rpm shown only in `readout`.
- Real numbers in `expert` texts must be correct; when unsure, give a rounded range rather than a precise figure.

## Previewing a new engine
A dev server runs at http://localhost:5173 (no-cache). Open `http://localhost:5173/#/anteprima/<file-name-without-.js>`
to load your file in the real viewer before it is in the catalog. `window.__viewer` is the live EngineViewer
(useful: `__viewer.setExplode(1)`, `__viewer.goToPhase(2)`, `__viewer.setThrottle(1)`, `__viewer.parts`).

## v2 — detail pass (read this before improving a model)
The site now has two explicit views, **Chiusa** (closed, how you'd see the real engine) and **Sezione**
(front half removed, z > 0). Both must look like a real, finished engine.

### Section caps (automatic)
Every mesh whose material is `cut(...)` AND whose geometry is a **closed solid** gets a hatched section cap
(orange/red diagonal hatch, like a technical drawing) where the cut plane passes through solid material.
- Model casings as closed solids with real wall thickness: `pipe()` (closed lathe profile), `lathe()` with a
  closed profile, `ExtrudeGeometry` of a shape with holes (bores, ports, passages), `RoundedBoxGeometry`, boxes.
- Open surfaces (open lathe profiles, `TubeGeometry`, open-ended cylinders/cones) get no cap: they look like
  paper-thin shells. Avoid them for casings.
- Solid boxes for a cylinder head or block are OK (they really are solid metal) but cut ports, bores,
  water jackets and oil galleries as holes so the section shows them.
- Hollow volumes modelled as closed solids (propellant tanks, gas volumes, air ducts) must use
  `cut(mat, { cap: false })`, otherwise they appear filled with hatch. Transparent materials never get caps.

### Closed view checklist
- Rounded edges: `import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js'`
  instead of sharp boxes for castings and covers.
- Fasteners: rows of bolt heads on flanges and covers (one `InstancedMesh` of a small hex cylinder per material
  is cheap). Gasket lines between castings (thin dark band).
- Ribs, fins, bosses, mounting brackets/engine mounts, lifting eyes.
- Ancillaries: hoses with clamps, wiring harness with connectors, sensors, oil filler cap, dipstick, filters,
  pumps, pulleys and accessory belt where the real engine has them.
- Materials: distinguish cast (rougher, darker) vs machined (bright) vs painted vs rubber vs plastic covers.
  A simple canvas-texture nameplate/logo plate (generic, no real brands) adds realism.
- Nothing floating: every part is attached to something.

### Section view checklist
- Inside the cut you should see believable internals: ports, passages, coolant jacket, oil galleries,
  bearings/bushings (thin bronze rings), seals, injector/plug bores, stator laminations, etc.
- Moving internals must not intersect casings at any point of the cycle.

### New optional fields
```js
variants: {               // "Versioni reali" table in Panoramica
  columns: ['Motore', 'Cilindrata', 'Potenza', 'Distribuzione'],
  rows: [['Fiat 1.3 Multijet', '1,3 l', '55–70 kW', 'Catena'], ...],
  note: 'optional short note',
},
options: [                // configurable variants shown as buttons in Panoramica
  { id: 'timing', label: 'Distribuzione', default: 'dryBelt', choices: [
    { id: 'dryBelt', label: 'Cinghia a secco', base: '...', expert: '...' },
    { id: 'wetBelt', label: 'Cinghia a bagno d’olio', base: '...', expert: '...' },
    { id: 'chain',   label: 'Catena', base: '...', expert: '...' },
  ] },
],
thumb: { orbit: [-35, 18], zoom: 1.1, view: 'section' }, // optional framing for the home-page card thumbnail
// build() may return setOption(id, value): show/hide the alternative geometry (register each alternative
// as its own ctx.part; the viewer hides invisible parts from labels and lists).
```
For piston engines add a `Distribuzione` row in `specs` (cinghia a secco / cinghia a bagno d’olio / catena /
ingranaggi / aste e bilancieri / nessuna, a luci) and list real versions in `variants`.
Facts must be correct; if unsure about a specific engine, leave it out.

### Taking parts off and the Officina guide (optional `disassembly`)
Every part can be taken off from its card (**Togli**, or **Svita** for screws) or with a right click.
Without rules a part simply lifts out. With `disassembly.parts` a part first needs the parts in its `after`
list: pressing Togli on a held part shows "Prima togli …" and makes those parts glow orange.
`disassembly.steps` adds the **Officina** tool: a guide that suggests an order, explains each step and
highlights the parts to tap (› takes the next one off for you, ‹ puts the last one back, play does it all).
Keep it educational and simplified: no torque values or real workshop procedures unless certain.
```js
disassembly: {
  intro: 'One sentence.',
  note: 'Simplified-sequence disclaimer (shown in the Officina tab of the sheet).',
  parts: {                              // how each part comes off and what holds it
    camCoverBolts: { motion: 'unscrew', turns: 6, out: 0.1 },
    camCover: { after: ['camCoverBolts', 'ignitionCoils'], with: ['coverGasket'], dir: [0, 1, 0], dist: 0.8 },
  },
  steps: [                              // the guide: suggested order + text
    { id: 'cover', name: 'Togli il coperchio', base: 'One sentence.', expert: '…', parts: ['camCoverBolts', 'camCover'] },
  ],
},
```
- `after`: parts that must be off first. A part hidden by the current option (e.g. the chain when the belt
  is chosen) counts as off, so list all alternatives. No cycles.
- `with`: parts that come off together with it (gasket with its cover, valves with the head).
- `motion: 'lift'` (default) moves the part along `dir` (in the part's parent frame; default: its explode
  direction) by `dist` and fades it. `motion: 'unscrew'` first spins every screw `turns` times while it backs out
  by `out`, then moves the part away (`dist`, default 0.3) and fades it. Optional `duration` in seconds
  (defaults 0.9 lift, 1.6 unscrew).
- Screws: each instance of the part's `InstancedMesh`es, or (with no instanced mesh) each direct child of the
  part's object, turning about and backing out along its own local **+Y**. So bolts are their own part
  (see `screw()` in `petrol4t.js` and `screws()` in `moto4t.js`: hex head with its shank, +Y out of the hole).
- Parts are put back in reverse order (undo stack). Changing an option puts everything back at once.
- While the guide is open the engine is paused and `state.explode` is reported as 1, so flows hide.
- `checkEngine()` validates ids, motions and cycles. References: `petrol4t.js`, `moto4t.js`.

### Seeing your model (works in a background tab)
```js
const { shoot } = await import('/js/dev/capture.js?t=' + Date.now());
await shoot('<file>', { view: 'closed' });                       // -> .claude/shots/<file>-closed.png
await shoot('<file>', { view: 'section', orbit: [-30, 15], zoom: 1.4, name: 'x' });
await shoot('<file>', { explode: 1, name: 'x-exploded' });
```
To save what the live engine page shows (e.g. mid-way through an Officina step), drive it by hand and use
`snapLive`: `__viewer.manual = true; __viewer.step(0.05); await (await import('/js/dev/capture.js?t=1')).snapLive('name')`
(set `__viewer.manual = false` afterwards).
Then open the PNG with the Read tool (it shows images). Iterate: closed, section, exploded, a close-up.
Reload your tab (navigate it to http://localhost:5173/?r=<n>#/) after editing helpers or the viewer;
engine files are re-imported fresh on each call.

## v3 — Officina games (every engine needs a `disassembly`)
The Officina button opens three games played on the live model (`js/game/workshop.js`):
**Smonta** (take the parts off in a valid order, against the clock), **Rimonta** (the parts start on the bench
and go back in a valid order) and **Trova il pezzo** (find the named part; needs no data).
Smonta and Rimonta are driven entirely by `disassembly`, so write it with the player in mind:
- The parts listed in `steps` are the game's targets (plus their `with` parts). Aim for 10–22 targets in 5–10 steps.
  Parts that are not in any step stay on the engine.
- Tools: `motion: 'unscrew'` parts need the **wrench** (the player holds the pointer on them while the screws
  turn), everything else comes off by **hand** (the player drags it along `dir`; an arrow shows the direction).
  Give every engine 2–4 fastener parts so the wrench has work to do: make them their own `ctx.part`
  (an `InstancedMesh` of screws, local +Y out of the hole — see `screw()` in `petrol4t.js`).
- The game is played in the **closed** view, with the pointer: a part can only be grabbed when the player can
  see it. So whatever hides a part (covers, casings, nacelles) must be in its `after` list and in an earlier step.
  Inner parts become reachable only once the shell around them is on the bench.
- `dir` is where the part visibly has room to move (in its parent's frame); `dist` far enough to clear its neighbours.
- A wrong order or the wrong tool costs points and shows the `name`s of the blocking parts, so `after` must be
  believable: "Prima togli: Viti del coperchio".
- Step texts are shown as the goal ("Togli il coperchio") and in the guide; `base`/`expert` as usual.

### Checking it (works in a background tab)
Open `http://localhost:5173/?r=<n>#/anteprima/<file>` (change `<n>` to reload after an edit), then in the console:
```js
(await import('/js/dev/check.js?t=' + Date.now())).checkEngine('<file>')     // errors must be []
const { playStrip } = await import('/js/dev/play.js?t=' + Date.now());
await playStrip()   // plays Smonta with a synthetic pointer -> { removed, total, unreachable, ... }
```
`removed` must equal `total` and `unreachable` must be empty: an unreachable part is hidden by something that is
not in its `after` list (or is too deep to see). Look at it too: `snapLive('name')` after `playStrip()` shows the
stripped engine; `__ws.stop()` puts everything back.
