// Workshop games played on the live 3D model of an engine page.
//   strip  "Smonta":  take the parts off in a valid order with the right tool (hand = drag along the arrow,
//                     wrench = hold on bolts and plugs), against the clock.
//   build  "Rimonta": everything starts on the bench; put the parts back in a valid order.
//   hunt   "Trova il pezzo": the game names a part, the player finds it on the model.
// The viewer does the 3D work (pull gestures, ghosts, hints); this file keeps score and draws the HUD.
import { sfx } from './sfx.js';
import { getRecord, saveRecord } from './records.js';

const svg = (d, extra = '') => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" ${extra}>${d}</svg>`;
export const GICON = {
  hand: svg('<path d="M8 13V5.5a1.5 1.5 0 0 1 3 0V12M11 11V4.5a1.5 1.5 0 0 1 3 0V12M14 11.5V6a1.5 1.5 0 0 1 3 0v8M17 11a1.5 1.5 0 0 1 3 0v4a7 7 0 0 1-7 7h-1a6 6 0 0 1-5-2.6L4.4 15a1.6 1.6 0 0 1 2.6-1.8L8 14.5"/>'),
  wrench: svg('<path d="M14.7 6.3a1 1 0 0 0 0 1.4l1.6 1.6a1 1 0 0 0 1.4 0l3.8-3.8a6 6 0 0 1-7.9 7.9l-6.9 6.9a2.1 2.1 0 0 1-3-3l6.9-6.9a6 6 0 0 1 7.9-7.9z"/>'),
  bulb: svg('<path d="M9 18h6M10 21h4M12 3a6 6 0 0 0-3.6 10.8c.6.5 1 1.3 1.1 2.2h5c.1-.9.5-1.700 1.100-2.200A6 6 0 0 0 12 3z"/>'),
  x: svg('<path d="M6 6l12 12M18 6L6 18"/>'),
  soundOn: svg('<path d="M4 9v6h4l5 4V5L8 9z"/><path d="M16.5 8.500a5 5 0 0 1 0 7M19 6a8.500 8.500 0 0 1 0 12"/>'),
  soundOff: svg('<path d="M4 9v6h4l5 4V5L8 9z"/><path d="M17 9.500l4 5M21 9.500l-4 5"/>'),
  book: svg('<path d="M4 5a2 2 0 0 1 2-2h13v16H6a2 2 0 0 0-2 2z"/><path d="M4 21a2 2 0 0 1 2-2h13v2H6"/><path d="M9 8h6"/>'),
  target: svg('<circle cx="12" cy="12" r="8.500"/><circle cx="12" cy="12" r="4.500"/><circle cx="12" cy="12" r="1" fill="currentColor"/>'),
  puzzle: svg('<path d="M10 4a2 2 0 1 1 4 0v2h4a1 1 0 0 1 1 1v3h-1.500a2 2 0 1 0 0 4H19v4a1 1 0 0 1-1 1h-3.500v-1.500a2 2 0 1 0-4 0V20H6a1 1 0 0 1-1-1v-4h1.500a2 2 0 1 0 0-4H5V7a1 1 0 0 1 1-1h4z"/>'),
  star: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M12 2.600l2.900 6 6.600.900-4.800 4.600 1.200 6.500L12 17.500l-5.900 3.100 1.200-6.500L2.500 9.500l6.600-.900z"/></svg>',
  bolt: svg('<path d="M7 3h10l2 4-2 4H7L5 7z"/><path d="M10 11v9l2 1.500 2-1.500v-9"/>'),
  play: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg>',
};

export const MODES = {
  strip: { name: 'Smonta', title: 'Sfida: smonta il motore', desc: 'Togli i pezzi nell’ordine giusto e con l’attrezzo giusto. Il tempo corre.', icon: 'wrench', done: 'Motore smontato!' },
  build: { name: 'Rimonta', title: 'Sfida: rimonta il motore', desc: 'I pezzi sono sul banco: rimettili al loro posto, uno dopo l’altro.', icon: 'puzzle', done: 'Motore rimontato!' },
  hunt: { name: 'Trova il pezzo', title: 'Trova il pezzo', desc: 'Ti dico un nome, tu lo trovi sul modello. Usa sezione ed esploso per guardare dentro.', icon: 'target', done: 'Caccia finita!' },
};

const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
const h = (html) => {
  const t = document.createElement('template');
  t.innerHTML = html.trim();
  return t.content.firstElementChild;
};
const firstSentence = (html) => {
  const t = String(html || '').replace(/<[^>]+>/g, '').trim();
  const m = t.match(/^.{20,}?[.!?](?=\s|$)/);
  return m ? m[0] : t;
};
const clock = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
const pts = (n) => Math.round(n).toLocaleString('it-IT');
const shuffle = (a) => {
  for (let i = a.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [a[i], a[j]] = [a[j], a[i]];
  }
  return a;
};
const starsHtml = (n, max = 3) => `<span class="stars" aria-label="${n} stelle su ${max}">${Array.from({ length: max }, (_, i) => `<i class="${i < n ? 'on' : ''}">${GICON.star}</i>`).join('')}</span>`;
export { starsHtml };

const HUNT_ROUNDS = 8;
const HUNT_SECONDS = 25;

export class Workshop {
  // ui: { setView(section), setExploded(on), setLabels(on), startGuide(), layout() } provided by the engine page.
  constructor({ viewer, def, stage, page, txt, ui }) {
    Object.assign(this, { viewer, def, stage, page, txt, ui });
    this.mode = null;
    this.tool = 'hand';
    this.hasRules = !!def.disassembly?.steps?.length;
    this.#build();
    viewer.onGrab = (part) => this.#canGrab(part);
    viewer.on('drag', (e) => this.#drag(e));
    viewer.on('select', (p) => this.#clicked(p));
    viewer.on('hover', (p) => this.#hover(p));
    viewer.on('frame', () => this.#frame());
  }

  get active() {
    return !!this.mode;
  }
  // The page hides part names (tooltip, labels, card) while the player has to recognise parts.
  get hidesNames() {
    return this.mode === 'hunt';
  }

  // ---------- DOM ----------
  #build() {
    this.hud = h(`<div class="game-hud" hidden>
      <div class="g-top">
        <div class="g-goal"><span class="g-mode"></span><b class="g-text"></b>
          <div class="g-progress"><i></i></div></div>
        <div class="g-stats">
          <div class="g-stat"><small>Tempo</small><b class="g-time">0:00</b></div>
          <div class="g-stat"><small>Punti</small><b class="g-score">0</b></div>
          <div class="g-stat g-combo"><small>Combo</small><b>×1</b><i></i></div>
          <button class="g-ico" data-act="sound" title="Suoni"></button>
          <button class="g-ico" data-act="exit" title="Esci dal gioco (Esc)">${GICON.x}</button>
        </div>
      </div>
      <aside class="bench"><h5>Banco <span></span></h5><div class="bench-list"></div><p class="bench-empty">I pezzi che togli finiscono qui.</p></aside>
      <div class="g-toast" role="status" hidden></div>
      <div class="toolbelt" role="toolbar" aria-label="Attrezzi">
        <button class="g-tool" data-tool="hand" aria-pressed="true" title="Mano: trascina il pezzo nella direzione della freccia (1)">${GICON.hand}<span>Mano</span><kbd>1</kbd></button>
        <button class="g-tool" data-tool="wrench" aria-pressed="false" title="Chiave: tieni premuto su viti e candele (2)">${GICON.wrench}<span>Chiave</span><kbd>2</kbd></button>
        <i class="g-sep"></i>
        <button class="g-tool hint" data-act="hint" title="Mostra la prossima mossa: costa 40 punti (H)">${GICON.bulb}<span>Aiuto</span><kbd>H</kbd></button>
      </div>
      <div class="g-ring" hidden><svg viewBox="0 0 44 44"><circle cx="22" cy="22" r="18"/><circle class="v" cx="22" cy="22" r="18" pathLength="100"/></svg>${GICON.wrench}</div>
      <div class="g-pops"></div>
    </div>`);
    this.modal = h(`<div class="g-modal" hidden></div>`);
    this.stage.append(this.hud, this.modal);
    const q = (s) => this.hud.querySelector(s);
    this.el = {
      mode: q('.g-mode'), text: q('.g-text'), bar: q('.g-progress i'), time: q('.g-time'), score: q('.g-score'),
      combo: q('.g-combo'), bench: q('.bench'), list: q('.bench-list'), count: q('.bench h5 span'), toast: q('.g-toast'),
      belt: q('.toolbelt'), ring: q('.g-ring'), pops: q('.g-pops'), sound: q('[data-act="sound"]'),
    };
    this.hud.querySelectorAll('[data-tool]').forEach((b) => b.addEventListener('click', () => this.setTool(b.dataset.tool)));
    q('[data-act="hint"]').addEventListener('click', () => this.hint());
    q('[data-act="exit"]').addEventListener('click', () => this.stop());
    this.el.sound.addEventListener('click', () => (sfx.toggle(), this.#syncSound()));
    this.#syncSound();
    this.modal.addEventListener('click', (e) => e.target === this.modal && this.modal.dataset.kind === 'menu' && this.closeMenu());
  }
  #syncSound() {
    this.el.sound.innerHTML = sfx.enabled ? GICON.soundOn : GICON.soundOff;
    this.el.sound.setAttribute('aria-pressed', sfx.enabled);
  }

  // ---------- Menu ----------
  openMenu() {
    const rec = (m) => {
      const r = getRecord(this.def.id, m);
      return r ? `${starsHtml(r.stars)}<small>${pts(r.score)} punti · ${clock(r.time)}</small>` : `${starsHtml(0)}<small>Mai giocato</small>`;
    };
    const card = (m) => `<button class="mode-card" data-mode="${m}"><span class="mode-ico">${GICON[MODES[m].icon]}</span>
        <b>${MODES[m].name}</b><p>${MODES[m].desc}</p><span class="mode-rec">${rec(m)}</span></button>`;
    this.modal.dataset.kind = 'menu';
    this.modal.innerHTML = `<div class="g-card menu" role="dialog" aria-label="Officina">
        <button class="g-ico close" aria-label="Chiudi">${GICON.x}</button>
        <span class="eyebrow">Officina</span>
        <h2>${esc(this.def.short || this.def.name)}: scegli la sfida</h2>
        <div class="mode-grid">
          ${this.hasRules ? `<button class="mode-card" data-mode="guide"><span class="mode-ico">${GICON.book}</span><b>Guida</b><p>Impara la sequenza di smontaggio, un passo alla volta e senza fretta.</p><span class="mode-rec"><small>Per cominciare</small></span></button>` : ''}
          ${this.hasRules ? card('strip') + card('build') : ''}
          ${card('hunt')}
        </div>
      </div>`;
    this.modal.hidden = false;
    this.modal.querySelector('.close').addEventListener('click', () => this.closeMenu());
    this.modal.querySelectorAll('[data-mode]').forEach((b) =>
      b.addEventListener('click', () => {
        this.closeMenu();
        if (b.dataset.mode === 'guide') this.ui.startGuide();
        else this.start(b.dataset.mode);
      })
    );
    this.modal.querySelector('[data-mode]').focus({ preventScroll: true });
    sfx.click();
  }
  closeMenu() {
    this.modal.hidden = true;
    this.modal.innerHTML = '';
  }
  get menuOpen() {
    return !this.modal.hidden;
  }

  // ---------- Start / stop ----------
  start(mode, { keepParts = false } = {}) {
    if (!MODES[mode] || (mode !== 'hunt' && !this.hasRules)) return;
    const v = this.viewer;
    const wasActive = this.active;
    if (!wasActive) this.before = { paused: v.state.paused, cutaway: v.cutaway };
    this.closeMenu();
    this.mode = mode;
    this.ended = false;
    Object.assign(this, { score: 0, combo: 0, lastHit: 0, errors: 0, hints: 0, moves: 0, t0: performance.now(), elapsed: 0, grab: null, hintTimer: 0 });
    this.ui.setLabels(false);
    this.ui.setExploded(false);
    v.selectPart(null);
    this.el.list.innerHTML = '';
    this.page.classList.add('gaming');
    this.page.dataset.game = mode;
    this.hud.hidden = false;
    this.el.mode.textContent = MODES[mode].name;

    if (mode === 'hunt') {
      v.setGame(true);
      v.resetParts();
      v.setPaused(false);
      this.ui.setView(true);
      const pool = shuffle(v.visibleParts.filter((p) => p.meshes.length));
      this.rounds = pool.slice(0, Math.min(HUNT_ROUNDS, pool.length));
      this.total = this.rounds.length;
      this.round = -1;
      this.#nextRound();
    } else {
      if (!(keepParts && mode === 'build')) v.resetParts();
      v.setGame(true);
      v.setPaused(true);
      this.ui.setView(false);
      // Parts the game is about: the ones listed in the guide (plus what comes off with them).
      const shown = new Set(v.parts.filter((p) => p.out || v.visibleParts.includes(p)).map((p) => p.id));
      const rules = this.def.disassembly.parts || {};
      this.stepIds = new Set(this.def.disassembly.steps.flatMap((s) => s.parts).filter((id) => shown.has(id)));
      this.owner = {}; // part that comes off "with" another -> that other part
      for (const id of this.stepIds) for (const w of rules[id]?.with || []) if (shown.has(w) && !this.stepIds.has(w)) this.owner[w] = id;
      this.targets = [...this.stepIds, ...Object.keys(this.owner)].map((id) => v.part(id)).filter(Boolean);
      this.total = this.targets.length;
      if (!keepParts || !this.thumbs) this.thumbs = keepParts ? this.thumbs || {} : v.partThumbs(this.targets.map((p) => p.id));
      if (mode === 'build') {
        if (!keepParts) v.stripAll();
        for (const group of shuffle([...v.history])) this.#benchAdd(group[0], true);
        this.setTool('hand');
        this.#toast('info', 'Scegli un pezzo dal banco: passaci sopra per vedere dove va.', 5000);
      } else {
        this.setTool('hand');
        this.#toast('info', '<b>Mano</b>: trascina il pezzo lungo la freccia. <b>Chiave</b>: tieni premuto sulle viti.', 6000);
      }
    }
    this.par = mode === 'hunt' ? this.total * 9 : this.total * (mode === 'build' ? 5 : 6.5) + 10;
    this.#refresh();
    this.ui.layout();
    sfx.go();
  }

  stop() {
    if (!this.active) return;
    const v = this.viewer;
    if (this.grab) v.endPull(false);
    this.grab = null;
    this.mode = null;
    this.ended = false;
    clearTimeout(this.toastTimer);
    clearTimeout(this.roundTimer);
    this.hud.hidden = true;
    this.modal.hidden = true;
    this.el.ring.hidden = true;
    this.page.classList.remove('gaming');
    delete this.page.dataset.game;
    delete this.stage.dataset.tool;
    v.setGame(false);
    v.resetParts();
    v.selectPart(null);
    v.setPaused(this.before?.paused ?? false);
    this.ui.setView(this.before?.cutaway ?? true);
    this.ui.layout();
  }

  setTool(tool) {
    this.tool = tool;
    this.stage.dataset.tool = tool;
    this.hud.querySelectorAll('[data-tool]').forEach((b) => b.setAttribute('aria-pressed', b.dataset.tool === tool));
    this.viewer.showArrow(null);
    if (this.active) sfx.click();
  }

  // Keyboard shortcuts while a game is on. Returns true when the key was used.
  key(k) {
    if (this.menuOpen && k === 'escape') return this.closeMenu(), true;
    if (!this.active) return false;
    if (k === 'escape') this.stop();
    else if (k === '1') this.setTool('hand');
    else if (k === '2') this.setTool('wrench');
    else if (k === 'h') this.hint();
    else if (k === 'v' || k === 's' || k === 'r' || (this.mode === 'hunt' && k === 'e')) return false; // view keys still work
    return true;
  }

  // ---------- Strip: gestures on the model ----------
  #need(part) {
    return this.viewer.motionOf(part.id) === 'unscrew' ? 'wrench' : 'hand';
  }
  #canGrab(part) {
    if (this.mode !== 'strip' || this.ended || this.grab) return false;
    return this.stepIds.has(part.id) && !part.out && !this.viewer.blockersOf(part.id).length && this.#need(part) === this.tool;
  }

  #drag(e) {
    if (this.mode !== 'strip') return;
    const v = this.viewer;
    if (e.phase === 'start') {
      if (!v.startPull(e.part.id)) return;
      this.grab = { part: e.part, tool: this.tool, progress: 0, axis: v.pullAxis(e.part.id), last: performance.now(), clicks: 0, x: e.x, y: e.y };
      v.selectPart(null);
      sfx.grab();
      if (this.tool === 'hand') v.showArrow(e.part.id);
      else this.#ring(e.x, e.y, 0);
      return;
    }
    const g = this.grab;
    if (!g) return;
    g.x = e.x;
    g.y = e.y;
    if (e.phase === 'move') {
      if (g.tool !== 'hand') return this.#ring(e.x, e.y, g.progress);
      const a = g.axis;
      const along = a.len < 28 ? Math.hypot(e.dx, e.dy) : (e.dx * a.x + e.dy * a.y) / a.len;
      g.progress = Math.min(1, Math.max(0, along / Math.min(Math.max(a.len, 90), 230)));
      v.setPull(g.progress);
      if (g.progress >= 1) this.#takeOff();
      return;
    }
    // end / cancel
    if (e.phase === 'end' && g.tool === 'hand' && g.progress > 0.5) return this.#takeOff();
    v.endPull(false);
    v.showArrow(null);
    this.el.ring.hidden = true;
    this.grab = null;
    if (e.phase === 'end' && g.progress < 0.08)
      this.#toast('info', g.tool === 'hand' ? 'Tieni premuto e <b>trascina</b> il pezzo lungo la freccia.' : '<b>Tieni premuto</b> finché le viti non sono uscite.');
  }

  #ring(x, y, progress) {
    const r = this.stage.getBoundingClientRect();
    const el = this.el.ring;
    el.hidden = false;
    el.style.transform = `translate(${x - r.left}px, ${y - r.top}px)`;
    el.querySelector('.v').style.strokeDashoffset = 100 - progress * 100;
  }

  #takeOff() {
    const g = this.grab;
    this.grab = null;
    this.el.ring.hidden = true;
    this.viewer.endPull(true);
    this.viewer.showArrow(null);
    sfx.pop();
    this.#benchAdd(g.part, false);
    this.#hit(g.part, g.x, g.y);
  }

  // A correct move: points, combo, a line about the part, progress.
  #hit(part, x, y, bonus = 0) {
    const now = performance.now();
    this.combo = now - this.lastHit < 6500 ? Math.min(this.combo + 1, 5) : 1;
    this.lastHit = now;
    const gain = 100 * this.combo + bonus;
    this.score += gain;
    this.moves++;
    setTimeout(() => sfx.score(this.combo), 90);
    this.#pop(`+${gain}`, x, y, this.combo > 1 ? `ok c${this.combo}` : 'ok');
    this.#toast('ok', `<b>${esc(part.name)}</b> ${esc(firstSentence(this.txt(part)))}`, 4200);
    this.#clearHint();
    this.#refresh();
    if (this.#complete()) setTimeout(() => this.#finish(), 900);
  }

  #miss(html, part, x, y) {
    this.errors++;
    this.combo = 0;
    this.score = Math.max(0, this.score - 25);
    sfx.error();
    if (part) this.viewer.shakePart(part.id);
    this.#toast('bad', html, 3200);
    if (x != null) this.#pop('−25', x, y, 'bad');
    this.hud.classList.remove('shake');
    void this.hud.offsetWidth;
    this.hud.classList.add('shake');
    this.#refresh();
  }

  #clicked(p) {
    if (!this.active || this.ended || !p) return;
    const v = this.viewer;
    const at = v.projectPart(p);
    const r = this.stage.getBoundingClientRect();
    const [x, y] = [at.x + r.left, at.y + r.top];
    if (this.mode === 'hunt') {
      v.selectPart(null);
      return this.#guess(p, x, y);
    }
    if (this.mode !== 'strip') return v.selectPart(null);
    v.selectPart(null);
    if (this.owner[p.id]) return this.#toast('info', `<b>${esc(p.name)}</b> viene via insieme a: ${esc(v.part(this.owner[p.id]).name)}.`);
    if (!this.stepIds.has(p.id)) return this.#toast('info', `<b>${esc(p.name)}</b> resta al suo posto in questa sfida.`);
    const blockers = v.blockersOf(p.id);
    if (blockers.length) {
      v.flashParts(blockers, 1.4);
      return this.#miss(`<b>${esc(p.name)}</b> è ancora fissato. Prima togli: ${blockers.map((b) => esc(b.name)).join(', ')}.`, p, x, y);
    }
    // Free but not grabbed: wrong tool.
    if (this.#need(p) !== this.tool)
      return this.#miss(this.#need(p) === 'wrench' ? `<b>${esc(p.name)}</b>: qui serve la <b>chiave</b>.` : `<b>${esc(p.name)}</b> si toglie con la <b>mano</b>.`, p, x, y);
  }

  #hover(p) {
    if (this.mode !== 'strip' || this.grab) return;
    this.viewer.showArrow(p && this.tool === 'hand' && this.#canGrab(p) ? p.id : null);
    this.stage.classList.toggle('can-grab', !!(p && this.#canGrab(p)));
  }

  // ---------- Bench ----------
  #benchAdd(part, pick) {
    const need = this.#need(part);
    const b = h(`<button class="bench-item" data-id="${esc(part.id)}" ${pick ? '' : 'tabindex="-1"'}>
        <span class="thumb">${this.thumbs?.[part.id] ? `<img src="${this.thumbs[part.id]}" alt="">` : ''}</span>
        <span class="nm">${esc(part.name)}</span><i class="need" title="${need === 'wrench' ? 'Chiave' : 'Mano'}">${GICON[need === 'wrench' ? 'bolt' : 'hand']}</i></button>`);
    if (pick) {
      b.addEventListener('click', (e) => this.#mount(part, e.clientX, e.clientY, b));
      b.addEventListener('pointerenter', () => this.viewer.previewPart(part.id));
      b.addEventListener('pointerleave', () => this.viewer.previewPart(null));
      b.addEventListener('focus', () => this.viewer.previewPart(part.id));
      b.addEventListener('blur', () => this.viewer.previewPart(null));
      this.el.list.append(b);
    } else {
      this.el.list.prepend(b);
    }
  }

  // ---------- Build: pick from the bench ----------
  #mount(part, x, y, btn) {
    if (this.mode !== 'build' || this.ended) return;
    const v = this.viewer;
    const blockers = v.mountBlockersOf(part.id);
    if (blockers.length) {
      for (const q of blockers) {
        const c = this.el.list.querySelector(`[data-id="${CSS.escape(q.id)}"]`);
        if (c) (c.classList.remove('flash'), void c.offsetWidth, c.classList.add('flash'));
      }
      return this.#miss(`Troppo presto per <b>${esc(part.name)}</b>. Prima rimonta: ${blockers.map((b) => esc(b.name)).join(', ')}.`, null, x, y);
    }
    if (this.#need(part) !== this.tool)
      return this.#miss(this.#need(part) === 'wrench' ? `<b>${esc(part.name)}</b>: per avvitare serve la <b>chiave</b>.` : `<b>${esc(part.name)}</b> si rimonta con la <b>mano</b>.`, null, x, y);
    v.previewPart(null);
    if (!v.mountPart(part.id)) return;
    btn.remove();
    sfx.clunk();
    if (this.#need(part) === 'wrench') [0.12, 0.24, 0.36].forEach((d) => setTimeout(() => sfx.ratchet(), d * 1000));
    this.#hit(part, x, y);
  }

  // ---------- Hunt ----------
  #nextRound() {
    clearTimeout(this.roundTimer);
    this.round++;
    if (this.round >= this.rounds.length) return this.#finish();
    this.tries = 0;
    this.roundT0 = performance.now();
    this.locked = false;
    this.#refresh();
    sfx.tick();
  }
  #guess(p, x, y) {
    if (this.locked) return;
    const target = this.rounds[this.round];
    if (p === target) {
      this.locked = true;
      const left = Math.max(0, HUNT_SECONDS - (performance.now() - this.roundT0) / 1000);
      this.viewer.flashParts([target], 1.1);
      sfx.pop();
      this.#hit(target, x, y, Math.round(left * 4));
      this.roundTimer = setTimeout(() => this.#nextRound(), 1500);
      return;
    }
    this.tries++;
    if (this.tries >= 3) return this.#reveal(`Quello è: <b>${esc(p.name)}</b>.`);
    this.#miss(`No, quello è: <b>${esc(p.name)}</b>. Riprova!`, p, x, y);
  }
  #reveal(prefix = '') {
    const target = this.rounds[this.round];
    this.locked = true;
    this.errors++;
    this.combo = 0;
    sfx.error();
    this.viewer.flashParts([target], 2.2);
    this.#toast('bad', `${prefix} <b>${esc(target.name)}</b> è il pezzo che lampeggia.`, 2600);
    this.#refresh();
    this.roundTimer = setTimeout(() => this.#nextRound(), 2500);
  }

  // ---------- Hints ----------
  #nextMoves() {
    const v = this.viewer;
    if (this.mode === 'strip') {
      const free = (p) => this.stepIds.has(p.id) && !p.out && !v.blockersOf(p.id).length;
      const todo = v.guideStatus.todo.filter(free);
      return todo.length ? todo : this.targets.filter(free);
    }
    if (this.mode === 'build') return v.history.map((g) => g[0]).filter((p) => !v.mountBlockersOf(p.id).length);
    return [];
  }
  hint() {
    if (!this.active || this.ended) return;
    const v = this.viewer;
    this.hints++;
    this.score = Math.max(0, this.score - 40);
    sfx.hint();
    if (this.mode === 'hunt') {
      if (this.locked) return;
      const t = this.rounds[this.round];
      this.#toast('info', `Indizio: ${esc(firstSentence(this.txt(t)))}`, 6000);
    } else {
      const next = this.#nextMoves().slice(0, 1);
      if (!next.length) return;
      const p = next[0];
      const tool = this.#need(p) === 'wrench' ? 'chiave' : 'mano';
      if (this.mode === 'strip') v.setHint(next);
      else {
        v.previewPart(p.id);
        const c = this.el.list.querySelector(`[data-id="${CSS.escape(p.id)}"]`);
        if (c) (c.classList.remove('flash'), void c.offsetWidth, c.classList.add('flash'), c.scrollIntoView({ block: 'nearest' }));
      }
      this.#toast('info', `Prossima mossa: <b>${esc(p.name)}</b> (${tool}).`, 4000);
      clearTimeout(this.hintTimer);
      this.hintTimer = setTimeout(() => this.#clearHint(), 4000);
    }
    this.#refresh();
  }
  #clearHint() {
    clearTimeout(this.hintTimer);
    if (!this.active) return;
    this.viewer.setHint([]);
    if (this.mode === 'build') this.viewer.previewPart(null);
  }

  // ---------- Progress, clock, end ----------
  #doneCount() {
    if (this.mode === 'hunt') return Math.max(0, this.round);
    const out = this.targets.filter((p) => p.out).length;
    return this.mode === 'strip' ? out : this.total - out;
  }
  #complete() {
    if (this.mode === 'hunt') return false; // rounds end by themselves
    return this.#doneCount() >= this.total;
  }

  #refresh() {
    if (!this.active) return;
    const done = this.#doneCount();
    this.el.bar.style.width = `${(done / Math.max(1, this.total)) * 100}%`;
    this.el.score.textContent = pts(this.score);
    const c = this.el.combo;
    c.querySelector('b').textContent = `×${Math.max(1, this.combo)}`;
    c.classList.toggle('hot', this.combo > 1);
    this.el.count.textContent = this.el.list.children.length || '';
    this.el.bench.classList.toggle('empty', !this.el.list.children.length);
    let text;
    if (this.mode === 'hunt') {
      const t = this.rounds[Math.min(this.round, this.rounds.length - 1)];
      text = `<small>${Math.min(this.round + 1, this.total)}/${this.total}</small> Trova: <em>${esc(t.name)}</em>`;
    } else if (this.mode === 'build') {
      text = `<small>${done}/${this.total}</small> Rimonta i pezzi dal banco`;
    } else {
      const { steps, index } = this.viewer.guideStatus;
      text = `<small>${done}/${this.total}</small> ${steps[index] ? esc(steps[index].name) : 'Fatto!'}`;
    }
    this.el.text.innerHTML = text;
  }

  #frame() {
    if (!this.active || this.ended) return;
    const now = performance.now();
    this.elapsed = (now - this.t0) / 1000;
    const label = clock(this.elapsed);
    if (label !== this.lastClock) this.el.time.textContent = this.lastClock = label;
    // Combo cools down.
    const left = this.combo > 1 ? Math.max(0, 1 - (now - this.lastHit) / 6500) : 0;
    this.el.combo.querySelector('i').style.transform = `scaleX(${left})`;
    if (this.combo > 1 && !left) {
      this.combo = 1;
      this.#refresh();
    }
    // Wrench: holding turns the screws.
    const g = this.grab;
    if (g && g.tool === 'wrench') {
      const dt = Math.min(0.05, (now - g.last) / 1000);
      g.last = now;
      const turns = this.def.disassembly.parts?.[g.part.id]?.turns ?? 5;
      g.progress = Math.min(1, g.progress + dt / (0.75 + turns * 0.07));
      this.viewer.setPull(g.progress);
      this.#ring(g.x, g.y, g.progress);
      const clicks = Math.floor(g.progress * 9);
      if (clicks > g.clicks) (g.clicks = clicks), sfx.ratchet();
      if (g.progress >= 1) this.#takeOff();
    }
    if (this.mode === 'hunt' && !this.locked) {
      const left = 1 - (now - this.roundT0) / 1000 / HUNT_SECONDS;
      this.el.bar.parentNode.style.setProperty('--round', Math.max(0, left));
      if (left <= 0) this.#reveal('Tempo scaduto!');
    }
  }

  #finish() {
    if (this.ended || !this.active) return;
    this.ended = true;
    const mode = this.mode;
    const time = this.elapsed;
    const bonus = Math.max(0, Math.round((this.par - time) * 5));
    const clean = this.errors === 0 ? 150 : 0;
    const score = this.score + bonus + clean;
    const stars = this.errors <= 1 && this.hints <= 1 && time <= this.par ? 3 : this.errors <= 4 && time <= this.par * 1.7 ? 2 : 1;
    const old = getRecord(this.def.id, mode);
    const isRecord = !this.dev && saveRecord(this.def.id, mode, { score, time, stars }); // dev: the test player keeps no records
    this.#clearHint();
    this.viewer.selectPart(null);
    const next = mode === 'strip' ? ['build', 'Rimonta il motore'] : mode === 'build' ? ['hunt', 'Trova il pezzo'] : this.hasRules ? ['strip', 'Smonta il motore'] : null;
    this.modal.dataset.kind = 'result';
    this.modal.innerHTML = `<div class="g-card result" role="dialog" aria-label="Risultato">
        <div class="res-stars">${starsHtml(stars)}</div>
        <span class="eyebrow">${esc(this.def.short || this.def.name)} · ${MODES[mode].name}</span>
        <h2>${MODES[mode].done}</h2>
        <div class="res-grid">
          <div><small>Tempo</small><b>${clock(time)}</b></div>
          <div><small>Errori</small><b>${this.errors}</b></div>
          <div><small>Aiuti</small><b>${this.hints}</b></div>
          <div><small>Bonus tempo</small><b>+${pts(bonus)}</b></div>
          ${clean ? `<div><small>Senza errori</small><b>+${clean}</b></div>` : ''}
          <div class="tot"><small>Punti</small><b>${pts(score)}</b></div>
        </div>
        <p class="res-rec ${isRecord ? 'new' : ''}">${isRecord ? (old ? `Nuovo record! Prima: ${pts(old.score)} punti.` : 'Primo record salvato!') : old ? `Il tuo record: ${pts(old.score)} punti ${'★'.repeat(old.stars)}` : ''}</p>
        <p class="res-tip">${stars === 3 ? 'Tre stelle: lavoro da capofficina.' : `Per le 3 stelle: al massimo 1 errore e 1 aiuto, entro ${clock(this.par)}.`}</p>
        <div class="cta">
          ${next ? `<button class="btn primary" data-go="${next[0]}">${next[1]}</button>` : ''}
          <button class="btn" data-go="${mode}">Riprova</button>
          <button class="btn ghost" data-go="">Esci</button>
        </div>
      </div>`;
    this.modal.hidden = false;
    sfx.win();
    for (let i = 0; i < stars; i++) setTimeout(() => sfx.star(i), 550 + i * 260);
    this.modal.querySelectorAll('[data-go]').forEach((b) =>
      b.addEventListener('click', () => {
        const go = b.dataset.go;
        this.modal.hidden = true;
        if (!go) return this.stop();
        // Straight from "Smonta" to "Rimonta": the parts are already on the bench.
        this.start(go, { keepParts: mode === 'strip' && go === 'build' });
      })
    );
    this.modal.querySelector('[data-go]').focus({ preventScroll: true });
  }

  // ---------- Small feedback ----------
  #toast(kind, html, ms = 3000) {
    const t = this.el.toast;
    t.className = `g-toast ${kind}`;
    t.innerHTML = html;
    t.hidden = false;
    t.style.animation = 'none';
    void t.offsetWidth;
    t.style.animation = '';
    clearTimeout(this.toastTimer);
    this.toastTimer = setTimeout(() => (t.hidden = true), ms);
  }
  #pop(text, x, y, cls = '') {
    const r = this.stage.getBoundingClientRect();
    const el = h(`<span class="g-pop ${cls}">${text}</span>`);
    el.style.left = `${Math.min(r.width - 40, Math.max(40, x - r.left))}px`;
    el.style.top = `${Math.min(r.height - 40, Math.max(60, y - r.top))}px`;
    this.el.pops.append(el);
    setTimeout(() => el.remove(), 1100);
  }
}
