import { EngineViewer } from './core/viewer.js';
import { FAMILIES, READY, loadEngine } from './catalog.js';
import { META } from './catalog-meta.js';

const app = document.getElementById('app');
let cleanup = () => {};

// ---------- Level (Base / Esperto) ----------
let level = 'base';
try {
  level = localStorage.getItem('motorilab-level') || 'base';
} catch {}
const levelListeners = new Set();
function setLevel(l) {
  level = l;
  try {
    localStorage.setItem('motorilab-level', l);
  } catch {}
  document.querySelectorAll('[data-level]').forEach((b) => b.setAttribute('aria-pressed', b.dataset.level === l));
  levelListeners.forEach((fn) => fn());
}
document.querySelectorAll('[data-level]').forEach((b) => b.addEventListener('click', () => setLevel(b.dataset.level)));
setLevel(level);
const txt = (o) => (o ? o[level] || o.base || '' : '');

// ---------- Helpers ----------
const h = (html) => {
  const t = document.createElement('template');
  t.innerHTML = html.trim();
  return t.content.firstElementChild;
};
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
const fmt = (v) => (typeof v === 'number' ? v.toLocaleString('it-IT') : v);
const ICON = {
  play: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg>',
  pause: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M7 5h4v14H7zM13 5h4v14h-4z"/></svg>',
  prev: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M15 5l-7 7 7 7"/></svg>',
  next: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M9 5l7 7-7 7"/></svg>',
  cut: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 3a9 9 0 1 0 0 18" /><path d="M12 3v18" stroke-dasharray="2 2.5"/><path d="M12 3a9 9 0 0 1 0 18" opacity=".35"/></svg>',
  tag: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 12V4h8l10 10-8 8z"/><circle cx="7.5" cy="8.5" r="1.5" fill="currentColor"/></svg>',
  down: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4"><path d="M6 9l6 6 6-6"/></svg>',
  closed: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="7" width="16" height="12" rx="2"/><path d="M8 7V4h8v3M4 12h16"/></svg>',
  explode: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="6" height="6" rx="1"/><path d="M4 4l3 3M20 4l-3 3M4 20l3-3M20 20l-3-3M4 4h3M4 4v3M20 4h-3M20 4v3M4 20h3M4 20v-3M20 20h-3M20 20v-3"/></svg>',
  info: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7.5v.5"/></svg>',
  zoom: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="11" cy="11" r="7"/><path d="M20 20l-4-4M8 11h6M11 8v6"/></svg>',
  tap: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3" fill="currentColor"/><circle cx="12" cy="12" r="8" stroke-dasharray="3 3"/></svg>',
  reset: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M4 12a8 8 0 1 0 2.4-5.7"/><path d="M4 4v4h4"/></svg>',
};
function setBgIfExists(el, url, prop) {
  const img = new Image();
  // Absolute URL: a relative one inside a CSS variable resolves against the stylesheet.
  img.onload = () => el.style.setProperty(prop, `url("${new URL(url, location.href).href}")`);
  img.src = url;
}
function familyOf(id) {
  return FAMILIES.find((f) => f.engines.some((e) => e.id === id));
}

// ---------- Router ----------
function route() {
  cleanup();
  cleanup = () => {};
  levelListeners.clear();
  window.scrollTo(0, 0);
  const [path, query = ''] = location.hash.replace(/^#\/?/, '').split('?');
  const parts = path.split('/');
  document.querySelectorAll('[data-nav]').forEach((a) => a.classList.remove('active'));
  const nav = (k) => document.querySelector(`[data-nav="${k}"]`)?.classList.add('active');
  if (parts[0] === 'motore' && parts[1]) return renderEngine(parts[1]);
  // Dev preview of an engine file not yet in the catalog: #/anteprima/<file name without .js>
  if (parts[0] === 'anteprima' && parts[1]) return renderEngine(parts[1], true);
  if (parts[0] === 'confronta') return nav('compare'), renderCompare();
  if (parts[0] === 'quiz') return nav('quiz'), renderQuiz();
  nav('home');
  renderHome(new URLSearchParams(query));
}
window.addEventListener('hashchange', route);
route();

// ---------- Home ----------
function renderHome(params = new URLSearchParams()) {
  app.innerHTML = '';
  const total = READY.length;
  const hero = h(`
    <section class="hero">
      <div>
        <div class="eyebrow">Laboratorio 3D interattivo</div>
        <h1>Guarda <em>dentro</em> i motori che muovono il mondo.</h1>
        <p>Auto, moto, aerei, razzi, treni e navi. Aprili, smontali e guarda cosa succede dentro.</p>
        <div class="cta">
          <a class="btn primary" href="#/motore/petrol4t">Inizia dal motore dell’auto</a>
          <a class="btn ghost" href="#esplora" data-scroll>Tutti i ${total} motori</a>
        </div>
        <div class="hero-stats">
          <div><b>${total}</b><span>motori in 3D</span></div>
          <div><b>${FAMILIES.length}</b><span>famiglie di mezzi</span></div>
          <div><b>${READY.length * 4}</b><span>domande di quiz</span></div>
        </div>
      </div>
      <div class="stage-hero"></div>
    </section>`);
  setBgIfExists(hero, 'assets/img/hero.png', '--hero-img');
  app.appendChild(hero);

  const fam0 = params.get('famiglia') || 'tutti';
  const explore = h(`<section class="explore" id="esplora">
      <div class="explore-head">
        <div>
          <h2>Esplora i motori</h2>
        </div>
        <label class="search"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="11" cy="11" r="7"/><path d="M20 20l-3.5-3.5"/></svg>
          <input type="search" placeholder="Cerca: diesel, elica, razzo…" aria-label="Cerca un motore"></label>
      </div>
      <div class="filters" role="tablist">
        <button data-f="tutti">Tutti <i>${total}</i></button>
        ${FAMILIES.map((f) => `<button data-f="${f.id}">${esc(f.name)} <i>${f.engines.filter((e) => e.load).length}</i></button>`).join('')}
      </div>
      <div class="groups"></div>
      <p class="empty" hidden>Nessun motore trovato. Prova con un’altra parola.</p>
    </section>`);
  app.appendChild(explore);
  app.appendChild(
    h(`<footer class="footer">MotoriLab — progetto didattico. Modelli 3D semplificati a scopo educativo; i valori sono tipici e indicativi.</footer>`)
  );

  const groups = explore.querySelector('.groups');
  const search = explore.querySelector('input');
  let filter = FAMILIES.some((f) => f.id === fam0) ? fam0 : 'tutti';
  const norm = (t) => (t || '').toLowerCase().normalize('NFD').replace(/[̀-ͯ]/g, '');

  function render() {
    explore.querySelectorAll('[data-f]').forEach((b) => b.setAttribute('aria-selected', b.dataset.f === filter));
    const q = norm(search.value.trim());
    let shown = 0;
    groups.innerHTML = '';
    for (const f of FAMILIES) {
      if (filter !== 'tutti' && f.id !== filter) continue;
      const list = f.engines.filter((e) => {
        if (!e.load) return false;
        if (!q) return true;
        const m = META[e.id] || {};
        return norm(`${e.short} ${e.vehicle} ${m.name} ${m.tagline} ${f.name}`).includes(q);
      });
      if (!list.length) continue;
      shown += list.length;
      const g = h(`<div class="group">
          <div class="group-head"><div class="cover"></div><h3>${esc(f.name)}</h3></div>
          <div class="cards"></div>
        </div>`);
      setBgIfExists(g.querySelector('.cover'), f.image, 'background-image', true);
      const cards = g.querySelector('.cards');
      for (const e of list) {
        const c = h(`<a class="card" href="#/motore/${e.id}">
            <div class="thumb"></div>
            <div class="card-body">
              <h4>${esc(e.short)}</h4>
              <small>${esc(e.vehicle)}</small>
            </div>
          </a>`);
        setBgIfExists(c.querySelector('.thumb'), `assets/thumbs/${e.id}.webp`, 'background-image', true);
        cards.appendChild(c);
      }
      groups.appendChild(g);
    }
    explore.querySelector('.empty').hidden = shown > 0;
  }
  explore.querySelectorAll('[data-f]').forEach((b) =>
    b.addEventListener('click', () => {
      filter = b.dataset.f;
      history.replaceState(null, '', filter === 'tutti' ? '#/' : `#/?famiglia=${filter}`);
      render();
    })
  );
  search.addEventListener('input', render);
  render();
  hero.querySelector('[data-scroll]').addEventListener('click', (e) => {
    e.preventDefault();
    explore.scrollIntoView({ behavior: 'smooth' });
  });
  if (params.get('famiglia')) requestAnimationFrame(() => explore.scrollIntoView());

  const viewer = new EngineViewer(hero.querySelector('.stage-hero'), { interactive: false, autoRotate: true });
  loadEngine('petrol4t').then((def) => {
    viewer.load(def);
    viewer.setCutaway(true, true);
    viewer.setThrottle(0.25);
  });
  cleanup = () => viewer.dispose();
}

// ---------- Engine page ----------
// Model first: the 3D view fills the page; controls float over it with icons and one-word labels;
// the long explanations live in a "Scopri di più" sheet that is closed by default.
const firstSentence = (html) => {
  const t = String(html).replace(/<[^>]+>/g, '').trim();
  const m = t.match(/^.{20,}?[.!?](?=\s|$)/);
  return m ? m[0] : t;
};
const isMobile = () => matchMedia('(max-width: 860px)').matches;
const isTouch = () => matchMedia('(hover: none)').matches;

async function renderEngine(id, preview = false) {
  app.innerHTML = `<div class="engine-page"><div class="stage main"><div class="loading">Caricamento motore…</div></div></div>`;
  let def;
  try {
    def = preview ? (await import(`./engines/${id}.js?t=${Date.now()}`)).default : await loadEngine(id);
  } catch (err) {
    console.error(err);
    app.innerHTML = `<div class="quiz-page"><h1>Motore non trovato</h1><p>${esc(err.message)}</p><a class="btn" href="#/">Torna alla home</a></div>`;
    return;
  }
  const page = app.querySelector('.engine-page');
  const stage = page.querySelector('.stage');
  stage.innerHTML = '';
  const fam = familyOf(def.id) || { name: def.family };
  document.title = `${def.name} – MotoriLab`;

  const viewer = new EngineViewer(stage);
  window.__viewer = viewer; // handy for debugging from the console
  viewer.load(def);
  viewer.setCutaway(true, true);
  cleanup = () => viewer.dispose();

  // ----- Overlays -----
  const idx = READY.findIndex((e) => e.id === def.id);
  const prev = READY[(idx - 1 + READY.length) % READY.length];
  const next = READY[(idx + 1) % READY.length];
  const switcher = FAMILIES.map(
    (f) =>
      `<optgroup label="${esc(f.name)}">${f.engines
        .filter((e) => e.load)
        .map((e) => `<option value="${e.id}" ${e.id === def.id ? 'selected' : ''}>${esc(e.short)} — ${esc(e.vehicle)}</option>`)
        .join('')}</optgroup>`
  ).join('');
  const top = h(`<header class="hud-top">
      <a class="crumb" href="#/?famiglia=${esc(def.family)}">${ICON.prev}<span>${esc(fam?.name || '')} · ${esc(def.vehicle)}</span></a>
      <div class="title-row">
        <label class="title-switch" title="Cambia motore">
          <h1 class="engine-title">${esc(def.name)}</h1>${ICON.down}
          <select class="engine-switch" aria-label="Cambia motore">${switcher}</select>
        </label>
        <a class="nav-arrow" href="#/motore/${prev.id}" title="Precedente: ${esc(prev.short)}" aria-label="Motore precedente">${ICON.prev}</a>
        <a class="nav-arrow" href="#/motore/${next.id}" title="Successivo: ${esc(next.short)}" aria-label="Motore successivo">${ICON.next}</a>
      </div>
    </header>`);
  const readout = h(`<div class="readout"></div>`);
  const tools = h(`<div class="hud-tools" role="toolbar" aria-label="Vista">
      <div class="tool-pair">
        <button class="tool" data-view="closed" aria-pressed="false" title="Il motore chiuso, com’è dal vero (V)">${ICON.closed}<span>Chiusa</span></button>
        <button class="tool" data-view="section" aria-pressed="true" title="Tagliato a metà per vedere dentro (V)">${ICON.cut}<span>Sezione</span></button>
      </div>
      <button class="tool" data-act="explode" aria-pressed="false" title="Separa i pezzi (E)">${ICON.explode}<span>Esploso</span></button>
      <button class="tool" data-act="labels" aria-pressed="false" title="Mostra i nomi dei pezzi (L)">${ICON.tag}<span>Nomi</span></button>
      <button class="tool" data-act="reset" title="Rimetti il motore al centro (R)">${ICON.reset}<span>Centra</span></button>
    </div>`);
  const bottom = h(`<div class="hud-bottom">
      <div class="part-card" hidden><button class="x" aria-label="Chiudi">×</button><h4></h4><p></p></div>
      <div class="caption">
        <button class="cap-nav" data-s="-1" aria-label="Fase precedente" title="Fase precedente (←)">${ICON.prev}</button>
        <div class="cap-body"><div class="dots" role="tablist" aria-label="Fasi"></div><b class="cap-name"></b><p class="cap-text"></p></div>
        <button class="cap-nav" data-s="1" aria-label="Fase successiva" title="Fase successiva (→)">${ICON.next}</button>
      </div>
      <div class="ctl-bar">
        <button class="play" data-act="play" aria-label="Pausa / riprendi" title="Pausa / riprendi (Spazio)">${ICON.pause}</button>
        <div class="thr">
          <div class="thr-top"><label for="thr">${esc(def.throttleLabel || 'Acceleratore')}</label><output class="g0"></output></div>
          <input id="thr" type="range" min="0" max="100" value="35">
        </div>
        <button class="info-btn" data-act="info" aria-expanded="false" title="Spiegazioni, dati e quiz (I)">${ICON.info}<span>Scopri di più</span></button>
      </div>
    </div>`);
  const tooltip = h(`<div class="tooltip" hidden></div>`);
  const labels = h(`<div class="labels" hidden></div>`);
  const touch = isTouch();
  const hints = h(`<div class="hints" hidden>
      <div>${ICON.reset}<span>${touch ? 'Trascina con un dito' : 'Trascina'} per girarlo</span></div>
      <div>${ICON.zoom}<span>${touch ? 'Due dita' : 'Rotella'} per lo zoom</span></div>
      <div>${ICON.tap}<span>${touch ? 'Tocca' : 'Clicca'} un pezzo per sapere cos’è</span></div>
    </div>`);
  const TABS = [
    ['overview', 'Come funziona'],
    ['phases', 'Fasi'],
    ['parts', 'Pezzi'],
    ['quiz', 'Quiz'],
  ];
  const sheet = h(`<aside class="sheet" aria-label="Scopri di più">
      <div class="sheet-head">
        <div class="tabs" role="tablist">${TABS.map(([k, n]) => `<button role="tab" data-tab="${k}">${n}</button>`).join('')}</div>
        <button class="x" aria-label="Chiudi">×</button>
      </div>
      <div class="panel-body"></div>
    </aside>`);
  stage.append(labels, top, readout, tools, tooltip, bottom, hints);
  page.append(sheet);
  top.querySelector('.engine-switch').addEventListener('change', (e) => (location.hash = `#/motore/${e.target.value}`));

  // ----- Layout: tools sit on the right on desktop, in the bottom stack on phones -----
  const caption = bottom.querySelector('.caption');
  let sheetOpen = false;
  const fitInsets = () => {
    const m = isMobile();
    if (m && tools.parentNode !== bottom) bottom.insertBefore(tools, caption);
    if (!m && tools.parentNode !== stage) stage.insertBefore(tools, tooltip);
    const sheetW = !m && sheetOpen ? sheet.offsetWidth : 0;
    page.style.setProperty('--sheet-w', `${sheetW}px`);
    viewer.setInsets(
      m
        ? { top: top.offsetTop + top.offsetHeight, bottom: bottom.offsetHeight + 8, right: 0 }
        : { top: top.offsetTop + top.offsetHeight, bottom: bottom.offsetHeight + 8, right: sheetW + tools.offsetWidth + 24 }
    );
  };
  const insetObserver = new ResizeObserver(fitInsets);
  insetObserver.observe(stage);
  let onKey = () => {};
  cleanup = () => (insetObserver.disconnect(), document.removeEventListener('keydown', onKey), viewer.dispose());

  // ----- Phases: dots + caption -----
  const dots = bottom.querySelector('.dots');
  def.phases.forEach((p, i) => {
    const b = h(`<button role="tab" title="${esc(p.name)}" aria-label="Fase ${i + 1}: ${esc(p.name)}"></button>`);
    b.addEventListener('click', () => (viewer.goToPhase(i), syncPlay(), renderCaption()));
    dots.appendChild(b);
  });
  const playBtn = bottom.querySelector('[data-act="play"]');
  const syncPlay = () => (playBtn.innerHTML = viewer.state.paused || viewer.stepMode ? ICON.play : ICON.pause);
  playBtn.addEventListener('click', () => {
    if (viewer.stepMode) viewer.exitStepMode();
    else viewer.togglePaused();
    syncPlay();
    renderCaption();
  });
  const step = (d) => {
    const base = viewer.phaseIndex < 0 ? (d > 0 ? -1 : 0) : viewer.phaseIndex;
    viewer.goToPhase(base + d);
    syncPlay();
    renderCaption();
  };
  bottom.querySelectorAll('.cap-nav').forEach((b) => b.addEventListener('click', () => step(+b.dataset.s)));

  const capName = bottom.querySelector('.cap-name');
  const capText = bottom.querySelector('.cap-text');
  function renderCaption() {
    const i = viewer.phaseIndex;
    const p = def.phases[i];
    [...dots.children].forEach((b, k) => (b.classList.toggle('on', k === i), b.setAttribute('aria-selected', k === i)));
    const still = viewer.stepMode || viewer.state.paused;
    caption.classList.toggle('reading', !!(still && p));
    if (!p) {
      capName.textContent = def.short;
      capText.innerHTML = 'Premi <b>›</b> per seguirlo una fase alla volta.';
      return;
    }
    capName.textContent = `${i + 1}. ${p.name}`;
    capText.innerHTML = still ? txt(p) : esc(firstSentence(txt(p)));
  }
  viewer.on('phase', () => {
    renderCaption();
    if (activeTab === 'phases') renderPanel();
  });

  // ----- Throttle -----
  const thr = bottom.querySelector('#thr');
  thr.addEventListener('input', () => viewer.setThrottle(thr.value / 100));
  viewer.setThrottle(thr.value / 100);

  // ----- View tools -----
  const viewBtns = [...tools.querySelectorAll('[data-view]')];
  const setView = (section) => {
    viewer.setCutaway(section);
    viewBtns.forEach((b) => b.setAttribute('aria-pressed', (b.dataset.view === 'section') === section));
  };
  viewBtns.forEach((b) => b.addEventListener('click', () => setView(b.dataset.view === 'section')));
  const expBtn = tools.querySelector('[data-act="explode"]');
  let exploded = false;
  const setExploded = (on) => {
    exploded = on;
    viewer.setExplode(on ? 1 : 0);
    expBtn.setAttribute('aria-pressed', on);
  };
  expBtn.addEventListener('click', () => setExploded(!exploded));
  tools.querySelector('[data-act="reset"]').addEventListener('click', () => viewer.resetCamera());

  // Part labels anchored in 3D
  const labelBtn = tools.querySelector('[data-act="labels"]');
  let labelEls = [];
  const buildLabels = () => {
    labels.innerHTML = '';
    labelEls = viewer.visibleParts.map((p) => {
      const el = h(`<button class="lbl">${esc(p.name)}</button>`);
      el.addEventListener('click', () => viewer.selectPart(p.id));
      labels.appendChild(el);
      return { p, el };
    });
  };
  const setLabels = (on) => {
    labels.hidden = !on;
    labelBtn.setAttribute('aria-pressed', on);
    if (on) buildLabels();
  };
  labelBtn.addEventListener('click', () => setLabels(labels.hidden));
  viewer.on('option', () => !labels.hidden && buildLabels());

  // ----- "Scopri di più" sheet -----
  const infoBtn = bottom.querySelector('[data-act="info"]');
  const setSheet = (open) => {
    sheetOpen = open;
    page.classList.toggle('sheet-open', open);
    infoBtn.setAttribute('aria-expanded', open);
    try {
      localStorage.setItem('motorilab-sheet', open ? '1' : '0');
    } catch {}
    fitInsets();
  };
  infoBtn.addEventListener('click', () => setSheet(!sheetOpen));
  sheet.querySelector('.x').addEventListener('click', () => setSheet(false));

  // ----- First visit: three quick hints instead of written instructions -----
  let hintsSeen = false;
  try {
    hintsSeen = localStorage.getItem('motorilab-hints') === '1';
  } catch {}
  if (!hintsSeen) {
    hints.hidden = false;
    const done = () => {
      hints.classList.add('out');
      setTimeout(() => (hints.hidden = true), 400);
      clearTimeout(timer);
      stage.removeEventListener('pointerdown', done, true);
      try {
        localStorage.setItem('motorilab-hints', '1');
      } catch {}
    };
    const timer = setTimeout(done, 7000);
    stage.addEventListener('pointerdown', done, true);
  }

  onKey = (e) => {
    if (e.target.closest('input, select, textarea') || e.ctrlKey || e.metaKey || e.altKey) return;
    const k = e.key.toLowerCase();
    if (k === ' ') playBtn.click();
    else if (k === 'arrowright') step(1);
    else if (k === 'arrowleft') step(-1);
    else if (k === 'v' || k === 's') setView(!viewer.cutaway);
    else if (k === 'e') setExploded(!exploded);
    else if (k === 'l') setLabels(labels.hidden);
    else if (k === 'r') viewer.resetCamera();
    else if (k === 'i' || k === '?') setSheet(!sheetOpen);
    else if (k === 'escape') sheetOpen ? setSheet(false) : viewer.selectPart(null);
    else if (k === '[') location.hash = `#/motore/${prev.id}`;
    else if (k === ']') location.hash = `#/motore/${next.id}`;
    else return;
    e.preventDefault();
  };
  document.addEventListener('keydown', onKey);

  // ----- Per-frame: labels, readouts -----
  const g0 = bottom.querySelector('.g0');
  let lastG = 0;
  viewer.on('frame', (s) => {
    if (!labels.hidden) {
      const cv = viewer.renderer.domElement;
      for (const { p, el } of labelEls) {
        const q = viewer.projectPart(p);
        const off = q.behind || q.x < cv.offsetLeft || q.y < cv.offsetTop || q.x > cv.offsetLeft + cv.clientWidth || q.y > cv.offsetTop + cv.clientHeight;
        el.style.display = off ? 'none' : '';
        if (!off) el.style.transform = `translate(${q.x}px, ${q.y}px) translate(-50%, -50%)`;
        el.classList.toggle('on', p === viewer.selected);
      }
    }
    const now = performance.now();
    if (now - lastG < 150) return;
    lastG = now;
    const gs = viewer.engine.readout(s);
    g0.textContent = gs[0] ? `${fmt(gs[0].value)} ${gs[0].unit || ''}` : '';
    readout.innerHTML = gs
      .slice(1)
      .map((g) => `<div><small>${esc(g.label)}</small><b>${fmt(g.value)}<i>${esc(g.unit || '')}</i></b></div>`)
      .join('');
  });

  // Hover tooltip (mouse only)
  if (!touch) {
    stage.addEventListener('pointermove', (e) => {
      const r = stage.getBoundingClientRect();
      tooltip.style.left = e.clientX - r.left + 'px';
      tooltip.style.top = e.clientY - r.top + 'px';
    });
    viewer.on('hover', (p) => {
      tooltip.hidden = !p;
      if (p) tooltip.textContent = p.name;
    });
  }

  // Selected part card
  const card = bottom.querySelector('.part-card');
  const renderCard = () => {
    const p = viewer.selected;
    card.hidden = !p;
    if (!p) return;
    card.querySelector('h4').textContent = p.name;
    card.querySelector('p').innerHTML = txt(p);
  };
  card.querySelector('.x').addEventListener('click', () => viewer.selectPart(null));
  viewer.on('select', () => {
    renderCard();
    if (activeTab === 'parts') renderPanel();
  });

  // ----- Sheet content -----
  let activeTab = 'overview';
  const body = sheet.querySelector('.panel-body');
  const tabBtns = [...sheet.querySelectorAll('[data-tab]')];
  const syncTabs = () => tabBtns.forEach((x) => x.setAttribute('aria-selected', x.dataset.tab === activeTab));
  tabBtns.forEach((b) =>
    b.addEventListener('click', () => {
      activeTab = b.dataset.tab;
      syncTabs();
      renderPanel();
      body.scrollTop = 0;
    })
  );
  syncTabs();
  const quizState = { i: 0, score: 0, answered: null };

  function renderPanel() {
    if (activeTab === 'overview') {
      body.innerHTML = `
        <p class="tagline">${esc(def.tagline)}</p>
        <p>${txt(def.intro)}</p>
        ${(def.options || [])
          .map((o) => {
            const cur = o.choices.find((c) => c.id === viewer.options[o.id]) || o.choices[0];
            return `<h3>${esc(o.label)}</h3>
              <div class="seg opt" role="group" aria-label="${esc(o.label)}">${o.choices
                .map((c) => `<button data-opt="${esc(o.id)}" data-val="${esc(c.id)}" aria-pressed="${c === cur}">${esc(c.label)}</button>`)
                .join('')}</div>
              <p class="opt-txt">${txt(cur)}</p>`;
          })
          .join('')}
        <h3>Dati tecnici</h3>
        <table class="specs">${def.specs.map((s) => `<tr><td>${esc(s.label)}</td><td>${esc(s.value)}</td></tr>`).join('')}</table>
        ${
          def.variants
            ? `<h3>Versioni reali</h3><div class="table-wrap"><table class="variants"><thead><tr>${def.variants.columns
                .map((c) => `<th>${esc(c)}</th>`)
                .join('')}</tr></thead><tbody>${def.variants.rows
                .map((r) => `<tr>${r.map((c) => `<td>${esc(c)}</td>`).join('')}</tr>`)
                .join('')}</tbody></table></div>${def.variants.note ? `<p class="note">${esc(def.variants.note)}</p>` : ''}`
            : ''
        }
        <h3>Dove lo trovi</h3><p class="note">${esc(def.examples || def.vehicle)}</p>
        <div class="kbd-help"><h3>Da tastiera</h3><p class="note">
          <kbd>Spazio</kbd> pausa · <kbd>←</kbd><kbd>→</kbd> fasi · <kbd>V</kbd> chiusa/sezione · <kbd>E</kbd> esploso ·
          <kbd>L</kbd> nomi · <kbd>R</kbd> centra · <kbd>I</kbd> questo pannello · <kbd>[</kbd><kbd>]</kbd> altro motore</p></div>`;
      body.querySelectorAll('[data-opt]').forEach((b) =>
        b.addEventListener('click', () => {
          viewer.setOption(b.dataset.opt, b.dataset.val);
          renderPanel();
        })
      );
    } else if (activeTab === 'phases') {
      body.innerHTML = `<ol class="phase-list">${def.phases
        .map(
          (p, i) => `<li class="${i === viewer.phaseIndex ? 'on' : ''}"><button data-i="${i}"><span class="n">${i + 1}</span><b>${esc(
            p.name
          )}</b></button><div class="txt">${txt(p)}</div></li>`
        )
        .join('')}</ol>`;
      body.querySelectorAll('[data-i]').forEach((b) => b.addEventListener('click', () => (viewer.goToPhase(+b.dataset.i), syncPlay(), renderCaption())));
    } else if (activeTab === 'parts') {
      body.innerHTML = `<p class="note">Tocca un pezzo qui o direttamente sul modello.</p>
        <ul class="part-list">${viewer.visibleParts
          .map(
            (p) => `<li class="${p === viewer.selected ? 'on' : ''}"><button data-id="${esc(p.id)}"><b>${esc(p.name)}</b></button><div class="txt">${txt(p)}</div></li>`
          )
          .join('')}</ul>`;
      body.querySelectorAll('[data-id]').forEach((b) =>
        b.addEventListener('click', () => viewer.selectPart(viewer.selected?.id === b.dataset.id ? null : b.dataset.id))
      );
    } else if (activeTab === 'quiz') {
      renderQuizInto(body, def.quiz, quizState, () => renderPanel(), def.short);
    }
  }
  renderPanel();
  renderCaption();
  let wantSheet = false;
  try {
    wantSheet = localStorage.getItem('motorilab-sheet') === '1' && !isMobile();
  } catch {}
  if (wantSheet) setSheet(true);
  else fitInsets();
  levelListeners.add(() => {
    renderPanel();
    renderCaption();
    renderCard();
  });
}

// ---------- Quiz (shared) ----------
function renderQuizInto(el, questions, st, rerender, tagFor) {
  if (st.i >= questions.length) {
    el.innerHTML = `<div class="score"><div class="quiz-meta"><span>Risultato</span></div>
      <div class="big">${st.score}/${questions.length}</div>
      <p>${st.score === questions.length ? 'Perfetto! Sei pronto per il prossimo motore.' : st.score >= questions.length / 2 ? 'Bene! Riguarda le fasi per fare il pieno.' : 'Rivedi le fasi passo-passo e riprova.'}</p>
      <button class="btn primary" data-restart>Ricomincia</button></div>`;
    el.querySelector('[data-restart]').addEventListener('click', () => {
      Object.assign(st, { i: 0, score: 0, answered: null });
      st.onRestart?.();
      rerender();
    });
    return;
  }
  const q = questions[st.i];
  const tag = typeof tagFor === 'function' ? tagFor(q) : tagFor;
  el.innerHTML = `
    <div class="quiz-meta"><span>Domanda ${st.i + 1} di ${questions.length}</span><span>Punti ${st.score}</span></div>
    <div class="quiz-bar"><i style="width:${(st.i / questions.length) * 100}%"></i></div>
    ${tag ? `<span class="quiz-tag">${esc(tag)}</span>` : ''}
    <p class="quiz-q">${esc(q.q)}</p>
    <div class="quiz-opts">${q.options.map((o, i) => `<button data-o="${i}">${esc(o)}</button>`).join('')}</div>
    <div class="after"></div>`;
  const btns = [...el.querySelectorAll('[data-o]')];
  const show = (choice) => {
    btns.forEach((b, i) => {
      b.disabled = true;
      if (i === q.correct) b.classList.add('right');
      else if (i === choice) b.classList.add('wrong');
    });
    el.querySelector('.after').innerHTML = `<div class="quiz-explain">${
      choice === q.correct ? '<b>Esatto!</b> ' : '<b>Non proprio.</b> '
    }${esc(q.explain)}</div><div class="cta"><button class="btn primary" data-next>${st.i + 1 < questions.length ? 'Avanti' : 'Vedi risultato'}</button></div>`;
    el.querySelector('[data-next]').addEventListener('click', () => {
      st.i++;
      st.answered = null;
      rerender();
    });
  };
  if (st.answered != null) show(st.answered);
  btns.forEach((b) =>
    b.addEventListener('click', () => {
      const c = +b.dataset.o;
      st.answered = c;
      if (c === q.correct) st.score++;
      show(c);
      el.querySelector('.quiz-meta span:last-child').textContent = `Punti ${st.score}`;
    })
  );
}

async function renderQuiz() {
  app.innerHTML = `<div class="quiz-page"><h1>Quiz dei motori</h1><p>Domande miste su tutti i motori disponibili. Dopo ogni risposta trovi la spiegazione.</p><div class="quiz-card">Caricamento…</div></div>`;
  const defs = await Promise.all(READY.map((e) => loadEngine(e.id).catch(() => null)));
  const cardEl = app.querySelector('.quiz-card');
  if (!cardEl) return;
  const build = () => {
    const all = defs.filter(Boolean).flatMap((d) => d.quiz.map((q) => ({ ...q, _tag: d.short })));
    for (let i = all.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [all[i], all[j]] = [all[j], all[i]];
    }
    return all.slice(0, 10);
  };
  let qs = build();
  const st = { i: 0, score: 0, answered: null, onRestart: () => (qs = build()) };
  const rerender = () => renderQuizInto(cardEl, qs, st, rerender, (q) => q._tag);
  rerender();
}

// ---------- Compare ----------
async function renderCompare() {
  app.innerHTML = `<div class="compare-page">
      <h1>Confronta due motori</h1>
      <p>Scegli due motori e guardali lavorare fianco a fianco. Le barre usano valori tipici di un esemplare reale.</p>
      <div class="compare-grid"></div>
      <div class="metrics"><h2>A confronto</h2><p>Potenza e potenza specifica sono in scala logaritmica: le differenze tra un’auto e un razzo sono enormi!</p><div class="metric-list"></div></div>
    </div>`;
  const grid = app.querySelector('.compare-grid');
  const viewers = [];
  const picked = [READY[0]?.id, READY[1]?.id];
  const defs = [null, null];
  const cols = picked.map((id, k) => {
    const col = h(`<div class="compare-col">
        <select aria-label="Motore ${k + 1}">${READY.map((e) => `<option value="${e.id}" ${e.id === id ? 'selected' : ''}>${esc(e.short)} — ${esc(e.vehicle)}</option>`).join('')}</select>
        <div class="stage"></div>
        <div class="throttle-mini"><span class="thr-label">Acceleratore</span><input type="range" min="0" max="100" value="45">
          <div class="seg"><button data-v="closed" aria-pressed="false">Chiusa</button><button data-v="section" aria-pressed="true">Sezione</button></div></div>
        <table class="specs"></table>
      </div>`);
    grid.appendChild(col);
    const v = new EngineViewer(col.querySelector('.stage'), { autoRotate: true });
    viewers.push(v);
    const thr = col.querySelector('input');
    thr.addEventListener('input', () => v.setThrottle(thr.value / 100));
    col.querySelectorAll('[data-v]').forEach((b) =>
      b.addEventListener('click', () => {
        v.setCutaway(b.dataset.v === 'section');
        col.querySelectorAll('[data-v]').forEach((x) => x.setAttribute('aria-pressed', x === b));
      })
    );
    const sel = col.querySelector('select');
    const load = async () => {
      const def = await loadEngine(sel.value);
      defs[k] = def;
      v.load(def);
      v.setCutaway(col.querySelector('[data-v="section"]').getAttribute('aria-pressed') === 'true', true);
      v.setThrottle(thr.value / 100);
      col.querySelector('.thr-label').textContent = def.throttleLabel || 'Acceleratore';
      col.querySelector('.specs').innerHTML = def.specs.map((s) => `<tr><td>${esc(s.label)}</td><td>${esc(s.value)}</td></tr>`).join('');
      renderMetrics();
    };
    sel.addEventListener('change', load);
    return { load };
  });
  cleanup = () => viewers.forEach((v) => v.dispose());

  const METRICS = [
    { key: 'efficiency', name: 'Rendimento', sub: 'energia trasformata in lavoro utile', unit: '%', log: false, max: 100 },
    { key: 'power', name: 'Potenza tipica', sub: 'di un singolo motore', unit: 'kW', log: true },
    { key: 'powerDensity', name: 'Potenza specifica', sub: 'kW per ogni kg di motore', unit: 'kW/kg', log: true },
  ];
  function renderMetrics() {
    if (!defs[0] || !defs[1]) return;
    const list = app.querySelector('.metric-list');
    list.innerHTML = METRICS.map((m) => {
      const vals = defs.map((d) => d.metrics?.[m.key] ?? 0);
      let pct;
      if (m.log) {
        const lo = 0.1;
        const hi = Math.max(...vals, 1) * 1.2;
        pct = vals.map((v) => (Math.log10(Math.max(v, lo) / lo) / Math.log10(hi / lo)) * 100);
      } else pct = vals.map((v) => (v / m.max) * 100);
      return `<div class="metric"><div class="name">${m.name}<small>${m.sub}</small></div><div class="bars">${defs
        .map(
          (d, i) =>
            `<div class="bar ${i ? 'b' : 'a'}"><div class="track"><div class="fill" style="width:${pct[i].toFixed(1)}%"></div></div><span>${fmt(vals[i])} ${m.unit}</span></div>`
        )
        .join('')}</div></div>`;
    }).join('');
  }
  await Promise.all(cols.map((c) => c.load()));
}
