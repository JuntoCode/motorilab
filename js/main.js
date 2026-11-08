import { EngineViewer } from './core/viewer.js';
import { FAMILIES, READY, loadEngine } from './catalog.js';
import { META } from './catalog-meta.js';
import { Workshop, GICON, starsHtml } from './game/workshop.js';
import { engineStars, totalStars, MODES as GAME_MODES } from './game/records.js';

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
  lift: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="6" y="13" width="12" height="7" rx="1.5"/><path d="M12 10V3M8.5 6.5L12 3l3.5 3.5"/></svg>',
  undo: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 14L4 9l5-5"/><path d="M4 9h10a6 6 0 0 1 0 12h-3"/></svg>',
  screw: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linejoin="round"><path d="M7 3h10v4H7z"/><path d="M10 7v13l2 2 2-2V7M10 11l4-1.5M10 15l4-1.5"/><path d="M19 4a3 3 0 0 1 0 4" stroke-linecap="round"/></svg>',
  restore: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="6" y="13" width="12" height="7" rx="1.5"/><path d="M12 3v7M8.5 6.5L12 10l3.5-3.5"/></svg>',
  wrench: GICON.wrench,
  reset: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M4 12a8 8 0 1 0 2.4-5.7"/><path d="M4 4v4h4"/></svg>',
  search: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="11" cy="11" r="7"/><path d="M20 20l-3.5-3.5"/></svg>',
  arrow: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M5 12h14M13 6l6 6-6 6"/></svg>',
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
const MAX_STARS = GAME_MODES.length * 3;

// ---------- Router ----------
function route() {
  cleanup();
  cleanup = () => {};
  levelListeners.clear();
  window.scrollTo(0, 0);
  const [path, query = ''] = location.hash.replace(/^#\/?/, '').split('?');
  const parts = path.split('/');
  const params = new URLSearchParams(query);
  document.body.classList.remove('on-engine');
  document.querySelectorAll('[data-nav]').forEach((a) => a.classList.remove('active'));
  const nav = (k) => document.querySelector(`[data-nav="${k}"]`)?.classList.add('active');
  if (parts[0] === 'motore' && parts[1]) return renderEngine(parts[1], false, params);
  // Dev preview of an engine file not yet in the catalog: #/anteprima/<file name without .js>
  if (parts[0] === 'anteprima' && parts[1]) return renderEngine(parts[1], true, params);
  if (parts[0] === 'confronta') return nav('compare'), renderCompare();
  if (parts[0] === 'quiz') return nav('quiz'), renderQuiz();
  nav('home');
  renderHome(params);
}
window.addEventListener('hashchange', route);
route();

// ---------- Home ----------
function renderHome(params = new URLSearchParams()) {
  app.innerHTML = '';
  const total = READY.length;
  const stars = totalStars(READY.map((e) => e.id));
  const maxStars = total * MAX_STARS;
  const hero = h(`
    <section class="hero">
      <div class="hero-copy">
        <div class="eyebrow">Laboratorio 3D interattivo</div>
        <h1>Guarda <em>dentro</em> i motori.<br>Poi <em>smontali</em>.</h1>
        <p>Auto, moto, aerei, razzi, treni e navi: ${total} motori da aprire in sezione, far girare al rallentatore e smontare pezzo per pezzo, con gli attrezzi in mano e il cronometro che corre.</p>
        <div class="cta">
          <a class="btn primary" href="#/motore/petrol4t">Apri il motore dell’auto ${ICON.arrow}</a>
          <a class="btn hazard" href="#/motore/petrol4t?gioca=menu">${GICON.wrench} Sfida in officina</a>
        </div>
        <dl class="hero-stats">
          <div><dt>${total}</dt><dd>motori in 3D</dd></div>
          <div><dt>3</dt><dd>sfide per motore</dd></div>
          <div><dt>${stars}<small>/${maxStars}</small></dt><dd>stelle conquistate</dd></div>
        </dl>
      </div>
      <div class="stage-hero"><span class="hero-tag">Benzina 4 tempi · in sezione</span></div>
    </section>`);
  app.appendChild(hero);

  app.appendChild(
    h(`<section class="how">
      <a class="how-card" href="#esplora" data-scroll><span class="how-n">01</span>${ICON.cut}<h3>Aprilo</h3><p>Sezione, esploso e fasi al rallentatore: vedi cosa succede dentro mentre gira.</p></a>
      <a class="how-card" href="#/motore/petrol4t?gioca=strip"><span class="how-n">02</span>${GICON.wrench}<h3>Smontalo</h3><p>Mano e chiave: trascina i pezzi, svita le viti, rispetta l’ordine. Ogni errore costa punti.</p></a>
      <a class="how-card" href="#/motore/petrol4t?gioca=build"><span class="how-n">03</span>${GICON.puzzle}<h3>Rimontalo</h3><p>I pezzi sono sul banco, alla rinfusa. Sai rimetterli al loro posto?</p></a>
      <a class="how-card" href="#/motore/turbofan?gioca=hunt"><span class="how-n">04</span>${GICON.target}<h3>Riconoscilo</h3><p>Trova il pezzo giusto sul modello prima che scada il tempo.</p></a>
    </section>`)
  );

  const fam0 = params.get('famiglia') || 'tutti';
  const explore = h(`<section class="explore" id="esplora">
      <div class="explore-head">
        <div>
          <div class="eyebrow">Il garage</div>
          <h2>Scegli un motore</h2>
        </div>
        <div class="garage" title="Stelle conquistate nelle sfide">
          <div class="garage-top"><span>Patentino del meccanico</span><b>${stars} / ${maxStars} ${GICON.star}</b></div>
          <div class="garage-bar"><i style="width:${(stars / maxStars) * 100}%"></i></div>
        </div>
      </div>
      <div class="explore-tools">
        <div class="filters" role="tablist">
          <button data-f="tutti">Tutti <i>${total}</i></button>
          ${FAMILIES.map((f) => `<button data-f="${f.id}">${esc(f.name)} <i>${f.engines.filter((e) => e.load).length}</i></button>`).join('')}
        </div>
        <label class="search">${ICON.search}
          <input type="search" placeholder="Cerca: diesel, elica, razzo…" aria-label="Cerca un motore"></label>
      </div>
      <div class="groups"></div>
      <p class="empty" hidden>Nessun motore trovato. Prova con un’altra parola.</p>
    </section>`);
  app.appendChild(explore);
  app.appendChild(
    h(`<footer class="footer"><span>Motori<b>Lab</b></span> Progetto didattico. Modelli 3D semplificati a scopo educativo; i valori sono tipici e indicativi, le sequenze di smontaggio sono semplificate.</footer>`)
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
          <div class="group-head"><div class="cover"></div><div><h3>${esc(f.name)}</h3><p>${esc(f.desc)}</p></div></div>
          <div class="cards"></div>
        </div>`);
      setBgIfExists(g.querySelector('.cover'), f.image, 'background-image');
      const cards = g.querySelector('.cards');
      for (const e of list) {
        const st = engineStars(e.id);
        const c = h(`<a class="card" href="#/motore/${e.id}">
            <div class="thumb"></div>
            <div class="card-body">
              <h4>${esc(e.short)}</h4>
              <small>${esc(e.vehicle)}</small>
              <p>${esc((META[e.id] || {}).tagline || '')}</p>
              <div class="card-foot"><span class="card-stars ${st ? 'has' : ''}">${GICON.star}${st}/${MAX_STARS}</span><span class="card-go">Apri ${ICON.arrow}</span></div>
            </div>
          </a>`);
        setBgIfExists(c.querySelector('.thumb'), `assets/thumbs/${e.id}.webp`, 'background-image');
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
  app.querySelectorAll('[data-scroll]').forEach((a) =>
    a.addEventListener('click', (e) => {
      e.preventDefault();
      explore.scrollIntoView({ behavior: 'smooth' });
    })
  );
  if (params.get('famiglia')) requestAnimationFrame(() => explore.scrollIntoView());

  const viewer = new EngineViewer(hero.querySelector('.stage-hero'), { interactive: false, autoRotate: true, fx: true });
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
// The Officina button opens the workshop games (js/game/workshop.js), played on the same model.
const firstSentence = (html) => {
  const t = String(html).replace(/<[^>]+>/g, '').trim();
  const m = t.match(/^.{20,}?[.!?](?=\s|$)/);
  return m ? m[0] : t;
};
const isMobile = () => matchMedia('(max-width: 860px)').matches;
const isTouch = () => matchMedia('(hover: none)').matches;

async function renderEngine(id, preview = false, params = new URLSearchParams()) {
  document.body.classList.add('on-engine');
  app.innerHTML = `<div class="engine-page"><div class="stage main"><div class="loading"><i></i>Caricamento motore…</div></div></div>`;
  let def;
  try {
    def = preview ? (await import(`./engines/${id}.js?t=${Date.now()}`)).default : await loadEngine(id);
  } catch (err) {
    console.error(err);
    document.body.classList.remove('on-engine');
    app.innerHTML = `<div class="quiz-page"><h1>Motore non trovato</h1><p>${esc(err.message)}</p><a class="btn" href="#/">Torna alla home</a></div>`;
    return;
  }
  const page = app.querySelector('.engine-page');
  if (!page) return; // navigated away while loading
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
      <button class="tool play-tool" data-act="workshop" aria-pressed="false" title="Officina: guida e sfide di smontaggio (O)">${ICON.wrench}<span>Officina</span></button>
    </div>`);
  const bottom = h(`<div class="hud-bottom">
      <div class="part-card" hidden><button class="x" aria-label="Chiudi">×</button><h4></h4><p></p>
        <div class="card-acts"><button class="act" data-act="remove" title="Togli questo pezzo (Canc)"></button></div>
        <div class="card-msg" hidden></div>
      </div>
      <div class="removed-bar" hidden><span class="n"></span><button class="act icon" data-act="undo" title="Rimetti l’ultimo pezzo (U)" aria-label="Rimetti l’ultimo pezzo">${ICON.undo}</button><button class="act" data-act="restore" title="Rimetti tutti i pezzi tolti">${ICON.restore}<span>Rimetti tutto</span></button></div>
      <div class="caption">
        <button class="cap-nav" data-s="-1" aria-label="Fase precedente" title="Fase precedente (←)">${ICON.prev}</button>
        <div class="cap-body"><div class="phase-steps" role="tablist" aria-label="Fasi"></div><div class="ws-dots" role="tablist" aria-label="Passi" hidden></div><b class="cap-name"></b><p class="cap-text"></p></div>
        <button class="cap-nav" data-s="1" aria-label="Fase successiva" title="Fase successiva (→)">${ICON.next}</button>
      </div>
      <div class="ctl-bar">
        <button class="play" data-act="play" aria-label="Pausa / riprendi" title="Pausa / riprendi (Spazio)">${ICON.pause}</button>
        <div class="thr">
          <div class="thr-top"><label for="thr">${esc(def.throttleLabel || 'Acceleratore')}</label><output class="g0"></output></div>
          <input id="thr" type="range" min="0" max="100" value="35">
        </div>
        <div class="ws-info" hidden><b></b><small></small></div>
        <button class="info-btn" data-act="info" aria-expanded="false" title="Spiegazioni, dati e quiz (I)">${ICON.info}<span>Scopri di più</span></button>
      </div>
    </div>`);
  const tooltip = h(`<div class="tooltip" hidden></div>`);
  const ctxMenu = h(`<div class="ctx-menu" role="menu" hidden></div>`);
  const labels = h(`<div class="labels" hidden></div>`);
  const touch = isTouch();
  const hints = h(`<div class="hints" hidden>
      <div>${ICON.reset}<span>${touch ? 'Trascina con un dito' : 'Trascina'} per girarlo</span></div>
      <div>${ICON.zoom}<span>${touch ? 'Due dita' : 'Rotella'} per lo zoom</span></div>
      <div>${ICON.tap}<span>${touch ? 'Tocca' : 'Clicca'} un pezzo per sapere cos’è</span></div>
      <div>${ICON.wrench}<span>Officina: smontalo a tempo</span></div>
    </div>`);
  const TABS = [
    ['overview', 'Come funziona'],
    ['phases', 'Fasi'],
    ['parts', 'Pezzi'],
    ...(def.disassembly ? [['workshop', 'Officina']] : []),
    ['quiz', 'Quiz'],
  ];
  const sheet = h(`<aside class="sheet" aria-label="Scopri di più">
      <div class="sheet-head">
        <div class="tabs" role="tablist">${TABS.map(([k, n]) => `<button role="tab" data-tab="${k}">${n}</button>`).join('')}</div>
        <button class="x" aria-label="Chiudi">×</button>
      </div>
      <div class="panel-body"></div>
    </aside>`);
  stage.append(labels, top, readout, tools, tooltip, bottom, hints, ctxMenu);
  page.append(sheet);
  top.querySelector('.engine-switch').addEventListener('change', (e) => (location.hash = `#/motore/${e.target.value}`));

  // ----- Layout: tools sit on the right on desktop, in the bottom stack on phones -----
  const caption = bottom.querySelector('.caption');
  let sheetOpen = false;
  let ws = null; // the workshop games, created below
  const fitInsets = () => {
    const m = isMobile();
    if (m && tools.parentNode !== bottom) bottom.insertBefore(tools, caption);
    if (!m && tools.parentNode !== stage) stage.insertBefore(tools, tooltip);
    const sheetW = !m && sheetOpen ? sheet.offsetWidth : 0;
    page.style.setProperty('--sheet-w', `${sheetW}px`);
    if (ws?.active) {
      // In a game the HUD is the top strip, the bench and the tool belt.
      const s = stage.getBoundingClientRect();
      const gTop = ws.hud.querySelector('.g-top').getBoundingClientRect();
      const belt = ws.el.belt.getBoundingClientRect();
      const bench = ws.el.bench.getBoundingClientRect();
      const benchOn = bench.width > 0;
      viewer.setInsets(
        m
          ? { top: gTop.bottom - s.top, bottom: s.bottom - Math.min(belt.top, benchOn ? bench.top : belt.top) + 8 }
          : { top: gTop.bottom - s.top, bottom: s.bottom - belt.top + 8, left: benchOn ? bench.right - s.left + 8 : 0, right: tools.offsetWidth + 24 }
      );
      return;
    }
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

  // ----- Phases: numbered steps + caption -----
  const steps = bottom.querySelector('.phase-steps');
  def.phases.forEach((p, i) => {
    const b = h(`<button role="tab" title="${esc(p.name)}" aria-label="Fase ${i + 1}: ${esc(p.name)}"><i>${i + 1}</i><span>${esc(p.name)}</span></button>`);
    b.addEventListener('click', () => (viewer.goToPhase(i), syncPlay(), renderCaption()));
    steps.appendChild(b);
  });
  steps.classList.toggle('many', def.phases.length > 5);
  const playBtn = bottom.querySelector('[data-act="play"]');
  const guideDone = () => viewer.guideStatus.index >= viewer.guideStatus.steps.length;
  const syncPlay = () => {
    if (viewer.guide) {
      const auto = viewer.autoplay !== 0;
      playBtn.innerHTML = auto ? ICON.pause : guideDone() ? ICON.restore : ICON.play;
      playBtn.title = auto ? 'Ferma (Spazio)' : guideDone() ? 'Rimonta tutto (Spazio)' : 'Smonta tutto da solo (Spazio)';
      return;
    }
    playBtn.title = 'Pausa / riprendi (Spazio)';
    playBtn.innerHTML = viewer.state.paused || viewer.stepMode ? ICON.play : ICON.pause;
  };
  playBtn.addEventListener('click', () => {
    if (viewer.guide) {
      if (viewer.autoplay) viewer.stopAuto();
      else if (guideDone()) viewer.restoreAll();
      else viewer.autoDisassemble();
      syncPlay();
      return;
    }
    if (viewer.stepMode) viewer.exitStepMode();
    else viewer.togglePaused();
    syncPlay();
    renderCaption();
  });
  const step = (d) => {
    if (viewer.guide) {
      if (viewer.autoplay) viewer.stopAuto();
      if (d > 0) viewer.guideDemoNext();
      else viewer.undo();
      return;
    }
    const base = viewer.phaseIndex < 0 ? (d > 0 ? -1 : 0) : viewer.phaseIndex;
    viewer.goToPhase(base + d);
    syncPlay();
    renderCaption();
  };
  bottom.querySelectorAll('.cap-nav').forEach((b) => b.addEventListener('click', () => step(+b.dataset.s)));

  const capName = bottom.querySelector('.cap-name');
  const capText = bottom.querySelector('.cap-text');
  const wsDots = bottom.querySelector('.ws-dots');
  function renderCaption() {
    if (viewer.guide) {
      const { steps: gs, index, todo } = viewer.guideStatus;
      [...wsDots.children].forEach((b, n) => (b.classList.toggle('done', n < index), b.classList.toggle('on', n === index)));
      caption.classList.add('reading');
      const st = gs[index];
      if (!st) {
        capName.textContent = 'Fatto!';
        capText.innerHTML = 'Smontaggio completo. Con <b>‹</b> rimonti un pezzo alla volta, con il tasto arancione rimonti tutto. Pronto per la <b>sfida a tempo</b>?';
        return;
      }
      const names = todo.map((p) => esc(p.name)).join(', ');
      capName.textContent = `${index + 1}. ${st.name}`;
      capText.innerHTML = `${txt(st)} <span class="cap-hint">Tocca: ${names}.</span>`;
      return;
    }
    const i = viewer.phaseIndex;
    const p = def.phases[i];
    [...steps.children].forEach((b, k) => (b.classList.toggle('on', k === i), b.setAttribute('aria-selected', k === i)));
    const still = viewer.stepMode || viewer.state.paused;
    caption.classList.toggle('reading', !!(still && p));
    if (!p) {
      capName.textContent = def.short;
      capText.innerHTML = 'Premi <b>›</b> per seguirlo una fase alla volta.';
      return;
    }
    capName.textContent = p.name;
    capText.innerHTML = still ? txt(p) : esc(firstSentence(txt(p)));
  }
  viewer.on('phase', () => {
    renderCaption();
    if (activeTab === 'phases') renderPanel();
  });

  // ----- Throttle -----
  const thr = bottom.querySelector('#thr');
  const paintThr = () => thr.style.setProperty('--v', `${thr.value}%`);
  thr.addEventListener('input', () => (viewer.setThrottle(thr.value / 100), paintThr()));
  viewer.setThrottle(thr.value / 100);
  paintThr();

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
    on = on && !ws?.active;
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

  // ----- First visit: quick hints instead of written instructions -----
  let hintsSeen = false;
  try {
    hintsSeen = localStorage.getItem('motorilab-hints') === '1';
  } catch {}
  if (!hintsSeen && !params.get('gioca')) {
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
    if (ws.key(k)) return e.preventDefault();
    if (ws.active && k === ' ') return e.preventDefault();
    if (k === ' ') playBtn.click();
    else if (k === 'arrowright') step(1);
    else if (k === 'arrowleft') step(-1);
    else if (k === 'v' || k === 's') setView(!viewer.cutaway);
    else if (k === 'e') setExploded(!exploded);
    else if (k === 'l') setLabels(labels.hidden);
    else if (k === 'r') viewer.resetCamera();
    else if (k === 'i' || k === '?') setSheet(!sheetOpen);
    else if (k === 'o') wsBtn.click();
    else if (k === 'u') viewer.undo();
    else if (k === 'escape') !ctxMenu.hidden ? closeMenu() : viewer.guide ? setGuide(false) : sheetOpen ? setSheet(false) : viewer.selectPart(null);
    else if ((k === 'delete' || k === 'backspace') && viewer.selected) viewer.removePart(viewer.selected.id);
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
      tooltip.hidden = !p || ws.hidesNames;
      if (p) tooltip.textContent = p.name;
    });
  }

  // Selected part card
  const card = bottom.querySelector('.part-card');
  const renderCard = () => {
    const p = viewer.selected;
    card.hidden = !p || ws.active;
    if (card.hidden) return;
    card.querySelector('h4').textContent = p.name;
    card.querySelector('p').innerHTML = txt(p);
    const screw = viewer.motionOf(p.id) === 'unscrew';
    removeBtn.innerHTML = `${screw ? ICON.screw : ICON.lift}<span>${screw ? 'Svita' : 'Togli'}</span>`;
    cardMsg.hidden = true; // the "take off first" note is shown again by the next 'blocked'
  };
  const removeBtn = card.querySelector('[data-act="remove"]');
  const cardMsg = card.querySelector('.card-msg');
  // Tried to take off a part that is still held: say what to take off first; tapping a name selects it.
  viewer.on('blocked', ({ part, blockers }) => {
    if (ws.active) return;
    if (viewer.selected !== part) viewer.selectPart(part.id);
    cardMsg.hidden = false;
    cardMsg.innerHTML = `<span>Prima togli</span>${blockers.map((b) => `<button class="chip" data-id="${esc(b.id)}">${esc(b.name)}</button>`).join('')}`;
    cardMsg.querySelectorAll('[data-id]').forEach((b) => b.addEventListener('click', () => viewer.selectPart(b.dataset.id)));
  });
  card.querySelector('.x').addEventListener('click', () => viewer.selectPart(null));
  removeBtn.addEventListener('click', () => viewer.selected && viewer.removePart(viewer.selected.id));
  viewer.on('select', () => {
    renderCard();
    if (activeTab === 'parts' && !ws.active) renderPanel();
  });

  // ----- Taking parts off (free exploration and the guide) -----
  const removedBar = bottom.querySelector('.removed-bar');
  removedBar.querySelector('[data-act="restore"]').addEventListener('click', () => viewer.restoreAll());
  removedBar.querySelector('[data-act="undo"]').addEventListener('click', () => viewer.undo());
  viewer.on('parts', (removed) => {
    if (ws.active) return;
    const n = removed.length;
    const was = !removedBar.hidden;
    removedBar.hidden = !n || !!viewer.guide;
    removedBar.querySelector('.n').textContent = n === 1 ? '1 pezzo tolto' : `${n} pezzi tolti`;
    if (was !== !removedBar.hidden) fitInsets();
    if (!labels.hidden) buildLabels();
    if (activeTab === 'parts' || activeTab === 'workshop') renderPanel();
    if (viewer.guide) {
      syncPlay();
      renderCaption();
    }
  });

  // Right click on the model (desktop): small menu with the same actions.
  const closeMenu = () => (ctxMenu.hidden = true);
  viewer.on('context', ({ part, x, y }) => {
    if (ws.active) return;
    const items = [];
    if (part) {
      const screw = viewer.motionOf(part.id) === 'unscrew';
      items.push(`<button role="menuitem" data-act="remove">${screw ? ICON.screw : ICON.lift}<span>${screw ? 'Svita' : 'Togli'} <b>${esc(part.name)}</b></span></button>`);
    }
    if (viewer.removedParts.length) {
      items.push(`<button role="menuitem" data-act="undo">${ICON.undo}<span>Rimetti l’ultimo</span></button>`);
      items.push(`<button role="menuitem" data-act="restore">${ICON.restore}<span>Rimetti tutto</span></button>`);
    }
    if (!items.length) return closeMenu();
    ctxMenu.innerHTML = items.join('');
    ctxMenu.hidden = false;
    const r = stage.getBoundingClientRect();
    const mx = Math.min(x - r.left, r.width - ctxMenu.offsetWidth - 8);
    const my = Math.min(y - r.top, r.height - ctxMenu.offsetHeight - 8);
    ctxMenu.style.left = `${Math.max(8, mx)}px`;
    ctxMenu.style.top = `${Math.max(8, my)}px`;
    ctxMenu.querySelector('[data-act="remove"]')?.addEventListener('click', () => (viewer.removePart(part.id), closeMenu()));
    ctxMenu.querySelector('[data-act="undo"]')?.addEventListener('click', () => (viewer.undo(), closeMenu()));
    ctxMenu.querySelector('[data-act="restore"]')?.addEventListener('click', () => (viewer.restoreAll(), closeMenu()));
    ctxMenu.querySelector('button').focus({ preventScroll: true });
  });
  stage.addEventListener('pointerdown', (e) => !ctxMenu.contains(e.target) && closeMenu(), true);
  stage.addEventListener('wheel', closeMenu, { passive: true });
  ctxMenu.addEventListener('contextmenu', (e) => e.preventDefault());

  // ----- Officina: the guide (a walkthrough over the free disassembly) and the games -----
  const wsBtn = tools.querySelector('[data-act="workshop"]');
  const wsInfo = bottom.querySelector('.ws-info');
  const thrBox = bottom.querySelector('.thr');
  const setGuide = (on) => {
    if (!def.disassembly?.steps || on === !!viewer.guide) return;
    if (on) {
      setExploded(false);
      setView(false); // a real engine on the bench: closed
      closeMenu();
    }
    viewer.setGuide(on);
  };
  ws = new Workshop({
    viewer,
    def,
    stage,
    page,
    txt,
    ui: {
      setView,
      setExploded,
      setLabels,
      startGuide: () => setGuide(true),
      // A game started or ended: the HUD changed, so re-frame the model in the free area.
      layout: () => {
        closeMenu();
        renderCard();
        removedBar.hidden = true;
        tooltip.hidden = true;
        if (sheetOpen && ws.active) setSheet(false);
        requestAnimationFrame(() => {
          fitInsets();
          viewer.resetCamera();
        });
        syncPlay();
        renderCaption();
      },
    },
  });
  window.__ws = ws; // debugging, like __viewer
  wsBtn.addEventListener('click', () => {
    if (viewer.guide) return setGuide(false);
    if (ws.menuOpen) return ws.closeMenu();
    ws.openMenu();
  });
  viewer.on('guide', (g) => {
    const on = !!g;
    wsBtn.setAttribute('aria-pressed', on);
    page.classList.toggle('guiding', on);
    steps.hidden = on;
    wsDots.hidden = !on;
    thrBox.hidden = on;
    wsInfo.hidden = !on;
    if (on && !wsDots.children.length)
      def.disassembly.steps.forEach((st, i) => wsDots.appendChild(h(`<span role="presentation" title="${i + 1}. ${esc(st.name)}"></span>`)));
    removedBar.hidden = on || !viewer.removedParts.length;
    syncPlay();
    renderCaption();
    fitInsets();
  });
  const syncGuideInfo = () => {
    if (!viewer.guide) return;
    const { steps: gs, index } = viewer.guideStatus;
    wsInfo.querySelector('b').textContent = index < gs.length ? `Passo ${index + 1} di ${gs.length}` : 'Smontato';
    wsInfo.querySelector('small').textContent = 'Guida semplificata';
    wsInfo.title = def.disassembly.note || '';
  };
  viewer.on('parts', syncGuideInfo);
  viewer.on('guide', syncGuideInfo);

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
          <kbd>L</kbd> nomi · <kbd>R</kbd> centra · <kbd>Canc</kbd> togli pezzo · <kbd>O</kbd> officina · <kbd>I</kbd> questo pannello · <kbd>[</kbd><kbd>]</kbd> altro motore</p></div>`;
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
    } else if (activeTab === 'workshop') {
      const { steps: gs, index } = viewer.guideStatus;
      body.innerHTML = `<p class="note">${esc(def.disassembly.note || '')}</p>
        <p>Tocca un pezzo e premi <b>Togli</b> (o <b>Svita</b>, per viti e candele): se è ancora fissato, la scheda ti dice cosa togliere prima.
        La <b>guida</b> suggerisce un ordine e illumina i pezzi da toccare; nelle <b>sfide</b> lo fai tu, con gli attrezzi e il cronometro.</p>
        <div class="cta ws-cta"><button class="btn primary ws-start">${viewer.guide ? 'Esci dalla guida' : 'Apri la guida'}</button>
        <button class="btn hazard ws-play">${ICON.wrench} Sfide</button></div>
        <ol class="phase-list ws-list">${gs
          .map(
            (st, i) => `<li class="${viewer.guide && i === index ? 'on' : ''}${i < index ? ' done' : ''}"><div class="step-h"><span class="n">${i + 1}</span><b>${esc(
              st.name
            )}</b></div><div class="txt">${txt(st)}</div></li>`
          )
          .join('')}</ol>`;
      body.querySelector('.ws-start').addEventListener('click', () => (setGuide(!viewer.guide), renderPanel()));
      body.querySelector('.ws-play').addEventListener('click', () => (setGuide(false), ws.openMenu()));
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
  const play = params.get('gioca');
  if (wantSheet && !play) setSheet(true);
  else fitInsets();
  // Deep link from the home page: #/motore/<id>?gioca=menu|strip|build|hunt
  if (play) {
    history.replaceState(null, '', `#/motore/${def.id}`);
    if (play === 'menu' || (play !== 'hunt' && !ws.hasRules)) ws.openMenu();
    else ws.start(play);
  }
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
