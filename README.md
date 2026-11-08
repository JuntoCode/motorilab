# MotoriLab

**Sito online:** https://juntocode.github.io/motorilab/

Visualizzatore 3D interattivo che spiega come funzionano i motori dei mezzi di oggi: auto, aerei, razzi e treni.
Vista chiusa e in sezione (con superfici di taglio tratteggiate), esploso, etichette dei pezzi, fasi passo-passo,
acceleratore live, varianti (es. distribuzione a cinghia o a catena), due livelli di spiegazione (Base / Esperto),
quiz, confronto e scorciatoie da tastiera (premi `?` nella pagina di un motore).

Sito statico: HTML + CSS + JavaScript (moduli ES) con [Three.js](https://threejs.org) r170 caricato da CDN. Nessuna build.

## Avvio in locale

```bash
python .claude/serve.py 5173
```

(`python -m http.server 5173` va bene per guardare il sito; `serve.py` disattiva la cache e abilita gli strumenti di sviluppo.)

Poi apri http://localhost:5173. Serve un server: aprire `index.html` con doppio clic non carica i moduli.

## Pubblicazione gratuita

- **GitHub Pages**: carica la cartella in un repository, Settings → Pages → branch `main`, cartella `/ (root)`.
- **Netlify / Vercel / Cloudflare Pages**: importa il repository, nessun comando di build, cartella di pubblicazione `/`.

## Struttura

- `js/core/viewer.js` – scena Three.js: camera, luci, spaccato, esploso, selezione pezzi, fasi.
- `js/core/helpers.js` – materiali, geometrie e particelle condivise.
- `js/engines/*.js` – un file per motore; il formato è descritto in `js/engines/CONTRACT.md`.
- `js/catalog.js` – famiglie e motori mostrati nel sito (quelli senza `load` appaiono come "in arrivo").
- `assets/img/` – copertine generate con Google Antigravity.

## Strumenti di sviluppo

- `#/anteprima/<file>` apre un motore non ancora presente nel catalogo.
- `js/dev/check.js` controlla un modulo (errori, NaN, triangoli, particelle).
- `js/dev/capture.js` salva foto dei modelli in `.claude/shots/` e le miniature delle schede in `assets/thumbs/`.
- `.claude/gen-meta.py` rigenera `js/catalog-meta.js` (nomi e frasi delle schede) dopo aver modificato i motori.
- `.claude/register.py` collega un motore al catalogo, `.claude/camzoom.py` regola l'inquadratura iniziale.

## Aggiungere un motore

1. Crea `js/engines/<id>.js` seguendo `CONTRACT.md` (puoi partire da `petrol4t.js`).
2. In `js/catalog.js` aggiungi `load: () => import('./engines/<id>.js')` alla voce del motore (o usa `.claude/register.py`).
3. Rigenera i metadati con `python .claude/gen-meta.py` e la miniatura con `thumbs(['<file>'])` da `js/dev/capture.js`.
