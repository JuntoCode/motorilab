// Tiny synthesised sound effects for the workshop games (Web Audio, no files).
let ctx = null;
let enabled = true;
try {
  enabled = localStorage.getItem('motorilab-sound') !== '0';
} catch {}

function audio() {
  if (!enabled) return null;
  try {
    ctx ||= new (window.AudioContext || window.webkitAudioContext)();
    if (ctx.state === 'suspended') ctx.resume();
  } catch {
    return null;
  }
  return ctx;
}

function tone(freq, dur, { type = 'sine', gain = 0.12, to = null, delay = 0 } = {}) {
  const a = audio();
  if (!a) return;
  const t = a.currentTime + delay;
  const o = a.createOscillator();
  const g = a.createGain();
  o.type = type;
  o.frequency.setValueAtTime(freq, t);
  if (to) o.frequency.exponentialRampToValueAtTime(to, t + dur);
  g.gain.setValueAtTime(0.0001, t);
  g.gain.exponentialRampToValueAtTime(gain, t + 0.008);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  o.connect(g).connect(a.destination);
  o.start(t);
  o.stop(t + dur + 0.02);
}

let noiseBuf = null;
function noise(dur, { freq = 2000, q = 1, gain = 0.12, delay = 0, type = 'bandpass' } = {}) {
  const a = audio();
  if (!a) return;
  if (!noiseBuf) {
    noiseBuf = a.createBuffer(1, a.sampleRate * 0.5, a.sampleRate);
    const d = noiseBuf.getChannelData(0);
    for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
  }
  const t = a.currentTime + delay;
  const src = a.createBufferSource();
  src.buffer = noiseBuf;
  const f = a.createBiquadFilter();
  f.type = type;
  f.frequency.value = freq;
  f.Q.value = q;
  const g = a.createGain();
  g.gain.setValueAtTime(gain, t);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  src.connect(f).connect(g).connect(a.destination);
  src.start(t, Math.random() * 0.3, dur + 0.02);
}

export const sfx = {
  get enabled() {
    return enabled;
  },
  toggle() {
    enabled = !enabled;
    try {
      localStorage.setItem('motorilab-sound', enabled ? '1' : '0');
    } catch {}
    if (enabled) sfx.click();
    return enabled;
  },
  click: () => tone(720, 0.05, { type: 'triangle', gain: 0.06 }),
  // One click of the ratchet wrench.
  ratchet: () => (noise(0.03, { freq: 3400, q: 3, gain: 0.16 }), tone(1500, 0.025, { type: 'square', gain: 0.025 })),
  // A part comes off: soft thump and a little air.
  pop: () => (tone(190, 0.16, { to: 70, gain: 0.2 }), noise(0.12, { freq: 900, q: 0.7, gain: 0.07 })),
  // A part goes back on: metallic clunk.
  clunk: () => (tone(130, 0.12, { to: 85, gain: 0.22 }), noise(0.05, { freq: 2600, q: 2, gain: 0.1 }), tone(880, 0.08, { type: 'triangle', gain: 0.04, delay: 0.01 })),
  grab: () => noise(0.05, { freq: 1300, q: 1, gain: 0.05 }),
  error: () => (tone(220, 0.14, { type: 'sawtooth', gain: 0.07 }), tone(160, 0.2, { type: 'sawtooth', gain: 0.07, delay: 0.1 })),
  // Rising blip: pitch follows the combo.
  score: (combo = 1) => tone(520 * Math.pow(1.122, Math.min(combo, 8) - 1), 0.12, { type: 'triangle', gain: 0.1, to: 780 * Math.pow(1.122, Math.min(combo, 8) - 1) }),
  hint: () => (tone(900, 0.08, { gain: 0.06 }), tone(1200, 0.12, { gain: 0.06, delay: 0.08 })),
  tick: () => tone(1000, 0.03, { type: 'square', gain: 0.03 }),
  go: () => (tone(660, 0.1, { type: 'triangle', gain: 0.1 }), tone(990, 0.22, { type: 'triangle', gain: 0.1, delay: 0.1 })),
  win: () => [523, 659, 784, 1047].forEach((f, i) => tone(f, 0.28, { type: 'triangle', gain: 0.11, delay: i * 0.11 })),
  star: (i = 0) => tone(880 * Math.pow(1.26, i), 0.25, { type: 'sine', gain: 0.12 }),
};
