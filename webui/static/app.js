'use strict';
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const BLANK = 'data:image/gif;base64,R0lGODlhAQABAAAAACw=';

const app = { ws: null, cid: null, settings: null, presets: {}, tel: null, tab: 'drive', msgId: null };

// ------------------------------------------------------------------ utils
const getPath = (obj, path) => path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), obj);
function nest(path, value) {
  const keys = path.split('.'), out = {};
  let o = out;
  keys.slice(0, -1).forEach(k => (o = o[k] = {}));
  o[keys[keys.length - 1]] = value;
  return out;
}
function deepAssign(t, s) {
  for (const k in s) {
    if (s[k] && typeof s[k] === 'object' && !Array.isArray(s[k])) { t[k] = t[k] || {}; deepAssign(t[k], s[k]); }
    else t[k] = s[k];
  }
}
const fmt = (v, d = 2) => (v == null || isNaN(v) ? '–' : Number(v).toFixed(d));
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const isField = el => el && (el.tagName === 'INPUT' || el.tagName === 'SELECT' || el.tagName === 'TEXTAREA');

let toastTimer;
function toast(msg) {
  const t = $('#toast');
  t.textContent = msg;
  t.classList.remove('hidden');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.add('hidden'), 3500);
}
async function api(method, url, body) {
  const r = await fetch(url, {
    method, headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  if (!r.ok) throw new Error((await r.text()) || r.statusText);
  return r.json();
}
function setChip(sel, ok, text) {
  const c = $(sel);
  c.classList.toggle('ok', !!ok);
  c.classList.toggle('bad', !ok);
  if (text) c.textContent = text;
}
const show = (sel, on) => $(sel).classList.toggle('hidden', !on);

// ------------------------------------------------------------------ websocket
function connect() {
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  app.ws = ws;
  ws.onopen = () => {
    show('#b-link', false);
    $$('.viewer').forEach(v => { v.img.dataset.src = ''; v.refresh(); });   // reopen streams after a restart
    syncVision();
    api('GET', '/api/state').then(s => { app.settings = s.settings; app.presets = s.presets; renderPresets(); renderSettings(); });
  };
  ws.onclose = () => {
    app.ws = null; app.tel = null;
    show('#b-link', true);
    setChip('#chip-link', false);
    setTimeout(connect, 1000);
  };
  ws.onmessage = e => {
    const m = JSON.parse(e.data);
    if (m.type === 'hello') app.cid = m.id;
    else if (m.type === 'telemetry') onTelemetry(m);
    else if (m.type === 'settings') { app.settings = m.settings; renderSettings(); }
    else if (m.type === 'error') toast(m.msg);
  };
}
const send = obj => { if (app.ws && app.ws.readyState === 1) app.ws.send(JSON.stringify(obj)); };

// ------------------------------------------------------------------ tabs + viewers
function showTab(name) {
  app.tab = name;
  $$('#tabs button').forEach(b => b.classList.toggle('active', b.dataset.tab === name));
  $$('.tab').forEach(s => s.classList.toggle('hidden', s.id !== 'tab-' + name));
  $$('.viewer').forEach(v => v.refresh());
  syncVision();
  if (name === 'logs') loadLogs();
  if (name === 'system') loadSubscriber();
  if (name === 'calib') loadPwm();
  try { localStorage.setItem('tab', name); } catch (e) { /* storage unavailable */ }
}
// Vision runs on the Pi while this browser shows something that needs it.
function syncVision() {
  const visibleNeedsVision = $$('.viewer').some(v => !v.closest('.tab').classList.contains('hidden') && v.sel.value !== 'raw');
  send({ t: 'vision', on: app.tab === 'auto' || app.tab === 'tune' || visibleNeedsVision });
}
function setupViewer(v) {
  const sel = document.createElement('select');
  [['raw', 'Camera'], ['lane', 'Lane (bird’s-eye)'], ['light', 'Traffic light']].forEach(([k, l]) => sel.add(new Option(l, k)));
  sel.value = v.dataset.default;
  const img = document.createElement('img');
  img.alt = '';
  v.append(img, sel);
  v.img = img; v.sel = sel;
  v.refresh = () => {
    const visible = !v.closest('.tab').classList.contains('hidden');
    const want = visible ? '/stream/' + sel.value : BLANK;
    if (img.dataset.src !== want) {
      img.dataset.src = want;
      img.src = want === BLANK ? BLANK : want + '?' + Date.now();
    }
  };
  sel.onchange = () => { v.refresh(); syncVision(); };
}

// ------------------------------------------------------------------ settings binding
const pending = {};
let pendTimer;
function queueSetting(path, v) {
  deepAssign(pending, nest(path, v));
  clearTimeout(pendTimer);
  pendTimer = setTimeout(flushSettings, 150);
}
async function flushSettings() {
  const body = JSON.parse(JSON.stringify(pending));
  for (const k in pending) delete pending[k];
  try { app.settings = await api('POST', '/api/settings', body); }
  catch (e) { toast('Setting rejected: ' + e.message); }
  renderSettings();
}
function bindSettings() {
  $$('[data-path]').forEach(el => {
    el.addEventListener(el.type === 'range' ? 'input' : 'change', () => {
      let v = el.value;
      if (el.type === 'checkbox') v = el.checked;
      else if (el.type === 'range' || el.type === 'number') { v = parseFloat(el.value); if (isNaN(v)) return; }
      updateOutput(el);
      queueSetting(el.dataset.path, v);
    });
  });
}
function updateOutput(el) {
  const o = el.nextElementSibling;
  if (o && o.tagName === 'OUTPUT') o.textContent = el.value;
}
function renderSettings() {
  if (!app.settings) return;
  renderHsv();
  $$('[data-path]').forEach(el => {
    if (el === document.activeElement && el.type !== 'checkbox') return;
    const v = getPath(app.settings, el.dataset.path);
    if (v === undefined) return;
    if (el.type === 'checkbox') el.checked = !!v; else el.value = v;
    updateOutput(el);
  });
  const cam = app.settings.camera;
  $('[data-path="camera.exposure_us"]').disabled = cam.auto_exposure;
  $('[data-path="camera.gain"]').disabled = cam.auto_exposure;
  const p = app.settings.pipeline;
  $('#preset').value = p.preset;
  const pr = app.presets[p.preset];
  $('#preset-hint').textContent = pr ? `Same behavior as: python -m ${pr.script.replace('.py', '').replace('/', '.')} --throttle ${p.throttle}` : '';
}
function renderPresets() {
  const sel = $('#preset');
  sel.innerHTML = '';
  Object.entries(app.presets).forEach(([k, v]) => sel.add(new Option(v.label, k)));
}

// ------------------------------------------------------------------ HSV tuning
const HSV_TARGETS = [
  { prefix: 'pipeline.single.red1', name: 'Lane red (range 1)', lane: 'single' },
  { prefix: 'pipeline.single.red2', name: 'Lane red (range 2)', lane: 'single' },
  { prefix: 'pipeline.two.yellow', name: 'Lane yellow', lane: 'two' },
  { prefix: 'pipeline.two.white', name: 'Lane white', lane: 'two' },
  { prefix: 'pipeline.light_hsv.red', name: 'Light red' },
  { prefix: 'pipeline.light_hsv.yellow', name: 'Light yellow' },
  { prefix: 'pipeline.light_hsv.green', name: 'Light green' },
];
function buildHsv() {
  const box = $('#hsv-ranges');
  HSV_TARGETS.forEach(t => {
    const row = document.createElement('div');
    row.className = 'hsv-row';
    row.dataset.lane = t.lane || '';
    row.innerHTML = `<span class="name">${t.name}</span>` +
      ['lo', 'hi'].map(s => `<span class="hint">${s === 'lo' ? 'low' : 'high'}</span>` +
        [0, 1, 2].map(i => `<input type="number" min="0" max="255" data-arr="${t.prefix}_${s}" data-i="${i}" title="${'HSV'[i]}">`).join('')).join('');
    box.append(row);
  });
  $$('[data-arr]').forEach(el => el.addEventListener('change', () => {
    const path = el.dataset.arr;
    const arr = [...(getPath(pending, path) || getPath(app.settings, path))];
    arr[+el.dataset.i] = clamp(parseInt(el.value || 0, 10), 0, 255);
    queueSetting(path, arr);
  }));
  const sel = $('#sample-target');
  HSV_TARGETS.forEach(t => sel.add(new Option(t.name, t.prefix)));
}
function renderHsv() {
  const lane = app.settings.pipeline.lane;
  $$('.hsv-row').forEach(r => r.classList.toggle('hidden', !!r.dataset.lane && r.dataset.lane !== lane));
  $$('#sample-target option').forEach(o => {
    const t = HSV_TARGETS.find(x => x.prefix === o.value);
    o.hidden = !!t.lane && t.lane !== lane;
  });
  const cur = HSV_TARGETS.find(x => x.prefix === $('#sample-target').value);
  if (cur && cur.lane && cur.lane !== lane) $('#sample-target').value = HSV_TARGETS.find(x => x.lane === lane).prefix;
  $$('[data-arr]').forEach(el => {
    if (el !== document.activeElement) el.value = getPath(app.settings, el.dataset.arr)[+el.dataset.i];
  });
  // area / scale inputs follow the active lane estimator
  $('#area-min').dataset.path = `pipeline.${lane}.area_min`;
  $('#px-to-m').dataset.path = `pipeline.${lane}.px_to_m`;
}
let lastSample = null;
async function sampleAt(e) {
  const v = $('#sample-viewer');
  if (v.sel.value !== 'raw') { toast('Switch this view to “Camera” to sample colors'); return; }
  const r = v.img.getBoundingClientRect();
  const x = (e.clientX - r.left) / r.width * 640, y = (e.clientY - r.top) / r.height * 360;
  let mark = v.querySelector('.mark');
  if (!mark) { mark = document.createElement('div'); mark.className = 'mark'; v.append(mark); }
  mark.style.left = (e.clientX - r.left) + 'px';
  mark.style.top = (e.clientY - r.top) + 'px';
  try {
    lastSample = await api('POST', '/api/sample', { x, y, size: 20 });
    $('#swatch').style.background = lastSample.color;
    const [h, s, val] = lastSample.hsv;
    $('#sample-hsv').textContent = `H ${h}  S ${s}  V ${val}`;
  } catch (err) { toast(err.message); }
}
function applySample() {
  if (!lastSample) { toast('Click the camera image first'); return; }
  const tol = ['#tol-h', '#tol-s', '#tol-v'].map(s => parseInt($(s).value || 0, 10));
  const max = [180, 255, 255];
  const lo = lastSample.hsv.map((v, i) => clamp(v - tol[i], 0, max[i]));
  const hi = lastSample.hsv.map((v, i) => clamp(v + tol[i], 0, max[i]));
  const prefix = $('#sample-target').value;
  queueSetting(prefix + '_lo', lo);
  queueSetting(prefix + '_hi', hi);
  toast(`${$('#sample-target').selectedOptions[0].text}: [${lo}] – [${hi}]`);
}

// ------------------------------------------------------------------ manual driving input
const input = { keys: new Set(), pad: null, test: null, testEngaged: false, kSteer: 0, kThr: 0, last: performance.now(), gpB: false };
const KEYMAP = { KeyW: 'up', ArrowUp: 'up', KeyS: 'down', ArrowDown: 'down', KeyA: 'left', ArrowLeft: 'left', KeyD: 'right', ArrowRight: 'right' };
const approach = (v, target, step) => (Math.abs(target - v) <= step ? target : v + Math.sign(target - v) * step);

function readGamepad() {
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  const gp = pads[0];
  $('#gp-name').textContent = gp ? 'Gamepad: ' + gp.id.slice(0, 40) : '';
  if (!gp) return null;
  const dz = v => (Math.abs(v) < 0.08 ? 0 : v);
  const lx = dz(gp.axes[0] || 0), rx = dz(gp.axes[2] || 0);
  const steer = Math.abs(rx) > Math.abs(lx) ? rx : lx;
  const btn = i => (gp.buttons[i] ? gp.buttons[i].value : 0);
  let thr = btn(7) - btn(6);          // RT forward, LT reverse (standard mapping)
  if (Math.abs(thr) < 0.05) thr = 0;
  const b = gp.buttons[1] && gp.buttons[1].pressed;
  if (b && !input.gpB) estop('gamepad');
  input.gpB = b;
  return { steer, thr, active: steer !== 0 || thr !== 0 };
}

function tick() {
  const now = performance.now(), dt = Math.min(0.1, (now - input.last) / 1000);
  input.last = now;
  const gp = readGamepad();
  const k = input.keys;
  const tS = (k.has('right') ? 1 : 0) - (k.has('left') ? 1 : 0);
  const tT = (k.has('up') ? 1 : 0) - (k.has('down') ? 1 : 0);
  input.kSteer = approach(input.kSteer, tS, (tS === 0 ? 8 : 4) * dt);
  input.kThr = tT === 0 ? 0 : approach(input.kThr, tT, 2 * dt);   // releasing the key stops immediately

  let steer = 0, thr = 0, raw = false;
  if (input.test) { steer = input.test.steer; thr = input.test.thr; raw = true; }
  else if (input.pad) { steer = input.pad.x; thr = input.pad.y; }
  else if (gp && gp.active) { steer = gp.steer; thr = gp.thr; }
  else { steer = input.kSteer; thr = input.kThr; }

  const knob = $('#knob');
  knob.style.left = `${50 + (raw ? 0 : steer) * 39}%`;
  knob.style.top = `${50 - (raw ? 0 : thr) * 39}%`;
  $('#in-steer').textContent = fmt(steer);
  $('#in-thr').textContent = fmt(thr);
  if ((app.tel && app.tel.you_control) || input.test) send({ t: 'drive', steer, throttle: thr, raw });
}

function setupPad() {
  const pad = $('#pad');
  const move = e => {
    const r = pad.getBoundingClientRect();
    let x = (e.clientX - r.left) / r.width * 2 - 1, y = -((e.clientY - r.top) / r.height * 2 - 1);
    const m = Math.hypot(x, y);
    if (m > 1) { x /= m; y /= m; }
    input.pad = { x, y };
  };
  pad.addEventListener('pointerdown', e => {
    if (!(app.tel && app.tel.you_control)) { toast('Press “Take control” first'); return; }
    pad.setPointerCapture(e.pointerId);
    move(e);
  });
  pad.addEventListener('pointermove', e => { if (input.pad) move(e); });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => pad.addEventListener(ev, () => { input.pad = null; }));
}

function setupKeys() {
  addEventListener('keydown', e => {
    if (isField(e.target)) return;
    if (e.code === 'Space') { e.preventDefault(); if (!e.repeat) estop('keyboard'); return; }
    if (e.code === 'Escape') { send({ t: 'release' }); return; }
    const k = KEYMAP[e.code];
    if (k && app.tel && app.tel.you_control) { input.keys.add(k); e.preventDefault(); }
  });
  addEventListener('keyup', e => {
    if (e.code === 'Space' && !isField(e.target)) e.preventDefault();   // don't "click" a focused button
    const k = KEYMAP[e.code];
    if (k) input.keys.delete(k);
  });
  addEventListener('blur', () => input.keys.clear());
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { input.keys.clear(); input.pad = null; input.test = null; send({ t: 'release' }); }
  });
}

function setupHoldButtons() {
  $$('.hold').forEach(b => {
    const start = e => {
      e.preventDefault();
      if (b.setPointerCapture) b.setPointerCapture(e.pointerId);
      if (!(app.tel && app.tel.you_control)) { send({ t: 'engage' }); input.testEngaged = true; }
      const p = parseFloat($('#test-power').value);
      const kind = b.dataset.test;
      input.test = kind === 'steer' ? { steer: parseFloat(b.dataset.v), thr: 0 }
        : { steer: 0, thr: kind === 'fwd' ? p : -p };
      b.classList.add('active');
    };
    const end = () => {
      if (!b.classList.contains('active')) return;
      b.classList.remove('active');
      input.test = null;
      if (input.testEngaged) { input.testEngaged = false; setTimeout(() => send({ t: 'release' }), 150); }
    };
    b.addEventListener('pointerdown', start);
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => b.addEventListener(ev, end));
  });
  $('#test-power').addEventListener('input', e => updateOutput(e.target));
}

function estop(source) { send({ t: 'estop', source }); input.keys.clear(); input.pad = null; input.test = null; }

// ------------------------------------------------------------------ telemetry
function onTelemetry(t) {
  app.tel = t;
  const bus = t.bus, cam = t.camera;
  setChip('#chip-link', true, 'Link');
  setChip('#chip-mqtt', bus.connected, bus.connected ? 'MQTT' : 'MQTT offline');
  setChip('#chip-pwm', bus.pwm_alive, bus.pwm_alive ? 'PWM' : 'PWM off');
  setChip('#chip-cam', cam.name && !cam.error,
    cam.error ? 'Camera error' : cam.name ? `${cam.name === 'simulator' ? 'Sim camera' : 'Camera'} ${fmt(cam.fps, 0)} fps` : 'Camera…');

  const mode = $('#mode');
  mode.className = 'mode ' + (t.estop ? 'estop' : t.mode);
  mode.textContent = t.estop ? 'E-STOP' : t.mode === 'manual' ? (t.you_control ? 'MANUAL · YOU' : 'MANUAL · OTHER') : t.mode.toUpperCase();

  show('#b-estop', t.estop);
  show('#b-pwm', !bus.pwm_alive);
  show('#b-foreign', bus.foreign_publisher);
  if (app.msgId !== null && t.message_id !== app.msgId && t.message) toast(t.message);
  app.msgId = t.message_id;

  // Drive tab
  const take = $('#take');
  take.disabled = t.estop || t.mode === 'auto';
  take.textContent = t.estop ? 'E-stop active' : t.you_control ? 'Release control' : t.mode === 'auto' ? 'Autopilot is driving'
    : t.mode === 'manual' ? 'Take over control' : 'Take control';
  take.className = 'big ' + (t.you_control ? '' : 'primary');
  $('#pad').classList.toggle('disabled', !t.you_control);
  const s = bus.last_sent, fresh = Date.now() / 1000 - s.ts < 0.5;
  $('#sent').textContent = fresh ? `steer ${fmt(s.steer)} · ${s.reverse ? 'reverse ' + fmt(s.rev_power) : 'throttle ' + fmt(s.throttle)}` : 'nothing (idle)';
  const pwm = bus.pwm;
  const duty = pwm ? `steer ${fmt(pwm.steer_pwm)}% · throttle ${fmt(pwm.throttle_pwm)}% · ${pwm.mode}` : 'subscriber not detected';
  $('#duty').textContent = duty;
  $('#calib-duty').textContent = duty;
  checkPwmMismatch(pwm);

  // Autopilot tab
  $('#auto-start').disabled = t.estop || t.mode === 'auto';
  $('#auto-stop').disabled = t.mode === 'idle';
  renderVision(t.vision, cam.fps);

  // Recording
  const rec = t.recording;
  $$('[data-rec]').forEach(b => {
    b.classList.toggle('on', rec.active);
    b.textContent = rec.active ? `■ Stop recording (${Math.floor(rec.seconds)} s)` : '● Record';
  });

  // System tab
  $('#sys-pwm').innerHTML = bus.pwm_alive ? `<span class="good">running</span> · ${pwm.mode}`
    : `<span class="bad">not detected</span>${bus.pwm_age != null ? ` (last seen ${fmt(bus.pwm_age, 0)} s ago)` : ''}`;
  $('#sys-mqtt').innerHTML = bus.connected ? `<span class="good">connected</span> · ${bus.host}` : `<span class="bad">offline</span> · ${bus.host}${bus.error ? ' · ' + bus.error : ''}`;
}

function renderVision(v, fps) {
  if (!v) {
    ['#v-lane', '#v-geo', '#v-curv', '#v-dist', '#v-light', '#v-steer', '#v-thr', '#v-reason', '#v-perf'].forEach(s => ($(s).textContent = '–'));
    $('#v-reason').textContent = 'vision off';
    return;
  }
  $('#v-lane').innerHTML = v.lane_ok ? `<span class="good">tracking</span> · ${v.lane_points} pts` : `<span class="bad">lost</span> · ${v.lane_points} pts`;
  $('#v-geo').textContent = `${fmt(v.offset_m, 3)} m · ${fmt(v.yaw_deg, 1)}°`;
  $('#v-curv').textContent = `${fmt(v.curvature, 3)} 1/m`;
  $('#v-dist').textContent = v.min_dist_cm == null ? (v.markers.length ? '–' : 'none') :
    `${fmt(v.min_dist_cm, 0)} cm` + (v.markers.length ? ` (ids ${v.markers.map(m => m.id).join(', ')})` : '');
  $('#v-light').innerHTML = v.light ? `<span class="light-${v.light}">${v.light}</span>` : 'off';
  $('#v-steer').textContent = fmt(v.steer);
  const sb = $('#v-steer-bar'), st = clamp(v.steer, -1, 1);
  sb.style.left = `${50 + Math.min(0, st) * 50}%`;
  sb.style.width = `${Math.abs(st) * 50}%`;
  $('#v-thr').textContent = `${fmt(v.throttle)} / ${fmt(v.target)}`;
  $('#v-thr-bar').style.width = `${clamp(v.throttle, 0, 1) * 100}%`;
  $('#v-reason').innerHTML = v.reason ? `<span class="warn-t">${v.reason}</span>${v.note ? ' · ' + v.note : ''}` : `<span class="good">driving</span>`;
  $('#v-perf').textContent = `${fmt(v.proc_ms, 1)} ms/frame · ${fmt(fps, 0)} fps`;
}

// ------------------------------------------------------------------ PWM calibration
const PWM_FIELDS = [
  ['steer_center', 'Steering center', 'Duty at steer = 0. Adjust until the wheels point straight.'],
  ['steer_range', 'Steering range', 'Duty change at full lock (steer = ±1).'],
  ['throttle_rest', 'Throttle neutral', 'Duty when stopped (ESC neutral). Adjust until the motor stays still.'],
  ['throttle_start', 'Throttle start offset', 'Offset from neutral where the motor starts moving (deadband).'],
  ['throttle_max', 'Throttle max offset', 'Offset from neutral at full throttle / full reverse.'],
];
const pwmState = { cfg: null, saved: null, defaults: null, timer: null };
function buildPwm() {
  const box = $('#pwm-rows');
  PWM_FIELDS.forEach(([key, label, help]) => {
    const row = document.createElement('div');
    row.className = 'pwm-row';
    row.innerHTML = `<span>${label}</span><span class="ctl"><button data-nudge="-0.05">−</button>` +
      `<input type="number" step="0.01" id="pwm-${key}"><button data-nudge="0.05">+</button></span><p class="hint">${help}</p>`;
    box.append(row);
    const inp = row.querySelector('input');
    inp.addEventListener('change', () => setPwm(key, parseFloat(inp.value)));
    row.querySelectorAll('[data-nudge]').forEach(b => b.addEventListener('click', () =>
      setPwm(key, Math.round((pwmState.cfg[key] + parseFloat(b.dataset.nudge)) * 100) / 100)));
  });
  $('#pwm-save').onclick = () => pushPwm(true);
  $('#pwm-revert').onclick = () => { pwmState.cfg = { ...pwmState.saved }; renderPwm(); pushPwm(false); };
  $('#pwm-defaults').onclick = () => { pwmState.cfg = { ...pwmState.defaults }; renderPwm(); pushPwm(false); };
}
async function loadPwm() {
  try {
    const r = await api('GET', '/api/pwm');
    Object.assign(pwmState, { cfg: r.config, saved: r.saved, defaults: r.defaults });
    $('#pwm-path').textContent = r.path;
    renderPwm();
  } catch (e) { toast(e.message); }
}
function setPwm(key, v) {
  if (isNaN(v)) return;
  pwmState.cfg[key] = v;
  renderPwm();
  clearTimeout(pwmState.timer);
  pwmState.timer = setTimeout(() => pushPwm(false), 120);
}
async function pushPwm(save) {
  try {
    const r = await api('POST', '/api/pwm', { config: pwmState.cfg, save });
    pwmState.cfg = r.config; pwmState.saved = r.saved;
    renderPwm();
    if (save) toast('Saved PWM calibration');
    else if (!r.live) toast('MQTT offline: change not sent to the car');
  } catch (e) { toast(e.message); }
}
function renderPwm() {
  const c = pwmState.cfg;
  if (!c) return;
  PWM_FIELDS.forEach(([k]) => { const el = $('#pwm-' + k); if (el !== document.activeElement) el.value = c[k]; });
  const dirty = PWM_FIELDS.some(([k]) => Math.abs(c[k] - pwmState.saved[k]) > 1e-9);
  $('#pwm-dirty').innerHTML = dirty ? '<span class="warn-t">Unsaved changes.</span>' : '';
  const f = v => v.toFixed(2) + '%';
  $('#pwm-derived').innerHTML = `
    <tr><th>Steer +1 / 0 / −1</th><td>${f(c.steer_center + c.steer_range)} / ${f(c.steer_center)} / ${f(c.steer_center - c.steer_range)}</td></tr>
    <tr><th>Stopped</th><td>${f(c.throttle_rest)}</td></tr>
    <tr><th>Forward (start → full)</th><td>${f(c.throttle_rest - c.throttle_start)} → ${f(c.throttle_rest - c.throttle_max)}</td></tr>
    <tr><th>Reverse (start → full)</th><td>${f(c.throttle_rest + c.throttle_start)} → ${f(c.throttle_rest + c.throttle_max)}</td></tr>`;
}
let mismatchShown = false;
function checkPwmMismatch(pwm) {
  if (!pwm || !pwmState.cfg || app.tab !== 'calib') return;
  const differs = PWM_FIELDS.some(([k]) => Math.abs(pwm.config[k] - pwmState.cfg[k]) > 1e-6);
  if (differs && !mismatchShown) { toast('Subscriber is using different values (restarted?). Re-sending.'); pushPwm(false); }
  mismatchShown = differs;
}

// ------------------------------------------------------------------ logs + system
async function loadLogs() {
  try {
    const r = await api('GET', '/api/logs');
    $('#logs-dir').textContent = r.dir;
    const rows = r.runs.map(run => {
      const links = ['mp4', 'csv'].map(x => run.files[x] ? `<a href="/logs/${run.files[x]}">${x}</a>` : '').join(' ');
      return `<tr><td>${run.name}</td><td>${new Date(run.mtime * 1000).toLocaleString()}</td>` +
        `<td>${(run.size / 1e6).toFixed(1)} MB</td><td>${links}</td><td><button data-del="${run.name}">Delete</button></td></tr>`;
    });
    $('#logs-table').innerHTML = rows.length ? '<tr><th>Run</th><th>Date</th><th>Size</th><th>Files</th><th></th></tr>' + rows.join('')
      : '<tr><td class="hint">No recordings yet.</td></tr>';
    $$('[data-del]').forEach(b => b.onclick = async () => {
      if (!confirm(`Delete ${b.dataset.del}?`)) return;
      try { await api('DELETE', '/api/logs/' + b.dataset.del); loadLogs(); } catch (e) { toast(e.message); }
    });
  } catch (e) { toast(e.message); }
}
async function loadSubscriber() {
  try {
    const r = await api('GET', '/api/subscriber');
    const log = $('#sub-log');
    const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 10;
    log.textContent = r.log || (r.pid ? '' : 'No log yet. Logs only appear for a subscriber started from this page.');
    if (atBottom) log.scrollTop = log.scrollHeight;
    $('#sub-stop').disabled = !r.pid;
    $('#sub-stop').title = r.pid ? `pid ${r.pid}` : 'Only a subscriber started from this page can be stopped here';
  } catch (e) { toast(e.message); }
}
async function subscriber(action) {
  if (action === 'stop' && !confirm('Turn the chassis power switch OFF before stopping the subscriber.\n\nIs the power switch OFF?')) return;
  try { await api('POST', '/api/subscriber', { action }); toast(`Subscriber ${action === 'start' ? 'started' : 'stopped'}`); }
  catch (e) { toast(e.message); }
  setTimeout(loadSubscriber, 800);
}

// ------------------------------------------------------------------ wiring
function init() {
  $$('.viewer').forEach(setupViewer);
  buildHsv();
  buildPwm();
  bindSettings();
  setupPad();
  setupKeys();
  setupHoldButtons();

  $$('#tabs button').forEach(b => (b.onclick = () => showTab(b.dataset.tab)));
  $$('[data-goto]').forEach(b => (b.onclick = () => showTab(b.dataset.goto)));
  $('#estop').onclick = () => estop('button');
  $('#b-estop-reset').onclick = () => send({ t: 'reset_estop' });
  $('#take').onclick = () => send({ t: app.tel && app.tel.you_control ? 'release' : 'engage' });
  $('#auto-start').onclick = () => {
    if (app.tel && !app.tel.bus.pwm_alive && !confirm('pwm_subscriber is not running, so the car will not move. Start anyway?')) return;
    send({ t: 'auto' });
  };
  $('#auto-stop').onclick = () => send({ t: 'stop' });
  $('#preset-load').onclick = async () => {
    try {
      app.settings = await api('POST', '/api/preset', { name: $('#preset').value });
      renderSettings();
      toast('Preset loaded');
    } catch (e) { toast(e.message); }
  };
  $$('[data-rec]').forEach(b => (b.onclick = () => send({ t: 'record', on: !(app.tel && app.tel.recording.active) })));
  $('#logs-refresh').onclick = loadLogs;
  $('#sub-start').onclick = () => subscriber('start');
  $('#sub-stop').onclick = () => subscriber('stop');
  $('#sub-refresh').onclick = loadSubscriber;
  $('#sample-viewer').img.addEventListener('click', sampleAt);
  $('#sample-apply').onclick = applySample;
  setInterval(() => { if (app.tab === 'system') loadSubscriber(); }, 2000);

  let tab = 'drive';
  try { tab = localStorage.getItem('tab') || 'drive'; } catch (e) { /* storage unavailable */ }
  showTab($('#tab-' + tab) ? tab : 'drive');
  connect();
  setInterval(tick, 50);
}
init();
