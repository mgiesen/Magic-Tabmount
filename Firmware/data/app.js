const conn = document.getElementById('conn');
const connText = document.getElementById('connText');
const manualToggle = document.getElementById('manualToggle');
const dot = document.getElementById('dot');
const statusText = document.getElementById('statusText');
const presDot = document.getElementById('presDot');
const presConn = document.getElementById('presConn');
const personDot = document.getElementById('personDot');
const personText = document.getElementById('personText');
const dashboardView = document.getElementById('dashboardView');
const sensorView = document.getElementById('sensorView');
const openSensorBtn = document.getElementById('openSensorBtn');
const backBtn = document.getElementById('backBtn');
const fwDot = document.getElementById('fwDot');
const fwVersion = document.getElementById('fwVersion');
const personDot2 = document.getElementById('personDot2');
const personText2 = document.getElementById('personText2');
const gateList = document.getElementById('gateList');
const gateCountRange = document.getElementById('gateCountRange');
const gateCountVal = document.getElementById('gateCountVal');
const displayToggle = document.getElementById('displayToggle');
const displayLabel = document.getElementById('displayLabel');
const color = document.getElementById('color');
const hex = document.getElementById('hex');
const dac = document.getElementById('dac');
const manualEls = document.querySelectorAll('.manual-only');

let ws = null;
let manual = false;                              // bestaetigter Modus
let manualPending = false, manualRequested = false;
let displayPending = false, displayRequested = false, displayTimer = null;
let colorReady = false, dacReady = false;

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// Manuelle Steuerelemente nur im manuellen Modus bedienbar (sonst gedimmt/gesperrt)
function refreshControls() {
  manualEls.forEach(el => el.classList.toggle('locked', !manual));
  color.disabled = !manual;
  dac.disabled = !manual;
  displayToggle.disabled = !manual || displayPending;
}

function resolveDisplay(on) {
  displayPending = false;
  clearTimeout(displayTimer);
  displayToggle.checked = on;
  displayLabel.textContent = on ? 'Display ein' : 'Display aus';
  refreshControls();
}

function applyState(s) {
  // Modus: gegen angeforderten Wert abgleichen -> kein Zurueckspringen des Schalters
  if (manualPending && s.manual === manualRequested) manualPending = false;
  if (!manualPending) {
    manual = s.manual;
    manualToggle.checked = s.manual;
  }

  // iPad-Erkennung (Hall-Sensor)
  dot.className = s.present ? 'dot present' : 'dot absent';
  statusText.textContent = s.present ? 'iPad anliegend' : 'iPad nicht anliegend';

  // Presence Sensor - Verbindungsstatus (Dashboard) + Firmware-Version (Detailseite)
  const sensorOnline = !!s.sensorVersion;
  presDot.className = sensorOnline ? 'dot present' : 'dot absent';
  presConn.textContent = sensorOnline ? 'Verbunden' : 'Offline';
  fwDot.className = sensorOnline ? 'dot present' : 'dot absent';
  fwVersion.textContent = sensorOnline ? 'Firmware ' + s.sensorVersion : 'Sensor offline';

  // Presence Sensor - Person erkannt / nicht erkannt (in beiden Ansichten gespiegelt)
  let personCls, personMsg;
  if (!sensorOnline) {
    personCls = 'dot'; personMsg = 'Keine Sensordaten';
  } else if (s.personPresent) {
    personCls = 'dot present'; personMsg = 'Person erkannt';
  } else {
    personCls = 'dot absent'; personMsg = 'Person nicht erkannt';
  }
  personDot.className = personCls;  personText.textContent = personMsg;
  personDot2.className = personCls; personText2.textContent = personMsg;

  // Empfindlichkeit pro Gate ("Bänder")
  if (sensorOnline && typeof s.gates === 'number') {
    updateGates(s.gates, s.maxGate, s.movingThr, s.statThr, s.movingEnergy, s.statEnergy);
  }

  // Display: im manuellen Modus mit Reconcile, sonst nur Spiegel des Ist-Zustands
  if (displayPending) {
    if (s.display === displayRequested) resolveDisplay(s.display);
  } else {
    displayToggle.checked = s.display;
    displayLabel.textContent = s.display ? 'Display ein' : 'Display aus';
  }

  // Farbe: im manuellen Modus einmalig uebernehmen (dann fuehrt der Nutzer),
  // im Auto-Modus die effektive LED-Farbe live spiegeln (inkl. schwarz = aus).
  if (manual) {
    if (!colorReady && s.hex) {
      color.value = s.hex;
      hex.textContent = s.hex.toUpperCase();
      colorReady = true;
    }
  } else if (s.ledColor) {
    color.value = s.ledColor;
    hex.textContent = s.ledColor.toUpperCase();
    colorReady = false; // beim Wechsel zu Manuell neu vom Wunschwert initialisieren
  }

  // Motorstrom einmalig vom Geraet uebernehmen
  if (!dacReady && typeof s.dac === 'number') {
    dac.value = s.dac;
    dacReady = true;
  }

  refreshControls();
}

// Ohne Live-Verbindung sind Sensordaten ungueltig -> neutral anzeigen
function setSensorWaiting() {
  presDot.className = 'dot';
  presConn.textContent = 'Warte auf Sensor…';
  fwDot.className = 'dot';
  fwVersion.textContent = 'Warte auf Sensor…';
  personDot.className = 'dot';  personText.textContent = 'Warte auf Sensor…';
  personDot2.className = 'dot'; personText2.textContent = 'Warte auf Sensor…';
}

// ============================================================
// Empfindlichkeit pro Gate ("Bänder")
//
// Pro Band zwei Kanäle: Bewegung (moving) und Ruhe (stationary).
// Je Kanal ein Regler: Wert = Schwelle (Soll), Balken-Hintergrund = Live-Energie.
// Energie >= Schwelle  ->  Kanal ausgelöst (Balken wird grün).
// ============================================================
const GATE_KINDS = [
  { kind: 'moving', tag: 'Dyn.' },
  { kind: 'stationary', tag: 'Stat.' }
];
const THR_OFF = 100;                         // Schwelle 100 -> Band deaktiviert (Energie kann nie > 100)
let gateCount = 0;
const eng = { moving: [], stationary: [] }; // Live-Energie pro Kanal
const thr = { moving: [], stationary: [] }; // vom Sensor bestätigte Schwellen
const reqThr = { moving: [], stationary: [] }; // angefordert, noch nicht bestätigt
let activeKey = null;                        // "<gate>-<kind>" des gerade bewegten Reglers
let maxGate = 8;                             // höchstes aktives Gate (Reichweite)
let reqMaxGate = null;                       // angefordert, noch nicht bestätigt
let gateCountActive = false;                 // Anzahl-Gates-Slider wird gerade bewegt

function buildGates(n) {
  gateCount = n;
  gateList.innerHTML = '';
  for (let i = 0; i < n; i++) {
    const gate = document.createElement('div');
    gate.className = 'gate';
    let html = '<div class="gate-idx">Gate ' + i + '</div>';
    GATE_KINDS.forEach(k => {
      html +=
        '<div class="gate-bar">' +
        '<span class="bar-tag">' + k.tag + '</span>' +
        '<input type="range" class="gate-range" min="0" max="100" value="0" data-gate="' + i + '" data-kind="' + k.kind + '">' +
        '<span class="gate-thr">--</span>' +
        '</div>';
    });
    html += '<div class="gate-overlay">Gate ' + i + ' deaktiviert</div>';
    gate.innerHTML = html;
    gate.querySelectorAll('.gate-range').forEach(r => {
      r.addEventListener('input', onGateInput);
      r.addEventListener('change', onGateChange);
    });
    gateList.appendChild(gate);
  }
}

// Setzt Reglerwert (Soll) und Balkenfüllung (Live-Energie) für einen Kanal
function paintBar(range) {
  const i = Number(range.dataset.gate);
  const kind = range.dataset.kind;
  const req = reqThr[kind][i];
  const t = (req != null) ? req : (thr[kind][i] ?? 0);

  // Reglerwert nur übernehmen, wenn der Nutzer ihn nicht gerade selbst bewegt
  if (activeKey !== i + '-' + kind) range.value = t;
  const bar = range.closest('.gate-bar');
  const val = Number(range.value);
  const off = val >= THR_OFF;               // Schwelle 100 -> Band deaktiviert
  bar.querySelector('.gate-thr').textContent = off ? 'Aus' : range.value;

  const e = Math.max(0, Math.min(100, eng[kind][i] ?? 0));
  const tripped = !off && e > val;          // strikt größer (lt. LD2410-Manual)
  // deaktiviert -> grau; ausgelöst -> grün; sonst -> blau
  const fill = off ? '#4a4f5a' : (tripped ? '#30d158' : '#0a84ff');
  range.style.background =
    'linear-gradient(90deg,' + fill + ' 0%,' + fill + ' ' + e + '%,#2a2f3a ' + e + '%,#2a2f3a 100%)';
  bar.classList.toggle('tripped', tripped);
  bar.classList.toggle('disabled', off);
}

function renderGates() {
  gateList.querySelectorAll('.gate-range').forEach(paintBar);
}

// Bänder 0..maxGate sind aktiv; der Rest bleibt sichtbar, wird aber mit einem
// Overlay "Gate X deaktiviert" überdeckt (kein Ausblenden -> stabile DOM-Höhe).
function applyGateVisibility() {
  for (let i = 0; i < gateCount; i++) {
    const card = gateList.children[i];
    if (card) card.classList.toggle('off', i > maxGate);
  }
}

function renderGateCount() {
  if (!gateCountActive) gateCountRange.value = maxGate;
  gateCountVal.textContent = (Number(gateCountRange.value) + 1); // Gates 0..N -> N+1 Stück
}

function updateGates(n, mg, mThr, sThr, mEng, sEng) {
  if (n !== gateCount) buildGates(n);
  if (Array.isArray(mThr)) thr.moving = mThr;
  if (Array.isArray(sThr)) thr.stationary = sThr;
  if (Array.isArray(mEng)) eng.moving = mEng;
  if (Array.isArray(sEng)) eng.stationary = sEng;

  // Reichweite übernehmen (sofern nicht gerade selbst geregelt / noch bestätigt wird)
  if (typeof mg === 'number') {
    if (reqMaxGate != null && mg === reqMaxGate) reqMaxGate = null;
    if (reqMaxGate == null && !gateCountActive) maxGate = mg;
  }

  // Angeforderte Schwellen freigeben, sobald der Sensor sie bestätigt hat
  ['moving', 'stationary'].forEach(kind => {
    reqThr[kind].forEach((v, i) => { if (v != null && thr[kind][i] === v) reqThr[kind][i] = null; });
  });

  renderGateCount();
  applyGateVisibility();
  renderGates();
}

// "Anzahl Gates": blendet Bänder live ein/aus und setzt die Reichweite am Sensor
gateCountRange.addEventListener('input', () => {
  gateCountActive = true;
  maxGate = Number(gateCountRange.value);
  gateCountVal.textContent = maxGate + 1;
  applyGateVisibility();
});
gateCountRange.addEventListener('change', () => {
  maxGate = Number(gateCountRange.value);
  reqMaxGate = maxGate;             // bis zur Bestätigung führend
  gateCountActive = false;
  send({ type: 'setMaxGate', value: maxGate });
});

function onGateInput(e) {
  const i = Number(e.target.dataset.gate);
  activeKey = i + '-' + e.target.dataset.kind;
  // Während des Ziehens nur visuell (Balken neu einfärben), noch nicht senden
  paintBar(e.target);
}

function onGateChange(e) {
  const i = Number(e.target.dataset.gate);
  const kind = e.target.dataset.kind;
  const val = parseInt(e.target.value, 10);
  reqThr[kind][i] = val;                  // bis zur Bestätigung führend
  activeKey = null;
  // Beide Schwellen mitschicken: der Sensor setzt sie gatewise gemeinsam
  send({
    type: 'setGateThreshold',
    gate: i,
    moving: kind === 'moving' ? val : (reqThr.moving[i] ?? thr.moving[i] ?? 0),
    stationary: kind === 'stationary' ? val : (reqThr.stationary[i] ?? thr.stationary[i] ?? 0)
  });
}

// --- Ansicht umschalten: Dashboard <-> Presence-Sensor-Einstellungen ---
openSensorBtn.addEventListener('click', () => {
  dashboardView.hidden = true;
  sensorView.hidden = false;
});
backBtn.addEventListener('click', () => {
  sensorView.hidden = true;
  dashboardView.hidden = false;
});

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => { conn.className = 'conn online'; connText.textContent = 'Verbunden'; };
  ws.onmessage = (e) => {
    try { applyState(JSON.parse(e.data)); }
    catch (err) { console.error('applyState:', err); }
  };
  ws.onclose = () => {
    conn.className = 'conn'; connText.textContent = 'Getrennt – neuer Versuch…';
    setSensorWaiting();
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
}

// --- Modus umschalten ---
manualToggle.addEventListener('change', () => {
  manualRequested = manualToggle.checked;
  manualPending = true;
  send({ type: 'setManualMode', on: manualRequested });
});

// --- Display (nur im manuellen Modus bedienbar) ---
displayToggle.addEventListener('change', () => {
  const on = displayToggle.checked;
  displayRequested = on;
  displayPending = true;
  displayLabel.textContent = on ? 'Aktiviere…' : 'Deaktiviere…';
  send({ type: 'setDisplay', on });
  refreshControls();
  clearTimeout(displayTimer);
  displayTimer = setTimeout(() => { displayPending = false; refreshControls(); }, 6000);
});

// --- LED-Farbe instant senden, aber gedrosselt (max. ~25 Updates/s) ---
const MIN_INTERVAL = 40; // ms
let lastSent = 0, pendingHex = null, sendTimer = null;
function sendColorNow(h) { send({ type: 'setLed', hex: h }); }
function queueColor() {
  const h = color.value.substring(1);
  const dt = performance.now() - lastSent;
  if (dt >= MIN_INTERVAL) {
    lastSent = performance.now();
    sendColorNow(h);
  } else {
    pendingHex = h;
    if (!sendTimer) {
      sendTimer = setTimeout(() => {
        sendTimer = null;
        lastSent = performance.now();
        if (pendingHex !== null) { sendColorNow(pendingHex); pendingHex = null; }
      }, MIN_INTERVAL - dt);
    }
  }
}
color.addEventListener('input', () => { hex.textContent = color.value.toUpperCase(); queueColor(); });
color.addEventListener('change', () => {
  hex.textContent = color.value.toUpperCase();
  lastSent = performance.now();
  sendColorNow(color.value.substring(1));
});

// --- Motorstrom (gedrosselt live, beim Loslassen persistent speichern) ---
let dacLastSent = 0, dacPending = null, dacTimer = null;
function dacSendNow(v, save) { send({ type: 'setMotorCurrent', value: v, save: !!save }); }
function dacQueue() {
  const v = parseInt(dac.value, 10);
  const dt = performance.now() - dacLastSent;
  if (dt >= MIN_INTERVAL) {
    dacLastSent = performance.now();
    dacSendNow(v, false);
  } else {
    dacPending = v;
    if (!dacTimer) {
      dacTimer = setTimeout(() => {
        dacTimer = null;
        dacLastSent = performance.now();
        if (dacPending !== null) { dacSendNow(dacPending, false); dacPending = null; }
      }, MIN_INTERVAL - dt);
    }
  }
}
dac.addEventListener('input', () => { dacQueue(); });
dac.addEventListener('change', () => {
  const v = parseInt(dac.value, 10);
  dacLastSent = performance.now();
  dacSendNow(v, true);
});

connect();
