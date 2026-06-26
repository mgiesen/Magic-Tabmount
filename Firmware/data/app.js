const dot = document.getElementById('dot');
const statusText = document.getElementById('statusText');
const color = document.getElementById('color');
const hex = document.getElementById('hex');
const dac = document.getElementById('dac');
const dacLabel = document.getElementById('dacLabel');
const displayToggle = document.getElementById('displayToggle');
const displayLabel = document.getElementById('displayLabel');
const sensorDot = document.getElementById('sensorDot');
const sensorVersion = document.getElementById('sensorVersion');
const personDot = document.getElementById('personDot');
const personText = document.getElementById('personText');
const conn = document.getElementById('conn');
const connText = document.getElementById('connText');

let ws = null;
let displayPending = false;     // wartet auf Bestaetigung der Motorbewegung
let displayRequested = false;   // zuletzt vom Nutzer angeforderter Zustand (Source of Truth)
let displayTimer = null;        // Sicherheitsnetz gegen dauerhaftes Haengenbleiben
let colorReady = false;         // Color Picker wurde initial gesetzt
let dacReady = false;           // Motorstrom-Slider wurde initial gesetzt

// DAC-Wert als Prozent der vollen DAC-Range (0-4095) darstellen
function dacText(v) {
  return (v / 4095 * 100).toFixed(1) + ' %';
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function resolveDisplay(on) {
  displayPending = false;
  clearTimeout(displayTimer);
  displayToggle.disabled = false;
  displayToggle.checked = on;
  displayLabel.textContent = on ? 'Display ein' : 'Display aus';
}

function applyState(s) {
  // iPad-Erkennung (Hall-Sensor)
  dot.className = s.present ? 'dot present' : 'dot absent';
  statusText.textContent = s.present ? 'iPad anliegend' : 'iPad nicht anliegend';

  // Presence Sensor (LD2410) - Verbindung + Firmware-Version
  const sensorOnline = !!s.sensorVersion;
  if (sensorOnline) {
    sensorDot.className = 'dot present';
    sensorVersion.textContent = 'Firmware ' + s.sensorVersion;
  } else {
    sensorDot.className = 'dot absent';
    sensorVersion.textContent = 'Sensor offline';
  }

  // Presence Sensor (LD2410) - Person erkannt / nicht erkannt
  if (!sensorOnline) {
    personDot.className = 'dot';                 // neutral: ohne Sensor keine Aussage
    personText.textContent = 'Keine Sensordaten';
  } else if (s.personPresent) {
    personDot.className = 'dot present';
    personText.textContent = 'Person erkannt';
  } else {
    personDot.className = 'dot absent';
    personText.textContent = 'Person nicht erkannt';
  }

  // Display-Schalter: gegen den ANGEFORDERTEN Wert abgleichen (nicht gegen das
  // live .checked, das durch Label/Firmware-Echo verfaelscht sein kann).
  if (displayPending) {
    if (s.display === displayRequested) {
      resolveDisplay(s.display);
    }
  } else {
    displayToggle.checked = s.display;
    displayLabel.textContent = s.display ? 'Display ein' : 'Display aus';
  }

  // LED-Farbe nur einmalig vom Geraet uebernehmen (danach fuehrt der Nutzer)
  if (!colorReady && s.hex) {
    color.value = s.hex;
    hex.textContent = s.hex.toUpperCase();
    colorReady = true;
  }

  // Motorstrom-Slider einmalig vom Geraet uebernehmen (danach fuehrt der Nutzer)
  if (!dacReady && typeof s.dac === 'number') {
    dac.value = s.dac;
    dacLabel.textContent = dacText(s.dac);
    dacReady = true;
  }
}

// Ohne Live-Verbindung sind Sensordaten ungueltig -> neutral anzeigen
function setSensorWaiting() {
  sensorDot.className = 'dot';
  sensorVersion.textContent = 'Warte auf Sensor…';
  personDot.className = 'dot';
  personText.textContent = 'Warte auf Sensor…';
}

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

displayToggle.addEventListener('change', () => {
  const on = displayToggle.checked;
  displayRequested = on;            // gemerkter Wunsch = alleinige Wahrheit
  displayPending = true;
  displayToggle.disabled = true;
  displayLabel.textContent = on ? 'Aktiviere…' : 'Deaktiviere…';
  send({ type: 'setDisplay', on });

  // Sicherheitsnetz: kommt wider Erwarten keine Bestaetigung, nicht dauerhaft sperren
  clearTimeout(displayTimer);
  displayTimer = setTimeout(() => {
    displayPending = false;
    displayToggle.disabled = false;
  }, 6000);
});

// LED-Farbe instant senden, aber gedrosselt (max. ~25 Updates/s).
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
    pendingHex = h;                       // neuesten Wert merken
    if (!sendTimer) {
      sendTimer = setTimeout(() => {
        sendTimer = null;
        lastSent = performance.now();
        if (pendingHex !== null) { sendColorNow(pendingHex); pendingHex = null; }
      }, MIN_INTERVAL - dt);
    }
  }
}

color.addEventListener('input', () => {
  hex.textContent = color.value.toUpperCase();
  queueColor();
});
color.addEventListener('change', () => {
  hex.textContent = color.value.toUpperCase();
  lastSent = performance.now();
  sendColorNow(color.value.substring(1));
});

// Motorstrom (DAC/VREF): live gedrosselt senden, beim Loslassen persistent speichern.
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

dac.addEventListener('input', () => {
  dacLabel.textContent = dacText(parseInt(dac.value, 10));
  dacQueue();
});
dac.addEventListener('change', () => {
  // Slider losgelassen -> finalen Wert senden UND persistent speichern
  const v = parseInt(dac.value, 10);
  dacLabel.textContent = dacText(v);
  dacLastSent = performance.now();
  dacSendNow(v, true);
});

connect();
