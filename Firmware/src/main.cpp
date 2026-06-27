// ************************************************************************
// Magic Tabmount - Firmware
//
// Der ESP32 oeffnet ab dem Start einen dauerhaften WLAN-Access-Point
// (Magic-Tabmount-<hex>, PW: tabmount) und hostet darueber ein Dashboard
// (LittleFS + WebSocket /ws). Die Kernsteuerung laeuft unabhaengig davon
// ab Boot weiter (kein WLAN noetig).
//
// Architektur (klar getrennte Verantwortlichkeiten):
//   - Eingaenge : Hall-Sensor (iPad anliegend, ISR) + LD2410-Radar (Person, UART)
//   - Policy    : controlStep() ist die EINZIGE Stelle, die Aktoren ansteuert.
//                 Sie berechnet aus Modus + Sensoren die Soll-Ausgaenge.
//   - Aktoren   : Stepper-Motor (Display-Trigger), NeoPixel-Logo-LED
//
// Betriebsmodi:
//   - Automatik (Default): Display folgt der Anwesenheit (nur wenn ein iPad
//     anliegt); LED nur an wenn KEIN iPad anliegt, Farbe weiss/blau je Person.
//   - Manuell: Nutzer steuert Display + Logo-Farbe ueber das Dashboard.
//
// Nebenlaeufigkeit (ESP32-S3 ist Dual-Core): ISR und WebSocket-Handler setzen
// Soll-/Flag-Variablen. Jede Hardware-Ressource wird nur aus genau einem Kontext
// bedient (Motor, Logo-LED und Radar im loop(); DAC im WS-Handler). NVS wird aus
// beiden Kontexten geschrieben, ist dort aber intern serialisiert.
// ************************************************************************

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP4725.h>
#include <ArduinoJson.h>
#include <MyLD2410.h>

#include "DeviceDetection.h"
#include "DisplayController.h"

// ************************************************************************
// Pin- und Konstanten-Definitionen
// ************************************************************************

#define DEBUG_SERIAL Serial
#define HOSTNAME "Magic-Tabmount"

// WLAN-Access-Point: dauerhaft offen (Konfiguration/Dashboard).
#define AP_SSID_PREFIX "Magic-Tabmount-"
#define AP_PASSWORD "tabmount" // WPA2 -> mind. 8 Zeichen

// Hall-Sensor zur iPad-Erkennung: aktiv LOW (LOW = iPad erkannt)
#define DEVICE_SENSOR_PIN 4

// Logo-LEDs (NeoPixel)
#define LED_PIN 6
#define NUMBER_OF_LEDS 4
#define LED_BRIGHTNESS 80

// Automatik-Modus: Logo-Farbe (nur aktiv, wenn KEIN iPad anliegt)
//   Person erkannt -> Weiss, keine Person -> Blau  (zum Tauschen hier aendern)
const uint8_t AUTO_PERSON_R = 255, AUTO_PERSON_G = 255, AUTO_PERSON_B = 255; // weiss
const uint8_t AUTO_IDLE_R = 0, AUTO_IDLE_G = 0, AUTO_IDLE_B = 255;           // blau

// Stepper-Motor (A4988) zur Display-Steuerung
#define STEP_PIN 9
#define DIR_PIN 10
#define STEPPER_ENABLE 8
#define STEPPER_SLEEP 15

// MCP4725 DAC zur Einstellung des Motorstroms (A4988 Vref)
#define DAC_I2C_ADDRESS 0x60
#define DAC_SDA_PIN 14
#define DAC_SCL_PIN 13
#define DAC_MOTOR_CURRENT 600 // 12-Bit-Default-Wert -> Vref fuer den Treiber
#define MOTOR_DAC_MIN 350     // erlaubtes Minimum (darunter zu wenig Drehmoment)
#define MOTOR_DAC_MAX 2000    // erlaubtes Maximum (darueber unnoetig Strom/Waerme)

// LD2410 Presence-Sensor (UART). Verschaltung gemaess main-Branch / Reference:
//   ESP RX 18 <- LD2410 TX, ESP TX 17 -> LD2410 RX, 256000 Baud (Serial1)
#define LD2410_SERIAL Serial1
#define LD2410_RX_PIN 18
#define LD2410_TX_PIN 17
#define LD2410_BAUD 256000

// Nach einer Motorbewegung den Radar fuer diese Zeit ignorieren: der Motor
// erzeugt selbst eine Bewegung, die der Sensor sonst als Person fehldeutet.
#define SENSOR_SETTLE_MS 2500

// ************************************************************************
// Globale Instanzen
// ************************************************************************

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Adafruit_NeoPixel logoLED(NUMBER_OF_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_MCP4725 dac;
TwoWire I2C = TwoWire(0);
// activationRevolutions=10, motorSteps=3500, speed=700
DisplayController displayController(STEP_PIN, DIR_PIN, STEPPER_ENABLE, STEPPER_SLEEP, 10, 3500, 700);

MyLD2410 presenceSensor(LD2410_SERIAL);

Preferences prefs; // persistenter Speicher (NVS) fuer den Display-Zustand

// ************************************************************************
// Laufzeit-Zustand
//
// Aenderungen werden ausschliesslich im loop() ausgewertet/gesendet.
// Die ISR und die (Async-)WebSocket-Handler setzen nur Flags/Wuensche.
// ************************************************************************

// --- Eingaenge (Sensoren) ---
volatile bool devicePresent = false; // Hall-Sensor: iPad liegt an
volatile bool stateDirty = true;     // true => Zustand an Clients senden

// --- Betriebsmodus + manuelle Sollwerte ---
// manualMode = false (Automatik) ist der Boot-Default. Im Automatik-Modus
// bestimmen die Sensoren Display und LED; im manuellen Modus der Nutzer.
volatile bool manualMode = false;
volatile bool manualDisplayOn = false;      // gewuenschter Display-Zustand (manuell)
uint8_t ledR = 255, ledG = 255, ledB = 255; // manuelle LED-Farbe
uint16_t motorCurrentDac = DAC_MOTOR_CURRENT; // DAC-Wert (0-4095) -> VREF -> Motorstrom
bool dacOk = false;                           // MCP4725 beim Init gefunden? (sonst kein Motorstrom)

// --- zuletzt an die LED geschriebener Zustand (idempotentes Schreiben) ---
bool appliedLedOn = false;
uint8_t appliedR = 0, appliedG = 0, appliedB = 0;

String sensorVersion = ""; // LD2410 Firmware-Version (einmalig bei Boot gelesen)

// --- LD2410 Empfindlichkeit pro Entfernungs-Gate ("Band") ---
// Der LD2410 unterteilt die Reichweite in 9 Gates (0-8) mit je einer Schwelle
// fuer Bewegung und Ruhe. Uebersteigt die Live-Energie eines Gates dessen
// Schwelle, gilt das Band als ausgeloest. Der Nutzer stellt diese Schwellen
// ("Soll") im Dashboard ein und sieht die Live-Energie der Baender.
//
// Schreibbefehle an den Sensor sind blockierend und regeln den Config-Mode
// selbst -> sie duerfen NUR aus dem loop() (einziger Radar-Kontext) laufen.
// Der WS-Handler hinterlegt daher nur die Wuensche; loop() schreibt sie.
const uint8_t LD2410_GATES = 9;
volatile bool gateWritePending[LD2410_GATES] = {false};
volatile uint8_t pendingMovingThr[LD2410_GATES] = {0};
volatile uint8_t pendingStatThr[LD2410_GATES] = {0};
// Gewuenschtes max. Gate (Reichweite, 0-8). -1 = kein ausstehender Wunsch.
volatile int pendingMaxGate = -1;

// Effektive (maskierte) Anwesenheit. Waehrend des Settle-Fensters nach einer
// Motorbewegung wird der Radar ignoriert und dieser letzte Wert eingefroren.
bool personDetected = false;
unsigned long sensorMaskUntil = 0;

// Drosselung der Live-Broadcasts der LD2410-Daten (Distanz aendert sich laufend)
unsigned long lastSensorBroadcast = 0;
const unsigned long SENSOR_BROADCAST_INTERVAL = 250; // ms -> max. 4 Updates/s

DeviceDetection deviceSensor(DEVICE_SENSOR_PIN, INPUT_PULLUP, devicePresent);

// ************************************************************************
// Hardware-Hilfsfunktionen
// ************************************************************************

// Schreibt die LED nur, wenn sich etwas geaendert hat (verhindert Flackern/Spam,
// da im loop() jede Iteration aufgerufen). on=false => LEDs aus.
void applyLed(bool on, uint8_t r, uint8_t g, uint8_t b)
{
  if (!on)
  {
    r = g = b = 0;
  }
  if (on == appliedLedOn && r == appliedR && g == appliedG && b == appliedB)
  {
    return;
  }
  for (int i = 0; i < NUMBER_OF_LEDS; i++)
  {
    logoLED.setPixelColor(i, logoLED.Color(r, g, b));
  }
  logoLED.show();
  appliedLedOn = on;
  appliedR = r;
  appliedG = g;
  appliedB = b;
}

// Callback der DeviceDetection (laeuft im ISR-Kontext) -> nur Flags setzen
void IRAM_ATTR onDevicePresenceChange(bool present)
{
  devicePresent = present;
  stateDirty = true;
}

// ************************************************************************
// Zustands-Serialisierung und Broadcast
// ************************************************************************

String buildStateJson()
{
  JsonDocument doc;
  doc["manual"] = manualMode;
  doc["present"] = devicePresent;
  doc["display"] = displayController.getActivationState();
  doc["sensorVersion"] = sensorVersion;

  // Live-Anwesenheit des LD2410 nur, wenn der Sensor erreichbar ist
  // (effektiver, waehrend Motorbewegungen maskierter Wert)
  if (sensorVersion.length() > 0)
  {
    doc["personPresent"] = personDetected;

    // Empfindlichkeit pro Gate ("Baender"): eingestellte Schwellen + Live-Energie.
    // Reine Reads (maxRange wurde beim Boot gesetzt) -> kein UART-IO hier.
    const MyLD2410::ValuesArray &movThr = presenceSensor.getMovingThresholds();
    const MyLD2410::ValuesArray &staThr = presenceSensor.getStationaryThresholds();
    const MyLD2410::ValuesArray &movEng = presenceSensor.getMovingSignals();
    const MyLD2410::ValuesArray &staEng = presenceSensor.getStationarySignals();
    doc["gates"] = LD2410_GATES;
    doc["maxGate"] = presenceSensor.getRange(); // hoechstes aktives Gate (Reichweite)
    JsonArray jMovThr = doc["movingThr"].to<JsonArray>();
    JsonArray jStaThr = doc["statThr"].to<JsonArray>();
    JsonArray jMovEng = doc["movingEnergy"].to<JsonArray>();
    JsonArray jStaEng = doc["statEnergy"].to<JsonArray>();
    for (byte i = 0; i < LD2410_GATES; i++)
    {
      jMovThr.add(movThr.values[i]);
      jStaThr.add(staThr.values[i]);
      jMovEng.add(movEng.values[i]);
      jStaEng.add(staEng.values[i]);
    }
  }

  char hex[8];
  snprintf(hex, sizeof(hex), "#%02x%02x%02x", ledR, ledG, ledB);
  doc["hex"] = hex; // manuelle Wunschfarbe (fuer den manuellen Modus)

  // Effektiv an der LED anliegende Farbe (inkl. schwarz = aus) -> Dashboard-Sync
  char ledHex[8];
  snprintf(ledHex, sizeof(ledHex), "#%02x%02x%02x", appliedR, appliedG, appliedB);
  doc["ledColor"] = ledHex;

  doc["dac"] = motorCurrentDac; // Motorstrom (VREF) als roher DAC-Wert 0-4095
  doc["dacOk"] = dacOk;         // false => DAC nicht gefunden, Motor ohne Strom

  String out;
  serializeJson(doc, out);
  return out;
}

void broadcastState()
{
  if (ws.count() > 0)
  {
    ws.textAll(buildStateJson());
  }
}

// ************************************************************************
// WebSocket: eingehende Befehle
// ************************************************************************

void handleCommand(const JsonDocument &doc)
{
  const char *type = doc["type"] | "";

  if (strcmp(type, "setManualMode") == 0)
  {
    bool on = doc["on"] | false;
    // Beim Wechsel in den manuellen Modus den aktuellen Display-Zustand
    // uebernehmen -> kein ueberraschender Motorlauf, nahtlose Weitersteuerung.
    if (on && !manualMode)
    {
      manualDisplayOn = displayController.getActivationState();
    }
    manualMode = on;
    stateDirty = true;
    DEBUG_SERIAL.printf("Modus: %s\n", manualMode ? "MANUELL" : "AUTOMATIK");
  }
  else if (strcmp(type, "setDisplay") == 0)
  {
    // Nur Sollwert setzen; wirksam erst durch die Policy (controlStep) im manuellen Modus
    manualDisplayOn = doc["on"] | false;
    stateDirty = true;
  }
  else if (strcmp(type, "setLed") == 0)
  {
    const char *hex = doc["hex"] | "";
    if (strlen(hex) == 6)
    {
      long value = strtol(hex, nullptr, 16);
      ledR = (value >> 16) & 0xFF;
      ledG = (value >> 8) & 0xFF;
      ledB = value & 0xFF;
      stateDirty = true; // angewendet wird in controlStep (nur manueller Modus)
    }
  }
  else if (strcmp(type, "setMotorCurrent") == 0)
  {
    long v = doc["value"] | (long)motorCurrentDac;
    if (v < MOTOR_DAC_MIN)
      v = MOTOR_DAC_MIN;
    if (v > MOTOR_DAC_MAX)
      v = MOTOR_DAC_MAX;
    motorCurrentDac = (uint16_t)v;
    dac.setVoltage(motorCurrentDac, false); // live setzen (nicht ins DAC-EEPROM)

    // NVS nur bei "save" (Slider losgelassen) schreiben -> schont den Flash
    if (doc["save"] | false)
    {
      prefs.putUShort("dacValue", motorCurrentDac);
      DEBUG_SERIAL.printf("Motorstrom gespeichert: DAC=%u\n", motorCurrentDac);
    }
    stateDirty = true;
  }
  else if (strcmp(type, "setGateThreshold") == 0)
  {
    // Schwelle(n) eines Gates setzen. Tatsaechlich geschrieben wird im loop()
    // (serviceGateWrites), da der Sensor nur aus EINEM Kontext bedient wird.
    int gate = doc["gate"] | -1;
    if (gate >= 0 && gate < LD2410_GATES)
    {
      // Fehlende Werte mit der aktuell gesetzten Schwelle auffuellen (cached Read).
      long mov = doc["moving"] | (long)presenceSensor.getMovingThresholds().values[gate];
      long sta = doc["stationary"] | (long)presenceSensor.getStationaryThresholds().values[gate];
      if (mov < 0) mov = 0;
      if (mov > 100) mov = 100;
      if (sta < 0) sta = 0;
      if (sta > 100) sta = 100;
      pendingMovingThr[gate] = (uint8_t)mov;
      pendingStatThr[gate] = (uint8_t)sta;
      gateWritePending[gate] = true;
    }
  }
  else if (strcmp(type, "setMaxGate") == 0)
  {
    // Reichweite (hoechstes aktives Gate) setzen. Anwendung im loop().
    int g = doc["value"] | -1;
    if (g >= 1 && g <= 8)
      pendingMaxGate = g;
  }
}

void onWsMessage(uint8_t *data, size_t len)
{
  JsonDocument doc;
  if (deserializeJson(doc, data, len))
  {
    return; // ungueltiges JSON ignorieren
  }
  handleCommand(doc);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    // Neuem Client sofort den vollstaendigen Zustand schicken
    client->text(buildStateJson());
    break;

  case WS_EVT_DATA:
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      onWsMessage(data, len);
    }
    break;
  }

  case WS_EVT_DISCONNECT:
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

// ************************************************************************
// Initialisierungsfunktionen
// ************************************************************************

void initializeSerial()
{
  DEBUG_SERIAL.begin(115200);
  delay(100);
}

void initializeFileSystem()
{
  if (!LittleFS.begin(true))
  {
    DEBUG_SERIAL.println("LittleFS konnte nicht gemountet werden!");
  }
}

void initializeSensors()
{
  deviceSensor.beginOutputObservation(onDevicePresenceChange);
  devicePresent = deviceSensor.isPresent();
}

void initializePresenceSensor()
{
  LD2410_SERIAL.begin(LD2410_BAUD, SERIAL_8N1, LD2410_RX_PIN, LD2410_TX_PIN);
  delay(200); // dem Sensor nach dem Power-up Zeit geben

  // begin() antwortet beim Kaltstart nicht immer sofort -> mehrere Versuche
  const int maxAttempts = 5;
  for (int attempt = 1; attempt <= maxAttempts; attempt++)
  {
    if (presenceSensor.begin())
    {
      sensorVersion = presenceSensor.getFirmware();
      DEBUG_SERIAL.printf("LD2410 verbunden (Versuch %d). Firmware: %s\n", attempt, sensorVersion.c_str());

      // Aktuelle Gate-Schwellen einlesen (setzt maxRange != 0, sodass die
      // Getter spaeter KEIN UART-IO mehr ausloesen -> aus WS-Kontext gefahrlos).
      presenceSensor.requestParameters();
      // Engineering-Mode: Sensor streamt zusaetzlich die Live-Energie pro Gate.
      presenceSensor.enhancedMode(true);
      return;
    }
    DEBUG_SERIAL.printf("LD2410 Init fehlgeschlagen (Versuch %d/%d)\n", attempt, maxAttempts);
    delay(300);
  }

  sensorVersion = "";
  DEBUG_SERIAL.println("LD2410 nicht erreichbar!");
}

void initializeLED()
{
  logoLED.begin();
  logoLED.setBrightness(LED_BRIGHTNESS);
  applyLed(false, 0, 0, 0); // initial aus; controlStep() setzt den echten Zustand
}

void initializeMotorControl()
{
  pinMode(STEPPER_SLEEP, OUTPUT);
  digitalWrite(STEPPER_SLEEP, HIGH);

  pinMode(STEPPER_ENABLE, OUTPUT);
  digitalWrite(STEPPER_ENABLE, HIGH);

  I2C.begin(DAC_SDA_PIN, DAC_SCL_PIN);
  dacOk = dac.begin(DAC_I2C_ADDRESS, &I2C);
  if (!dacOk)
  {
    DEBUG_SERIAL.println("WARNUNG: MCP4725 (DAC) nicht gefunden -> Motor erhaelt keinen Strom!");
  }

  prefs.begin("tabmount", false);

  // Zuletzt eingestellten Motorstrom (VREF) wiederherstellen
  motorCurrentDac = prefs.getUShort("dacValue", DAC_MOTOR_CURRENT);
  dac.setVoltage(motorCurrentDac, false);
  DEBUG_SERIAL.printf("Motorstrom (DAC) gesetzt: %u\n", motorCurrentDac);

  // Zuletzt persistierten Display-Zustand wiederherstellen, OHNE den Motor zu
  // bewegen. Der Magnet behaelt seine physische Position ueber Reboots hinweg;
  // so bleiben Firmware-Zustand und Realitaet konsistent und der Motor faehrt
  // nicht erneut gegen den Anschlag.
  bool storedDisplay = prefs.getBool("displayOn", false);
  displayController.setActivationState(storedDisplay);
  manualDisplayOn = storedDisplay; // sinnvoller Startwert fuer den manuellen Modus
  DEBUG_SERIAL.printf("Display-Zustand wiederhergestellt: %s\n", storedDisplay ? "EIN" : "AUS");
}

// Oeffnet den Konfigurations-Access-Point. SSID enthaelt eine geraetespezifische
// Hex-Kennung (aus der eFuse-MAC), damit mehrere Geraete unterscheidbar sind.
void initializeAccessPoint()
{
  char ssid[32];
  // Geraetespezifischer Teil der MAC = obere 3 Bytes. Die unteren 3 Bytes sind
  // der herstellergleiche OUI-Praefix und waeren bei baugleichen Boards identisch.
  uint32_t id = (uint32_t)((ESP.getEfuseMac() >> 24) & 0xFFFFFF);
  snprintf(ssid, sizeof(ssid), AP_SSID_PREFIX "%06X", id);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, AP_PASSWORD);

  DEBUG_SERIAL.printf("AP gestartet: %s (PW: %s)\n", ssid, AP_PASSWORD);
  DEBUG_SERIAL.print("AP-IP-Adresse: ");
  DEBUG_SERIAL.println(WiFi.softAPIP());

  if (MDNS.begin(HOSTNAME))
  {
    MDNS.addService("http", "tcp", 80);
    DEBUG_SERIAL.printf("mDNS aktiv: http://%s.local\n", HOSTNAME);
  }
}

void initializeWebServer()
{
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Statische Web-Assets (index.html, style.css, app.js) aus dem LittleFS.
  // Waehrend der Entwicklung NICHT cachen, damit Browser immer die aktuelle
  // Version laden (verhindert stale/mismatched HTML+JS).
  server.serveStatic("/", LittleFS, "/")
      .setDefaultFile("index.html")
      .setCacheControl("no-store");

  server.begin();
  DEBUG_SERIAL.println("HTTP-Server gestartet");
}

// Schreibt ausstehende Gate-Schwellen auf den Sensor. Laeuft im loop() (einziger
// Radar-Kontext). setGateParameters() ist blockierend und regelt den Config-Mode
// selbst -> daher pro Durchlauf nur EIN Gate, damit der loop reaktionsfaehig bleibt.
void serviceGateWrites()
{
  if (sensorVersion.length() == 0)
    return;

  // Reichweite (max. Gate) zuerst anwenden, falls gewuenscht. setMaxGate setzt
  // dasselbe Gate fuer Bewegung und Ruhe und behaelt das No-One-Window bei.
  if (pendingMaxGate >= 0)
  {
    byte g = (byte)pendingMaxGate;
    pendingMaxGate = -1;
    bool ok = presenceSensor.setMaxGate(g, g, presenceSensor.getNoOneWindow());
    DEBUG_SERIAL.printf("Max-Gate gesetzt: %u (%s)\n", g, ok ? "OK" : "FAIL");
    presenceSensor.enhancedMode(true);
    stateDirty = true;
    return;
  }

  for (byte g = 0; g < LD2410_GATES; g++)
  {
    if (!gateWritePending[g])
      continue;

    uint8_t mov = pendingMovingThr[g];
    uint8_t sta = pendingStatThr[g];
    gateWritePending[g] = false; // vor dem Schreiben loeschen (neuere Wuensche gehen nicht verloren)

    bool ok = presenceSensor.setGateParameters(g, mov, sta);
    DEBUG_SERIAL.printf("Gate %u Schwellen: mov=%u sta=%u (%s)\n", g, mov, sta, ok ? "OK" : "FAIL");

    // Die Config-Session kann den Engineering-Mode beenden -> wieder aktivieren,
    // damit weiterhin Live-Energie pro Gate gestreamt wird.
    presenceSensor.enhancedMode(true);
    stateDirty = true;
    return; // nur ein Gate pro loop()-Durchlauf
  }
}

// ************************************************************************
// Steuerlogik (Policy)
//
// Einzige Stelle, die die Aktoren (Display-Motor, Logo-LED) ansteuert.
// Berechnet aus Modus + Sensoren die Soll-Ausgaenge und wendet sie an.
// Wird ausschliesslich im loop() aufgerufen (Motorbewegung ist blockierend).
// ************************************************************************

void controlStep()
{
  bool sensorOnline = sensorVersion.length() > 0;

  // Anwesenheit nur ausserhalb des Settle-Fensters frisch uebernehmen. Waehrend/
  // direkt nach einer Motorbewegung bleibt der letzte Wert eingefroren, damit die
  // Eigenbewegung des Motors keine Person vortaeuscht (Rueckkopplung verhindern).
  // Vergleich als vorzeichenbehaftete Differenz -> rollover-sicher (~49 Tage).
  bool masked = (long)(millis() - sensorMaskUntil) < 0;
  if (!sensorOnline)
  {
    personDetected = false;
  }
  else if (!masked)
  {
    personDetected = presenceSensor.presenceDetected();
  }

  bool targetDisplay;
  bool ledOn;
  uint8_t r, g, b;

  if (manualMode)
  {
    // Manuell: Display und Farbe kommen vom Nutzer, LED dauerhaft an.
    targetDisplay = manualDisplayOn;
    ledOn = true;
    r = ledR;
    g = ledG;
    b = ledB;
  }
  else
  {
    // Automatik: Display folgt der Anwesenheit, ABER nur wenn ein iPad anliegt
    // (ohne iPad gibt es nichts zu steuern -> Motor nicht unnoetig bewegen).
    // LED nur an, wenn kein iPad anliegt, Farbe je nach Anwesenheit.
    targetDisplay = devicePresent ? personDetected : displayController.getActivationState();
    ledOn = !devicePresent;
    if (personDetected)
    {
      r = AUTO_PERSON_R, g = AUTO_PERSON_G, b = AUTO_PERSON_B; // weiss
    }
    else
    {
      r = AUTO_IDLE_R, g = AUTO_IDLE_G, b = AUTO_IDLE_B; // blau
    }
  }

  // Display anwenden: nur bewegen, wenn sich der Zustand aendert.
  if (targetDisplay != displayController.getActivationState())
  {
    displayController.activate(targetDisplay);
    // Eigenbewegung ausblenden: Radar fuer das Settle-Fenster maskieren.
    sensorMaskUntil = millis() + SENSOR_SETTLE_MS;
    // Physische Position persistieren (Grind-Vermeidung beim Boot). Schreibrate
    // ist durch echte Motorbewegungen begrenzt; NVS verteilt die Writes (wear-
    // leveling), daher unkritisch.
    prefs.putBool("displayOn", displayController.getActivationState());
    stateDirty = true;
  }

  // LED anwenden (idempotent, schreibt nur bei Aenderung).
  applyLed(ledOn, r, g, b);
}

// ************************************************************************
// Arduino Hauptfunktionen
// ************************************************************************

void setup()
{
  initializeSerial();
  initializeSensors();
  initializePresenceSensor();
  initializeLED();
  initializeMotorControl();
  initializeFileSystem();
  initializeAccessPoint(); // startet AP + Netzwerk-Stack (sofort verfuegbar)
  initializeWebServer();   // braucht den initialisierten Stack
}

void loop()
{
  // LD2410-Datenframes einlesen. Bei neuen Daten wird ein Broadcast angestossen,
  // jedoch gedrosselt, da sich Distanz/Status laufend aendern.
  if (presenceSensor.check() == MyLD2410::Response::DATA)
  {
    unsigned long now = millis();
    if (now - lastSensorBroadcast >= SENSOR_BROADCAST_INTERVAL)
    {
      lastSensorBroadcast = now;
      stateDirty = true;
    }
  }

  // Ausstehende Gate-Schwellen auf den Sensor schreiben (ein Gate pro Durchlauf).
  serviceGateWrites();

  // Steuerlogik: Soll-Ausgaenge berechnen und anwenden (Motor/LED).
  controlStep();

  // Zustandsaenderungen an alle Clients pushen.
  if (stateDirty)
  {
    stateDirty = false;
    broadcastState();
  }

  ws.cleanupClients();
}
