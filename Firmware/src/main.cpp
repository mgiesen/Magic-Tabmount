// ************************************************************************
// Magic Tabmount - Firmware
//
// Der ESP32 verbindet sich mit dem Heimnetz (WLAN-Client), ist unter dem
// Hostnamen "Magic-Tabmount" erreichbar und hostet ein schlichtes Dashboard.
//
// Kommunikation laeuft ueber eine einzige WebSocket-Verbindung (/ws):
//   - Server -> Client: pusht den Geraetezustand bei jeder Aenderung
//   - Client -> Server: sendet Befehle (Display schalten, LED-Farbe setzen)
//
// Zustand:
//   - iPad-Erkennung    : Hall-Sensor, interrupt-getrieben (DeviceDetection)
//   - Display ein/aus    : Stepper-Motor verschiebt den Trigger-Magneten
//   - Logo-Beleuchtung   : NeoPixel-LEDs, frei einstellbare Farbe
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
#include "secrets.h"

// ************************************************************************
// Pin- und Konstanten-Definitionen
// ************************************************************************

#define DEBUG_SERIAL Serial
#define HOSTNAME "Magic-Tabmount"

// Hall-Sensor zur iPad-Erkennung: aktiv LOW (LOW = iPad erkannt)
#define DEVICE_SENSOR_PIN 4

// Logo-LEDs (NeoPixel)
#define LED_PIN 6
#define NUMBER_OF_LEDS 4
#define LED_BRIGHTNESS 80

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

volatile bool devicePresent = false;  // vom Hall-Sensor-Interrupt gesetzt
volatile bool desiredDisplayOn = false; // vom Client gewuenschter Display-Zustand
volatile bool stateDirty = true;        // true => Zustand an Clients senden

// Drosselung der Live-Broadcasts der LD2410-Daten (Distanz aendert sich laufend)
unsigned long lastSensorBroadcast = 0;
const unsigned long SENSOR_BROADCAST_INTERVAL = 250; // ms -> max. 4 Updates/s

uint8_t ledR = 255, ledG = 255, ledB = 255; // aktuelle LED-Farbe (Default: Weiss)

uint16_t motorCurrentDac = DAC_MOTOR_CURRENT; // DAC-Wert (0-4095) -> VREF -> Motorstrom

String sensorVersion = ""; // LD2410 Firmware-Version (einmalig bei Boot gelesen)

DeviceDetection deviceSensor(DEVICE_SENSOR_PIN, INPUT_PULLUP, devicePresent);

// ************************************************************************
// Hardware-Hilfsfunktionen
// ************************************************************************

void applyLedColor()
{
  for (int i = 0; i < NUMBER_OF_LEDS; i++)
  {
    logoLED.setPixelColor(i, logoLED.Color(ledR, ledG, ledB));
  }
  logoLED.show();
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
  doc["present"] = devicePresent;
  doc["display"] = displayController.getActivationState();
  doc["sensorVersion"] = sensorVersion;

  // Live-Anwesenheit des LD2410 nur, wenn der Sensor erreichbar ist
  if (sensorVersion.length() > 0)
  {
    doc["personPresent"] = presenceSensor.presenceDetected();
  }

  char hex[8];
  snprintf(hex, sizeof(hex), "#%02x%02x%02x", ledR, ledG, ledB);
  doc["hex"] = hex;

  doc["dac"] = motorCurrentDac; // Motorstrom (VREF) als roher DAC-Wert 0-4095

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

  if (strcmp(type, "setDisplay") == 0)
  {
    desiredDisplayOn = doc["on"] | false;
    DEBUG_SERIAL.printf("Befehl setDisplay: %s\n", desiredDisplayOn ? "EIN" : "AUS");
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
      applyLedColor();
      stateDirty = true;
      DEBUG_SERIAL.printf("Befehl setLed: #%s\n", hex);
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
  applyLedColor();
}

void initializeMotorControl()
{
  pinMode(STEPPER_SLEEP, OUTPUT);
  digitalWrite(STEPPER_SLEEP, HIGH);

  pinMode(STEPPER_ENABLE, OUTPUT);
  digitalWrite(STEPPER_ENABLE, HIGH);

  I2C.begin(DAC_SDA_PIN, DAC_SCL_PIN);
  if (!dac.begin(DAC_I2C_ADDRESS, &I2C))
  {
    DEBUG_SERIAL.println("MCP4725 (DAC) nicht gefunden!");
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
  desiredDisplayOn = storedDisplay;
  DEBUG_SERIAL.printf("Display-Zustand wiederhergestellt: %s\n", storedDisplay ? "EIN" : "AUS");
}

void initializeWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true); // Verbindungsabbrueche im Betrieb selbst heilen
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  DEBUG_SERIAL.printf("Verbinde mit WLAN '%s'", WIFI_SSID);

  // Nach einem Reset assoziiert das WLAN gelegentlich nicht sofort (RF-Settling).
  // Statt ewig zu blockieren, wird begin() periodisch neu ausgeloest, bis die
  // Verbindung steht -> kein dauerhaftes "Geraet nicht online" mehr.
  unsigned long lastAttempt = millis();
  int attempt = 1;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    DEBUG_SERIAL.print(".");
    if (millis() - lastAttempt > 8000)
    {
      attempt++;
      DEBUG_SERIAL.printf("\nWLAN nicht verbunden, neuer Versuch (%d)...", attempt);
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      lastAttempt = millis();
    }
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("Verbunden. IP-Adresse: ");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void initializeMDNS()
{
  if (MDNS.begin(HOSTNAME))
  {
    MDNS.addService("http", "tcp", 80);
    DEBUG_SERIAL.printf("mDNS aktiv: http://%s.local\n", HOSTNAME);
  }
  else
  {
    DEBUG_SERIAL.println("mDNS konnte nicht gestartet werden");
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
  initializeWiFi();
  initializeMDNS();
  initializeWebServer();
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

  // Display-Wunsch umsetzen. Die Motorbewegung ist blockierend und darf
  // daher nicht im Async-Handler laufen, sondern nur hier im loop().
  if (desiredDisplayOn != displayController.getActivationState())
  {
    displayController.activate(desiredDisplayOn);
    prefs.putBool("displayOn", displayController.getActivationState());
    stateDirty = true;
  }

  // Zustandsaenderungen (Sensor-ISR, Display, LED) an alle Clients pushen.
  if (stateDirty)
  {
    stateDirty = false;
    broadcastState();
  }

  ws.cleanupClients();
}
