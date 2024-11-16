// ************************************************************************
// Includes und Definitionen
// ************************************************************************

// Kern-Bibliotheken für Arduino & ESP32
#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <Wire.h>

// Kommunikationsbibliotheken
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MCP4725.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Benutzerdefinierte Hardware-Bibliotheken
#include "LogoLED.h"
#include "DeviceDetection.h"
#include "DisplayController.h"
#include "LD2410.h"
#include "FirmwareUpdate.h"

// ************************************************************************
// Pin und Konstanten Definitionen
// ************************************************************************

// Namensgebung
#define DEBUG_SERIAL Serial
#define RADAR_SERIAL Serial2

// Logo LED
#define NUMBER_OF_LEDS 4
#define LED_PIN 6

// Geräteerkennung
#define DEVICE_SENSOR_PIN 4

// LD2410 Sensor zur Anwesenheitserkennung
#define HUMAN_PRESENCE_PIN 5
#define HUMAN_PRESENCE_RX 17
#define HUMAN_PRESENCE_TX 18

// Schrittmotor
#define STEP_PIN 9
#define DIR_PIN 10
#define STEPPER_ENABLE 8
#define STEPPER_SLEEP 15

#define DAC_I2C_ADDRESS 0x60
#define DAC_SDA_PIN 14
#define DAC_SCL_PIN 13

// ************************************************************************
// Globale Variablen und Instanzen
// ************************************************************************

// Konfiguration des DAC
Adafruit_MCP4725 dac;
TwoWire I2C = TwoWire(0);

// Konfiguration des Webservers
IPAddress apIP(192, 168, 4, 1);
const char *ssid = "Magic Tabmount DEV";
const char *password = "12345678";

// Webserver- und WebSocket-Instanzen
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Vorwärtsdeklarationen
void onHumanPresenceChange(bool presence);
void onDevicePresenceChange(bool presence);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

// Laufzeitvariablen
volatile bool humanPresent = false;
volatile bool devicePresent = false;
volatile bool stateChange = true;

// Instanzen
LD2410 humanSensor;
LogoLED<LED_PIN> logoLED(NUMBER_OF_LEDS);
DeviceDetection deviceSensor(DEVICE_SENSOR_PIN, INPUT_PULLUP, devicePresent);
DisplayController displayController(STEP_PIN, DIR_PIN, STEPPER_ENABLE, STEPPER_SLEEP, 10, 3500, 700);
FirmwareUpdateManager *firmwareUpdateManager;

// Logo Helligkeit
int logoBrightness = 230;

// Warteschlange für Statusänderungen
struct StateUpdate
{
  bool human;
  bool device;
  bool valid;
} stateQueue;

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

// Timer für verzögerte Aktualisierungen
hw_timer_t *updateTimer = NULL;
volatile bool updatePending = false;

// ************************************************************************
// Interrupt Service Routinen
// ************************************************************************

void IRAM_ATTR onUpdateTimer()
{
  updatePending = true;
}

void IRAM_ATTR onHumanPresenceChange(bool presence)
{
  portENTER_CRITICAL_ISR(&stateMux);
  humanPresent = presence;
  stateQueue.human = presence;
  stateQueue.valid = true;
  stateChange = true;
  portEXIT_CRITICAL_ISR(&stateMux);
}

void IRAM_ATTR onDevicePresenceChange(bool presence)
{
  portENTER_CRITICAL_ISR(&stateMux);
  devicePresent = presence;
  stateQueue.device = presence;
  stateQueue.valid = true;
  stateChange = true;
  portEXIT_CRITICAL_ISR(&stateMux);
}

// ************************************************************************
// WebSocket Funktionen
// ************************************************************************

void notifyClients()
{
  if (ws.count() > 0)
  { // Nur senden, wenn Clients verbunden sind
    StaticJsonDocument<200> doc;
    doc["humanPresent"] = humanPresent;
    doc["devicePresent"] = devicePresent;
    doc["logoBrightness"] = logoBrightness;

    String jsonString;
    serializeJson(doc, jsonString);

    ws.textAll(jsonString);
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    String message = String((char *)data).substring(0, len);

    if (message.startsWith("setBrightness:"))
    {
      logoBrightness = message.substring(14).toInt();
      logoLED.setBrightness(logoBrightness);
      notifyClients();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    DEBUG_SERIAL.printf("WebSocket-Client #%u verbunden von %s\n", client->id(), client->remoteIP().toString().c_str());
    notifyClients();
    break;
  case WS_EVT_DISCONNECT:
    DEBUG_SERIAL.printf("WebSocket-Client #%u getrennt\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

// ************************************************************************
// Initialisierungsfunktionen
// ************************************************************************

void initializeLED()
{
  logoLED.begin();
  logoLED.setBrightness(logoBrightness);
  logoLED.setColor(255, 255, 255);
  logoLED.on();
}

void initializeMotorControl()
{
  pinMode(STEPPER_SLEEP, OUTPUT);
  digitalWrite(STEPPER_SLEEP, HIGH);

  I2C.begin(DAC_SDA_PIN, DAC_SCL_PIN);
  if (!dac.begin(DAC_I2C_ADDRESS, &I2C))
  {
    DEBUG_SERIAL.println("MCP4725 nicht gefunden!");
  }
  dac.setVoltage(1350, false);
}

void initializeFileSystem()
{
  if (!SPIFFS.begin(true))
  {
    DEBUG_SERIAL.println("SPIFFS Fehler beim Mounten");
  }
}

void initializeWiFi()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

  DEBUG_SERIAL.print("AP IP Adresse: ");
  DEBUG_SERIAL.println(WiFi.softAPIP());
}

void initializeWebServer()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // OTA Manager initialisieren
  firmwareUpdateManager = new FirmwareUpdateManager(&server, "admin", "admin");
  firmwareUpdateManager->begin();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  server.begin();
  DEBUG_SERIAL.println("HTTP Server gestartet");
}

void initializeTimer()
{
  updateTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(updateTimer, &onUpdateTimer, true);
  timerAlarmWrite(updateTimer, 100000, true);
  timerAlarmEnable(updateTimer);
}

void initializeSensors()
{
  deviceSensor.beginOutputObservation(onDevicePresenceChange);

  humanSensor.useDebug(DEBUG_SERIAL);
  humanSensor.beginUART(HUMAN_PRESENCE_TX, HUMAN_PRESENCE_RX, RADAR_SERIAL);
  humanSensor.beginOutputObservation(HUMAN_PRESENCE_PIN, onHumanPresenceChange, INPUT_PULLDOWN);

  stateQueue.valid = false;
}

void initializeSerial()
{
  delay(10);
  DEBUG_SERIAL.begin(115200);
  delay(100);
}

void initializeOTA()
{
  // Standard ArduinoOTA konfigurieren
  ArduinoOTA
      .setHostname("Magic-Tabmount-DEV") // Optional: Name im Netzwerk
      .setPassword("73849502537")        // Optional: Passwort für Upload
      .setPort(8266)                     // Optional: Port für Updates
      .onStart([]()
               {
            // Wichtige Tasks stoppen
            displayController.activate(false);
            logoLED.off(); })
      .onEnd([]()
             {
               // Optional: Ende-Handling
             })
      .onProgress([](unsigned int progress, unsigned int total)
                  {
            // Optional: Fortschritt anzeigen
            float percentage = (progress / (float)total) * 100;
            DEBUG_SERIAL.printf("Progress: %u%%\r", percentage); })
      .onError([](ota_error_t error)
               {
            DEBUG_SERIAL.printf("Error[%u]: ", error);
            switch (error) {
                case OTA_AUTH_ERROR: 
                    DEBUG_SERIAL.println("Auth Failed");
                    break;
                case OTA_BEGIN_ERROR: 
                    DEBUG_SERIAL.println("Begin Failed");
                    break;
                case OTA_CONNECT_ERROR: 
                    DEBUG_SERIAL.println("Connect Failed");
                    break;
                case OTA_RECEIVE_ERROR: 
                    DEBUG_SERIAL.println("Receive Failed");
                    break;
                case OTA_END_ERROR: 
                    DEBUG_SERIAL.println("End Failed");
                    break;
            } });

  ArduinoOTA.begin();
}

// ************************************************************************
// Hauptfunktionen für den Loop
// ************************************************************************

void handleStateUpdates()
{
  if (updatePending)
  {
    portENTER_CRITICAL(&stateMux);
    bool shouldUpdate = stateQueue.valid;
    if (shouldUpdate)
    {
      stateQueue.valid = false;
    }
    portEXIT_CRITICAL(&stateMux);

    if (shouldUpdate)
    {
      notifyClients();
    }
    updatePending = false;
  }
}

void updateDeviceState()
{
  if (stateChange)
  {
    if (devicePresent == false)
    {
      if (humanPresent)
      {
        logoLED.setColor(0, 128, 0);
      }
      else
      {
        logoLED.setColor(255, 255, 255);
      }
    }
    else
    {
      logoLED.setColor(0, 0, 0);

      if (humanPresent)
      {
        displayController.activate(true);
      }
      else
      {
        displayController.activate(false);
      }
    }
    stateChange = false;
  }
}

// ************************************************************************
// Arduino Hauptfunktionen
// ************************************************************************

void setup()
{
  initializeSerial();
  initializeLED();
  initializeMotorControl();
  initializeFileSystem();
  initializeWiFi();
  initializeOTA();
  initializeWebServer();
  initializeTimer();
  initializeSensors();

  DEBUG_SERIAL.println("Setup abgeschlossen");
}

void loop()
{
  ArduinoOTA.handle();
  ws.cleanupClients();
  handleStateUpdates();
  updateDeviceState();
}