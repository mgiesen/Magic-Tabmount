// ************************************************************************
// Magic Tabmount - Firmware
//
// Stufe 1: Der ESP32 verbindet sich mit dem Heimnetz (WLAN-Client) und
// hostet ein Dashboard zur Steuerung des Hubs. Das Dashboard zeigt
// aktuell "Hello World".
// ************************************************************************

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "secrets.h"

// ************************************************************************
// Konstanten und globale Instanzen
// ************************************************************************

#define DEBUG_SERIAL Serial

AsyncWebServer server(80);

// ************************************************************************
// Dashboard (statisches HTML)
// ************************************************************************

const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Magic Tabmount</title>
  <style>
    body {
      margin: 0;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0f1115;
      color: #f5f5f7;
    }
    h1 { font-size: 2.5rem; margin: 0; }
    p { color: #8a8a8f; margin-top: 0.5rem; }
  </style>
</head>
<body>
  <h1>Hello World</h1>
  <p>Magic Tabmount Dashboard</p>
</body>
</html>
)rawliteral";

// ************************************************************************
// Initialisierungsfunktionen
// ************************************************************************

void initializeSerial()
{
  DEBUG_SERIAL.begin(115200);
  delay(100);
}

void initializeWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  DEBUG_SERIAL.printf("Verbinde mit WLAN '%s'", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUG_SERIAL.print(".");
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("Verbunden. IP-Adresse: ");
  DEBUG_SERIAL.println(WiFi.localIP());
}

void initializeWebServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", DASHBOARD_HTML); });

  server.begin();
  DEBUG_SERIAL.println("HTTP-Server gestartet");
}

// ************************************************************************
// Arduino Hauptfunktionen
// ************************************************************************

void setup()
{
  initializeSerial();
  initializeWiFi();
  initializeWebServer();
}

void loop()
{
  // AsyncWebServer arbeitet ereignisbasiert, der Loop bleibt frei.
}
