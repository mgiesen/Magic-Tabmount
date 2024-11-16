#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"

class FirmwareUpdateManager
{
private:
    AsyncWebServer *server;
    const char *updatePath = "/update";
    const char *updateUsername;
    const char *updatePassword;
    bool updating = false;

    // Callback Handler für den Update-Prozess
    void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
    {
        if (!index)
        {
            Serial.println("Update Start");
            updating = true;

            // Prüfe ob genug Speicher verfügbar ist
            if (!Update.begin(UPDATE_SIZE_UNKNOWN))
            {
                Update.printError(Serial);
                return;
            }
        }

        if (Update.write(data, len) != len)
        {
            Update.printError(Serial);
            return;
        }

        if (final)
        {
            if (!Update.end(true))
            {
                Update.printError(Serial);
                return;
            }
            Serial.println("Update Success");
            Serial.println("Rebooting...");
            updating = false;
            ESP.restart();
        }
    }

    void addUpdateEndpoints()
    {
        // Status Endpoint
        server->on("/ota/status", HTTP_GET, [this](AsyncWebServerRequest *request)
                   {
            StaticJsonDocument<200> doc;
            doc["updating"] = this->updating;
            doc["progress"] = Update.progress();
            doc["total"] = Update.size();
            
            String response;
            serializeJson(doc, response);
            request->send(200, "application/json", response); });

        // Update Endpoint
        server->on("/ota/update", HTTP_POST, [](AsyncWebServerRequest *request)
                   {
                bool success = !Update.hasError();
                AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", success ? "OK" : "FAIL");
                response->addHeader("Connection", "close");
                request->send(response); }, [this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
                   { this->handleUpload(request, filename, index, data, len, final); });
    }

public:
    FirmwareUpdateManager(AsyncWebServer *server, const char *username = "admin", const char *password = "admin")
        : server(server), updateUsername(username), updatePassword(password) {}

    void begin()
    {
        addUpdateEndpoints();

        // Stelle Update-Interface bereit
        server->on("/ota", HTTP_GET, [this](AsyncWebServerRequest *request)
                   {
            if(!request->authenticate(this->updateUsername, this->updatePassword))
                return request->requestAuthentication();
            request->send(SPIFFS, "/ota.html", "text/html"); });
    }
};
