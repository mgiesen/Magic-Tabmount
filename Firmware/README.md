# Firmware

ESP32-S3-Firmware des Magic Tabmount. Sie steuert den Display-Magneten (Stepper-Motor), die Logo-LEDs und die Anwesenheitserkennung (LD2410-Radar) und stellt ein Konfigurations-Dashboard über einen WLAN-Access-Point bereit.

## Voraussetzungen

- [PlatformIO](https://platformio.org/) (CLI oder VS-Code-Erweiterung)
- Board: `esp32-s3-devkitc-1` (in `platformio.ini` vorkonfiguriert)

Externe Bibliotheken lädt PlatformIO automatisch anhand der `lib_deps` in `platformio.ini` — ein manueller Schritt ist nicht nötig. Projekteigene Bibliotheken liegen direkt unter `lib/` (`DeviceDetection`, `DisplayController`). Das Projekt nutzt **keine** Git-Submodule.

## Bauen und Flashen

Es gibt zwei Upload-Ziele:

```bash
# Firmware (src/) bauen und flashen
pio run -t upload

# Web-Dashboard (data/) als LittleFS-Image flashen
pio run -t uploadfs
```

Nach Änderungen an `data/` (HTML/CSS/JS) ist `uploadfs` erforderlich — ein normaler `upload` aktualisiert das Dashboard nicht.

## Konfiguration

Nach dem Start öffnet das Gerät dauerhaft einen WLAN-Access-Point `Magic-Tabmount-<ID>` (Passwort `tabmount`). Die `<ID>` wird aus dem geräteindividuellen Teil der MAC-Adresse abgeleitet und ist je Gerät eindeutig. Das Dashboard ist anschließend unter `http://192.168.4.1` oder `http://Magic-Tabmount.local` erreichbar.

Die Kernsteuerung (Anwesenheitserkennung, Display, LEDs) läuft ab dem Boot unabhängig vom WLAN.
