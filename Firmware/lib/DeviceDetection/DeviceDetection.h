// DeviceDetection.h
//
// Interrupt-basierte Erkennung des iPads ueber den Hall-Sensor.
// Der Sensor ist aktiv LOW: LOW => iPad anliegend (present).
#ifndef DEVICEDETECTION_H
#define DEVICEDETECTION_H

#include <Arduino.h>

class DeviceDetection
{
public:
    // Pin und Pin-Modus (INPUT oder INPUT_PULLUP); initialState wird gesetzt.
    DeviceDetection(uint8_t pin, uint8_t mode, volatile bool &initialState);

    // Startet die Beobachtung; callback wird bei jeder Zustandsaenderung gerufen.
    bool beginOutputObservation(void (*callback)(bool));

    // Stoppt die Beobachtung.
    void endOutputObservation();

    // Aktueller Zustand (true => iPad anliegend).
    bool isPresent() const;

private:
    uint8_t _pin;
    uint8_t _mode;

    bool _monitoring;
    bool _currentState;
    void (*_callback)(bool);

    static void IRAM_ATTR handleInterrupt(void *arg);
};

#endif
