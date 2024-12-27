// DeviceDetection.h
#ifndef DEVICEDETECTION_H
#define DEVICEDETECTION_H

#include <Arduino.h>

class DeviceDetection
{
public:
    // Constructor: Takes pin number and optional pin mode (INPUT or INPUT_PULLUP)
    DeviceDetection(uint8_t pin, uint8_t mode, volatile bool &initialState);

    // Begin monitoring with callback function
    bool beginOutputObservation(void (*callback)(bool));

    // Stop monitoring
    void endOutputObservation();

    // Get current device state
    bool isPresent() const;

private:
    // Pin configuration
    uint8_t _pin;
    uint8_t _mode;

    // Monitoring state
    bool _monitoring;
    bool _currentState;
    void (*_callback)(bool);

    // Static interrupt handler
    static void IRAM_ATTR handleInterrupt(void *arg);
};

#endif