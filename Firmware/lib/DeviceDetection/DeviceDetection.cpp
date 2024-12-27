// DeviceDetection.cpp
#include "DeviceDetection.h"

DeviceDetection::DeviceDetection(uint8_t pin, uint8_t mode, volatile bool &initialState)
    : _pin(pin), _mode(mode), _monitoring(false), _currentState(false), _callback(nullptr)
{
    initialState = digitalRead(_pin) == LOW;
}

bool DeviceDetection::beginOutputObservation(void (*callback)(bool))
{
    if (callback == nullptr)
    {
        return false;
    }

    // Disable existing interrupt if already monitoring
    if (_monitoring)
    {
        endOutputObservation();
    }

    // Store callback and configure pin
    _callback = callback;
    pinMode(_pin, _mode);
    _currentState = digitalRead(_pin) == LOW;

    // Attach interrupt for both rising and falling edges
    attachInterruptArg(
        digitalPinToInterrupt(_pin),
        handleInterrupt,
        this,
        CHANGE);

    _monitoring = true;
    return true;
}

void DeviceDetection::endOutputObservation()
{
    if (_monitoring)
    {
        detachInterrupt(digitalPinToInterrupt(_pin));
        _monitoring = false;
    }
}

bool DeviceDetection::isPresent() const
{
    return _currentState;
}

void IRAM_ATTR DeviceDetection::handleInterrupt(void *arg)
{
    DeviceDetection *instance = static_cast<DeviceDetection *>(arg);
    if (instance && instance->_monitoring && instance->_callback)
    {
        bool newState = digitalRead(instance->_pin) == LOW;
        if (newState != instance->_currentState)
        {
            instance->_currentState = newState;
            instance->_callback(newState);
        }
    }
}