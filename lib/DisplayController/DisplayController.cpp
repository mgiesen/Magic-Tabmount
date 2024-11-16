#include "DisplayController.h"

DisplayController::DisplayController(int stepPin, int dirPin, int enablePin, int sleepPin, int activationRevolutions, int motorSteps, int speed)
    : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin), _sleepPin(sleepPin), _activationRevolutions(activationRevolutions), _motorSteps(motorSteps), _speed(speed), _stepper(motorSteps, dirPin, stepPin, enablePin)
{
    _isActive = false;
    _stepper.begin(_speed);
    _stepper.disable();
}

void DisplayController::motorEnable(bool enable)
{
    enable = !enable; // Zustand aus semantischen Gründen umkehren

    if (enable)
    {
        _stepper.enable();
    }
    else
    {
        _stepper.disable();
    }
}

void DisplayController::activate(bool active)
{
    if (_isActive == active)
    {
        return;
    }

    motorEnable(true);

    if (active)
    {
        _stepper.move(_activationSteps);
    }
    else
    {
        _stepper.move(_activationSteps * -1);
    }

    motorEnable(false);

    _isActive = active;
}

bool DisplayController::getActivationState()
{
    return _isActive;
}

void DisplayController::setSpeed(int speed)
{
    _speed = speed;
    _stepper.setRPM(_speed);
}
