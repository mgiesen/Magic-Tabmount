/*
    Der DisplayController aktiviert das iPad-Display durch eine Verschiebung des
    Trigger-Magneten. Die Verschiebung erfolgt durch einen Stepper-Motor (A4988).
    Die Bewegung wird durch die activate-Methode gesteuert:
    true  => Display aktivieren (Magnet verschieben),
    false => Display deaktivieren (Magnet zurueck).
*/

#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include "A4988.h"

class DisplayController
{
public:
    DisplayController(int stepPin, int dirPin, int enablePin, int sleepPin, int activationRevolutions, int motorSteps, int speed);

    void activate(bool active);          // Aktiviert oder deaktiviert das iPad-Display
    bool getActivationState();           // Gibt den Aktivierungsstatus zurueck
    void setActivationState(bool active); // Setzt den internen Zustand OHNE Motorbewegung (z.B. Wiederherstellung beim Boot)
    void setSpeed(int speed);            // Setzt die Geschwindigkeit des Motors

private:
    int _stepPin;
    int _dirPin;
    int _enablePin;
    int _sleepPin;
    bool _isActive;
    int _speed;
    int _activationRevolutions;
    int _motorSteps;
    int _activationSteps = _activationRevolutions * _motorSteps;

    A4988 _stepper;

    void motorEnable(bool enable);
};

#endif
