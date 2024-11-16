#ifndef LOGOLED_H
#define LOGOLED_H

#include <Adafruit_NeoPixel.h>

template <uint8_t DataPin>
class LogoLED
{
public:
    LogoLED(uint8_t numLeds);
    void begin();
    void on();
    void off();
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void setBrightness(uint8_t brightness);

private:
    uint8_t _numLeds;
    Adafruit_NeoPixel _pixels;
    bool _isOn;
};

template <uint8_t DataPin>
LogoLED<DataPin>::LogoLED(uint8_t numLeds) : _numLeds(numLeds),
                                             _pixels(numLeds, DataPin, NEO_GRB + NEO_KHZ800),
                                             _isOn(false)
{
}

template <uint8_t DataPin>
void LogoLED<DataPin>::begin()
{
    _pixels.begin();
    off();
}

template <uint8_t DataPin>
void LogoLED<DataPin>::on()
{
    _isOn = true;
    _pixels.show();
}

template <uint8_t DataPin>
void LogoLED<DataPin>::off()
{
    _isOn = false;
    for (int i = 0; i < _numLeds; i++)
    {
        _pixels.setPixelColor(i, 0);
    }
    _pixels.show();
}

template <uint8_t DataPin>
void LogoLED<DataPin>::setColor(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < _numLeds; i++)
    {
        _pixels.setPixelColor(i, _pixels.Color(r, g, b));
    }
    if (_isOn)
    {
        _pixels.show();
    }
}

template <uint8_t DataPin>
void LogoLED<DataPin>::setBrightness(uint8_t brightness)
{
    _pixels.setBrightness(brightness);
    if (_isOn)
    {
        _pixels.show();
    }
}

#endif