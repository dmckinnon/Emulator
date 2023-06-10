#pragma once
#ifdef RP2040

#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "Adafruit_GFX.h"
#include "build/st7789_driver.pio.h"

#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define DISPLAY_WIDTH 160
#define DISPLAY_HEIGHT 144

// This extends Adafruit's GFX canvas class to add in the
// functionality that will draw the buffer to the LCD.
// It also performs the other tasks of setting up the LCD
// This saves me managing a separate canvas and then 
// 
// TODO:
// there's an optimisation here where we can get the SPI
// PIO port to send 32 bits at a time instead of 8 bits
class ST7789
{
public:
    ST7789();

    void InitLCD();
    void SetDisplayArea(int x, int y);
    void ClearScreen(bool black);

    // This uses the current width and height and assumes
    // that that is the size of the buffer
    void WriteBuffer(uint16_t* buffer);
private:
    uint8_t currentWidth;
    uint8_t currentHeight;

    uint sm;
    PIO pio;

    bool initialised;
};

#endif