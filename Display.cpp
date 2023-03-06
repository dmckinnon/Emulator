#include "Display.h"

#include <iostream>

#define PIXELS_PER_PIXEL 4

Display::Display(
    std::function<void()> setVBlankInterrupt,
    std::function<void()> setLCDStatInterrupt,
    std::function<void(uint8_t)> setJoypadInterrupt) :
    SetVBlankInterrupt(setVBlankInterrupt),
    SetLCDStatInterrupt(setLCDStatInterrupt),
    SetJoypadInterrupt(setJoypadInterrupt)
    //curFrame("/home/dave/Emulator/blank.png")
{
    //gtk_init(0, nullptr);
    // make image blank
    for (int i = 0; i < GAMEBOY_HEIGHT; ++i)
    {
        memset(frameBuffer[i], 0,GAMEBOY_WIDTH*3);
    }

    // Start display thread
    windowThread = std::thread(Display::WindowThreadProcThunk, this);
}

Display::~Display()
{
}

void Display::WindowThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->WindowThreadProc();
}

void Display::FrameThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->FrameThreadProc();
}

void Display::WindowThreadProc()
{
    
}

void Display::FrameThreadProc()
{
    while (true)
    {
        // update image

        // flash to curFrame object


        // sleep for 1/60th of a second
    }
}