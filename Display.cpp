#include "Display.h"

#include <iostream>

#define PIXELS_PER_PIXEL 4

Display::Display(
    std::shared_ptr<MMU> mmu,
    std::function<void()> setVBlankInterrupt,
    std::function<void()> setLCDStatInterrupt,
    std::function<void(uint8_t)> setJoypadInterrupt) :
    mmu(mmu),
    SetVBlankInterrupt(setVBlankInterrupt),
    SetLCDStatInterrupt(setLCDStatInterrupt),
    SetJoypadInterrupt(setJoypadInterrupt),
    scanlineCycleCounter(0)
    //curFrame("/home/dave/Emulator/blank.png")
{
    // make image blank
    for (int i = 0; i < GAMEBOY_HEIGHT; ++i)
    {
        memset(frameBuffer[i], 0,GAMEBOY_WIDTH*3);
    }

    // Start display thread
    windowThread = std::thread(Display::WindowThreadProcThunk, this);

    // TODO when LCD is disabled set mode to 1
}

Display::~Display()
{
}

bool Display::IsLCDEnabled()
{
    return mmu->ReadFromAddress(MMU::LCDControlAddress) & MMU::LCDEnableBit;
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

        // one scanline takes 456 clock cycles, and goes status 2 -> 3 -> 0
        // vblank is mode 1
        // when mode changes to 0, 1, 2 then this is LCD interrupt
    }
}

void UpdateLCDStatus()
{
    uint8_t currentStatus = mmu->ReadFromAddress(MMU::LCDStatusAddress);

    if (!IsLCDEnabled())
    {
        // set mode to 1
        currentStatus &= LCDModeMask;
        currentStatus |= VBlankMode;

        // reset scanline counter. Writing to this address zeroes the value regardless of input
        mmu->WriteToAddress(MMU::ScanLineCounterAddress, 0);

        return;
    }

    uint8_t currentMode = currentStatus & LCDModeMask;
    uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);

    // Check which mode we should be in
    uint8_t newMode = 0;
    if (currentScanLine >= VisibleScanlines)
    {
        // Mode 1 VBlank
        currentStatus &= LCDModeMask;
        currentStatus |= VBlankMode;
    }
    else
    {
        if (scanlineCycleCounter < CyclesForMode2)
        {
            // Mode 2 Searching for Sprite Attributes
        }
        else if (scanlineCycleCounter < CyclesForMode2 + CyclesForMode3)
        {
            // Mode 3 Transfer Data to LCD

        }
        else
        {
            // Mode 0 HBlank
        }
    }

}