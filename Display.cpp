#include "Display.h"

#include <chrono>
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
    frameBuffer = cv::Mat(cv::Size(GAMEBOY_HEIGHT, GAMEBOY_WIDTH), CV_8UC1, cv::Scalar(0));

    // Start display thread
    windowThread = std::thread(Display::WindowThreadProcThunk, this);

    // TODO when LCD is disabled set mode to 1
}

Display::~Display()
{
}

bool Display::IsLCDEnabled()
{
    return mmu->ReadFromAddress(MMU::LCDControlAddress) & LCDEnableBit;
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
    using namespace std::chrono_literals;
    while (true)
    {
        if (!IsLCDEnabled())
        {
            // wait a frame
            continue;
        }

        // update image
        // simple: draw each scanline based on whether background or sprites or window are enabled
        // complex: draw that which changes

        // flash to curFrame object
        cv::imshow("GameBoy", frameBuffer);

        // sleep for 1/60th of a second
        std::this_thread::sleep_for(16ms);

        // one scanline takes 456 clock cycles, and goes status 2 -> 3 -> 0
        // vblank is mode 1
        // when mode changes to 0, 1, 2 then this is LCD interrupt
    }
}

void Display::UpdateLCDStatus()
{
    uint8_t currentStatus = mmu->ReadFromAddress(MMU::LCDStatusAddress);

    if (!IsLCDEnabled())
    {
        // set mode to 1
        currentStatus &= LCDModeMask;
        currentStatus |= VBlankMode;

        // reset scanline counter. Writing to this address zeroes the value regardless of input
        mmu->WriteToAddress(MMU::ScanLineCounterAddress, (uint8_t)0x00);

        return;
    }

    uint8_t oldMode = currentStatus & LCDModeMask;
    uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);

    // Check which mode we should be in
    uint8_t requestInterrupt = 0;
    if (currentScanLine >= VisibleScanlines)
    {
        // Mode 1 VBlank
        currentStatus &= LCDModeMask;
        currentStatus |= VBlankMode;
        requestInterrupt = currentStatus & VBlankInternalInterruptBit;
    }
    else
    {
        if (scanlineCycleCounter < CyclesForMode2)
        {
            // Mode 2 Searching for Sprite Attributes
            currentStatus &= LCDModeMask;
            currentStatus |= SearchingSpriteAttrsMode;
            requestInterrupt = currentStatus & SpriteInternalInterruptBit;
        }
        else if (scanlineCycleCounter < CyclesForMode2 + CyclesForMode3)
        {
            // Mode 3 Transfer Data to LCD
            currentStatus &= LCDModeMask;
            currentStatus |= XferDataToLcdMode;
        }
        else
        {
            // Mode 0 HBlank
            currentStatus &= LCDModeMask;
            currentStatus |= HBlankMode;
            requestInterrupt = currentStatus & HBlankInternalInterruptBit;
        }
    }

    // if we are not in mode 3, but mode has changed,
    // request LCD stat interrupt if mode change interrupt bit is set
    uint8_t newMode = currentStatus & LCDModeMask;
    if (requestInterrupt && newMode != oldMode)
    {
        SetLCDStatInterrupt();
    }

    // Check coincidence flag - is this the scanline we're looking for?
    uint8_t desiredScanline = mmu->ReadFromAddress(MMU::DesiredScanlineRegisterAddress);
    if (currentScanLine = desiredScanline)
    {
        // set the coincidence bit in status register
        // Fire interrupt if coincidence interrupt bit is set
        currentStatus |= CoincidenceBit;
        if (currentStatus & CoincidenceInternalInterruptBit)
        {
            SetLCDStatInterrupt();
        }
    }
    else
    {
        // clear coincidence bit
        currentStatus &= ~CoincidenceBit;
    }

    // Write status back out
    mmu->WriteToAddress(MMU::LCDStatusAddress, currentStatus);
}