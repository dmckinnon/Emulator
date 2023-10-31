#pragma once

// This only works for not embedded
#ifndef RP2040

#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "MMU.h"

//#include "Display.h"

class Debugger
{
public:
    Debugger(std::shared_ptr<MMU> mmu);
    ~Debugger();

    void ShowTiles(uint16_t address);

    void DebuggerThread();

    void MaybeBlock();

private:
    std::thread debuggerThread;
    bool blockEveryFrame;
    bool moveToNextFrame = false;

    uint16_t tileAddress;

    std::mutex frameMutex;
    std::condition_variable debuggerBlockCV;

    std::shared_ptr<MMU> mmu;

    cv::Mat tiles;

    // largely copies of what is in Display
    void RenderTiles(
        uint8_t controlReg,
        uint16_t curScanline,
        uint16_t xOffset,
        uint8_t scrollX,
        uint8_t scrollY, 
        uint8_t windowX, 
        uint8_t windowY);
    uint8_t GetColour(uint8_t colourNum, uint16_t paletteAddress);


};

#endif