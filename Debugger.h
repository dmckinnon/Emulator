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
    bool runDebugger = true;

    uint16_t tileAddress;

    std::mutex frameMutex;
    std::condition_variable debuggerBlockCV;

    std::shared_ptr<MMU> mmu;

    cv::Mat tiles;

    // largely copies of what is in Display
    void RenderTiles(uint16_t address, int height);
    void RenderSprites(
        uint8_t controlReg,
        uint16_t address,
        uint16_t tileNumber,
        uint16_t xOffset,
        uint16_t yOffset);
    uint8_t GetColour(uint8_t colourNum, uint16_t paletteAddress);


};

#endif