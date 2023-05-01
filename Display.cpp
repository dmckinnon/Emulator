#include "Display.h"

#include <chrono>
#include <iostream>


#define PIXELS_PER_PIXEL 4

Display::Display(
    std::shared_ptr<MMU> memMgmntUnit,
    std::function<void()> setVBlankInterrupt,
    std::function<void()> setLCDStatInterrupt,
    std::function<void(uint8_t)> setJoypadInterrupt) :
    mmu(memMgmntUnit),
    SetVBlankInterrupt(setVBlankInterrupt),
    SetLCDStatInterrupt(setLCDStatInterrupt),
    SetJoypadInterrupt(setJoypadInterrupt),
    scanlineCycleCounter(0)
    //curFrame("/home/dave/Emulator/blank.png")
{
    
    displaying = false;

    // TODO when LCD is disabled set mode to 1

    // TODO how do the tiles get into OAM in the first place? 
    // especilaly systemRom tiles
    // boot rom should copy logo
}

Display::~Display()
{
    if (frameUpdateThread.joinable())
        frameUpdateThread.join();
}

bool Display::IsLCDEnabled()
{
    return mmu->ReadFromAddress(MMU::LCDControlAddress) & LCDEnableBit;
}

void Display::StartDisplay()
{
    #ifdef USE_DISPLAY
    frameUpdateThread = std::thread(FrameThreadProcThunk, this);
    #else
    displaying = true;
    #endif
}

void Display::StopDisplay()
{
    displaying = false;
    if (frameUpdateThread.joinable())
    {
        frameUpdateThread.join();
    }
    
}

void Display::FrameThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->FrameThreadProc();
}

void Display::ClockSignalForScanline()
{
    std::unique_lock lk(clockSignalMutex);
    drawNextScanline = true;
    clockSignalCv.notify_one();
}

void Display::FrameThreadProc()
{
    using namespace std::chrono_literals;
    auto frameBuffer = cv::Mat(cv::Size(GAMEBOY_HEIGHT, GAMEBOY_WIDTH), CV_8UC1, cv::Scalar(0));
    displaying = true;
    while (displaying)
    {
        // use a keypress to break out of here
        char key = (char) cv::waitKey(30);   // explicit cast
        if (key == 27) break; 

        UpdateLCDStatus();

        if (!IsLCDEnabled())
        {
            // if things are not all blank, draw all blank, then just wait until next period
            continue;
        }

        // Wait until we can consider another scanline
        std::unique_lock lk(clockSignalMutex);
        clockSignalCv.wait(lk, [this]{return this->drawNextScanline;});

        // Consider the next scanline
        uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);
        currentScanLine ++;
        mmu->WriteToScanlineCounter_Allowed(currentScanLine);

        // confirm that currentScanline is valid
        if (currentScanLine < 0 || currentScanLine > 160)
        {
            lk.unlock();
            continue;
        }
        
        if (currentScanLine > MaxScanlines)
        {
            // reset scan line counter
            mmu->WriteToAddress(MMU::ScanLineCounterAddress, (uint8_t)0);
        }
        else if (currentScanLine < VisibleScanlines)
        {
            // draw current scan line
            DrawScanLine(frameBuffer, currentScanLine);
        }
        else
        {   // Are we in vblank? If so, set explicit VBLANK interrupt
            SetVBlankInterrupt();
            // during vblank, blit image to screen
            cv::imshow("GameBoy", frameBuffer);
            // sleep for a spell to maintain 60fps
            //std::this_thread::sleep_for(2ms);
        }

        // update image
        // simple: draw each scanline based on whether background or sprites or window are enabled
        // complex: draw that which changes

        drawNextScanline = false;
        lk.unlock();
        

        // one scanline takes 456 clock cycles, and goes status 2 -> 3 -> 0
        // vblank is mode 1
        // when mode changes to 0, 1, 2 then this is LCD interrupt
    }
    displaying = false;
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

void Display::DrawScanLine(cv::Mat& buffer, uint8_t curScanline)
{
    uint8_t lcdControl = mmu->ReadFromAddress(MMU::LCDControlAddress);

    // check if we can write tiles; if so, write
    if (lcdControl & BgEnabledBit)
    {
        RenderTiles(buffer, lcdControl, curScanline);
    }

    // check if we can write sprites; if so, write
    if (lcdControl & SpritesEnabledBit)
    {
        RenderSprites(buffer, lcdControl, curScanline);
    }
}

void Display::RenderTiles(cv::Mat& buffer, uint8_t controlReg, uint8_t curScanline)
{
    // Get X and Y locations for window and BG scroll
    uint8_t scrollX = mmu->ReadFromAddress(MMU::BGScrollXAddress);
    uint8_t scrollY = mmu->ReadFromAddress(MMU::BGScrollYAddress);

    // the minus 7 is just something in the gameboy
    uint8_t windowX = mmu->ReadFromAddress(MMU::WindowXAddress) - 7;
    uint8_t windowY = mmu->ReadFromAddress(MMU::WindowYAddress);

    // Check if window is enabled
    bool drawWindow = false;
    if (controlReg & WindowEnableBit)
    {
        // is current scanline within window y position
        if (windowY <= curScanline)
        {
            drawWindow = true;
        }
    }

    // Which region of memory are we grabbing window from 
    // 0x8000 or 0x8800 holds the tile data 
    // 0x9800 and 0x9C00 hold the tile layout
    // TODO remove magic numbers
    uint16_t windowTileDataAddress;
    bool unsig = true;
    if (controlReg & BgWindowTilesetBit)
    {
        windowTileDataAddress = 0x8000;
    }
    else
    {
        // this region uses signed bytes for tile ids
        windowTileDataAddress = 0x8800;
        unsig = false;
    }

    // Which tile layout to use?
    uint16_t backgroundMemory;
    if (!drawWindow)
    {
        // check BG memory map
        if (controlReg & BgTileMapBit)
        {
            backgroundMemory = 0x9C00;
        }
        else
        {
            backgroundMemory = 0x9800;
        }
    }
    else
    {
        // check window memory map
        if (controlReg & WindowTileMapBit)
        {
            backgroundMemory = 0x9C00;
        }
        else
        {
            backgroundMemory = 0x9800;
        }
    }
    
    // Now that we know where in memory we draw from, we calculate from the y position
    // which tile we have, look this up in the layout, and then retrieve the line data
    // for that tile

    // which of the 32 vertical tiles is the current scanline in?
    // not sure I understand this logic for window
    uint8_t yPos = 0;
    if (!drawWindow)
    {
        // Not drawing window. Check from the top of the background
        // plus the current scanline
        // eg. background starts at 16, scanline 3 = 19
        yPos = scrollY + curScanline;
    }
    else
    {
        // Drawing window. Check from current scanline minus window
        // start point, eg. scanline 3, window starts at 2, so check
        // from y position 1?? Well, this would be tile 1? 
        // This doesn't make sense to me 
        yPos = curScanline - windowY;
    }

    // Which vertical pixel row of the tile is the currentScanline in?
    uint16_t tileRow = (yPos/8)*32;

    // now draw the row 
    for (uint8_t i = 0; i < GAMEBOY_WIDTH; ++i)
    {
        // get the value for background offset
        uint8_t bgX = i + scrollX;

        if (drawWindow)
        {
            if (i >= windowX)
            {
                bgX = i - windowX;
            }
        }

        // find which tile of the 32 across this position falls in
        uint16_t tileCol = bgX/8;
        int16_t tileNum = 0;

        // get tile id
        uint16_t tileAddress = backgroundMemory + tileRow + tileCol;
        if (unsig)
        {
            tileNum = mmu->ReadFromAddress(tileAddress);
        }
        else
        {   
            tileNum = (int16_t)mmu->ReadFromAddress(tileAddress);
        }

        // Now get the tile from memory using id, and knowing whether we are
        // in the signed map or the unsigned map
        uint16_t tileLocation = windowTileDataAddress + (unsig? tileNum*16 : (tileNum+128)*16);

        // Now get the correct vertical line in the tile so we know what to draw.
        // Each line takes up two bytes of memory, to represent the colours
        uint8_t line = (yPos % 8) * 2;
        uint8_t byte1 = mmu->ReadFromAddress(tileLocation + line);
        uint8_t byte2 = mmu->ReadFromAddress(tileLocation + line + 1);    

        // Now separate out the individual colour for this pixel
        // we need the xPos as a relative coordinate from 0-7 inside the tile
        // and this gives us the bit number we need from the two bytes for colour
        uint8_t bitNumber = 7 - (bgX % 8);
        uint8_t bitPosition = 0x01 << bitNumber;
        uint8_t colourNumber = byte2 & bitPosition? 0x2 : 0x0;
        colourNumber |= (byte1 & bitPosition? 0x1 : 0x0);

        // now we have colour number - lookup palette for actual colour
        uint8_t grayscale = GetColour(colourNumber, MMU::BackgroundColourPaletteAddress);

        // set pixel colour
        // actual gameboy has RGB; for windows, just doing grayscale
        buffer.at<uchar>(cv::Point(i, curScanline)) = grayscale;
    }
}

void Display::RenderSprites(cv::Mat& buffer, uint8_t controlReg, uint8_t curScanline)
{
    // Always draw all 40 sprites. If a sprite is unused, then I suppose it will be off camera
    // Is there a way we can know asap whether it's worth drawing the sprite?
    uint16_t spriteAttrTable = MMU::SpriteAttributeTableAddress;
    for (uint8_t i = 0; i < NumSprites; ++i)
    {
        // Get next sprite attribute set
        uint8_t spriteIndex = i*4;
        uint8_t yPos = mmu->ReadFromAddress(spriteAttrTable + spriteIndex) - 16;
        uint8_t xPos = mmu->ReadFromAddress(spriteAttrTable + spriteIndex + 1) - 8;
        uint8_t tileOffset = mmu->ReadFromAddress(spriteAttrTable + spriteIndex + 2);
        uint8_t spriteAttributes = mmu->ReadFromAddress(spriteAttrTable + spriteIndex + 3);

        // Sprites can be 8x8 or 8x16
        int ySize = controlReg & SpriteSizeBit? 16 : 8;

        // we're in the sprite, y-wise
        if ((curScanline >= yPos) && (curScanline < yPos+ySize))
        {
            // get this setup out of the way so we can check xpixel to back out asap
            bool xFlip = spriteAttributes & SpriteXFlipBit;
            bool yFlip = spriteAttributes & SpriteYFlipBit;
        
            // get scan line relative in sprite
            uint8_t lineNum = curScanline - yPos;
            if (yFlip)
            {
                lineNum = ySize - lineNum;
            }

            // Get the colours for the horizontal pixels of the sprite
            lineNum *= 2;
            uint16_t dataAddress = MMU::characterRamOffset + tileOffset*16 + lineNum;
            uint8_t colourLine1 = mmu->ReadFromAddress(dataAddress);
            uint8_t colourLine2 = mmu->ReadFromAddress(dataAddress + 1);

            // right to left is pixel 7 to 0, and this makes it easy to
            // get colour data as we can shift the colourLine vars
            for (int pixel = 0; pixel < 8; ++pixel)
            {
                // first check if this pixel is valid
                int xLoc = xPos + pixel;
                if (xLoc < 0 || xLoc >= MaxCols)
                {
                    continue;
                }

                uint8_t pixelNum = xFlip? 7-pixel : pixel;
                uint8_t colourMask = 0x01 << pixelNum;
                uint8_t colourNumber = colourLine2 & colourMask? 0x2 : 0x0;
                colourNumber |= (colourLine1 & colourMask? 0x1 : 0x0);
                uint16_t paletteAddress = spriteAttributes & SpritePaletteNumberBit? MMU::SpriteColurPaletteAddress1 : MMU::SpriteColurPaletteAddress0;
                uint8_t grayscale = GetColour(colourNumber, paletteAddress);

                // transparent, for sprites
                if (grayscale == WHITE)
                {
                    continue;
                }

                // are we hiding sprite behind background?
                if (spriteAttributes & SpriteBgPriorityBit)
                {
                    if (buffer.at<uchar>(xLoc, curScanline) == WHITE)
                    {
                        buffer.at<uchar>(xLoc, curScanline) = grayscale;
                    }
                }
                else
                {
                    buffer.at<uchar>(xLoc, curScanline) = grayscale;
                }
            }
        }
    }
}

uint8_t Display::GetColour(uint8_t colourNum, uint16_t paletteAddress)
{
    // read palette
    uint8_t palette = mmu->ReadFromAddress(paletteAddress);
    uint8_t paletteMask = 0;
    uint8_t colour = 0;
    uint8_t grayscale = 0;
    switch (colourNum)
    {
        case 0:
        {
            paletteMask = 0x03;
            colour = palette & paletteMask;
            break;
        }
        case 1:
        {
            paletteMask = 0x0C;
            colour = (palette & paletteMask) >> 2;
            break;
        }
        case 2:
        {
            paletteMask = 0x30;
            colour = (palette & paletteMask) >> 4;
            break;
        }
        case 3:
        {
            paletteMask = 0xC0;
            colour = (palette & paletteMask) >> 6;
            break;
        }
    }

    switch (colour)
    {
        // white
        case 0: 
        {
            grayscale = WHITE;
            break;
        }
        // light gray
        case 1:
        {
            grayscale = LIGHT_GRAY;
            break;
        }
        // dark gray
        case 2:
        {
            grayscale = DARK_GRAY;
            break;
        }
        // black
        case 3:
        {
            grayscale = BLACK;
            break;
        }
    }
    
    return grayscale;
}