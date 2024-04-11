#include "Display.h"

#include <chrono>
#include <iostream>


#define PIXELS_PER_PIXEL 4

Display::Display(
    MMU* memMgmntUnit,
    std::shared_ptr<ST7789> lcd,
    std::function<void()> setVBlankInterrupt,
    std::function<void()> setLCDStatInterrupt,
    std::function<void(uint8_t)> setJoypadInterrupt,
    bool useDebugger) :
    mmu(memMgmntUnit),
    lcd(lcd),
    SetVBlankInterrupt(setVBlankInterrupt),
    SetLCDStatInterrupt(setLCDStatInterrupt),
    SetJoypadInterrupt(setJoypadInterrupt)
#ifdef RP2040
    ,canvas(GAMEBOY_WIDTH, GAMEBOY_HEIGHT)
#endif
{
    displaying = false;
    scanlineCycleCounter = 0;
    m_displayClock = 0;

#ifdef RP2040
    mutex_init(&clockSignalMutex);
    lcd->SetDisplayArea(GAMEBOY_WIDTH, GAMEBOY_HEIGHT);
    lcd->ClearScreen(true);
#else
    frameBuffer = cv::Mat(cv::Size(GAMEBOY_HEIGHT, GAMEBOY_WIDTH), CV_8UC1, cv::Scalar(0));
    if (useDebugger)
        debugger = std::make_shared<Debugger>(mmu);
#endif
}

Display::~Display()
{
    // On embedded, display does not contain more than one thread
    // and destructor is never called. Just switch the device off
#ifndef RP2040
    if (frameUpdateThread.joinable())
        frameUpdateThread.join();
#endif
}

bool Display::IsLCDEnabled()
{
    return mmu->ReadFromAddress(MMU::LCDControlAddress) & LCDEnableBit;
}

void Display::StartDisplay()
{
#ifndef RP2040
    #ifdef USE_DISPLAY
    frameUpdateThread = std::thread(FrameThreadProcThunk, this);
    #else
    displaying = true;
    #endif
#else
    // on embedded, main thread becomes display thread
    // Start running display, and ... realistically, we don't stop
    FrameThreadProc();
#endif
}

void Display::StopDisplay()
{
    // See dtor. Same logic applies here
#ifndef RP2040
    displaying = false;
    if (frameUpdateThread.joinable())
    {
        frameUpdateThread.join();
    }
#endif
}

void Display::FrameThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->FrameThreadProc();
}

void Display::ClockSignalForScanline()
{
#ifndef RP2040
    std::unique_lock lk(clockSignalMutex);
    drawNextScanline = true;
    clockSignalCv.notify_one();
#else
    mutex_enter_blocking(&clockSignalMutex);

    drawNextScanline = true;
    // main thread is spinning on checking this so there
    // is no need to notify

    mutex_exit(&clockSignalMutex);
#endif
}

void Display::MaybeDrawOneScanLine(int cycles)
{
    //UpdateLCDStatus();

    if (!IsLCDEnabled())
    {
        // if things are not all blank, draw all blank, then just wait until next period
        return;
    }

    // Every 456 cycles we signal display so it can draw another scanline
    // This is independent of other timers
    m_displayClock += 4*cycles;


    // TODO add the timing/mode switching here???
    // This comes from https://imrannazar.com/series/gameboy-emulation-in-javascript/gpu-timing
    // Codeslinger apparently forgot this?

    uint8_t status = mmu->ReadFromAddress(MMU::LCDStatusAddress);
    uint8_t mode = status & LCDModeMask;
    uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);
    uint8_t requestInterrupt = 0;

    switch (mode)
    {
        case SearchingSpriteAttrsMode: // also called OAM mode
        {
            // OAM read mode, scanline active
            // TODO what does it mean that scanline is active here? Read GPU timings
            if (m_displayClock >= 80)
            {
                // switch to mode 3
                m_displayClock = m_displayClock % 80;
                status &= ~LCDModeMask;
                status |= XferDataToLcdMode;
            }
            break;
        }
        case XferDataToLcdMode: // also called VRAM read mode
        {
            // VRAM read mode, scanline active. Treat end of mode 3 as end of scanline
            if (m_displayClock >= 172)
            {
                m_displayClock = m_displayClock % 172;
                // switch to mode 0
                status &= ~LCDModeMask;
                status |= HBlankMode;
                requestInterrupt = status & HBlankInternalInterruptBit;

                // they have here renderscan, which I assume is for the currrent line but I will check
                // yeah looks like this is what they meant
                // Draw the current line
                //DrawScanLine(currentScanLine);

                // Do some more stuff here?
            }
            break;
        }
        case HBlankMode:
        {
            // After last HBlank, push framebuffer to window (?)
            if (m_displayClock >= 204)
            {
                m_displayClock = m_displayClock % 204;

                // line ++
                // increment scanline?
                currentScanLine ++;
                mmu->WriteToScanlineCounter_Allowed(currentScanLine);

                DrawScanLine(currentScanLine);

                if (currentScanLine == 144) // since currentScanLine is 0 indexed
                {
                    // enter vblank mode
                    status &= ~LCDModeMask;
                    status |= VBlankMode;

                    requestInterrupt = status & VBlankInternalInterruptBit;
                    SetVBlankInterrupt();

                    // All lines have finished; write data to screen
                    // TODO: check this timing works
                    // On embedded, signal other thread. Wait here if other thread has not finished
                    //UpdateDisplay();
                }
                else
                {
                    // Enter OAM mode
                    status &= ~LCDModeMask;
                    status |= SearchingSpriteAttrsMode;
                    requestInterrupt = status & SpriteInternalInterruptBit;
                }
            }
            break;
        }
        case VBlankMode:
        {
            if (m_displayClock >= 456)
            {
                m_displayClock = 0;

                // line ++
                currentScanLine ++;
                mmu->WriteToScanlineCounter_Allowed(currentScanLine);

                if (currentScanLine >= MaxScanlines)
                {
                    // restart scanning modes
                    currentScanLine = 0;
                    mmu->WriteToScanlineCounter_Allowed(currentScanLine);

                    // Enter OAM mode
                    status &= ~LCDModeMask;
                    status |= SearchingSpriteAttrsMode;
                    requestInterrupt = status & SpriteInternalInterruptBit;

                    UpdateDisplay();
                }
            }
            break;
        }
    }

    uint8_t newMode = status & LCDModeMask;

    // if we are not in mode 3, but mode has changed,
    // request LCD stat interrupt if mode change interrupt bit is set
    //uint8_t newMode = currentStatus & LCDModeMask;
    if (requestInterrupt && newMode != mode)
    {
        SetLCDStatInterrupt();
    }

    // Check coincidence flag - is this the scanline we're looking for?
    uint8_t desiredScanline = mmu->ReadFromAddress(MMU::DesiredScanlineRegisterAddress);
    if (currentScanLine == desiredScanline)
    {
        // set the coincidence bit in status register
        // Fire interrupt if coincidence interrupt bit is set
        status |= CoincidenceBit;
        if (status & CoincidenceInternalInterruptBit)
        {
            SetLCDStatInterrupt();
        }
    }
    else
    {
        // clear coincidence bit
        status &= ~CoincidenceBit;
    }

    // write out new status
    mmu->WriteToAddress(MMU::LCDStatusAddress, status);

    return;


    ///////////////////////////////////////////////

    // ignore all below this for now

    if (m_displayClock >= 456)
    {
        m_displayClock = 0;
    }
    else
    {
        // not ready yet to check the scanline
        return;
    }


    // Consider the next scanline
   //uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);
    currentScanLine ++;
    mmu->WriteToScanlineCounter_Allowed(currentScanLine);

    // confirm that currentScanline is valid
    if (currentScanLine < 0 || currentScanLine > 160)
    {
        // need to throw an error 
        return;
    }

    if (currentScanLine > MaxScanlines)
    {
        // reset scan line counter
        mmu->WriteToAddress(MMU::ScanLineCounterAddress, (uint8_t)0);
    }
    else if (currentScanLine < VisibleScanlines)
    {

        // since we're drawing to the frame, take a lock on the frame buffer
        //displayMutex.lock();

        // draw current scan line
        DrawScanLine(currentScanLine);

        //displayMutex.unlock();
    }
    else if (currentScanLine == VisibleScanlines)
    {   // Are we in vblank? If so, set explicit VBLANK interrupt
        SetVBlankInterrupt();
        // during vblank, blit image to screen
        
        
    }

    drawNextScanline = false;
}

void Display::UpdateDisplay()
{
    uint8_t scrollX = mmu->ReadFromAddress(MMU::BGScrollXAddress);
    uint8_t scrollY = mmu->ReadFromAddress(MMU::BGScrollYAddress);

    // the minus 7 is just something in the gameboy
    uint8_t windowX = mmu->ReadFromAddress(MMU::WindowXAddress) - 7;
    uint8_t windowY = mmu->ReadFromAddress(MMU::WindowYAddress);

    //printf("Background scroll y: %x\nWindow scroll y: %x\n\n", scrollY, windowY);

#ifndef RP2040
    cv::imshow("GameBoy", frameBuffer);
    cv::waitKey(16); 
#else
    // TODOOOO
    // check with display writing thread 
    // if it can draw, do it
#endif
}

void Display::FrameThreadProc()
{  
    displaying = true;
    while (displaying)
    {
#ifndef RP2040
        // In embedded we don't leave
        // use a keypress to break out of here
        //char key = (char) cv::waitKey(1);   // explicit cast
        //if (key == 27) break; 

        //displayMutex.lock();
        //cv::imshow("GameBoy", frameBuffer);
        //displayMutex.unlock();
#endif
    }

    displaying = false;
}
/*

        UpdateLCDStatus();

        if (!IsLCDEnabled())
        {
            // if things are not all blank, draw all blank, then just wait until next period
            continue;
        }

        // Wait until we can consider another scanline
#ifdef RP2040
        // lock mechanism for rp2040
        bool canProceed = false;
        while (!canProceed)
        {
            // This is how we wait on a condition var in rp2040
            // could also use a thread queue
            mutex_enter_blocking(&clockSignalMutex);
            canProceed = drawNextScanline;
            mutex_exit(&clockSignalMutex);
        }
        // Take the lock again
        mutex_enter_blocking(&clockSignalMutex);
#else
        std::unique_lock lk(clockSignalMutex);
        clockSignalCv.wait(lk, [this]{return this->drawNextScanline;});

        // if we have the debugger attached, block on it waiting
        if (debugger)
        {
            debugger->MaybeBlock();
        }
#endif

        // Consider the next scanline
        uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);
        currentScanLine ++;
        mmu->WriteToScanlineCounter_Allowed(currentScanLine);

        // confirm that currentScanline is valid
        if (currentScanLine < 0 || currentScanLine > 160)
        {
#ifdef RP2040
            // unlock so that CPU thread can tke mutex
            mutex_exit(&clockSignalMutex);
#else
            lk.unlock();
#endif
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
            DrawScanLine(currentScanLine);
        }
        else
        {   // Are we in vblank? If so, set explicit VBLANK interrupt
            SetVBlankInterrupt();
            // during vblank, blit image to screen
#ifdef RP2040
            lcd->WriteBuffer(canvas.getBuffer());
#else
            cv::imshow("GameBoy", frameBuffer);
#endif
        }

        drawNextScanline = false;
#ifdef RP2040
        mutex_exit(&clockSignalMutex);
#else
        lk.unlock();
#endif
        
        // one scanline takes 456 clock cycles, and goes status 2 -> 3 -> 0
        // vblank is mode 1
        // when mode changes to 0, 1, 2 then this is LCD interrupt
    }


    displaying = false;
}
*/

void Display::UpdateLCDStatus()
{
    uint8_t currentStatus = mmu->ReadFromAddress(MMU::LCDStatusAddress);

    if (!IsLCDEnabled())
    {
        // set mode to 1
        currentStatus &= ~LCDModeMask;
        currentStatus |= VBlankMode;

        // write out the status
        mmu->WriteToAddress(MMU::LCDStatusAddress, currentStatus);

        // reset scanline counter. Writing to this address zeroes the value regardless of input
        mmu->WriteToAddress(MMU::ScanLineCounterAddress, (uint8_t)0x00);
        m_displayClock = 0;

        return;
    }

    uint8_t oldMode = currentStatus & LCDModeMask;
    uint8_t currentScanLine = mmu->ReadFromAddress(MMU::ScanLineCounterAddress);

    // Check which mode we should be in
    uint8_t requestInterrupt = 0;
    uint8_t newMode = 0;
    if (currentScanLine >= VisibleScanlines)
    {
        // Mode 1 VBlank
        newMode = 1;
        currentStatus &= ~LCDModeMask;
        currentStatus |= VBlankMode;
        requestInterrupt = currentStatus & VBlankInternalInterruptBit;
    }
    else
    {
        if (m_displayClock < CyclesForMode2)
        {
            // Mode 2 Searching for Sprite Attributes
            newMode = 2;
            currentStatus &= ~LCDModeMask;
            currentStatus |= SearchingSpriteAttrsMode;
            requestInterrupt = currentStatus & SpriteInternalInterruptBit;
        }
        else if (m_displayClock < CyclesForMode2 + CyclesForMode3)
        {
            // Mode 3 Transfer Data to LCD
            newMode = 3;
            currentStatus &= ~LCDModeMask;
            currentStatus |= XferDataToLcdMode;
        }
        else
        {
            // Mode 0 HBlank
            newMode = 0;
            currentStatus &= ~LCDModeMask;
            currentStatus |= HBlankMode;
            requestInterrupt = currentStatus & HBlankInternalInterruptBit;
        }
    }

    // if we are not in mode 3, but mode has changed,
    // request LCD stat interrupt if mode change interrupt bit is set
    //uint8_t newMode = currentStatus & LCDModeMask;
    if (requestInterrupt && newMode != oldMode)
    {
        SetLCDStatInterrupt();
    }

    // Check coincidence flag - is this the scanline we're looking for?
    uint8_t desiredScanline = mmu->ReadFromAddress(MMU::DesiredScanlineRegisterAddress);
    if (currentScanLine == desiredScanline)
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

void Display::DrawScanLine(uint8_t curScanline)
{
    uint8_t lcdControl = mmu->ReadFromAddress(MMU::LCDControlAddress); 

    // Write background
    if (lcdControl & BgEnabledBit)
    {
        // Which tile layout to use?
        
        RenderTiles(lcdControl, curScanline);
    }
    else
    {
        // background is not enabled for this scanline. Draw white.
        for (int i = 0; i < GAMEBOY_WIDTH; ++i)
        {
#ifndef RP2040
            frameBuffer.at<uchar>(cv::Point(i, curScanline)) = WHITE;
#else
            uint16_t newColour = GrayscaleToR5G6B5(WHITE);
            canvas.drawPixel(i, curScanline, newColour);
#endif
        } 
    }

    // write the window
    if (lcdControl & WindowEnableBit)
    {
        RenderWindow(lcdControl, curScanline);
    }

    

    // check if we can write sprites; if so, write
    if (lcdControl & SpritesEnabledBit)
    {
        RenderSprites(lcdControl, curScanline);
    }
}

void Display::RenderTiles(uint8_t controlReg, uint8_t curScanline)
{
    // Get X and Y locations for window and BG scroll
    uint8_t scrollX = mmu->ReadFromAddress(MMU::BGScrollXAddress);
    // Still something weird with scroll Y
    uint8_t scrollY = 32;//mmu->ReadFromAddress(MMU::BGScrollYAddress);

    // Which region of memory are we grabbing window from 
    // 0x8000 or 0x8800 holds the tile data 
    // 0x9800 and 0x9C00 hold the tile layout

    uint16_t backgroundMemory;
    // check BG memory map
    if (controlReg & BgTileMapBit)
    {
        backgroundMemory = UnsignedBgTileLayoutAddress;
    }
    else
    {
        backgroundMemory = SignedBgTileLayoutAddress;
    }

    // need to figure out if this bit is correct or not
    uint16_t windowTileDataAddress;
    bool unsig = true;
    if (controlReg & BgWindowTileSetBit)
    {
        windowTileDataAddress = UnsignedBgTileDataAddress;
    }
    else
    {
        // this region uses signed bytes for tile ids
        windowTileDataAddress = SignedBgTileDataAddress;
        unsig = false;
    }

    // Which tile layout to use?

    
    // Now that we know where in memory we draw from, we calculate from the y position
    // which tile we have, look this up in the layout, and then retrieve the line data
    // for that tile

    // which of the 32 vertical tiles is the current scanline in?
    // not sure I understand this logic for window
    uint8_t yPos = scrollY + curScanline;

    // Which vertical pixel row of the tile is the currentScanline in?
    uint16_t tileRow = (uint16_t)(yPos/8)*32; // >> 3 << 5

    // now draw the row 
    for (uint8_t i = 0; i < GAMEBOY_WIDTH; ++i)
    {
        // get the value for background offset
        // This is &ed with 7 as well?? where does this come from?
        uint8_t bgX = i + scrollX;


        // find which tile of the 32 across this position falls in
        uint16_t tileCol = bgX/8; // this is cinoop's lineoffset
        int32_t tileNum = 0;

        // get tile id
        // cinoop's mapoffset + lineoffset
        // thing is, they build an index of the tiles separate to memory, whereas I'm trying to recreate things exactly
        // they also have no consideration for the window? It is not handled????
        // but this undoubtedly works ... 
        uint16_t tileAddress = backgroundMemory + tileRow + tileCol;
        if (unsig)
        {
            tileNum = (uint16_t)mmu->ReadFromAddress(tileAddress);
        }
        else
        {   
            tileNum = (int16_t)mmu->ReadFromAddress(tileAddress);
        }

        // Now get the tile from memory using id, and knowing whether we are
        // in the signed map or the unsigned map
        // POTENTIAL BUG HERE in calculationg things
        // ALSO do we have tile attributes? Do tiles have attributes?
        if (unsig && tileNum < 128)
        {
            tileNum += 256;
        }
        uint16_t tileLocation = windowTileDataAddress + tileNum*16;//(unsig? tileNum*16 : (tileNum+128)*16);

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
#ifdef RP2040
        uint16_t newColour = GrayscaleToR5G6B5(grayscale);
        canvas.drawPixel(i, curScanline, newColour);
#else
        frameBuffer.at<uchar>(cv::Point(i, curScanline)) = grayscale;
#endif
    }
}

void Display::RenderWindow(uint8_t controlReg, uint8_t curScanline)
{
    // Get X and Y locations for window and BG scroll
    uint8_t scrollX = mmu->ReadFromAddress(MMU::BGScrollXAddress);
    // Still something weird with scroll Y
    uint8_t scrollY = 32;//mmu->ReadFromAddress(MMU::BGScrollYAddress);


    uint8_t windowX = mmu->ReadFromAddress(MMU::WindowXAddress) - 7;
    uint8_t windowY = mmu->ReadFromAddress(MMU::WindowYAddress);

    if (curScanline > windowY || windowX > GAMEBOY_WIDTH || windowY > GAMEBOY_HEIGHT)
    {
        return;
    }

    // DO I need an internal window line counter??

    // Which region of memory are we grabbing window from 
    // 0x8000 or 0x8800 holds the tile data 
    // 0x9800 and 0x9C00 hold the tile layout

    uint16_t windowMemory;
    // check window memory map
    if (controlReg & WindowTileMapBit)
    {
        windowMemory = 0x9C00;
    }
    else
    {
        windowMemory = 0x9800;
    }

    // need to figure out if this bit is correct or not
    uint16_t windowTileDataAddress;
    bool unsig = true;
    if (controlReg & BgWindowTileSetBit)
    {
        windowTileDataAddress = UnsignedBgTileDataAddress;
    }
    else
    {
        // this region uses signed bytes for tile ids
        windowTileDataAddress = SignedBgTileDataAddress;
        unsig = false;
    }

    // Which tile layout to use?

    
    // Now that we know where in memory we draw from, we calculate from the y position
    // which tile we have, look this up in the layout, and then retrieve the line data
    // for that tile

    // which of the 32 vertical tiles is the current scanline in?
    // not sure I understand this logic for window
    uint8_t yPos = curScanline - windowY;

    // Which vertical pixel row of the tile is the currentScanline in?
    uint16_t tileRow = (uint16_t)(yPos/8)*32; // >> 3 << 5

    // now draw the row 
    for (uint8_t i = 0; i < GAMEBOY_WIDTH; ++i)
    {
        // get the value for background offset
        // This is &ed with 7 as well?? where does this come from?
        uint8_t bgX = i + windowX; // trying + not -
        //if (i >= windowX)
        ///{
        //    bgX = i - windowX;
        //}


        // find which tile of the 32 across this position falls in
        uint16_t tileCol = bgX/8; // this is cinoop's lineoffset
        int32_t tileNum = 0;

        // get tile id
        // cinoop's mapoffset + lineoffset
        // thing is, they build an index of the tiles separate to memory, whereas I'm trying to recreate things exactly
        // they also have no consideration for the window? It is not handled????
        // but this undoubtedly works ... 
        uint16_t tileAddress = windowMemory + tileRow + tileCol;
        if (unsig)
        {
            tileNum = (uint16_t)mmu->ReadFromAddress(tileAddress);
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
#ifdef RP2040
        uint16_t newColour = GrayscaleToR5G6B5(grayscale);
        canvas.drawPixel(i, curScanline, newColour);
#else
        frameBuffer.at<uchar>(cv::Point(i, curScanline)) = grayscale;
#endif
    }
}

void Display::RenderSprites(uint8_t controlReg, uint8_t curScanline)
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
                uint16_t r5g6b5 = GrayscaleToR5G6B5(grayscale);

                // transparent, for sprites
                if (grayscale == WHITE)
                {
                    continue;
                }

                // are we hiding sprite behind background?
                if (spriteAttributes & SpriteBgPriorityBit)
                {
#ifdef RP2040
                    // white is 0
                    if (canvas.getPixel(xLoc, curScanline) == 0x0000)
                    {
                        canvas.drawPixel(xLoc, curScanline, r5g6b5);
                    }
        
                } else if (canvas.getPixel(i, curScanline) == GrayscaleToR5G6B5(WHITE))
#else
                    frameBuffer.at<uchar>(xLoc, curScanline) = grayscale;
                } else if (frameBuffer.at<uchar>(xLoc, curScanline) == WHITE)
#endif
                
                {
#ifdef RP2040
                    canvas.drawPixel(xLoc, curScanline, r5g6b5);
#else
                    frameBuffer.at<uchar>(xLoc, curScanline) = grayscale;
#endif
                }
                else
                {
                    // do nothing
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

uint16_t Display::GrayscaleToR5G6B5(uint8_t colour)
{
    // We invert the colours here, so black is 0xFFFF
    // and white is 0x0000. This is because it is a black-on-white
    // LCD, not a white with black pixels like the gameboy
    uint16_t newColour = 0x0000;
    switch (colour)
    {
        case BLACK://WHITE:
        {
            newColour = 0x0000;
            break;
        }
        case DARK_GRAY://LIGHT_GRAY:
        {
            newColour = 0x1833;
            break;
        }
        case LIGHT_GRAY://DARK_GRAY:
        {
            newColour = 0x38F7;
            break;
        }
        case WHITE://BLACK:
        {
            newColour = 0xFFFF;
            break;
        }
    }

    return newColour;
}