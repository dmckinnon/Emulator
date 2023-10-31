#include "Debugger.h"
#include "Display.h"

#ifndef RP2040

#define DEBUG_WINDOW_SIZE 640

// TODO don't we have 16 x 8 sprites?
#define SPRITE_WIDTH 8
#define SPRITE_HEIGHT 8

Debugger::Debugger(std::shared_ptr<MMU> mmu)
{
    this->mmu = mmu;
    blockEveryFrame = false;

    cv::Mat img(DEBUG_WINDOW_SIZE, DEBUG_WINDOW_SIZE, CV_8UC3);
    tiles = img;
    //tiles = cv::Scalar(255, 255, 255);
    tileAddress = 0;

    // Do we also want cpu? Probably later

    // Initialise window
    cv::namedWindow("Debugger");
    cv::resizeWindow("Debugger", DEBUG_WINDOW_SIZE, DEBUG_WINDOW_SIZE);

    // Need a way of specifying address
    // use cin prompted by a mouse click or something
}

Debugger::~Debugger()
{

}

void Debugger::ShowTiles(uint16_t address)
{
    // clear previous mat
    tiles = cv::Scalar(255, 255, 255);

    // draw a grid
    // 8 80x80 squares, with lines 2 pixels thick, means each is 79 pixels?
    for (int i = 0; i < DEBUG_WINDOW_SIZE; i += 80)
    {
        cv::line(tiles, cv::Point(i, 0), cv::Point(i, DEBUG_WINDOW_SIZE), cv::Scalar(0, 0, 0), 2); 
        cv::line(tiles, cv::Point(0, i), cv::Point(DEBUG_WINDOW_SIZE, i), cv::Scalar(0, 0, 0), 2); 
    }
    //line(whiteMatrix, starting, ending, line_Color, thickness);//using line() function to draw the line//
   

    // create control register
    uint8_t controlRegister = 0;


    // From starting address, render each pixel of each tile?




}

void Debugger::DebuggerThread()
{
    while (true)
    {
        char key = (char) cv::waitKey(30);   // explicit cast
        if (key == 'a')
        {
            std::cout << "address in ascii: ";
            std::string address = "";
            std::cin >> address;

            // get address as uint16_t
            uint16_t addr = 0;

            ShowTiles(addr);
        } 
        else if (key == 'c')
        {
            // release lock on frame
            moveToNextFrame = true;
        }
    }
}

void Debugger::MaybeBlock()
{
    if (blockEveryFrame)
    {
        std::unique_lock lk(frameMutex);
        debuggerBlockCV.wait(lk, [this]{return this->moveToNextFrame;});
        moveToNextFrame = true;
    }
}


void Debugger::RenderTiles(
    uint8_t controlReg,
    uint16_t curScanline,
    uint16_t xOffset,
    uint8_t scrollX,
    uint8_t scrollY, 
    uint8_t windowX, 
    uint8_t windowY)
{
    // the minus 7 is just something in the gameboy
    //uint8_t windowX = mmu->ReadFromAddress(MMU::WindowXAddress) - 7;

    // Check if window is enabled
    bool drawWindow = false;
    if (controlReg & WindowEnableBit)
    {
        // is current scanline within window y position
        // TODO check this
        //if (windowY <= curScanline)
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
    uint16_t backgroundMemory;
    if (!drawWindow)
    {
        // check BG memory map
        if (controlReg & BgTileMapBit)
        {
            backgroundMemory = UnsignedBgTileLayoutAddress;
        }
        else
        {
            backgroundMemory = SignedBgTileLayoutAddress;
        }
    }
    else
    {
        // TODO are these correct?
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

    for (int yPos = 0; yPos < SPRITE_HEIGHT; yPos ++)
    {
        // Which vertical pixel row of the tile is the currentScanline in?
        uint16_t tileRow = (yPos/8)*32;

        // Draw this row of the sprite
        for (uint8_t i = 0; i < SPRITE_WIDTH; ++i)
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
            tiles.at<uchar>(cv::Point(i + xOffset, curScanline + yPos)) = grayscale;
        }
    }
}


uint8_t Debugger::GetColour(uint8_t colourNum, uint16_t paletteAddress)
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

#endif