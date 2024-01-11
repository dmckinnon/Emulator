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
    moveToNextFrame = false;

    cv::Mat img(32, 300, CV_8UC1);
    tiles = img;
    tileAddress = 0;

    // Do we also want cpu? Probably later

    // Need a way of specifying address
    // use cin prompted by a mouse click or something

    debuggerThread = std::thread([this](){
        this->DebuggerThread();
    });
}

Debugger::~Debugger()
{
    runDebugger = false;
    debuggerThread.join();
}

void Debugger::ShowTiles(uint16_t address)
{
    // clear previous mat
    tiles = cv::Scalar(255, 255, 255);

    RenderTiles(0x8000, 0);
    RenderTiles(0x8800, 16);
    //RenderSprites(mmu->ReadFromAddress(MMU::LCDControlAddress), address, 0, 0, 32);
}

void Debugger::DebuggerThread()
{
    while (runDebugger)
    {
        cv::imshow("Debugger", tiles);
        //ShowTiles(0);
        char key = (char) cv::waitKey(30);   // explicit cast
        if (key == 's')
        {
            uint16_t addr = 0;//(uint16_t)temp;

            ShowTiles(0);
        } 
        else if (key == 'n')
        {
            // release lock on frame
            moveToNextFrame = true;
        }
        else if (key == 'c')
        {
            blockEveryFrame =false;
        }
        else if (key == 'b')
        {
            blockEveryFrame = true;
            moveToNextFrame = false;
        }
        
    }
}

void Debugger::MaybeBlock()
{
    if (blockEveryFrame)
    {
        std::unique_lock lk(frameMutex);
        debuggerBlockCV.wait(lk, [this]{return this->moveToNextFrame;});
        moveToNextFrame = false;
    }
}


void Debugger::RenderSprites(
    uint8_t controlReg,
    uint16_t address,
    uint16_t tileNumber,
    uint16_t xOffset,
    uint16_t yOffset)
{
    uint16_t spriteAttrTable = MMU::SpriteAttributeTableAddress;
    for (uint8_t i = 0; i < NumSprites; ++i)
    {
        // Get next sprite attribute set
        uint8_t spriteIndex = i*4;
        uint8_t tileOffset = mmu->ReadFromAddress(spriteAttrTable + spriteIndex + 2);

        // Sprites can be 8x8 or 8x16
        // might have to manually set this
        int ySize = controlReg & SpriteSizeBit? 16 : 8;

        // scan through all the pixels in the sprite
        for (int yPos = 0; yPos < ySize; yPos ++)
        {
            // Draw this row of the sprite
            uint16_t lineNum = yPos*2;
            uint16_t dataAddress = MMU::characterRamOffset + tileOffset*16 + lineNum;
            uint8_t colourLine1 = mmu->ReadFromAddress(dataAddress);
            uint8_t colourLine2 = mmu->ReadFromAddress(dataAddress + 1);  
            for (uint8_t x = 0; x < SPRITE_WIDTH; ++x)
            {
                uint8_t pixelNum = x;
                uint8_t colourMask = 0x01 << pixelNum;
                uint8_t colourNumber = colourLine2 & colourMask? 0x2 : 0x0;
                colourNumber |= (colourLine1 & colourMask? 0x1 : 0x0);
                uint16_t spriteAttributes = 0;
                uint16_t paletteAddress = spriteAttributes & SpritePaletteNumberBit? MMU::SpriteColurPaletteAddress1 : MMU::SpriteColurPaletteAddress0;
                uint8_t grayscale = GetColour(colourNumber, paletteAddress);
                // now we have colour number - lookup palette for actual colour
                //uint8_t grayscale = GetColour(colourNumber, MMU::BackgroundColourPaletteAddress);

                // set pixel colour
                // actual gameboy has RGB; for windows, just doing grayscale
                tiles.at<uchar>(cv::Point(i + xOffset, yPos + yOffset)) = grayscale;
            }
        }
    }
}


void Debugger::RenderTiles(uint16_t address, int height)
{
    //uint16_t address = 0x8000;
    // render tiles at 8010 for now
    // There's 24 tiles
    // tiles are 8x8 pixels, but each pixel is 2 bytes so 16 bits
    int yOffset = 0;
    int xOffset = 0;
    for (int i = 0; i < 32; ++i)
    {
        auto tileAddress = address + 16*i;
        for (int y = 0; y < 8; ++y)
        {
            int yPos = y + yOffset; 
            uint8_t byte1 = mmu->ReadFromAddress(tileAddress + y*2);
            uint8_t byte2 = mmu->ReadFromAddress(tileAddress + y*2 + 1);
            for (int x = 0; x < 8; ++x)
            {
                int xPos = x + xOffset;
                
                uint8_t bitNumber = 7 - x;
                uint8_t bitPosition = 0x01 << bitNumber;
                uint8_t colourNumber = byte2 & bitPosition? 0x2 : 0x0;
                colourNumber |= (byte1 & bitPosition? 0x1 : 0x0);
                uint8_t grayscale = GetColour(colourNumber, MMU::BackgroundColourPaletteAddress);
                tiles.at<uchar>(cv::Point(xPos, y + height)) = grayscale;
            }
        }
        xOffset += 8;
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