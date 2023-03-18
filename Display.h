
#include <functional>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "MMU.h"

#define GAMEBOY_WIDTH 160
#define GAMEBOY_HEIGHT 144

/*
    This class is essentially the video chip and the display.
    On desktop, it will create a GUI window and have a display
    thread writing to it at 60Hz and setting the VBLANK and LCDSTAT
    interrupts with the CPU.

    Given that Gtk also registers keyboard events, on desktop this
    class will also handle input events. 

    This class IS the display window, and contains any buttons/menus/
    other GUI elements necessary.
*/
class Display //: public Gtk::Window
{
public:
    // Args are lambdas for setting CPU interrupts
    Display(
        std::shared_ptr<MMU> memMgmntUnit,
        std::function<void()> setVBlankInterrupt,
        std::function<void()> setLCDStatInterrupt,
        std::function<void(uint8_t)> setJoypadInterrupt);
    ~Display();

    bool IsLCDEnabled();

    void StartDisplay();

    bool Displaying() 
    {
        return displaying;
    }

    static void FrameThreadProcThunk(void* context);

private:
    // need key press handlers

    // interrupt handlers
    std::function<void()> SetVBlankInterrupt;
    std::function<void()> SetLCDStatInterrupt;
    std::function<void(uint8_t)> SetJoypadInterrupt;

    std::shared_ptr<MMU> mmu;

    // Image to show in window and update at frame rate
    //uint8_t frameBuffer[GAMEBOY_HEIGHT][GAMEBOY_WIDTH];
    // opencv image
    //cv::Mat frameBuffer;
    std::mutex imageMutex;
    bool displaying;

    // LCD status vars
    static const uint8_t HBlankMode = 0x00;
    static const uint8_t VBlankMode = 0x01;
    static const uint8_t SearchingSpriteAttrsMode = 0x02;
    static const uint8_t XferDataToLcdMode = 0x03; 
    static const uint8_t LCDModeMask = 0xFC;
    static const uint8_t CoincidenceBit = 0x04;

    // LCD Stat internal interrupt vars
    static const uint8_t HBlankInternalInterruptBit = 0x08;
    static const uint8_t VBlankInternalInterruptBit = 0x10;
    static const uint8_t SpriteInternalInterruptBit = 0x20;
    static const uint8_t CoincidenceInternalInterruptBit = 0x40;

    // Scanline states to set mode
    static const uint8_t VisibleScanlines = 144;
    static const uint8_t MaxScanlines = 153;
    static const uint8_t MaxCols = 160;

    uint16_t scanlineCycleCounter;
    static const uint16_t MaxScanlineCount = 456;
    static const uint16_t CyclesForMode2 = 80;
    static const uint16_t CyclesForMode3 = 172;

    // Control register bits
    static const uint8_t LCDEnableBit = 0x80;
    static const uint8_t WindowTileMapBit = 0x40; // determines range of tile map
    static const uint8_t WindowEnableBit = 0x20;
    static const uint8_t BgWindowTilesetBit = 0x10; // determines range of tileset
    static const uint8_t BgTileMapBit = 0x08; // determines range of background map
    static const uint8_t SpriteSizeBit = 0x04; // 8x8 or 16x16
    static const uint8_t SpritesEnabledBit = 0x02;
    static const uint8_t BgEnabledBit = 0x01; // disabled background is white/no colour
    static const uint16_t tileSizeInMemory = 16;

    // Sprites
    static const uint8_t NumSprites = 40;
    static const uint8_t SpriteBgPriorityBit = 0x80;
    static const uint8_t SpriteYFlipBit = 0x40;
    static const uint8_t SpriteXFlipBit = 0x20;
    static const uint8_t SpritePaletteNumberBit = 0x10;

    // colours
    static const uint8_t WHITE = 0xFF;
    static const uint8_t LIGHT_GRAY = 0xCC;
    static const uint8_t DARK_GRAY = 0x77;
    static const uint8_t BLACK = 0x00;


    // main display thread
    std::thread windowThread;
    std::thread frameUpdateThread;
    void FrameThreadProc();

    // need a mutex for accessing OAM and VRAM during mode 0 and 1, but not mode 3

    void UpdateLCDStatus();
    void DrawScanLine(cv::Mat& buffer, uint8_t curScanline);
    void RenderTiles(cv::Mat& buffer, uint8_t controlReg, uint8_t curScanline);
    void RenderSprites(cv::Mat& buffer, uint8_t controlReg, uint8_t curScanline);
    uint8_t GetColour(uint8_t colourNum, uint16_t paletteAddress);
};