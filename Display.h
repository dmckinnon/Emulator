
#include <functional>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
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
        std::shared_ptr<MMU> mmu,
        std::function<void()> setVBlankInterrupt,
        std::function<void()> setLCDStatInterrupt,
        std::function<void(uint8_t)> setJoypadInterrupt);
    ~Display();

    bool IsLCDEnabled();

    static void WindowThreadProcThunk(void* context);
    static void FrameThreadProcThunk(void* context);

private:
    // need key press handlers

    // interrupt handlers
    std::function<void()> SetVBlankInterrupt;
    std::function<void()> SetLCDStatInterrupt;
    std::function<void(uint8_t)> SetJoypadInterrupt;

    std::shared_ptr<MMU> mmu;

    // Image to show in window and update at frame rate
    uint8_t frameBuffer[GAMEBOY_HEIGHT][GAMEBOY_WIDTH*3];
    std::mutex imageMutex;

    // LCD status vars
    static const uint8_t HBlankMode = 0x00;
    static const uint8_t VBlankMode = 0x01;
    static const uint8_t SearchingSpriteAttrsMode = 0x02;
    static const uint8_t XferDataToLcdMode = 0x03; 
    static const uint8_t LCDModeMask = 0xFC;

    // Scanline states to set mode
    static const uint8_t VisibleScanlines = 144;
    static const uint8_t MaxScanlines = 153;

    uint16_t scanlineCycleCounter;
    static const uint16_t MaxScanlineCount = 456;
    static const uint16_t CyclesForMode2 = 80;
    static const uint16_t CyclesForMode3 = 172;

    // main display thread
    std::thread windowThread;
    std::thread frameUpdateThread;
    void WindowThreadProc();
    void FrameThreadProc();

    void UpdateLCDStatus();
};