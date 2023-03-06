#include <gtkmm.h>
#include <gtkmm/window.h>
#include <gtkmm/image.h>
#include <functional>
#include <thread>
#include <mutex>

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
        std::function<void()> setVBlankInterrupt,
        std::function<void()> setLCDStatInterrupt,
        std::function<void(uint8_t)> setJoypadInterrupt);
    ~Display();

    static void WindowThreadProcThunk(void* context);
    static void FrameThreadProcThunk(void* context);

private:
    // need key press handlers

    // interrupt handlers
    std::function<void()> SetVBlankInterrupt;
    std::function<void()> SetLCDStatInterrupt;
    std::function<void(uint8_t)> SetJoypadInterrupt;

    // Image to show in window and update at frame rate
    uint8_t frameBuffer[GAMEBOY_HEIGHT][GAMEBOY_WIDTH*3];
    Gtk::Image curFrame;
    std::mutex imageMutex;

    // main display thread
    std::thread windowThread;
    std::thread frameUpdateThread;
    void WindowThreadProc();
    void FrameThreadProc();
};