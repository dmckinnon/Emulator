#include "Display.h"

Display::Display(
    std::function<void()> setVBlankInterrupt,
    std::function<void()> setLCDStatInterrupt,
    std::function<void(uint8_t)> setJoypadInterrupt) :
    SetVBlankInterrupt(setVBlankInterrupt),
    SetLCDStatInterrupt(setLCDStatInterrupt),
    SetJoypadInterrupt(setJoypadInterrupt)
{
    // set border width of window
    //set_border_width(10);

    // Start display thread
    displayThread = std::thread(Display::ThreadProcThunk, this);
}

Display::~Display()
{
}

void Display::ThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->ThreadProc();
}

void Display::ThreadProc()
{
    auto app = Gtk::Application::create("example");

    auto window = Gtk::Window();

    app->run(window);
}