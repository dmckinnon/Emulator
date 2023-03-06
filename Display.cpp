#include "Display.h"

#define PIXELS_PER_PIXEL 4

Display::Display(
    std::function<void()> setVBlankInterrupt,
    std::function<void()> setLCDStatInterrupt,
    std::function<void(uint8_t)> setJoypadInterrupt) :
    SetVBlankInterrupt(setVBlankInterrupt),
    SetLCDStatInterrupt(setLCDStatInterrupt),
    SetJoypadInterrupt(setJoypadInterrupt)
{
    // make image blank
    for (int i = 0; i < GAMEBOY_HEIGHT; ++i)
    {
        memset(frameBuffer[i], 0,GAMEBOY_WIDTH*3);
    }

    // Start display thread
    windowThread = std::thread(Display::WindowThreadProcThunk, this);
}

Display::~Display()
{
}

void Display::WindowThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->WindowThreadProc();
}

void Display::FrameThreadProcThunk(void* context)
{
    Display* d = (Display*)context;
    d->FrameThreadProc();
}

void Display::WindowThreadProc()
{
    auto app = Gtk::Application::create("example");

    auto window = Gtk::Window();
    // cannot resize window
    window.set_resizable(false);

    // set window size to 160x144 x 4 = 640x576
    window.set_default_size(GAMEBOY_WIDTH*PIXELS_PER_PIXEL, GAMEBOY_HEIGHT*PIXELS_PER_PIXEL);

    {
        // get image mutex and set image
        std::lock_guard<std::mutex> lk(imageMutex);
        const Gdk::Pixbuf::SlotDestroyData d;
        auto img = Gdk::Pixbuf::create_from_data(
            (const unsigned char *)frameBuffer,
            Gdk::COLORSPACE_RGB,
            false,
            8,
            GAMEBOY_WIDTH,
            GAMEBOY_HEIGHT,
            (int) 3,
            d); // what should stride be?
        curFrame.set(img);
    }

    window.add(curFrame);

    app->run(window); 
}

void Display::FrameThreadProc()
{
    while (true)
    {
        // update image

        // flash to curFrame object


        // sleep for 1/60th of a second
    }
}