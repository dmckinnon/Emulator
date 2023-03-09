#include "Gameboy.h"
#include <string.h>
#include <chrono>
#include <thread>


Gameboy::Gameboy(std::shared_ptr<Rom> systemRom) :
    mmu(std::make_shared<MMU>(*systemRom)),
    cpu(mmu),
    display(
        mmu,
        // Lambda to set VBLANK interrupt in CPU
        [this](){
            this->cpu.SetVBlankInterrupt();
        },
        // Lambda to set LCD STAT interrupt in CPU
        [this](){
            this->cpu.SetLCDStatInterrupt();
        },
        // Lambda to set Joypad interrupt in CPU
        [this](uint8_t joypadRegister){
            this->cpu.SetJoypadInterrupt(joypadRegister);
            // debounce
            //std::this_thread::sleep_for(std::chrono::milliseconds(5));
        })
{
}

Gameboy::~Gameboy()
{
    // join each of the cpu and display threads and destroy them


    // destroy the CPU first as it has pointers to memory
    cpu.~CPU();

    // mmu will destroy itself
}

bool Gameboy::LoadRom(std::shared_ptr<Rom> rom)
{
    // If a rom is already loaded, fail
    if (rom != nullptr)
    {
        return false;
    }

    mmu->LoadRomToMemory(rom);

    return true;
}

bool Gameboy::Run()
{
    // Create the display thread and CPU thread
    //auto cpuThread = std::thread([this](){cpu.ExecuteCode(rom);});

    // cpu thread
    auto cpuThread = std::thread([&](){
        cpu.ExecuteCode();
    });
    
    

    // display thread
    /*auto displayThread = std::thread([&](){

        auto app = Gtk::Application::create("example");

        app->run(display);
    });

    displayThread.join();*/

    cpuThread.join();

    return true;
}


