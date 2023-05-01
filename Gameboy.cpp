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
    cpu.SetDisplaySignalFunc([this](){
        this->display.ClockSignalForScanline();
    });
}

Gameboy::~Gameboy()
{
    // join each of the cpu and display threads and destroy them
    cpu.~CPU();
    display.~Display();

    // mmu will destroy itself
}

bool Gameboy::LoadRom(std::shared_ptr<Rom> rom)
{
    if (rom == nullptr)
    {
        return false;
    }

    mmu->LoadRomToMemory(rom);

    return true;
}

bool Gameboy::Run()
{
    using namespace std::chrono_literals;
    // Create the display thread and CPU thread

    // cpu thread
    auto cpuThread = std::thread([this](){
        this->cpu.ExecuteCode();
    });

    display.StartDisplay();

    // wait for both threads to finish
    while (/*display.Displaying() &&*/ cpu.Executing())
    {
        std::this_thread::sleep_for(10ms);
    }

    if (cpuThread.joinable())
    {
        cpuThread.join();
    }

    display.StopDisplay();

    // display 

    return true;
}


