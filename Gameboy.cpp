#include "Gameboy.h"
#include <string.h>
#include <chrono>

#ifndef RP2040
#include <thread>

class ST7789;
#else
#include "pico/multicore.h"
#endif

Gameboy* g = nullptr;

Gameboy::Gameboy(
    std::shared_ptr<Rom> systemRom,
    std::shared_ptr<ST7789> lcd) :
    mmu(std::make_shared<MMU>(*systemRom)),
    cpu(mmu),
    display(
        mmu,
        lcd,
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

#ifdef RP2040
    // So we can use a static arg-less function for core 1
    g = this;
#endif
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

void Gameboy::StartCPU()
{
    cpu.ExecuteCode();
}

static void StartCPUExecuting()
{
    g->StartCPU();
}

bool Gameboy::Run()
{
    using namespace std::chrono_literals;
    // Create the display thread and CPU thread

    // cpu thread
#ifndef RP2040
    auto cpuThread = std::thread([this](){
        this->cpu.ExecuteCode();
    });
#else
    multicore_reset_core1();
    // Start the CPU thread on core 1
    multicore_launch_core1(StartCPUExecuting);
#endif

    // On embedded, this becomes the display thread (core 0)
    // as it never needs to exit
    display.StartDisplay();

#ifndef RP2040
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
#endif

    return true;
}


