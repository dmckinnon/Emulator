#include "Gameboy.h"
#include <string.h>

Gameboy::Gameboy(std::shared_ptr<Rom> rom) :
    rom(rom)
{
    // allocate the sram and vram and clear the
    sram = new byte[GAMEBOY_SRAM];
    vram = new byte[GAMEBOY_VRAM];

    memset(sram, 0, sizeof(GAMEBOY_SRAM));
    memset(vram, 0, sizeof(GAMEBOY_VRAM));

    cpu = CPU(sram, vram);

    rom = nullptr;
}

Gameboy::~Gameboy()
{
    // destroy the CPU first as it has pointers to memory
    cpu.~CPU();

    // clean up
    delete sram;
    delete vram;
}

bool Gameboy::Run()
{
    // If a rom is not loaded, fail
    if (rom == nullptr || rom->bytes == nullptr || rom->size == 0)
    {
        return false;
    }

    // Create the display thread and CPU thread
    //auto cpuThread = std::thread([this](){cpu.ExecuteCode(rom);});

    cpu.ExecuteCode(rom);

    //cpuThread.join();

    return true;
}


