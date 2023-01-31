#include "Gameboy.h"
#include <string.h>

Gameboy::Gameboy(std::shared_ptr<Rom> systemRom)
{
    // allocate the MMU
    mmu = std::make_shared<MMU>(*systemRom);

    cpu = CPU(mmu);
}

Gameboy::~Gameboy()
{
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

    cpu.ExecuteCode();

    //cpuThread.join();

    return true;
}


