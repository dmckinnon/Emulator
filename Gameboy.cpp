#include "Gameboy.h"
#include <string.h>

Gameboy::Gameboy(std::shared_ptr<Rom> rom) :
    rom(rom)
{
    // allocate the MMU
    mmu = std::make_shared<MMU>();

    // Load ROM into memory ready for CPU to execute
    // Is this expected? Would the chip do this? No - the CPU just executes the ROM
    mmu->LoadRomToMemory(*rom);

    cpu = CPU(mmu);
}

Gameboy::~Gameboy()
{
    // destroy the CPU first as it has pointers to memory
    cpu.~CPU();

    // mmu will destroy itself
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


