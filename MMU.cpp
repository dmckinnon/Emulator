#include "MMU.h"
#include <string.h>

MMU::MMU(Rom& systemRom)
{
    // memory is statically allocated
    // clear all memory to 0 and then set up any specific registers
    memset(memory, 0, sizeof(memory));

    // Start with system ROM mapped and load default system ROM to it
    useSystemRom = true;
    memcpy(&this->systemRom, systemRom.bytes, systemRom.size);

    // set up registers

    // Start with both video and the joypad interrupts enabled
    memory[interruptEnableRegisterAddress] = 0x13;

    // Interrupt flags - top 3 bits are always 1
    memory[interruptFlagRegisterAddress] = 0xE0;

}

MMU::~MMU()
{
}

void MMU::LoadRomToMemory(std::shared_ptr<Rom> rom)
{
    currentRom = rom;

    // copy the rom into memory
    // This fills up Rom bank 0 and the first switchable bank
    // If we need more banks - that is, when the program counter tries to
    // get higher - we swap out the switchable bank
    // TODO read into this
    if (rom->size < cartridgeRomBank0Size)
    {
        memcpy(&memory[cartridgeRomBank0Offset], rom->bytes, rom->size);
    }
    else
    {
        memcpy(&memory[cartridgeRomBank0Offset], rom->bytes, cartridgeRomBank0Size);
        memcpy(&memory[cartridgeRomBankSwitchableOffset], &rom->bytes[cartridgeRomBank0Size], rom->size - cartridgeRomBank0Size);
    }
}

void MMU::UnmapSystemRom()
{
    useSystemRom = false;
}

void MMU::SwapOutRomBank(int bank)
{
    // Swap out the switchable bank with the bank specified
    memcpy(&memory[cartridgeRomBankSwitchableOffset], &currentRom->bytes[cartridgeRomBank0Size + (bank * cartridgeRomBankSwitchableSize)], cartridgeRomBankSwitchableSize);
}