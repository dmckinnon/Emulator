#include "MMU.h"
#include <string.h>

MMU::MMU()
{
    // memory is statically allocated
    // clear all memory to 0 and then set up any specific registers
    memset(memory, 0, sizeof(memory));

    // set up registers
}

MMU::~MMU()
{
}

void MMU::LoadRomToMemory(Rom& rom)
{
    // copy the rom into memory
    // This fills up Rom bank 0 and the first switchable bank
    // If we need more banks - that is, when the program counter tries to
    // get higher - we swap out the switchable bank
    // TODO read into this
    if (rom.size < cartridgeRomBank0Size)
    {
        memcpy(&memory[cartridgeRomBank0Offset], rom.bytes, rom.size);
    }
    else
    {
        memcpy(&memory[cartridgeRomBank0Offset], rom.bytes, cartridgeRomBank0Size);
        memcpy(&memory[cartridgeRomBankSwitchableOffset], &rom.bytes[cartridgeRomBank0Size], rom.size - cartridgeRomBank0Size);
    }
}

void MMU::SwapOutRomBank(Rom& rom, int bank)
{
    // Swap out the switchable bank with the bank specified
    memcpy(&memory[cartridgeRomBankSwitchableOffset], &rom.bytes[cartridgeRomBank0Size + (bank * cartridgeRomBankSwitchableSize)], cartridgeRomBankSwitchableSize);
}

byte MMU::ReadFromAddress(uint16_t address)
{
    return memory[address];
}

void MMU::WriteToAddress(uint16_t address, byte value)
{
    memory[address] = value;
}

void MMU::WriteToAddress(uint16_t address, uint16_t value)
{
    memory[address] = value & 0xFF;
    memory[address + 1] = (value >> 8) & 0xFF;
}