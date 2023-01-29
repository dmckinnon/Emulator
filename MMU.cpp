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
    if (useSystemRom && address < cartridgeHeaderOffset)
    {
        return systemRom[address];
    }
    return memory[address];
}

void MMU::WriteToAddress(uint16_t address, byte value)
{
    // If this write is to ROM bank 0, this is interpreted as a ROM bank switch
    // the value is the bank to switch to
    // TODO this is MBC1. How to use MBC3?
    if (address >= cartridgeRomBank0Offset &&
        address < cartridgeRomBankSwitchableOffset + cartridgeRomBankSwitchableSize)
    {
        // need to keep the damn rom lol
        SwapOutRomBank(rom, value);
    }

    // If write is not allowed, do nothing
    
    memory[address] = value;
}

void MMU::WriteToAddress(uint16_t address, uint16_t value)
{
    // If this write is to a ROM bank, intercept and check it's allowed

    memory[address] = value & 0xFF;
    memory[address + 1] = (value >> 8) & 0xFF;
}