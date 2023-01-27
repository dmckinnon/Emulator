#pragma once
#include "Rom.h"

#define MEMORY_SIZE 0xFFFF

class MMU
{
public:
    // Constructor memsets all usable memory to 0
    // but may have some initialiser values for registers
    MMU();
    ~MMU();

    void LoadRomToMemory(Rom& rom);

    void SwapOutRomBank(Rom& rom, int bank);

    inline byte ReadFromAddress(uint16_t address);
    inline void WriteToAddress(uint16_t address, byte value);
    inline void WriteToAddress(uint16_t address, uint16_t value);


private:
    byte memory[MEMORY_SIZE];

    // offsets and bounds
    int interruptsOffset = 0xFF;
    int cartridgeHeaderOffset = 0x100;
    int cartridgeHeaderSize = 0x50;
    int cartridgeRomBank0Offset = 0x150;
    int cartridgeRomBank0Size = 0x4000 - 0x150;
    int cartridgeRomBankSwitchableOffset = 0x4000;
    int cartridgeRomBankSwitchableSize = 0x4000;
    int characterRamOffset = 0x8000;
    int characterRamSize = 0x1800;
    int bgMapData1Offset = 0x9800;
    int bgMapData1Size = 0x400;
    int bgMapData2Offset = 0x9C00;
    int bgMapData2Size = 0x400;
    int cartridgeRamOffset = 0xA000;
    int cartridgeRamSize = 0x2000;
    int internalRamOffset = 0xC000;
    int internalRamSize = 0x2000;
    int internalRamSwitchableOffset = 0xD000;
    int internalRamSwitchableSize = 0x1000;
    int echoRamOffset = 0xE000;
    int echoRamSize = 0x1E00;
    int oamOffset = 0xFE00;
    int oamSize = 0xA0;
    int ioRegistersOffset = 0xFF00;
    int ioRegistersSize = 0x80;
    int highRamOffset = 0xFF80;
    int highRamSize = 0x7F;
    int interruptEnableRegisterAddress = 0xFFFF;


};