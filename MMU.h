#pragma once
#include "Rom.h"

#define MEMORY_SIZE 0xFFFF

class MMU
{
public:
    // Constructor memsets all usable memory to 0
    // but may have some initialiser values for registers
    MMU(Rom& systemRom);
    ~MMU();

    void LoadRomToMemory(Rom& rom);

    void UnmapSystemRom();

    void SwapOutRomBank(Rom& rom, int bank);

    inline byte ReadFromAddress(uint16_t address);
    inline void WriteToAddress(uint16_t address, byte value);
    inline void WriteToAddress(uint16_t address, uint16_t value);

    // offsets and bounds
    static const int interruptsOffset = 0xFF;
    static constexpr int systemRomSize = 0x100;
    static const int cartridgeHeaderOffset = 0x100;
    static const int cartridgeHeaderSize = 0x50;
    static const int cartridgeRomBank0Offset = 0x150;
    static const int cartridgeRomBank0Size = 0x4000 - 0x150;
    static const int cartridgeRomBankSwitchableOffset = 0x4000;
    static const int cartridgeRomBankSwitchableSize = 0x4000;
    static const int characterRamOffset = 0x8000;
    static const int characterRamSize = 0x1800;
    static const int bgMapData1Offset = 0x9800;
    static const int bgMapData1Size = 0x400;
    static const int bgMapData2Offset = 0x9C00;
    static const int bgMapData2Size = 0x400;
    static const int cartridgeRamOffset = 0xA000;
    static const int cartridgeRamSize = 0x2000;
    static const int internalRamOffset = 0xC000;
    static const int internalRamSize = 0x2000;
    static const int internalRamSwitchableOffset = 0xD000;
    static const int internalRamSwitchableSize = 0x1000;
    static const int echoRamOffset = 0xE000;
    static const int echoRamSize = 0x1E00;
    static const int oamOffset = 0xFE00;
    static const int oamSize = 0xA0;
    static const int ioRegistersOffset = 0xFF00;
    static const int interruptFlagRegisterAddress = 0xFF0F;
    static const int ioRegistersSize = 0x80;
    static const int highRamOffset = 0xFF80;
    static const int highRamSize = 0x7F;
    static const int interruptEnableRegisterAddress = 0xFFFF;


private:
    byte memory[MEMORY_SIZE];

    bool useSystemRom;
    byte systemRom[systemRomSize];


};