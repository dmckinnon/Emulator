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

    void LoadRomToMemory(std::shared_ptr<Rom> rom);

    void UnmapSystemRom();

    inline byte ReadFromAddress(uint16_t address)
    {
        if (useSystemRom && address < cartridgeHeaderOffset)
        {
            return systemRom[address];
        }
        return memory[address];
    }

    inline void WriteToAddress(uint16_t address, byte value)
    {
        // Writing to the ROM is interpreted as a ROM/RAM bank switch
        // so we handle this. The logic is intricate.
        if (address < cartridgeRomBankSwitchableOffset + cartridgeRomBankSwitchableSize)
        {
            HandleBanking(address, value);
            return;
        }

        // If we write to RAM we also need to write to the same relative address in ECHO ram?
        if (address >= echoRamOffset && address < echoRamOffset + echoRamSize)
        {
            memory[address - echoRamOffset + cartridgeRamOffset] = value;
        }

        // Writing to the clock divider register resets it, regardless of value
        // TODO
        if (address == DIVRegisterAddress)
        {
            value = 0;
        }

        memory[address] = value;
    }

    // According to ChatGPT, a 16-bit write functions as two 8-bit writes
    // so we implement it as such
    inline void WriteToAddress(uint16_t address, uint16_t value)
    {
        WriteToAddress(address, (byte)(value & 0xFF));
        WriteToAddress(address + 1, (byte)(value >> 8));
    }

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


    // Timer registers and bits
    static const int DIVRegisterAddress = 0xFF04;
    static const int TIMARegisterAddress = 0xFF05;
    static const int TMARegisterAddress = 0xFF06;
    static const int TMCRegisterAddress = 0xFF07;
    static const uint8_t ClockEnableBit = 0x4;
    static const uint8_t TimerInterruptBit = 0x4;

    // Special function just for writing to clock divider
    inline void WriteToDivRegister_Allowed(byte val)
    {
        memory[DIVRegisterAddress] = val;
    }


private:
    byte memory[MEMORY_SIZE];

    bool useSystemRom;
    byte systemRom[systemRomSize];

    std::shared_ptr<Rom> currentRom;

    int currentRomBank = 1;
    int mbc = 0;
    bool RomBankEnabled = true;
    static constexpr int MBC_MODE_ADDRESS = 0x147;

    int currentRamBank = 0;
    int numRamBanks = 1;
    bool RAMBankEnabled = false;
    static constexpr int ramBankAddress = 0x148;
    byte* allRam = nullptr;

    void EnableRamBank(uint16_t address, byte value);
    void SwitchLoRomBank(byte value);
    void SwitchHiRomBank(byte value);
    void SwitchRamBank(byte value);
    void SwitchRomRamMode(byte value);
    void SwapOutRomBank(int bank);
    void HandleBanking(uint16_t address, byte value);
};