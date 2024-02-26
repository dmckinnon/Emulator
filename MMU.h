#pragma once
#include "Rom.h"

#include <functional>

#ifndef RP2040
#include <mutex>
#include <iostream>
#else
#include "pico/mutex.h"
#endif
//#include <semaphore>

#define MEMORY_SIZE 0x10000

class MMU
{
public:
    // Constructor memsets all usable memory to 0
    // but may have some initialiser values for registers
    MMU(Rom& systemRom);
    ~MMU();

    void LoadRomToMemory(std::shared_ptr<Rom> rom);

    void UnmapSystemRom();

    void SetTimerReset(std::function<void(uint8_t)> timerReset)
    {
        CpuMaybeResetTimer = timerReset;
    }

    inline uint8_t ReadFromAddress(uint16_t address)
    {
        if (useSystemRom && address < cartridgeHeaderOffset)
        {
            return systemRom[address];
        }
        return memory[address];
    }

    inline void WriteToAddress(uint16_t address, uint8_t value)
    {
        // Debug with serial
         if (address == 0xFF02 && value == 0x81)
        {
            unsigned char v = ReadFromAddress(0xFF01);
            std::cout << v << std::flush;

        }


        // Semaphore to allow only one writer to memory at a time
        //writeSemaphore.acquire();
        //const std::lock_guard<std::mutex> lock(oamMutex);

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

        // Writing to the clock divider register or scanline counter register resets it, regardless of value
        // TODO something?
        if (address == DIVRegisterAddress || address == ScanLineCounterAddress)
        {
            value = 0;
        }

        if (address == TMCRegisterAddress)
        {
            // if new frequency is different, reset timer
            CpuMaybeResetTimer(value);
        }

        // Lock access to OAM and VRAM so that only one process can access at once
        // if addreess is VRAM or OAM
        // get mutex

        // If we write to DMA register, capture and cause a DMA transfer
        if (address == DMARegisterAddress)
        {
            memory[address] = value;
            DoDMATransfer();
            //writeSemaphore.release();
            return;
        }

        memory[address] = value;
       // writeSemaphore.release();
    }

    // According to ChatGPT, a 16-bit write functions as two 8-bit writes
    // so we implement it as such
    inline void WriteToAddress(uint16_t address, uint16_t value)
    {
        WriteToAddress(address, (uint8_t)(value & 0xFF));
        WriteToAddress(address + 1, (uint8_t)(value >> 8));
    }

    // offsets and bounds
    static const int interruptsOffset = 0xFF;
    static constexpr int systemRomSize = 0x100;
    static const int cartridgeHeaderOffset = 0x100;
    static const int cartridgeHeaderSize = 0x50;
    static const int cartridgeRomBank0Offset = 0x100;
    static const int cartridgeRomBank0Size = 0x4000;
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

    // Interrupt bits
    static const uint8_t vblankInterruptBit = 0x1;
    static const uint16_t VBlankISRAddress = 0x0040;
    static const uint8_t lcdInterruptBit = 0x2;
    static const uint16_t LCDISRAddress = 0x0048;
    static const uint8_t timerInterruptBit = 0x4;
    static const uint16_t TimerISRAddress = 0x0050;
    static const uint8_t serialInterruptBit = 0x8;
    static const uint16_t SerialISRAddress = 0x0058;
    static const uint8_t joypadInterruptBit = 0x16;
    static const uint16_t JoypadISRAddress = 0x0060;

    // Timer registers and bits
    static const int DIVRegisterAddress = 0xFF04;
    static const int TIMARegisterAddress = 0xFF05;
    static const int TMARegisterAddress = 0xFF06;
    static const int TMCRegisterAddress = 0xFF07;
    static const uint8_t ClockEnableBit = 0x4;
    static const uint8_t TimerControllerBits = 0x3;

    // Display addresses
    static const uint16_t ScanLineCounterAddress = 0xFF44;
    static const uint16_t DesiredScanlineRegisterAddress = 0xFF45;
    static const uint16_t LCDStatusAddress = 0xFF41;
    static const uint16_t LCDControlAddress = 0xFF40;
    static const uint16_t BGScrollYAddress = 0xFF42;
    static const uint16_t BGScrollXAddress = 0xFF43;
    static const uint16_t WindowYAddress = 0xFF4A;
    static const uint16_t WindowXAddress = 0xFF4B;
    static const uint16_t BackgroundColourPaletteAddress = 0xFF47;
    static const uint16_t SpriteColurPaletteAddress0 = 0xFF48;
    static const uint16_t SpriteColurPaletteAddress1 = 0xFF49;
    static const uint16_t SpriteAttributeTableAddress = 0xFE00; 

    // DMA register
    static const uint16_t DMARegisterAddress = 0xFF46;
    

    // Special function just for writing to clock divider
    inline void WriteToDivRegister_Allowed(uint8_t val)
    {
        memory[DIVRegisterAddress] = val;
    }

    inline void WriteToScanlineCounter_Allowed(uint8_t val)
    {
        memory[ScanLineCounterAddress] = val;
    }


private:
    uint8_t memory[MEMORY_SIZE];

    bool useSystemRom;
    uint8_t systemRom[systemRomSize];

    std::shared_ptr<Rom> currentRom;

    //std::counting_semaphore<1> writeSemaphore{1};

    int currentRomBank = 1;
    int mbc = 0;
    bool RomBankEnabled = true;
    static constexpr int MBC_MODE_ADDRESS = 0x147;

    int currentRamBank = 0;
    int numRamBanks = 1;
    bool RAMBankEnabled = false;
    static constexpr int ramBankAddress = 0x148;
    uint8_t* allRam = nullptr;

    void EnableRamBank(uint16_t address, uint8_t value);
    void SwitchLoRomBank(uint8_t value);
    void SwitchHiRomBank(uint8_t value);
    void SwitchRamBank(uint8_t value);
    void SwitchRomRamMode(uint8_t value);
    void SwapOutRomBank(int bank);
    void HandleBanking(uint16_t address, uint8_t value);

    std::function<void(uint8_t)> CpuMaybeResetTimer;

#if defined(__linux) || defined(_WIN32)
    std::mutex oamMutex;
#else
    mutex_t oamMutex;
#endif
    void DoDMATransfer();
};