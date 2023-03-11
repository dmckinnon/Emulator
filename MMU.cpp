#include "MMU.h"
#include <string.h>
#include <iostream>

MMU::MMU(Rom& systemRom)
{
    // memory is statically allocated
    // clear all memory to 0 and then set up any specific registers
    memset(memory, 0, sizeof(memory));

    // Start with system ROM mapped and load default system ROM to it
    useSystemRom = true;
    memcpy(&this->systemRom, systemRom.uint8_ts, systemRom.size);

    // set up registers. See 
    // http://www.codeslinger.co.uk/pages/projects/gameboy/hardware.html
    // and
    // https://gbdev.io/pandocs/Power_Up_Sequence.html
    // I'll just use absolute values here, since there are the links to read
    memory[0xFF00] = 0xCF; // P1
    memory[0xFF01] = 0x00; // SB
    memory[0xFF02] = 0x7E; // SC
    memory[0xFF04] = 0xAB; // DIV
    memory[0xFF05] = 0x00; // TIMA
    memory[0xFF06] = 0x00; // TMA
    memory[0xFF07] = 0xF8; // TAC
    memory[0xFF0F] = 0xE1; // IF
    memory[0xFF10] = 0x80; // NR10
    memory[0xFF11] = 0xBF; // NR11
    memory[0xFF12] = 0xF3; // NR12
    memory[0xFF13] = 0xFF; // NR13
    memory[0xFF14] = 0xBF; // NR14
    memory[0xFF16] = 0x3F; // NR21
    memory[0xFF16] = 0x00; // NR22
    memory[0xFF18] = 0xFF; // NR23
    memory[0xFF19] = 0xBF; // NR24
    memory[0xFF1A] = 0x7F; // NR30
    memory[0xFF1B] = 0xFF; // NR31
    memory[0xFF1C] = 0x9F; // NR32
    memory[0xFF1D] = 0xFF; // NR33
    memory[0xFF1E] = 0xBF; // NR34
    memory[0xFF20] = 0xFF; // NR41
    memory[0xFF21] = 0x00; // NR42
    memory[0xFF22] = 0x00; // NR43
    memory[0xFF23] = 0xBF; // NR44
    memory[0xFF24] = 0x77; // NR50
    memory[0xFF25] = 0xF3; // NR51
    memory[0xFF26] = 0xF1; // NR52
    memory[0xFF40] = 0x91; // LCDC
    memory[0xFF41] = 0x81; // STAT
    memory[0xFF42] = 0x00; // SCY
    memory[0xFF43] = 0x00; // SCX
    memory[0xFF44] = 0x91; // LY
    memory[0xFF45] = 0x00; // LYC
    memory[0xFF46] = 0xFF; // DMA
    memory[0xFF47] = 0xFC; // BGP

    // Note: these two could cause troube visually
    // with their init value. Do either 0 or FF
    memory[0xFF48] = 0x00; // OBP0
    memory[0xFF49] = 0x00; // OBP1

    memory[0xFF4A] = 0x00; // WY
    memory[0xFF4B] = 0x00; // WX
    memory[0xFF4D] = 0xFF; // KEY1
    memory[0xFF4F] = 0xFF; // VBK
    memory[0xFF51] = 0xFF; // HDMA1
    memory[0xFF52] = 0xFF; // HDMA2
    memory[0xFF53] = 0xFF; // HDMA3
    memory[0xFF54] = 0xFF; // HDMA4
    memory[0xFF55] = 0xFF; // HDMA5
    memory[0xFF56] = 0xFF; // RP
    memory[0xFF68] = 0xFF; // BCPS
    memory[0xFF69] = 0xFF; // BCPD
    memory[0xFF6A] = 0xFF; // OCPS
    memory[0xFF6B] = 0xFF; // OCPD
    memory[0xFF70] = 0xFF; // SVBK
    memory[0xFFFF] = 0x00; // IE
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
        memcpy(&memory[cartridgeRomBank0Offset], rom->uint8_ts, rom->size);
    }
    else
    {
        memcpy(&memory[cartridgeRomBank0Offset], rom->uint8_ts, cartridgeRomBank0Size);
        memcpy(&memory[cartridgeRomBankSwitchableOffset], &rom->uint8_ts[cartridgeRomBank0Size], rom->size - cartridgeRomBank0Size);
    }

    // Determine which MBC the game uses
    int mbcMode = memory[MBC_MODE_ADDRESS];
    switch (mbcMode)
    {
        case 1:
        case 2:
        case 3:
            mbc = 1;
            break;
        case 5:
        case 6:
            mbc = 2;
            break;
        // TODO what's MBC 3
        default:
            mbc = 0;
            break;
    }

    // Check how many RAM banks the game wants
    // TODO have I got the sizes right here?
    numRamBanks = memory[ramBankAddress];
    if (mbc == 1)
    {
        allRam = new uint8_t[numRamBanks * cartridgeRamSize];
    }
    
}

void MMU::UnmapSystemRom()
{
    useSystemRom = false;
    if (allRam != nullptr)
        delete allRam;
}

void MMU::HandleBanking(uint16_t address, uint8_t value)
{
    // Handle RAM enabling
    if (address < 0x2000)
    {
        if (mbc == 1 || mbc == 2)
        {
            EnableRamBank(address, value);
        }
    }

    // Handle ROM bank switching for lower addresses
    else if (address >= 0x2000 && address < 0x4000)
    {
        if (mbc == 1 || mbc == 2)
        {
            SwitchLoRomBank(value);
        }
    }

    // Handle ROM or RAM bank switching for higher addresses
    else if (address >= 0x4000 && address < 0x6000)
    {
        if (mbc == 1)
        {
            if (RomBankEnabled)
            {
                SwitchHiRomBank(value);
            }
            else
            {
                SwitchRamBank(value);
            }
        }
    }

    // Handle ROM/RAM banking mode switching
    else if (address >= 0x6000 && address < 0x8000)
    {
        if (mbc == 1)
        {
            SwitchRomRamMode(value);
        }
    }
}

void MMU::EnableRamBank(uint16_t address, uint8_t value)
{
    if (mbc == 2)
    {
        // Under MBC2, the 4th bit of the address must be 0 to perform RAM enable this way
        if ((address & 0x0010) == 0x0010)
        {
            return;
        }
    }

    // All other logic is the same regardless of MBC1 or 2
    if ((value & 0x0F) == 0x0A)
    {
        RAMBankEnabled = true;
    }
    else
    {
        RAMBankEnabled = false;
    }
}

void MMU::SwitchLoRomBank(uint8_t value)
{
    int bank = 0;
    // MBC2 only uses the lower 4 bits of the value written to
    // determine which ROM bank to use
    if (mbc == 2)
    {
        bank = value & 0x0F;
    }
    // MBC1 uses the lower 5 bits of the value
    else if (mbc == 1)
    {
        bank = value & 0x1F;
        // Yeet the bottom 5 bits of the current bank
        // and replace with what we just got
        bank |= currentRomBank & 0xE0;
    }

    // Swap out the switchable bank with the bank specified
    SwapOutRomBank(bank);
}

void MMU::SwitchHiRomBank(uint8_t value)
{
    // Note: this is only used for MBC1

    // the new bank value is the old bank value
    // with the top 3 bits replaced by the top 3 bits incoming
    int bank = currentRomBank & 0x1F;
    bank |= value & 0xE0;
    // Swap out the switchable bank with the bank specified
    SwapOutRomBank(bank);
}

void MMU::SwitchRamBank(uint8_t value)
{
    // Note: this is only used for MBC1

    if (allRam == nullptr)
    {
        // something bad happened
        std::cerr << "Tried to switch RAM bank when no RAM banks exist" << std::endl;
    }

    // page out old ram
    memcpy(&allRam[cartridgeRamSize*currentRamBank], &memory[cartridgeRamOffset], cartridgeRamSize);

    // MBC1 uses the lower 2 bits of the value
    int currentRamBank = value & 0x03;

    // Swap out the switchable bank with the bank specified
    memcpy(&memory[cartridgeRamOffset], &allRam[cartridgeRamSize*currentRamBank], cartridgeRamSize);
}

void MMU::SwapOutRomBank(int bank)
{
    // bank 0 is always loaded; make it bank 1 instead
    if (bank == 0)
    {
        bank = 1;
    }

    // If the value is the same as the current bank, do nothing
    if (bank == currentRomBank)
    {
        return;
    }

    // Swap out the switchable bank with the bank specified
    currentRomBank = bank;
    memcpy(&memory[cartridgeRomBankSwitchableOffset], &currentRom->uint8_ts[cartridgeRomBank0Size + (bank * cartridgeRomBankSwitchableSize)], cartridgeRomBankSwitchableSize);
}

void MMU::SwitchRomRamMode(uint8_t value)
{
    RomBankEnabled = value & 0x01? false : true;
    if (RomBankEnabled)
    {
        uint8_t newRamBank = 0;
        SwitchRamBank(newRamBank);
    }
}

void MMU::DoDMATransfer()
{
    uint16_t address = memory[DMARegisterAddress] << 8;
    // copy 160 bytes from address to OAM
    std::lock_guard<std::mutex> lock(oamMutex);
    memcpy(memory+address, memory+oamOffset, oamSize);
}