#include "CPU.h"
#include "Instructions.h"

#include <iostream>
#include <string.h>
#include <thread>

#define CLOCKSPEED 4194304 

using namespace std;

CPU::CPU()
{
    clockCounter = 0;
    clockDivider = 0;
    executing = false;
    InitialiseRegisters();
}

CPU::CPU(std::shared_ptr<MMU> mmu)
{
    this->mmu = mmu;
    clockCounter = 0;
    clockDivider = 0;
    executing = false;
    
    InitialiseRegisters();
}

CPU::~CPU()
{

}

void CPU::SetVBlankInterrupt()
{
    uint8_t interrupts = mmu->ReadFromAddress(MMU::interruptFlagRegisterAddress);
    mmu->WriteToAddress(MMU::interruptFlagRegisterAddress, (uint8_t)(interrupts | MMU::vblankInterruptBit));
}

void CPU::SetLCDStatInterrupt()
{
    uint8_t interrupts = mmu->ReadFromAddress(MMU::interruptFlagRegisterAddress);
    mmu->WriteToAddress(MMU::interruptFlagRegisterAddress, (uint8_t)(interrupts | MMU::lcdInterruptBit));
}

// This takes the input format of the 0xFF00 register
void CPU::SetJoypadInterrupt(uint8_t joypadRegister)
{
    // Set the value for which IO channel is raised
    uint8_t ioReg = mmu->ReadFromAddress(MMU::ioRegistersOffset);
    mmu->WriteToAddress(MMU::ioRegistersOffset, (uint8_t)(ioReg | joypadRegister));

    // Set the IO interrupt flag
    uint8_t interrupts = mmu->ReadFromAddress(MMU::interruptFlagRegisterAddress);
    mmu->WriteToAddress(MMU::interruptFlagRegisterAddress, (uint8_t)(interrupts | MMU::joypadInterruptBit));
}

void CPU::InitialiseRegisters()
{
    // The stack starts at 0xFFFE and grows down
    // This is High ram - and an overflow will go into
    // io register space. So we need to block that and throw a
    // stack overflow exception in that case
    registers.shorts[SP] = 0xFFFE;

    // the program counter just starts here, unless we want the system ROM
    // Realistically, we can skip the system ROM
#ifdef SKIP_BOOT_ROM
    registers.shorts[PC] = 0x100;
#else
    registers.shorts[PC] = 0;
#endif

    // These registers just contain these values apparently
    // see https://gbdev.io/pandocs/Power_Up_Sequence.html
    registers.shorts[AF] = 0x0100;
    registers.shorts[BC] = 0xFF13;
    registers.shorts[DE] = 0x00C1;
    registers.shorts[HL] = 0x8403;

}

void CPU::ExecuteCycles(int numMCycles)
{
    clockDivider += 4*numMCycles;
    mmu->WriteToDivRegister_Allowed(clockDivider >> 8);


    // Note: clock must be enabled to update clock
    bool isClockEnabled = mmu->ReadFromAddress(MMU::TMCRegisterAddress) & MMU::ClockEnableBit;
    if (isClockEnabled)
    {
        clockCounter += 4*numMCycles;

        // TIMA only gets updated at the set frequency, downsampled from the 
        // system clock
        int frequency = GetTmcFrequency();
        if (clockCounter >= CLOCKSPEED/frequency)
        {   
            // reset clockCounter

            // Read clock
            uint8_t timaVal = mmu->ReadFromAddress(MMU::TIMARegisterAddress);
            timaVal ++;
            // Check overflow
            if (timaVal == 0)
            {
                // clock overflow occurred - request timer interrupt
                uint8_t currentInterrupts = mmu->ReadFromAddress(MMU::interruptFlagRegisterAddress);
                mmu->WriteToAddress(MMU::interruptFlagRegisterAddress, (uint8_t)(MMU::timerInterruptBit | currentInterrupts));

                // replace TIMA register with TMA register
                // NOTE: technically this should happen one m-cycle *after*
                // the overflow occurred - and it is possible to write in this period and overload the TMA write
                // Right now, we are ignoring this since m-cycles are grouped
                mmu->WriteToAddress(MMU::TIMARegisterAddress, mmu->ReadFromAddress(MMU::TMARegisterAddress));
                // add the m-cycle that should occur here
                clockCounter += 4;
            }
            else
            {
                mmu->WriteToAddress(MMU::TIMARegisterAddress, timaVal);
            }
        }
    }
}

void CPU::CheckAndMaybeHandleInterrupts()
{
    // Check if any interrupt flags are set. If they are, handle the top priority one
    // take a lock on reading interupt flags as other threads can set them

    // check that IE bit and IF bit are both one before running that interrupt

    // this takes 20 clock cycles to just dispatch

    // remember to disable interrupts after handling
    // it is expected that an ISR uses RETI to return

    // check interrupts in priority order
    uint8_t interruptEnable = mmu->ReadFromAddress(MMU::interruptEnableRegisterAddress);
    uint8_t interruptRequests = mmu->ReadFromAddress(MMU::interruptFlagRegisterAddress);

    uint8_t whichInterrupt = interruptEnable & interruptRequests;
    // if we had any interrupt, push PC onto stack, disable interrupts
    if (whichInterrupt != 0)
    {
        // push PC onto Stack
        mmu->WriteToAddress(--registers.shorts[SP], registers.bytes[PC]);
        mmu->WriteToAddress(--registers.shorts[SP], registers.bytes[PC + 1]);

        // disable interrupts
        interruptsAreEnabled = 0;
    }

    // Set new PC based on interrupt
    if (whichInterrupt & MMU::vblankInterruptBit)
    {
        // Set PC to start of VBlank ISR
        registers.shorts[PC] = MMU::VBlankISRAddress;      
    }
    else if (whichInterrupt & MMU::lcdInterruptBit)
    {
        // Set PC to start of LCD Stat ISR
        registers.shorts[PC] = MMU::LCDISRAddress;
    }
    else if (whichInterrupt & MMU::timerInterruptBit)
    {
        registers.shorts[PC] = MMU::TimerISRAddress;
    }
    else if (whichInterrupt & MMU::serialInterruptBit)
    {
        registers.shorts[PC] = MMU::SerialISRAddress;
    }
    else if (whichInterrupt & MMU::joypadInterruptBit)
    {
        registers.shorts[PC] = MMU::JoypadISRAddress;
    }

    // execute 20 clock cycles = 5 machine cycles
    // note that this won't trigger an interrupt if the timer overflows
    // as we are currently serving an ISR
    ExecuteCycles(5);

    // continue execution
}

void CPU::ExecuteCode()
{
    using namespace std::chrono_literals;
    executing = true;
    bool stillInSystemRom = true;

    if (!mmu)
    {
        return;
    }
    
    while (true)
    {
        // If PC has reached 0x100, then cartridge is valid;
        // unmap system ROM and use cartridge ROM from now on.
        // TODO: only do this once
        if (registers.shorts[PC] >= 0x100 && stillInSystemRom)
        {
            mmu->UnmapSystemRom();
            stillInSystemRom = false;
            while (true)
            {

            }
        }

        // The loop logic is:
        // Execute next instruction and get number of cycles it took
        // Increment the timer by this number of cycles
        // Check for any interrupts and affect the PC if need be

        int cycles = ExecuteNextInstruction();
        
        // Spin the clock for the number of mCycles (= 4 regular cycles) the previous instruction took
        // timers to increment, timer interrupts to fire, etc. If an interrupt occurs,
        // break and handle this before continuing
        ExecuteCycles(cycles);

        // Check if interrupts are pending:
        // if they are, execute
        // Do not do this if interrupts were just enabled - that lets them happen on the next cycle
        // This boolean functions as the IME flag - this en/dis-ables jumping to interrupt vectors
        if (interruptsAreEnabled)
        {
            CheckAndMaybeHandleInterrupts();
        }

        // EI, DI and RETI change interrupt enablement on the cycle after their instruction
        if (enableInterruptsNextCycle)
        {
            interruptsAreEnabled = true;
            enableInterruptsNextCycle = false;
            // Write to interrupt enable register

        }
        else if (disableInterruptsNextCycle)
        {
            interruptsAreEnabled = false;
            disableInterruptsNextCycle = false;
        }

        // Stop executing if program counter goes past ROM
        if (registers.shorts[PC] >= MMU::cartridgeRomBankSwitchableOffset + MMU::cartridgeRomBankSwitchableSize)
        {
            break;
        }

        std::this_thread::sleep_for(1ms);
    }

    // end execution
    executing = false;
}


inline void CPU::Add8(uint8_t A, uint8_t B, uint8_t& C)
{
    // check half carry flag:
    if ((((A & 0xf) + (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A + (uint16_t)B;
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~CarryFlag;
    }

    // check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // Reset subtraction flag
    registers.bytes[F] &= ~AddSubFlag;
}

inline void CPU::Inc8(uint8_t& A)
{
    // check half carry flag
    if (((A & 0xf) + 1 & 0xF0) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    A = A + 1;

    // check zero:
    if (A == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // reset the subtract flag
    registers.bytes[F] &= ~AddSubFlag;
}

inline void CPU::Dec8(uint8_t& A)
{
    // check half carry flag
    if (((A & 0xf) - 1 & 0xF0) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    A = A - 1;

    // set or clear zero flag
    if (A == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // Activate the subtract flag
    registers.bytes[F] |= AddSubFlag;
}

inline void CPU::Sub8(uint8_t A, uint8_t B, uint8_t& C)
{
    // check half carry flag:
    if ((((A & 0xf) - (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A - (uint16_t)B;
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~CarryFlag;
    }

    // Check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // activate subtraction flag
    registers.bytes[F] |= AddSubFlag;
}

inline void CPU::AddC8(uint8_t A, uint8_t B, uint8_t& C)
{
    // check half carry flag:
    if ((((A & 0xf) + (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A + (uint16_t)B + (uint16_t)(registers.bytes[F] & CarryFlag >> 4);
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~CarryFlag;
    }

    // check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // Reset subtraction flag
    registers.bytes[F] &= ~AddSubFlag;
}

inline void CPU::SubC8(uint8_t A, uint8_t B, uint8_t& C)
{
    // check half carry flag:
    if ((((A & 0xf) - (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A - (uint16_t)B - (uint16_t)(registers.bytes[F] & CarryFlag >> 4);
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~CarryFlag;
    }

    // Check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // activate subtraction flag
    registers.bytes[F] |= AddSubFlag;
}

inline void CPU::And8(uint8_t A, uint8_t B, uint8_t& C)
{
    C = A & B;

    // check zero:
    if (C == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // reset subtraction flag
    registers.bytes[F] &= ~AddSubFlag;

    // reset carry flag
    registers.bytes[F] &= ~CarryFlag;

    // set half carry flag
    registers.bytes[F] |= HalfCarryFlag;
}

inline void CPU::Or8(uint8_t A, uint8_t B, uint8_t& C)
{
    C = A | B;

    // check zero:
    if (C == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // reset subtraction flag
    registers.bytes[F] &= ~AddSubFlag;

    // reset carry flag
    registers.bytes[F] &= ~CarryFlag;

    // reset half carry flag
    registers.bytes[F] &= ~HalfCarryFlag;
}

inline void CPU::Xor8(uint8_t A, uint8_t B, uint8_t& C)
{
    C = A ^ B;

    // check zero:
    if (C == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // reset subtraction flag
    registers.bytes[F] &= ~AddSubFlag;

    // reset carry flag
    registers.bytes[F] &= ~CarryFlag;

    // reset half carry flag
    registers.bytes[F] &= ~HalfCarryFlag;
}

inline void CPU::Cp8(uint8_t A, uint8_t B)
{
    // check half carry flag:
    if ((((A & 0xf) - (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A - (uint16_t)B;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~CarryFlag;
    }

    // Check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }
    else
    {
        registers.bytes[F] &= ~ZeroFlag;
    }

    // activate subtraction flag
    registers.bytes[F] |= AddSubFlag;
}

inline void CPU::Add16(uint16_t& A, uint16_t& B, uint16_t& C)
{
    // check half carry flag:
    if ((((A & 0xfff) + (B & 0xfff)) & 0x1000) == 0x1000)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~HalfCarryFlag;
    }
    

    // check full carry flag
    uint32_t r = (uint32_t)A + (uint32_t)B;
    C = (uint16_t)r;

    if (r & 0x10000)
    {
        registers.bytes[F] |= CarryFlag;
    }
    else
    {
        registers.bytes[F] &= ~CarryFlag;
    }

    // Reset subtraction flag
    registers.bytes[F] &= ~AddSubFlag;
}


int CPU::ExecuteNextInstruction()
{
    // get instruction at program counter
    uint8_t instruction = mmu->ReadFromAddress(registers.shorts[PC]);
    bool pcChanged = false;

    // number of cycles the instruction will take to execute
    int mCycles = 1;
    int oldPC = registers.shorts[PC];

    switch (instruction)
    {
        // Control
        case NOP:
        {
            // NOP takes 1 M-cycle == 4 clock cycles
            mCycles += 1;
            break;
        }
        case STOP:
        {
            // HALT EXECUTION
            // hack for now
            // not actually sure what this shoudl do
            return 0;
        }
        case DI:
        {
            disableInterruptsNextCycle = true;
            mCycles += 1;
            break;
        }
        case EI:
        {
            enableInterruptsNextCycle = true;
            mCycles += 1;
            break;
        }

        ////////////////////
        // Jumps

        case JUMP_HL:
        {
            // Jump to address in HL
            pcChanged = true;
            registers.shorts[PC] = registers.shorts[HL];
            mCycles += 1;
            break;
        }
        case JUMP_a16:
        {
            pcChanged = true;
            // Jump to address in next 2 bytes
            registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            mCycles += 3;
            break;
        }

        // Conditional jumps
        case JUMP_C_a16:
        {
            pcChanged = true;
            // Jump to address in next 2 bytes if carry flag is set
            if (registers.bytes[F] & CarryFlag)
            {
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 2;
            }
            break;
        }
        case JUMP_NC_a16:
        {
            pcChanged = true;
            // Jump to address in next 2 bytes if carry flag is not set
            if (!(registers.bytes[F] & CarryFlag))
            {
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 2;
            }
            break;
        }
        case JUMP_Z_a16:
        {
            pcChanged = true;
            // Jump to address in next 2 bytes if zero flag is set
            if (registers.bytes[F] & ZeroFlag)
            {
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 2;
            }
            break;
        }
        case JUMP_NZ_a16:
        {
            pcChanged = true;
            // Jump to address in next 2 bytes if zero flag is not set
            if (!(registers.bytes[F] & ZeroFlag))
            {
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 2;
            }
            break;
        }

        // Relative jumps
        case JUMP_REL_r8:
        {
            pcChanged = true;
            // Jump to address relative to current PC
            // This is a signed 8 bit value
            int8_t offset = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] += offset + 2;
            mCycles += 3;
            break;
        }
        case JUMP_REL_C_r8:
        {
            pcChanged = true;
            // Jump to address relative to current PC if carry flag is set
            // This is a signed 8 bit value
            if (registers.bytes[F] & CarryFlag)
            {
                int8_t offset = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                registers.shorts[PC] += offset + 2;
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 2;
                mCycles += 2;
            }
            break;
        }
        case JUMP_REL_NC_r8:
        {
            pcChanged = true;
            // Jump to address relative to current PC if carry flag is not set
            // This is a signed 8 bit value
            if (!(registers.bytes[F] & CarryFlag))
            {
                int8_t offset = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                registers.shorts[PC] += offset + 2;
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 2;
                mCycles += 2;
            }
            break;
        }
        case JUMP_REL_Z_r8:
        {
            pcChanged = true;
            // Jump to address relative to current PC if zero flag is set
            // This is a signed 8 bit value
            if (registers.bytes[F] & ZeroFlag)
            {
                int8_t offset = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                registers.shorts[PC] += offset + 2;
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 2;
                mCycles += 2;
            }
            break;
        }
        case JUMP_REL_NZ_r8:
        {
            pcChanged = true;
            // Jump to address relative to current PC if zero flag is not set
            // This is a signed 8 bit value
            if (!(registers.bytes[F] & ZeroFlag))
            {
                int8_t offset = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                registers.shorts[PC] += offset + 2;
                mCycles += 3;
            }
            else
            {
                registers.shorts[PC] += 2;
                mCycles += 2;
            }
            break;
        }

        // Calls and returns
        case CALL_a16:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address in next 2 bytes
            printf("Push current PC: %x. Jumping to %x\n",
                registers.shorts[PC]+3,
                mmu->ReadFromAddress(registers.shorts[PC] + 1));
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 3));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 3) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            mCycles += 6;
            break;
        }
        case CALL_C_a16:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address in next 2 bytes if carry flag is set
            if (registers.bytes[F] & CarryFlag)
            {
                mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 3));
                mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 3) >> 8));
                --registers.shorts[SP];
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 6;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 3;
            }
            break;
        }
        case CALL_NC_a16:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address in next 2 bytes if carry flag is not set
            if (!(registers.bytes[F] & CarryFlag))
            {
                mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 3));
                mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 3) >> 8));
                --registers.shorts[SP];
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 6;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 3;
            }
            break;
        }
        case CALL_Z_a16:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address in next 2 bytes if zero flag is set
            if (registers.bytes[F] & ZeroFlag)
            {
                mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 3));
                mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 3) >> 8));
                --registers.shorts[SP];
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 6;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 3;
            }
            break;
        }
        case CALL_NZ_a16:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address in next 2 bytes if zero flag is not set
            if (!(registers.bytes[F] & ZeroFlag))
            {
                mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 3));
                mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 3) >> 8));
                --registers.shorts[SP];
                registers.shorts[PC] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
                mCycles += 6;
            }
            else
            {
                registers.shorts[PC] += 3;
                mCycles += 3;
            }
            break;
        }

        case RETURN:
        {
            pcChanged = true;
            // Pop 2 bytes from stack and jump to that address
            uint16_t val = (mmu->ReadFromAddress(registers.shorts[SP]+1) << 8) | (mmu->ReadFromAddress(registers.shorts[SP] + 2));

            printf("Return to %x, %d\n", val, val);
            registers.shorts[PC] = mmu->ReadFromAddress(++registers.shorts[SP]) << 8;
            uint16_t topBits = mmu->ReadFromAddress(++registers.shorts[SP]);
            registers.shorts[PC] |= topBits;
            mCycles += 4;
            break;
        }
        case RETURN_I:
        {
            pcChanged = true;
            // Pop 2 bytes from stack and jump to that address
            registers.shorts[PC] = mmu->ReadFromAddress(++registers.shorts[SP]) << 8;
            registers.shorts[PC] |= mmu->ReadFromAddress(++registers.shorts[SP]);
            // Enable interrupts
            enableInterruptsNextCycle = true;
            mCycles += 4;
            break;
        }
        case RETURN_C:
        {
            pcChanged = true;
            // Pop 2 bytes from stack and jump to that address if carry flag is set
            if (registers.bytes[F] & CarryFlag)
            {
                registers.shorts[PC] = mmu->ReadFromAddress(++registers.shorts[SP]) << 8;
                registers.shorts[PC] |= mmu->ReadFromAddress(++registers.shorts[SP]);
                mCycles += 5;
            }
            else
            {
                mCycles += 2;
            }
            break;
        }
        case RETURN_NC:
        {
            pcChanged = true;
            // Pop 2 bytes from stack and jump to that address if carry flag is not set
            if (!(registers.bytes[F] & CarryFlag))
            {
                registers.shorts[PC] = mmu->ReadFromAddress(++registers.shorts[SP]) << 8;
                registers.shorts[PC] |= mmu->ReadFromAddress(++registers.shorts[SP]);
                mCycles += 5;
            }
            else
            {
                mCycles += 2;
            }
            break;
        }
        case RETURN_Z:
        {
            pcChanged = true;
            // Pop 2 bytes from stack and jump to that address if zero flag is set
            if (registers.bytes[F] & ZeroFlag)
            {
                registers.shorts[PC] = mmu->ReadFromAddress(++registers.shorts[SP]) << 8;
                registers.shorts[PC] |= mmu->ReadFromAddress(++registers.shorts[SP]);
                mCycles += 5;
            }
            else
            {
                mCycles += 2;
            }
            break;
        }
        case RETURN_NZ:
        {
            pcChanged = true;
            // Pop 2 bytes from stack and jump to that address if zero flag is not set
            if (!(registers.bytes[F] & ZeroFlag))
            {
                registers.shorts[PC] = mmu->ReadFromAddress(++registers.shorts[SP]) << 8;
                registers.shorts[PC] |= mmu->ReadFromAddress(++registers.shorts[SP]);
                mCycles += 5;
            }
            else
            {
                mCycles += 2;
            }
            break;
        }

        // RESET instructions
        case RESET_00H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0000
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0000;
            mCycles += 4;
            break;
        }
        case RESET_08H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0008
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0008;
            mCycles += 4;
            break;
        }
        case RESET_10H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0010
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0010;
            mCycles += 4;
            break;
        }
        case RESET_18H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0018
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0018;
            mCycles += 4;
            break;
        }
        case RESET_20H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0020
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0020;
            mCycles += 4;
            break;
        }
        case RESET_28H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0028
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0028;
            mCycles += 4;
            break;
        }
        case RESET_30H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0030
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0030;
            mCycles += 4;
            break;
        }
        case RESET_38H:
        {
            pcChanged = true;
            // Push current PC onto stack and jump to address 0x0038
            mmu->WriteToAddress(registers.shorts[SP], (uint8_t)(registers.shorts[PC] + 1));
            mmu->WriteToAddress(--registers.shorts[SP], (uint8_t)((registers.shorts[PC] + 1) >> 8));
            --registers.shorts[SP];
            registers.shorts[PC] = 0x0038;
            mCycles += 4;
            break;
        }
        

        /////////////////////////////
        // 8 bit arithmetic and logic

        // Set carry flag instruction
        case SCF:
        {
            // Set carry, clear N and H
            registers.bytes[F] |= CarryFlag;
            // reset subtraction flag
            registers.bytes[F] &= ~AddSubFlag;
            // reset half carry flag
            registers.bytes[F] &= ~HalfCarryFlag;
            mCycles += 1;
            break;
        }

        // Decimal Adjust Accumulator instruction
        // This adjusts binary computation and flags to be BCD
        case DAA:
        {
            mCycles += 1;
            uint8_t a = registers.bytes[A];
            if (!(registers.bytes[F] & AddSubFlag))
            {  // after an addition, adjust if (half-)carry occurred or if result is out of bounds
                if ((registers.bytes[F] & CarryFlag) || a > 0x99)
                {
                    a += 0x60;
                    registers.bytes[F] |= CarryFlag;
                }
                if ((registers.bytes[F] & HalfCarryFlag) || (a & 0x0f) > 0x09)
                {
                    a += 0x6;
                }
            }
            else
            {  // after a subtraction, only adjust if (half-)carry occurred
                if (registers.bytes[F] & CarryFlag)
                {
                    a -= 0x60;
                }
                if (registers.bytes[F] & HalfCarryFlag)
                {
                    a -= 0x6;
                }
            }
            // these flags are always updated
            // the usual z flag
            if (a == 0)
            {
                registers.bytes[F] |= ZeroFlag;
            }
            registers.bytes[F] &= ~HalfCarryFlag; // h flag is always cleared

            // Put the updated value back into register A
            registers.bytes[A] = a;

            break;
        }

        // Complement Accumulator instruction
        // This flips all bits in the accumulator
        case CPL:
        {
            // Flip all bits in A
            registers.bytes[A] = ~registers.bytes[A];
            // Set subtraction flag
            registers.bytes[F] |= AddSubFlag;
            // Set half carry flag
            registers.bytes[F] |= HalfCarryFlag;
            mCycles += 1;
            break;
        }

        // Complement Carry Flag instruction
        // This flips the carry flag
        case CCF:
        {
            // Flip carry flag
            registers.bytes[F] ^= CarryFlag;
            // Reset subtraction flag
            registers.bytes[F] &= ~AddSubFlag;
            // Reset half carry flag
            registers.bytes[F] &= ~HalfCarryFlag;
            mCycles += 1;
            break;
        }

        // Add each register to A
        case ADD_A_A:
        {
            Add8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_B:
        {
            Add8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_C:
        {
            Add8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_D:
        {
            Add8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_E:
        {
            Add8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_H:
        {
            Add8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_L:
        {
            Add8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_HL:
        {
            uint8_t hl = mmu->ReadFromAddress(registers.shorts[HL]);
            Add8(registers.bytes[A], hl, registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADD_A_d8:
        {
            Add8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // Subtract each register from A
        case SUB_A:
        {
            Sub8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_B:
        {
            Sub8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_C:
        {
            Sub8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_D:
        {
            Sub8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_E:
        {
            Sub8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_H:
        {
            Sub8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_L:
        {
            Sub8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_HL:
        {
            Sub8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]), registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SUB_d8:
        {
            Sub8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // Add each register with A and the carry flag
        case ADC_A:
        {
            AddC8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_B:
        {
            AddC8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_C:
        {
            AddC8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_D:
        {
            AddC8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_E:
        {
            AddC8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_H:
        {
            AddC8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_L:
        {
            AddC8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_HL:
        {
            AddC8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]), registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case ADC_d8:
        {
            AddC8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // Subtract each register and the carry flag from A 
        case SBC_A:
        {
            SubC8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_B:
        {
            SubC8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_C:
        {
            SubC8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_D:
        {
            SubC8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_E:
        {
            SubC8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_H:
        {
            SubC8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_L:
        {
            SubC8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_HL:
        {
            SubC8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]), registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case SBC_d8:
        {
            SubC8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // AND each register with A
        case AND_A:
        {
            And8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_B:
        {
            And8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_C:
        {
            And8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_D:
        {
            And8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_E:
        {
            And8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_H:
        {
            And8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_L:
        {
            And8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_HL:
        {
            And8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]), registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case AND_d8:
        {
            And8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // OR each register with A
        case OR_A:
        {
            Or8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_B:
        {
            Or8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_C:
        {
            Or8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_D:
        {
            Or8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_E:
        {
            Or8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_H:
        {
            Or8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_L:
        {
            Or8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_HL:
        {
            Or8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]), registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case OR_d8:
        {
            Or8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // XOR each register with A
        case XOR_A:
        {
            Xor8(registers.bytes[A], registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_B:
        {
            Xor8(registers.bytes[A], registers.bytes[B], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_C:
        {
            Xor8(registers.bytes[A], registers.bytes[C], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_D:
        {
            Xor8(registers.bytes[A], registers.bytes[D], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_E:
        {
            Xor8(registers.bytes[A], registers.bytes[E], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_H:
        {
            Xor8(registers.bytes[A], registers.bytes[H], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_L:
        {
            Xor8(registers.bytes[A], registers.bytes[L], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_HL:
        {
            Xor8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]), registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case XOR_d8:
        {
            Xor8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // Compare the register with A and store the result in the flags
        case CP_A:
        {
            Cp8(registers.bytes[A], registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case CP_B:
        {
            Cp8(registers.bytes[A], registers.bytes[B]);
            mCycles += 1;
            break;
        }
        case CP_C:
        {
            Cp8(registers.bytes[A], registers.bytes[C]);
            mCycles += 1;
            break;
        }
        case CP_D:
        {
            Cp8(registers.bytes[A], registers.bytes[D]);
            mCycles += 1;
            break;
        }
        case CP_E:
        {
            Cp8(registers.bytes[A], registers.bytes[E]);
            mCycles += 1;
            break;
        }
        case CP_H:
        {
            Cp8(registers.bytes[A], registers.bytes[H]);
            mCycles += 1;
            break;
        }
        case CP_L:
        {
            Cp8(registers.bytes[A], registers.bytes[L]);
            mCycles += 1;
            break;
        }
        case CP_HL:
        {
            Cp8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[HL]));
            mCycles += 1;
            break;
        }
        case CP_d8:
        {
            Cp8(registers.bytes[A], mmu->ReadFromAddress(registers.shorts[PC] + 1));
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // Increment each register  
        case INC_A:
        {
            Inc8(registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case INC_B:
        {
            Inc8(registers.bytes[B]);
            mCycles += 1;
            break;
        }
        case INC_C:
        {
            Inc8(registers.bytes[C]);
            mCycles += 1;
            break;
        }
        case INC_D:
        {
            Inc8(registers.bytes[D]);
            mCycles += 1;
            break;
        }
        case INC_E:
        {
            Inc8(registers.bytes[E]);
            mCycles += 1;
            break;
        }
        case INC_L:
        {
            Inc8(registers.bytes[L]);
            mCycles += 1;
            break;
        }
        case INC_H:
        {
            Inc8(registers.bytes[H]);
            mCycles += 1;
            break;
        }
        case INC_HL_v:
        {
            // This increments the value pointed to by HL
            uint8_t val = mmu->ReadFromAddress(registers.shorts[HL]);
            Inc8(val);
            mmu->WriteToAddress(registers.shorts[HL], val);
            mCycles += 2;
            break;
        }

        // Decrement each register
        case DEC_A:
        {
            Dec8(registers.bytes[A]);
            mCycles += 1;
            break;
        }
        case DEC_B:
        {
            Dec8(registers.bytes[B]);
            mCycles += 1;
            break;
        }
        case DEC_C:
        {
            Dec8(registers.bytes[C]);
            mCycles += 1;
            break;
        }
        case DEC_D:
        {
            Dec8(registers.bytes[D]);
            mCycles += 1;
            break;
        }
        case DEC_E:
        {
            Dec8(registers.bytes[E]);
            mCycles += 1;
            break;
        }
        case DEC_L:
        {
            Dec8(registers.bytes[L]);
            mCycles += 1;
            break;
        }
        case DEC_H:
        {
            Dec8(registers.bytes[H]);
            mCycles += 1;
            break;
        }
        case DEC_HL_v:
        {
            uint8_t val = mmu->ReadFromAddress(registers.shorts[HL]);
            Dec8(val);
            mmu->WriteToAddress(registers.shorts[HL], val);
            mCycles += 2;
            break;
        }

        // 8-bit rotate/shift instructions
        case ROTATE_LEFT_A:
        {
            // capture the MSB
            bool msb = (registers.bytes[A] & 0x80) != 0;
            // shift the A register left by one bit
            registers.bytes[A] <<= 1;
            // set the LSB to the value of the carry flag
            registers.bytes[A] |= (registers.bytes[F] & CarryFlag) ? 0x01 : 0x00;
            // store the MSB in the carry flag
            registers.bytes[F] &= msb? CarryFlag : 0x00;

            // Set all other flags to 0
            registers.bytes[F] &= ~(ZeroFlag | AddSubFlag | HalfCarryFlag);
            mCycles += 1;
            break;
        }
        case ROTATE_LEFT_CA:
        {
            // Capture the MSB
            bool msb = (registers.bytes[A] & 0x80) != 0;
            // Shift the A register left by one bit
            registers.bytes[A] <<= 1;
            // Set the LSB to the value of the MSB
            registers.bytes[A] |= msb ? 0x01 : 0x00;
            // Store the MSB in the carry flag
            registers.bytes[F] &= msb ? CarryFlag : 0x00;

            // Set all other flags to 0
            registers.bytes[F] &= ~(ZeroFlag | AddSubFlag | HalfCarryFlag);
            mCycles += 1;
            break;
        }
        case ROTATE_RIGHT_A:
        {
            // Capture the LSB
            bool lsb = (registers.bytes[A] & 0x01) != 0;
            // Shift the A register right by one bit
            registers.bytes[A] >>= 1;
            // Set the MSB to the value of the carry flag
            registers.bytes[A] |= (registers.bytes[F] & CarryFlag) ? 0x80 : 0x00;
            // Store the LSB in the carry flag
            registers.bytes[F] &= lsb ? CarryFlag : 0x00;

            // Set all other flags to 0
            registers.bytes[F] &= ~(ZeroFlag | AddSubFlag | HalfCarryFlag);
            mCycles += 1;
            break;
        }
        case ROTATE_RIGHT_CA:
        {
            // Capture the LSB
            bool lsb = (registers.bytes[A] & 0x01) != 0;
            // Shift the A register right by one bit
            registers.bytes[A] >>= 1;
            // Set the MSB to the value of the LSB
            registers.bytes[A] |= lsb ? 0x80 : 0x00;
            // Store the LSB in the carry flag
            registers.bytes[F] &= lsb ? CarryFlag : 0x00;

            // Set all other flags to 0
            registers.bytes[F] &= ~(ZeroFlag | AddSubFlag | HalfCarryFlag);
            mCycles += 1;
            break;
        }

        /////////////////////////////
        // 8 bit loading

        // Load immediate value into each register
        case LOAD_A_d8:
        {
            registers.bytes[A] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_B_d8:
        {
            registers.bytes[B] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_C_d8:
        {
            registers.bytes[C] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_D_d8:
        {
            registers.bytes[D] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_E_d8:
        {
            registers.bytes[E] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_H_d8:
        {
            registers.bytes[H] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_L_d8:
        {
            registers.bytes[L] = mmu->ReadFromAddress(registers.shorts[PC] + 1);
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }
        case LOAD_HL_d8:
        {
            mmu->WriteToAddress(registers.shorts[HL], mmu->ReadFromAddress(registers.shorts[PC] + 1));
            registers.shorts[PC] ++;
            mCycles += 2;
            break;
        }

        // Load value from one register into A
        case LOAD_A_B:
        {
            registers.bytes[A] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_A_C:
        {
            registers.bytes[A] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_A_D:
        {
            registers.bytes[A] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_A_E:
        {
            registers.bytes[A] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_A_H:
        {
            registers.bytes[A] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_A_L:
        {
            registers.bytes[A] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_A_HL:
        {
            registers.bytes[A] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }
        case LOAD_A_A:
        {
            registers.bytes[A] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        
        // Load another register into B
        case LOAD_B_A:
        {
            registers.bytes[B] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        case LOAD_B_B:
        {
            registers.bytes[B] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_B_C:
        {
            registers.bytes[B] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_B_D:
        {
            registers.bytes[B] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_B_E:
        {
            registers.bytes[B] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_B_H:
        {
            registers.bytes[B] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_B_L:
        {
            registers.bytes[B] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_B_HL:
        {
            registers.bytes[B] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        // Load another register into C
        case LOAD_C_A:
        {
            registers.bytes[C] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        case LOAD_C_B:
        {
            registers.bytes[C] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_C_C:
        {
            registers.bytes[C] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_C_D:
        {
            registers.bytes[C] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_C_E:
        {
            registers.bytes[C] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_C_H:
        {
            registers.bytes[C] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_C_L:
        {
            registers.bytes[C] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_C_HL:
        {
            registers.bytes[C] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        // Load another register into D
        case LOAD_D_A:
        {
            registers.bytes[D] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        case LOAD_D_B:
        {
            registers.bytes[D] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_D_C:
        {
            registers.bytes[D] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_D_D:
        {
            registers.bytes[D] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_D_E:
        {
            registers.bytes[D] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_D_H:
        {
            registers.bytes[D] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_D_L:
        {
            registers.bytes[D] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_D_HL:
        {
            registers.bytes[D] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        // Load another register into E
        case LOAD_E_A:
        {
            registers.bytes[E] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        case LOAD_E_B:
        {
            registers.bytes[E] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_E_C:
        {
            registers.bytes[E] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_E_D:
        {
            registers.bytes[E] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_E_E:
        {
            registers.bytes[E] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_E_H:
        {
            registers.bytes[E] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_E_L:
        {
            registers.bytes[E] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_E_HL:
        {
            registers.bytes[E] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        // Load another register into H
        case LOAD_H_A:
        {
            registers.bytes[H] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        case LOAD_H_B:
        {
            registers.bytes[H] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_H_C:
        {
            registers.bytes[H] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_H_D:
        {
            registers.bytes[H] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_H_E:
        {
            registers.bytes[H] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_H_H:
        {
            registers.bytes[H] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_H_L:
        {
            registers.bytes[H] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_H_HL:
        {
            registers.bytes[H] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        // Load another register into L
        case LOAD_L_A:
        {
            registers.bytes[L] = registers.bytes[A];
            mCycles += 1;
            break;
        }
        case LOAD_L_B:
        {
            registers.bytes[L] = registers.bytes[B];
            mCycles += 1;
            break;
        }
        case LOAD_L_C:
        {
            registers.bytes[L] = registers.bytes[C];
            mCycles += 1;
            break;
        }
        case LOAD_L_D:
        {
            registers.bytes[L] = registers.bytes[D];
            mCycles += 1;
            break;
        }
        case LOAD_L_E:
        {
            registers.bytes[L] = registers.bytes[E];
            mCycles += 1;
            break;
        }
        case LOAD_L_H:
        {
            registers.bytes[L] = registers.bytes[H];
            mCycles += 1;
            break;
        }
        case LOAD_L_L:
        {
            registers.bytes[L] = registers.bytes[L];
            mCycles += 1;
            break;
        }
        case LOAD_L_HL:
        {
            registers.bytes[L] = mmu->ReadFromAddress(registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        // Load the value of a register into the memory address pointed to by HL
        case LOAD_HL_A:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[A]);
            mCycles += 2;
            break;
        }
        case LOAD_HL_B:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[B]);
            mCycles += 2;
            break;
        }
        case LOAD_HL_C:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[C]);
            mCycles += 2;
            break;
        }
        case LOAD_HL_D:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[D]);
            mCycles += 2;
            break;
        }
        case LOAD_HL_E:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[E]);
            mCycles += 2;
            break;
        }
        case LOAD_HL_H:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[H]);
            mCycles += 2;
            break;
        }
        case LOAD_HL_L:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[L]);
            mCycles += 2;
            break;
        }

        // 8 bit special load instructions

        // load A into BC or DE, and vice versa
        case LOAD_A_BC:
        {
            registers.bytes[A] = mmu->ReadFromAddress(registers.shorts[BC]);
            mCycles += 2;
            break;
        }
        case LOAD_A_DE:
        {
            registers.bytes[A] = mmu->ReadFromAddress(registers.shorts[DE]);
            mCycles += 2;
            break;
        }
        case LOAD_BC_A:
        {
            mmu->WriteToAddress(registers.shorts[BC], registers.bytes[A]);
            mCycles += 2;
            break;
        }
        case LOAD_DE_A:
        {
            mmu->WriteToAddress(registers.shorts[DE], registers.bytes[A]);
            mCycles += 2;
            break;
        }

        // Load and store with offset address
        case LOAD_Cv_A:
        {
            mmu->WriteToAddress(MMU::ioRegistersOffset + registers.bytes[C], registers.bytes[A]);
            mCycles += 2;
            break;
        }
        case LOAD_A_Cv:
        {
            registers.bytes[A] = mmu->ReadFromAddress(MMU::ioRegistersOffset + registers.bytes[C]);
            mCycles += 2;
            break;
        }

        // Load and store to absolute address
        case LOAD_a16_A:
        {
            uint16_t address = mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8);
            registers.shorts[PC] += 2;
            mmu->WriteToAddress(address, registers.bytes[A]);
            mCycles += 4;
            break;
        }
        case LOAD_A_a16:
        {
            uint16_t address = mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8);
            registers.shorts[PC] += 2;
            registers.bytes[A] = mmu->ReadFromAddress(address);
            mCycles += 4;
            break;
        }

        // Load and store from HL pointer to A, incrementing or decrementing the pointer after the fact
        case LOAD_A_HLplus:
        {
            registers.bytes[A] = mmu->ReadFromAddress(registers.shorts[HL]);
            registers.shorts[HL] ++;
            mCycles += 2;
            break;
        }
        case LOAD_A_HLminus:
        {
            registers.bytes[A] = mmu->ReadFromAddress(registers.shorts[HL]);
            registers.shorts[HL] --;
            mCycles += 2;
            break;
        }
        case LOAD_HLplus_A:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[A]);
            registers.shorts[HL] ++;
            mCycles += 2;
            break;
        }
        case LOAD_HLminus_A:
        {
            mmu->WriteToAddress(registers.shorts[HL], registers.bytes[A]);
            registers.shorts[HL] --;
            mCycles += 2;
            break;
        }

        // Load and store from absolute 8 bit address offset by 0xFF00
        case LOAD_H_a8_A:
        {
            mmu->WriteToAddress(MMU::ioRegistersOffset + mmu->ReadFromAddress(registers.shorts[PC] + 1), registers.bytes[A]);
            registers.shorts[PC] ++;
            mCycles += 3;
            break;
        }
        case LOAD_H_A_a8:
        {
            registers.bytes[A] = mmu->ReadFromAddress(MMU::ioRegistersOffset + mmu->ReadFromAddress(registers.shorts[PC] + 1));
            registers.shorts[PC] ++;
            mCycles += 3;
            break;
        }

        /////////////////////////////////
        // 16 bit arithmetic instructions
        case INC_BC:
        {
            registers.shorts[BC] ++;
            mCycles += 2;
            break;
        }
        case INC_DE:
        {
            registers.shorts[DE] ++;
            mCycles += 2;
            break;
        }
        case INC_HL:
        {
            registers.shorts[HL] ++;
            mCycles += 2;
            break;
        }
        case INC_SP:
        {
            registers.shorts[SP] ++;
            mCycles += 2;
            break;
        }

        case DEC_BC:
        {
            registers.shorts[BC] --;
            mCycles += 2;
            break;
        }
        case DEC_DE:
        {
            registers.shorts[DE] --;
            mCycles += 2;
            break;
        }
        case DEC_HL:
        {
            registers.shorts[HL] --;
            mCycles += 2;
            break;
        }
        case DEC_SP:
        {
            registers.shorts[SP] --;
            mCycles += 2;
            break;
        }

        case ADD_HL_BC:
        {
            Add16(registers.shorts[HL], registers.shorts[BC], registers.shorts[HL]);
            mCycles += 2;
            break;
        }
        case ADD_HL_DE:
        {
            Add16(registers.shorts[HL], registers.shorts[DE], registers.shorts[HL]);
            mCycles += 2;
            break;
        }
        case ADD_HL_HL:
        {
            Add16(registers.shorts[HL], registers.shorts[HL], registers.shorts[HL]);
            mCycles += 2;
            break;
        }
        case ADD_HL_SP:
        {
            Add16(registers.shorts[HL], registers.shorts[SP], registers.shorts[HL]);
            mCycles += 2;
            break;
        }

        /////////////////////////////////
        // 16 bit load instructions

        // Load 16 bit immediate value into register
        case LOAD_BC_d16:
        {
            registers.shorts[BC] = mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8);
            registers.shorts[PC] += 2;
            mCycles += 3;
            break;
        }
        case LOAD_DE_d16:
        {
            registers.shorts[DE] = mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8);
            registers.shorts[PC] += 2;
            mCycles += 3;
            break;
        }
        case LOAD_HL_d16:
        {
            registers.shorts[HL] = mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8);
            registers.shorts[PC] += 2;
            mCycles += 3;
            break;
        }

        /////////////////////////
        // Stack instructions
        // which are part of 16 bit load instructions

        // Load an immediate into Stack pointer
        case LOAD_SP_d16:
        {
            registers.shorts[SP] = mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8);
            registers.shorts[PC] += 2;
            mCycles += 3;
            break;
        }

        // Load the stack pointer into the data pointed to by given address
        case LOAD_a16_SP:
        {
            mmu->WriteToAddress(mmu->ReadFromAddress(registers.shorts[PC] + 1) | (mmu->ReadFromAddress(registers.shorts[PC] + 2) << 8), registers.shorts[SP]);
            registers.shorts[PC] += 2;
            mCycles += 5;
            break;
        }

        // Load HL into the Stack pointer
        case LOAD_SP_HL:
        {
            registers.shorts[SP] = registers.shorts[HL];
            mCycles += 2;
            break;
        }

        // Add immediate value to SP and store in HL
        case LOAD_HL_SP_r8:
        {
            uint16_t sp = registers.shorts[SP];
            uint16_t hl = registers.shorts[HL];
            uint8_t r = mmu->ReadFromAddress(registers.shorts[PC] + 1);

            // check half carry flag:
            if ((((sp & 0xf) + (r & 0xf)) & 0x10) == 0x10)
            {
                registers.bytes[F] |= HalfCarryFlag;
            }

            // check full carry flag
            hl = sp + (uint16_t)r;

            if (hl & 0x0100)
            {
                registers.bytes[F] |= CarryFlag;
            }
            registers.shorts[HL] = hl;

            // reset 0 flag??
            registers.bytes[F] &= ~ZeroFlag;

            // Reset subtraction flag
            registers.bytes[F] &= ~AddSubFlag;
            
            registers.shorts[PC] ++;
            mCycles += 3;
            break;
        }

        // Push 16 bit register onto stack
        case PUSH_BC:
        {
            mmu->WriteToAddress(registers.shorts[SP], registers.bytes[B]);
            mmu->WriteToAddress(--registers.shorts[SP], registers.bytes[C]);
            registers.shorts[SP] --;
            mCycles += 4;
            break;
        }
        case PUSH_DE:
        {
            mmu->WriteToAddress(registers.shorts[SP], registers.bytes[D]);
            mmu->WriteToAddress(--registers.shorts[SP], registers.bytes[E]);
            registers.shorts[SP] --;
            mCycles += 4;
            break;
        }
        case PUSH_HL:
        {
            mmu->WriteToAddress(registers.shorts[SP], registers.bytes[H]);
            mmu->WriteToAddress(registers.shorts[SP] - 1, registers.bytes[L]);
            registers.shorts[SP] -= 2;
            mCycles += 4;
            break;
        }
        case PUSH_AF:
        {
            mmu->WriteToAddress(registers.shorts[SP], registers.bytes[A]);
            mmu->WriteToAddress(registers.shorts[SP] - 1, registers.bytes[F]);
            registers.shorts[SP] -= 2;
            mCycles += 4;
            break;
        }
        
        // Pop 16 bit register from stack
        case POP_BC:
        {
            registers.bytes[C] = mmu->ReadFromAddress(++registers.shorts[SP]);
            registers.bytes[B] = mmu->ReadFromAddress(++registers.shorts[SP]);
            mCycles += 3;
            break;
        }
        case POP_DE:
        {
            registers.bytes[E] = mmu->ReadFromAddress(++registers.shorts[SP]);
            registers.bytes[D] = mmu->ReadFromAddress(++registers.shorts[SP] + 1);
            mCycles += 3;
            break;
        }
        case POP_HL:
        {
            registers.bytes[L] = mmu->ReadFromAddress(++registers.shorts[SP]);
            registers.bytes[H] = mmu->ReadFromAddress(++registers.shorts[SP]);
            mCycles += 3;
            break;
        }
        case POP_AF:
        {
            registers.bytes[F] = mmu->ReadFromAddress(++registers.shorts[SP]);
            registers.bytes[A] = mmu->ReadFromAddress(++registers.shorts[SP]);
            mCycles += 3;
            break;
        }

        /////////////////////////////////
        // Prefix'd instructions
        case PREFIX:
        {
            // carry out instruction in following byte
            registers.shorts[PC] ++;
            instruction = mmu->ReadFromAddress(registers.shorts[PC]);
            mCycles += ExecutePrefixInstruction(instruction);
            break;
        }

        // Jumps / calls
        default:
            printf("instruction: %x  at PC: %d\n", instruction, oldPC);
            std::cout << "Bad opcode!" << std::endl;
            //exit(-1);
    }

    printf("instruction: %x  at PC: %d  %x\n", instruction, oldPC, oldPC);

    // Print all registers for debug
    printf("A: %x\tB: %x\tC: %x\tD: %x\nE: %x\tH: %x\tL: %x\tF: %x\n",
            registers.bytes[A], registers.bytes[B], registers.bytes[C], registers.bytes[D], 
            registers.bytes[E], registers.bytes[H], registers.bytes[L], 
            registers.bytes[F]);
    printf("AF: %x\tBC: %x\tDE: %x\tHL: %x\nSP: %x\tPC: %x\n",
            registers.shorts[AF], registers.shorts[BC], registers.shorts[DE], registers.shorts[HL], 
            registers.shorts[SP], registers.shorts[PC]);

    // Update PC if a jump instruction has not been executed
    if (!pcChanged)
    {
        // TODO when this goes above ROM banks, need to manually switch out the next ROM bank and
        // reset the PC to the start of the next ROM bank
        registers.shorts[PC] ++;
    }

    return mCycles;
}

int CPU::ExecutePrefixInstruction(uint8_t instruction)
{
    int mCycles = 2;

    // First two bits tell us which operation: rot, test bit, clear bit, set bit
    // Second three bits tell us which bit/which operation
    // third three bits tell us which register to operate on
    uint8_t operation = instruction & PrefixOperationMask;
    uint8_t bitNumber = instruction & PrefixBitMask;
    uint8_t rotOperation = instruction & PrefixRotOperationMask;
    uint8_t reg = instruction & PrefixRegisterMask;

    switch (operation)
    {
        case PrefixRot:
        {
            // check rotOperation
            mCycles += PerformPrefixedRotOperation(rotOperation, GetRegisterFromPrefixIndex(reg), reg == RegisterThatsActuallyMemory);
            break;
        }
        case PrefixBit:
        {
            // test bit bitNumber in register reg
            if (reg == RegisterThatsActuallyMemory)
            {
                uint8_t val = mmu->ReadFromAddress(registers.shorts[HL]);
                registers.bytes[F] |= val & (0x1 << bitNumber)? ZeroFlag : 0x00;
                registers.bytes[F] &= ~AddSubFlag;
                registers.bytes[F] |= HalfCarryFlag;
                mCycles += 1;
            }
            else
            {
                uint8_t regIndex = GetRegisterFromPrefixIndex(reg);
                registers.bytes[F] |= registers.bytes[regIndex] & (0x1 << bitNumber)? ZeroFlag : 0x00;
                registers.bytes[F] &= ~AddSubFlag;
                registers.bytes[F] |= HalfCarryFlag;
            }
            break;
        }
        case PrefixRes:
        {
            // clear bit bitNumber in register reg
            if (reg == RegisterThatsActuallyMemory)
            {
                uint8_t val = mmu->ReadFromAddress(registers.shorts[HL]);
                val &= ~(0x01 << bitNumber);
                mmu->WriteToAddress(registers.shorts[HL], (uint8_t)val);
                mCycles += 2;
            }
            else
            {
                uint8_t regIndex = GetRegisterFromPrefixIndex(reg);
                registers.bytes[regIndex] &= ~(0x01 << bitNumber);
            }
            break;
        }
        case PrefixSet:
        {
            // set bit bitNumber in register reg
            if (reg == RegisterThatsActuallyMemory)
            {
                uint8_t val = mmu->ReadFromAddress(registers.shorts[HL]);
                val |= 0x01 << bitNumber;
                mmu->WriteToAddress(registers.shorts[HL], (uint8_t)val);
                mCycles += 2;
            }
            else
            {
                uint8_t regIndex = GetRegisterFromPrefixIndex(reg);
                registers.bytes[regIndex] |= 0x01 << bitNumber;
            }
            break;
        }
    }

    return mCycles; 
}

uint8_t CPU::GetRegisterFromPrefixIndex(uint8_t reg)
{
    switch (reg)
    {
        case 0: return B; break;
        case 1: return C; break;
        case 2: return D; break;
        case 3: return E; break;
        case 4: return H; break;
        case 5: return L; break;
        case 7: return A; break;
        default: return 0;
    }
}

int CPU::PerformPrefixedRotOperation(uint8_t operation, uint8_t regIndex, bool useHL)
{
    int extra_mCycles = 0;
     uint8_t val = 0;

    switch (operation)
    {
        case PrefixRLC:
        {
            // TODO is this arithmetic or logical shift?
            // Rotate left through carry
            bool setCarry = false;
            uint8_t carryBit = registers.bytes[F] & CarryFlag? 1 : 0;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x80;
                val = (val << 1) | carryBit;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x80;
                val = (val << 1) | carryBit;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
        case PrefixRRC:
        {
            // Rotate right through carry
            bool setCarry = false;
            uint8_t carryBit = registers.bytes[F] & CarryFlag? 0x80 : 0;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x80;
                val = (val >> 1) | carryBit;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x80;
                val = (val >> 1) | carryBit;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
        case PrefixRL:
        {
            // Rotate left, but not through carry. MSB goes to LSB
            bool setCarry = false;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x80;
                uint8_t msb = val & 0x80? 1 : 0;
                val = (val << 1) | msb;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x80;
                uint8_t msb = val & 0x80? 1 : 0;
                val = (val << 1) | msb;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
        case PrefixRR:
        {
            // Rotate left, but not through carry. LSB goes to MSB
            bool setCarry = false;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x80;
                uint8_t lsb = val & 0x01? 0x80 : 0;
                val = (val >> 1) | lsb;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x80;
                uint8_t lsb = val & 0x01? 0x80 : 0;
                val = (val >> 1) | lsb;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
        case PrefixSLA:
        {
            // Shift left logical. Carry bit comes from msb
            bool setCarry = false;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x80;
                val = val << 1;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x80;
                val = val << 1;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
        case PrefixSRA:
        {
            // Shift right arithmetic. Carry bit comes from lsb, msb doesn't change
            bool setCarry = false;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x01;
                uint8_t msb = val & 0x80;
                val = (val >> 1) | msb;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x01;
                uint8_t msb = val & 0x80;
                val = (val >> 1) | msb;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
        case PrefixSWAP:
        {
            // exchange low and high nibble
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                uint8_t highNibble = val & 0xF0;
                uint8_t lowNibble = val & 0x0F;
                val = (highNibble >> 4) | (lowNibble << 4);
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                uint8_t highNibble = val & 0xF0;
                uint8_t lowNibble = val & 0x0F;
                registers.bytes[regIndex] = (highNibble >> 4) | (lowNibble << 4);
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~AddSubFlag;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~CarryFlag;
            break;
        }
        case PrefixSRL:
        {
            // Shift right logical. Carry bit comes from lsb
            bool setCarry = false;
            if (useHL)
            {
                val = mmu->ReadFromAddress(registers.shorts[HL]);
                setCarry = val & 0x01;
                val = val >> 1;
                mmu->WriteToAddress(registers.shorts[HL], val);
                extra_mCycles = 2;
            }
            else
            {
                val = registers.bytes[regIndex];
                setCarry = val & 0x01;
                val = val >> 1;
                registers.bytes[regIndex] = val;
            }

            // Zero subtract, half carry, and carry flags
            registers.bytes[F] |= setCarry? CarryFlag : 0;
            registers.bytes[F] |= val == 0? ZeroFlag : 0;
            registers.bytes[F] &= ~HalfCarryFlag;
            registers.bytes[F] &= ~AddSubFlag;
            break;
        }
    }

    return extra_mCycles; 
}