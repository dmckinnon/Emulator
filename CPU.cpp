#include "CPU.h"
#include "Instructions.h"

#include <iostream>
#include <string.h>

#define MAX_PROGRAM_LENGTH 32

using namespace std;

CPU::CPU()
{
    // The program counter just starts at this address. It just does. 
    registers.shorts[PC] = MMU::cartridgeHeaderOffset;
    mCycles = 0;
}

CPU::CPU(std::shared_ptr<MMU> mmu)
{
    this->mmu = mmu;
    registers.shorts[PC] = MMU::cartridgeHeaderOffset;
    mCycles = 0;
    
    // Set registers to 0
    memset(&registers, 0, 12);
}

CPU::~CPU()
{

}

void CPU::ExecuteCode(std::shared_ptr<Rom> rom)
{
    if (!mmu)
    {
        return;
    }

    int i = 0;

    // EI, DI, and RETI change interrupts on the cycle after
    bool enableInterruptsNextCycle = false;
    bool disableInterruptsNextCycle = false;
    
    while (true)
    {
        // If PC has reached 0x100, then cartridge is valid;
        // unmap system ROM and use cartridge ROM from now on.
        mmu->UnmapSystemRom();


        // Before executing an instruction, check if interrupts are pending:
        // Do not do this if interrupts were just enabled - that lets them happen on the next cycle
        if (interruptsAreEnabled)
        {

        }

        // EI, DI and RETI change interrupt enablement on the cycle after their instruction
        if (enableInterruptsNextCycle)
        {
            interruptsAreEnabled = true;
            enableInterruptsNextCycle = false;
        }
        else if (disableInterruptsNextCycle)
        {
            interruptsAreEnabled = false;
            disableInterruptsNextCycle = false;
        }

        // get instruction at program counter
        uint8_t instruction = rom->bytes[registers.shorts[PC]];

        printf("instruction: %x\n", instruction);

        // execute instruction
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
                return;
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

            // 16 bit arithmetic
            case INC_BC:
            {
                registers.shorts[BC] ++;
                break;
            }
            case INC_DE:
            {
                registers.shorts[DE] ++;
                break;
            }
            case INC_HL:
            {
                registers.shorts[HL] ++;
                break;
            }
            case INC_SP:
            {
                registers.shorts[SP] ++;
                break;
            }

            case DEC_BC:
            {
                registers.shorts[BC] --;
                break;
            }
            case DEC_DE:
            {
                registers.shorts[DE] --;
                break;
            }
            case DEC_HL:
            {
                registers.shorts[HL] --;
                break;
            }
            case DEC_SP:
            {
                registers.shorts[SP] --;
                break;
            }

            case ADD_HL_BC:
            {
                registers.shorts[HL] += registers.shorts[BC];
                // Set flags H, C, reset N
                registers.bytes[F] ^= AddSubFlag;
                // how to read half carry and carry from OS?
                break;
            }


            /////////////////////////////
            // 8 bit arithmetic and logic

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
                Add8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case ADD_A_d8:
            {
                Add8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                Sub8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case SUB_d8:
            {
                Sub8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                AddC8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case ADC_d8:
            {
                AddC8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                SubC8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case SBC_d8:
            {
                SubC8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                And8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case AND_d8:
            {
                And8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                Or8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case OR_d8:
            {
                Or8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                Xor8(registers.bytes[A], sram[registers.shorts[HL]], registers.bytes[A]);
                mCycles += 1;
                break;
            }
            case XOR_d8:
            {
                Xor8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1], registers.bytes[A]);
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
                Cp8(registers.bytes[A], sram[registers.shorts[HL]]);
                mCycles += 1;
                break;
            }
            case CP_d8:
            {
                Cp8(registers.bytes[A], rom->bytes[registers.shorts[PC] + 1]);
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
                Inc8(sram[registers.shorts[HL]]);
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
                // This decrements the value pointed to by HL
                Dec8(sram[registers.shorts[HL]]);
                mCycles += 2;
                break;
            }

            /////////////////////////////
            // 8 bit loading

            // Load immediate value into each register
            case LOAD_A_d8:
            {
                registers.bytes[A] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_B_d8:
            {
                registers.bytes[B] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_C_d8:
            {
                registers.bytes[C] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_D_d8:
            {
                registers.bytes[D] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_E_d8:
            {
                registers.bytes[E] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_H_d8:
            {
                registers.bytes[H] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_L_d8:
            {
                registers.bytes[L] = rom->bytes[registers.shorts[PC] + 1];
                registers.shorts[PC] ++;
                mCycles += 2;
                break;
            }
            case LOAD_HL_d8:
            {
                sram[registers.shorts[HL]] = rom->bytes[registers.shorts[PC] + 1];
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
                registers.bytes[A] = sram[registers.shorts[HL]];
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
                registers.bytes[B] = sram[registers.shorts[HL]];
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
                registers.bytes[C] = sram[registers.shorts[HL]];
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
                registers.bytes[D] = sram[registers.shorts[HL]];
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
                registers.bytes[E] = sram[registers.shorts[HL]];
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
                registers.bytes[H] = sram[registers.shorts[HL]];
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
                registers.bytes[L] = sram[registers.shorts[HL]];
                mCycles += 2;
                break;
            }

            // Load the value of a register into the memory address pointed to by HL
            case LOAD_HL_A:
            {
                sram[registers.shorts[HL]] = registers.bytes[A];
                mCycles += 2;
                break;
            }
            case LOAD_HL_B:
            {
                sram[registers.shorts[HL]] = registers.bytes[B];
                mCycles += 2;
                break;
            }
            case LOAD_HL_C:
            {
                sram[registers.shorts[HL]] = registers.bytes[C];
                mCycles += 2;
                break;
            }
            case LOAD_HL_D:
            {
                sram[registers.shorts[HL]] = registers.bytes[D];
                mCycles += 2;
                break;
            }
            case LOAD_HL_E:
            {
                sram[registers.shorts[HL]] = registers.bytes[E];
                mCycles += 2;
                break;
            }
            case LOAD_HL_H:
            {
                sram[registers.shorts[HL]] = registers.bytes[H];
                mCycles += 2;
                break;
            }
            case LOAD_HL_L:
            {
                sram[registers.shorts[HL]] = registers.bytes[L];
                mCycles += 2;
                break;
            }

            // 8 bit special load instructions

            // load A into BC or DE, and vice versa
            case LOAD_A_BC:
            {
                registers.bytes[A] = sram[registers.shorts[BC]];
                mCycles += 2;
                break;
            }
            case LOAD_A_DE:
            {
                registers.bytes[A] = sram[registers.shorts[DE]];
                mCycles += 2;
                break;
            }
            case LOAD_BC_A:
            {
                sram[registers.shorts[BC]] = registers.bytes[A];
                mCycles += 2;
                break;
            }
            case LOAD_DE_A:
            {
                sram[registers.shorts[DE]] = registers.bytes[A];
                mCycles += 2;
                break;
            }

            // Load and store with offset address
            case LOAD_Cv_A:
            {
                sram[0xFF00 + registers.bytes[C]] = registers.bytes[A];
                mCycles += 2;
                break;
            }
            case LOAD_A_Cv:
            {
                registers.bytes[A] = sram[0xFF00 + registers.bytes[C]];
                mCycles += 2;
                break;
            }

            // Load and store to absolute address
            case LOAD_a16_A:
            {
                uint16_t address = rom->bytes[registers.shorts[PC] + 1] | (rom->bytes[registers.shorts[PC] + 2] << 8);
                registers.shorts[PC] += 2;
                sram[address] = registers.bytes[A];
                mCycles += 4;
                break;
            }
            case LOAD_A_a16:
            {
                uint16_t address = rom->bytes[registers.shorts[PC] + 1] | (rom->bytes[registers.shorts[PC] + 2] << 8);
                registers.shorts[PC] += 2;
                registers.bytes[A] = sram[address];
                mCycles += 4;
                break;
            }

            // Load and store from HL pointer to A, incrementing or decrementing the pointer after the fact
            case LOAD_A_HLplus:
            {
                registers.bytes[A] = sram[registers.shorts[HL]];
                registers.shorts[HL] ++;
                mCycles += 2;
                break;
            }
            case LOAD_A_HLminus:
            {
                registers.bytes[A] = sram[registers.shorts[HL]];
                registers.shorts[HL] --;
                mCycles += 2;
                break;
            }
            case LOAD_HLplus_A:
            {
                sram[registers.shorts[HL]] = registers.bytes[A];
                registers.shorts[HL] ++;
                mCycles += 2;
                break;
            }
            case LOAD_HLminus_A:
            {
                sram[registers.shorts[HL]] = registers.bytes[A];
                registers.shorts[HL] --;
                mCycles += 2;
                break;
            }

            // Load and store from absolute 8 bit address offset by 0xFF00
            case LOAD_H_a8_A:
            {
                sram[0xFF00 + rom->bytes[registers.shorts[PC] + 1]] = registers.bytes[A];
                registers.shorts[PC] ++;
                mCycles += 3;
                break;
            }
            case LOAD_H_A_a8:
            {
                registers.bytes[A] = sram[0xFF00 + rom->bytes[registers.shorts[PC] + 1]];
                registers.shorts[PC] ++;
                mCycles += 3;
                break;
            }

            // Before doing any more instructions, test the load and store ones
            // you have done up until now

            // Jumps / calls
            default:
                std::cout << "Bad opcode!" << std::endl;
                //exit(-1);
        }

        // Print all registers for debug
        printf("A: %x\tB: %x\tC: %x\tD: %x\nE: %x\tH: %x\tL: %x\nS: %x\tP: %x\nF: %x\n",
                registers.bytes[A], registers.bytes[B], registers.bytes[C], registers.bytes[D], 
                registers.bytes[E], registers.bytes[H], registers.bytes[L], 
                registers.bytes[S], registers.bytes[P], 
                registers.bytes[F]);

        // Loop over first 8 bytes of memory and print them
        for (int i = 0; i < 8; i++)
        {
            printf("%x ", sram[i]);
        }
        printf("\n");

        registers.shorts[PC] ++;

        // KEEP PROGRAM SMALL
        if (registers.shorts[PC] >= MAX_PROGRAM_LENGTH)
        {
            break;
        }
    }

    
}

inline void CPU::Add8(byte A, byte B, byte& C)
{
    // check half carry flag:
    if ((((A & 0xf) + (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A + (uint16_t)B;
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }

    // check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // Reset subtraction flag
    registers.bytes[F] ^= AddSubFlag;
}

inline void CPU::Inc8(byte& A)
{
    // check half carry flag
    if (((A & 0xf) + 1 & 0xF0) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    A = A + 1;

    // check zero:
    if (A == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // reset the subtract flag
    registers.bytes[F] ^= AddSubFlag;
}

inline void CPU::Dec8(byte& A)
{
    // check half carry flag
    if (((A & 0xf) - 1 & 0xF0) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    A = A - 1;

    // check zero:
    if (A == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // Activate the subtract flag
    registers.bytes[F] &= AddSubFlag;
}

inline void CPU::Sub8(byte A, byte B, byte& C)
{
    // check half carry flag:
    if ((((A & 0xf) - (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A - (uint16_t)B;
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }

    // Check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // activate subtraction flag
    registers.bytes[F] |= AddSubFlag;
}

inline void CPU::AddC8(byte A, byte B, byte& C)
{
    // check half carry flag:
    if ((((A & 0xf) + (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A + (uint16_t)B + (uint16_t)(registers.bytes[F] & CarryFlag >> 4);
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }

    // check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // Reset subtraction flag
    registers.bytes[F] ^= AddSubFlag;
}

inline void CPU::SubC8(byte A, byte B, byte& C)
{
    // check half carry flag:
    if ((((A & 0xf) - (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A - (uint16_t)B - (uint16_t)(registers.bytes[F] & CarryFlag >> 4);
    C = (uint8_t)r;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }

    // Check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // activate subtraction flag
    registers.bytes[F] |= AddSubFlag;
}

inline void CPU::And8(byte A, byte B, byte& C)
{
    C = A & B;

    // check zero:
    if (C == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // reset subtraction flag
    registers.bytes[F] ^= AddSubFlag;

    // reset carry flag
    registers.bytes[F] ^= CarryFlag;

    // set half carry flag
    registers.bytes[F] |= HalfCarryFlag;
}

inline void CPU::Or8(byte A, byte B, byte& C)
{
    C = A | B;

    // check zero:
    if (C == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // reset subtraction flag
    registers.bytes[F] ^= AddSubFlag;

    // reset carry flag
    registers.bytes[F] ^= CarryFlag;

    // reset half carry flag
    registers.bytes[F] ^= HalfCarryFlag;
}

inline void CPU::Xor8(byte A, byte B, byte& C)
{
    C = A ^ B;

    // check zero:
    if (C == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // reset subtraction flag
    registers.bytes[F] ^= AddSubFlag;

    // reset carry flag
    registers.bytes[F] ^= CarryFlag;

    // reset half carry flag
    registers.bytes[F] ^= HalfCarryFlag;
}

inline void CPU::Cp8(byte A, byte B)
{
    // check half carry flag:
    if ((((A & 0xf) - (B & 0xf)) & 0x10) == 0x10)
    {
        registers.bytes[F] |= HalfCarryFlag;
    }

    // check full carry flag
    uint16_t r = (uint16_t)A - (uint16_t)B;

    if (r & 0x0100)
    {
        registers.bytes[F] |= CarryFlag;
    }

    // Check zero:
    if (r == 0)
    {
        registers.bytes[F] |= ZeroFlag;
    }

    // activate subtraction flag
    registers.bytes[F] |= AddSubFlag;
}