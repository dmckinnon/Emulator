#include "CPU.h"
#include "Instructions.h"

#include <iostream>

#define OPCODE_MASK 0xE0

using namespace std;

CPU::CPU()
{
    registers.shorts[PC] = 0;
    mCycles = 0;
}

CPU::~CPU()
{

}

void CPU::ExecuteCode(std::shared_ptr<Rom> rom)
{
    int i = 0;
    
    while (true)
    {
        // get instruction at program counter
        uint8_t instruction = rom->bytes[registers.shorts[PC]];

        // get opcode
        uint8_t opcode = (OPCODE_MASK & instruction) >> 5;

        printf("instruction: %x\n", instruction);

        // execute instruction
        switch (opcode)
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

            }
            case DI:
            {

            }
            case EI:
            {
                
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
                if ()
                {


                }
                break;
            }
            case DEC_DE:
            {
                registers.shorts[DE] --;
                // TODO set flags
                break;
            }
            case DEC_HL:
            {
                registers.shorts[HL] --;
                // TODO set flags
                break;
            }
            case DEC_SP:
            {
                registers.shorts[SP] --;
                // TODO set flags
                break;
            }



            // Jumps / calls
            default:
                std::cout << "Bad opcode!" << std::endl;
                //exit(-1);
        }


        registers.shorts[PC] ++;

        if (registers.shorts[PC] >= 16)
        {
            break;
        }
    }
}