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