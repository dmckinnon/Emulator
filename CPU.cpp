#include "CPU.h"

#include <iostream>

#define OPCODE_MASK 0xE0

using namespace std;

CPU::CPU()
{
    registers.shorts[PC] = 0;
}

CPU::~CPU()
{

}

void CPU::ExecuteCode(std::shared_ptr<Rom> rom)
{
    int i = 0;
    
    while (true)
    {
        // execute instruction at program counter
        uint8_t instruction = rom->bytes[registers.shorts[PC]];

        // get opcode
        uint8_t opcode = (OPCODE_MASK & instruction) >> 5;

        printf("instruction: %x\n", instruction);
        //std::cout << std::hex << (int)opcode << std::endl;

        switch (opcode)
        {
            
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