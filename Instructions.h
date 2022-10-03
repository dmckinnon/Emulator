// This file defines all the opcodes and opcode masks for the SM38 instruction set
// These are specified in https://gbdev.io/gb-opcodes/optables/ and https://gekkio.fi/files/gb-docs/gbctr.pdf
// Since there are so few registers, we just enumerate all possibilities
// instead of trying to pull out the opcode and then the arguments

// Control instructions
#define NOP         0x00
#define STOP        0x10
#define DI          0xF3
#define EI          0xFB
#define PREFIX      0xCB

// Jumps / calls


// 8 bit load instructions

// 16 bit load instructions

// 8 bit arithmetic / logic



// 16 bit arithmetic / logic
#define INC_BC      0x03
#define INC_DE      0x13
#define INC_HL      0x23
#define INC_SP      0x33

#define DEC_BC      0xB3
#define DEC_DE      0xB3
#define DEC_HL      0xB3
#define DEC_SP      0xB3

#define ADD_HL_BC   0x09
#define ADD_HL_DE   0x19
#define ADD_HL_HL   0x29
#define ADD_HL_SP   0x39

// 8 bit shift / rotate and bit instructions