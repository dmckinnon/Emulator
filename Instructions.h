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
#define JUMP_RETURN_NZ_r8  0x20
#define JUMP_RETURN_NC_r8  0x30
#define JUMP_RETURN_r8     0x18
#define JUMP_RETURN_Z_r8   0x28
#define JUMP_RETURN_C_r8   0x38
#define JUMP_NZ_a16        0xC2
#define JUMP_NC_a16        0xD2
#define JUMP_a16           0xC3
#define JUMP_HL            0xE9
#define JUMP_Z_a16         0xCA
#define JUMP_C_a16         0xDA
#define RETURN_NZ          0xC0
#define RETURN_NC          0xD0
#define RETURN_Z           0xC8
#define RETURN_C           0xE8
#define RETURN_I           0xD9
#define RETURN             0xC9
#define CALL_Z_a16         0xCC
#define CALL_C_a16         0xDC
#define CALL_a16           0xCD
#define CALL_NZ_a16        0xC4
#define CALL_NC_a16        0xD4
#define RESET_00H          0xC7
#define RESET_10H          0xD7
#define RESET_20H          0xE7
#define RESET_30H          0xF7
#define RESET_08H          0xCF
#define RESET_18H          0xDF
#define RESET_28H          0xEF
#define RESET_38H          0xFF


// 8 bit load instructions

// 16 bit load instructions
#define LOAD_BC_d16   0x01 // TODO does this need the next instr?
#define LOAD_DE_d16   0x11
#define LOAD_HL_d16   0x21
#define LOAD_SP_d16   0x31
#define LOAD_a16_SP   0x08
#define POP_BC        0xC1
#define PUSH_BC       0xC5
#define POP_DE        0xD1
#define PUSH_DE       0xD5
#define POP_HL        0xE1
#define PUSH_HL       0xE5
#define POP_AF        0xF1
#define PUSH_AF       0xF5
#define LOAD_a16_SP   0x08
#define LOAD_HL_SP_r8 0xF8
#define LOAD_SP_HL    0xF9




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
#define ROTATE_LEFT_CA  0x07
#define ROTATE_LEFT_A   0x17
#define ROTATE_RIGHT_CA 0x0F
#define ROTATE_RIGHT_A  0x1F

