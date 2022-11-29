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
#define LOAD_BC_A      0x02
#define LOAD_DE_A      0x12
#define LOAD_A_BC      0x0A
#define LOAD_A_DE      0x1A
#define LOAD_C_A       0xE2
#define LOAD_A_C       0xF2
#define LOAD_a16_A     0xEA
#define LOAD_A_a16     0xFA
#define LOAD_A_HLplus  0x2A
#define LOAD_A_HLminus 0x3A
#define LOAD_HLplus_A  0x22
#define LOAD_HLminus_A 0x32
#define LOAD_H_a8_A    0xE0
#define LOAD_H_A_a8    0xF0

#define LOAD_A_d8      0x3e
#define LOAD_B_d8      0x06
#define LOAD_C_d8      0x0E
#define LOAD_D_d8      0x16
#define LOAD_E_d8      0x1E
#define LOAD_L_d8      0x2E
#define LOAD_H_d8      0x26
#define LOAD_HL_d8     0x36

#define LOAD_A_A   0x7F
#define LOAD_A_B   0x78
#define LOAD_A_C   0x79
#define LOAD_A_D   0x7A
#define LOAD_A_E   0x7B
#define LOAD_A_H   0x7C
#define LOAD_A_L   0x7D
#define LOAD_A_HL  0x7E

#define LOAD_B_A    0x47
#define LOAD_B_B    0x40
#define LOAD_B_C    0x41
#define LOAD_B_D    0x42
#define LOAD_B_E    0x43
#define LOAD_B_H    0x44
#define LOAD_B_L    0x45
#define LOAD_B_HL   0x46

#define LOAD_C_A    0x4F
#define LOAD_C_B    0x48
#define LOAD_C_C    0x49
#define LOAD_C_D    0x4A
#define LOAD_C_E    0x4B
#define LOAD_C_H    0x4C
#define LOAD_C_L    0x4D
#define LOAD_C_HL   0x4E

#define LOAD_D_A    0x57
#define LOAD_D_B    0x50
#define LOAD_D_C    0x51
#define LOAD_D_D    0x52
#define LOAD_D_E    0x53
#define LOAD_D_H    0x54
#define LOAD_D_L    0x55
#define LOAD_D_HL   0x56

#define LOAD_E_A    0x5F
#define LOAD_E_B    0x58
#define LOAD_E_C    0x59
#define LOAD_E_D    0x5A
#define LOAD_E_E    0x5B
#define LOAD_E_H    0x5C
#define LOAD_E_L    0x5D
#define LOAD_E_HL   0x5E

#define LOAD_H_A    0x67
#define LOAD_H_B    0x60
#define LOAD_H_C    0x61
#define LOAD_H_D    0x62
#define LOAD_H_E    0x63
#define LOAD_H_H    0x64
#define LOAD_H_L    0x65
#define LOAD_H_HL   0x66

#define LOAD_L_A    0x6F
#define LOAD_L_B    0x68
#define LOAD_L_C    0x69
#define LOAD_L_D    0x6A
#define LOAD_L_E    0x6B
#define LOAD_L_H    0x6C
#define LOAD_L_L    0x6D
#define LOAD_L_HL   0x6E

#define LOAD_HL_A    0x77
#define LOAD_HL_B    0x70
#define LOAD_HL_C    0x71
#define LOAD_HL_D    0x72
#define LOAD_HL_E    0x73
#define LOAD_HL_H    0x74
#define LOAD_HL_L    0x75

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
#define INC_A    0x3C
#define INC_B    0x04
#define INC_C    0x0C
#define INC_D    0x14
#define INC_E    0x1C
#define INC_L    0x2C
#define INC_H    0x34
#define INC_HL   0x04

#define DEC_A    0x3D
#define DEC_B    0x05
#define DEC_C    0x0D
#define DEC_D    0x15
#define DEC_E    0x1D
#define DEC_L    0x2D
#define DEC_H    0x25
#define DEC_HL   0x35

#define ADD_A_A    0x87
#define ADD_A_B    0x80
#define ADD_A_C    0x81
#define ADD_A_D    0x82
#define ADD_A_E    0x83
#define ADD_A_H    0x84
#define ADD_A_L    0x85
#define ADD_A_HL   0x86
#define ADD_A_d8   0xC6

#define ADC_A_A    0x8F
#define ADC_A_B    0x88
#define ADC_A_C    0x89
#define ADC_A_D    0x8A
#define ADC_A_E    0x8B
#define ADC_A_H    0x8C
#define ADC_A_L    0x8D
#define ADC_A_HL   0x8E
#define ADC_A_d8   0xCE

#define SUB_A     0x97
#define SUB_B     0x90
#define SUB_C     0x91
#define SUB_D     0x92
#define SUB_E     0x93
#define SUB_H     0x94
#define SUB_L     0x95
#define SUB_HL    0x96
#define SUB_d8    0xD6

#define OR_A      0xB7
#define OR_B      0xB0
#define OR_C      0xB1
#define OR_D      0xB2
#define OR_E      0xB3
#define OR_H      0xB4
#define OR_L      0xB5
#define OR_HL     0xB6
#define OR_d8     0xF6

#define AND_A     0xA7
#define AND_B     0xA0
#define AND_C     0xA1
#define AND_D     0xA2
#define AND_E     0xA3
#define AND_H     0xA4
#define AND_L     0xA5
#define AND_HL    0xA6
#define AND_d8    0xE6

#define CP_

#define XOR_

#define SBC_

#define CPL       0x2F
#define CCF       0x3F


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

// All the things prefixed by CB
// TODO 256 of the buggers