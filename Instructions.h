// This file defines all the opcodes and opcode masks for the SM38 instruction set
// These are specified in https://gbdev.io/gb-opcodes/optables/ and https://gekkio.fi/files/gb-docs/gbctr.pdf
// Since there are so few registers, we just enumerate all possibilities
// instead of trying to pull out the opcode and then the arguments


// new plan, using these conditions to make it a log decision tree instead of just considering each every time
// https://gb-archive.github.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html#cb
#define LOAD_PREFIX 0x01
#define ARITH_PREFIX 0x02
#define STACK_PREFIX 0x03
#define ALU_CODE_BITS 0x38
#define REG_BITS 0x07
#define PREFIX_SHIFT 6
#define INC_DEC_CONDITION(x, z) (x == 0 && (z > 3 && z < 6))
#define LOAD_IMMEDIATE_CONDITION(x, z) (x == 0 && z == 6)
#define LOAD_IMMEDIATE_ADD_CONDITION(x, z) (x == 0 && z == 1)
#define INDIRECT_LOAD_CONDITION(x, z) (x == 0 && z == 2)
#define TWO_BYTE_INC_DEC 0x03
#define INC 0x04
#define DEC 0x05

#define ADD 0
#define ADC 1
#define SUB 2
#define SUBC 3
#define AND 4
#define XOR 5
#define OR 6
#define CP 7


// Control instructions
#define NOP         0x00
#define STOP        0x10
#define HALT        0x76
#define DI          0xF3
#define EI          0xFB
#define PREFIX      0xCB

// Jumps / calls
#define JUMP_REL_NZ_r8     0x20
#define JUMP_REL_NC_r8     0x30
#define JUMP_REL_r8        0x18
#define JUMP_REL_Z_r8      0x28
#define JUMP_REL_C_r8      0x38
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
#define LOAD_Cv_A      0xE2
#define LOAD_A_Cv      0xF2
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

#define LOAD_A_A    0x7F
#define LOAD_A_B    0x78
#define LOAD_A_C    0x79
#define LOAD_A_D    0x7A
#define LOAD_A_E    0x7B
#define LOAD_A_H    0x7C
#define LOAD_A_L    0x7D
#define LOAD_A_HL   0x7E

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

#define LOAD_HL_A   0x77
#define LOAD_HL_B   0x70
#define LOAD_HL_C   0x71
#define LOAD_HL_D   0x72
#define LOAD_HL_E   0x73
#define LOAD_HL_H   0x74
#define LOAD_HL_L   0x75

// 16 bit load instructions
#define LOAD_BC_d16   0x01 // TODO does this need the next instr?
#define LOAD_DE_d16   0x11
#define LOAD_HL_d16   0x21

// Stack pointer instructions
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
#define LOAD_HL_SP_r8 0xF8
#define LOAD_SP_HL    0xF9

// 8 bit arithmetic / logic
#define SCF        0x37
#define DAA        0x27

#define INC_A      0x3C
#define INC_B      0x04
#define INC_C      0x0C
#define INC_D      0x14
#define INC_E      0x1C
#define INC_L      0x2C
#define INC_H      0x24
#define INC_HL_v   0x34

#define DEC_A      0x3D
#define DEC_B      0x05
#define DEC_C      0x0D
#define DEC_D      0x15
#define DEC_E      0x1D
#define DEC_L      0x2D
#define DEC_H      0x25
#define DEC_HL_v   0x35

#define ADD_A_A    0x87
#define ADD_A_B    0x80
#define ADD_A_C    0x81
#define ADD_A_D    0x82
#define ADD_A_E    0x83
#define ADD_A_H    0x84
#define ADD_A_L    0x85
#define ADD_A_HL   0x86
#define ADD_A_d8   0xC6

#define ADC_A      0x8F
#define ADC_B      0x88
#define ADC_C      0x89
#define ADC_D      0x8A
#define ADC_E      0x8B
#define ADC_H      0x8C
#define ADC_L      0x8D
#define ADC_HL     0x8E
#define ADC_d8     0xCE

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

#define CP_A      0xBF
#define CP_B      0xB8
#define CP_C      0xB9
#define CP_D      0xBA
#define CP_E      0xBB
#define CP_H      0xBC
#define CP_L      0xBD
#define CP_HL     0xBE
#define CP_d8     0xFE

#define XOR_A     0xAF
#define XOR_B     0xA8
#define XOR_C     0xA9
#define XOR_D     0xAA
#define XOR_E     0xAB
#define XOR_H     0xAC
#define XOR_L     0xAD
#define XOR_HL    0xAE
#define XOR_d8    0xEE

#define SBC_A     0x9F
#define SBC_B     0x98
#define SBC_C     0x99
#define SBC_D     0x9A
#define SBC_E     0x9B
#define SBC_H     0x9C
#define SBC_L     0x9D
#define SBC_HL    0x9E
#define SBC_d8    0xDE

#define CPL       0x2F
#define CCF       0x3F


// 8 bit shift / rotate and bit instructions
#define ROTATE_LEFT_CA  0x07
#define ROTATE_LEFT_A   0x17
#define ROTATE_RIGHT_CA 0x0F
#define ROTATE_RIGHT_A  0x1F

// 16 bit arithmetic / logic
#define INC_BC      0x03
#define INC_DE      0x13
#define INC_HL      0x23
#define INC_SP      0x33

#define DEC_BC      0x0B
#define DEC_DE      0x1B
#define DEC_HL      0x2B
#define DEC_SP      0x3B

#define ADD_HL_BC   0x09
#define ADD_HL_DE   0x19
#define ADD_HL_HL   0x29
#define ADD_HL_SP   0x39

// masks and possible values for prefixed ops
#define PrefixOperationMask 0xC0
#define PrefixBitMask 0x38
#define PrefixRotOperationMask 0x38
#define PrefixRegisterMask 0x07

// prefixed ops
#define PrefixRot 0x00
#define PrefixBit 0x40
#define PrefixRes 0x80
#define PrefixSet 0x30

// Rotation ops
#define PrefixRLC  0
#define PrefixRRC  1
#define PrefixRL   2
#define PrefixRR   3
#define PrefixSLA  4
#define PrefixSRA  5
#define PrefixSWAP 6
#define PrefixSRL  7

// The register bit from memory
#define RegisterThatsActuallyMemory 6

// All the things prefixed by CB
#define RLC_B    0x00
#define RLC_C    0x01
#define RLC_D    0x02
#define RLC_E    0x03
#define RLC_H    0x04
#define RLC_L    0x05
#define RLC_HL   0x06
#define RLC_A    0x07

#define RRC_B    0x08
#define RRC_C    0x09
#define RRC_D    0x0A
#define RRC_E    0x0B
#define RRC_H    0x0C
#define RRC_L    0x0D
#define RRC_HL   0x0E
#define RRC_A    0x0F

#define RL_B     0x10
#define RL_C     0x11
#define RL_D     0x12
#define RL_E     0x13
#define RL_H     0x14
#define RL_L     0x15
#define RL_HL    0x16
#define RL_A     0x17

#define RR_B     0x18
#define RR_C     0x19
#define RR_D     0x1A
#define RR_E     0x1B
#define RR_H     0x1C
#define RR_L     0x1D
#define RR_HL    0x1E
#define RR_A     0x1F

#define SLA_B    0x20
#define SLA_C    0x21
#define SLA_D    0x22
#define SLA_E    0x23
#define SLA_H    0x24
#define SLA_L    0x25
#define SLA_HL   0x26
#define SLA_A    0x27

#define SRA_B    0x28
#define SRA_C    0x29
#define SRA_D    0x2A
#define SRA_E    0x2B
#define SRA_H    0x2C
#define SRA_L    0x2D
#define SRA_HL   0x2E
#define SRA_A    0x2F

#define SWAP_B   0x30
#define SWAP_C   0x31
#define SWAP_D   0x32
#define SWAP_E   0x33
#define SWAP_H   0x34
#define SWAP_L   0x35
#define SWAP_HL  0x36
#define SWAP_A   0x37

#define SRL_B    0x38
#define SRL_C    0x39
#define SRL_D    0x3A
#define SRL_E    0x3B
#define SRL_H    0x3C
#define SRL_L    0x3D
#define SRL_HL   0x3E
#define SRL_A    0x3F

#define BIT_0_B  0x40
#define BIT_0_C  0x41
#define BIT_0_D  0x42
#define BIT_0_E  0x43
#define BIT_0_H  0x44
#define BIT_0_L  0x45
#define BIT_0_HL 0x46
#define BIT_0_A  0x47

#define BIT_1_B  0x48
#define BIT_1_C  0x49
#define BIT_1_D  0x4A
#define BIT_1_E  0x4B
#define BIT_1_H  0x4C
#define BIT_1_L  0x4D
#define BIT_1 HL 0x4E
#define BIT_1_A  0x4F

#define BIT_2_B  0x50
#define BIT_2_C  0x51
#define BIT_2_D  0x52
#define BIT_2_E  0x53
#define BIT_2_H  0x54
#define BIT_2_L  0x55
#define BIT_2_HL 0x56
#define BIT_2_A  0x57

#define BIT_3_B  0x58
#define BIT_3_C  0x59
#define BIT_3_D  0x5A
#define BIT_3_E  0x5B
#define BIT_3_H  0x5C
#define BIT_3_L  0x5D
#define BIT_3 HL 0x5E
#define BIT_3_A  0x5F

#define BIT_4_B  0x60
#define BIT_4_C  0x61
#define BIT_4_D  0x62
#define BIT_4_E  0x63
#define BIT_4_H  0x64
#define BIT_4_L  0x65
#define BIT_4_HL 0x66
#define BIT_4_A  0x67

#define BIT_5_B  0x68
#define BIT_5_C  0x69
#define BIT_5_D  0x6A
#define BIT_5_E  0x6B
#define BIT_5_H  0x6C
#define BIT_5_L  0x6D
#define BIT_5 HL 0x6E
#define BIT_5_A  0x6F

#define BIT_6_B  0x70
#define BIT_6_C  0x71
#define BIT_6_D  0x72
#define BIT_6_E  0x73
#define BIT_6_H  0x74
#define BIT_6_L  0x75
#define BIT_6_HL 0x76
#define BIT_6_A  0x77

#define BIT_7_B  0x78
#define BIT_7_C  0x79
#define BIT_7_D  0x7A
#define BIT_7_E  0x7B
#define BIT_7_H  0x7C
#define BIT_7_L  0x7D
#define BIT_7 HL 0x7E
#define BIT_7_A  0x7F

#define RES_0_B  0x80
#define RES_0_C  0x81
#define RES_0_D  0x82
#define RES_0_E  0x83
#define RES_0_H  0x84
#define RES_0_L  0x85
#define RES_0_HL 0x86
#define RES_0_A  0x87

#define RES_1_B  0x88
#define RES_1_C  0x89
#define RES_1_D  0x8A
#define RES_1_E  0x8B
#define RES_1_H  0x8C
#define RES_1_L  0x8D
#define RES_1 HL 0x8E
#define RES_1_A  0x8F

#define RES_2_B  0x90
#define RES_2_C  0x91
#define RES_2_D  0x92
#define RES_2_E  0x93
#define RES_2_H  0x94
#define RES_2_L  0x95
#define RES_2_HL 0x96
#define RES_2_A  0x97

#define RES_3_B  0x98
#define RES_3_C  0x99
#define RES_3_D  0x9A
#define RES_3_E  0x9B
#define RES_3_H  0x9C
#define RES_3_L  0x9D
#define RES_3 HL 0x9E
#define RES_3_A  0x9F

#define RES_4_B  0xA0
#define RES_4_C  0xA1
#define RES_4_D  0xA2
#define RES_4_E  0xA3
#define RES_4_H  0xA4
#define RES_4_L  0xA5
#define RES_4_HL 0xA6
#define RES_4_A  0xA7

#define RES_5_B  0xA8
#define RES_5_C  0xA9
#define RES_5_D  0xAA
#define RES_5_E  0xAB
#define RES_5_H  0xAC
#define RES_5_L  0xAD
#define RES_5 HL 0xAE
#define RES_5_A  0xAF

#define RES_6_B  0xB0
#define RES_6_C  0xB1
#define RES_6_D  0xB2
#define RES_6_E  0xB3
#define RES_6_H  0xB4
#define RES_6_L  0xB5
#define RES_6_HL 0xB6
#define RES_6_A  0xB7

#define RES_7_B  0xB8
#define RES_7_C  0xB9
#define RES_7_D  0xBA
#define RES_7_E  0xBB
#define RES_7_H  0xBC
#define RES_7_L  0xBD
#define RES_7 HL 0xBE
#define RES_7_A  0xBF

#define SET_0_B  0xC0
#define SET_0_C  0xC1
#define SET_0_D  0xC2
#define SET_0_E  0xC3
#define SET_0_H  0xC4
#define SET_0_L  0xC5
#define SET_0_HL 0xC6
#define SET_0_A  0xC7

#define SET_1_B  0xC8
#define SET_1_C  0xC9
#define SET_1_D  0xCA
#define SET_1_E  0xCB
#define SET_1_H  0xCC
#define SET_1_L  0xCD
#define SET_1 HL 0xCE
#define SET_1_A  0xCF

#define SET_2_B  0xD0
#define SET_2_C  0xD1
#define SET_2_D  0xD2
#define SET_2_E  0xD3
#define SET_2_H  0xD4
#define SET_2_L  0xD5
#define SET_2_HL 0xD6
#define SET_2_A  0xD7

#define SET_3_B  0xD8
#define SET_3_C  0xD9
#define SET_3_D  0xDA
#define SET_3_E  0xDB
#define SET_3_H  0xDC
#define SET_3_L  0xDD
#define SET_3 HL 0xDE
#define SET_3_A  0xDF

#define SET_4_B  0xE0
#define SET_4_C  0xE1
#define SET_4_D  0xE2
#define SET_4_E  0xE3
#define SET_4_H  0xE4
#define SET_4_L  0xE5
#define SET_4_HL 0xE6
#define SET_4_A  0xE7

#define SET_5_B  0xE8
#define SET_5_C  0xE9
#define SET_5_D  0xEA
#define SET_5_E  0xEB
#define SET_5_H  0xEC
#define SET_5_L  0xED
#define SET_5 HL 0xEE
#define SET_5_A  0xEF

#define SET_6_B  0xF0
#define SET_6_C  0xF1
#define SET_6_D  0xF2
#define SET_6_E  0xF3
#define SET_6_H  0xF4
#define SET_6_L  0xF5
#define SET_6_HL 0xF6
#define SET_6_A  0xF7

#define SET_7_B  0xF8
#define SET_7_C  0xF9
#define SET_7_D  0xFA
#define SET_7_E  0xFB
#define SET_7_H  0xFC
#define SET_7_L  0xFD
#define SET_7 HL 0xFE
#define SET_7_A  0xFF