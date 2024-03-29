#include "Rom.h"
#include "MMU.h"
#include <cstdint>
#include <functional>

class CPU
{
public:
    CPU();
    CPU(MMU* mmu);
    ~CPU();

    void ExecuteCode();

    void SetDisplaySignalFunc(std::function<void(int)> signalDisplayForNextScanline)
    {
        SignalDisplayForNextScanline = signalDisplayForNextScanline;
    }

    void SetDisplayUpdateFunc(std::function<void()> signalDisplayToUpdate)
    {
        SignalDisplayToUpdate = signalDisplayToUpdate;
    }

    bool Executing() 
    {
        return executing;
    }



    void SetVBlankInterrupt();
    void SetLCDStatInterrupt();
    // This takes the input format of the 0xFF00 register
    void SetJoypadInterrupt(uint8_t joypadRegister);

    void MaybeResetTimer(uint8_t newTmcVal)
    {
        uint8_t newF = GetTmcFrequency(newTmcVal);
        uint8_t oldF = GetTmcFrequency(mmu->ReadFromAddress(MMU::TMCRegisterAddress));
        if (newF != oldF)
        {
            clockCounter = 0;
        }
    }

private:
    enum SixteenBitRegisters
    {
        AF = 0,
        BC,
        DE,
        HL,
        SP,
        PC
    };

    // opposite because endianness
    enum EightBitRegisters
    {
        F = 0,
        A,
        C,
        B,
        E,
        D,
        L,
        H,
        P,
        S,
        Co,
        Pr,
    };

    // Register map from assembly-indexed registers to my indexing
    // That is, register B is 0 in assembly but cannot be 0 (at most 1)
    // here because of the endianness of the 16 bit registers.
    // we explicitly call out that one "register" is a HL pointer
    uint8_t HL_pointer = 0xff;
    uint8_t regMap[8] = {B, C, D, E, H, L, HL_pointer, A};
    uint8_t doubleRegMap[4] = {BC, DE, HL, SP};

    union Registers
    {
        std::uint16_t shorts[6];
        std::uint8_t bytes[12];
    };

    // Register flags
    #define ZeroFlag      0x80 // Z
    #define AddSubFlag    0x40 // N
    #define HalfCarryFlag 0x20 // H
    #define CarryFlag     0x10 // C
    #define AllFlags      0xF0
    #define MostSigBit    0x80
    #define LeastSigBit   0x01

    Registers registers;

    // The cycle counter works in M-cycles, not # of clock ticks
    // Each instruction is a multiple of 4 clock ticks == 1 m-cycle. 
    uint16_t clockCounter;
    
    uint32_t cycles = 0;

    uint16_t m_dividerRegister = 0;

    MMU* mmu;

    // EI, DI, and RETI change interrupts on the cycle after
    bool enableInterruptsNextCycle = false;
    bool disableInterruptsNextCycle = false;
    bool interruptsAreEnabled = false;

    // Handling Harold Halt mode
    bool haltMode = false;
    uint16_t haltReturnPC = 0;

    bool executing;

    void InitialiseRegisters();

    std::function<void(int)> SignalDisplayForNextScanline;

    std::function<void()> SignalDisplayToUpdate;

    // This processes clock cycles and anything related,
    // such as timers and interrupts
    void ExecuteCycles(int numMCycles);

    // Execute next instruction, increment PC
    // returns number of cycles of execution it took
    // in mCycles, which is 4*clock cycles
    int ExecuteNextInstruction();
    int ExecutePrefixInstruction(uint8_t instruction);

    void CheckAndMaybeHandleInterrupts();

    inline int GetTmcFrequency(uint8_t f)
    {
        f = f & MMU::TimerControllerBits;
        switch (f)
        {
            case 0:
                return 4096; //Hz, so 1024 cycles
            case 1:
                return 262144; //Hz, so 16 cycles
            case 2:
                return 65536; //Hz, so 64 cycles
            case 3:
                return 16384; //Hz, so 256 cycles
            default:
                // crash?
                break;
        }
        // this will cause a crash
        return 0;
    }

    inline void Add8(uint8_t A, uint8_t B, uint8_t& C);
    inline void Sub8(uint8_t A, uint8_t B, uint8_t& C);
    inline void AddC8(uint8_t A, uint8_t B, uint8_t& C);
    inline void SubC8(uint8_t A, uint8_t B, uint8_t& C);
    inline void And8(uint8_t A, uint8_t B, uint8_t& C);
    inline void Or8(uint8_t A, uint8_t B, uint8_t& C);
    inline void Xor8(uint8_t A, uint8_t B, uint8_t& C);
    inline void Cp8(uint8_t A, uint8_t B);
    inline void Inc8(uint8_t& A);
    inline void Dec8(uint8_t& A);

    inline void Add16(uint16_t& A, uint16_t& B, uint16_t& C);
    
    inline uint8_t GetRegisterFromPrefixIndex(uint8_t reg);
    int PerformPrefixedRotOperation(uint8_t operation, uint8_t regIndex, bool useHL);
};