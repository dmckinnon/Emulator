#include "Rom.h"
#include "MMU.h"
#include <cstdint>

class CPU
{
public:
    CPU();
    CPU(std::shared_ptr<MMU> mmu);
    ~CPU();

    void ExecuteCode();

    void Execute();

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

    enum EightBitRegisters
    {
        A = 0,
        F,
        B,
        C,
        D,
        E,
        H,
        L,
        S,
        P,
        Pr,
        Co,
    };

    union Registers
    {
        std::uint16_t shorts[6];
        std::uint8_t bytes[12];
    };

    // Register flags
    #define ZeroFlag      0x80
    #define AddSubFlag    0x40
    #define HalfCarryFlag 0x20
    #define CarryFlag     0x10

    Registers registers;

    // The cycle counter works in M-cycles, not # of clock ticks
    // Each instruction is a multiple of 4 clock ticks == 1 m-cycle. 
    std::uint64_t mCycles;

    std::shared_ptr<MMU> mmu;

    bool interruptsAreEnabled = false;

    void InitialiseRegisters();

    inline void Add8(byte A, byte B, byte& C);
    inline void Sub8(byte A, byte B, byte& C);
    inline void AddC8(byte A, byte B, byte& C);
    inline void SubC8(byte A, byte B, byte& C);
    inline void And8(byte A, byte B, byte& C);
    inline void Or8(byte A, byte B, byte& C);
    inline void Xor8(byte A, byte B, byte& C);
    inline void Cp8(byte A, byte B);
    inline void Inc8(byte& A);
    inline void Dec8(byte& A);
};