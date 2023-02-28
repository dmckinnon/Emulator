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
        std::uint8_t uint8_ts[12];
    };

    // Register flags
    #define ZeroFlag      0x80
    #define AddSubFlag    0x40
    #define HalfCarryFlag 0x20
    #define CarryFlag     0x10

    Registers registers;

    // The cycle counter works in M-cycles, not # of clock ticks
    // Each instruction is a multiple of 4 clock ticks == 1 m-cycle. 
    uint16_t clockCounter;
    uint16_t clockDivider;

    std::shared_ptr<MMU> mmu;

    // EI, DI, and RETI change interrupts on the cycle after
    bool enableInterruptsNextCycle = false;
    bool disableInterruptsNextCycle = false;
    bool interruptsAreEnabled = false;

    void InitialiseRegisters();

    // This processes clock cycles and anything related,
    // such as timers and interrupts
    void ExecuteCycles(int numMCycles);

    // Execute next instruction, increment PC
    // returns number of cycles of execution it took
    // in mCycles, which is 4*clock cycles
    int ExecuteNextInstruction();

    void CheckAndMaybeHandleInterrupts();

    inline int GetTmcFrequency()
    {
        uint8_t f = mmu->ReadFromAddress(MMU::TMCRegisterAddress);
        switch (f)
        {
            case 0:
                return 1024; // 4096 Hz
            case 1:
                return 16;   // 262144 Hz
            case 3:
                return 64;   // 65536 Hz
            case 4:
                return 256;  // 16384 Hz
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
};