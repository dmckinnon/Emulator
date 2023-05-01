#include "Rom.h"
#include "MMU.h"
#include <cstdint>
#include <functional>

class CPU
{
public:
    CPU();
    CPU(std::shared_ptr<MMU> mmu);
    ~CPU();

    void ExecuteCode();

    void SetDisplaySignalFunc(std::function<void()> signalDisplayForNextScanline)
    {
        SignalDisplayForNextScanline = signalDisplayForNextScanline;
    }

    bool Executing() 
    {
        return executing;
    }



    void SetVBlankInterrupt();
    void SetLCDStatInterrupt();
    // This takes the input format of the 0xFF00 register
    void SetJoypadInterrupt(uint8_t joypadRegister);

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
    #define AllFlags      0xF0
    #define MostSigBit    0x80
    #define LeastSigBit   0x10

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

    bool executing;

    void InitialiseRegisters();

    std::function<void()> SignalDisplayForNextScanline;

    // This processes clock cycles and anything related,
    // such as timers and interrupts
    void ExecuteCycles(int numMCycles);

    // Execute next instruction, increment PC
    // returns number of cycles of execution it took
    // in mCycles, which is 4*clock cycles
    int ExecuteNextInstruction();
    int ExecutePrefixInstruction(uint8_t instruction);

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
    
    inline uint8_t GetRegisterFromPrefixIndex(uint8_t reg);
    int PerformPrefixedRotOperation(uint8_t operation, uint8_t regIndex, bool useHL);
};