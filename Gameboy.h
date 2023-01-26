#include <thread>

#include "Rom.h"
#include "CPU.h"

// 8 KiB of RAM
#define GAMEBOY_SRAM 8192
#define GAMEBOY_VRAM 8192

// clock speeds
#define GAMEBOY_CLOCK_HZ 4194304
#define GAMEBOY_COLOR_CLOCK_HZ 8388608

class Gameboy
{
public:
    Gameboy(std::shared_ptr<Rom> rom);
    ~Gameboy();

    bool LoadRom(std::shared_ptr<Rom> rom);

    bool Run();

private:
    CPU cpu;

    int clockSpeed = GAMEBOY_CLOCK_HZ;

    // some memory pool?
    // define this in some header so we have offset?
    byte* sram;
    byte* vram;

    // some graphics nonsense

    std::shared_ptr<Rom> rom;
};