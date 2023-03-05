#include <thread>

#include "Rom.h"
#include "MMU.h"
#include "CPU.h"
#include "Display.h"

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
    Display display;

    int clockSpeed = GAMEBOY_CLOCK_HZ;

    std::shared_ptr<MMU> mmu;

    // some graphics nonsense

    std::shared_ptr<Rom> rom;
};