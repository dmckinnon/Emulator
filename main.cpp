#include <iostream>
#include <string.h>
#if defined(__linux__) || defined(_WIN32)
#else
#include "pico/stdlib.h"
#endif

#include "Gameboy.h"

using namespace std;

struct Parameters
{
    // default to make things easier for me
    string romFilename = "/home/dave/Emulator/tests/05-op rp.gb";

};  

Parameters ParseArgs(int argc, char* argv[])
{
    Parameters p;
    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "-f") == 0)
        {
            p.romFilename = argv[++i];
        }
        
        // more args
        /*else if ()
        {

        }*/
    }

    return p;
}

int main(int argc, char* argv[])
{ 
    auto parameters = ParseArgs(argc, argv);

    // Tests:
    // load 255 into register A and B
    // add A and B
    // check result
    // PASSED
    //
    // load two numbers into registers
    // write these to memory at 0 and 1
    // read these back into registers
    // PASSED

    // TODO before any further tests:
    // sort out the memory mapping

    //
    // load an address into H and L
    // load a value into memory at HL
    // read value from memory at HL
    //
    // Test the funky 8 bit load instructions
    //
    // Load a value into memory at 0x02
    // and add it to a value loaded into A
    //
    // Test address offsetting

    // Need also:
    // copy ROM into read-only memory
    // IO interrupts
    // graphics??
    // automated testing

    // load rom from file to uint8_t buffer
    std::shared_ptr<Rom> systemRom = make_shared<Rom>(); 
    std::shared_ptr<Rom> gameRom = make_shared<Rom>(); 
    if (!LoadRomFromFile(parameters.romFilename, gameRom))
    {
        cerr << "Could not load game ROM from " << parameters.romFilename << std::endl;
    }
    if (!LoadRomFromFile("/home/dave/Emulator/systemRom_noloops.bin", systemRom))
    {
        cerr << "Could not load system ROM from " << "/home/dave/Emulator/systemRom.bin" << std::endl;
    }

    // go into ROM and start running instructions

    std::cout << "Size of game ROM: " << gameRom->size << std::endl;

    {
        Gameboy g = Gameboy(systemRom);
        g.LoadRom(gameRom);
        g.Run();
    }

    return 0;
}