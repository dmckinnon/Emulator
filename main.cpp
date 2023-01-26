#include <iostream>
#include <string.h>

#include "Gameboy.h"

using namespace std;

struct Parameters
{
    // default to make things easier for me
    string romFilename = "/home/dmckinnon/Emulator/math.gb";

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
    //
    // load an address into H and L
    // load a value into memory at HL
    // read value from memory at HL
    //
    // Load a value into memory at 0x02
    // and add it to a value loaded into A
    //
    // Test vram

    // load rom from file to byte buffer
    std::shared_ptr<Rom> rom = make_shared<Rom>();
    if (!LoadRomFromFile(parameters.romFilename, rom))
    {
        cerr << "Could not load ROM from " << parameters.romFilename << std::endl;
    }

    // go into ROM and start running instructions

    std::cout << "Size of ROM: " << rom->size << std::endl;

    Gameboy g = Gameboy(rom);

    g.Run();

    // get GUI?


    return 0;
}