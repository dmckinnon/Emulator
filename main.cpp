#include <iostream>
#include <string.h>
#ifdef RASPBERRYPI_PICO
#include <stdio.h>
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
    stdio_init_all();

    auto parameters = ParseArgs(argc, argv);

    // Start display
    std::shared_ptr<ST7789> lcd;
#ifdef RASPBERRYPI_PICO
    lcd = make_shared<ST7789>();
#endif

    // load rom from file to uint8_t buffer
    std::shared_ptr<Rom> systemRom = make_shared<Rom>(); 
    std::shared_ptr<Rom> gameRom = make_shared<Rom>();

    // get ROM from SD Card, ostensibly

    if (!LoadRomFromFile(parameters.romFilename, gameRom))
    {
        if (lcd)
        {

        }
        else
        {
            cerr << "Could not load game ROM from " << parameters.romFilename << std::endl;
        }
    }
    if (!LoadRomFromFile("/home/dave/Emulator/systemRom_noloops.bin", systemRom))
    {
        cerr << "Could not load system ROM from " << "/home/dave/Emulator/systemRom.bin" << std::endl;
    }

    // go into ROM and start running instructions

    std::cout << "Size of game ROM: " << gameRom->size << std::endl;

    {
        Gameboy g = Gameboy(systemRom, lcd);
        g.LoadRom(gameRom);
        g.Run();
    }

    return 0;
}