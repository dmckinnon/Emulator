#include <iostream>
#include <string.h>

#include "Rom.h"

using namespace std;

struct Parameters
{
    string romFilename = "";

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

    // load rom from file to byte buffer
    Rom rom;
    if (!LoadRomFromFile(parameters.romFilename, rom))
    {
        cerr << "Could not load ROM from " << parameters.romFilename << std::endl;
    }

    // go into ROM and start running instructions

    // get GUI?


    return 0;
}