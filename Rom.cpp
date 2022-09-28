#include "Rom.h"

#include <fstream>

bool LoadRomFromFile(std::string filename, std::shared_ptr<Rom> rom)
{
    if (filename.empty())
    {
        return false;
    }

    std::ifstream romFile(filename);
    if (romFile.is_open())
    {
        //get length of file
        romFile.seekg(0, std::ios::end);
        size_t length = romFile.tellg();
        romFile.seekg(0, std::ios::beg);

        rom->size = length;
        romFile.read((char*)rom->bytes, rom->size);
    }
    else
    {
        return false;
    }

    return true;
}