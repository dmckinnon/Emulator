#include "Rom.h"

#include <fstream>
#include <string>
#include <cstring>

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

        rom->SetSize(length);
        romFile.read((char*)rom->bytes, rom->size);
    }
    else
    {
        return false;
    }

    return true;
}

bool LoadRomFromBinary(uint8_t* buffer, unsigned int size, std::shared_ptr<Rom> rom)
{
    // check this ROM is small enough
    if (size > MAX_ROM_SIZE_BYTES)
    {
        return false;
    }

    memcpy(rom->bytes, buffer, size);
    rom->size = size;

    return true;
}