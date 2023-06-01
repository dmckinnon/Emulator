#pragma once

#include <memory>

// 8 Mb
#define MAX_ROM_SIZE_BYTES (8*1024*1024)

//typedef unsigned char uint8_t;

struct Rom
{
    uint8_t bytes[MAX_ROM_SIZE_BYTES];
    unsigned int size;
};

bool LoadRomFromFile(std::string filename, std::shared_ptr<Rom> rom);

bool LoadRomFromBinary(uint8_t* buffer, unsigned int size, std::shared_ptr<Rom> rom);