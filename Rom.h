#pragma once

#include <string>
#include <memory>

// 8 Mb
#define MAX_ROM_SIZE_uint8_tS (8*1024*1024)

//typedef unsigned char uint8_t;

struct Rom
{
    uint8_t uint8_ts[MAX_ROM_SIZE_uint8_tS];
    unsigned int size;
};

bool LoadRomFromFile(std::string filename, std::shared_ptr<Rom> rom);