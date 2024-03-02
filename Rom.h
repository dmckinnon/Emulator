#pragma once

#include <memory>

// 1 Mb
#define MAX_ROM_SIZE_BYTES (1024*1024)

//typedef unsigned char uint8_t;

struct Rom
{
    unsigned int size;
    uint8_t* bytes;
    

    Rom(uint32_t size)
    {
        bytes = (uint8_t *)malloc(size);
    }

    Rom()
    {
        size = 0;
        bytes = nullptr;
    }

    ~Rom()
    {
        if (bytes != nullptr)
        {
            free(bytes);
        }
    }

    void SetSize(uint32_t size)
    {
        if (bytes != nullptr)
        {
            free(bytes);
        }

        bytes = (uint8_t *)malloc(size);
    }
};

bool LoadRomFromFile(std::string filename, std::shared_ptr<Rom> rom);

bool LoadRomFromBinary(uint8_t* buffer, unsigned int size, std::shared_ptr<Rom> rom);