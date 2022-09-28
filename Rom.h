#include <string>
#include <memory>

// 8 Mb
#define MAX_ROM_SIZE_BYTES (8*1024*1024)

typedef unsigned char byte;

struct Rom
{
    byte bytes[MAX_ROM_SIZE_BYTES];
    unsigned int size;
};

bool LoadRomFromFile(std::string filename, std::shared_ptr<Rom> rom);