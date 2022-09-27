#include <string>

#define MAX_ROM_SIZE_BYTES 1024

typedef unsigned char byte;

struct Rom
{
    byte bytes[MAX_ROM_SIZE_BYTES];
    unsigned int size;
};

bool LoadRomFromFile(std::string filename, Rom& rom);