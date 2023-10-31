#include <iostream>
#include <string.h>
#include "SystemRom.h"
#ifdef RP2040
#include <stdio.h>
#include "pico/stdlib.h"


#include "Adafruit_GFX.h"

// games, until we can get SD card working
//#include "Tetris.h"
#include "SuperMario.h"
//#include "LegendOfZeldaLinksAwakening.h"
#else
class ST7789;
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

#ifdef RP2040
int main()
{
    stdio_init_all();
#else
int main(int argc, char* argv[])
{ 
    auto parameters = ParseArgs(argc, argv);
#endif

    // Start display
    std::shared_ptr<ST7789> lcd;
#ifdef RP2040
    lcd = make_shared<ST7789>();
    lcd->InitLCD();
    lcd->ClearScreen(true);
    lcd->SetDisplayArea(LCD_WIDTH, LCD_HEIGHT);



    GFXcanvas16 canvas(LCD_WIDTH, LCD_HEIGHT);
    canvas.setTextColor(0x0780);
    canvas.writeString("Loading System ROM");
    lcd->WriteBuffer(canvas.getBuffer());
    canvas.setCursor(0, 10);
#endif

    // load rom from file to uint8_t buffer
    std::shared_ptr<Rom> systemRom = make_shared<Rom>(systemRomSize);
    std::shared_ptr<Rom> gameRom;
    

    // get ROM from SD Card, ostensibly
#ifdef RP2040
    // load binary from a header
    if (!LoadRomFromBinary(systemRomBinary, systemRomSize, systemRom))
    {
        // print something to the display
        // We kinda need the graphics for this ... 
        // TODO add GFX
        canvas.writeString("System ROM failed to load");
        lcd->WriteBuffer(canvas.getBuffer());
    }
    else
    {
        canvas.writeString("System ROM loaded.");
        lcd->WriteBuffer(canvas.getBuffer());
    }

    canvas.setCursor(0, 20);
    canvas.writeString("Loading game ROM: ");
    canvas.writeString(SuperMarioName);
    lcd->WriteBuffer(canvas.getBuffer());
    canvas.setCursor(0, 30);
    // get size of game ROM and game name

    // get game size
    //gameRom = *SuperMarioRom;// make_shared<Rom>(SuperMarioRom);
    /*if (!LoadRomFromBinary(SuperMarioRom, SuperMarioRomSize, gameRom))
    {
        canvas.writeString("No game ROM loaded. Running only system ROM");
        lcd->WriteBuffer(canvas.getBuffer());
    }
    else*/
    gameRom = std::make_shared<Rom>();
    gameRom->size = SuperMarioRomSize;
    gameRom->bytes = (uint8_t*)SuperMarioRomBinary;
    {
        canvas.writeString("Loaded game. Running ... ");
        lcd->WriteBuffer(canvas.getBuffer());
    }

    sleep_ms(5000);

#else
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
#endif



    // go into ROM and start running instructions

    //std::cout << "Size of game ROM: " << gameRom->size << std::endl;

    {
#ifdef RP2040
        // destroy the original canvas and free up the memory
        canvas.~GFXcanvas16();
#endif

        Gameboy g = Gameboy(systemRom, lcd);
        g.LoadRom(gameRom);

#ifdef RP2040
        // We've copied game rom and system rom. Delete the objects now to save memory
        systemRom.reset();
        gameRom.reset();
#endif

        g.Run();
    }

    return 0;
}