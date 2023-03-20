# Emulator
Project: GBC emulator
Initial attempt is to write a Gameboy colour emulator for linux, with a GUI and controlled by keyboard.
Next step - once this is tested, using Pokemon as the benchmark game - is to implement this on an embedded system, preferably an ARM chip. 
Finally, design a PCB and 3D printed case and make this physically

Current progress: partway through testing instructions on linux

All the instructions: https://gbdev.io/gb-opcodes/optables/

Technical reference doc: https://gekkio.fi/files/gb-docs/gbctr.pdf

Cycle-accurate docs: https://github.com/geaz/emu-gameboy/blob/master/docs/The%20Cycle-Accurate%20Game%20Boy%20Docs.pdf

CPU manual: http://www.codeslinger.co.uk/pages/projects/gameboy/files/GB.pdf

Pan docs: https://gbdev.io/pandocs/Power_Up_Sequence.html

Some blogs where others have implemented the same, and written various levels of detail:
+ https://robertovaccari.com/blog/2020_09_26_gameboy/
+ http://gameboy.mongenel.com/dmg/asmmemmap.html
+ http://www.codeslinger.co.uk/pages/projects/gameboy/timers.html
+ http://imrannazar.com/GameBoy-Emulation-in-JavaScript:-Memory this guy did it in javascript what the hell
+ https://cturt.github.io/cinoop.html


Som etest roms: https://gbdev.gg8.se/wiki/articles/Test_ROMs and their download package: https://github.com/c-sp/gameboy-test-roms/releases

Helpful links: 
+ https://gb-archive.github.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html#cb

+ https://github.com/espressif/esp32-nesemu  

+ https://arstechnica.com/gaming/2011/08/accuracy-takes-power-one-mans-3ghz-quest-to-build-a-perfect-snes-emulator/  

+ https://forum.arduino.cc/index.php?topic=182137.0  

+ https://www.crystalfontz.com/product/cfal9664bfb2-graphic-96x64-oled-rgb?kw=&origin=pla&gclid=Cj0KCQiAieTUBRCaARIsAHeLDCTMpSWEkBIuy-yXdANbvPykkWKVy4LIp63WGejUZBG-O9HH9lcfc2EaAu2eEALw_wcB  

+https://www.neoplc.org/shop/display-oled-64-x-48  

+ Actually writing an emulator: https://www.youtube.com/watch?v=y71lli8MS8s&feature=youtu.be
+ the code from that: https://bisqwit.iki.fi/jutut/kuvat/programming_examples/nesemu1/
+ Another emu: https://medium.com/@fogleman/i-made-an-nes-emulator-here-s-what-i-learned-about-the-original-nintendo-2e078c9b28fe - It is worth mentioning that this links CPU reference for the 6052, and has a vast store of technical details:
+ http://nesdev.com/NESDoc.pdf/
+ http://wiki.nesdev.com/w/index.php/NES_reference_guide
+ http://www.obelisk.demon.co.uk/6502/
+ even another emu, WITH SOURCE CODE: https://github.com/amhndu/SimpleNES
