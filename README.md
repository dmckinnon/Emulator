# Emulator
Project: GBC emulator
Initial attempt is to write a Gameboy colour emulator for linux, with a GUI and controlled by keyboard.
Next step - once this is tested, using Pokemon as the benchmark game - is to implement this on an embedded system, preferably an ARM chip. 
Finally, design a PCB and 3D printed case and make this physically

Current progress: partway through testing instructions on linux




Helpful links: 
+https://github.com/espressif/esp32-nesemu
+https://arstechnica.com/gaming/2011/08/accuracy-takes-power-one-mans-3ghz-quest-to-build-a-perfect-snes-emulator/
+https://forum.arduino.cc/index.php?topic=182137.0
+https://www.crystalfontz.com/product/cfal9664bfb2-graphic-96x64-oled-rgb?kw=&origin=pla&gclid=Cj0KCQiAieTUBRCaARIsAHeLDCTMpSWEkBIuy-yXdANbvPykkWKVy4LIp63WGejUZBG-O9HH9lcfc2EaAu2eEALw_wcB
+https://www.neoplc.org/shop/display-oled-64-x-48
+ Actually writing an emulator: https://www.youtube.com/watch?v=y71lli8MS8s&feature=youtu.be
+ the code from that: https://bisqwit.iki.fi/jutut/kuvat/programming_examples/nesemu1/
+ Another emu: https://medium.com/@fogleman/i-made-an-nes-emulator-here-s-what-i-learned-about-the-original-nintendo-2e078c9b28fe - It is worth mentioning that this links CPU reference for the 6052, and has a vast store of technical details:
+ http://nesdev.com/NESDoc.pdf/
+ http://wiki.nesdev.com/w/index.php/NES_reference_guide
+ http://www.obelisk.demon.co.uk/6502/
+ even another emu, WITH SOURCE CODE: https://github.com/amhndu/SimpleNES
