# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(emulator C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()



# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.cpp
    CPU.cpp
    Rom.cpp 
    Display.cpp
    st7789_driver.cpp
    Gameboy.cpp 
    Adafruit_GFX.cpp
    glcdfont.c
    Joypad.cpp 
    MMU.cpp 
)

target_compile_definitions(${PROJECT_NAME} PUBLIC RP2040)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

pico_generate_pio_header(${PROJECT_NAME} /home/dmckinnon/Emulator/st7789_driver.pio)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pio
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)