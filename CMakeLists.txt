# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoFlight C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PicoFlight PicoFlight.cpp)

# Add any other source code
add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)
add_subdirectory(modules/flight_data)
add_subdirectory(modules/flash_spi)
add_subdirectory(modules/mpu6050)

pico_set_program_name(PicoFlight "PicoFlight")
pico_set_program_version(PicoFlight "0.1")

pico_enable_stdio_uart(PicoFlight 1)
pico_enable_stdio_usb(PicoFlight 0)

# Add the standard library to the build
target_link_libraries(PicoFlight pico_stdlib)

# Add any user requested libraries
target_link_libraries(PicoFlight
        hardware_spi
        hardware_i2c
        FatFs_SPI
        flash_spi
        flight_data
        mpu6050
        )

pico_add_extra_outputs(PicoFlight)

