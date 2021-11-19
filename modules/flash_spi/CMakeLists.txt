cmake_minimum_required(VERSION 3.0)

add_library(flash_spi INTERFACE)
    
target_sources(flash_spi INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/flash_spi.cpp
)

target_include_directories(flash_spi INTERFACE
        src
)

target_link_libraries(flash_spi INTERFACE
        pico_stdlib
        hardware_spi
)
