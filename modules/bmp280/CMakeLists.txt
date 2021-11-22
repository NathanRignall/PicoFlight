cmake_minimum_required(VERSION 3.0)

add_library(bmp2080 INTERFACE)
    
target_sources(bmp2080 INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/bmp2080.cpp
)

target_include_directories(bmp2080 INTERFACE
        include
)

target_link_libraries(bmp2080 INTERFACE
        pico_stdlib
        hardware_i2c
)
