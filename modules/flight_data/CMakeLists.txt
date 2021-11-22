cmake_minimum_required(VERSION 3.0)

add_library(flight_data INTERFACE)
    
target_sources(flight_data INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/flight_data.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/hw_config.c
)

target_include_directories(flight_data INTERFACE
        include
)

target_link_libraries(flight_data INTERFACE
        pico_stdlib
        FatFs_SPI
        flash_spi
)
