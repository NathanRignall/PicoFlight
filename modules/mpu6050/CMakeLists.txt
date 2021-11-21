cmake_minimum_required(VERSION 3.0)

add_library(mpu6050 INTERFACE)
    
target_sources(mpu6050 INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/mpu6050.cpp
)

target_include_directories(mpu6050 INTERFACE
        include
)

target_link_libraries(mpu6050 INTERFACE
        pico_stdlib
        hardware_i2c
)
