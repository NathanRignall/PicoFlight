add_library(flash_spi INTERFACE)

target_sources(flash_spi INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/flash_spi.c
)

target_include_directories(flash_spi INTERFACE
    src
)

target_link_libraries(flash_spi INTERFACE
        hardware_spi
        pico_stdlib
)
