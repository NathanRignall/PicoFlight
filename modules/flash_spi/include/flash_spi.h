#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define FLASH_SPI_PAGE_SIZE 256

#define FLASH_SPI_STATUS_BUSY_MASK 0x01
#define FLASH_SPI_CMD_PAGE_PROGRAM 0x02
#define FLASH_SPI_CMD_READ 0x03
#define FLASH_SPI_CMD_STATUS 0x05
#define FLASH_SPI_CMD_WRITE_EN 0x06
#define FLASH_SPI_CMD_CHIP_ERASE 0xC7
#define FLASH_SPI_CMD_SECTOR_ERASE 0x20
#define FLASH_SPI_CMD_RELEASE_POWER_DOWN 0xAB

struct flash_spi
{
    spi_inst_t *hw_inst;
    uint cs_gpio;
};

void flash_spi_power_up(flash_spi *inst);
void flash_spi_read(flash_spi *inst, uint16_t page, uint8_t *buffer, size_t length);
void flash_spi_chip_erase(flash_spi *inst);
void flash_spi_sector_erase(flash_spi *inst, uint16_t page);
void flash_spi_page_program(flash_spi *inst, uint16_t page, uint8_t data[]);
void printbuf(uint8_t buf[FLASH_SPI_PAGE_SIZE]);
