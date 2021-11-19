#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "flash_spi.h"

static inline void cs_select(uint cs_pin)
{
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin)
{
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void write_enable(spi_inst_t *spi, uint cs_pin)
{
    cs_select(cs_pin);
    uint8_t cmd = FLASH_SPI_CMD_WRITE_EN;
    spi_write_blocking(spi, &cmd, 1);
    cs_deselect(cs_pin);
}

void wait_done(spi_inst_t *spi, uint cs_pin)
{
    uint8_t status;
    do
    {
        cs_select(cs_pin);
        uint8_t buf[2] = {FLASH_SPI_CMD_STATUS, 0};
        spi_write_read_blocking(spi, buf, buf, 2);
        cs_deselect(cs_pin);
        status = buf[1];
    } while (status & FLASH_SPI_STATUS_BUSY_MASK);
}

// public
void flash_spi_read(flash_spi *inst, uint32_t address, uint32_t page, uint8_t *buffer, size_t length)
{
    cs_select(inst->cs_gpio);
    uint8_t cmdbuf[4] = {
        FLASH_SPI_CMD_READ,
        (page >> 8) & 0xFF,
        (page >> 0) & 0xFF,
        address};

    spi_write_blocking(inst->hw_inst, cmdbuf, 4);
    spi_read_blocking(inst->hw_inst, 0, buffer, length);
    cs_deselect(inst->cs_gpio);
}

void flash_spi_chip_erase(flash_spi *inst)
{
    cs_select(inst->cs_gpio);
    uint8_t cmd = FLASH_SPI_CMD_CHIP_ERASE;
    spi_write_blocking(inst->hw_inst, &cmd, 1);
    cs_select(inst->cs_gpio);
}

void flash_spi_sector_erase(flash_spi *inst, uint32_t address, uint32_t page)
{
    uint8_t cmdbuf[4] = {
        FLASH_SPI_CMD_SECTOR_ERASE,
        (page >> 8) & 0xFF,
        (page >> 0) & 0xFF,
        address};

    write_enable(inst->hw_inst, inst->cs_gpio);

    cs_select(inst->cs_gpio);
    spi_write_blocking(inst->hw_inst, cmdbuf, 4);
    cs_deselect(inst->cs_gpio);

    wait_done(inst->hw_inst, inst->cs_gpio);
}

void flash_spi_page_program(flash_spi *inst, uint32_t addr, uint32_t page, uint8_t data[])
{
    uint8_t cmdbuf[4] = {
        FLASH_SPI_CMD_PAGE_PROGRAM,
        (page >> 8) & 0xFF,
        (page >> 0) & 0xFF,
        addr};

    write_enable(inst->hw_inst, inst->cs_gpio);

    cs_select(inst->cs_gpio);
    spi_write_blocking(inst->hw_inst, cmdbuf, 4);
    spi_write_blocking(inst->hw_inst, data, FLASH_SPI_PAGE_SIZE);
    cs_deselect(inst->cs_gpio);

    wait_done(inst->hw_inst, inst->cs_gpio);
}

void printbuf(uint8_t buf[FLASH_SPI_PAGE_SIZE])
{
    for (int i = 0; i < FLASH_SPI_PAGE_SIZE; ++i)
    {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }
}