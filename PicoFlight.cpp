#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

#include "ff.h"

#include "hw_config.h"
#include "flash_spi.h"
#include "mpu6050.h"

#define SPI_0_PORT spi0
#define SPI_0_PIN_MISO 16
#define SPI_0_PIN_SCK 18
#define SPI_0_PIN_MOSI 19

#define SPI_FLASH_PIN_CS 17

#define I2C_0_PORT i2c1
#define I2C_0_PIN_SDA 14
#define I2C_0_PIN_SCL 15

#define RUN_TOTAL 1024

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_0_PORT, 1000 * 1000);
    gpio_set_function(SPI_0_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_0_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SPI_0_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_FLASH_PIN_CS, GPIO_FUNC_SIO);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(SPI_FLASH_PIN_CS, GPIO_OUT);
    gpio_put(SPI_FLASH_PIN_CS, 1);

    // /I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_0_PORT, 400 * 1000);

    gpio_set_function(I2C_0_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_0_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_0_PIN_SDA);
    gpio_pull_up(I2C_0_PIN_SCL);

    // setup sd card
    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    char buf[100];
    char filename[] = "test15.txt";

    if (!sd_init_driver())
    {
        puts("ERROR: Could not initialize SD card\r\n");
        while (true)
            ;
    }

    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        while (true)
            ;
    }

    // setup MPU6050
    mpu6050 *mpu6050_inst_0;
    mpu6050_inst_0 = (mpu6050 *)malloc(sizeof(mpu6050));
    mpu6050_inst_0->hw_inst = I2C_0_PORT;
    mpu6050_inst_0->addr = 0x68;

    mpu6050_reset(mpu6050_inst_0);
    uint16_t acceleration[3], gyro[3], temp;

    //setup clock
    uint32_t bootTime;

    // init the flash chip
    flash_spi *flash_spi_inst_0;
    flash_spi_inst_0 = (flash_spi *)malloc(sizeof(flash_spi));
    flash_spi_inst_0->hw_inst = SPI_0_PORT;
    flash_spi_inst_0->cs_gpio = SPI_FLASH_PIN_CS;

    flash_spi_chip_erase(flash_spi_inst_0);

    uint8_t page_buf[FLASH_SPI_PAGE_SIZE] = {0};
    uint16_t page = 0;

    printf("START PROGRAM \n");

    while (1)
    {
        bootTime = to_ms_since_boot(get_absolute_time());

        mpu6050_read_raw(mpu6050_inst_0, acceleration, gyro, &temp);

        printf("%d   Acc. X = %d, Y = %d, Z = %d   Gyro. X = %d, Y = %d, Z = %d\n", bootTime, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);

        // set the correct vars for data log
        page_buf[0] = uint8_t(acceleration[0] >> 8);
        page_buf[1] = uint8_t(acceleration[0] & 0x00FF);
        page_buf[2] = uint8_t(acceleration[1] >> 8);
        page_buf[3] = uint8_t(acceleration[1] & 0x00FF);
        page_buf[4] = uint8_t(acceleration[2] >> 8);
        page_buf[5] = uint8_t(acceleration[2] & 0x00FF);

        page_buf[6] = uint8_t(gyro[0] >> 8);
        page_buf[7] = uint8_t(gyro[0] & 0x00FF);
        page_buf[8] = uint8_t(gyro[1] >> 8);
        page_buf[9] = uint8_t(gyro[1] & 0x00FF);
        page_buf[10] = uint8_t(gyro[2] >> 8);
        page_buf[11] = uint8_t(gyro[2] & 0x00FF);

        page_buf[12] = uint8_t(temp >> 8);
        page_buf[13] = uint8_t(temp & 0x00FF);

        page_buf[20] = (bootTime >> 24) & 0xFF;
        page_buf[21] = (bootTime >> 16) & 0xFF;
        page_buf[22] = (bootTime >> 8) & 0xFF;
        page_buf[23] = bootTime & 0xFF;

        flash_spi_page_program(flash_spi_inst_0, page, page_buf);

        page = page + 1;

        if (page > (RUN_TOTAL))
        {
            page = 0;
            break;
        }
    }

    printf("END Record\n");

    // Open file for writing
    fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK)
    {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        while (true)
            ;
    }

    // Write something to file
    ret = f_printf(&fil, "page,a,b,c,d,boot,ax,ay,az,gx,gy,gz\n");
    if (ret < 0)
    {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        while (true)
            ;
    }

    while (1)
    {

        flash_spi_read(flash_spi_inst_0, page, page_buf, FLASH_SPI_PAGE_SIZE);

        acceleration[0] = (page_buf[0] << 8) | (page_buf[1] & 0x00FF);
        acceleration[1] = (page_buf[2] << 8) | (page_buf[3] & 0x00FF);
        acceleration[2] = (page_buf[4] << 8) | (page_buf[5] & 0x00FF);

        gyro[0] = (page_buf[6] << 8) | (page_buf[7] & 0xff);
        gyro[1] = (page_buf[8] << 8) | (page_buf[9] & 0xff);
        gyro[2] = (page_buf[10] << 8) | (page_buf[11] & 0xff);

        temp = (page_buf[12] << 8) | (page_buf[13] & 0xff);

        bootTime = page_buf[23];
        bootTime = bootTime | (page_buf[22] << 8);
        bootTime = bootTime | (page_buf[21] << 16);
        bootTime = bootTime | (page_buf[20] << 24);

        printf("%d   Acc. X = %d, Y = %d, Z = %d   Gyro. X = %d, Y = %d, Z = %d\n", bootTime, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);

        // Write page of flash to file
        ret = f_printf(&fil, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", page, page_buf[20], page_buf[21], page_buf[22], page_buf[23], bootTime, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);

        if (ret < 0)
        {
            printf("ERROR: Could not write to file (%d)\r\n", ret);
            f_close(&fil);
            while (true)
                ;
        }

        page = page + 1;

        if (page > (RUN_TOTAL))
        {
            page = 0;
            break;
        }
    }

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK)
    {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true)
            ;
    }

    // Unmount drive
    f_unmount("0:");

    printf("END PROGRAM\n");
}
