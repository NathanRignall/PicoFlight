#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

#include "ff.h"

#include "hw_config.h"
#include "flash_spi.h"

#define SPI_0_PORT spi0
#define SPI_0_PIN_MISO 16
#define SPI_0_PIN_SCK 18
#define SPI_0_PIN_MOSI 19

#define SPI_FLASH_PIN_CS 17

#define I2C_0_PORT i2c1
#define I2C_0_PIN_SDA 14
#define I2C_0_PIN_SCL 15

static int MPU6050_ADDR = 0x68;

#define FLASH_PAGE_SIZE 256

#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_READ 0x03
#define FLASH_CMD_STATUS 0x05
#define FLASH_CMD_WRITE_EN 0x06
#define FLASH_CMD_SECTOR_ERASE 0x20

#define FLASH_STATUS_BUSY_MASK 0x01

static void mpu6050_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_0_PORT, MPU6050_ADDR, buf, 2, false);
}

static void mpu6050_read_raw(uint16_t accel[3], uint16_t gyro[3], uint16_t *temp)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_0_PORT, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_0_PORT, MPU6050_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_0_PORT, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_0_PORT, MPU6050_ADDR, buffer, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        ;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(I2C_0_PORT, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(I2C_0_PORT, MPU6050_ADDR, buffer, 2, false); // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

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

    // setup spi flash
    uint8_t page_buf[FLASH_PAGE_SIZE] = {0};

    uint16_t page = 0;

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
    mpu6050_reset();
    uint16_t acceleration[3], gyro[3], temp;

    //setup clock
    uint32_t bootTime;

    // init the flash chip
    flash_spi *flash_spi_inst_0;
    flash_spi_inst_0 = (flash_spi *)malloc(sizeof(flash_spi));

    flash_spi_inst_0->hw_inst = SPI_0_PORT;
    flash_spi_inst_0->cs_gpio = SPI_FLASH_PIN_CS;

    // turn on
    flash_spi_power_up(flash_spi_inst_0);

    sleep_ms(100);
    flash_spi_chip_erase(flash_spi_inst_0);
    sleep_ms(100);

    // while (1)
    // {
    //     flash_spi_sector_erase(flash_spi_inst_0, page);

    //     page = page + 1;

    //     //sleep_ms(5);

    //     if (page > (16384))
    //     {
    //         page = 0;
    //         break;
    //     }
    // }

    //flash_spi_chip_erase(flash_spi_inst_0);

    printf("START PROGRAM \n");

    while (1)
    {
        bootTime = to_ms_since_boot(get_absolute_time());

        mpu6050_read_raw(acceleration, gyro, &temp);

        //printf("%d   Acc. X = %d, Y = %d, Z = %d   Gyro. X = %d, Y = %d, Z = %d\n", bootTime, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);

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

        // page_buf[200] = uint8_t(101);
        // page_buf[201] = uint8_t(102);
        // page_buf[202] = uint8_t(103);
        // page_buf[203] = uint8_t(103);

        page_buf[240] = uint8_t(page);

        flash_spi_page_program(flash_spi_inst_0, page, page_buf);

        page = page + 1;

        //sleep_ms(5);

        if (page > (16384))
        {
            page = 0;
            break;
        }
    }

    for (int i = 0; i < FLASH_PAGE_SIZE; ++i)
        page_buf[i] = 0;

    printf("END READ\n");

    // Open file for writing ()
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

        flash_spi_read(flash_spi_inst_0, page, page_buf, FLASH_PAGE_SIZE);

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

        //printf("%d   Acc. X = %d, Y = %d, Z = %d   Gyro. X = %d, Y = %d, Z = %d\n", bootTime, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);

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

        //sleep_ms(5);

        if (page > (16384))
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
