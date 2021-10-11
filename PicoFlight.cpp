#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hw_config.h"
#include "ff.h"

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

void __not_in_flash_func(flash_read)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint32_t page, uint8_t *buf, size_t len)
{
    cs_select(cs_pin);
    uint8_t cmdbuf[4] = {
        FLASH_CMD_READ,
        (page >> 8) & 0xFF,
        (page >> 0) & 0xFF,
        addr};
    spi_write_blocking(spi, cmdbuf, 4);
    spi_read_blocking(spi, 0, buf, len);
    cs_deselect(cs_pin);
}

void __not_in_flash_func(flash_write_enable)(spi_inst_t *spi, uint cs_pin)
{
    cs_select(cs_pin);
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    spi_write_blocking(spi, &cmd, 1);
    cs_deselect(cs_pin);
}

void __not_in_flash_func(flash_wait_done)(spi_inst_t *spi, uint cs_pin)
{
    uint8_t status;
    do
    {
        cs_select(cs_pin);
        uint8_t buf[2] = {FLASH_CMD_STATUS, 0};
        spi_write_read_blocking(spi, buf, buf, 2);
        cs_deselect(cs_pin);
        status = buf[1];
    } while (status & FLASH_STATUS_BUSY_MASK);
}

void __not_in_flash_func(flash_sector_erase)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint32_t page)
{
    uint8_t cmdbuf[4] = {
        FLASH_CMD_SECTOR_ERASE,
        (page >> 8) & 0xFF,
        (page >> 0) & 0xFF,
        addr};
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void __not_in_flash_func(flash_page_program)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint32_t page, uint8_t data[])
{
    uint8_t cmdbuf[4] = {
        FLASH_CMD_PAGE_PROGRAM,
        (page >> 8) & 0xFF,
        (page >> 0) & 0xFF,
        addr};
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    spi_write_blocking(spi, data, FLASH_PAGE_SIZE);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void printbuf(uint8_t buf[FLASH_PAGE_SIZE])
{
    for (int i = 0; i < FLASH_PAGE_SIZE; ++i)
    {
        if (i % 16 == 15)
            printf("%02x\n", buf[i]);
        else
            printf("%02x ", buf[i]);
    }
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

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_0_PORT, 400 * 1000);

    gpio_set_function(I2C_0_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_0_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_0_PIN_SDA);
    gpio_pull_up(I2C_0_PIN_SCL);

    // setup spi flash
    uint8_t page_buf[FLASH_PAGE_SIZE] = {0};

    uint32_t target_addr = 0;
    uint32_t page = 0;

    while (1)
    {
        //flash_sector_erase(SPI_0_PORT, SPI_FLASH_PIN_CS, target_addr, page);

        page = page + 1;

        if (page > (128 * 8))
        {
            page = 0;
            break;
        }
    }

    // setup sd card
    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    char buf[100];
    char filename[] = "test04.txt";

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
        page_buf[8] = uint8_t(gyro[0] >> 8);
        page_buf[9] = uint8_t(gyro[0] & 0x00FF);
        page_buf[10] = uint8_t(gyro[0] >> 8);
        page_buf[11] = uint8_t(gyro[0] & 0x00FF);

        page_buf[12] = uint8_t(temp >> 8);
        page_buf[13] = uint8_t(temp & 0x00FF);

        page_buf[14] = uint8_t(bootTime >> 8);
        page_buf[15] = uint8_t(bootTime & 0x00FF);

        flash_page_program(SPI_0_PORT, SPI_FLASH_PIN_CS, target_addr, page, page_buf);

        page = page + 1;

        sleep_ms(50);

        if (page > (128 * 8))
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
    ret = f_printf(&fil, "page,boot,ax,ay,az,gx,gy,gz\n");
    if (ret < 0)
    {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
        while (true)
            ;
    }

    while (1)
    {

        flash_read(SPI_0_PORT, SPI_FLASH_PIN_CS, target_addr, page, page_buf, FLASH_PAGE_SIZE);

        acceleration[0] = (page_buf[0] << 8) | (page_buf[1] & 0x00FF);
        acceleration[1] = (page_buf[2] << 8) | (page_buf[3] & 0x00FF);
        acceleration[2] = (page_buf[4] << 8) | (page_buf[5] & 0x00FF);

        gyro[0] = (page_buf[6] << 8) | (page_buf[7] & 0xff);
        gyro[1] = (page_buf[8] << 8) | (page_buf[9] & 0xff);
        gyro[2] = (page_buf[10] << 8) | (page_buf[11] & 0xff);

        temp = (page_buf[12] << 8) | (page_buf[13] & 0xff);

        bootTime = (page_buf[14] << 8) | (page_buf[15] & 0xff);

        // Write something to file
        ret = f_printf(&fil, "%d,%d,%d,%d,%d,%d,%d,%d\n", page, bootTime, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);
        if (ret < 0)
        {
            printf("ERROR: Could not write to file (%d)\r\n", ret);
            f_close(&fil);
            while (true)
                ;
        }

        page = page + 1;

        //sleep_ms(100);

        if (page > (128 * 8))
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
