#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

#include "ff.h"

#include "flight_data.h"
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

#define RUN_TOTAL 100

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

    // setup any system hardware
    // init the flash chip
    flash_spi *flash_spi_inst_0;
    flash_spi_inst_0 = (flash_spi *)malloc(sizeof(flash_spi));
    flash_spi_inst_0->hw_inst = SPI_0_PORT;
    flash_spi_inst_0->cs_gpio = SPI_FLASH_PIN_CS;
    flash_spi_inst_0->page = 0;

    // init the flight data class
    flight_data_system flight(flash_spi_inst_0);

    // setup any sensors
    // setup MPU6050
    mpu6050 *mpu6050_inst_0;
    mpu6050_inst_0 = (mpu6050 *)malloc(sizeof(mpu6050));
    mpu6050_inst_0->hw_inst = I2C_0_PORT;
    mpu6050_inst_0->addr = 0x68;
    mpu6050_reset(mpu6050_inst_0);

    printf("START PROGRAM \n");

    int i;

    // main program loop
    while (i < 10)
    {
        flight.data->system_clock_now = to_ms_since_boot(get_absolute_time());

        mpu6050_read_data(mpu6050_inst_0, flight.data->acceleration, flight.data->gyroscope, &flight.data->temperature);

        flight.save_to_flash();

        printf("YO %d   Acc. X = %d, Y = %d, Z = %d   Gyro. X = %d, Y = %d, Z = %d\n", flight.data->system_clock_now, flight.data->acceleration[0], flight.data->acceleration[1], flight.data->acceleration[2], flight.data->gyroscope[0], flight.data->gyroscope[1], flight.data->gyroscope[2]);

        i++;
        sleep_ms(300);
    }

    //flight.save_all_to_sd();
}
