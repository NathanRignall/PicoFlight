#include "flight_data.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flash_spi.h"

#include "hw_config.h"

flight_data_system::flight_data_system(flash_spi *flash_spi_inst)
{
    // setup system vars
    has_sd_init = false;
    flash_spi_local_inst = flash_spi_inst;

    // setup flight data vars
    data = (flight_data *)malloc(sizeof(flight_data));

    // if flash not backed up back it up!
    flash_spi_local_inst->page = 0;
    flash_spi_read(flash_spi_local_inst, flash_spi_local_inst->page, flash_spi_local_inst->page_buf, FLASH_SPI_PAGE_SIZE);

    printf("[INFO] SAVE STATUS %d \n", flash_spi_local_inst->page_buf[0]);
    if (flash_spi_local_inst->page_buf[0] != 2)
    {
        printf("[INFO] NEED TO WRITE \n");
        save_all_to_sd();
    }

    // always reset the flash to begin writing
    reset_flash();
}

void flight_data_system::save_to_flash()
{
    // check if hit max flash on current chip
    if (flash_spi_local_inst->page >= FLASH_SPI_PAGES)
    {
        printf("[ERROR] RUN OUT OF FLASH! \n");
    }
    else
    {  
        // copy flight data save to array buffer
        memcpy (flash_spi_local_inst->page_buf, data, sizeof(flight_data));

        // set incramental save byte
        flash_spi_local_inst->page_buf[255] = uint8_t(flash_spi_local_inst->page);

        // write buffer to a page of flash
        flash_spi_page_program(flash_spi_local_inst, flash_spi_local_inst->page, flash_spi_local_inst->page_buf);

        flash_spi_local_inst->page++;
    }
}

void flight_data_system::init_sd()
{
    // init the sd card
    if (!sd_init_driver())
    {
        puts("ERROR: Could not initialize SD card\r\n");
        while (true)
            ;
    }

    // mount the sd card
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        while (true)
            ;
    }

    has_sd_init = true;
    printf("[DONE] INIT SD \n");
}

void flight_data_system::save_all_to_sd()
{
    // only init sd if need to
    if (has_sd_init == false)
    {
        init_sd();
    }

    // Open file for writing
    fr = f_open(&fil, "main.csv", FA_WRITE | FA_CREATE_ALWAYS);
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

    flash_spi_local_inst->page = 1;

    while (flash_spi_local_inst->page <= FLASH_SPI_PAGES)
    {
        // read a page of flash
        flash_spi_read(flash_spi_local_inst, flash_spi_local_inst->page, flash_spi_local_inst->page_buf, FLASH_SPI_PAGE_SIZE);

        // check incramental save byte
        if (flash_spi_local_inst->page_buf[255] != uint8_t(flash_spi_local_inst->page))
        {
            printf("[INFO] FINISHED SD AT - %d - %d \n", flash_spi_local_inst->page_buf[255], flash_spi_local_inst->page);
            break;
        }

        // copy buffer to flight data save
        memcpy (data, flash_spi_local_inst->page_buf, sizeof(flight_data));

        // print flight data for debuging
        printf("SAVE   Time. N = %d   Acc. X = %f, Y = %f, Z = %f   Gyro. X = %f, Y = %f, Z = %f\n", (int)data->system_clock_now, data->acceleration[0], data->acceleration[1],
               data->acceleration[2], data->gyroscope[0], data->gyroscope[1], data->gyroscope[2]);

        // save flight data to sd
        ret = f_printf(&fil, "%d,%d,%f,%f,%f,%f,%f,%f \n", flash_spi_local_inst->page, (int)data->system_clock_now, data->acceleration[0], data->acceleration[1],
                       data->acceleration[2], data->gyroscope[0], data->gyroscope[1], data->gyroscope[2]);

        flash_spi_local_inst->page++;
    }

    // Close file
    fr = f_close(&fil);
    if (fr != FR_OK)
    {
        printf("ERROR: Could not close file (%d)\r\n", fr);
        while (true)
            ;
    }

    // once saved to sd set flash page 1 to false
    flash_spi_local_inst->page = 0;
    flash_spi_sector_erase(flash_spi_local_inst, flash_spi_local_inst->page);
    flash_spi_local_inst->page_buf[0] = 2;
    flash_spi_page_program(flash_spi_local_inst, flash_spi_local_inst->page, flash_spi_local_inst->page_buf);

    printf("[DONE] SAVING TO SD + FLASH MARK \n");
}

void flight_data_system::reset_flash()
{
    // erase the whole chip
    flash_spi_chip_erase(flash_spi_local_inst);

    // write the first packet
    flash_spi_local_inst->page = 0;
    flash_spi_local_inst->page_buf[0] = 1;
    flash_spi_page_program(flash_spi_local_inst, flash_spi_local_inst->page, flash_spi_local_inst->page_buf);

    // increase flash to starting point
    flash_spi_local_inst->page = 1;

    printf("[INFO] RESET FLASH \n");
}

void flight_data_system::unmount_sd()
{
    f_unmount("0:");
    has_sd_init = false;

    printf("[INFO] UNMOUNT SD \n");
}
