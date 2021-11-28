#include "flash_spi.h"

#include "hw_config.h"

struct flight_data
{
    uint64_t system_clock_now;
    uint64_t system_clock_end;
    uint64_t system_clock_dt;
    uint32_t acceleration[3];
    uint32_t gyroscope[3];
    uint32_t temperature;
};

class flight_data_system
{

public:
    flight_data *data;

    flight_data_system(flash_spi *flash_spi_inst);
    void save_to_flash();
    void save_all_to_sd();
    void init_sd();
    void unmount_sd();

private:
    void reset_flash();

    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    bool has_sd_init;

    //flash chip
    flash_spi *flash_spi_local_inst;
};