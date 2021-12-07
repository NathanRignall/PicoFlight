#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

struct mpu6050
{
    i2c_inst_t *hw_inst;
    uint8_t addr;
};

void mpu6050_reset(mpu6050 *inst);
void mpu6050_read_data(mpu6050 *inst, long double accel[3], long double  gyro[3], float *temp);