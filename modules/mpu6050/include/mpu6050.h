#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

struct mpu6050
{
    i2c_inst_t *hw_inst;
    uint8_t addr;
};

void mpu6050_reset(mpu6050 *inst);
void mpu6050_read_raw(mpu6050 *inst, uint16_t accel[3], uint16_t gyro[3], uint16_t *temp);