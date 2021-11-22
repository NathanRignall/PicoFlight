#include "mpu6050.h"

void mpu6050_reset(mpu6050 *inst)
{
    // Two byte reset. First byte register, second byte data
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(inst->hw_inst, inst->addr, buf, 2, false);
}

void mpu6050_read_raw(mpu6050 *inst, uint16_t accel[3], uint16_t gyro[3], uint16_t *temp)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(inst->hw_inst, inst->addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(inst->hw_inst, inst->addr, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(inst->hw_inst, inst->addr, &val, 1, true);
    i2c_read_blocking(inst->hw_inst, inst->addr, buffer, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        ;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(inst->hw_inst, inst->addr, &val, 1, true);
    i2c_read_blocking(inst->hw_inst, inst->addr, buffer, 2, false); // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
