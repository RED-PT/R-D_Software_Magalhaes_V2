/*
 * BNO055.c
 *
 * Created on: Oct 16, 2025
 * Author: texman
 */

#include "BNO055.h"
#include <string.h>
#include <math.h>
#include "cmsis_os2.h"

// Private helper functions
static bool BNO055_ReadRegister(BNO055_t *dev, uint8_t reg, uint8_t *data);
static bool BNO055_WriteRegister(BNO055_t *dev, uint8_t reg, uint8_t data);

bool BNO055_Init(BNO055_t *dev, I2C_HandleTypeDef *hi2c) {
    uint8_t chip_id;

    if (!dev || !hi2c) {
        return false;
    }

    dev->hi2c = hi2c;
    dev->i2c_addr = BNO055_I2C_ADDR << 1;  // I2C addresses are shifted left by 1
    dev->data_ready = 0;
    memset(dev->read_buffer, 0, sizeof(dev->read_buffer));

    HAL_Delay(50);  // Power-up time

    // Read and verify chip ID
    if (!BNO055_ReadRegister(dev, BNO055_REG_CHIP_ID, &chip_id)) {
        return false;
    }

    if (chip_id != BNO055_CHIP_ID_VALUE) {
        return false;
    }

    return true;
}

bool BNO055_Configure(BNO055_t *dev) {
    if (!dev) {
        return false;
    }

    // Switch to CONFIG mode first
    if (!BNO055_SetOpMode(dev, BNO055_MODE_CONFIGMODE)) {
        return false;
    }
    HAL_Delay(25);

    // Set to NDOF mode (full 9-DOF sensor fusion)
    if (!BNO055_SetOpMode(dev, BNO055_MODE_NDOF)) {
        return false;
    }
    HAL_Delay(100);  // Mode change takes time

    // Set power mode to normal
    if (!BNO055_WriteRegister(dev, BNO055_REG_PWR_MODE, BNO055_POWER_NORMAL)) {
        return false;
    }
    HAL_Delay(10);

    return true;
}

bool BNO055_StartReadDMA(BNO055_t *dev) {
    if (!dev || !dev->hi2c) {
        return false;
    }

    // Read euler angles + quaternion + linear accel
    // Starting from EULER_H_LSB (0x1A) for 20 bytes total
    // This covers: Euler(6) + Quat(8) + LinAccel(6)

    uint8_t start_reg = BNO055_REG_EULER_H_LSB;
    uint8_t num_bytes = 20;  // Euler(6) + Quat(8) + LinAccel(6)

    // Prepare command: I2C address write, then register address, then read
    if (HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->i2c_addr, start_reg, I2C_MEMADD_SIZE_8BIT, dev->read_buffer, num_bytes) != HAL_OK) {
    	printf("ERROR123\r\n");
    	return false;
    }

    return true;
}

void BNO055_ParseDMABuffer(BNO055_t *dev) {
    if (!dev) {
        return;
    }

    // Buffer layout (starting from EULER_H_LSB):
    // [0:1] Heading, [2:3] Roll, [4:5] Pitch
    // [6:7] Quat W, [8:9] Quat X, [10:11] Quat Y, [12:13] Quat Z
    // [14:15] LinAccel X, [16:17] LinAccel Y, [18:19] LinAccel Z

    dev->euler_raw[0] = (int16_t)((dev->read_buffer[1] << 8) | dev->read_buffer[0]);  // Heading
    dev->euler_raw[1] = (int16_t)((dev->read_buffer[3] << 8) | dev->read_buffer[2]);  // Roll
    dev->euler_raw[2] = (int16_t)((dev->read_buffer[5] << 8) | dev->read_buffer[4]);  // Pitch

    dev->quat_raw[0] = (int16_t)((dev->read_buffer[7] << 8) | dev->read_buffer[6]);   // W
    dev->quat_raw[1] = (int16_t)((dev->read_buffer[9] << 8) | dev->read_buffer[8]);   // X
    dev->quat_raw[2] = (int16_t)((dev->read_buffer[11] << 8) | dev->read_buffer[10]); // Y
    dev->quat_raw[3] = (int16_t)((dev->read_buffer[13] << 8) | dev->read_buffer[12]); // Z

    dev->lia_raw[0] = (int16_t)((dev->read_buffer[15] << 8) | dev->read_buffer[14]);  // X
    dev->lia_raw[1] = (int16_t)((dev->read_buffer[17] << 8) | dev->read_buffer[16]);  // Y
    dev->lia_raw[2] = (int16_t)((dev->read_buffer[19] << 8) | dev->read_buffer[18]);  // Z

    dev->data_ready = 1;
}

bool BNO055_ProcessData(BNO055_t *dev, BNO_t *output) {
    if (!dev || !output || !dev->data_ready) {
        return false;
    }

    // Convert Euler angles (16-bit signed, 1/16 degree per LSB)
    output->heading_deg = (float)dev->euler_raw[0] / 16.0f;
    output->roll_deg = (float)dev->euler_raw[1] / 16.0f;
    output->pitch_deg = (float)dev->euler_raw[2] / 16.0f;

    // Normalize heading to 0-360
    if (output->heading_deg < 0) {
        output->heading_deg += 360.0f;
    }

    // Convert Quaternion (16-bit signed, 1/16384 per LSB)
    output->quat_w = (float)dev->quat_raw[0] / 16384.0f;
    output->quat_x = (float)dev->quat_raw[1] / 16384.0f;
    output->quat_y = (float)dev->quat_raw[2] / 16384.0f;
    output->quat_z = (float)dev->quat_raw[3] / 16384.0f;

    // Linear Acceleration (16-bit signed, 1/100 m/s^2 per LSB)
    output->accel_x_mg = (float)dev->lia_raw[0] / 100.0f * 1000.0f;  // Convert to mg
    output->accel_y_mg = (float)dev->lia_raw[1] / 100.0f * 1000.0f;
    output->accel_z_mg = (float)dev->lia_raw[2] / 100.0f * 1000.0f;

    // Gyro, Mag, Raw accel would need separate reads for full data
    // For now, setting placeholders
    output->gyro_x_dps = 0.0f;
    output->gyro_y_dps = 0.0f;
    output->gyro_z_dps = 0.0f;

    output->mag_x_uT = 0.0f;
    output->mag_y_uT = 0.0f;
    output->mag_z_uT = 0.0f;

    // Get calibration status - **REMOVIDO** para evitar bloqueio do I2C
    // BNO055_GetCalibStatus(dev, &dev->calib_status);
    // output->calibration_status = (dev->calib_status >> 6) & 0x03;
    output->calibration_status = 0; // Placeholder

    // Temperature (1 byte, 1°C per LSB)
    output->timestamp_ms = HAL_GetTick();

    dev->data_ready = 0;

    // **BLOCO DE CÓDIGO INCORRETO REMOVIDO DAQUI**

    return true;
}

bool BNO055_SetOpMode(BNO055_t *dev, uint8_t mode) {
    if (!dev) {
        return false;
    }

    return BNO055_WriteRegister(dev, BNO055_REG_OPR_MODE, mode);
}

bool BNO055_GetCalibStatus(BNO055_t *dev, uint8_t *calib_status) {
    if (!dev || !calib_status) {
        return false;
    }

    return BNO055_ReadRegister(dev, BNO055_REG_CALIB_STAT, calib_status);
}

// Private Functions

static bool BNO055_ReadRegister(BNO055_t *dev, uint8_t reg, uint8_t *data) {
    if (!dev || !data) {
        return false;
    }

    if (HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg,
                         I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    return true;
}

static bool BNO055_WriteRegister(BNO055_t *dev, uint8_t reg, uint8_t data) {
    if (!dev) {
        return false;
    }

    if (HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, reg,
                          I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    return true;
}
