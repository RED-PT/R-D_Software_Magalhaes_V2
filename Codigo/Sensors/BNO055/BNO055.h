/*
 * BNO055.h
 *
 *  Created on: Oct 16, 2025
 *      Author: texman
 */

#ifndef SENSORS_BNO055_BNO055_H_
#define SENSORS_BNO055_BNO055_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "defs.h"

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "defs.h"

// Register addresses
#define BNO055_REG_CHIP_ID           0x00
#define BNO055_REG_ACCEL_X_LSB       0x08
#define BNO055_REG_ACCEL_X_MSB       0x09
#define BNO055_REG_ACCEL_Y_LSB       0x0A
#define BNO055_REG_ACCEL_Y_MSB       0x0B
#define BNO055_REG_ACCEL_Z_LSB       0x0C
#define BNO055_REG_ACCEL_Z_MSB       0x0D

#define BNO055_REG_GYRO_X_LSB        0x14
#define BNO055_REG_GYRO_X_MSB        0x15
#define BNO055_REG_GYRO_Y_LSB        0x16
#define BNO055_REG_GYRO_Y_MSB        0x17
#define BNO055_REG_GYRO_Z_LSB        0x18
#define BNO055_REG_GYRO_Z_MSB        0x19

#define BNO055_REG_MAG_X_LSB         0x0E
#define BNO055_REG_MAG_X_MSB         0x0F
#define BNO055_REG_MAG_Y_LSB         0x10
#define BNO055_REG_MAG_Y_MSB         0x11
#define BNO055_REG_MAG_Z_LSB         0x12
#define BNO055_REG_MAG_Z_MSB         0x13

#define BNO055_REG_EULER_H_LSB       0x1A  // Heading (Yaw)
#define BNO055_REG_EULER_H_MSB       0x1B
#define BNO055_REG_EULER_R_LSB       0x1C  // Roll
#define BNO055_REG_EULER_R_MSB       0x1D
#define BNO055_REG_EULER_P_LSB       0x1E  // Pitch
#define BNO055_REG_EULER_P_MSB       0x1F

#define BNO055_REG_QUAT_W_LSB        0x20  // Quaternion
#define BNO055_REG_QUAT_W_MSB        0x21
#define BNO055_REG_QUAT_X_LSB        0x22
#define BNO055_REG_QUAT_X_MSB        0x23
#define BNO055_REG_QUAT_Y_LSB        0x24
#define BNO055_REG_QUAT_Y_MSB        0x25
#define BNO055_REG_QUAT_Z_LSB        0x26
#define BNO055_REG_QUAT_Z_MSB        0x27

#define BNO055_REG_LIA_X_LSB         0x28  // Linear Acceleration
#define BNO055_REG_LIA_X_MSB         0x29
#define BNO055_REG_LIA_Y_LSB         0x2A
#define BNO055_REG_LIA_Y_MSB         0x2B
#define BNO055_REG_LIA_Z_LSB         0x2C
#define BNO055_REG_LIA_Z_MSB         0x2D

#define BNO055_REG_GRAVITY_X_LSB     0x2E
#define BNO055_REG_GRAVITY_X_MSB     0x2F
#define BNO055_REG_GRAVITY_Y_LSB     0x30
#define BNO055_REG_GRAVITY_Y_MSB     0x31
#define BNO055_REG_GRAVITY_Z_LSB     0x32
#define BNO055_REG_GRAVITY_Z_MSB     0x33

#define BNO055_REG_TEMP              0x34

#define BNO055_REG_CALIB_STAT        0x35
#define BNO055_REG_SYS_STATUS        0x39
#define BNO055_REG_OPR_MODE          0x3D
#define BNO055_REG_PWR_MODE          0x3E
#define BNO055_REG_SYS_TRIGGER       0x3F

// Operation modes
#define BNO055_MODE_CONFIGMODE       0x00
#define BNO055_MODE_ACCONLY          0x01
#define BNO055_MODE_MAGONLY          0x02
#define BNO055_MODE_GYROONLY         0x03
#define BNO055_MODE_ACCMAG           0x04
#define BNO055_MODE_ACCGYRO          0x05
#define BNO055_MODE_MAGGYRO          0x06
#define BNO055_MODE_AMG              0x07
#define BNO055_MODE_IMU              0x08
#define BNO055_MODE_COMPASS          0x09
#define BNO055_MODE_M4G              0x0A
#define BNO055_MODE_NDOF_FMC_OFF     0x0B
#define BNO055_MODE_NDOF             0x0C  // Full 9-DOF fusion

// Power modes
#define BNO055_POWER_NORMAL          0x00
#define BNO055_POWER_LOWPOWER        0x01
#define BNO055_POWER_SUSPEND         0x02

// Calibration status bits
#define BNO055_CALIB_SYS_MASK        0xC0
#define BNO055_CALIB_GYRO_MASK       0x30
#define BNO055_CALIB_ACCEL_MASK      0x0C
#define BNO055_CALIB_MAG_MASK        0x03

// Default I2C address
#define BNO055_I2C_ADDR              0x28
#define BNO055_CHIP_ID_VALUE         0xA0

// Driver context
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;
    uint8_t read_buffer[20];  // DMA buffer for readings
    volatile uint8_t data_ready;

    // Raw sensor data
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    int16_t euler_raw[3];      // Heading, Roll, Pitch
    int16_t quat_raw[4];       // W, X, Y, Z
    int16_t lia_raw[3];        // Linear acceleration
    int16_t grav_raw[3];       // Gravity vector
    uint8_t temp_raw;

    uint8_t calib_status;      // Calibration status
} BNO055_t;

// Public API
bool BNO055_Init(BNO055_t *dev, I2C_HandleTypeDef *hi2c);
bool BNO055_Configure(BNO055_t *dev);
bool BNO055_StartReadDMA(BNO055_t *dev);
bool BNO055_ProcessData(BNO055_t *dev, BNO_t *output);

// For DMA callback integration
void BNO055_ParseDMABuffer(BNO055_t *dev);

// Helper functions
bool BNO055_SetOpMode(BNO055_t *dev, uint8_t mode);
bool BNO055_GetCalibStatus(BNO055_t *dev, uint8_t *calib_status);

#endif /* SENSORS_BNO055_BNO055_H_ */
