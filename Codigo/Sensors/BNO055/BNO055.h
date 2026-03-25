/**
 * @file BNO055.h
 * @brief BNO055 9-DOF absolute orientation IMU driver for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This driver provides an interface to the Bosch BNO055 9-axis absolute
 * orientation sensor with integrated sensor fusion.
 *
 * @section bno_features Features
 * - 3-axis accelerometer, gyroscope, and magnetometer
 * - On-chip sensor fusion (NDOF mode)
 * - Euler angles output (heading, roll, pitch)
 * - Quaternion output
 * - Linear acceleration (gravity removed)
 * - I2C interface up to 400 kHz
 * - Self-calibration capability
 *
 * @section bno_modes Operation Modes
 * | Mode | Description |
 * |------|-------------|
 * | CONFIGMODE | Configuration mode |
 * | IMU | Accel + Gyro fusion |
 * | COMPASS | Accel + Mag fusion |
 * | NDOF | Full 9-DOF fusion (recommended) |
 *
 * @section bno_calibration Calibration
 * The BNO055 performs automatic calibration. Calibration status is available
 * through the calibration_status field (0-3 for each sensor, 3=fully calibrated).
 *
 * @see BNO_t for output data structure
 */

#ifndef SENSORS_BNO055_BNO055_H_
#define SENSORS_BNO055_BNO055_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "defs.h"

/**
 * @defgroup BNORegisters Register Addresses
 * @brief BNO055 register map (partial)
 * @{
 */
#define BNO055_REG_CHIP_ID           0x00    /**< Chip ID register (0xA0) */
#define BNO055_REG_ACCEL_X_LSB       0x08    /**< Accelerometer X LSB */
#define BNO055_REG_ACCEL_X_MSB       0x09    /**< Accelerometer X MSB */
#define BNO055_REG_ACCEL_Y_LSB       0x0A    /**< Accelerometer Y LSB */
#define BNO055_REG_ACCEL_Y_MSB       0x0B    /**< Accelerometer Y MSB */
#define BNO055_REG_ACCEL_Z_LSB       0x0C    /**< Accelerometer Z LSB */
#define BNO055_REG_ACCEL_Z_MSB       0x0D    /**< Accelerometer Z MSB */

#define BNO055_REG_GYRO_X_LSB        0x14    /**< Gyroscope X LSB */
#define BNO055_REG_GYRO_X_MSB        0x15    /**< Gyroscope X MSB */
#define BNO055_REG_GYRO_Y_LSB        0x16    /**< Gyroscope Y LSB */
#define BNO055_REG_GYRO_Y_MSB        0x17    /**< Gyroscope Y MSB */
#define BNO055_REG_GYRO_Z_LSB        0x18    /**< Gyroscope Z LSB */
#define BNO055_REG_GYRO_Z_MSB        0x19    /**< Gyroscope Z MSB */

#define BNO055_REG_MAG_X_LSB         0x0E    /**< Magnetometer X LSB */
#define BNO055_REG_MAG_X_MSB         0x0F    /**< Magnetometer X MSB */
#define BNO055_REG_MAG_Y_LSB         0x10    /**< Magnetometer Y LSB */
#define BNO055_REG_MAG_Y_MSB         0x11    /**< Magnetometer Y MSB */
#define BNO055_REG_MAG_Z_LSB         0x12    /**< Magnetometer Z LSB */
#define BNO055_REG_MAG_Z_MSB         0x13    /**< Magnetometer Z MSB */

#define BNO055_REG_EULER_H_LSB       0x1A    /**< Euler heading (yaw) LSB */
#define BNO055_REG_EULER_H_MSB       0x1B    /**< Euler heading (yaw) MSB */
#define BNO055_REG_EULER_R_LSB       0x1C    /**< Euler roll LSB */
#define BNO055_REG_EULER_R_MSB       0x1D    /**< Euler roll MSB */
#define BNO055_REG_EULER_P_LSB       0x1E    /**< Euler pitch LSB */
#define BNO055_REG_EULER_P_MSB       0x1F    /**< Euler pitch MSB */

#define BNO055_REG_QUAT_W_LSB        0x20    /**< Quaternion W LSB */
#define BNO055_REG_QUAT_W_MSB        0x21    /**< Quaternion W MSB */
#define BNO055_REG_QUAT_X_LSB        0x22    /**< Quaternion X LSB */
#define BNO055_REG_QUAT_X_MSB        0x23    /**< Quaternion X MSB */
#define BNO055_REG_QUAT_Y_LSB        0x24    /**< Quaternion Y LSB */
#define BNO055_REG_QUAT_Y_MSB        0x25    /**< Quaternion Y MSB */
#define BNO055_REG_QUAT_Z_LSB        0x26    /**< Quaternion Z LSB */
#define BNO055_REG_QUAT_Z_MSB        0x27    /**< Quaternion Z MSB */

#define BNO055_REG_LIA_X_LSB         0x28    /**< Linear acceleration X LSB */
#define BNO055_REG_LIA_X_MSB         0x29    /**< Linear acceleration X MSB */
#define BNO055_REG_LIA_Y_LSB         0x2A    /**< Linear acceleration Y LSB */
#define BNO055_REG_LIA_Y_MSB         0x2B    /**< Linear acceleration Y MSB */
#define BNO055_REG_LIA_Z_LSB         0x2C    /**< Linear acceleration Z LSB */
#define BNO055_REG_LIA_Z_MSB         0x2D    /**< Linear acceleration Z MSB */

#define BNO055_REG_GRAVITY_X_LSB     0x2E    /**< Gravity vector X LSB */
#define BNO055_REG_GRAVITY_X_MSB     0x2F    /**< Gravity vector X MSB */
#define BNO055_REG_GRAVITY_Y_LSB     0x30    /**< Gravity vector Y LSB */
#define BNO055_REG_GRAVITY_Y_MSB     0x31    /**< Gravity vector Y MSB */
#define BNO055_REG_GRAVITY_Z_LSB     0x32    /**< Gravity vector Z LSB */
#define BNO055_REG_GRAVITY_Z_MSB     0x33    /**< Gravity vector Z MSB */

#define BNO055_REG_TEMP              0x34    /**< Temperature register */

#define BNO055_REG_CALIB_STAT        0x35    /**< Calibration status register */
#define BNO055_REG_SYS_STATUS        0x39    /**< System status register */
#define BNO055_REG_OPR_MODE          0x3D    /**< Operation mode register */
#define BNO055_REG_PWR_MODE          0x3E    /**< Power mode register */
#define BNO055_REG_SYS_TRIGGER       0x3F    /**< System trigger register */
/** @} */

/**
 * @defgroup BNOModes Operation Mode Constants
 * @brief BNO055 operation mode values for OPR_MODE register
 * @{
 */
#define BNO055_MODE_CONFIGMODE       0x00    /**< Configuration mode */
#define BNO055_MODE_ACCONLY          0x01    /**< Accelerometer only */
#define BNO055_MODE_MAGONLY          0x02    /**< Magnetometer only */
#define BNO055_MODE_GYROONLY         0x03    /**< Gyroscope only */
#define BNO055_MODE_ACCMAG           0x04    /**< Accelerometer + Magnetometer */
#define BNO055_MODE_ACCGYRO          0x05    /**< Accelerometer + Gyroscope */
#define BNO055_MODE_MAGGYRO          0x06    /**< Magnetometer + Gyroscope */
#define BNO055_MODE_AMG              0x07    /**< All sensors, no fusion */
#define BNO055_MODE_IMU              0x08    /**< Accel + Gyro fusion */
#define BNO055_MODE_COMPASS          0x09    /**< Accel + Mag fusion */
#define BNO055_MODE_M4G              0x0A    /**< Mag for gyro */
#define BNO055_MODE_NDOF_FMC_OFF     0x0B    /**< NDOF, fast mag cal off */
#define BNO055_MODE_NDOF             0x0C    /**< Full 9-DOF fusion (recommended) */
/** @} */

/**
 * @defgroup BNOPower Power Mode Constants
 * @brief BNO055 power mode values
 * @{
 */
#define BNO055_POWER_NORMAL          0x00    /**< Normal power mode */
#define BNO055_POWER_LOWPOWER        0x01    /**< Low power mode */
#define BNO055_POWER_SUSPEND         0x02    /**< Suspend mode */
/** @} */

/**
 * @defgroup BNOCalib Calibration Status Masks
 * @brief Bit masks for calibration status register
 * @{
 */
#define BNO055_CALIB_SYS_MASK        0xC0    /**< System calibration (bits 7:6) */
#define BNO055_CALIB_GYRO_MASK       0x30    /**< Gyroscope calibration (bits 5:4) */
#define BNO055_CALIB_ACCEL_MASK      0x0C    /**< Accelerometer calibration (bits 3:2) */
#define BNO055_CALIB_MAG_MASK        0x03    /**< Magnetometer calibration (bits 1:0) */
/** @} */

/**
 * @defgroup BNOConstants Device Constants
 * @brief Device identification and configuration constants
 * @{
 */
#define BNO055_I2C_ADDR              0x28    /**< Default I2C address (7-bit) */
#define BNO055_CHIP_ID_VALUE         0xA0    /**< Expected CHIP_ID register value */
/** @} */

/**
 * @brief BNO055 driver context
 *
 * Contains all state for a single BNO055 device instance.
 * Stores I2C handle, DMA buffers, and parsed sensor data.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;        /**< I2C peripheral handle */
    uint8_t i2c_addr;               /**< Device I2C address (7-bit) */
    uint8_t read_buffer[20];        /**< DMA buffer for burst reads */
    volatile uint8_t data_ready;    /**< Flag set when new data available */

    /* Raw sensor data */
    int16_t accel_raw[3];           /**< Raw accelerometer X,Y,Z */
    int16_t gyro_raw[3];            /**< Raw gyroscope X,Y,Z */
    int16_t mag_raw[3];             /**< Raw magnetometer X,Y,Z */
    int16_t euler_raw[3];           /**< Raw Euler: heading, roll, pitch */
    int16_t quat_raw[4];            /**< Raw quaternion: W, X, Y, Z */
    int16_t lia_raw[3];             /**< Raw linear acceleration X,Y,Z */
    int16_t grav_raw[3];            /**< Raw gravity vector X,Y,Z */
    uint8_t temp_raw;               /**< Raw temperature */

    uint8_t calib_status;           /**< Current calibration status byte */
} BNO055_t;

/**
 * @defgroup BNOAPI BNO055 Public API
 * @brief Driver functions for BNO055 9-DOF IMU
 * @{
 */

/**
 * @brief Initialize BNO055 driver
 *
 * Initializes the driver context and verifies communication.
 * Reads the CHIP_ID register to confirm device presence.
 *
 * @param[out] dev  Pointer to driver context to initialize
 * @param[in]  hi2c I2C peripheral handle
 *
 * @return true if initialization successful and device responds
 * @return false if communication error or wrong chip ID
 */
bool BNO055_Init(BNO055_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief Configure BNO055 operating parameters
 *
 * Sets up the sensor with:
 * - NDOF mode (full 9-DOF fusion)
 * - Normal power mode
 * - Default units (degrees, m/s^2)
 *
 * @param[in] dev Pointer to initialized driver context
 *
 * @return true if configuration successful
 * @return false if communication error
 *
 * @note Sensor requires ~650ms to switch to NDOF mode
 */
bool BNO055_Configure(BNO055_t *dev);

/**
 * @brief Start DMA read of all sensor data
 *
 * Initiates non-blocking I2C DMA transfer to read orientation
 * and sensor data registers.
 *
 * @param[in] dev Pointer to driver context
 *
 * @return true if DMA transfer started
 * @return false if I2C busy or error
 */
bool BNO055_StartReadDMA(BNO055_t *dev);

/**
 * @brief Process raw data into engineering units
 *
 * Converts raw sensor data to physical units and populates
 * the BNO_t output structure with:
 * - Euler angles (degrees)
 * - Quaternion (normalized)
 * - Acceleration (m/s^2 or mg)
 * - Angular rate (dps)
 * - Magnetic field (uT)
 *
 * @param[in]  dev    Pointer to driver context with valid raw data
 * @param[out] output Pointer to BNO_t structure to populate
 *
 * @return true if data processed successfully
 * @return false if invalid parameters
 */
bool BNO055_ProcessData(BNO055_t *dev, BNO_t *output);

/**
 * @brief Parse DMA buffer into raw values
 *
 * Called from DMA complete callback to extract raw values
 * from the receive buffer.
 *
 * @param[in,out] dev Pointer to driver context
 */
void BNO055_ParseDMABuffer(BNO055_t *dev);

/**
 * @brief Set operation mode
 *
 * Changes the sensor operation mode (e.g., NDOF, IMU, COMPASS).
 *
 * @param[in] dev  Pointer to driver context
 * @param[in] mode Operation mode constant (BNO055_MODE_*)
 *
 * @return true if mode change successful
 * @return false if communication error
 *
 * @note Mode changes require time to take effect
 */
bool BNO055_SetOpMode(BNO055_t *dev, uint8_t mode);

/**
 * @brief Read calibration status
 *
 * Reads the current calibration status for all sensors.
 * Each sensor reports 0-3 (3 = fully calibrated).
 *
 * @param[in]  dev          Pointer to driver context
 * @param[out] calib_status Pointer to receive calibration byte
 *
 * @return true if read successful
 * @return false if communication error
 *
 * @note Use BNO055_CALIB_*_MASK to extract individual values
 */
bool BNO055_GetCalibStatus(BNO055_t *dev, uint8_t *calib_status);

/** @} */ /* End of BNOAPI group */

#endif /* SENSORS_BNO055_BNO055_H_ */
