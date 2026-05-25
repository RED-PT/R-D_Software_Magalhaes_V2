/**
 * @file INA219.h
 * @brief INA219 High-Side Current/Voltage/Power Monitor driver (I2C)
 * @author Tomas Teixeira
 * @date March 2026
 *
 * Driver for the TI INA219 on the Buzz V4 PCB.
 * Hardware configuration: 1 mOhm shunt resistor, I2C1, address 0x40.
 */

#ifndef SENSORS_INA219_INA219_H_
#define SENSORS_INA219_INA219_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

/**
 * @defgroup INA219_RegMap INA219 Register Map
 * @brief Internal register addresses of the INA219
 * @{
 */
#define INA219_REG_CONFIG        0x00  /**< Configuration register */
#define INA219_REG_SHUNT_VOLTAGE 0x01  /**< Shunt voltage register (read-only) */
#define INA219_REG_BUS_VOLTAGE   0x02  /**< Bus voltage register (read-only) */
#define INA219_REG_POWER         0x03  /**< Power register (read-only) */
#define INA219_REG_CURRENT       0x04  /**< Current register (read-only) */
#define INA219_REG_CALIBRATION   0x05  /**< Calibration register */
/** @} */

/**
 * @defgroup INA219_ConfigBits INA219 Configuration Register Bits
 * @brief Bit-field values for the INA219 configuration register
 * @{
 */
#define INA219_CONFIG_RESET              0x8000  /**< System reset bit */

/** @name Bus Voltage Range */
/** @{ */
#define INA219_CONFIG_BVOLTAGERANGE_16V  0x0000  /**< 16 V bus voltage range */
#define INA219_CONFIG_BVOLTAGERANGE_32V  0x2000  /**< 32 V bus voltage range */
/** @} */

/** @name PGA Gain (shunt voltage range) */
/** @{ */
#define INA219_CONFIG_GAIN_1_40MV        0x0000  /**< +/-40mV  -> +/-40A  @ 1 mOhm */
#define INA219_CONFIG_GAIN_2_80MV        0x0800  /**< +/-80mV  -> +/-80A  @ 1 mOhm */
#define INA219_CONFIG_GAIN_4_160MV       0x1000  /**< +/-160mV -> +/-160A @ 1 mOhm */
#define INA219_CONFIG_GAIN_8_320MV       0x1800  /**< +/-320mV -> +/-320A @ 1 mOhm */
/** @} */

/** @name ADC Resolution / Averaging */
/** @{ */
#define INA219_CONFIG_BADCRES_12BIT      0x0180  /**< Bus ADC: 12-bit, single sample */
#define INA219_CONFIG_SADCRES_12BIT      0x0018  /**< Shunt ADC: 12-bit, single sample */
/** @} */

/** @name Operating Mode */
/** @{ */
#define INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS 0x0007  /**< Continuous shunt + bus measurement */
/** @} */
/** @} */

/**
 * @defgroup INA219_Constants INA219 Device Constants
 * @brief Fixed hardware constants for the Buzz V4 PCB
 * @{
 */
#define INA219_I2C_ADDR          0x40    /**< 7-bit I2C address (A0=GND, A1=GND) */
#define INA219_SHUNT_OHMS        0.001f  /**< Shunt resistor value: 1 mOhm */
#define INA219_SHUNT_LSB_UV      10.0f   /**< Shunt voltage LSB: 10 uV per bit */
#define INA219_BUS_LSB_MV        4.0f    /**< Bus voltage LSB: 4 mV per bit */
/** @} */

/**
 * @defgroup INA219_API INA219 Driver API
 * @brief Public types and functions for the INA219 driver
 * @{
 */

/**
 * @brief INA219 driver context and latest readings
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;    /**< @brief HAL I2C peripheral handle */
    uint8_t i2c_addr;           /**< @brief 7-bit address shifted left for HAL */
    float current_lsb;          /**< @brief Amps per LSB (set by calibration) */
    float power_lsb;            /**< @brief Watts per LSB (20 x current_lsb) */

    float shunt_voltage_mv;     /**< @brief Latest shunt voltage reading in mV */
    float bus_voltage_v;        /**< @brief Latest bus voltage reading in V */
    float current_a;            /**< @brief Latest current reading in A */
    float power_w;              /**< @brief Latest power reading in W */
    bool overflow;              /**< @brief Math overflow flag from bus voltage register */
    bool valid;                 /**< @brief true if last read completed successfully */
} INA219_t;

/**
 * @brief Initialize INA219 and set calibration for 1 mOhm shunt
 * @param[in,out] dev  Pointer to driver context to initialize
 * @param[in]     hi2c HAL I2C peripheral handle
 * @return true if the device responds and is configured successfully
 * @return false on I2C communication error
 */
bool INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief Read all measurements (shunt voltage, bus voltage, current, power)
 * @param[in,out] dev Pointer to driver context (readings stored in struct fields)
 * @return true if all register reads succeeded
 * @return false on I2C communication error (dev->valid set to false)
 */
bool INA219_ReadAll(INA219_t *dev);

/**
 * @brief Software reset the INA219
 * @param[in,out] dev Pointer to driver context
 * @return true if reset command was acknowledged
 * @return false on I2C communication error
 */
bool INA219_Reset(INA219_t *dev);

/** @} */

#endif /* SENSORS_INA219_INA219_H_ */
