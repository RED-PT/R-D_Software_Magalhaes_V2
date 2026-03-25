/*
 * INA219.h
 *
 * INA219 High-Side Current/Voltage/Power Monitor (I2C)
 * Buzz V4 PCB: 1mΩ shunt resistor, I2C1, address 0x40
 *
 *  Created on: Mar 25, 2026
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_INA219_INA219_H_
#define SENSORS_INA219_INA219_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

// ============================================================================
// Register Map
// ============================================================================
#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

// ============================================================================
// Configuration Register Bits
// ============================================================================
#define INA219_CONFIG_RESET              0x8000

// Bus voltage range
#define INA219_CONFIG_BVOLTAGERANGE_16V  0x0000
#define INA219_CONFIG_BVOLTAGERANGE_32V  0x2000

// PGA gain (shunt voltage range)
#define INA219_CONFIG_GAIN_1_40MV        0x0000  // ±40mV  → ±40A  @ 1mΩ
#define INA219_CONFIG_GAIN_2_80MV        0x0800  // ±80mV  → ±80A  @ 1mΩ
#define INA219_CONFIG_GAIN_4_160MV       0x1000  // ±160mV → ±160A @ 1mΩ
#define INA219_CONFIG_GAIN_8_320MV       0x1800  // ±320mV → ±320A @ 1mΩ

// ADC resolution / averaging (bus and shunt)
#define INA219_CONFIG_BADCRES_12BIT      0x0180  // 12-bit, single sample
#define INA219_CONFIG_SADCRES_12BIT      0x0018  // 12-bit, single sample

// Operating mode
#define INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS 0x0007

// ============================================================================
// Device Constants
// ============================================================================
#define INA219_I2C_ADDR          0x40    // A0=GND, A1=GND

// Shunt resistor on Buzz V4 PCB
#define INA219_SHUNT_OHMS        0.001f  // 1 mΩ

// LSB values from datasheet
#define INA219_SHUNT_LSB_UV      10.0f   // 10 µV per LSB
#define INA219_BUS_LSB_MV        4.0f    // 4 mV per LSB

// ============================================================================
// Driver Context
// ============================================================================
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;           // 7-bit address shifted left
    float current_lsb;          // Amps per LSB (set by calibration)
    float power_lsb;            // Watts per LSB (20 × current_lsb)

    // Latest readings
    float shunt_voltage_mv;     // Shunt voltage in mV
    float bus_voltage_v;        // Bus voltage in V
    float current_a;            // Current in A
    float power_w;              // Power in W
    bool overflow;              // Math overflow flag
    bool valid;                 // Last read was successful
} INA219_t;

// ============================================================================
// Public API
// ============================================================================

/**
 * @brief Initialize INA219 and set calibration for 1mΩ shunt
 * @param dev Driver context
 * @param hi2c I2C handle
 * @return true if device responds and is configured
 */
bool INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief Read all measurements (shunt voltage, bus voltage, current, power)
 * @param dev Driver context
 * @return true if all reads succeeded
 */
bool INA219_ReadAll(INA219_t *dev);

/**
 * @brief Software reset the INA219
 * @param dev Driver context
 * @return true if reset command succeeded
 */
bool INA219_Reset(INA219_t *dev);

#endif /* SENSORS_INA219_INA219_H_ */
