/*
 * INA219.c
 *
 * INA219 High-Side Current/Voltage/Power Monitor (I2C)
 *
 *  Created on: Mar 25, 2026
 *      Author: Tomas Teixeira
 */

#include "INA219.h"

#define INA219_TIMEOUT_MS  50

// ============================================================================
// Low-level I2C helpers (blocking, 16-bit register access)
// ============================================================================

static bool INA219_WriteReg(INA219_t *dev, uint8_t reg, uint16_t value) {
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;  // MSB first
    data[1] = value & 0xFF;
    return HAL_I2C_Mem_Write(dev->hi2c, dev->i2c_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2, INA219_TIMEOUT_MS) == HAL_OK;
}

static bool INA219_ReadReg(INA219_t *dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(dev->hi2c, dev->i2c_addr, reg,
                         I2C_MEMADD_SIZE_8BIT, data, 2, INA219_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *value = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

// ============================================================================
// Public API
// ============================================================================

bool INA219_Init(INA219_t *dev, I2C_HandleTypeDef *hi2c) {
    if (!dev || !hi2c) return false;

    dev->hi2c = hi2c;
    dev->i2c_addr = INA219_I2C_ADDR << 1;
    dev->valid = false;

    // Reset device
    if (!INA219_WriteReg(dev, INA219_REG_CONFIG, INA219_CONFIG_RESET)) {
        return false;
    }
    HAL_Delay(5);

    // Configure: 32V range, PGA ÷8 (±320mV → ±320A @ 1mΩ), 12-bit, continuous
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV |
                      INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT |
                      INA219_CONFIG_MODE_SHUNT_BUS_CONTINUOUS;

    if (!INA219_WriteReg(dev, INA219_REG_CONFIG, config)) {
        return false;
    }

    // Calibration register calculation:
    //   Current_LSB = Max_Expected_Current / 2^15
    //   For 320A max: Current_LSB = 320 / 32768 ≈ 0.009766 A/LSB
    //   Round up to 0.01 A/LSB (10 mA) for clean numbers
    //
    //   Cal = trunc(0.04096 / (Current_LSB × R_shunt))
    //       = trunc(0.04096 / (0.01 × 0.001))
    //       = trunc(4096)
    //       = 4096
    dev->current_lsb = 0.01f;         // 10 mA per LSB
    dev->power_lsb = 20.0f * 0.01f;   // 200 mW per LSB

    uint16_t cal = 4096;
    if (!INA219_WriteReg(dev, INA219_REG_CALIBRATION, cal)) {
        return false;
    }

    return true;
}

bool INA219_ReadAll(INA219_t *dev) {
    uint16_t raw;
    dev->valid = false;

    // Shunt voltage (signed, 10µV/LSB)
    if (!INA219_ReadReg(dev, INA219_REG_SHUNT_VOLTAGE, &raw)) return false;
    dev->shunt_voltage_mv = (int16_t)raw * INA219_SHUNT_LSB_UV / 1000.0f;

    // Bus voltage (bits 15:3, 4mV/LSB; bit 1=CNVR, bit 0=OVF)
    if (!INA219_ReadReg(dev, INA219_REG_BUS_VOLTAGE, &raw)) return false;
    dev->overflow = (raw & 0x01) != 0;
    dev->bus_voltage_v = (float)((int16_t)(raw >> 3)) * INA219_BUS_LSB_MV / 1000.0f;

    // Current (signed, Current_LSB per LSB)
    if (!INA219_ReadReg(dev, INA219_REG_CURRENT, &raw)) return false;
    dev->current_a = (int16_t)raw * dev->current_lsb;

    // Power (unsigned, Power_LSB per LSB)
    if (!INA219_ReadReg(dev, INA219_REG_POWER, &raw)) return false;
    dev->power_w = raw * dev->power_lsb;

    dev->valid = true;
    return true;
}

bool INA219_Reset(INA219_t *dev) {
    return INA219_WriteReg(dev, INA219_REG_CONFIG, INA219_CONFIG_RESET);
}
