/*
 * MS5607.c
 *
 *  Created on: Oct 16, 2025
 *      Author: texman
 */

#include "MS5607.h"
#include <string.h>
#include <math.h>

// Private helper functions
static void MS5607_Select(MS5607_t *dev);
static void MS5607_Deselect(MS5607_t *dev);
static HAL_StatusTypeDef MS5607_SendCommand(MS5607_t *dev, uint8_t cmd);
static bool MS5607_ReadADC_DMA(MS5607_t *dev);

bool MS5607_Init(MS5607_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (!dev || !hspi || !cs_port) {
        return false;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->state = MS5607_STATE_IDLE;
    dev->data_ready = 0;
    memset(dev->read_buffer, 0, sizeof(dev->read_buffer));

    // Reset sensor
    MS5607_SendCommand(dev, CMD_RESET);
    HAL_Delay(10);

    // Read calibration coefficients from PROM (blocking - only during init)
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t addr = CMD_PROM_READ + (i * 2);
        uint8_t tx_buf = addr;
        uint8_t rx_buf[2];

        MS5607_Select(dev);
        HAL_SPI_Transmit(dev->hspi, &tx_buf, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(dev->hspi, rx_buf, 2, HAL_MAX_DELAY);
        MS5607_Deselect(dev);

        dev->calibration[i] = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    }

    return true;
}

bool MS5607_Configure(MS5607_t *dev) {
    if (!dev) {
        return false;
    }

    dev->state = MS5607_STATE_IDLE;
    return true;
}

bool MS5607_StartRead(MS5607_t *dev) {
    if (!dev) {
        return false;
    }

    // Only start if idle
    if (dev->state != MS5607_STATE_IDLE) {
        return false;
    }

    // Start D2 (temperature) conversion
    if (MS5607_SendCommand(dev, CMD_CONV_D2) != HAL_OK) {
        return false;
    }

    dev->state = MS5607_STATE_CONV_D2;
    dev->conversion_start_time = HAL_GetTick();

    return true;
}

bool MS5607_Update(MS5607_t *dev) {
    uint32_t elapsed_ms;

    if (!dev) {
        return false;
    }

    elapsed_ms = HAL_GetTick() - dev->conversion_start_time;

    switch (dev->state) {
        case MS5607_STATE_CONV_D2:
            // Wait for D2 conversion (~9ms for high resolution)
            if (elapsed_ms >= 10) {
                // Start DMA read of D2 ADC
                if (MS5607_ReadADC_DMA(dev)) {
                    dev->state = MS5607_STATE_READ_D2;
                    return false;  // Wait for DMA to complete
                } else {
                    dev->state = MS5607_STATE_IDLE;
                    return false;
                }
            }
            return false;  // Not ready yet

        case MS5607_STATE_READ_D2:
            // DMA read complete - data already in read_buffer
            // Parse D2
            dev->d2_raw = ((uint32_t)dev->read_buffer[1] << 16) |
                         ((uint32_t)dev->read_buffer[2] << 8) |
                          dev->read_buffer[3];

            // Start D1 (pressure) conversion
            if (MS5607_SendCommand(dev, CMD_CONV_D1) != HAL_OK) {
                dev->state = MS5607_STATE_IDLE;
                return false;
            }

            dev->state = MS5607_STATE_CONV_D1;
            dev->conversion_start_time = HAL_GetTick();
            return false;

        case MS5607_STATE_CONV_D1:
            // Wait for D1 conversion (~9ms for high resolution)
            if (elapsed_ms >= 10) {
                // Start DMA read of D1 ADC
                if (MS5607_ReadADC_DMA(dev)) {
                    dev->state = MS5607_STATE_READ_D1;
                    return false;  // Wait for DMA to complete
                } else {
                    dev->state = MS5607_STATE_IDLE;
                    return false;
                }
            }
            return false;  // Not ready yet

        case MS5607_STATE_READ_D1:
            // DMA read complete - data already in read_buffer
            // Parse D1
            dev->d1_raw = ((uint32_t)dev->read_buffer[1] << 16) |
                         ((uint32_t)dev->read_buffer[2] << 8) |
                          dev->read_buffer[3];

            dev->state = MS5607_STATE_IDLE;
            dev->data_ready = 1;
            return true;  // Data ready

        default:
            dev->state = MS5607_STATE_IDLE;
            return false;
    }
}

bool MS5607_ProcessData(MS5607_t *dev, BARO_t *output) {
    if (!dev || !output || !dev->data_ready) {
        return false;
    }

    // Convert raw ADC values to physical units
    // Using formulas from MS5607 datasheet

    int64_t d1 = (int64_t)dev->d1_raw;
    int64_t d2 = (int64_t)dev->d2_raw;

    int64_t c1 = (int64_t)dev->calibration[0];
    int64_t c2 = (int64_t)dev->calibration[1];
    int64_t c3 = (int64_t)dev->calibration[2];
    int64_t c4 = (int64_t)dev->calibration[3];
    int64_t c5 = (int64_t)dev->calibration[4];
    int64_t c6 = (int64_t)dev->calibration[5];

    // Calculate temperature
    int64_t dt = d2 - (c5 << 8);
    int64_t temp_raw = 2000 + ((dt * c6) >> 23);

    output->temperature_c = (float)temp_raw / 100.0f;

    // Calculate pressure
    int64_t off = (c2 << 16) + ((dt * c4) >> 7);
    int64_t sens = (c1 << 15) + ((dt * c3) >> 8);

    int64_t pressure_raw = (((d1 * sens) >> 21) - off) >> 15;
    output->pressure_mbar = (float)pressure_raw / 100.0f;

    // Calculate altitude (simple approximation)
    #define SEA_LEVEL_PRESSURE 1013.25f
    float pressure_ratio = output->pressure_mbar / SEA_LEVEL_PRESSURE;
    output->altitude_m = 44330.0f * (1.0f - powf(pressure_ratio, 1.0f / 5.255f));

    output->timestamp_ms = HAL_GetTick();

    dev->data_ready = 0;

    return true;
}

// Private Functions

static void MS5607_Select(MS5607_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void MS5607_Deselect(MS5607_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MS5607_SendCommand(MS5607_t *dev, uint8_t cmd) {
    HAL_StatusTypeDef status;

    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    return status;
}

static bool MS5607_ReadADC_DMA(MS5607_t *dev) {
    // Prepare TX buffer: [ADC_READ_CMD][DUMMIES]
    uint8_t tx_buffer[4];
    tx_buffer[0] = CMD_ADC_READ;
    memset(&tx_buffer[1], 0x00, 3);

    MS5607_Select(dev);

    // Use blocking SPI TransmitReceive for simplicity
    // (The DMA callback won't fire for blocking calls)
    // Alternative: Use HAL_SPI_TransmitReceive_DMA if you want true async
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_buffer, dev->read_buffer, 4, HAL_MAX_DELAY) != HAL_OK) {
        MS5607_Deselect(dev);
        return false;
    }

    MS5607_Deselect(dev);

    return true;
}
