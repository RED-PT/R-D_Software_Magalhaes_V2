/*
 * MS5607.c - Altimeter Driver - CORRECTED BIT SHIFTS
 *
 *  Created on: Oct 16, 2025
 *      Author: Tomas Teixeira
 *  Fixed: Bit shift error causing pressure to be half of actual value
 */

#include "MS5607.h"
#include <string.h>
#include <math.h>

// Global calibration coefficients
uint16_t C[7] = {0};

// Private helper functions
static void MS5607_Select(MS5607_t *dev);
static void MS5607_Deselect(MS5607_t *dev);

bool MS5607_Init(MS5607_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    if (!dev || !hspi || !cs_port) {
        return false;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    // Reset sensor
    //printf("MS5607: Sending reset command...\r\n");
    uint8_t cmd = CMD_RESET;
    MS5607_Select(dev);
    HAL_Delay(1);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        printf("ERROR: MS5607 reset failed (SPI error: %d)\r\n", status);
        return false;
    }

    HAL_Delay(10);
    //printf("MS5607: Reset complete, reading calibration...\r\n");

    // Read calibration coefficients from PROM
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t addr = CMD_PROM_READ + (i * 2);
        uint8_t rx_buf[2];

        MS5607_Select(dev);
        status = HAL_SPI_Transmit(dev->hspi, &addr, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            MS5607_Deselect(dev);
            printf("ERROR: Failed to send PROM read command for C[%d]\r\n", i);
            return false;
        }

        status = HAL_SPI_Receive(dev->hspi, rx_buf, 2, HAL_MAX_DELAY);
        MS5607_Deselect(dev);

        if (status != HAL_OK) {
            printf("ERROR: Failed to receive PROM data for C[%d]\r\n", i);
            return false;
        }

        C[i] = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
        //printf("C[%d] = 0x%04X (%u) [raw bytes: 0x%02X 0x%02X]\r\n",
        //i, C[i], C[i], rx_buf[0], rx_buf[1]);
    }

    // Validate calibration coefficients
    if (C[0] == 0x0000 || C[0] == 0xFFFF) {
        printf("ERROR: Invalid C[0] = 0x%04X - sensor not responding!\r\n", C[0]);
        printf("Check: SPI wiring, chip select, power supply\r\n");
        return false;
    }

    // Additional validation - typical ranges from datasheet
    if (C[1] < 30000 || C[1] > 50000) {
        printf("WARNING: C[1] = %u is outside typical range (30000-50000)\r\n", C[1]);
    }

    //printf("MS5607: Initialization successful!\r\n");
    return true;
}

// Read temperature and pressure - call this from timer callback
bool MS5607_ReadTemperatureandPressure(MS5607_t *dev, BARO_t *output) {
    if (!dev || !output) {
        return false;
    }

    uint8_t cmd;
    uint32_t D2, D1;
    HAL_StatusTypeDef status;

    // Read temperature (D2)
    cmd = CMD_CONV_D2;
    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        printf("ERROR: Failed to send D2 conversion command\r\n");
        return false;
    }

    HAL_Delay(10);  // Wait for conversion

    // Read D2 ADC value
    cmd = CMD_ADC_READ;
    uint8_t rx_buf[3];
    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        MS5607_Deselect(dev);
        printf("ERROR: Failed to send ADC read command for D2\r\n");
        return false;
    }

    status = HAL_SPI_Receive(dev->hspi, rx_buf, 3, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        printf("ERROR: Failed to receive D2 ADC data\r\n");
        return false;
    }

    D2 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];

    // Read pressure (D1)
    cmd = CMD_CONV_D1;
    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        printf("ERROR: Failed to send D1 conversion command\r\n");
        return false;
    }

    HAL_Delay(10);  // Wait for conversion

    // Read D1 ADC value
    cmd = CMD_ADC_READ;
    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        MS5607_Deselect(dev);
        printf("ERROR: Failed to send ADC read command for D1\r\n");
        return false;
    }

    status = HAL_SPI_Receive(dev->hspi, rx_buf, 3, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        printf("ERROR: Failed to receive D1 ADC data\r\n");
        return false;
    }

    D1 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];

    // Calculate temperature using MS5607 formula
    int64_t dT = (int64_t)D2 - ((int64_t)C[5] << 8);
    int32_t TEMP = 2000 + (int32_t)((dT * (int64_t)C[6]) >> 23);

    // Calculate pressure offset and sensitivity
    // CRITICAL FIX: Changed << 16 to << 17 for OFF, and << 15 to << 16 for SENS
    int64_t OFF = ((int64_t)C[2] << 17) + (((int64_t)dT * (int64_t)C[4]) >> 7);
    int64_t SENS = ((int64_t)C[1] << 16) + (((int64_t)dT * (int64_t)C[3]) >> 8);

    // Calculate final pressure
    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);


    // Convert to output format
    output->temperature_c = (float)TEMP / 100.0f;
    output->pressure_mbar = (float)P / 100.0f;

    // Calculate altitude using barometric formula
    #define SEA_LEVEL_PRESSURE 1013.25f
    float pressure_ratio = output->pressure_mbar / SEA_LEVEL_PRESSURE;
    output->altitude_m = 44330.0f * (1.0f - powf(pressure_ratio, 1.0f / 5.255f));

    output->timestamp_ms = HAL_GetTick();

    // Debug output
    //printf("D2=%lu D1=%lu T=%.2fC P=%.2fmbar Alt=%.1fm\r\n",
    //       D2, D1, output->temperature_c, output->pressure_mbar, output->altitude_m);

    return true;
}

// Private Functions
static void MS5607_Select(MS5607_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void MS5607_Deselect(MS5607_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}
