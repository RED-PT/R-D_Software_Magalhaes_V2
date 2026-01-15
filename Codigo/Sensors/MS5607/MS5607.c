/*
 * MS5607.c - Altimeter Driver with Launch Pad Calibration
 *
 *  Created on: Oct 16, 2025
 *      Author: Tomas Teixeira
 *
 *  Calibration feature: Stores reference pressure at launch pad to compute
 *  AGL (Above Ground Level) altitude instead of MSL altitude.
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
    }

    // Validate calibration coefficients
    if (C[0] == 0x0000 || C[0] == 0xFFFF) {
        printf("ERROR: Invalid C[0] = 0x%04X - sensor not responding!\r\n", C[0]);
        return false;
    }

    return true;
}

bool MS5607_Configure(MS5607_t *dev) {
    // MS5607 doesn't need additional configuration after init
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
        return false;
    }

    status = HAL_SPI_Receive(dev->hspi, rx_buf, 3, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        return false;
    }

    D2 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];

    // Read pressure (D1)
    cmd = CMD_CONV_D1;
    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        return false;
    }

    HAL_Delay(10);  // Wait for conversion

    // Read D1 ADC value
    cmd = CMD_ADC_READ;
    MS5607_Select(dev);
    status = HAL_SPI_Transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        MS5607_Deselect(dev);
        return false;
    }

    status = HAL_SPI_Receive(dev->hspi, rx_buf, 3, HAL_MAX_DELAY);
    MS5607_Deselect(dev);

    if (status != HAL_OK) {
        return false;
    }

    D1 = ((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | rx_buf[2];

    // Calculate temperature using MS5607 formula
    int64_t dT = (int64_t)D2 - ((int64_t)C[5] << 8);
    int32_t TEMP = 2000 + (int32_t)((dT * (int64_t)C[6]) >> 23);

    // Calculate pressure offset and sensitivity
    // MS5607 formulas (different from MS5611!):
    //   OFF  = C2 * 2^17 + (C4 * dT) / 2^6
    //   SENS = C1 * 2^16 + (C3 * dT) / 2^7
    int64_t OFF = ((int64_t)C[2] << 17) + (((int64_t)dT * (int64_t)C[4]) >> 6);
    int64_t SENS = ((int64_t)C[1] << 16) + (((int64_t)dT * (int64_t)C[3]) >> 7);

    // Calculate final pressure
    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    // Convert to output format
    output->temperature_c = (float)TEMP / 100.0f;
    output->pressure_mbar = (float)P / 100.0f;

    // Calculate MSL altitude using barometric formula (uncalibrated)
    #define SEA_LEVEL_PRESSURE 1013.25f
    float pressure_ratio = output->pressure_mbar / SEA_LEVEL_PRESSURE;
    output->altitude_m = 44330.0f * (1.0f - powf(pressure_ratio, 1.0f / 5.255f));

    output->timestamp_ms = HAL_GetTick();

    return true;
}

// ============================================================================
// Calibration Functions
// ============================================================================

bool MS5607_StartCalibration(MS5607_t *dev, baro_calibration_t *cal) {
    if (!cal) return false;

    // Reset calibration state
    cal->is_calibrated = false;
    cal->samples_collected = 0;
    cal->pressure_sum = 0.0f;
    cal->reference_pressure_mbar = 0.0f;
    cal->reference_altitude_m = 0.0f;
    cal->temperature_at_cal_c = 0.0f;
    cal->calibration_timestamp = 0;

    printf("[BARO] Starting calibration (%d samples)...\r\n", BARO_CALIBRATION_SAMPLES);
    return true;
}

bool MS5607_AddCalibrationSample(MS5607_t *dev, baro_calibration_t *cal) {
    if (!dev || !cal) return false;
    if (cal->samples_collected >= BARO_CALIBRATION_SAMPLES) return false;

    BARO_t reading;
    if (!MS5607_ReadTemperatureandPressure(dev, &reading)) {
        printf("[BARO] Failed to read sample %d\r\n", cal->samples_collected);
        return false;
    }

    cal->pressure_sum += reading.pressure_mbar;
    cal->temperature_at_cal_c = reading.temperature_c;  // Use last temp
    cal->samples_collected++;

    return true;
}

bool MS5607_FinishCalibration(baro_calibration_t *cal) {
    if (!cal) return false;
    if (cal->samples_collected < BARO_CALIBRATION_SAMPLES) {
        printf("[BARO] Not enough samples: %d/%d\r\n",
               cal->samples_collected, BARO_CALIBRATION_SAMPLES);
        return false;
    }

    // Calculate average pressure
    cal->reference_pressure_mbar = cal->pressure_sum / (float)cal->samples_collected;
    cal->calibration_timestamp = HAL_GetTick();
    cal->is_calibrated = true;

    printf("[BARO] Calibration complete!\r\n");
    printf("[BARO]   Reference pressure: %.2f mbar\r\n", cal->reference_pressure_mbar);
    printf("[BARO]   Temperature: %.1f C\r\n", cal->temperature_at_cal_c);
    printf("[BARO]   Samples: %d\r\n", cal->samples_collected);

    return true;
}

bool MS5607_ReadWithCalibration(MS5607_t *dev, BARO_t *output,
                                 const baro_calibration_t *cal) {
    // First get the raw reading
    if (!MS5607_ReadTemperatureandPressure(dev, output)) {
        return false;
    }

    // If calibrated, compute AGL altitude
    if (cal && cal->is_calibrated) {
        output->altitude_m = MS5607_PressureToAltitude(output->pressure_mbar,
                                                        cal->reference_pressure_mbar);
    }
    // Otherwise altitude_m contains MSL altitude (from standard pressure)

    return true;
}

float MS5607_PressureToAltitude(float pressure_mbar, float ref_pressure_mbar) {
    // Hypsometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    // When ref_pressure is launch pad pressure, result is AGL altitude
    if (ref_pressure_mbar <= 0.0f) {
        ref_pressure_mbar = 1013.25f;  // Fallback to sea level
    }

    float pressure_ratio = pressure_mbar / ref_pressure_mbar;
    return 44330.0f * (1.0f - powf(pressure_ratio, 1.0f / 5.255f));
}

// Private Functions
static void MS5607_Select(MS5607_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void MS5607_Deselect(MS5607_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}
