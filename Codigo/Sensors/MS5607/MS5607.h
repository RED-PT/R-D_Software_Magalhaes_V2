/*
 * MS5607.h - Altimeter Driver with Launch Pad Calibration
 *
 *  Created on: Oct 16, 2025
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_MS5607_MS5607_H_
#define SENSORS_MS5607_MS5607_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "defs.h"

// MS5607 SPI commands
#define CMD_RESET      0x1E
#define CMD_CONV_D1    0x48
#define CMD_CONV_D2    0x58
#define CMD_ADC_READ   0x00
#define CMD_PROM_READ  0xA0

// Calibration settings
#define BARO_CALIBRATION_SAMPLES    50      // Number of samples for averaging
#define BARO_CALIBRATION_DELAY_MS   20      // Delay between samples

// Driver context
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} MS5607_t;

// Calibration context (stored in flight_computer.h fsm_ctx)
typedef struct {
    float reference_pressure_mbar;    // Launch pad pressure
    float reference_altitude_m;       // GPS altitude at calibration (optional)
    float temperature_at_cal_c;       // Temperature during calibration
    uint32_t calibration_timestamp;   // When calibration was done
    bool is_calibrated;               // Calibration valid flag
    uint8_t samples_collected;        // For averaging during calibration
    float pressure_sum;               // Accumulator for averaging
} baro_calibration_t;

// Public API
bool MS5607_Init(MS5607_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

bool MS5607_Configure(MS5607_t *dev);

// Main function - call from timer callback
bool MS5607_ReadTemperatureandPressure(MS5607_t *dev, BARO_t *output);

// Calibration functions
bool MS5607_StartCalibration(MS5607_t *dev, baro_calibration_t *cal);
bool MS5607_AddCalibrationSample(MS5607_t *dev, baro_calibration_t *cal);
bool MS5607_FinishCalibration(baro_calibration_t *cal);

// Read with calibration applied (returns AGL altitude)
bool MS5607_ReadWithCalibration(MS5607_t *dev, BARO_t *output,
                                 const baro_calibration_t *cal);

// Utility
float MS5607_PressureToAltitude(float pressure_mbar, float ref_pressure_mbar);

#endif /* SENSORS_MS5607_MS5607_H_ */
