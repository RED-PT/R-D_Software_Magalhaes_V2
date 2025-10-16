/*
 * MS5607.h
 *
 *  Created on: Oct 16, 2025
 *      Author: texman
 */

#ifndef SENSORS_MS5607_MS5607_H_
#define SENSORS_MS5607_MS5607_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "defs.h"
#include "MS5607SPI.h"

// MS5607 State Machine
typedef enum {
    MS5607_STATE_IDLE,
    MS5607_STATE_CONV_D1,         // Pressure conversion in progress
    MS5607_STATE_READ_D1,         // Reading pressure ADC
    MS5607_STATE_CONV_D2,         // Temperature conversion in progress
    MS5607_STATE_READ_D2,         // Reading temperature ADC
} MS5607_state_t;

// Driver context
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    // Calibration coefficients
    uint16_t calibration[7];

    // Raw ADC values
    uint32_t d1_raw;              // Pressure ADC
    uint32_t d2_raw;              // Temperature ADC

    // State machine
    volatile MS5607_state_t state;
    uint32_t conversion_start_time;

    // DMA buffer for ADC reads
    uint8_t read_buffer[3];
    volatile uint8_t data_ready;
} MS5607_t;

// Public API
bool MS5607_Init(MS5607_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);
bool MS5607_Configure(MS5607_t *dev);
bool MS5607_StartRead(MS5607_t *dev);
bool MS5607_Update(MS5607_t *dev);  // Call periodically to advance state machine
bool MS5607_ProcessData(MS5607_t *dev, BARO_t *output);

#endif /* SENSORS_MS5607_MS5607_H_ */
