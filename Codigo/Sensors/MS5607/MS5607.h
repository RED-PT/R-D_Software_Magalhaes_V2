/*
 * MS5607.h - Simplified Altimeter Driver
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

// Driver context - very simple
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} MS5607_t;


// Public API - very simple
bool MS5607_Init(MS5607_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

bool MS5607_Configure(MS5607_t *dev);

// Main function - call from timer callback
bool MS5607_ReadTemperatureandPressure(MS5607_t *dev, BARO_t *output);

#endif /* SENSORS_MS5607_MS5607_H_ */
