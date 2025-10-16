/*
 * ASM330LHHX.h
 *
 *  Created on: Oct 14, 2025
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_ASM330LHHX_ASM330LHHX_H_
#define SENSORS_ASM330LHHX_ASM330LHHX_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "asm330lhhx_reg.h"
#include "defs.h"
#include "config.h"

#define BOOT_TIME 10  // ms

// Driver context - owns all sensor state
typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t read_buffer[15];      // DMA buffer
    volatile uint8_t data_ready;
    // Internal raw data
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;
} ASM330LHHX_t;

// Public API
bool ASM330LHHX_Init(ASM330LHHX_t *dev, SPI_HandleTypeDef *hspi);
bool ASM330LHHX_Configure(ASM330LHHX_t *dev);
bool ASM330LHHX_StartReadDMA(ASM330LHHX_t *dev);
bool ASM330LHHX_ProcessData(ASM330LHHX_t *dev, IMU_t *output);

// For DMA callback integration
void ASM330LHHX_ParseDMABuffer(ASM330LHHX_t *dev);

#endif /* SENSORS_ASM330LHHX_ASM330LHHX_H_ */
