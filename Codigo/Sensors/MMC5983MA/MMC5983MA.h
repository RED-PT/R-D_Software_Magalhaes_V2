/*
 * MMC5983MA.h
 *
 *  Created on: Oct 16, 2025
 *      Author: texman
 */

#ifndef SENSORS_MMC5983MA_MMC5983MA_H_
#define SENSORS_MMC5983MA_MMC5983MA_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "defs.h"
#include "config.h"

// Register addresses
#define MMC5983MA_REG_XOUT0          0x00
#define MMC5983MA_REG_XOUT1          0x01
#define MMC5983MA_REG_YOUT0          0x02
#define MMC5983MA_REG_YOUT1          0x03
#define MMC5983MA_REG_ZOUT0          0x04
#define MMC5983MA_REG_ZOUT1          0x05
#define MMC5983MA_REG_XYZOUT2        0x06
#define MMC5983MA_REG_TOUT           0x07
#define MMC5983MA_REG_STATUS         0x08
#define MMC5983MA_REG_CTRL0          0x09
#define MMC5983MA_REG_CTRL1          0x0A
#define MMC5983MA_REG_CTRL2          0x0B
#define MMC5983MA_REG_CTRL3          0x0C
#define MMC5983MA_REG_PRODUCT_ID     0x2F

// Control register bits
#define MMC5983MA_CTRL0_TM_M         0x01
#define MMC5983MA_CTRL0_INT_EN       0x04
#define MMC5983MA_CTRL0_SET          0x08
#define MMC5983MA_CTRL0_AUTO_SR      0x20

#define MMC5983MA_CTRL1_BW0          0x01
#define MMC5983MA_CTRL1_BW1          0x02

#define MMC5983MA_CTRL2_CMM_EN       0x04

// Calibration constants
#define MMC5983MA_SENSITIVITY_18BIT  16384.0f
#define MMC5983MA_TEMP_SCALE         0.8f
#define MMC5983MA_TEMP_OFFSET        -75

// Driver context
typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t read_buffer[8];       // DMA buffer
    volatile uint8_t data_ready;
    // Internal raw data
    int32_t mag_x_raw;
    int32_t mag_y_raw;
    int32_t mag_z_raw;
    uint8_t temp_raw;
} MMC5983MA_t;

// Public API
bool MMC5983MA_Init(MMC5983MA_t *dev, SPI_HandleTypeDef *hspi);
bool MMC5983MA_Configure(MMC5983MA_t *dev);
bool MMC5983MA_StartReadDMA(MMC5983MA_t *dev);
bool MMC5983MA_ProcessData(MMC5983MA_t *dev, MAG_t *output);

// For DMA callback integration
void MMC5983MA_ParseDMABuffer(MMC5983MA_t *dev);

// Helper functions
static inline bool MMC5983MA_IsDataReady(MMC5983MA_t *dev) {
    return dev->data_ready;
}

#endif /* SENSORS_MMC5983MA_MMC5983MA_H_ */
