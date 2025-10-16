/*
 * BNO055.h
 *
 *  Created on: Oct 16, 2025
 *      Author: texman
 */

#ifndef SENSORS_BNO055_BNO055_H_
#define SENSORS_BNO055_BNO055_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "defs.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t read_buffer[20];
    volatile uint8_t data_ready;
} BNO055_t;

bool BNO055_Init(BNO055_t *dev, I2C_HandleTypeDef *hi2c);
bool BNO055_Configure(BNO055_t *dev);
bool BNO055_StartReadDMA(BNO055_t *dev);
bool BNO055_ProcessData(BNO055_t *dev, BNO_t *output);

#endif /* SENSORS_BNO055_BNO055_H_ */
