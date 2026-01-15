/*
 * fx29.h
 *
 *  Created on: Jan 11, 2026
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_FX29_FX29_H_
#define SENSORS_FX29_FX29_H_

#include <stdbool.h>
#include <stdint.h>
#include "defs.h"
#include "stm32f4xx_hal.h"

// I2C Addresses (selectable via part number)
#define FX29_ADDR_0     0x28    // Default
#define FX29_ADDR_1     0x36
#define FX29_ADDR_2     0x46
#define FX29_ADDR_3     0x48
#define FX29_ADDR_4     0x51

// Status bits (upper 2 bits of first byte)
#define FX29_STATUS_MASK        0xC0
#define FX29_STATUS_NORMAL      0x00    // Normal operation, valid data
#define FX29_STATUS_COMMAND     0x40    // Command mode
#define FX29_STATUS_STALE       0x80    // Stale data (already read)
#define FX29_STATUS_FAULT       0xC0    // Diagnostic condition

// Output counts
#define FX29_COUNT_MIN          1000    // 0% force
#define FX29_COUNT_MAX          15000   // 100% force
#define FX29_COUNT_SPAN         14000   // Full scale span

// Timing
#define FX29_RESPONSE_TIME_MS   3       // Non-sleep mode
#define FX29_SLEEP_WAKEUP_MS    9       // Sleep mode wakeup

// Tare/Zero calibration
#define FX29_TARE_SAMPLES       20
#define FX29_TARE_DELAY_MS      10

// Available force ranges (Newtons)
typedef enum {
    FX29_RANGE_50N   = 50,
    FX29_RANGE_125N  = 125,
    FX29_RANGE_250N  = 250,
    FX29_RANGE_500N  = 500,
    FX29_RANGE_1000N = 1000
} FX29_Range_t;

// Status from last read
typedef enum {
    FX29_OK = 0,
    FX29_STALE_DATA,
    FX29_FAULT,
    FX29_COMM_ERROR
} FX29_Status_t;

// Driver context
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t i2c_addr;           // 7-bit address
    FX29_Range_t force_range_n; // Sensor range in Newtons
    int32_t tare_offset;        // Raw counts offset for zeroing
    bool is_tared;
} FX29_t;

// Initialization
bool FX29_Init(FX29_t *dev, I2C_HandleTypeDef *hi2c,
               uint8_t i2c_addr, FX29_Range_t range);

// Check if sensor is responding
bool FX29_IsConnected(FX29_t *dev);

// Read force measurement (blocking poll)
bool FX29_Read(FX29_t *dev, LOADCELL_t *output);

// Read with associated PWM value for test logging
bool FX29_ReadWithPWM(FX29_t *dev, LOADCELL_t *output, uint16_t pwm_value);

// Tare/Zero functions
bool FX29_StartTare(FX29_t *dev);
bool FX29_Tare(FX29_t *dev);  // Blocking: takes multiple samples
void FX29_ClearTare(FX29_t *dev);

// Utility
float FX29_CountsToForce(FX29_t *dev, uint16_t counts);
const char* FX29_StatusToString(FX29_Status_t status);

#endif /* SENSORS_FX29_FX29_H_ */
