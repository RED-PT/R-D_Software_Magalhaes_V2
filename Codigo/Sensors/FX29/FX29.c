/*
 * FX29.c - TE Connectivity FX29 I2C Load Cell Driver
 *
 *  Created on: Jan 11, 2026
 *      Author: Tomas Teixeira
 *
 *  I2C Protocol:
 *    - Simple 3-byte read (no command required)
 *    - Byte 0: [Status(2)] [Bridge Data(6)]  (upper 6 bits of 14-bit data)
 *    - Byte 1: [Bridge Data(8)]              (lower 8 bits of 14-bit data)
 *    - Byte 2: [Temperature(8)]              (optional, for compensation)
 *
 *  Force Calculation:
 *    OUTPUT = (counts - 1000) / 14000 * FORCE_RANGE
 */

#include "FX29.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// I2C timeout
#define FX29_I2C_TIMEOUT_MS     100

// Private functions
static FX29_Status_t FX29_ParseStatus(uint8_t status_byte);
static uint16_t FX29_ParseCounts(uint8_t *data);

bool FX29_Init(FX29_t *dev, I2C_HandleTypeDef *hi2c,
               uint8_t i2c_addr, FX29_Range_t range) {
    if (!dev || !hi2c) {
        return false;
    }

    dev->hi2c = hi2c;
    dev->i2c_addr = i2c_addr;
    dev->force_range_n = range;
    dev->tare_offset = 0;
    dev->is_tared = false;
    dev->scale_factor = 1.0f;
    dev->is_calibrated = false;

    // Verify sensor is responding
    if (!FX29_IsConnected(dev)) {
        printf("ERROR: FX29 not responding at 0x%02X\r\n", i2c_addr);
        return false;
    }

    printf("[FX29] Initialized: addr=0x%02X, range=%dN\r\n",
           i2c_addr, (int)range);

    return true;
}

bool FX29_IsConnected(FX29_t *dev) {
    if (!dev || !dev->hi2c) {
        return false;
    }

    // Try to read from sensor
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
        dev->hi2c,
        (uint16_t)(dev->i2c_addr << 1),
        3,                              // Trials
        FX29_I2C_TIMEOUT_MS
    );

    return (status == HAL_OK);
}

bool FX29_Read(FX29_t *dev, LOADCELL_t *output) {
    return FX29_ReadWithPWM(dev, output, 0);
}

bool FX29_ReadWithPWM(FX29_t *dev, LOADCELL_t *output, uint16_t pwm_value) {
    if (!dev || !dev->hi2c || !output) {
        return false;
    }

    memset(output, 0, sizeof(LOADCELL_t));
    output->timestamp_ms = HAL_GetTick();
    output->pwm_value = pwm_value;

    // Read 2 bytes (status + bridge data)
    // Some variants have 3rd temperature byte, but we only need force
    uint8_t rx_buf[2];
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(
        dev->hi2c,
        (uint16_t)(dev->i2c_addr << 1),
        rx_buf,
        2,
        FX29_I2C_TIMEOUT_MS
    );

    if (hal_status != HAL_OK) {
        output->status = FX29_COMM_ERROR;
        return false;
    }

    // Parse status bits
    output->status = FX29_ParseStatus(rx_buf[0]);

    // Parse 14-bit counts
    output->raw_counts = FX29_ParseCounts(rx_buf);

    // Convert to force (uncorrected)
    output->force_raw_n = FX29_CountsToForce(dev, output->raw_counts);

    // Apply tare if set
    if (dev->is_tared) {
        // Subtract tare offset to get delta from zero point
        int32_t corrected_counts = (int32_t)output->raw_counts - dev->tare_offset;
        // Allow negative values for compression, clamp extreme negatives
        if (corrected_counts < -FX29_COUNT_SPAN) corrected_counts = -FX29_COUNT_SPAN;
        output->force_n = ((float)corrected_counts / (float)FX29_COUNT_SPAN)
                          * (float)dev->force_range_n;
    } else {
        output->force_n = output->force_raw_n;
    }

    // Apply scale factor calibration if set
    if (dev->is_calibrated) {
        output->force_n *= dev->scale_factor;
    }

    return true;
}

bool FX29_StartTare(FX29_t *dev) {
    if (!dev) return false;
    dev->tare_offset = 0;
    dev->is_tared = false;
    return true;
}

bool FX29_Tare(FX29_t *dev) {
    if (!dev) return false;

    printf("[FX29] Starting tare (%d samples)...\r\n", FX29_TARE_SAMPLES);

    int32_t sum = 0;
    int valid_samples = 0;
    LOADCELL_t reading;

    for (int i = 0; i < FX29_TARE_SAMPLES; i++) {
        HAL_Delay(FX29_TARE_DELAY_MS);

        if (FX29_Read(dev, &reading)) {
            if (reading.status == FX29_OK || reading.status == FX29_STALE_DATA) {
                sum += reading.raw_counts;
                valid_samples++;
            }
        }
    }

    if (valid_samples < (FX29_TARE_SAMPLES / 2)) {
        printf("[FX29] Tare failed: only %d valid samples\r\n", valid_samples);
        return false;
    }

    // Store the average raw counts as the tare offset
    // This makes the current reading = 0 after tare
    int32_t avg_counts = sum / valid_samples;
    dev->tare_offset = avg_counts;
    dev->is_tared = true;

    printf("[FX29] Tare complete: zero_offset=%ld raw counts\r\n", avg_counts);

    return true;
}

void FX29_ClearTare(FX29_t *dev) {
    if (dev) {
        dev->tare_offset = 0;
        dev->is_tared = false;
        printf("[FX29] Tare cleared\r\n");
    }
}

bool FX29_Calibrate(FX29_t *dev, float known_weight1_n, float measured1_n,
                    float known_weight2_n, float measured2_n) {
    if (!dev) return false;

    // Calculate delta for both known and measured
    float known_delta = known_weight2_n - known_weight1_n;
    float measured_delta = measured2_n - measured1_n;

    // Check for division by zero
    if (measured_delta == 0.0f || measured_delta < 0.001f && measured_delta > -0.001f) {
        printf("[FX29] Calibration failed: measured values too close\r\n");
        return false;
    }

    // Calculate scale factor: how much to multiply measured to get actual
    dev->scale_factor = known_delta / measured_delta;
    dev->is_calibrated = true;

    printf("[FX29] Calibration complete: scale_factor=%.4f\r\n", dev->scale_factor);
    printf("[FX29] Known: %.2fN -> %.2fN, Measured: %.2fN -> %.2fN\r\n",
           known_weight1_n, known_weight2_n, measured1_n, measured2_n);

    return true;
}

void FX29_SetScaleFactor(FX29_t *dev, float scale_factor) {
    if (dev) {
        dev->scale_factor = scale_factor;
        dev->is_calibrated = true;
        printf("[FX29] Scale factor set: %.4f\r\n", scale_factor);
    }
}

float FX29_GetScaleFactor(FX29_t *dev) {
    if (!dev) return 1.0f;
    return dev->scale_factor;
}

void FX29_ClearCalibration(FX29_t *dev) {
    if (dev) {
        dev->scale_factor = 1.0f;
        dev->is_calibrated = false;
        printf("[FX29] Calibration cleared\r\n");
    }
}

float FX29_CountsToForce(FX29_t *dev, uint16_t counts) {
    if (!dev) return 0.0f;

    // Formula from datasheet:
    // Force% = (counts - 1000) / 14000 * 100
    // Force_N = Force% * Range_N / 100

    int32_t offset_counts = (int32_t)counts - FX29_COUNT_MIN;
    if (offset_counts < 0) offset_counts = 0;

    float force = ((float)offset_counts / (float)FX29_COUNT_SPAN)
                  * (float)dev->force_range_n;

    return force;
}

const char* FX29_StatusToString(FX29_Status_t status) {
    switch (status) {
        case FX29_OK:         return "OK";
        case FX29_STALE_DATA: return "STALE";
        case FX29_FAULT:      return "FAULT";
        case FX29_COMM_ERROR: return "COMM_ERR";
        default:              return "UNKNOWN";
    }
}

// ============================================================================
// Private Functions
// ============================================================================

static FX29_Status_t FX29_ParseStatus(uint8_t status_byte) {
    uint8_t status_bits = status_byte & FX29_STATUS_MASK;

    switch (status_bits) {
        case FX29_STATUS_NORMAL:
            return FX29_OK;
        case FX29_STATUS_STALE:
            return FX29_STALE_DATA;
        case FX29_STATUS_FAULT:
        case FX29_STATUS_COMMAND:
            return FX29_FAULT;
        default:
            return FX29_FAULT;
    }
}

static uint16_t FX29_ParseCounts(uint8_t *data) {
    // 14-bit data spread across 2 bytes:
    // Byte 0: [S1 S0 B13 B12 B11 B10 B9 B8]  (status + upper 6 bits)
    // Byte 1: [B7 B6 B5 B4 B3 B2 B1 B0]      (lower 8 bits)

    uint16_t counts = ((uint16_t)(data[0] & 0x3F) << 8) | data[1];
    return counts;
}
