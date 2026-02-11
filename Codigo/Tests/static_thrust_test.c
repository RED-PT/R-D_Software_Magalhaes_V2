/*
 * static_thrust_test.c
 *
 * Static Motor Thrust Test Module
 * Records (Thrust, PWM) data points during throttle ramp
 *
 *  Created on: Jan 11, 2026
 *      Author: Tomas Teixeira
 */

#include "static_thrust_test.h"
#include "Atuadores/ESC/PWM_FUNCTIONS.h"
#include "Storage/sd_card_thread.h"
#include "config.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

// Use I2C_LOADCELL from config.h (hi2c2)
extern I2C_HandleTypeDef hi2c2;

// Load cell calibration (from linear regression with 11 points)
// Formula: F_real = LOADCELL_SCALE_FACTOR * F_measured
#define LOADCELL_SCALE_FACTOR  0.4481f

// Test context (singleton)
static static_test_ctx_t test_ctx = {0};
static char test_filename[32] = {0};

// Private functions
static bool save_test_data_to_sd(void);
static void take_sample(void);

void StaticTest_Init(void) {
    memset(&test_ctx, 0, sizeof(test_ctx));
    test_ctx.state = STATIC_TEST_IDLE;
    test_ctx.max_samples = STATIC_TEST_MAX_SAMPLES;

    printf("[STATIC_TEST] Module initialized\r\n");
}

bool StaticTest_Start(uint8_t max_throttle_percent) {
    // Validate parameters
    if (max_throttle_percent < 1 || max_throttle_percent > 100) {
        test_ctx.error_msg = "Invalid throttle percentage (1-100)";
        printf("[STATIC_TEST] ERROR: %s\r\n", test_ctx.error_msg);
        return false;
    }

    // Check if test already running
    if (test_ctx.state != STATIC_TEST_IDLE &&
        test_ctx.state != STATIC_TEST_COMPLETE &&
        test_ctx.state != STATIC_TEST_FAILED) {
        test_ctx.error_msg = "Test already in progress";
        printf("[STATIC_TEST] ERROR: %s\r\n", test_ctx.error_msg);
        return false;
    }

    // Reset context
    memset(&test_ctx, 0, sizeof(test_ctx));

    // Configure test
    test_ctx.config.max_throttle_percent = max_throttle_percent;
    test_ctx.config.ramp_duration_ms = STATIC_TEST_RAMP_DURATION_MS;
    test_ctx.config.hold_duration_ms = STATIC_TEST_HOLD_DURATION_MS;
    test_ctx.config.sample_interval_ms = 1000 / STATIC_TEST_SAMPLE_RATE_HZ;
    test_ctx.max_samples = STATIC_TEST_MAX_SAMPLES;

    // Generate filename
    snprintf(test_filename, sizeof(test_filename), "static_test_pwm_%02d.csv", max_throttle_percent);

    printf("[STATIC_TEST] Starting test: max=%d%%, file=%s\r\n",
           max_throttle_percent, test_filename);

    // Pause SD card to avoid EMI interference from motor
    sd_card_pause();

    // Start initialization phase
    test_ctx.state = STATIC_TEST_INIT;
    test_ctx.test_start_tick = HAL_GetTick();
    test_ctx.phase_start_tick = HAL_GetTick();

    return true;
}

static_test_state_t StaticTest_Update(void) {
    uint32_t now = HAL_GetTick();
    uint32_t phase_elapsed = now - test_ctx.phase_start_tick;

    switch (test_ctx.state) {
        case STATIC_TEST_IDLE:
        case STATIC_TEST_COMPLETE:
        case STATIC_TEST_FAILED:
            // Nothing to do
            break;

        case STATIC_TEST_INIT: {
            // Initialize load cell using I2C2 (I2C_LOADCELL from config.h)
            printf("[STATIC_TEST] Initializing load cell on I2C2...\r\n");

            // First check if load cell is responding
            if (HAL_I2C_IsDeviceReady(&hi2c2, (FX29_ADDR_0 << 1), 3, 100) != HAL_OK) {
                test_ctx.error_msg = "Load cell not connected (I2C2 addr 0x28)";
                printf("[STATIC_TEST] ERROR: %s\r\n", test_ctx.error_msg);
                test_ctx.state = STATIC_TEST_FAILED;
                sd_card_resume();  // Resume SD on failure
                break;
            }
            printf("[STATIC_TEST] Load cell found on I2C2\r\n");

            if (!FX29_Init(&test_ctx.loadcell, &hi2c2, FX29_ADDR_0, FX29_RANGE_500N)) {
                test_ctx.error_msg = "Load cell init failed";
                printf("[STATIC_TEST] ERROR: %s\r\n", test_ctx.error_msg);
                test_ctx.state = STATIC_TEST_FAILED;
                sd_card_resume();  // Resume SD on failure
                break;
            }
            test_ctx.loadcell_initialized = true;

            // Apply calibration scale factor
            FX29_SetScaleFactor(&test_ctx.loadcell, LOADCELL_SCALE_FACTOR);
            printf("[STATIC_TEST] Calibration applied: scale=%.4f\r\n", LOADCELL_SCALE_FACTOR);

            // Tare the load cell
            printf("[STATIC_TEST] Taring load cell...\r\n");
            if (!FX29_Tare(&test_ctx.loadcell)) {
                test_ctx.error_msg = "Load cell tare failed";
                printf("[STATIC_TEST] ERROR: %s\r\n", test_ctx.error_msg);
                test_ctx.state = STATIC_TEST_FAILED;
                sd_card_resume();  // Resume SD on failure
                break;
            }

            // Ensure motor is at 0%
            PWM_SetThrottle(0.0f);

            // Move to settle phase
            printf("[STATIC_TEST] Init complete, settling...\r\n");
            test_ctx.state = STATIC_TEST_SETTLE;
            test_ctx.phase_start_tick = now;
            break;
        }

        case STATIC_TEST_SETTLE: {
            if (phase_elapsed >= STATIC_TEST_SETTLE_DELAY_MS) {
                // STEP: Jump directly to max throttle (no ramp)
                printf("[STATIC_TEST] STEP to %d%%\r\n",
                       test_ctx.config.max_throttle_percent);
                test_ctx.current_throttle = (float)test_ctx.config.max_throttle_percent;
                PWM_SetThrottle(test_ctx.current_throttle);

                test_ctx.state = STATIC_TEST_HOLD;
                test_ctx.phase_start_tick = now;
                test_ctx.last_sample_tick = now;
            }
            break;
        }

        case STATIC_TEST_HOLD: {
            // Hold at max throttle
            test_ctx.current_throttle = (float)test_ctx.config.max_throttle_percent;
            PWM_SetThrottle(test_ctx.current_throttle);

            // Continue sampling
            if ((now - test_ctx.last_sample_tick) >= test_ctx.config.sample_interval_ms) {
                take_sample();
                test_ctx.last_sample_tick = now;
            }

            // Check if hold complete
            if (phase_elapsed >= test_ctx.config.hold_duration_ms) {
                // STEP: Jump directly to 0% (no ramp)
                printf("[STATIC_TEST] Hold complete, STEP to 0%%\r\n");
                PWM_SetThrottle(0.0f);
                test_ctx.current_throttle = 0.0f;
                test_ctx.state = STATIC_TEST_SAVING;
                test_ctx.phase_start_tick = now;
            }
            break;
        }

        case STATIC_TEST_SAVING: {
            // Save data to SD card
            if (save_test_data_to_sd()) {
                printf("[STATIC_TEST] TEST COMPLETE! %d samples saved to %s\r\n",
                       test_ctx.data_count, test_filename);
                printf("[STATIC_TEST] Max thrust: %.2f N at %.1f%% PWM\r\n",
                       test_ctx.max_thrust_n, test_ctx.max_thrust_pwm);
                test_ctx.state = STATIC_TEST_COMPLETE;
                sd_card_resume();  // Resume SD after test
            } else {
                test_ctx.error_msg = "Failed to save data";
                printf("[STATIC_TEST] ERROR: %s\r\n", test_ctx.error_msg);
                test_ctx.state = STATIC_TEST_FAILED;
                sd_card_resume();  // Resume SD after test
            }
            break;
        }
    }

    return test_ctx.state;
}

static void take_sample(void) {
    if (test_ctx.data_count >= test_ctx.max_samples) {
        return; // Buffer full
    }

    LOADCELL_t reading;
    if (FX29_ReadWithPWM(&test_ctx.loadcell, &reading, (uint16_t)(test_ctx.current_throttle * 10))) {
        thrust_data_point_t *dp = &test_ctx.data[test_ctx.data_count];
        dp->timestamp_ms = HAL_GetTick() - test_ctx.test_start_tick;
        dp->thrust_n = reading.force_n;
        dp->pwm_percent = test_ctx.current_throttle;
        dp->raw_counts = reading.raw_counts;

        test_ctx.current_thrust = reading.force_n;
        test_ctx.data_count++;

        // Track max thrust
        if (reading.force_n > test_ctx.max_thrust_n) {
            test_ctx.max_thrust_n = reading.force_n;
            test_ctx.max_thrust_pwm = test_ctx.current_throttle;
        }
    }
}

static bool save_test_data_to_sd(void) {
    // Data is sent via telemetry to dashboard - no terminal output needed
    // Dashboard provides CSV download functionality
    return true;
}

static_test_state_t StaticTest_GetState(void) {
    return test_ctx.state;
}

const static_test_ctx_t* StaticTest_GetContext(void) {
    return &test_ctx;
}

void StaticTest_Cancel(void) {
    if (test_ctx.state != STATIC_TEST_IDLE &&
        test_ctx.state != STATIC_TEST_COMPLETE &&
        test_ctx.state != STATIC_TEST_FAILED) {

        printf("[STATIC_TEST] Test CANCELLED\r\n");

        // Safety: stop motor
        PWM_SetThrottle(0.0f);
        test_ctx.current_throttle = 0.0f;

        test_ctx.error_msg = "Test cancelled by user";
        test_ctx.state = STATIC_TEST_FAILED;
        sd_card_resume();  // Resume SD after cancel
    }
}

bool StaticTest_IsRunning(void) {
    return (test_ctx.state != STATIC_TEST_IDLE &&
            test_ctx.state != STATIC_TEST_COMPLETE &&
            test_ctx.state != STATIC_TEST_FAILED);
}

const char* StaticTest_GetFilename(void) {
    return test_filename;
}

const char* StaticTest_StateToString(static_test_state_t state) {
    switch (state) {
        case STATIC_TEST_IDLE:      return "IDLE";
        case STATIC_TEST_INIT:      return "INIT";
        case STATIC_TEST_SETTLE:    return "SETTLE";
        case STATIC_TEST_RAMP_UP:   return "RAMP_UP";
        case STATIC_TEST_HOLD:      return "HOLD";
        case STATIC_TEST_RAMP_DOWN: return "RAMP_DOWN";
        case STATIC_TEST_SAVING:    return "SAVING";
        case STATIC_TEST_COMPLETE:  return "COMPLETE";
        case STATIC_TEST_FAILED:    return "FAILED";
        default:                    return "UNKNOWN";
    }
}
