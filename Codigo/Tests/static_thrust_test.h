/*
 * static_thrust_test.h
 *
 * Static Motor Thrust Test Module
 * Records (Thrust, PWM) data points during throttle ramp
 *
 *  Created on: Jan 11, 2026
 *      Author: Tomas Teixeira
 */

#ifndef TESTS_STATIC_THRUST_TEST_H_
#define TESTS_STATIC_THRUST_TEST_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Sensors/FX29/FX29.h"

// Test configuration
#define STATIC_TEST_SAMPLE_RATE_HZ      20      // Samples per second
#define STATIC_TEST_RAMP_DURATION_MS    5000    // Time to ramp from 0 to max %
#define STATIC_TEST_HOLD_DURATION_MS    2000    // Hold at max before ramp down
#define STATIC_TEST_SETTLE_DELAY_MS     500     // Delay before starting test
#define STATIC_TEST_MAX_SAMPLES         256     // Max data points to store

// Test states
typedef enum {
    STATIC_TEST_IDLE = 0,
    STATIC_TEST_INIT,           // Initializing load cell, taring
    STATIC_TEST_SETTLE,         // Waiting for system to settle
    STATIC_TEST_RAMP_UP,        // Ramping throttle 0 -> max%
    STATIC_TEST_HOLD,           // Holding at max throttle
    STATIC_TEST_RAMP_DOWN,      // Ramping throttle max% -> 0
    STATIC_TEST_SAVING,         // Saving data to SD card
    STATIC_TEST_COMPLETE,       // Test finished successfully
    STATIC_TEST_FAILED          // Test failed (error)
} static_test_state_t;

// Single data point
typedef struct {
    uint32_t timestamp_ms;
    float thrust_n;         // Force in Newtons
    float pwm_percent;      // PWM percentage (0-100)
    uint16_t raw_counts;    // Raw ADC counts for debugging
} thrust_data_point_t;

// Test configuration
typedef struct {
    uint8_t max_throttle_percent;   // Max throttle (e.g., 20 for 20%)
    uint32_t ramp_duration_ms;      // Ramp time
    uint32_t hold_duration_ms;      // Hold time at max
    uint32_t sample_interval_ms;    // Time between samples
} static_test_config_t;

// Test context
typedef struct {
    static_test_state_t state;
    static_test_config_t config;

    // Timing
    uint32_t test_start_tick;
    uint32_t phase_start_tick;
    uint32_t last_sample_tick;

    // Load cell
    FX29_t loadcell;
    bool loadcell_initialized;

    // Data storage
    thrust_data_point_t data[STATIC_TEST_MAX_SAMPLES];
    uint16_t data_count;
    uint16_t max_samples;

    // Current values
    float current_throttle;
    float current_thrust;

    // Results
    float max_thrust_n;
    float max_thrust_pwm;

    // Error info
    const char* error_msg;
} static_test_ctx_t;

// Initialize the static test module
void StaticTest_Init(void);

// Start a new static test
// @param max_throttle_percent: Maximum throttle percentage (1-100)
// @return true if test started successfully
bool StaticTest_Start(uint8_t max_throttle_percent);

// Update the test state machine (call periodically, e.g., every 10ms)
// @return Current test state
static_test_state_t StaticTest_Update(void);

// Get current test state
static_test_state_t StaticTest_GetState(void);

// Get test context for status reporting
const static_test_ctx_t* StaticTest_GetContext(void);

// Cancel running test
void StaticTest_Cancel(void);

// Check if a test is currently running
bool StaticTest_IsRunning(void);

// Get test result filename
const char* StaticTest_GetFilename(void);

// State to string for debugging
const char* StaticTest_StateToString(static_test_state_t state);

#endif /* TESTS_STATIC_THRUST_TEST_H_ */
