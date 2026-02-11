/**
 * @file static_thrust_test.h
 * @brief Static motor thrust test module for the Magalhaes Test Stand
 * @author Tomas Teixeira
 * @date January 2026
 *
 * This module implements automated static thrust testing for motor
 * characterization. It controls motor throttle while recording
 * thrust data from a load cell.
 *
 * @section static_test_overview Overview
 * The static thrust test performs:
 * 1. Load cell initialization and tare
 * 2. Throttle ramp from 0% to max%
 * 3. Hold at maximum throttle
 * 4. Throttle ramp down to 0%
 * 5. Data save to SD card
 *
 * @section static_test_output Output
 * Results are saved as CSV with columns:
 * - timestamp_ms: Time since test start
 * - thrust_n: Force in Newtons
 * - pwm_percent: Current throttle percentage
 * - raw_counts: Load cell ADC counts
 *
 * @section static_test_usage Usage
 * @code
 * // Initialize module
 * StaticTest_Init();
 *
 * // Start test at 50% max throttle
 * if (StaticTest_Start(50)) {
 *     // Update periodically
 *     while (StaticTest_IsRunning()) {
 *         StaticTest_Update();
 *         vTaskDelay(10);
 *     }
 *     // Check result
 *     if (StaticTest_GetState() == STATIC_TEST_COMPLETE) {
 *         printf("Max thrust: %.2f N\n", StaticTest_GetContext()->max_thrust_n);
 *     }
 * }
 * @endcode
 *
 * @see FX29.h for load cell driver
 * @see PWM_FUNCTIONS.h for motor control
 */

#ifndef TESTS_STATIC_THRUST_TEST_H_
#define TESTS_STATIC_THRUST_TEST_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "Sensors/FX29/FX29.h"

/**
 * @defgroup StaticTestConfig Test Configuration
 * @brief Configuration parameters for static thrust tests
 * @{
 */
#define STATIC_TEST_SAMPLE_RATE_HZ      20      /**< Data samples per second */
#define STATIC_TEST_HOLD_DURATION_MS    10000    /**< Hold time at max throttle (ms) */
#define STATIC_TEST_RAMP_DURATION_MS    500
#define STATIC_TEST_SETTLE_DELAY_MS     500     /**< Settle time before test (ms) */
#define STATIC_TEST_MAX_SAMPLES         256     /**< Maximum data points to store */
/** @} */

/**
 * @brief Static test state machine states
 */
typedef enum {
    STATIC_TEST_IDLE = 0,       /**< No test running */
    STATIC_TEST_INIT,           /**< Initializing load cell, performing tare */
    STATIC_TEST_SETTLE,         /**< Waiting for system to settle, then STEP to max */
    STATIC_TEST_RAMP_UP,        /**< (unused - kept for compatibility) */
    STATIC_TEST_HOLD,           /**< Holding at maximum throttle, sampling data */
    STATIC_TEST_RAMP_DOWN,      /**< (unused - kept for compatibility) */
    STATIC_TEST_SAVING,         /**< Saving data to SD card */
    STATIC_TEST_COMPLETE,       /**< Test finished successfully */
    STATIC_TEST_FAILED          /**< Test failed with error */
} static_test_state_t;

/**
 * @brief Single thrust data point
 *
 * Captured during test at STATIC_TEST_SAMPLE_RATE_HZ.
 */
typedef struct {
    uint32_t timestamp_ms;      /**< Time since test start (ms) */
    float thrust_n;             /**< Measured force (Newtons) */
    float pwm_percent;          /**< PWM throttle percentage (0-100) */
    uint16_t raw_counts;        /**< Raw load cell ADC counts */
} thrust_data_point_t;

/**
 * @brief Test configuration parameters
 *
 * Customize test behavior for different motor profiles.
 */
typedef struct {
    uint8_t max_throttle_percent;   /**< Maximum throttle during test (0-100) */
    uint32_t ramp_duration_ms;      /**< Ramp up/down duration */
    uint32_t hold_duration_ms;      /**< Hold duration at max */
    uint32_t sample_interval_ms;    /**< Time between samples */
} static_test_config_t;

/**
 * @brief Static test context
 *
 * Contains all state for a running or completed test.
 */
typedef struct {
    static_test_state_t state;      /**< Current test state */
    static_test_config_t config;    /**< Test configuration */

    /* Timing */
    uint32_t test_start_tick;       /**< Tick when test started */
    uint32_t phase_start_tick;      /**< Tick when current phase started */
    uint32_t last_sample_tick;      /**< Tick of last data sample */

    /* Load cell */
    FX29_t loadcell;                /**< Load cell driver instance */
    bool loadcell_initialized;      /**< Load cell init status */

    /* Data storage */
    thrust_data_point_t data[STATIC_TEST_MAX_SAMPLES];  /**< Data array */
    uint16_t data_count;            /**< Number of samples collected */
    uint16_t max_samples;           /**< Maximum samples to collect */

    /* Current values */
    float current_throttle;         /**< Current throttle percentage */
    float current_thrust;           /**< Current thrust reading (N) */

    /* Results */
    float max_thrust_n;             /**< Maximum thrust recorded (N) */
    float max_thrust_pwm;           /**< PWM at maximum thrust */

    /* Error info */
    const char* error_msg;          /**< Error message if failed */
} static_test_ctx_t;

/**
 * @defgroup StaticTestAPI Static Test API
 * @brief Public functions for static thrust testing
 * @{
 */

/**
 * @brief Initialize static test module
 *
 * Call once at startup before running tests.
 */
void StaticTest_Init(void);

/**
 * @brief Start a new static test
 *
 * Begins a thrust test with specified maximum throttle.
 *
 * @param[in] max_throttle_percent Maximum throttle (1-100%)
 *
 * @return true if test started successfully
 * @return false if already running or invalid parameter
 */
bool StaticTest_Start(uint8_t max_throttle_percent);

/**
 * @brief Update test state machine
 *
 * Call periodically (e.g., every 10ms) during test.
 * Handles state transitions, data collection, and motor control.
 *
 * @return Current test state
 */
static_test_state_t StaticTest_Update(void);

/**
 * @brief Get current test state
 *
 * @return Current state
 */
static_test_state_t StaticTest_GetState(void);

/**
 * @brief Get test context
 *
 * Access full test context for status reporting or data retrieval.
 *
 * @return Pointer to test context (read-only)
 */
const static_test_ctx_t* StaticTest_GetContext(void);

/**
 * @brief Cancel running test
 *
 * Stops motor immediately and ends test.
 */
void StaticTest_Cancel(void);

/**
 * @brief Check if test is running
 *
 * @return true if test in progress
 * @return false if idle or complete
 */
bool StaticTest_IsRunning(void);

/**
 * @brief Get result filename
 *
 * Returns the filename used for saving test data.
 *
 * @return Pointer to filename string
 */
const char* StaticTest_GetFilename(void);

/**
 * @brief Convert state to string
 *
 * For debugging and status display.
 *
 * @param[in] state Test state
 *
 * @return Pointer to state name string
 */
const char* StaticTest_StateToString(static_test_state_t state);

/** @} */

#endif /* TESTS_STATIC_THRUST_TEST_H_ */
