/**
 * @file PWM_FUNCTIONS.h
 * @brief PWM control functions for ESC motor control
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This module provides PWM-based control for Electronic Speed Controllers (ESC)
 * used in the Magalhaes rocket motor control system.
 *
 * @section pwm_overview Overview
 * Standard hobby ESCs expect a PWM signal:
 * - Period: 20ms (50Hz)
 * - Pulse width: 1ms (min/arm) to 2ms (max throttle)
 * - This corresponds to 5% to 10% duty cycle at 50Hz
 *
 * @section pwm_features Features
 * - Direct throttle control (0-100%)
 * - Ramped throttle changes (blocking and non-blocking)
 * - ESC arm signal generation
 * - Emergency stop function
 * - ESC calibration state machine
 *
 * @section pwm_safety Safety
 * - Always arm ESC before use
 * - Use emergency stop for immediate shutdown
 * - Test ramp functions at low throttle first
 *
 * @see controller_thread.h for higher-level motor control
 */

#ifndef PWM_FUNCTIONS_H
#define PWM_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup PWMLimits ESC PWM Duty Cycle Limits
 * @brief Duty cycle range for standard ESC signals
 *
 * At 50Hz (20ms period):
 * - 5% duty = 1ms pulse = minimum throttle
 * - 10% duty = 2ms pulse = maximum throttle
 * @{
 */
#define DC_MIN 5.5f         /**< Minimum duty cycle (ESC arm signal) */
#define DC_MAX 9.7f        /**< Maximum duty cycle (full throttle) */
/** @} */

/**
 * @defgroup ThrottleLimits Throttle Percentage Limits
 * @brief User-facing throttle range
 * @{
 */
#define THROTTLE_MIN 0.0f   /**< Minimum throttle percentage */
#define THROTTLE_MAX 100.0f /**< Maximum throttle percentage */
/** @} */

/**
 * @defgroup RampRates Ramp Rate Constants
 * @brief Rate of change for throttle ramping
 * @{
 */
#define RAMP_RATE_NORMAL 0.5f       /**< Normal ramp rate (% per step) */
#define RAMP_RATE_EMERGENCY 5.0f    /**< Fast shutdown rate (% per step) */
/** @} */

/** @brief Current throttle percentage (0-100) */
extern float currentThrottle;

/**
 * @defgroup PWMCore Core PWM Functions
 * @brief Basic PWM control functions
 * @{
 */

/**
 * @brief Initialize PWM for ESC control
 *
 * Starts ESC PWM output and sets to minimum (arm) signal.
 * Must be called before using other PWM functions.
 */
void PWM_Init(void);

/**
 * @brief Debug test function for PWM output
 *
 * Cycles through MIN/MID/MAX duty cycles on ESC PWM output.
 * Use oscilloscope to verify output.
 */
void PWM_DebugTest(void);

/**
 * @brief Set throttle directly
 *
 * Immediately sets throttle to specified percentage.
 * No ramping is applied.
 *
 * @param[in] percentage Throttle value (0-100%)
 */
void PWM_SetThrottle(float percentage);

/**
 * @brief Get current throttle percentage
 *
 * @return Current throttle (0-100%)
 */
float PWM_GetThrottle(void);

/**
 * @brief Convert throttle percentage to PWM duty cycle
 *
 * Internal function that maps 0-100% throttle to 5-10% duty cycle.
 *
 * @param[in] percentage_motor Motor throttle percentage (0-100)
 */
void DC_to_Period(float percentage_motor);

/** @} */

/**
 * @defgroup PWMRamp Ramped Control Functions
 * @brief Functions for gradual throttle changes
 * @{
 */

/**
 * @brief Ramp motor ON (blocking)
 *
 * Gradually increases throttle from 0% to target percentage.
 *
 * @param[in] percentage_motor_max Target throttle percentage (0-100)
 *
 * @note Blocking function with internal delays
 */
void PWM_MOTOR_ON(float percentage_motor_max);

/**
 * @brief Ramp motor OFF (blocking)
 *
 * Gradually decreases throttle from current to 0%.
 *
 * @param[in] percentage_motor_max Starting throttle percentage
 * @param[in] rate_descida Ramp down rate (% per step)
 *
 * @note Blocking function with internal delays
 */
void PWM_MOTOR_OFF(float percentage_motor_max, float rate_descida);

/**
 * @brief Start non-blocking ramp
 *
 * Initiates a gradual throttle change. Call PWM_RampUpdate()
 * periodically to progress the ramp.
 *
 * @param[in] target_percentage Target throttle (0-100%)
 * @param[in] rate Change rate (% per update)
 */
void PWM_StartRamp(float target_percentage, float rate);

/**
 * @brief Update ramp progress
 *
 * Advances the ramp by one step. Call periodically from control loop.
 *
 * @return true if ramp is complete
 * @return false if ramp still in progress
 */
bool PWM_RampUpdate(void);

/**
 * @brief Check if ramp is in progress
 *
 * @return true if currently ramping
 * @return false if at target
 */
bool PWM_IsRamping(void);

/**
 * @brief Cancel in-progress ramp
 *
 * Stops ramping at current throttle level.
 */
void PWM_CancelRamp(void);

/** @} */

/**
 * @defgroup PWMArm ESC Arm/Disarm Functions
 * @brief Functions for ESC arming and safety
 * @{
 */

/**
 * @brief Send ESC arm signal
 *
 * Outputs minimum throttle signal for specified duration.
 * Most ESCs require this before accepting throttle commands.
 *
 * @param[in] duration_ms Arm signal duration in milliseconds
 *
 * @note Blocking function
 */
void PWM_ArmESC(uint32_t duration_ms);

/**
 * @brief Emergency stop
 *
 * Immediately cuts throttle to minimum. Use in case of emergency.
 *
 * @warning Does not ramp down - instant throttle cut
 */
void PWM_EmergencyStop(void);

/** @} */

/**
 * @defgroup ESCCalibration ESC Calibration
 * @brief ESC throttle range calibration functions
 *
 * ESC calibration teaches the ESC the throttle range:
 * 1. Phase 1: Send MAX throttle, power cycle ESC
 * 2. Phase 2: Send MIN throttle to complete
 *
 * @see HOBBYWING FlyFun ESC manual for procedure details
 * @{
 */

/**
 * @brief ESC calibration state machine states
 */
typedef enum {
    ESC_CAL_IDLE = 0,       /**< Not calibrating */
    ESC_CAL_PHASE1_MAX,     /**< Sending MAX throttle, awaiting ESC power cycle */
    ESC_CAL_PHASE2_MIN,     /**< Sending MIN throttle to complete */
    ESC_CAL_COMPLETE,       /**< Calibration successful */
    ESC_CAL_FAILED          /**< Calibration failed or cancelled */
} esc_calibration_state_t;

/**
 * @brief ESC calibration context
 */
typedef struct {
    esc_calibration_state_t state;  /**< Current calibration state */
    uint32_t phase_start_tick;      /**< Tick when current phase started */
    uint32_t phase1_duration_ms;    /**< Phase 1 duration (MAX throttle) */
    uint32_t phase2_duration_ms;    /**< Phase 2 duration (MIN throttle) */
    bool user_acknowledged;         /**< User confirmed ESC beeped */
} esc_calibration_ctx_t;

/**
 * @defgroup ESCCalTiming Calibration Timing Defaults
 * @{
 */
#define ESC_CAL_PHASE1_DEFAULT_MS   30000   /**< Default phase 1 duration (30s) */
#define ESC_CAL_PHASE2_DEFAULT_MS   3000    /**< Default phase 2 duration (3s) */
/** @} */

/**
 * @brief Start ESC calibration
 *
 * Begins calibration by sending MAX throttle signal.
 *
 * @param[in] phase1_ms Duration for MAX phase (0 = use default)
 *
 * @return true if calibration started
 * @return false if already calibrating or error
 */
bool PWM_ESC_StartCalibration(uint32_t phase1_ms);

/**
 * @brief Update calibration state machine
 *
 * Call periodically (e.g., every 50ms) during calibration.
 *
 * @return Current calibration state
 */
esc_calibration_state_t PWM_ESC_CalibrationUpdate(void);

/**
 * @brief Get current calibration state
 *
 * @return Current state
 */
esc_calibration_state_t PWM_ESC_GetCalibrationState(void);

/**
 * @brief Cancel ongoing calibration
 */
void PWM_ESC_CancelCalibration(void);

/**
 * @brief Acknowledge ESC beep
 *
 * User confirms ESC has beeped, can skip to phase 2 early.
 */
void PWM_ESC_AcknowledgeBeep(void);

/**
 * @brief Get calibration context
 *
 * @return Pointer to calibration context for status reporting
 */
const esc_calibration_ctx_t* PWM_ESC_GetCalibrationContext(void);

/** @} */

#endif // PWM_FUNCTIONS_H
