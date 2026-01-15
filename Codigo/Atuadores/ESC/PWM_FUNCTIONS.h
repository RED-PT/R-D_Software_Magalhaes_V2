/*
 * PWM_FUNCTIONS.h
 *
 * PWM control for ESC motor
 * Uses TIM3 for ESC (per config.h)
 */

#ifndef PWM_FUNCTIONS_H
#define PWM_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>

// ESC PWM duty cycle limits (percentage)
// Standard ESC: 5% = 1ms (min), 10% = 2ms (max) at 50Hz (20ms period)
#define DC_MIN 5.0f      // Minimum throttle (ESC arm signal)
#define DC_MAX 10.0f     // Maximum throttle

// Throttle percentage limits (0-100%)
#define THROTTLE_MIN 0.0f
#define THROTTLE_MAX 100.0f

// Ramp rates (percentage per step)
#define RAMP_RATE_NORMAL 0.5f    // Normal ramp rate
#define RAMP_RATE_EMERGENCY 5.0f // Fast shutdown rate

// Current throttle state
extern float currentThrottle;

// ============================================================================
// Core PWM Functions
// ============================================================================

/**
 * Initialize PWM for ESC
 * Starts TIM3 PWM channels and sets to minimum (arm) signal
 */
void PWM_Init(void);

/**
 * Set throttle directly (0-100%)
 * Converts percentage to duty cycle and applies to PWM
 */
void PWM_SetThrottle(float percentage);

/**
 * Get current throttle percentage
 */
float PWM_GetThrottle(void);

/**
 * Convert motor percentage (0-100%) to duty cycle and apply
 * Internal function used by other functions
 */
void DC_to_Period(float percentage_motor);

// ============================================================================
// Ramped Control Functions
// ============================================================================

/**
 * Ramp motor ON from 0% to target percentage
 * Blocking function with delays
 * @param percentage_motor_max Target throttle percentage (0-100)
 */
void PWM_MOTOR_ON(float percentage_motor_max);

/**
 * Ramp motor OFF from current to 0%
 * Blocking function with delays
 * @param percentage_motor_max Current throttle percentage
 * @param rate_descida Ramp down rate (percentage per step)
 */
void PWM_MOTOR_OFF(float percentage_motor_max, float rate_descida);

// ============================================================================
// Non-blocking Ramp Functions (for use in RTOS)
// ============================================================================

/**
 * Start a non-blocking ramp to target throttle
 * Call PWM_RampUpdate() periodically to progress
 */
void PWM_StartRamp(float target_percentage, float rate);

/**
 * Update ramp progress - call from control loop
 * Returns true when ramp is complete
 */
bool PWM_RampUpdate(void);

/**
 * Check if ramp is in progress
 */
bool PWM_IsRamping(void);

/**
 * Cancel any in-progress ramp
 */
void PWM_CancelRamp(void);

// ============================================================================
// ESC Arm/Disarm
// ============================================================================

/**
 * Send ESC arm signal (minimum throttle for specified duration)
 * Blocking function
 */
void PWM_ArmESC(uint32_t duration_ms);

/**
 * Emergency stop - immediately cut throttle to minimum
 */
void PWM_EmergencyStop(void);

// ============================================================================
// ESC Calibration (min/max throttle range calibration)
// ============================================================================

/**
 * ESC Calibration State Machine
 *
 * Calibration procedure (per HOBBYWING FlyFun ESC manual):
 * 1. PHASE 1: Send MAX throttle (100%) - ESC waits for power cycle
 *    - User should power cycle ESC while receiving max signal
 *    - ESC will beep "123" then 2 short beeps (max accepted)
 * 2. PHASE 2: Send MIN throttle (0%) within 5 seconds
 *    - ESC will accept min, beep cell count, then long beep = done
 */
typedef enum {
    ESC_CAL_IDLE = 0,           // Not calibrating
    ESC_CAL_PHASE1_MAX,         // Sending MAX throttle, waiting for ESC power cycle
    ESC_CAL_PHASE2_MIN,         // Sending MIN throttle to complete calibration
    ESC_CAL_COMPLETE,           // Calibration successful
    ESC_CAL_FAILED              // Calibration failed/cancelled
} esc_calibration_state_t;

typedef struct {
    esc_calibration_state_t state;
    uint32_t phase_start_tick;      // When current phase started
    uint32_t phase1_duration_ms;    // How long to hold MAX (user configurable)
    uint32_t phase2_duration_ms;    // How long to hold MIN before declaring complete
    bool user_acknowledged;          // User confirmed ESC beeped (optional)
} esc_calibration_ctx_t;

// Default timing (can be adjusted)
#define ESC_CAL_PHASE1_DEFAULT_MS   10000   // 10 seconds at MAX (user powers ESC during this)
#define ESC_CAL_PHASE2_DEFAULT_MS   3000    // 3 seconds at MIN

/**
 * Start ESC calibration process
 * @param phase1_ms Duration for MAX throttle phase (ms), 0 = use default
 * @return true if calibration started successfully
 */
bool PWM_ESC_StartCalibration(uint32_t phase1_ms);

/**
 * Update ESC calibration state machine
 * Call periodically (e.g., every 50ms) during calibration
 * @return Current calibration state
 */
esc_calibration_state_t PWM_ESC_CalibrationUpdate(void);

/**
 * Get current calibration state
 */
esc_calibration_state_t PWM_ESC_GetCalibrationState(void);

/**
 * Cancel ongoing ESC calibration
 */
void PWM_ESC_CancelCalibration(void);

/**
 * User acknowledgment that ESC beeped (optional - can skip to phase 2 early)
 */
void PWM_ESC_AcknowledgeBeep(void);

/**
 * Get calibration context for status reporting
 */
const esc_calibration_ctx_t* PWM_ESC_GetCalibrationContext(void);

#endif // PWM_FUNCTIONS_H
