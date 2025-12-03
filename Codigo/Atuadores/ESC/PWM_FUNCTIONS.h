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

#endif // PWM_FUNCTIONS_H
