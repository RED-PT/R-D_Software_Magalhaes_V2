/*
 * PWM_FUNCTIONS.c
 *
 * PWM control for ESC motor
 * Uses TIM3 for ESC (per config.h)
 */

#include "PWM_FUNCTIONS.h"
#include "config.h"
#include "main.h"
#include <stdio.h>

// Current state
float currentThrottle = 0.0f;

// Ramp state (for non-blocking ramps)
static bool ramp_active = false;
static float ramp_target = 0.0f;
static float ramp_rate = 0.0f;
static float ramp_direction = 0.0f;  // +1 or -1

// ============================================================================
// Core PWM Functions
// ============================================================================

void PWM_Init(void) {
    // Start PWM on TIM3 (ESC timer per config.h)
    HAL_TIM_PWM_Start(PWM_ESC_TIM, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(PWM_ESC_TIM, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(PWM_ESC_TIM, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(PWM_ESC_TIM, TIM_CHANNEL_4);
    
    // Set to minimum (arm) position
    PWM_SetThrottle(0.0f);
    
    printf("[PWM] Initialized on TIM3\r\n");
}

void DC_to_Period(float percentage_motor) {
    // Convert motor percentage (0-100%) to duty cycle
    // DC_MIN (5%) = 0% throttle, DC_MAX (10%) = 100% throttle
    float DC = DC_MIN + (percentage_motor / 100.0f) * (DC_MAX - DC_MIN);
    
    // Clamp to valid range
    if (DC > DC_MAX) {
        DC = DC_MAX;
    } else if (DC < DC_MIN) {
        DC = DC_MIN;
    }
    
    // Calculate CCR value
    // CCR = (DC / 100) * Period
    uint32_t ccr = (uint32_t)(DC * htim3.Init.Period / 100.0f);
    
    // Apply to all channels (TIM3 for ESC)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
}

void PWM_SetThrottle(float percentage) {
    // Clamp input
    if (percentage < THROTTLE_MIN) percentage = THROTTLE_MIN;
    if (percentage > THROTTLE_MAX) percentage = THROTTLE_MAX;
    
    currentThrottle = percentage;
    DC_to_Period(percentage);
}

float PWM_GetThrottle(void) {
    return currentThrottle;
}

// ============================================================================
// Blocking Ramp Functions (legacy compatibility)
// ============================================================================

void PWM_MOTOR_ON(float percentage_motor_max) {
    printf("[PWM] Ramping ON to %.1f%%\r\n", percentage_motor_max);
    
    for (float pct = 0.0f; pct < percentage_motor_max; pct += RAMP_RATE_NORMAL) {
        PWM_SetThrottle(pct);
        HAL_Delay(50);
    }
    
    // Ensure we hit the target exactly
    PWM_SetThrottle(percentage_motor_max);
    printf("[PWM] Ramp ON complete\r\n");
}

void PWM_MOTOR_OFF(float percentage_motor_max, float rate_descida) {
    printf("[PWM] Ramping OFF from %.1f%% (rate=%.1f)\r\n", percentage_motor_max, rate_descida);
    
    for (float pct = percentage_motor_max; pct > 0.0f; pct -= rate_descida) {
        PWM_SetThrottle(pct);
        HAL_Delay(50);
    }
    
    // Ensure we hit zero
    PWM_SetThrottle(0.0f);
    printf("[PWM] Ramp OFF complete\r\n");
}

// ============================================================================
// Non-blocking Ramp Functions (for RTOS)
// ============================================================================

void PWM_StartRamp(float target_percentage, float rate) {
    // Clamp target
    if (target_percentage < THROTTLE_MIN) target_percentage = THROTTLE_MIN;
    if (target_percentage > THROTTLE_MAX) target_percentage = THROTTLE_MAX;
    
    ramp_target = target_percentage;
    ramp_rate = rate;
    ramp_direction = (target_percentage > currentThrottle) ? 1.0f : -1.0f;
    ramp_active = true;
    
    printf("[PWM] Starting ramp: %.1f%% -> %.1f%% (rate=%.2f)\r\n", 
           currentThrottle, target_percentage, rate);
}

bool PWM_RampUpdate(void) {
    if (!ramp_active) {
        return true;  // No ramp in progress, consider "complete"
    }
    
    // Calculate new throttle
    float new_throttle = currentThrottle + (ramp_direction * ramp_rate);
    
    // Check if we've reached or passed target
    if (ramp_direction > 0) {
        // Ramping up
        if (new_throttle >= ramp_target) {
            new_throttle = ramp_target;
            ramp_active = false;
        }
    } else {
        // Ramping down
        if (new_throttle <= ramp_target) {
            new_throttle = ramp_target;
            ramp_active = false;
        }
    }
    
    PWM_SetThrottle(new_throttle);
    
    return !ramp_active;
}

bool PWM_IsRamping(void) {
    return ramp_active;
}

void PWM_CancelRamp(void) {
    ramp_active = false;
}

// ============================================================================
// ESC Arm/Disarm
// ============================================================================

void PWM_ArmESC(uint32_t duration_ms) {
    printf("[PWM] Arming ESC (sending min signal for %lu ms)...\r\n", duration_ms);
    
    // Send minimum throttle (arm signal)
    PWM_SetThrottle(0.0f);
    
    // Wait for ESC to recognize arm signal
    HAL_Delay(duration_ms);
    
    printf("[PWM] ESC arm complete\r\n");
}

void PWM_EmergencyStop(void) {
    printf("[PWM] EMERGENCY STOP!\r\n");
    
    // Cancel any ramp
    ramp_active = false;
    
    // Immediately set to minimum
    currentThrottle = 0.0f;
    DC_to_Period(0.0f);
}
