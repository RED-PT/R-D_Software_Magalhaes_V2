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

// ============================================================================
// ESC Calibration
// ============================================================================

static esc_calibration_ctx_t esc_cal_ctx = {0};

bool PWM_ESC_StartCalibration(uint32_t phase1_ms) {
    // Cannot start calibration if already in progress
    if (esc_cal_ctx.state == ESC_CAL_PHASE1_MAX ||
        esc_cal_ctx.state == ESC_CAL_PHASE2_MIN) {
        printf("[PWM] ESC calibration already in progress!\r\n");
        return false;
    }

    // Initialize calibration context
    esc_cal_ctx.state = ESC_CAL_PHASE1_MAX;
    esc_cal_ctx.phase_start_tick = HAL_GetTick();
    esc_cal_ctx.phase1_duration_ms = (phase1_ms > 0) ? phase1_ms : ESC_CAL_PHASE1_DEFAULT_MS;
    esc_cal_ctx.phase2_duration_ms = ESC_CAL_PHASE2_DEFAULT_MS;
    esc_cal_ctx.user_acknowledged = false;

    // Cancel any ongoing ramp
    ramp_active = false;

    // Set throttle to MAXIMUM
    printf("[PWM] ESC CALIBRATION STARTED\r\n");
    printf("[PWM] PHASE 1: Sending MAX throttle (100%%)\r\n");
    printf("[PWM] >>> POWER CYCLE the ESC NOW! <<<\r\n");
    printf("[PWM] Waiting %lu ms for ESC to beep twice...\r\n", esc_cal_ctx.phase1_duration_ms);

    PWM_SetThrottle(100.0f);

    return true;
}

esc_calibration_state_t PWM_ESC_CalibrationUpdate(void) {
    if (esc_cal_ctx.state == ESC_CAL_IDLE ||
        esc_cal_ctx.state == ESC_CAL_COMPLETE ||
        esc_cal_ctx.state == ESC_CAL_FAILED) {
        return esc_cal_ctx.state;
    }

    uint32_t elapsed = HAL_GetTick() - esc_cal_ctx.phase_start_tick;

    switch (esc_cal_ctx.state) {
        case ESC_CAL_PHASE1_MAX:
            // Check if user acknowledged or timeout elapsed
            if (esc_cal_ctx.user_acknowledged || elapsed >= esc_cal_ctx.phase1_duration_ms) {
                // Transition to Phase 2: MIN throttle
                printf("[PWM] PHASE 2: Sending MIN throttle (0%%)\r\n");
                printf("[PWM] ESC should beep cell count then long beep...\r\n");

                PWM_SetThrottle(0.0f);

                esc_cal_ctx.state = ESC_CAL_PHASE2_MIN;
                esc_cal_ctx.phase_start_tick = HAL_GetTick();
            }
            break;

        case ESC_CAL_PHASE2_MIN:
            // Wait for phase 2 duration to complete
            if (elapsed >= esc_cal_ctx.phase2_duration_ms) {
                printf("[PWM] ESC CALIBRATION COMPLETE!\r\n");
                printf("[PWM] ESC should now be calibrated for 0-100%% throttle range.\r\n");

                esc_cal_ctx.state = ESC_CAL_COMPLETE;
            }
            break;

        default:
            break;
    }

    return esc_cal_ctx.state;
}

esc_calibration_state_t PWM_ESC_GetCalibrationState(void) {
    return esc_cal_ctx.state;
}

void PWM_ESC_CancelCalibration(void) {
    if (esc_cal_ctx.state == ESC_CAL_PHASE1_MAX ||
        esc_cal_ctx.state == ESC_CAL_PHASE2_MIN) {
        printf("[PWM] ESC calibration CANCELLED\r\n");

        // Set throttle to minimum for safety
        PWM_SetThrottle(0.0f);

        esc_cal_ctx.state = ESC_CAL_FAILED;
    }
}

void PWM_ESC_AcknowledgeBeep(void) {
    if (esc_cal_ctx.state == ESC_CAL_PHASE1_MAX) {
        printf("[PWM] User acknowledged ESC beep - advancing to Phase 2\r\n");
        esc_cal_ctx.user_acknowledged = true;
    }
}

const esc_calibration_ctx_t* PWM_ESC_GetCalibrationContext(void) {
    return &esc_cal_ctx;
}
