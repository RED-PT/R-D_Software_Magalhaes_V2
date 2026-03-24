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

// ESC pulse width limits (µs)
// With Prescaler=83 and TIM3 at 84MHz: 1 timer count = 1µs exactly
// Hobbywing FlyFun / Futaba standard: 1100µs = min, 1940µs = max
#define PULSE_MIN_US  1100U
#define PULSE_MAX_US  1940U

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
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(PWM_ESC_TIM, TIM_CHANNEL_1);
    printf("[PWM] TIM3 CH1 Start: %s\r\n", (status == HAL_OK) ? "OK" : "FAIL");

    // Set to minimum (arm) position
    PWM_SetThrottle(0.0f);

    // Debug: show timer config
    printf("[PWM] TIM3 Prescaler=%lu, Period=%lu, CCR1=%lu\r\n",
           htim3.Init.Prescaler, htim3.Init.Period, htim3.Instance->CCR1);
    printf("[PWM] Initialized on TIM3 CH1 (PC6)\r\n");
}

// Debug function to test PWM output directly
void PWM_DebugTest(void) {
    printf("\r\n========== PWM DEBUG TEST ==========\r\n");
    printf("[PWM DEBUG] Testing PC6 (TIM3_CH1)\r\n");
    printf("[PWM DEBUG] NUCLEO-F446ZE: PC6 is on CN10 pin 4 (Arduino D1)\r\n");
    printf("[PWM DEBUG] LD1 (Green) will blink during test\r\n");

    // Blink LD1 to show test is starting
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
    }

    // Step 1: Test GPIO manually first
    printf("\r\n[STEP 1] GPIO Toggle Test on PC6 (should see 1Hz square wave)\r\n");
    printf("         Also testing LD2 (Blue) as visual reference\r\n");
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);  // Stop PWM first

    // Reconfigure PC6 as regular GPIO output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // Visual reference
        printf("[GPIO] PC6 = HIGH (LD2 ON)\r\n");
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        printf("[GPIO] PC6 = LOW (LD2 OFF)\r\n");
        HAL_Delay(500);
    }

    // Step 2: Reconfigure as TIM3 PWM
    printf("\r\n[STEP 2] Reconfiguring PC6 as TIM3_CH1 PWM...\r\n");
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Step 3: Start PWM and test different duty cycles
    printf("\r\n[STEP 3] Starting TIM3 PWM...\r\n");
    printf("  TIM3->PSC  = %lu (Prescaler)\r\n", TIM3->PSC);
    printf("  TIM3->ARR  = %lu (Period)\r\n", TIM3->ARR);
    printf("  TIM3->CR1  = 0x%04lX (Control)\r\n", TIM3->CR1);
    printf("  TIM3->CCER = 0x%04lX (Capture/Compare Enable)\r\n", TIM3->CCER);

    // Enable TIM3 clock if not already enabled
    __HAL_RCC_TIM3_CLK_ENABLE();

    // Force timer to be running
    TIM3->CR1 |= TIM_CR1_CEN;  // Enable counter
    TIM3->CCER |= TIM_CCER_CC1E;  // Enable CH1 output

    printf("  After enable: TIM3->CR1 = 0x%04lX, TIM3->CCER = 0x%04lX\r\n", TIM3->CR1, TIM3->CCER);

    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    printf("  HAL_TIM_PWM_Start: %s\r\n", (status == HAL_OK) ? "OK" : "FAIL");

    printf("\r\n[STEP 4] Testing duty cycles (check oscilloscope on PC6)...\r\n");
    printf("         LD1 will toggle every 3 seconds\r\n");

    printf("  50%% duty (CCR=10000) - 10ms HIGH, 10ms LOW at 50Hz\r\n");
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 10000);
    printf("  TIM3->CCR1 = %lu, CNT = %lu\r\n", TIM3->CCR1, TIM3->CNT);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);

    printf("  5%% duty (CCR=1000) - 1ms HIGH, 19ms LOW (ESC MIN)\r\n");
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
    printf("  TIM3->CCR1 = %lu, CNT = %lu\r\n", TIM3->CCR1, TIM3->CNT);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
    HAL_Delay(3000);

    printf("  10%% duty (CCR=2000) - 2ms HIGH, 18ms LOW (ESC MAX)\r\n");
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
    printf("  TIM3->CCR1 = %lu, CNT = %lu\r\n", TIM3->CCR1, TIM3->CNT);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);

    printf("  Back to 5%% (MIN)\r\n");
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

    printf("\r\n========== TEST COMPLETE ==========\r\n");
    printf("If GPIO toggle worked but PWM didn't: Timer config issue\r\n");
    printf("If GPIO toggle didn't work: Check PC6 wiring\r\n");
    printf("PC6 location on NUCLEO-F446ZE: CN10 pin 4 (Arduino D1)\r\n");
}

void DC_to_Period(float percentage_motor) {
    // With Prescaler=83 and TIM3 at 84MHz: 1 timer count = 1µs exactly
    // CCR value directly equals pulse width in microseconds
    // Map 0-100% throttle → PULSE_MIN_US to PULSE_MAX_US
    uint32_t pulse_us = PULSE_MIN_US + (uint32_t)((percentage_motor / 100.0f) * (PULSE_MAX_US - PULSE_MIN_US));

    // Clamp to valid ESC range
    if (pulse_us < PULSE_MIN_US) pulse_us = PULSE_MIN_US;
    if (pulse_us > PULSE_MAX_US) pulse_us = PULSE_MAX_US;

    // Apply to all channels (TIM3 for ESC)
    PWM_ESC_CHANNEL_WRITE = pulse_us;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_us);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_us);
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

    // Ensure PWM timer is running before calibration
    PWM_Init();

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
