/*
 * controller_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 *
 * Motor control thread using PWM_FUNCTIONS for ESC control
 * PID altitude/velocity control
 */

#include "controller_thread.h"
#include "Flight Computer/flight_computer.h"
#include "Data Handler/flash_data_handler.h"
#include "Atuadores/ESC/PWM_FUNCTIONS.h"
#include "cmsis_os.h"
#include <math.h>

// PID update rate
#define CONTROL_LOOP_PERIOD_MS  20  // 50Hz control loop

// Global controller state
controller_state_t ctrl_state = {0};

// PID gains (tune these for your vehicle!)
static pid_gains_t altitude_pid = {
    .kp = 0.5f,
    .ki = 0.1f,
    .kd = 0.2f,
    .integral_limit = 0.3f
};

static pid_gains_t velocity_pid = {
    .kp = 0.8f,
    .ki = 0.05f,
    .kd = 0.1f,
    .integral_limit = 0.2f
};

// ============================================================================
// Motor Control API (called by FSM)
// ============================================================================

void controller_init_motor(void) {
    printf("[CTRL] Initializing motor/ESC...\r\n");

    // Initialize PWM hardware (timer per config.h) — outputs the min/arm pulse
    PWM_Init();

    /* NOTE: no blocking arm delay here anymore. This is called from the FSM
     * thread, and the old PWM_ArmESC(2000) = HAL_Delay(2000) froze command
     * processing (including ABORT) for 2 s. The ESC sees the minimum pulse
     * from PWM_Init() onward; the ARMED sub-state machine's
     * MOTOR_ARM_CAL_DELAY_MS (3 s) wait provides the arming dwell time. */

    ctrl_state.motor_initialized = true;
    ctrl_state.emergency_stop = false;
    ctrl_state.throttle_output = 0.0f;

    printf("[CTRL] Motor initialized (arm dwell handled by FSM state machine)\r\n");
}

void controller_set_throttle(float throttle) {
    if (ctrl_state.emergency_stop) {
        throttle = 0.0f;
    }

    // Apply limits from profile (0.0-1.0 normalized).
    // An explicit 0 (stop/disarm) bypasses throttle_min: with a profile
    // where throttle_min > 0, "set 0" used to leave the motor spinning
    // at min throttle.
    if (throttle > 0.0f && throttle < ctrl_state.throttle_min) throttle = ctrl_state.throttle_min;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > ctrl_state.throttle_max) throttle = ctrl_state.throttle_max;

    ctrl_state.throttle_output = throttle;
    ctrl_state.manual_throttle = throttle;

    if (ctrl_state.motor_initialized) {
        // Convert 0.0-1.0 to 0-100%
        PWM_SetThrottle(throttle * 100.0f);
    }
}

void controller_emergency_stop(void) {
    printf("[CTRL] EMERGENCY STOP!\r\n");

    ctrl_state.emergency_stop = true;
    ctrl_state.throttle_output = 0.0f;

    // Use PWM emergency stop
    PWM_EmergencyStop();

    // Reset PID state
    ctrl_state.altitude_error_integral = 0.0f;
    ctrl_state.velocity_error_integral = 0.0f;
}

void controller_set_mode(controller_mode_t mode) {
    printf("[CTRL] Mode: %d -> %d\r\n", ctrl_state.mode, mode);

    // Reset PID state on mode change
    ctrl_state.altitude_error_integral = 0.0f;
    ctrl_state.altitude_error_prev = 0.0f;
    ctrl_state.velocity_error_integral = 0.0f;
    ctrl_state.velocity_error_prev = 0.0f;

    ctrl_state.mode = mode;
}

void controller_set_target_altitude(float altitude_m) {
    ctrl_state.target_altitude_m = altitude_m;
}

void controller_set_target_velocity(float velocity_ms) {
    ctrl_state.target_velocity_ms = velocity_ms;
}

void controller_update_state(float altitude_m, float velocity_ms) {
    ctrl_state.current_altitude_m = altitude_m;
    ctrl_state.current_velocity_ms = velocity_ms;
}

// ============================================================================
// PID Control Functions
// ============================================================================

static float pid_compute(float error, float *integral, float *prev_error,
                          const pid_gains_t *gains, float dt) {
    // Proportional
    float p_term = gains->kp * error;

    // Integral with anti-windup
    *integral += error * dt;
    if (*integral > gains->integral_limit) *integral = gains->integral_limit;
    if (*integral < -gains->integral_limit) *integral = -gains->integral_limit;
    float i_term = gains->ki * (*integral);

    // Derivative
    float derivative = (error - *prev_error) / dt;
    float d_term = gains->kd * derivative;
    *prev_error = error;

    return p_term + i_term + d_term;
}

static float compute_altitude_control(float target_alt, float current_alt, float dt) {
    float error = target_alt - current_alt;

    float output = pid_compute(error,
                                &ctrl_state.altitude_error_integral,
                                &ctrl_state.altitude_error_prev,
                                &altitude_pid, dt);

    // Add feedforward for hover (gravity compensation)
    // This is vehicle-specific - adjust for your thrust-to-weight ratio
    float hover_throttle = 0.5f;  // ~50% throttle to hover

    return hover_throttle + output;
}

static float compute_velocity_control(float target_vel, float current_vel, float dt) {
    float error = target_vel - current_vel;

    float output = pid_compute(error,
                                &ctrl_state.velocity_error_integral,
                                &ctrl_state.velocity_error_prev,
                                &velocity_pid, dt);

    // Add feedforward
    float hover_throttle = 0.5f;

    return hover_throttle + output;
}

// ============================================================================
// Control Loop
// ============================================================================

static void run_control_loop(float dt) {
    if (ctrl_state.emergency_stop) {
        PWM_EmergencyStop();
        return;
    }

    float throttle_cmd = 0.0f;

    switch (ctrl_state.mode) {
        case CTRL_MODE_IDLE:
            throttle_cmd = 0.0f;
            break;

        case CTRL_MODE_MANUAL:
            throttle_cmd = ctrl_state.manual_throttle;
            break;

        case CTRL_MODE_ALTITUDE_HOLD:
            throttle_cmd = compute_altitude_control(
                ctrl_state.target_altitude_m,
                ctrl_state.current_altitude_m,
                dt
            );
            break;

        case CTRL_MODE_VELOCITY_CTRL:
            throttle_cmd = compute_velocity_control(
                ctrl_state.target_velocity_ms,
                ctrl_state.current_velocity_ms,
                dt
            );
            break;

        case CTRL_MODE_LANDING:
            // Landing uses velocity control with decreasing target
            // Target velocity is negative (descending)
            throttle_cmd = compute_velocity_control(
                -fsm_ctx.profile.touchdown_velocity_ms,
                ctrl_state.current_velocity_ms,
                dt
            );
            break;
    }

    // Apply limits (normalized 0-1)
    if (throttle_cmd < ctrl_state.throttle_min) throttle_cmd = ctrl_state.throttle_min;
    if (throttle_cmd > ctrl_state.throttle_max) throttle_cmd = ctrl_state.throttle_max;

    ctrl_state.throttle_output = throttle_cmd;

    if (ctrl_state.motor_initialized) {
        // Convert 0.0-1.0 to 0-100%
        PWM_SetThrottle(throttle_cmd * 100.0f);
    }
}

// ============================================================================
// Ramp Control (for test stand profiles)
// ============================================================================

void controller_start_ramp(float target_throttle, float duration_s) {
    // Calculate rate based on duration and control loop period
    float steps = (duration_s * 1000.0f) / CONTROL_LOOP_PERIOD_MS;
    float rate = (target_throttle * 100.0f) / steps;  // Rate in % per step

    printf("[CTRL] Starting ramp to %.1f%% over %.1fs\r\n",
           target_throttle * 100.0f, duration_s);

    PWM_StartRamp(target_throttle * 100.0f, rate);
    ctrl_state.mode = CTRL_MODE_MANUAL;
}

void controller_update_ramp(void) {
    if (PWM_IsRamping()) {
        PWM_RampUpdate();
        ctrl_state.throttle_output = PWM_GetThrottle() / 100.0f;
    }
}

// ============================================================================
// Main Thread
// ============================================================================

void controller_thread_function(void *argument) {
    (void)argument;
    printf("[CTRL] Controller Thread started\r\n");
    fsm_report_thread_started("CONTROLLER");

    // Initialize state
    memset(&ctrl_state, 0, sizeof(ctrl_state));
    ctrl_state.mode = CTRL_MODE_IDLE;
    ctrl_state.throttle_min = 0.0f;
    ctrl_state.throttle_max = 1.0f;

    TickType_t last_loop_tick = xTaskGetTickCount();
    TickType_t last_stats_tick = xTaskGetTickCount();

    // Thread starts suspended, waiting for FSM to resume us
    printf("[CTRL] Suspending until armed...\r\n");
    vTaskSuspend(NULL);

    printf("[CTRL] Resumed - control active\r\n");

    // Get limits from profile
    ctrl_state.throttle_min = fsm_ctx.profile.throttle_min;
    ctrl_state.throttle_max = fsm_ctx.profile.throttle_max;

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Calculate dt for PID
        float dt = (float)(now - last_loop_tick) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;  // Minimum dt to avoid divide by zero
        last_loop_tick = now;

        // Update limits from FSM profile (in case they changed)
        ctrl_state.throttle_min = fsm_ctx.profile.throttle_min;
        ctrl_state.throttle_max = fsm_ctx.profile.throttle_max;

        // Update ramp if active
        controller_update_ramp();

        // Run control loop
        run_control_loop(dt);

        // Periodic stats
        if ((now - last_stats_tick) >= pdMS_TO_TICKS(5000)) {
            printf("[CTRL] Mode=%d Alt=%.1f/%.1f Vel=%.2f Thr=%.1f%%\r\n",
                   ctrl_state.mode,
                   ctrl_state.current_altitude_m,
                   ctrl_state.target_altitude_m,
                   ctrl_state.current_velocity_ms,
                   ctrl_state.throttle_output * 100.0f);
            last_stats_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}
