/*
 * controller_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef CONTROLLER_CONTROLLER_THREAD_H_
#define CONTROLLER_CONTROLLER_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"

// Motor control modes
typedef enum {
    CTRL_MODE_IDLE = 0,
    CTRL_MODE_MANUAL,           // Direct throttle control
    CTRL_MODE_ALTITUDE_HOLD,    // Hold at target altitude
    CTRL_MODE_VELOCITY_CTRL,    // Control vertical velocity
    CTRL_MODE_LANDING           // Landing sequence
} controller_mode_t;

// Controller state
typedef struct {
    controller_mode_t mode;

    // Setpoints
    float target_altitude_m;
    float target_velocity_ms;
    float manual_throttle;

    // Current state (from estimator)
    float current_altitude_m;
    float current_velocity_ms;

    // Output
    float throttle_output;      // 0.0 to 1.0

    // PID state
    float altitude_error_integral;
    float altitude_error_prev;
    float velocity_error_integral;
    float velocity_error_prev;

    // Limits
    float throttle_min;
    float throttle_max;

    // Status
    bool motor_initialized;
    bool emergency_stop;
} controller_state_t;

// PID gains (can be tuned)
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
} pid_gains_t;

extern controller_state_t ctrl_state;

// Thread function
void controller_thread_function();

// Motor control API (called by FSM)
void controller_init_motor(void);
void controller_set_throttle(float throttle);
void controller_emergency_stop(void);

// Mode control
void controller_set_mode(controller_mode_t mode);
void controller_set_target_altitude(float altitude_m);
void controller_set_target_velocity(float velocity_ms);

// State update (called by estimator)
void controller_update_state(float altitude_m, float velocity_ms);

// Ramp control (for test stand profiles)
void controller_start_ramp(float target_throttle, float duration_s);
void controller_update_ramp(void);

#endif /* CONTROLLER_CONTROLLER_THREAD_H_ */
