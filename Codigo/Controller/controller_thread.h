/**
 * @file controller_thread.h
 * @brief Flight controller thread for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This module implements the flight control system including motor control,
 * altitude hold, and velocity control for the TVC (Thrust Vector Control) rocket.
 *
 * @section ctrl_modes Control Modes
 * | Mode | Description |
 * |------|-------------|
 * | IDLE | No motor control active |
 * | MANUAL | Direct throttle from commands |
 * | ALTITUDE_HOLD | PID control to maintain altitude |
 * | VELOCITY_CTRL | PID control for vertical velocity |
 * | LANDING | Autonomous landing sequence |
 *
 * @section ctrl_pid PID Control
 * The controller implements cascaded PID loops:
 * - Outer loop: Altitude error -> velocity setpoint
 * - Inner loop: Velocity error -> throttle command
 *
 * @see PWM_FUNCTIONS.h for low-level motor control
 * @see sensors_thread.h for sensor data input
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

/**
 * @brief Motor control modes
 *
 * Defines the operating mode of the flight controller.
 */
typedef enum {
    CTRL_MODE_IDLE = 0,         /**< No motor control, motors off */
    CTRL_MODE_MANUAL,           /**< Direct throttle from user/FSM */
    CTRL_MODE_ALTITUDE_HOLD,    /**< PID hold at target altitude */
    CTRL_MODE_VELOCITY_CTRL,    /**< PID control vertical velocity */
    CTRL_MODE_LANDING           /**< Autonomous landing sequence */
} controller_mode_t;

/**
 * @brief Controller state structure
 *
 * Contains all state for the flight controller including setpoints,
 * measured values, PID state, and output.
 */
typedef struct {
    controller_mode_t mode;         /**< Current control mode */

    /* Setpoints (targets) */
    float target_altitude_m;        /**< Target altitude for ALTITUDE_HOLD */
    float target_velocity_ms;       /**< Target vertical velocity */
    float manual_throttle;          /**< Manual throttle command (0-1) */

    /* Current state (from estimator/sensors) */
    float current_altitude_m;       /**< Current measured altitude */
    float current_velocity_ms;      /**< Current vertical velocity */

    /* Output */
    float throttle_output;          /**< Final throttle command (0-1) */

    /* Altitude PID state */
    float altitude_error_integral;  /**< Altitude error integral */
    float altitude_error_prev;      /**< Previous altitude error */

    /* Velocity PID state */
    float velocity_error_integral;  /**< Velocity error integral */
    float velocity_error_prev;      /**< Previous velocity error */

    /* Throttle limits */
    float throttle_min;             /**< Minimum throttle limit */
    float throttle_max;             /**< Maximum throttle limit */

    /* Status */
    bool motor_initialized;         /**< true if motor PWM initialized */
    bool emergency_stop;            /**< true if emergency stop active */
} controller_state_t;

/**
 * @brief PID gains structure
 *
 * Tunable parameters for PID controllers.
 */
typedef struct {
    float kp;                   /**< Proportional gain */
    float ki;                   /**< Integral gain */
    float kd;                   /**< Derivative gain */
    float integral_limit;       /**< Anti-windup integral limit */
} pid_gains_t;

/** @brief Global controller state */
extern controller_state_t ctrl_state;

/**
 * @defgroup CtrlThread Controller Thread
 * @brief Thread entry point
 * @{
 */

/**
 * @brief Controller thread main function
 *
 * FreeRTOS task entry point for the flight controller.
 * Runs the control loop at fixed rate.
 *
 * @note This function runs as a FreeRTOS task and never returns
 */
void controller_thread_function();

/** @} */

/**
 * @defgroup CtrlMotorAPI Motor Control API
 * @brief Functions called by FSM for motor control
 * @{
 */

/**
 * @brief Initialize motor subsystem
 *
 * Initializes PWM hardware and arms the ESC.
 * Must be called before using motor commands.
 */
void controller_init_motor(void);

/**
 * @brief Set throttle directly
 *
 * Sets motor throttle in MANUAL mode.
 *
 * @param[in] throttle Throttle value (0.0 to 1.0)
 */
void controller_set_throttle(float throttle);

/**
 * @brief Emergency stop
 *
 * Immediately cuts motor power. Sets emergency_stop flag.
 */
void controller_emergency_stop(void);

/** @} */

/**
 * @defgroup CtrlModeAPI Mode Control API
 * @brief Functions for changing control mode and setpoints
 * @{
 */

/**
 * @brief Set control mode
 *
 * Changes the operating mode of the controller.
 * Resets PID integrators on mode change.
 *
 * @param[in] mode New control mode
 */
void controller_set_mode(controller_mode_t mode);

/**
 * @brief Set target altitude
 *
 * Sets the altitude setpoint for ALTITUDE_HOLD mode.
 *
 * @param[in] altitude_m Target altitude in meters (AGL)
 */
void controller_set_target_altitude(float altitude_m);

/**
 * @brief Set target velocity
 *
 * Sets the velocity setpoint for VELOCITY_CTRL mode.
 *
 * @param[in] velocity_ms Target vertical velocity (m/s, positive=up)
 */
void controller_set_target_velocity(float velocity_ms);

/** @} */

/**
 * @defgroup CtrlStateAPI State Update API
 * @brief Functions for updating controller with sensor data
 * @{
 */

/**
 * @brief Update state estimate
 *
 * Called by estimator/sensor thread with current state.
 * Used by PID controllers for feedback.
 *
 * @param[in] altitude_m Current altitude (meters AGL)
 * @param[in] velocity_ms Current vertical velocity (m/s)
 */
void controller_update_state(float altitude_m, float velocity_ms);

/** @} */

/**
 * @defgroup CtrlRampAPI Ramp Control API
 * @brief Functions for throttle ramping (test stand profiles)
 * @{
 */

/**
 * @brief Start throttle ramp
 *
 * Initiates a gradual throttle change over specified duration.
 * Used for test stand motor profiles.
 *
 * @param[in] target_throttle Target throttle (0.0 to 1.0)
 * @param[in] duration_s Ramp duration in seconds
 */
void controller_start_ramp(float target_throttle, float duration_s);

/**
 * @brief Update ramp progress
 *
 * Call periodically to advance throttle ramp.
 */
void controller_update_ramp(void);

/** @} */

#endif /* CONTROLLER_CONTROLLER_THREAD_H_ */
