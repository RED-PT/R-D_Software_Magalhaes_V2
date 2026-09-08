/**
 * @file estimator_thread.h
 * @brief State Estimation Thread Interface
 * @author Tomás Teixeira
 * @date October 2025
 * @version 2.0
 *
 * @details
 * This header defines the interface for the State Estimation thread, which
 * processes sensor data to estimate the rocket's navigation state and
 * detect critical flight events.
 *
 * ## Thread Responsibilities
 * - **State Estimation**: Calculate altitude and velocity from sensor data
 * - **Flight Event Detection**: Detect LIFTOFF, APOGEE, FLARE, TOUCHDOWN, LANDED
 * - **Safety Monitoring**: Check altitude and velocity limits
 * - **FSM Event Generation**: Send internal events to FSM thread
 *
 * ## Thread Configuration
 * | Parameter | Value |
 * |-----------|-------|
 * | Stack Size | 4096 bytes |
 * | Priority | osPriorityHigh |
 * | Period | Event-driven (100ms timeout) |
 *
 * ## Flight Event Detection
 *
 * @verbatim
 *   ARMED  ───► LIFTOFF ───► ASCENT ───► APOGEE ───► DESCENT ───► FLARE ───► TOUCHDOWN ───► LANDED
 *            vel > 2m/s    climbing   vel < 0     descending    alt ≤ 5m    alt ≤ 0.5m     stable 3s
 * @endverbatim
 *
 * ## Input Queue
 * - **queue_to_estimator**: Receives DATA_TYPE_BARO and DATA_TYPE_IMU packets
 *
 * ## Output Events (to FSM)
 * - FSM_EVT_LIFTOFF: Velocity exceeds 2 m/s
 * - FSM_EVT_APOGEE: Velocity crosses zero (descending)
 * - FSM_EVT_FLARE_ALT: Altitude reaches flare threshold
 * - FSM_EVT_TOUCHDOWN: Altitude below 0.5m
 * - FSM_EVT_LANDED: Stable on ground for 3 seconds
 * - FSM_EVT_ALTITUDE_LIMIT: Safety limit exceeded
 * - FSM_EVT_VELOCITY_LIMIT: Safety limit exceeded
 *
 * @see estimator_thread.c for implementation
 * @see flight_computer.h for FSM event types
 *
 * @defgroup Estimator State Estimation
 * @{
 */

#ifndef ESTIMATOR_ESTIMATOR_THREAD_H_
#define ESTIMATOR_ESTIMATOR_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "print.h"

#include "Data Handler/flash_data_handler.h"
#include "Flight Computer/flight_computer.h"

/**
 * @brief Estimator Thread Entry Point
 *
 * @details
 * Main function for the state estimation thread. This function:
 * 1. Receives sensor data from queue_to_estimator
 * 2. Updates altitude and velocity estimates
 * 3. Detects flight events and sends them to FSM
 * 4. Monitors safety limits
 *
 * **Current Implementation:**
 * - Simple altitude differentiation for velocity
 * - Barometer-based altitude only
 *
 * **Future Improvements:**
 * - Kalman filter fusion of IMU and barometer
 * - GPS-aided navigation
 * - BNO055 orientation integration
 *
 * @note Starts suspended; resumed by FSM when entering ARMED state.
 */
void estimator_thread_function(void *argument);

/** @} */ // End of Estimator group

#endif /* ESTIMATOR_ESTIMATOR_THREAD_H_ */
