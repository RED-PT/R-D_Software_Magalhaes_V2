/**
 * @file flight_computer_thread.h
 * @brief Flight State Machine Thread Interface
 * @author Tomás Teixeira
 * @date October 10, 2025
 * @version 2.0
 *
 * @details
 * This header defines the interface for the Flight State Machine (FSM) thread,
 * which is the central control component of the Magalhães Flight Computer.
 *
 * ## Thread Responsibilities
 * - **State Management**: Controls system state transitions (BOOT → IDLE → ARMED → FLIGHT, etc.)
 * - **Command Processing**: Handles GS commands (ARM, LAUNCH, ABORT, CALIBRATE, etc.)
 * - **Event Processing**: Responds to internal events (LIFTOFF, APOGEE, TOUCHDOWN)
 * - **Boot Sequence**: Orchestrates system initialization and validation
 * - **Thread Coordination**: Suspends/resumes Estimator and Controller threads
 *
 * ## Thread Configuration
 * | Parameter | Value |
 * |-----------|-------|
 * | Stack Size | 4096 bytes |
 * | Priority | osPriorityAboveNormal1 |
 * | Period | 50ms (20 Hz) |
 *
 * ## Queue Interfaces
 * - **queue_cmd_to_fsm**: Commands from radio thread (input)
 * - **queue_event_to_fsm**: Internal events from estimator (input)
 * - **queue_fsm_events**: Telemetry events to radio (output)
 *
 * @see flight_computer.h for FSM types and structures
 * @see flight_computer_thread.c for implementation
 *
 * @ingroup Flight_Computer
 */

#ifndef FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_
#define FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "flight_computer.h"

/**
 * @brief FSM Thread Entry Point
 * @param argument Unused (reserved for CMSIS-RTOS compatibility)
 *
 * @details
 * Main function for the Flight State Machine thread. This function:
 * 1. Initializes the FSM (fsm_init())
 * 2. Reports thread startup
 * 3. Enters main loop processing commands, events, and state handlers
 *
 * **Main Loop Structure:**
 * - Process all pending commands from queue_cmd_to_fsm
 * - Process all pending events from queue_event_to_fsm
 * - Run calibration/test state machines
 * - Execute current state handler
 * - Periodic statistics logging (every 10s)
 * - Sleep for 50ms
 *
 * @note This function never returns; it runs indefinitely.
 */
void fsm_thread_function(void *argument);

/**
 * @brief Send a telemetry event to the ground station
 * @param type Event type (EVT_STATE_CHANGE, EVT_LIFTOFF, etc.)
 * @param payload Pointer to event-specific payload data
 * @param size Size of payload in bytes
 *
 * @details
 * Public interface for other threads to send telemetry events through
 * the FSM. Events are queued to queue_fsm_events and transmitted by
 * the radio thread.
 *
 * **Common Event Types:**
 * - EVT_STATE_CHANGE: FSM state/substate changed
 * - EVT_LIFTOFF: Rocket launch detected
 * - EVT_APOGEE: Maximum altitude reached
 * - EVT_LANDING: Touchdown confirmed
 * - EVT_ABORT_TRIGGERED: Emergency abort initiated
 * - EVT_BARO_CALIBRATED: Barometer calibration complete
 *
 * @see telemetry_event_type_t for complete event list
 */
void fsm_send_telemetry_event(telemetry_event_type_t type, const void *payload, uint16_t size);

#endif /* FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_ */
