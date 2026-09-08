/**
 * @file create_threads.h
 * @brief FreeRTOS Thread Creation and Resource Definitions
 * @author Tomás Teixeira (texman)
 * @date October 6, 2025
 * @version 2.0
 *
 * @details
 * This header defines all FreeRTOS resources (threads, queues, timers) for
 * the Magalhães Flight Computer system. It provides the central point of
 * reference for inter-thread communication and synchronization.
 *
 * ## Thread Architecture
 *
 * @verbatim
 *   ┌────────────────────────────────────────────────────────────────────────┐
 *   │                       Thread Priority Hierarchy                        │
 *   ├────────────────────────────────────────────────────────────────────────┤
 *   │  Priority          │ Thread        │ Stack  │ Period  │ Role          │
 *   ├────────────────────┼───────────────┼────────┼─────────┼───────────────┤
 *   │  osPriorityHigh3   │ Sensors       │ 6144   │ Event   │ Sensor DMA    │
 *   │  osPriorityHigh2   │ DataHandler   │ 4096   │ Event   │ Data routing  │
 *   │  osPriorityHigh1   │ Radio         │ 6144   │ TDMA    │ LoRa comms    │
 *   │  osPriorityHigh    │ Estimator     │ 4096   │ 10ms    │ State est.    │
 *   │  osPriorityHigh    │ Controller    │ 4096   │ 10ms    │ TVC control   │
 *   │  osPriorityAbvNorm1│ FSM           │ 4096   │ 50ms    │ State machine │
 *   │  osPriorityAbvNorm │ Telemetry     │ 6144   │ TDMA    │ Packet build  │
 *   │  osPriorityNormal  │ SDCard        │ 6144   │ Event   │ Data logging  │
 *   └────────────────────┴───────────────┴────────┴─────────┴───────────────┘
 * @endverbatim
 *
 * ## Queue Connections
 *
 * @verbatim
 *                        ┌─────────────┐
 *                        │   Sensors   │
 *                        └──────┬──────┘
 *                               │ sensor_data
 *                               ▼
 *                        ┌─────────────┐
 *                        │ DataHandler │
 *                        └─┬────────┬──┘
 *              estimator_q │        │ sd_card_q
 *                          ▼        ▼
 *              ┌───────────┐  ┌─────────┐
 *              │ Estimator │  │ SDCard  │
 *              └─────┬─────┘  └─────────┘
 *                    │ event_to_fsm
 *                    ▼
 *              ┌───────────┐
 *              │    FSM    │◄─── queue_cmd_to_fsm ◄── Radio
 *              └─────┬─────┘
 *                    │ queue_fsm_events
 *                    ▼
 *              ┌───────────┐
 *              │ Telemetry │
 *              └─────┬─────┘
 *                    │ queue_to_radio_tx
 *                    ▼
 *              ┌───────────┐
 *              │   Radio   │ ◄──► LoRa Module
 *              └───────────┘
 * @endverbatim
 *
 * @see create_threads.c for implementation
 *
 * @defgroup Thread_Management Thread Management
 * @{
 */

#ifndef INC_CREATE_THREADS_H_
#define INC_CREATE_THREADS_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cmsis_os2.h"
#include "message_buffer.h"
#include "queue.h"
#include "timers.h"

#include "Sensors/sensors_thread.h"
#include "Data Handler/data_handler_thread.h"
#include "Estimator/estimator_thread.h"
#include "Controller/controller_thread.h"
#include "Storage/sd_card_thread.h"
#include "Telemetry/telemetry_thread.h"
#include "Radio/radio_thread.h"
#include "Flight Computer/flight_computer_thread.h"

/** @name Thread Handles
 *  @brief FreeRTOS thread identifiers for all system threads
 *  @{
 */
extern osThreadId_t defaultTaskHandle;      /**< CubeMX default task (terminated on startup) */
extern osThreadId_t sensors_thread_id;      /**< Sensor acquisition thread */
extern osThreadId_t data_handler_thread_id; /**< Data routing thread */
extern osThreadId_t estimator_thread_id;    /**< State estimation thread */
extern osThreadId_t controller_thread_id;   /**< Flight controller thread */
extern osThreadId_t sd_card_thread_id;      /**< SD card logging thread */
extern osThreadId_t telemetry_thread_id;    /**< Telemetry packet thread */
extern osThreadId_t radio_thread_id;        /**< LoRa radio thread */
extern osThreadId_t fsm_thread_id;          /**< Flight state machine thread */
/** @} */

/** @name Thread Attributes
 *  @brief Configuration structures for thread creation (stack, priority, name)
 *  @{
 */
extern const osThreadAttr_t sensors_thread_attr;
extern const osThreadAttr_t data_handler_thread_attr;
extern const osThreadAttr_t estimator_thread_attr;
extern const osThreadAttr_t controller_thread_attr;
extern const osThreadAttr_t sd_card_thread_attr;
extern const osThreadAttr_t telemetry_thread_attr;
extern const osThreadAttr_t radio_thread_attr;
extern const osThreadAttr_t fsm_thread_attr;
/** @} */

/** @name Software Timers
 *  @brief FreeRTOS software timers for periodic sensor polling
 *  @{
 */
extern TimerHandle_t xBaroTimer;  /**< MS5607 barometer polling timer */
extern TimerHandle_t xBnoTimer;   /**< BNO055 orientation polling timer */
/** @} */

/** @name Inter-Thread Queues
 *  @brief FreeRTOS queues for thread-safe data exchange
 *  @{
 */
extern QueueHandle_t queue_fsm_events;      /**< FSM events to radio for TX */
extern QueueHandle_t queue_cmd_to_fsm;      /**< Parsed commands to FSM */
extern QueueHandle_t queue_event_to_fsm;    /**< Internal events to FSM */
/** @} */

/* (GPS stream buffer removed — GPS RX uses its own circular DMA buffer.) */

/**
 * @brief Create all system threads, queues, and timers
 *
 * @details
 * This function initializes all FreeRTOS resources for the flight computer:
 * 1. Terminates the CubeMX default task
 * 2. Creates inter-thread queues (FSM events, commands, etc.)
 * 3. Creates software timers (barometer, BNO055)
 * 4. Creates all system threads in proper order
 *
 * **Thread Creation Order:**
 * 1. SDCard - Slow, needs early init for boot logging
 * 2. DataHandler - Routes data to other threads
 * 3. Sensors - Starts hardware init
 * 4. FSM - Central control
 * 5. Estimator - State estimation (starts suspended)
 * 6. Controller - Flight control (starts suspended)
 * 7. Telemetry - Packet building
 * 8. Radio - Communication
 *
 * @warning This function must be called from the default FreeRTOS task.
 * @note After this function returns, the calling task is terminated.
 */
void create_threads(void);

/** @} */ // End of Thread_Management group

#endif /* INC_CREATE_THREADS_H_ */

