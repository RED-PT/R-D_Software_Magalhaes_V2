/**
 * @file data_handler_thread.h
 * @brief Data Handler Thread Interface
 * @author Tomás Teixeira
 * @date October 10, 2025
 * @version 2.0
 *
 * @details
 * This header defines the interface for the Data Handler thread, which
 * routes sensor data from circular buffers to destination queues
 * (SD card, telemetry, estimator).
 *
 * ## Thread Responsibilities
 * - **Buffer Management**: Monitor circular buffer fill levels
 * - **Data Routing**: Flush data to SD, telemetry, and estimator queues
 * - **Flow Control**: Prevent buffer overflows through timely flushing
 * - **Diagnostics**: Track overflow and flush statistics
 *
 * ## Thread Configuration
 * | Parameter | Value |
 * |-----------|-------|
 * | Stack Size | 4096 bytes |
 * | Priority | osPriorityHigh2 |
 * | Period | Event-driven (200ms timeout) |
 *
 * ## Flush Triggers
 * 1. **Threshold**: Buffer exceeds 50% capacity (notified by sensors)
 * 2. **Safety**: 1 second timeout since last flush
 * 3. **Opportunistic**: SD queue has space and buffers have data
 *
 * ## Data Flow
 *
 * @verbatim
 *   Sensor Circular Buffers                    Destination Queues
 *   ┌──────────┐                              ┌─────────────────┐
 *   │  cb_imu  │───┐                    ┌────►│ queue_to_sd     │
 *   ├──────────┤   │  ┌──────────────┐  │     ├─────────────────┤
 *   │ cb_baro  │───┼─►│ Data Handler │──┼────►│ queue_to_telem  │
 *   ├──────────┤   │  │    Thread    │  │     ├─────────────────┤
 *   │  cb_mag  │───┤  └──────────────┘  └────►│ queue_to_est    │
 *   ├──────────┤   │                          └─────────────────┘
 *   │  cb_bno  │───┤
 *   ├──────────┤   │
 *   │  cb_gps  │───┘
 *   └──────────┘
 * @endverbatim
 *
 * @see data_handler_thread.c for implementation
 * @see flash_data_handler.h for circular buffer API
 *
 * @defgroup Data_Handler Data Handler
 * @{
 */

#ifndef DATA_HANDLER_DATA_HANDLER_THREAD_H_
#define DATA_HANDLER_DATA_HANDLER_THREAD_H_

#include "flash_data_handler.h"
#include "Threads/create_threads.h"
#include "print.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>

/** @name Flush Configuration
 *  @brief Parameters controlling when buffers are flushed to queues
 *  @{
 */
#define BUFFER_FULL_THRESHOLD_PCT   50      /**< Trigger flush at 50% full */
#define SAFETY_FLUSH_TIMEOUT_MS     1000    /**< Safety: flush every 1s minimum */
/** @} */

/** @name Thread Notification Bits
 *  @brief Bit flags for task notification events
 *  @{
 */
#define DATA_HANDLER_NOTIFY_THRESHOLD   (1 << 0)  /**< Buffer threshold crossed */
/** @} */

/** @brief Minimum SD queue space before attempting flush */
#define MIN_QUEUE_SPACE_FOR_FLUSH       2

/** @brief Data handler thread handle (global for notifications) */
extern osThreadId_t data_handler_thread_id;

/**
 * @brief Data Handler Thread Entry Point
 *
 * @details
 * Main function for the data handler thread. Monitors circular buffers
 * and routes data to destination queues based on:
 * - Threshold notifications from sensors
 * - Safety timeout (1 second)
 * - Opportunistic flushing when queues have space
 *
 * **Statistics Reported (every 10s):**
 * - Buffer fill percentages
 * - Overflow counts per sensor
 * - Queue depths
 * - Flush counts by trigger type
 */
void data_handler_thread_function();

/**
 * @brief Notify data handler that a buffer threshold was crossed
 *
 * @details
 * Called from ISR context when any circular buffer exceeds 50% capacity.
 * Wakes the data handler thread to flush buffers immediately.
 *
 * @note ISR-safe - uses xTaskNotifyFromISR()
 */
void data_handler_notify_threshold(void);

/** @} */ // End of Data_Handler group

#endif /* DATA_HANDLER_DATA_HANDLER_THREAD_H_ */
