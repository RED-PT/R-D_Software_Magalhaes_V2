/**
 * @file telemetry_thread.h
 * @brief Telemetry Data Aggregation Thread Interface
 * @author Tomás Teixeira
 * @date 2025
 * @version 2.0
 *
 * @details
 * This header defines the interface for the Telemetry thread, which
 * aggregates sensor data and provides it to the Radio thread for
 * transmission to the ground station.
 *
 * ## Thread Responsibilities
 * - **Data Aggregation**: Collect latest sensor readings from queues
 * - **Data Caching**: Maintain latest values for each sensor type
 * - **Radio Interface**: Periodically update Radio thread with current data
 *
 * ## Thread Configuration
 * | Parameter | Value |
 * |-----------|-------|
 * | Stack Size | 6144 bytes |
 * | Priority | osPriorityAboveNormal |
 * | Update Rate | 20 Hz (50ms) |
 *
 * ## Data Flow
 *
 * @verbatim
 *   queue_to_telemetry
 *         │
 *         ▼
 *   ┌─────────────┐
 *   │  Telemetry  │──► radio_update_sensor_data()
 *   │   Thread    │        │
 *   └─────────────┘        ▼
 *                    ┌───────────┐
 *                    │   Radio   │
 *                    │   Thread  │
 *                    └───────────┘
 * @endverbatim
 *
 * ## Cached Sensor Data
 * | Type | Structure | Source |
 * |------|-----------|--------|
 * | IMU | IMU_t | ASM330LHHX (416 Hz) |
 * | BARO | BARO_t | MS5607 (50 Hz) |
 * | BNO | BNO_t | BNO055 (100 Hz) |
 * | GPS | GPS_t | u-blox (1 Hz) |
 *
 * @see telemetry_thread.c for implementation
 * @see radio_thread.h for packet transmission
 *
 * @defgroup Telemetry Telemetry System
 * @{
 */

#ifndef TELEMETRY_TELEMETRY_THREAD_H_
#define TELEMETRY_TELEMETRY_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "defs.h"
#include "Data Handler/flash_data_handler.h"

/**
 * @brief Telemetry Thread Entry Point
 *
 * @details
 * Main function for the telemetry data aggregation thread. This function:
 * 1. Receives sensor data from queue_to_telemetry
 * 2. Caches the latest reading for each sensor type
 * 3. Periodically (20 Hz) calls radio_update_sensor_data() to provide
 *    the radio thread with current sensor values
 *
 * **Design Note:**
 * This thread decouples sensor sampling rates from telemetry rates.
 * Sensors may update at different frequencies (IMU at 416 Hz, GPS at 1 Hz),
 * but telemetry always has the most recent data available.
 *
 * @note Does not directly handle radio - that's the Radio thread's job.
 */
void telemetry_thread_function(void *argument);

/** @} */ // End of Telemetry group

#endif /* TELEMETRY_TELEMETRY_THREAD_H_ */
