/**
 * @file radio_thread.h
 * @brief TDMA Radio Thread for the Magalhaes Flight Computer (Slave Mode)
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This module implements the radio communication thread using a TDMA scheme
 * where the Flight Computer operates as a slave, synchronized to the Ground
 * Station's timing.
 *
 * Two radio backends are selected at compile time:
 * - **RADIO_INTERFACE_UART**: E22 module via UART DMA (F446ZE dev board)
 * - **RADIO_INTERFACE_SPI**: SX126x via SPI DMA (Buzz V4 PCB / H743)
 *
 * @section radio_arch Architecture
 * The radio thread manages:
 * - TDMA slot timing and synchronization
 * - Telemetry packet transmission (fast, slow, event)
 * - Command reception from Ground Station
 * - Link quality monitoring and statistics
 *
 * @section radio_sync Synchronization
 * The FC synchronizes to the GS using sync beacon packets received in slot 9.
 * If synchronization is lost (multiple missed sync packets), the FC continues
 * transmitting in free-running mode until sync is reacquired.
 *
 * @see telemetry.h for packet structure definitions
 * @see e22_uart_dma.h for UART radio driver (RADIO_INTERFACE_UART)
 * @see lora_sx126x.h for SPI radio driver (RADIO_INTERFACE_SPI)
 */

#ifndef RADIO_RADIO_THREAD_H_
#define RADIO_RADIO_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "defs.h"
#include "Telemetry/telemetry.h"
#include "Flight Computer/flight_computer.h"

/**
 * @defgroup RadioTDMA TDMA State Machine
 * @brief TDMA synchronization state management
 * @{
 */

/**
 * @brief TDMA synchronization state
 *
 * Tracks whether the FC is synchronized to the GS timing.
 */
typedef enum {
    TDMA_UNSYNCED = 0,  /**< Not synchronized, free-running mode */
    TDMA_SYNCED         /**< Synchronized to Ground Station timing */
} tdma_state_t;

/**
 * @brief TDMA context structure
 *
 * Maintains all state for TDMA timing, synchronization, and command
 * acknowledgment. Updated by the radio thread on each superframe.
 */
typedef struct {
    /* Synchronization state */
    tdma_state_t state;             /**< Current TDMA sync state */
    uint8_t frame_id;               /**< Current superframe counter (0-255) */
    uint8_t slot_index;             /**< Current slot within superframe (0-9) */
    uint32_t frame_start_tick;      /**< FreeRTOS tick when current frame started */
    uint32_t last_sync_tick;        /**< FreeRTOS tick of last sync packet received */
    uint8_t missed_sync_count;      /**< Consecutive missed sync packets */

    /* Command acknowledgment tracking */
    uint8_t last_cmd_seq_received;  /**< Sequence number of last received command */
    uint8_t last_cmd_status;        /**< Status of last command: 0=OK, 1=rejected, 2=error */

    /* Packet sequence counters (for loss detection) */
    uint8_t fast_seq;               /**< Fast telemetry packet sequence counter */
    uint8_t slow_seq;               /**< Slow telemetry packet sequence counter */
    uint8_t event_seq;              /**< Event packet sequence counter */
} tdma_ctx_t;

/** @} */ /* End of RadioTDMA group */

/**
 * @defgroup RadioStats Radio Statistics
 * @brief Communication link quality monitoring
 * @{
 */

/**
 * @brief Radio communication statistics
 *
 * Counters for transmitted/received packets and errors.
 * Used for link quality monitoring and debugging.
 */
typedef struct {
    uint32_t tx_fast;           /**< Fast telemetry packets transmitted */
    uint32_t tx_slow;           /**< Slow telemetry packets transmitted */
    uint32_t tx_event;          /**< Event packets transmitted */
    uint32_t rx_cmd;            /**< Command packets received */
    uint32_t rx_sync;           /**< Sync beacon packets received */
    uint32_t crc_errors;        /**< Packets rejected due to CRC mismatch */
    uint32_t sync_corrections;  /**< Number of timing corrections applied */
} radio_stats_t;

/** @} */ /* End of RadioStats group */

/**
 * @defgroup RadioGlobals Radio Global Variables
 * @brief Shared radio state accessible from other modules
 * @{
 */

/**
 * @brief Global TDMA context
 *
 * Contains current TDMA timing and synchronization state.
 * Read by telemetry modules, written by radio thread.
 *
 * @note Access should be protected in multi-threaded context
 */
extern tdma_ctx_t tdma_ctx;

/**
 * @brief Global radio statistics
 *
 * Cumulative counters for radio operations.
 * Can be read by other modules for status display.
 */
extern radio_stats_t radio_stats;

/** @} */ /* End of RadioGlobals group */

/**
 * @defgroup RadioFunctions Radio Thread Functions
 * @brief Radio thread entry point and API
 * @{
 */

/**
 * @brief Radio thread main function
 *
 * FreeRTOS task entry point for the radio communication thread.
 * Implements the TDMA slot timing and packet transmission/reception loop.
 *
 * Thread responsibilities:
 * - Maintain TDMA slot timing
 * - Transmit telemetry packets in appropriate slots
 * - Receive and process commands from GS
 * - Track synchronization status
 * - Update statistics
 *
 * @note This function runs as a FreeRTOS task and never returns
 * @note Task priority should be high to maintain TDMA timing accuracy
 *
 * @par Example
 * @code
 * // In main.c, create the radio task
 * xTaskCreate(radio_thread_function, "Radio", 512, NULL,
 *             configMAX_PRIORITIES - 1, NULL);
 * @endcode
 */
void radio_thread_function();

/**
 * @brief Update sensor data for telemetry transmission
 *
 * Called by the sensor/telemetry thread to provide latest sensor readings
 * for inclusion in telemetry packets. Data is copied to internal buffers.
 *
 * @param[in] imu  Pointer to latest IMU data (can be NULL if unavailable)
 * @param[in] baro Pointer to latest barometer data (can be NULL if unavailable)
 * @param[in] bno  Pointer to latest BNO055 data (can be NULL if unavailable)
 * @param[in] gps  Pointer to latest GPS data (can be NULL if unavailable)
 *
 * @note Thread-safe, can be called from sensor thread context
 * @note Data is copied internally, caller can reuse pointers after return
 *
 * @par Example
 * @code
 * // In sensor thread, after reading sensors
 * IMU_t imu_data;
 * BARO_t baro_data;
 * BNO_t bno_data;
 * GPS_t gps_data;
 *
 * // Read all sensors...
 *
 * // Update radio with latest data
 * radio_update_sensor_data(&imu_data, &baro_data, &bno_data, &gps_data);
 * @endcode
 */
void radio_update_sensor_data(const IMU_t *imu, const BARO_t *baro,
                               const BNO_t *bno, const GPS_t *gps);

/**
 * @brief Phase 3-A (A4): request a TDMA preset switch.
 *
 * The change is queued and applied at the next superframe boundary, so
 * existing in-flight slot timing isn't disrupted. Effective range:
 * - TDMA_MODE_FLIGHT (default): 1000 ms / 10×100 ms slots
 * - TDMA_MODE_TEST_INTERACTIVE: 200 ms / 5×40 ms slots (10 Hz RX/TX)
 */
void radio_request_tdma_mode(tdma_mode_t mode);

/** @brief Currently active TDMA preset (one of tdma_mode_t). */
tdma_mode_t radio_get_tdma_mode(void);

/** @} */ /* End of RadioFunctions group */

#endif /* RADIO_RADIO_THREAD_H_ */
