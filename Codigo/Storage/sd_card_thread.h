/**
 * @file sd_card_thread.h
 * @brief SD card logging thread for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This module implements the FreeRTOS task responsible for logging
 * flight data to an SD card using the FatFS file system.
 *
 * @section sd_features Features
 * - Continuous data logging at configurable rate
 * - CSV format output for easy analysis
 * - Automatic file naming with timestamps
 * - Pause/resume support for motor tests
 * - Graceful shutdown with data flush
 *
 * @section sd_format Log File Format
 * Data is logged in CSV format with columns:
 * - timestamp_ms, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
 * - pressure, altitude, temperature, latitude, longitude, gps_alt
 * - heading, roll, pitch, state, substate
 *
 * @section sd_pause Pause/Resume
 * SD operations can be paused during motor tests to prevent
 * interference with time-critical operations. Data is buffered
 * and flushed before pausing.
 *
 * @see fatfs_funcoes_auxiliares.h for file system helpers
 */

#ifndef STORAGE_SD_CARD_THREAD_H_
#define STORAGE_SD_CARD_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "defs.h"
#include "print.h"

/**
 * @defgroup SDCardAPI SD Card Thread API
 * @brief Public functions for SD card logging
 * @{
 */

/**
 * @brief SD card thread main function
 *
 * FreeRTOS task entry point for SD card logging.
 * Handles file creation, data writing, and cleanup.
 *
 * @param[in] argument FreeRTOS task argument (unused)
 *
 * @note This function runs as a FreeRTOS task and never returns
 * @note Creates new log file on each boot with timestamp name
 */
void sd_card_thread_function(void *argument);

/**
 * @brief Close SD card files
 *
 * Flushes any pending data and closes open files.
 * Call before system shutdown or SD card removal.
 *
 * @note Blocks until all data is written
 */
void sd_card_close(void);

/** @} */

/**
 * @defgroup SDCardPause Pause/Resume Functions
 * @brief Control SD operations during motor tests
 *
 * During motor tests, SD card operations may interfere with
 * timing-critical code. These functions allow pausing SD writes
 * while maintaining system stability.
 * @{
 */

/**
 * @brief Pause SD card operations
 *
 * Flushes current buffer and stops new writes.
 * Data received while paused may be lost.
 *
 * @note Call before starting motor tests
 * @note Thread-safe, can be called from any context
 */
void sd_card_pause(void);

/**
 * @brief Resume SD card operations
 *
 * Restarts logging after a pause.
 *
 * @note Call after motor test completes
 * @note Thread-safe, can be called from any context
 */
void sd_card_resume(void);

/**
 * @brief Check if SD operations are paused
 *
 * @return true if currently paused
 * @return false if logging normally
 */
bool sd_card_is_paused(void);

/**
 * @brief SD health for telemetry
 * @return 0 = OK (mounted, log file open), 1 = error
 */
uint8_t sd_card_status(void);

/** @} */

#endif /* STORAGE_SD_CARD_THREAD_H_ */
