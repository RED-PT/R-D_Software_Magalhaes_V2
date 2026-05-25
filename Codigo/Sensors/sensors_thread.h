/**
 * @file sensors_thread.h
 * @brief Sensor management thread for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This module implements the FreeRTOS task responsible for coordinating
 * all sensor readings. It manages multiple sensors across different buses
 * (SPI, I2C, UART) using DMA for efficient data transfer.
 *
 * @section sensor_hw Sensor Hardware
 * SPI instances vary by board; actual peripherals are abstracted via
 * config.h macros (SPI_IMU_BARO, SPI_MAG, I2C_BNO, UART_UBLOX).
 *
 * | Sensor | config.h macro | F446ZE (Nucleo) | H743ZI (Buzz V4) | Function |
 * |------------|----------------|-----------------|------------------|---------------------------|
 * | ASM330LHHX | SPI_IMU_BARO | SPI1 | SPI1 | 6-axis IMU (accel + gyro) |
 * | MMC5983MA | SPI_MAG | SPI3 | SPI6 | 3-axis magnetometer |
 * | MS5607 | SPI_IMU_BARO | SPI1 | SPI1 | Barometric pressure/altitude |
 * | BNO055 | I2C_BNO | I2C1 | I2C1 | 9-DOF with sensor fusion |
 * | UBLOX GPS | UART_UBLOX | USART2 | USART1 | Position/velocity |
 *
 * @section sensor_timing Timing
 * - IMU/Magnetometer: Interrupt-driven (data ready)
 * - Barometer: Timer-driven (100ms period)
 * - BNO055: Timer-driven (100ms period)
 * - GPS: Continuous UART DMA
 *
 * @see defs.h for sensor data structures
 */

#ifndef SENSORS_THREAD_H
#define SENSORS_THREAD_H

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "print.h"
#include <string.h>
#include <limits.h>

#include "Data Handler/flash_data_handler.h"
#include "ASM330LHHX/ASM330LHHX.h"
#include "MMC5983MA/MMC5983MA.h"
#include "MS5607/MS5607.h"
#include "BNO055/BNO055.h"
#include "GPS/GPS.h"
#include "Threads/create_threads.h"

/**
 * @defgroup SensorInstances Sensor Device Instances
 * @brief Global sensor device structures
 * @{
 */

/** @brief ASM330LHHX 6-axis IMU device instance */
extern ASM330LHHX_t imu_device;

/** @brief MMC5983MA magnetometer device instance */
extern MMC5983MA_t mag_device;

/** @brief MS5607 barometer device instance */
extern MS5607_t baro_device;

/** @brief BNO055 9-DOF IMU device instance */
extern BNO055_t bno_device;

/** @brief UBLOX GPS device instance */
extern UBLOX_GPS_t gps_device;

/** @} */ /* End of SensorInstances group */

/**
 * @defgroup SensorBusTracking Bus Activity Tracking
 * @brief Tracks which sensor is currently using each bus
 *
 * Used by DMA callbacks to route data to the correct sensor driver.
 * @{
 */

/**
 * @brief Active sensor identifier for bus arbitration
 */
typedef enum {
    ACTIVE_SENSOR_NONE = 0,     /**< No sensor active on bus */
    ACTIVE_SENSOR_IMU,          /**< ASM330LHHX is using the bus */
    ACTIVE_SENSOR_BARO,         /**< MS5607 is using the bus */
    ACTIVE_SENSOR_MAG,          /**< MMC5983MA is using the bus */
    ACTIVE_SENSOR_BNO           /**< BNO055 is using the bus */
} active_sensor_t;

/** @brief Current active sensor on SPI_IMU_BARO bus (IMU or barometer) */
extern volatile active_sensor_t spi1_active_sensor;

/** @brief Current active sensor on SPI_MAG bus (magnetometer; SPI3 on F446ZE, SPI6 on H743ZI) */
extern volatile active_sensor_t spi_mag_active_sensor;

/** @brief Current active sensor on I2C1 bus */
extern volatile active_sensor_t i2c1_active_sensor;

/** @} */ /* End of SensorBusTracking group */

/**
 * @defgroup SensorNotifications Thread Notification Bits
 * @brief FreeRTOS task notification flags for sensor events
 *
 * The sensor thread uses task notifications to wake up when
 * sensor data is available or timers expire.
 * @{
 */
#define SENSOR_NOTIFY_IMU_DRDY      (1 << 0)    /**< IMU data ready interrupt */
#define SENSOR_NOTIFY_MAG_DRDY      (1 << 1)    /**< Magnetometer data ready interrupt */
#define SENSOR_NOTIFY_BARO_TIMER    (1 << 2)    /**< Barometer timer expired */
#define SENSOR_NOTIFY_BNO_TIMER     (1 << 3)    /**< BNO055 timer expired */
#define SENSOR_NOTIFY_DMA_COMPLETE  (1 << 4)    /**< DMA transfer complete */
#define SENSOR_NOTIFY_DMA_ERROR     (1 << 5)    /**< DMA transfer error */
#define SENSOR_NOTIFY_GPS_DATA      (1 << 6)    /**< GPS data available */
#define SENSOR_NOTIFY_GPS_DR        (1 << 7)    /**< GPS data ready flag */
/** @} */ /* End of SensorNotifications group */

/**
 * @defgroup SensorTimingConfig Sensor Update Rates
 * @brief Timer periods for polled sensors
 * @{
 */
#define BARO_UPDATE_RATE_MS   100   /**< Barometer polling period (milliseconds) */
#define BNO_UPDATE_RATE_MS    100   /**< BNO055 polling period (milliseconds) */
/** @} */ /* End of SensorTimingConfig group */

/**
 * @defgroup SensorThreadAPI Sensor Thread API
 * @brief Public functions for sensor thread
 * @{
 */

/**
 * @brief Initialize all sensor hardware
 *
 * Initializes all sensor devices and configures their operating modes.
 * Must be called before starting the sensor thread.
 *
 * Initialization sequence:
 * 1. ASM330LHHX (IMU) - SPI_IMU_BARO
 * 2. MMC5983MA (Magnetometer) - SPI_MAG
 * 3. MS5607 (Barometer) - SPI_IMU_BARO
 * 4. BNO055 (9-DOF) - I2C_BNO
 * 5. UBLOX GPS - UART_UBLOX
 *
 * @note Assumes HAL peripheral handles are already initialized
 */
void sensors_thread_init(void);

/**
 * @brief Sensor thread main function
 *
 * FreeRTOS task entry point for the sensor management thread.
 * Handles sensor data acquisition using a combination of:
 * - Interrupt-driven reads (IMU, magnetometer)
 * - Timer-driven polling (barometer, BNO055)
 * - Continuous DMA (GPS)
 *
 * @param[in] argument FreeRTOS task argument (unused)
 *
 * @note This function runs as a FreeRTOS task and never returns
 */
void sensors_thread_function(void *argument);

/**
 * @brief Barometer timer callback
 *
 * Called by FreeRTOS timer to trigger barometer readings.
 * Sends SENSOR_NOTIFY_BARO_TIMER notification to sensor thread.
 *
 * @param[in] xTimer Timer handle (unused)
 */
void vBaroTimerCallback(TimerHandle_t xTimer);

/**
 * @brief BNO055 timer callback
 *
 * Called by FreeRTOS timer to trigger BNO055 readings.
 * Sends SENSOR_NOTIFY_BNO_TIMER notification to sensor thread.
 *
 * @param[in] xTimer Timer handle (unused)
 */
void vBnoTimerCallback(TimerHandle_t xTimer);

/** @} */ /* End of SensorThreadAPI group */

#endif /* SENSORS_THREAD_H */
