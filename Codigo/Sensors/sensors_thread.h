/*
 * sensors_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: texman
 */

#ifndef SENSORS_THREAD_H
#define SENSORS_THREAD_H

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
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

// Sensor instances
extern ASM330LHHX_t imu_device;
extern MMC5983MA_t mag_device;
extern MS5607_t baro_device;
extern BNO055_t bno_device;
extern UBLOX_GPS_t gps_device;

// Sensor state tracking
typedef enum {
    ACTIVE_SENSOR_NONE = 0,
    ACTIVE_SENSOR_IMU,
    ACTIVE_SENSOR_BARO,
    ACTIVE_SENSOR_MAG,
    ACTIVE_SENSOR_BNO
} active_sensor_t;

// Export bus tracking for hal_callbacks.c
extern volatile active_sensor_t spi1_active_sensor;
extern volatile active_sensor_t spi3_active_sensor;
extern volatile active_sensor_t i2c1_active_sensor;

// Thread notification bits
#define SENSOR_NOTIFY_IMU_DRDY      (1 << 0)
#define SENSOR_NOTIFY_MAG_DRDY      (1 << 1)
#define SENSOR_NOTIFY_BARO_TIMER    (1 << 2)
#define SENSOR_NOTIFY_BNO_TIMER     (1 << 3)
#define SENSOR_NOTIFY_DMA_COMPLETE  (1 << 4)
#define SENSOR_NOTIFY_DMA_ERROR     (1 << 5)
#define SENSOR_NOTIFY_GPS_DATA      (1 << 6)
#define SENSOR_NOTIFY_GPS_DR        (1 << 7)

// Update rates
#define BARO_UPDATE_RATE_MS   100
#define BNO_UPDATE_RATE_MS    100

// Function prototypes
void sensors_thread_init(void);
void sensors_thread_function(void *argument);
void vBaroTimerCallback(TimerHandle_t xTimer);
void vBnoTimerCallback(TimerHandle_t xTimer);

#endif /* SENSORS_THREAD_H */
