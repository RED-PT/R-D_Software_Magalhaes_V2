/*
 * sensors_thread.h
 *
 *	Sensors Thread: Reads sensors via DMA from multiple SPI buses
 *	SPI_X: Altimeter (MS5607) + IMU (ASM330LHHX)
 *	SPI_XX: Magnetometer (MMC5983MA)
 *	SPI_XXX: BNO055 (absolute orientation sensor)
 *
 *	Fuses data and sends to the Data Handler thread
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_SENSORS_THREAD_H_
#define SENSORS_SENSORS_THREAD_H_

// Include Definitions, Data Handler, Etc
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "Threads/create_threads.h"

// Structures
// Sensor state tracking
typedef enum {
    SENSOR_STATE_IDLE,
    SENSOR_STATE_READING,
    SENSOR_STATE_PROCESSING
} sensor_state_t;

typedef enum {
    ACTIVE_SENSOR_NONE = 0,
    ACTIVE_SENSOR_IMU,
    ACTIVE_SENSOR_BARO,
    ACTIVE_SENSOR_MAG,
    ACTIVE_SENSOR_BNO
} active_sensor_t;



// Thread notification bits
#define SENSOR_NOTIFY_IMU_DRDY      (1 << 0)   // IMU data ready interrupt
#define SENSOR_NOTIFY_MAG_DRDY      (1 << 1)   // MAG data ready interrupt
#define SENSOR_NOTIFY_BARO_TIMER    (1 << 2)   // BARO timer expired
#define SENSOR_NOTIFY_BNO_TIMER     (1 << 3)   // BNO timer expired
#define SENSOR_NOTIFY_DMA_COMPLETE  (1 << 4)   // DMA transfer complete
#define SENSOR_NOTIFY_DMA_ERROR     (1 << 5)   // DMA transfer error

// Sensor update rates
#define BARO_UPDATE_RATE_MS   20    // 50 Hz
#define BNO_UPDATE_RATE_MS    10    // 100 Hz

// DMA buffer sizes
#define IMU_DMA_BUFFER_SIZE   15    // 1 byte cmd + 14 bytes data (gyro + accel + temp)
#define MAG_DMA_BUFFER_SIZE   10    // 1 byte cmd + 9 bytes data
#define BARO_DMA_BUFFER_SIZE  4     // 1 byte cmd + 3 bytes ADC result
#define BNO_DMA_BUFFER_SIZE   20    // Buffer for BNO055 data

// Function prototypes
void sensors_thread_init(void);
void sensors_thread_function();

// Sensor driver functions (to be implemented in separate driver files)
// Substituir por drivers reais (basta dar include nos drives no sensors_thread.c)
bool imu_init(void);
bool imu_configure(void);
bool sensor_imu_read(void);
void sensor_imu_process(void);
bool imu_start_read_dma(void);
bool imu_process_data(const uint8_t *buffer, IMU_t *imu_data);

bool baro_init(void);
bool baro_configure(void);
bool baro_start_conversion(uint8_t conversion_type);
bool baro_read_adc_dma(void);
bool baro_process_data(const uint8_t *buffer, BARO_t *baro_data);

bool mag_init(void);
bool mag_configure(void);
bool sensor_mag_read(void);
void sensor_mag_process(void);
bool mag_start_read_dma(void);
bool mag_process_data(const uint8_t *buffer, MAG_t *mag_data);

bool bno_init(void);
bool bno_configure(void);
bool bno_start_read_dma(void);
bool bno_process_data(const uint8_t *buffer, BNO_t *bno_data);

// Timers Callbakcs
void vBaroTimerCallback(TimerHandle_t xTimer);
void vBnoTimerCallback(TimerHandle_t xTimer);

#endif /* SENSORS_SENSORS_THREAD_H_ */
