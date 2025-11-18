/*
 * flash_data_handler.h
 *
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef DATA_HANDLER_FLASH_DATA_HANDLER_H_
#define DATA_HANDLER_FLASH_DATA_HANDLER_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "defs.h"
#include "Telemetry/telemetry.h"

// Optimized buffer sizes for STM32F446ZE (128KB RAM)
#define RAM_IMU_BUFFER_SIZE     512   // ~76ms @ 6667Hz
#define RAM_BARO_BUFFER_SIZE    128
#define RAM_MAG_BUFFER_SIZE     128
#define RAM_BNO_BUFFER_SIZE     128
#define RAM_GPS_BUFFER_SIZE     32

#define BUFFER_FULL_THRESHOLD_PCT   50  // Notify at 50% full

typedef struct {
    void *buffer;
    uint32_t capacity;
    volatile uint32_t head;
    volatile uint32_t tail;
    uint32_t sample_size;
    uint32_t overflow_count;
    SemaphoreHandle_t mutex;
} ram_circular_buffer_t;

typedef enum {
    DATA_TYPE_IMU = 0,
    DATA_TYPE_BARO,
    DATA_TYPE_MAG,
    DATA_TYPE_BNO,
    DATA_TYPE_GPS,
    DATA_TYPE_EVENT,
    DATA_TYPE_NAV_STATE
} data_type_t;

typedef struct {
    data_type_t type;
    const void *data_ptr;
    uint32_t timestamp_ms;
    uint32_t sequence;
    ram_circular_buffer_t *source_cb;
} data_packet_t;

extern ram_circular_buffer_t cb_imu;
extern ram_circular_buffer_t cb_baro;
extern ram_circular_buffer_t cb_mag;
extern ram_circular_buffer_t cb_bno;
extern ram_circular_buffer_t cb_gps;

// Queue sizes balanced for STM32F446ZE
#define QUEUE_LENGTH_ESTIMATOR  4
#define QUEUE_LENGTH_TELEMETRY  30
#define QUEUE_LENGTH_SD         80  // Increased but reasonable for F446ZE

extern QueueHandle_t queue_to_estimator;
extern QueueHandle_t queue_to_telemetry;
extern QueueHandle_t queue_to_sd;

void data_handler_init(void);

void ram_circular_buffer_init(ram_circular_buffer_t *cb, void *buffer, uint32_t capacity, uint32_t sample_size);
bool ram_circular_buffer_write(ram_circular_buffer_t *cb, const void *data, const void **written_ptr);
uint32_t ram_circular_buffer_available(ram_circular_buffer_t *cb);
bool ram_circular_buffer_is_empty(ram_circular_buffer_t *cb);

static inline void data_packet_lock(const data_packet_t *packet) {
    if (packet->source_cb && packet->source_cb->mutex) {
        xSemaphoreTake(packet->source_cb->mutex, portMAX_DELAY);
    }
}

static inline void data_packet_unlock(const data_packet_t *packet) {
    if (packet->source_cb && packet->source_cb->mutex) {
        xSemaphoreGive(packet->source_cb->mutex);
    }
}

static inline void data_packet_copy_imu(const data_packet_t *packet, IMU_t *dest) {
    memcpy(dest, packet->data_ptr, sizeof(IMU_t));
}

static inline void data_packet_copy_baro(const data_packet_t *packet, BARO_t *dest) {
    memcpy(dest, packet->data_ptr, sizeof(BARO_t));
}

static inline void data_packet_copy_mag(const data_packet_t *packet, MAG_t *dest) {
    memcpy(dest, packet->data_ptr, sizeof(MAG_t));
}

static inline void data_packet_copy_bno(const data_packet_t *packet, BNO_t *dest) {
    memcpy(dest, packet->data_ptr, sizeof(BNO_t));
}

static inline void data_packet_copy_gps(const data_packet_t *packet, GPS_t *dest) {
    memcpy(dest, packet->data_ptr, sizeof(GPS_t));
}

void data_handler_store_imu(const IMU_t *imu_data);
void data_handler_store_baro(const BARO_t *baro_data);
void data_handler_store_gps(const GPS_t *gps_data);
void data_handler_store_mag(const MAG_t *mag_data);
void data_handler_store_bno(const BNO_t *bno_data);
void data_handler_store_event(const telemetry_event_t *event_data);

#endif /* DATA_HANDLER_FLASH_DATA_HANDLER_H_ */
