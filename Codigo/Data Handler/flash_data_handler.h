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
// REDUCED to avoid heap exhaustion during boot
#define RAM_IMU_BUFFER_SIZE     256   // ~38ms @ 6667Hz (saves 8KB)
#define RAM_BARO_BUFFER_SIZE    64    // saves 1KB
#define RAM_MAG_BUFFER_SIZE     64    // saves 1.3KB
#define RAM_BNO_BUFFER_SIZE     64    // saves 4KB
#define RAM_GPS_BUFFER_SIZE     16    // saves 0.7KB

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

/**
 * Queue item carrying a full COPY of the sample.
 *
 * The previous design queued a pointer into the circular buffer plus the
 * buffer's mutex. That was unsound: the writer overwrites the oldest slot
 * when the buffer is full, and the flush path frees slots (advances tail)
 * immediately after queueing — so consumers could dereference a slot that
 * had already been recycled. The mutex only guarded *concurrent* access,
 * not *stale* pointers. Copying ~80 bytes per item (≈4 KB extra across all
 * queues) removes that whole class of bugs and the consumer-side locking.
 */
typedef struct {
    data_type_t type;
    uint32_t timestamp_ms;
    uint32_t sequence;
    union {
        IMU_t  imu;
        BARO_t baro;
        MAG_t  mag;
        BNO_t  bno;
        GPS_t  gps;
    } payload;
} data_packet_t;

extern ram_circular_buffer_t cb_imu;
extern ram_circular_buffer_t cb_baro;
extern ram_circular_buffer_t cb_mag;
extern ram_circular_buffer_t cb_bno;
extern ram_circular_buffer_t cb_gps;

// Queue sizes reduced to save heap memory
#define QUEUE_LENGTH_ESTIMATOR  4
#define QUEUE_LENGTH_TELEMETRY  16
#define QUEUE_LENGTH_SD         40  // Reduced to save ~1KB heap

extern QueueHandle_t queue_to_estimator;
extern QueueHandle_t queue_to_telemetry;
extern QueueHandle_t queue_to_sd;

void data_handler_init(void);

void ram_circular_buffer_init(ram_circular_buffer_t *cb, void *buffer, uint32_t capacity, uint32_t sample_size);
bool ram_circular_buffer_write(ram_circular_buffer_t *cb, const void *data, const void **written_ptr);
uint32_t ram_circular_buffer_available(ram_circular_buffer_t *cb);
bool ram_circular_buffer_is_empty(ram_circular_buffer_t *cb);

/* Packets now carry copies — locking is unnecessary. Kept as no-ops so any
 * straggling caller still compiles. */
static inline void data_packet_lock(const data_packet_t *packet)   { (void)packet; }
static inline void data_packet_unlock(const data_packet_t *packet) { (void)packet; }

static inline void data_packet_copy_imu(const data_packet_t *packet, IMU_t *dest) {
    memcpy(dest, &packet->payload.imu, sizeof(IMU_t));
}

static inline void data_packet_copy_baro(const data_packet_t *packet, BARO_t *dest) {
    memcpy(dest, &packet->payload.baro, sizeof(BARO_t));
}

static inline void data_packet_copy_mag(const data_packet_t *packet, MAG_t *dest) {
    memcpy(dest, &packet->payload.mag, sizeof(MAG_t));
}

static inline void data_packet_copy_bno(const data_packet_t *packet, BNO_t *dest) {
    memcpy(dest, &packet->payload.bno, sizeof(BNO_t));
}

static inline void data_packet_copy_gps(const data_packet_t *packet, GPS_t *dest) {
    memcpy(dest, &packet->payload.gps, sizeof(GPS_t));
}

void data_handler_store_imu(const IMU_t *imu_data);
void data_handler_store_baro(const BARO_t *baro_data);
void data_handler_store_gps(const GPS_t *gps_data);
void data_handler_store_mag(const MAG_t *mag_data);
void data_handler_store_bno(const BNO_t *bno_data);
/* data_handler_store_event() removed: it queued a pointer to the caller's
 * stack variable (dangling by design) and had no callers. */

#endif /* DATA_HANDLER_FLASH_DATA_HANDLER_H_ */
