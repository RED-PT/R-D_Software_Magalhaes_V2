/*
 * flash_data_handler.h
 *
 * Flash-based circular buffer data handler for sensor data
 * Uses zero-copy design - passes pointers instead of data
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef DATA_HANDLER_FLASH_DATA_HANDLER_H_
#define DATA_HANDLER_FLASH_DATA_HANDLER_H_

// Includes FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

// Include Definitions
#include "defs.h"
#include "Telemetry/telemetry.h"

// Defines: Buffer sizes (Power of 2 como o Lucas disse)
#define RAM_IMU_BUFFER_SIZE     512   // Store last 512 IMU samples (~76ms @ 6667Hz)
#define RAM_BARO_BUFFER_SIZE    128   // Store last 128 BARO samples
#define RAM_MAG_BUFFER_SIZE     64    // Store last 64 MAG samples
#define RAM_BNO_BUFFER_SIZE     128   // Store last 128 BNO samples
#define RAM_GPS_BUFFER_SIZE     32    // Store last 32 GPS samples

// Structures
// - Circular Buffer Structure
typedef struct {
    void *buffer;               // RAM buffer pointer
    uint32_t capacity;          // Number of samples (not bytes!)
    volatile uint32_t head;     // Write index
    volatile uint32_t tail;     // Read index
    uint32_t sample_size;       // Size of each sample in bytes
    uint32_t overflow_count;    // Overflow counter
    SemaphoreHandle_t mutex;    // Protects buffer access
} ram_circular_buffer_t;

// - Data Type Structure Enum
typedef enum {
    DATA_TYPE_IMU = 0,
    DATA_TYPE_BARO,
    DATA_TYPE_MAG,
    DATA_TYPE_BNO,
    DATA_TYPE_GPS,
    DATA_TYPE_EVENT,
    DATA_TYPE_NAV_STATE
} data_type_t;

// - Data Structure
typedef struct {
    data_type_t type;
    const void *data_ptr;       // Pointer to data in circular buffer (READ-ONLY!)
    uint32_t timestamp_ms;
    uint32_t sequence;
    ram_circular_buffer_t *source_cb;  // Source buffer (for mutex access)
} data_packet_t;

// Global Circular Buffers
extern ram_circular_buffer_t cb_imu;
extern ram_circular_buffer_t cb_baro;
extern ram_circular_buffer_t cb_mag;
extern ram_circular_buffer_t cb_bno;
extern ram_circular_buffer_t cb_gps;

// Queues
#define QUEUE_LENGTH_ESTIMATOR  20   // High-rate sensors
#define QUEUE_LENGTH_TELEMETRY  10   // Lower rate
#define QUEUE_LENGTH_LOGGER     50   // Buffered logging

extern QueueHandle_t queue_to_estimator;    // High priority - IMU, MAG
extern QueueHandle_t queue_to_telemetry;    // Medium priority - all data
extern QueueHandle_t queue_to_logger;       // Low priority - SD card logging

// Function Prototypes
// Initialize flash storage system
void data_handler_init(void);

// RAM Circular Buffer Operations
void ram_circular_buffer_init(ram_circular_buffer_t *cb, void *buffer,uint32_t capacity, uint32_t sample_size);
bool ram_circular_buffer_write(ram_circular_buffer_t *cb, const void *data, const void **written_ptr);
uint32_t ram_circular_buffer_available(ram_circular_buffer_t *cb);
bool ram_circular_buffer_is_empty(ram_circular_buffer_t *cb);

// Helper: Lock/unlock buffer for safe reading
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

// Helper: Copy data from packet (must call between lock/unlock or immediately)
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


// - Helpers for Sensors
void data_handler_store_imu(const IMU_t *imu_data);
void data_handler_store_baro(const BARO_t *baro_data);
void data_handler_store_gps(const GPS_t *gps_data);
void data_handler_store_mag(const MAG_t *mag_data);
void data_handler_store_bno(const BNO_t *bno_data);
void data_handler_store_event(const telemetry_event_t *event_data);

#endif /* DATA_HANDLER_FLASH_DATA_HANDLER_H_ */
