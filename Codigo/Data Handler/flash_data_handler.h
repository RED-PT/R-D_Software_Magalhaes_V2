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
#include "config.h" // definition of the flash memory address

// Defines: Buffer sizes (Power of 2 como o Lucas disse)
#define FLASH_IMU_BUFFER_SIZE    8192   // 8KB (~400 samples)
#define FLASH_BARO_BUFFER_SIZE   4096   // 4KB (~200 samples)
#define FLASH_MAG_BUFFER_SIZE    2048   // 2KB (~100 samples)
#define FLASH_BNO_BUFFER_SIZE    4096   // 4KB (~66 samples)
#define FLASH_GPS_BUFFER_SIZE    2048   // 2KB (~40 samples)
#define FLASH_EVENT_BUFFER_SIZE  1024   // 1KB (~32 events)

// Flash addresses (consecutive allocation)
#define FLASH_IMU_ADDR    (FLASH_DATA_START_ADDR)
#define FLASH_BARO_ADDR   (FLASH_IMU_ADDR + FLASH_IMU_BUFFER_SIZE)
#define FLASH_MAG_ADDR    (FLASH_BARO_ADDR + FLASH_BARO_BUFFER_SIZE)
#define FLASH_BNO_ADDR    (FLASH_MAG_ADDR + FLASH_MAG_BUFFER_SIZE)
#define FLASH_GPS_ADDR    (FLASH_BNO_ADDR + FLASH_BNO_BUFFER_SIZE)
#define FLASH_EVENT_ADDR  (FLASH_GPS_ADDR + FLASH_GPS_BUFFER_SIZE)

// Total flash usage
#define FLASH_TOTAL_USAGE (FLASH_IMU_BUFFER_SIZE + FLASH_BARO_BUFFER_SIZE + \
                          FLASH_MAG_BUFFER_SIZE + FLASH_BNO_BUFFER_SIZE + \
                          FLASH_GPS_BUFFER_SIZE + FLASH_EVENT_BUFFER_SIZE)

// Verify we don't exceed available flash
_Static_assert(FLASH_TOTAL_USAGE <= FLASH_DATA_SIZE, "Flash buffers exceed available space");


// Structures
// - Circular Buffer Structure
typedef struct {
    uint32_t flash_addr;        // Flash memory address
    uint32_t size;              // Buffer size (must be power of 2)
    volatile uint32_t head;     // Write index (in RAM)
    volatile uint32_t tail;     // Read index (in RAM)
    uint32_t sample_size;       // Size of each sample
    uint32_t overflow_count;    // Count of buffer overflows
    SemaphoreHandle_t mutex;    // Thread safety
} flash_circular_buffer_t;

// - Data Type Structure Enum
typedef enum {
    DATA_TYPE_IMU = 0,
    DATA_TYPE_BARO,
    DATA_TYPE_MAG,
    DATA_TYPE_BNO,
    DATA_TYPE_GPS,
    DATA_TYPE_EVENT,
    DATA_TYPE_NAV_STATE //?
} data_type_t;

// - Data Structure
typedef struct {
    data_type_t type;
	void *data_ptr;			// Pointer to data in circular buffer
    uint32_t size;          // Size of data
    uint32_t timestamp_ms;  // Timestamp when data was captured
    uint32_t sequence;      // Sequence number for tracking
} data_packet_t;

// Global Circular Buffers
extern flash_circular_buffer_t cb_imu;
extern flash_circular_buffer_t cb_baro;
extern flash_circular_buffer_t cb_mag;
extern flash_circular_buffer_t cb_bno;
extern flash_circular_buffer_t cb_gps;
extern flash_circular_buffer_t cb_events;

// Queues
#define QUEUE_LENGTH 20

extern QueueHandle_t queue_to_sd_card;      // Data Handler → SD Card
extern QueueHandle_t queue_to_estimator;    // Data Handler → Estimator
extern QueueHandle_t queue_to_telemetry;    // Data Handler → Telemetry

// Function Prototypes
// Initialize flash storage system
void flash_storage_init(void);

// Erase flash sector for circular buffers
bool flash_erase_sector(uint32_t sector);

// Initialize a flash circular buffer
void flash_circular_buffer_init(flash_circular_buffer_t *cb, uint32_t flash_addr, uint32_t size, uint32_t sample_size);

// Write to flash circular buffer
bool flash_circular_buffer_write(flash_circular_buffer_t *cb, const void *data, uint32_t size);

// Read from flash circular buffer
bool flash_circular_buffer_read(flash_circular_buffer_t *cb, void *data, uint32_t size);

// Get available data size
uint32_t flash_circular_buffer_available(flash_circular_buffer_t *cb);

// Get free space
uint32_t flash_circular_buffer_free_space(flash_circular_buffer_t *cb);

// Check if empty
bool flash_circular_buffer_is_empty(flash_circular_buffer_t *cb);

// Check if full
bool flash_circular_buffer_is_full(flash_circular_buffer_t *cb);

// Clear buffer (reset indices only, no flash erase)
void flash_circular_buffer_clear(flash_circular_buffer_t *cb);

// - Helper Suggested By Claude_ Sends the data to all "consumers"
void distribute_data_packet(data_packet_t *packet);

// - Helpers for Sensors
void data_handler_store_imu(IMU_t *imu_data);
void data_handler_store_baro(BARO_t *baro_data);
void data_handler_store_gps(GPS_t *gps_data);
void data_handler_store_mag(MAG_t *mag_data);
void data_handler_store_bno(BNO_t *bno_data);
void data_handler_store_event(telemetry_event_t *event_data);


#endif /* DATA_HANDLER_FLASH_DATA_HANDLER_H_ */
