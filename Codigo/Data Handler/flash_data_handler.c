/*
 * flash_data_handler.c
 *
 * Implementation of flash-based circular buffer system
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */
// Includes
#include "flash_data_handler.h"
#include <string.h>
#include "print.h"

// Circular Buffers Instances
flash_circular_buffer_t cb_imu;
flash_circular_buffer_t cb_baro;
flash_circular_buffer_t cb_mag;
flash_circular_buffer_t cb_bno;
flash_circular_buffer_t cb_gps;
flash_circular_buffer_t cb_events;

// FreeRTOS Queues
QueueHandle_t queue_to_sd_card = NULL;
QueueHandle_t queue_to_estimator = NULL;
QueueHandle_t queue_to_telemetry = NULL;

// Sequence Counters
static uint32_t seq_imu = 0;
static uint32_t seq_baro = 0;
static uint32_t seq_gps = 0;

// Functions
// - Initialization Functions
void flash_storage_init(void) {

    printf("Initializing Flash Storage System...\n");
    printf("Flash data region: 0x%08lX - 0x%08lX (%lu KB)\n",
           FLASH_DATA_START_ADDR,
           FLASH_DATA_START_ADDR + FLASH_DATA_SIZE,
           FLASH_DATA_SIZE / 1024);

    // Erase flash sector(s) for circular buffers
    // WARNING: This will erase all data in the sector!
    // Only do this once during initialization
    printf("Erasing flash sector for circular buffers...\n");
    if (!flash_erase_sector(FLASH_SECTOR_DATA)) {
        printf("ERROR: Flash erase failed!\n");
        return;
    }
    printf("Flash erased successfully\n");

    // Initialize circular buffers
    flash_circular_buffer_init(&cb_imu, FLASH_IMU_ADDR, FLASH_IMU_BUFFER_SIZE, sizeof(IMU_t));
    flash_circular_buffer_init(&cb_baro, FLASH_BARO_ADDR, FLASH_BARO_BUFFER_SIZE, sizeof(BARO_t));
    flash_circular_buffer_init(&cb_mag, FLASH_MAG_ADDR, FLASH_MAG_BUFFER_SIZE, sizeof(MAG_t));
    flash_circular_buffer_init(&cb_bno, FLASH_BNO_ADDR, FLASH_BNO_BUFFER_SIZE, sizeof(BNO_t));
    flash_circular_buffer_init(&cb_gps, FLASH_GPS_ADDR, FLASH_GPS_BUFFER_SIZE, sizeof(GPS_t));
    flash_circular_buffer_init(&cb_events, FLASH_EVENT_ADDR, FLASH_EVENT_BUFFER_SIZE, sizeof(telemetry_event_t));
}

bool flash_erase_sector(uint32_t sector) {

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = sector;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V - 3.6V

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK);
}

void flash_circular_buffer_init(flash_circular_buffer_t *cb, uint32_t flash_addr, uint32_t size, uint32_t sample_size) {

	cb->flash_addr = flash_addr;
    cb->size = size;
    cb->head = 0;
    cb->tail = 0;
    cb->sample_size = sample_size;
    cb->overflow_count = 0;
    cb->mutex = xSemaphoreCreateMutex();

    if (cb->mutex == NULL) {
        printf("ERROR: Failed to create flash circular buffer mutex\n");
    }
}

// CB Operations Functions
bool flash_circular_buffer_write(flash_circular_buffer_t *cb,
                                 const void *data,
                                 uint32_t size) {

    if (size > cb->size) {
        return false;
    }

    xSemaphoreTake(cb->mutex, portMAX_DELAY);

    uint32_t free_space = flash_circular_buffer_free_space(cb);

    if (size > free_space) {
        // Buffer overflow - advance tail to make room
        cb->overflow_count++;
        cb->tail = (cb->tail + size) & (cb->size - 1);
    }

    // Write to flash
    uint32_t head = cb->head;
    uint32_t write_addr = cb->flash_addr + head;

    HAL_FLASH_Unlock();

    // Flash writes must be word-aligned (4 bytes)
    uint32_t *data_ptr = (uint32_t*)data;
    uint32_t words_to_write = (size + 3) / 4;  // Round up

    for (uint32_t i = 0; i < words_to_write; i++) {
        uint32_t addr = write_addr + (i * 4);

        // Handle wrap-around
        if (addr >= cb->flash_addr + cb->size) {
            addr = cb->flash_addr + (addr - (cb->flash_addr + cb->size));
        }

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data_ptr[i]);
    }

    HAL_FLASH_Lock();

    // Update head (power of 2 modulo)
    cb->head = (head + size) & (cb->size - 1);

    xSemaphoreGive(cb->mutex);

    return true;
}

bool flash_circular_buffer_read(flash_circular_buffer_t *cb,
                                void *data,
                                uint32_t size) {

    xSemaphoreTake(cb->mutex, portMAX_DELAY);

    uint32_t available = flash_circular_buffer_available(cb);

    if (size > available) {
        xSemaphoreGive(cb->mutex);
        return false;
    }

    // Read from flash
    uint32_t tail = cb->tail;
    uint32_t read_addr = cb->flash_addr + tail;

    // Handle wrap-around
    uint32_t first_part = cb->size - tail;

    if (size <= first_part) {
        // No wrap-around
        memcpy(data, (void*)read_addr, size);
    } else {
        // Wrap-around
        memcpy(data, (void*)read_addr, first_part);
        memcpy((uint8_t*)data + first_part, (void*)cb->flash_addr, size - first_part);
    }

    // Update tail
    cb->tail = (tail + size) & (cb->size - 1);

    xSemaphoreGive(cb->mutex);

    return true;
}

uint32_t flash_circular_buffer_available(flash_circular_buffer_t *cb) {
    uint32_t head = cb->head;
    uint32_t tail = cb->tail;
    return (head - tail) & (cb->size - 1);
}

uint32_t flash_circular_buffer_free_space(flash_circular_buffer_t *cb) {
    return cb->size - flash_circular_buffer_available(cb) - 1;
}

bool flash_circular_buffer_is_empty(flash_circular_buffer_t *cb) {
    return cb->head == cb->tail;
}

bool flash_circular_buffer_is_full(flash_circular_buffer_t *cb) {
    return flash_circular_buffer_free_space(cb) == 0;
}

void flash_circular_buffer_clear(flash_circular_buffer_t *cb) {
    xSemaphoreTake(cb->mutex, portMAX_DELAY);
    cb->head = 0;
    cb->tail = 0;
    cb->overflow_count = 0;
    xSemaphoreGive(cb->mutex);
}

// Helper Functions for Sensors
void data_handler_store_imu(IMU_t *imu_data) {

    // Write to circular buffer
    if (flash_circular_buffer_write(&cb_imu, imu_data, sizeof(IMU_t))) {

        // Create data packet with pointer
        data_packet_t packet = {
            .type = DATA_TYPE_IMU,
            .data_ptr = imu_data,  // Pointer to data in sensor thread's memory
            .size = sizeof(IMU_t),
            .timestamp_ms = imu_data->timestamp_ms,
            .sequence = seq_imu++
        };

        // Distribute
        distribute_data_packet(&packet);
    }
}

void data_handler_store_baro(BARO_t *baro_data) {

    if (flash_circular_buffer_write(&cb_baro, baro_data, sizeof(BARO_t))) {

        data_packet_t packet = {
            .type = DATA_TYPE_BARO,
            .data_ptr = baro_data,
            .size = sizeof(BARO_t),
            .timestamp_ms = baro_data->timestamp_ms,
            .sequence = seq_baro++
        };

        distribute_data_packet(&packet);
    }
}

void data_handler_store_gps(GPS_t *gps_data) {

    if (flash_circular_buffer_write(&cb_gps, gps_data, sizeof(GPS_t))) {

        data_packet_t packet = {
            .type = DATA_TYPE_GPS,
            .data_ptr = gps_data,
            .size = sizeof(GPS_t),
            .timestamp_ms = gps_data->timestamp_ms,
            .sequence = seq_gps++
        };

        distribute_data_packet(&packet);
    }
}

void data_handler_store_mag(MAG_t *mag_data) {
    if (flash_circular_buffer_write(&cb_mag, mag_data, sizeof(MAG_t))) {
        data_packet_t packet = {
            .type = DATA_TYPE_MAG,
            .data_ptr = mag_data,
            .size = sizeof(MAG_t),
            .timestamp_ms = mag_data->timestamp_ms,
            .sequence = 0
        };
        distribute_data_packet(&packet);
    }
}

void data_handler_store_bno(BNO_t *bno_data) {
    if (flash_circular_buffer_write(&cb_bno, bno_data, sizeof(BNO_t))) {
        data_packet_t packet = {
            .type = DATA_TYPE_BNO,
            .data_ptr = bno_data,
            .size = sizeof(BNO_t),
            .timestamp_ms = bno_data->timestamp_ms,
            .sequence = 0
        };
        distribute_data_packet(&packet);
    }
}

void data_handler_store_event(telemetry_event_t *event_data) {
    if (flash_circular_buffer_write(&cb_events, event_data, sizeof(telemetry_event_t))) {
        data_packet_t packet = {
            .type = DATA_TYPE_EVENT,
            .data_ptr = event_data,
            .size = sizeof(telemetry_event_t),
            .timestamp_ms = event_data->time,
            .sequence = 0
        };
        distribute_data_packet(&packet);
    }
}

