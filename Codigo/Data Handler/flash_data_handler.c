/*
 * flash_data_handler.c
 *
 * Implementation of flash-based circular buffer system with threshold notifications
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */
// Includes
#include "flash_data_handler.h"
#include <string.h>
#include "print.h"
#include "data_handler_thread.h"

// Circular Buffers Instances
static IMU_t imu_buffer[RAM_IMU_BUFFER_SIZE];
static BARO_t baro_buffer[RAM_BARO_BUFFER_SIZE];
static MAG_t mag_buffer[RAM_MAG_BUFFER_SIZE];
static BNO_t bno_buffer[RAM_BNO_BUFFER_SIZE];
static GPS_t gps_buffer[RAM_GPS_BUFFER_SIZE];

// RAM Circular Buffer Instances
ram_circular_buffer_t cb_imu;
ram_circular_buffer_t cb_baro;
ram_circular_buffer_t cb_mag;
ram_circular_buffer_t cb_bno;
ram_circular_buffer_t cb_gps;

// FreeRTOS Queues
QueueHandle_t queue_to_estimator = NULL;
QueueHandle_t queue_to_telemetry = NULL;
QueueHandle_t queue_to_sd = NULL;

// Sequence Counters
static uint32_t seq_imu = 0;
static uint32_t seq_baro = 0;
static uint32_t seq_mag = 0;
static uint32_t seq_bno = 0;
static uint32_t seq_gps = 0;

// Helper: Check if buffer crossed threshold and notify data handler
static void check_and_notify_threshold(void) {
    // Only notify if data_handler_thread_id is set (it's running)
    uint32_t available_imu = ram_circular_buffer_available(&cb_imu);
    uint32_t available_baro = ram_circular_buffer_available(&cb_baro);
    uint32_t available_mag = ram_circular_buffer_available(&cb_mag);
    uint32_t available_bno = ram_circular_buffer_available(&cb_bno);
    uint32_t available_gps = ram_circular_buffer_available(&cb_gps);

    uint32_t threshold_imu = (cb_imu.capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;
    uint32_t threshold_baro = (cb_baro.capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;
    uint32_t threshold_mag = (cb_mag.capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;
    uint32_t threshold_bno = (cb_bno.capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;
    uint32_t threshold_gps = (cb_gps.capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;

    if ((available_imu > threshold_imu) ||
        (available_baro > threshold_baro) ||
        (available_mag > threshold_mag) ||
        (available_bno > threshold_bno) ||
        (available_gps > threshold_gps)) {

        // Notify data handler that a buffer crossed threshold
        data_handler_notify_threshold();
    }
}

// Functions
// - Initialization Functions
void data_handler_init(void) {
    printf("Initializing Data Handler System...\r\n");

    // Initialize RAM circular buffers
    ram_circular_buffer_init(&cb_imu, imu_buffer, RAM_IMU_BUFFER_SIZE, sizeof(IMU_t));
    ram_circular_buffer_init(&cb_baro, baro_buffer, RAM_BARO_BUFFER_SIZE, sizeof(BARO_t));
    ram_circular_buffer_init(&cb_mag, mag_buffer, RAM_MAG_BUFFER_SIZE, sizeof(MAG_t));
    ram_circular_buffer_init(&cb_bno, bno_buffer, RAM_BNO_BUFFER_SIZE, sizeof(BNO_t));
    ram_circular_buffer_init(&cb_gps, gps_buffer, RAM_GPS_BUFFER_SIZE, sizeof(GPS_t));

    // Create queues (only passing pointers, so small size)
    queue_to_estimator = xQueueCreate(QUEUE_LENGTH_ESTIMATOR, sizeof(data_packet_t));
    queue_to_telemetry = xQueueCreate(QUEUE_LENGTH_TELEMETRY, sizeof(data_packet_t));
    queue_to_sd = xQueueCreate(QUEUE_LENGTH_SD, sizeof(data_packet_t));

    if (!queue_to_estimator || !queue_to_telemetry || !queue_to_sd) {
        printf("ERROR: Failed to create queues!\r\n");
    }

    printf("Data Handler initialized successfully\r\n");
    printf("Memory usage:\r\n");
    printf("IMU buffer:  %u bytes\r\n", RAM_IMU_BUFFER_SIZE * sizeof(IMU_t));
    printf("BARO buffer: %u bytes\r\n", RAM_BARO_BUFFER_SIZE * sizeof(BARO_t));
    printf("MAG buffer:  %u bytes\r\n", RAM_MAG_BUFFER_SIZE * sizeof(MAG_t));
    printf("BNO buffer:  %u bytes\r\n", RAM_BNO_BUFFER_SIZE * sizeof(BNO_t));
    printf("GPS buffer:  %u bytes\r\n", RAM_GPS_BUFFER_SIZE * sizeof(GPS_t));
}

void ram_circular_buffer_init(ram_circular_buffer_t *cb, void *buffer, uint32_t capacity, uint32_t sample_size) {
    cb->buffer = buffer;
    cb->capacity = capacity;
    cb->head = 0;
    cb->tail = 0;
    cb->sample_size = sample_size;
    cb->overflow_count = 0;
    cb->mutex = xSemaphoreCreateMutex();

    if (cb->mutex == NULL) {
        printf("ERROR: Failed to create mutex\n");
    }
}

bool ram_circular_buffer_write(ram_circular_buffer_t *cb, const void *data, const void **written_ptr) {
    // Take mutex with timeout to prevent deadlock
    if (xSemaphoreTake(cb->mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return false;
    }

    // Calculate next head position
    uint32_t next_head = (cb->head + 1) % cb->capacity;

    // Check for overflow
    if (next_head == cb->tail) {
        cb->overflow_count++;
        // Advance tail (overwrite oldest data)
        cb->tail = (cb->tail + 1) % cb->capacity;
    }

    // Calculate address in buffer
    uint8_t *buffer = (uint8_t*)cb->buffer;
    uint8_t *write_addr = buffer + (cb->head * cb->sample_size);

    // Copy data to buffer
    memcpy(write_addr, data, cb->sample_size);

    // Return pointer to written data
    if (written_ptr) {
        *written_ptr = write_addr;
    }

    // Update head
    cb->head = next_head;

    xSemaphoreGive(cb->mutex);
    return true;
}

uint32_t ram_circular_buffer_available(ram_circular_buffer_t *cb) {
    uint32_t head = cb->head;
    uint32_t tail = cb->tail;

    if (head >= tail) {
        return head - tail;
    } else {
        return cb->capacity - tail + head;
    }
}

bool ram_circular_buffer_is_empty(ram_circular_buffer_t *cb) {
    return cb->head == cb->tail;
}

static void distribute_data_packet(const data_packet_t *packet) {
    // Check FSM state for conditional routing
    // NOTE: Use different names to avoid shadowing the global queue handles!
    bool should_queue_to_est = fsm_should_queue_to_estimator();
    bool should_queue_to_sd = fsm_should_queue_to_sd();

    // Send to estimator only if in appropriate state
    if (should_queue_to_est) {
        if (packet->type == DATA_TYPE_IMU ||
            packet->type == DATA_TYPE_BARO ||
            packet->type == DATA_TYPE_MAG) {
            xQueueSend(queue_to_estimator, packet, 0);
        }
    }

    // Always send to telemetry (for GS monitoring)
    xQueueSend(queue_to_telemetry, packet, 0);

    // Send to SD only if logging enabled
    if (should_queue_to_sd) {
        xQueueSend(queue_to_sd, packet, 0);
    }
}

// Helper Functions for Sensors
void data_handler_store_imu(const IMU_t *imu_data) {
    const void *data_ptr = NULL;

    // Store in RAM circular buffer and get pointer to stored data
    if (ram_circular_buffer_write(&cb_imu, imu_data, &data_ptr)) {
        // Create data packet with pointer to data in buffer
        data_packet_t packet = {
            .type = DATA_TYPE_IMU,
            .data_ptr = data_ptr,
            .timestamp_ms = imu_data->timestamp_ms,
            .sequence = seq_imu++,
            .source_cb = &cb_imu
        };

        // Distribute to consumers
        distribute_data_packet(&packet);

        // Check if buffer crossed threshold
        check_and_notify_threshold();
    }
}

void data_handler_store_baro(const BARO_t *baro_data) {
    const void *data_ptr = NULL;

    if (ram_circular_buffer_write(&cb_baro, baro_data, &data_ptr)) {
        data_packet_t packet = {
            .type = DATA_TYPE_BARO,
            .data_ptr = data_ptr,
            .timestamp_ms = baro_data->timestamp_ms,
            .sequence = seq_baro++,
            .source_cb = &cb_baro
        };
        // Send to estimator immediately (high priority)
        xQueueSend(queue_to_estimator, &packet, 0);

        // Check if buffer crossed threshold (data_handler will flush to other queues)
        check_and_notify_threshold();
    }
}

void data_handler_store_mag(const MAG_t *mag_data) {
    const void *data_ptr = NULL;

    if (ram_circular_buffer_write(&cb_mag, mag_data, &data_ptr)) {
        data_packet_t packet = {
            .type = DATA_TYPE_MAG,
            .data_ptr = data_ptr,
            .timestamp_ms = mag_data->timestamp_ms,
            .sequence = seq_mag++,
            .source_cb = &cb_mag
        };
        // Send to estimator immediately (high priority)
        xQueueSend(queue_to_estimator, &packet, 0);

        // Check if buffer crossed threshold
        check_and_notify_threshold();
    }
}

void data_handler_store_bno(const BNO_t *bno_data) {
    const void *data_ptr = NULL;

    if (ram_circular_buffer_write(&cb_bno, bno_data, &data_ptr)) {
        data_packet_t packet = {
            .type = DATA_TYPE_BNO,
            .data_ptr = data_ptr,
            .timestamp_ms = bno_data->timestamp_ms,
            .sequence = seq_bno++,
            .source_cb = &cb_bno
        };
        // Send to estimator immediately
        xQueueSend(queue_to_estimator, &packet, 0);

        // Check if buffer crossed threshold
        check_and_notify_threshold();
    }
}

void data_handler_store_gps(const GPS_t *gps_data) {
    const void *data_ptr = NULL;

    if (ram_circular_buffer_write(&cb_gps, gps_data, &data_ptr)) {
        data_packet_t packet = {
            .type = DATA_TYPE_GPS,
            .data_ptr = data_ptr,
            .timestamp_ms = gps_data->timestamp_ms,
            .sequence = seq_gps++,
            .source_cb = &cb_gps
        };
        // Send to estimator immediately
        xQueueSend(queue_to_estimator, &packet, 0);

        // Check if buffer crossed threshold
        check_and_notify_threshold();
    }
}

void data_handler_store_event(const telemetry_event_t *event_data) {
    // Events don't go in circular buffer, send directly
    data_packet_t packet = {
        .type = DATA_TYPE_EVENT,
        .data_ptr = event_data,
        .timestamp_ms = event_data->time,
        .sequence = 0,
        .source_cb = NULL  // No circular buffer for events
    };
    distribute_data_packet(&packet);
}
