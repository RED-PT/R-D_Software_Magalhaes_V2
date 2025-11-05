/*
 * data_handler_thread.c
 *
 * Redesigned to handle periodic flushing and buffer overflow management
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "flash_data_handler.h"
#include "print.h"
#include "cmsis_os.h"

// Flush thresholds (as percentage of buffer capacity)
#define BUFFER_FULL_THRESHOLD_PCT   80

// Flush timing
#define FAST_PACKET_PERIOD_MS       20   // 50Hz
#define SLOW_PACKET_PERIOD_MS       200  // 5Hz
#define MONITOR_PERIOD_MS           1000 // Status print every 1s

// Helper function to create packets from circular buffer and send to queues
// NOTE: Packets are sent with locked mutexes - consumers MUST call data_packet_unlock()
// after reading the data_ptr. Failure to unlock will cause deadlock!
static void flush_buffer_to_queues(ram_circular_buffer_t *cb, data_type_t type, uint32_t *seq) {
    if (ram_circular_buffer_is_empty(cb)) {return;}

    // Iterate through all available data in the buffer and send each entry
    while (!ram_circular_buffer_is_empty(cb)) {
        // Lock buffer for safe reading
        if (xSemaphoreTake(cb->mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            break; // Couldn't acquire lock, skip this flush
        }

        if (ram_circular_buffer_is_empty(cb)) {
            xSemaphoreGive(cb->mutex);
            break;
        }

        // Calculate current tail position
        uint8_t *buffer = (uint8_t*)cb->buffer;
        uint8_t *read_addr = buffer + (cb->tail * cb->sample_size);

        // Create packet pointing to data in buffer
        // IMPORTANT: Mutex is held - receiver MUST unlock via data_packet_unlock()
        data_packet_t packet = {
            .type = type,
            .data_ptr = read_addr,
            .timestamp_ms = 0, // Will be set by specific sensor type below
            .sequence = (*seq)++,
            .source_cb = cb  // Mutex stays locked - consumer must unlock!
        };

        // Extract timestamp based on data type
        switch (type) {
            case DATA_TYPE_IMU:
                packet.timestamp_ms = ((IMU_t*)read_addr)->timestamp_ms;
                break;
            case DATA_TYPE_BARO:
                packet.timestamp_ms = ((BARO_t*)read_addr)->timestamp_ms;
                break;
            case DATA_TYPE_MAG:
                packet.timestamp_ms = ((MAG_t*)read_addr)->timestamp_ms;
                break;
            case DATA_TYPE_BNO:
                packet.timestamp_ms = ((BNO_t*)read_addr)->timestamp_ms;
                break;
            case DATA_TYPE_GPS:
                packet.timestamp_ms = ((GPS_t*)read_addr)->timestamp_ms;
                break;
            default:
                break;
        }

        // Send to telemetry and sd queues (non-blocking, can drop if full)
        xQueueSend(queue_to_telemetry, &packet, 0);
        xQueueSend(queue_to_sd, &packet, 0);

        // Advance tail WHILE MUTEX IS HELD to prevent race conditions
        cb->tail = (cb->tail + 1) % cb->capacity;

        // Release mutex - consumers should have copied or locked the packet by now
        xSemaphoreGive(cb->mutex);
    }
}

// Helper function to check if buffer is above threshold
static bool is_buffer_above_threshold(ram_circular_buffer_t *cb) {
    uint32_t available = ram_circular_buffer_available(cb);
    uint32_t threshold = (cb->capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;
    return available > threshold;
}

// Helper function to check if any buffer is above threshold
static bool any_buffer_above_threshold(void) {
    return is_buffer_above_threshold(&cb_imu) ||
           is_buffer_above_threshold(&cb_baro) ||
           is_buffer_above_threshold(&cb_mag) ||
           is_buffer_above_threshold(&cb_bno) ||
           is_buffer_above_threshold(&cb_gps);
}

// Helper function to flush all buffers to telemetry and sd
static void flush_all_buffers(void) {
    static uint32_t seq_tel_imu = 0;
    static uint32_t seq_tel_baro = 0;
    static uint32_t seq_tel_mag = 0;
    static uint32_t seq_tel_bno = 0;
    static uint32_t seq_tel_gps = 0;

    flush_buffer_to_queues(&cb_imu, DATA_TYPE_IMU, &seq_tel_imu);
    flush_buffer_to_queues(&cb_baro, DATA_TYPE_BARO, &seq_tel_baro);
    flush_buffer_to_queues(&cb_mag, DATA_TYPE_MAG, &seq_tel_mag);
    flush_buffer_to_queues(&cb_bno, DATA_TYPE_BNO, &seq_tel_bno);
    flush_buffer_to_queues(&cb_gps, DATA_TYPE_GPS, &seq_tel_gps);
}

void data_handler_thread_function(void *argument) {
    printf("Data Handler monitoring thread started\r\n");
    osDelay(50);

    TickType_t last_fast_flush = xTaskGetTickCount();
    TickType_t last_slow_flush = xTaskGetTickCount();
    TickType_t last_monitor = xTaskGetTickCount();

    const TickType_t fast_flush_period = pdMS_TO_TICKS(FAST_PACKET_PERIOD_MS);   // 20ms (50Hz)
    const TickType_t slow_flush_period = pdMS_TO_TICKS(SLOW_PACKET_PERIOD_MS);   // 200ms (5Hz)
    const TickType_t monitor_period = pdMS_TO_TICKS(MONITOR_PERIOD_MS);          // 1s

    while(1) {
        TickType_t now = xTaskGetTickCount();

        // Fast flush for high-rate data (50Hz - every 20ms)
        if ((now - last_fast_flush) >= fast_flush_period) {
            // Check for buffer overflow or high occupancy
            if (any_buffer_above_threshold()) {
                printf("Buffer threshold exceeded - flushing to telemetry/sd\r\n");
                flush_all_buffers();
            }
            last_fast_flush = now;
        }

        // Slow flush for periodic data (5Hz - every 200ms)
        if ((now - last_slow_flush) >= slow_flush_period) {
            // Always flush, even if buffers aren't full - ensures telemetry gets regular updates
            flush_all_buffers();
            last_slow_flush = now;
        }

        // Monitor and print stats (every 1s)
        if ((now - last_monitor) >= monitor_period) {
            printf("Data Handler Status\r\n");

            // Check for overflows
            if (cb_imu.overflow_count > 0) {
                printf("IMU overflows: %lu\r\n", cb_imu.overflow_count);
            }
            if (cb_baro.overflow_count > 0) {
                printf("BARO overflows: %lu\r\n", cb_baro.overflow_count);
            }
            if (cb_mag.overflow_count > 0) {
                printf("MAG overflows: %lu\r\n", cb_mag.overflow_count);
            }
            if (cb_bno.overflow_count > 0) {
                printf("BNO overflows: %lu\r\n", cb_bno.overflow_count);
            }
            if (cb_gps.overflow_count > 0) {
                printf("GPS overflows: %lu\r\n", cb_gps.overflow_count);
            }

            // Check buffer occupancy
            uint32_t imu_avail = ram_circular_buffer_available(&cb_imu);
            uint32_t baro_avail = ram_circular_buffer_available(&cb_baro);
            uint32_t mag_avail = ram_circular_buffer_available(&cb_mag);
            uint32_t bno_avail = ram_circular_buffer_available(&cb_bno);
            uint32_t gps_avail = ram_circular_buffer_available(&cb_gps);

            printf("Buffer occupancy: IMU=%lu/%u, BARO=%lu/%u, MAG=%lu/%u, BNO=%lu/%u, GPS=%lu/%u\r\n",
                   imu_avail, RAM_IMU_BUFFER_SIZE,
                   baro_avail, RAM_BARO_BUFFER_SIZE,
                   mag_avail, RAM_MAG_BUFFER_SIZE,
                   bno_avail, RAM_BNO_BUFFER_SIZE,
                   gps_avail, RAM_GPS_BUFFER_SIZE);

            // Check queue levels
            UBaseType_t est_msgs = uxQueueMessagesWaiting(queue_to_estimator);
            UBaseType_t tel_msgs = uxQueueMessagesWaiting(queue_to_telemetry);
            UBaseType_t sd_msgs = uxQueueMessagesWaiting(queue_to_sd);

            printf("Queue levels: EST=%lu/%u, TEL=%lu/%u, SD=%lu/%u\r\n",
                   est_msgs, QUEUE_LENGTH_ESTIMATOR,
                   tel_msgs, QUEUE_LENGTH_TELEMETRY,
                   sd_msgs, QUEUE_LENGTH_SD);

            last_monitor = now;
        }

        // Small delay to prevent CPU spinning
        osDelay(5);
    }
}
