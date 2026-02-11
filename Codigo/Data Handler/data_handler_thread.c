/**
 * @file data_handler_thread.c
 * @brief Data Handler Thread Implementation
 * @author Tomás Teixeira
 * @date October 10, 2025
 * @version 2.0
 *
 * @details
 * Implements the data routing logic for the Magalhães Flight Computer.
 * This thread manages the flow of sensor data from circular buffers to
 * processing queues, preventing data loss while maintaining system responsiveness.
 *
 * ## Algorithm
 * The thread uses a multi-trigger flush strategy:
 * 1. **Threshold Flush**: When notified that a buffer is >50% full
 * 2. **Safety Flush**: Every 1 second regardless of fill level
 * 3. **Opportunistic Flush**: When SD queue has space available
 *
 * ## Data Packet Structure
 * Each packet includes a pointer to data in the circular buffer along with
 * a mutex for thread-safe access. The destination thread must:
 * 1. Call data_packet_lock() before reading
 * 2. Copy data to local storage
 * 3. Call data_packet_unlock() to release
 *
 * ## Performance Considerations
 * - SD queue has priority (5ms blocking timeout)
 * - Telemetry queue is non-blocking (data can be dropped)
 * - Buffer overflow tracking helps identify throughput issues
 *
 * @see data_handler_thread.h for interface documentation
 * @ingroup Data_Handler
 */

#include "data_handler_thread.h"

void data_handler_notify_threshold(void) {
    if (data_handler_thread_id != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(data_handler_thread_id, DATA_HANDLER_NOTIFY_THRESHOLD, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void flush_buffer_to_queues(ram_circular_buffer_t *cb, data_type_t type, uint32_t *seq) {
    if (ram_circular_buffer_is_empty(cb)) {return;}

    while (!ram_circular_buffer_is_empty(cb)) {
        if (xSemaphoreTake(cb->mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            break;
        }

        if (ram_circular_buffer_is_empty(cb)) {
            xSemaphoreGive(cb->mutex);
            break;
        }

        uint8_t *buffer = (uint8_t*)cb->buffer;
        uint8_t *read_addr = buffer + (cb->tail * cb->sample_size);

        data_packet_t packet = {
            .type = type,
            .data_ptr = read_addr,
            .timestamp_ms = 0,
            .sequence = (*seq)++,
            .source_cb = cb
        };

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

        // Prioritize SD with blocking send (short timeout)
        xQueueSend(queue_to_sd, &packet, pdMS_TO_TICKS(5));

        // Telemetry is lower priority
        xQueueSend(queue_to_telemetry, &packet, 0);

        cb->tail = (cb->tail + 1) % cb->capacity;
        xSemaphoreGive(cb->mutex);
    }
}

static bool is_buffer_above_threshold(ram_circular_buffer_t *cb) {
    uint32_t available = ram_circular_buffer_available(cb);
    uint32_t threshold = (cb->capacity * BUFFER_FULL_THRESHOLD_PCT) / 100;
    return available > threshold;
}

static bool any_buffer_above_threshold(void) {
    return is_buffer_above_threshold(&cb_imu) ||
           is_buffer_above_threshold(&cb_baro) ||
           is_buffer_above_threshold(&cb_mag) ||
           is_buffer_above_threshold(&cb_bno) ||
           is_buffer_above_threshold(&cb_gps);
}

static bool has_queue_space(void) {
    UBaseType_t sd_space = uxQueueSpacesAvailable(queue_to_sd);
    return sd_space > MIN_QUEUE_SPACE_FOR_FLUSH;
}

static bool any_buffer_has_data(void) {
    return !ram_circular_buffer_is_empty(&cb_imu) ||
           !ram_circular_buffer_is_empty(&cb_baro) ||
           !ram_circular_buffer_is_empty(&cb_mag) ||
           !ram_circular_buffer_is_empty(&cb_bno) ||
           !ram_circular_buffer_is_empty(&cb_gps);
}

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

void data_handler_thread_function() {
    printf("Data Handler thread started...\r\n");
    fsm_report_thread_started("DATA_HANDLER");
    fsm_report_init_status("DATA_HANDLER", true);

    TickType_t last_safety_flush = xTaskGetTickCount();
    const TickType_t safety_timeout = pdMS_TO_TICKS(SAFETY_FLUSH_TIMEOUT_MS);

    TickType_t last_monitor = xTaskGetTickCount();
    const TickType_t monitor_period = pdMS_TO_TICKS(10000);

    uint32_t ulNotificationValue;
    uint32_t flush_count = 0;
    uint32_t safety_flush_count = 0;
    uint32_t opportunistic_flush_count = 0;

    while(1) {
        TickType_t now = xTaskGetTickCount();

        // Wait for events with 200ms timeout for opportunistic flushing
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, pdMS_TO_TICKS(200));

        bool should_flush = false;
        const char *flush_reason = "";
        (void)flush_reason;

        // EVENT 1: Buffer threshold crossed (50% full)
        if (ulNotificationValue & DATA_HANDLER_NOTIFY_THRESHOLD) {
            if (any_buffer_above_threshold()) {
                should_flush = true;
                flush_reason = "THRESHOLD";
                last_safety_flush = now;
            }
        }

        // EVENT 2: Safety timeout (always flush if data exists)
        if (!should_flush && (now - last_safety_flush) >= safety_timeout) {
            if (any_buffer_has_data()) {
                should_flush = true;
                flush_reason = "SAFETY";
                safety_flush_count++;
                last_safety_flush = now;
            }
        }

        // EVENT 3: Opportunistic flush (SD queue has space and buffers not empty)
        if (!should_flush && has_queue_space() && any_buffer_has_data()) {
            should_flush = true;
            flush_reason = "OPPORTUNISTIC";
            opportunistic_flush_count++;
        }

        if (should_flush) {
            flush_all_buffers();
            flush_count++;
        }

        // MONITORING
        if ((now - last_monitor) >= monitor_period) {
            printf("Data Handler Status\r\n");

            // Overflows
            if (cb_imu.overflow_count > 0) printf("WARNING: IMU overflows: %lu\r\n", cb_imu.overflow_count);
            if (cb_baro.overflow_count > 0) printf("WARNING: BARO overflows: %lu\r\n", cb_baro.overflow_count);
            if (cb_mag.overflow_count > 0) printf("WARNING: MAG overflows: %lu\r\n", cb_mag.overflow_count);
            if (cb_bno.overflow_count > 0) printf("WARNING: BNO overflows: %lu\r\n", cb_bno.overflow_count);
            if (cb_gps.overflow_count > 0) printf("WARNING: GPS overflows: %lu\r\n", cb_gps.overflow_count);

            // Buffer percentages
            uint32_t imu_pct = (ram_circular_buffer_available(&cb_imu) * 100) / RAM_IMU_BUFFER_SIZE;
            uint32_t baro_pct = (ram_circular_buffer_available(&cb_baro) * 100) / RAM_BARO_BUFFER_SIZE;
            uint32_t mag_pct = (ram_circular_buffer_available(&cb_mag) * 100) / RAM_MAG_BUFFER_SIZE;
            uint32_t bno_pct = (ram_circular_buffer_available(&cb_bno) * 100) / RAM_BNO_BUFFER_SIZE;
            uint32_t gps_pct = (ram_circular_buffer_available(&cb_gps) * 100) / RAM_GPS_BUFFER_SIZE;

            printf("Buffers: IMU=%lu%%, BARO=%lu%%, MAG=%lu%%, BNO=%lu%%, GPS=%lu%%\r\n",
                   imu_pct, baro_pct, mag_pct, bno_pct, gps_pct);

            // Queues
            UBaseType_t est_msgs = uxQueueMessagesWaiting(queue_to_estimator);
            UBaseType_t tel_msgs = uxQueueMessagesWaiting(queue_to_telemetry);
            UBaseType_t sd_msgs = uxQueueMessagesWaiting(queue_to_sd);
            UBaseType_t sd_space = uxQueueSpacesAvailable(queue_to_sd);

            printf("Queues: EST=%lu/%u, TEL=%lu/%u, SD=%lu/%u (free:%lu)\r\n",
                   est_msgs, QUEUE_LENGTH_ESTIMATOR,
                   tel_msgs, QUEUE_LENGTH_TELEMETRY,
                   sd_msgs, QUEUE_LENGTH_SD, sd_space);

            printf("Flushes: total=%lu, threshold=%lu, safety=%lu, opportunistic=%lu\r\n",
                   flush_count, flush_count - safety_flush_count - opportunistic_flush_count,
                   safety_flush_count, opportunistic_flush_count);

            last_monitor = now;
        }
    }
}
