/*
 * data_handler_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

// Include Flash Data Handler Drivers
#include "flash_data_handler.h"

void data_handler_thread_function(void *argument) {

    printf("Data Handler thread started\n");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(20); // 50 Hz monitoring

    while(1) {
        // This thread is primarily event-driven via callbacks
        // But we can periodically check buffer health

        // Monitor buffer usage
    	/*
        uint32_t imu_usage = (flash_circular_buffer_available(&cb_imu) * 100) / FLASH_IMU_BUFFER_SIZE;
        uint32_t baro_usage = (flash_circular_buffer_available(&cb_baro) * 100) / FLASH_BARO_BUFFER_SIZE;
        uint32_t gps_usage = (flash_circular_buffer_available(&cb_gps) * 100) / FLASH_GPS_BUFFER_SIZE;
        uint32_t mag_usage = (flash_circular_buffer_available(&cb_mag) * 100) / FLASH_MAG_BUFFER_SIZE;
        uint32_t bno_usage = (flash_circular_buffer_available(&cb_bno) * 100) / FLASH_BNO_BUFFER_SIZE;
        uint32_t events_usage = (flash_circular_buffer_available(&cb_events) * 100) / FLASH_EVENT_BUFFER_SIZE;
		*/

        // Check for buffer overflows
        if (cb_imu.overflow_count > 0) {
            printf("WARNING: IMU buffer overflow count: %lu\n", cb_imu.overflow_count);
        }
        if (cb_baro.overflow_count > 0) {
            printf("WARNING: BARO buffer overflow count: %lu\n", cb_baro.overflow_count);
        }
        if (cb_gps.overflow_count > 0) {
            printf("WARNING: GPS buffer overflow count: %lu\n", cb_gps.overflow_count);
        }
        if (cb_mag.overflow_count > 0) {
			printf("WARNING: MAG buffer overflow count: %lu\n", cb_mag.overflow_count);
        }
        if (cb_bno.overflow_count > 0) {
			printf("WARNING: BNO buffer overflow count: %lu\n", cb_bno.overflow_count);
		}
        if (cb_events.overflow_count > 0) {
			printf("WARNING: EVENTS buffer overflow count: %lu\n", cb_events.overflow_count);
		}

        // Optional: Log buffer statistics every 10 seconds
        /*
        static uint32_t stats_counter = 0;
        if (++stats_counter >= 500) { // 50Hz * 10s
            printf("Buffer usage: IMU=%lu%%, BARO=%lu%%, GPS=%lu%%\n",
                   imu_usage, baro_usage, gps_usage);
            stats_counter = 0;
        }
        */

        vTaskDelayUntil(&last_wake_time, frequency);
    }
}


