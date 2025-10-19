/*
 * data_handler_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

// Include Flash Data Handler Drivers
#include "flash_data_handler.h"
#include "print.h"

void data_handler_thread_function(void *argument) {
	printf("Data Handler monitoring thread started\r\n");

	    TickType_t last_wake_time = xTaskGetTickCount();
	    const TickType_t frequency = pdMS_TO_TICKS(5000); // 5 seconds

	    while(1) {
	        // Just monitor buffer health every 5 seconds
	        printf("Data Handler Status\r\n");

	        // Check for overflows
	        if (cb_imu.overflow_count > 0) {
	            printf("IMU overflows: %lu\n", cb_imu.overflow_count);
	        }
	        if (cb_baro.overflow_count > 0) {
	            printf("BARO overflows: %lu\n", cb_baro.overflow_count);
	        }
	        if (cb_mag.overflow_count > 0) {
	            printf("MAG overflows: %lu\n", cb_mag.overflow_count);
	        }
	        if (cb_bno.overflow_count > 0) {
	            printf("BNO overflows: %lu\n", cb_bno.overflow_count);
	        }
	        if (cb_gps.overflow_count > 0) {
	            printf("GPS overflows: %lu\n", cb_gps.overflow_count);
	        }

	        // Check queue levels
	        UBaseType_t est_msgs = uxQueueMessagesWaiting(queue_to_estimator);
	        UBaseType_t tel_msgs = uxQueueMessagesWaiting(queue_to_telemetry);
	        UBaseType_t log_msgs = uxQueueMessagesWaiting(queue_to_logger);

	        printf("Queue levels: EST=%lu/%u, TEL=%lu/%u, LOG=%lu/%u\r\n", est_msgs, QUEUE_LENGTH_ESTIMATOR, tel_msgs, QUEUE_LENGTH_TELEMETRY, log_msgs, QUEUE_LENGTH_LOGGER);

	        vTaskDelayUntil(&last_wake_time, frequency);
	    }
}
