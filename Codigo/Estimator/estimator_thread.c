/*
 * estimar_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "estimator_thread.h"
#include "cmsis_os.h"

void estimator_thread_function() {
	printf("Estimator Thread started...\r\n");
    data_packet_t packet;

    while(1) {
        // Just consume packets, don't process them yet
        if (xQueueReceive(queue_to_estimator, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
            data_packet_unlock(&packet);  // Release the buffer lock
            // Discard for now (we're testing data flow)
        }
    }
}


