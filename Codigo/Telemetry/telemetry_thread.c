/*
 * telemetry_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "telemetry_thread.h"
#include "cmsis_os.h"

void telemetry_thread_function() {
	printf("Telemetry Thread started...\r\n");
	osDelay(50);
	vTaskSuspend(NULL);
}

