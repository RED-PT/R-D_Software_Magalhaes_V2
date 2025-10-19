/*
 * telemetry_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "telemetry_thread.h"

void telemetry_thread_function() {
	printf("Flight Computer Thread started...\r\n");
	vTaskSuspend(NULL);
}

