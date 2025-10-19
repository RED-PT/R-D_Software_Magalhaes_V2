/*
 * gps_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "gps_thread.h"

void ublox_gps_thread_function() {
	printf("GPS Thread started...\r\n");
	vTaskSuspend(NULL);
}


