/*
 * flight_computer_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "flight_computer_thread.h"
#include "cmsis_os.h"

void fsm_thread_function() {
	printf("Flight Computer Thread started...\r\n");
	osDelay(50);
	vTaskSuspend(NULL);
}

