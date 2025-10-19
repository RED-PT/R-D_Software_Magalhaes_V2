/*
 * controller_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "controller_thread.h"

void controller_thread_function() {
	printf("Controller Thread started...\r\n");
	vTaskSuspend(NULL);
}


