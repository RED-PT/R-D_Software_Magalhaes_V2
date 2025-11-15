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
	vTaskSuspend(NULL);
}


