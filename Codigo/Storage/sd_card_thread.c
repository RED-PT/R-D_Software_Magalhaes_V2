/*
 * sd_card_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "sd_card_thread.h"

void sd_card_thread_function() {
	printf("SD Card Thread started...\r\n");
	vTaskSuspend(NULL);
}

