/*
 * sd_card_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef STORAGE_SD_CARD_THREAD_H_
#define STORAGE_SD_CARD_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"

// Thread function - run as a FreeRTOS task
void sd_card_thread_function(void *argument);

// Cleanup function - call before shutdown
void sd_card_close(void);

// Pause/resume SD operations during motor tests
void sd_card_pause(void);
void sd_card_resume(void);
bool sd_card_is_paused(void);


#endif /* STORAGE_SD_CARD_THREAD_H_ */
