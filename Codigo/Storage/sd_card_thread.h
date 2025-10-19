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
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"


void sd_card_thread_function();


#endif /* STORAGE_SD_CARD_THREAD_H_ */
