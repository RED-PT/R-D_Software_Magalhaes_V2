/*
 * radio_thread.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Tomas Teixeira
 */

#ifndef RADIO_RADIO_THREAD_H_
#define RADIO_RADIO_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"
#include "Flight Computer/flight_computer.h"

extern QueueHandle_t queue_to_radio;

void radio_thread_function();
bool radio_is_gs_online(void);

#endif /* RADIO_RADIO_THREAD_H_ */
