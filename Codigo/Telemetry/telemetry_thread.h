/*
 * telemetry_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef TELEMETRY_TELEMETRY_THREAD_H_
#define TELEMETRY_TELEMETRY_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"
#include "Data Handler/flash_data_handler.h"
#include "telemetry.h"

// Queue handles (defined in create_threads.c)
extern QueueHandle_t queue_to_radio_tx;
extern QueueHandle_t queue_fsm_events;

void telemetry_thread_function();

#endif /* TELEMETRY_TELEMETRY_THREAD_H_ */
