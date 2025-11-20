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
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"

#include "Data Handler/flash_data_handler.h"


void telemetry_thread_function();


#endif /* TELEMETRY_TELEMETRY_THREAD_H_ */
