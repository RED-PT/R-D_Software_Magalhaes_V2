/*
 * controller_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef CONTROLLER_CONTROLLER_THREAD_H_
#define CONTROLLER_CONTROLLER_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"

void controller_thread_function();



#endif /* CONTROLLER_CONTROLLER_THREAD_H_ */
