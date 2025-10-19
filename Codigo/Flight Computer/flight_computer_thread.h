/*
 * flight_computer_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_
#define FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"

void fsm_thread_function();


#endif /* FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_ */
