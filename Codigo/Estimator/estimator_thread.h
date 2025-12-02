/*
 * estimator_thread.h
 */

#ifndef ESTIMATOR_ESTIMATOR_THREAD_H_
#define ESTIMATOR_ESTIMATOR_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"

#include "Data Handler/flash_data_handler.h"
#include "Flight Computer/flight_computer.h"

void estimator_thread_function();

#endif /* ESTIMATOR_ESTIMATOR_THREAD_H_ */
