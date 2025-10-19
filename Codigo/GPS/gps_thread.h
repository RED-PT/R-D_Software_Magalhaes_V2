/*
 * gps_thread.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef GPS_GPS_THREAD_H_
#define GPS_GPS_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "print.h"


void ublox_gps_thread_function();


#endif /* GPS_GPS_THREAD_H_ */
