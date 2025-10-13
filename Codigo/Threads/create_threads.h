/*
 * create_threads.h
 *
 *  Created on: Oct 6, 2025
 *      Author: texman
 */

#ifndef INC_CREATE_THREADS_H_
#define INC_CREATE_THREADS_H_


//  freertos libraries
#include "FreeRTOS.h" //Don't remove !!!!!!!!!!!
#include "semphr.h"
#include "task.h"
#include "cmsis_os2.h"
#include "message_buffer.h"
#include "timers.h"

#include "Sensors/sensors_thread.h"
#include "Data Handler/data_handler_thread.h"
#include "Estimator/estimator_thread.h"
#include "Controller/controller_thread.h"
#include "Storage/sd_card_thread.h"
#include "Telemetry/telemetry_thread.h"
#include "Flight Computer/flight_computer_thread.h"
#include "GPS/gps_thread.h"
#include "Sensors/sensors_thread.h"

//	ID threads

// Thread chata
extern osThreadId_t defaultTaskHandle;

// Thread IDs
extern osThreadId_t sensors_thread_id;
extern osThreadId_t data_handler_thread_id;
extern osThreadId_t estimator_thread_id;
extern osThreadId_t controller_thread_id;
extern osThreadId_t sd_card_thread_id;
extern osThreadId_t telemetry_thread_id;
extern osThreadId_t fsm_thread_id;
extern osThreadId_t ublox_gps_thread_id;

// Thread Attributes
extern const osThreadAttr_t sensors_thread_attr;
extern const osThreadAttr_t data_handler_thread_attr;
extern const osThreadAttr_t estimator_thread_attr;
extern const osThreadAttr_t controller_thread_attr;
extern const osThreadAttr_t sd_card_thread_attr;
extern const osThreadAttr_t telemetry_thread_attr;
extern const osThreadAttr_t fsm_thread_attr;
extern const osThreadAttr_t ublox_gps_thread_attr;

//Timers
extern TimerHandle_t xBaroTimer;
extern TimerHandle_t xBnoTimer;



// Buffers
// Stream Buffers
extern StreamBufferHandle_t stream_buffer_gps;

// Initialization threads functions
void create_threads(void);

#endif /* INC_CREATE_THREADS_H_ */

