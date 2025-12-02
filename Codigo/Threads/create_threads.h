/*
 * create_threads.h
 *
 *  Created on: Oct 6, 2025
 *      Author: texman
 */

#ifndef INC_CREATE_THREADS_H_
#define INC_CREATE_THREADS_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cmsis_os2.h"
#include "message_buffer.h"
#include "queue.h"
#include "timers.h"

#include "Sensors/sensors_thread.h"
#include "Data Handler/data_handler_thread.h"
#include "Estimator/estimator_thread.h"
#include "Controller/controller_thread.h"
#include "Storage/sd_card_thread.h"
#include "Telemetry/telemetry_thread.h"
#include "Radio/radio_thread.h"  // ADD THIS
#include "Flight Computer/flight_computer_thread.h"

// Thread IDs
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t sensors_thread_id;
extern osThreadId_t data_handler_thread_id;
extern osThreadId_t estimator_thread_id;
extern osThreadId_t controller_thread_id;
extern osThreadId_t sd_card_thread_id;
extern osThreadId_t telemetry_thread_id;
extern osThreadId_t radio_thread_id;
extern osThreadId_t fsm_thread_id;

// Thread Attributes
extern const osThreadAttr_t sensors_thread_attr;
extern const osThreadAttr_t data_handler_thread_attr;
extern const osThreadAttr_t estimator_thread_attr;
extern const osThreadAttr_t controller_thread_attr;
extern const osThreadAttr_t sd_card_thread_attr;
extern const osThreadAttr_t telemetry_thread_attr;
extern const osThreadAttr_t radio_thread_attr;
extern const osThreadAttr_t fsm_thread_attr;

// Timers
extern TimerHandle_t xBaroTimer;
extern TimerHandle_t xBnoTimer;

// Queue Handles
extern QueueHandle_t queue_to_radio_tx;
extern QueueHandle_t queue_radio_rx_to_fsm;
extern QueueHandle_t queue_fsm_events;
extern QueueHandle_t queue_cmd_to_fsm;
extern QueueHandle_t queue_event_to_fsm;

// Stream Buffers
extern StreamBufferHandle_t stream_buffer_gps;

void create_threads(void);

#endif /* INC_CREATE_THREADS_H_ */

