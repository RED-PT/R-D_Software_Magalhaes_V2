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

//	ID threads
// Thread chata
extern osThreadId_t defaultTaskHandle;
// Threads
extern osThreadId_t ublox_gps_thread_id;
extern osThreadId_t fsm_thread_id;

// Buffers
// Stream Buffers
extern StreamBufferHandle_t stream_buffer_gps;

// create threads function
extern void create_threads();

#endif /* INC_CREATE_THREADS_H_ */

