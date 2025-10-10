// Includes
#include "create_threads.h"
#include "threads.h"
#include "flags.h"
#include "print.h"
#include <stdio.h>
#include <string.h>
#include "defs.h"
#include "config.h"

// Threads Ids
osThreadId_t ublox_gps_thread_id;
osThreadId_t fsm_thread_id;

// Stream Buffer
StreamBufferHandle_t stream_buffer_gps;

// Threads Attributes
const osThreadAttr_t ublox_gps_thread_attributes = { .name = "thread_ublox_gps",
		.stack_size = 1000 * 4, .priority = (osPriority_t) osPriorityNormal1, };

const osThreadAttr_t fsm_thread_attributes = { .name =
		"fsm_thread", .stack_size = 500 * 4, .priority =
		(osPriority_t) osPriorityHigh, };

// Functions
void create_threads() {

	// Terminate Task chata FreeRTOS
	osThreadTerminate(defaultTaskHandle);

	fsm_thread_id = osThreadNew(fsm_thread_function, NULL, &fsm_thread_attributes);
	if (fsm_thread_id == NULL) printf("FSM Thread creation failed\n");

	ublox_gps_thread_id = osThreadNew(ublox_gps_function, NULL, &ublox_gps_thread_attributes);
	if (ublox_gps_thread_id == NULL) printf("UBLOX GPS thread creation failed\n");
}


