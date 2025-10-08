// Includes
#include "create_threads.h"
#include "threads.h"
#include "retarget.h"
#include "flags.h"
#include "print.h"
#include <stdio.h>
#include <string.h>
#include "defs.h"
#include "config.h"
#include "test.h"

// Threads Ids
osThreadId_t ublox_gps_thread_id;

// Stream Buffer
StreamBufferHandle_t stream_buffer_gps;

// Threads Attributes
const osThreadAttr_t ublox_gps_thread_attributes = { .name = "thread_ublox_gps",
		.stack_size = 1000 * 4, .priority = (osPriority_t) osPriorityNormal1, };

// Functions
void create_threads() {

	// NÃO REMOVER - faz funcionar os printf()
	RetargetInit(UART_DEBUG);

	// Terminate Task chata FreeRTOS
	osThreadTerminate(defaultTaskHandle);

	ublox_gps_thread_id = osThreadNew(ublox_gps_function, NULL, &ublox_gps_thread_attributes);
	if (ublox_gps_thread_id == NULL) printf("UBLOX GPS thread creation failed\n");
}


