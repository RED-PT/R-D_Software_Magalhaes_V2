/*
 * data_handler_thread.h
 *
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef DATA_HANDLER_DATA_HANDLER_THREAD_H_
#define DATA_HANDLER_DATA_HANDLER_THREAD_H_

#include "flash_data_handler.h"
#include "Threads/create_threads.h"
#include "print.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>

// Aggressive flush thresholds
#define BUFFER_FULL_THRESHOLD_PCT   50      // Trigger flush at 50% full
#define SAFETY_FLUSH_TIMEOUT_MS     1000    // Safety: flush every 1s

// Thread notification bits
#define DATA_HANDLER_NOTIFY_THRESHOLD   (1 << 0)

// Relaxed queue space requirement
#define MIN_QUEUE_SPACE_FOR_FLUSH       2

extern osThreadId_t data_handler_thread_id;

void data_handler_thread_function(void *argument);
void data_handler_notify_threshold(void);

#endif /* DATA_HANDLER_DATA_HANDLER_THREAD_H_ */
