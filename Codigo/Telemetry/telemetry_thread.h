/*
 * telemetry_thread.h
 */

#ifndef TELEMETRY_TELEMETRY_THREAD_H_
#define TELEMETRY_TELEMETRY_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "defs.h"
#include "Data Handler/flash_data_handler.h"

void telemetry_thread_function();

#endif /* TELEMETRY_TELEMETRY_THREAD_H_ */
