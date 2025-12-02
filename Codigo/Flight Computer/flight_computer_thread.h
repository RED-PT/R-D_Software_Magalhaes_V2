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
#include "queue.h"
#include "flight_computer.h"

// Thread function
void fsm_thread_function(void *argument);

// Send telemetry event (called internally)
void fsm_send_telemetry_event(telemetry_event_type_t type, const void *payload, uint16_t size);

#endif /* FLIGHT_COMPUTER_FLIGHT_COMPUTER_THREAD_H_ */
