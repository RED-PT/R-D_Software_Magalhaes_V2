/*
 * radio_thread.h
 *
 *  Created on: Nov 18, 2025
 *      Author: Tomas Teixeira
 */

#ifndef RADIO_RADIO_THREAD_H_
#define RADIO_RADIO_THREAD_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "config.h"
#include "defs.h"
#include "LORA Drivers/lora_sx1276.h"
#include "print.h"
#include "Flight Computer/flight_computer.h"
#include "Telemetry/telemetry.h"  // For radio_tx_packet_t

// Command packet structure (received from ground station)
typedef struct {
    uint8_t command;
    uint8_t payload[32];
    uint8_t payload_length;
} command_packet_t;

// Queue handles (defined in create_threads.c)
extern QueueHandle_t queue_to_radio_tx;
extern QueueHandle_t queue_radio_rx_to_fsm;

// Function prototypes
void radio_thread_function();
bool parse_command(uint8_t *data, uint8_t length, command_packet_t *cmd);

#endif /* RADIO_RADIO_THREAD_H_ */
