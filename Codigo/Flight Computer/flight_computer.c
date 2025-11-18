/*
 * flight_computer.c
 *
 *  Created on: Oct 7, 2025
 *      Author: texman
 */

#include "flight_computer.h"
#include "flags.h"
#include "defs.h"
#include "retarget.h"
#include "config.h"
#include "Sensors/sensors_thread.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Variables
fsm_ctx_t fsm_ctx = {};
SemaphoreHandle_t xFsmMutex = NULL;

// Functions
void fsm_init() {
	printf("\r\n\r\nInitializing Flight Computer\r\n");

	// Create FreeRTOS mutex
	xFsmMutex = xSemaphoreCreateMutex();
	if (xFsmMutex == NULL) {
		printf("Failed to create FSM mutex\r\n");
		return;
	}
	printf("FSM Mutex created successfully\r\n");

	memset(&fsm_ctx, 0, sizeof(fsm_ctx_t));
	fsm_ctx.state = BOOT;
	fsm_ctx.substate = SUB_NONE;
	fsm_ctx.profile.type = NO_PROFILE;

	printf("FSM Configured!\r\n");
}

int get_fsm_state() {
	int state;
	if (xSemaphoreTake(xFsmMutex, portMAX_DELAY) == pdTRUE) {
		state = fsm_ctx.state;
		xSemaphoreGive(xFsmMutex);
	} else {
		state = -1;  // Error
	}
	return state;
}

int get_fsm_substate() {
	int substate;
	if (xSemaphoreTake(xFsmMutex, portMAX_DELAY) == pdTRUE) {
		substate = fsm_ctx.substate;
		xSemaphoreGive(xFsmMutex);
	} else {
		substate = -1;  // Error
	}
	return substate;
}

void set_fsm_state(int value) {
	if (xSemaphoreTake(xFsmMutex, portMAX_DELAY) == pdTRUE) {
		fsm_ctx.state = value;
		xSemaphoreGive(xFsmMutex);
	}
}

void set_fsm_substate(int value) {
	if (xSemaphoreTake(xFsmMutex, portMAX_DELAY) == pdTRUE) {
		fsm_ctx.substate = value;
		xSemaphoreGive(xFsmMutex);
	}
}
