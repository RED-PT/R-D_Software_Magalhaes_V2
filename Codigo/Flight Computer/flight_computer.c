/*
 * flight_computer.c
 *
 *  Created on: Oct 7, 2025
 *      Author: texman
 */

// Includes
#include "flight_computer.h"
#include "flags.h"
#include "defs.h"

// Variables
fsm_ctx_t fsm_ctx = {};

//Functions
void fsm_init() {

	mutex_lock(&mutex_flags);

	memset(&fsm_ctx, 0, sizeof(fsm_ctx_t)); // clear all fields

	fsm_ctx.state = BOOT;
	fsm_ctx.substate = SUB_NONE;

	mutex_unlock(&mutex_flags);
}

int get_fsm_state() {

	mutex_lock(&mutex_flags);
	int fsm_state = fsm_ctx.state;
	mutex_unlock(&mutex_flags);
	return fsm_state;
}

int get_fsm_substate() {

	mutex_lock(&mutex_flags);
	int fsm_substate = fsm_ctx.substate;
	mutex_unlock(&mutex_flags);
	return fsm_substate;
}

void set_fsm_state(int value) {

	mutex_lock(&mutex_flags);
	fsm_ctx.state = value;
	mutex_unlock(&mutex_flags);
}

void set_fsm_substate(int value) {

	mutex_lock(&mutex_flags);
	fsm_ctx.substate = value;
	mutex_unlock(&mutex_flags);
}



