/*
 * flight_computer.h
 *
 *  Created on: Oct 7, 2025
 *      Author: texman
 */

#ifndef FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_
#define FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_

#include "defs.h"

//	FSM States, Sub-states & Profiles
typedef enum{
	 BOOT = 0,
	 IDLE,
	 CONFIG,
	 ARMED,
	 TEST_STAND,
	 FLIGHT,
	 ABORT,
	 SAFE
} fsm_state_t;

typedef enum{
	SUB_NONE = 0,
	// TEST_STAND
	SUB_TS_SENSOR_CHECK,
	SUB_TS_THROTTLE_RAMP,
	// FLIGHT
	SUB_FL_IGNITION,
	SUB_FL_LIFTOFF_DETECT,
	SUB_FL_ASCENT,
	SUB_FL_COAST,
	SUB_FL_DESCENT_BRAKE,
	SUB_FL_LANDING_FLARE,
	SUB_FL_TOUCHDOWN,
	SUB_FL_RECOVERY
} fsm_substate_t;

typedef enum{
	GUTTER_RAMP = 0,
	GUTTER_HOLD,
	FLIGHT_PARAMETRIC
} flight_profile_t;

#endif /* FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_ */
