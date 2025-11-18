/*
 * flight_computer.h
 *
 *  Created on: Oct 7, 2025
 *      Author: texman
 */

#ifndef FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_
#define FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_

#include "defs.h"
#include "FreeRTOS.h"
#include "semphr.h"


//	FSM States, Sub-states, Profiles & Parameters
// States
typedef enum{
	 BOOT = 0,
	 IDLE,
	 CONFIGED,
	 ARMED,
	 TEST_STAND,
	 FLIGHT,
	 ABORT,
	 SAFE
} fsm_state_t;

// Sub-states
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

// Profiles
typedef enum{
	NO_PROFILE = 0,
	GUTTER_RAMP,
	GUTTER_HOLD,
	FLIGHT_PARAMETRIC
} flight_profile_t;

//	Parameters
typedef struct{
	flight_profile_t type;

	//	Common
	float target_altitude; //[m]
	float flare_altitude; //[m]
	float touchdown_velocity; //[m/s]

	// Gutter Ramp Profile

	// Gutter Hold Profile
	float hold_throttle;

	// Safety Limits
	float throttle_min, throttle_max;

} profile_params_t;

// FSM Context
typedef struct{
	fsm_state_t state;
	fsm_substate_t substate;
	profile_params_t profile;

	// flags


} fsm_ctx_t;

//	Commands (inputs from GroundStation→FlightComputer)
typedef enum{
	CMD_NONE = 0,
	CMD_PING,
	CMD_SET_PROFILE,
	CMD_ARM,
	CMD_DISARM,
	CMD_START_TEST,
	CMD_LAUNCH,
	CMD_ABORT,
	CMD_FORCE_SAFE,
	CMD_SET_TARGET_ALT
} command_t;

// Events (from FlightComputer to GroundStation; Used in Event Packet)
typedef enum {
    EVT_STATE_CHANGE = 0,
	EVT_FAULT,
	EVT_ABORT,
	EVT_PROFILE_LOADED,
	EVT_CHECKS_GREEN,
	EVT_CHECKS_RED,
	EVT_GENERIC_MSG
} event_t;

// State Changed Details
typedef struct {
    uint8_t old_state;
    uint8_t old_sub;
    uint8_t new_state;
    uint8_t new_sub;
    uint8_t reason_evt; // which event caused it
} event_state_change_t;

// Event Fault Info
typedef struct {
    uint16_t code; // numerical identifier that tells what kind of fault happened
    char desc[32]; // ASCII short description
} event_fault_t;

typedef union {
    event_state_change_t state;
    event_fault_t        fault;
    profile_params_t profile;
    char msg[48];
} event_payload_u;


// FSM Functions Prototypes
// FSM Functions Prototypes
void fsm_init();
void fsm_handle_event(fsm_ctx_t* ctx, event_t event, const void* payload, uint16_t size);
int get_fsm_state();
int get_fsm_substate();
void set_fsm_state(int value);
void set_fsm_substate(int value);

// Variables
extern fsm_ctx_t fsm_ctx;
extern SemaphoreHandle_t xFsmMutex;  // FreeRTOS mutex

#endif /* FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_ */
