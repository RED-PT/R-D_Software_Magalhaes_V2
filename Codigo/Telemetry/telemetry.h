/*
 * telemetry.h
 *
 *  Created on: Oct 7, 2025
 *      Author: Tomas Teixeira
 */

#ifndef TELEMETRY_TELEMETRY_H_
#define TELEMETRY_TELEMETRY_H_

#include "defs.h"
#include "Flight Computer/flight_computer.h"

//	Telemetry
//	Fast Packet (50Hz)
typedef struct{
	uint32_t time;
	uint8_t state;
	uint8_t substate;
	float altitude;
	float velocity;
	float aceleration;
	float euler_angles[3];	// roll, pitch, yaw [deg]
	float gyro[3];  // p,q,r [dps]
	float throttle;
	float servo_deg[2];
	uint16_t crc16;
} telemetry_fast_t;

//	Slow Packet (5Hz)
typedef struct{
	uint32_t time;
	float  temp_ms;
	float temp_cpu;
	uint16_t vbat;
	float latitude;
	float longitude;
} telemetry_slow_t;

//	Event Packet (on change)
typedef struct{
	uint32_t time;
	event_t type;
	event_payload_u payload;
	uint16_t crc16;

} telemetry_event_t;

#endif /* TELEMETRY_TELEMETRY_H_ */
