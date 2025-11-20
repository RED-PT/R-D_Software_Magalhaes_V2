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
#include "stm32f4xx_hal.h"

#define TELEM_PACKET_FAST  0x01
#define TELEM_PACKET_SLOW  0x02
#define TELEM_PACKET_EVENT 0x03

typedef struct __attribute__((packed)) {
	uint8_t packet_type;
	uint32_t time;
	uint8_t state;
	uint8_t substate;
	float altitude;
	float velocity;
	float aceleration;
	float euler_angles[3];
	float gyro[3];
	float throttle;
	float servo_deg[2];
	uint16_t crc16;
} telemetry_fast_t;

typedef struct __attribute__((packed)) {
	uint8_t packet_type;
	uint32_t time;
	float temp_ms;
	float temp_cpu;
	uint16_t vbat;
	float latitude;
	float longitude;
	uint16_t crc16;
} telemetry_slow_t;

typedef struct __attribute__((packed)) {
	uint8_t packet_type;
	uint32_t time;
	event_t type;
	event_payload_u payload;
	uint16_t crc16;
} telemetry_event_t;

typedef struct {
	uint8_t buffer[128];
	uint16_t length;
} radio_packet_t;

uint16_t telemetry_build_fast(telemetry_fast_t *pkt, fsm_ctx_t *fsm,
                               IMU_t *imu, BARO_t *baro, BNO_t *bno);
uint16_t telemetry_build_slow(telemetry_slow_t *pkt, temperature_readings_t *temp, GPS_t *gps);
uint16_t telemetry_build_event(telemetry_event_t *pkt, event_t type, event_payload_u *payload);

#endif /* TELEMETRY_TELEMETRY_H_ */
