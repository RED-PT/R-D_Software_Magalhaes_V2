/*
 * telemetry.c
 *
 *  Created on: Oct 7, 2025
 *      Author: Tomas Teixeira
 */

#include "Telemetry/telemetry.h"
#include "Radio/CRC16/crc16.h"
#include <string.h>

uint16_t telemetry_build_fast(telemetry_fast_t *pkt, fsm_ctx_t *fsm,
                               IMU_t *imu, BARO_t *baro, BNO_t *bno) {
    if (!pkt || !fsm) return 0;

    memset(pkt, 0, sizeof(telemetry_fast_t));

    pkt->packet_type = TELEM_PACKET_FAST;
    pkt->time = HAL_GetTick();
    pkt->state = fsm->state;
    pkt->substate = fsm->substate;

    if (baro) {
        pkt->altitude = baro->altitude_m;
    }

    if (imu) {
        pkt->aceleration = imu->accel_z;
        pkt->gyro[0] = imu->gyro_x;
        pkt->gyro[1] = imu->gyro_y;
        pkt->gyro[2] = imu->gyro_z;
    }

    if (bno) {
        pkt->euler_angles[0] = bno->roll_deg;
        pkt->euler_angles[1] = bno->pitch_deg;
        pkt->euler_angles[2] = bno->heading_deg;
    }

    pkt->throttle = 0.0f;
    pkt->servo_deg[0] = 0.0f;
    pkt->servo_deg[1] = 0.0f;

    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_fast_t) - 2);

    return sizeof(telemetry_fast_t);
}

uint16_t telemetry_build_slow(telemetry_slow_t *pkt, temperature_readings_t *temp, GPS_t *gps) {
    if (!pkt) return 0;

    memset(pkt, 0, sizeof(telemetry_slow_t));

    pkt->packet_type = TELEM_PACKET_SLOW;
    pkt->time = HAL_GetTick();

    if (temp) {
        pkt->temp_ms = temp->temp_ms;
        pkt->temp_cpu = temp->temp_cpu;
        pkt->vbat = (uint16_t)(temp->adc_voltage * 1000);
    }

    if (gps) {
        pkt->latitude = gps->dec_latitude;
        pkt->longitude = gps->dec_longitude;
    }

    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_slow_t) - 2);

    return sizeof(telemetry_slow_t);
}

uint16_t telemetry_build_event(telemetry_event_t *pkt, event_t type, event_payload_u *payload) {
    if (!pkt) return 0;

    memset(pkt, 0, sizeof(telemetry_event_t));

    pkt->packet_type = TELEM_PACKET_EVENT;
    pkt->time = HAL_GetTick();
    pkt->type = type;

    if (payload) {
        memcpy(&pkt->payload, payload, sizeof(event_payload_u));
    }

    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_event_t) - 2);

    return sizeof(telemetry_event_t);
}
