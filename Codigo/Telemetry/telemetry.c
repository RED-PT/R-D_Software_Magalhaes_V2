/*
 * telemetry.c
 *
 * Telemetry packet building functions
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "Telemetry/telemetry.h"
#include "Flight Computer/flight_computer.h"

uint16_t telemetry_build_fast(telemetry_fast_t *pkt, const void *ctx, const IMU_t *imu, const BARO_t *baro, const BNO_t *bno) {
    if (pkt == NULL) return 0;

    const fsm_ctx_t *fsm = (const fsm_ctx_t*)ctx;

    memset(pkt, 0, sizeof(telemetry_fast_t));

    pkt->packet_type = TELEM_PACKET_FAST;
    pkt->time = HAL_GetTick();

    // FSM state
    if (fsm != NULL) {
        pkt->state = fsm->state;
        pkt->substate = fsm->substate;
    }

    // IMU data - scale to int16
    if (imu != NULL) {
        pkt->accel_x = (int16_t)(imu->accel_x * 1000.0f);  // g -> mg
        pkt->accel_y = (int16_t)(imu->accel_y * 1000.0f);
        pkt->accel_z = (int16_t)(imu->accel_z * 1000.0f);
        pkt->gyro_x = (int16_t)(imu->gyro_x * 100.0f);     // dps -> 0.01 dps
        pkt->gyro_y = (int16_t)(imu->gyro_y * 100.0f);
        pkt->gyro_z = (int16_t)(imu->gyro_z * 100.0f);
    }

    // Barometer data
    if (baro != NULL) {
        pkt->pressure = (int32_t)(baro->pressure_mbar * 100.0f);  // mbar -> 0.01 mbar
        pkt->baro_temp = (int16_t)(baro->temperature_c * 100.0f); // C -> 0.01 C
        pkt->altitude = (int16_t)(baro->altitude_m * 10.0f);      // m -> 0.1 m
    }

    // BNO055 orientation
    if (bno != NULL) {
        pkt->heading = (int16_t)(bno->heading_deg * 100.0f);  // deg -> 0.01 deg
        pkt->pitch = (int16_t)(bno->pitch_deg * 100.0f);
        pkt->roll = (int16_t)(bno->roll_deg * 100.0f);
    }

    // Calculate CRC
    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_fast_t) - 2);

    return sizeof(telemetry_fast_t);
}

uint16_t telemetry_build_slow(telemetry_slow_t *pkt, const temperature_readings_t *temps, const GPS_t *gps) {
    if (pkt == NULL) return 0;

    memset(pkt, 0, sizeof(telemetry_slow_t));

    pkt->packet_type = TELEM_PACKET_SLOW;
    pkt->time = HAL_GetTick();

    // GPS data
    if (gps != NULL) {
        pkt->latitude = (int32_t)(gps->dec_latitude * 1e7);   // deg -> 1e-7 deg
        pkt->longitude = (int32_t)(gps->dec_longitude * 1e7);
        pkt->gps_altitude = (int16_t)(gps->msl_altitude * 10.0f); // m -> 0.1 m
        pkt->gps_lock = gps->lock;
        pkt->satellites = gps->satellites;
        pkt->hdop = (uint16_t)(gps->hdop * 100.0f);  // -> 0.01
    }

    // Temperature readings
    if (temps != NULL) {
        pkt->temp_imu = (int16_t)(temps->imu_temp_c * 100.0f);
        pkt->temp_baro = (int16_t)(temps->baro_temp_c * 100.0f);
    }

    // System status
    pkt->battery_pct = 100;  // TODO: Implement
    pkt->sd_status = 1;      // TODO: Implement
    pkt->free_heap = (uint16_t)xPortGetFreeHeapSize();

    // Calculate CRC
    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_slow_t) - 2);

    return sizeof(telemetry_slow_t);
}

uint16_t telemetry_build_event(telemetry_event_t *pkt, uint8_t event_type, uint8_t state, uint8_t substate, const void *payload, uint16_t payload_size) {
    if (pkt == NULL) return 0;

    memset(pkt, 0, sizeof(telemetry_event_t));

    pkt->packet_type = TELEM_PACKET_EVENT;
    pkt->time = HAL_GetTick();
    pkt->event_type = event_type;
    pkt->state = state;
    pkt->substate = substate;

    // Copy payload if provided
    if (payload != NULL && payload_size > 0) {
        uint16_t copy_size = (payload_size > sizeof(pkt->payload)) ? sizeof(pkt->payload) : payload_size;
        memcpy(&pkt->payload, payload, copy_size);
    }

    // Calculate CRC
    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_event_t) - 2);

    return sizeof(telemetry_event_t);
}
