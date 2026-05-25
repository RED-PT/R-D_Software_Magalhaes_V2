/*
 * telemetry.c
 *
 * TDMA-aware telemetry packet building
 */

#include "Telemetry/telemetry.h"
#include "Flight Computer/flight_computer.h"
#include "main.h"

uint16_t telemetry_build_fast(telemetry_fast_t *pkt, uint8_t frame_id, uint8_t slot_id,
                               uint8_t seq, const void *ctx,
                               const IMU_t *imu, const BARO_t *baro, const BNO_t *bno,
                               uint8_t last_cmd_seq, uint8_t last_cmd_status) {
    if (!pkt) return 0;

    const fsm_ctx_t *fsm = (const fsm_ctx_t*)ctx;
    memset(pkt, 0, sizeof(telemetry_fast_t));

    // Header
    pkt->packet_type = TELEM_PACKET_FAST;
    pkt->frame_id = frame_id;
    pkt->slot_id = slot_id;
    pkt->seq = seq;
    pkt->flags = 0;
    pkt->time = HAL_GetTick();

    // FSM state
    if (fsm) {
        pkt->state = fsm->state;
        pkt->substate = fsm->substate;

        // Set flags based on state
        if (fsm->state == STATE_ARMED) pkt->flags |= TELEM_FLAG_ARMED;
        if (fsm->state == STATE_FLIGHT) pkt->flags |= TELEM_FLAG_FLIGHT;
    }

    // Command ACK piggyback
    pkt->last_cmd_seq_acked = last_cmd_seq;
    pkt->last_cmd_status = last_cmd_status;

    // IMU data
    if (imu) {
        pkt->accel_x = (int16_t)(imu->accel_x * 1000.0f);
        pkt->accel_y = (int16_t)(imu->accel_y * 1000.0f);
        pkt->accel_z = (int16_t)(imu->accel_z * 1000.0f);
        pkt->gyro_x = (int16_t)(imu->gyro_x * 100.0f);
        pkt->gyro_y = (int16_t)(imu->gyro_y * 100.0f);
        pkt->gyro_z = (int16_t)(imu->gyro_z * 100.0f);
    }

    // Barometer
    if (baro) {
        pkt->altitude = (int16_t)(baro->altitude_m * 10.0f);
        pkt->vario = 0;  // TODO: compute from altitude history
    }

    // Orientation
    if (bno) {
        pkt->pitch = (int16_t)(bno->pitch_deg * 10.0f);
        pkt->roll = (int16_t)(bno->roll_deg * 10.0f);
        pkt->yaw = (int16_t)(bno->heading_deg * 10.0f);
    }

    // CRC
    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_fast_t) - 2);

    return sizeof(telemetry_fast_t);
}

uint16_t telemetry_build_slow(telemetry_slow_t *pkt, uint8_t frame_id, uint8_t seq,
                               const GPS_t *gps, const BARO_t *baro,
                               uint8_t battery_pct, uint8_t sd_status,
                               uint8_t tdma_mode, uint8_t slow_slot) {
    if (!pkt) return 0;

    memset(pkt, 0, sizeof(telemetry_slow_t));

    // Header
    pkt->packet_type = TELEM_PACKET_SLOW;
    pkt->frame_id = frame_id;
    pkt->slot_id = slow_slot;
    pkt->seq = seq;
    pkt->time = HAL_GetTick();

    // GPS
    if (gps) {
        pkt->latitude = (int32_t)(gps->dec_latitude * 1e7);
        pkt->longitude = (int32_t)(gps->dec_longitude * 1e7);
        pkt->gps_altitude = (int16_t)(gps->msl_altitude * 10.0f);
        pkt->gps_lock = gps->lock;
        pkt->satellites = gps->satellites;
    }

    // Baro/System
    if (baro) {
        pkt->pressure = (int16_t)((baro->pressure_mbar - 1000.0f) * 10.0f);
        pkt->temp_baro = (int16_t)(baro->temperature_c * 10.0f);
    }

    pkt->battery_pct = battery_pct;
    pkt->sd_status = sd_status;
    pkt->free_heap = (uint16_t)(xPortGetFreeHeapSize() / 10);  // In units of 10 bytes
    pkt->tdma_mode = tdma_mode;

    // CRC (must be after every field is populated)
    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_slow_t) - 2);

    return sizeof(telemetry_slow_t);
}

uint16_t telemetry_build_event(telemetry_event_t *pkt, uint8_t frame_id, uint8_t slot_id,
                                uint8_t seq, uint8_t event_type,
                                uint8_t state, uint8_t substate,
                                const void *payload, uint16_t payload_size) {
    if (!pkt) return 0;

    memset(pkt, 0, sizeof(telemetry_event_t));

    pkt->packet_type = TELEM_PACKET_EVENT;
    pkt->frame_id = frame_id;
    pkt->slot_id = slot_id;
    pkt->seq = seq;
    pkt->time = HAL_GetTick();
    pkt->event_type = event_type;
    pkt->state = state;
    pkt->substate = substate;

    if (payload && payload_size > 0) {
        uint16_t copy_size = (payload_size > sizeof(pkt->payload)) ? sizeof(pkt->payload) : payload_size;
        memcpy(pkt->payload, payload, copy_size);
    }

    pkt->crc16 = crc16_calculate((uint8_t*)pkt, sizeof(telemetry_event_t) - 2);

    return sizeof(telemetry_event_t);
}
