/*
 * radio_thread.h
 *
 * TDMA Radio Thread - Flight Computer (Slave)
 */

#ifndef RADIO_RADIO_THREAD_H_
#define RADIO_RADIO_THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "defs.h"
#include "stm32f4xx_hal.h"
#include "Telemetry/telemetry.h"
#include "Flight Computer/flight_computer.h"

// TDMA State
typedef enum {
    TDMA_UNSYNCED = 0,
    TDMA_SYNCED
} tdma_state_t;

// TDMA Context
typedef struct {
    tdma_state_t state;
    uint8_t frame_id;               // Current superframe (0-255)
    uint8_t slot_index;             // Current slot (0-9)
    uint32_t frame_start_tick;      // Tick when current frame started
    uint32_t last_sync_tick;        // Tick of last sync received
    uint8_t missed_sync_count;      // Consecutive missed syncs

    // Command ACK state
    uint8_t last_cmd_seq_received;  // Last command SEQ we received
    uint8_t last_cmd_status;        // Status of last command (0=OK)

    // Sequence counters
    uint8_t fast_seq;
    uint8_t slow_seq;
    uint8_t event_seq;
} tdma_ctx_t;

// Statistics
typedef struct {
    uint32_t tx_fast;
    uint32_t tx_slow;
    uint32_t tx_event;
    uint32_t rx_cmd;
    uint32_t rx_sync;
    uint32_t crc_errors;
    uint32_t sync_corrections;
} radio_stats_t;

extern tdma_ctx_t tdma_ctx;
extern radio_stats_t radio_stats;

void radio_thread_function();

// Called by telemetry_thread to queue sensor data
void radio_update_sensor_data(const IMU_t *imu, const BARO_t *baro,
                               const BNO_t *bno, const GPS_t *gps);

#endif /* RADIO_RADIO_THREAD_H_ */
