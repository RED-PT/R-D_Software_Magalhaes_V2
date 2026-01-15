/*
 * telemetry.h
 *
 * TDMA-aware telemetry packet definitions
 */

#ifndef TELEMETRY_TELEMETRY_H_
#define TELEMETRY_TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "defs.h"
#include "Radio/CRC16/crc16.h"

// ============================================================================
// Packet Type Identifiers
// ============================================================================
#define TELEM_PACKET_FAST       0x01
#define TELEM_PACKET_SLOW       0x02
#define TELEM_PACKET_EVENT      0x03
#define TELEM_PACKET_COMMAND    0x04
#define TELEM_PACKET_SYNC       0x05

// ============================================================================
// TDMA Configuration
// ============================================================================
#define TDMA_SUPERFRAME_MS      1000    // 1 second superframe
#define TDMA_SLOT_MS            100     // 100ms per slot
#define TDMA_SLOTS_PER_FRAME    10      // 10 slots per superframe
#define TDMA_TX_SLOTS           8       // Slots 0-7 for fast telemetry
#define TDMA_SLOW_SLOT          8       // Slot 8 for slow telemetry
#define TDMA_RX_SLOT            9       // Slot 9 for GS commands

// ============================================================================
// Fast Telemetry Packet (slots 0-7)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_FAST
    uint8_t frame_id;           // Superframe counter (mod 256)
    uint8_t slot_id;            // Current slot (0-9)
    uint8_t seq;                // Sequence number
    uint8_t flags;              // Event flags
    uint32_t time;              // Timestamp [ms]

    // FSM state
    uint8_t state;              // FSM state
    uint8_t substate;           // FSM substate

    // Command ACK (piggyback)
    uint8_t last_cmd_seq_acked; // Last command SEQ acknowledged
    uint8_t last_cmd_status;    // 0=OK, 1=rejected, 2=error

    // IMU data
    int16_t accel_x;            // [mg]
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;             // [0.01 dps]
    int16_t gyro_y;
    int16_t gyro_z;

    // Barometer
    int16_t altitude;           // [0.1 m]
    int16_t vario;              // [0.01 m/s] vertical velocity

    // Orientation
    int16_t pitch;              // [0.1 deg]
    int16_t roll;               // [0.1 deg]
    int16_t yaw;                // [0.1 deg] heading from BNO055

    uint16_t crc16;
} telemetry_fast_t;  // 37 bytes

// ============================================================================
// Slow Telemetry Packet (slot 8)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_SLOW
    uint8_t frame_id;           // Superframe counter
    uint8_t slot_id;            // Should be 8
    uint8_t seq;                // Sequence number
    uint32_t time;              // Timestamp [ms]

    // GPS
    int32_t latitude;           // [1e-7 deg]
    int32_t longitude;          // [1e-7 deg]
    int16_t gps_altitude;       // [0.1 m]
    uint8_t gps_lock;
    uint8_t satellites;

    // System status
    int16_t pressure;           // [0.1 mbar offset from 1000]
    int16_t temp_baro;          // [0.1 C]
    uint8_t battery_pct;
    uint8_t sd_status;
    uint16_t free_heap;

    uint16_t crc16;
} telemetry_slow_t;  // 30 bytes (4+4+8+2+2+4+2+4)

// ============================================================================
// Event Packet (sent in any TX slot when event occurs)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_EVENT
    uint8_t frame_id;
    uint8_t slot_id;
    uint8_t seq;
    uint32_t time;

    uint8_t event_type;
    uint8_t state;
    uint8_t substate;
    uint8_t payload[24];

    uint16_t crc16;
} telemetry_event_t;  // 37 bytes (4+4+3+24+2)

// ============================================================================
// Command Packet (GS → FC, slot 9)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_COMMAND
    uint8_t frame_id;           // GS superframe counter
    uint8_t cmd_id;             // Command type
    uint8_t cmd_seq;            // Command sequence for ACK
    uint32_t time;              // GS timestamp
    uint8_t params[8];          // Command parameters
    uint16_t crc16;
} command_packet_t;  // 16 bytes

// ============================================================================
// Sync Beacon Packet (GS → FC, slot 9, when no commands)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_SYNC
    uint8_t frame_id;           // GS superframe counter
    uint8_t slot_id;            // Should be 9
    uint8_t reserved;
    uint32_t gs_time;           // GS timestamp for sync
    uint16_t crc16;
} sync_packet_t;  // 10 bytes

// ============================================================================
// Telemetry Flags
// ============================================================================
#define TELEM_FLAG_EVENT_PENDING    (1 << 0)
#define TELEM_FLAG_LOW_BATTERY      (1 << 1)
#define TELEM_FLAG_SD_ERROR         (1 << 2)
#define TELEM_FLAG_GPS_LOCK         (1 << 3)
#define TELEM_FLAG_ARMED            (1 << 4)
#define TELEM_FLAG_FLIGHT           (1 << 5)

// ============================================================================
// Radio Packet Wrapper
// ============================================================================
#define RADIO_MAX_PACKET_SIZE   48

typedef struct {
    uint16_t length;
    uint8_t buffer[RADIO_MAX_PACKET_SIZE];
} radio_packet_t;

// ============================================================================
// Temperature Readings
// ============================================================================
typedef struct {
    float imu_temp_c;
    float baro_temp_c;
} temperature_readings_t;

// ============================================================================
// Build Functions
// ============================================================================
uint16_t telemetry_build_fast(telemetry_fast_t *pkt, uint8_t frame_id, uint8_t slot_id,
                               uint8_t seq, const void *fsm_ctx,
                               const IMU_t *imu, const BARO_t *baro, const BNO_t *bno,
                               uint8_t last_cmd_seq, uint8_t last_cmd_status);

uint16_t telemetry_build_slow(telemetry_slow_t *pkt, uint8_t frame_id, uint8_t seq,
                               const GPS_t *gps, const BARO_t *baro,
                               uint8_t battery_pct, uint8_t sd_status);

uint16_t telemetry_build_event(telemetry_event_t *pkt, uint8_t frame_id, uint8_t slot_id,
                                uint8_t seq, uint8_t event_type,
                                uint8_t state, uint8_t substate,
                                const void *payload, uint16_t payload_size);

#endif /* TELEMETRY_TELEMETRY_H_ */
