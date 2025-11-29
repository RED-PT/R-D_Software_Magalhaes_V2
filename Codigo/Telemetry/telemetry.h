/*
 * telemetry.h
 *
 * Telemetry packet definitions for ground station communication
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
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
#define TELEM_PACKET_STATUS     0x05

// ============================================================================
// Fast Telemetry Packet (10 Hz during flight)
// Critical sensor data for real-time monitoring
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_FAST
    uint32_t time;              // Timestamp [ms]
    uint8_t state;              // FSM state
    uint8_t substate;           // FSM substate

    // IMU data (if available)
    int16_t accel_x;            // [mg] scaled
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;             // [0.01 dps] scaled
    int16_t gyro_y;
    int16_t gyro_z;

    // Barometer data
    int32_t pressure;           // [0.01 mbar] scaled
    int16_t baro_temp;          // [0.01 C] scaled
    int16_t altitude;           // [0.1 m] scaled

    // BNO055 orientation
    int16_t heading;            // [0.01 deg] scaled
    int16_t pitch;              // [0.01 deg] scaled
    int16_t roll;               // [0.01 deg] scaled

    uint16_t crc16;             // CRC16 checksum
} telemetry_fast_t;

// ============================================================================
// Slow Telemetry Packet (1 Hz or slower)
// Non-critical data, status info
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_SLOW
    uint32_t time;              // Timestamp [ms]

    // GPS data
    int32_t latitude;           // [1e-7 deg]
    int32_t longitude;          // [1e-7 deg]
    int16_t gps_altitude;       // [0.1 m]
    uint8_t gps_lock;           // Fix type
    uint8_t satellites;         // Number of satellites
    uint16_t hdop;              // [0.01]

    // Temperature readings
    int16_t temp_imu;           // [0.01 C]
    int16_t temp_baro;          // [0.01 C]

    // System status
    uint8_t battery_pct;        // Battery percentage
    uint8_t sd_status;          // SD card status
    uint16_t free_heap;         // FreeRTOS heap [bytes]

    uint16_t crc16;             // CRC16 checksum
} telemetry_slow_t;

// ============================================================================
// Event Packet (sent on state changes, faults, etc.)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_EVENT
    uint32_t time;              // Timestamp [ms]
    uint8_t event_type;         // Event type (EVT_*)
    uint8_t state;              // Current FSM state
    uint8_t substate;           // Current FSM substate

    // Event payload (union for different event types)
    union {
        // State change
        struct {
            uint8_t old_state;
            uint8_t old_sub;
            uint8_t new_state;
            uint8_t new_sub;
            uint8_t reason;
        } state_change;

        // Fault
        struct {
            uint16_t code;
            char desc[32];
        } fault;

        // Generic message
        char msg[40];

        // Raw bytes
        uint8_t raw[40];
    } payload;

    uint16_t crc16;             // CRC16 checksum
} telemetry_event_t;

// ============================================================================
// Command Packet (ground station -> flight computer)
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        // TELEM_PACKET_COMMAND
    uint32_t time;              // Timestamp [ms]
    uint8_t cmd;                // Command type (CMD_*)
    uint8_t ack_status;         // 0=OK, 1=rejected, 2=queued (for responses)
    uint8_t current_state;      // Current FSM state (for responses)

    // Command payload (optional, for commands with parameters)
    union {
        struct {
            uint8_t profile_type;
            float target_alt;
            float flare_alt;
        } profile;

        float target_altitude;

        uint8_t raw[16];
    } payload;

    uint16_t crc16;             // CRC16 checksum
} command_packet_t;

// ============================================================================
// Radio Packet Wrapper (for queue transmission)
// ============================================================================
#define RADIO_MAX_PACKET_SIZE   64

typedef struct {
    uint16_t length;
    uint8_t buffer[RADIO_MAX_PACKET_SIZE];
} radio_packet_t;

// ============================================================================
// Temperature Readings Structure
// ============================================================================
typedef struct {
    float imu_temp_c;
    float baro_temp_c;
} temperature_readings_t;

// ============================================================================
// Telemetry Build Functions
// ============================================================================

// Build fast telemetry packet
// Returns: packet length on success, 0 on failure
uint16_t telemetry_build_fast(telemetry_fast_t *pkt,
                               const void *fsm_ctx,
                               const IMU_t *imu,
                               const BARO_t *baro,
                               const BNO_t *bno);

// Build slow telemetry packet
uint16_t telemetry_build_slow(telemetry_slow_t *pkt,
                               const temperature_readings_t *temps,
                               const GPS_t *gps);

// Build event packet
uint16_t telemetry_build_event(telemetry_event_t *pkt,
                                uint8_t event_type,
                                uint8_t state,
                                uint8_t substate,
                                const void *payload,
                                uint16_t payload_size);

#endif /* TELEMETRY_TELEMETRY_H_ */
