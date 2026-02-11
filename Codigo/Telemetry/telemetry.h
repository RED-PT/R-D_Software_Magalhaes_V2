/**
 * @file telemetry.h
 * @brief TDMA-aware telemetry packet definitions for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This file defines the telemetry packet structures and TDMA configuration
 * for communication between the Flight Computer (FC) and Ground Station (GS).
 *
 * @section tdma_overview TDMA Overview
 * The communication system uses a Time Division Multiple Access (TDMA) scheme:
 * - **Superframe**: 1000ms total duration
 * - **Slots**: 10 slots of 100ms each (0-9)
 * - **Slots 0-7**: Fast telemetry (IMU, barometer, orientation)
 * - **Slot 8**: Slow telemetry (GPS, system status)
 * - **Slot 9**: Reserved for GS commands or sync beacons
 *
 * @section packet_types Packet Types
 * | Type | ID | Direction | Purpose |
 * |------|----|-----------|---------|
 * | FAST | 0x01 | FC→GS | High-rate sensor data (8 per second) |
 * | SLOW | 0x02 | FC→GS | Low-rate GPS/status (1 per second) |
 * | EVENT | 0x03 | FC→GS | State changes, alerts |
 * | COMMAND | 0x04 | GS→FC | Commands to flight computer |
 * | SYNC | 0x05 | GS→FC | Time synchronization beacon |
 *
 * @note All packets include CRC16 for integrity verification
 * @see radio_thread.h for radio transmission handling
 */

#ifndef TELEMETRY_TELEMETRY_H_
#define TELEMETRY_TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "defs.h"
#include "Radio/CRC16/crc16.h"

/**
 * @defgroup TelemetryPacketTypes Packet Type Identifiers
 * @brief Identifiers for different telemetry packet types
 * @{
 */
#define TELEM_PACKET_FAST       0x01    /**< Fast telemetry packet (IMU, baro, orientation) */
#define TELEM_PACKET_SLOW       0x02    /**< Slow telemetry packet (GPS, system status) */
#define TELEM_PACKET_EVENT      0x03    /**< Event notification packet */
#define TELEM_PACKET_COMMAND    0x04    /**< Command packet from GS to FC */
#define TELEM_PACKET_SYNC       0x05    /**< Synchronization beacon from GS */
/** @} */

/**
 * @defgroup TDMAConfig TDMA Configuration
 * @brief Time Division Multiple Access timing parameters
 *
 * The TDMA scheme divides each second into 10 slots:
 * ```
 * |--0--|--1--|--2--|--3--|--4--|--5--|--6--|--7--|--8--|--9--|
 * |FAST |FAST |FAST |FAST |FAST |FAST |FAST |FAST |SLOW | RX  |
 * |100ms|100ms|100ms|100ms|100ms|100ms|100ms|100ms|100ms|100ms|
 * ```
 * @{
 */
#define TDMA_SUPERFRAME_MS      1000    /**< Total superframe duration (milliseconds) */
#define TDMA_SLOT_MS            100     /**< Duration of each slot (milliseconds) */
#define TDMA_SLOTS_PER_FRAME    10      /**< Number of slots per superframe */
#define TDMA_TX_SLOTS           8       /**< Number of TX slots for fast telemetry (0-7) */
#define TDMA_SLOW_SLOT          8       /**< Slot number for slow telemetry */
#define TDMA_RX_SLOT            9       /**< Slot number for receiving GS commands */
/** @} */

/**
 * @defgroup TelemetryPackets Telemetry Packet Structures
 * @brief Data structures for telemetry packets
 * @{
 */

/**
 * @brief Fast telemetry packet structure (slots 0-7)
 *
 * Sent 8 times per second, contains high-rate sensor data:
 * - IMU readings (accelerometer, gyroscope)
 * - Barometric altitude and vertical velocity
 * - Orientation (pitch, roll, yaw)
 * - FSM state information
 * - Command acknowledgment (piggybacked)
 *
 * @note Total size: 37 bytes
 * @note Scaling factors applied to reduce bandwidth:
 *       - Acceleration: 1 LSB = 1 mg
 *       - Gyroscope: 1 LSB = 0.01 dps
 *       - Altitude: 1 LSB = 0.1 m
 *       - Angles: 1 LSB = 0.1 degrees
 */
typedef struct __attribute__((packed)) {
    /* Packet header */
    uint8_t packet_type;        /**< Packet type identifier (TELEM_PACKET_FAST) */
    uint8_t frame_id;           /**< Superframe counter (0-255, wraps) */
    uint8_t slot_id;            /**< Current TDMA slot (0-9) */
    uint8_t seq;                /**< Packet sequence number for loss detection */
    uint8_t flags;              /**< Status flags (see @ref TelemetryFlags) */
    uint32_t time;              /**< System timestamp (milliseconds since boot) */

    /* FSM state */
    uint8_t state;              /**< Current FSM state (see fsm_state_t) */
    uint8_t substate;           /**< Current FSM substate (see fsm_substate_t) */

    /* Command acknowledgment (piggybacked on fast telemetry) */
    uint8_t last_cmd_seq_acked; /**< Sequence number of last acknowledged command */
    uint8_t last_cmd_status;    /**< Command status: 0=OK, 1=rejected, 2=error */

    /* IMU data (scaled integers for bandwidth efficiency) */
    int16_t accel_x;            /**< X-axis acceleration (milli-g) */
    int16_t accel_y;            /**< Y-axis acceleration (milli-g) */
    int16_t accel_z;            /**< Z-axis acceleration (milli-g) */
    int16_t gyro_x;             /**< X-axis angular rate (0.01 dps) */
    int16_t gyro_y;             /**< Y-axis angular rate (0.01 dps) */
    int16_t gyro_z;             /**< Z-axis angular rate (0.01 dps) */

    /* Barometer */
    int16_t altitude;           /**< Altitude above reference (0.1 m) */
    int16_t vario;              /**< Vertical velocity (0.01 m/s) */

    /* Orientation from BNO055 sensor fusion */
    int16_t pitch;              /**< Pitch angle (0.1 degrees, -900 to +900) */
    int16_t roll;               /**< Roll angle (0.1 degrees, -1800 to +1800) */
    int16_t yaw;                /**< Yaw/heading angle (0.1 degrees, 0 to 3600) */

    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} telemetry_fast_t;  /* 37 bytes */

/**
 * @brief Slow telemetry packet structure (slot 8)
 *
 * Sent once per second, contains low-rate data:
 * - GPS position and status
 * - System health (battery, SD card, memory)
 * - Environmental data (pressure, temperature)
 *
 * @note Total size: 30 bytes
 * @note GPS coordinates use 1e-7 degree scaling for 1.1cm precision
 */
typedef struct __attribute__((packed)) {
    /* Packet header */
    uint8_t packet_type;        /**< Packet type identifier (TELEM_PACKET_SLOW) */
    uint8_t frame_id;           /**< Superframe counter (should match fast packets) */
    uint8_t slot_id;            /**< TDMA slot (should be 8) */
    uint8_t seq;                /**< Packet sequence number */
    uint32_t time;              /**< System timestamp (milliseconds since boot) */

    /* GPS data */
    int32_t latitude;           /**< Latitude (1e-7 degrees, ~1.1cm resolution) */
    int32_t longitude;          /**< Longitude (1e-7 degrees, ~1.1cm resolution) */
    int16_t gps_altitude;       /**< GPS altitude MSL (0.1 m) */
    uint8_t gps_lock;           /**< GPS fix quality: 0=none, 1=GPS, 2=DGPS */
    uint8_t satellites;         /**< Number of satellites in use */

    /* System status */
    int16_t pressure;           /**< Pressure offset from 1000 mbar (0.1 mbar) */
    int16_t temp_baro;          /**< Barometer temperature (0.1 C) */
    uint8_t battery_pct;        /**< Battery charge level (0-100%) */
    uint8_t sd_status;          /**< SD card status: 0=OK, non-zero=error code */
    uint16_t free_heap;         /**< Free RTOS heap memory (bytes) */

    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} telemetry_slow_t;  /* 30 bytes */

/**
 * @brief Event telemetry packet structure
 *
 * Sent when significant events occur (state changes, alerts, errors).
 * Can be sent in any TX slot, replacing the normal fast packet.
 *
 * @note Total size: 37 bytes (same as fast packet for slot compatibility)
 * @note Payload is event-type specific, see telemetry_event_type_t
 */
typedef struct __attribute__((packed)) {
    /* Packet header */
    uint8_t packet_type;        /**< Packet type identifier (TELEM_PACKET_EVENT) */
    uint8_t frame_id;           /**< Superframe counter when event occurred */
    uint8_t slot_id;            /**< TDMA slot used for transmission */
    uint8_t seq;                /**< Packet sequence number */
    uint32_t time;              /**< System timestamp when event occurred */

    /* Event information */
    uint8_t event_type;         /**< Event type (see telemetry_event_type_t) */
    uint8_t state;              /**< FSM state when event occurred */
    uint8_t substate;           /**< FSM substate when event occurred */
    uint8_t payload[24];        /**< Event-specific payload data */

    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} telemetry_event_t;  /* 37 bytes */

/**
 * @brief Command packet structure (GS to FC, slot 9)
 *
 * Used by Ground Station to send commands to Flight Computer.
 * Commands are acknowledged via the fast telemetry packet.
 *
 * @note Total size: 16 bytes
 * @see fsm_cmd_id_t for available command types
 */
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        /**< Packet type identifier (TELEM_PACKET_COMMAND) */
    uint8_t frame_id;           /**< GS superframe counter */
    uint8_t cmd_id;             /**< Command identifier (see fsm_cmd_id_t) */
    uint8_t cmd_seq;            /**< Command sequence number for ACK matching */
    uint32_t time;              /**< GS timestamp when command was sent */
    uint8_t params[8];          /**< Command-specific parameters */
    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} command_packet_t;  /* 16 bytes */

/**
 * @brief Sync beacon packet structure (GS to FC, slot 9)
 *
 * Sent by Ground Station when no commands are pending.
 * Used for TDMA time synchronization between FC and GS.
 *
 * @note Total size: 10 bytes
 */
typedef struct __attribute__((packed)) {
    uint8_t packet_type;        /**< Packet type identifier (TELEM_PACKET_SYNC) */
    uint8_t frame_id;           /**< GS superframe counter */
    uint8_t slot_id;            /**< TDMA slot (should be 9) */
    uint8_t reserved;           /**< Reserved for future use */
    uint32_t gs_time;           /**< GS timestamp for synchronization */
    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} sync_packet_t;  /* 10 bytes */

/** @} */ /* End of TelemetryPackets group */

/**
 * @defgroup TelemetryFlags Telemetry Status Flags
 * @brief Bit flags for the telemetry_fast_t.flags field
 *
 * Multiple flags can be combined using bitwise OR.
 * Example: `flags = TELEM_FLAG_GPS_LOCK | TELEM_FLAG_ARMED;`
 * @{
 */
#define TELEM_FLAG_EVENT_PENDING    (1 << 0)    /**< Event packet queued for transmission */
#define TELEM_FLAG_LOW_BATTERY      (1 << 1)    /**< Battery level below threshold */
#define TELEM_FLAG_SD_ERROR         (1 << 2)    /**< SD card error detected */
#define TELEM_FLAG_GPS_LOCK         (1 << 3)    /**< GPS has valid fix */
#define TELEM_FLAG_ARMED            (1 << 4)    /**< System is armed for launch */
#define TELEM_FLAG_FLIGHT           (1 << 5)    /**< Currently in flight mode */
/** @} */

/**
 * @defgroup RadioPacket Radio Packet Wrapper
 * @brief Generic radio packet buffer for transmission/reception
 * @{
 */

/** Maximum size of any radio packet in bytes */
#define RADIO_MAX_PACKET_SIZE   48

/**
 * @brief Generic radio packet container
 *
 * Wraps any telemetry packet type for radio transmission.
 * Used by the radio thread for TX/RX operations.
 */
typedef struct {
    uint16_t length;                        /**< Actual packet length in bytes */
    uint8_t buffer[RADIO_MAX_PACKET_SIZE];  /**< Packet data buffer */
} radio_packet_t;

/** @} */ /* End of RadioPacket group */

/**
 * @defgroup TemperatureData Temperature Readings
 * @brief Aggregated temperature data from multiple sensors
 * @{
 */

/**
 * @brief Temperature readings from onboard sensors
 *
 * Combines temperature data from IMU and barometer sensors
 * for thermal monitoring and compensation.
 */
typedef struct {
    float imu_temp_c;           /**< IMU sensor temperature (Celsius) */
    float baro_temp_c;          /**< Barometer sensor temperature (Celsius) */
} temperature_readings_t;

/** @} */ /* End of TemperatureData group */

/**
 * @defgroup TelemetryBuild Telemetry Build Functions
 * @brief Functions to construct telemetry packets
 * @{
 */

/**
 * @brief Build a fast telemetry packet
 *
 * Populates a fast telemetry packet with current sensor data and FSM state.
 * Applies scaling factors to convert floating-point values to integers.
 * Calculates and appends CRC16 checksum.
 *
 * @param[out] pkt          Pointer to packet structure to populate
 * @param[in]  frame_id     Current superframe counter (0-255)
 * @param[in]  slot_id      Current TDMA slot (0-7 for fast packets)
 * @param[in]  seq          Packet sequence number
 * @param[in]  fsm_ctx      Pointer to FSM context (cast to void* for decoupling)
 * @param[in]  imu          Pointer to current IMU data
 * @param[in]  baro         Pointer to current barometer data
 * @param[in]  bno          Pointer to current BNO055 orientation data
 * @param[in]  last_cmd_seq Sequence number of last received command
 * @param[in]  last_cmd_status Status of last command (0=OK, 1=rejected, 2=error)
 *
 * @return Size of the built packet in bytes (37)
 *
 * @note Scaling applied:
 *       - Acceleration: g → mg (multiply by 1000)
 *       - Gyroscope: dps → 0.01 dps (multiply by 100)
 *       - Altitude: m → 0.1 m (multiply by 10)
 *       - Angles: deg → 0.1 deg (multiply by 10)
 */
uint16_t telemetry_build_fast(telemetry_fast_t *pkt, uint8_t frame_id, uint8_t slot_id,
                               uint8_t seq, const void *fsm_ctx,
                               const IMU_t *imu, const BARO_t *baro, const BNO_t *bno,
                               uint8_t last_cmd_seq, uint8_t last_cmd_status);

/**
 * @brief Build a slow telemetry packet
 *
 * Populates a slow telemetry packet with GPS and system status data.
 * Sent once per superframe in slot 8.
 *
 * @param[out] pkt          Pointer to packet structure to populate
 * @param[in]  frame_id     Current superframe counter (0-255)
 * @param[in]  seq          Packet sequence number
 * @param[in]  gps          Pointer to current GPS data
 * @param[in]  baro         Pointer to current barometer data
 * @param[in]  battery_pct  Current battery percentage (0-100)
 * @param[in]  sd_status    SD card status code (0=OK)
 *
 * @return Size of the built packet in bytes (30)
 *
 * @note GPS coordinates scaled: degrees × 1e7 for integer representation
 */
uint16_t telemetry_build_slow(telemetry_slow_t *pkt, uint8_t frame_id, uint8_t seq,
                               const GPS_t *gps, const BARO_t *baro,
                               uint8_t battery_pct, uint8_t sd_status);

/**
 * @brief Build an event telemetry packet
 *
 * Populates an event packet for state changes or alerts.
 * Event packets can be sent in any TX slot, replacing the normal fast packet.
 *
 * @param[out] pkt          Pointer to packet structure to populate
 * @param[in]  frame_id     Current superframe counter (0-255)
 * @param[in]  slot_id      TDMA slot used for transmission
 * @param[in]  seq          Packet sequence number
 * @param[in]  event_type   Event type identifier (see telemetry_event_type_t)
 * @param[in]  state        Current FSM state when event occurred
 * @param[in]  substate     Current FSM substate when event occurred
 * @param[in]  payload      Pointer to event-specific payload data (can be NULL)
 * @param[in]  payload_size Size of payload in bytes (max 24)
 *
 * @return Size of the built packet in bytes (37)
 *
 * @warning payload_size must not exceed 24 bytes
 */
uint16_t telemetry_build_event(telemetry_event_t *pkt, uint8_t frame_id, uint8_t slot_id,
                                uint8_t seq, uint8_t event_type,
                                uint8_t state, uint8_t substate,
                                const void *payload, uint16_t payload_size);

/** @} */ /* End of TelemetryBuild group */

#endif /* TELEMETRY_TELEMETRY_H_ */
