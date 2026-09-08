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
#define TELEM_PACKET_TEST_CTRL  0x06    /**< Manual test control packet from GS to FC (Phase 2 contract) */
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
#define TDMA_SUPERFRAME_MS      1000    /**< Flight superframe duration (milliseconds) */
#define TDMA_SLOT_MS            100     /**< Flight slot duration (milliseconds) */
#define TDMA_SLOTS_PER_FRAME    10      /**< Flight slots per superframe */
#define TDMA_TX_SLOTS           8       /**< Flight: number of FC-TX slots for fast (0-7) */
#define TDMA_SLOW_SLOT          8       /**< Flight: slot for FC slow telemetry */
#define TDMA_RX_SLOT            9       /**< Flight: slot for FC RX (GS commands) */

/* ----------------------------------------------------------------------- */
/*  Phase 3-A (A4): test-interactive TDMA preset                           */
/* ----------------------------------------------------------------------- */
/**
 * @brief Currently active TDMA preset, announced in slow.tdma_mode.
 *
 * - FLIGHT          : legacy 1000 ms / 10×100 ms layout (above)
 * - TEST_INTERACTIVE: 200 ms / 5×40 ms, layout F·R·F·R·S → 10 Hz both ways
 *                     plus 5 Hz slow. Used during STATE_TEST_<KIND>.
 *
 * Mode switches happen at superframe boundaries to avoid mid-frame skew.
 */
typedef enum {
    TDMA_MODE_FLIGHT           = 0,
    TDMA_MODE_TEST_INTERACTIVE = 1,
} tdma_mode_t;

/* TEST_INTERACTIVE preset: 5 slots × 100 ms = 500 ms.
 *   slot 0: FAST (FC tx)
 *   slot 1: RX   (FC listens for test_control_packet_t)
 *   slot 2: FAST (FC tx)
 *   slot 3: RX   (FC listens)
 *   slot 4: SLOW (FC tx, also re-announces tdma_mode)
 */
/* Phase 3-A bugfix: slot widened from 40 ms → 100 ms (superframe 200 → 500 ms).
 * The 40 ms slot was shorter than the LoRa air-time of even our smallest
 * packet (sync ≈ 40–100 ms at SF7/BW125; worse on E22 default config), so
 * every FC TX overran into the next slot and clobbered the GS TX. Net effect
 * was a permanent "Lost sync (no rx for 5000 ms)" cycle the moment SET_PROFILE
 * switched TDMA. 100 ms matches the flight preset which is known-good; command
 * rate drops from 10 Hz → 5 Hz, still fine for manual control. */
#define TDMA_TI_SUPERFRAME_MS   500
#define TDMA_TI_SLOT_MS         100
#define TDMA_TI_SLOTS_PER_FRAME 5
#define TDMA_TI_SLOW_SLOT       4
#define TDMA_TI_RX_SLOT_A       1
#define TDMA_TI_RX_SLOT_B       3
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

    /* Phase 3-A (A4): TDMA mode announce (one of tdma_mode_t).
     * GS reads this to know which superframe layout to follow. */
    uint8_t tdma_mode;          /**< Current FC TDMA preset (tdma_mode_t) */
    uint8_t reserved;           /**< Pad to even length; future use */

    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} telemetry_slow_t;  /* 32 bytes */

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
    /* Phase 3-A (A7): bumped 8 → 32 bytes to fit gutter (26 B) and torque_cal
     * (17 B) profiles in CMD_SET_TEST_PROFILE. Older single-param commands
     * (CMD_STATIC_TEST, CMD_PROFILE_*) still only use the first few bytes. */
    uint8_t params[32];         /**< Command-specific parameters */
    uint16_t crc16;             /**< CRC-16 checksum for packet integrity */
} command_packet_t;  /* 40 bytes */

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

/**
 * @brief Manual test control packet (GS → FC)
 *
 * Sent by the Ground Station every TDMA RX slot during a TEST in Manual
 * mode (slider-driven control). The FC's heartbeat watchdog tracks
 * arrival of these packets — if none are received within
 * `TEST_HEARTBEAT_TIMEOUT_MS`, the test is auto-aborted (motor → 0).
 *
 * The `field_mask` lets the GS send only what changed; unset fields are
 * ignored on the FC side. Encoded values use scaled integers to keep the
 * packet small (no floats on the wire).
 *
 * @note Total size: 17 bytes
 * @see test_runner.h for the on_run_tick callback that consumes these
 */
typedef struct test_control_packet_s {
    uint8_t  packet_type;       /**< TELEM_PACKET_TEST_CTRL */
    uint8_t  frame_id;          /**< GS superframe counter */
    uint8_t  seq;               /**< Per-test sequence counter */
    uint8_t  field_mask;        /**< bit0=throttle bit1=alpha bit2=h_ref bit3=h_ref_dot */
    uint16_t throttle_milli;    /**< 0..1000 (0=off, 1000=full) */
    int16_t  alpha_centideg;    /**< -18000..+18000 (servo angle, centidegrees) */
    int16_t  h_ref_dm;          /**< 0..32767 (target altitude, decimetres) */
    int16_t  h_ref_dot_cms;     /**< Optional rate (cm/s) for slew limiting */
    uint8_t  flags;             /**< bit0=run, bit1=hold, bit2=abort */
    uint16_t crc16;             /**< CRC-16 checksum */
} __attribute__((packed)) test_control_packet_t;  /* 17 bytes */

/** @brief Field mask bits for test_control_packet_t.field_mask */
#define TEST_CTRL_FIELD_THROTTLE    (1u << 0)
#define TEST_CTRL_FIELD_ALPHA       (1u << 1)
#define TEST_CTRL_FIELD_H_REF       (1u << 2)
#define TEST_CTRL_FIELD_H_REF_DOT   (1u << 3)

/** @brief Flag bits for test_control_packet_t.flags */
#define TEST_CTRL_FLAG_RUN          (1u << 0)
#define TEST_CTRL_FLAG_HOLD         (1u << 1)
#define TEST_CTRL_FLAG_ABORT        (1u << 2)

/** @brief How long the FC waits for a fresh test_control_packet_t in
 *  Manual mode before declaring the GS link dead and aborting.
 *  3 s: long enough to ride out one bad superframe + a resync, short enough
 *  that a live motor is never more than 3 s away from an auto-abort after
 *  real link loss. (The old 30000 was a debug value that masked the link
 *  drops instead of fixing them — the drops themselves are addressed in
 *  radio_thread.c / lora_sx126x.c: any valid RX refreshes the link, RX is
 *  polled every loop iteration, and the SX126x TX path can no longer wedge.) */
#define TEST_HEARTBEAT_TIMEOUT_MS   3000

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
/* Phase 3-A (A4): now also takes the active TDMA mode and the slot id to
 * embed (so the test-interactive preset can use slot 4 instead of 8). */
uint16_t telemetry_build_slow(telemetry_slow_t *pkt, uint8_t frame_id, uint8_t seq,
                               const GPS_t *gps, const BARO_t *baro,
                               uint8_t battery_pct, uint8_t sd_status,
                               uint8_t tdma_mode, uint8_t slow_slot);

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
