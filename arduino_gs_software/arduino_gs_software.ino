/**
 * @file arduino_gs_fixed_2.ino
 * @brief Ground Station firmware for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This Arduino sketch implements the Ground Station (GS) that communicates
 * with the Magalhaes Flight Computer via E22 LoRa radio and relays data
 * to a PC dashboard via USB Serial.
 *
 * @section gs_overview Overview
 * The Ground Station performs:
 * - Receive telemetry from Flight Computer (slots 0-8)
 * - Forward telemetry to PC dashboard via framed serial protocol
 * - Send commands from PC to Flight Computer (slot 9)
 * - Maintain TDMA synchronization with FC
 * - Command acknowledgment tracking
 *
 * @section gs_hardware Hardware
 * - Arduino Nano/Uno (ATmega328P)
 * - E22 LoRa module on SoftwareSerial (pins 2,3)
 * - E22 mode control (M0=pin 4, M1=pin 5)
 * - USB Serial to PC at 115200 baud
 *
 * @section gs_protocol PC Communication Protocol
 * All messages to PC use framed format:
 * | Sync1 | Sync2 | LenH | LenL | Type | Payload... | CRCH | CRCL |
 * |-------|-------|------|------|------|------------|------|------|
 * | 0xAA  | 0x55  |      |      |      |            |      |      |
 *
 * @section gs_commands PC Commands (single character)
 * | Char | Command | Description |
 * |------|---------|-------------|
 * | P | PING | Test connectivity |
 * | C | CALIBRATE | Calibrate barometer |
 * | M | MOTOR_CAL | ESC calibration mode |
 * | E | STATIC_TEST | Static thrust test (E20 = 20% max) |
 * | 1/2/3 | PROFILE | Select flight profile |
 * | A | ARM | Arm for launch |
 * | D | DISARM | Disarm |
 * | T | TEST | Start test sequence |
 * | L | LAUNCH | Launch command |
 * | X | ABORT | Abort mission |
 * | S | SAFE | Force safe mode |
 * | R | RESET_STATS | Reset statistics |
 * | ? | DEBUG | Toggle debug output |
 *
 * @section gs_tdma TDMA Structure
 * Synchronized to Flight Computer timing:
 * - Slots 0-7: Receive fast telemetry
 * - Slot 8: Receive slow telemetry
 * - Slot 9: Transmit commands or sync beacon
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

/**
 * @defgroup GSConfig Configuration Options
 * @brief Compile-time configuration
 * @{
 */

/** @brief Enable simulation mode for testing without FC */
#define SIMULATION_MODE false

/** @brief Enable debug output to serial */
#define DEBUG_MODE true

/** @} */

/**
 * @defgroup E22Pins E22 LoRa Module Pins
 * @brief Hardware pin definitions for E22 module
 * @{
 */
#define E22_RX 2    /**< E22 TX -> Arduino RX (SoftwareSerial) */
#define E22_TX 3    /**< E22 RX <- Arduino TX (SoftwareSerial) */
#define E22_M0 4    /**< E22 M0 mode pin */
#define E22_M1 5    /**< E22 M1 mode pin */
/** @} */

/** @brief SoftwareSerial instance for E22 radio communication */
SoftwareSerial RADIO_SERIAL(E22_RX, E22_TX);

/** @brief Hardware serial for PC communication */
#define PC_SERIAL Serial

/** @brief PC serial baud rate */
#define PC_BAUD 115200

/**
 * @defgroup FrameSync Frame Synchronization
 * @brief Sync bytes for framed messages to PC
 * @{
 */
#define FRAME_SYNC_1        0xAA    /**< First sync byte */
#define FRAME_SYNC_2        0x55    /**< Second sync byte */
/** @} */

/**
 * @defgroup MsgTypes Message Types to PC
 * @brief Message type identifiers for PC protocol
 * @{
 */
#define MSG_TYPE_FC_FAST    0x01    /**< Fast telemetry from FC */
#define MSG_TYPE_FC_SLOW    0x02    /**< Slow telemetry from FC */
#define MSG_TYPE_FC_EVENT   0x03    /**< Event from FC */
#define MSG_TYPE_GS_STATUS  0x10    /**< GS status update */
#define MSG_TYPE_GS_ACK     0x11    /**< Command acknowledgment */
#define MSG_TYPE_GS_PONG    0x12    /**< Ping response */
#define MSG_TYPE_GS_STATS   0x13    /**< GS statistics */
#define MSG_TYPE_GS_LOG     0x20    /**< Log message text */
/** @} */

/**
 * @defgroup PCCommands PC Command Characters
 * @brief Single-character commands from PC
 * @{
 */
#define PC_CMD_PING         'P'     /**< Ping FC */
#define PC_CMD_CALIBRATE    'C'     /**< Calibrate barometer */
#define PC_CMD_MOTOR_CAL    'M'     /**< ESC calibration */
#define PC_CMD_STATIC_TEST  'E'     /**< Static thrust test */
#define PC_CMD_ARM          'A'     /**< Arm for launch */
#define PC_CMD_DISARM       'D'     /**< Disarm */
#define PC_CMD_TEST         'T'     /**< Start test */
#define PC_CMD_LAUNCH       'L'     /**< Launch */
#define PC_CMD_ABORT        'X'     /**< Abort */
#define PC_CMD_SAFE         'S'     /**< Force safe mode */
#define PC_CMD_RESET_STATS  'R'     /**< Reset statistics */
#define PC_CMD_DEBUG        '?'     /**< Toggle debug */
/** @} */

/**
 * @defgroup TDMAConfig TDMA Configuration
 * @brief Time slot configuration matching FC
 * @{
 */
/* Flight preset (default at boot) */
#define TDMA_FLIGHT_SUPERFRAME_MS   1000
#define TDMA_FLIGHT_SLOT_MS         100
#define TDMA_FLIGHT_SLOTS           10
#define TDMA_FLIGHT_TX_SLOT         9
/* Phase 3-A (A4): test-interactive preset (200 ms / 5×40 ms — see telemetry.h) */
/* Phase 3-A bugfix: must match FC's telemetry.h (TDMA_TI_*). Widened from
 * 40 ms → 100 ms because LoRa air time of a single sync/cmd exceeded the
 * old 40 ms slot, causing permanent inter-slot collision after the FC
 * switched to test-interactive on SET_PROFILE. */
#define TDMA_TEST_SUPERFRAME_MS     500
#define TDMA_TEST_SLOT_MS           100
#define TDMA_TEST_SLOTS             5
#define TDMA_TEST_TX_SLOT_A         1
#define TDMA_TEST_TX_SLOT_B         3

/* Active TDMA preset state — flipped via slow.tdma_mode announcement */
enum TdmaMode { TDMA_MODE_FLIGHT = 0, TDMA_MODE_TEST_INTERACTIVE = 1 };
uint8_t  tdma_mode_current = TDMA_MODE_FLIGHT;
uint8_t  tdma_mode_pending = TDMA_MODE_FLIGHT;
uint16_t tdma_superframe_ms = TDMA_FLIGHT_SUPERFRAME_MS;
uint8_t  tdma_slot_ms       = TDMA_FLIGHT_SLOT_MS;
uint8_t  tdma_slots         = TDMA_FLIGHT_SLOTS;

/* Convenience: legacy aliases used by older code. Re-evaluated each call. */
#define TDMA_SUPERFRAME_MS      tdma_superframe_ms
#define TDMA_SLOT_MS            tdma_slot_ms
#define TDMA_SLOTS_PER_FRAME    tdma_slots

/* TX windows scaled per slot (5%..80% of the active slot duration). */
#define TX_START_OFFSET_MS      ((tdma_slot_ms * 5) / 100)
#define TX_END_OFFSET_MS        ((tdma_slot_ms * 80) / 100)

/** @brief True iff this slot is a GS-TX slot under the active preset. */
inline bool tdma_slot_is_tx(uint8_t slot) {
    if (tdma_mode_current == TDMA_MODE_TEST_INTERACTIVE)
        return (slot == TDMA_TEST_TX_SLOT_A) || (slot == TDMA_TEST_TX_SLOT_B);
    return slot == TDMA_FLIGHT_TX_SLOT;
}
/** @} */

/**
 * @defgroup FCPacketTypes FC Packet Type IDs
 * @brief Packet type identifiers from FC (must match telemetry.h)
 * @{
 */
#define TELEM_PACKET_FAST       0x01    /**< Fast telemetry */
#define TELEM_PACKET_SLOW       0x02    /**< Slow telemetry */
#define TELEM_PACKET_EVENT      0x03    /**< Event packet */
#define TELEM_PACKET_COMMAND    0x04    /**< Command packet */
#define TELEM_PACKET_SYNC       0x05    /**< Sync beacon */
/** @} */

/**
 * @defgroup PacketSizes Packet Sizes
 * @brief Expected packet sizes (must match STM32 structs)
 * @{
 */
#define FAST_PKT_SIZE   37      /**< telemetry_fast_t size */
#define SLOW_PKT_SIZE   32      /**< telemetry_slow_t size (Phase 3-A: +tdma_mode +reserved) */
#define EVENT_PKT_SIZE  37      /**< telemetry_event_t size */
/** @} */

/**
 * @brief Command IDs for FC (matches fsm_cmd_id_t)
 */
enum FCCommand {
    CMD_NONE = 0,           /**< No command */
    CMD_PING,               /**< Ping/heartbeat */
    CMD_SET_PROFILE,        /**< Set flight profile */
    CMD_SET_PARAM,          /**< Set parameter */
    CMD_ARM,                /**< Arm for launch */
    CMD_DISARM,             /**< Disarm */
    CMD_START_TEST,         /**< Start test mode */
    CMD_LAUNCH,             /**< Launch command */
    CMD_ABORT,              /**< Abort mission */
    CMD_FORCE_SAFE,         /**< Force safe state */
    CMD_CALIBRATE_BARO,     /**< Calibrate barometer */
    CMD_CALIBRATE_MOTOR,    /**< ESC calibration */
    CMD_STATIC_TEST,        /**< Static thrust test (legacy fast-path) */
    CMD_RESYNC,             /**< Resync TDMA */
    /* Phase 3-A (A3): new test FSM commands. */
    CMD_SET_TEST_PROFILE,   /**< Load profile_t into test runner (params[0]=kind) */
    CMD_HOLD,               /**< Manual: pause running test */
    CMD_RESUME,             /**< Manual: resume from hold */
    CMD_STOP_TEST           /**< Graceful end (running → finishing → done) */
};

/**
 * @defgroup PacketStructs Packet Structures
 * @brief Data structures for telemetry packets (packed for wire format)
 * @{
 */
#pragma pack(push, 1)

/**
 * @brief Fast telemetry packet structure (37 bytes)
 * @see telemetry_fast_t in telemetry.h
 */
typedef struct {
    uint8_t packet_type;        /**< TELEM_PACKET_FAST */
    uint8_t frame_id;           /**< Superframe counter */
    uint8_t slot_id;            /**< TDMA slot */
    uint8_t seq;                /**< Sequence number */
    uint8_t flags;              /**< Status flags */
    uint32_t time;              /**< FC timestamp (ms) */
    uint8_t state;              /**< FSM state */
    uint8_t substate;           /**< FSM substate */
    uint8_t last_cmd_seq_acked; /**< ACK'd command seq */
    uint8_t last_cmd_status;    /**< ACK'd command status */
    int16_t accel_x;            /**< Acceleration X (mg) */
    int16_t accel_y;            /**< Acceleration Y (mg) */
    int16_t accel_z;            /**< Acceleration Z (mg) */
    int16_t gyro_x;             /**< Gyro X (0.01 dps) */
    int16_t gyro_y;             /**< Gyro Y (0.01 dps) */
    int16_t gyro_z;             /**< Gyro Z (0.01 dps) */
    int16_t altitude;           /**< Altitude (0.1 m) */
    int16_t vario;              /**< Vertical velocity (0.01 m/s) */
    int16_t pitch;              /**< Pitch (0.1 deg) */
    int16_t roll;               /**< Roll (0.1 deg) */
    int16_t yaw;                /**< Yaw (0.1 deg) */
    uint16_t crc16;             /**< CRC-16 checksum */
} TelemetryFast_t;

/**
 * @brief Slow telemetry packet structure (30 bytes)
 * @see telemetry_slow_t in telemetry.h
 */
typedef struct {
    uint8_t packet_type;        /**< TELEM_PACKET_SLOW */
    uint8_t frame_id;           /**< Superframe counter */
    uint8_t slot_id;            /**< TDMA slot (8) */
    uint8_t seq;                /**< Sequence number */
    uint32_t time;              /**< FC timestamp (ms) */
    int32_t latitude;           /**< Latitude (1e-7 deg) */
    int32_t longitude;          /**< Longitude (1e-7 deg) */
    int16_t gps_altitude;       /**< GPS altitude (0.1 m) */
    uint8_t gps_lock;           /**< GPS fix quality */
    uint8_t satellites;         /**< Satellites in use */
    int16_t pressure;           /**< Pressure offset (0.1 mbar) */
    int16_t temp_baro;          /**< Temperature (0.1 C) */
    uint8_t battery_pct;        /**< Battery percentage */
    uint8_t sd_status;          /**< SD card status */
    uint16_t free_heap;         /**< Free heap (bytes) */
    uint8_t tdma_mode;          /**< Phase 3-A: active TDMA preset (TdmaMode) */
    uint8_t reserved;           /**< Reserved (pad to even length) */
    uint16_t crc16;             /**< CRC-16 checksum */
} TelemetrySlow_t;

/**
 * @brief Command packet structure (16 bytes)
 * @see command_packet_t in telemetry.h
 */
typedef struct {
    uint8_t packet_type;        /**< TELEM_PACKET_COMMAND */
    uint8_t frame_id;           /**< GS superframe counter */
    uint8_t cmd_id;             /**< Command ID */
    uint8_t cmd_seq;            /**< Command sequence for ACK */
    uint32_t time;              /**< GS timestamp (ms) */
    /* Phase 3-A (A7): bumped 8 → 32 bytes to fit gutter / torque_cal profiles. */
    uint8_t params[32];         /**< Command parameters */
    uint16_t crc16;             /**< CRC-16 checksum */
} CommandPacket_t;

/**
 * @brief Sync beacon packet structure (10 bytes)
 * @see sync_packet_t in telemetry.h
 */
typedef struct {
    uint8_t packet_type;        /**< TELEM_PACKET_SYNC */
    uint8_t frame_id;           /**< GS superframe counter */
    uint8_t slot_id;            /**< TDMA slot (9) */
    uint8_t reserved;           /**< Reserved */
    uint32_t gs_time;           /**< GS timestamp for sync */
    uint16_t crc16;             /**< CRC-16 checksum */
} SyncPacket_t;

/**
 * @brief GS status structure for PC
 */
typedef struct {
    uint8_t tdma_synced;        /**< TDMA sync status */
    uint8_t frame_id;           /**< Current frame ID */
    uint8_t slot_id;            /**< Current slot ID */
    uint8_t cmd_pending;        /**< Command pending flag */
    uint8_t awaiting_ack;       /**< Awaiting ACK flag */
    uint8_t last_ack_seq;       /**< Last ACK'd sequence */
    uint8_t last_ack_status;    /**< Last ACK status */
    uint32_t uptime_ms;         /**< GS uptime (ms) */
} GSStatus_t;

/**
 * @brief Ping response structure
 */
typedef struct {
    uint8_t ping_seq;           /**< Ping sequence number */
    uint32_t rtt_ms;            /**< Round-trip time (ms) */
    uint32_t fc_timestamp;      /**< FC timestamp from pong */
} GSPong_t;

/**
 * @brief GS statistics structure
 */
typedef struct {
    uint32_t rx_fast;           /**< Fast packets received */
    uint32_t rx_slow;           /**< Slow packets received */
    uint32_t rx_event;          /**< Event packets received */
    uint32_t tx_cmd;            /**< Commands transmitted */
    uint32_t tx_sync;           /**< Sync beacons transmitted */
    uint32_t crc_errors;        /**< CRC errors detected */
    uint32_t ack_ok;            /**< Successful ACKs */
    uint32_t ack_timeout;       /**< ACK timeouts */
    uint32_t cmd_retries;       /**< Command retry attempts */
} GSStats_t;

#pragma pack(pop)
/** @} */

/**
 * @defgroup GlobalState Global State Variables
 * @brief Runtime state for TDMA and command handling
 * @{
 */

/** @brief Current superframe counter */
uint8_t frame_id = 0;

/** @brief Timestamp when current frame started */
uint32_t frame_start_ms = 0;

/** @brief Flag: TX completed this slot */
bool tx_done_this_slot = false;

/** @brief Flag: TDMA synchronized with FC */
bool tdma_synced = false;

/** @brief Pending command to transmit */
CommandPacket_t pending_cmd = {0};

/** @brief Flag: command waiting to send */
bool cmd_pending = false;

/* Phase 3-A bugfix: small FIFO so back-to-back clicks (e.g. SET → ARM in
 * flight mode where TX cadence is 1 Hz) are preserved in order instead of
 * the second one clobbering the first. */
#define CMD_QUEUE_DEPTH 6
CommandPacket_t cmd_queue[CMD_QUEUE_DEPTH];
uint8_t cmd_queue_head = 0;
uint8_t cmd_queue_count = 0;

/* Phase 3-A (A5): test_control_packet_t buffer received from PC and queued
 * for the next TX slot in test-interactive mode. 15 bytes wire format:
 *   type(1) frame_id(1) seq(1) field_mask(1) throttle_milli(2)
 *   alpha_centideg(2) h_ref_dm(2) h_ref_dot_cms(2) flags(1) crc16(2). */
#define TEST_CTRL_PKT_SIZE 15
uint8_t pending_test_ctrl[TEST_CTRL_PKT_SIZE] = {0};
bool    test_ctrl_pending = false;

/** @brief Command sequence counter */
uint8_t cmd_seq = 0;

/** @brief Sequence number awaiting ACK */
uint8_t awaiting_ack_seq = 0;

/** @brief Flag: waiting for command ACK */
bool awaiting_ack = false;

/** @brief Timestamp when command was sent */
uint32_t cmd_sent_time = 0;

/** @brief Command retry counter */
uint8_t cmd_retry_count = 0;

/** @brief Command timeout (ms) */
#define CMD_TIMEOUT_MS      3000

/** @brief Maximum command retries */
#define CMD_MAX_RETRIES     5

/** @brief Ping sequence sent */
uint8_t ping_seq_sent = 0;

/** @brief Timestamp when ping was sent */
uint32_t ping_sent_time = 0;

/** @brief Flag: awaiting pong response */
bool ping_awaiting_pong = false;

/** @brief Communication statistics */
GSStats_t stats = {0};

/** @brief RX buffer size */
#define RX_BUFFER_SIZE 64

/** @brief Receive buffer */
uint8_t rx_buffer[RX_BUFFER_SIZE];

/** @brief Current RX buffer index */
uint16_t rx_idx = 0;

/** @brief Timestamp of last received byte */
uint32_t rx_last_byte_ms = 0;

/** @brief RX timeout (ms) */
#define RX_TIMEOUT_MS 50

/** @brief Timestamp of last status send */
uint32_t last_status_ms = 0;

/** @brief Status update interval (ms) */
#define STATUS_INTERVAL_MS 500

/** @brief Debug output enabled */
bool debug_enabled = DEBUG_MODE;

/* Simulation state variables */
uint8_t sim_state = 1;
uint8_t sim_substate = 0;
uint8_t sim_seq = 0;
float sim_altitude = 0;
float sim_pitch = 0;
float sim_roll = 0;
uint32_t last_sim_fast_ms = 0;
uint32_t last_sim_slow_ms = 0;
#define SIM_FAST_INTERVAL_MS 100
#define SIM_SLOW_INTERVAL_MS 1000

/** @} */

/**
 * @defgroup CRC CRC-16 Calculation
 * @{
 */

/**
 * @brief Calculate CRC-16 (Modbus polynomial)
 *
 * Same algorithm as STM32 for compatibility.
 *
 * @param[in] data   Pointer to data buffer
 * @param[in] length Number of bytes
 *
 * @return CRC-16 value
 */
uint16_t crc16_calc(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/** @} */

/**
 * @defgroup PCComm PC Communication Functions
 * @{
 */

/**
 * @brief Send framed message to PC
 *
 * Wraps payload in framed format with sync bytes and CRC.
 *
 * @param[in] msgType Message type identifier
 * @param[in] data    Pointer to payload data
 * @param[in] len     Payload length
 */
void sendToPC(uint8_t msgType, const void* data, uint16_t len) {
    uint16_t totalLen = len + 1;

    uint8_t crc_buf[64];
    crc_buf[0] = msgType;
    if (len > 0 && data != NULL && len < 63) {
        memcpy(&crc_buf[1], data, len);
    }
    uint16_t crc = crc16_calc(crc_buf, totalLen);

    PC_SERIAL.write(FRAME_SYNC_1);
    PC_SERIAL.write(FRAME_SYNC_2);
    PC_SERIAL.write((uint8_t)(totalLen >> 8));
    PC_SERIAL.write((uint8_t)(totalLen & 0xFF));
    PC_SERIAL.write(msgType);
    if (len > 0 && data != NULL) {
        PC_SERIAL.write((const uint8_t*)data, len);
    }
    PC_SERIAL.write((uint8_t)(crc >> 8));
    PC_SERIAL.write((uint8_t)(crc & 0xFF));
}

/**
 * @brief Send log message to PC
 *
 * @param[in] msg Null-terminated string
 */
void sendLog(const char* msg) {
    sendToPC(MSG_TYPE_GS_LOG, msg, strlen(msg));
}

/**
 * @brief Send log message from flash memory
 *
 * @param[in] msg Flash string (F() macro)
 */
void sendLogF(const __FlashStringHelper* msg) {
    char buf[64];
    strncpy_P(buf, (const char*)msg, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;
    sendLog(buf);
}

/** @} */

/**
 * @defgroup Debug Debug Output Functions
 * @{
 */

/**
 * @brief Print debug message
 *
 * @param[in] msg Message string
 */
void debugPrint(const char* msg) {
    if (debug_enabled) {
        PC_SERIAL.print(F("[DBG] "));
        PC_SERIAL.println(msg);
    }
}

/**
 * @brief Print hex dump of data
 *
 * @param[in] data Pointer to data
 * @param[in] len  Number of bytes
 */
void debugHex(const uint8_t* data, uint16_t len) {
    if (!debug_enabled) return;
    PC_SERIAL.print(F("[HEX] "));
    for (uint16_t i = 0; i < len && i < 20; i++) {
        if (data[i] < 0x10) PC_SERIAL.print('0');
        PC_SERIAL.print(data[i], HEX);
        PC_SERIAL.print(' ');
    }
    if (len > 20) PC_SERIAL.print(F("..."));
    PC_SERIAL.println();
}

/** @} */

/**
 * @defgroup TDMA TDMA Timing Functions
 * @{
 */

/**
 * @brief Get current TDMA slot number
 *
 * @return Slot number (0-9)
 */
uint8_t get_current_slot() {
    uint32_t elapsed = millis() - frame_start_ms;
    return (elapsed / TDMA_SLOT_MS) % TDMA_SLOTS_PER_FRAME;
}

/**
 * @brief Get time elapsed in current slot
 *
 * @return Milliseconds since slot start
 */
uint32_t get_time_in_slot() {
    uint32_t elapsed = millis() - frame_start_ms;
    return elapsed % TDMA_SLOT_MS;
}

/**
 * @brief Advance to next superframe.
 *
 * Phase 3-A (A4): also applies any pending TDMA preset switch announced by
 * the FC in slow telemetry. Switches happen at superframe boundaries so we
 * don't reshape a slot mid-frame.
 */
void advance_frame() {
    frame_id++;
    frame_start_ms += TDMA_SUPERFRAME_MS;

    if (tdma_mode_pending != tdma_mode_current) {
        tdma_mode_current = tdma_mode_pending;
        if (tdma_mode_current == TDMA_MODE_TEST_INTERACTIVE) {
            tdma_superframe_ms = TDMA_TEST_SUPERFRAME_MS;
            tdma_slot_ms       = TDMA_TEST_SLOT_MS;
            tdma_slots         = TDMA_TEST_SLOTS;
            sendLog("TDMA -> TEST_INTERACTIVE (500ms / 5×100ms)");
        } else {
            tdma_superframe_ms = TDMA_FLIGHT_SUPERFRAME_MS;
            tdma_slot_ms       = TDMA_FLIGHT_SLOT_MS;
            tdma_slots         = TDMA_FLIGHT_SLOTS;
            sendLog("TDMA -> FLIGHT (1000ms / 10×100ms)");
        }
        /* Phase 3-A bugfix: drop sync so the next received packet re-aligns
         * frame_start_ms to FC's clock under the new slot_ms. Without this,
         * the old frame_start (computed under the old slot_ms) leaves
         * Arduino's slot mapping permanently shifted vs FC. */
        tdma_synced = false;
    }
}

/** @} */

// ============================================================================
// Simulation Functions (compiled only if SIMULATION_MODE is true)
// ============================================================================
#if SIMULATION_MODE

void sim_generate_fast_telemetry() {
    TelemetryFast_t pkt = {0};

    pkt.packet_type = TELEM_PACKET_FAST;
    pkt.frame_id = frame_id;
    pkt.slot_id = 0;
    pkt.seq = sim_seq++;
    pkt.flags = 0x01;
    pkt.time = millis();
    pkt.state = sim_state;
    pkt.substate = sim_substate;
    pkt.last_cmd_seq_acked = awaiting_ack_seq;
    pkt.last_cmd_status = 0;

    float t = millis() / 1000.0f;

    sim_altitude = 100.0f + 50.0f * sin(t * 0.5f);
    pkt.altitude = (int16_t)(sim_altitude * 10);
    pkt.vario = (int16_t)(25.0f * cos(t * 0.5f) * 100);

    sim_pitch = 5.0f * sin(t * 0.3f);
    sim_roll = 3.0f * cos(t * 0.4f);
    float sim_yaw = fmod(t * 10.0f, 360.0f);
    pkt.pitch = (int16_t)(sim_pitch * 10);
    pkt.roll = (int16_t)(sim_roll * 10);
    pkt.yaw = (int16_t)(sim_yaw * 10);

    pkt.accel_x = (int16_t)(50 * sin(t * 2.0f));
    pkt.accel_y = (int16_t)(30 * cos(t * 1.5f));
    pkt.accel_z = (int16_t)(1000 + 20 * sin(t));

    pkt.gyro_x = (int16_t)(100 * sin(t * 0.8f));
    pkt.gyro_y = (int16_t)(80 * cos(t * 0.7f));
    pkt.gyro_z = (int16_t)(50 * sin(t * 0.5f));

    pkt.crc16 = crc16_calc((uint8_t*)&pkt, sizeof(pkt) - 2);

    if (awaiting_ack) {
        awaiting_ack = false;
        cmd_pending = false;
        stats.ack_ok++;

        uint8_t ack_data[2] = {awaiting_ack_seq, 0};
        sendToPC(MSG_TYPE_GS_ACK, ack_data, 2);
    }

    sendToPC(MSG_TYPE_FC_FAST, &pkt, sizeof(pkt));
    stats.rx_fast++;
}

void sim_generate_slow_telemetry() {
    TelemetrySlow_t pkt = {0};

    pkt.packet_type = TELEM_PACKET_SLOW;
    pkt.frame_id = frame_id;
    pkt.slot_id = 8;
    pkt.seq = sim_seq;
    pkt.time = millis();

    pkt.latitude = 387765432;
    pkt.longitude = -91234567;
    pkt.gps_altitude = 500;
    pkt.gps_lock = 1;
    pkt.satellites = 8 + (millis() / 10000) % 5;

    float temp = 22.0f + 2.0f * sin(millis() / 30000.0f);
    pkt.temp_baro = (int16_t)(temp * 10);

    float pressure = 1013.25f + 5.0f * sin(millis() / 20000.0f);
    pkt.pressure = (int16_t)((pressure - 1000.0f) * 10);

    pkt.battery_pct = 85 - (millis() / 600000) % 20;
    pkt.sd_status = 1;
    pkt.free_heap = 2000;

    pkt.crc16 = crc16_calc((uint8_t*)&pkt, sizeof(pkt) - 2);

    sendToPC(MSG_TYPE_FC_SLOW, &pkt, sizeof(pkt));
    stats.rx_slow++;
}

void sim_handle_ping() {
    if (ping_awaiting_pong) {
        uint32_t rtt = millis() - ping_sent_time;
        ping_awaiting_pong = false;

        GSPong_t pong;
        pong.ping_seq = ping_seq_sent;
        pong.rtt_ms = rtt;
        pong.fc_timestamp = millis();
        sendToPC(MSG_TYPE_GS_PONG, &pong, sizeof(pong));

        sendLog("PONG!");
    }
}

void sim_update() {
    uint32_t now = millis();

    if ((now - last_sim_fast_ms) >= SIM_FAST_INTERVAL_MS) {
        sim_generate_fast_telemetry();
        last_sim_fast_ms = now;
        sim_handle_ping();
    }

    if ((now - last_sim_slow_ms) >= SIM_SLOW_INTERVAL_MS) {
        sim_generate_slow_telemetry();
        last_sim_slow_ms = now;
    }
}

void sim_process_command(uint8_t cmd_id) {
    switch (cmd_id) {
        case CMD_ARM:
            sim_state = 3;
            sim_substate = 11;
            sendLog("SIM: ARMED");
            break;
        case CMD_DISARM:
            sim_state = 1;
            sim_substate = 0;
            sendLog("SIM: DISARMED");
            break;
        case CMD_LAUNCH:
            sim_state = 5;
            sim_substate = 3;
            sendLog("SIM: LAUNCH!");
            break;
        case CMD_ABORT:
            sim_state = 6;
            sim_substate = 0;
            sendLog("SIM: ABORT!");
            break;
        case CMD_FORCE_SAFE:
            sim_state = 7;
            sim_substate = 0;
            sendLog("SIM: SAFE");
            break;
        case CMD_SET_PROFILE:
            sim_state = 2;
            sendLog("SIM: Profile set");
            break;
        case CMD_CALIBRATE_BARO:
            sendLog("SIM: Baro calibrated");
            break;
    }
}

#endif // SIMULATION_MODE

/**
 * @defgroup RadioComm Radio Communication Functions
 * @{
 */

/**
 * @brief Send sync beacon to FC
 */
void send_sync() {
    SyncPacket_t sync = {0};
    sync.packet_type = TELEM_PACKET_SYNC;
    sync.frame_id = frame_id;
    /* Phase 3-A: in test-interactive mode the GS uses two TX slots; the
     * sync alignment slot is the first one (TX_SLOT_A), matching the FC's
     * `rx_slot_for_sync`. */
    sync.slot_id = (tdma_mode_current == TDMA_MODE_TEST_INTERACTIVE)
                       ? TDMA_TEST_TX_SLOT_A : TDMA_FLIGHT_TX_SLOT;
    sync.gs_time = millis();
    sync.crc16 = crc16_calc((uint8_t*)&sync, sizeof(sync) - 2);

    RADIO_SERIAL.write((uint8_t*)&sync, sizeof(sync));
    stats.tx_sync++;
}

/**
 * @brief Queue command for transmission to FC
 *
 * @param[in] cmd_id    Command identifier
 * @param[in] params    Command parameters (optional)
 * @param[in] param_len Parameter length
 */
void send_command(uint8_t cmd_id, uint8_t* params, uint8_t param_len) {
    /* Build the packet into a queue slot (or drop if queue is full). */
    if (cmd_queue_count >= CMD_QUEUE_DEPTH) {
        sendLog("Cmd queue FULL — dropped");
        return;
    }
    uint8_t slot = (cmd_queue_head + cmd_queue_count) % CMD_QUEUE_DEPTH;
    CommandPacket_t *qp = &cmd_queue[slot];

    qp->packet_type = TELEM_PACKET_COMMAND;
    qp->frame_id = frame_id;
    qp->cmd_id   = cmd_id;
    qp->cmd_seq  = ++cmd_seq;
    qp->time     = millis();

    memset(qp->params, 0, sizeof(qp->params));
    if (params && param_len > 0) {
        uint8_t cap = sizeof(qp->params);
        memcpy(qp->params, params, (param_len < cap) ? param_len : cap);
    }
    qp->crc16 = crc16_calc((uint8_t*)qp, sizeof(*qp) - 2);

    cmd_queue_count++;
    /* If this is the only queued cmd and nothing is currently outgoing,
     * promote it to pending so the next TX slot picks it up. */
    if (!cmd_pending) {
        memcpy(&pending_cmd, qp, sizeof(pending_cmd));
        cmd_queue_head = (cmd_queue_head + 1) % CMD_QUEUE_DEPTH;
        cmd_queue_count--;
        cmd_pending = true;
        awaiting_ack = true;
        awaiting_ack_seq = pending_cmd.cmd_seq;
        cmd_sent_time = millis();
        cmd_retry_count = 0;

        if (cmd_id == CMD_PING) {
            ping_seq_sent = pending_cmd.cmd_seq;
            ping_sent_time = millis();
            ping_awaiting_pong = true;
        }
    }

    #if SIMULATION_MODE
    sim_process_command(cmd_id);
    #endif
}

/* Phase 3-A bugfix: pop the head of cmd_queue into pending_cmd. Called after
 * a successful TX so the next slot has something to send. */
static void cmd_queue_pop_to_pending(void) {
    if (cmd_queue_count == 0) return;
    memcpy(&pending_cmd, &cmd_queue[cmd_queue_head], sizeof(pending_cmd));
    cmd_queue_head = (cmd_queue_head + 1) % CMD_QUEUE_DEPTH;
    cmd_queue_count--;
    cmd_pending = true;
    awaiting_ack = true;
    awaiting_ack_seq = pending_cmd.cmd_seq;
    cmd_sent_time = millis();
    cmd_retry_count = 0;

    if (pending_cmd.cmd_id == CMD_PING) {
        ping_seq_sent = pending_cmd.cmd_seq;
        ping_sent_time = millis();
        ping_awaiting_pong = true;
    }
}

/**
 * @brief Transmit pending command over radio
 */
void transmit_pending_command() {
    if (!cmd_pending) return;

    #if !SIMULATION_MODE
    pending_cmd.frame_id = frame_id;
    pending_cmd.time = millis();
    pending_cmd.crc16 = crc16_calc((uint8_t*)&pending_cmd, sizeof(pending_cmd) - 2);

    RADIO_SERIAL.write((uint8_t*)&pending_cmd, sizeof(pending_cmd));
    stats.tx_cmd++;

    if (debug_enabled) {
        PC_SERIAL.print(F("[TX CMD] id="));
        PC_SERIAL.print(pending_cmd.cmd_id);
        PC_SERIAL.print(F(" seq="));
        PC_SERIAL.println(pending_cmd.cmd_seq);
    }
    #endif

    /* Phase 3-A bugfix: TX done → mark pending_cmd consumed and pull the
     * next queued command (if any) into pending_cmd so the very next TX
     * slot can transmit it without waiting for a new send_command call. */
    cmd_pending = false;
    cmd_queue_pop_to_pending();
}

/** @} */

/**
 * @defgroup PacketProc Packet Processing Functions
 * @{
 */

/**
 * @brief Get expected packet size for type
 *
 * @param[in] type Packet type ID
 *
 * @return Expected size in bytes, 0 if unknown
 */
uint16_t get_packet_size(uint8_t type) {
    switch (type) {
        case TELEM_PACKET_FAST:  return FAST_PKT_SIZE;
        case TELEM_PACKET_SLOW:  return SLOW_PKT_SIZE;
        case TELEM_PACKET_EVENT: return EVENT_PKT_SIZE;
        default: return 0;
    }
}

/**
 * @brief Process received FC packet
 *
 * Validates CRC, extracts data, forwards to PC, handles ACKs.
 *
 * @param[in] data Packet data buffer
 * @param[in] len  Packet length
 * @param[in] type Packet type
 */
void process_fc_packet(uint8_t* data, uint16_t len, uint8_t type) {
    // STM32 sends CRC as little-endian (low byte first)
    uint16_t pkt_crc = (uint16_t)data[len-2] | ((uint16_t)data[len-1] << 8);

    uint16_t calc_crc = crc16_calc(data, len - 2);

    if (pkt_crc != calc_crc) {
        stats.crc_errors++;
        if (debug_enabled) {
            PC_SERIAL.print(F("[CRC ERR] type="));
            PC_SERIAL.print(type, HEX);
            PC_SERIAL.print(F(" got="));
            PC_SERIAL.print(pkt_crc, HEX);
            PC_SERIAL.print(F(" calc="));
            PC_SERIAL.println(calc_crc, HEX);
            debugHex(data, len);
        }
        return;
    }

    if (debug_enabled) {
        PC_SERIAL.print(F("[RX OK] type="));
        PC_SERIAL.print(type, HEX);
        PC_SERIAL.print(F(" len="));
        PC_SERIAL.println(len);
    }

    uint8_t msgType;
    switch (type) {
        case TELEM_PACKET_FAST:
            msgType = MSG_TYPE_FC_FAST;
            stats.rx_fast++;
            /* Phase 3-A bugfix: detect TDMA mode mismatch from slot_id. In
             * test-interactive layout, FC fast TX only goes in slots 0 and 2;
             * in flight, slots 0..7. So a fast packet with slot_id >= 5 means
             * the FC is in flight mode while we are still in test. Without
             * this, the slow packet that announces the switch arrives in a
             * slot where we are TXing → collision → Arduino never recovers. */
            if (data[2] >= 5 && tdma_mode_current == TDMA_MODE_TEST_INTERACTIVE) {
                tdma_mode_pending = TDMA_MODE_FLIGHT;
            }

            // Check for command ACK in fast packet
            if (awaiting_ack && data[11] == awaiting_ack_seq) {
                awaiting_ack = false;
                cmd_pending = false;
                stats.ack_ok++;

                uint8_t ack_data[2] = {data[11], data[12]};
                sendToPC(MSG_TYPE_GS_ACK, ack_data, 2);

                if (debug_enabled) {
                    PC_SERIAL.print(F("[ACK] seq="));
                    PC_SERIAL.println(data[11]);
                }
            }
            break;

        case TELEM_PACKET_SLOW:
            msgType = MSG_TYPE_FC_SLOW;
            stats.rx_slow++;
            /* Phase 3-A (A4): the FC announces its active TDMA preset in the
             * slow packet. Queue the switch — it's applied on the next GS
             * superframe rollover so we don't break a slot mid-frame. */
            if (len >= SLOW_PKT_SIZE) {
                uint8_t announced = ((TelemetrySlow_t*)data)->tdma_mode;
                if (announced != tdma_mode_pending) {
                    tdma_mode_pending = announced;
                }
            }
            break;

        case TELEM_PACKET_EVENT:
            msgType = MSG_TYPE_FC_EVENT;
            stats.rx_event++;

            // Check for pong in event packet
            if (data[8] == 13 && ping_awaiting_pong) {
                uint8_t pong_seq = data[11];
                if (pong_seq == ping_seq_sent) {
                    uint32_t rtt = millis() - ping_sent_time;
                    ping_awaiting_pong = false;

                    GSPong_t pong;
                    pong.ping_seq = pong_seq;
                    pong.rtt_ms = rtt;
                    memcpy(&pong.fc_timestamp, &data[12], 4);
                    sendToPC(MSG_TYPE_GS_PONG, &pong, sizeof(pong));

                    if (debug_enabled) {
                        PC_SERIAL.print(F("[PONG] RTT="));
                        PC_SERIAL.println(rtt);
                    }
                }
            }
            break;

        default:
            return;
    }

    sendToPC(msgType, data, len);

    // Sync TDMA timing from received packet
    if (!tdma_synced) {
        tdma_synced = true;
        /* Phase 3-A (A4): if this first packet is a slow announcing test mode,
         * apply the preset switch *before* aligning so the slot_id math uses
         * the correct slot_ms. */
        if (type == TELEM_PACKET_SLOW && len >= SLOW_PKT_SIZE) {
            uint8_t announced = ((TelemetrySlow_t*)data)->tdma_mode;
            if (announced != tdma_mode_current) {
                tdma_mode_current = announced;
                tdma_mode_pending = announced;
                if (announced == TDMA_MODE_TEST_INTERACTIVE) {
                    tdma_superframe_ms = TDMA_TEST_SUPERFRAME_MS;
                    tdma_slot_ms       = TDMA_TEST_SLOT_MS;
                    tdma_slots         = TDMA_TEST_SLOTS;
                }
            }
        }
        frame_start_ms = millis() - (data[2] * TDMA_SLOT_MS);
        frame_id = data[1];
        sendLog("TDMA synced!");
    }
}

/**
 * @brief Process single received byte
 *
 * Accumulates bytes into packet, triggers processing when complete.
 *
 * @param[in] byte Received byte
 *
 * @return true if complete packet processed
 */
bool process_rx_byte(uint8_t byte) {
    uint32_t now = millis();

    // Timeout: reset buffer
    if (rx_idx > 0 && (now - rx_last_byte_ms) > RX_TIMEOUT_MS) {
        if (debug_enabled && rx_idx > 0) {
            PC_SERIAL.print(F("[TIMEOUT] dropped "));
            PC_SERIAL.print(rx_idx);
            PC_SERIAL.println(F(" bytes"));
        }
        rx_idx = 0;
    }
    rx_last_byte_ms = now;

    // First byte: check if valid packet type
    if (rx_idx == 0) {
        uint16_t expected = get_packet_size(byte);
        if (expected == 0) {
            if (debug_enabled) {
                PC_SERIAL.print(F("[?] Unknown type: 0x"));
                PC_SERIAL.println(byte, HEX);
            }
            return false;
        }
    }

    // Store byte
    if (rx_idx < RX_BUFFER_SIZE) {
        rx_buffer[rx_idx++] = byte;
    } else {
        rx_idx = 0;
        return false;
    }

    // Check for complete packet
    uint8_t type = rx_buffer[0];
    uint16_t expected = get_packet_size(type);

    if (rx_idx >= expected) {
        process_fc_packet(rx_buffer, expected, type);
        rx_idx = 0;
        return true;
    }

    return false;
}

/**
 * @brief Process all available radio RX data
 */
void process_radio_rx() {
    while (RADIO_SERIAL.available()) {
        uint8_t b = RADIO_SERIAL.read();
        process_rx_byte(b);
    }
}

/** @} */

/**
 * @defgroup SlotHandlers TDMA Slot Handlers
 * @{
 */

/**
 * @brief Handle TX slot (slot 9)
 */
void handle_tx_slot() {
    if (tx_done_this_slot) return;

    uint32_t time_in_slot = get_time_in_slot();

    if (time_in_slot >= TX_START_OFFSET_MS && time_in_slot < TX_END_OFFSET_MS) {
        /* Phase 3-A: in test-interactive mode, slot A is the sync-alignment
         * slot (mirrors flight slot 9). Slot B prefers test_ctrl/cmd. */
        bool is_sync_slot = (tdma_mode_current != TDMA_MODE_TEST_INTERACTIVE)
                         || (get_current_slot() == TDMA_TEST_TX_SLOT_A);

        /* Phase 3-A bugfix: in test mode with manual control, test_ctrl is
         * always pending — without a forced fallback, sync is starved and the
         * FC trips its 5 s sync timeout. Force a sync at least once per
         * SYNC_FORCE_INTERVAL_MS, even if test_ctrl is also pending. */
        static uint32_t last_sync_tx_ms = 0;
        const uint32_t SYNC_FORCE_INTERVAL_MS = 1000;  // 1 Hz minimum
        bool force_sync = is_sync_slot
                       && ((millis() - last_sync_tx_ms) > SYNC_FORCE_INTERVAL_MS);

        /* Priority: cmd > [forced sync] > test_ctrl > opportunistic sync */
        if (cmd_pending) {
            transmit_pending_command();
        } else if (force_sync) {
            #if !SIMULATION_MODE
            send_sync();
            last_sync_tx_ms = millis();
            #endif
        } else if (test_ctrl_pending) {
            RADIO_SERIAL.write(pending_test_ctrl, TEST_CTRL_PKT_SIZE);
            test_ctrl_pending = false;
            stats.tx_cmd++;
        } else if (is_sync_slot) {
            #if !SIMULATION_MODE
            send_sync();
            last_sync_tx_ms = millis();
            #endif
        }
        tx_done_this_slot = true;
    }
}

/**
 * @brief Handle RX slots (slots 0-8)
 */
void handle_rx_slots() {
    #if !SIMULATION_MODE
    process_radio_rx();
    #endif
}

/** @} */

/**
 * @brief Check for command timeout and retry
 */
void check_command_timeout() {
    #if SIMULATION_MODE
    return;
    #endif

    if (!awaiting_ack) return;

    if ((millis() - cmd_sent_time) >= CMD_TIMEOUT_MS) {
        cmd_retry_count++;

        if (cmd_retry_count >= CMD_MAX_RETRIES) {
            awaiting_ack = false;
            cmd_pending = false;
            stats.ack_timeout++;
            ping_awaiting_pong = false;

            sendLog("CMD timeout");
        } else {
            /* Phase 3-A bugfix: previous version updated cmd_sent_time but
             * left cmd_pending=false, so the cmd was never actually
             * re-transmitted — 5 silent retries × 3 s = a 15 s black hole.
             * Re-arm cmd_pending so the next TX slot re-sends the packet. */
            cmd_pending = true;
            cmd_sent_time = millis();
            stats.cmd_retries++;
            if (debug_enabled) {
                PC_SERIAL.print(F("[RETRY] attempt "));
                PC_SERIAL.println(cmd_retry_count);
            }
        }
    }
}

/* Phase 3-A bugfix: non-blocking PC-input state machine.
 * The previous version did `while (got<N && millis()<deadline)` blocking
 * reads inside `case '~'` and `case '#'`. At 10 Hz manual-test cadence
 * that's 500 ms/sec of CPU stall — and any 50 ms blocking call destroys
 * the 40 ms test-mode TDMA slots, causing collisions and lost packets.
 *
 * The state machine consumes whatever bytes are available each loop pass,
 * accumulates multi-byte sequences across loop iterations, and never
 * blocks. A 200 ms watchdog recovers from corrupted partials. */
/* Binary frame framing (PC -> Arduino):
 *   sync1=0xAA, sync2=0x55, type=0x01 (TEST_CTRL) -> 15 payload bytes
 *   sync1=0xAA, sync2=0x55, type=0x02 (TEST_PROFILE) -> 32 payload bytes
 * The 2-byte sync (both non-ASCII) makes accidental misparse of payload bytes
 * as single-char commands impossible — the old '~'/'#' single-byte prefix was
 * indistinguishable from a payload byte 0x7E/0x23, so a lost prefix turned the
 * remaining bytes into 'P'/'T'/'A'/'S'/etc. commands. */
#define PC_FRAME_SYNC_A      0xAA
#define PC_FRAME_SYNC_B      0x55
#define PC_FRAME_TEST_CTRL    0x01
#define PC_FRAME_TEST_PROFILE 0x02

typedef enum {
    PC_S_IDLE = 0,            /**< Waiting for the next command char */
    PC_S_SYNC1,               /**< Got 0xAA, waiting for 0x55 */
    PC_S_FRAME_TYPE,          /**< Got sync, waiting for frame type byte */
    PC_S_TEST_CTRL,           /**< Collecting 15 binary bytes */
    PC_S_TEST_PROFILE,        /**< Collecting 32 binary bytes */
    PC_S_E_DIGITS             /**< After 'E', collecting 0–3 decimal digits */
} pc_input_state_t;

static pc_input_state_t pc_state = PC_S_IDLE;
static uint8_t  pc_buf[32];
static uint8_t  pc_count = 0;
static uint8_t  pc_e_throttle = 0;
static uint8_t  pc_e_digits = 0;
static uint32_t pc_state_started_ms = 0;
#define PC_STATE_TIMEOUT_MS 1000

static void pc_dispatch_static_test(void) {
    uint8_t throttle = pc_e_throttle == 0 ? 20 : pc_e_throttle;
    if (throttle > 100) throttle = 100;
    uint8_t params[8] = {throttle, 0, 0, 0, 0, 0, 0, 0};
    send_command(CMD_STATIC_TEST, params, 1);
    char msg[40];
    snprintf(msg, sizeof(msg), "Static test: %d%% throttle", throttle);
    sendLog(msg);
}

static void pc_handle_single_char(char c) {
    switch (c) {
        case PC_CMD_PING:        send_command(CMD_PING, NULL, 0);            sendLog("Sending PING"); break;
        case PC_CMD_CALIBRATE:   send_command(CMD_CALIBRATE_BARO, NULL, 0);  sendLog("Sending CALIBRATE BARO"); break;
        case PC_CMD_MOTOR_CAL:   send_command(CMD_CALIBRATE_MOTOR, NULL, 0); sendLog("Sending MOTOR CAL - POWER CYCLE ESC NOW!"); break;
        case PC_CMD_ARM:        send_command(CMD_ARM, NULL, 0);          sendLog("Sending ARM"); break;
        case PC_CMD_DISARM:     send_command(CMD_DISARM, NULL, 0);       sendLog("Sending DISARM"); break;
        case PC_CMD_TEST:       send_command(CMD_START_TEST, NULL, 0);   sendLog("Sending TEST"); break;
        case PC_CMD_LAUNCH:     send_command(CMD_LAUNCH, NULL, 0);       sendLog("Sending LAUNCH"); break;
        case PC_CMD_ABORT:      send_command(CMD_ABORT, NULL, 0);        sendLog("Sending ABORT"); break;
        case PC_CMD_SAFE:       send_command(CMD_FORCE_SAFE, NULL, 0);   sendLog("Sending SAFE"); break;
        case PC_CMD_RESET_STATS: memset(&stats, 0, sizeof(stats));        sendLog("Stats reset"); break;
        case PC_CMD_DEBUG:      debug_enabled = !debug_enabled;          sendLog(debug_enabled ? "Debug ON" : "Debug OFF"); break;
        /* Phase 3-A new test-FSM lifecycle commands. */
        case 'H':               send_command(CMD_HOLD, NULL, 0);          sendLog("Sending HOLD"); break;
        case 'U':               send_command(CMD_RESUME, NULL, 0);        sendLog("Sending RESUME"); break;
        case 'K':               send_command(CMD_STOP_TEST, NULL, 0);     sendLog("Sending STOP_TEST"); break;
        default: break;
    }
}

void process_pc_input() {
    uint32_t now = millis();

    /* Watchdog: if a multi-byte sequence stalls > 200 ms, recover. For
     * 'E' specifically, dispatch with whatever digits we have (this is how
     * a bare 'E' with no digits gets the default 20%). */
    if (pc_state != PC_S_IDLE && (now - pc_state_started_ms) > PC_STATE_TIMEOUT_MS) {
        if (pc_state == PC_S_E_DIGITS) pc_dispatch_static_test();
        pc_state = PC_S_IDLE;
        pc_count = 0;
    }

    /* Drain whatever is available without ever blocking. */
    while (PC_SERIAL.available()) {
        uint8_t c = PC_SERIAL.read();

        switch (pc_state) {
            case PC_S_IDLE:
                if (c == PC_FRAME_SYNC_A) {
                    pc_state = PC_S_SYNC1; pc_state_started_ms = now;
                } else if (c == PC_CMD_STATIC_TEST) {
                    pc_state = PC_S_E_DIGITS; pc_e_throttle = 0; pc_e_digits = 0;
                    pc_state_started_ms = now;
                } else {
                    pc_handle_single_char((char)c);
                }
                break;

            case PC_S_SYNC1:
                if (c == PC_FRAME_SYNC_B) {
                    pc_state = PC_S_FRAME_TYPE; pc_state_started_ms = now;
                } else {
                    /* Not a real frame — abandon and re-process this byte from IDLE.
                     * 0xAA followed by anything-but-0x55 is noise. */
                    pc_state = PC_S_IDLE;
                    /* Re-feed: if it's a known single-char cmd dispatch, else drop. */
                    if (c != PC_FRAME_SYNC_A) pc_handle_single_char((char)c);
                    else { pc_state = PC_S_SYNC1; pc_state_started_ms = now; }
                }
                break;

            case PC_S_FRAME_TYPE:
                pc_state_started_ms = now;
                pc_count = 0;
                if (c == PC_FRAME_TEST_CTRL)       pc_state = PC_S_TEST_CTRL;
                else if (c == PC_FRAME_TEST_PROFILE) pc_state = PC_S_TEST_PROFILE;
                else                                 pc_state = PC_S_IDLE; /* unknown type */
                break;

            case PC_S_TEST_CTRL:
                /* Refresh watchdog per byte. Without this, a slow main-loop
                 * iteration (radio TX, status reports) can cause the 200 ms
                 * watchdog to fire mid-frame, reset state to IDLE, and let the
                 * remaining payload bytes be misparsed as ASCII commands —
                 * which is exactly how 0x50/'P', 0x54/'T', 0x53/'S' etc. were
                 * accidentally dispatching PING/TEST/FORCE_SAFE during tests. */
                pc_state_started_ms = now;
                pending_test_ctrl[pc_count++] = c;
                if (pc_count >= TEST_CTRL_PKT_SIZE) {
                    test_ctrl_pending = true;
                    pc_state = PC_S_IDLE;
                }
                break;

            case PC_S_TEST_PROFILE:
                pc_state_started_ms = now;
                pc_buf[pc_count++] = c;
                if (pc_count >= 32) {
                    send_command(CMD_SET_TEST_PROFILE, pc_buf, 32);
                    sendLog("Sending SET_TEST_PROFILE");
                    pc_state = PC_S_IDLE;
                }
                break;

            case PC_S_E_DIGITS:
                if (c >= '0' && c <= '9' && pc_e_digits < 3) {
                    pc_e_throttle = pc_e_throttle * 10 + (c - '0');
                    pc_e_digits++;
                } else {
                    /* Non-digit terminates the sequence; dispatch and re-feed
                     * the byte to IDLE so it can start a fresh command. */
                    pc_dispatch_static_test();
                    pc_state = PC_S_IDLE;
                    if (c == '~' || c == '#' || c == PC_CMD_STATIC_TEST) {
                        /* Restart sequence; we lose this byte's effect, accept it. */
                        if (c == '~') { pc_state = PC_S_TEST_CTRL; pc_count = 0; pc_state_started_ms = now; }
                        else if (c == '#') { pc_state = PC_S_TEST_PROFILE; pc_count = 0; pc_state_started_ms = now; }
                        else { pc_state = PC_S_E_DIGITS; pc_e_throttle = 0; pc_e_digits = 0; pc_state_started_ms = now; }
                    } else {
                        pc_handle_single_char((char)c);
                    }
                }
                break;
        }
    }
}

/**
 * @brief Send status update to PC
 */
void send_status() {
    GSStatus_t status;
    #if SIMULATION_MODE
    status.tdma_synced = 1;
    #else
    status.tdma_synced = tdma_synced ? 1 : 0;
    #endif
    status.frame_id = frame_id;
    status.slot_id = get_current_slot();
    status.cmd_pending = cmd_pending ? 1 : 0;
    status.awaiting_ack = awaiting_ack ? 1 : 0;
    status.last_ack_seq = awaiting_ack_seq;
    status.last_ack_status = 0;
    status.uptime_ms = millis();

    sendToPC(MSG_TYPE_GS_STATUS, &status, sizeof(status));
    sendToPC(MSG_TYPE_GS_STATS, &stats, sizeof(stats));
}

/**
 * @brief Arduino setup function
 *
 * Initializes serial ports, E22 mode pins, and state.
 */
void setup() {
    PC_SERIAL.begin(PC_BAUD);
    while (!PC_SERIAL) delay(10);

    PC_SERIAL.println(F("\n\n=== Ground Station Starting ==="));

    // Configure E22 mode pins - LOW for normal operation
    pinMode(E22_M0, OUTPUT);
    pinMode(E22_M1, OUTPUT);
    digitalWrite(E22_M0, LOW);
    digitalWrite(E22_M1, LOW);
    PC_SERIAL.println(F("E22 M0=LOW, M1=LOW (Normal mode)"));

    delay(100);

    RADIO_SERIAL.begin(9600);
    PC_SERIAL.println(F("Radio Serial @ 9600 baud"));

    frame_start_ms = millis();
    frame_id = 0;

    delay(100);

    #if SIMULATION_MODE
    PC_SERIAL.println(F("*** SIMULATION MODE ***"));
    sendLog("GS Ready (SIMULATION)");
    tdma_synced = true;
    #else
    PC_SERIAL.println(F("*** REAL MODE - Waiting for FC ***"));
    sendLog("GS Ready");
    #endif

    PC_SERIAL.println(F("Commands: P=Ping C=Cal 1/2/3=Profile A=Arm D=Disarm T=Test L=Launch X=Abort S=Safe ?=Debug"));
    PC_SERIAL.println(F("===============================\n"));
}

/**
 * @brief Arduino main loop
 *
 * Handles TDMA timing, slot processing, simulation, and PC communication.
 */
void loop() {
    uint32_t now = millis();

    // Frame rollover
    if ((now - frame_start_ms) >= TDMA_SUPERFRAME_MS) {
        advance_frame();
    }

    // Slot handling
    uint8_t slot = get_current_slot();
    static uint8_t last_slot = 255;
    if (slot != last_slot) {
        last_slot = slot;
        tx_done_this_slot = false;
    }

    if (tdma_slot_is_tx(slot)) {
        handle_tx_slot();
    } else {
        handle_rx_slots();
    }

    #if SIMULATION_MODE
    sim_update();
    #endif

    check_command_timeout();
    process_pc_input();

    // Periodic status
    if ((now - last_status_ms) >= STATUS_INTERVAL_MS) {
        send_status();
        last_status_ms = now;

        if (debug_enabled) {
            PC_SERIAL.print(F("[STATS] fast="));
            PC_SERIAL.print(stats.rx_fast);
            PC_SERIAL.print(F(" slow="));
            PC_SERIAL.print(stats.rx_slow);
            PC_SERIAL.print(F(" crc_err="));
            PC_SERIAL.println(stats.crc_errors);
        }
    }

    /* Phase 3-A debug: per-second TX rate dump. Discriminates between
     * "Arduino not transmitting" and "transmitting but FC not receiving". */
    static uint32_t last_tx_dump_ms = 0;
    static uint32_t prev_tx_sync = 0, prev_tx_cmd = 0;
    if ((now - last_tx_dump_ms) >= 1000) {
        uint32_t dsync = stats.tx_sync - prev_tx_sync;
        uint32_t dcmd  = stats.tx_cmd  - prev_tx_cmd;
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "TX/s sync=%lu cmd=%lu synced=%d mode=%d pending=%d retries=%lu",
                 (unsigned long)dsync, (unsigned long)dcmd,
                 tdma_synced ? 1 : 0, tdma_mode_current,
                 cmd_pending ? 1 : 0, (unsigned long)stats.cmd_retries);
        sendLog(buf);
        prev_tx_sync = stats.tx_sync;
        prev_tx_cmd  = stats.tx_cmd;
        last_tx_dump_ms = now;
    }

    delay(1);
}
