/*
 * Ground Station Arduino - Passthrough Mode
 * ==========================================
 *
 * Simple USB-to-LoRa bridge. All packet parsing done on PC.
 *
 * Responsibilities:
 *   - Forward raw LoRa RX -> PC (with framing)
 *   - Forward PC commands -> LoRa TX (in TDMA slot 9)
 *   - Basic TDMA timing for TX slot
 *
 * Frame format to PC:
 *   [0xAA] [0x55] [len_hi] [len_lo] [type] [payload...] [crc_hi] [crc_lo]
 *
 * Hardware:
 *   - Arduino Mega 2560
 *   - E22-900T22S LoRa module on Serial1
 */

#include <SoftwareSerial.h>

// ============================================================================
// Configuration
// ============================================================================
#define RADIO_RX_PIN    2
#define RADIO_TX_PIN    3
#define RADIO_M0_PIN    4
#define RADIO_M1_PIN    5

#define PC_BAUDRATE     115200
#define RADIO_BAUDRATE  9600

SoftwareSerial RadioSerial(RADIO_RX_PIN, RADIO_TX_PIN);

// ============================================================================
// Protocol Constants
// ============================================================================
#define FRAME_SYNC_1        0xAA
#define FRAME_SYNC_2        0x55

// Message types TO PC
#define MSG_RAW_LORA_RX     0x01  // Raw LoRa packet received
#define MSG_GS_STATUS       0x10  // GS status update
#define MSG_GS_LOG          0x20  // Debug log message

// Message types FROM PC (single char commands)
#define PC_CMD_PING         'P'
#define PC_CMD_CALIBRATE    'C'
#define PC_CMD_PROFILE_1    '1'
#define PC_CMD_PROFILE_2    '2'
#define PC_CMD_PROFILE_3    '3'
#define PC_CMD_ARM          'A'
#define PC_CMD_DISARM       'D'
#define PC_CMD_TEST         'T'
#define PC_CMD_LAUNCH       'L'
#define PC_CMD_ABORT        'X'
#define PC_CMD_SAFE         'S'
#define PC_CMD_DEBUG        '?'

// FC Command IDs (must match STM32)
#define FC_CMD_PING         1
#define FC_CMD_SET_PROFILE  2
#define FC_CMD_ARM          4
#define FC_CMD_DISARM       5
#define FC_CMD_START_TEST   6
#define FC_CMD_LAUNCH       7
#define FC_CMD_ABORT        8
#define FC_CMD_FORCE_SAFE   9
#define FC_CMD_CALIBRATE    10

// TDMA Configuration
#define TDMA_SUPERFRAME_MS  1000
#define TDMA_SLOT_MS        100
#define TDMA_SLOTS          10
#define TDMA_TX_SLOT        9     // GS transmits in slot 9
#define TX_WINDOW_START_MS  10
#define TX_WINDOW_END_MS    70

// ============================================================================
// Packet Structures
// ============================================================================
#pragma pack(push, 1)
typedef struct {
    uint8_t packet_type;    // 0x04 = command
    uint8_t frame_id;
    uint8_t cmd_id;
    uint8_t cmd_seq;
    uint32_t time;
    uint8_t params[8];
    uint16_t crc16;
} CommandPacket_t;
#pragma pack(pop)

// ============================================================================
// State
// ============================================================================
uint8_t rx_buffer[64];
uint8_t rx_index = 0;
uint32_t rx_last_byte_time = 0;

uint32_t frame_start_time = 0;
uint8_t frame_id = 0;
bool synced = false;

uint8_t cmd_seq = 0;
bool tx_done_this_slot = false;
uint8_t last_slot = 255;

CommandPacket_t pending_cmd;
bool cmd_pending = false;

uint32_t last_status_time = 0;
bool debug_mode = false;

// Statistics
uint32_t rx_packets = 0;
uint32_t tx_commands = 0;
uint32_t rx_bytes_total = 0;

// ============================================================================
// CRC-16 Modbus
// ============================================================================
uint16_t crc16_update(uint16_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA001;
        else crc >>= 1;
    }
    return crc;
}

uint16_t crc16_calc(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

// ============================================================================
// Send Framed Message to PC
// ============================================================================
void sendToPC(uint8_t msgType, const uint8_t* data, uint16_t len) {
    uint16_t total_len = 1 + len;  // type + payload
    uint16_t crc = crc16_calc(data, len);

    Serial.write(FRAME_SYNC_1);
    Serial.write(FRAME_SYNC_2);
    Serial.write((total_len >> 8) & 0xFF);
    Serial.write(total_len & 0xFF);
    Serial.write(msgType);
    Serial.write(data, len);
    Serial.write((crc >> 8) & 0xFF);
    Serial.write(crc & 0xFF);
}

void sendLog(const char* msg) {
    sendToPC(MSG_GS_LOG, (const uint8_t*)msg, strlen(msg));
}

// ============================================================================
// TDMA Timing
// ============================================================================
uint8_t getCurrentSlot() {
    if (!synced) return 255;
    uint32_t time_in_frame = millis() - frame_start_time;
    return (time_in_frame / TDMA_SLOT_MS) % TDMA_SLOTS;
}

uint32_t getTimeInSlot() {
    if (!synced) return 0;
    uint32_t time_in_frame = millis() - frame_start_time;
    return time_in_frame % TDMA_SLOT_MS;
}

bool isInTxWindow() {
    uint32_t t = getTimeInSlot();
    return (t >= TX_WINDOW_START_MS && t < TX_WINDOW_END_MS);
}

void syncToFrame() {
    // Simple sync: assume frame starts now
    // Better: sync to received FC packets
    frame_start_time = millis();
    synced = true;
}

// ============================================================================
// Radio Setup
// ============================================================================
void setupRadio() {
    pinMode(RADIO_M0_PIN, OUTPUT);
    pinMode(RADIO_M1_PIN, OUTPUT);

    // Normal mode (M0=0, M1=0)
    digitalWrite(RADIO_M0_PIN, LOW);
    digitalWrite(RADIO_M1_PIN, LOW);

    delay(100);
    RadioSerial.begin(RADIO_BAUDRATE);
}

bool radioReady() {
    // No AUX pin - always assume ready
    return true;
}

// ============================================================================
// Queue Command for TX
// ============================================================================
void queueCommand(uint8_t cmdId, const uint8_t* params, uint8_t paramLen) {
    if (cmd_pending) {
        sendLog("CMD busy");
        return;
    }

    memset(&pending_cmd, 0, sizeof(pending_cmd));
    pending_cmd.packet_type = 0x04;  // TELEM_PACKET_COMMAND
    pending_cmd.frame_id = frame_id;
    pending_cmd.cmd_id = cmdId;
    pending_cmd.cmd_seq = ++cmd_seq;
    pending_cmd.time = millis();

    if (params && paramLen > 0) {
        memcpy(pending_cmd.params, params, min(paramLen, (uint8_t)8));
    }

    pending_cmd.crc16 = crc16_calc((uint8_t*)&pending_cmd, sizeof(pending_cmd) - 2);
    cmd_pending = true;

    if (debug_mode) {
        char buf[32];
        snprintf(buf, sizeof(buf), "CMD queued: %d", cmdId);
        sendLog(buf);
    }
}

// ============================================================================
// Transmit Pending Command
// ============================================================================
void transmitCommand() {
    if (!cmd_pending || !radioReady()) return;

    pending_cmd.frame_id = frame_id;
    pending_cmd.time = millis();
    pending_cmd.crc16 = crc16_calc((uint8_t*)&pending_cmd, sizeof(pending_cmd) - 2);

    RadioSerial.write((uint8_t*)&pending_cmd, sizeof(pending_cmd));
    tx_commands++;

    // Command sent - clear pending (no retry in passthrough mode)
    cmd_pending = false;

    if (debug_mode) {
        char buf[32];
        snprintf(buf, sizeof(buf), "CMD TX: id=%d seq=%d", pending_cmd.cmd_id, pending_cmd.cmd_seq);
        sendLog(buf);
    }
}

// ============================================================================
// Process PC Input
// ============================================================================
void processPCInput() {
    while (Serial.available()) {
        char c = Serial.read();

        switch (c) {
            case PC_CMD_PING:
                queueCommand(FC_CMD_PING, NULL, 0);
                break;
            case PC_CMD_CALIBRATE:
                queueCommand(FC_CMD_CALIBRATE, NULL, 0);
                break;
            case PC_CMD_PROFILE_1: {
                uint8_t params[1] = {1};
                queueCommand(FC_CMD_SET_PROFILE, params, 1);
                break;
            }
            case PC_CMD_PROFILE_2: {
                uint8_t params[1] = {2};
                queueCommand(FC_CMD_SET_PROFILE, params, 1);
                break;
            }
            case PC_CMD_PROFILE_3: {
                uint8_t params[1] = {3};
                queueCommand(FC_CMD_SET_PROFILE, params, 1);
                break;
            }
            case PC_CMD_ARM:
                queueCommand(FC_CMD_ARM, NULL, 0);
                break;
            case PC_CMD_DISARM:
                queueCommand(FC_CMD_DISARM, NULL, 0);
                break;
            case PC_CMD_TEST:
                queueCommand(FC_CMD_START_TEST, NULL, 0);
                break;
            case PC_CMD_LAUNCH:
                queueCommand(FC_CMD_LAUNCH, NULL, 0);
                break;
            case PC_CMD_ABORT:
                queueCommand(FC_CMD_ABORT, NULL, 0);
                break;
            case PC_CMD_SAFE:
                queueCommand(FC_CMD_FORCE_SAFE, NULL, 0);
                break;
            case PC_CMD_DEBUG:
                debug_mode = !debug_mode;
                sendLog(debug_mode ? "Debug ON" : "Debug OFF");
                break;
            case 'R':  // Reset sync
                synced = false;
                sendLog("Sync reset");
                break;
            default:
                break;
        }
    }
}

// ============================================================================
// Process Radio RX - Passthrough Mode
// ============================================================================
void processRadioRX() {
    while (RadioSerial.available()) {
        uint8_t b = RadioSerial.read();
        rx_bytes_total++;

        // Timeout: reset buffer if no data for 50ms
        if (millis() - rx_last_byte_time > 50) {
            rx_index = 0;
        }
        rx_last_byte_time = millis();

        // Add to buffer
        if (rx_index < sizeof(rx_buffer)) {
            rx_buffer[rx_index++] = b;
        }

        // Check for complete packet
        if (rx_index >= 2) {
            uint8_t pkt_type = rx_buffer[0];
            uint16_t expected_len = 0;

            // Determine expected length based on packet type
            switch (pkt_type) {
                case 0x01: expected_len = 35; break;  // FAST
                case 0x02: expected_len = 30; break;  // SLOW
                case 0x03: expected_len = 37; break;  // EVENT
                case 0x05: expected_len = 10; break;  // SYNC
                default:
                    // Unknown type - shift buffer
                    memmove(rx_buffer, rx_buffer + 1, rx_index - 1);
                    rx_index--;
                    continue;
            }

            if (rx_index >= expected_len) {
                // Got complete packet - forward to PC
                sendToPC(MSG_RAW_LORA_RX, rx_buffer, expected_len);
                rx_packets++;

                // Sync to FC timing (use packet arrival as reference)
                if (!synced) {
                    syncToFrame();
                    sendLog("Synced to FC");
                }

                // Shift buffer
                if (rx_index > expected_len) {
                    memmove(rx_buffer, rx_buffer + expected_len, rx_index - expected_len);
                    rx_index -= expected_len;
                } else {
                    rx_index = 0;
                }
            }
        }
    }
}

// ============================================================================
// Send Status to PC
// ============================================================================
void sendStatus() {
    uint8_t status[12];
    status[0] = synced ? 1 : 0;
    status[1] = frame_id;
    status[2] = getCurrentSlot();
    status[3] = cmd_pending ? 1 : 0;

    // Stats (as uint32_t little-endian)
    memcpy(&status[4], &rx_packets, 4);
    memcpy(&status[8], &tx_commands, 4);

    sendToPC(MSG_GS_STATUS, status, sizeof(status));
}

// ============================================================================
// TDMA Handler
// ============================================================================
void handleTDMA() {
    if (!synced) {
        // Not synced - just listen
        return;
    }

    uint8_t slot = getCurrentSlot();

    // Reset TX flag on slot change
    if (slot != last_slot) {
        last_slot = slot;
        tx_done_this_slot = false;

        // Increment frame ID when slot wraps
        if (slot == 0) {
            frame_id++;
        }
    }

    // TX in slot 9 only
    if (slot == TDMA_TX_SLOT && !tx_done_this_slot) {
        if (isInTxWindow() && cmd_pending) {
            transmitCommand();
            tx_done_this_slot = true;
        }
    }
}

// ============================================================================
// Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(PC_BAUDRATE);
    setupRadio();

    delay(500);
    sendLog("GS Passthrough Ready");
}

void loop() {
    processPCInput();
    processRadioRX();
    handleTDMA();

    // Send status every 500ms
    if (millis() - last_status_time >= 500) {
        last_status_time = millis();
        sendStatus();
    }
}
