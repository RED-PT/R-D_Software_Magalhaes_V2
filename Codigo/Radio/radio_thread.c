/*
 * radio_thread.c
 *
 * Fixed: Added proper packet accumulation for RX
 */

#include "radio_thread.h"
#include "Radio/LORA Drivers/e22_uart_dma.h"
#include "Telemetry/telemetry.h"
#include "Radio/CRC16/crc16.h"
#include "cmsis_os.h"

#define QUEUE_RADIO_LENGTH 10

// RX packet accumulator
#define RX_PACKET_BUFFER_SIZE 64
static uint8_t rx_packet_buffer[RX_PACKET_BUFFER_SIZE];
static uint16_t rx_packet_idx = 0;
static uint32_t rx_last_byte_time = 0;
#define RX_PACKET_TIMEOUT_MS 100  // Reset if no data for 100ms

QueueHandle_t queue_to_radio = NULL;

extern osThreadId_t radio_thread_id;
extern fsm_ctx_t fsm_ctx;

// ============================================================================
// Get expected packet size based on packet type
// ============================================================================
static uint16_t get_expected_packet_size(uint8_t packet_type) {
    switch (packet_type) {
        case TELEM_PACKET_FAST:    return sizeof(telemetry_fast_t);
        case TELEM_PACKET_SLOW:    return sizeof(telemetry_slow_t);
        case TELEM_PACKET_EVENT:   return sizeof(telemetry_event_t);
        case TELEM_PACKET_COMMAND: return sizeof(command_packet_t);
        default: return 0;  // Unknown packet type
    }
}

// ============================================================================
// Send ACK
// ============================================================================
static void send_command_ack(uint8_t cmd, uint8_t ack_status, uint8_t current_state) {
    command_packet_t ack;
    memset(&ack, 0, sizeof(command_packet_t));

    ack.packet_type = TELEM_PACKET_COMMAND;
    ack.time = HAL_GetTick();
    ack.cmd = cmd;
    ack.ack_status = ack_status;
    ack.current_state = current_state;
    ack.crc16 = crc16_calculate((uint8_t*)&ack, sizeof(command_packet_t) - 2);

    printf("[RADIO] Sending ACK for cmd=%d status=%d\r\n", cmd, ack_status);

    // Wait for TX to be free
    uint32_t wait_start = HAL_GetTick();
    while (E22_IsBusy() && (HAL_GetTick() - wait_start < 100)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (E22_Transmit((uint8_t*)&ack, sizeof(command_packet_t))) {
        printf("[RADIO] ACK sent OK\r\n");
    } else {
        printf("[RADIO] ACK TX FAIL\r\n");
    }
}

// ============================================================================
// Process Command
// ============================================================================
static void process_command(command_packet_t *cmd) {
    printf("[RADIO] Processing command: CMD=%d\r\n", cmd->cmd);

    // Verify CRC
    uint16_t calc_crc = crc16_calculate((uint8_t*)cmd, sizeof(command_packet_t) - 2);

    if (calc_crc != cmd->crc16) {
        printf("[RADIO] CRC FAIL! calc=0x%04X recv=0x%04X\r\n", calc_crc, cmd->crc16);
        return;
    }

    printf("[RADIO] CRC OK! CMD=%d\r\n", cmd->cmd);

    // TODO: Actually process the command (queue to FSM, etc.)
    // For now, just send ACK
    send_command_ack(cmd->cmd, 0, fsm_ctx.state);
}

// ============================================================================
// Process received byte - accumulates into packet buffer
// Returns true if a complete packet was processed
// ============================================================================
static bool process_rx_byte(uint8_t byte) {
    uint32_t now = HAL_GetTick();

    // Timeout: reset if too long since last byte
    if (rx_packet_idx > 0 && (now - rx_last_byte_time) > RX_PACKET_TIMEOUT_MS) {
        printf("[RADIO] RX timeout, resetting buffer (had %u bytes)\r\n", rx_packet_idx);
        rx_packet_idx = 0;
    }

    rx_last_byte_time = now;

    // First byte: check if it's a valid packet type
    if (rx_packet_idx == 0) {
        uint16_t expected = get_expected_packet_size(byte);
        if (expected == 0) {
            // Invalid packet type, discard
            printf("[RADIO] Invalid packet type: 0x%02X\r\n", byte);
            return false;
        }
    }

    // Store byte
    if (rx_packet_idx < RX_PACKET_BUFFER_SIZE) {
        rx_packet_buffer[rx_packet_idx++] = byte;
    } else {
        // Buffer overflow, reset
        printf("[RADIO] RX buffer overflow!\r\n");
        rx_packet_idx = 0;
        return false;
    }

    // Check if we have a complete packet
    uint8_t packet_type = rx_packet_buffer[0];
    uint16_t expected_size = get_expected_packet_size(packet_type);

    if (rx_packet_idx >= expected_size) {
        // Complete packet received!
        printf("[RADIO] Complete packet: type=0x%02X size=%u\r\n", packet_type, rx_packet_idx);

        // Process based on type
        if (packet_type == TELEM_PACKET_COMMAND) {
            command_packet_t *cmd = (command_packet_t*)rx_packet_buffer;
            process_command(cmd);
        } else {
            printf("[RADIO] Ignoring non-command packet type=0x%02X\r\n", packet_type);
        }

        // Reset for next packet
        rx_packet_idx = 0;
        return true;
    }

    return false;
}

// ============================================================================
// Radio Thread
// ============================================================================
void radio_thread_function() {
    radio_packet_t tx_pkt;
    uint32_t tx_count = 0;
    uint32_t rx_count = 0;
    uint32_t loop_count = 0;

    printf("[RADIO] Thread started\r\n");

    // Create TX queue
    queue_to_radio = xQueueCreate(QUEUE_RADIO_LENGTH, sizeof(radio_packet_t));
    if (!queue_to_radio) {
        printf("[RADIO] Queue creation FAILED!\r\n");
        vTaskSuspend(NULL);
    }
    printf("[RADIO] Queue created\r\n");

    // Initialize E22
    if (!E22_Init(UART_RADIO)) {
        printf("[RADIO] E22 Init FAILED!\r\n");
        vTaskSuspend(NULL);
    }

    printf("[RADIO] E22 Init OK\r\n");
    printf("[RADIO] Packet sizes: CMD=%u FAST=%u SLOW=%u\r\n",
           sizeof(command_packet_t), sizeof(telemetry_fast_t), sizeof(telemetry_slow_t));
    printf("[RADIO] Loop starting...\r\n");

    // Initialize RX state
    rx_packet_idx = 0;
    rx_last_byte_time = HAL_GetTick();

    while(1) {
        loop_count++;

        // Stats every 5 seconds
        if (loop_count % 500 == 0) {
            printf("[RADIO] Loop %lu | TX=%lu RX=%lu\r\n", loop_count, tx_count, rx_count);
        }

        // ========================================
        // TX: Send telemetry packets from queue
        // ========================================
        if (xQueueReceive(queue_to_radio, &tx_pkt, 0) == pdTRUE) {
            // Wait for TX to be free
            uint32_t wait_start = HAL_GetTick();
            while (E22_IsBusy() && (HAL_GetTick() - wait_start < 100)) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (E22_Transmit(tx_pkt.buffer, tx_pkt.length)) {
                tx_count++;
            } else {
                printf("[RADIO] TX FAIL\r\n");
            }
        }

        // ========================================
        // RX: Process incoming bytes one at a time
        // ========================================
        if (E22_Available()) {
            uint8_t temp_buf[64];
            int bytes_read = E22_Receive(temp_buf, sizeof(temp_buf));

            if (bytes_read > 0) {
                // Process each byte through the packet accumulator
                for (int i = 0; i < bytes_read; i++) {
                    if (process_rx_byte(temp_buf[i])) {
                        rx_count++;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
