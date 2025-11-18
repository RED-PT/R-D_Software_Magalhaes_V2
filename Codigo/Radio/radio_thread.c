/*
 * radio_thread.c
 *
 *  Created on: Nov 18, 2025
 *      Author: Tomas Teixeira
 */

#include "radio_thread.h"
#include "cmsis_os.h"

void radio_thread_function() {
    radio_tx_packet_t tx_packet;
    uint8_t rx_buffer[256];
    LoRa_RxInfo_t rx_info;
    command_packet_t cmd;

    // LoRa configuration
    LoRa_Config_t lora_config = {
        .frequency_hz = 868000000,
        .spreading_factor = LORA_SF_7,
        .bandwidth = LORA_BW_125_KHZ,
        .coding_rate = LORA_CR_4_5,
        .tx_power_dbm = 17,
        .preamble_length = 8,
        .sync_word = 0x12,  // Private network
        .enable_crc = true,
        .implicit_header = false
    };

    printf("Radio Thread started...\r\n");

    // Initialize LoRa
    if (!LoRa_Init(&lora_config)) {
        printf("ERROR: LoRa Init FAILED!\r\n");
        vTaskSuspend(NULL);
    }
    printf("LoRa Initialized successfully\r\n");

    // Start in RX mode - ALWAYS listening for commands
    LoRa_SetModeRx();

    uint32_t rx_count = 0;
    uint32_t tx_count = 0;

    while(1) {
        // PRIORITY 1: Check for incoming commands (RX is CRITICAL)
        if (LoRa_Available()) {
            int rx_len = LoRa_Receive(rx_buffer, sizeof(rx_buffer), &rx_info);

            if (rx_len > 0) {
                rx_count++;

                // Parse command packet
                if (parse_command(rx_buffer, rx_len, &cmd)) {
                    // Send to FSM thread immediately
                    if (xQueueSend(queue_radio_rx_to_fsm, &cmd, 0) == pdTRUE) {
                        printf("RX CMD: %d, RSSI: %d dBm, SNR: %d dB [#%lu]\r\n",
                               cmd.command, rx_info.rssi, rx_info.snr, rx_count);
                    } else {
                        printf("ERROR: FSM queue full!\r\n");
                    }
                } else {
                    printf("WARN: Command parse failed (CRC or format error)\r\n");
                }
            } else if (rx_len == -1) {
                printf("WARN: RX CRC error\r\n");
            }

            // CRITICAL: Return to RX mode immediately
            LoRa_SetModeRx();
        }

        // PRIORITY 2: Check for telemetry to transmit
        if (xQueueReceive(queue_to_radio_tx, &tx_packet, 0) == pdTRUE) {
            tx_count++;

            // Transmit packet
            if (LoRa_TransmitDMA(tx_packet.data, tx_packet.length)) {
                // Wait for TX complete (max 100ms)
                LoRa_WaitTxComplete(100);

                if (LoRa_IsTxDone()) {
                    // Log only important packets
                    if (tx_packet.priority == 0) {
                        printf("TX EVENT sent [#%lu]\r\n", tx_count);
                    }
                } else {
                    printf("WARN: TX timeout\r\n");
                }
            } else {
                printf("WARN: TX failed (busy)\r\n");
            }

            // CRITICAL: Return to RX mode immediately
            LoRa_SetModeRx();
        }

        // Check RX frequently (100Hz = 10ms period)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool parse_command(uint8_t *data, uint8_t length, command_packet_t *cmd) {
    // Minimum packet: [CMD_TYPE][CRC16_HIGH][CRC16_LOW] = 3 bytes
    if (length < 3) {
        return false;
    }

    // Verify CRC16 (last 2 bytes)
    uint16_t received_crc = (data[length-2] << 8) | data[length-1];
    uint16_t calculated_crc = calculate_crc16(data, length - 2);

    if (received_crc != calculated_crc) {
        return false;
    }

    // Parse command type
    cmd->command = (command_t)data[0];
    cmd->payload_length = length - 3;  // Subtract: CMD(1) + CRC(2)

    // Copy payload if present
    if (cmd->payload_length > 0) {
        if (cmd->payload_length > sizeof(cmd->payload)) {
            cmd->payload_length = sizeof(cmd->payload);  // Truncate
        }
        memcpy(cmd->payload, &data[1], cmd->payload_length);
    }

    return true;
}
