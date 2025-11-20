/*
 * radio_thread.c
 *
 *  Created on: Nov 19, 2025
 *      Author: Tomas Teixeira
 *
 *  Radio thread for SX126x LoRa communication
 */

#include "radio_thread.h"
#include "Radio/LORA Drivers/lora_sx126x.h"
#include "Telemetry/telemetry.h"
#include "Radio/CRC16/crc16.h"
#include "cmsis_os.h"

#define RADIO_RX_BUFFER_SIZE 256
#define GS_COMM_ATTEMPTS 5
#define GS_COMM_TIMEOUT_MS 2000
#define QUEUE_RADIO_LENGTH 10

typedef struct __attribute__((packed)) {
    uint32_t time;
    command_t cmd;
    uint8_t payload[32];
    uint16_t crc16;
} command_packet_t;

static uint8_t rx_buffer[RADIO_RX_BUFFER_SIZE];
static bool gs_online = false;

QueueHandle_t queue_to_radio = NULL;

extern osThreadId_t radio_thread_id;
extern fsm_ctx_t fsm_ctx;

static bool ping_ground_station(void) {
    command_packet_t ping_pkt = {0};
    ping_pkt.time = HAL_GetTick();
    ping_pkt.cmd = CMD_PING;
    ping_pkt.crc16 = crc16_calculate((uint8_t*)&ping_pkt, sizeof(command_packet_t) - 2);

    if (!SX126x_TransmitDMA((uint8_t*)&ping_pkt, sizeof(command_packet_t))) {
        return false;
    }

    SX126x_WaitTxComplete(1000);

    SX126x_SetRx(GS_COMM_TIMEOUT_MS);

    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < GS_COMM_TIMEOUT_MS) {
        if (SX126x_Available()) {
            SX126x_RxInfo_t rx_info;
            int len = SX126x_Receive(rx_buffer, RADIO_RX_BUFFER_SIZE, &rx_info);

            if (len == sizeof(command_packet_t)) {
                command_packet_t *rx_pkt = (command_packet_t*)rx_buffer;
                uint16_t calc_crc = crc16_calculate(rx_buffer, sizeof(command_packet_t) - 2);

                if (calc_crc == rx_pkt->crc16 && rx_pkt->cmd == CMD_PING) {
                    printf("[RADIO] GS OK (RSSI=%d SNR=%d)\r\n", rx_info.rssi, rx_info.snr);
                    return true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

static bool init_radio_comm(void) {
    SX126x_LoRaConfig_t config = {
        .frequency_hz = 433000000,
        .spreading_factor = SX126X_LORA_SF7,
        .bandwidth = SX126X_LORA_BW_125,
        .coding_rate = SX126X_LORA_CR_4_5,
        .tx_power_dbm = 22,
        .preamble_length = 8,
        .sync_word = 0x1424,
        .enable_crc = true,
        .invert_iq = false
    };

    if (!SX126x_Init(&config)) {
        printf("[RADIO] INIT FAIL\r\n");
        return false;
    }

    printf("[RADIO] Init OK, ping GS...\r\n");

    for (int i = 0; i < GS_COMM_ATTEMPTS; i++) {
        if (ping_ground_station()) {
            gs_online = true;
            printf("[RADIO] GS online\r\n");
            return true;
        }
        printf("[RADIO] Ping %d/%d fail\r\n", i+1, GS_COMM_ATTEMPTS);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    printf("[RADIO] GS offline\r\n");
    return false;
}

static void process_command(command_packet_t *pkt) {
    switch (pkt->cmd) {
        case CMD_ARM:
            printf("[RADIO] CMD_ARM\r\n");
            break;

        case CMD_DISARM:
            printf("[RADIO] CMD_DISARM\r\n");
            break;

        case CMD_LAUNCH:
            printf("[RADIO] CMD_LAUNCH\r\n");
            break;

        case CMD_ABORT:
            printf("[RADIO] CMD_ABORT\r\n");
            break;

        case CMD_FORCE_SAFE:
            printf("[RADIO] CMD_FORCE_SAFE\r\n");
            break;

        default:
            break;
    }
}

void radio_thread_function() {
    radio_packet_t tx_pkt;

    printf("[RADIO] Thread started\r\n");

    queue_to_radio = xQueueCreate(QUEUE_RADIO_LENGTH, sizeof(radio_packet_t));
    if (queue_to_radio == NULL) {
        printf("[RADIO] Queue create FAIL\r\n");
        vTaskSuspend(NULL);
        return;
    }

    if (!init_radio_comm()) {
        printf("[RADIO] Init failed, suspending\r\n");
        vTaskSuspend(NULL);
        return;
    }

    SX126x_SetRx(0xFFFFFF);

    while(1) {
        if (xQueueReceive(queue_to_radio, &tx_pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (!SX126x_IsTxBusy()) {
                SX126x_TransmitDMA(tx_pkt.buffer, tx_pkt.length);
            }
        }

        if (SX126x_Available()) {
            SX126x_RxInfo_t rx_info;
            int len = SX126x_Receive(rx_buffer, RADIO_RX_BUFFER_SIZE, &rx_info);

            if (len == sizeof(command_packet_t)) {
                command_packet_t *pkt = (command_packet_t*)rx_buffer;
                uint16_t calc_crc = crc16_calculate(rx_buffer, sizeof(command_packet_t) - 2);

                if (calc_crc == pkt->crc16) {
                    process_command(pkt);
                } else {
                    printf("[RADIO] CRC fail\r\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool radio_is_gs_online(void) {
    return gs_online;
}
