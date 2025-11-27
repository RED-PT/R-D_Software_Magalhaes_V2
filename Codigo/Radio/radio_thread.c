/*
 * radio_thread.c
 *
 *  Simple radio thread - just transmits telemetry
 */

#include "radio_thread.h"
#include "Radio/LORA Drivers/e22_uart_dma.h"
#include "Telemetry/telemetry.h"
#include "Radio/CRC16/crc16.h"
#include "cmsis_os.h"

#define QUEUE_RADIO_LENGTH 10
#define RX_BUFFER_SIZE 256

QueueHandle_t queue_to_radio = NULL;

extern osThreadId_t radio_thread_id;
extern fsm_ctx_t fsm_ctx;

static uint8_t rx_buffer[RX_BUFFER_SIZE];

static void send_command_response(command_t cmd) {
    command_packet_t response;
    memset(&response, 0, sizeof(command_packet_t));

    response.packet_type = TELEM_PACKET_COMMAND;
    response.time = HAL_GetTick();
    response.cmd = cmd;
    response.crc16 = crc16_calculate((uint8_t*)&response, sizeof(command_packet_t) - 2);

    // Send directly (bypass queue for commands)
    while (E22_IsBusy()) vTaskDelay(pdMS_TO_TICKS(1));
    E22_Transmit((uint8_t*)&response, sizeof(command_packet_t));
}

static void process_command(command_packet_t *cmd) {
    // Verify CRC
    uint16_t calc_crc = crc16_calculate((uint8_t*)cmd, sizeof(command_packet_t) - 2);
    if (calc_crc != cmd->crc16) {
        printf("[RADIO] Bad CRC: %04X != %04X\r\n", calc_crc, cmd->crc16);
        return;
    }

    printf("[RADIO] RX: ");

    switch(cmd->cmd) {
        case CMD_PING:
            printf("PING -> sending PONG\r\n");
            send_command_response(CMD_PING);  // Send PING back as acknowledgment
            break;

        case CMD_ARM:
            printf("ARM (queued for FSM)\r\n");
            // TODO: When FSM ready, queue command
            send_command_response(CMD_ARM);
            break;

        case CMD_DISARM:
            printf("DISARM (queued for FSM)\r\n");
            send_command_response(CMD_DISARM);
            break;

        case CMD_START_TEST:
            printf("START_TEST (queued for FSM)\r\n");
            send_command_response(CMD_START_TEST);
            break;

        case CMD_LAUNCH:
            printf("LAUNCH (queued for FSM)\r\n");
            send_command_response(CMD_LAUNCH);
            break;

        case CMD_ABORT:
            printf("ABORT (queued for FSM)\r\n");
            send_command_response(CMD_ABORT);
            break;

        case CMD_FORCE_SAFE:
            printf("FORCE_SAFE (queued for FSM)\r\n");
            send_command_response(CMD_FORCE_SAFE);
            break;

        case CMD_SET_PROFILE:
            printf("SET_PROFILE (queued for FSM)\r\n");
            send_command_response(CMD_SET_PROFILE);
            break;

        case CMD_SET_TARGET_ALT:
            printf("SET_TARGET_ALT (queued for FSM)\r\n");
            send_command_response(CMD_SET_TARGET_ALT);
            break;

        default:
            printf("UNKNOWN (%d)\r\n", cmd->cmd);
            break;
    }
}

static bool init_radio(void) {
    printf("[RADIO] Initializing E22-900T22S...\r\n");

    if (!E22_Init(UART_RADIO)) {
        printf("[RADIO] E22 Init FAIL\r\n");
        return false;
    }

    printf("[RADIO] E22 OK (using current config)\r\n");
    return true;
}

void radio_thread_function() {
    radio_packet_t tx_pkt;
    uint32_t tx_count = 0;
    uint32_t rx_count = 0;
    uint32_t rx_check_count = 0;  // Contador de checks

    uint8_t packet_buffer[sizeof(command_packet_t) * 2]; // Espaço para acumular
    uint16_t packet_len = 0;

    printf("[RADIO] Thread started\r\n");

    queue_to_radio = xQueueCreate(QUEUE_RADIO_LENGTH, sizeof(radio_packet_t));
    if (queue_to_radio == NULL) {
        printf("[RADIO] Queue create FAIL\r\n");
        vTaskSuspend(NULL);
        return;
    }

    if (!init_radio()) {
        printf("[RADIO] Init failed\r\n");
        vTaskSuspend(NULL);
        return;
    }

    printf("[RADIO] TX/RX loop started\r\n");

    while(1) {
        // TX: Send telemetry
        if (xQueueReceive(queue_to_radio, &tx_pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint32_t wait_start = HAL_GetTick();
            while (E22_IsBusy() && (HAL_GetTick() - wait_start < 100)) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (!E22_IsBusy()) {
                if (E22_Transmit(tx_pkt.buffer, tx_pkt.length)) {
                    tx_count++;
                    if (tx_count % 20 == 0) {
                        printf("[RADIO] TX: %lu, RX: %lu\r\n", tx_count, rx_count);
                    }
                }
            }
        }

        // RX: Check for incoming commands
        rx_check_count++;

        // Debug every 500 checks (~5s)
        if (rx_check_count % 500 == 0) {
            if (E22_Available()) {
                printf("[RADIO] RX available!\r\n");
            } else {
                printf("[RADIO] No RX data (checked %lu times)\r\n", rx_check_count);
            }
        }

        if (E22_Available()) {
            // 1. Ler o que chegou para um buffer temporário
            int len = E22_Receive(rx_buffer, RX_BUFFER_SIZE);

            // 2. Adicionar ao nosso acumulador (protegendo contra overflow)
            if (packet_len + len <= sizeof(packet_buffer)) {
                memcpy(&packet_buffer[packet_len], rx_buffer, len);
                packet_len += len;
            } else {
                // Buffer cheio e sem pacote válido? Reset para evitar travamento
                packet_len = 0;
                printf("[RADIO] Buffer overflow, resetting RX\r\n");
            }

            // 3. Verificar se já temos pelo menos um pacote inteiro (40 bytes)
            if (packet_len >= sizeof(command_packet_t)) {

                command_packet_t *cmd = (command_packet_t*)packet_buffer;

                // Verificação simples de sincronia (O primeiro byte deve ser 0x10)
                if (cmd->packet_type == TELEM_PACKET_COMMAND) {

                    // Temos um pacote válido!
                    printf("[RADIO] Command received! Processing...\r\n");
                    process_command(cmd); // Processa o comando

                    // 4. Limpar o pacote processado do buffer (Shift left)
                    uint16_t remaining = packet_len - sizeof(command_packet_t);
                    if (remaining > 0) {
                        memmove(packet_buffer, &packet_buffer[sizeof(command_packet_t)], remaining);
                    }
                    packet_len = remaining;

                } else {
                    // Perdemos a sincronia (o primeiro byte não é o cabeçalho)
                    // Descartar 1 byte e tentar encontrar o cabeçalho no próximo loop
                    printf("[RADIO] Sync lost (byte 0x%02X), shifting...\r\n", packet_buffer[0]);
                    packet_len--;
                    memmove(packet_buffer, &packet_buffer[1], packet_len);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
