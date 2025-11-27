#include "e22_uart_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *e22_uart = NULL;

#define RX_BUFFER_SIZE 512
static uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_read_pos = 0;

static volatile bool tx_busy = false;
static uint8_t tx_buffer[256];

static int16_t last_rssi = -999;

// Use pinos do config.h
#define M0_E22_PORT RADIO_M0_PORT
#define M0_E22_PIN RADIO_M0_PIN
#define M1_E22_PORT RADIO_M1_PORT
#define M1_E22_PIN RADIO_M1_PIN

#define M0_LOW()  HAL_GPIO_WritePin(M0_E22_PORT, M0_E22_PIN, GPIO_PIN_RESET)
#define M0_HIGH() HAL_GPIO_WritePin(M0_E22_PORT, M0_E22_PIN, GPIO_PIN_SET)
#define M1_LOW()  HAL_GPIO_WritePin(M1_E22_PORT, M1_E22_PIN, GPIO_PIN_RESET)
#define M1_HIGH() HAL_GPIO_WritePin(M1_E22_PORT, M1_E22_PIN, GPIO_PIN_SET)

void E22_SetMode(uint8_t mode) {
    switch(mode) {
        case E22_MODE_NORMAL: M0_LOW(); M1_LOW(); break;
        case E22_MODE_WOR: M0_HIGH(); M1_LOW(); break;
        case E22_MODE_CONFIG: M0_LOW(); M1_HIGH(); break;
        case E22_MODE_SLEEP: M0_HIGH(); M1_HIGH(); break;
    }
    HAL_Delay(100);
}

bool E22_Init(UART_HandleTypeDef *huart) {
    e22_uart = huart;
    rx_read_pos = 0;
    tx_busy = false;

    printf("[E22] Forcing Normal Mode...\r\n");

    // FORÇA modo normal!
    HAL_GPIO_WritePin(RADIO_M0_PORT, RADIO_M0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RADIO_M1_PORT, RADIO_M1_PIN, GPIO_PIN_RESET);
    HAL_Delay(500);

    printf("[E22] M0=0, M1=0\r\n");

    HAL_UART_Receive_DMA(e22_uart, rx_dma_buffer, RX_BUFFER_SIZE);

    printf("[E22] Init complete\r\n");
    return true;
}

bool E22_Reset(void) {
    printf("[E22] Resetting...\r\n");
    E22_SetMode(E22_MODE_SLEEP);
    HAL_Delay(200);
    E22_SetMode(E22_MODE_NORMAL);
    HAL_Delay(200);

    HAL_UART_AbortReceive(e22_uart);
    rx_read_pos = 0;
    HAL_UART_Receive_DMA(e22_uart, rx_dma_buffer, RX_BUFFER_SIZE);

    return true;
}

bool E22_Configure(E22_Config_t *config) {
    if (!config) return false;

    printf("[E22] Config: ADDR=%02X%02X CH=%d\r\n",
           config->address_high, config->address_low, config->channel);

    E22_SetMode(E22_MODE_CONFIG);
    HAL_Delay(200);

    // Escrever SÓ os registos essenciais
    uint8_t cfg[6];
    cfg[0] = 0xC0;  // Write command
    cfg[1] = 0x00;  // Start ADDH register
    cfg[2] = 0x03;  // Write 3 bytes
    cfg[3] = config->address_high;  // REG0: ADDH
    cfg[4] = config->address_low;   // REG1: ADDL
    cfg[5] = config->channel;       // REG5: Channel

    // Enviar via blocking (mais confiável para config)
    HAL_UART_Transmit(e22_uart, cfg, 6, 1000);
    HAL_Delay(200);

    E22_SetMode(E22_MODE_NORMAL);
    HAL_Delay(200);

    printf("[E22] Config done\r\n");
    return true;
}

bool E22_Transmit(uint8_t *data, uint16_t length) {
    if (!data || length == 0 || length > 240 || tx_busy) {
        return false;
    }

    memcpy(tx_buffer, data, length);

    tx_busy = true;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(e22_uart, tx_buffer, length);

    if (status != HAL_OK) {
        tx_busy = false;
        return false;
    }

    return true;
}

void E22_UART_TxCpltCallback(void) {
    tx_busy = false;
}

void E22_UART_RxHalfCpltCallback(void) {
}

void E22_UART_RxCpltCallback(void) {
}

bool E22_Available(void) {
    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(e22_uart->hdmarx);
    uint16_t dma_pos = RX_BUFFER_SIZE - dma_remaining;

    // Debug
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print > 5000) {
        printf("[E22] DMA: remaining=%u, pos=%u, read=%u\r\n",
               dma_remaining, dma_pos, rx_read_pos);
        last_print = HAL_GetTick();
    }

    return (dma_pos != rx_read_pos);
}

int E22_Receive(uint8_t *buffer, uint16_t max_length) {
    if (!buffer || max_length == 0) return 0;

    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(e22_uart->hdmarx);
    uint16_t dma_pos = RX_BUFFER_SIZE - dma_remaining;

    if (dma_pos == rx_read_pos) return 0;

    uint16_t count = 0;

    while (rx_read_pos != dma_pos && count < max_length) {
        buffer[count++] = rx_dma_buffer[rx_read_pos];
        rx_read_pos++;
        if (rx_read_pos >= RX_BUFFER_SIZE) rx_read_pos = 0;
    }

    return count;
}

bool E22_IsBusy(void) {
    return tx_busy;
}

int16_t E22_GetLastRSSI(void) {
    return last_rssi;
}
