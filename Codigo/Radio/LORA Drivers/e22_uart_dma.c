/*
 * e22_uart_dma.c
 *
 * Clean E22 UART driver - minimal debug output, production ready
 * Only compiled when RADIO_INTERFACE_UART is defined (F446ZE dev board).
 * The Buzz V4 (H743) uses SPI directly to the SX1262.
 */

#include "config.h"

#ifdef RADIO_INTERFACE_UART

#include "e22_uart_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static UART_HandleTypeDef *e22_uart = NULL;

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 128
#define E22_MAX_PACKET_SIZE 64

// Circular RX buffer
static uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_read_pos = 0;

// TX buffer
static volatile bool tx_busy = false;
static uint8_t tx_buffer[TX_BUFFER_SIZE];

// Statistics
static E22_Stats_t stats = {0};

// Mode control macros
#define M0_LOW()  HAL_GPIO_WritePin(RADIO_M0_PORT, RADIO_M0_PIN, GPIO_PIN_RESET)
#define M0_HIGH() HAL_GPIO_WritePin(RADIO_M0_PORT, RADIO_M0_PIN, GPIO_PIN_SET)
#define M1_LOW()  HAL_GPIO_WritePin(RADIO_M1_PORT, RADIO_M1_PIN, GPIO_PIN_RESET)
#define M1_HIGH() HAL_GPIO_WritePin(RADIO_M1_PORT, RADIO_M1_PIN, GPIO_PIN_SET)

void E22_SetMode(uint8_t mode) {
    switch(mode) {
        case E22_MODE_NORMAL: M0_LOW(); M1_LOW(); break;
        case E22_MODE_WOR:    M0_HIGH(); M1_LOW(); break;
        case E22_MODE_CONFIG: M0_LOW(); M1_HIGH(); break;
        case E22_MODE_SLEEP:  M0_HIGH(); M1_HIGH(); break;
    }
    HAL_Delay(50);  // Mode switch settling time
}

bool E22_Init(UART_HandleTypeDef *huart) {
    if (!huart) return false;

    e22_uart = huart;
    rx_read_pos = 0;
    tx_busy = false;
    memset(&stats, 0, sizeof(stats));

    // Force normal mode
    M0_LOW();
    M1_LOW();
    HAL_Delay(200);

    // Start circular DMA reception
    HAL_UART_Receive_DMA(e22_uart, rx_dma_buffer, RX_BUFFER_SIZE);

    return true;
}

bool E22_Reset(void) {
    E22_SetMode(E22_MODE_SLEEP);
    HAL_Delay(100);
    E22_SetMode(E22_MODE_NORMAL);
    HAL_Delay(100);

    // Restart DMA
    HAL_UART_AbortReceive(e22_uart);
    rx_read_pos = 0;
    HAL_UART_Receive_DMA(e22_uart, rx_dma_buffer, RX_BUFFER_SIZE);

    return true;
}

E22_Status_t E22_Transmit(uint8_t *data, uint16_t length) {
    if (!data || length == 0) return E22_ERR_INVALID_PARAM;
    if (length > E22_MAX_PACKET_SIZE) return E22_ERR_INVALID_PARAM;
    if (tx_busy) return E22_ERR_BUSY;

    memcpy(tx_buffer, data, length);
    tx_busy = true;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(e22_uart, tx_buffer, length);

    if (status != HAL_OK) {
        tx_busy = false;
        stats.tx_failures++;
        return E22_ERR_TIMEOUT;
    }

    stats.tx_packets++;
    stats.tx_bytes += length;
    return E22_OK;
}

void E22_UART_TxCpltCallback(void) {
    tx_busy = false;
}

void E22_UART_ErrorCallback(void) {
    // Clear error flags and restart DMA
    __HAL_UART_CLEAR_OREFLAG(e22_uart);
    __HAL_UART_CLEAR_NEFLAG(e22_uart);
    __HAL_UART_CLEAR_FEFLAG(e22_uart);

    stats.rx_overruns++;

    HAL_UART_AbortReceive(e22_uart);
    rx_read_pos = 0;
    HAL_UART_Receive_DMA(e22_uart, rx_dma_buffer, RX_BUFFER_SIZE);
}

uint16_t E22_Available(void) {
    // Check for UART errors
    if (__HAL_UART_GET_FLAG(e22_uart, UART_FLAG_ORE)) {
        E22_UART_ErrorCallback();
        return 0;
    }

    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(e22_uart->hdmarx);
    uint16_t dma_pos = RX_BUFFER_SIZE - dma_remaining;

    if (dma_pos >= rx_read_pos) {
        return dma_pos - rx_read_pos;
    } else {
        return RX_BUFFER_SIZE - rx_read_pos + dma_pos;
    }
}

int E22_Receive(uint8_t *buffer, uint16_t max_length) {
    if (!buffer || max_length == 0) return 0;

    uint16_t available = E22_Available();
    if (available == 0) return 0;

    uint16_t to_read = (available < max_length) ? available : max_length;
    uint16_t count = 0;

    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(e22_uart->hdmarx);
    uint16_t dma_pos = RX_BUFFER_SIZE - dma_remaining;

    while (rx_read_pos != dma_pos && count < to_read) {
        buffer[count++] = rx_dma_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % RX_BUFFER_SIZE;
    }

    if (count > 0) {
        stats.rx_packets++;
        stats.rx_bytes += count;
    }

    return count;
}

void E22_FlushRx(void) {
    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(e22_uart->hdmarx);
    rx_read_pos = RX_BUFFER_SIZE - dma_remaining;
}

bool E22_IsBusy(void) {
    return tx_busy;
}

E22_Stats_t E22_GetStats(void) {
    return stats;
}

void E22_ResetStats(void) {
    memset(&stats, 0, sizeof(stats));
}

void E22_UART_RxCpltCallback(void) {
    // Circular DMA - nothing to do
}

#endif /* RADIO_INTERFACE_UART */
