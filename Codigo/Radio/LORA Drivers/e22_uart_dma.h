/*
 * e22_uart_dma.h
 *
 * Clean E22 driver with minimal debug output
 */

#ifndef RADIO_LORA_DRIVERS_E22_UART_DMA_H_
#define RADIO_LORA_DRIVERS_E22_UART_DMA_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "config.h"

// Operating modes
#define E22_MODE_NORMAL  0  // M0=0, M1=0: TX/RX
#define E22_MODE_WOR     1  // M0=1, M1=0: Wake on Radio
#define E22_MODE_CONFIG  2  // M0=0, M1=1: Configuration
#define E22_MODE_SLEEP   3  // M0=1, M1=1: Sleep

// Error codes
typedef enum {
    E22_OK = 0,
    E22_ERR_BUSY,
    E22_ERR_TIMEOUT,
    E22_ERR_OVERFLOW,
    E22_ERR_INVALID_PARAM
} E22_Status_t;

// Statistics
typedef struct {
    uint32_t tx_packets;
    uint32_t tx_bytes;
    uint32_t rx_packets;
    uint32_t rx_bytes;
    uint32_t rx_overruns;
    uint32_t tx_failures;
} E22_Stats_t;

// Init/Config
bool E22_Init(UART_HandleTypeDef *huart);
void E22_SetMode(uint8_t mode);
bool E22_Reset(void);

// TX/RX
E22_Status_t E22_Transmit(uint8_t *data, uint16_t length);
uint16_t E22_Available(void);
int E22_Receive(uint8_t *buffer, uint16_t max_length);
void E22_FlushRx(void);

// Status
bool E22_IsBusy(void);
E22_Stats_t E22_GetStats(void);
void E22_ResetStats(void);

// DMA Callbacks
void E22_UART_TxCpltCallback(void);
void E22_UART_RxCpltCallback(void);
void E22_UART_ErrorCallback(void);

#endif /* RADIO_LORA_DRIVERS_E22_UART_DMA_H_ */
