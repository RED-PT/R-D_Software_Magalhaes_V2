/*
 * e22_uart_dma.h
 *
 *  Created on: Nov 24, 2025
 *      Author: Tomas Teixeira
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

// Config structure
typedef struct {
    uint8_t address_high;
    uint8_t address_low;
    uint8_t channel;        // 0-83 (862-931MHz)
    uint8_t air_data_rate;  // 0=2.4k, 1=4.8k, 2=9.6k, etc
    uint8_t tx_power;       // 0=22dBm, 1=17dBm, 2=13dBm, 3=10dBm
} E22_Config_t;

typedef struct {
    uint8_t buffer[256];
    uint16_t length;
    int16_t rssi;
} E22_Packet_t;

// Init/Config
bool E22_Init(UART_HandleTypeDef *huart);
void E22_SetMode(uint8_t mode);
bool E22_Configure(E22_Config_t *config);
bool E22_Reset(void);

// TX/RX
bool E22_Transmit(uint8_t *data, uint16_t length);
bool E22_Available(void);
int E22_Receive(uint8_t *buffer, uint16_t max_length);

// Status
bool E22_IsBusy(void);
int16_t E22_GetLastRSSI(void);

// DMA Callbacks (called from hal_callbacks.c)
void E22_UART_TxCpltCallback(void);
void E22_UART_RxCpltCallback(void);
void E22_UART_RxHalfCpltCallback(void);


#endif /* RADIO_LORA_DRIVERS_E22_UART_DMA_H_ */
