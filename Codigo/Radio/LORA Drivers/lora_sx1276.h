/*
 * lora_sx1276.h
 *
 *  Created on: Nov 4, 2025
 *      Author: texman
 */

#ifndef RADIO_LORA_DRIVERS_LORA_SX1276_H_
#define RADIO_LORA_DRIVERS_LORA_SX1276_H_

#include "config.h"
#include "defs.h"
#include <stdbool.h>
#include <stdint.h>

// Register Map
// Common registers
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_PA_RAMP                 0x0A
#define REG_OCP                     0x0B
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_MODEM_CONFIG_3          0x26
#define REG_FREQ_ERROR_MSB          0x28
#define REG_FREQ_ERROR_MID          0x29
#define REG_FREQ_ERROR_LSB          0x2A
#define REG_RSSI_WIDEBAND           0x2C
#define REG_DETECTION_OPTIMIZE      0x31
#define REG_INVERTIQ                0x33
#define REG_DETECTION_THRESHOLD     0x37
#define REG_SYNC_WORD               0x39
#define REG_INVERTIQ2               0x3B
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42
#define REG_PA_DAC                  0x4D

// Operating modes
#define MODE_LONG_RANGE_MODE        0x80
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RX_CONTINUOUS          0x05
#define MODE_RX_SINGLE              0x06

// IRQ flags
#define IRQ_TX_DONE_MASK            0x08
#define IRQ_RX_DONE_MASK            0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK  0x20
#define IRQ_CAD_DONE_MASK           0x04
#define IRQ_CAD_DETECTED_MASK       0x01

// PA config
#define PA_BOOST                    0x80
#define PA_OUTPUT_RFO_PIN           0x00
#define PA_OUTPUT_PA_BOOST_PIN      0x8

// Max settings
#define MAX_PKT_LENGTH              255

//Structures
typedef enum {
    LORA_BW_7_8_KHZ = 0,
    LORA_BW_10_4_KHZ,
    LORA_BW_15_6_KHZ,
    LORA_BW_20_8_KHZ,
    LORA_BW_31_25_KHZ,
    LORA_BW_41_7_KHZ,
    LORA_BW_62_5_KHZ,
    LORA_BW_125_KHZ,
    LORA_BW_250_KHZ,
    LORA_BW_500_KHZ
} LoRa_Bandwidth_e;

typedef enum {
    LORA_SF_6 = 6,
    LORA_SF_7,
    LORA_SF_8,
    LORA_SF_9,
    LORA_SF_10,
    LORA_SF_11,
    LORA_SF_12
} LoRa_SpreadingFactor_e;

typedef enum {
    LORA_CR_4_5 = 1,  // 4/5
    LORA_CR_4_6,      // 4/6
    LORA_CR_4_7,      // 4/7
    LORA_CR_4_8       // 4/8
} LoRa_CodingRate_e;

typedef struct {
    uint32_t frequency_hz;              // Center frequency (e.g., 868000000)
    LoRa_SpreadingFactor_e spreading_factor;
    LoRa_Bandwidth_e bandwidth;
    LoRa_CodingRate_e coding_rate;
    uint8_t tx_power_dbm;               // 2-20 dBm
    uint16_t preamble_length;           // Typically 8
    uint8_t sync_word;                  // 0x12 = private, 0x34 = LoRaWAN
    bool enable_crc;
    bool implicit_header;
} LoRa_Config_t;

typedef struct {
    int16_t rssi;           // RSSI in dBm
    int8_t snr;             // SNR in dB
    uint8_t length;         // Packet length
} LoRa_RxInfo_t;

// Function Prototypes
bool LoRa_Init(LoRa_Config_t *config);
bool LoRa_Reset(void);
bool LoRa_CheckVersion(void);
void LoRa_SetConfig(LoRa_Config_t *config);

// Operating mode control
void LoRa_Sleep(void);
void LoRa_Standby(void);
void LoRa_SetModeRx(void);
void LoRa_SetModeTx(void);

// Transmit functions
bool LoRa_TransmitDMA(uint8_t *data, uint8_t length);
bool LoRa_IsTxBusy(void);
bool LoRa_IsTxDone(void);
void LoRa_WaitTxComplete(uint32_t timeout_ms);

// Receive functions
bool LoRa_Available(void);
int LoRa_Receive(uint8_t *buffer, uint8_t max_length, LoRa_RxInfo_t *rx_info);
bool LoRa_ReceiveDMA(uint8_t *buffer, uint8_t max_length);

// Signal quality
int16_t LoRa_GetRssi(void);
int8_t LoRa_GetSnr(void);

// Low-level SPI functions
void LoRa_WriteReg(uint8_t reg, uint8_t value);
uint8_t LoRa_ReadReg(uint8_t reg);
void LoRa_WriteRegDMA(uint8_t reg, uint8_t value);

// Interrupt handler (call from EXTI callback)
void LoRa_DIO0_IRQ_Handler(void);  // DIO0 interrupt (RX/TX done)
void LoRa_SPI_TxCpltCallback(void); // SPI DMA TX complete
void LoRa_SPI_RxCpltCallback(void); // SPI DMA RX complete

#endif /* RADIO_LORA_DRIVERS_LORA_SX1276_H_ */
