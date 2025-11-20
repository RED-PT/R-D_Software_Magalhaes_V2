/*
 * sx126x.h
 *
 *  Created on: Nov 19, 2025
 *      Author: Tomas Teixeira
 *
 *  SX126x LoRa Driver for STM32F446ZE
 */

#ifndef RADIO_SX126X_H_
#define RADIO_SX126X_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "defs.h"

// ============================================================================
// Pin Mapping from config.h
// ============================================================================
// SPI2: LoRa SPI interface
// CS:    PG1  (CS_LORA_PORT/PIN)
// RESET: PF9  (RESET_LORA_PORT/PIN)
// BUSY:  PG0  (BUSY_LORA_PORT/PIN)
// DIO1:  PB2  (EXTI_LORA_PORT/PIN)

// Use macros from config.h
#define SX126X_CS_LOW()      CS_LORA_LOW()
#define SX126X_CS_HIGH()     CS_LORA_HIGH()
#define SX126X_RESET_LOW()   RESET_LORA_LOW()
#define SX126X_RESET_HIGH()  RESET_LORA_HIGH()
#define SX126X_IS_BUSY()     LORA_IS_BUSY()

#define SX126X_DIO1_PORT     EXTI_LORA_PORT
#define SX126X_DIO1_PIN      EXTI_LORA_PIN

// SPI Handle
#define SPI_SX126X           SPI_LORA
#define SPI_SX126X_INSTANCE  SPI_LORA_INSTANCE

// ============================================================================
// SX126x Commands
// ============================================================================

// Operational Modes Commands
#define SX126X_CMD_SET_SLEEP                0x84
#define SX126X_CMD_SET_STANDBY              0x80
#define SX126X_CMD_SET_FS                   0xC1
#define SX126X_CMD_SET_TX                   0x83
#define SX126X_CMD_SET_RX                   0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE   0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE        0x94
#define SX126X_CMD_SET_CAD                  0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE   0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE 0xD2
#define SX126X_CMD_SET_REGULATOR_MODE       0x96
#define SX126X_CMD_CALIBRATE                0x89
#define SX126X_CMD_CALIBRATE_IMAGE          0x98
#define SX126X_CMD_SET_PA_CONFIG            0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE  0x93

// Register and Buffer Access Commands
#define SX126X_CMD_WRITE_REGISTER           0x0D
#define SX126X_CMD_READ_REGISTER            0x1D
#define SX126X_CMD_WRITE_BUFFER             0x0E
#define SX126X_CMD_READ_BUFFER              0x1E

// DIO and IRQ Control Commands
#define SX126X_CMD_SET_DIO_IRQ_PARAMS       0x08
#define SX126X_CMD_GET_IRQ_STATUS           0x12
#define SX126X_CMD_CLR_IRQ_STATUS           0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH    0x9D
#define SX126X_CMD_SET_DIO3_AS_TCXO_CTRL    0x97

// RF Modulation and Packet Commands
#define SX126X_CMD_SET_RF_FREQUENCY         0x86
#define SX126X_CMD_SET_PKT_TYPE             0x8A
#define SX126X_CMD_GET_PKT_TYPE             0x11
#define SX126X_CMD_SET_TX_PARAMS            0x8E
#define SX126X_CMD_SET_MODULATION_PARAMS    0x8B
#define SX126X_CMD_SET_PKT_PARAMS           0x8C
#define SX126X_CMD_SET_CAD_PARAMS           0x88
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS  0x8F
#define SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

// Communication Status and Information Commands
#define SX126X_CMD_GET_STATUS               0xC0
#define SX126X_CMD_GET_RX_BUFFER_STATUS     0x13
#define SX126X_CMD_GET_PKT_STATUS           0x14
#define SX126X_CMD_GET_RSSI_INST            0x15
#define SX126X_CMD_GET_STATS                0x10
#define SX126X_CMD_RESET_STATS              0x00

// ============================================================================
// SX126x Registers
// ============================================================================

#define SX126X_REG_WHITENING_INITIAL        0x06B8
#define SX126X_REG_CRC_INITIAL              0x06BC
#define SX126X_REG_CRC_POLYNOMIAL           0x06BE
#define SX126X_REG_SYNC_WORD_0              0x06C0
#define SX126X_REG_SYNC_WORD_1              0x06C1
#define SX126X_REG_SYNC_WORD_2              0x06C2
#define SX126X_REG_SYNC_WORD_3              0x06C3
#define SX126X_REG_SYNC_WORD_4              0x06C4
#define SX126X_REG_SYNC_WORD_5              0x06C5
#define SX126X_REG_SYNC_WORD_6              0x06C6
#define SX126X_REG_SYNC_WORD_7              0x06C7
#define SX126X_REG_NODE_ADDRESS             0x06CD
#define SX126X_REG_BROADCAST_ADDRESS        0x06CE
#define SX126X_REG_IQ_POLARITY              0x0736
#define SX126X_REG_LORA_SYNC_WORD_MSB       0x0740
#define SX126X_REG_LORA_SYNC_WORD_LSB       0x0741
#define SX126X_REG_RANDOM_NUMBER_0          0x0819
#define SX126X_REG_RANDOM_NUMBER_1          0x081A
#define SX126X_REG_RANDOM_NUMBER_2          0x081B
#define SX126X_REG_RANDOM_NUMBER_3          0x081C
#define SX126X_REG_TX_MODULATION            0x0889
#define SX126X_REG_RX_GAIN                  0x08AC
#define SX126X_REG_TX_CLAMP_CONFIG          0x08D8
#define SX126X_REG_OCP_CONFIGURATION        0x08E7
#define SX126X_REG_RTC_CTRL                 0x0902
#define SX126X_REG_XTA_TRIM                 0x0911
#define SX126X_REG_XTB_TRIM                 0x0912
#define SX126X_REG_DIO3_OUTPUT_CTRL         0x0920
#define SX126X_REG_EVENT_MASK               0x0944

// ============================================================================
// IRQ Masks
// ============================================================================

#define SX126X_IRQ_TX_DONE                  0x0001
#define SX126X_IRQ_RX_DONE                  0x0002
#define SX126X_IRQ_PREAMBLE_DETECTED        0x0004
#define SX126X_IRQ_SYNC_WORD_VALID          0x0008
#define SX126X_IRQ_HEADER_VALID             0x0010
#define SX126X_IRQ_HEADER_ERROR             0x0020
#define SX126X_IRQ_CRC_ERROR                0x0040
#define SX126X_IRQ_CAD_DONE                 0x0080
#define SX126X_IRQ_CAD_DETECTED             0x0100
#define SX126X_IRQ_TIMEOUT                  0x0200
#define SX126X_IRQ_ALL                      0x03FF

// ============================================================================
// Enumerations
// ============================================================================

typedef enum {
    SX126X_PACKET_TYPE_GFSK = 0x00,
    SX126X_PACKET_TYPE_LORA = 0x01
} SX126x_PacketType_e;

typedef enum {
    SX126X_LORA_SF5 = 0x05,
    SX126X_LORA_SF6 = 0x06,
    SX126X_LORA_SF7 = 0x07,
    SX126X_LORA_SF8 = 0x08,
    SX126X_LORA_SF9 = 0x09,
    SX126X_LORA_SF10 = 0x0A,
    SX126X_LORA_SF11 = 0x0B,
    SX126X_LORA_SF12 = 0x0C
} SX126x_LoRaSpreadingFactor_e;

typedef enum {
    SX126X_LORA_BW_7_8 = 0x00,      // 7.8 kHz
    SX126X_LORA_BW_10_4 = 0x08,     // 10.4 kHz
    SX126X_LORA_BW_15_6 = 0x01,     // 15.6 kHz
    SX126X_LORA_BW_20_8 = 0x09,     // 20.8 kHz
    SX126X_LORA_BW_31_25 = 0x02,    // 31.25 kHz
    SX126X_LORA_BW_41_7 = 0x0A,     // 41.7 kHz
    SX126X_LORA_BW_62_5 = 0x03,     // 62.5 kHz
    SX126X_LORA_BW_125 = 0x04,      // 125 kHz
    SX126X_LORA_BW_250 = 0x05,      // 250 kHz
    SX126X_LORA_BW_500 = 0x06       // 500 kHz
} SX126x_LoRaBandwidth_e;

typedef enum {
    SX126X_LORA_CR_4_5 = 0x01,      // 4/5
    SX126X_LORA_CR_4_6 = 0x02,      // 4/6
    SX126X_LORA_CR_4_7 = 0x03,      // 4/7
    SX126X_LORA_CR_4_8 = 0x04       // 4/8
} SX126x_LoRaCodingRate_e;

typedef enum {
    SX126X_STANDBY_RC = 0x00,
    SX126X_STANDBY_XOSC = 0x01
} SX126x_StandbyMode_e;

typedef enum {
    SX126X_REGULATOR_LDO = 0x00,
    SX126X_REGULATOR_DC_DC = 0x01
} SX126x_RegulatorMode_e;

// ============================================================================
// Configuration Structures
// ============================================================================

typedef struct {
    uint32_t frequency_hz;
    SX126x_LoRaSpreadingFactor_e spreading_factor;
    SX126x_LoRaBandwidth_e bandwidth;
    SX126x_LoRaCodingRate_e coding_rate;
    int8_t tx_power_dbm;        // -9 to +22 dBm
    uint16_t preamble_length;
    uint16_t sync_word;
    bool enable_crc;
    bool invert_iq;
} SX126x_LoRaConfig_t;

typedef struct {
    int16_t rssi;
    int8_t snr;
    int8_t signal_rssi;
    uint8_t length;
} SX126x_RxInfo_t;

// ============================================================================
// DMA State Machine
// ============================================================================

typedef enum {
    SX126X_DMA_IDLE = 0,
    SX126X_DMA_TX_BUSY,
    SX126X_DMA_RX_BUSY,
    SX126X_DMA_COMPLETE,
    SX126X_DMA_ERROR
} SX126x_DMA_State_e;

typedef struct {
    volatile SX126x_DMA_State_e spi_state;
    volatile bool tx_done;
    volatile bool rx_done;
    volatile bool dma_tx_complete;
    volatile bool dma_rx_complete;
    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];
    uint8_t rx_length;
    int16_t last_rssi;
    int8_t last_snr;
} SX126x_DMA_t;

// ============================================================================
// Function Prototypes
// ============================================================================

// Initialization
bool SX126x_Init(SX126x_LoRaConfig_t *config);
bool SX126x_Reset(void);
void SX126x_SetConfig(SX126x_LoRaConfig_t *config);

// Operating Modes
void SX126x_SetSleep(void);
void SX126x_SetStandby(SX126x_StandbyMode_e mode);
void SX126x_SetTx(uint32_t timeout_ms);
void SX126x_SetRx(uint32_t timeout_ms);

// Transmit Functions
bool SX126x_TransmitDMA(uint8_t *data, uint8_t length);
bool SX126x_IsTxBusy(void);
bool SX126x_IsTxDone(void);
void SX126x_WaitTxComplete(uint32_t timeout_ms);

// Receive Functions
bool SX126x_Available(void);
int SX126x_Receive(uint8_t *buffer, uint16_t max_length, SX126x_RxInfo_t *rx_info);

// Signal Quality
int16_t SX126x_GetRssi(void);
int8_t SX126x_GetSnr(void);

// Low-level SPI Functions
void SX126x_WriteCommand(uint8_t cmd, uint8_t *data, uint8_t length);
void SX126x_ReadCommand(uint8_t cmd, uint8_t *data, uint8_t length);
void SX126x_WriteRegister(uint16_t address, uint8_t *data, uint8_t length);
void SX126x_ReadRegister(uint16_t address, uint8_t *data, uint8_t length);

// Interrupt Handlers
void SX126x_DIO1_IRQ_Handler(void);
void SX126x_SPI_TxCpltCallback(void);
void SX126x_SPI_RxCpltCallback(void);

// Helper Functions
void SX126x_WaitNotBusy(void);
uint16_t SX126x_GetIrqStatus(void);
void SX126x_ClearIrqStatus(uint16_t irq_mask);

#endif /* RADIO_SX126X_H_ */
