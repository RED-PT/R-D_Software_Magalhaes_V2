/*
 * lora_sx1276.c
 *
 *  Created on: Nov 4, 2025
 *      Author: texman
 */

#include "lora_sx1276.h"

typedef enum {
    DMA_IDLE = 0,
    DMA_TX_BUSY,
    DMA_RX_BUSY,
    DMA_COMPLETE,
    DMA_ERROR
} DMA_State_e;

typedef struct {
    volatile DMA_State_e spi_state;
    volatile bool tx_done;          // LoRa TX done (from DIO0)
    volatile bool rx_done;          // LoRa RX done (from DIO0)
    volatile bool dma_tx_complete;  // DMA transfer complete
    volatile bool dma_rx_complete;  // DMA receive complete
    uint8_t tx_buffer[256];         // DMA-safe TX buffer
    uint8_t rx_buffer[256];         // DMA-safe RX buffer
    uint8_t rx_length;              // Received packet length
    int16_t last_rssi;              // Last packet RSSI
    int8_t last_snr;                // Last packet SNR
} LoRa_DMA_t;

static LoRa_DMA_t g_lora_dma = {0};

void LoRa_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = {reg | 0x80, value};
    CS_LORA_LOW();
    HAL_SPI_Transmit(SPI_LORA, tx_data, 2, 100);
    CS_LORA_HIGH();
}

uint8_t LoRa_ReadReg(uint8_t reg) {
    uint8_t tx_data = reg & 0x7F;
    uint8_t rx_data = 0;
    CS_LORA_LOW();
    HAL_SPI_Transmit(SPI_LORA, &tx_data, 1, 100);
    HAL_SPI_Receive(SPI_LORA, &rx_data, 1, 100);
    CS_LORA_HIGH();
    return rx_data;
}

static void LoRa_WriteFifoDMA(uint8_t *data, uint8_t length) {
    // Wait for any previous DMA to complete
    while (g_lora_dma.spi_state != DMA_IDLE);

    // Prepare DMA buffer: [REG_FIFO | 0x80] [data...]
    g_lora_dma.tx_buffer[0] = REG_FIFO | 0x80;
    memcpy(&g_lora_dma.tx_buffer[1], data, length);

    g_lora_dma.spi_state = DMA_TX_BUSY;
    g_lora_dma.dma_tx_complete = false;

    CS_LORA_LOW();
    HAL_SPI_Transmit_DMA(SPI_LORA, g_lora_dma.tx_buffer, length + 1);
}

static void LoRa_ReadFifoDMA(uint8_t length) {
    // Wait for any previous DMA to complete
    while (g_lora_dma.spi_state != DMA_IDLE);

    // Prepare DMA buffer: [REG_FIFO & 0x7F]
    g_lora_dma.tx_buffer[0] = REG_FIFO & 0x7F;

    g_lora_dma.spi_state = DMA_RX_BUSY;
    g_lora_dma.dma_rx_complete = false;

    CS_LORA_LOW();
    // First send register address
    HAL_SPI_Transmit(SPI_LORA, g_lora_dma.tx_buffer, 1, 100);
    // Then receive data via DMA
    HAL_SPI_Receive_DMA(SPI_LORA, g_lora_dma.rx_buffer, length);
}

void LoRa_SPI_TxCpltCallback(void) {
    CS_LORA_HIGH();
    g_lora_dma.dma_tx_complete = true;
    g_lora_dma.spi_state = DMA_IDLE;
}

void LoRa_SPI_RxCpltCallback(void) {
    CS_LORA_HIGH();
    g_lora_dma.dma_rx_complete = true;
    g_lora_dma.spi_state = DMA_IDLE;
}

// DIO0 interrupt handler (TX done / RX done)
// IMPORTANT: This is called from ISR context!
void LoRa_DIO0_IRQ_Handler(void) {
    // Don't read registers here - just set flags
    // The thread will handle reading IRQ flags
    uint8_t mode = LoRa_ReadReg(REG_OP_MODE) & 0x07;

    if (mode == MODE_TX) {
        g_lora_dma.tx_done = true;
    } else if (mode == MODE_RX_CONTINUOUS || mode == MODE_RX_SINGLE) {
        g_lora_dma.rx_done = true;
    }
}

bool LoRa_Reset(void) {
    LoRa_Sleep();
    HAL_Delay(10);

    // Initialize DMA state
    g_lora_dma.spi_state = DMA_IDLE;
    g_lora_dma.tx_done = false;
    g_lora_dma.rx_done = false;
    g_lora_dma.dma_tx_complete = false;
    g_lora_dma.dma_rx_complete = false;
    g_lora_dma.rx_length = 0;
    g_lora_dma.last_rssi = 0;
    g_lora_dma.last_snr = 0;

    return true;
}

bool LoRa_CheckVersion(void) {
    uint8_t version = LoRa_ReadReg(REG_VERSION);
    return (version == 0x12);
}

bool LoRa_Init(LoRa_Config_t *config) {
    // 1. Software reset
    LoRa_Reset();

    // 2. Check version
    if (!LoRa_CheckVersion()) {
        return false;
    }

    // 3. Set sleep mode
    LoRa_Sleep();
    HAL_Delay(10);

    // 4. Set LoRa mode
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
    HAL_Delay(10);

    // 5. Set to standby mode
    LoRa_Standby();
    HAL_Delay(10);

    // 6. Set FIFO pointers
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0x00);
    LoRa_WriteReg(REG_FIFO_RX_BASE_ADDR, 0x00);

    // 7. Apply configuration
    LoRa_SetConfig(config);

    // 8. Set DIO mapping (DIO0 = TxDone/RxDone)
    LoRa_WriteReg(REG_DIO_MAPPING_1, 0x00);

    // 9. Clear IRQ flags
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);

    return true;
}

void LoRa_SetConfig(LoRa_Config_t *config) {
    // Set frequency
    uint64_t frf = ((uint64_t)config->frequency_hz << 19) / 32000000;
    LoRa_WriteReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_WriteReg(REG_FRF_LSB, (uint8_t)(frf >> 0));

    // Set bandwidth, coding rate, implicit header
    uint8_t modem_config1 = (config->bandwidth << 4) |
                            (config->coding_rate << 1) |
                            (config->implicit_header ? 1 : 0);
    LoRa_WriteReg(REG_MODEM_CONFIG_1, modem_config1);

    // Set spreading factor and CRC
    uint8_t modem_config2 = (config->spreading_factor << 4) |
                            (config->enable_crc ? 0x04 : 0x00);
    LoRa_WriteReg(REG_MODEM_CONFIG_2, modem_config2);

    // Set LowDataRateOptimize for SF11/SF12
    uint8_t modem_config3 = LoRa_ReadReg(REG_MODEM_CONFIG_3);
    if (config->spreading_factor >= LORA_SF_11) {
        modem_config3 |= 0x08;
    } else {
        modem_config3 &= ~0x08;
    }
    LoRa_WriteReg(REG_MODEM_CONFIG_3, modem_config3);

    // Set preamble length
    LoRa_WriteReg(REG_PREAMBLE_MSB, (uint8_t)(config->preamble_length >> 8));
    LoRa_WriteReg(REG_PREAMBLE_LSB, (uint8_t)(config->preamble_length & 0xFF));

    // Set sync word
    LoRa_WriteReg(REG_SYNC_WORD, config->sync_word);

    // Set TX power
    if (config->tx_power_dbm > 17) {
        LoRa_WriteReg(REG_PA_DAC, 0x87);
        config->tx_power_dbm = (config->tx_power_dbm > 20) ? 20 : config->tx_power_dbm;
        LoRa_WriteReg(REG_PA_CONFIG, PA_BOOST | (config->tx_power_dbm - 5));
    } else {
        LoRa_WriteReg(REG_PA_DAC, 0x84);
        config->tx_power_dbm = (config->tx_power_dbm < 2) ? 2 : config->tx_power_dbm;
        LoRa_WriteReg(REG_PA_CONFIG, PA_BOOST | (config->tx_power_dbm - 2));
    }

    // Set OCP and LNA
    LoRa_WriteReg(REG_OCP, 0x0B | 0x20);
    LoRa_WriteReg(REG_LNA, LoRa_ReadReg(REG_LNA) | 0x03);
}

void LoRa_Sleep(void) {
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa_Standby(void) {
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa_SetModeRx(void) {
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_RX_BASE_ADDR));
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    g_lora_dma.rx_done = false;
}

void LoRa_SetModeTx(void) {
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    g_lora_dma.tx_done = false;
}

bool LoRa_TransmitDMA(uint8_t *data, uint8_t length) {
    if (length > MAX_PKT_LENGTH) {
        return false;
    }

    // Check if previous transmission is still in progress
    if (g_lora_dma.spi_state != DMA_IDLE || !g_lora_dma.tx_done) {
        return false;  // Busy
    }

    // 1. Go to standby
    LoRa_Standby();

    // 2. Clear IRQ flags
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);

    // 3. Set FIFO pointer
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, LoRa_ReadReg(REG_FIFO_TX_BASE_ADDR));

    // 4. Write payload via DMA
    LoRa_WriteFifoDMA(data, length);

    // 5. Wait for DMA to complete
    while (!g_lora_dma.dma_tx_complete);

    // 6. Set payload length
    LoRa_WriteReg(REG_PAYLOAD_LENGTH, length);

    // 7. Start transmission
    LoRa_SetModeTx();

    return true;
}

bool LoRa_IsTxBusy(void) {
    return (g_lora_dma.spi_state != DMA_IDLE) || (!g_lora_dma.tx_done);
}

bool LoRa_IsTxDone(void) {
    return g_lora_dma.tx_done;
}

void LoRa_WaitTxComplete(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (!g_lora_dma.tx_done && ((HAL_GetTick() - start) < timeout_ms)) {
        // Wait
    }
}

bool LoRa_Available(void) {
    return g_lora_dma.rx_done;
}

int LoRa_Receive(uint8_t *buffer, uint8_t max_length, LoRa_RxInfo_t *rx_info) {
    if (!g_lora_dma.rx_done) {
        return 0;  // No packet available
    }

    // Read IRQ flags
    uint8_t irq_flags = LoRa_ReadReg(REG_IRQ_FLAGS);

    // Clear RX done flag
    LoRa_WriteReg(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);
    g_lora_dma.rx_done = false;

    // Check for CRC error
    if (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        LoRa_WriteReg(REG_IRQ_FLAGS, IRQ_PAYLOAD_CRC_ERROR_MASK);
        return -1;  // CRC error
    }

    // Read packet length
    uint8_t rx_length = LoRa_ReadReg(REG_RX_NB_BYTES);
    if (rx_length > max_length) {
        rx_length = max_length;
    }

    // Set FIFO address to start of received packet
    uint8_t fifo_rx_addr = LoRa_ReadReg(REG_FIFO_RX_CURRENT_ADDR);
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, fifo_rx_addr);

    // Read FIFO via DMA
    LoRa_ReadFifoDMA(rx_length);

    // Wait for DMA to complete
    while (!g_lora_dma.dma_rx_complete);

    // Copy data from DMA buffer to user buffer
    memcpy(buffer, g_lora_dma.rx_buffer, rx_length);

    // Get signal quality metrics
    if (rx_info != NULL) {
        rx_info->rssi = LoRa_ReadReg(REG_PKT_RSSI_VALUE) - 157;
        rx_info->snr = (int8_t)LoRa_ReadReg(REG_PKT_SNR_VALUE) / 4;
        rx_info->length = rx_length;

        // Store for later retrieval
        g_lora_dma.last_rssi = rx_info->rssi;
        g_lora_dma.last_snr = rx_info->snr;
    }

    return rx_length;
}

int16_t LoRa_GetRssi(void) {
    return g_lora_dma.last_rssi;
}

int8_t LoRa_GetSnr(void) {
    return g_lora_dma.last_snr;
}
