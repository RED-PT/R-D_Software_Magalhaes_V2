/*
 * lora_sx126x.c
 */

#include "lora_sx126x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static DMA_BUFFER SX126x_DMA_t sx126x_state;

// ============================================================================
// LOW-LEVEL SPI
// ============================================================================

static void wait_not_busy(void) {
    uint32_t timeout = HAL_GetTick() + 1000;
    while (SX126X_IS_BUSY()) {
        if (HAL_GetTick() > timeout) break;
    }
}

void SX126x_WriteCommand(uint8_t cmd, uint8_t *data, uint8_t length) {
    wait_not_busy();
    SX126X_CS_LOW();
    HAL_SPI_Transmit(SPI_SX126X, &cmd, 1, 100);
    if (length > 0 && data != NULL) {
        HAL_SPI_Transmit(SPI_SX126X, data, length, 100);
    }
    SX126X_CS_HIGH();

    if (cmd != SX126X_CMD_SET_SLEEP) {
        wait_not_busy();
    }
}

void SX126x_ReadCommand(uint8_t cmd, uint8_t *data, uint8_t length) {
    wait_not_busy();
    SX126X_CS_LOW();
    HAL_SPI_Transmit(SPI_SX126X, &cmd, 1, 100);
    uint8_t dummy = 0x00;
    HAL_SPI_Transmit(SPI_SX126X, &dummy, 1, 100);
    HAL_SPI_Receive(SPI_SX126X, data, length, 100);
    SX126X_CS_HIGH();
}

void SX126x_WriteRegister(uint16_t address, uint8_t *data, uint8_t length) {
    wait_not_busy();
    uint8_t buf[2] = {(address >> 8) & 0xFF, address & 0xFF};
    SX126X_CS_LOW();
    uint8_t cmd = SX126X_CMD_WRITE_REGISTER;
    HAL_SPI_Transmit(SPI_SX126X, &cmd, 1, 100);
    HAL_SPI_Transmit(SPI_SX126X, buf, 2, 100);
    HAL_SPI_Transmit(SPI_SX126X, data, length, 100);
    SX126X_CS_HIGH();
}

void SX126x_ReadRegister(uint16_t address, uint8_t *data, uint8_t length) {
    wait_not_busy();
    uint8_t buf[2] = {(address >> 8) & 0xFF, address & 0xFF};
    SX126X_CS_LOW();
    uint8_t cmd = SX126X_CMD_READ_REGISTER;
    HAL_SPI_Transmit(SPI_SX126X, &cmd, 1, 100);
    HAL_SPI_Transmit(SPI_SX126X, buf, 2, 100);
    uint8_t dummy = 0x00;
    HAL_SPI_Transmit(SPI_SX126X, &dummy, 1, 100);
    HAL_SPI_Receive(SPI_SX126X, data, length, 100);
    SX126X_CS_HIGH();
}

// ============================================================================
// IRQ
// ============================================================================

uint16_t SX126x_GetIrqStatus(void) {
    uint8_t status[2] = {0};
    SX126x_ReadCommand(SX126X_CMD_GET_IRQ_STATUS, status, 2);
    return ((uint16_t)status[0] << 8) | status[1];
}

void SX126x_ClearIrqStatus(uint16_t irq_mask) {
    uint8_t buf[2] = {(irq_mask >> 8) & 0xFF, irq_mask & 0xFF};
    SX126x_WriteCommand(SX126X_CMD_CLR_IRQ_STATUS, buf, 2);
}

// ============================================================================
// MODE CONTROL
// ============================================================================

void SX126x_SetStandby(SX126x_StandbyMode_e mode) {
    uint8_t data = mode;
    SX126x_WriteCommand(SX126X_CMD_SET_STANDBY, &data, 1);
}

void SX126x_SetSleep(void) {
    uint8_t data = 0x04;
    SX126x_WriteCommand(SX126X_CMD_SET_SLEEP, &data, 1);
}

void SX126x_SetTx(uint32_t timeout_ms) {
#ifdef RADIO_INTERFACE_SPI
    RADIO_RF_SWITCH_TX();
#endif
    uint8_t buf[3];
    uint32_t timeout = timeout_ms * 64;
    buf[0] = (timeout >> 16) & 0xFF;
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;
    SX126x_WriteCommand(SX126X_CMD_SET_TX, buf, 3);
}

void SX126x_SetRx(uint32_t timeout_ms) {
#ifdef RADIO_INTERFACE_SPI
    RADIO_RF_SWITCH_RX();
#endif
    uint8_t buf[3];
    if (timeout_ms == 0xFFFFFF) {
        buf[0] = 0xFF; buf[1] = 0xFF; buf[2] = 0xFF;
    } else {
        uint32_t timeout = timeout_ms * 64;
        buf[0] = (timeout >> 16) & 0xFF;
        buf[1] = (timeout >> 8) & 0xFF;
        buf[2] = timeout & 0xFF;
    }
    SX126x_WriteCommand(SX126X_CMD_SET_RX, buf, 3);
}

// ============================================================================
// RESET & INIT
// ============================================================================

bool SX126x_Reset(void) {
    SX126X_RESET_LOW();
    HAL_Delay(10);
    SX126X_RESET_HIGH();
    HAL_Delay(10);
    wait_not_busy();
    return true;
}

bool SX126x_Init(SX126x_LoRaConfig_t *config) {
    memset(&sx126x_state, 0, sizeof(sx126x_state));

    if (!SX126x_Reset()) return false;

    SX126x_SetStandby(SX126X_STANDBY_RC);

    uint8_t reg_mode = SX126X_REGULATOR_DC_DC;
    SX126x_WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &reg_mode, 1);

    uint8_t calib = 0x7F;
    SX126x_WriteCommand(SX126X_CMD_CALIBRATE, &calib, 1);
    HAL_Delay(10);

    uint8_t pkt_type = SX126X_PACKET_TYPE_LORA;
    SX126x_WriteCommand(SX126X_CMD_SET_PKT_TYPE, &pkt_type, 1);

    uint32_t freq = (uint32_t)((double)config->frequency_hz / 32000000.0 * 33554432.0);
    uint8_t freq_buf[4] = {
        (freq >> 24) & 0xFF, (freq >> 16) & 0xFF,
        (freq >> 8) & 0xFF, freq & 0xFF
    };
    SX126x_WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, freq_buf, 4);

    uint8_t mod_params[4] = {
        config->spreading_factor,
        config->bandwidth,
        config->coding_rate,
        0x00
    };
    SX126x_WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, mod_params, 4);

    uint8_t pkt_params[6] = {
        (config->preamble_length >> 8) & 0xFF,
        config->preamble_length & 0xFF,
        0x00,
        config->enable_crc ? 0x01 : 0x00,
        config->invert_iq ? 0x01 : 0x00,
        0x00
    };
    SX126x_WriteCommand(SX126X_CMD_SET_PKT_PARAMS, pkt_params, 6);

    uint8_t pa_config[4] = {0x04, 0x07, 0x00, 0x01};
    SX126x_WriteCommand(SX126X_CMD_SET_PA_CONFIG, pa_config, 4);

    uint8_t tx_params[2] = {config->tx_power_dbm, 0x04};
    SX126x_WriteCommand(SX126X_CMD_SET_TX_PARAMS, tx_params, 2);

    uint8_t base_addr[2] = {0x00, 0x00};
    SX126x_WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, base_addr, 2);

    uint8_t sync_word[2] = {
        (config->sync_word >> 8) & 0xFF,
        config->sync_word & 0xFF
    };
    SX126x_WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, sync_word, 2);

#ifdef RADIO_INTERFACE_SPI
    // E22-900M22S has external RF switch (RXEN/TXEN GPIO) — do NOT use DIO2
    uint8_t dio2_rf = 0x00;
    SX126x_WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH, &dio2_rf, 1);
    RADIO_RF_SWITCH_OFF();
#else
    // Direct SX1262 modules use DIO2 as internal RF switch
    uint8_t dio2_rf = 0x01;
    SX126x_WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH, &dio2_rf, 1);
#endif

    uint16_t irq_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE |
                        SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERROR;
    uint8_t irq_params[8] = {
        (irq_mask >> 8) & 0xFF, irq_mask & 0xFF,
        (irq_mask >> 8) & 0xFF, irq_mask & 0xFF,
        0x00, 0x00, 0x00, 0x00
    };
    SX126x_WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, irq_params, 8);

    SX126x_ClearIrqStatus(SX126X_IRQ_ALL);

    return true;
}

// ============================================================================
// TRANSMIT DMA
// ============================================================================

bool SX126x_TransmitDMA(uint8_t *data, uint8_t length) {
    if (sx126x_state.spi_state != SX126X_DMA_IDLE) return false;
    if (length == 0 || length > 255) return false;

    sx126x_state.tx_done = false;
    sx126x_state.spi_state = SX126X_DMA_TX_BUSY;

    SX126x_SetStandby(SX126X_STANDBY_RC);

    uint8_t pkt_params[6];
    SX126x_ReadCommand(SX126X_CMD_SET_PKT_PARAMS, pkt_params, 6);
    pkt_params[0] = 0x00;
    pkt_params[1] = 0x08;
    pkt_params[2] = length;
    SX126x_WriteCommand(SX126X_CMD_SET_PKT_PARAMS, pkt_params, 6);

    wait_not_busy();
    SX126X_CS_LOW();
    uint8_t cmd = SX126X_CMD_WRITE_BUFFER;
    HAL_SPI_Transmit(SPI_SX126X, &cmd, 1, 100);
    uint8_t offset = 0x00;
    HAL_SPI_Transmit(SPI_SX126X, &offset, 1, 100);

    memcpy(sx126x_state.tx_buffer, data, length);
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(SPI_SX126X, sx126x_state.tx_buffer, length);

    if (status != HAL_OK) {
        SX126X_CS_HIGH();
        sx126x_state.spi_state = SX126X_DMA_IDLE;
        return false;
    }

    return true;
}

void SX126x_SPI_TxCpltCallback(void) {
    SX126X_CS_HIGH();

    SX126x_ClearIrqStatus(SX126X_IRQ_ALL);
    SX126x_SetTx(1000);

    sx126x_state.spi_state = SX126X_DMA_IDLE;
}

void SX126x_SPI_RxCpltCallback(void) {
    SX126X_CS_HIGH();
    sx126x_state.dma_rx_complete = true;
}

bool SX126x_IsTxBusy(void) {
    return (sx126x_state.spi_state == SX126X_DMA_TX_BUSY) || !sx126x_state.tx_done;
}

bool SX126x_IsTxDone(void) {
    return sx126x_state.tx_done;
}

void SX126x_WaitTxComplete(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (!sx126x_state.tx_done && (HAL_GetTick() - start < timeout_ms)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// RECEIVE
// ============================================================================

bool SX126x_Available(void) {
    return sx126x_state.rx_done;
}

int SX126x_Receive(uint8_t *buffer, uint16_t max_length, SX126x_RxInfo_t *rx_info) {
    if (!sx126x_state.rx_done) return 0;

    SX126x_SetStandby(SX126X_STANDBY_RC);

    uint8_t status[2];
    SX126x_ReadCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, status, 2);
    uint8_t rx_len = status[0];
    uint8_t rx_offset = status[1];

    if (rx_len > max_length) rx_len = max_length;

    wait_not_busy();
    SX126X_CS_LOW();
    uint8_t cmd = SX126X_CMD_READ_BUFFER;
    HAL_SPI_Transmit(SPI_SX126X, &cmd, 1, 100);
    HAL_SPI_Transmit(SPI_SX126X, &rx_offset, 1, 100);
    uint8_t dummy = 0x00;
    HAL_SPI_Transmit(SPI_SX126X, &dummy, 1, 100);
    HAL_SPI_Receive(SPI_SX126X, buffer, rx_len, 100);
    SX126X_CS_HIGH();

    if (rx_info) {
        uint8_t pkt_status[3];
        SX126x_ReadCommand(SX126X_CMD_GET_PKT_STATUS, pkt_status, 3);
        rx_info->rssi = -pkt_status[0] / 2;
        rx_info->snr = (int8_t)pkt_status[1] / 4;
        rx_info->signal_rssi = -pkt_status[2] / 2;
        rx_info->length = rx_len;

        sx126x_state.last_rssi = rx_info->rssi;
        sx126x_state.last_snr = rx_info->snr;
    }

    sx126x_state.rx_done = false;
    SX126x_ClearIrqStatus(SX126X_IRQ_ALL);
    SX126x_SetRx(0xFFFFFF);

    return rx_len;
}

int16_t SX126x_GetRssi(void) {
    return sx126x_state.last_rssi;
}

int8_t SX126x_GetSnr(void) {
    return sx126x_state.last_snr;
}

// ============================================================================
// DIO1 IRQ HANDLER
// ============================================================================

void SX126x_DIO1_IRQ_Handler(void) {
    uint16_t irq_status = SX126x_GetIrqStatus();

    if (irq_status & SX126X_IRQ_TX_DONE) {
        sx126x_state.tx_done = true;
        SX126x_ClearIrqStatus(SX126X_IRQ_TX_DONE);
    }

    if (irq_status & SX126X_IRQ_RX_DONE) {
        sx126x_state.rx_done = true;
        SX126x_ClearIrqStatus(SX126X_IRQ_RX_DONE);
    }

    if (irq_status & SX126X_IRQ_TIMEOUT) {
        SX126x_ClearIrqStatus(SX126X_IRQ_TIMEOUT);
    }

    if (irq_status & SX126X_IRQ_CRC_ERROR) {
        SX126x_ClearIrqStatus(SX126X_IRQ_CRC_ERROR);
        SX126x_SetRx(0xFFFFFF);
    }
}

void SX126x_WaitNotBusy(void) {
    wait_not_busy();
}
