/*
 * MMC5983MA.c
 *
 *  Created on: Oct 16, 2025
 *      Author: texman
 */

#include "MMC5983MA.h"
#include <string.h>
#include "cmsis_os2.h"
#include "print.h"

// Private helper functions
static bool MMC5983MA_ReadRegister(MMC5983MA_t *dev, uint8_t reg, uint8_t *data);
static bool MMC5983MA_WriteRegister(MMC5983MA_t *dev, uint8_t reg, uint8_t data);

bool MMC5983MA_Init(MMC5983MA_t *dev, SPI_HandleTypeDef *hspi) {
    uint8_t product_id;

    if (!dev || !hspi) {
        return false;
    }

    dev->hspi = hspi;
    dev->data_ready = 0;
    memset(dev->read_buffer, 0, sizeof(dev->read_buffer));

    HAL_Delay(10);

    // Read and verify Product ID
    product_id = 0x00;
    if (!MMC5983MA_ReadRegister(dev, MMC5983MA_REG_PRODUCT_ID, &product_id)) {
        printf("[MAG] Failed to read product ID\r\n");
        return false;
    }
    printf("[MAG] Product ID = 0x%02X (expected 0x30)\r\n", product_id);
    if (product_id != 0x30) {  // Product ID should be 0x30
        printf("[MAG] Product ID mismatch!\r\n");
        return false;
    }

    // Perform software reset via SET operation
    if (!MMC5983MA_WriteRegister(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET)) {
        return false;
    }

    HAL_Delay(2);

    return true;
}

bool MMC5983MA_Configure(MMC5983MA_t *dev) {
    if (!dev) {
        return false;
    }

    // Reset to default
    if (!MMC5983MA_WriteRegister(dev, MMC5983MA_REG_CTRL1, 0x00)) {
        return false;
    }
    osDelay(1);

    // Set BW[1:0] = 11 (800Hz, 0.5ms measurement time)
    if (!MMC5983MA_WriteRegister(dev, MMC5983MA_REG_CTRL1,
                                 (MMC5983MA_CTRL1_BW0 | MMC5983MA_CTRL1_BW1))) {
        return false;
    }
    osDelay(1);

    // Enable continuous measurement at 1000 Hz
    if (!MMC5983MA_WriteRegister(dev, MMC5983MA_REG_CTRL2,
                                 (0x04 | 0x07))) {  // Cmm_en | CM_Freq=1000Hz
        return false;
    }
    osDelay(1);

    // Enable interrupt and auto set/reset
    if (!MMC5983MA_WriteRegister(dev, MMC5983MA_REG_CTRL0,
                                 (MMC5983MA_CTRL0_INT_EN | MMC5983MA_CTRL0_AUTO_SR))) {
        return false;
    }
    osDelay(1);

    return true;
}

bool MMC5983MA_StartReadDMA(MMC5983MA_t *dev) {
    if (!dev || !dev->hspi) {
        return false;
    }

    // Prepare TX buffer: [COMMAND][DUMMIES]
    // Read from XOUT0 (0x00) through TOUT (0x07) = 8 bytes of data
    uint8_t tx_buffer[9];
    tx_buffer[0] = (1 << 7) | (MMC5983MA_REG_XOUT0 << 1);  // Read from XOUT0
    memset(&tx_buffer[1], 0x00, 8);

    // Single DMA TransmitReceive
    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, tx_buffer, dev->read_buffer, 9) != HAL_OK) {
        return false;
    }

    return true;
}

void MMC5983MA_ParseDMABuffer(MMC5983MA_t *dev) {
    if (!dev) {
        return;
    }

    // Parse 18-bit magnetic field data (skip first byte which is command echo)
    // Bytes: [CMD][XOUT0][XOUT1][YOUT0][YOUT1][ZOUT0][ZOUT1][XYZout2][TOUT]

    dev->mag_x_raw = ((int32_t)dev->read_buffer[1] << 10) |
                     ((int32_t)dev->read_buffer[2] << 2) |
                     ((int32_t)(dev->read_buffer[7] >> 6) & 0x03);

    dev->mag_y_raw = ((int32_t)dev->read_buffer[3] << 10) |
                     ((int32_t)dev->read_buffer[4] << 2) |
                     ((int32_t)(dev->read_buffer[7] >> 4) & 0x03);

    dev->mag_z_raw = ((int32_t)dev->read_buffer[5] << 10) |
                     ((int32_t)dev->read_buffer[6] << 2) |
                     ((int32_t)(dev->read_buffer[7] >> 2) & 0x03);

    // Parse temperature (TOUT register at byte 8)
    dev->temp_raw = dev->read_buffer[8];

    dev->data_ready = 1;
}

bool MMC5983MA_ProcessData(MMC5983MA_t *dev, MAG_t *output) {
    if (!dev || !output || !dev->data_ready) {
        return false;
    }

    // Convert raw counts to Gauss (18-bit: 16384 Counts/G)
    output->mag_x = (float)dev->mag_x_raw / MMC5983MA_SENSITIVITY_18BIT;
    output->mag_y = (float)dev->mag_y_raw / MMC5983MA_SENSITIVITY_18BIT;
    output->mag_z = (float)dev->mag_z_raw / MMC5983MA_SENSITIVITY_18BIT;

    // Convert temperature: (TOUT_raw * 0.8) - 75
    // Range: -75~125°C at 0.8°C/LSB
    output->temperature_c = (float)dev->temp_raw * MMC5983MA_TEMP_SCALE + MMC5983MA_TEMP_OFFSET;
    output->timestamp_ms = HAL_GetTick();

    dev->data_ready = 0;

    return true;
}

// Private Functions

static bool MMC5983MA_ReadRegister(MMC5983MA_t *dev, uint8_t reg, uint8_t *data) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    if (!dev || !data) {
        return false;
    }

    tx_buf[0] = (1 << 7) | (reg << 1);
    tx_buf[1] = 0x00;

    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_buf, rx_buf, 2, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET);

    *data = rx_buf[1];
    return true;
}

static bool MMC5983MA_WriteRegister(MMC5983MA_t *dev, uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    if (!dev) {
        return false;
    }

    tx_buf[0] = (0 << 7) | (reg << 1);
    tx_buf[1] = data;

    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_buf, rx_buf, 2, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET);

    return true;
}
