/**
 * @file MMC5983MA.c
 * @brief MMC5983MA 3-Axis Magnetometer Driver Implementation
 * @author Tomas Teixeira
 * @date October 2025
 * @version 2.0
 *
 * @details
 * Implements the driver for the MEMSIC MMC5983MA 3-axis magnetometer.
 * Uses DMA transfers for non-blocking sensor reads.
 * Board-agnostic: SPI instance and chip-select pins are resolved via config.h
 * macros (SPI_MAG, CS_MAG_PORT/PIN). On F446ZE this maps to SPI3, on H743ZI
 * (Buzz V4) it maps to SPI6.
 *
 * @see MMC5983MA.h for interface documentation
 * @ingroup Sensors
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

    osDelay(10);

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

    osDelay(2);

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

    // Enable continuous measurement at 1000 Hz.
    // Cmm_en is BIT 3 (0x08) in Internal Control 2; the old value 0x04|0x07
    // (= 0x07) set only CM_FREQ and never actually enabled continuous mode,
    // so no measurements (and no DRDY interrupts) were generated.
    if (!MMC5983MA_WriteRegister(dev, MMC5983MA_REG_CTRL2,
                                 (0x08 | 0x07))) {  // Cmm_en | CM_Freq=1000Hz
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
    // Read from XOUT0 (0x00) through TOUT (0x07) = 8 bytes of data.
    // MMC5983MA SPI frame: bit7 = READ, bits[6:0] = register address
    // (no shift — verify on hardware, see MMC5983MA_ReadRegister note).
    dev->tx_buffer[0] = 0x80 | MMC5983MA_REG_XOUT0;  // Read from XOUT0
    memset(&dev->tx_buffer[1], 0x00, 8);

    /* Assert CS for the DMA frame (was missing — with software CS the sensor
     * never saw the transaction). Deasserted in HAL_SPI_TxRxCpltCallback /
     * the error path. */
    CS_MAG_LOW();

    // Single DMA TransmitReceive
    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, dev->tx_buffer, dev->read_buffer, 9) != HAL_OK) {
        CS_MAG_HIGH();
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

    // Convert raw counts to Gauss (18-bit: 16384 Counts/G).
    // The output is OFFSET-BINARY: null field = 2^17 = 131072 counts.
    // Without subtracting the offset every reading came out positive,
    // centred at ~+8 G.
    output->mag_x = (float)(dev->mag_x_raw - 131072) / MMC5983MA_SENSITIVITY_18BIT;
    output->mag_y = (float)(dev->mag_y_raw - 131072) / MMC5983MA_SENSITIVITY_18BIT;
    output->mag_z = (float)(dev->mag_z_raw - 131072) / MMC5983MA_SENSITIVITY_18BIT;

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

    /* MMC5983MA SPI: bit7 = READ, bits[6:0] = register address. The old
     * `(reg << 1)` encoding addressed the wrong register (e.g. Product ID
     * 0x2F became 0x5E) — if the mag ever passed init with that encoding,
     * re-verify this change on hardware with a logic analyzer. */
    tx_buf[0] = 0x80 | (reg & 0x7F);
    tx_buf[1] = 0x00;

    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_buf, rx_buf, 2, 100) != HAL_OK) {
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

    tx_buf[0] = (reg & 0x7F);   /* write: bit7 = 0, 7-bit address (no shift) */
    tx_buf[1] = data;

    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_buf, rx_buf, 2, 100) != HAL_OK) {
        HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET);

    return true;
}
