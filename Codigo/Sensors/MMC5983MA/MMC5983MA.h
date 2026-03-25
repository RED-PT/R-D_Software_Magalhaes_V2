/**
 * @file MMC5983MA.h
 * @brief MMC5983MA 3-axis magnetometer driver for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This driver provides an interface to the MEMSIC MMC5983MA high-performance
 * 3-axis magnetometer for heading/compass applications.
 *
 * @section mmc_features Features
 * - 3-axis magnetic field measurement
 * - 18-bit resolution (0.0625 mGauss/LSB)
 * - Full scale range: +/-8 Gauss
 * - SPI interface up to 10 MHz
 * - Automatic SET/RESET for offset elimination
 * - DMA-based data transfer
 *
 * @section mmc_usage Usage
 * @code
 * MMC5983MA_t mag;
 * MAG_t data;
 *
 * MMC5983MA_Init(&mag, &hspi1);
 * MMC5983MA_Configure(&mag);
 *
 * // Triggered by data ready or timer
 * MMC5983MA_StartReadDMA(&mag);
 *
 * // After DMA complete
 * MMC5983MA_ParseDMABuffer(&mag);
 * MMC5983MA_ProcessData(&mag, &data);
 * @endcode
 *
 * @see MAG_t for output data structure
 */

#ifndef SENSORS_MMC5983MA_MMC5983MA_H_
#define SENSORS_MMC5983MA_MMC5983MA_H_

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "defs.h"

/**
 * @defgroup MMCRegisters Register Addresses
 * @brief MMC5983MA register map
 * @{
 */
#define MMC5983MA_REG_XOUT0          0x00    /**< X-axis output bits [17:10] */
#define MMC5983MA_REG_XOUT1          0x01    /**< X-axis output bits [9:2] */
#define MMC5983MA_REG_YOUT0          0x02    /**< Y-axis output bits [17:10] */
#define MMC5983MA_REG_YOUT1          0x03    /**< Y-axis output bits [9:2] */
#define MMC5983MA_REG_ZOUT0          0x04    /**< Z-axis output bits [17:10] */
#define MMC5983MA_REG_ZOUT1          0x05    /**< Z-axis output bits [9:2] */
#define MMC5983MA_REG_XYZOUT2        0x06    /**< XYZ output bits [1:0] */
#define MMC5983MA_REG_TOUT           0x07    /**< Temperature output */
#define MMC5983MA_REG_STATUS         0x08    /**< Device status */
#define MMC5983MA_REG_CTRL0          0x09    /**< Control register 0 */
#define MMC5983MA_REG_CTRL1          0x0A    /**< Control register 1 */
#define MMC5983MA_REG_CTRL2          0x0B    /**< Control register 2 */
#define MMC5983MA_REG_CTRL3          0x0C    /**< Control register 3 */
#define MMC5983MA_REG_PRODUCT_ID     0x2F    /**< Product ID register (0x30) */
/** @} */

/**
 * @defgroup MMCControl Control Register Bits
 * @brief Control register bit definitions
 * @{
 */
#define MMC5983MA_CTRL0_TM_M         0x01    /**< Take measurement (magnetic) */
#define MMC5983MA_CTRL0_INT_EN       0x04    /**< Interrupt enable */
#define MMC5983MA_CTRL0_SET          0x08    /**< SET operation */
#define MMC5983MA_CTRL0_AUTO_SR      0x20    /**< Automatic SET/RESET */

#define MMC5983MA_CTRL1_BW0          0x01    /**< Bandwidth select bit 0 */
#define MMC5983MA_CTRL1_BW1          0x02    /**< Bandwidth select bit 1 */

#define MMC5983MA_CTRL2_CMM_EN       0x04    /**< Continuous measurement mode */
/** @} */

/**
 * @defgroup MMCCalibration Calibration Constants
 * @brief Conversion factors for raw data
 * @{
 */
#define MMC5983MA_SENSITIVITY_18BIT  16384.0f    /**< LSB per Gauss (18-bit mode) */
#define MMC5983MA_TEMP_SCALE         0.8f        /**< Temperature scale (C per LSB) */
#define MMC5983MA_TEMP_OFFSET        -75         /**< Temperature offset (C at 0 count) */
/** @} */

/**
 * @brief MMC5983MA driver context
 *
 * Contains all state for a single MMC5983MA device instance.
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        /**< SPI peripheral handle */
    uint8_t tx_buffer[9];           /**< DMA transmit buffer (1 cmd + 8 dummy) */
    uint8_t read_buffer[9];         /**< DMA receive buffer (1 dummy + 8 data) */
    volatile uint8_t data_ready;    /**< Flag set when new data available */

    /* Internal raw data (18-bit values) */
    int32_t mag_x_raw;              /**< Raw X-axis magnetic field */
    int32_t mag_y_raw;              /**< Raw Y-axis magnetic field */
    int32_t mag_z_raw;              /**< Raw Z-axis magnetic field */
    uint8_t temp_raw;               /**< Raw temperature */
} MMC5983MA_t;

/**
 * @defgroup MMCAPI MMC5983MA Public API
 * @brief Driver functions for MMC5983MA magnetometer
 * @{
 */

/**
 * @brief Initialize MMC5983MA driver
 *
 * Initializes the driver context and verifies communication.
 * Reads the Product ID register to confirm device presence.
 *
 * @param[out] dev  Pointer to driver context to initialize
 * @param[in]  hspi SPI peripheral handle
 *
 * @return true if initialization successful
 * @return false if communication error
 */
bool MMC5983MA_Init(MMC5983MA_t *dev, SPI_HandleTypeDef *hspi);

/**
 * @brief Configure MMC5983MA operating parameters
 *
 * Sets up the sensor with:
 * - Automatic SET/RESET enabled
 * - Continuous measurement mode
 * - Default bandwidth
 *
 * @param[in] dev Pointer to initialized driver context
 *
 * @return true if configuration successful
 * @return false if communication error
 */
bool MMC5983MA_Configure(MMC5983MA_t *dev);

/**
 * @brief Start DMA read of magnetic field data
 *
 * Initiates non-blocking DMA transfer to read all output registers.
 *
 * @param[in] dev Pointer to driver context
 *
 * @return true if DMA transfer started
 * @return false if SPI busy or error
 */
bool MMC5983MA_StartReadDMA(MMC5983MA_t *dev);

/**
 * @brief Process raw data into engineering units
 *
 * Converts raw 18-bit magnetic field data to Gauss.
 *
 * @param[in]  dev    Pointer to driver context with valid raw data
 * @param[out] output Pointer to MAG_t structure to populate
 *
 * @return true if data processed successfully
 * @return false if invalid parameters
 */
bool MMC5983MA_ProcessData(MMC5983MA_t *dev, MAG_t *output);

/**
 * @brief Parse DMA buffer into raw values
 *
 * Called from DMA complete callback to extract raw values.
 *
 * @param[in,out] dev Pointer to driver context
 */
void MMC5983MA_ParseDMABuffer(MMC5983MA_t *dev);

/**
 * @brief Check if new data is available
 *
 * @param[in] dev Pointer to driver context
 *
 * @return true if new data ready
 * @return false if no new data
 */
static inline bool MMC5983MA_IsDataReady(MMC5983MA_t *dev) {
    return dev->data_ready;
}

/** @} */ /* End of MMCAPI group */

#endif /* SENSORS_MMC5983MA_MMC5983MA_H_ */
