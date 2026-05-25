/**
 * @file ASM330LHHX.h
 * @brief ASM330LHHX 6-axis IMU driver for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This driver provides an interface to the STMicroelectronics ASM330LHHX
 * automotive-grade 6-axis inertial measurement unit (accelerometer + gyroscope).
 *
 * @section asm330_features Features
 * - 3-axis accelerometer: +/-2/4/8/16g selectable
 * - 3-axis gyroscope: +/-125/250/500/1000/2000 dps selectable
 * - SPI interface up to 10 MHz (connected via SPI_IMU_BARO; see config.h)
 * - Hardware interrupt for data ready
 * - DMA-based data transfer for efficiency
 * - Supports multiple board targets (F446ZE Nucleo, H743ZI Buzz V4)
 *
 * @section asm330_usage Usage
 * @code
 * ASM330LHHX_t imu;
 * IMU_t data;
 *
 * // Initialize
 * ASM330LHHX_Init(&imu, &hspi1);
 * ASM330LHHX_Configure(&imu);
 *
 * // In interrupt handler
 * ASM330LHHX_StartReadDMA(&imu);
 *
 * // After DMA complete
 * ASM330LHHX_ParseDMABuffer(&imu);
 * ASM330LHHX_ProcessData(&imu, &data);
 * @endcode
 *
 * @see IMU_t for output data structure
 */

#ifndef SENSORS_ASM330LHHX_ASM330LHHX_H_
#define SENSORS_ASM330LHHX_ASM330LHHX_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "asm330lhhx_reg.h"
#include "defs.h"

/** @brief Boot time required after power-on (milliseconds) */
#define BOOT_TIME 10

/**
 * @brief ASM330LHHX driver context
 *
 * Contains all state for a single ASM330LHHX device instance.
 * Maintains SPI handle, DMA buffers, and raw sensor data.
 */
typedef struct {
    SPI_HandleTypeDef *hspi;        /**< SPI peripheral handle (SPI_IMU_BARO from config.h) */
    uint8_t tx_buffer[15];          /**< DMA transmit buffer (kept in struct for DMA cache coherency) */
    uint8_t read_buffer[15];        /**< DMA receive buffer (1 cmd + 14 data bytes) */
    volatile uint8_t data_ready;    /**< Flag set when new data available */

    /* Internal raw data (parsed from DMA buffer) */
    int16_t accel_raw[3];           /**< Raw accelerometer X,Y,Z */
    int16_t gyro_raw[3];            /**< Raw gyroscope X,Y,Z */
    int16_t temp_raw;               /**< Raw temperature */
} ASM330LHHX_t;

/**
 * @defgroup ASM330API ASM330LHHX Public API
 * @brief Driver functions for ASM330LHHX IMU
 * @{
 */

/**
 * @brief Initialize ASM330LHHX driver
 *
 * Initializes the driver context and verifies communication with the sensor.
 * Reads the WHO_AM_I register to confirm device presence.
 *
 * @param[out] dev  Pointer to driver context to initialize
 * @param[in]  hspi SPI peripheral handle (must be initialized)
 *
 * @return true if initialization successful and device responds
 * @return false if communication error or wrong device ID
 */
bool ASM330LHHX_Init(ASM330LHHX_t *dev, SPI_HandleTypeDef *hspi);

/**
 * @brief Configure ASM330LHHX operating parameters
 *
 * Sets up the sensor with default configuration:
 * - Accelerometer: +/-16g range, 416 Hz ODR
 * - Gyroscope: +/-2000 dps range, 416 Hz ODR
 * - Data ready interrupt enabled on INT1
 *
 * @param[in] dev Pointer to initialized driver context
 *
 * @return true if configuration successful
 * @return false if communication error
 *
 * @pre ASM330LHHX_Init() must have been called successfully
 */
bool ASM330LHHX_Configure(ASM330LHHX_t *dev);

/**
 * @brief Start DMA read of sensor data
 *
 * Initiates a non-blocking DMA transfer to read all sensor registers
 * (accelerometer, gyroscope, temperature) in a single burst.
 *
 * @param[in] dev Pointer to driver context
 *
 * @return true if DMA transfer started
 * @return false if SPI busy or error
 *
 * @note Call ASM330LHHX_ParseDMABuffer() after DMA complete interrupt
 */
bool ASM330LHHX_StartReadDMA(ASM330LHHX_t *dev);

/**
 * @brief Process raw data into engineering units
 *
 * Converts raw sensor data to physical units:
 * - Acceleration: g (1g = 9.81 m/s^2)
 * - Angular rate: degrees per second (dps)
 * - Temperature: degrees Celsius
 *
 * @param[in]  dev    Pointer to driver context with valid raw data
 * @param[out] output Pointer to IMU_t structure to populate
 *
 * @return true if data processed successfully
 * @return false if invalid parameters
 *
 * @pre ASM330LHHX_ParseDMABuffer() must have been called
 */
bool ASM330LHHX_ProcessData(ASM330LHHX_t *dev, IMU_t *output);

/**
 * @brief Parse DMA buffer into raw values
 *
 * Called from DMA complete callback to extract raw sensor values
 * from the receive buffer. Updates the internal raw data fields.
 *
 * @param[in,out] dev Pointer to driver context with completed DMA transfer
 *
 * @note Call this from HAL_SPI_RxCpltCallback or similar
 */
void ASM330LHHX_ParseDMABuffer(ASM330LHHX_t *dev);

/** @} */ /* End of ASM330API group */

#endif /* SENSORS_ASM330LHHX_ASM330LHHX_H_ */
