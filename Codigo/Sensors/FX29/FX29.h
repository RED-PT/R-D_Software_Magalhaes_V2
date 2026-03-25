/**
 * @file FX29.h
 * @brief FX29 load cell driver for the Magalhaes Static Test Stand
 * @author Tomas Teixeira
 * @date January 2026
 *
 * This driver provides an interface to the TE Connectivity FX29 series
 * force sensors for thrust measurement in static motor tests.
 *
 * @section fx29_features Features
 * - Available ranges: 50N, 125N, 250N, 500N, 1000N
 * - I2C interface (multiple address options)
 * - 14-bit ADC resolution
 * - Tare/zero offset calibration
 * - Status monitoring (stale data, faults)
 *
 * @section fx29_measurement Measurement
 * The sensor outputs raw counts proportional to applied force:
 * - 1000 counts = 0% of range (zero force)
 * - 15000 counts = 100% of range (full scale)
 *
 * @section fx29_usage Usage
 * @code
 * FX29_t loadcell;
 * LOADCELL_t data;
 *
 * FX29_Init(&loadcell, &hi2c1, FX29_ADDR_0, FX29_RANGE_250N);
 *
 * // Tare before test
 * FX29_Tare(&loadcell);
 *
 * // Read during test
 * FX29_ReadWithPWM(&loadcell, &data, current_pwm);
 * @endcode
 *
 * @see LOADCELL_t for output data structure
 */

#ifndef SENSORS_FX29_FX29_H_
#define SENSORS_FX29_FX29_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "defs.h"

/**
 * @defgroup FX29Addresses I2C Addresses
 * @brief Available I2C addresses (selectable by part number)
 * @{
 */
#define FX29_ADDR_0     0x28    /**< Default address */
#define FX29_ADDR_1     0x36    /**< Alternate address 1 */
#define FX29_ADDR_2     0x46    /**< Alternate address 2 */
#define FX29_ADDR_3     0x48    /**< Alternate address 3 */
#define FX29_ADDR_4     0x51    /**< Alternate address 4 */
/** @} */

/**
 * @defgroup FX29Status Status Bits
 * @brief Status field values from upper 2 bits of first byte
 * @{
 */
#define FX29_STATUS_MASK        0xC0    /**< Mask for status bits */
#define FX29_STATUS_NORMAL      0x00    /**< Normal operation, valid data */
#define FX29_STATUS_COMMAND     0x40    /**< Device in command mode */
#define FX29_STATUS_STALE       0x80    /**< Stale data (already read) */
#define FX29_STATUS_FAULT       0xC0    /**< Diagnostic fault condition */
/** @} */

/**
 * @defgroup FX29Counts Output Count Range
 * @brief ADC output count boundaries
 * @{
 */
#define FX29_COUNT_MIN          1000    /**< Count at 0% force */
#define FX29_COUNT_MAX          15000   /**< Count at 100% force */
#define FX29_COUNT_SPAN         14000   /**< Full scale span (counts) */
/** @} */

/**
 * @defgroup FX29Timing Timing Constants
 * @brief Response time specifications
 * @{
 */
#define FX29_RESPONSE_TIME_MS   3       /**< Response time (non-sleep mode) */
#define FX29_SLEEP_WAKEUP_MS    9       /**< Wakeup time from sleep */
/** @} */

/**
 * @defgroup FX29Tare Tare Configuration
 * @brief Parameters for zero calibration
 * @{
 */
#define FX29_TARE_SAMPLES       20      /**< Number of samples for tare averaging */
#define FX29_TARE_DELAY_MS      10      /**< Delay between tare samples (ms) */
/** @} */

/**
 * @brief Available force sensor ranges
 *
 * Select based on the specific FX29 part number installed.
 */
typedef enum {
    FX29_RANGE_50N   = 50,      /**< 50 Newton range */
    FX29_RANGE_125N  = 125,     /**< 125 Newton range */
    FX29_RANGE_250N  = 250,     /**< 250 Newton range */
    FX29_RANGE_500N  = 500,     /**< 500 Newton range */
    FX29_RANGE_1000N = 1000     /**< 1000 Newton range */
} FX29_Range_t;

/**
 * @brief Sensor read status codes
 *
 * Indicates the result of the last read operation.
 */
typedef enum {
    FX29_OK = 0,            /**< Valid fresh data */
    FX29_STALE_DATA,        /**< Data already read (not updated) */
    FX29_FAULT,             /**< Sensor diagnostic fault */
    FX29_COMM_ERROR         /**< I2C communication error */
} FX29_Status_t;

/**
 * @brief FX29 driver context
 *
 * Contains all state for a single FX29 load cell instance.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;    /**< I2C peripheral handle */
    uint8_t i2c_addr;           /**< Device I2C address (7-bit) */
    FX29_Range_t force_range_n; /**< Sensor full scale range (Newtons) */
    int32_t tare_offset;        /**< Raw counts offset for zeroing */
    bool is_tared;              /**< true if tare calibration applied */
    float scale_factor;         /**< Calibration scale factor (default 1.0) */
    bool is_calibrated;         /**< true if two-point calibration applied */
} FX29_t;

/**
 * @defgroup FX29API FX29 Public API
 * @brief Driver functions for FX29 load cell
 * @{
 */

/**
 * @brief Initialize FX29 driver
 *
 * Sets up the driver context with I2C handle, address, and range.
 *
 * @param[out] dev      Pointer to driver context to initialize
 * @param[in]  hi2c     I2C peripheral handle
 * @param[in]  i2c_addr Device I2C address (7-bit, e.g., FX29_ADDR_0)
 * @param[in]  range    Sensor force range (must match hardware)
 *
 * @return true if initialization successful
 * @return false if invalid parameters
 */
bool FX29_Init(FX29_t *dev, I2C_HandleTypeDef *hi2c,
               uint8_t i2c_addr, FX29_Range_t range);

/**
 * @brief Check if sensor is responding
 *
 * Attempts to communicate with the sensor to verify connection.
 *
 * @param[in] dev Pointer to initialized driver context
 *
 * @return true if sensor responds
 * @return false if no response or error
 */
bool FX29_IsConnected(FX29_t *dev);

/**
 * @brief Read force measurement
 *
 * Reads the current force from the sensor.
 * Applies tare offset if calibrated.
 *
 * @param[in]  dev    Pointer to driver context
 * @param[out] output Pointer to LOADCELL_t structure to populate
 *
 * @return true if read successful
 * @return false if communication error
 *
 * @note Blocking function (~3ms)
 */
bool FX29_Read(FX29_t *dev, LOADCELL_t *output);

/**
 * @brief Read force with associated PWM value
 *
 * Reads force and stores the current PWM command for correlation
 * in test logging. Used during motor throttle sweeps.
 *
 * @param[in]  dev       Pointer to driver context
 * @param[out] output    Pointer to LOADCELL_t structure to populate
 * @param[in]  pwm_value Current PWM command value to associate
 *
 * @return true if read successful
 * @return false if communication error
 */
bool FX29_ReadWithPWM(FX29_t *dev, LOADCELL_t *output, uint16_t pwm_value);

/** @} */

/**
 * @defgroup FX29Tare Tare/Zero Functions
 * @brief Zero calibration functions
 * @{
 */

/**
 * @brief Start tare calibration
 *
 * Prepares for tare calibration by clearing previous offset.
 * Call this before FX29_Tare().
 *
 * @param[in,out] dev Pointer to driver context
 *
 * @return true if started
 * @return false if error
 */
bool FX29_StartTare(FX29_t *dev);

/**
 * @brief Perform blocking tare calibration
 *
 * Takes multiple samples and calculates the zero offset.
 * After this call, readings will be relative to current force.
 *
 * @param[in,out] dev Pointer to driver context
 *
 * @return true if tare successful
 * @return false if error
 *
 * @note Blocking function (~200ms for 20 samples)
 * @note Ensure no force is applied during tare
 */
bool FX29_Tare(FX29_t *dev);

/**
 * @brief Clear tare calibration
 *
 * Removes the zero offset, returning to absolute measurements.
 *
 * @param[in,out] dev Pointer to driver context
 */
void FX29_ClearTare(FX29_t *dev);

/**
 * @brief Two-point scale factor calibration
 *
 * Calculates a scale factor based on two known weights.
 * Call this after tare with no load, then apply two known weights.
 *
 * @param[in,out] dev Pointer to driver context
 * @param[in] known_weight1_n First known weight in Newtons
 * @param[in] measured1_n First measured reading in Newtons (from FX29_Read)
 * @param[in] known_weight2_n Second known weight in Newtons
 * @param[in] measured2_n Second measured reading in Newtons (from FX29_Read)
 *
 * @return true if calibration successful
 * @return false if invalid parameters (division by zero)
 *
 * @note Use weights that span your expected measurement range
 * @note Formula: scale = (known2 - known1) / (measured2 - measured1)
 */
bool FX29_Calibrate(FX29_t *dev, float known_weight1_n, float measured1_n,
                    float known_weight2_n, float measured2_n);

/**
 * @brief Set scale factor directly
 *
 * Allows setting the scale factor without running calibration.
 * Useful for loading a previously calculated value.
 *
 * @param[in,out] dev Pointer to driver context
 * @param[in] scale_factor Scale factor to apply (typically 0.8 - 1.2)
 */
void FX29_SetScaleFactor(FX29_t *dev, float scale_factor);

/**
 * @brief Get current scale factor
 *
 * @param[in] dev Pointer to driver context
 * @return Current scale factor (1.0 if not calibrated)
 */
float FX29_GetScaleFactor(FX29_t *dev);

/**
 * @brief Clear scale calibration
 *
 * Resets scale factor to 1.0
 *
 * @param[in,out] dev Pointer to driver context
 */
void FX29_ClearCalibration(FX29_t *dev);

/** @} */

/**
 * @defgroup FX29Utility Utility Functions
 * @brief Helper functions
 * @{
 */

/**
 * @brief Convert raw counts to force
 *
 * Applies the sensor range to convert ADC counts to Newtons.
 *
 * @param[in] dev    Pointer to driver context (for range)
 * @param[in] counts Raw ADC counts
 *
 * @return Force in Newtons
 */
float FX29_CountsToForce(FX29_t *dev, uint16_t counts);

/**
 * @brief Get status description string
 *
 * Returns a human-readable string for a status code.
 *
 * @param[in] status Status code
 *
 * @return Pointer to static string describing status
 */
const char* FX29_StatusToString(FX29_Status_t status);

/** @} */

#endif /* SENSORS_FX29_FX29_H_ */
