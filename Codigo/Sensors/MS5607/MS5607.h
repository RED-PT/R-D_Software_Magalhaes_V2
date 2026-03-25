/**
 * @file MS5607.h
 * @brief MS5607-02BA barometric pressure sensor driver with launch pad calibration
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This driver provides an interface to the TE Connectivity MS5607-02BA
 * barometric pressure sensor for altitude measurement in rocket applications.
 *
 * @section ms5607_features Features
 * - Pressure range: 10-1200 mbar
 * - Resolution: 0.024 mbar (24-bit ADC)
 * - Temperature compensated
 * - SPI interface up to 20 MHz
 * - Launch pad calibration for AGL altitude
 *
 * @section ms5607_calibration Calibration
 * Before flight, the barometer must be calibrated to establish the reference
 * pressure at ground level. This allows altitude to be reported as AGL
 * (Above Ground Level) rather than absolute altitude.
 *
 * Calibration process:
 * 1. Call MS5607_StartCalibration() to begin
 * 2. Collect samples with MS5607_AddCalibrationSample()
 * 3. Finalize with MS5607_FinishCalibration()
 * 4. Use MS5607_ReadWithCalibration() for AGL altitude
 *
 * @see BARO_t for output data structure
 * @see baro_calibration_t for calibration state
 */

#ifndef SENSORS_MS5607_MS5607_H_
#define SENSORS_MS5607_MS5607_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "defs.h"

/**
 * @defgroup MS5607Commands SPI Command Bytes
 * @brief MS5607 SPI protocol commands
 * @{
 */
#define CMD_RESET      0x1E     /**< Reset command */
#define CMD_CONV_D1    0x48     /**< Start pressure conversion (OSR=4096) */
#define CMD_CONV_D2    0x58     /**< Start temperature conversion (OSR=4096) */
#define CMD_ADC_READ   0x00     /**< Read ADC result */
#define CMD_PROM_READ  0xA0     /**< Read PROM calibration (base address) */
/** @} */

/**
 * @defgroup MS5607CalConfig Calibration Settings
 * @brief Parameters for launch pad calibration
 * @{
 */
#define BARO_CALIBRATION_SAMPLES    50      /**< Number of samples for averaging */
#define BARO_CALIBRATION_DELAY_MS   20      /**< Delay between samples (milliseconds) */
/** @} */

/**
 * @brief MS5607 driver context
 *
 * Contains SPI handle and chip select GPIO for a single MS5607 device.
 */
typedef struct {
    SPI_HandleTypeDef *hspi;    /**< SPI peripheral handle */
    GPIO_TypeDef *cs_port;      /**< Chip select GPIO port */
    uint16_t cs_pin;            /**< Chip select GPIO pin */
} MS5607_t;

/**
 * @brief Barometric calibration context
 *
 * Stores the reference pressure measured at launch pad for AGL calculations.
 * This structure should be stored in the flight computer context.
 */
typedef struct {
    float reference_pressure_mbar;  /**< Launch pad pressure (reference) */
    float reference_altitude_m;     /**< GPS altitude at calibration (optional) */
    float temperature_at_cal_c;     /**< Temperature during calibration */
    uint32_t calibration_timestamp; /**< System tick when calibrated */
    bool is_calibrated;             /**< true if calibration is valid */
    uint8_t samples_collected;      /**< Counter during calibration */
    float pressure_sum;             /**< Accumulator for averaging */
} baro_calibration_t;

/**
 * @defgroup MS5607API MS5607 Public API
 * @brief Driver functions for MS5607 barometer
 * @{
 */

/**
 * @brief Initialize MS5607 driver
 *
 * Sets up the driver context with SPI and GPIO handles.
 * Does not communicate with the sensor.
 *
 * @param[out] dev     Pointer to driver context to initialize
 * @param[in]  hspi    SPI peripheral handle
 * @param[in]  cs_port Chip select GPIO port
 * @param[in]  cs_pin  Chip select GPIO pin
 *
 * @return true if parameters valid
 * @return false if invalid parameters
 */
bool MS5607_Init(MS5607_t *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief Configure MS5607 and read calibration data
 *
 * Resets the sensor and reads factory calibration coefficients from PROM.
 * Must be called before any pressure readings.
 *
 * @param[in] dev Pointer to initialized driver context
 *
 * @return true if configuration successful
 * @return false if communication error or invalid calibration data
 */
bool MS5607_Configure(MS5607_t *dev);

/**
 * @brief Read temperature and pressure (blocking)
 *
 * Performs a complete measurement cycle:
 * 1. Start temperature conversion, wait, read
 * 2. Start pressure conversion, wait, read
 * 3. Apply temperature compensation
 * 4. Calculate altitude using standard atmosphere
 *
 * @param[in]  dev    Pointer to configured driver context
 * @param[out] output Pointer to BARO_t structure to populate
 *
 * @return true if reading successful
 * @return false if communication error
 *
 * @note Blocking function, takes ~20ms for both conversions
 * @note Altitude is relative to standard atmosphere (1013.25 mbar)
 */
bool MS5607_ReadTemperatureandPressure(MS5607_t *dev, BARO_t *output);

/** @} */

/**
 * @defgroup MS5607Calibration Calibration Functions
 * @brief Functions for launch pad calibration
 * @{
 */

/**
 * @brief Start calibration process
 *
 * Initializes the calibration context for sample collection.
 * Call this before collecting calibration samples.
 *
 * @param[in]  dev Pointer to driver context
 * @param[out] cal Pointer to calibration context to initialize
 *
 * @return true if started successfully
 * @return false if invalid parameters
 */
bool MS5607_StartCalibration(MS5607_t *dev, baro_calibration_t *cal);

/**
 * @brief Add a calibration sample
 *
 * Reads current pressure and adds to the running average.
 * Call this BARO_CALIBRATION_SAMPLES times with delays.
 *
 * @param[in]     dev Pointer to driver context
 * @param[in,out] cal Pointer to calibration context
 *
 * @return true if sample added (more samples needed)
 * @return false if all samples collected or error
 */
bool MS5607_AddCalibrationSample(MS5607_t *dev, baro_calibration_t *cal);

/**
 * @brief Finalize calibration
 *
 * Calculates the reference pressure from collected samples.
 * Sets is_calibrated flag if successful.
 *
 * @param[in,out] cal Pointer to calibration context with all samples
 *
 * @return true if calibration finalized successfully
 * @return false if insufficient samples
 */
bool MS5607_FinishCalibration(baro_calibration_t *cal);

/**
 * @brief Read with calibration applied
 *
 * Reads pressure and calculates altitude relative to the calibrated
 * reference (AGL - Above Ground Level).
 *
 * @param[in]  dev    Pointer to driver context
 * @param[out] output Pointer to BARO_t structure to populate
 * @param[in]  cal    Pointer to valid calibration context
 *
 * @return true if reading successful
 * @return false if communication error or not calibrated
 *
 * @pre MS5607_FinishCalibration() must have returned true
 */
bool MS5607_ReadWithCalibration(MS5607_t *dev, BARO_t *output,
                                 const baro_calibration_t *cal);

/** @} */

/**
 * @defgroup MS5607Utility Utility Functions
 * @brief Helper functions for altitude calculation
 * @{
 */

/**
 * @brief Convert pressure to altitude
 *
 * Uses the barometric formula to calculate altitude difference
 * between measured pressure and reference pressure.
 *
 * Formula: h = 44330 * (1 - (P/P0)^0.1903)
 *
 * @param[in] pressure_mbar     Measured pressure (millibars)
 * @param[in] ref_pressure_mbar Reference pressure (millibars)
 *
 * @return Altitude in meters above reference
 */
float MS5607_PressureToAltitude(float pressure_mbar, float ref_pressure_mbar);

/** @} */

#endif /* SENSORS_MS5607_MS5607_H_ */
