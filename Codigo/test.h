/*
 * test.h
 *
 *  Created on: Nov 5, 2025
 *      Author: texman
 */

#ifndef TEST_H_
#define TEST_H_

#include "defs.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * COMPILE-TIME CONFIGURATION FLAGS
 * ============================================================================ */

// Master enable/disable for all testing
#define TEST_ENABLED                    1

// Individual sensor simulation flags
#define TEST_IMU_ENABLED                1
#define TEST_MAGNETOMETER_ENABLED       1
#define TEST_BAROMETER_ENABLED          1
#define TEST_BNO055_ENABLED             1
#define TEST_GPS_ENABLED                1

// Data generation modes
#define TEST_MODE_RANDOM                0  // Random data within realistic ranges
#define TEST_MODE_LINEAR                1  // Linear increasing/decreasing values
#define TEST_MODE_STATIC                2  // Static/constant values

// Select the data generation mode for each sensor
#define TEST_IMU_MODE                   TEST_MODE_RANDOM
#define TEST_MAGNETOMETER_MODE          TEST_MODE_RANDOM
#define TEST_BAROMETER_MODE             TEST_MODE_RANDOM
#define TEST_BNO055_MODE                TEST_MODE_RANDOM
#define TEST_GPS_MODE                   TEST_MODE_RANDOM

// Output options
#define TEST_PRINT_DATA                 1  // Print generated data to console
#define TEST_PRINT_STATISTICS           1  // Print generation statistics

/* ============================================================================
 * REALISTIC DATA RANGES
 * ============================================================================ */

// IMU (ASM330LHHX) - Accelerometer and Gyroscope ranges
#define IMU_ACCEL_MIN                   -16.0f  // m/s^2 (±16g)
#define IMU_ACCEL_MAX                    16.0f  // m/s^2
#define IMU_GYRO_MIN                   -2000.0f // dps (±2000°/s)
#define IMU_GYRO_MAX                    2000.0f // dps
#define IMU_TEMP_MIN                     -40.0f // Celsius
#define IMU_TEMP_MAX                      85.0f // Celsius

// Magnetometer (MMC5983MA) - Magnetic field in µT
#define MAG_FIELD_MIN                   -49152.0f // µT
#define MAG_FIELD_MAX                    49152.0f // µT
#define MAG_TEMP_MIN                     -40.0f   // Celsius
#define MAG_TEMP_MAX                      85.0f   // Celsius

// Barometer (MS5607-02BA) - Pressure and altitude
#define BARO_PRESSURE_MIN                300.0f  // mbar (high altitude)
#define BARO_PRESSURE_MAX               1100.0f  // mbar (sea level)
#define BARO_TEMP_MIN                   -40.0f   // Celsius
#define BARO_TEMP_MAX                     85.0f  // Celsius
#define BARO_ALTITUDE_MIN              -100.0f   // meters (below sea level)
#define BARO_ALTITUDE_MAX              10000.0f  // meters (high altitude)

// BNO055 (9-DOF Absolute Orientation)
#define BNO_HEADING_MIN                   0.0f   // degrees
#define BNO_HEADING_MAX                 360.0f   // degrees
#define BNO_ROLL_MIN                   -180.0f   // degrees
#define BNO_ROLL_MAX                    180.0f   // degrees
#define BNO_PITCH_MIN                   -90.0f   // degrees
#define BNO_PITCH_MAX                    90.0f   // degrees
#define BNO_ACCEL_MIN                   -16000.0f // mg (±16g)
#define BNO_ACCEL_MAX                    16000.0f // mg
#define BNO_GYRO_MIN                    -2000.0f  // dps
#define BNO_GYRO_MAX                     2000.0f  // dps
#define BNO_MAG_MIN                     -49152.0f // µT
#define BNO_MAG_MAX                      49152.0f // µT

// GPS (U-Blox NEO-9M)
#define GPS_LAT_MIN                     -90.0    // degrees
#define GPS_LAT_MAX                      90.0    // degrees
#define GPS_LON_MIN                    -180.0    // degrees
#define GPS_LON_MAX                     180.0    // degrees
#define GPS_ALT_MIN                      -100.0f  // meters
#define GPS_ALT_MAX                      10000.0f // meters
#define GPS_SPEED_MIN                     0.0f   // knots
#define GPS_SPEED_MAX                   100.0f   // knots
#define GPS_COURSE_MIN                    0.0f   // degrees
#define GPS_COURSE_MAX                  360.0f   // degrees

/* ============================================================================
 * UPDATE RATES (milliseconds)
 * ============================================================================ */

#define TEST_IMU_UPDATE_RATE_MS          5     // 200 Hz
#define TEST_MAG_UPDATE_RATE_MS         10     // 100 Hz
#define TEST_BARO_UPDATE_RATE_MS        20     // 50 Hz
#define TEST_BNO_UPDATE_RATE_MS         10     // 100 Hz
#define TEST_GPS_UPDATE_RATE_MS        100     // 10 Hz

/* ============================================================================
 * DATA GENERATION STATISTICS
 * ============================================================================ */

typedef struct {
    uint32_t imu_generated;
    uint32_t mag_generated;
    uint32_t baro_generated;
    uint32_t bno_generated;
    uint32_t gps_generated;
    uint32_t total_samples;
    uint32_t generation_errors;
} test_statistics_t;

/* ============================================================================
 * PUBLIC FUNCTION DECLARATIONS
 * ============================================================================ */

/**
 * @brief Initialize the test framework
 * @return true if successful, false otherwise
 */
bool test_init(void);

/**
 * @brief Terminate the test framework
 */
void test_deinit(void);

/**
 * @brief Check if testing is enabled globally
 * @return true if testing is enabled, false otherwise
 */
bool test_is_enabled(void);

/* ============================================================================
 * IMU DATA GENERATION
 * ============================================================================ */

/**
 * @brief Generate simulated IMU data
 * @param imu_data Pointer to IMU_t structure to populate
 * @param sample_num Sample number (for linear mode progression)
 * @return true if successful, false otherwise
 */
bool test_generate_imu_data(IMU_t *imu_data, uint32_t sample_num);

/**
 * @brief Inject generated IMU data (calls data_handler_store_imu)
 * @return true if successful, false otherwise
 */
bool test_inject_imu_data(void);

/* ============================================================================
 * MAGNETOMETER DATA GENERATION
 * ============================================================================ */

/**
 * @brief Generate simulated Magnetometer data
 * @param mag_data Pointer to MAG_t structure to populate
 * @param sample_num Sample number (for linear mode progression)
 * @return true if successful, false otherwise
 */
bool test_generate_mag_data(MAG_t *mag_data, uint32_t sample_num);

/**
 * @brief Inject generated Magnetometer data
 * @return true if successful, false otherwise
 */
bool test_inject_mag_data(void);

/* ============================================================================
 * BAROMETER DATA GENERATION
 * ============================================================================ */

/**
 * @brief Generate simulated Barometer data
 * @param baro_data Pointer to BARO_t structure to populate
 * @param sample_num Sample number (for linear mode progression)
 * @return true if successful, false otherwise
 */
bool test_generate_baro_data(BARO_t *baro_data, uint32_t sample_num);

/**
 * @brief Inject generated Barometer data
 * @return true if successful, false otherwise
 */
bool test_inject_baro_data(void);

/* ============================================================================
 * BNO055 DATA GENERATION
 * ============================================================================ */

/**
 * @brief Generate simulated BNO055 data (9-DOF)
 * @param bno_data Pointer to BNO_t structure to populate
 * @param sample_num Sample number (for linear mode progression)
 * @return true if successful, false otherwise
 */
bool test_generate_bno_data(BNO_t *bno_data, uint32_t sample_num);

/**
 * @brief Inject generated BNO055 data
 * @return true if successful, false otherwise
 */
bool test_inject_bno_data(void);

/* ============================================================================
 * GPS DATA GENERATION
 * ============================================================================ */

/**
 * @brief Generate simulated GPS data
 * @param gps_data Pointer to GPS_t structure to populate
 * @param sample_num Sample number (for linear mode progression)
 * @return true if successful, false otherwise
 */
bool test_generate_gps_data(GPS_t *gps_data, uint32_t sample_num);

/**
 * @brief Inject generated GPS data
 * @return true if successful, false otherwise
 */
bool test_inject_gps_data(void);

/* ============================================================================
 * BATCH GENERATION AND STATISTICS
 * ============================================================================ */

/**
 * @brief Generate and inject all enabled sensor data (single cycle)
 * @return true if at least one sensor generated data successfully
 */
bool test_generate_all_sensors(void);

/**
 * @brief Get current test statistics
 * @param stats Pointer to statistics structure to populate
 */
void test_get_statistics(test_statistics_t *stats);

/**
 * @brief Reset test statistics
 */
void test_reset_statistics(void);

/**
 * @brief Print current statistics to console
 */
void test_print_statistics(void);

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Get a random float value within specified range
 * @param min Minimum value
 * @param max Maximum value
 * @return Random float value between min and max
 */
float test_random_float(float min, float max);

/**
 * @brief Get a random integer value within specified range
 * @param min Minimum value
 * @param max Maximum value
 * @return Random integer value between min and max
 */
int32_t test_random_int(int32_t min, int32_t max);

/**
 * @brief Set seed for random number generation (for reproducibility)
 * @param seed Seed value
 */
void test_set_random_seed(uint32_t seed);

#endif /* TEST_H_ */
