/**
 * @file defs.h
 * @brief Core data structure definitions for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This file defines the data structures used throughout the flight computer
 * for sensor data, telemetry, and inter-thread communication.
 *
 * @note All sensor structures include a timestamp_ms field for data synchronization
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_

/* C standard libraries */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* FreeRTOS buffers for inter-thread communication */
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "stream_buffer.h"

/**
 * @defgroup SensorStructures Sensor Data Structures
 * @brief Data structures for sensor readings
 * @{
 */

/**
 * @brief IMU data structure (ASM330LHHX 6-axis sensor)
 *
 * Contains accelerometer and gyroscope readings from the ASM330LHHX
 * 6-axis inertial measurement unit.
 *
 * @note Accelerometer units: g (1g = 9.81 m/s^2)
 * @note Gyroscope units: degrees per second (dps)
 */
typedef struct {
    float accel_x;          /**< X-axis acceleration (g) */
    float accel_y;          /**< Y-axis acceleration (g) */
    float accel_z;          /**< Z-axis acceleration (g) */
    float gyro_x;           /**< X-axis angular rate (dps) */
    float gyro_y;           /**< Y-axis angular rate (dps) */
    float gyro_z;           /**< Z-axis angular rate (dps) */
    float temperature_c;    /**< Sensor temperature (Celsius) */
    uint32_t timestamp_ms;  /**< System timestamp (milliseconds) */
} IMU_t;

/**
 * @brief Magnetometer data structure (MMC5983MA 3-axis sensor)
 *
 * Contains magnetic field readings from the MMC5983MA sensor.
 * Used for heading calculation and compass functionality.
 *
 * @note Magnetic field units: Gauss
 */
typedef struct {
    float mag_x;            /**< X-axis magnetic field (Gauss) */
    float mag_y;            /**< Y-axis magnetic field (Gauss) */
    float mag_z;            /**< Z-axis magnetic field (Gauss) */
    float temperature_c;    /**< Sensor temperature (Celsius) */
    uint32_t timestamp_ms;  /**< System timestamp (milliseconds) */
} MAG_t;

/**
 * @brief 9-DOF Absolute Orientation IMU data structure (BNO055)
 *
 * Contains sensor-fused orientation data from the BNO055 9-axis IMU.
 * Provides both Euler angles and quaternion representations for attitude.
 *
 * The BNO055 performs on-chip sensor fusion, providing absolute orientation
 * without requiring external processing.
 */
typedef struct {
    /* Euler angles (absolute orientation) */
    float heading_deg;      /**< Yaw/Heading angle (0-360 degrees) */
    float roll_deg;         /**< Roll angle (-180 to +180 degrees) */
    float pitch_deg;        /**< Pitch angle (-90 to +90 degrees) */

    /* Quaternion (alternative orientation representation) */
    float quat_w;           /**< Quaternion W component */
    float quat_x;           /**< Quaternion X component */
    float quat_y;           /**< Quaternion Y component */
    float quat_z;           /**< Quaternion Z component */

    /* Raw sensor data */
    float accel_x_mg;       /**< X-axis acceleration (milli-g) */
    float accel_y_mg;       /**< Y-axis acceleration (milli-g) */
    float accel_z_mg;       /**< Z-axis acceleration (milli-g) */
    float gyro_x_dps;       /**< X-axis angular rate (dps) */
    float gyro_y_dps;       /**< Y-axis angular rate (dps) */
    float gyro_z_dps;       /**< Z-axis angular rate (dps) */
    float mag_x_uT;         /**< X-axis magnetic field (micro-Tesla) */
    float mag_y_uT;         /**< Y-axis magnetic field (micro-Tesla) */
    float mag_z_uT;         /**< Z-axis magnetic field (micro-Tesla) */

    uint8_t calibration_status; /**< System calibration status (0=uncalibrated, 3=fully calibrated) */
    uint32_t timestamp_ms;      /**< System timestamp (milliseconds) */
} BNO_t;

/**
 * @brief Barometer data structure (MS5607-02BA)
 *
 * Contains atmospheric pressure and temperature readings from the MS5607
 * barometric sensor. Altitude is calculated using the barometric formula.
 *
 * @note Altitude calculation requires a reference pressure (sea level or ground)
 */
typedef struct {
    float pressure_mbar;    /**< Atmospheric pressure (millibars) */
    float temperature_c;    /**< Air temperature (Celsius) */
    float altitude_m;       /**< Calculated altitude above reference (meters) */
    uint32_t timestamp_ms;  /**< System timestamp (milliseconds) */
} BARO_t;

/**
 * @brief GPS data structure (U-Blox Neo-9M/7M)
 *
 * Contains position, velocity, and quality data from the GPS receiver.
 * Position is in WGS84 geodetic coordinates.
 *
 * @note Coordinates are in decimal degrees format
 */
typedef struct {
    /* Position (WGS84 geodetic coordinates) */
    double dec_latitude;    /**< Latitude (-90 to +90 degrees) */
    double dec_longitude;   /**< Longitude (-180 to +180 degrees) */
    float msl_altitude;     /**< Altitude above mean sea level (meters) */

    /* Velocity */
    float speed_k;          /**< Ground speed (knots) */
    float course_d;         /**< Course over ground (0-360 degrees) */

    /* Quality indicators */
    uint8_t lock;           /**< Fix quality: 0=invalid, 1=GPS, 2=DGPS */
    uint8_t satellites;     /**< Number of satellites in use */
    float hdop;             /**< Horizontal dilution of precision (lower is better) */

    /* Time */
    float utc_time;         /**< UTC time in HHMMSS.sss format */

    uint32_t timestamp_ms;  /**< System timestamp (milliseconds) */
} GPS_t;

/**
 * @brief Load cell data structure (FX29)
 *
 * Contains force measurement data from the FX29 load cell.
 * Used in the static test stand for thrust measurement.
 */
typedef struct {
    float force_n;          /**< Calibrated force reading (Newtons) */
    float force_raw_n;      /**< Raw force before tare offset (Newtons) */
    uint16_t raw_counts;    /**< Raw ADC counts from sensor */
    uint16_t pwm_value;     /**< Associated PWM value during measurement */
    uint8_t status;         /**< Sensor status (0=OK, non-zero=error) */
    uint32_t timestamp_ms;  /**< System timestamp (milliseconds) */
} LOADCELL_t;

/** @} */ /* End of SensorStructures group */

#endif /* INC_DEFS_H_ */
