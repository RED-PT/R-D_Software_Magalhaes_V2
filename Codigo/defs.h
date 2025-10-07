/*
 * defs.c
 *
 *  Created on: Oct 6, 2025
 *      Author: texman
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_

//	C libraries
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

//	buffers
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "stream_buffer.h"


//	structures
//	IMU: ASM330LHHX (6-axis: 3-axis accelerometer + 3-axis gyroscope)
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature_c;
    uint32_t timestamp_ms; // Timestamp (milliseconds)
} IMU_t;

//	Magnetometer: MMC5983MA (3-axis magnetic field sensor)
typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
    float temperature_c;
    uint32_t timestamp_ms; // Timestamp (milliseconds)
} MAG_t;

//	Absolute-Orientation IMU: BNO055 (9-DOF with sensor fusion)
typedef struct {
    // Euler angles (absolute orientation)
    float heading_deg;    // Yaw/Heading (0-360°)
    float roll_deg;       // Roll (-180 to +180°)
    float pitch_deg;      // Pitch (-90 to +90°)

    // Quaternion (alternative orientation representation)
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    // Raw sensor data
    float accel_x_mg;
    float accel_y_mg;
    float accel_z_mg;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float mag_x_uT;
    float mag_y_uT;
    float mag_z_uT;

    uint8_t calibration_status; // System calibration status (0-3)
    uint32_t timestamp_ms; // Timestamp (milliseconds)
} BNO_t;

//	Altimeter: MS5607-02BA (Barometric pressure and temperature sensor)
typedef struct {
    float pressure_mbar;
    float temperature_c;
    float altitude_m;     // Calculated altitude (meters, requires reference pressure)
    uint32_t timestamp_ms; // Timestamp (milliseconds)
} BARO_t;

//	GPS: U-Blox Neo-9M
typedef struct{
	// Position
	double latitude_deg;  // Latitude (degrees, -90 to +90)
	double longitude_deg; // Longitude (degrees, -180 to +180)
	float altitude_m;     // Altitude above mean sea level (meters)

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock; // https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
    int satelites;
    float hdop; // https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)#Interpretation
    float msl_altitude;
    char msl_units;
    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;
    float magnetic_dev; // magnetic
    char magnetic_dev_unit;
//    // GLL
//    char gll_status;
//    // VTG - Course over ground, ground speed
//    float course_t; // ground speed true
//    char course_t_unit;
//    char speed_k_unit;
//    float speed_km; // speek km/hr
//    char speed_km_unit;

    uint32_t timestamp_ms; // Timestamp (milliseconds)
} GPS_t;

//	Temperature Readings
typedef struct {
	float  temp_ms;
	float temp_cpu;
	float  adc_voltage;
	uint32_t timestamp;
} temperature_readings_t;

// Extern Variables
extern IMU_t imu;
extern MAG_t mag;
extern BNO_t bno;
extern BARO_t baro;
extern GPS_t ublox_gps;

#endif /* INC_DEFS_H_ */
