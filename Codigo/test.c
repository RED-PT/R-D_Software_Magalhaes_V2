/*
 * test.c
 *
 *  Created on: Nov 5, 2025
 *      Author: texman
 */

#include "test.h"
#include "print.h"
#include "Data Handler/flash_data_handler.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

#if TEST_ENABLED

/* ============================================================================
 * INTERNAL STATE AND STATISTICS
 * ============================================================================ */

static test_statistics_t test_stats = {0};
static uint32_t test_seed_value = 0;
static bool test_initialized = false;

// Last generated values for linear mode (smooth transitions)
static struct {
    float imu_accel_x, imu_accel_y, imu_accel_z;
    float imu_gyro_x, imu_gyro_y, imu_gyro_z;
    float imu_temp;

    float mag_x, mag_y, mag_z;
    float mag_temp;

    float baro_pressure, baro_temp, baro_altitude;

    float bno_heading, bno_roll, bno_pitch;
    float bno_accel_x, bno_accel_y, bno_accel_z;
    float bno_gyro_x, bno_gyro_y, bno_gyro_z;
    float bno_mag_x, bno_mag_y, bno_mag_z;

    double gps_lat, gps_lon;
    float gps_alt, gps_speed, gps_course;
} last_values = {0};

/* ============================================================================
 * RANDOM NUMBER GENERATION
 * ============================================================================ */

/**
 * @brief Linear congruential generator for deterministic randomness
 */
static uint32_t lcg_next(void) {
    test_seed_value = (1103515245U * test_seed_value + 12345U) & 0x7fffffffU;
    return test_seed_value;
}

float test_random_float(float min, float max) {
    if (!test_initialized) {
        return min;
    }
    float random = (float)lcg_next() / 0x7fffffffU;
    return min + random * (max - min);
}

int32_t test_random_int(int32_t min, int32_t max) {
    if (!test_initialized) {
        return min;
    }
    if (min > max) {
        return min;
    }
    int32_t range = max - min + 1;
    return min + (lcg_next() % range);
}

void test_set_random_seed(uint32_t seed) {
    test_seed_value = (seed == 0) ? 1 : seed;
}

/* ============================================================================
 * LINEAR INTERPOLATION HELPER
 * ============================================================================ */

/**
 * @brief Smoothly interpolate a value for linear mode
 */
static float interpolate_linear(float current, float min, float max, uint32_t sample_num) {
    // Create a smooth oscillating pattern
    uint32_t cycle_samples = 100;  // Cycle every 100 samples
    float cycle_pos = (float)(sample_num % cycle_samples) / cycle_samples;

    // Oscillate between min and max using sine wave
    float value = (max + min) / 2.0f + (max - min) / 2.0f * sinf(2.0f * 3.14159f * cycle_pos);
    return value;
}

/* ============================================================================
 * IMU DATA GENERATION
 * ============================================================================ */

bool test_generate_imu_data(IMU_t *imu_data, uint32_t sample_num) {
    if (!test_initialized || !imu_data) {
        return false;
    }

#if TEST_IMU_ENABLED

    switch (TEST_IMU_MODE) {
        case TEST_MODE_RANDOM:
            imu_data->accel_x = test_random_float(IMU_ACCEL_MIN, IMU_ACCEL_MAX);
            imu_data->accel_y = test_random_float(IMU_ACCEL_MIN, IMU_ACCEL_MAX);
            imu_data->accel_z = test_random_float(IMU_ACCEL_MIN, IMU_ACCEL_MAX);
            imu_data->gyro_x = test_random_float(IMU_GYRO_MIN, IMU_GYRO_MAX);
            imu_data->gyro_y = test_random_float(IMU_GYRO_MIN, IMU_GYRO_MAX);
            imu_data->gyro_z = test_random_float(IMU_GYRO_MIN, IMU_GYRO_MAX);
            imu_data->temperature_c = test_random_float(IMU_TEMP_MIN, IMU_TEMP_MAX);
            break;

        case TEST_MODE_LINEAR:
            imu_data->accel_x = interpolate_linear(last_values.imu_accel_x, IMU_ACCEL_MIN, IMU_ACCEL_MAX, sample_num);
            imu_data->accel_y = interpolate_linear(last_values.imu_accel_y, IMU_ACCEL_MIN, IMU_ACCEL_MAX, sample_num + 1);
            imu_data->accel_z = interpolate_linear(last_values.imu_accel_z, IMU_ACCEL_MIN, IMU_ACCEL_MAX, sample_num + 2);
            imu_data->gyro_x = interpolate_linear(last_values.imu_gyro_x, IMU_GYRO_MIN, IMU_GYRO_MAX, sample_num);
            imu_data->gyro_y = interpolate_linear(last_values.imu_gyro_y, IMU_GYRO_MIN, IMU_GYRO_MAX, sample_num + 1);
            imu_data->gyro_z = interpolate_linear(last_values.imu_gyro_z, IMU_GYRO_MIN, IMU_GYRO_MAX, sample_num + 2);
            imu_data->temperature_c = interpolate_linear(last_values.imu_temp, IMU_TEMP_MIN, IMU_TEMP_MAX, sample_num);

            // Update last values
            last_values.imu_accel_x = imu_data->accel_x;
            last_values.imu_accel_y = imu_data->accel_y;
            last_values.imu_accel_z = imu_data->accel_z;
            last_values.imu_gyro_x = imu_data->gyro_x;
            last_values.imu_gyro_y = imu_data->gyro_y;
            last_values.imu_gyro_z = imu_data->gyro_z;
            last_values.imu_temp = imu_data->temperature_c;
            break;

        case TEST_MODE_STATIC:
            imu_data->accel_x = 0.0f;
            imu_data->accel_y = 0.0f;
            imu_data->accel_z = 9.81f;  // 1g downward
            imu_data->gyro_x = 0.0f;
            imu_data->gyro_y = 0.0f;
            imu_data->gyro_z = 0.0f;
            imu_data->temperature_c = 25.0f;
            break;

        default:
            return false;
    }

    imu_data->timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    test_stats.imu_generated++;
    test_stats.total_samples++;

#if TEST_PRINT_DATA
    printf("[TEST] IMU: Accel=(%.2f, %.2f, %.2f) m/s² | Gyro=(%.2f, %.2f, %.2f) dps | Temp=%.1f°C\r\n",
           imu_data->accel_x, imu_data->accel_y, imu_data->accel_z,
           imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z,
           imu_data->temperature_c);
#endif

    return true;

#else
    return false;
#endif
}

bool test_inject_imu_data(void) {
    if (!test_initialized) {
        return false;
    }

    IMU_t imu_data;
    if (test_generate_imu_data(&imu_data, test_stats.imu_generated)) {
        data_handler_store_imu(&imu_data);
        return true;
    }
    return false;
}

/* ============================================================================
 * MAGNETOMETER DATA GENERATION
 * ============================================================================ */

bool test_generate_mag_data(MAG_t *mag_data, uint32_t sample_num) {
    if (!test_initialized || !mag_data) {
        return false;
    }

#if TEST_MAGNETOMETER_ENABLED

    switch (TEST_MAGNETOMETER_MODE) {
        case TEST_MODE_RANDOM:
            mag_data->mag_x = test_random_float(MAG_FIELD_MIN, MAG_FIELD_MAX);
            mag_data->mag_y = test_random_float(MAG_FIELD_MIN, MAG_FIELD_MAX);
            mag_data->mag_z = test_random_float(MAG_FIELD_MIN, MAG_FIELD_MAX);
            mag_data->temperature_c = test_random_float(MAG_TEMP_MIN, MAG_TEMP_MAX);
            break;

        case TEST_MODE_LINEAR:
            mag_data->mag_x = interpolate_linear(last_values.mag_x, MAG_FIELD_MIN, MAG_FIELD_MAX, sample_num);
            mag_data->mag_y = interpolate_linear(last_values.mag_y, MAG_FIELD_MIN, MAG_FIELD_MAX, sample_num + 1);
            mag_data->mag_z = interpolate_linear(last_values.mag_z, MAG_FIELD_MIN, MAG_FIELD_MAX, sample_num + 2);
            mag_data->temperature_c = interpolate_linear(last_values.mag_temp, MAG_TEMP_MIN, MAG_TEMP_MAX, sample_num);

            last_values.mag_x = mag_data->mag_x;
            last_values.mag_y = mag_data->mag_y;
            last_values.mag_z = mag_data->mag_z;
            last_values.mag_temp = mag_data->temperature_c;
            break;

        case TEST_MODE_STATIC:
            mag_data->mag_x = 20000.0f;   // Typical Earth magnetic field
            mag_data->mag_y = 0.0f;
            mag_data->mag_z = 40000.0f;
            mag_data->temperature_c = 25.0f;
            break;

        default:
            return false;
    }

    mag_data->timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    test_stats.mag_generated++;
    test_stats.total_samples++;

#if TEST_PRINT_DATA
    printf("[TEST] MAG: Field=(%.2f, %.2f, %.2f) µT | Temp=%.1f°C\r\n",
           mag_data->mag_x, mag_data->mag_y, mag_data->mag_z,
           mag_data->temperature_c);
#endif

    return true;

#else
    return false;
#endif
}

bool test_inject_mag_data(void) {
    if (!test_initialized) {
        return false;
    }

    MAG_t mag_data;
    if (test_generate_mag_data(&mag_data, test_stats.mag_generated)) {
        data_handler_store_mag(&mag_data);
        return true;
    }
    return false;
}

/* ============================================================================
 * BAROMETER DATA GENERATION
 * ============================================================================ */

bool test_generate_baro_data(BARO_t *baro_data, uint32_t sample_num) {
    if (!test_initialized || !baro_data) {
        return false;
    }

#if TEST_BAROMETER_ENABLED

    switch (TEST_BAROMETER_MODE) {
        case TEST_MODE_RANDOM:
            baro_data->pressure_mbar = test_random_float(BARO_PRESSURE_MIN, BARO_PRESSURE_MAX);
            baro_data->temperature_c = test_random_float(BARO_TEMP_MIN, BARO_TEMP_MAX);
            baro_data->altitude_m = test_random_float(BARO_ALTITUDE_MIN, BARO_ALTITUDE_MAX);
            break;

        case TEST_MODE_LINEAR:
            baro_data->pressure_mbar = interpolate_linear(last_values.baro_pressure, BARO_PRESSURE_MIN, BARO_PRESSURE_MAX, sample_num);
            baro_data->temperature_c = interpolate_linear(last_values.baro_temp, BARO_TEMP_MIN, BARO_TEMP_MAX, sample_num);
            baro_data->altitude_m = interpolate_linear(last_values.baro_altitude, BARO_ALTITUDE_MIN, BARO_ALTITUDE_MAX, sample_num);

            last_values.baro_pressure = baro_data->pressure_mbar;
            last_values.baro_temp = baro_data->temperature_c;
            last_values.baro_altitude = baro_data->altitude_m;
            break;

        case TEST_MODE_STATIC:
            baro_data->pressure_mbar = 1013.25f;  // Standard atmosphere
            baro_data->temperature_c = 15.0f;
            baro_data->altitude_m = 0.0f;         // Sea level
            break;

        default:
            return false;
    }

    baro_data->timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    test_stats.baro_generated++;
    test_stats.total_samples++;

#if TEST_PRINT_DATA
    printf("[TEST] BARO: Pressure=%.2f mbar | Temp=%.1f°C | Altitude=%.1f m\r\n",
           baro_data->pressure_mbar, baro_data->temperature_c, baro_data->altitude_m);
#endif

    return true;

#else
    return false;
#endif
}

bool test_inject_baro_data(void) {
    if (!test_initialized) {
        return false;
    }

    BARO_t baro_data;
    if (test_generate_baro_data(&baro_data, test_stats.baro_generated)) {
        data_handler_store_baro(&baro_data);
        return true;
    }
    return false;
}

/* ============================================================================
 * BNO055 DATA GENERATION (9-DOF)
 * ============================================================================ */

bool test_generate_bno_data(BNO_t *bno_data, uint32_t sample_num) {
    if (!test_initialized || !bno_data) {
        return false;
    }

#if TEST_BNO055_ENABLED

    switch (TEST_BNO055_MODE) {
        case TEST_MODE_RANDOM:
            bno_data->heading_deg = test_random_float(BNO_HEADING_MIN, BNO_HEADING_MAX);
            bno_data->roll_deg = test_random_float(BNO_ROLL_MIN, BNO_ROLL_MAX);
            bno_data->pitch_deg = test_random_float(BNO_PITCH_MIN, BNO_PITCH_MAX);

            // Generate random quaternion (rough approximation)
            float angle = test_random_float(0.0f, 2.0f * 3.14159f);
            bno_data->quat_w = cosf(angle / 2.0f);
            bno_data->quat_x = sinf(angle / 2.0f);
            bno_data->quat_y = test_random_float(-1.0f, 1.0f) * 0.1f;
            bno_data->quat_z = test_random_float(-1.0f, 1.0f) * 0.1f;

            bno_data->accel_x_mg = test_random_float(BNO_ACCEL_MIN, BNO_ACCEL_MAX);
            bno_data->accel_y_mg = test_random_float(BNO_ACCEL_MIN, BNO_ACCEL_MAX);
            bno_data->accel_z_mg = test_random_float(BNO_ACCEL_MIN, BNO_ACCEL_MAX);

            bno_data->gyro_x_dps = test_random_float(BNO_GYRO_MIN, BNO_GYRO_MAX);
            bno_data->gyro_y_dps = test_random_float(BNO_GYRO_MIN, BNO_GYRO_MAX);
            bno_data->gyro_z_dps = test_random_float(BNO_GYRO_MIN, BNO_GYRO_MAX);

            bno_data->mag_x_uT = test_random_float(BNO_MAG_MIN, BNO_MAG_MAX);
            bno_data->mag_y_uT = test_random_float(BNO_MAG_MIN, BNO_MAG_MAX);
            bno_data->mag_z_uT = test_random_float(BNO_MAG_MIN, BNO_MAG_MAX);

            bno_data->calibration_status = test_random_int(0, 3);
            break;

        case TEST_MODE_LINEAR:
            bno_data->heading_deg = interpolate_linear(last_values.bno_heading, BNO_HEADING_MIN, BNO_HEADING_MAX, sample_num);
            bno_data->roll_deg = interpolate_linear(last_values.bno_roll, BNO_ROLL_MIN, BNO_ROLL_MAX, sample_num + 1);
            bno_data->pitch_deg = interpolate_linear(last_values.bno_pitch, BNO_PITCH_MIN, BNO_PITCH_MAX, sample_num + 2);

            bno_data->accel_x_mg = interpolate_linear(last_values.bno_accel_x, BNO_ACCEL_MIN, BNO_ACCEL_MAX, sample_num);
            bno_data->accel_y_mg = interpolate_linear(last_values.bno_accel_y, BNO_ACCEL_MIN, BNO_ACCEL_MAX, sample_num + 1);
            bno_data->accel_z_mg = interpolate_linear(last_values.bno_accel_z, BNO_ACCEL_MIN, BNO_ACCEL_MAX, sample_num + 2);

            bno_data->gyro_x_dps = interpolate_linear(last_values.bno_gyro_x, BNO_GYRO_MIN, BNO_GYRO_MAX, sample_num);
            bno_data->gyro_y_dps = interpolate_linear(last_values.bno_gyro_y, BNO_GYRO_MIN, BNO_GYRO_MAX, sample_num + 1);
            bno_data->gyro_z_dps = interpolate_linear(last_values.bno_gyro_z, BNO_GYRO_MIN, BNO_GYRO_MAX, sample_num + 2);

            bno_data->mag_x_uT = interpolate_linear(last_values.bno_mag_x, BNO_MAG_MIN, BNO_MAG_MAX, sample_num);
            bno_data->mag_y_uT = interpolate_linear(last_values.bno_mag_y, BNO_MAG_MIN, BNO_MAG_MAX, sample_num + 1);
            bno_data->mag_z_uT = interpolate_linear(last_values.bno_mag_z, BNO_MAG_MIN, BNO_MAG_MAX, sample_num + 2);

            bno_data->calibration_status = 3;  // Fully calibrated

            // Update last values
            last_values.bno_heading = bno_data->heading_deg;
            last_values.bno_roll = bno_data->roll_deg;
            last_values.bno_pitch = bno_data->pitch_deg;
            last_values.bno_accel_x = bno_data->accel_x_mg;
            last_values.bno_accel_y = bno_data->accel_y_mg;
            last_values.bno_accel_z = bno_data->accel_z_mg;
            last_values.bno_gyro_x = bno_data->gyro_x_dps;
            last_values.bno_gyro_y = bno_data->gyro_y_dps;
            last_values.bno_gyro_z = bno_data->gyro_z_dps;
            last_values.bno_mag_x = bno_data->mag_x_uT;
            last_values.bno_mag_y = bno_data->mag_y_uT;
            last_values.bno_mag_z = bno_data->mag_z_uT;
            break;

        case TEST_MODE_STATIC:
            bno_data->heading_deg = 0.0f;
            bno_data->roll_deg = 0.0f;
            bno_data->pitch_deg = 0.0f;
            bno_data->quat_w = 1.0f;
            bno_data->quat_x = 0.0f;
            bno_data->quat_y = 0.0f;
            bno_data->quat_z = 0.0f;
            bno_data->accel_x_mg = 0.0f;
            bno_data->accel_y_mg = 0.0f;
            bno_data->accel_z_mg = 9810.0f;
            bno_data->gyro_x_dps = 0.0f;
            bno_data->gyro_y_dps = 0.0f;
            bno_data->gyro_z_dps = 0.0f;
            bno_data->mag_x_uT = 20000.0f;
            bno_data->mag_y_uT = 0.0f;
            bno_data->mag_z_uT = 40000.0f;
            bno_data->calibration_status = 3;
            break;

        default:
            return false;
    }

    bno_data->timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    test_stats.bno_generated++;
    test_stats.total_samples++;

#if TEST_PRINT_DATA
    printf("[TEST] BNO: Euler=(%.1f, %.1f, %.1f)° | Accel=(%.1f, %.1f, %.1f)mg | Cal=0x%X\r\n",
           bno_data->heading_deg, bno_data->roll_deg, bno_data->pitch_deg,
           bno_data->accel_x_mg, bno_data->accel_y_mg, bno_data->accel_z_mg,
           bno_data->calibration_status);
#endif

    return true;

#else
    return false;
#endif
}

bool test_inject_bno_data(void) {
    if (!test_initialized) {
        return false;
    }

    BNO_t bno_data;
    if (test_generate_bno_data(&bno_data, test_stats.bno_generated)) {
        data_handler_store_bno(&bno_data);
        return true;
    }
    return false;
}

/* ============================================================================
 * GPS DATA GENERATION
 * ============================================================================ */

bool test_generate_gps_data(GPS_t *gps_data, uint32_t sample_num) {
    if (!test_initialized || !gps_data) {
        return false;
    }

#if TEST_GPS_ENABLED

    // Initialize structure
    memset(gps_data, 0, sizeof(GPS_t));

    switch (TEST_GPS_MODE) {
        case TEST_MODE_RANDOM:
            gps_data->dec_latitude = test_random_float(GPS_LAT_MIN, GPS_LAT_MAX);
            gps_data->dec_longitude = test_random_float(GPS_LON_MIN, GPS_LON_MAX);
            gps_data->altitude_m = test_random_float(GPS_ALT_MIN, GPS_ALT_MAX);
            gps_data->speed_k = test_random_float(GPS_SPEED_MIN, GPS_SPEED_MAX);
            gps_data->course_d = test_random_float(GPS_COURSE_MIN, GPS_COURSE_MAX);
            gps_data->satelites = test_random_int(4, 12);
            gps_data->hdop = test_random_float(1.0f, 5.0f);
            gps_data->lock = 1;
            gps_data->rmc_status = 'A';
            break;

        case TEST_MODE_LINEAR:
            gps_data->dec_latitude = interpolate_linear(last_values.gps_lat, GPS_LAT_MIN, GPS_LAT_MAX, sample_num);
            gps_data->dec_longitude = interpolate_linear(last_values.gps_lon, GPS_LON_MIN, GPS_LON_MAX, sample_num + 1);
            gps_data->altitude_m = interpolate_linear(last_values.gps_alt, GPS_ALT_MIN, GPS_ALT_MAX, sample_num);
            gps_data->speed_k = interpolate_linear(last_values.gps_speed, GPS_SPEED_MIN, GPS_SPEED_MAX, sample_num);
            gps_data->course_d = interpolate_linear(last_values.gps_course, GPS_COURSE_MIN, GPS_COURSE_MAX, sample_num + 1);
            gps_data->satelites = 10;
            gps_data->hdop = 2.0f;
            gps_data->lock = 1;
            gps_data->rmc_status = 'A';

            last_values.gps_lat = gps_data->dec_latitude;
            last_values.gps_lon = gps_data->dec_longitude;
            last_values.gps_alt = gps_data->altitude_m;
            last_values.gps_speed = gps_data->speed_k;
            last_values.gps_course = gps_data->course_d;
            break;

        case TEST_MODE_STATIC:
            gps_data->dec_latitude = 40.7128;   // New York City
            gps_data->dec_longitude = -74.0060;
            gps_data->altitude_m = 10.0f;
            gps_data->speed_k = 0.0f;
            gps_data->course_d = 0.0f;
            gps_data->satelites = 12;
            gps_data->hdop = 1.0f;
            gps_data->lock = 1;
            gps_data->rmc_status = 'A';
            break;

        default:
            return false;
    }

    gps_data->nmea_latitude = (float)gps_data->dec_latitude;
    gps_data->nmea_longitude = (float)gps_data->dec_longitude;
    gps_data->ns = (gps_data->dec_latitude >= 0) ? 'N' : 'S';
    gps_data->ew = (gps_data->dec_longitude >= 0) ? 'E' : 'W';
    gps_data->msl_altitude = gps_data->altitude_m;
    gps_data->msl_units = 'M';
    gps_data->magnetic_dev = test_random_float(-15.0f, 15.0f);
    gps_data->magnetic_dev_unit = 'E';
    gps_data->utc_time = (float)((xTaskGetTickCount() / 1000) % 86400);  // Seconds in day
    gps_data->date = 051125;  // DD/MM/YY

    gps_data->timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    test_stats.gps_generated++;
    test_stats.total_samples++;

#if TEST_PRINT_DATA
    printf("[TEST] GPS: Lat=%.6f° | Lon=%.6f° | Alt=%.1f m | Sats=%d | Speed=%.2f k\r\n",
           gps_data->dec_latitude, gps_data->dec_longitude, gps_data->altitude_m,
           gps_data->satelites, gps_data->speed_k);
#endif

    return true;

#else
    return false;
#endif
}

bool test_inject_gps_data(void) {
    if (!test_initialized) {
        return false;
    }

    GPS_t gps_data;
    if (test_generate_gps_data(&gps_data, test_stats.gps_generated)) {
        data_handler_store_gps(&gps_data);
        return true;
    }
    return false;
}

/* ============================================================================
 * FRAMEWORK FUNCTIONS
 * ============================================================================ */

bool test_init(void) {
    if (test_initialized) {
        return true;
    }

    // Initialize random seed
    test_set_random_seed((uint32_t)time(NULL));

    // Initialize statistics
    memset(&test_stats, 0, sizeof(test_stats));
    memset(&last_values, 0, sizeof(last_values));

    test_initialized = true;

    printf("\r\n");
    printf("╔════════════════════════════════════════════════════════════╗\r\n");
    printf("║       SENSOR SIMULATION FRAMEWORK INITIALIZED             ║\r\n");
    printf("╠════════════════════════════════════════════════════════════╣\r\n");
    printf("║  IMU (ASM330LHHX):        %s (Mode: %d)            ║\r\n",
           TEST_IMU_ENABLED ? "ENABLED " : "DISABLED", TEST_IMU_MODE);
    printf("║  Magnetometer (MMC5983MA): %s (Mode: %d)            ║\r\n",
           TEST_MAGNETOMETER_ENABLED ? "ENABLED " : "DISABLED", TEST_MAGNETOMETER_MODE);
    printf("║  Barometer (MS5607):       %s (Mode: %d)            ║\r\n",
           TEST_BAROMETER_ENABLED ? "ENABLED " : "DISABLED", TEST_BAROMETER_MODE);
    printf("║  BNO055 (9-DOF):          %s (Mode: %d)            ║\r\n",
           TEST_BNO055_ENABLED ? "ENABLED " : "DISABLED", TEST_BNO055_MODE);
    printf("║  GPS (U-Blox):            %s (Mode: %d)            ║\r\n",
           TEST_GPS_ENABLED ? "ENABLED " : "DISABLED", TEST_GPS_MODE);
    printf("╠════════════════════════════════════════════════════════════╣\r\n");
    printf("║  Data Printing:  %s                                  ║\r\n",
           TEST_PRINT_DATA ? "ENABLED " : "DISABLED");
    printf("║  Statistics:     %s                                  ║\r\n",
           TEST_PRINT_STATISTICS ? "ENABLED " : "DISABLED");
    printf("╚════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");

    return true;
}

void test_deinit(void) {
    if (!test_initialized) {
        return;
    }

    test_print_statistics();
    test_initialized = false;

    printf("Sensor simulation framework deinitialized\r\n");
}

bool test_is_enabled(void) {
    return test_initialized;
}

bool test_generate_all_sensors(void) {
    if (!test_initialized) {
        return false;
    }

    bool any_success = false;

    // Generate data for all enabled sensors
    if (test_inject_imu_data()) any_success = true;
    if (test_inject_mag_data()) any_success = true;
    if (test_inject_baro_data()) any_success = true;
    if (test_inject_bno_data()) any_success = true;
    if (test_inject_gps_data()) any_success = true;

    return any_success;
}

void test_get_statistics(test_statistics_t *stats) {
    if (!stats) {
        return;
    }
    memcpy(stats, &test_stats, sizeof(test_statistics_t));
}

void test_reset_statistics(void) {
    memset(&test_stats, 0, sizeof(test_statistics_t));
}

void test_print_statistics(void) {
    if (!test_initialized) {
        return;
    }

#if TEST_PRINT_STATISTICS
    printf("\r\n");
    printf("╔════════════════════════════════════════════════════════════╗\r\n");
    printf("║           TEST FRAMEWORK STATISTICS SUMMARY                ║\r\n");
    printf("╠════════════════════════════════════════════════════════════╣\r\n");
    printf("║  Total Samples Generated:  %lu                             ║\r\n", test_stats.total_samples);
    printf("╠════════════════════════════════════════════════════════════╣\r\n");
    printf("║  IMU Samples:              %lu                             ║\r\n", test_stats.imu_generated);
    printf("║  Magnetometer Samples:     %lu                             ║\r\n", test_stats.mag_generated);
    printf("║  Barometer Samples:        %lu                             ║\r\n", test_stats.baro_generated);
    printf("║  BNO055 Samples:          %lu                              ║\r\n", test_stats.bno_generated);
    printf("║  GPS Samples:              %lu                             ║\r\n", test_stats.gps_generated);
    printf("╠════════════════════════════════════════════════════════════╣\r\n");
    printf("║  Generation Errors:        %lu                             ║\r\n", test_stats.generation_errors);
    printf("╚════════════════════════════════════════════════════════════╝\r\n");
    printf("\r\n");
#endif
}

#else  /* TEST_ENABLED */

/* Stub implementations when testing is disabled */

bool test_init(void) { return false; }
void test_deinit(void) {}
bool test_is_enabled(void) { return false; }
bool test_generate_imu_data(IMU_t *imu_data, uint32_t sample_num) { (void)imu_data; (void)sample_num; return false; }
bool test_inject_imu_data(void) { return false; }
bool test_generate_mag_data(MAG_t *mag_data, uint32_t sample_num) { (void)mag_data; (void)sample_num; return false; }
bool test_inject_mag_data(void) { return false; }
bool test_generate_baro_data(BARO_t *baro_data, uint32_t sample_num) { (void)baro_data; (void)sample_num; return false; }
bool test_inject_baro_data(void) { return false; }
bool test_generate_bno_data(BNO_t *bno_data, uint32_t sample_num) { (void)bno_data; (void)sample_num; return false; }
bool test_inject_bno_data(void) { return false; }
bool test_generate_gps_data(GPS_t *gps_data, uint32_t sample_num) { (void)gps_data; (void)sample_num; return false; }
bool test_inject_gps_data(void) { return false; }
bool test_generate_all_sensors(void) { return false; }
void test_get_statistics(test_statistics_t *stats) { if (stats) memset(stats, 0, sizeof(test_statistics_t)); }
void test_reset_statistics(void) {}
void test_print_statistics(void) {}
float test_random_float(float min, float max) { (void)min; (void)max; return 0.0f; }
int32_t test_random_int(int32_t min, int32_t max) { (void)min; (void)max; return 0; }
void test_set_random_seed(uint32_t seed) { (void)seed; }

#endif /* TEST_ENABLED */



