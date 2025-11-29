/*
 * sd_card_thread.c
 *
 * High-performance SD card logging - F446ZE optimized
 * Updated for cleaned GPS_t struct
 *
 * Created on: Oct 10, 2025
 * Author: Tomas Teixeira
 */

#include "sd_card_thread.h"
#include "Data Handler/flash_data_handler.h"
#include "ff.h"
#include "fatfs_sd.h"
#include "print.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

static FATFS fs;
static FIL file;
static bool sd_initialized = false;
static bool sd_file_open = false;

#define SD_WRITE_BUFFER_SIZE    4096
static char write_buffer[SD_WRITE_BUFFER_SIZE];
static uint32_t buffer_pos = 0;

#define SD_FLUSH_TIMEOUT_MS     200

// Updated CSV header with cleaned GPS fields
#define CSV_HEADER "timestamp_ms,type,seq,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,pressure_mbar,altitude_m,mag_x,mag_y,mag_z,heading_deg,roll_deg,pitch_deg,latitude,longitude,gps_alt_m,lock,satellites,hdop,speed_kts,course_deg,quat_w,quat_x,quat_y,quat_z\r\n"

static void sd_card_init(void);
static void sd_card_configure(void);
static void flush_write_buffer(void);
static void log_data_packet(const data_packet_t *packet);

static void sd_card_init(void) {
    printf("Initializing SD card...\r\n");

    FRESULT res = f_mount(&fs, "", 0);
    if (res != FR_OK) {
        printf("ERROR: Failed to mount SD (error %d)\r\n", res);
        return;
    }

    printf("SD card mounted\r\n");
    sd_initialized = true;
}

static void sd_card_configure(void) {
    if (!sd_initialized) {
        printf("ERROR: SD not initialized\r\n");
        return;
    }

    printf("Configuring SD logging...\r\n");

    char filename[32];
    snprintf(filename, sizeof(filename), "log_0.csv");

    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("ERROR: Failed to open file (error %d)\r\n", res);
        return;
    }

    printf("Log file: %s\r\n", filename);
    sd_file_open = true;

    UINT bw;
    res = f_write(&file, CSV_HEADER, strlen(CSV_HEADER), &bw);
    if (res != FR_OK || bw != strlen(CSV_HEADER)) {
        printf("ERROR: Failed to write header\r\n");
        return;
    }

    buffer_pos = 0;
    printf("SD ready for logging\r\n");
}

static void flush_write_buffer(void) {
    if (buffer_pos == 0 || !sd_file_open) {
        return;
    }

    UINT bw;
    FRESULT res = f_write(&file, write_buffer, buffer_pos, &bw);

    if (res != FR_OK) {
        printf("ERROR: SD write failed (error %d)\r\n", res);
        buffer_pos = 0;
        return;
    }

    if (bw != buffer_pos) {
        printf("WARNING: Partial write (%u/%lu)\r\n", bw, buffer_pos);
    }

    res = f_sync(&file);
    if (res != FR_OK) {
        printf("ERROR: f_sync failed (error %d)\r\n", res);
    }

    buffer_pos = 0;
}

static void log_data_packet(const data_packet_t *packet) {
    if (!sd_file_open) {
        return;
    }

    int line_len = 0;
    char line_buffer[512];

    data_packet_lock(packet);

    switch (packet->type) {
        case DATA_TYPE_IMU: {
            IMU_t imu;
            data_packet_copy_imu(packet, &imu);
            // Columns 1-9: timestamp, type, seq, accel(3), gyro(3), temp
            // Columns 10-30: empty (21 commas)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,IMU,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,,,,,,,,,,,,,,,,,,,\r\n",
                packet->timestamp_ms, packet->sequence,
                imu.accel_x, imu.accel_y, imu.accel_z,
                imu.gyro_x, imu.gyro_y, imu.gyro_z, imu.temperature_c);
            break;
        }

        case DATA_TYPE_BARO: {
            BARO_t baro;
            data_packet_copy_baro(packet, &baro);
            // Columns 1-3: timestamp, type, seq
            // Columns 4-9: empty (6 commas)
            // Columns 10-12: temp, pressure, altitude
            // Columns 13-30: empty (18 commas)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,BARO,%lu,,,,,,,%.2f,%.2f,%.2f,,,,,,,,,,,,,,,,\r\n",
                packet->timestamp_ms, packet->sequence,
                baro.temperature_c, baro.pressure_mbar, baro.altitude_m);
            break;
        }

        case DATA_TYPE_MAG: {
            MAG_t mag;
            data_packet_copy_mag(packet, &mag);
            // Columns 1-3: timestamp, type, seq
            // Columns 4-12: empty (9 commas)
            // Columns 13-15: mag_x, mag_y, mag_z
            // Columns 16-30: empty (15 commas)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,MAG,%lu,,,,,,,,,,,%.3f,%.3f,%.3f,,,,,,,,,,,,,\r\n",
                packet->timestamp_ms, packet->sequence,
                mag.mag_x, mag.mag_y, mag.mag_z);
            break;
        }

        case DATA_TYPE_BNO: {
            BNO_t bno;
            data_packet_copy_bno(packet, &bno);
            // Columns 1-3: timestamp, type, seq
            // Columns 4-15: empty (12 commas)
            // Columns 16-18: heading, roll, pitch
            // Columns 19-26: empty (8 commas)
            // Columns 27-30: quat_w, quat_x, quat_y, quat_z
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,BNO,%lu,,,,,,,,,,,,,%.2f,%.2f,%.2f,,,,,,,,,%.4f,%.4f,%.4f,%.4f\r\n",
                packet->timestamp_ms, packet->sequence,
                bno.heading_deg, bno.roll_deg, bno.pitch_deg,
                bno.quat_w, bno.quat_x, bno.quat_y, bno.quat_z);
            break;
        }

        case DATA_TYPE_GPS: {
            GPS_t gps;
            data_packet_copy_gps(packet, &gps);

            // Only log valid GPS data when lock acquired
            if (gps.lock > 0 && gps.satellites > 0) {
                // Columns 1-3: timestamp, type, seq
                // Columns 4-18: empty (15 commas)
                // Columns 19-25: lat, lon, alt, lock, sats, hdop, speed, course
                // Columns 26-30: empty (5 commas)
                line_len = snprintf(line_buffer, sizeof(line_buffer),
                    "%lu,GPS,%lu,,,,,,,,,,,,,,,,,%.6f,%.6f,%.2f,%d,%d,%.2f,%.2f,%.1f,,,,,\r\n",
                    packet->timestamp_ms, packet->sequence,
                    gps.dec_latitude, gps.dec_longitude, gps.msl_altitude,
                    gps.lock, gps.satellites, gps.hdop, gps.speed_k, gps.course_d);
            } else {
                // No GPS lock - log minimal info
                line_len = snprintf(line_buffer, sizeof(line_buffer),
                    "%lu,GPS,%lu,,,,,,,,,,,,,,,,,0.0,0.0,0.0,0,0,99.9,0.0,0.0,,,,,\r\n",
                    packet->timestamp_ms, packet->sequence);
            }
            break;
        }

        case DATA_TYPE_EVENT:
            break;

        default:
            break;
    }

    data_packet_unlock(packet);

    if (line_len > 0 && line_len < (int)(sizeof(line_buffer) - 1)) {
        if (buffer_pos + line_len > (SD_WRITE_BUFFER_SIZE - 512)) {
            flush_write_buffer();
        }

        memcpy(&write_buffer[buffer_pos], line_buffer, line_len);
        buffer_pos += line_len;
    }
}

void sd_card_thread_function(void *argument) {
    printf("SD Logger thread started...\r\n");

    sd_card_init();
    sd_card_configure();

    TickType_t last_flush = xTaskGetTickCount();
    const TickType_t flush_timeout = pdMS_TO_TICKS(SD_FLUSH_TIMEOUT_MS);

    data_packet_t packet;
    uint32_t stats_packets = 0;
    uint32_t stats_queue_empty = 0;
    TickType_t last_stats = xTaskGetTickCount();

    while(1) {
        TickType_t now = xTaskGetTickCount();

        if (xQueueReceive(queue_to_sd, &packet, pdMS_TO_TICKS(50)) == pdTRUE) {
            log_data_packet(&packet);
            stats_packets++;

            if (buffer_pos > ((SD_WRITE_BUFFER_SIZE * 3) / 4)) {
                flush_write_buffer();
                last_flush = now;
            }
        } else {
            stats_queue_empty++;
        }

        if ((now - last_flush) >= flush_timeout) {
            if (buffer_pos > 0) {
                flush_write_buffer();
            }
            last_flush = now;
        }

        if ((now - last_stats) >= pdMS_TO_TICKS(10000)) {
            UBaseType_t queue_msgs = uxQueueMessagesWaiting(queue_to_sd);
            printf("SD: %lu pkts logged, buf:%lu/%u bytes, queue:%lu msgs\r\n",
                   stats_packets, buffer_pos, SD_WRITE_BUFFER_SIZE, queue_msgs);

            if (stats_queue_empty > 80) {
                printf("SD: queue empty %lu times\r\n", stats_queue_empty);
            }

            stats_packets = 0;
            stats_queue_empty = 0;
            last_stats = now;
        }
    }
}

void sd_card_close(void) {
    printf("Closing SD...\r\n");

    flush_write_buffer();

    if (sd_file_open) {
        f_sync(&file);
        f_close(&file);
        sd_file_open = false;
    }

    f_mount(NULL, "", 0);
    sd_initialized = false;

    printf("SD closed\r\n");
}
