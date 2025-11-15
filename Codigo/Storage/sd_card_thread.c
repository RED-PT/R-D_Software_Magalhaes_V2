/*
 * sd_card_thread.c
 *
 * High-performance SD card logging thread using FatFS
 * Uses existing SD driver from fatfs_sd.c
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

// FatFS objects
static FATFS fs;
static FIL file;
static bool sd_initialized = false;
static bool sd_file_open = false;

// Write buffer for batch operations (4KB - typical SD page size)
#define SD_WRITE_BUFFER_SIZE    4096
static char write_buffer[SD_WRITE_BUFFER_SIZE];
static uint32_t buffer_pos = 0;

// Timeout for periodic flushes
#define SD_FLUSH_TIMEOUT_MS     500

// CSV Header format
// CORRIGIDO: Adicionados os campos Quat e Aceleração Linear (lia). Total: 30 Colunas.
#define CSV_HEADER "timestamp_ms,type,seq,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,pressure_mbar,altitude_m,mag_x,mag_y,mag_z,heading_deg,roll_deg,pitch_deg,latitude,longitude,gps_alt,lock,satellites,quat_w,quat_x,quat_y,quat_z,lia_x,lia_y,lia_z\r\n"

// Forward declarations
static void sd_card_init(void);
static void sd_card_configure(void);
static void flush_write_buffer(void);
static void log_data_packet(const data_packet_t *packet);

// Initialize SD card using existing driver
static void sd_card_init(void) {
    printf("Initializing SD card hardware...\r\n");

    // The SD_disk_initialize from fatfs_sd.c handles all hardware init
    // It will be called automatically by FatFS on first mount

    printf("Mounting FatFS...\r\n");
    FRESULT res = f_mount(&fs, "", 0);

    if (res != FR_OK) {
        printf("ERROR: Failed to mount SD card (FatFS error %d)\r\n", res);
        return;
    }

    printf("SD card mounted successfully\r\n");
    sd_initialized = true;
}

// Configure SD card and create log file
static void sd_card_configure(void) {
    if (!sd_initialized) {
        printf("ERROR: SD card not initialized, skipping configure\r\n");
        return;
    }

    printf("Configuring SD card logging...\r\n");

    // Create/open log file with a fixed name
    char filename[32];
    // Usa um nome de ficheiro fixo, para apagar o antigo
    snprintf(filename, sizeof(filename), "log_0.csv");

    printf("Attempting to open file: %s\r\n", filename);
    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) {
        printf("ERROR: Failed to open log file (FatFS error %d)\r\n", res);
        return;
    }

    printf("Log file created: %s\r\n", filename);
    sd_file_open = true;

    // Write CSV header
    UINT bw;
    res = f_write(&file, CSV_HEADER, strlen(CSV_HEADER), &bw);
    if (res != FR_OK || bw != strlen(CSV_HEADER)) {
        printf("ERROR: Failed to write CSV header (wrote %u/%u bytes)\r\n", bw, strlen(CSV_HEADER));
        return;
    }

    buffer_pos = 0;
    printf("SD card configured and ready for logging\r\n");
}

// Flush write buffer to SD card
static void flush_write_buffer(void) {
    if (buffer_pos == 0 || !sd_file_open) {
        return; // Nothing to write or file not open
    }

    UINT bw;
    FRESULT res = f_write(&file, write_buffer, buffer_pos, &bw);

    if (res != FR_OK) {
        printf("ERROR: SD write failed (FatFS error %d)\r\n", res);
        return;
    }

    if (bw != buffer_pos) {
        printf("WARNING: Partial SD write (%u/%lu bytes)\r\n", bw, buffer_pos);
    }

    // Sync file to disk - CRITICAL for data to actually be written
    res = f_sync(&file);
    if (res != FR_OK) {
        printf("ERROR: f_sync failed (error %d)\r\n", res);
    }

    buffer_pos = 0;
}

// Format and add packet data to write buffer
static void log_data_packet(const data_packet_t *packet) {
    if (!sd_file_open) {
        return;
    }

    int line_len = 0;
    char line_buffer[512];

    // Must lock the packet to safely read the data pointer
    data_packet_lock(packet);

    // Format based on data type
    switch (packet->type) {
        case DATA_TYPE_IMU: {
            IMU_t imu;
            data_packet_copy_imu(packet, &imu);
            // Colunas: 1-10
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,IMU,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f\r\n",
                packet->timestamp_ms, packet->sequence,
                imu.accel_x, imu.accel_y, imu.accel_z,
                imu.gyro_x, imu.gyro_y, imu.gyro_z, imu.temperature_c);
            break;
        }

        case DATA_TYPE_BARO: {
            BARO_t baro;
            data_packet_copy_baro(packet, &baro);
            // Colunas: 1-3, 10-12 (6 vírgulas para saltar Colunas 4-9)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,BARO,%lu,,,,,,%.2f,%.2f,%.2f\r\n",
                packet->timestamp_ms, packet->sequence,
                baro.temperature_c, baro.pressure_mbar, baro.altitude_m);
            break;
        }

        case DATA_TYPE_MAG: {
            MAG_t mag;
            data_packet_copy_mag(packet, &mag);
            // Colunas: 1-3, 13-15 (9 vírgulas para saltar Colunas 4-12)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,MAG,%lu,,,,,,,,,%.3f,%.3f,%.3f\r\n",
                packet->timestamp_ms, packet->sequence,
                mag.mag_x, mag.mag_y, mag.mag_z);
            break;
        }

        case DATA_TYPE_BNO: {
            BNO_t bno;
            data_packet_copy_bno(packet, &bno);
            // Colunas: 1-3, 16-18 (Euler), 24-30 (Quat e LIA)
            // 12 vírgulas (4-15), 5 vírgulas (19-23)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,BNO,%lu,,,,,,,,,,,,%.2f,%.2f,%.2f,,,,,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\r\n",
                packet->timestamp_ms, packet->sequence,
                bno.heading_deg, bno.roll_deg, bno.pitch_deg,
                // 5 vírgulas aqui saltam os 5 campos GPS (Cols 19-23)
                bno.quat_w, bno.quat_x, bno.quat_y, bno.quat_z,
                // Linear Acceleration (LIA) - convertendo mg para m/s^2
                bno.accel_x_mg / 1000.0f, bno.accel_y_mg / 1000.0f, bno.accel_z_mg / 1000.0f);
            break;
        }

        case DATA_TYPE_GPS: {
            GPS_t gps;
            data_packet_copy_gps(packet, &gps);
            // Colunas: 1-3, 19-23 (15 vírgulas para saltar Colunas 4-18)
            line_len = snprintf(line_buffer, sizeof(line_buffer),
                "%lu,GPS,%lu,,,,,,,,,,,,,,,%.6f,%.6f,%.2f,%d,%d\r\n",
                packet->timestamp_ms, packet->sequence,
                gps.dec_latitude, gps.dec_longitude, gps.altitude_m,
                gps.lock, gps.satelites);
            break;
        }

        case DATA_TYPE_EVENT: {
            // Events don't go to CSV log
            break;
        }

        default:
            break;
    }

    data_packet_unlock(packet);

    // Add line to buffer if it fits
    if (line_len > 0 && line_len < (int)(sizeof(line_buffer) - 1)) {
        // Check if line fits in buffer, if not flush first
        if (buffer_pos + line_len > SD_WRITE_BUFFER_SIZE) {
            flush_write_buffer();
        }

        // Copy line to buffer
        memcpy(&write_buffer[buffer_pos], line_buffer, line_len);
        buffer_pos += line_len;
    }
}

// Thread function
void sd_card_thread_function(void *argument) {
    printf("SD Card Logger thread started\r\n");

    // Initialize SD card
    sd_card_init();

    // Configure logging
    sd_card_configure();

    TickType_t last_flush = xTaskGetTickCount();
    const TickType_t flush_timeout = pdMS_TO_TICKS(SD_FLUSH_TIMEOUT_MS);

    data_packet_t packet;
    uint32_t stats_packets_logged = 0;
    TickType_t last_stats = xTaskGetTickCount();

    while(1) {
        TickType_t now = xTaskGetTickCount();

        // Try to receive packet from logger queue (non-blocking with timeout)
        if (xQueueReceive(queue_to_sd, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Successfully received packet
            log_data_packet(&packet);
            stats_packets_logged++;

            // Flush buffer if it's getting full (leave 512 bytes margin for large lines)
            if (buffer_pos > (SD_WRITE_BUFFER_SIZE - 512)) {
                flush_write_buffer();
                last_flush = now;
            }
        }

        // Periodic flush even if no data (prevents data loss on sudden power loss)
        if ((now - last_flush) >= flush_timeout) {
            if (buffer_pos > 0) {
                flush_write_buffer();
            }
            last_flush = now;
        }

        // Print statistics every 5 seconds
        if ((now - last_stats) >= pdMS_TO_TICKS(5000)) {
            printf("Logger: %lu packets logged, buffer pos: %lu/%u\r\n",
                   stats_packets_logged, buffer_pos, SD_WRITE_BUFFER_SIZE);
            stats_packets_logged = 0;
            last_stats = now;
        }
    }
}

// Cleanup function (call on shutdown)
void sd_card_close(void) {
    printf("Closing SD logger...\r\n");

    // Final flush
    flush_write_buffer();

    // Sync and close file
    if (sd_file_open) {
        f_sync(&file);
        f_close(&file);
        sd_file_open = false;
    }

    // Unmount
    f_mount(NULL, "", 0);
    sd_initialized = false;

    printf("SD logger closed\r\n");
}
