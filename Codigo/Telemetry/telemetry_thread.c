/*
 * telemetry_thread.c
 *
 * Collects sensor data and provides it to radio_thread
 * No longer handles radio directly - just data aggregation
 */

#include "telemetry_thread.h"
#include "Radio/radio_thread.h"
#include "cmsis_os.h"

// Latest sensor data
static IMU_t latest_imu = {0};
static BARO_t latest_baro = {0};
static BNO_t latest_bno = {0};
static GPS_t latest_gps = {0};

static bool have_imu = false;
static bool have_baro = false;
static bool have_bno = false;
static bool have_gps = false;

void telemetry_thread_function() {
	printf("[TELEM] Thread started\r\n");
	fsm_report_thread_started("TELEMETRY");
	data_packet_t packet;

    TickType_t last_update = xTaskGetTickCount();
    const TickType_t update_period = pdMS_TO_TICKS(50);  // 20Hz update to radio

    while (1) {
        // Receive sensor data from queues
        while (xQueueReceive(queue_to_telemetry, &packet, 0) == pdTRUE) {
            data_packet_lock(&packet);

            switch (packet.type) {
                case DATA_TYPE_IMU:
                    data_packet_copy_imu(&packet, &latest_imu);
                    have_imu = true;
                    break;
                case DATA_TYPE_BARO:
                    data_packet_copy_baro(&packet, &latest_baro);
                    have_baro = true;
                    break;
                case DATA_TYPE_BNO:
                    data_packet_copy_bno(&packet, &latest_bno);
                    have_bno = true;
                    break;
                case DATA_TYPE_GPS:
                    data_packet_copy_gps(&packet, &latest_gps);
                    have_gps = true;
                    break;
                default:
                    break;
            }

            data_packet_unlock(&packet);
        }

        // Periodically update radio thread with latest data
        TickType_t now = xTaskGetTickCount();
        if ((now - last_update) >= update_period) {
            radio_update_sensor_data(
                have_imu ? &latest_imu : NULL,
                have_baro ? &latest_baro : NULL,
                have_bno ? &latest_bno : NULL,
                have_gps ? &latest_gps : NULL
            );
            last_update = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
