/*
 * telemetry_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "telemetry_thread.h"
#include "telemetry.h"
#include "Data Handler/flash_data_handler.h"
#include "Radio/radio_thread.h"
#include "cmsis_os.h"

#define FAST_TELEM_PERIOD_MS 100
#define SLOW_TELEM_PERIOD_MS 1000

extern osThreadId_t telemetry_thread_id;
extern fsm_ctx_t fsm_ctx;

static TickType_t last_fast_tick = 0;
static TickType_t last_slow_tick = 0;

static IMU_t latest_imu = {0};
static BARO_t latest_baro = {0};
static BNO_t latest_bno = {0};
static GPS_t latest_gps = {0};
static temperature_readings_t latest_temp = {0};

static bool have_imu = false;
static bool have_baro = false;
static bool have_bno = false;

static void send_fast_telemetry(void) {
    if (!have_baro) return;

    telemetry_fast_t fast_pkt;
    uint16_t len = telemetry_build_fast(&fast_pkt, &fsm_ctx,
                                        have_imu ? &latest_imu : NULL,
                                        &latest_baro,
                                        have_bno ? &latest_bno : NULL);

    radio_packet_t radio_pkt;
    radio_pkt.length = len;
    memcpy(radio_pkt.buffer, &fast_pkt, len);

    xQueueSend(queue_to_radio, &radio_pkt, 0);
}

static void send_slow_telemetry(void) {
    telemetry_slow_t slow_pkt;
    uint16_t len = telemetry_build_slow(&slow_pkt, &latest_temp, &latest_gps);

    radio_packet_t radio_pkt;
    radio_pkt.length = len;
    memcpy(radio_pkt.buffer, &slow_pkt, len);

    xQueueSend(queue_to_radio, &radio_pkt, 0);
}

void telemetry_thread_function() {
    data_packet_t packet;

    printf("[TELEM] Thread started\r\n");

    if (!radio_is_gs_online()) {
        printf("[TELEM] GS offline, suspending\r\n");
        vTaskSuspend(NULL);
        return;
    }

    last_fast_tick = xTaskGetTickCount();
    last_slow_tick = xTaskGetTickCount();

    while(1) {
        if (xQueueReceive(queue_to_telemetry, &packet, pdMS_TO_TICKS(10)) == pdTRUE) {
            data_packet_lock(&packet);

            switch(packet.type) {
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
                    break;

                default:
                    break;
            }

            data_packet_unlock(&packet);
        }

        TickType_t now = xTaskGetTickCount();

        if ((now - last_fast_tick) >= pdMS_TO_TICKS(FAST_TELEM_PERIOD_MS)) {
            send_fast_telemetry();
            last_fast_tick = now;
        }

        if ((now - last_slow_tick) >= pdMS_TO_TICKS(SLOW_TELEM_PERIOD_MS)) {
            send_slow_telemetry();
            last_slow_tick = now;
        }
    }
}
