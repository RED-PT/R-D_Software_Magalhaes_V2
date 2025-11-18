/*
 * telemetry_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "telemetry_thread.h"
#include "cmsis_os.h"
#include "Flight Computer/flight_computer.h"

void telemetry_thread_function() {
    data_packet_t packet;
    telemetry_fast_t fast_packet;
    telemetry_slow_t slow_packet;
    telemetry_event_t event_packet;
    radio_tx_packet_t radio_packet;

    // Temporary storage for sensor data
    BARO_t baro_data;
    BNO_t bno_data;
    GPS_t gps_data;

    uint32_t last_fast_tx = 0;  // 50Hz = 20ms
    uint32_t last_slow_tx = 0;  // 2Hz = 500ms

    printf("Telemetry Thread started...\r\n");

    while(1) {
        uint32_t now = HAL_GetTick();

        // PRIORITY 1: Check for FSM events (highest priority)
        if (xQueueReceive(queue_fsm_events, &event_packet, 0) == pdTRUE) {
            // Package event packet for radio
            radio_packet.length = sizeof(telemetry_event_t);
            memcpy(radio_packet.data, &event_packet, radio_packet.length);
            radio_packet.priority = 0;  // Highest priority

            // Send to radio thread (block up to 50ms if queue full)
            if (xQueueSend(queue_to_radio_tx, &radio_packet, pdMS_TO_TICKS(50)) != pdTRUE) {
                printf("WARN: Event packet dropped!\r\n");
            }
        }

        // PRIORITY 2: Fast packet (50Hz = every 20ms)
        if ((now - last_fast_tx) >= 20) {
            // Initialize packet with zeros
            memset(&fast_packet, 0, sizeof(telemetry_fast_t));

            fast_packet.time = now;
            fast_packet.state = get_fsm_state();
            fast_packet.substate = get_fsm_substate();

            // Get latest sensor data from queue
            if (xQueueReceive(queue_to_telemetry, &packet, 0) == pdTRUE) {
                // Extract data based on packet type
                switch(packet.type) {
                    case DATA_TYPE_BARO:
                        data_packet_copy_baro(&packet, &baro_data);
                        fast_packet.altitude = baro_data.altitude_m;
                        // TODO: velocity/acceleration from estimator
                        break;

                    case DATA_TYPE_BNO:
                        data_packet_copy_bno(&packet, &bno_data);
                        fast_packet.euler_angles[0] = bno_data.roll_deg;
                        fast_packet.euler_angles[1] = bno_data.pitch_deg;
                        fast_packet.euler_angles[2] = bno_data.heading_deg;
                        fast_packet.gyro[0] = bno_data.gyro_x_dps;
                        fast_packet.gyro[1] = bno_data.gyro_y_dps;
                        fast_packet.gyro[2] = bno_data.gyro_z_dps;
                        break;

                    default:
                        // Ignore other packet types for fast telemetry
                        break;
                }

                data_packet_unlock(&packet);
            }

            // TODO: Extract servo/throttle from controller
            fast_packet.throttle = 0.0f;
            fast_packet.servo_deg[0] = 0.0f;
            fast_packet.servo_deg[1] = 0.0f;

            // Calculate CRC (exclude the CRC field itself)
            fast_packet.crc16 = calculate_crc16((uint8_t*)&fast_packet,
                                                sizeof(telemetry_fast_t) - sizeof(uint16_t));

            // Package for radio
            radio_packet.length = sizeof(telemetry_fast_t);
            memcpy(radio_packet.data, &fast_packet, radio_packet.length);
            radio_packet.priority = 1;

            // Send to radio (don't block - drop if full)
            xQueueSend(queue_to_radio_tx, &radio_packet, 0);

            last_fast_tx = now;
        }

        // PRIORITY 3: Slow packet (2Hz = every 500ms)
        if ((now - last_slow_tx) >= 500) {
            memset(&slow_packet, 0, sizeof(telemetry_slow_t));
            slow_packet.time = now;

            // Get latest GPS data if available
            if (xQueuePeek(queue_to_telemetry, &packet, 0) == pdTRUE) {
                if (packet.type == DATA_TYPE_GPS) {
                    data_packet_copy_gps(&packet, &gps_data);
                    slow_packet.latitude = (float)gps_data.dec_latitude;
                    slow_packet.longitude = (float)gps_data.dec_longitude;
                }
            }

            // TODO: Fill in temperature and voltage from ADC
            slow_packet.temp_ms = 0.0f;
            slow_packet.temp_cpu = 0.0f;
            slow_packet.vbat = 0;

            slow_packet.crc16 = calculate_crc16((uint8_t*)&slow_packet,
                                                sizeof(telemetry_slow_t) - sizeof(uint16_t));

            // Package for radio
            radio_packet.length = sizeof(telemetry_slow_t);
            memcpy(radio_packet.data, &slow_packet, radio_packet.length);
            radio_packet.priority = 2;

            // Send to radio (don't block)
            xQueueSend(queue_to_radio_tx, &radio_packet, 0);

            last_slow_tx = now;
        }

        // Run at ~200Hz (5ms period)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
