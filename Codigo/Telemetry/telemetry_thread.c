/**
 * @file telemetry_thread.c
 * @brief Telemetry Data Aggregation Thread Implementation
 * @author Tomás Teixeira
 * @date 2025
 * @version 2.0
 *
 * @details
 * Implements sensor data aggregation for the Magalhães Flight Computer
 * telemetry system. This thread collects data from multiple sensor sources
 * and provides a unified interface for the Radio thread.
 *
 * ## Architecture
 * The telemetry thread acts as a data aggregation layer:
 * - Receives data packets from queue_to_telemetry
 * - Caches the most recent reading for each sensor type
 * - Provides cached data to Radio thread at 20 Hz
 *
 * ## Thread Timing
 * - Main loop: 10ms period (100 Hz)
 * - Radio update: 50ms period (20 Hz)
 * - Queue timeout: 0ms (non-blocking)
 *
 * @see telemetry_thread.h for interface documentation
 * @see radio_thread.c for packet transmission
 * @ingroup Telemetry
 */

#include "telemetry_thread.h"
#include "Radio/radio_thread.h"
#include "cmsis_os.h"

/** @name Cached Sensor Data
 *  @brief Latest readings from each sensor type
 *  @{
 */
static IMU_t latest_imu = {0};    /**< Latest IMU reading (accel, gyro) */
static BARO_t latest_baro = {0};  /**< Latest barometer reading (pressure, altitude) */
static BNO_t latest_bno = {0};    /**< Latest BNO055 reading (orientation) */
static GPS_t latest_gps = {0};    /**< Latest GPS reading (position, velocity) */
/** @} */

/** @name Data Availability Flags
 *  @brief Track which sensors have provided data
 *  @{
 */
static bool have_imu = false;     /**< IMU data received at least once */
static bool have_baro = false;    /**< Barometer data received at least once */
static bool have_bno = false;     /**< BNO055 data received at least once */
static bool have_gps = false;     /**< GPS data received at least once */
/** @} */

/**
 * @brief Telemetry thread main function
 * @see telemetry_thread.h for detailed documentation
 */
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
