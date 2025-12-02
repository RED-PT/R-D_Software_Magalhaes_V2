/*
 * estimator_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "estimator_thread.h"
#include "Flight Computer/flight_computer.h"
#include "Data Handler/flash_data_handler.h"
#include "cmsis_os.h"
#include <math.h>

// Thresholds for flight event detection
#define LIFTOFF_VELOCITY_THRESHOLD_MS   2.0f
#define APOGEE_VELOCITY_THRESHOLD_MS    1.0f
#define TOUCHDOWN_ALT_THRESHOLD_M       0.5f
#define LANDED_VELOCITY_THRESHOLD_MS    0.3f
#define LANDED_TIME_MS                  3000

// State tracking
static float max_altitude_m = 0.0f;
static bool liftoff_detected = false;
static bool apogee_detected = false;
static bool flare_triggered = false;
static bool touchdown_detected = false;
static uint32_t touchdown_time = 0;

// ============================================================================
// Flight Event Detection
// ============================================================================
static void check_flight_events(float altitude_m, float velocity_ms) {
    // Update FSM context with current nav data
    fsm_ctx.altitude_agl_m = altitude_m;
    fsm_ctx.velocity_vertical_ms = velocity_ms;

    // Track max altitude
    if (altitude_m > max_altitude_m) {
        max_altitude_m = altitude_m;
        fsm_ctx.max_altitude_reached_m = max_altitude_m;
    }

    // Only detect events during flight
    if (fsm_ctx.state != STATE_FLIGHT) {
        return;
    }

    // Liftoff detection
    if (!liftoff_detected && velocity_ms > LIFTOFF_VELOCITY_THRESHOLD_MS) {
        liftoff_detected = true;
        printf("[EST] LIFTOFF detected! vel=%.2f m/s\r\n", velocity_ms);
        fsm_send_event(FSM_EVT_LIFTOFF, &velocity_ms);
    }

    // Apogee detection (velocity crosses zero going negative)
    if (liftoff_detected && !apogee_detected) {
        if (velocity_ms < 0 && fabsf(velocity_ms) > APOGEE_VELOCITY_THRESHOLD_MS) {
            apogee_detected = true;
            printf("[EST] APOGEE detected! alt=%.2f m\r\n", altitude_m);
            fsm_send_event(FSM_EVT_APOGEE, &altitude_m);
        }
    }

    // Flare altitude detection
    if (apogee_detected && !flare_triggered) {
        if (altitude_m <= fsm_ctx.profile.flare_altitude_m) {
            flare_triggered = true;
            printf("[EST] FLARE altitude reached! alt=%.2f m\r\n", altitude_m);
            fsm_send_event(FSM_EVT_FLARE_ALT, &altitude_m);
        }
    }

    // Touchdown detection
    if (flare_triggered && !touchdown_detected) {
        if (altitude_m <= TOUCHDOWN_ALT_THRESHOLD_M) {
            touchdown_detected = true;
            touchdown_time = HAL_GetTick();
            printf("[EST] TOUCHDOWN detected! alt=%.2f m\r\n", altitude_m);
            fsm_send_event(FSM_EVT_TOUCHDOWN, &altitude_m);
        }
    }

    // Landed confirmation (stable on ground for 3 seconds)
    if (touchdown_detected) {
        if (fabsf(velocity_ms) < LANDED_VELOCITY_THRESHOLD_MS) {
            if ((HAL_GetTick() - touchdown_time) > LANDED_TIME_MS) {
                printf("[EST] LANDED confirmed!\r\n");
                fsm_send_event(FSM_EVT_LANDED, NULL);
            }
        } else {
            // Reset timer if still moving
            touchdown_time = HAL_GetTick();
        }
    }

    // Safety: Altitude limit exceeded
    if (altitude_m > fsm_ctx.profile.max_altitude_m) {
        printf("[EST] WARNING: Max altitude exceeded!\r\n");
        fsm_send_event(FSM_EVT_ALTITUDE_LIMIT, &altitude_m);
    }

    // Safety: Velocity limit exceeded
    if (fabsf(velocity_ms) > fsm_ctx.profile.max_velocity_ms) {
        printf("[EST] WARNING: Max velocity exceeded!\r\n");
        fsm_send_event(FSM_EVT_VELOCITY_LIMIT, &velocity_ms);
    }
}

// ============================================================================
// Reset state (call when entering flight state)
// ============================================================================
static void reset_flight_detection(void) {
    max_altitude_m = 0.0f;
    liftoff_detected = false;
    apogee_detected = false;
    flare_triggered = false;
    touchdown_detected = false;
    touchdown_time = 0;
}

// ============================================================================
// Main Thread
// ============================================================================
void estimator_thread_function() {

    printf("[EST] Estimator Thread started...\r\n");
    fsm_report_thread_started("ESTIMATOR");

    data_packet_t packet;
    float current_altitude_m = 0.0f;
    float current_velocity_ms = 0.0f;

    // Simple velocity estimation from altitude changes
    float prev_altitude_m = 0.0f;
    uint32_t prev_time_ms = 0;

    TickType_t last_stats = xTaskGetTickCount();

    while(1) {
        // Receive sensor data
        if (xQueueReceive(queue_to_estimator, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
            data_packet_lock(&packet);

            switch(packet.type) {
                case DATA_TYPE_BARO: {
                    BARO_t baro;
                    data_packet_copy_baro(&packet, &baro);
                    current_altitude_m = baro.altitude_m;

                    // Simple velocity estimation (derivative of altitude)
                    uint32_t now = HAL_GetTick();
                    if (prev_time_ms > 0) {
                        float dt_s = (now - prev_time_ms) / 1000.0f;
                        if (dt_s > 0.001f) {
                            current_velocity_ms = (current_altitude_m - prev_altitude_m) / dt_s;
                        }
                    }
                    prev_altitude_m = current_altitude_m;
                    prev_time_ms = now;
                    break;
                }

                case DATA_TYPE_IMU: {
                    // TODO: Use IMU for better state estimation
                    break;
                }

                default:
                    break;
            }

            data_packet_unlock(&packet);

            // Check for flight events
            check_flight_events(current_altitude_m, current_velocity_ms);
        }

        // Reset detection when entering flight
        static fsm_state_t prev_state = STATE_BOOT;
        if (fsm_ctx.state == STATE_FLIGHT && prev_state != STATE_FLIGHT) {
            reset_flight_detection();
        }
        prev_state = fsm_ctx.state;

        // Periodic stats
        TickType_t now = xTaskGetTickCount();
        if ((now - last_stats) >= pdMS_TO_TICKS(10000)) {
            printf("[EST] Alt=%.2f m, Vel=%.2f m/s, MaxAlt=%.2f m\r\n",
                   current_altitude_m, current_velocity_ms, max_altitude_m);
            last_stats = now;
        }
    }
}
