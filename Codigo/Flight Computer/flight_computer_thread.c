/*
 * flight_computer_thread.c
 *
 * FSM Thread - BOOT and IDLE state handling
 */

#include "flight_computer_thread.h"
#include "Telemetry/telemetry.h"
#include "Data Handler/flash_data_handler.h"
#include "Threads/create_threads.h"
#include "cmsis_os.h"
#include "print.h"
#include <string.h>

#define BOOT_TIMEOUT_MS     10000   // 10 seconds max boot time
#define BOOT_MIN_WAIT_MS    3000    // Minimum 3s to let threads start
#define STATS_INTERVAL_MS   10000

// Parameter IDs (must match GS)
#define PARAM_TARGET_ALTITUDE   0
#define PARAM_FLARE_ALTITUDE    1
#define PARAM_TOUCHDOWN_VEL     2
#define PARAM_HOLD_THROTTLE     3
#define PARAM_RAMP_DURATION     4
#define PARAM_MAX_ALTITUDE      5
#define PARAM_THROTTLE_MIN      6
#define PARAM_THROTTLE_MAX      7

// Event sequence
static uint8_t event_seq = 0;

// Boot report storage
static boot_report_t boot_report = {0};
static uint8_t boot_error_idx = 0;

// ============================================================================
// Helper: Add error to boot report
// ============================================================================
static void add_boot_error(const char *error) {
    if (boot_error_idx < BOOT_REPORT_MAX_ERRORS) {
        strncpy(boot_report.errors[boot_error_idx], error, BOOT_ERROR_MSG_LEN - 1);
        boot_report.errors[boot_error_idx][BOOT_ERROR_MSG_LEN - 1] = '\0';
        boot_error_idx++;
        boot_report.error_count = boot_error_idx;
    }
}

// ============================================================================
// Helper: Build boot report from status
// ============================================================================
static void build_boot_report(void) {
    boot_status_t *bs = &fsm_ctx.boot_status;

    boot_report.total_checks = bs->total_checks;
    boot_report.passed = bs->passed_checks;
    boot_report.failed = bs->failed_checks;
    boot_report.critical = bs->critical_failures;
    boot_report.boot_time_ms = HAL_GetTick() - fsm_ctx.boot_start_tick;

    // Add specific errors
    boot_error_idx = 0;

    if (bs->imu_init != 1) add_boot_error("IMU_INIT_FAIL");
    if (bs->imu_config != 1 && bs->imu_init == 1) add_boot_error("IMU_CFG_FAIL");
    if (bs->baro_init != 1) add_boot_error("BARO_INIT_FAIL");
    if (bs->mag_init != 1) add_boot_error("MAG_INIT_FAIL");
    if (bs->bno_init != 1) add_boot_error("BNO_INIT_FAIL");
    if (bs->gps_init != 1) add_boot_error("GPS_INIT_FAIL");
    if (bs->sd_card_init != 1) add_boot_error("SD_INIT_FAIL");
    if (bs->radio_init != 1) add_boot_error("RADIO_FAIL");

    if (bs->sensors_thread_started != 1) add_boot_error("SENS_THR_FAIL");
    if (bs->data_handler_thread_started != 1) add_boot_error("DH_THR_FAIL");
    if (bs->telemetry_thread_started != 1) add_boot_error("TEL_THR_FAIL");
    if (bs->radio_thread_started != 1) add_boot_error("RAD_THR_FAIL");
}

// ============================================================================
// Telemetry Event Helper
// ============================================================================
static void send_telem_event(telemetry_event_type_t type, const void *payload, uint16_t size) {
    telemetry_event_t evt;
    memset(&evt, 0, sizeof(evt));

    evt.packet_type = TELEM_PACKET_EVENT;
    evt.time = HAL_GetTick();
    evt.event_type = (uint8_t)type;
    evt.state = (uint8_t)fsm_ctx.state;
    evt.substate = (uint8_t)fsm_ctx.substate;

    if (payload && size > 0) {
        uint16_t copy_size = (size > sizeof(evt.payload)) ? sizeof(evt.payload) : size;
        memcpy(&evt.payload, payload, copy_size);
    }

    evt.crc16 = crc16_calculate((uint8_t*)&evt, sizeof(telemetry_event_t) - 2);

    data_handler_store_event(&evt);
}

// ============================================================================
// State Transition
// ============================================================================
static void transition_to(fsm_state_t new_state, fsm_substate_t new_sub, telemetry_event_type_t reason) {
    fsm_state_t old_state = fsm_ctx.state;
    fsm_substate_t old_sub = fsm_ctx.substate;

    if (old_state == new_state && old_sub == new_sub) return;

    fsm_ctx.prev_state = old_state;
    fsm_ctx.prev_substate = old_sub;
    fsm_ctx.state = new_state;
    fsm_ctx.substate = new_sub;
    fsm_ctx.state_entry_tick = HAL_GetTick();

    printf("[FSM] %s.%s -> %s.%s (reason=%u)\r\n",
           fsm_state_to_str(old_state), fsm_substate_to_str(old_sub),
           fsm_state_to_str(new_state), fsm_substate_to_str(new_sub),
           (uint8_t)reason);

    // Send state change event
    event_state_change_t change = {
        .old_state = (uint8_t)old_state,
        .old_substate = (uint8_t)old_sub,
        .new_state = (uint8_t)new_state,
        .new_substate = (uint8_t)new_sub,
        .reason = (uint8_t)reason
    };
    send_telem_event(EVT_STATE_CHANGE, &change, sizeof(change));

    // State-specific actions on entry
    switch (new_state) {
        case STATE_IDLE:
            // Suspend estimator and controller
            if (estimator_thread_id != NULL) {
                vTaskSuspend(estimator_thread_id);
                printf("[FSM] Estimator suspended\r\n");
            }
            if (controller_thread_id != NULL) {
                vTaskSuspend(controller_thread_id);
                printf("[FSM] Controller suspended\r\n");
            }
            break;

        case STATE_ARMED:
        case STATE_TEST_STAND:
        case STATE_FLIGHT:
            // Resume estimator and controller
            if (estimator_thread_id != NULL) {
                vTaskResume(estimator_thread_id);
                fsm_ctx.flags.estimator_running = 1;
                printf("[FSM] Estimator resumed\r\n");
            }
            if (controller_thread_id != NULL) {
                vTaskResume(controller_thread_id);
                fsm_ctx.flags.controller_running = 1;
                printf("[FSM] Controller resumed\r\n");
            }
            // Enable SD logging
            fsm_ctx.flags.sd_logging_enabled = 1;
            break;

        case STATE_SAFE:
        case STATE_ABORT:
            // Disable controller, keep logging
            if (controller_thread_id != NULL) {
                vTaskSuspend(controller_thread_id);
                fsm_ctx.flags.controller_running = 0;
            }
            break;

        default:
            break;
    }
}

// ============================================================================
// BOOT State Handler
// ============================================================================
static void handle_state_boot(void) {
    uint32_t elapsed = HAL_GetTick() - fsm_ctx.boot_start_tick;
    boot_status_t *bs = &fsm_ctx.boot_status;

    // Wait minimum time for threads to start
    if (elapsed < BOOT_MIN_WAIT_MS) {
        return;
    }

    // Check if boot complete
    bool boot_ok = fsm_is_boot_complete();

    // Check for timeout
    if (elapsed >= BOOT_TIMEOUT_MS) {
        printf("[FSM] BOOT TIMEOUT after %lu ms\r\n", elapsed);

        // Build and send boot report
        build_boot_report();

        printf("[FSM] Boot Report: %u/%u passed, %u critical failures\r\n",
               boot_report.passed, boot_report.total_checks, boot_report.critical);

        for (int i = 0; i < boot_report.error_count; i++) {
            printf("[FSM]   - %s\r\n", boot_report.errors[i]);
        }

        // Send CHECKS_RED event
        send_telem_event(EVT_CHECKS_RED, &boot_report, sizeof(boot_report));

        // If critical failures, go to SAFE
        if (bs->critical_failures > 0) {
            printf("[FSM] CRITICAL FAILURES - going to SAFE\r\n");
            transition_to(STATE_SAFE, SUB_NONE, EVT_CHECKS_RED);
        } else {
            // Non-critical failures, still go to IDLE but warn
            printf("[FSM] Non-critical failures, proceeding to IDLE\r\n");
            transition_to(STATE_IDLE, SUB_NONE, EVT_CHECKS_RED);
        }
        return;
    }

    // Boot complete successfully?
    if (boot_ok) {
        printf("[FSM] Boot complete in %lu ms\r\n", elapsed);

        // Build boot report
        build_boot_report();

        printf("[FSM] Boot Report: %u/%u passed\r\n",
               boot_report.passed, boot_report.total_checks);

        // Send CHECKS_GREEN event
        send_telem_event(EVT_CHECKS_GREEN, &boot_report, sizeof(boot_report));

        // Transition to IDLE
        transition_to(STATE_IDLE, SUB_NONE, EVT_CHECKS_GREEN);
    }
}

// ============================================================================
// IDLE State Handler
// ============================================================================
static void handle_state_idle(void) {
    // In IDLE:
    // - Sensors running, sending to telemetry (NOT estimator, NOT SD)
    // - Waiting for profile configuration

    // Nothing special to do here, just wait for commands
    // The state queries (fsm_should_queue_to_*) handle data routing
}

// ============================================================================
// Command Handlers
// ============================================================================
static void handle_cmd_ping(fsm_cmd_msg_t *msg) {
    printf("[FSM] PING received\r\n");
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_set_profile(fsm_cmd_msg_t *msg) {
    uint8_t profile_type = msg->payload.raw[0];

    printf("[FSM] SET_PROFILE: type=%u (%s)\r\n",
           profile_type, fsm_profile_to_str(profile_type));

    if (profile_type < 1 || profile_type > 3) {
        printf("[FSM] Invalid profile type!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    fsm_ctx.profile.type = (flight_profile_t)profile_type;

    // Send profile loaded event
    send_telem_event(EVT_PROFILE_LOADED, &profile_type, 1);

    // Transition IDLE -> CONFIGED
    if (fsm_ctx.state == STATE_IDLE) {
        transition_to(STATE_CONFIGED, SUB_NONE, EVT_PROFILE_LOADED);
    }

    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_set_param(fsm_cmd_msg_t *msg) {
    uint8_t param_id = msg->payload.raw[0];
    float value = 0.0f;
    memcpy(&value, &msg->payload.raw[1], sizeof(float));

    const char* param_names[] = {
        "TARGET_ALT", "FLARE_ALT", "TOUCHDOWN_VEL", "HOLD_THROTTLE",
        "RAMP_DUR", "MAX_ALT", "THROTTLE_MIN", "THROTTLE_MAX"
    };

    if (param_id > 7) {
        printf("[FSM] Invalid param ID %u\r\n", param_id);
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    printf("[FSM] SET_PARAM: %s = %.2f\r\n", param_names[param_id], value);

    switch (param_id) {
        case PARAM_TARGET_ALTITUDE:
            fsm_ctx.profile.target_altitude_m = value;
            break;
        case PARAM_FLARE_ALTITUDE:
            fsm_ctx.profile.flare_altitude_m = value;
            break;
        case PARAM_TOUCHDOWN_VEL:
            fsm_ctx.profile.touchdown_velocity_ms = value;
            break;
        case PARAM_HOLD_THROTTLE:
            fsm_ctx.profile.hold_throttle = value;
            break;
        case PARAM_RAMP_DURATION:
            fsm_ctx.profile.ramp_duration_s = value;
            break;
        case PARAM_MAX_ALTITUDE:
            fsm_ctx.profile.max_altitude_m = value;
            break;
        case PARAM_THROTTLE_MIN:
            fsm_ctx.profile.throttle_min = value;
            break;
        case PARAM_THROTTLE_MAX:
            fsm_ctx.profile.throttle_max = value;
            break;
    }

    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_arm(fsm_cmd_msg_t *msg) {
    printf("[FSM] ARM requested\r\n");

    if (fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] Cannot ARM from %s\r\n", fsm_state_to_str(fsm_ctx.state));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    if (fsm_ctx.profile.type == PROFILE_NONE) {
        printf("[FSM] No profile set!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // Check critical sensors
    if (fsm_ctx.boot_status.imu_init != 1 || fsm_ctx.boot_status.baro_init != 1) {
        printf("[FSM] Critical sensors not ready!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    transition_to(STATE_ARMED, SUB_NONE, EVT_ARMED);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_disarm(fsm_cmd_msg_t *msg) {
    printf("[FSM] DISARM requested\r\n");

    if (fsm_ctx.state == STATE_ARMED) {
        transition_to(STATE_CONFIGED, SUB_NONE, EVT_DISARMED);
        fsm_ctx.last_cmd_status = 0;
    } else {
        printf("[FSM] Not armed\r\n");
        fsm_ctx.last_cmd_status = 1;
    }
}

static void handle_cmd_launch(fsm_cmd_msg_t *msg) {
    printf("[FSM] LAUNCH requested\r\n");

    if (fsm_ctx.state != STATE_ARMED) {
        printf("[FSM] Not armed!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    fsm_ctx.flight_start_tick = HAL_GetTick();
    transition_to(STATE_FLIGHT, SUB_FL_IGNITION, EVT_STATE_CHANGE);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_abort(fsm_cmd_msg_t *msg) {
    printf("[FSM] ABORT!\r\n");
    transition_to(STATE_ABORT, SUB_NONE, EVT_ABORT_TRIGGERED);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_force_safe(fsm_cmd_msg_t *msg) {
    printf("[FSM] FORCE_SAFE\r\n");
    transition_to(STATE_SAFE, SUB_NONE, EVT_STATE_CHANGE);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_calibrate_baro(fsm_cmd_msg_t *msg) {
    printf("[FSM] CALIBRATE_BARO\r\n");
    fsm_ctx.flags.baro_calibrated = 1;
    // TODO: Actually calibrate barometer (store reference pressure)
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_start_test(fsm_cmd_msg_t *msg) {
    printf("[FSM] START_TEST\r\n");

    if (fsm_ctx.state != STATE_ARMED) {
        printf("[FSM] Not armed!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    transition_to(STATE_TEST_STAND, SUB_TS_SENSOR_CHECK, EVT_STATE_CHANGE);
    fsm_ctx.last_cmd_status = 0;
}

// ============================================================================
// Command Dispatcher
// ============================================================================
static void process_command(fsm_cmd_msg_t *msg) {
    printf("[FSM] CMD=%u seq=%u\r\n", msg->cmd, msg->cmd_seq);

    fsm_ctx.last_cmd_seq = msg->cmd_seq;

    switch (msg->cmd) {
        case CMD_PING:           handle_cmd_ping(msg); break;
        case CMD_SET_PROFILE:    handle_cmd_set_profile(msg); break;
        case CMD_SET_PARAM:      handle_cmd_set_param(msg); break;
        case CMD_ARM:            handle_cmd_arm(msg); break;
        case CMD_DISARM:         handle_cmd_disarm(msg); break;
        case CMD_LAUNCH:         handle_cmd_launch(msg); break;
        case CMD_ABORT:          handle_cmd_abort(msg); break;
        case CMD_FORCE_SAFE:     handle_cmd_force_safe(msg); break;
        case CMD_CALIBRATE_BARO: handle_cmd_calibrate_baro(msg); break;
        case CMD_START_TEST:     handle_cmd_start_test(msg); break;
        default:
            printf("[FSM] Unknown CMD %u\r\n", msg->cmd);
            fsm_ctx.last_cmd_status = 2;
            break;
    }
}

// ============================================================================
// Other State Handlers (stubs for now)
// ============================================================================
static void handle_state_configed(void) {
    // Waiting for ARM
}

static void handle_state_armed(void) {
    // Waiting for LAUNCH or START_TEST
}

static void handle_state_test_stand(void) {
    // TODO: Test stand state machine
}

static void handle_state_flight(void) {
    // TODO: Flight state machine
}

static void handle_state_abort(void) {
    // Emergency state - cut throttle, deploy recovery
}

static void handle_state_safe(void) {
    // Final safe state - do nothing
}

// ============================================================================
// Main Thread
// ============================================================================
void fsm_thread_function(void *argument) {
    printf("[FSM] Thread starting...\r\n");


    // Initialize FSM
    fsm_init();

    // Report thread started
    fsm_report_thread_started("FSM");

    fsm_cmd_msg_t cmd_msg;
    fsm_event_msg_t evt_msg;

    TickType_t last_stats = xTaskGetTickCount();

    while (1) {
        // Process commands from radio thread
        while (xQueueReceive(queue_cmd_to_fsm, &cmd_msg, 0) == pdTRUE) {
            process_command(&cmd_msg);
        }

        // Process internal events from estimator
        while (xQueueReceive(queue_event_to_fsm, &evt_msg, 0) == pdTRUE) {
            // TODO: Handle internal events (liftoff, apogee, etc.)
            printf("[FSM] Internal event: type=%u\r\n", evt_msg.type);
        }

        // State-specific processing
        switch (fsm_ctx.state) {
            case STATE_BOOT:       handle_state_boot(); break;
            case STATE_IDLE:       handle_state_idle(); break;
            case STATE_CONFIGED:   handle_state_configed(); break;
            case STATE_ARMED:      handle_state_armed(); break;
            case STATE_TEST_STAND: handle_state_test_stand(); break;
            case STATE_FLIGHT:     handle_state_flight(); break;
            case STATE_ABORT:      handle_state_abort(); break;
            case STATE_SAFE:       handle_state_safe(); break;
            default: break;
        }

        // Periodic stats
        TickType_t now = xTaskGetTickCount();
        if ((now - last_stats) >= pdMS_TO_TICKS(STATS_INTERVAL_MS)) {
            printf("[FSM] State=%s.%s | Profile=%s | Target=%.1fm Flare=%.1fm\r\n",
                   fsm_state_to_str(fsm_ctx.state),
                   fsm_substate_to_str(fsm_ctx.substate),
                   fsm_profile_to_str(fsm_ctx.profile.type),
                   fsm_ctx.profile.target_altitude_m,
                   fsm_ctx.profile.flare_altitude_m);
            last_stats = now;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Public function for other threads to send telemetry events
void fsm_send_telemetry_event(telemetry_event_type_t type, const void *payload, uint16_t size) {
    send_telem_event(type, payload, size);
}
