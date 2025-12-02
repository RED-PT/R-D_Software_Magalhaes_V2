/*
 * flight_computer.c
 */

#include "flight_computer.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "print.h"
#include <string.h>

// Global context
fsm_ctx_t fsm_ctx = {0};

// Mutex for thread-safe access
static SemaphoreHandle_t fsm_mutex = NULL;

// ============================================================================
// State/Substate String Tables
// ============================================================================
static const char* state_strings[] = {
    "BOOT", "IDLE", "CONFIGED", "ARMED",
    "TEST_STAND", "FLIGHT", "ABORT", "SAFE"
};

static const char* substate_strings[] = {
    "NONE",
    "TS_SENSOR_CHECK", "TS_THROTTLE_RAMP",
    "FL_IGNITION", "FL_LIFTOFF_DETECT", "FL_ASCENT", "FL_COAST",
    "FL_DESCENT_BRAKE", "FL_LANDING_FLARE", "FL_TOUCHDOWN", "FL_RECOVERY"
};

static const char* profile_strings[] = {
    "NONE", "GUTTER_RAMP", "GUTTER_HOLD", "FLIGHT_PARAM"
};

const char* fsm_state_to_str(fsm_state_t state) {
    if (state <= STATE_SAFE) return state_strings[state];
    return "UNKNOWN";
}

const char* fsm_substate_to_str(fsm_substate_t substate) {
    if (substate <= SUB_FL_RECOVERY) return substate_strings[substate];
    return "UNKNOWN";
}

const char* fsm_profile_to_str(flight_profile_t profile) {
    if (profile <= PROFILE_FLIGHT_PARAM) return profile_strings[profile];
    return "UNKNOWN";
}

// ============================================================================
// Initialization
// ============================================================================
void fsm_init(void) {
    printf("[FSM] Initializing...\r\n");

    fsm_mutex = xSemaphoreCreateMutex();
    if (fsm_mutex == NULL) {
        printf("[FSM] ERROR: Failed to create mutex!\r\n");
    }

    // Clear context
    memset(&fsm_ctx, 0, sizeof(fsm_ctx_t));

    // Initialize boot status to NOT_CHECKED (-1)
    fsm_ctx.boot_status.imu_init = -1;
    fsm_ctx.boot_status.baro_init = -1;
    fsm_ctx.boot_status.mag_init = -1;
    fsm_ctx.boot_status.bno_init = -1;
    fsm_ctx.boot_status.gps_init = -1;
    fsm_ctx.boot_status.sd_card_init = -1;
    fsm_ctx.boot_status.radio_init = -1;
    fsm_ctx.boot_status.data_handler_init = -1;

    fsm_ctx.boot_status.imu_config = -1;
    fsm_ctx.boot_status.baro_config = -1;
    fsm_ctx.boot_status.mag_config = -1;
    fsm_ctx.boot_status.bno_config = -1;
    fsm_ctx.boot_status.gps_config = -1;

    fsm_ctx.boot_status.sensors_thread_started = -1;
    fsm_ctx.boot_status.data_handler_thread_started = -1;
    fsm_ctx.boot_status.telemetry_thread_started = -1;
    fsm_ctx.boot_status.radio_thread_started = -1;
    fsm_ctx.boot_status.sd_card_thread_started = -1;
    fsm_ctx.boot_status.estimator_thread_started = -1;
    fsm_ctx.boot_status.controller_thread_started = -1;

    // Initial state
    fsm_ctx.state = STATE_BOOT;
    fsm_ctx.substate = SUB_NONE;
    fsm_ctx.profile.type = PROFILE_NONE;

    // Set default profile limits
    fsm_ctx.profile.throttle_min = 0.0f;
    fsm_ctx.profile.throttle_max = 1.0f;
    fsm_ctx.profile.max_altitude_m = 100.0f;
    fsm_ctx.profile.max_velocity_ms = 50.0f;

    // Timing
    fsm_ctx.boot_start_tick = HAL_GetTick();
    fsm_ctx.state_entry_tick = HAL_GetTick();

    printf("[FSM] Init complete, state=BOOT\r\n");
}

// ============================================================================
// State Access (Thread-Safe)
// ============================================================================
fsm_state_t fsm_get_state(void) {
    fsm_state_t state;
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = fsm_ctx.state;
        xSemaphoreGive(fsm_mutex);
    } else {
        state = fsm_ctx.state;
    }
    return state;
}

fsm_substate_t fsm_get_substate(void) {
    fsm_substate_t substate;
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        substate = fsm_ctx.substate;
        xSemaphoreGive(fsm_mutex);
    } else {
        substate = fsm_ctx.substate;
    }
    return substate;
}

void fsm_set_state(fsm_state_t state) {
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        fsm_ctx.state = state;
        xSemaphoreGive(fsm_mutex);
    }
}

void fsm_set_substate(fsm_substate_t substate) {
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        fsm_ctx.substate = substate;
        xSemaphoreGive(fsm_mutex);
    }
}

// ============================================================================
// Boot Status Reporting
// ============================================================================
void fsm_report_init_status(const char *component, bool success) {
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        printf("[FSM] WARNING: Could not lock mutex for init report\r\n");
        return;
    }

    boot_status_t *bs = &fsm_ctx.boot_status;
    int8_t status = success ? 1 : 0;

    if (strcmp(component, "IMU_INIT") == 0) bs->imu_init = status;
    else if (strcmp(component, "IMU_CONFIG") == 0) bs->imu_config = status;
    else if (strcmp(component, "BARO_INIT") == 0) bs->baro_init = status;
    else if (strcmp(component, "BARO_CONFIG") == 0) bs->baro_config = status;
    else if (strcmp(component, "MAG_INIT") == 0) bs->mag_init = status;
    else if (strcmp(component, "MAG_CONFIG") == 0) bs->mag_config = status;
    else if (strcmp(component, "BNO_INIT") == 0) bs->bno_init = status;
    else if (strcmp(component, "BNO_CONFIG") == 0) bs->bno_config = status;
    else if (strcmp(component, "GPS_INIT") == 0) bs->gps_init = status;
    else if (strcmp(component, "GPS_CONFIG") == 0) bs->gps_config = status;
    else if (strcmp(component, "SD_CARD") == 0) bs->sd_card_init = status;
    else if (strcmp(component, "RADIO") == 0) bs->radio_init = status;
    else if (strcmp(component, "DATA_HANDLER") == 0) bs->data_handler_init = status;

    bs->total_checks++;
    if (success) {
        bs->passed_checks++;
    } else {
        bs->failed_checks++;
        if (strstr(component, "IMU") || strstr(component, "BARO")) {
            bs->critical_failures++;
        }
    }

    if (fsm_mutex) xSemaphoreGive(fsm_mutex);
    printf("[FSM] Init report: %s = %s\r\n", component, success ? "OK" : "FAIL");
}

void fsm_report_thread_started(const char *thread_name) {
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    boot_status_t *bs = &fsm_ctx.boot_status;

    if (strcmp(thread_name, "SENSORS") == 0) bs->sensors_thread_started = 1;
    else if (strcmp(thread_name, "DATA_HANDLER") == 0) bs->data_handler_thread_started = 1;
    else if (strcmp(thread_name, "TELEMETRY") == 0) bs->telemetry_thread_started = 1;
    else if (strcmp(thread_name, "RADIO") == 0) bs->radio_thread_started = 1;
    else if (strcmp(thread_name, "SD_CARD") == 0) bs->sd_card_thread_started = 1;
    else if (strcmp(thread_name, "ESTIMATOR") == 0) bs->estimator_thread_started = 1;
    else if (strcmp(thread_name, "CONTROLLER") == 0) bs->controller_thread_started = 1;

    if (fsm_mutex) xSemaphoreGive(fsm_mutex);
    printf("[FSM] Thread started: %s\r\n", thread_name);
}

// ============================================================================
// State Queries
// ============================================================================
bool fsm_should_queue_to_estimator(void) {
    fsm_state_t state = fsm_get_state();
    return (state == STATE_ARMED || state == STATE_TEST_STAND || state == STATE_FLIGHT);
}

bool fsm_should_queue_to_sd(void) {
    fsm_state_t state = fsm_get_state();
    return (state == STATE_ARMED || state == STATE_TEST_STAND ||
            state == STATE_FLIGHT || state == STATE_ABORT);
}

bool fsm_is_logging_enabled(void) {
    return fsm_ctx.flags.sd_logging_enabled;
}

bool fsm_is_boot_complete(void) {
    boot_status_t *bs = &fsm_ctx.boot_status;

    if (bs->sensors_thread_started != 1) return false;
    if (bs->data_handler_thread_started != 1) return false;
    if (bs->telemetry_thread_started != 1) return false;
    if (bs->radio_thread_started != 1) return false;
    if (bs->imu_init != 1) return false;
    if (bs->baro_init != 1) return false;

    return true;
}

// ============================================================================
// Command/Event Sending (for other threads)
// ============================================================================
bool fsm_send_command(fsm_command_t cmd, uint8_t cmd_seq, const void *payload, uint16_t size) {
    if (queue_cmd_to_fsm == NULL) {
        printf("[FSM] ERROR: queue_cmd_to_fsm is NULL\r\n");
        return false;
    }

    fsm_cmd_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.cmd = cmd;
    msg.cmd_seq = cmd_seq;

    if (payload && size > 0) {
        uint16_t copy_size = (size > sizeof(msg.payload)) ? sizeof(msg.payload) : size;
        memcpy(&msg.payload, payload, copy_size);
    }

    if (xQueueSend(queue_cmd_to_fsm, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        printf("[FSM] ERROR: Failed to queue command\r\n");
        return false;
    }

    return true;
}

bool fsm_send_event(fsm_internal_event_t event_type, const void *data) {
    if (queue_event_to_fsm == NULL) {
        printf("[FSM] ERROR: queue_event_to_fsm is NULL\r\n");
        return false;
    }

    fsm_event_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = event_type;
    msg.timestamp = HAL_GetTick();

    if (data) {
        memcpy(&msg.data, data, sizeof(msg.data));
    }

    if (xQueueSend(queue_event_to_fsm, &msg, pdMS_TO_TICKS(10)) != pdTRUE) {
        printf("[FSM] ERROR: Failed to queue event\r\n");
        return false;
    }

    return true;
}
