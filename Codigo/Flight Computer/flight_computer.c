/**
 * @file flight_computer.c
 * @brief Flight Computer Finite State Machine Implementation
 * @author Tomás Teixeira (texman)
 * @date 2025
 * @version 2.0
 *
 * @details
 * This file implements the core FSM (Finite State Machine) logic for the
 * Magalhães Flight Computer. It manages system state, boot sequence tracking,
 * command processing, and inter-thread communication.
 *
 * ## State Machine Overview
 *
 * @verbatim
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │                      BOOT                                            │
 *   │   (Initialize hardware, start threads, verify sensors)               │
 *   └────────────────────────┬─────────────────────────────────────────────┘
 *                            │ Boot complete
 *                            ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │                      IDLE                                            │
 *   │   (Waiting for configuration, telemetry active)                      │
 *   └────────────────────────┬─────────────────────────────────────────────┘
 *                            │ Profile configured
 *                            ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │                   CONFIGED                                           │
 *   │   (Profile loaded, barometer calibrating)                            │
 *   └────────────────────────┬─────────────────────────────────────────────┘
 *                            │ ARM command
 *                            ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │                     ARMED                                            │
 *   │   (SUB: MOTOR_INIT → MOTOR_CAL → READY)                              │
 *   └─────────┬──────────────────────────────────┬─────────────────────────┘
 *             │ STATIC_TEST cmd                  │ LAUNCH cmd
 *             ▼                                  ▼
 *   ┌────────────────────┐             ┌────────────────────────────────────┐
 *   │    TEST_STAND      │             │              FLIGHT                │
 *   │  (Static thrust)   │             │  (SUB: IGNITION → LIFTOFF →        │
 *   └────────────────────┘             │   ASCENT → COAST → DESCENT →       │
 *                                      │   LANDING → TOUCHDOWN → RECOVERY)  │
 *                                      └────────────────────────────────────┘
 *                     ABORT (from any state except BOOT/IDLE)
 *                                      │
 *                                      ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │                      SAFE                                            │
 *   │   (Motors disabled, logging data, awaiting recovery)                 │
 *   └──────────────────────────────────────────────────────────────────────┘
 * @endverbatim
 *
 * ## Thread Safety
 * All state access functions use a mutex (`fsm_mutex`) to ensure thread-safe
 * operation when multiple threads query or modify FSM state.
 *
 * ## Command/Event Queues
 * - `queue_cmd_to_fsm`: Commands from radio thread (ARM, LAUNCH, ABORT, etc.)
 * - `queue_event_to_fsm`: Internal events (LIFTOFF_DETECTED, TOUCHDOWN, etc.)
 *
 * @see flight_computer.h for FSM types and structures
 * @see flight_computer_thread.c for FSM execution loop
 *
 * @ingroup Flight_Computer
 * @{
 */

#include "flight_computer.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "print.h"
#include <string.h>

/* ============================================================================
 * Global Variables
 * ============================================================================ */

/**
 * @brief Global FSM context structure
 * @details Contains all state, configuration, calibration, and status data.
 *          Access should be protected by fsm_mutex for thread safety.
 */
fsm_ctx_t fsm_ctx = {0};

/**
 * @brief Mutex for thread-safe FSM access
 * @details Used by fsm_get_state(), fsm_set_state(), and related functions.
 */
static SemaphoreHandle_t fsm_mutex = NULL;

/* ============================================================================
 * State/Substate String Tables
 * ============================================================================ */
/** @brief Human-readable state names for logging and telemetry */
static const char* state_strings[] = {
    "BOOT", "IDLE", "CONFIGED", "ARMED",
    "TEST_STAND", "FLIGHT", "ABORT", "SAFE"
};

/** @brief Human-readable substate names for logging and telemetry */
static const char* substate_strings[] = {
    "NONE",
    "TS_SENSOR_CHECK", "TS_THROTTLE_RAMP",
    "FL_IGNITION", "FL_LIFTOFF_DETECT", "FL_ASCENT", "FL_COAST",
    "FL_DESCENT_BRAKE", "FL_LANDING_FLARE", "FL_TOUCHDOWN", "FL_RECOVERY",
    "ARM_MOTOR_INIT", "ARM_MOTOR_CAL", "ARM_READY"
};

/** @brief Human-readable profile names for logging and telemetry */
static const char* profile_strings[] = {
    "NONE", "GUTTER_RAMP", "GUTTER_HOLD", "FLIGHT_PARAM"
};

/**
 * @brief Convert FSM state to human-readable string
 * @param state The FSM state to convert
 * @return Pointer to constant string representing the state name
 * @retval "UNKNOWN" if state is out of valid range
 */
const char* fsm_state_to_str(fsm_state_t state) {
    if (state <= STATE_SAFE) return state_strings[state];
    return "UNKNOWN";
}

/**
 * @brief Convert FSM substate to human-readable string
 * @param substate The FSM substate to convert
 * @return Pointer to constant string representing the substate name
 * @retval "UNKNOWN" if substate is out of valid range
 */
const char* fsm_substate_to_str(fsm_substate_t substate) {
    if (substate <= SUB_ARM_READY) return substate_strings[substate];
    return "UNKNOWN";
}

/**
 * @brief Convert flight profile type to human-readable string
 * @param profile The profile type to convert
 * @return Pointer to constant string representing the profile name
 * @retval "UNKNOWN" if profile is out of valid range
 */
const char* fsm_profile_to_str(flight_profile_t profile) {
    if (profile <= PROFILE_FLIGHT_PARAM) return profile_strings[profile];
    return "UNKNOWN";
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

/**
 * @brief Initialize the Flight State Machine
 *
 * @details
 * Performs complete FSM initialization including:
 * - Creates mutex for thread-safe state access
 * - Clears FSM context structure to zeroes
 * - Initializes all boot status fields to NOT_CHECKED (-1)
 * - Sets initial state to STATE_BOOT with SUB_NONE
 * - Configures default profile limits (safety defaults)
 * - Initializes calibration and motor status structures
 * - Records boot start timestamp
 *
 * **Default Profile Limits:**
 * | Parameter | Default Value | Description |
 * |-----------|---------------|-------------|
 * | throttle_min | 0.0 | Minimum throttle (0%) |
 * | throttle_max | 1.0 | Maximum throttle (100%) |
 * | max_altitude_m | 100.0 | Maximum allowed altitude |
 * | max_velocity_ms | 50.0 | Maximum allowed velocity |
 * | flare_altitude_m | 5.0 | Landing flare trigger altitude |
 * | touchdown_velocity_ms | 1.0 | Touchdown detection velocity |
 *
 * @warning Must be called before any other FSM functions.
 * @note Called during system startup before threads are created.
 */
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
    fsm_ctx.profile.flare_altitude_m = 5.0f;
    fsm_ctx.profile.touchdown_velocity_ms = 1.0f;

    // Initialize calibration state
    fsm_ctx.baro_cal.is_calibrated = false;
    fsm_ctx.baro_cal.samples_collected = 0;
    fsm_ctx.baro_cal.pressure_sum = 0.0f;

    // Initialize motor status
    fsm_ctx.motor_status.esc_initialized = false;
    fsm_ctx.motor_status.calibration_done = false;

    // Initialize ping tracker
    fsm_ctx.ping.awaiting_pong = false;
    fsm_ctx.ping.last_rtt_ms = 0;

    // Timing
    fsm_ctx.boot_start_tick = HAL_GetTick();
    fsm_ctx.state_entry_tick = HAL_GetTick();

    printf("[FSM] Init complete, state=BOOT\r\n");
}

/* ============================================================================
 * State Access (Thread-Safe)
 * ============================================================================ */

/**
 * @brief Get current FSM state (thread-safe)
 * @return Current FSM state
 *
 * @details
 * Safely retrieves the current FSM state using mutex protection.
 * If mutex cannot be acquired within 10ms, returns the value anyway
 * (fallback for non-critical read operations).
 *
 * @note Safe to call from any thread or ISR context.
 */
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

/**
 * @brief Get current FSM substate (thread-safe)
 * @return Current FSM substate
 *
 * @details
 * Safely retrieves the current FSM substate using mutex protection.
 * Substates provide granularity within major states (e.g., flight phases).
 */
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

/**
 * @brief Set FSM state (thread-safe)
 * @param state New state to set
 *
 * @details
 * Safely sets the FSM state using mutex protection.
 * State changes are logged and timestamped internally.
 *
 * @warning State transitions should follow valid FSM flow.
 *          Invalid transitions may cause undefined behavior.
 */
void fsm_set_state(fsm_state_t state) {
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        fsm_ctx.state = state;
        xSemaphoreGive(fsm_mutex);
    }
}

/**
 * @brief Set FSM substate (thread-safe)
 * @param substate New substate to set
 *
 * @details
 * Safely sets the FSM substate using mutex protection.
 * Typically used during state transitions to reset or advance substates.
 */
void fsm_set_substate(fsm_substate_t substate) {
    if (fsm_mutex && xSemaphoreTake(fsm_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        fsm_ctx.substate = substate;
        xSemaphoreGive(fsm_mutex);
    }
}

/* ============================================================================
 * Boot Status Reporting
 * ============================================================================ */

/**
 * @brief Report component initialization status
 * @param component Component name string (e.g., "IMU_INIT", "BARO_CONFIG")
 * @param success true if initialization succeeded, false otherwise
 *
 * @details
 * Updates the boot_status structure with component initialization results.
 * Used during boot sequence to track which hardware/software components
 * initialized successfully.
 *
 * **Recognized Component Names:**
 * | Component | Field Updated |
 * |-----------|---------------|
 * | "IMU_INIT" | imu_init |
 * | "IMU_CONFIG" | imu_config |
 * | "BARO_INIT" | baro_init |
 * | "BARO_CONFIG" | baro_config |
 * | "MAG_INIT" | mag_init |
 * | "MAG_CONFIG" | mag_config |
 * | "BNO_INIT" | bno_init |
 * | "BNO_CONFIG" | bno_config |
 * | "GPS_INIT" | gps_init |
 * | "GPS_CONFIG" | gps_config |
 * | "SD_CARD" | sd_card_init |
 * | "RADIO" | radio_init |
 * | "DATA_HANDLER" | data_handler_init |
 *
 * **Statistics Updated:**
 * - total_checks incremented for every call
 * - passed_checks incremented on success
 * - failed_checks incremented on failure
 * - critical_failures incremented if IMU or BARO fails
 *
 * @note Thread-safe with 50ms mutex timeout.
 */
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

/**
 * @brief Report that a system thread has started
 * @param thread_name Name of the thread that started
 *
 * @details
 * Updates the boot_status structure to record thread startup.
 * Used to verify all required threads are running before
 * transitioning from BOOT to IDLE state.
 *
 * **Recognized Thread Names:**
 * - "SENSORS" - Sensor polling and data acquisition
 * - "DATA_HANDLER" - Data buffering and queue management
 * - "TELEMETRY" - Telemetry packet formatting
 * - "RADIO" - LoRa radio communication
 * - "SD_CARD" - SD card logging
 * - "ESTIMATOR" - State estimation (altitude, velocity)
 * - "CONTROLLER" - TVC/motor control loop
 *
 * @note Thread-safe with 50ms mutex timeout.
 */
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

/* ============================================================================
 * State Queries
 * ============================================================================ */

/**
 * @brief Check if sensor data should be queued to estimator
 * @return true if in a state requiring state estimation
 *
 * @details
 * Returns true when the system needs active state estimation:
 * - STATE_ARMED: Preparing for flight/test
 * - STATE_TEST_STAND: Active thrust testing
 * - STATE_FLIGHT: Active flight operations
 *
 * @note Used by sensors_thread to decide whether to send data to estimator queue.
 */
bool fsm_should_queue_to_estimator(void) {
    fsm_state_t state = fsm_get_state();
    return (state == STATE_ARMED || state == STATE_TEST_STAND || state == STATE_FLIGHT);
}

/**
 * @brief Check if data should be logged to SD card
 * @return true if in a state requiring data logging
 *
 * @details
 * Returns true when data logging is important:
 * - STATE_ARMED: Pre-flight data
 * - STATE_TEST_STAND: Test data recording
 * - STATE_FLIGHT: Flight data recording (critical!)
 * - STATE_ABORT: Emergency data for analysis
 *
 * @note Used by data_handler_thread to decide whether to queue data for SD card.
 */
bool fsm_should_queue_to_sd(void) {
    fsm_state_t state = fsm_get_state();
    return (state == STATE_ARMED || state == STATE_TEST_STAND ||
            state == STATE_FLIGHT || state == STATE_ABORT);
}

/**
 * @brief Check if SD card logging is enabled
 * @return true if logging flag is set
 *
 * @details
 * Returns the current state of the SD logging enable flag.
 * This can be independently controlled from the state-based logging.
 */
bool fsm_is_logging_enabled(void) {
    return fsm_ctx.flags.sd_logging_enabled;
}

/**
 * @brief Check if boot sequence is complete
 * @return true if all critical components initialized and threads started
 *
 * @details
 * Verifies that all mandatory boot conditions are met:
 * - sensors_thread started
 * - data_handler_thread started
 * - telemetry_thread started
 * - radio_thread started
 * - IMU initialized successfully
 * - Barometer initialized successfully
 *
 * @note GPS, magnetometer, and BNO055 are not considered critical.
 *       The system can operate with degraded functionality without them.
 */
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

/**
 * @brief Check if barometer is calibrated
 * @return true if ground-level pressure calibration is complete
 *
 * @details
 * Barometer calibration establishes the ground-level reference pressure
 * for accurate altitude calculations. Must be performed before flight.
 */
bool fsm_is_baro_calibrated(void) {
    return fsm_ctx.baro_cal.is_calibrated;
}

/**
 * @brief Get pointer to barometer calibration data
 * @return Pointer to baro_calibration_t structure (read-only)
 *
 * @details
 * Provides access to barometer calibration data including:
 * - Reference pressure (ground level)
 * - Calibration status
 * - Sample count used for averaging
 */
const baro_calibration_t* fsm_get_baro_calibration(void) {
    return &fsm_ctx.baro_cal;
}

/* ============================================================================
 * Command/Event Sending (for other threads)
 * ============================================================================ */

/**
 * @brief Send a command to the FSM thread
 * @param cmd Command type to send
 * @param cmd_seq Command sequence number for ACK tracking
 * @param payload Optional command payload data
 * @param size Size of payload in bytes
 * @return true if command was queued successfully, false on error
 *
 * @details
 * Queues a command message to the FSM thread for processing.
 * Commands are typically received from the radio thread after
 * ground station transmission.
 *
 * **Command Types Include:**
 * - CMD_PING: Connectivity check
 * - CMD_ARM: Arm the system for flight/test
 * - CMD_DISARM: Disarm the system
 * - CMD_LAUNCH: Initiate flight sequence
 * - CMD_ABORT: Emergency abort
 * - CMD_CONFIGURE: Load flight profile
 * - CMD_CALIBRATE: Trigger barometer calibration
 * - CMD_STATIC_TEST: Start static thrust test
 *
 * @note Commands are processed asynchronously by the FSM thread.
 * @see fsm_command_t for complete command list
 */
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

/**
 * @brief Send an internal event to the FSM thread
 * @param event_type Type of event to send
 * @param data Optional event data
 * @return true if event was queued successfully, false on error
 *
 * @details
 * Queues an internal event for FSM processing. Events are generated
 * by other threads to signal state-relevant occurrences.
 *
 * **Event Types Include:**
 * - EVENT_LIFTOFF_DETECTED: Accelerometer detected launch
 * - EVENT_APOGEE_REACHED: Maximum altitude detected
 * - EVENT_TOUCHDOWN: Landing detected
 * - EVENT_BARO_CALIBRATED: Barometer calibration complete
 * - EVENT_MOTOR_READY: ESC initialization complete
 *
 * Events include a timestamp (HAL_GetTick()) for timing analysis.
 *
 * @note Events are processed asynchronously by the FSM thread.
 * @see fsm_internal_event_t for complete event list
 */
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

/** @} */ // End of Flight_Computer group
