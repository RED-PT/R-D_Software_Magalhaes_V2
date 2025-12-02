/*
 * flight_computer.h
 *
 *  Created on: Oct 7, 2025
 *      Author: Tomas Teixeira
 */

#ifndef FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_
#define FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_

#include "defs.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// FSM States
// ============================================================================
typedef enum {
    STATE_BOOT = 0,
    STATE_IDLE,
    STATE_CONFIGED,
    STATE_ARMED,
    STATE_TEST_STAND,
    STATE_FLIGHT,
    STATE_ABORT,
    STATE_SAFE
} fsm_state_t;

// ============================================================================
// FSM Sub-states
// ============================================================================
typedef enum {
    SUB_NONE = 0,
    // TEST_STAND
    SUB_TS_SENSOR_CHECK,
    SUB_TS_THROTTLE_RAMP,
    // FLIGHT
    SUB_FL_IGNITION,
    SUB_FL_LIFTOFF_DETECT,
    SUB_FL_ASCENT,
    SUB_FL_COAST,
    SUB_FL_DESCENT_BRAKE,
    SUB_FL_LANDING_FLARE,
    SUB_FL_TOUCHDOWN,
    SUB_FL_RECOVERY
} fsm_substate_t;

// ============================================================================
// Flight Profiles
// ============================================================================
typedef enum {
    PROFILE_NONE = 0,
    PROFILE_GUTTER_RAMP = 1,
    PROFILE_GUTTER_HOLD = 2,
    PROFILE_FLIGHT_PARAM = 3
} flight_profile_t;

// ============================================================================
// Internal FSM Events (from estimator to FSM)
// ============================================================================
typedef enum {
    FSM_EVT_NONE = 0,
    FSM_EVT_LIFTOFF,
    FSM_EVT_APOGEE,
    FSM_EVT_DESCENT_START,
    FSM_EVT_FLARE_ALT,
    FSM_EVT_TOUCHDOWN,
    FSM_EVT_LANDED,
    FSM_EVT_ALTITUDE_LIMIT,
    FSM_EVT_VELOCITY_LIMIT
} fsm_internal_event_t;

// ============================================================================
// Profile Parameters
// ============================================================================
typedef struct {
    flight_profile_t type;

    // Common parameters
    float target_altitude_m;
    float flare_altitude_m;
    float touchdown_velocity_ms;
    float max_altitude_m;
    float max_velocity_ms;      // Safety limit

    // Gutter Hold specific
    float hold_throttle;

    // Gutter Ramp specific
    float ramp_duration_s;

    // Safety limits
    float throttle_min;
    float throttle_max;
} profile_params_t;

// ============================================================================
// Boot Status
// ============================================================================
typedef struct {
    int8_t imu_init;
    int8_t baro_init;
    int8_t mag_init;
    int8_t bno_init;
    int8_t gps_init;
    int8_t sd_card_init;
    int8_t radio_init;
    int8_t data_handler_init;

    int8_t imu_config;
    int8_t baro_config;
    int8_t mag_config;
    int8_t bno_config;
    int8_t gps_config;

    int8_t sensors_thread_started;
    int8_t data_handler_thread_started;
    int8_t telemetry_thread_started;
    int8_t radio_thread_started;
    int8_t sd_card_thread_started;
    int8_t estimator_thread_started;
    int8_t controller_thread_started;

    uint8_t total_checks;
    uint8_t passed_checks;
    uint8_t failed_checks;
    uint8_t critical_failures;
} boot_status_t;

// ============================================================================
// Boot Report
// ============================================================================
#define BOOT_REPORT_MAX_ERRORS 8
#define BOOT_ERROR_MSG_LEN     16

typedef struct __attribute__((packed)) {
    uint8_t total_checks;
    uint8_t passed;
    uint8_t failed;
    uint8_t critical;
    uint32_t boot_time_ms;
    char errors[BOOT_REPORT_MAX_ERRORS][BOOT_ERROR_MSG_LEN];
    uint8_t error_count;
} boot_report_t;

// ============================================================================
// FSM Flags
// ============================================================================
typedef struct {
    uint8_t baro_calibrated : 1;
    uint8_t gps_locked : 1;
    uint8_t sensors_healthy : 1;
    uint8_t sd_logging_enabled : 1;
    uint8_t estimator_running : 1;
    uint8_t controller_running : 1;
    uint8_t abort_requested : 1;
    uint8_t reserved : 1;
} fsm_flags_t;

// ============================================================================
// FSM Context
// ============================================================================
typedef struct {
    // State
    fsm_state_t state;
    fsm_substate_t substate;
    fsm_state_t prev_state;
    fsm_substate_t prev_substate;

    // Profile
    profile_params_t profile;

    // Real-time navigation data (updated by estimator)
    float altitude_agl_m;
    float velocity_vertical_ms;
    float max_altitude_reached_m;

    // Timing
    uint32_t state_entry_tick;
    uint32_t flight_start_tick;
    uint32_t boot_start_tick;

    // Command tracking
    uint8_t last_cmd_seq;
    uint8_t last_cmd_status;

    // Boot status
    boot_status_t boot_status;

    // Runtime flags
    fsm_flags_t flags;

} fsm_ctx_t;

// ============================================================================
// Commands (GS → FC)
// ============================================================================
typedef enum {
    CMD_NONE = 0,
    CMD_PING,
    CMD_SET_PROFILE,
    CMD_SET_PARAM,
    CMD_ARM,
    CMD_DISARM,
    CMD_START_TEST,
    CMD_LAUNCH,
    CMD_ABORT,
    CMD_FORCE_SAFE,
    CMD_CALIBRATE_BARO,
    CMD_RESYNC
} fsm_command_t;

// ============================================================================
// Telemetry Events (FC → GS)
// ============================================================================
typedef enum {
    EVT_STATE_CHANGE = 0,
    EVT_FAULT,
    EVT_ABORT_TRIGGERED,
    EVT_PROFILE_LOADED,
    EVT_CHECKS_GREEN,
    EVT_CHECKS_RED,
    EVT_BOOT_REPORT,
    EVT_ARMED,
    EVT_DISARMED,
    EVT_LIFTOFF,
    EVT_APOGEE,
    EVT_LANDING,
    EVT_GENERIC_MSG
} telemetry_event_type_t;

// ============================================================================
// Command Message (radio_thread → fsm_thread)
// ============================================================================
typedef struct {
    fsm_command_t cmd;
    uint8_t cmd_seq;
    union {
        struct __attribute__((packed)) {
            uint8_t profile_type;
            float param1;
            float param2;
        } profile;
        float target_altitude;
        uint8_t raw[16];
    } payload;
} fsm_cmd_msg_t;

// ============================================================================
// Event Message (estimator_thread → fsm_thread)
// ============================================================================
typedef struct {
    fsm_internal_event_t type;
    uint32_t timestamp;
    union {
        float altitude;
        float velocity;
        uint8_t raw[8];
    } data;
} fsm_event_msg_t;

// ============================================================================
// State Change Payload
// ============================================================================
typedef struct __attribute__((packed)) {
    uint8_t old_state;
    uint8_t old_substate;
    uint8_t new_state;
    uint8_t new_substate;
    uint8_t reason;
} event_state_change_t;

// ============================================================================
// Fault Payload
// ============================================================================
typedef struct __attribute__((packed)) {
    uint16_t code;
    char desc[32];
} event_fault_t;

// ============================================================================
// Queues (declared in create_threads.c or flight_computer_thread.c)
// ============================================================================
extern QueueHandle_t queue_cmd_to_fsm;
extern QueueHandle_t queue_event_to_fsm;

// ============================================================================
// Global Context
// ============================================================================
extern fsm_ctx_t fsm_ctx;

// ============================================================================
// Function Prototypes
// ============================================================================

// Init
void fsm_init(void);

// State access (thread-safe)
fsm_state_t fsm_get_state(void);
fsm_substate_t fsm_get_substate(void);
void fsm_set_state(fsm_state_t state);
void fsm_set_substate(fsm_substate_t substate);

// Boot status reporting (called by other threads)
void fsm_report_init_status(const char *component, bool success);
void fsm_report_thread_started(const char *thread_name);
bool fsm_is_boot_complete(void);

// State queries for other threads
bool fsm_should_queue_to_estimator(void);
bool fsm_should_queue_to_sd(void);
bool fsm_is_logging_enabled(void);

// Command/Event sending (for other threads)
bool fsm_send_command(fsm_command_t cmd, uint8_t cmd_seq, const void *payload, uint16_t size);
bool fsm_send_event(fsm_internal_event_t event_type, const void *data);

// String helpers
const char* fsm_state_to_str(fsm_state_t state);
const char* fsm_substate_to_str(fsm_substate_t substate);
const char* fsm_profile_to_str(flight_profile_t profile);

#endif /* FLIGHT_COMPUTER_FLIGHT_COMPUTER_H_ */
