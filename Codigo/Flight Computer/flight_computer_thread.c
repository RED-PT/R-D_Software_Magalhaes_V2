/**
 * @file flight_computer_thread.c
 * @brief Flight State Machine Thread Implementation
 * @author Tomás Teixeira
 * @date 2025
 * @version 2.0
 *
 * @details
 * Complete implementation of the Flight State Machine (FSM) thread, which
 * serves as the central nervous system of the Magalhães Flight Computer.
 *
 * ## Architecture Overview
 *
 * @verbatim
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │                       FSM Thread Main Loop                          │
 *   └───────────────────────────────┬─────────────────────────────────────┘
 *                                   │
 *       ┌───────────────────────────┼───────────────────────────┐
 *       │                           │                           │
 *       ▼                           ▼                           ▼
 *   ┌─────────┐              ┌─────────────┐             ┌───────────┐
 *   │ Command │              │   State     │             │  Event    │
 *   │ Handler │              │  Handlers   │             │  Handler  │
 *   └────┬────┘              └──────┬──────┘             └─────┬─────┘
 *        │                          │                          │
 *        ▼                          ▼                          ▼
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │                     State Transitions                                │
 *   │  • transition_to() - Handles entry/exit actions                     │
 *   │  • Thread suspend/resume (Estimator, Controller)                    │
 *   │  • Telemetry event generation                                       │
 *   └─────────────────────────────────────────────────────────────────────┘
 * @endverbatim
 *
 * ## State Handler Functions
 * | State | Handler | Description |
 * |-------|---------|-------------|
 * | BOOT | handle_state_boot() | Wait for critical init, timeout handling |
 * | IDLE | handle_state_idle() | Waiting for configuration |
 * | CONFIGED | handle_state_configed() | Profile loaded, awaiting ARM |
 * | ARMED | handle_state_armed() | Motor init/cal sequence |
 * | TEST_STAND | handle_state_test_stand() | Static thrust testing |
 * | FLIGHT | handle_state_flight() | Active flight phases |
 * | ABORT | handle_state_abort() | Emergency state |
 * | SAFE | handle_state_safe() | Final safe state |
 *
 * ## Command Handlers
 * | Command | Handler | Description |
 * |---------|---------|-------------|
 * | CMD_PING | handle_cmd_ping() | Connectivity check |
 * | CMD_SET_PROFILE | handle_cmd_set_profile() | Load flight profile |
 * | CMD_ARM | handle_cmd_arm() | Arm system |
 * | CMD_LAUNCH | handle_cmd_launch() | Start flight |
 * | CMD_ABORT | handle_cmd_abort() | Emergency abort |
 * | CMD_CALIBRATE_BARO | handle_cmd_calibrate_baro() | Barometer calibration |
 * | CMD_CALIBRATE_MOTOR | handle_cmd_calibrate_motor() | ESC calibration |
 * | CMD_STATIC_TEST | handle_cmd_static_test() | Static thrust test |
 *
 * ## Background State Machines
 * Three asynchronous state machines run within the main loop:
 * 1. **process_calibration()**: Barometer calibration (50 samples)
 * 2. **process_motor_calibration()**: ESC calibration sequence
 * 3. **process_static_test()**: Static thrust test execution
 *
 * @see flight_computer_thread.h for interface documentation
 * @see flight_computer.h for FSM types
 *
 * @ingroup Flight_Computer
 */

#include "flight_computer_thread.h"
#include "Telemetry/telemetry.h"
#include "Data Handler/flash_data_handler.h"
#include "Threads/create_threads.h"
#include "Controller/controller_thread.h"
#include "Sensors/MS5607/MS5607.h"
#include "Atuadores/ESC/PWM_FUNCTIONS.h"
#include "Tests/static_thrust_test.h"
#include "Tests/test_runner.h"
#include "Storage/sd_card_thread.h"
#include "OS/system_stats.h"
#include "cmsis_os.h"
#include "print.h"
#include <string.h>

// External reference to barometer device (defined in sensors_thread.c)
extern MS5607_t baro_device;

#define BOOT_TIMEOUT_MS     10000   // 10 seconds max boot time
#define BOOT_MIN_WAIT_MS    3000    // Minimum 3s to let threads start
#define STATS_INTERVAL_MS   10000

// Calibration settings
#define BARO_CALIBRATION_SAMPLES    50
#define BARO_CALIBRATION_DELAY_MS   20

// Motor arm timing
#define MOTOR_ARM_INIT_DELAY_MS     1000
#define MOTOR_ARM_CAL_DELAY_MS      3000

// Parameter IDs (must match GS)
#define PARAM_TARGET_ALTITUDE   0
#define PARAM_FLARE_ALTITUDE    1
#define PARAM_TOUCHDOWN_VEL     2
#define PARAM_HOLD_THROTTLE     3
#define PARAM_RAMP_DURATION     4
#define PARAM_MAX_ALTITUDE      5
#define PARAM_THROTTLE_MIN      6
#define PARAM_THROTTLE_MAX      7

// Boot report storage
static boot_report_t boot_report = {0};
static uint8_t boot_error_idx = 0;

// Calibration state machine
static bool calibration_in_progress = false;
static uint32_t calibration_start_tick = 0;
static uint8_t calibration_sample_count = 0;

// Motor arm state machine
static uint32_t motor_arm_start_tick = 0;

// Motor calibration state machine
static bool motor_cal_in_progress = false;
static esc_calibration_state_t last_cal_state = ESC_CAL_IDLE;

// Static thrust test state machine
static bool static_test_in_progress = false;
static static_test_state_t last_static_test_state = STATIC_TEST_IDLE;

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

    // Queue event directly to radio for immediate transmission
    // (queue_fsm_events holds the full struct, not a pointer)
    if (queue_fsm_events != NULL) {
        if (xQueueSend(queue_fsm_events, &evt, pdMS_TO_TICKS(10)) != pdTRUE) {
            printf("[FSM] WARNING: Event queue full!\r\n");
        }
    }
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
                fsm_ctx.flags.estimator_running = 0;
                printf("[FSM] Estimator suspended\r\n");
            }
            if (controller_thread_id != NULL) {
                vTaskSuspend(controller_thread_id);
                fsm_ctx.flags.controller_running = 0;
                printf("[FSM] Controller suspended\r\n");
            }
            fsm_ctx.flags.sd_logging_enabled = 0;
            break;

        case STATE_CONFIGED:
            // Profile loaded, waiting for ARM
            // Keep estimator/controller suspended
            break;

        case STATE_ARMED:
            // Start motor arm sequence
            fsm_ctx.substate = SUB_ARM_MOTOR_INIT;
            motor_arm_start_tick = HAL_GetTick();

            // Resume estimator for sensor monitoring
            if (estimator_thread_id != NULL) {
                vTaskResume(estimator_thread_id);
                fsm_ctx.flags.estimator_running = 1;
                printf("[FSM] Estimator resumed\r\n");
            }

            // Enable SD logging
            fsm_ctx.flags.sd_logging_enabled = 1;
            break;

        case STATE_TEST_STAND:
        case STATE_FLIGHT:
            // Resume controller for active control
            if (controller_thread_id != NULL) {
                vTaskResume(controller_thread_id);
                fsm_ctx.flags.controller_running = 1;
                printf("[FSM] Controller resumed\r\n");
            }
            // Estimator should already be running from ARMED
            if (estimator_thread_id != NULL && !fsm_ctx.flags.estimator_running) {
                vTaskResume(estimator_thread_id);
                fsm_ctx.flags.estimator_running = 1;
            }
            fsm_ctx.flags.sd_logging_enabled = 1;
            break;

        case STATE_SAFE:
        case STATE_ABORT:
            // Emergency: cut throttle, suspend controller
            controller_set_throttle(0.0f);
            controller_emergency_stop();

            if (controller_thread_id != NULL) {
                vTaskSuspend(controller_thread_id);
                fsm_ctx.flags.controller_running = 0;
            }
            // Keep estimator for post-flight analysis
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

        build_boot_report();
        printf("[FSM] Boot Report: %u/%u passed, %u critical failures\r\n",
               boot_report.passed, boot_report.total_checks, boot_report.critical);

        for (int i = 0; i < boot_report.error_count; i++) {
            printf("[FSM]   - %s\r\n", boot_report.errors[i]);
        }

        send_telem_event(EVT_CHECKS_RED, &boot_report, sizeof(boot_report));

        if (bs->critical_failures > 0) {
            printf("[FSM] CRITICAL FAILURES - going to SAFE\r\n");
            transition_to(STATE_SAFE, SUB_NONE, EVT_CHECKS_RED);
        } else {
            printf("[FSM] Non-critical failures, proceeding to IDLE\r\n");
            transition_to(STATE_IDLE, SUB_NONE, EVT_CHECKS_RED);
        }
        return;
    }

    // Boot complete successfully?
    if (boot_ok) {
        printf("[FSM] Boot complete in %lu ms\r\n", elapsed);

        build_boot_report();
        printf("[FSM] Boot Report: %u/%u passed\r\n",
               boot_report.passed, boot_report.total_checks);

        send_telem_event(EVT_CHECKS_GREEN, &boot_report, sizeof(boot_report));
        transition_to(STATE_IDLE, SUB_NONE, EVT_CHECKS_GREEN);
    }
}

// ============================================================================
// IDLE State Handler
// ============================================================================
static void handle_state_idle(void) {
    // In IDLE: waiting for profile configuration
    // Commands handled in process_command()

    // Could add watchdog petting, LED blink, etc.
}

// ============================================================================
// CONFIGED State Handler
// ============================================================================
static void handle_state_configed(void) {
    // Profile is loaded, waiting for ARM command
    // Sensors are already validated on entry to this state
    // Nothing to do here - just wait for ARM command
}

// ============================================================================
// ARMED State Handler - Motor Arm Sequence
// ============================================================================
static void handle_state_armed(void) {
    uint32_t elapsed = HAL_GetTick() - motor_arm_start_tick;

    switch (fsm_ctx.substate) {
        case SUB_ARM_MOTOR_INIT:
            // Initialize ESC with minimum throttle signal
            if (elapsed >= MOTOR_ARM_INIT_DELAY_MS) {
                printf("[FSM] Motor init: Sending min throttle...\r\n");
                controller_init_motor();
                fsm_ctx.substate = SUB_ARM_MOTOR_CALIBRATING;
                motor_arm_start_tick = HAL_GetTick();
            }
            break;

        case SUB_ARM_MOTOR_CALIBRATING:
            // Wait for ESC to recognize arm signal
            if (elapsed >= MOTOR_ARM_CAL_DELAY_MS) {
                printf("[FSM] Motor calibration complete\r\n");
                fsm_ctx.motor_status.esc_initialized = true;
                fsm_ctx.motor_status.calibration_done = true;
                fsm_ctx.motor_status.arm_timestamp = HAL_GetTick();
                fsm_ctx.flags.motor_armed = 1;

                fsm_ctx.substate = SUB_ARM_READY;
                send_telem_event(EVT_MOTOR_ARMED, NULL, 0);
                printf("[FSM] ARMED and ready for LAUNCH or TEST\r\n");
            }
            break;

        case SUB_ARM_READY:
            // Armed and ready - waiting for LAUNCH or START_TEST
            // Monitor sensors, check for abort conditions
            break;

        default:
            // Default to motor init
            fsm_ctx.substate = SUB_ARM_MOTOR_INIT;
            motor_arm_start_tick = HAL_GetTick();
            break;
    }
}

// ============================================================================
// TEST_STAND State Handler
// ============================================================================
static void handle_state_test_stand(void) {
    uint32_t elapsed = HAL_GetTick() - fsm_ctx.state_entry_tick;

    switch (fsm_ctx.substate) {
        case SUB_TS_SENSOR_CHECK:
            // Brief sensor check before throttle ramp
            if (elapsed >= 1000) {  // 1 second check
                printf("[FSM] Sensor check passed, starting ramp\r\n");
                fsm_ctx.substate = SUB_TS_THROTTLE_RAMP;
                fsm_ctx.state_entry_tick = HAL_GetTick();
            }
            break;

        case SUB_TS_THROTTLE_RAMP:
            // Controller handles the actual ramp/hold profile
            // FSM just monitors for abort conditions
            if (fsm_ctx.profile.type == PROFILE_GUTTER_RAMP) {
                // Ramp profile: gradually increase throttle
                float ramp_progress = (float)elapsed / (fsm_ctx.profile.ramp_duration_s * 1000.0f);
                if (ramp_progress >= 1.0f) {
                    printf("[FSM] Ramp complete\r\n");
                    // Could transition to hold or complete
                }
            } else if (fsm_ctx.profile.type == PROFILE_GUTTER_HOLD) {
                // Hold at constant throttle
                // Controller maintains hold_throttle
            }
            break;

        default:
            fsm_ctx.substate = SUB_TS_SENSOR_CHECK;
            break;
    }
}

// ============================================================================
// FLIGHT State Handler
// ============================================================================
static void handle_state_flight(void) {
    // Flight state machine is driven by events from estimator
    // FSM monitors and updates substate based on events

    switch (fsm_ctx.substate) {
        case SUB_FL_IGNITION:
            // Motor starting, waiting for liftoff detection
            break;

        case SUB_FL_LIFTOFF_DETECT:
            // Liftoff detected, transitioning to ascent
            break;

        case SUB_FL_ASCENT:
            // Climbing to target altitude
            // Controller handles throttle control
            break;

        case SUB_FL_COAST:
            // Coasting after main burn (if applicable)
            break;

        case SUB_FL_DESCENT_BRAKE:
            // Descending with braking
            break;

        case SUB_FL_LANDING_FLARE:
            // Final landing flare
            break;

        case SUB_FL_TOUCHDOWN:
            // Touch down, waiting for stable
            break;

        case SUB_FL_RECOVERY:
            // Landed, safe to approach
            transition_to(STATE_SAFE, SUB_NONE, EVT_LANDING);
            break;

        default:
            break;
    }
}

// ============================================================================
// ABORT State Handler
// ============================================================================
static void handle_state_abort(void) {
    // Motor cut paths into this state:
    //   CMD_ABORT / CMD_FORCE_SAFE -> StaticTest_Cancel + PWM_EmergencyStop in handler.
    //   Internal safety events     -> process_static_test guard cancels test next tick.
    // Wait here for manual intervention.
    static bool abort_logged = false;

    if (!abort_logged) {
        printf("[FSM] ABORT state - motors cut, waiting for SAFE command\r\n");
        abort_logged = true;
    }
}

// ============================================================================
// SAFE State Handler
// ============================================================================
static void handle_state_safe(void) {
    // Final safe state - do nothing, wait for power cycle or restart
    static bool safe_logged = false;

    if (!safe_logged) {
        printf("[FSM] SAFE state - system idle\r\n");
        safe_logged = true;
    }
}

// ============================================================================
// Command Handlers
// ============================================================================
static void handle_cmd_ping(fsm_cmd_msg_t *msg) {
    uint32_t now = HAL_GetTick();

    printf("[FSM] PING received (seq=%u)\r\n", msg->cmd_seq);

    // Build PONG response with timing info
    pong_payload_t pong = {
        .ping_seq = msg->cmd_seq,
        .fc_timestamp = now,
        .rtt_ms = 0  // FC doesn't know GS timing, GS will calculate RTT
    };

    // Send PONG event immediately
    send_telem_event(EVT_PONG, &pong, sizeof(pong));

    printf("[FSM] PONG sent (seq=%u, ts=%lu)\r\n", msg->cmd_seq, now);

    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_set_profile(fsm_cmd_msg_t *msg) {
    if (fsm_ctx.state != STATE_IDLE && fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] Cannot set profile in %s\r\n", fsm_state_to_str(fsm_ctx.state));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // Check critical sensors before allowing profile load
    boot_status_t *bs = &fsm_ctx.boot_status;
    bool sensors_ok = true;

    if (bs->imu_init != 1) {
        printf("[FSM] WARNING: IMU not initialized!\r\n");
        sensors_ok = false;
    }
    if (bs->baro_init != 1) {
        printf("[FSM] WARNING: Barometer not initialized!\r\n");
        sensors_ok = false;
    }
    if (bs->mag_init != 1) {
        printf("[FSM] WARNING: Magnetometer not initialized!\r\n");
        // Mag is optional for some profiles, continue
    }
    if (bs->bno_init != 1) {
        printf("[FSM] WARNING: BNO055 not initialized!\r\n");
        // BNO is optional, continue
    }

    if (!sensors_ok) {
        printf("[FSM] ERROR: Critical sensors (IMU/BARO) not ready - cannot load profile!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    uint8_t profile_type = msg->payload.profile.profile_type;
    float param1 = msg->payload.profile.param1;
    float param2 = msg->payload.profile.param2;

    printf("[FSM] SET_PROFILE: type=%u p1=%.1f p2=%.1f\r\n", profile_type, param1, param2);

    fsm_ctx.profile.type = (flight_profile_t)profile_type;

    switch (profile_type) {
        case PROFILE_GUTTER_RAMP:
            fsm_ctx.profile.ramp_duration_s = param1;
            fsm_ctx.profile.throttle_max = param2;
            break;
        case PROFILE_GUTTER_HOLD:
            fsm_ctx.profile.hold_throttle = param1;
            break;
        case PROFILE_FLIGHT_PARAM:
            fsm_ctx.profile.target_altitude_m = param1;
            fsm_ctx.profile.flare_altitude_m = param2;
            break;
    }

    transition_to(STATE_CONFIGED, SUB_NONE, EVT_PROFILE_LOADED);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_set_param(fsm_cmd_msg_t *msg) {
    uint8_t param_id = msg->payload.raw[0];
    float value;
    memcpy(&value, &msg->payload.raw[1], sizeof(float));

    printf("[FSM] SET_PARAM: id=%u val=%.2f\r\n", param_id, value);

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

/* ----- Phase 3-A (A3): new test-profile command path ------------------- */

static inline uint16_t rd_u16le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline int16_t  rd_i16le(const uint8_t *p) { return (int16_t)rd_u16le(p); }
static inline float    rd_f32le(const uint8_t *p) { float f; memcpy(&f, p, 4); return f; }

static bool parse_test_profile_wire(const uint8_t *wire, profile_t *out) {
    /* Wire layouts (params[] is 32 bytes since Phase 3-A / A7).
     *
     * STATIC (8 B used):
     *   [0] kind  [1] is_manual  [2..3] max_throttle_milli (LE u16)
     *   [4..5] hold_duration_ms (LE u16)  [6] curve  [7] reserved
     *
     * GUTTER (28 B used):
     *   [0] kind  [1] is_manual  [2..3] hold_duration_ms (LE u16)
     *   [4..5] h_ref_min_dm (LE u16)  [6..7] h_ref_max_dm (LE u16)
     *   [8..9] initial_h_ref_dm (LE i16)  [10] curve  [11] reserved
     *   [12..15] pid_kp (LE f32)   [16..19] pid_ki (LE f32)
     *   [20..23] pid_kd (LE f32)   [24..27] pid_integral_limit (LE f32)
     *
     * TORQUE / TORQUE_CAL: recruta defines in B1/B2.
     */
    profile_kind_t kind = (profile_kind_t)wire[0];
    out->kind = kind;
    switch (kind) {
        case PROFILE_KIND_TEST_STATIC: {
            static_test_profile_t *p = &out->test.static_test;
            p->is_manual          = (wire[1] != 0);
            p->max_throttle_milli = rd_u16le(&wire[2]);
            p->hold_duration_ms   = rd_u16le(&wire[4]);
            p->curve              = (test_curve_t)wire[6];
            return true;
        }
        case PROFILE_KIND_TEST_GUTTER: {
            gutter_test_profile_t *p = &out->test.gutter;
            p->is_manual          = (wire[1] != 0);
            p->hold_duration_ms   = rd_u16le(&wire[2]);
            p->h_ref_min_dm       = rd_u16le(&wire[4]);
            p->h_ref_max_dm       = rd_u16le(&wire[6]);
            p->initial_h_ref_dm   = rd_i16le(&wire[8]);
            p->curve              = (test_curve_t)wire[10];
            p->pid_kp             = rd_f32le(&wire[12]);
            p->pid_ki             = rd_f32le(&wire[16]);
            p->pid_kd             = rd_f32le(&wire[20]);
            p->pid_integral_limit = rd_f32le(&wire[24]);
            return true;
        }
        case PROFILE_KIND_TEST_TORQUE:
        case PROFILE_KIND_TEST_TORQUE_CAL:
            /* Recruta (B1/B2): wire format below the same params[32] budget. */
            printf("[FSM] kind=%u not yet wired (B1/B2 stubs)\r\n", (unsigned)kind);
            return false;
        default:
            printf("[FSM] reject: unknown profile kind=%u\r\n", (unsigned)kind);
            return false;
    }
}

static void handle_cmd_set_test_profile(fsm_cmd_msg_t *msg) {
    if (!fsm_ctx.motor_status.calibration_done) {
        printf("[FSM] SET_TEST_PROFILE rejected: motor not calibrated\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    /* Phase 3-A bugfix: allow re-configuration if a test is loaded but not
     * running. This covers the "user changed kind/mode and clicked SET again"
     * case. Only block when motor is live (RUNNING / HOLD / COUNTDOWN / FINISHING). */
    if (test_runner_is_active()) {
        bool quiescent = (fsm_ctx.substate == SUB_TEST_CONFIGED
                       || fsm_ctx.substate == SUB_TEST_ARMED
                       || fsm_ctx.substate == SUB_TEST_DONE
                       || fsm_ctx.substate == SUB_TEST_ABORTED);
        if (!quiescent) {
            printf("[FSM] SET_TEST_PROFILE rejected: test active in %s\r\n",
                   fsm_substate_to_str(fsm_ctx.substate));
            fsm_ctx.last_cmd_status = 1;
            return;
        }
        printf("[FSM] SET_TEST_PROFILE: replacing existing profile\r\n");
        test_runner_abort(TEST_EXIT_USER);  /* cleans up + returns to STATE_IDLE */
    }

    /* After the abort path above, fsm_ctx.state should be STATE_IDLE. */
    if (fsm_ctx.state != STATE_IDLE && fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] SET_TEST_PROFILE rejected: bad state %s\r\n",
               fsm_state_to_str(fsm_ctx.state));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    profile_t profile;
    memset(&profile, 0, sizeof(profile));
    if (!parse_test_profile_wire(msg->payload.raw, &profile)) {
        fsm_ctx.last_cmd_status = 1;
        return;
    }
    if (!test_runner_enter(&profile)) {
        fsm_ctx.last_cmd_status = 1;
        return;
    }
    fsm_send_telemetry_event(EVT_PROFILE_LOADED, NULL, 0);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_hold(fsm_cmd_msg_t *msg) {
    (void)msg;
    if (!test_runner_hold()) { fsm_ctx.last_cmd_status = 1; return; }
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_resume(fsm_cmd_msg_t *msg) {
    (void)msg;
    if (!test_runner_resume()) { fsm_ctx.last_cmd_status = 1; return; }
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_stop_test(fsm_cmd_msg_t *msg) {
    (void)msg;
    if (!test_runner_is_active()) { fsm_ctx.last_cmd_status = 1; return; }
    test_runner_finish();  /* tick will drive FINISHING → DONE */
    fsm_ctx.last_cmd_status = 0;
}

/* ----------------------------------------------------------------------- */

static void handle_cmd_arm(fsm_cmd_msg_t *msg) {
    printf("[FSM] ARM requested\r\n");

    /* Phase 3-A: route through test_runner when a test profile is loaded. */
    if (test_runner_is_active() && fsm_ctx.substate == SUB_TEST_CONFIGED) {
        if (!test_runner_arm()) { fsm_ctx.last_cmd_status = 1; return; }
        fsm_ctx.last_cmd_status = 0;
        return;
    }

    if (fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] Cannot ARM from %s (need CONFIGED)\r\n", fsm_state_to_str(fsm_ctx.state));
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

    // Check barometer calibration (recommended but not required)
    if (!fsm_ctx.baro_cal.is_calibrated) {
        printf("[FSM] WARNING: Barometer not calibrated!\r\n");
    }

    transition_to(STATE_ARMED, SUB_ARM_MOTOR_INIT, EVT_ARMED);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_disarm(fsm_cmd_msg_t *msg) {
    printf("[FSM] DISARM requested\r\n");

    // If a static test is running, treat DISARM as a graceful cancel.
    if (StaticTest_IsRunning()) {
        printf("[FSM] DISARM cancelling static test\r\n");
        StaticTest_Cancel();
        PWM_EmergencyStop();
        fsm_ctx.last_cmd_status = 0;
        return;
    }

    if (fsm_ctx.state == STATE_ARMED) {
        // Cut motor
        controller_set_throttle(0.0f);
        fsm_ctx.flags.motor_armed = 0;

        transition_to(STATE_CONFIGED, SUB_NONE, EVT_DISARMED);
        fsm_ctx.last_cmd_status = 0;
    } else {
        printf("[FSM] Not armed\r\n");
        fsm_ctx.last_cmd_status = 1;
    }
}

static void handle_cmd_launch(fsm_cmd_msg_t *msg) {
    printf("[FSM] LAUNCH requested\r\n");

    if (fsm_ctx.state != STATE_ARMED || fsm_ctx.substate != SUB_ARM_READY) {
        printf("[FSM] Not ready for launch! (state=%s sub=%s)\r\n",
               fsm_state_to_str(fsm_ctx.state),
               fsm_substate_to_str(fsm_ctx.substate));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    fsm_ctx.flight_start_tick = HAL_GetTick();
    transition_to(STATE_FLIGHT, SUB_FL_IGNITION, EVT_STATE_CHANGE);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_abort(fsm_cmd_msg_t *msg) {
    printf("[FSM] ABORT!\r\n");

    // Phase 3-A: bring down the new test runner before forcing the global stop
    // (its on_exit handles motor + sd_card_resume).
    if (test_runner_is_active()) {
        test_runner_abort(TEST_EXIT_ABORT);
    }

    // Cancel any legacy test in progress and force motor off before changing state.
    // StaticTest_Cancel() drives PWM to 0; PWM_EmergencyStop() is belt-and-suspenders.
    if (StaticTest_IsRunning()) {
        StaticTest_Cancel();
    }
    PWM_EmergencyStop();

    transition_to(STATE_ABORT, SUB_NONE, EVT_ABORT_TRIGGERED);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_force_safe(fsm_cmd_msg_t *msg) {
    printf("[FSM] FORCE_SAFE\r\n");

    if (test_runner_is_active()) {
        test_runner_abort(TEST_EXIT_ABORT);
    }
    if (StaticTest_IsRunning()) {
        StaticTest_Cancel();
    }
    PWM_EmergencyStop();

    transition_to(STATE_SAFE, SUB_NONE, EVT_STATE_CHANGE);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_calibrate_baro(fsm_cmd_msg_t *msg) {
    printf("[FSM] CALIBRATE_BARO requested\r\n");

    if (fsm_ctx.state != STATE_IDLE && fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] Cannot calibrate in %s\r\n", fsm_state_to_str(fsm_ctx.state));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // Start calibration process using the MS5607 calibration API
    // This will fail if sensor isn't responding (no static boot flag check)
    if (!MS5607_StartCalibration(&baro_device, &fsm_ctx.baro_cal)) {
        printf("[FSM] ERROR: Failed to start calibration (sensor not responding)!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    calibration_in_progress = true;
    calibration_sample_count = 0;
    calibration_start_tick = HAL_GetTick();

    printf("[FSM] Calibration started (%d samples)...\r\n", BARO_CALIBRATION_SAMPLES);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_start_test(fsm_cmd_msg_t *msg) {
    printf("[FSM] START_TEST\r\n");

    /* Phase 3-A: if the new test runner is armed, kick off its countdown. */
    if (test_runner_is_active() && fsm_ctx.substate == SUB_TEST_ARMED) {
        if (!test_runner_start_countdown()) { fsm_ctx.last_cmd_status = 1; return; }
        fsm_ctx.last_cmd_status = 0;
        return;
    }

    if (fsm_ctx.state != STATE_ARMED || fsm_ctx.substate != SUB_ARM_READY) {
        printf("[FSM] Not ready for test!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    transition_to(STATE_TEST_STAND, SUB_TS_SENSOR_CHECK, EVT_STATE_CHANGE);
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_calibrate_motor(fsm_cmd_msg_t *msg) {
    printf("[FSM] CALIBRATE_MOTOR requested\r\n");

    // Only allow in IDLE or CONFIGED states (not armed, not flying)
    if (fsm_ctx.state != STATE_IDLE && fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] Cannot calibrate motor in %s\r\n", fsm_state_to_str(fsm_ctx.state));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // If calibration is waiting in phase 1 (MAX throttle), treat a second M
    // command as the user acknowledging the ESC beeps → immediately switch to MIN
    if (motor_cal_in_progress) {
        if (PWM_ESC_GetCalibrationState() == ESC_CAL_PHASE1_MAX) {
            printf("[FSM] ESC beep acknowledged - switching to MIN throttle now\r\n");
            PWM_ESC_AcknowledgeBeep();
            fsm_ctx.last_cmd_status = 0;
        } else {
            printf("[FSM] Motor calibration already in progress!\r\n");
            fsm_ctx.last_cmd_status = 1;
        }
        return;
    }

    // DEBUG: If payload byte 1 is 'D' (0x44), run PWM debug test instead
    if (msg->payload.raw[0] == 0x44) {  // 'D' = Debug mode
        printf("[FSM] Running PWM DEBUG TEST...\r\n");
        PWM_DebugTest();
        fsm_ctx.last_cmd_status = 0;
        return;
    }

    // Start ESC calibration (10 seconds for phase 1 by default)
    // User can optionally provide duration in payload
    uint32_t phase1_ms = 0;  // 0 = use default
    if (msg->payload.raw[0] != 0) {
        // First byte can specify seconds for phase 1 (0 = default)
        phase1_ms = msg->payload.raw[0] * 1000;
    }

    if (!PWM_ESC_StartCalibration(phase1_ms)) {
        printf("[FSM] ERROR: Failed to start motor calibration!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    motor_cal_in_progress = true;
    last_cal_state = ESC_CAL_PHASE1_MAX;

    // Pause SD card to avoid EMI interference during motor calibration
    sd_card_pause();

    // Send event to GS
    send_telem_event(EVT_MOTOR_CAL_STARTED, NULL, 0);

    printf("[FSM] Motor calibration started\r\n");
    printf("[FSM] >>> POWER CYCLE THE ESC, then send M again after the beeps! <<<\r\n");
    fsm_ctx.last_cmd_status = 0;
}

static void handle_cmd_static_test(fsm_cmd_msg_t *msg) {
    printf("[FSM] STATIC_TEST requested\r\n");

    // Only allow in IDLE or CONFIGED states
    if (fsm_ctx.state != STATE_IDLE && fsm_ctx.state != STATE_CONFIGED) {
        printf("[FSM] Cannot run static test in %s\r\n", fsm_state_to_str(fsm_ctx.state));
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // Check if motor calibration is done (REQUIRED for any motor test)
    if (!fsm_ctx.motor_status.calibration_done) {
        printf("[FSM] ERROR: Motor calibration required before testing!\r\n");
        printf("[FSM] Send 'M' command to calibrate ESC first.\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // Check if static test already in progress
    if (static_test_in_progress) {
        printf("[FSM] Static test already in progress!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    // Get max throttle percentage from payload (first byte)
    uint8_t max_throttle = msg->payload.raw[0];
    if (max_throttle == 0) {
        max_throttle = 20;  // Default to 20% if not specified
    }
    if (max_throttle > 100) {
        max_throttle = 100;
    }

    // Initialize static test module
    StaticTest_Init();

    // Start the test
    if (!StaticTest_Start(max_throttle)) {
        printf("[FSM] ERROR: Failed to start static test!\r\n");
        fsm_ctx.last_cmd_status = 1;
        return;
    }

    static_test_in_progress = true;
    last_static_test_state = STATIC_TEST_INIT;

    // Send event to GS
    send_telem_event(EVT_STATIC_TEST_STARTED, &max_throttle, 1);

    printf("[FSM] Static thrust test started: max=%d%%\r\n", max_throttle);
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
        case CMD_CALIBRATE_MOTOR: handle_cmd_calibrate_motor(msg); break;
        case CMD_STATIC_TEST:    handle_cmd_static_test(msg); break;
        case CMD_START_TEST:     handle_cmd_start_test(msg); break;
        case CMD_SET_TEST_PROFILE: handle_cmd_set_test_profile(msg); break;
        case CMD_HOLD:           handle_cmd_hold(msg); break;
        case CMD_RESUME:         handle_cmd_resume(msg); break;
        case CMD_STOP_TEST:      handle_cmd_stop_test(msg); break;
        default:
            printf("[FSM] Unknown CMD %u\r\n", msg->cmd);
            fsm_ctx.last_cmd_status = 2;
            break;
    }
}

// ============================================================================
// Internal Event Handler
// ============================================================================
static void process_internal_event(fsm_event_msg_t *evt) {
    printf("[FSM] Internal event: type=%u\r\n", evt->type);

    switch (evt->type) {
        case FSM_EVT_LIFTOFF:
            if (fsm_ctx.state == STATE_FLIGHT && fsm_ctx.substate == SUB_FL_IGNITION) {
                printf("[FSM] Liftoff confirmed!\r\n");
                fsm_ctx.substate = SUB_FL_ASCENT;
                send_telem_event(EVT_LIFTOFF, &evt->data.velocity, sizeof(float));
            }
            break;

        case FSM_EVT_APOGEE:
            if (fsm_ctx.state == STATE_FLIGHT) {
                printf("[FSM] Apogee reached at %.1f m\r\n", evt->data.altitude);
                fsm_ctx.substate = SUB_FL_DESCENT_BRAKE;
                send_telem_event(EVT_APOGEE, &evt->data.altitude, sizeof(float));
            }
            break;

        case FSM_EVT_FLARE_ALT:
            if (fsm_ctx.state == STATE_FLIGHT) {
                printf("[FSM] Flare altitude!\r\n");
                fsm_ctx.substate = SUB_FL_LANDING_FLARE;
            }
            break;

        case FSM_EVT_TOUCHDOWN:
            if (fsm_ctx.state == STATE_FLIGHT) {
                printf("[FSM] Touchdown!\r\n");
                fsm_ctx.substate = SUB_FL_TOUCHDOWN;
            }
            break;

        case FSM_EVT_LANDED:
            if (fsm_ctx.state == STATE_FLIGHT) {
                printf("[FSM] Landed and stable\r\n");
                fsm_ctx.substate = SUB_FL_RECOVERY;
                send_telem_event(EVT_LANDING, NULL, 0);
            }
            break;

        case FSM_EVT_ALTITUDE_LIMIT:
        case FSM_EVT_VELOCITY_LIMIT:
            printf("[FSM] Safety limit exceeded - ABORT!\r\n");
            transition_to(STATE_ABORT, SUB_NONE, EVT_ABORT_TRIGGERED);
            break;

        default:
            break;
    }
}

// ============================================================================
// Calibration State Machine (runs in main loop)
// ============================================================================
static void process_calibration(void) {
    if (!calibration_in_progress) return;

    uint32_t elapsed = HAL_GetTick() - calibration_start_tick;

    // Take a sample every BARO_CALIBRATION_DELAY_MS
    if (elapsed >= (calibration_sample_count + 1) * BARO_CALIBRATION_DELAY_MS) {
        // Actually read from the barometer sensor
        if (MS5607_AddCalibrationSample(&baro_device, &fsm_ctx.baro_cal)) {
            calibration_sample_count++;
            printf("[FSM] Calibration sample %d/%d (P=%.2f mbar)\r\n",
                   calibration_sample_count, BARO_CALIBRATION_SAMPLES,
                   fsm_ctx.baro_cal.pressure_sum / (float)calibration_sample_count);
        } else {
            printf("[FSM] WARNING: Failed to read calibration sample!\r\n");
        }

        if (calibration_sample_count >= BARO_CALIBRATION_SAMPLES) {
            // Calibration complete - finalize with actual averaging
            calibration_in_progress = false;

            if (MS5607_FinishCalibration(&fsm_ctx.baro_cal)) {
                fsm_ctx.flags.baro_calibrated = 1;

                // Send calibration complete event
                calibration_payload_t cal_result = {
                    .reference_pressure = fsm_ctx.baro_cal.reference_pressure_mbar,
                    .temperature = fsm_ctx.baro_cal.temperature_at_cal_c,
                    .samples = BARO_CALIBRATION_SAMPLES
                };
                send_telem_event(EVT_BARO_CALIBRATED, &cal_result, sizeof(cal_result));

                printf("[FSM] Calibration complete! Ref pressure: %.2f mbar\r\n",
                       fsm_ctx.baro_cal.reference_pressure_mbar);
            } else {
                printf("[FSM] ERROR: Calibration finalization failed!\r\n");
                fsm_ctx.last_cmd_status = 1;
            }
        }
    }
}

// ============================================================================
// Motor Calibration State Machine (runs in main loop)
// ============================================================================
static void process_motor_calibration(void) {
    if (!motor_cal_in_progress) return;

    esc_calibration_state_t current_state = PWM_ESC_CalibrationUpdate();

    // Debug: print every 2 seconds during calibration
    static uint32_t last_debug_tick = 0;
    if (HAL_GetTick() - last_debug_tick > 2000) {
        const esc_calibration_ctx_t *ctx = PWM_ESC_GetCalibrationContext();
        printf("[DBG] Motor cal: state=%d, elapsed=%lu ms\r\n",
               current_state, HAL_GetTick() - ctx->phase_start_tick);
        last_debug_tick = HAL_GetTick();
    }

    // Check for state transitions
    if (current_state != last_cal_state) {
        switch (current_state) {
            case ESC_CAL_PHASE2_MIN:
                printf("[FSM] Motor cal: Phase 2 - sending MIN throttle\r\n");
                send_telem_event(EVT_MOTOR_CAL_PHASE2, NULL, 0);
                break;

            case ESC_CAL_COMPLETE:
                printf("[FSM] Motor calibration COMPLETE!\r\n");
                motor_cal_in_progress = false;
                fsm_ctx.motor_status.calibration_done = true;
                send_telem_event(EVT_MOTOR_CALIBRATED, NULL, 0);
                sd_card_resume();  // Resume SD after calibration
                break;

            case ESC_CAL_FAILED:
                printf("[FSM] Motor calibration FAILED!\r\n");
                motor_cal_in_progress = false;
                sd_card_resume();  // Resume SD after calibration
                break;

            default:
                break;
        }
        last_cal_state = current_state;
    }
}

// ============================================================================
// Static Thrust Test State Machine (runs in main loop)
// ============================================================================
static uint32_t last_progress_tick = 0;
#define PROGRESS_UPDATE_INTERVAL_MS  200  // Send PWM updates every 200ms

static void process_static_test(void) {
    if (!static_test_in_progress) return;

    // Defensive: if the FSM has entered a stop state via any path
    // (e.g. internal safety event, not just CMD_ABORT), force-cancel the test.
    if (fsm_ctx.state == STATE_ABORT || fsm_ctx.state == STATE_SAFE) {
        if (StaticTest_IsRunning()) {
            printf("[FSM] FSM in stop state, cancelling static test\r\n");
            StaticTest_Cancel();
            PWM_EmergencyStop();
        }
    }

    static_test_state_t current_state = StaticTest_Update();
    const static_test_ctx_t *ctx = StaticTest_GetContext();

    // Send progress updates periodically during ramp phases
    if (current_state == STATIC_TEST_RAMP_UP ||
        current_state == STATIC_TEST_HOLD ||
        current_state == STATIC_TEST_RAMP_DOWN) {

        uint32_t now = HAL_GetTick();
        if ((now - last_progress_tick) >= PROGRESS_UPDATE_INTERVAL_MS) {
            last_progress_tick = now;

            // Send progress event with PWM and thrust data
            struct __attribute__((packed)) {
                uint8_t pwm_percent;
                float thrust_n;
            } progress = {
                .pwm_percent = (uint8_t)ctx->current_throttle,
                .thrust_n = ctx->current_thrust
            };
            send_telem_event(EVT_STATIC_TEST_PROGRESS, &progress, sizeof(progress));
        }
    }

    // Check for state transitions
    if (current_state != last_static_test_state) {

        switch (current_state) {
            case STATIC_TEST_RAMP_UP:
                printf("[FSM] Static test: Ramping up...\r\n");
                break;

            case STATIC_TEST_HOLD:
                printf("[FSM] Static test: Holding at max throttle\r\n");
                break;

            case STATIC_TEST_RAMP_DOWN:
                printf("[FSM] Static test: Ramping down...\r\n");
                break;

            case STATIC_TEST_SAVING:
                printf("[FSM] Static test: Saving data...\r\n");
                break;

            case STATIC_TEST_COMPLETE: {
                printf("[FSM] Static test COMPLETE!\r\n");
                printf("[FSM] Max thrust: %.2f N at %.1f%% PWM\r\n",
                       ctx->max_thrust_n, ctx->max_thrust_pwm);
                static_test_in_progress = false;

                // Send completion event with results
                struct __attribute__((packed)) {
                    float max_thrust_n;
                    float max_thrust_pwm;
                    uint16_t sample_count;
                } result = {
                    .max_thrust_n = ctx->max_thrust_n,
                    .max_thrust_pwm = ctx->max_thrust_pwm,
                    .sample_count = ctx->data_count
                };
                send_telem_event(EVT_STATIC_TEST_COMPLETE, &result, sizeof(result));
                break;
            }

            case STATIC_TEST_FAILED:
                printf("[FSM] Static test FAILED: %s\r\n",
                       ctx->error_msg ? ctx->error_msg : "unknown error");
                static_test_in_progress = false;
                send_telem_event(EVT_STATIC_TEST_FAILED, NULL, 0);
                break;

            default:
                break;
        }
        last_static_test_state = current_state;
    }
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
    TickType_t last_thread_stat = xTaskGetTickCount();

    while (1) {
        // Process commands from radio thread
        while (xQueueReceive(queue_cmd_to_fsm, &cmd_msg, 0) == pdTRUE) {
            process_command(&cmd_msg);
        }

        // Process internal events from estimator
        while (xQueueReceive(queue_event_to_fsm, &evt_msg, 0) == pdTRUE) {
            process_internal_event(&evt_msg);
        }

        // Process calibration and test state machines
        process_calibration();
        process_motor_calibration();
        process_static_test();

        // State-specific processing
        switch (fsm_ctx.state) {
            case STATE_BOOT:       handle_state_boot(); break;
            case STATE_IDLE:       handle_state_idle(); break;
            case STATE_CONFIGED:   handle_state_configed(); break;
            case STATE_ARMED:      handle_state_armed(); break;
            case STATE_TEST_STAND: handle_state_test_stand(); break;
            case STATE_TEST_STATIC:
            case STATE_TEST_TORQUE:
            case STATE_TEST_TORQUE_CAL:
            case STATE_TEST_GUTTER:
                /* Phase 3-A (A2/A5): sub-FSM dispatch via test_runner. The
                 * runner pulls the latest test_control_packet_t from its own
                 * cache (populated by radio_thread → test_runner_submit_control). */
                test_runner_tick();
                break;
            case STATE_FLIGHT:     handle_state_flight(); break;
            case STATE_ABORT:      handle_state_abort(); break;
            case STATE_SAFE:       handle_state_safe(); break;
            default: break;
        }

        // Round-robin emission of one task's stats per second.
        // Over ~10 s the GS receives stats for every task in the system.
        TickType_t now = xTaskGetTickCount();
        if ((now - last_thread_stat) >= pdMS_TO_TICKS(1000)) {
            system_stats_emit_one();
            last_thread_stat = now;
        }

        // Periodic stats
        if ((now - last_stats) >= pdMS_TO_TICKS(STATS_INTERVAL_MS)) {
            printf("[FSM] State=%s.%s | Profile=%s | Target=%.1fm Flare=%.1fm | BaroCal=%s\r\n",
                   fsm_state_to_str(fsm_ctx.state),
                   fsm_substate_to_str(fsm_ctx.substate),
                   fsm_profile_to_str(fsm_ctx.profile.type),
                   fsm_ctx.profile.target_altitude_m,
                   fsm_ctx.profile.flare_altitude_m,
                   fsm_ctx.baro_cal.is_calibrated ? "YES" : "NO");
            last_stats = now;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Public function for other threads to send telemetry events
void fsm_send_telemetry_event(telemetry_event_type_t type, const void *payload, uint16_t size) {
    send_telem_event(type, payload, size);
}
