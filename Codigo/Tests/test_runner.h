/**
 * @file test_runner.h
 * @brief Per-test ops vtable contract (Phase 2 contract)
 * @author Tomás Teixeira
 * @date 2026
 *
 * @details
 * Defines the small "vtable" each test kind (STATIC / TORQUE / TORQUE_CAL /
 * GUTTER) must implement. The sub-FSM dispatches to whichever ops struct
 * matches `fsm_ctx.profile.kind` — adding a new test kind is "drop a new
 * .c file with the ops struct + register it in test_kind_table[]" and
 * nothing else.
 *
 * ## Lifecycle each test goes through (sub-FSM, defined in Phase 3-A)
 *
 *     CONFIGED ──► ARMED ──► COUNTDOWN ──► RUNNING ──► FINISHING ──► DONE
 *                                              │
 *                                              ├──► HOLD (manual only, resumable)
 *                                              │
 *                                              └──► ABORTED (any failure / abort)
 *
 * ## When each callback fires
 *
 * | Callback        | Sub-state transition          | Purpose                                           |
 * |-----------------|-------------------------------|---------------------------------------------------|
 * | on_configed     | (any) → CONFIGED              | Validate profile, init driver-level hardware      |
 * | on_arm          | CONFIGED → ARMED              | Tare load cell, ESC arm, capture baseline         |
 * | on_countdown    | ARMED → COUNTDOWN             | Optional banner / safety announcement             |
 * | on_run_tick     | RUNNING (every TDMA tick)     | Drive actuators, sample sensors, emit telemetry   |
 * | on_hold         | RUNNING → HOLD (manual)       | Smoothly halt motor while keeping test "live"     |
 * | on_resume       | HOLD → RUNNING                | Re-engage from a HOLD                             |
 * | on_finishing    | RUNNING → FINISHING           | Decelerate, flush sample buffer                   |
 * | on_exit         | * → DONE / ABORTED            | Cleanup: motor off, SD resume, free resources     |
 *
 * Any callback may be NULL — the runner must treat NULL as "no-op".
 *
 * @ingroup Tests
 */

#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include <stdbool.h>
#include <stdint.h>
#include "Flight Computer/test_profile.h"
#include "Telemetry/telemetry.h"  /* test_control_packet_t */

/**
 * @brief Why a test exited — passed into on_exit() so cleanup can
 * differentiate normal completion from a forced stop.
 */
typedef enum {
    TEST_EXIT_DONE     = 0,   /**< Auto profile reached its planned end */
    TEST_EXIT_USER     = 1,   /**< Manual stop, no error */
    TEST_EXIT_ABORT    = 2,   /**< CMD_ABORT or internal safety event */
    TEST_EXIT_HEARTBEAT = 3,  /**< GS link dead longer than the heartbeat timeout */
    TEST_EXIT_FAILURE  = 4,   /**< Hardware fault during the test */
} test_exit_reason_t;

/**
 * @brief Per-test callback table.
 *
 * Each test kind exports one of these as a `const` global (e.g.
 * `extern const test_kind_ops_t static_test_ops;`). The runner picks the
 * right one via `test_kind_table[fsm_ctx.profile.kind]`.
 */
typedef struct {
    /** Human-readable name used only for logging. */
    const char *name;

    /**
     * @brief Validate the profile and prepare per-test hardware.
     *
     * Called once on entry to STATE_TEST_<KIND>_CONFIGED. Returning false
     * blocks the transition and the FSM rejects the SET_PROFILE command
     * with `last_cmd_status != 0`.
     *
     * @param  profile The profile from the SET_PROFILE command.
     * @retval true    Profile valid, hardware reachable, safe to arm.
     * @retval false   Refuse to enter the test.
     */
    bool (*on_configed)(const profile_t *profile);

    /**
     * @brief Arm the test. Tare load cell, send ESC arm pulse, etc.
     *
     * Called on entry to TEST_ARMED. If this fails, on_exit(ABORT) is
     * called and the FSM goes to STATE_ABORT.
     *
     * @retval true   Armed and ready.
     * @retval false  Arming failed.
     */
    bool (*on_arm)(void);

    /**
     * @brief Optional pre-run banner / staging. May be NULL.
     *
     * Called once on entry to TEST_COUNTDOWN (the FSM emits the
     * EVT_TEST_COUNTDOWN events itself).
     */
    void (*on_countdown)(void);

    /**
     * @brief Per-tick driver of the test while in TEST_RUNNING.
     *
     * Called every TDMA tick (~100 ms in flight mode, ~40 ms in
     * test-interactive mode). Reads the latest GS control packet (NULL
     * if heartbeat is stale), drives the actuators, samples sensors,
     * pushes telemetry events.
     *
     * @param ctrl  Latest GS test_control_packet_t, or NULL if no fresh one.
     */
    void (*on_run_tick)(const test_control_packet_t *ctrl);

    /**
     * @brief Manual mode: graceful pause. Motor to 0, no exit. May be NULL
     * for tests that don't support pause (auto-only tests).
     */
    void (*on_hold)(void);

    /**
     * @brief Manual mode: resume after hold. May be NULL.
     */
    void (*on_resume)(void);

    /**
     * @brief Final flush before transitioning out of RUNNING.
     *
     * Called on entry to TEST_FINISHING. Decelerate motor, push any
     * remaining samples to telemetry. May be NULL.
     */
    void (*on_finishing)(void);

    /**
     * @brief Cleanup. Always called on exit, regardless of reason.
     *
     * Must be idempotent and reentrant — may be invoked from the abort
     * path with the test in an arbitrary state.
     *
     * Concrete duties:
     *  - PWM_EmergencyStop() (or controller throttle 0 if active)
     *  - sd_card_resume()
     *  - Release any per-test driver state
     */
    void (*on_exit)(test_exit_reason_t reason);
} test_kind_ops_t;

/**
 * @brief Resolve a profile kind to its ops struct. Returns NULL if the
 * kind is not a test (e.g. PROFILE_KIND_FLIGHT) or has no implementation.
 *
 * Defined in Phase 3-A — for now declared so the recruit can reference
 * `extern const test_kind_ops_t static_test_ops` etc. without needing
 * the table layout finalized.
 */
const test_kind_ops_t *test_runner_get_ops(profile_kind_t kind);

/**
 * @defgroup TestRunnerLifecycle Sub-FSM lifecycle API (Phase 3-A / A2)
 * @brief Drives the SUB_TEST_<...> sub-state machine.
 *
 * Each function below maps 1:1 to a sub-state transition. The FSM thread
 * (and only the FSM thread) calls these — typically from `handle_cmd_*`
 * (Phase 3-A / A3) and from the per-tick `test_runner_tick()`.
 *
 * Invalid transitions are rejected (return false / no-op). Aborts are
 * always accepted.
 * @{
 */

/**
 * @brief Validate `profile`, run on_configed, transition to SUB_TEST_CONFIGED.
 * Also sets `fsm_ctx.state` to STATE_TEST_<KIND> matching `profile->kind`.
 * @return false if kind not implemented or on_configed rejected.
 */
bool test_runner_enter(const profile_t *profile);

/** @brief SUB_TEST_CONFIGED → SUB_TEST_ARMED via on_arm. */
bool test_runner_arm(void);

/** @brief SUB_TEST_ARMED → SUB_TEST_COUNTDOWN. Tick drives countdown to RUNNING. */
bool test_runner_start_countdown(void);

/** @brief Manual only: SUB_TEST_RUNNING → SUB_TEST_HOLD via on_hold. */
bool test_runner_hold(void);

/** @brief SUB_TEST_HOLD → SUB_TEST_RUNNING via on_resume. */
bool test_runner_resume(void);

/**
 * @brief Request graceful end. Tick will fire on_finishing → SUB_TEST_FINISHING
 * → on_exit(DONE) → SUB_TEST_DONE. Used by CMD_STOP_TEST in manual mode and
 * implicitly by the auto-mode timer.
 */
void test_runner_finish(void);

/**
 * @brief Force-abort from any sub-state. Calls on_exit(reason), transitions
 * to SUB_TEST_ABORTED. Idempotent.
 */
void test_runner_abort(test_exit_reason_t reason);

/**
 * @brief Per-tick driver. Call every FSM tick while
 * `fsm_ctx.state` is one of STATE_TEST_<KIND>.
 *
 * Reads the latest control packet from the internal cache (populated by
 * test_runner_submit_control()), dispatches on_run_tick(), drives the
 * countdown timer, triggers auto-mode hold_duration finishing, and
 * enforces the heartbeat watchdog in manual mode.
 */
void test_runner_tick(void);

/**
 * @brief Phase 3-A (A5): radio_thread RX hook for inbound test_control_packet_t.
 *
 * CRC is already validated by the caller. This stores a copy of the packet
 * along with the receive timestamp; the next test_runner_tick() consumes it.
 * Safe to call from the radio thread context.
 */
void test_runner_submit_control(const test_control_packet_t *ctrl);

/** @brief True iff a test is currently active (CONFIGED..FINISHING). */
bool test_runner_is_active(void);

/** @} */

/* Per-test ops globals — to be defined by each test driver file in
 * Phase 3-A (eu) and Phase 3-B (recruta). Declared here so the table
 * resolver can name them. */
extern const test_kind_ops_t static_test_ops;     /**< Tu (A1) */
extern const test_kind_ops_t torque_test_ops;     /**< Recruta (B1) */
extern const test_kind_ops_t torque_cal_test_ops; /**< Recruta (B2) */
extern const test_kind_ops_t gutter_test_ops;     /**< Tu (A7) */

#endif /* TEST_RUNNER_H */
