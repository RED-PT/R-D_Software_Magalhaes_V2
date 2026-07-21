/**
 * @file test_runner.c
 * @brief Sub-FSM dispatcher for STATE_TEST_<KIND> (Phase 3-A / A2)
 * @author Tomás Teixeira
 * @date 2026
 *
 * @details
 * Owns the SUB_TEST_<...> lifecycle state machine, the kind→ops lookup
 * table, and the cached active profile. The main FSM thread calls
 * `test_runner_*` lifecycle functions from `handle_cmd_*` (A3) and
 * `test_runner_tick()` once per FSM iteration while in any STATE_TEST_*.
 *
 * Sub-state graph (mirrors the table in test_runner.h):
 *
 *     CONFIGED ──► ARMED ──► COUNTDOWN ──► RUNNING ──► FINISHING ──► DONE
 *                                              │           ▲
 *                                              ├──► HOLD ──┘ (manual: resume)
 *                                              │
 *                                              └──► (any) ──► ABORTED
 *
 * @ingroup Tests
 */

#include "test_runner.h"
#include "Flight Computer/flight_computer.h"
#include "Flight Computer/flight_computer_thread.h"
#include "Flight Computer/test_profile.h"
#include "Telemetry/telemetry.h"
#include "Radio/radio_thread.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define COUNTDOWN_DURATION_MS  3000
/* Hold the last commanded setpoint between GS packets up to this age. Anything
 * older falls through to on_run_tick with ctrl=NULL (which the ops module
 * interprets as "coast to 0"). 1500 ms covers the 1 Hz keepalive with margin;
 * absolute upper bound is TEST_HEARTBEAT_TIMEOUT_MS (after which we abort). */
#define CTRL_FRESH_THRESHOLD_MS 3000

/* ------------------------------------------------------------------------- */
/*  Module state                                                             */
/* ------------------------------------------------------------------------- */

static profile_t              cached_profile;
static const test_kind_ops_t *current_ops    = NULL;
static uint32_t               run_start_tick = 0;
static bool                   finish_requested = false;

/* Phase 3-A (A5): latest control packet cache. radio_thread writes via
 * test_runner_submit_control(); the FSM thread reads in test_runner_tick(). */
static test_control_packet_t  latest_ctrl;
static volatile uint32_t      latest_ctrl_tick = 0;
static volatile bool          ctrl_ever_received = false;

/* ------------------------------------------------------------------------- */
/*  Vtable lookup                                                            */
/* ------------------------------------------------------------------------- */

const test_kind_ops_t *test_runner_get_ops(profile_kind_t kind) {
    switch (kind) {
        case PROFILE_KIND_TEST_STATIC:     return &static_test_ops;
        /* Recruta (B1/B2) and tu (A7) plug their ops in here. */
        case PROFILE_KIND_TEST_TORQUE:     return NULL;  /* TODO B1 */
        case PROFILE_KIND_TEST_TORQUE_CAL: return NULL;  /* TODO B2 */
        case PROFILE_KIND_TEST_GUTTER:     return &gutter_test_ops;
        default:                           return NULL;
    }
}

static fsm_state_t kind_to_state(profile_kind_t kind) {
    switch (kind) {
        case PROFILE_KIND_TEST_STATIC:     return STATE_TEST_STATIC;
        case PROFILE_KIND_TEST_TORQUE:     return STATE_TEST_TORQUE;
        case PROFILE_KIND_TEST_TORQUE_CAL: return STATE_TEST_TORQUE_CAL;
        case PROFILE_KIND_TEST_GUTTER:     return STATE_TEST_GUTTER;
        default:                           return STATE_IDLE;
    }
}

static bool kind_is_manual(profile_kind_t kind) {
    switch (kind) {
        case PROFILE_KIND_TEST_STATIC:     return cached_profile.test.static_test.is_manual;
        case PROFILE_KIND_TEST_TORQUE:     return cached_profile.test.torque_test.is_manual;
        case PROFILE_KIND_TEST_TORQUE_CAL: return cached_profile.test.torque_cal.is_manual;
        case PROFILE_KIND_TEST_GUTTER:     return cached_profile.test.gutter.is_manual;
        default:                           return false;
    }
}

static uint16_t kind_hold_duration_ms(profile_kind_t kind) {
    switch (kind) {
        case PROFILE_KIND_TEST_STATIC:     return cached_profile.test.static_test.hold_duration_ms;
        case PROFILE_KIND_TEST_TORQUE:     return cached_profile.test.torque_test.hold_duration_ms;
        case PROFILE_KIND_TEST_TORQUE_CAL: return cached_profile.test.torque_cal.hold_duration_ms;
        case PROFILE_KIND_TEST_GUTTER:     return cached_profile.test.gutter.hold_duration_ms;
        default:                           return 0;
    }
}

/* ------------------------------------------------------------------------- */
/*  Sub-state transition helper                                              */
/* ------------------------------------------------------------------------- */

static void runner_transition(fsm_state_t st, fsm_substate_t sub) {
    fsm_state_t    old_state = fsm_ctx.state;
    fsm_substate_t old_sub   = fsm_ctx.substate;

    if (old_state == st && old_sub == sub) return;

    fsm_ctx.prev_state       = old_state;
    fsm_ctx.prev_substate    = old_sub;
    fsm_ctx.state            = st;
    fsm_ctx.substate         = sub;
    fsm_ctx.state_entry_tick = HAL_GetTick();

    event_state_change_t change = {
        .old_state    = (uint8_t)old_state,
        .old_substate = (uint8_t)old_sub,
        .new_state    = (uint8_t)st,
        .new_substate = (uint8_t)sub,
        .reason       = 0,
    };
    fsm_send_telemetry_event(EVT_STATE_CHANGE, &change, sizeof(change));

    printf("[RUNNER] %s.%s -> %s.%s\r\n",
           fsm_state_to_str(old_state), fsm_substate_to_str(old_sub),
           fsm_state_to_str(st),        fsm_substate_to_str(sub));
}

/* ------------------------------------------------------------------------- */
/*  Lifecycle API                                                            */
/* ------------------------------------------------------------------------- */

bool test_runner_enter(const profile_t *profile) {
    if (!profile) return false;

    const test_kind_ops_t *ops = test_runner_get_ops(profile->kind);
    if (!ops) {
        printf("[RUNNER] enter: kind=%u has no ops\r\n", (unsigned)profile->kind);
        return false;
    }
    if (ops->on_configed && !ops->on_configed(profile)) {
        printf("[RUNNER] enter: %s on_configed rejected\r\n", ops->name);
        return false;
    }

    cached_profile   = *profile;
    current_ops      = ops;
    finish_requested = false;
    run_start_tick   = 0;

    /* Phase 3-A (A4): switch to test-interactive TDMA so the GS can drive
     * manual sliders at 10 Hz. Effective at next superframe boundary. */
    radio_request_tdma_mode(TDMA_MODE_TEST_INTERACTIVE);

    runner_transition(kind_to_state(profile->kind), SUB_TEST_CONFIGED);
    return true;
}

bool test_runner_arm(void) {
    if (!current_ops) return false;
    if (fsm_ctx.substate != SUB_TEST_CONFIGED) {
        printf("[RUNNER] arm: bad substate %s\r\n", fsm_substate_to_str(fsm_ctx.substate));
        return false;
    }
    if (current_ops->on_arm && !current_ops->on_arm()) {
        printf("[RUNNER] arm: on_arm failed -> ABORT\r\n");
        test_runner_abort(TEST_EXIT_ABORT);
        return false;
    }
    runner_transition(fsm_ctx.state, SUB_TEST_ARMED);
    return true;
}

bool test_runner_start_countdown(void) {
    if (!current_ops || fsm_ctx.substate != SUB_TEST_ARMED) {
        printf("[RUNNER] countdown: bad substate\r\n");
        return false;
    }
    if (current_ops->on_countdown) current_ops->on_countdown();
    runner_transition(fsm_ctx.state, SUB_TEST_COUNTDOWN);
    return true;
}

bool test_runner_hold(void) {
    if (!current_ops || fsm_ctx.substate != SUB_TEST_RUNNING) return false;
    if (!kind_is_manual(cached_profile.kind)) {
        printf("[RUNNER] hold rejected: auto mode\r\n");
        return false;
    }
    if (current_ops->on_hold) current_ops->on_hold();
    runner_transition(fsm_ctx.state, SUB_TEST_HOLD);
    return true;
}

bool test_runner_resume(void) {
    if (!current_ops || fsm_ctx.substate != SUB_TEST_HOLD) return false;
    if (current_ops->on_resume) current_ops->on_resume();
    runner_transition(fsm_ctx.state, SUB_TEST_RUNNING);
    return true;
}

void test_runner_finish(void) {
    if (!current_ops) return;
    /* Defer until next tick — we want to fire on_finishing inside the same
     * code path that drives RUNNING, not from an arbitrary command handler. */
    finish_requested = true;
}

void test_runner_abort(test_exit_reason_t reason) {
    if (!current_ops) return;
    if (current_ops->on_exit) current_ops->on_exit(reason);
    runner_transition(fsm_ctx.state, SUB_TEST_ABORTED);
    current_ops      = NULL;
    finish_requested = false;
    /* Phase 3-A (A4): hand TDMA back to flight preset. */
    radio_request_tdma_mode(TDMA_MODE_FLIGHT);
    /* Bugfix: return to IDLE so the next CMD_SET_TEST_PROFILE can be accepted.
     * Without this the FSM stayed at STATE_TEST_<KIND>.SUB_TEST_ABORTED and
     * `handle_cmd_set_test_profile` rejected on the state gate. */
    runner_transition(STATE_IDLE, SUB_NONE);
}

bool test_runner_is_active(void) {
    return current_ops != NULL;
}

void test_runner_submit_control(const test_control_packet_t *ctrl) {
    if (!ctrl) return;
    /* Plain memcpy — radio thread is the sole writer; FSM thread reads in
     * test_runner_tick. The volatile tick stamp is the publish marker. */
    memcpy(&latest_ctrl, ctrl, sizeof(latest_ctrl));
    latest_ctrl_tick   = HAL_GetTick();
    ctrl_ever_received = true;
}

/* ------------------------------------------------------------------------- */
/*  Per-tick driver                                                          */
/* ------------------------------------------------------------------------- */

void test_runner_tick(void) {
    if (!current_ops) return;

    const uint32_t now = HAL_GetTick();

    switch (fsm_ctx.substate) {
        case SUB_TEST_COUNTDOWN: {
            uint32_t elapsed = now - fsm_ctx.state_entry_tick;

            /* Phase 3-A (A6): emit one EVT_TEST_COUNTDOWN per integer second
             * (3, 2, 1, 0). Payload = seconds remaining as a single byte. */
            static uint8_t last_beacon_value = 0xFF;
            uint32_t remaining_ms = (elapsed >= COUNTDOWN_DURATION_MS)
                                  ? 0 : (COUNTDOWN_DURATION_MS - elapsed);
            uint8_t  remaining_s  = (uint8_t)((remaining_ms + 999) / 1000); /* ceil */
            if (remaining_s != last_beacon_value) {
                last_beacon_value = remaining_s;
                fsm_send_telemetry_event(EVT_TEST_COUNTDOWN, &remaining_s, 1);
                printf("[RUNNER] T-%u\r\n", (unsigned)remaining_s);
            }

            if (elapsed >= COUNTDOWN_DURATION_MS) {
                last_beacon_value = 0xFF;  /* reset for next test */
                run_start_tick = now;
                /* Seed heartbeat clock so we don't trip on the first tick after
                 * countdown if a ctrl hasn't arrived yet — GS gets a full window. */
                latest_ctrl_tick = now;
                runner_transition(fsm_ctx.state, SUB_TEST_RUNNING);
            }
            break;
        }

        case SUB_TEST_RUNNING: {
            /* Phase 3-A (A5): pull cached ctrl, age-check, enforce heartbeat. */
            const test_control_packet_t *ctrl = NULL;
            uint32_t age = now - latest_ctrl_tick;
            bool is_manual = kind_is_manual(cached_profile.kind);

            if (is_manual && age >= TEST_HEARTBEAT_TIMEOUT_MS) {
                printf("[RUNNER] heartbeat timeout (age=%lums) -> ABORT\r\n",
                       (unsigned long)age);
                test_runner_abort(TEST_EXIT_HEARTBEAT);
                break;
            }

            if (ctrl_ever_received && age <= CTRL_FRESH_THRESHOLD_MS) {
                ctrl = &latest_ctrl;
                /* Honour explicit ABORT bit from the GS slider UI. */
                if (ctrl->flags & TEST_CTRL_FLAG_ABORT) {
                    printf("[RUNNER] ctrl flag ABORT received\r\n");
                    test_runner_abort(TEST_EXIT_USER);
                    break;
                }
            }

            if (current_ops->on_run_tick) current_ops->on_run_tick(ctrl);
            /* on_run_tick may itself have called test_runner_abort (e.g.
             * gutter hard altitude trip). If so, current_ops is NULL — bail
             * before any further dereference in this tick. */
            if (!current_ops) break;

            if (finish_requested) {
                finish_requested = false;
                if (current_ops->on_finishing) current_ops->on_finishing();
                runner_transition(fsm_ctx.state, SUB_TEST_FINISHING);
                break;
            }

            /* Auto mode: hold_duration_ms acts as the run budget. */
            if (!kind_is_manual(cached_profile.kind)) {
                uint16_t hold = kind_hold_duration_ms(cached_profile.kind);
                if (hold > 0 && (now - run_start_tick) >= (uint32_t)hold) {
                    if (current_ops->on_finishing) current_ops->on_finishing();
                    runner_transition(fsm_ctx.state, SUB_TEST_FINISHING);
                }
            }
            break;
        }

        case SUB_TEST_FINISHING: {
            if (current_ops->on_exit) current_ops->on_exit(TEST_EXIT_DONE);
            runner_transition(fsm_ctx.state, SUB_TEST_DONE);
            current_ops = NULL;
            /* Phase 3-A (A4): graceful end → back to flight TDMA. */
            radio_request_tdma_mode(TDMA_MODE_FLIGHT);
            /* Bugfix: return to IDLE for the next test (mirror abort path). */
            runner_transition(STATE_IDLE, SUB_NONE);
            break;
        }

        /* Quiescent sub-states — wait for an external command to advance. */
        case SUB_TEST_CONFIGED:
        case SUB_TEST_ARMED:
        case SUB_TEST_HOLD:
        case SUB_TEST_DONE:
        case SUB_TEST_ABORTED:
        default:
            break;
    }
}
