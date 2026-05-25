/**
 * @file gutter_test_ops.c
 * @brief GUTTER test driver — closed-loop altitude tracking on a tethered rail
 * @author Tomás Teixeira
 * @date 2026
 *
 * @details
 * Phase 3-A (A7). Differs from the static/torque tests in three ways:
 *   1. Closed-loop: a baro-driven altitude PID generates the throttle
 *   2. Riskier: requires baro init + estimator alive before arming, plus a
 *      hard altitude trip that aborts the test if h goes h_ref_max + margin
 *   3. Profile carries the PID gains — they are applied per-test
 *
 * Implementation choice: this module owns its own PID state (mirroring how
 * static_test_ops owns its sample buffer). It does NOT route through
 * controller_thread.c, because that thread is geared toward flight and its
 * gain table is hard-coded. Keeping the test self-contained means we can
 * validate gutter without touching flight-mode behaviour.
 *
 * Recruta: this is the third reference impl. The shape is the same — cached
 * profile + small state struct + 8 callbacks driven by on_run_tick. The PID
 * loop in on_run_tick is the one piece that's gutter-specific.
 *
 * @ingroup Tests
 */

#include "test_runner.h"
#include "Flight Computer/test_profile.h"
#include "Flight Computer/flight_computer.h"
#include "Flight Computer/flight_computer_thread.h"
#include "Atuadores/ESC/PWM_FUNCTIONS.h"
#include "Storage/sd_card_thread.h"
#include "Telemetry/telemetry.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Tunable safety constants — kept here, not in profile, because they're not
 * mission-tunable; they are guard-rails. */
#define HOVER_FF             0.50f   /* Feed-forward hover throttle (TWR-dependent; tune per airframe) */
#define MAX_THROTTLE_FRAC    0.85f   /* Hard cap on PID output (0..1) */
#define ALT_HARD_OVER_M      2.0f    /* h > h_ref_max + this → ABORT */
#define DT_DEFAULT_S         0.05f
#define DT_MAX_S             0.50f   /* Clamp pathological dt (e.g. after HOLD) */
#define LOG_INTERVAL_MS      500

typedef struct {
    gutter_test_profile_t profile;
    bool                  armed;

    /* PID state */
    float    integral;
    float    prev_error;
    uint32_t last_tick;

    /* Telemetry pacing */
    uint32_t run_start_tick;
    uint32_t last_log_tick;
    bool     run_started;

    /* Latest setpoint and measurements (for logging / debugging) */
    float h_ref_m;
    float current_alt_m;
    float current_vel_ms;
    float current_throttle;
} gutter_state_t;

static gutter_state_t g = {0};

static inline float clamp_f(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ------------------------------------------------------------------------- */

static bool gto_on_configed(const profile_t *profile) {
    if (!profile || profile->kind != PROFILE_KIND_TEST_GUTTER) return false;
    const gutter_test_profile_t *p = &profile->test.gutter;

    if (p->h_ref_max_dm <= p->h_ref_min_dm) {
        printf("[GUTTER_OPS] reject: h_ref_max <= h_ref_min\r\n");
        return false;
    }
    if (p->initial_h_ref_dm < (int16_t)p->h_ref_min_dm
            || p->initial_h_ref_dm > (int16_t)p->h_ref_max_dm) {
        printf("[GUTTER_OPS] reject: initial outside [min,max]\r\n");
        return false;
    }
    if (p->pid_kp <= 0.0f || p->pid_integral_limit <= 0.0f) {
        printf("[GUTTER_OPS] reject: invalid gains (kp=%.3f I-lim=%.3f)\r\n",
               p->pid_kp, p->pid_integral_limit);
        return false;
    }
    if (!p->is_manual && p->hold_duration_ms == 0) {
        printf("[GUTTER_OPS] reject: auto needs hold_duration_ms > 0\r\n");
        return false;
    }

    /* Closed-loop needs a healthy baro (estimator depends on it). */
    if (fsm_ctx.boot_status.baro_init != 1) {
        printf("[GUTTER_OPS] reject: baro not initialized\r\n");
        return false;
    }
    if (!fsm_ctx.baro_cal.is_calibrated) {
        printf("[GUTTER_OPS] reject: baro not calibrated (run CALIBRATE_BARO first)\r\n");
        return false;
    }

    memset(&g, 0, sizeof(g));
    g.profile = *p;
    printf("[GUTTER_OPS] configed: %s, href=[%u..%u]dm init=%d kp=%.3f ki=%.3f kd=%.3f Ilim=%.2f\r\n",
           p->is_manual ? "MANUAL" : "AUTO",
           p->h_ref_min_dm, p->h_ref_max_dm, p->initial_h_ref_dm,
           p->pid_kp, p->pid_ki, p->pid_kd, p->pid_integral_limit);
    return true;
}

static bool gto_on_arm(void) {
    g.integral   = 0.0f;
    g.prev_error = 0.0f;
    g.h_ref_m    = (float)g.profile.initial_h_ref_dm / 10.0f;

    PWM_SetThrottle(0.0f);
    sd_card_pause();
    g.armed = true;
    g.last_tick = HAL_GetTick();

    printf("[GUTTER_OPS] armed (baseline alt=%.2f m, h_ref=%.2f m)\r\n",
           fsm_ctx.altitude_agl_m, g.h_ref_m);
    return true;
}

static void gto_on_countdown(void) {
    printf("[GUTTER_OPS] countdown\r\n");
}

/** @brief Compute h_ref(t) for the current tick. Always in [min, max]. */
static float compute_setpoint(uint32_t now, const test_control_packet_t *ctrl) {
    const gutter_test_profile_t *p = &g.profile;
    float min_m  = (float)p->h_ref_min_dm / 10.0f;
    float max_m  = (float)p->h_ref_max_dm / 10.0f;
    float init_m = (float)p->initial_h_ref_dm / 10.0f;

    if (p->is_manual) {
        if (ctrl == NULL) {
            /* Sub-heartbeat staleness: hold last commanded h_ref (don't snap
             * to min — that would cause a sudden descent on a single missed
             * packet). The 1 s heartbeat handles real link loss. */
            return clamp_f(g.h_ref_m, min_m, max_m);
        }
        if (ctrl->field_mask & TEST_CTRL_FIELD_H_REF) {
            return clamp_f((float)ctrl->h_ref_dm / 10.0f, min_m, max_m);
        }
        return clamp_f(g.h_ref_m, min_m, max_m);
    }

    /* Auto */
    if (p->curve == CURVE_RAMP && p->hold_duration_ms > 0) {
        uint32_t elapsed = now - g.run_start_tick;
        if (elapsed >= p->hold_duration_ms) return max_m;
        float frac = (float)elapsed / (float)p->hold_duration_ms;
        return init_m + (max_m - init_m) * frac;
    }
    return init_m;  /* CURVE_STEP: hold at initial */
}

static void gto_on_run_tick(const test_control_packet_t *ctrl) {
    uint32_t now = HAL_GetTick();

    if (!g.run_started) {
        g.run_started     = true;
        g.run_start_tick  = now;
        g.last_log_tick   = now;
        g.last_tick       = now;
    }

    /* dt for the PID */
    float dt = (float)(now - g.last_tick) / 1000.0f;
    if (dt <= 0.0f) dt = DT_DEFAULT_S;
    if (dt > DT_MAX_S) dt = DT_MAX_S;
    g.last_tick = now;

    /* Pull latest estimator state */
    g.current_alt_m  = fsm_ctx.altitude_agl_m;
    g.current_vel_ms = fsm_ctx.velocity_vertical_ms;

    /* ---- Hard altitude trip (safety) ----------------------------------- */
    float max_m = (float)g.profile.h_ref_max_dm / 10.0f;
    if (g.current_alt_m > max_m + ALT_HARD_OVER_M) {
        printf("[GUTTER_OPS] HARD TRIP: alt=%.2f > max+%.1f → ABORT\r\n",
               g.current_alt_m, ALT_HARD_OVER_M);
        test_runner_abort(TEST_EXIT_FAILURE);
        return;
    }

    /* ---- Setpoint ------------------------------------------------------ */
    g.h_ref_m = compute_setpoint(now, ctrl);

    /* ---- PID ----------------------------------------------------------- */
    float error = g.h_ref_m - g.current_alt_m;

    g.integral += error * dt;
    g.integral  = clamp_f(g.integral,
                          -g.profile.pid_integral_limit,
                           g.profile.pid_integral_limit);

    float deriv = (error - g.prev_error) / dt;
    g.prev_error = error;

    float u = g.profile.pid_kp * error
            + g.profile.pid_ki * g.integral
            + g.profile.pid_kd * deriv;

    float throttle = clamp_f(HOVER_FF + u, 0.0f, MAX_THROTTLE_FRAC);
    g.current_throttle = throttle;
    PWM_SetThrottle(throttle * 100.0f);

    /* ---- Periodic log (no new telem event for now — fast packet's
     * altitude field already gives the GS dashboard what it needs) ------- */
    if ((now - g.last_log_tick) >= LOG_INTERVAL_MS) {
        g.last_log_tick = now;
        printf("[GUTTER_OPS] h=%.2f h_ref=%.2f err=%.2f thr=%.2f%%\r\n",
               g.current_alt_m, g.h_ref_m, error, throttle * 100.0f);
    }
}

static void gto_on_hold(void) {
    PWM_SetThrottle(0.0f);
    g.current_throttle = 0.0f;
    /* Reset PID — when we resume, restart from a clean slate to avoid the
     * integral wind-up that built up while motor was off. */
    g.integral   = 0.0f;
    g.prev_error = 0.0f;
    printf("[GUTTER_OPS] hold (motor 0%%, PID reset)\r\n");
}

static void gto_on_resume(void) {
    /* Avoid huge dt on first tick after a long HOLD (would spike d-term). */
    g.last_tick = HAL_GetTick();
    printf("[GUTTER_OPS] resume\r\n");
}

static void gto_on_finishing(void) {
    PWM_SetThrottle(0.0f);
    g.current_throttle = 0.0f;
    printf("[GUTTER_OPS] finishing (final alt=%.2f m)\r\n", g.current_alt_m);
}

static void gto_on_exit(test_exit_reason_t reason) {
    PWM_EmergencyStop();
    g.current_throttle = 0.0f;

    if (g.armed) {
        sd_card_resume();
        g.armed = false;
    }

    printf("[GUTTER_OPS] exit (reason=%u)\r\n", (unsigned)reason);
}

/* ------------------------------------------------------------------------- */

const test_kind_ops_t gutter_test_ops = {
    .name         = "GUTTER",
    .on_configed  = gto_on_configed,
    .on_arm       = gto_on_arm,
    .on_countdown = gto_on_countdown,
    .on_run_tick  = gto_on_run_tick,
    .on_hold      = gto_on_hold,
    .on_resume    = gto_on_resume,
    .on_finishing = gto_on_finishing,
    .on_exit      = gto_on_exit,
};
