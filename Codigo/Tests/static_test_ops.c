/**
 * @file static_test_ops.c
 * @brief STATIC test driver — `test_kind_ops_t` reference implementation
 * @author Tomás Teixeira
 * @date 2026
 *
 * @details
 * Phase 3-A (A1): clean implementation of the new per-test ops vtable for
 * STATIC thrust tests. The legacy `static_thrust_test.{c,h}` module stays
 * alive until A2/A3 swap the FSM to dispatch through `test_runner_get_ops()`;
 * once that lands, this file becomes the single source of truth and the
 * legacy module can be deleted.
 *
 * Recruta: this file is the template you copy for `torque_test_ops.c` and
 * `torque_cal_test_ops.c` (B1 / B2). The shape — cached profile + small
 * internal state struct + 8 callbacks driven by `on_run_tick` — is what
 * every test kind looks like.
 *
 * @ingroup Tests
 */

#include "test_runner.h"
#include "Flight Computer/test_profile.h"
#include "Flight Computer/flight_computer_thread.h"
#include "Atuadores/ESC/PWM_FUNCTIONS.h"
#include "Storage/sd_card_thread.h"
#include "Sensors/FX29/FX29.h"
#include "Telemetry/telemetry.h"
#include "config.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c2;  /* I2C_LOADCELL */

#define LOADCELL_SCALE_FACTOR     0.4481f
#define SAMPLE_INTERVAL_MS        50    /* 20 Hz */
#define PROGRESS_INTERVAL_MS      200   /* 5 Hz telemetry to GS */

typedef struct {
    static_test_profile_t profile;     /**< Cached at on_configed */
    bool                  armed;       /**< Hardware ready (load cell, sd paused) */

    uint32_t run_start_tick;           /**< HAL tick when on_run_tick first fired */
    uint32_t last_sample_tick;
    uint32_t last_progress_tick;
    bool     run_started;              /**< First on_run_tick has fired */

    FX29_t   loadcell;
    float    current_throttle_pct;     /**< Last commanded throttle (0..100) */
    float    current_thrust_n;
    float    max_thrust_n;
    float    max_thrust_pwm;
    uint16_t sample_count;
} static_test_state_t;

static static_test_state_t s = {0};

/* ------------------------------------------------------------------------- */
/*  Helpers                                                                  */
/* ------------------------------------------------------------------------- */

/** @brief Convert profile-domain milli (0..1000) to PWM percent (0..100). */
static inline float milli_to_pct(uint16_t milli) {
    if (milli > 1000) milli = 1000;
    return (float)milli * 0.1f;
}

/** @brief Auto-mode setpoint as a function of elapsed time. */
static float auto_setpoint_pct(uint32_t elapsed_ms) {
    float target = milli_to_pct(s.profile.max_throttle_milli);

    if (s.profile.curve == CURVE_RAMP && s.profile.hold_duration_ms > 0) {
        if (elapsed_ms >= s.profile.hold_duration_ms) return target;
        return target * ((float)elapsed_ms / (float)s.profile.hold_duration_ms);
    }
    return target;  /* CURVE_STEP */
}

static void take_sample(uint32_t now) {
    LOADCELL_t reading;
    if (!FX29_ReadWithPWM(&s.loadcell, &reading,
                          (uint16_t)(s.current_throttle_pct * 10.0f))) {
        return;
    }
    s.current_thrust_n = reading.force_n;
    s.sample_count++;
    if (reading.force_n > s.max_thrust_n) {
        s.max_thrust_n   = reading.force_n;
        s.max_thrust_pwm = s.current_throttle_pct;
    }
    (void)now;
}

static void emit_progress(void) {
    struct __attribute__((packed)) {
        uint8_t pwm_percent;
        float   thrust_n;
    } p = {
        .pwm_percent = (uint8_t)s.current_throttle_pct,
        .thrust_n    = s.current_thrust_n,
    };
    fsm_send_telemetry_event(EVT_STATIC_TEST_PROGRESS, &p, sizeof(p));
}

/* ------------------------------------------------------------------------- */
/*  Callbacks                                                                */
/* ------------------------------------------------------------------------- */

static bool sto_on_configed(const profile_t *profile) {
    if (!profile || profile->kind != PROFILE_KIND_TEST_STATIC) return false;

    const static_test_profile_t *p = &profile->test.static_test;
    if (p->max_throttle_milli == 0 || p->max_throttle_milli > 1000) {
        printf("[STATIC_OPS] reject: max_throttle_milli=%u\r\n", p->max_throttle_milli);
        return false;
    }
    if (!p->is_manual && p->hold_duration_ms == 0) {
        printf("[STATIC_OPS] reject: auto mode needs hold_duration_ms > 0\r\n");
        return false;
    }

    memset(&s, 0, sizeof(s));
    s.profile = *p;
    printf("[STATIC_OPS] configed: %s, max=%u milli, hold=%u ms, curve=%u\r\n",
           p->is_manual ? "MANUAL" : "AUTO",
           p->max_throttle_milli, p->hold_duration_ms, (unsigned)p->curve);
    return true;
}

static bool sto_on_arm(void) {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (FX29_ADDR_0 << 1), 3, 100) != HAL_OK) {
        printf("[STATIC_OPS] arm: load cell not on I2C2\r\n");
        return false;
    }
    if (!FX29_Init(&s.loadcell, &hi2c2, FX29_ADDR_0, FX29_RANGE_500N)) {
        printf("[STATIC_OPS] arm: FX29_Init failed\r\n");
        return false;
    }
    FX29_SetScaleFactor(&s.loadcell, LOADCELL_SCALE_FACTOR);
    if (!FX29_Tare(&s.loadcell)) {
        printf("[STATIC_OPS] arm: FX29_Tare failed\r\n");
        return false;
    }

    PWM_SetThrottle(0.0f);
    sd_card_pause();
    s.armed = true;

    uint8_t throttle_pct = (uint8_t)(s.profile.max_throttle_milli / 10);
    fsm_send_telemetry_event(EVT_STATIC_TEST_STARTED, &throttle_pct, 1);

    printf("[STATIC_OPS] armed (tare ok, sd paused)\r\n");
    return true;
}

static void sto_on_countdown(void) {
    printf("[STATIC_OPS] countdown\r\n");
}

static void sto_on_run_tick(const test_control_packet_t *ctrl) {
    uint32_t now = HAL_GetTick();

    if (!s.run_started) {
        s.run_started       = true;
        s.run_start_tick    = now;
        s.last_sample_tick  = now;
        s.last_progress_tick = now;
    }

    /* ----- Compute setpoint ----- */
    float setpoint_pct;
    if (s.profile.is_manual) {
        if (ctrl == NULL || !(ctrl->field_mask & TEST_CTRL_FIELD_THROTTLE)) {
            /* Heartbeat stale or field not present — coast at 0 for safety. */
            setpoint_pct = 0.0f;
        } else {
            uint16_t cmd = ctrl->throttle_milli;
            if (cmd > s.profile.max_throttle_milli) cmd = s.profile.max_throttle_milli;
            setpoint_pct = milli_to_pct(cmd);
        }
    } else {
        uint32_t elapsed = now - s.run_start_tick;
        setpoint_pct = auto_setpoint_pct(elapsed);
    }

    s.current_throttle_pct = setpoint_pct;
    PWM_SetThrottle(setpoint_pct);

    /* ----- Sample at 20 Hz ----- */
    if ((now - s.last_sample_tick) >= SAMPLE_INTERVAL_MS) {
        take_sample(now);
        s.last_sample_tick = now;
    }

    /* ----- Progress telemetry at 5 Hz ----- */
    if ((now - s.last_progress_tick) >= PROGRESS_INTERVAL_MS) {
        emit_progress();
        s.last_progress_tick = now;
    }
}

static void sto_on_hold(void) {
    PWM_SetThrottle(0.0f);
    s.current_throttle_pct = 0.0f;
    printf("[STATIC_OPS] hold (motor 0%%)\r\n");
}

static void sto_on_resume(void) {
    /* Next on_run_tick re-applies setpoint; nothing to do here. */
    printf("[STATIC_OPS] resume\r\n");
}

static void sto_on_finishing(void) {
    PWM_SetThrottle(0.0f);
    s.current_throttle_pct = 0.0f;

    struct __attribute__((packed)) {
        float    max_thrust_n;
        float    max_thrust_pwm;
        uint16_t sample_count;
    } result = {
        .max_thrust_n   = s.max_thrust_n,
        .max_thrust_pwm = s.max_thrust_pwm,
        .sample_count   = s.sample_count,
    };
    fsm_send_telemetry_event(EVT_STATIC_TEST_COMPLETE, &result, sizeof(result));

    printf("[STATIC_OPS] finishing: %u samples, max=%.2f N @ %.1f%%\r\n",
           s.sample_count, s.max_thrust_n, s.max_thrust_pwm);
}

static void sto_on_exit(test_exit_reason_t reason) {
    PWM_EmergencyStop();
    s.current_throttle_pct = 0.0f;

    if (s.armed) {
        sd_card_resume();
        s.armed = false;
    }

    if (reason != TEST_EXIT_DONE) {
        fsm_send_telemetry_event(EVT_STATIC_TEST_FAILED, NULL, 0);
    }

    printf("[STATIC_OPS] exit (reason=%u)\r\n", (unsigned)reason);
}

/* ------------------------------------------------------------------------- */
/*  Vtable                                                                   */
/* ------------------------------------------------------------------------- */

const test_kind_ops_t static_test_ops = {
    .name         = "STATIC",
    .on_configed  = sto_on_configed,
    .on_arm       = sto_on_arm,
    .on_countdown = sto_on_countdown,
    .on_run_tick  = sto_on_run_tick,
    .on_hold      = sto_on_hold,
    .on_resume    = sto_on_resume,
    .on_finishing = sto_on_finishing,
    .on_exit      = sto_on_exit,
};
