/**
 * @file test_profile.h
 * @brief Profile types for the new test-mode FSM (Phase 2 contract)
 * @author Tomás Teixeira
 * @date 2026
 *
 * @details
 * Defines the type vocabulary that the new sub-FSM (STATE_TEST_<KIND>) and
 * the per-test ops modules consume. This is a **contract-only** header — no
 * runtime behaviour is implemented here. The migration that wires these
 * types into the FSM is Phase 3-A; the recruit fills in the per-test
 * implementations referencing these types.
 *
 * ## Naming relative to legacy types
 * The pre-existing `flight_profile_t` (enum) and `profile_params_t` (struct)
 * are intentionally **not touched** by this header — they continue to drive
 * the flight code path until Phase 3-A migrates everything to the unified
 * `profile_t` defined below. The new enum is therefore named `profile_kind_t`
 * to avoid a name collision.
 *
 * ## Kinds
 * - PROFILE_FLIGHT          : free-flight, autopilot in command
 * - PROFILE_TEST_STATIC     : motor on test rig, thrust vs throttle
 * - PROFILE_TEST_TORQUE     : like static + moment arm → torque calc
 * - PROFILE_TEST_TORQUE_CAL : torque test with servo-driven alpha sweep
 * - PROFILE_TEST_GUTTER     : tethered vertical climb tracking h_ref (closed loop)
 *
 * Each TEST kind has Manual and Automatic sub-modes (`is_manual` field).
 *
 * @ingroup FSM
 */

#ifndef TEST_PROFILE_H
#define TEST_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Profile kind (replaces the legacy `flight_profile_t` enum after
 * Phase 3-A migration). For now both coexist.
 */
typedef enum {
    PROFILE_KIND_NONE         = 0,
    PROFILE_KIND_FLIGHT       = 1,
    PROFILE_KIND_TEST_STATIC  = 2,
    PROFILE_KIND_TEST_TORQUE  = 3,
    PROFILE_KIND_TEST_TORQUE_CAL = 4,
    PROFILE_KIND_TEST_GUTTER  = 5,
} profile_kind_t;

/**
 * @brief Throttle (or h_ref / alpha) curve shape during automatic mode.
 */
typedef enum {
    CURVE_STEP = 0,   /**< Instant jump to setpoint, then hold */
    CURVE_RAMP = 1,   /**< Linear interpolation from 0 to setpoint over duration */
} test_curve_t;

/* ============================================================================
 * Per-test profile structs
 * ----------------------------------------------------------------------------
 * Manual mode: most numeric fields are ignored — the GS drives the actuator
 * via the test_control_packet_t (see telemetry.h).
 * Automatic mode: fields define the deterministic recipe the FC executes.
 * ============================================================================ */

/**
 * @brief STATIC: horizontal-rail motor on load cell, measure thrust vs throttle.
 */
typedef struct {
    bool         is_manual;          /**< true = GS drives throttle, false = follow curve */
    uint16_t     max_throttle_milli; /**< Auto: setpoint (0..1000); Manual: upper safety clamp */
    uint16_t     hold_duration_ms;   /**< Auto only: time to hold at max throttle */
    test_curve_t curve;              /**< Auto only: STEP or RAMP */
} static_test_profile_t;

/**
 * @brief TORQUE: like STATIC but with a moment arm so we get torque = F·L.
 */
typedef struct {
    bool         is_manual;
    uint16_t     max_throttle_milli;
    uint16_t     hold_duration_ms;
    test_curve_t curve;
    float        moment_arm_m;       /**< Distance from load cell to motor centerline (metres) */
} torque_test_profile_t;

/**
 * @brief TORQUE_CAL: TORQUE test plus a servo (alpha) sweep to characterise
 * the relationship between alpha and produced torque.
 */
typedef struct {
    bool         is_manual;
    uint16_t     max_throttle_milli;
    uint16_t     hold_duration_ms;
    test_curve_t throttle_curve;
    int16_t      alpha_min_centideg; /**< Servo limits (centidegrees, –18000..+18000) */
    int16_t      alpha_max_centideg;
    test_curve_t alpha_curve;        /**< STEP between extremes vs continuous RAMP */
    uint16_t     alpha_period_ms;    /**< Auto: full sweep duration */
    float        moment_arm_m;
} torque_cal_test_profile_t;

/**
 * @brief GUTTER: tethered vertical climb on a rail, FC tracks h_ref via
 * its closed-loop controller. Riskier — needs baro+estimator+controller
 * all healthy before arming.
 */
typedef struct {
    bool         is_manual;
    uint16_t     hold_duration_ms;
    uint16_t     h_ref_min_dm;       /**< Hard lower bound (decimetres, ≥0) */
    uint16_t     h_ref_max_dm;       /**< Hard upper bound (decimetres) */
    int16_t      initial_h_ref_dm;   /**< Setpoint at run-start */
    test_curve_t curve;              /**< Auto only: h_ref(t) shape */
    float        pid_kp;             /**< Controller gains, applied at TEST_ARMED entry */
    float        pid_ki;
    float        pid_kd;
    float        pid_integral_limit; /**< Anti-windup cap */
} gutter_test_profile_t;

/**
 * @brief Tagged union over the 4 test kinds.
 *
 * Pair this with the `kind` field in `profile_t` below to know which
 * variant is live. C unions don't carry their own tag — always check
 * `profile.kind` before reading.
 */
typedef union {
    static_test_profile_t     static_test;
    torque_test_profile_t     torque_test;
    torque_cal_test_profile_t torque_cal;
    gutter_test_profile_t     gutter;
} test_profile_t;

/**
 * @brief Flight profile parameters (parallel struct to legacy
 * `profile_params_t` — fields a 1:1 subset of the flight-relevant ones).
 *
 * @note During Phase 3-A migration, `profile_params_t` will be deleted and
 * code that consumes `fsm_ctx.profile.target_altitude_m` etc. will be
 * updated to read from `fsm_ctx.profile_v2.flight.target_altitude_m`.
 */
typedef struct {
    float target_altitude_m;
    float flare_altitude_m;
    float touchdown_velocity_ms;
    float max_altitude_m;
    float max_velocity_ms;
    float landing_lat;
    float landing_lon;
    float throttle_min;
    float throttle_max;
} flight_profile_params_t;

/**
 * @brief Top-level profile carrier. The `kind` field selects which arm of
 * the union is live; readers must check it before access.
 */
typedef struct {
    profile_kind_t kind;
    union {
        flight_profile_params_t flight;
        test_profile_t          test;
    };
} profile_t;

#endif /* TEST_PROFILE_H */
