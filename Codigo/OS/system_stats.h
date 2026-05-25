/**
 * @file system_stats.h
 * @brief Round-robin emission of FreeRTOS task runtime stats as telemetry events.
 *
 * @details
 * Reads `uxTaskGetSystemState()` and emits one EVT_THREAD_STAT per call,
 * cycling through the task list. The Ground Station accumulates these
 * over a few seconds to build a complete table of stack usage and
 * relative CPU time per task.
 *
 * Requires FreeRTOSConfig.h to have:
 *   - configUSE_TRACE_FACILITY = 1
 *   - configGENERATE_RUN_TIME_STATS = 1
 *   - INCLUDE_uxTaskGetStackHighWaterMark = 1
 * (all already enabled in this project).
 *
 * @ingroup OS
 */

#ifndef SYSTEM_STATS_H
#define SYSTEM_STATS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Emit one thread's stats as an EVT_THREAD_STAT event.
 *
 * Internal cursor advances each call so successive invocations cycle
 * through every task. Safe to call from any task context.
 *
 * Recommended cadence: 1 Hz from the FSM thread.
 *
 * @retval true  An event was queued (or task list was non-empty).
 * @retval false Task list snapshot failed (only happens on heap exhaustion).
 */
bool system_stats_emit_one(void);

#endif /* SYSTEM_STATS_H */
