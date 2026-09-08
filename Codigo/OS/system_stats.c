/**
 * @file system_stats.c
 * @brief Round-robin emission of FreeRTOS task runtime stats.
 * @see system_stats.h
 */

#include "system_stats.h"
#include "Flight Computer/flight_computer.h"        /* EVT_THREAD_STAT, thread_stat_payload_t */
#include "Flight Computer/flight_computer_thread.h" /* fsm_send_telemetry_event */
#include "main.h"                                   /* HAL_GetTick() — portable across F446/H743 via CubeMX */
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define MAX_TRACKED_TASKS  16

/** Cursor advances on every call so the GS gets all tasks over time. */
static uint8_t cursor = 0;

/* One snapshot per round-robin CYCLE, cached. The old code re-ran
 * uxTaskGetSystemState() on every call (once per second, one task emitted
 * each time): task N's counter was sampled N seconds after task 0's, and the
 * total came from yet another moment — the GS percentages were mutually
 * inconsistent and summed to >100 %. All values emitted during one cycle now
 * come from the same instant, and we emit per-cycle DELTAS rather than
 * cumulative counters, so percentages are exact for the last cycle window
 * and immune to the 32-bit DWT counter wrap (~24 s @ 180 MHz). */
static TaskStatus_t snap[MAX_TRACKED_TASKS];
static uint32_t     prev_runtime[MAX_TRACKED_TASKS];   /* keyed by xTaskNumber%MAX */
static uint32_t     delta_runtime[MAX_TRACKED_TASKS];
static UBaseType_t  snap_total_tasks = 0;
static uint32_t     snap_total_delta = 0;
static uint32_t     prev_total_runtime = 0;

static void take_snapshot(void) {
    uint32_t total_runtime = 0;
    UBaseType_t n = uxTaskGetSystemState(snap, MAX_TRACKED_TASKS, &total_runtime);
    if (n > MAX_TRACKED_TASKS) n = MAX_TRACKED_TASKS;
    snap_total_tasks = n;

    snap_total_delta = total_runtime - prev_total_runtime;   /* modular: wrap-safe */
    prev_total_runtime = total_runtime;

    for (UBaseType_t i = 0; i < n; i++) {
        uint32_t key = snap[i].xTaskNumber % MAX_TRACKED_TASKS;
        delta_runtime[i] = snap[i].ulRunTimeCounter - prev_runtime[key];
        prev_runtime[key] = snap[i].ulRunTimeCounter;
    }
}

bool system_stats_emit_one(void) {
    /* Refresh the snapshot only when a new cycle starts. */
    if (cursor == 0 || snap_total_tasks == 0) {
        take_snapshot();
        if (snap_total_tasks == 0) {
            return false;  /* heap too low for the snapshot, or no tasks */
        }
    }

    UBaseType_t total = snap_total_tasks;
    uint8_t idx = cursor % total;
    cursor = (cursor + 1) % total;

    const TaskStatus_t *t = &snap[idx];

    thread_stat_payload_t pl = {0};
    pl.task_idx        = idx;
    pl.total_tasks     = (uint8_t)total;
    pl.priority        = (uint8_t)t->uxCurrentPriority;
    pl.state           = (uint8_t)t->eCurrentState;
    pl.stack_high_water = (uint16_t)t->usStackHighWaterMark;
    /* Per-cycle DELTA, consistent with sys.total_runtime below. */
    pl.runtime_counter = delta_runtime[idx];
    if (t->pcTaskName) {
        strncpy(pl.name, t->pcTaskName, sizeof(pl.name) - 1);
        pl.name[sizeof(pl.name) - 1] = '\0';
    }

    fsm_send_telemetry_event(EVT_THREAD_STAT, &pl, sizeof(pl));

    /* Emit a global stats packet whenever the cursor wraps to 0
     * (i.e. once per full round-robin cycle, ~every total_tasks seconds).
     * total_runtime is the DELTA of the same snapshot the per-task numbers
     * came from, so GS %  = task_delta / total_delta sums to ≤100 %. */
    if (cursor == 0) {
        system_stat_payload_t sys = {0};
        sys.uptime_ms          = HAL_GetTick();
        sys.free_heap_bytes    = (uint32_t)xPortGetFreeHeapSize();
        sys.min_ever_free_heap = (uint32_t)xPortGetMinimumEverFreeHeapSize();
        sys.total_runtime      = snap_total_delta;
        sys.task_count         = (uint16_t)total;
        fsm_send_telemetry_event(EVT_SYSTEM_STAT, &sys, sizeof(sys));
    }
    return true;
}
