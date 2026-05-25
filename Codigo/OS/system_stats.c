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

bool system_stats_emit_one(void) {
    TaskStatus_t snapshot[MAX_TRACKED_TASKS];
    uint32_t total_runtime = 0;
    UBaseType_t total = uxTaskGetSystemState(snapshot, MAX_TRACKED_TASKS, &total_runtime);

    if (total == 0) {
        return false;  /* heap too low for the snapshot, or no tasks */
    }
    if (total > MAX_TRACKED_TASKS) {
        total = MAX_TRACKED_TASKS;  /* should not happen given the cap */
    }

    /* Pick the next task in round-robin. */
    uint8_t idx = cursor % total;
    cursor = (cursor + 1) % total;

    const TaskStatus_t *t = &snapshot[idx];

    thread_stat_payload_t pl = {0};
    pl.task_idx        = idx;
    pl.total_tasks     = (uint8_t)total;
    pl.priority        = (uint8_t)t->uxCurrentPriority;
    pl.state           = (uint8_t)t->eCurrentState;
    pl.stack_high_water = (uint16_t)t->usStackHighWaterMark;
    pl.runtime_counter = (uint32_t)t->ulRunTimeCounter;
    if (t->pcTaskName) {
        strncpy(pl.name, t->pcTaskName, sizeof(pl.name) - 1);
        pl.name[sizeof(pl.name) - 1] = '\0';
    }

    fsm_send_telemetry_event(EVT_THREAD_STAT, &pl, sizeof(pl));

    /* Emit a global stats packet whenever the cursor wraps to 0
     * (i.e. once per full round-robin cycle, ~every total_tasks seconds). */
    if (cursor == 0) {
        system_stat_payload_t sys = {0};
        sys.uptime_ms          = HAL_GetTick();
        sys.free_heap_bytes    = (uint32_t)xPortGetFreeHeapSize();
        sys.min_ever_free_heap = (uint32_t)xPortGetMinimumEverFreeHeapSize();
        sys.total_runtime      = total_runtime;
        sys.task_count         = (uint16_t)total;
        fsm_send_telemetry_event(EVT_SYSTEM_STAT, &sys, sizeof(sys));
    }
    return true;
}
