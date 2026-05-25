/**
 * @file radio_thread.c
 * @brief LoRa Radio Communication Thread Implementation (TDMA v3)
 * @author Tomás Teixeira
 * @date 2025
 * @version 3.1
 *
 * @details
 * Implements the LoRa radio communication system for the Magalhães Flight
 * Computer using Time Division Multiple Access (TDMA) synchronization with
 * the Ground Station.
 *
 * Two radio interface backends are compiled via preprocessor selection:
 * - **RADIO_INTERFACE_UART**: E22 module over UART DMA (F446ZE dev board),
 *   uses e22_uart_dma.h driver
 * - **RADIO_INTERFACE_SPI**: SX126x over SPI DMA (Buzz V4 PCB / H743),
 *   uses lora_sx126x.h driver
 *
 * Both backends share the same TDMA frame structure and telemetry layer.
 *
 * ## TDMA Frame Structure
 *
 * @verbatim
 *   ├────────────────── 1000ms Superframe ──────────────────┤
 *   │                                                        │
 *   ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 *   │  0  │  1  │  2  │  3  │  4  │  5  │  6  │  7  │  8  │  9  │
 *   ├─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┤
 *   │ FAST│FAST │FAST │FAST │FAST │FAST │FAST │FAST │SLOW │ RX  │
 *   │ TX  │ TX  │ TX  │ TX  │ TX  │ TX  │ TX  │ TX  │ TX  │only │
 *   └─────────────────────────────────────────────────────────────┘
 *
 *   Slots 0-7: Fast telemetry (8 Hz) - FC transmits
 *   Slot 8:    Slow telemetry (1 Hz) - FC transmits GPS/temp
 *   Slot 9:    RX only - FC listens for GS commands/sync
 * @endverbatim
 *
 * ## TX Timing Within Slot
 * | Phase | Time (ms) | Action |
 * |-------|-----------|--------|
 * | Start | 0-5 | Guard time |
 * | TX Window | 5-70 | Transmit telemetry |
 * | Guard | 70-100 | Prepare for next slot |
 *
 * ## Synchronization
 * - FC starts in UNSYNCED mode, beaconing every 5 seconds
 * - GS sends SYNC packets in slot 9
 * - FC aligns frame timing to received SYNC
 * - Lost sync after 10 missed frames
 *
 * ## Key Improvements in v3
 * - slot_index properly tracked globally
 * - Single TX per slot enforced
 * - 20ms guard before RX slot
 * - Improved ACK piggyback in telemetry
 *
 * @see radio_thread.h for interface documentation
 * @see telemetry.h for packet formats
 * @ingroup Radio_Communication
 */

#include "radio_thread.h"
#ifdef RADIO_INTERFACE_UART
#include "Radio/LORA Drivers/e22_uart_dma.h"
#endif
#include "Radio/CRC16/crc16.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include "Threads/create_threads.h"
#include "Tests/test_runner.h"

// ============================================================================
// Phase 3-A (A4): TDMA preset abstraction (shared between UART and SPI)
// ============================================================================
typedef struct {
    uint16_t superframe_ms;
    uint16_t slot_ms;
    uint8_t  slots_per_frame;
    uint8_t  slow_slot;
    uint8_t  rx_slot_a;
    uint8_t  rx_slot_b;          /* 0xFF if no second RX slot */
    uint8_t  rx_slot_for_sync;   /* RX slot the GS is expected to TX in (sync alignment) */
} tdma_preset_t;

static const tdma_preset_t TDMA_PRESET_FLIGHT = {
    .superframe_ms    = TDMA_SUPERFRAME_MS,
    .slot_ms          = TDMA_SLOT_MS,
    .slots_per_frame  = TDMA_SLOTS_PER_FRAME,
    .slow_slot        = TDMA_SLOW_SLOT,
    .rx_slot_a        = TDMA_RX_SLOT,
    .rx_slot_b        = 0xFF,
    .rx_slot_for_sync = TDMA_RX_SLOT,
};

static const tdma_preset_t TDMA_PRESET_TEST = {
    .superframe_ms    = TDMA_TI_SUPERFRAME_MS,
    .slot_ms          = TDMA_TI_SLOT_MS,
    .slots_per_frame  = TDMA_TI_SLOTS_PER_FRAME,
    .slow_slot        = TDMA_TI_SLOW_SLOT,
    .rx_slot_a        = TDMA_TI_RX_SLOT_A,
    .rx_slot_b        = TDMA_TI_RX_SLOT_B,
    .rx_slot_for_sync = TDMA_TI_RX_SLOT_A,
};

static volatile tdma_mode_t current_tdma_mode = TDMA_MODE_FLIGHT;
static volatile tdma_mode_t pending_tdma_mode = TDMA_MODE_FLIGHT;

static inline const tdma_preset_t *active_preset(void) {
    return (current_tdma_mode == TDMA_MODE_TEST_INTERACTIVE)
        ? &TDMA_PRESET_TEST : &TDMA_PRESET_FLIGHT;
}

static inline bool slot_is_rx(uint8_t slot) {
    const tdma_preset_t *p = active_preset();
    return (slot == p->rx_slot_a) || (p->rx_slot_b != 0xFF && slot == p->rx_slot_b);
}

static inline bool slot_is_slow(uint8_t slot) {
    return slot == active_preset()->slow_slot;
}

static inline bool slot_is_tx_fast(uint8_t slot) {
    /* Everything else is fast TX */
    return !slot_is_rx(slot) && !slot_is_slow(slot);
}

/** @brief Public API: queue a TDMA mode switch (deferred to next superframe). */
void radio_request_tdma_mode(tdma_mode_t mode) {
    pending_tdma_mode = mode;
}

/** @brief Public: current active TDMA mode (e.g. for slow packet field). */
tdma_mode_t radio_get_tdma_mode(void) { return current_tdma_mode; }

#ifdef RADIO_INTERFACE_UART
// ============================================================================
// E22 UART Radio Implementation (F446ZE dev board)
// ============================================================================

// TDMA timing — slot/superframe durations are now driven by active_preset()
// (tdma_preset_t in shared block above). TX windows scale with slot_ms.
/* Phase 3-A bugfix: time-based sync timeout instead of frame count.
 * The old "10 frames" scaled poorly between presets — 10 s in flight (fine)
 * but only 2 s in test mode, which trips falsely when the Arduino prioritises
 * test_ctrl over sync (manual flow). 5 s of absolute silence is the trip. */
#define SYNC_TIMEOUT_MS         5000
#define SYNC_CORRECTION_THRESH  10
#define E22_TURNAROUND_MS       30      // E22 TX->RX turnaround time

// TX windows are computed as fractions of the slot rather than absolute ms,
// so they automatically rescale between flight (100 ms) and test (40 ms) slots.
#define TX_START_PCT            5       // Start TX after 5% of slot
#define TX_END_PCT              70      // Stop TX after 70% of slot
#define RX_GUARD_PCT            20      // Last 20% of slow slot reserved (flight)

// When UNSYNCED, TX beacon every N seconds
#define UNSYNC_BEACON_INTERVAL_MS  5000

// State
tdma_ctx_t tdma_ctx = {0};
radio_stats_t radio_stats = {0};

extern fsm_ctx_t fsm_ctx;

// Latest sensor data
static IMU_t latest_imu = {0};
static BARO_t latest_baro = {0};
static BNO_t latest_bno = {0};
static GPS_t latest_gps = {0};
static SemaphoreHandle_t sensor_mutex = NULL;

// RX packet accumulator
#define RX_BUFFER_SIZE 64
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_idx = 0;
static uint32_t rx_last_byte_tick = 0;
#define RX_TIMEOUT_MS 50

// TX tracking
static uint32_t last_tx_tick = 0;
static uint32_t last_unsync_beacon_tick = 0;
static bool tx_done_this_slot = false;  // Prevent multiple TX per slot

// ============================================================================
// Sensor Data Update
// ============================================================================
void radio_update_sensor_data(const IMU_t *imu, const BARO_t *baro,
                               const BNO_t *bno, const GPS_t *gps) {
    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (imu) memcpy(&latest_imu, imu, sizeof(IMU_t));
        if (baro) memcpy(&latest_baro, baro, sizeof(BARO_t));
        if (bno) memcpy(&latest_bno, bno, sizeof(BNO_t));
        if (gps) memcpy(&latest_gps, gps, sizeof(GPS_t));
        xSemaphoreGive(sensor_mutex);
    }
}

// ============================================================================
// TDMA Timing
// ============================================================================
static uint8_t get_current_slot(void) {
    const tdma_preset_t *p = active_preset();
    uint32_t elapsed = HAL_GetTick() - tdma_ctx.frame_start_tick;
    return (elapsed / p->slot_ms) % p->slots_per_frame;
}

static uint32_t get_time_in_slot(void) {
    const tdma_preset_t *p = active_preset();
    uint32_t elapsed = HAL_GetTick() - tdma_ctx.frame_start_tick;
    return elapsed % p->slot_ms;
}

static void advance_frame(void) {
    tdma_ctx.frame_id++;
    tdma_ctx.frame_start_tick += active_preset()->superframe_ms;

    // Phase 3-A (A4): apply queued mode switch only at superframe boundary.
    if (pending_tdma_mode != current_tdma_mode) {
        printf("[RADIO] TDMA mode: %u -> %u\r\n",
               (unsigned)current_tdma_mode, (unsigned)pending_tdma_mode);
        current_tdma_mode = pending_tdma_mode;
    }
    // Don't reset slot_index here - it's calculated dynamically
}

static bool can_transmit(void) {
    // Don't TX if we recently transmitted (E22 turnaround)
    uint32_t since_tx = HAL_GetTick() - last_tx_tick;
    if (since_tx < E22_TURNAROUND_MS) return false;

    // Don't TX if E22 is busy
    if (E22_IsBusy()) return false;

    // Already transmitted this slot
    if (tx_done_this_slot) return false;

    return true;
}

static void sync_to_gs(uint8_t gs_frame_id, uint32_t gs_time) {
    uint32_t now = HAL_GetTick();
    const tdma_preset_t *p = active_preset();

    // When we receive a sync, we know GS sent it at the START of its TX slot
    // Align our frame so that slot starts at this moment.
    uint32_t new_frame_start = now - ((uint32_t)p->rx_slot_for_sync * p->slot_ms);

    if (tdma_ctx.state == TDMA_SYNCED) {
        // Already synced - check if we need correction
        int32_t error = (int32_t)(new_frame_start - tdma_ctx.frame_start_tick);

        if (error > SYNC_CORRECTION_THRESH || error < -SYNC_CORRECTION_THRESH) {
            tdma_ctx.frame_start_tick = new_frame_start;
            radio_stats.sync_corrections++;
        }
    } else {
        // First sync - hard align
        tdma_ctx.frame_start_tick = new_frame_start;
        tdma_ctx.state = TDMA_SYNCED;
        printf("[RADIO] SYNCED! frame=%u\r\n", gs_frame_id);
    }

    tdma_ctx.frame_id = gs_frame_id;
    tdma_ctx.last_sync_tick = now;
    tdma_ctx.missed_sync_count = 0;
}

// ============================================================================
// TX Functions
// ============================================================================
static void tx_fast_telemetry(uint8_t slot) {
    if (!can_transmit()) return;

    telemetry_fast_t pkt;

    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        telemetry_build_fast(&pkt, tdma_ctx.frame_id, slot,
                             tdma_ctx.fast_seq++, &fsm_ctx,
                             &latest_imu, &latest_baro, &latest_bno,
                             tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
        xSemaphoreGive(sensor_mutex);
    } else {
        telemetry_build_fast(&pkt, tdma_ctx.frame_id, slot,
                             tdma_ctx.fast_seq++, &fsm_ctx,
                             NULL, NULL, NULL,
                             tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
    }

    if (E22_Transmit((uint8_t*)&pkt, sizeof(pkt)) == E22_OK) {
        radio_stats.tx_fast++;
        last_tx_tick = HAL_GetTick();
        tx_done_this_slot = true;
    }
}

static void tx_slow_telemetry(uint8_t slot) {
    if (!can_transmit()) return;

    telemetry_slow_t pkt;
    uint8_t mode      = (uint8_t)current_tdma_mode;
    uint8_t slow_slot = active_preset()->slow_slot;

    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        telemetry_build_slow(&pkt, tdma_ctx.frame_id, tdma_ctx.slow_seq++,
                             &latest_gps, &latest_baro, 100, 1, mode, slow_slot);
        xSemaphoreGive(sensor_mutex);
    } else {
        telemetry_build_slow(&pkt, tdma_ctx.frame_id, tdma_ctx.slow_seq++,
                             NULL, NULL, 100, 1, mode, slow_slot);
    }

    if (E22_Transmit((uint8_t*)&pkt, sizeof(pkt)) == E22_OK) {
        radio_stats.tx_slow++;
        last_tx_tick = HAL_GetTick();
        tx_done_this_slot = true;
    }
    (void)slot;
}

// ============================================================================
// Event Transmission (PONG, state changes, etc.)
// ============================================================================
static void tx_pending_events(void) {
    if (!can_transmit()) return;
    if (queue_fsm_events == NULL) return;

    telemetry_event_t evt;

    // Check for pending events (non-blocking)
    if (xQueueReceive(queue_fsm_events, &evt, 0) == pdTRUE) {
        // Update frame/slot info
        evt.frame_id = tdma_ctx.frame_id;
        evt.slot_id = tdma_ctx.slot_index;
        evt.seq = tdma_ctx.event_seq++;

        // Recalculate CRC after updating header
        evt.crc16 = crc16_calculate((uint8_t*)&evt, sizeof(telemetry_event_t) - 2);

        if (E22_Transmit((uint8_t*)&evt, sizeof(evt)) == E22_OK) {
            radio_stats.tx_event++;
            last_tx_tick = HAL_GetTick();
            // EVT_THREAD_STAT (23) and EVT_SYSTEM_STAT (24) are emitted ~1Hz —
            // skip the print to avoid log spam. Other events still log.
            if (evt.event_type != EVT_THREAD_STAT && evt.event_type != EVT_SYSTEM_STAT) {
                printf("[RADIO] Event TX: type=%u\r\n", evt.event_type);
            }
        }
    }
}

// ============================================================================
// RX Functions
// ============================================================================
static uint16_t get_packet_size(uint8_t type) {
    switch (type) {
        case TELEM_PACKET_COMMAND:   return sizeof(command_packet_t);
        case TELEM_PACKET_SYNC:      return sizeof(sync_packet_t);
        case TELEM_PACKET_TEST_CTRL: return sizeof(test_control_packet_t);
        default: return 0;
    }
}

/* Phase 3-A (A5): inbound test_control_packet_t from GS during manual tests. */
static void process_test_ctrl(test_control_packet_t *ctrl) {
    uint16_t calc_crc = crc16_calculate((uint8_t*)ctrl, sizeof(test_control_packet_t) - 2);
    if (calc_crc != ctrl->crc16) {
        radio_stats.crc_errors++;
        return;
    }
    /* Phase 3-A bugfix: any valid RX from GS counts as link alive — don't trip
     * the sync timeout when test_ctrl is flowing but sync isn't. */
    tdma_ctx.last_sync_tick = HAL_GetTick();
    test_runner_submit_control(ctrl);
}

static void process_command(command_packet_t *cmd) {
    uint16_t calc_crc = crc16_calculate((uint8_t*)cmd, sizeof(command_packet_t) - 2);
    if (calc_crc != cmd->crc16) {
        radio_stats.crc_errors++;
        return;
    }

    radio_stats.rx_cmd++;

    // Store for ACK piggyback
    tdma_ctx.last_cmd_seq_received = cmd->cmd_seq;

    // Forward command to FSM thread via queue
    fsm_cmd_msg_t fsm_msg = {0};
    fsm_msg.cmd = (fsm_command_t)cmd->cmd_id;
    fsm_msg.cmd_seq = cmd->cmd_seq;
    memcpy(&fsm_msg.payload, cmd->params, sizeof(cmd->params));

    if (fsm_send_command(fsm_msg.cmd, fsm_msg.cmd_seq, &fsm_msg.payload, sizeof(fsm_msg.payload))) {
        tdma_ctx.last_cmd_status = 0;  // Queued OK
        printf("[RADIO] CMD %d -> FSM queue\r\n", cmd->cmd_id);
    } else {
        tdma_ctx.last_cmd_status = 2;  // Queue full
        printf("[RADIO] CMD queue FULL!\r\n");
    }

    // Sync timing
    sync_to_gs(cmd->frame_id, cmd->time);
}
static void process_sync(sync_packet_t *sync) {
    uint16_t calc_crc = crc16_calculate((uint8_t*)sync, sizeof(sync_packet_t) - 2);
    if (calc_crc != sync->crc16) {
        radio_stats.crc_errors++;
        return;
    }

    radio_stats.rx_sync++;
    sync_to_gs(sync->frame_id, sync->gs_time);
}

static bool process_rx_byte(uint8_t byte) {
    uint32_t now = HAL_GetTick();

    // Timeout: reset buffer if no bytes for a while
    if (rx_idx > 0 && (now - rx_last_byte_tick) > RX_TIMEOUT_MS) {
        rx_idx = 0;
    }
    rx_last_byte_tick = now;

    // First byte: validate packet type
    if (rx_idx == 0) {
        uint16_t expected = get_packet_size(byte);
        if (expected == 0) {
            return false;  // Not a packet type we're looking for
        }
    }

    // Store byte
    if (rx_idx < RX_BUFFER_SIZE) {
        rx_buffer[rx_idx++] = byte;
    } else {
        rx_idx = 0;
        return false;
    }

    // Check for complete packet
    uint8_t type = rx_buffer[0];
    uint16_t expected = get_packet_size(type);

    if (rx_idx >= expected) {
        if (type == TELEM_PACKET_COMMAND) {
            process_command((command_packet_t*)rx_buffer);
        } else if (type == TELEM_PACKET_SYNC) {
            process_sync((sync_packet_t*)rx_buffer);
        } else if (type == TELEM_PACKET_TEST_CTRL) {
            process_test_ctrl((test_control_packet_t*)rx_buffer);
        }
        rx_idx = 0;
        return true;
    }

    return false;
}

static void process_rx_data(void) {
    uint8_t temp[64];
    int bytes = E22_Receive(temp, sizeof(temp));

    for (int i = 0; i < bytes; i++) {
        process_rx_byte(temp[i]);
    }
}

// ============================================================================
// Mode Handlers
// ============================================================================
static void handle_synced_mode(void) {
    uint32_t now = HAL_GetTick();
    const tdma_preset_t *p = active_preset();

    // TX windows scaled per slot (so test mode 40 ms slot still gets a sane window)
    uint32_t tx_start_ms = (p->slot_ms * TX_START_PCT) / 100;
    uint32_t tx_end_ms   = (p->slot_ms * TX_END_PCT)   / 100;
    uint32_t rx_guard_ms = (p->slot_ms * RX_GUARD_PCT) / 100;

    // Check for frame rollover
    if ((now - tdma_ctx.frame_start_tick) >= p->superframe_ms) {
        advance_frame();
        p = active_preset();  /* re-read in case mode just switched */
        tx_start_ms = (p->slot_ms * TX_START_PCT) / 100;
        tx_end_ms   = (p->slot_ms * TX_END_PCT)   / 100;
        rx_guard_ms = (p->slot_ms * RX_GUARD_PCT) / 100;
    }

    // Phase 3-A bugfix: absolute-time sync timeout (mode-independent).
    if ((now - tdma_ctx.last_sync_tick) > SYNC_TIMEOUT_MS) {
        tdma_ctx.state = TDMA_UNSYNCED;
        printf("[RADIO] Lost sync (no rx for %lu ms)\r\n",
               (unsigned long)(now - tdma_ctx.last_sync_tick));
        return;
    }

    uint8_t slot = get_current_slot();
    uint32_t time_in_slot = get_time_in_slot();
    tdma_ctx.slot_index = slot;

    static uint8_t last_slot = 255;
    if (slot != last_slot) {
        last_slot = slot;
        tx_done_this_slot = false;
    }

    if (slot_is_tx_fast(slot)) {
        // FC TX (fast telemetry)
        if (time_in_slot >= tx_start_ms && time_in_slot < tx_end_ms) {
            tx_pending_events();
            tx_fast_telemetry(slot);
        }
        if (time_in_slot >= tx_end_ms || tx_done_this_slot) {
            process_rx_data();
        }
    }
    else if (slot_is_slow(slot)) {
        // FC TX slow telemetry — leave guard at end so GS RX slot isn't clobbered
        if (time_in_slot >= tx_start_ms && time_in_slot < (p->slot_ms - rx_guard_ms)) {
            tx_pending_events();
            tx_slow_telemetry(slot);
        }
        if (tx_done_this_slot) {
            process_rx_data();
        }
    }
    else {
        // RX slot — listen for GS commands/sync only
        process_rx_data();
    }
}

static void handle_unsynced_mode(void) {
    uint32_t now = HAL_GetTick();

    // In UNSYNCED mode: mostly listen, occasionally beacon

    // Always try to send pending events (like PONG) even when unsynced
    tx_pending_events();

    // Send a beacon every UNSYNC_BEACON_INTERVAL_MS so GS knows we're alive
    if ((now - last_unsync_beacon_tick) >= UNSYNC_BEACON_INTERVAL_MS) {
        if (!E22_IsBusy()) {
            // Send with slot=0 since we don't know real slot
            telemetry_fast_t pkt;
            if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                telemetry_build_fast(&pkt, tdma_ctx.frame_id, 0,
                                     tdma_ctx.fast_seq++, &fsm_ctx,
                                     &latest_imu, &latest_baro, &latest_bno,
                                     tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
                xSemaphoreGive(sensor_mutex);
            } else {
                telemetry_build_fast(&pkt, tdma_ctx.frame_id, 0,
                                     tdma_ctx.fast_seq++, &fsm_ctx,
                                     NULL, NULL, NULL,
                                     tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
            }

            if (E22_Transmit((uint8_t*)&pkt, sizeof(pkt)) == E22_OK) {
                radio_stats.tx_fast++;
                last_tx_tick = HAL_GetTick();
            }
            last_unsync_beacon_tick = now;
        }
    }

    // Continuously process RX data
    process_rx_data();

    // Keep frame counter advancing (free-running when unsynced)
    if ((now - tdma_ctx.frame_start_tick) >= active_preset()->superframe_ms) {
        tdma_ctx.frame_id++;
        tdma_ctx.frame_start_tick = now;
    }
}

// ============================================================================
// Main Thread
// ============================================================================
void radio_thread_function() {
    printf("[RADIO] Thread starting...\r\n");
    fsm_report_thread_started("RADIO");

    // Initialize
    sensor_mutex = xSemaphoreCreateMutex();
    if (!sensor_mutex) {
        printf("[RADIO] Mutex create failed!\r\n");
    }

    if (!E22_Init(UART_RADIO)) {
        printf("[RADIO] Init FAIL\r\n");
        fsm_report_init_status("RADIO", false);
        vTaskSuspend(NULL);
    }
    fsm_report_init_status("RADIO", true);
    printf("[RADIO] E22 initialized\r\n");
    printf("[RADIO] Packet sizes: FAST=%u SLOW=%u CMD=%u SYNC=%u\r\n",
           sizeof(telemetry_fast_t), sizeof(telemetry_slow_t),
           sizeof(command_packet_t), sizeof(sync_packet_t));

    // Start in UNSYNCED state
    memset(&tdma_ctx, 0, sizeof(tdma_ctx));
    tdma_ctx.state = TDMA_UNSYNCED;
    tdma_ctx.frame_start_tick = HAL_GetTick();
    last_tx_tick = 0;
    last_unsync_beacon_tick = 0;
    tx_done_this_slot = false;

    uint32_t last_stats_tick = HAL_GetTick();

    printf("[RADIO] Waiting for GS sync...\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();

        if (tdma_ctx.state == TDMA_SYNCED) {
            handle_synced_mode();
        } else {
            handle_unsynced_mode();
        }

        // Stats every 10 seconds
        if ((now - last_stats_tick) >= 10000) {
            (void)E22_GetStats();

            printf("[RADIO] %s F=%u slot=%u | TX: f=%lu s=%lu | RX: cmd=%lu sync=%lu | CRC=%lu | ack_seq=%u\r\n",
                   tdma_ctx.state == TDMA_SYNCED ? "SYNC" : "UNSYNC",
                   tdma_ctx.frame_id, tdma_ctx.slot_index,
                   radio_stats.tx_fast, radio_stats.tx_slow,
                   radio_stats.rx_cmd, radio_stats.rx_sync,
                   radio_stats.crc_errors,
                   tdma_ctx.last_cmd_seq_received);

            last_stats_tick = now;
        }

        // Fast loop for responsive operation
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

#else /* RADIO_INTERFACE_SPI */
// ============================================================================
// SX126x SPI Radio Implementation (Buzz V4 PCB)
// ============================================================================

#include "Radio/LORA Drivers/lora_sx126x.h"

// TDMA timing — slot/superframe durations now driven by active_preset() (see shared block).
/* Phase 3-A bugfix: time-based sync timeout instead of frame count.
 * The old "10 frames" scaled poorly between presets — 10 s in flight (fine)
 * but only 2 s in test mode, which trips falsely when the Arduino prioritises
 * test_ctrl over sync (manual flow). 5 s of absolute silence is the trip. */
#define SYNC_TIMEOUT_MS         5000
#define SYNC_CORRECTION_THRESH  10

// TX windows scaled per slot — see UART variant for rationale.
#define TX_START_PCT            5
#define TX_END_PCT              70
#define RX_GUARD_PCT            20

// When UNSYNCED, TX beacon every N seconds
#define UNSYNC_BEACON_INTERVAL_MS  5000

// LoRa configuration — matches ground station E22-900M22T (Waveshare)
// GS config: Ch18, Air rate 62.5kbps, 22dBm, NET ID 23, Addr 0, Key 0
static SX126x_LoRaConfig_t lora_config = {
    .frequency_hz     = 868125000,          // 850.125 + Ch18 = 868.125 MHz
    .spreading_factor = SX126X_LORA_SF5,    // SF5 — 62.5 kbps air rate
    .bandwidth        = SX126X_LORA_BW_500, // 500 kHz
    .coding_rate      = SX126X_LORA_CR_4_5, // 4/5
    .tx_power_dbm     = 22,                 // +22 dBm (matches GS)
    .preamble_length  = 12,                 // EBYTE default
    .sync_word        = 0x1474,             // Derived from NET ID 23 (0x17)
    .enable_crc       = true,
    .invert_iq        = false,
};

// State
tdma_ctx_t tdma_ctx = {0};
radio_stats_t radio_stats = {0};

extern fsm_ctx_t fsm_ctx;

// Latest sensor data
static IMU_t latest_imu = {0};
static BARO_t latest_baro = {0};
static BNO_t latest_bno = {0};
static GPS_t latest_gps = {0};
static SemaphoreHandle_t sensor_mutex = NULL;

// TX tracking
static uint32_t last_tx_tick = 0;
static uint32_t last_unsync_beacon_tick = 0;
static bool tx_done_this_slot = false;

// ============================================================================
// Sensor Data Update
// ============================================================================

/** @copydoc radio_update_sensor_data */
void radio_update_sensor_data(const IMU_t *imu, const BARO_t *baro,
                               const BNO_t *bno, const GPS_t *gps) {
    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (imu) memcpy(&latest_imu, imu, sizeof(IMU_t));
        if (baro) memcpy(&latest_baro, baro, sizeof(BARO_t));
        if (bno) memcpy(&latest_bno, bno, sizeof(BNO_t));
        if (gps) memcpy(&latest_gps, gps, sizeof(GPS_t));
        xSemaphoreGive(sensor_mutex);
    }
}

// ============================================================================
// TDMA Timing
// ============================================================================

/**
 * @brief Get the current TDMA slot index within the superframe
 * @return Slot index (0 to TDMA_SLOTS_PER_FRAME-1)
 */
static uint8_t get_current_slot(void) {
    const tdma_preset_t *p = active_preset();
    uint32_t elapsed = HAL_GetTick() - tdma_ctx.frame_start_tick;
    return (elapsed / p->slot_ms) % p->slots_per_frame;
}

/**
 * @brief Get elapsed time within the current TDMA slot
 * @return Milliseconds elapsed since the current slot started
 */
static uint32_t get_time_in_slot(void) {
    const tdma_preset_t *p = active_preset();
    uint32_t elapsed = HAL_GetTick() - tdma_ctx.frame_start_tick;
    return elapsed % p->slot_ms;
}

/**
 * @brief Advance the TDMA superframe counter and update frame start tick.
 *        Phase 3-A (A4): also applies any pending TDMA mode switch.
 */
static void advance_frame(void) {
    tdma_ctx.frame_id++;
    tdma_ctx.frame_start_tick += active_preset()->superframe_ms;

    if (pending_tdma_mode != current_tdma_mode) {
        printf("[RADIO] TDMA mode: %u -> %u\r\n",
               (unsigned)current_tdma_mode, (unsigned)pending_tdma_mode);
        current_tdma_mode = pending_tdma_mode;
    }
}

/**
 * @brief Check whether a transmission is allowed in the current slot
 * @retval true  SX126x is idle and no TX has occurred this slot
 * @retval false Transmission not allowed (busy or already transmitted)
 */
static bool can_transmit(void) {
    // Don't TX if SX126x is busy with DMA
    if (SX126x_IsTxBusy()) return false;

    // Already transmitted this slot
    if (tx_done_this_slot) return false;

    return true;
}

/**
 * @brief Synchronize TDMA frame timing to a Ground Station sync/command packet
 * @param gs_frame_id Frame ID received from the Ground Station
 * @param gs_time     GS timestamp (currently unused, reserved)
 *
 * @details
 * On first sync, hard-aligns the local frame start tick.
 * When already synced, applies a correction only if the timing error
 * exceeds SYNC_CORRECTION_THRESH milliseconds.
 */
static void sync_to_gs(uint8_t gs_frame_id, uint32_t gs_time) {
    uint32_t now = HAL_GetTick();
    const tdma_preset_t *p = active_preset();
    uint32_t new_frame_start = now - ((uint32_t)p->rx_slot_for_sync * p->slot_ms);

    if (tdma_ctx.state == TDMA_SYNCED) {
        int32_t error = (int32_t)(new_frame_start - tdma_ctx.frame_start_tick);
        if (error > SYNC_CORRECTION_THRESH || error < -SYNC_CORRECTION_THRESH) {
            tdma_ctx.frame_start_tick = new_frame_start;
            radio_stats.sync_corrections++;
        }
    } else {
        tdma_ctx.frame_start_tick = new_frame_start;
        tdma_ctx.state = TDMA_SYNCED;
        printf("[RADIO] SYNCED! frame=%u\r\n", gs_frame_id);
    }

    tdma_ctx.frame_id = gs_frame_id;
    tdma_ctx.last_sync_tick = now;
    tdma_ctx.missed_sync_count = 0;
}

// ============================================================================
// TX Functions
// ============================================================================

/**
 * @brief Build and transmit a fast telemetry packet via SX126x SPI DMA
 * @param slot Current TDMA slot index (included in packet header)
 *
 * @details
 * Acquires sensor_mutex to snapshot latest IMU/baro/BNO data, builds
 * the packet with telemetry_build_fast(), and transmits via SX126x_TransmitDMA().
 * Sets tx_done_this_slot to prevent duplicate transmissions.
 */
static void tx_fast_telemetry(uint8_t slot) {
    if (!can_transmit()) return;

    telemetry_fast_t pkt;

    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        telemetry_build_fast(&pkt, tdma_ctx.frame_id, slot,
                             tdma_ctx.fast_seq++, &fsm_ctx,
                             &latest_imu, &latest_baro, &latest_bno,
                             tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
        xSemaphoreGive(sensor_mutex);
    } else {
        telemetry_build_fast(&pkt, tdma_ctx.frame_id, slot,
                             tdma_ctx.fast_seq++, &fsm_ctx,
                             NULL, NULL, NULL,
                             tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
    }

    if (SX126x_TransmitDMA((uint8_t*)&pkt, sizeof(pkt))) {
        radio_stats.tx_fast++;
        last_tx_tick = HAL_GetTick();
        tx_done_this_slot = true;
    }
}

/**
 * @brief Build and transmit a slow telemetry packet (GPS, baro, battery)
 * @param slot Current TDMA slot index (included in packet header)
 *
 * @details
 * Sent once per superframe in TDMA_SLOW_SLOT. Contains GPS position,
 * barometric altitude, battery voltage, and system flags.
 */
static void tx_slow_telemetry(uint8_t slot) {
    if (!can_transmit()) return;

    telemetry_slow_t pkt;
    uint8_t mode      = (uint8_t)current_tdma_mode;
    uint8_t slow_slot = active_preset()->slow_slot;

    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        telemetry_build_slow(&pkt, tdma_ctx.frame_id, tdma_ctx.slow_seq++,
                             &latest_gps, &latest_baro, 100, 1, mode, slow_slot);
        xSemaphoreGive(sensor_mutex);
    } else {
        telemetry_build_slow(&pkt, tdma_ctx.frame_id, tdma_ctx.slow_seq++,
                             NULL, NULL, 100, 1, mode, slow_slot);
    }

    if (SX126x_TransmitDMA((uint8_t*)&pkt, sizeof(pkt))) {
        radio_stats.tx_slow++;
        last_tx_tick = HAL_GetTick();
        tx_done_this_slot = true;
    }
    (void)slot;
}

// ============================================================================
// Event Transmission
// ============================================================================

/**
 * @brief Transmit any pending FSM event packets (PONG, state changes, etc.)
 *
 * @details
 * Dequeues one event from queue_fsm_events (non-blocking), updates its
 * frame/slot/seq header fields, recalculates CRC, and transmits via
 * SX126x_TransmitDMA().
 */
static void tx_pending_events(void) {
    if (!can_transmit()) return;
    if (queue_fsm_events == NULL) return;

    telemetry_event_t evt;

    if (xQueueReceive(queue_fsm_events, &evt, 0) == pdTRUE) {
        evt.frame_id = tdma_ctx.frame_id;
        evt.slot_id = tdma_ctx.slot_index;
        evt.seq = tdma_ctx.event_seq++;
        evt.crc16 = crc16_calculate((uint8_t*)&evt, sizeof(telemetry_event_t) - 2);

        if (SX126x_TransmitDMA((uint8_t*)&evt, sizeof(evt))) {
            radio_stats.tx_event++;
            last_tx_tick = HAL_GetTick();
            // EVT_THREAD_STAT (23) and EVT_SYSTEM_STAT (24) are emitted ~1Hz —
            // skip the print to avoid log spam. Other events still log.
            if (evt.event_type != EVT_THREAD_STAT && evt.event_type != EVT_SYSTEM_STAT) {
                printf("[RADIO] Event TX: type=%u\r\n", evt.event_type);
            }
        }
    }
}

// ============================================================================
// RX Functions
// ============================================================================

/**
 * @brief Get the expected packet size for a given packet type byte
 * @param type Packet type identifier (first byte of packet)
 * @return Expected packet size in bytes, or 0 if type is unknown
 */
static uint16_t get_packet_size(uint8_t type) {
    switch (type) {
        case TELEM_PACKET_COMMAND:   return sizeof(command_packet_t);
        case TELEM_PACKET_SYNC:      return sizeof(sync_packet_t);
        case TELEM_PACKET_TEST_CTRL: return sizeof(test_control_packet_t);
        default: return 0;
    }
}

/* Phase 3-A (A5): inbound test_control_packet_t (manual test slider input). */
static void process_test_ctrl(test_control_packet_t *ctrl) {
    uint16_t calc_crc = crc16_calculate((uint8_t*)ctrl, sizeof(test_control_packet_t) - 2);
    if (calc_crc != ctrl->crc16) {
        radio_stats.crc_errors++;
        return;
    }
    test_runner_submit_control(ctrl);
}

/**
 * @brief Validate and process a received command packet from the Ground Station
 * @param cmd Pointer to the received command packet
 *
 * @details
 * Verifies CRC-16 integrity, forwards the command to the FSM thread via
 * fsm_send_command(), stores the sequence number for ACK piggyback,
 * and re-synchronizes TDMA timing from the command's frame ID.
 */
static void process_command(command_packet_t *cmd) {
    uint16_t calc_crc = crc16_calculate((uint8_t*)cmd, sizeof(command_packet_t) - 2);
    if (calc_crc != cmd->crc16) {
        radio_stats.crc_errors++;
        return;
    }

    radio_stats.rx_cmd++;
    tdma_ctx.last_cmd_seq_received = cmd->cmd_seq;

    fsm_cmd_msg_t fsm_msg = {0};
    fsm_msg.cmd = (fsm_command_t)cmd->cmd_id;
    fsm_msg.cmd_seq = cmd->cmd_seq;
    memcpy(&fsm_msg.payload, cmd->params, sizeof(cmd->params));

    if (fsm_send_command(fsm_msg.cmd, fsm_msg.cmd_seq, &fsm_msg.payload, sizeof(fsm_msg.payload))) {
        tdma_ctx.last_cmd_status = 0;
        printf("[RADIO] CMD %d -> FSM queue\r\n", cmd->cmd_id);
    } else {
        tdma_ctx.last_cmd_status = 2;
        printf("[RADIO] CMD queue FULL!\r\n");
    }

    sync_to_gs(cmd->frame_id, cmd->time);
}

/**
 * @brief Validate and process a received TDMA sync beacon from the Ground Station
 * @param sync Pointer to the received sync packet
 *
 * @details
 * Verifies CRC-16, then calls sync_to_gs() to align local TDMA timing.
 */
static void process_sync(sync_packet_t *sync) {
    uint16_t calc_crc = crc16_calculate((uint8_t*)sync, sizeof(sync_packet_t) - 2);
    if (calc_crc != sync->crc16) {
        radio_stats.crc_errors++;
        return;
    }

    radio_stats.rx_sync++;
    sync_to_gs(sync->frame_id, sync->gs_time);
}

/**
 * @brief Poll the SX126x for received packets and dispatch by type
 *
 * @details
 * Checks SX126x_Available(), reads the packet via SX126x_Receive(),
 * validates the packet type and minimum length, then dispatches to
 * process_command() or process_sync() accordingly.
 */
static void process_rx_data(void) {
    if (!SX126x_Available()) return;

    uint8_t rx_buf[RADIO_MAX_PACKET_SIZE];
    SX126x_RxInfo_t rx_info;
    int len = SX126x_Receive(rx_buf, sizeof(rx_buf), &rx_info);

    if (len <= 0) return;

    uint8_t type = rx_buf[0];
    uint16_t expected = get_packet_size(type);

    if (expected == 0 || len < expected) return;

    if (type == TELEM_PACKET_COMMAND) {
        process_command((command_packet_t*)rx_buf);
    } else if (type == TELEM_PACKET_SYNC) {
        process_sync((sync_packet_t*)rx_buf);
    } else if (type == TELEM_PACKET_TEST_CTRL) {
        process_test_ctrl((test_control_packet_t*)rx_buf);
    }
}

// ============================================================================
// Mode Handlers
// ============================================================================

/**
 * @brief Run one iteration of the TDMA state machine in SYNCED mode
 *
 * @details
 * Checks for superframe rollover, monitors sync timeout, then executes
 * slot-specific behaviour: fast TX (slots 0-7), slow TX (slot 8), or
 * RX-only (slot 9). Calls tx_pending_events() before regular TX to
 * prioritize event packets (PONG, state changes).
 */
static void handle_synced_mode(void) {
    uint32_t now = HAL_GetTick();
    const tdma_preset_t *p = active_preset();
    uint32_t tx_start_ms = (p->slot_ms * TX_START_PCT) / 100;
    uint32_t tx_end_ms   = (p->slot_ms * TX_END_PCT)   / 100;
    uint32_t rx_guard_ms = (p->slot_ms * RX_GUARD_PCT) / 100;

    if ((now - tdma_ctx.frame_start_tick) >= p->superframe_ms) {
        advance_frame();
        p = active_preset();
        tx_start_ms = (p->slot_ms * TX_START_PCT) / 100;
        tx_end_ms   = (p->slot_ms * TX_END_PCT)   / 100;
        rx_guard_ms = (p->slot_ms * RX_GUARD_PCT) / 100;
    }

    // Phase 3-A bugfix: absolute-time sync timeout (mode-independent).
    if ((now - tdma_ctx.last_sync_tick) > SYNC_TIMEOUT_MS) {
        tdma_ctx.state = TDMA_UNSYNCED;
        printf("[RADIO] Lost sync (no rx for %lu ms)\r\n",
               (unsigned long)(now - tdma_ctx.last_sync_tick));
        return;
    }

    uint8_t slot = get_current_slot();
    uint32_t time_in_slot = get_time_in_slot();
    tdma_ctx.slot_index = slot;

    static uint8_t last_slot = 255;
    if (slot != last_slot) {
        last_slot = slot;
        tx_done_this_slot = false;
    }

    if (slot_is_tx_fast(slot)) {
        if (time_in_slot >= tx_start_ms && time_in_slot < tx_end_ms) {
            tx_pending_events();
            tx_fast_telemetry(slot);
        }
        if (time_in_slot >= tx_end_ms || tx_done_this_slot) {
            process_rx_data();
        }
    }
    else if (slot_is_slow(slot)) {
        if (time_in_slot >= tx_start_ms && time_in_slot < (p->slot_ms - rx_guard_ms)) {
            tx_pending_events();
            tx_slow_telemetry(slot);
        }
        if (tx_done_this_slot) {
            process_rx_data();
        }
    }
    else {
        // RX slot — listen for GS commands/sync only
        process_rx_data();
    }
}

/**
 * @brief Run one iteration of the TDMA state machine in UNSYNCED mode
 *
 * @details
 * Transmits pending events and a periodic beacon (every UNSYNC_BEACON_INTERVAL_MS)
 * so the Ground Station can detect the FC. Continuously polls for RX data
 * to acquire the first sync packet. Frame counter advances in free-running mode.
 */
static void handle_unsynced_mode(void) {
    uint32_t now = HAL_GetTick();

    tx_pending_events();

    // Send beacon periodically
    if ((now - last_unsync_beacon_tick) >= UNSYNC_BEACON_INTERVAL_MS) {
        if (!SX126x_IsTxBusy()) {
            telemetry_fast_t pkt;
            if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                telemetry_build_fast(&pkt, tdma_ctx.frame_id, 0,
                                     tdma_ctx.fast_seq++, &fsm_ctx,
                                     &latest_imu, &latest_baro, &latest_bno,
                                     tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
                xSemaphoreGive(sensor_mutex);
            } else {
                telemetry_build_fast(&pkt, tdma_ctx.frame_id, 0,
                                     tdma_ctx.fast_seq++, &fsm_ctx,
                                     NULL, NULL, NULL,
                                     tdma_ctx.last_cmd_seq_received, tdma_ctx.last_cmd_status);
            }

            if (SX126x_TransmitDMA((uint8_t*)&pkt, sizeof(pkt))) {
                radio_stats.tx_fast++;
                last_tx_tick = HAL_GetTick();
            }
            last_unsync_beacon_tick = now;
        }
    }

    process_rx_data();

    if ((now - tdma_ctx.frame_start_tick) >= active_preset()->superframe_ms) {
        tdma_ctx.frame_id++;
        tdma_ctx.frame_start_tick = now;
    }
}

// ============================================================================
// Main Thread
// ============================================================================

/** @copydoc radio_thread_function */
void radio_thread_function() {
    printf("[RADIO] SPI thread starting...\r\n");
    fsm_report_thread_started("RADIO");

    sensor_mutex = xSemaphoreCreateMutex();
    if (!sensor_mutex) {
        printf("[RADIO] Mutex create failed!\r\n");
    }

    // Initialize SX126x with LoRa config
    if (!SX126x_Init(&lora_config)) {
        printf("[RADIO] SX126x Init FAIL\r\n");
        fsm_report_init_status("RADIO", false);
        vTaskSuspend(NULL);
    }
    fsm_report_init_status("RADIO", true);

    printf("[RADIO] SX126x initialized — freq=%luHz SF=%u BW=%u\r\n",
           lora_config.frequency_hz,
           lora_config.spreading_factor,
           lora_config.bandwidth);
    printf("[RADIO] Packet sizes: FAST=%u SLOW=%u CMD=%u SYNC=%u\r\n",
           sizeof(telemetry_fast_t), sizeof(telemetry_slow_t),
           sizeof(command_packet_t), sizeof(sync_packet_t));

    // Enter continuous RX mode
    SX126x_SetRx(0xFFFFFF);

    // Start in UNSYNCED state
    memset(&tdma_ctx, 0, sizeof(tdma_ctx));
    tdma_ctx.state = TDMA_UNSYNCED;
    tdma_ctx.frame_start_tick = HAL_GetTick();
    last_tx_tick = 0;
    last_unsync_beacon_tick = 0;
    tx_done_this_slot = false;

    uint32_t last_stats_tick = HAL_GetTick();

    printf("[RADIO] Waiting for GS sync...\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();

        if (tdma_ctx.state == TDMA_SYNCED) {
            handle_synced_mode();
        } else {
            handle_unsynced_mode();
        }

        // Stats every 10 seconds
        if ((now - last_stats_tick) >= 10000) {
            printf("[RADIO] %s F=%u slot=%u | TX: f=%lu s=%lu | RX: cmd=%lu sync=%lu | CRC=%lu | RSSI=%d SNR=%d\r\n",
                   tdma_ctx.state == TDMA_SYNCED ? "SYNC" : "UNSYNC",
                   tdma_ctx.frame_id, tdma_ctx.slot_index,
                   radio_stats.tx_fast, radio_stats.tx_slow,
                   radio_stats.rx_cmd, radio_stats.rx_sync,
                   radio_stats.crc_errors,
                   SX126x_GetRssi(), SX126x_GetSnr());
            last_stats_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

#endif /* RADIO_INTERFACE_UART / SPI */
