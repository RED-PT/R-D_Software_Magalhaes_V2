/**
 * @file radio_thread.c
 * @brief LoRa Radio Communication Thread Implementation (TDMA v3)
 * @author Tomás Teixeira
 * @date 2025
 * @version 3.0
 *
 * @details
 * Implements the LoRa radio communication system for the Magalhães Flight
 * Computer using Time Division Multiple Access (TDMA) synchronization with
 * the Ground Station.
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

#ifdef RADIO_INTERFACE_UART
// ============================================================================
// E22 UART Radio Implementation (F446ZE dev board)
// ============================================================================

// TDMA timing
#define SLOT_DURATION_MS        TDMA_SLOT_MS
#define FRAME_DURATION_MS       TDMA_SUPERFRAME_MS
#define SYNC_TIMEOUT_FRAMES     10
#define SYNC_CORRECTION_THRESH  10
#define E22_TURNAROUND_MS       30      // E22 TX->RX turnaround time

// TX timing within slot
#define TX_START_OFFSET_MS      5       // Start TX 5ms into slot
#define TX_END_OFFSET_MS        70      // Stop TX 70ms into slot (leave 30ms guard)
#define RX_SLOT_GUARD_MS        20      // Don't TX in last 20ms of slot 8

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
    uint32_t elapsed = HAL_GetTick() - tdma_ctx.frame_start_tick;
    return (elapsed / SLOT_DURATION_MS) % TDMA_SLOTS_PER_FRAME;
}

static uint32_t get_time_in_slot(void) {
    uint32_t elapsed = HAL_GetTick() - tdma_ctx.frame_start_tick;
    return elapsed % SLOT_DURATION_MS;
}

static void advance_frame(void) {
    tdma_ctx.frame_id++;
    tdma_ctx.frame_start_tick += FRAME_DURATION_MS;
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

    // When we receive a sync, we know GS sent it at the START of slot 9
    // Align our frame so slot 9 starts at this moment
    uint32_t new_frame_start = now - (TDMA_RX_SLOT * SLOT_DURATION_MS);

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

    if (sensor_mutex && xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        telemetry_build_slow(&pkt, tdma_ctx.frame_id, tdma_ctx.slow_seq++,
                             &latest_gps, &latest_baro, 100, 1);
        xSemaphoreGive(sensor_mutex);
    } else {
        telemetry_build_slow(&pkt, tdma_ctx.frame_id, tdma_ctx.slow_seq++,
                             NULL, NULL, 100, 1);
    }

    if (E22_Transmit((uint8_t*)&pkt, sizeof(pkt)) == E22_OK) {
        radio_stats.tx_slow++;
        last_tx_tick = HAL_GetTick();
        tx_done_this_slot = true;
    }
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
            printf("[RADIO] Event TX: type=%u\r\n", evt.event_type);
        }
    }
}

// ============================================================================
// RX Functions
// ============================================================================
static uint16_t get_packet_size(uint8_t type) {
    switch (type) {
        case TELEM_PACKET_COMMAND: return sizeof(command_packet_t);
        case TELEM_PACKET_SYNC:    return sizeof(sync_packet_t);
        default: return 0;
    }
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

    // Check for frame rollover
    if ((now - tdma_ctx.frame_start_tick) >= FRAME_DURATION_MS) {
        advance_frame();

        // Check sync timeout
        tdma_ctx.missed_sync_count++;
        if (tdma_ctx.missed_sync_count >= SYNC_TIMEOUT_FRAMES) {
            tdma_ctx.state = TDMA_UNSYNCED;
            printf("[RADIO] Lost sync!\r\n");
            return;
        }
    }

    // Get current slot and time within slot
    uint8_t slot = get_current_slot();
    uint32_t time_in_slot = get_time_in_slot();

    // Update global slot_index for other code that needs it
    tdma_ctx.slot_index = slot;

    // Detect slot change -> reset tx_done flag
    static uint8_t last_slot = 255;
    if (slot != last_slot) {
        last_slot = slot;
        tx_done_this_slot = false;
    }

    // Slot behavior based on current slot
    if (slot < TDMA_TX_SLOTS) {
        // Slots 0-7: Fast telemetry
        // TX window: 5ms to 70ms into slot
        if (time_in_slot >= TX_START_OFFSET_MS && time_in_slot < TX_END_OFFSET_MS) {
            // First check for pending events (high priority - PONG, etc.)
            tx_pending_events();
            // Then send regular fast telemetry if we can still transmit
            tx_fast_telemetry(slot);
        }
        // RX in second half of slot (after TX done)
        if (time_in_slot >= TX_END_OFFSET_MS || tx_done_this_slot) {
            process_rx_data();
        }
    }
    else if (slot == TDMA_SLOW_SLOT) {
        // Slot 8: Slow telemetry
        // TX early, but stop before slot 9 to avoid collision with GS
        if (time_in_slot >= TX_START_OFFSET_MS && time_in_slot < (SLOT_DURATION_MS - RX_SLOT_GUARD_MS)) {
            // First check for pending events
            tx_pending_events();
            tx_slow_telemetry(slot);
        }
        // RX after TX
        if (tx_done_this_slot) {
            process_rx_data();
        }
    }
    else {
        // Slot 9: RX only - listen for GS commands/sync
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
    if ((now - tdma_ctx.frame_start_tick) >= FRAME_DURATION_MS) {
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
// SX126x SPI Radio Implementation (Buzz V4 PCB) — TODO
// ============================================================================

void radio_thread_function() {
    printf("[RADIO] SPI radio thread — not yet implemented\r\n");
    fsm_report_thread_started("RADIO");
    fsm_report_init_status("RADIO", false);
    vTaskSuspend(NULL);
}

#endif /* RADIO_INTERFACE_UART / SPI */
