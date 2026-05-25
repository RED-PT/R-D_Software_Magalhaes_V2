/**
 * @file create_threads.c
 * @brief FreeRTOS Thread Creation Implementation
 * @author Tomás Teixeira (texman)
 * @date October 6, 2025
 * @version 2.0
 *
 * @details
 * This file implements the thread creation and resource initialization for
 * the Magalhães Flight Computer. It sets up all FreeRTOS threads, queues,
 * timers, and stream buffers required for system operation.
 *
 * ## Memory Allocation
 * All FreeRTOS objects use static allocation from the FreeRTOS heap.
 * Total estimated memory usage:
 * - Thread stacks: ~36 KB
 * - Queues: ~2 KB
 * - Timers: ~200 bytes
 *
 * ## Thread Stack Sizes
 * Stack sizes are tuned for each thread's requirements:
 * - Sensors: 6144 bytes (sensor drivers, DMA handling)
 * - DataHandler: 4096 bytes (queue operations)
 * - Estimator: 4096 bytes (floating-point calculations)
 * - Controller: 4096 bytes (control loop, PWM)
 * - Telemetry: 6144 bytes (packet building)
 * - Radio: 6144 bytes (LoRa driver, buffers)
 * - SDCard: 6144 bytes (FatFS operations)
 * - FSM: 4096 bytes (state machine)
 *
 * @see create_threads.h for interface documentation
 *
 * @ingroup Thread_Management
 */

#include "create_threads.h"
#include "print.h"
#include <stdio.h>

/* ============================================================================
 * Thread Handle Definitions
 * ============================================================================ */

/** @name Thread Handles
 *  @brief Global thread handles for inter-thread reference
 *  @{
 */
osThreadId_t sensors_thread_id = NULL;
osThreadId_t data_handler_thread_id = NULL;
osThreadId_t estimator_thread_id = NULL;
osThreadId_t controller_thread_id = NULL;
osThreadId_t sd_card_thread_id = NULL;
osThreadId_t telemetry_thread_id = NULL;
osThreadId_t radio_thread_id = NULL;
osThreadId_t fsm_thread_id = NULL;
/** @} */

/* ============================================================================
 * Timer Handle Definitions
 * ============================================================================ */

/** @name Timer Handles
 *  @{
 */
TimerHandle_t xBaroTimer = NULL;  /**< Triggers barometer reading at 50 Hz */
TimerHandle_t xBnoTimer = NULL;   /**< Triggers BNO055 reading at 100 Hz */
/** @} */

/* ============================================================================
 * Queue Handle Definitions
 * ============================================================================ */

/** @name Queue Handles
 *  @{
 */
QueueHandle_t queue_to_radio_tx = NULL;     /**< Telemetry → Radio (tx packets) */
QueueHandle_t queue_radio_rx_to_fsm = NULL; /**< Radio → FSM (raw rx, deprecated) */
QueueHandle_t queue_fsm_events = NULL;      /**< FSM → Radio (event packets, 5 items) */
QueueHandle_t queue_cmd_to_fsm = NULL;      /**< Radio → FSM (parsed commands, 8 items) */
QueueHandle_t queue_event_to_fsm = NULL;    /**< Estimator → FSM (internal events, 8 items) */
/** @} */

/* ============================================================================
 * Stream Buffer Definitions
 * ============================================================================ */

/** @brief GPS NMEA data stream buffer */
StreamBufferHandle_t stream_buffer_gps;

/* ============================================================================
 * Thread Attribute Definitions
 * ============================================================================ */

/**
 * @brief Sensors thread configuration
 * @details Highest priority - sensor data-ready interrupts must be handled
 *          immediately to prevent data loss. Large stack for driver buffers.
 */
const osThreadAttr_t sensors_thread_attr = {
    .name = "Sensors",
    .stack_size = 1536 * 4,
    .priority = osPriorityHigh3      // Highest - must read sensors immediately
};

/**
 * @brief Data handler thread configuration
 * @details Second highest priority - buffers must be flushed to queues
 *          before they overflow.
 */
const osThreadAttr_t data_handler_thread_attr = {
    .name = "DataHandler",
    .stack_size = 1024 * 4,
    .priority = osPriorityHigh2,     // Second - flush buffers to queues
};

/**
 * @brief Estimator thread configuration
 * @details High priority for real-time state estimation. Runs at 100 Hz.
 *          Requires stack for floating-point Kalman filter operations.
 */
const osThreadAttr_t estimator_thread_attr = {
    .name = "Estimator",
    .stack_size = 1024 * 4,
    .priority = osPriorityHigh,      // Real-time control
};

/**
 * @brief Controller thread configuration
 * @details High priority for real-time motor control. Runs at 100 Hz.
 *          Manages PWM output and control loop calculations.
 */
const osThreadAttr_t controller_thread_attr = {
    .name = "Controller",
    .stack_size = 1024 * 4,
    .priority = osPriorityHigh,      // Real-time control
};

/**
 * @brief Telemetry thread configuration
 * @details Above normal priority for packet building. Synchronizes with
 *          TDMA slot timing. Large stack for packet structures.
 */
const osThreadAttr_t telemetry_thread_attr = {
    .name = "Telemetry",
    .stack_size = 1536 * 4,          // INCREASED for radio operations
    .priority = osPriorityAboveNormal,
};

/**
 * @brief Radio thread configuration
 * @details High priority for command reception and TDMA timing.
 *          Must respond quickly to incoming commands.
 */
const osThreadAttr_t radio_thread_attr = {
    .name = "Radio",
    .stack_size = 1536 * 4,          // Radio RX/TX handling
    .priority = osPriorityHigh1,     // HIGH priority for command reception
};

/**
 * @brief SD card thread configuration
 * @details Normal priority - SD card I/O is slow and non-critical.
 *          Large stack for FatFS file operations.
 */
const osThreadAttr_t sd_card_thread_attr = {
    .name = "SDCard",
    .stack_size = 1536 * 4,          // INCREASED for FatFS operations
    .priority = osPriorityNormal,    // Lower than telemetry (SD is slow)
};

/**
 * @brief FSM thread configuration
 * @details Above normal priority for state machine decisions.
 *          Must process commands and events promptly.
 */
const osThreadAttr_t fsm_thread_attr = {
    .name = "FSM",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal1,
};

/* ============================================================================
 * Thread Creation Function
 * ============================================================================ */

/**
 * @brief Create all system threads, queues, and timers
 *
 * @details
 * Initializes the complete FreeRTOS environment for the flight computer.
 * This function is called from the CubeMX-generated default task and
 * terminates that task after creating the application threads.
 *
 * **Initialization Sequence:**
 * 1. Terminate default CubeMX task
 * 2. Create inter-thread queues
 * 3. Create software timers for periodic sensors
 * 4. Create threads in dependency order
 *
 * **Queue Specifications:**
 * | Queue | Depth | Item Size | Purpose |
 * |-------|-------|-----------|---------|
 * | queue_fsm_events | 5 | ~60 bytes | FSM events to radio |
 * | queue_cmd_to_fsm | 8 | ~32 bytes | Commands to FSM |
 * | queue_event_to_fsm | 8 | ~24 bytes | Internal events |
 *
 * **Timer Specifications:**
 * | Timer | Period | Callback | Purpose |
 * |-------|--------|----------|---------|
 * | xBaroTimer | 20ms | vBaroTimerCallback | MS5607 50 Hz polling |
 * | xBnoTimer | 10ms | vBnoTimerCallback | BNO055 100 Hz polling |
 *
 * @warning If any resource creation fails, an error is logged but execution
 *          continues. Check serial output for "ERROR:" messages.
 *
 * @note The Estimator and Controller threads are created but start suspended.
 *       They are resumed by the FSM when entering ARMED state.
 */
void create_threads() {

	// Terminate CubeMX default task to free resources
	osThreadTerminate(defaultTaskHandle);

	printf("Creating queues...\r\n");

	// FSM events queue: 5 events * ~60 bytes = ~300 bytes
	queue_fsm_events = xQueueCreate(5, sizeof(telemetry_event_t));
	if (queue_fsm_events == NULL) {
		printf("ERROR: Failed to create queue_fsm_events\r\n");
	}

    queue_cmd_to_fsm = xQueueCreate(8, sizeof(fsm_cmd_msg_t));
    if (queue_cmd_to_fsm == NULL) {
        printf("ERROR: Failed to create queue_cmd_to_fsm\r\n");
    }

    queue_event_to_fsm = xQueueCreate(8, sizeof(fsm_event_msg_t));
    if (queue_event_to_fsm == NULL) {
        printf("ERROR: Failed to create queue_event_to_fsm\r\n");
    }

	printf("Queues created successfully\r\n");

	printf("Creating timers...\r\n");

    xBaroTimer = xTimerCreate("BaroTimer", pdMS_TO_TICKS(BARO_UPDATE_RATE_MS), pdTRUE, NULL, vBaroTimerCallback);
    xBnoTimer = xTimerCreate("BnoTimer", pdMS_TO_TICKS(BNO_UPDATE_RATE_MS), pdTRUE, NULL, vBnoTimerCallback);

    if (xBaroTimer == NULL || xBnoTimer == NULL) {
    	printf("ERROR: Failed to create timers\r\n");
    }

	printf("Creating threads...\r\n");

	sd_card_thread_id = osThreadNew(sd_card_thread_function, NULL, &sd_card_thread_attr);
	if (sd_card_thread_id == NULL) {
		printf("ERROR: SD Card thread creation failed\r\n");
	}

	data_handler_thread_id = osThreadNew(data_handler_thread_function, NULL, &data_handler_thread_attr);
	if (data_handler_thread_id == NULL) {
		printf("ERROR: Data Handler thread creation failed\r\n");
	}

	sensors_thread_id = osThreadNew(sensors_thread_function, NULL, &sensors_thread_attr);
	if (sensors_thread_id == NULL) {
		printf("ERROR: Sensors thread creation failed\r\n");
	}

	fsm_thread_id = osThreadNew(fsm_thread_function, NULL, &fsm_thread_attr);
	if (fsm_thread_id == NULL) {
		printf("ERROR: FSM thread creation failed\r\n");
	}

	estimator_thread_id = osThreadNew(estimator_thread_function, NULL, &estimator_thread_attr);
	if (estimator_thread_id == NULL) {
		printf("ERROR: Estimator thread creation failed\r\n");
	}

	controller_thread_id = osThreadNew(controller_thread_function, NULL, &controller_thread_attr);
	if (controller_thread_id == NULL) {
		printf("ERROR: Controller thread creation failed\r\n");
	}

	telemetry_thread_id = osThreadNew(telemetry_thread_function, NULL, &telemetry_thread_attr);
	if (telemetry_thread_id == NULL) {
		printf("ERROR: Telemetry thread creation failed\r\n");
	}

	radio_thread_id = osThreadNew(radio_thread_function, NULL, &radio_thread_attr);
	if (radio_thread_id == NULL) {
		printf("ERROR: Radio thread creation failed\r\n");
	}

	printf("All threads created successfully\r\n");

	/* Start ESC PWM timer using board-agnostic macros from config.h
	 * (F446ZE: TIM3/CH1, Buzz V4 H743: TIM4/CH1) */
	HAL_TIM_PWM_Start(PWM_ESC_TIM, PWM_ESC_CHANNEL);
	PWM_ESC_CHANNEL_WRITE = 0;

}
