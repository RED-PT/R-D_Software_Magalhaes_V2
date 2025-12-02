// Includes
#include "create_threads.h"
#include "print.h"
#include <stdio.h>

// Thread IDs
osThreadId_t sensors_thread_id = NULL;
osThreadId_t data_handler_thread_id = NULL;
osThreadId_t estimator_thread_id = NULL;
osThreadId_t controller_thread_id = NULL;
osThreadId_t sd_card_thread_id = NULL;
osThreadId_t telemetry_thread_id = NULL;
osThreadId_t radio_thread_id = NULL;
osThreadId_t fsm_thread_id = NULL;

// Timers
TimerHandle_t xBaroTimer = NULL;
TimerHandle_t xBnoTimer = NULL;

// Queue Handles
QueueHandle_t queue_to_radio_tx = NULL;
QueueHandle_t queue_radio_rx_to_fsm = NULL;
QueueHandle_t queue_fsm_events = NULL;
QueueHandle_t queue_cmd_to_fsm = NULL;
QueueHandle_t queue_event_to_fsm = NULL;
// Stream Buffer
StreamBufferHandle_t stream_buffer_gps;

// Thread Attributes
const osThreadAttr_t sensors_thread_attr = {
    .name = "Sensors",
    .stack_size = 1536 * 4,
    .priority = osPriorityHigh3      // Highest - must read sensors immediately
};

const osThreadAttr_t data_handler_thread_attr = {
    .name = "DataHandler",
    .stack_size = 1024 * 4,
    .priority = osPriorityHigh2,     // Second - flush buffers to queues
};

const osThreadAttr_t estimator_thread_attr = {
    .name = "Estimator",
    .stack_size = 1024 * 4,
    .priority = osPriorityHigh,      // Real-time control
};

const osThreadAttr_t controller_thread_attr = {
    .name = "Controller",
    .stack_size = 1024 * 4,
    .priority = osPriorityHigh,      // Real-time control
};

const osThreadAttr_t telemetry_thread_attr = {
    .name = "Telemetry",
    .stack_size = 1536 * 4,          // INCREASED for radio operations
    .priority = osPriorityAboveNormal,
};

const osThreadAttr_t radio_thread_attr = {
    .name = "Radio",
    .stack_size = 1536 * 4,          // Radio RX/TX handling
    .priority = osPriorityHigh1,     // HIGH priority for command reception
};

const osThreadAttr_t sd_card_thread_attr = {
    .name = "SDCard",
    .stack_size = 1536 * 4,          // INCREASED for FatFS operations
    .priority = osPriorityNormal,    // Lower than telemetry (SD is slow)
};

const osThreadAttr_t fsm_thread_attr = {
    .name = "FSM",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal1,
};

// Functions
void create_threads() {

	// Terminate Task chata FreeRTOS
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
}
