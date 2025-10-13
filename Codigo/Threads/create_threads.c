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
osThreadId_t fsm_thread_id = NULL;
osThreadId_t ublox_gps_thread_id = NULL;
//osThreadId_t radio_rx_thread_id = NULL;

// Timers
TimerHandle_t xBaroTimer = NULL;
TimerHandle_t xBnoTimer = NULL;

// Threads Attributes
const osThreadAttr_t sensors_thread_attr = {
    .name = "Sensors",
    .stack_size = 2048 * 4,
    .priority = osPriorityRealtime,  //???
};

const osThreadAttr_t data_handler_thread_attr = {
    .name = "DataHandler",
    .stack_size = 1024 * 4,
    .priority = osPriorityAboveNormal,
};

const osThreadAttr_t estimator_thread_attr = {
    .name = "Estimator",
    .stack_size = 2048 * 4,
    .priority = osPriorityHigh,
};

const osThreadAttr_t controller_thread_attr = {
    .name = "Controller",
    .stack_size = 2048 * 4,
    .priority = osPriorityHigh,
};

const osThreadAttr_t sd_card_thread_attr = {
    .name = "SDCard",
    .stack_size = 2048 * 4,
    .priority = osPriorityLow,
};

const osThreadAttr_t telemetry_thread_attr = {
    .name = "Telemetry",
    .stack_size = 1536 * 4,
    .priority = osPriorityNormal,
};

const osThreadAttr_t fsm_thread_attr = {
    .name = "FSM",
    .stack_size = 1536 * 4,
    .priority = osPriorityAboveNormal1,
};

const osThreadAttr_t ublox_gps_thread_attr = {
    .name = "GPS",
    .stack_size = 1536 * 4,
    .priority = osPriorityNormal,
};

/*
const osThreadAttr_t radio_rx_thread_attr = {
    .name = "RadioRX",
    .stack_size = 1536 * 4,
    .priority = osPriorityAboveNormal,  // Important for command reception
};
*/



// Stream Buffer
StreamBufferHandle_t stream_buffer_gps;

// Functions
void create_threads() {

	// Terminate Task chata FreeRTOS
	osThreadTerminate(defaultTaskHandle);

	printf("Creating timers...\n");

    xBaroTimer = xTimerCreate("BaroTimer",pdMS_TO_TICKS(BARO_UPDATE_RATE_MS),
                              pdTRUE,  // Auto-reload
                              NULL,
                              vBaroTimerCallback);

    xBnoTimer = xTimerCreate("BnoTimer", pdMS_TO_TICKS(BNO_UPDATE_RATE_MS),
                             pdTRUE,  // Auto-reload
                             NULL,
                             vBnoTimerCallback);

    if (xBaroTimer == NULL || xBnoTimer == NULL) {printf("ERROR: Failed to create timers\n");}

	printf("Creating threads...\n");

	sensors_thread_id = osThreadNew(sensors_thread_function, NULL, &sensors_thread_attr);
	if (sensors_thread_id == NULL) {printf("ERROR: Sensor Fusion thread creation failed\n");}

	data_handler_thread_id = osThreadNew(data_handler_thread_function, NULL, &data_handler_thread_attr);
	if (data_handler_thread_id == NULL) { printf("ERROR: Data Handler thread creation failed\n");}

	estimator_thread_id = osThreadNew(estimator_thread_function, NULL, &estimator_thread_attr);
	if (estimator_thread_id == NULL) {printf("ERROR: Estimator thread creation failed\n");}

	controller_thread_id = osThreadNew(controller_thread_function, NULL, &controller_thread_attr);
	if (controller_thread_id == NULL) {printf("ERROR: Controller thread creation failed\n");}

	sd_card_thread_id = osThreadNew(sd_card_thread_function, NULL, &sd_card_thread_attr);
	if (sd_card_thread_id == NULL) {printf("ERROR: SD Card thread creation failed\n");}

	telemetry_thread_id = osThreadNew(telemetry_thread_function, NULL, &telemetry_thread_attr);
	if (telemetry_thread_id == NULL) {printf("ERROR: Telemetry thread creation failed\n");}

	fsm_thread_id = osThreadNew(fsm_thread_function, NULL, &fsm_thread_attr);
	if (fsm_thread_id == NULL) {printf("ERROR: FSM thread creation failed\n");}

	ublox_gps_thread_id = osThreadNew(ublox_gps_thread_function, NULL, &ublox_gps_thread_attr);
	if (ublox_gps_thread_id == NULL) {printf("ERROR: GPS thread creation failed\n");}

	//radio_rx_thread_id = osThreadNew(radio_rx_thread_function, NULL, &radio_rx_thread_attr);
	//if (radio_rx_thread_id == NULL) {printf("ERROR: Radio RX thread creation failed\n");}

	printf("\nAll threads created successfully");

}


