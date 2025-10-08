/*
 * threads.c
 *
 *  Created on: Oct 8, 2025
 *      Author: texman
 */

// -- STANDARD libraries--
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "math.h"

// -- Codigo --
#include "threads.h"
#include "create_threads.h"
#include "config.h"
#include "Flight Computer/flight_computer.h"
#include "defs.h"
#include "dma_msg.h"
#include "test.h"

// -- Modular Drivers --
#include "flags.h"
#include "malloc.h"
#include "buffer_read_write.h"
#include "conditional_variable.h"
#include "retarget.h"

// -- sensors --
// Altimeter
#include "Sensors/MS5607/MS5607SPI.h"

// GPS
#include "Sensors/GPS/gps.h"

// -- UBLOX --
DMA_Event_t dma_ublox_gps_event = { 0, 0, DMA_GPS_BUF_SIZE };
char dma_ublox_gps_aux_buffer[DMA_GPS_BUF_SIZE]; /* Circular buffer used for DMA */
char data_ublox_gps[DMA_GPS_BUF_SIZE]; /* Data buffer that contains newly received data */

// MEGA BUFFER
#define SIZE_BUFFER_DATA 2 // ~ 4096 bytes (sd card sector size)
#define MAX_ENTRIES 5

// COMMUNICATION PROTOCOL
#define MESSAGE_GS_SIZE 300 // Full Message size
#define STREAM_BUFFER_SIZE 200

// MESSAGE BUFFERS
size_t bytes_sent;
size_t bytes_received;
temperature_readings_t temp_data;
uint8_t data_buffer[60] = "\0";

// Threads Functions
void ublox_gps_function() {

	while (!get_flag_value(&flag_sd_card_thread_ready)) {
	}
	printf("ola da thread do ublox\n");
	UART_HandleTypeDef *huart_ublox_gps = UART_UBLOX;
	if (HAL_UART_Receive_DMA(huart_ublox_gps,
			(uint8_t*) dma_ublox_gps_aux_buffer, DMA_GPS_BUF_SIZE) != HAL_OK) {
		printf("DMA ublox_gps danificado\n\r");
		Error_Handler();
	}
	__HAL_DMA_DISABLE_IT(huart_ublox_gps->hdmarx, DMA_IT_HT);

	//DMA UBLOX
	// dma ublox_gps
	char dma_ublox_gps_aux_buffer[DMA_GPS_AUX_BUF_SIZE + 1] = "\0";
	char dma_msg_restored[DMA_GPS_BUF_SIZE + 1] = "\0";
	bool flag_dma_ublox_gps_aux_buf_has_key = false;
	uint16_t dma_ublox_gps_aux_offset = 0;

	printf("Thread ublox_gps inicializada!\r\n");
	set_true_flag_value(&flag_status_ublox_gps);
	while (1) {
		while (!get_flag_value(&flag_new_data_dma_ublox_gps))
			wait_thread(&ublox_gps_thread_id);

		set_false_flag_value(&flag_new_data_dma_ublox_gps);

		process_dma_buffer(dma_msg_restored, data_ublox_gps,
				dma_ublox_gps_event.length, "$GN", 3, dma_ublox_gps_aux_buffer,
				DMA_GPS_AUX_BUF_SIZE, &flag_dma_ublox_gps_aux_buf_has_key,
				&dma_ublox_gps_aux_offset); // MUDAR "$Gx" DEPENDENDO DA CONFIGURAÇÃO
		printf("DATA UBLOX GPS : %s \nDMA buffer restored: %s \n",
				data_ublox_gps, dma_msg_restored);
		// save to struct
		GPS_save_data(&ublox_gps, dma_msg_restored);
		ublox_gps.timestamp_ms = time_in_millis;
//		printf("DMA buffer saved\n");

		ublox_gps.timestamp_ms = time_in_millis;

		//	memset(dma_msg_restored, 0, DMA_GPS_BUF_SIZE + 1);

		// debugging - usar para definir DMA_GPS_BUF_SIZE
		//	if (DEBUG_GPS)
		printf("%s\r\n", dma_msg_restored);
	}

}

