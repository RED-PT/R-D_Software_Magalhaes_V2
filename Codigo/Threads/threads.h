/*
 * threads.h
 *
 *  Created on: Oct 8, 2025
 *      Author: texman
 */

#ifndef THREADS_THREADS_H_
#define THREADS_THREADS_H_

// C libraries
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>

// Codigo folder library
#include "create_threads.h"
#include "dma_msg.h"

// Threads Functions
void ublox_gps_function();

// GPS
#include "Sensors/GPS/gps.h"
extern DMA_Event_t dma_ublox_gps_event;
extern char data_ublox_gps[DMA_GPS_BUF_SIZE]; /* Data buffer that contains newly received data */
extern char dma_ublox_gps_aux_buffer[DMA_GPS_BUF_SIZE]; /* Circular buffer used for DMA */

#endif /* THREADS_THREADS_H_ */
