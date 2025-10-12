/*
 * sensors_thread.h
 *
 *	Sensors Thread: Reads sensors via DMA from multiple SPI buses
 *	SPI_X: Altimeter (MS5607) + IMU (ASM330LHHX)
 *	SPI_XX: Magnetometer (MMC5983MA)
 *	SPI_XXX: BNO055 (absolute orientation sensor)
 *
 *	Fuses data and sends to the Data Handler thread
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_SENSORS_THREAD_H_
#define SENSORS_SENSORS_THREAD_H_

// Include Definitions, Data Handler, Etc
#include "defs.h"
#include "Data Handler/flash_data_handler.h"
#include "stm32f4xx_hal.h"


// Thread Function
void sensors_thread_function();


#endif /* SENSORS_SENSORS_THREAD_H_ */
