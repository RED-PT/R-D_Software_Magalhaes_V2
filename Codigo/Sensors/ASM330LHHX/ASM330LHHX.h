/*
 * ASM330LHHX.h
 *
 *  Created on: Oct 14, 2025
 *      Author: Tomas Teixeira
 */

#ifndef SENSORS_ASM330LHHX_ASM330LHHX_H_
#define SENSORS_ASM330LHHX_ASM330LHHX_H_

// Include Definitios from ST, C libraries and others
#include <string.h>
#include <stdio.h>
#include "asm330lhhx_reg.h"
#include "stm32f4xx_hal.h"
#include "config.h"
#include <stdbool.h>
#include "defs.h"

// Private Macro
#define BOOT_TIME 10 //ms

//Prototypes
bool imu_init(void);
bool imu_configure(void);
bool imu_start_read_dma(void);
bool imu_process_data(IMU_t *imu_data);


#endif /* SENSORS_ASM330LHHX_ASM330LHHX_H_ */
