/*
 MS5607-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
 05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
 Changelog:
 2012-05-23 - initial release.
 */
/* ============================================================================================
 MS5607-02 device SPI library code for ARM STM32F103xx is placed under the MIT license
 Copyright (c) 2020 João Pedro Vilas Boas

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 ================================================================================================
 */

#ifndef _MS5607SPI_H_
#define _MS5607SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal.h"


// MS5607 commands
enum MS_CMD {
	CMD_RESET = 0x1E,
	CMD_CONV_D1 = 0x48,
	CMD_CONV_D2 = 0x58,
	CMD_ADC_READ = 0x00,
	CMD_PROM_READ = 0xA0
};

// Calibration coefficients
extern uint16_t C[7];

// Function Prototypes
void MS5611_Select(GPIO_TypeDef *Port, uint16_t Pin);
void MS5611_Deselect(GPIO_TypeDef *Port, uint16_t Pin);
HAL_StatusTypeDef MS5611_Reset(SPI_HandleTypeDef *spi, GPIO_TypeDef *Port,
		uint16_t Pin);
HAL_StatusTypeDef MS5611_ReadPROM(SPI_HandleTypeDef *spi, GPIO_TypeDef *Port,
		uint16_t Pin);
uint32_t MS5611_ReadADC(SPI_HandleTypeDef *spi, GPIO_TypeDef *Port,
		uint16_t Pin);
int32_t MS5611_ReadTemperatureandPressure(float *temperature, int32_t *pressure,
		SPI_HandleTypeDef *spi, GPIO_TypeDef *Port, uint16_t Pin);
#ifdef __cplusplus
}
#endif

#endif /* _MS5607SPI_H_ */
