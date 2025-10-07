/*
 * MS5607SPI.c
 *
 *  Created on: Oct 7, 2025
 *      Author: Tomas Teixeira
 */
#include "MS5607SPI.h"
#include "math.h"

uint16_t C[7];

void MS5611_Select(GPIO_TypeDef *Port, uint16_t Pin) {
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
}

void MS5611_Deselect(GPIO_TypeDef *Port, uint16_t Pin) {
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef MS5611_Reset(SPI_HandleTypeDef *spi, GPIO_TypeDef *Port,
		uint16_t Pin) {
	uint8_t cmd = CMD_RESET;
	MS5611_Select(Port, Pin);
	HAL_Delay(1);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
	MS5611_Deselect(Port, Pin);
	HAL_Delay(10);
	return status;
}

HAL_StatusTypeDef MS5611_ReadPROM(SPI_HandleTypeDef *spi, GPIO_TypeDef *Port,
		uint16_t Pin) {
	HAL_StatusTypeDef status;

	for (uint8_t i = 0; i < 7; i++) {
		uint8_t buf[2];
		uint8_t addr = CMD_PROM_READ + (i * 2);
		MS5611_Select(Port, Pin);
		status = HAL_SPI_Transmit(spi, &addr, 1, HAL_MAX_DELAY);
		if (status != HAL_OK) {
			MS5611_Deselect(Port, Pin);
			return status;
		}
		status = HAL_SPI_Receive(spi, buf, 2, HAL_MAX_DELAY);
		MS5611_Deselect(Port, Pin);
		if (status != HAL_OK)
			return status;
		C[i] = ((uint16_t) buf[0] << 8) | buf[1];
	}
	return HAL_OK;
}

uint32_t MS5611_ReadADC(SPI_HandleTypeDef *spi, GPIO_TypeDef *Port,
		uint16_t Pin) {
	uint8_t cmd = CMD_ADC_READ;
	uint8_t buf[3];
	MS5611_Select(Port, Pin);
	HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spi, buf, 3, HAL_MAX_DELAY);
	MS5611_Deselect(Port, Pin);
	return ((uint32_t) buf[0] << 16) | ((uint32_t) buf[1] << 8) | buf[2];
}

int32_t MS5611_ReadTemperatureandPressure(float *temperature, int32_t *pressure,
		SPI_HandleTypeDef *spi, GPIO_TypeDef *Port, uint16_t Pin) {
	uint8_t cmd = CMD_CONV_D2;
	MS5611_Select(Port, Pin);
	HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
	MS5611_Deselect(Port, Pin);
	HAL_Delay(10);  // Wait for conversion (~9.04ms)

	uint32_t D2 = MS5611_ReadADC(spi, Port, Pin);

	int64_t dT = (int64_t) D2 - ((int64_t) C[5] << 8);
	int32_t TEMP = 2000 + ((dT * C[6]) >> 23);
	*temperature = (float) ((int) ceil(TEMP / 100))
			+ ((float) 0.01 * (TEMP % 100));
	;
//	*temperature = (float*) ((int*) ceil(TEMP / 100))
//			+ ((int*) 0.01 * (TEMP % 100));

	cmd = CMD_CONV_D1;
	MS5611_Select(Port, Pin);
	HAL_SPI_Transmit(spi, &cmd, 1, HAL_MAX_DELAY);
	MS5611_Deselect(Port, Pin);
	HAL_Delay(10);

	uint32_t D1 = MS5611_ReadADC(spi, Port, Pin);

	int64_t OFF = ((int64_t) C[2] << 16) + ((dT * C[4]) >> 7);
	int64_t SENS = ((int64_t) C[1] << 15) + ((dT * C[3]) >> 8);

	int32_t P = (int32_t) (((((int64_t) D1 * SENS) >> 21) - OFF) >> 15);
	*pressure = P;
	return HAL_OK;
}



