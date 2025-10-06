/*
 * config.h
 *
 *  Created on: Oct 5, 2025
 *      Author: texman
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
//include do main

#include "main.h"

//extern handles
extern I2C_HandleTypeDef hi2c1;

extern SD_HandleTypeDef hsd;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

//I2C
#define I2C_BNO &hi2c1

//SDIO
#define SDIO_SD_CARD &hsd

//UART
#define UART_DEBUG &huart1
#define UART_GPS &huart3

//SPI
#define SPI_IMU_BARO &hspi1
#define CS_IMU_PORT GPIOB
#define CS_IMU_PIN GPIO_PIN_1
#define CS_BARO_PORT GPIOB
#define CS_BARO_PIN GPIO_PIN_3

#define SPI_LORA &hspi2
#define CS_LORA_PORT GPIOB
#define CS_LORA_PIN GPIO_PIN_12

#define SPI_MAG &hspi3
#define CS_MAG_PORT GPIOB
#define CS_MAG_PIN GPIO_PIN_13

//TIMERS
#define STM_CLOCK_PERIOD_NS 5.952 //nano segundos
#define time_in_microseconds (uint32_t) (DWT->CYCCNT * STM_CLOCK_PERIOD_NS / 1000) // us
#define time_in_millis HAL_GetTick()

//TIM1: PWM SERVOS
#define PWM_SERVOS_TIM &htim1
#define PWM_SERVOS_TIM_INSTANCE TIM1
#define PWM_SERVOS_CHANNEL TIM_CHANNEL_1
#define PWM_SERVOS_CHANNEL_WRITE TIM1->CCR4

//TIM3: PWM ESC
#define PWM_ESC_TIM &htim3
#define PWM_ESC_TIM_INSTANCE TIM3
#define PWM_ESC_CHANNEL TIM_CHANNEL_1
#define PWM_ESC_CHANNEL_WRITE TIM3->CCR4

//TIM2: TIME IN (ms) 1KHz
#define TIMER_TIME_IN_MILLIS &htim2

// GPIO - INTERRUPTS - input
#define EXTI_LORA_PORT GPIOB
#define EXTI_LORA_PIN GPIO_PIN_2

#define EXTI_IMU_PORT GPIOB
#define EXTI_IMU_PIN GPIO_PIN_11

#endif /* INC_CONFIG_H_ */
