/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_BUSY_Pin GPIO_PIN_3
#define RADIO_BUSY_GPIO_Port GPIOE
#define RXEN_Pin GPIO_PIN_2
#define RXEN_GPIO_Port GPIOF
#define TXEN_Pin GPIO_PIN_3
#define TXEN_GPIO_Port GPIOF
#define RADIO_DIO1_Pin GPIO_PIN_4
#define RADIO_DIO1_GPIO_Port GPIOF
#define RADIO_DIO1_EXTI_IRQn EXTI4_IRQn
#define RADIO_DIO2_Pin GPIO_PIN_5
#define RADIO_DIO2_GPIO_Port GPIOF
#define RADIO_DIO2_EXTI_IRQn EXTI9_5_IRQn
#define RADIO_NRST_Pin GPIO_PIN_6
#define RADIO_NRST_GPIO_Port GPIOF
#define RADIO_CS_Pin GPIO_PIN_10
#define RADIO_CS_GPIO_Port GPIOF
#define IMU_CS_Pin GPIO_PIN_5
#define IMU_CS_GPIO_Port GPIOC
#define ALT_CS_Pin GPIO_PIN_0
#define ALT_CS_GPIO_Port GPIOB
#define MAG_CS_Pin GPIO_PIN_14
#define MAG_CS_GPIO_Port GPIOE
#define MAG_INTERRUPT_Pin GPIO_PIN_15
#define MAG_INTERRUPT_GPIO_Port GPIOE
#define MAG_INTERRUPT_EXTI_IRQn EXTI15_10_IRQn
#define SD_CS_Pin GPIO_PIN_8
#define SD_CS_GPIO_Port GPIOD
#define PWM_ESC_Pin GPIO_PIN_12
#define PWM_ESC_GPIO_Port GPIOD
#define GPIO_BUZZER_Pin GPIO_PIN_4
#define GPIO_BUZZER_GPIO_Port GPIOG
#define GPS_RESET_N_Pin GPIO_PIN_11
#define GPS_RESET_N_GPIO_Port GPIOA
#define GPS_INTERRUPT_Pin GPIO_PIN_12
#define GPS_INTERRUPT_GPIO_Port GPIOA
#define GPS_INTERRUPT_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
