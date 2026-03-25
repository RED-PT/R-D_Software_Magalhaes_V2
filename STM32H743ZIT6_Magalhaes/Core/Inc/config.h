/*
 * config.h
 *
 * Hardware abstraction layer for Buzz V4 PCB (STM32H743ZIT6)
 *
 *  Created on: Mar 24, 2026
 *      Author: Tomas Teixeira
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"

// ============================================================================
// Board / Radio Interface Selection
// ============================================================================
// RADIO_INTERFACE_UART  -> E22-xxxT30D module via UART (Waveshare hat, F446ZE dev)
// RADIO_INTERFACE_SPI   -> E22-900M22S module via SPI  (Buzz V4 PCB)
#define RADIO_INTERFACE_SPI

// ============================================================================
// Extern HAL Handles
// ============================================================================

// SPI
extern SPI_HandleTypeDef hspi1;     // IMU (ASM330LHH) + Barometer (MS5607)
extern SPI_HandleTypeDef hspi2;     // SD Card
extern SPI_HandleTypeDef hspi5;     // Radio E22-900M22S (SX1262)
extern SPI_HandleTypeDef hspi6;     // Magnetometer (MMC5983MA)

// I2C
extern I2C_HandleTypeDef hi2c1;     // BNO055

// UART
extern UART_HandleTypeDef huart1;   // GPS (NEO-M9N)
extern UART_HandleTypeDef huart2;   // Debug / ST-Link

// Timers
extern TIM_HandleTypeDef htim1;     // Servos S1 (CH2), S2 (CH3)
extern TIM_HandleTypeDef htim2;     // Servos S4 (CH1), S5 (CH2)
extern TIM_HandleTypeDef htim4;     // ESC (CH1)

// DMA - SPI
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi5_tx;

// DMA - I2C
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

// DMA - UART
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

// BDMA - SPI6 (D3 domain, must use BDMA)
extern DMA_HandleTypeDef hdma_spi6_rx;
extern DMA_HandleTypeDef hdma_spi6_tx;

// ============================================================================
// I2C
// ============================================================================
#define I2C_BNO                 &hi2c1
#define I2C_INA                 &hi2c1

// ============================================================================
// UART
// ============================================================================
#define UART_UBLOX              &huart1
#define UART_UBLOX_INSTANCE     USART1
#define UART_DEBUG              &huart2
#define UART_DEBUG_INSTANCE     USART2

// ============================================================================
// SPI - IMU + Barometer (SPI1, shared bus)
// ============================================================================
#define SPI_IMU_BARO            &hspi1
#define SPI_IMU_BARO_INSTANCE   SPI1

#define CS_IMU_PORT             IMU_CS_GPIO_Port
#define CS_IMU_PIN              IMU_CS_Pin
#define CS_BARO_PORT            ALT_CS_GPIO_Port
#define CS_BARO_PIN             ALT_CS_Pin

#define CS_IMU_LOW()            HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_RESET)
#define CS_IMU_HIGH()           HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET)
#define CS_BARO_LOW()           HAL_GPIO_WritePin(CS_BARO_PORT, CS_BARO_PIN, GPIO_PIN_RESET)
#define CS_BARO_HIGH()          HAL_GPIO_WritePin(CS_BARO_PORT, CS_BARO_PIN, GPIO_PIN_SET)

// ============================================================================
// SPI - Magnetometer (SPI6, BDMA)
// ============================================================================
#define SPI_MAG                 &hspi6
#define SPI_MAG_INSTANCE        SPI6

#define CS_MAG_PORT             MAG_CS_GPIO_Port
#define CS_MAG_PIN              MAG_CS_Pin

#define CS_MAG_LOW()            HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_RESET)
#define CS_MAG_HIGH()           HAL_GPIO_WritePin(CS_MAG_PORT, CS_MAG_PIN, GPIO_PIN_SET)

// ============================================================================
// SPI - SD Card (SPI2)
// ============================================================================
#define SPI_SDCARD              &hspi2
#define SPI_SDCARD_INSTANCE     SPI2

#define CS_SDCARD_PORT          SD_CS_GPIO_Port
#define CS_SDCARD_PIN           SD_CS_Pin

// SD Card SPI prescaler for high-speed after init (80MHz / 4 = 20MHz)
#define SD_SPI_PRESCALER_FAST   SPI_BAUDRATEPRESCALER_4

// ============================================================================
// SPI - Radio E22-900M22S / SX1262 (SPI5)
// ============================================================================
#ifdef RADIO_INTERFACE_SPI

#define SPI_LORA                &hspi5
#define SPI_LORA_INSTANCE       SPI5

#define CS_LORA_PORT            RADIO_CS_GPIO_Port
#define CS_LORA_PIN             RADIO_CS_Pin
#define RESET_LORA_PORT         RADIO_NRST_GPIO_Port
#define RESET_LORA_PIN          RADIO_NRST_Pin
#define BUSY_LORA_PORT          RADIO_BUSY_GPIO_Port
#define BUSY_LORA_PIN           RADIO_BUSY_Pin

#define CS_LORA_LOW()           HAL_GPIO_WritePin(CS_LORA_PORT, CS_LORA_PIN, GPIO_PIN_RESET)
#define CS_LORA_HIGH()          HAL_GPIO_WritePin(CS_LORA_PORT, CS_LORA_PIN, GPIO_PIN_SET)
#define RESET_LORA_LOW()        HAL_GPIO_WritePin(RESET_LORA_PORT, RESET_LORA_PIN, GPIO_PIN_RESET)
#define RESET_LORA_HIGH()       HAL_GPIO_WritePin(RESET_LORA_PORT, RESET_LORA_PIN, GPIO_PIN_SET)
#define LORA_IS_BUSY()          (HAL_GPIO_ReadPin(BUSY_LORA_PORT, BUSY_LORA_PIN) == GPIO_PIN_SET)

// EXTI - Radio interrupts
#define EXTI_LORA_PORT          RADIO_DIO1_GPIO_Port
#define EXTI_LORA_PIN           RADIO_DIO1_Pin

#define EXTI_LORA_DIO2_PORT     RADIO_DIO2_GPIO_Port
#define EXTI_LORA_DIO2_PIN      RADIO_DIO2_Pin

// RF switch control (E22-900M22S external RXEN/TXEN)
#define RADIO_RXEN_PORT         RXEN_GPIO_Port
#define RADIO_RXEN_PIN          RXEN_Pin
#define RADIO_TXEN_PORT         TXEN_GPIO_Port
#define RADIO_TXEN_PIN          TXEN_Pin

#define RADIO_RXEN_HIGH()       HAL_GPIO_WritePin(RADIO_RXEN_PORT, RADIO_RXEN_PIN, GPIO_PIN_SET)
#define RADIO_RXEN_LOW()        HAL_GPIO_WritePin(RADIO_RXEN_PORT, RADIO_RXEN_PIN, GPIO_PIN_RESET)
#define RADIO_TXEN_HIGH()       HAL_GPIO_WritePin(RADIO_TXEN_PORT, RADIO_TXEN_PIN, GPIO_PIN_SET)
#define RADIO_TXEN_LOW()        HAL_GPIO_WritePin(RADIO_TXEN_PORT, RADIO_TXEN_PIN, GPIO_PIN_RESET)

// RF switch mode helpers
#define RADIO_RF_SWITCH_RX()    do { RADIO_RXEN_HIGH(); RADIO_TXEN_LOW();  } while(0)
#define RADIO_RF_SWITCH_TX()    do { RADIO_RXEN_LOW();  RADIO_TXEN_HIGH(); } while(0)
#define RADIO_RF_SWITCH_OFF()   do { RADIO_RXEN_LOW();  RADIO_TXEN_LOW();  } while(0)

#endif /* RADIO_INTERFACE_SPI */

// ============================================================================
// GPIO - Interrupts (EXTI)
// ============================================================================
#define EXTI_IMU_PORT           IMU_CS_GPIO_Port    // IMU interrupt disabled (EXTI4 conflict)
#define EXTI_IMU_PIN            IMU_CS_Pin          // Using polling instead

#define EXTI_MAG_PORT           MAG_INTERRUPT_GPIO_Port
#define EXTI_MAG_PIN            MAG_INTERRUPT_Pin

#define EXTI_GPS_PORT           GPS_INTERRUPT_GPIO_Port
#define EXTI_GPS_PIN            GPS_INTERRUPT_Pin

// ============================================================================
// GPIO - GPS Control
// ============================================================================
#define GPS_RESET_PORT          GPS_RESET_N_GPIO_Port
#define GPS_RESET_PIN           GPS_RESET_N_Pin

#define GPS_RESET_ASSERT()      HAL_GPIO_WritePin(GPS_RESET_PORT, GPS_RESET_PIN, GPIO_PIN_RESET)
#define GPS_RESET_RELEASE()     HAL_GPIO_WritePin(GPS_RESET_PORT, GPS_RESET_PIN, GPIO_PIN_SET)

// ============================================================================
// GPIO - Buzzer
// ============================================================================
#define BUZZER_PORT             GPIO_BUZZER_GPIO_Port
#define BUZZER_PIN              GPIO_BUZZER_Pin

#define BUZZER_ON()             HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET)
#define BUZZER_OFF()            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET)
#define BUZZER_TOGGLE()         HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN)

// ============================================================================
// Timers - PWM ESC
// ============================================================================
#define PWM_ESC_TIM             &htim4
#define PWM_ESC_TIM_INSTANCE    TIM4
#define PWM_ESC_CHANNEL         TIM_CHANNEL_1
#define PWM_ESC_CHANNEL_WRITE   TIM4->CCR1

// ============================================================================
// Timers - PWM Servos
// ============================================================================
#define PWM_SERVO_S1_TIM        &htim1
#define PWM_SERVO_S1_CHANNEL    TIM_CHANNEL_2
#define PWM_SERVO_S1_WRITE      TIM1->CCR2

#define PWM_SERVO_S2_TIM        &htim1
#define PWM_SERVO_S2_CHANNEL    TIM_CHANNEL_3
#define PWM_SERVO_S2_WRITE      TIM1->CCR3

#define PWM_SERVO_S4_TIM        &htim2
#define PWM_SERVO_S4_CHANNEL    TIM_CHANNEL_1
#define PWM_SERVO_S4_WRITE      TIM2->CCR1

#define PWM_SERVO_S5_TIM        &htim2
#define PWM_SERVO_S5_CHANNEL    TIM_CHANNEL_2
#define PWM_SERVO_S5_WRITE      TIM2->CCR2

// ============================================================================
// Timing
// ============================================================================
#define STM_CLOCK_PERIOD_NS     2.083f  // 1/480MHz in nanoseconds
#define time_in_microseconds    (uint32_t)(DWT->CYCCNT * STM_CLOCK_PERIOD_NS / 1000.0f)
#define time_in_millis          HAL_GetTick()

// ============================================================================
// DMA Buffer Placement (Cortex-M7 D-Cache coherency)
// ============================================================================
// DMA1/DMA2 buffers → D2 SRAM (0x30000000) — non-cacheable
// BDMA buffers (SPI6) → D3 SRAM (0x38000000) — BDMA-only domain
#define DMA_BUFFER      __attribute__((section(".dma_buffer"), aligned(32)))
#define BDMA_BUFFER     __attribute__((section(".bdma_buffer"), aligned(32)))

#endif /* INC_CONFIG_H_ */
