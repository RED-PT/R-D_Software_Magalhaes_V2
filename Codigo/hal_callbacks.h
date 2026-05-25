/**
 * @file hal_callbacks.h
 * @brief HAL Interrupt Callback Declarations
 * @author Tomás Teixeira (texman)
 * @date November 18, 2025
 * @version 2.1
 *
 * @details
 * This header file provides the interface for HAL (Hardware Abstraction Layer)
 * interrupt callbacks used throughout the Magalhães Flight Computer system.
 *
 * Supports dual-board configurations:
 * - **STM32F446ZE** (dev board): UART radio, SPI3 magnetometer, USART6 debug
 * - **STM32H743ZIT6** (Buzz V4 PCB): SPI radio, SPI6 magnetometer, USART2 debug
 *
 * ## Overview
 * The HAL callback system provides centralized interrupt handling for:
 * - **GPIO EXTI**: Sensor data-ready signals (IMU, MAG, LoRa DIO1)
 * - **SPI**: DMA transfers for high-speed sensor communication
 * - **I2C**: DMA transfers for BNO055 orientation sensor
 * - **UART**: GPS NMEA data and LoRa radio communication
 *
 * ## Architecture
 * All callbacks are implemented in hal_callbacks.c and override weak STM32 HAL
 * definitions. They use FreeRTOS task notifications to wake sensor/radio threads.
 *
 * @verbatim
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │                    Interrupt Sources                            │
 *   ├─────────────────────────────────────────────────────────────────┤
 *   │  GPIO EXTI     │  SPI DMA       │  I2C DMA      │  UART DMA    │
 *   │  - IMU DRDY    │  - IMU RX      │  - BNO055 RX  │  - GPS RX    │
 *   │  - MAG DRDY    │  - MAG RX      │               │  - LoRa RX   │
 *   │  - LoRa DIO1   │  - LoRa TX/RX  │               │  - LoRa TX   │
 *   └───────┬────────┴───────┬────────┴───────┬───────┴───────┬──────┘
 *           │                │                │               │
 *           ▼                ▼                ▼               ▼
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │              HAL Callbacks (hal_callbacks.c)                    │
 *   │         xTaskNotifyFromISR() → Wake appropriate thread          │
 *   └─────────────────────────────────────────────────────────────────┘
 * @endverbatim
 *
 * ## Thread Safety
 * All callbacks check `osKernelGetState() != osKernelRunning` before
 * attempting task notifications to prevent crashes during boot.
 *
 * @see hal_callbacks.c for implementation details
 * @see sensors_thread.h for sensor notification flags
 * @see radio_thread.h for radio interrupt handling
 *
 * @defgroup HAL_Callbacks HAL Interrupt Callbacks
 * @{
 */

#ifndef HAL_CALLBACKS_H_
#define HAL_CALLBACKS_H_

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/**
 * @}
 */

#endif /* HAL_CALLBACKS_H_ */
