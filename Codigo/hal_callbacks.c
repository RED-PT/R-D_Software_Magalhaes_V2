/*
 * hal_callbacks.c
 *
 *  Created on: Nov 18, 2025
 *      Author: texman
 */

#include "hal_callbacks.h"
#include "config.h"
#include "Sensors/sensors_thread.h"
#include "Radio/LORA Drivers/lora_sx126x.h"
#include "Radio/LORA Drivers/e22_uart_dma.h"

// External thread handles
extern osThreadId_t sensors_thread_id;

// External sensor tracking variables (from sensors_thread.c)
extern volatile active_sensor_t spi1_active_sensor;
extern volatile active_sensor_t spi3_active_sensor;
extern volatile active_sensor_t i2c1_active_sensor;

// External sensor devices (from sensors_thread.c)
extern ASM330LHHX_t imu_device;
extern MMC5983MA_t mag_device;
extern BNO055_t bno_device;

// ============================================================================
// GPIO EXTI Callbacks
// ============================================================================

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Sensor DRDY pins
    if (GPIO_Pin == EXTI_IMU_PIN) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_IMU_DRDY,
                          eSetBits, &xHigherPriorityTaskWoken);
    }
    else if (GPIO_Pin == EXTI_MAG_PIN) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_MAG_DRDY,
                          eSetBits, &xHigherPriorityTaskWoken);
    }
    // SX126x DIO1 interrupt (PB2)
    else if (GPIO_Pin == EXTI_LORA_PIN) {
        SX126x_DIO1_IRQ_Handler();
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================================
// SPI Callbacks
// ============================================================================

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Parse DMA buffer based on active sensor
    if (hspi->Instance == SPI_IMU_BARO_INSTANCE) {
        if (spi1_active_sensor == ACTIVE_SENSOR_IMU) {
            ASM330LHHX_ParseDMABuffer(&imu_device);
        }
    }
    else if (hspi->Instance == SPI_MAG_INSTANCE) {
        if (spi3_active_sensor == ACTIVE_SENSOR_MAG) {
            MMC5983MA_ParseDMABuffer(&mag_device);
        }
    }

    // Notify sensors thread
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_COMPLETE,
                      eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    // SX126x SPI TX complete
    if (hspi->Instance == SPI_LORA_INSTANCE) {
        SX126x_SPI_TxCpltCallback();
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    // SX126x SPI RX complete
    if (hspi->Instance == SPI_LORA_INSTANCE) {
        SX126x_SPI_RxCpltCallback();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify sensors thread of error
    if (hspi->Instance == SPI_IMU_BARO_INSTANCE || hspi->Instance == SPI_MAG_INSTANCE) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_ERROR,
                          eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================================
// I2C Callbacks
// ============================================================================

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        BNO055_ParseDMABuffer(&bno_device);
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_COMPLETE,
                          eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_ERROR,
                          eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================================
// UART Callbacks (GPS)
// ============================================================================
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // E22 TX complete
    if (huart->Instance == UART_RADIO_INSTANCE) {
        E22_UART_TxCpltCallback();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == UART_UBLOX_INSTANCE) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_GPS_DATA,
                          eSetBits, &xHigherPriorityTaskWoken);
    }

    else if (huart->Instance == UART_RADIO_INSTANCE) {
            E22_UART_RxCpltCallback();
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART_RADIO_INSTANCE) {
        E22_UART_RxHalfCpltCallback();
    }
}
*/


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6) {
		printf("ERROERRO DMA\r\n");
	}
}
