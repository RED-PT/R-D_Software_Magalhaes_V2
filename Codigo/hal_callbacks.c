/**
 * @file hal_callbacks.c
 * @brief HAL Interrupt Callback Implementations
 * @author Tomás Teixeira (texman)
 * @date November 18, 2025
 * @version 2.0
 *
 * @details
 * This file implements all STM32 HAL interrupt callbacks for the Magalhães
 * Flight Computer. It serves as the central interrupt handling module that
 * bridges hardware interrupts with FreeRTOS task notifications.
 *
 * ## Callback Categories
 *
 * ### 1. GPIO External Interrupts (EXTI)
 * | Pin | Source | Handler | Notification |
 * |-----|--------|---------|--------------|
 * | EXTI_IMU_PIN | ASM330LHHX DRDY | sensors_thread | SENSOR_NOTIFY_IMU_DRDY |
 * | EXTI_MAG_PIN | MMC5983MA DRDY | sensors_thread | SENSOR_NOTIFY_MAG_DRDY |
 * | EXTI_LORA_PIN | SX126x DIO1 | SX126x_DIO1_IRQ_Handler() | Internal |
 *
 * ### 2. SPI DMA Callbacks
 * | SPI Instance | Active Sensor | Action |
 * |--------------|---------------|--------|
 * | SPI1 (IMU/BARO) | ACTIVE_SENSOR_IMU | ASM330LHHX_ParseDMABuffer() |
 * | SPI_MAG (MAG) | ACTIVE_SENSOR_MAG | MMC5983MA_ParseDMABuffer() |
 * | SPI_LORA | N/A | SX126x_SPI_TxCpltCallback() |
 *
 * ### 3. I2C DMA Callbacks
 * | I2C Instance | Device | Action |
 * |--------------|--------|--------|
 * | I2C1 | BNO055 | BNO055_ParseDMABuffer() |
 *
 * ### 4. UART Callbacks
 * | UART Instance | Device | Handler |
 * |---------------|--------|---------|
 * | UART_UBLOX | GPS | sensors_thread notification |
 * | UART_RADIO | E22 LoRa (UART only) | E22_UART_RxCpltCallback() |
 *
 * ## Design Considerations
 * - All callbacks first verify `osKernelGetState() == osKernelRunning`
 * - Uses `xTaskNotifyFromISR()` with `eSetBits` for event accumulation
 * - Always calls `portYIELD_FROM_ISR()` for proper context switching
 * - Error callbacks notify SENSOR_NOTIFY_DMA_ERROR for recovery handling
 *
 * @see hal_callbacks.h for interface documentation
 * @see sensors_thread.c for notification handling
 * @see radio_thread.c for radio interrupt processing
 *
 * @ingroup HAL_Callbacks
 */

#include "hal_callbacks.h"
#include "config.h"
#include "Sensors/sensors_thread.h"
#include "Radio/LORA Drivers/lora_sx126x.h"
#ifdef RADIO_INTERFACE_UART
#include "Radio/LORA Drivers/e22_uart_dma.h"
#endif

/** @name External Thread Handles
 *  @brief Thread handles defined in other modules
 *  @{
 */
extern osThreadId_t sensors_thread_id;  /**< Sensors thread handle from sensors_thread.c */
/** @} */

/** @name External Sensor Tracking Variables
 *  @brief Active sensor tracking from sensors_thread.c
 *  @{
 */
extern volatile active_sensor_t spi1_active_sensor;  /**< Currently active sensor on SPI1 bus */
extern volatile active_sensor_t spi_mag_active_sensor;  /**< Currently active sensor on MAG SPI bus */
extern volatile active_sensor_t i2c1_active_sensor;  /**< Currently active sensor on I2C1 bus */
/** @} */

/** @name External Sensor Device Instances
 *  @brief Sensor device structures from sensors_thread.c
 *  @{
 */
extern ASM330LHHX_t imu_device;   /**< 6-axis IMU device instance */
extern MMC5983MA_t mag_device;    /**< Magnetometer device instance */
extern BNO055_t bno_device;       /**< 9-DOF orientation sensor instance */
/** @} */

/* ============================================================================
 * GPIO EXTI Callbacks
 * ============================================================================ */

/**
 * @brief GPIO External Interrupt callback handler
 * @param GPIO_Pin The GPIO pin that triggered the interrupt (bitmask)
 *
 * @details
 * Handles external interrupt events from sensor data-ready pins and LoRa DIO1.
 * This callback is invoked by the HAL when any configured EXTI line triggers.
 *
 * **Handled Pins:**
 * - `EXTI_IMU_PIN`: IMU data-ready, notifies sensors thread with SENSOR_NOTIFY_IMU_DRDY
 * - `EXTI_MAG_PIN`: Magnetometer data-ready, notifies with SENSOR_NOTIFY_MAG_DRDY
 * - `EXTI_LORA_PIN`: SX126x DIO1 interrupt, calls SX126x_DIO1_IRQ_Handler()
 *
 * @note Returns immediately if RTOS kernel is not running (boot safety)
 * @see SENSOR_NOTIFY_IMU_DRDY
 * @see SENSOR_NOTIFY_MAG_DRDY
 * @see SX126x_DIO1_IRQ_Handler()
 */
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

/* ============================================================================
 * SPI Callbacks
 * ============================================================================ */

/**
 * @brief SPI transmit/receive complete callback
 * @param hspi Pointer to the SPI handle that completed the transfer
 *
 * @details
 * Called when a full-duplex SPI DMA transfer completes. Parses the received
 * data based on which sensor was active, then notifies the sensors thread.
 *
 * **Active Sensor Handling:**
 * - `SPI_IMU_BARO_INSTANCE` + `ACTIVE_SENSOR_IMU`: Calls ASM330LHHX_ParseDMABuffer()
 * - `SPI_MAG_INSTANCE` + `ACTIVE_SENSOR_MAG`: Calls MMC5983MA_ParseDMABuffer()
 *
 * After parsing, sends SENSOR_NOTIFY_DMA_COMPLETE to sensors_thread.
 *
 * @note The active sensor tracking prevents parsing wrong data when multiple
 *       sensors share the same SPI bus.
 */
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
        if (spi_mag_active_sensor == ACTIVE_SENSOR_MAG) {
            MMC5983MA_ParseDMABuffer(&mag_device);
        }
    }

    // Notify sensors thread
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_COMPLETE,
                      eSetBits, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief SPI transmit-only complete callback
 * @param hspi Pointer to the SPI handle that completed transmission
 *
 * @details
 * Called when an SPI transmit-only DMA transfer completes.
 * Currently only handles SX126x LoRa radio SPI communications.
 *
 * @see SX126x_SPI_TxCpltCallback()
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    // SX126x SPI TX complete
    if (hspi->Instance == SPI_LORA_INSTANCE) {
        SX126x_SPI_TxCpltCallback();
    }
}

/**
 * @brief SPI receive-only complete callback
 * @param hspi Pointer to the SPI handle that completed reception
 *
 * @details
 * Called when an SPI receive-only DMA transfer completes.
 * Currently only handles SX126x LoRa radio SPI communications.
 *
 * @see SX126x_SPI_RxCpltCallback()
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    // SX126x SPI RX complete
    if (hspi->Instance == SPI_LORA_INSTANCE) {
        SX126x_SPI_RxCpltCallback();
    }
}

/**
 * @brief SPI error callback
 * @param hspi Pointer to the SPI handle that encountered an error
 *
 * @details
 * Called when an SPI communication error occurs (overrun, CRC, frame, etc.).
 * Notifies sensors_thread with SENSOR_NOTIFY_DMA_ERROR for error recovery.
 *
 * @warning DMA errors may indicate hardware issues or timing problems.
 *          The sensors thread should handle recovery and potentially
 *          reinitialize the affected sensor.
 */
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

/* ============================================================================
 * I2C Callbacks
 * ============================================================================ */

/**
 * @brief I2C memory receive complete callback
 * @param hi2c Pointer to the I2C handle that completed the transfer
 *
 * @details
 * Called when an I2C memory read DMA transfer completes.
 * Currently handles BNO055 9-DOF orientation sensor data reception.
 *
 * **Processing Flow:**
 * 1. Verify RTOS kernel is running
 * 2. Call BNO055_ParseDMABuffer() to process raw data
 * 3. Notify sensors_thread with SENSOR_NOTIFY_DMA_COMPLETE
 *
 * @note The BNO055 provides pre-fused orientation data (Euler angles,
 *       quaternions) reducing CPU load for attitude estimation.
 */
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

/**
 * @brief I2C error callback
 * @param hi2c Pointer to the I2C handle that encountered an error
 *
 * @details
 * Called when an I2C communication error occurs (NACK, bus error, etc.).
 * Notifies sensors_thread with SENSOR_NOTIFY_DMA_ERROR for recovery.
 *
 * **Common I2C Errors:**
 * - NACK: Device not responding or wrong address
 * - Bus error: SDA/SCL timing violation
 * - Arbitration lost: Multi-master conflict
 *
 * @warning I2C errors on BNO055 may require device reset sequence.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hi2c->Instance == I2C1) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_ERROR,
                          eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================================================================
 * UART Callbacks (GPS & LoRa Radio)
 * ============================================================================ */

/**
 * @brief UART transmit complete callback
 * @param huart Pointer to the UART handle that completed transmission
 *
 * @details
 * Called when a UART DMA transmit transfer completes.
 * Currently handles E22 LoRa module command transmission.
 *
 * @see E22_UART_TxCpltCallback()
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
#ifdef RADIO_INTERFACE_UART
    // E22 TX complete
    if (huart->Instance == UART_RADIO_INSTANCE) {
        E22_UART_TxCpltCallback();
    }
#endif
}

/**
 * @brief UART receive complete callback
 * @param huart Pointer to the UART handle that completed reception
 *
 * @details
 * Called when a UART DMA receive transfer completes. Handles two sources:
 *
 * **UART_UBLOX_INSTANCE (GPS):**
 * - Notifies sensors_thread with SENSOR_NOTIFY_GPS_DATA
 * - GPS thread will then parse NMEA sentences from the buffer
 *
 * **UART_RADIO_INSTANCE (E22 LoRa):**
 * - Calls E22_UART_RxCpltCallback() for radio data processing
 * - Handles telemetry packets and command responses
 *
 * @see GPS_ProcessBuffer() for NMEA parsing
 * @see E22_UART_RxCpltCallback() for radio handling
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (osKernelGetState() != osKernelRunning) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == UART_UBLOX_INSTANCE) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_GPS_DATA,
                          eSetBits, &xHigherPriorityTaskWoken);
    }
#ifdef RADIO_INTERFACE_UART
    else if (huart->Instance == UART_RADIO_INSTANCE) {
        E22_UART_RxCpltCallback();
    }
#endif

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief UART half-receive complete callback (currently disabled)
 * @param huart Pointer to the UART handle
 *
 * @details
 * This callback is currently disabled. It was intended for double-buffering
 * UART reception on the E22 radio module.
 *
 * @note Enable this callback if implementing continuous reception with
 *       circular DMA buffer processing.
 */
/*
#ifdef RADIO_INTERFACE_UART
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART_RADIO_INSTANCE) {
        E22_UART_RxHalfCpltCallback();
    }
}
#endif
*/

/**
 * @brief UART error callback
 * @param huart Pointer to the UART handle that encountered an error
 *
 * @details
 * Called when a UART communication error occurs (overrun, framing, noise, etc.).
 * Currently only logs errors for USART6 for debugging purposes.
 *
 * **Common UART Errors:**
 * - Overrun: Data received before previous byte read
 * - Framing: Stop bit not detected at expected time
 * - Noise: Noise detected on RX line
 * - Parity: Parity check failed (if enabled)
 *
 * @todo Implement proper error recovery and statistics tracking
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART_DEBUG_INSTANCE) {
		printf("ERRO DMA UART\r\n");
	}
}
