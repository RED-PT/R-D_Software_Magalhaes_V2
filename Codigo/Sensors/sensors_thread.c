/*
 * sensors_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "sensors_thread.h"
#include "print.h"
#include "Data Handler/flash_data_handler.h"
#include <string.h>

// Sensor Instances
ASM330LHHX_t imu_device;
MMC5983MA_t mag_device;
MS5607_t baro_device;
BNO055_t bno_device;

// Bus Tracking
static volatile active_sensor_t spi1_active_sensor = ACTIVE_SENSOR_NONE;
static volatile active_sensor_t spi3_active_sensor = ACTIVE_SENSOR_NONE;
static volatile active_sensor_t i2c1_active_sensor = ACTIVE_SENSOR_NONE;

// Statistics
static struct {
    uint32_t imu_samples;
    uint32_t mag_samples;
    uint32_t baro_samples;
    uint32_t bno_samples;
    uint32_t imu_errors;
    uint32_t mag_errors;
    uint32_t baro_errors;
    uint32_t bno_errors;
    uint32_t dma_errors;
} sensor_stats = {0};

// Timer Callbacks
void vBaroTimerCallback(TimerHandle_t xTimer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_BARO_TIMER, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vBnoTimerCallback(TimerHandle_t xTimer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_BNO_TIMER, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// GPIO EXTI Callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == EXTI_IMU_PIN) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_IMU_DRDY, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if (GPIO_Pin == EXTI_MAG_PIN) {
    	xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_MAG_DRDY, eSetBits, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// DMA Callbacks
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Parse DMA buffer based on active sensor
    if (hspi->Instance == SPI1) {
    	if (spi1_active_sensor == ACTIVE_SENSOR_IMU) {
            ASM330LHHX_ParseDMABuffer(&imu_device);
        }
        else if (spi1_active_sensor == ACTIVE_SENSOR_BARO) {
            // Barometer parsing
        }
    }
    else if (hspi->Instance == SPI3) {
        if (spi3_active_sensor == ACTIVE_SENSOR_MAG) {
            MMC5983MA_ParseDMABuffer(&mag_device);
        }
    }

    // Notify thread
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_COMPLETE, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    sensor_stats.dma_errors++;
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_ERROR, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1) {
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_COMPLETE, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1) {
        sensor_stats.dma_errors++;
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_ERROR, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Initialization
void sensors_thread_init(void) {
    printf("Initializing sensors...\n");

    // Initialize IMU
    printf("Initializing IMU...\n");
    if (!ASM330LHHX_Init(&imu_device, SPI_IMU_BARO)) {
        printf("ERROR: IMU init failed!\n");
    }
    if (!ASM330LHHX_Configure(&imu_device)) {
        printf("ERROR: IMU configure failed!\n");
    }

    // Initialize Magnetometer
    printf("Initializing Magnetometer...\n");
    if (!MMC5983MA_Init(&mag_device, SPI_MAG)) {
        printf("ERROR: MAG init failed!\n");
    }
    if (!MMC5983MA_Configure(&mag_device)) {
        printf("ERROR: MAG configure failed!\n");
    }

    // Initialize Barometer
    printf("Initializing Barometer...\n");
    if (!MS5607_Init(&baro_device, SPI_IMU_BARO, CS_BARO_PORT, CS_BARO_PIN)) {
        printf("ERROR: BARO init failed!\n");
    }
    if (!MS5607_Configure(&baro_device)) {
        printf("ERROR: BARO configure failed!\n");
    }

    // Initialize BNO055
    printf("Initializing BNO055...\n");
    if (!BNO055_Init(&bno_device, I2C_BNO)) {
        printf("ERROR: BNO init failed!\n");
    }
    if (!BNO055_Configure(&bno_device)) {
        printf("ERROR: BNO configure failed!\n");
    }

    printf("Sensors initialized successfully!\n");
}

// Thread Main Loop

void sensors_thread_function(void *argument) {
    printf("Sensors thread started\n");

    uint32_t ulNotificationValue;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);

    xTimerStart(xBaroTimer, 0);
    xTimerStart(xBnoTimer, 0);

    TickType_t last_stats_time = xTaskGetTickCount();

    while(1) {
    	// UPDATE BAROMETER STATE MACHINE
		// Call continuously (not just on timer) for responsive state transitions
		if (spi1_active_sensor == ACTIVE_SENSOR_BARO) {
			if (MS5607_Update(&baro_device)) {
			// Conversion complete, will be processed on next DMA_COMPLETE notification
			}
		}

    	// WAIT FOR NOTIFICATIONS
        ulNotificationValue = ulTaskNotifyTake(pdFALSE, xMaxBlockTime);

        if (ulNotificationValue == 0) {
            continue;  // Timeout
        }

        // INITIATE SENSOR READS
        if (ulNotificationValue & SENSOR_NOTIFY_IMU_DRDY) {
            // Check if SPI1 available
            if (spi1_active_sensor == ACTIVE_SENSOR_NONE) {
                spi1_active_sensor = ACTIVE_SENSOR_IMU;
                if (!ASM330LHHX_StartReadDMA(&imu_device)) {
                    spi1_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.imu_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_MAG_DRDY) {
            // Check if SPI3 available
            if (spi3_active_sensor == ACTIVE_SENSOR_NONE) {
                spi3_active_sensor = ACTIVE_SENSOR_MAG;
                if (!MMC5983MA_StartReadDMA(&mag_device)) {
                    spi3_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.mag_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BARO_TIMER) {
			// Check if SPI1 available (shared with IMU)
			if (spi1_active_sensor == ACTIVE_SENSOR_NONE) {
				// If not currently reading, start a new read
				if (baro_device.state == MS5607_STATE_IDLE) {
					spi1_active_sensor = ACTIVE_SENSOR_BARO;
					if (!MS5607_StartRead(&baro_device)) {
						spi1_active_sensor = ACTIVE_SENSOR_NONE;
						sensor_stats.baro_errors++;
					}
				}
			}
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BNO_TIMER) {
            // Check if I2C1 available
            if (i2c1_active_sensor == ACTIVE_SENSOR_NONE) {
                i2c1_active_sensor = ACTIVE_SENSOR_BNO;
                if (!BNO055_StartReadDMA(&bno_device)) {
                    i2c1_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.bno_errors++;
                }
            }
        }

        // PROCESS COMPLETED DATA

        if (ulNotificationValue & SENSOR_NOTIFY_DMA_COMPLETE) {

            if (spi1_active_sensor == ACTIVE_SENSOR_IMU) {
                IMU_t imu_data;
                if (ASM330LHHX_ProcessData(&imu_device, &imu_data)) {
                    sensor_stats.imu_samples++;
                    data_handler_store_imu(&imu_data);
                } else {
                    sensor_stats.imu_errors++;
                }
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
            }

            else if (spi1_active_sensor == ACTIVE_SENSOR_BARO) {
                BARO_t baro_data;
                if (MS5607_ProcessData(&baro_device, &baro_data)) {
                    sensor_stats.baro_samples++;
                    data_handler_store_baro(&baro_data);
                } else {
                    sensor_stats.baro_errors++;
                }
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
            }

            else if (spi3_active_sensor == ACTIVE_SENSOR_MAG) {
                MAG_t mag_data;
                if (MMC5983MA_ProcessData(&mag_device, &mag_data)) {
                    sensor_stats.mag_samples++;
                    data_handler_store_mag(&mag_data);
                } else {
                    sensor_stats.mag_errors++;
                }
                spi3_active_sensor = ACTIVE_SENSOR_NONE;
            }

            else if (i2c1_active_sensor == ACTIVE_SENSOR_BNO) {
                BNO_t bno_data;
                if (BNO055_ProcessData(&bno_device, &bno_data)) {
                    sensor_stats.bno_samples++;
                    data_handler_store_bno(&bno_data);
                } else {
                    sensor_stats.bno_errors++;
                }
                i2c1_active_sensor = ACTIVE_SENSOR_NONE;
            }
        }

        // ERROR HANDLING
        if (ulNotificationValue & SENSOR_NOTIFY_DMA_ERROR) {
            printf("ERROR: DMA error occurred\n");

            // Reset all buses
            CS_IMU_HIGH();
            CS_BARO_HIGH();
            CS_MAG_HIGH();
            spi1_active_sensor = ACTIVE_SENSOR_NONE;
            spi3_active_sensor = ACTIVE_SENSOR_NONE;
            i2c1_active_sensor = ACTIVE_SENSOR_NONE;
        }

        // STATISTICS

        if ((xTaskGetTickCount() - last_stats_time) > pdMS_TO_TICKS(5000)) {
            printf("\n=== Sensor Statistics ===\n");
            printf("IMU:  %lu samples, %lu errors\n", sensor_stats.imu_samples, sensor_stats.imu_errors);
            printf("MAG:  %lu samples, %lu errors\n", sensor_stats.mag_samples, sensor_stats.mag_errors);
            printf("BARO: %lu samples, %lu errors\n", sensor_stats.baro_samples, sensor_stats.baro_errors);
            printf("BNO:  %lu samples, %lu errors\n", sensor_stats.bno_samples, sensor_stats.bno_errors);
            printf("DMA errors: %lu\n", sensor_stats.dma_errors);

            last_stats_time = xTaskGetTickCount();
        }
    }
}
