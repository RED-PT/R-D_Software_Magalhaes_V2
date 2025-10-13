/*
 * sensors_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

// Includes
#include "sensors_thread.h"
#include "Data Handler/flash_data_handler.h"
#include "print.h"
#include <string.h>
#include "Threads/create_threads.h"

// Sensors Includes
#include "MS5607/MS5607SPI.h"
#include "ASM330LHHX/asm330lhhx_reg.h"


// DMA buffers (must be in DMA-accessible memory)
static uint8_t imu_dma_tx_buffer[IMU_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t imu_dma_rx_buffer[IMU_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

static uint8_t mag_dma_tx_buffer[MAG_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t mag_dma_rx_buffer[MAG_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

static uint8_t baro_dma_tx_buffer[BARO_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t baro_dma_rx_buffer[BARO_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

static uint8_t bno_dma_buffer[BNO_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

// Track which sensor is using which SPI bus
static volatile active_sensor_t spi1_active_sensor = ACTIVE_SENSOR_NONE;  // IMU or BARO
static volatile active_sensor_t spi3_active_sensor = ACTIVE_SENSOR_NONE;  // MAG
static volatile active_sensor_t i2c1_active_sensor = ACTIVE_SENSOR_NONE;  // BNO

// Barometer state machine
typedef enum {
    BARO_STATE_IDLE,
    BARO_STATE_CONVERT_D1,      // Pressure conversion
    BARO_STATE_READ_D1,         // Read pressure
    BARO_STATE_CONVERT_D2,      // Temperature conversion
    BARO_STATE_READ_D2,         // Read temperature
    BARO_STATE_CALCULATE
} baro_state_t;

static volatile baro_state_t baro_state = BARO_STATE_IDLE;
static uint32_t baro_d1_raw = 0;  // Raw pressure
static uint32_t baro_d2_raw = 0;  // Raw temperature

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

uint8_t* get_imu_tx_buffer(void) { return imu_dma_tx_buffer; }
uint8_t* get_imu_rx_buffer(void) { return imu_dma_rx_buffer; }
uint8_t* get_mag_tx_buffer(void) { return mag_dma_tx_buffer; }
uint8_t* get_mag_rx_buffer(void) { return mag_dma_rx_buffer; }
uint8_t* get_baro_tx_buffer(void) { return baro_dma_tx_buffer; }
uint8_t* get_baro_rx_buffer(void) { return baro_dma_rx_buffer; }
uint8_t* get_bno_buffer(void) { return bno_dma_buffer; }

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


// GPIO EXTI Callbacks (Data Ready Interrupts)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (GPIO_Pin == EXTI_IMU_PIN) {
        // IMU data ready
        xTaskNotifyFromISR(sensors_thread_id,SENSOR_NOTIFY_IMU_DRDY, eSetBits, &xHigherPriorityTaskWoken);
    }
    else if (GPIO_Pin == EXTI_MAG_PIN) {
        // Magnetometer data ready
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_MAG_DRDY, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// DMA Callbacks
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Notify sensor task that DMA is complete
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_COMPLETE, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    sensor_stats.dma_errors++;
    // Notify sensor task of error
    xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_DMA_ERROR, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (hi2c->Instance == I2C1) {
        // BNO055 DMA complete
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

// Sensors Read Functions
// IMU
bool sensor_imu_read(void) {
    // Check if SPI1 is available (shared with BARO)
    if (spi1_active_sensor != ACTIVE_SENSOR_NONE) {
        return false;  // Bus busy
    }
    spi1_active_sensor = ACTIVE_SENSOR_IMU;
    // Start DMA read
    if (!imu_start_read_dma()) {
        spi1_active_sensor = ACTIVE_SENSOR_NONE;
        sensor_stats.imu_errors++;
        return false;
    }
    return true;
}

void sensor_imu_process(void) {
    IMU_t imu_data;
    // Process raw DMA buffer into structured data
    if (imu_process_data(imu_dma_rx_buffer, &imu_data)) {
        sensor_stats.imu_samples++;

        // Send to data handler
        data_handler_store_imu(&imu_data);
    } else {
        sensor_stats.imu_errors++;
    }

    // Release SPI1 bus
    spi1_active_sensor = ACTIVE_SENSOR_NONE;
}

bool imu_start_read_dma(void) {
	return true;
}

bool imu_process_data(const uint8_t *buffer, IMU_t *imu_data) {
	return true;
}

// MAG
bool sensor_mag_read(void) {
    // Check if SPI3 is available
    if (spi3_active_sensor != ACTIVE_SENSOR_NONE) {
        return false;  // Bus busy
    }

    spi3_active_sensor = ACTIVE_SENSOR_MAG;

    // Start DMA read
    if (!mag_start_read_dma()) {
        spi3_active_sensor = ACTIVE_SENSOR_NONE;
        sensor_stats.mag_errors++;
        return false;
    }

    return true;
}

void sensor_mag_process(void) {
    MAG_t mag_data;

    // Process raw DMA buffer
    if (mag_process_data(mag_dma_rx_buffer, &mag_data)) {
        sensor_stats.mag_samples++;

        // Send to data handler
        data_handler_store_mag(&mag_data);
    } else {
        sensor_stats.mag_errors++;
    }

    // Release SPI3 bus
    spi3_active_sensor = ACTIVE_SENSOR_NONE;
}

bool mag_start_read_dma(void) {
	return true;
}

bool mag_process_data(const uint8_t *buffer, MAG_t *mag_data) {
	return true;
}

// BARO

bool sensor_baro_read(void) {
    // Check if SPI1 is available (shared with IMU)
    if (spi1_active_sensor != ACTIVE_SENSOR_NONE) {
        return false;  // Bus busy - IMU has priority
    }

    spi1_active_sensor = ACTIVE_SENSOR_BARO;

    // MS5607 requires a state machine
    switch (baro_state) {
        case BARO_STATE_IDLE:
            // Start pressure conversion
            if (baro_start_conversion(0xD1)) {  // D1 = pressure
                baro_state = BARO_STATE_CONVERT_D1;
            } else {
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
                sensor_stats.baro_errors++;
                return false;
            }
            // Need to wait for conversion (will be called again by timer)
            spi1_active_sensor = ACTIVE_SENSOR_NONE;
            return true;

        case BARO_STATE_CONVERT_D1:
            // Conversion complete, read ADC
            if (baro_read_adc_dma()) {
                baro_state = BARO_STATE_READ_D1;
                return true;  // DMA in progress
            } else {
                baro_state = BARO_STATE_IDLE;
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
                sensor_stats.baro_errors++;
                return false;
            }

        case BARO_STATE_READ_D1:
            // D1 read complete, start temperature conversion
            baro_d1_raw = (baro_dma_rx_buffer[1] << 16) |
                         (baro_dma_rx_buffer[2] << 8) |
                          baro_dma_rx_buffer[3];

            if (baro_start_conversion(0xD2)) {  // D2 = temperature
                baro_state = BARO_STATE_CONVERT_D2;
            } else {
                baro_state = BARO_STATE_IDLE;
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
                sensor_stats.baro_errors++;
                return false;
            }
            spi1_active_sensor = ACTIVE_SENSOR_NONE;
            return true;

        case BARO_STATE_CONVERT_D2:
            // Read temperature ADC
            if (baro_read_adc_dma()) {
                baro_state = BARO_STATE_READ_D2;
                return true;  // DMA in progress
            } else {
                baro_state = BARO_STATE_IDLE;
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
                sensor_stats.baro_errors++;
                return false;
            }

        default:
            baro_state = BARO_STATE_IDLE;
            spi1_active_sensor = ACTIVE_SENSOR_NONE;
            return false;
    }
}

void sensor_baro_process(void) {
    if (baro_state == BARO_STATE_READ_D2) {
        // Both D1 and D2 complete
        baro_d2_raw = (baro_dma_rx_buffer[1] << 16) |
                     (baro_dma_rx_buffer[2] << 8) |
                      baro_dma_rx_buffer[3];

        BARO_t baro_data;

        // Process using calibration coefficients
        if (baro_process_data(baro_dma_rx_buffer, &baro_data)) {
            sensor_stats.baro_samples++;

            // Send to data handler
            data_handler_store_baro(&baro_data);
        } else {
            sensor_stats.baro_errors++;
        }

        baro_state = BARO_STATE_IDLE;
    }

    // Release SPI1 bus
    spi1_active_sensor = ACTIVE_SENSOR_NONE;
}

bool baro_start_conversion(uint8_t conversion_type) {
	return true;
}

bool baro_read_adc_dma(void) {
	return true;
}

bool baro_process_data(const uint8_t *buffer, BARO_t *baro_data) {
	return true;
}

// BNO
bool sensor_bno_read(void) {
    // Check if I2C1 is available
    if (i2c1_active_sensor != ACTIVE_SENSOR_NONE) {
        return false;  // Bus busy
    }

    i2c1_active_sensor = ACTIVE_SENSOR_BNO;

    // Start DMA read
    if (!bno_start_read_dma()) {
        i2c1_active_sensor = ACTIVE_SENSOR_NONE;
        sensor_stats.bno_errors++;
        return false;
    }

    return true;
}

void sensor_bno_process(void) {
    BNO_t bno_data;

    // Process raw I2C buffer
    if (bno_process_data(bno_dma_buffer, &bno_data)) {
        sensor_stats.bno_samples++;

        // Send to data handler
        data_handler_store_bno(&bno_data);
    } else {
        sensor_stats.bno_errors++;
    }

    // Release I2C1 bus
    i2c1_active_sensor = ACTIVE_SENSOR_NONE;
}

bool bno_start_read_dma(void) {
	return true;
}

bool bno_process_data(const uint8_t *buffer, BNO_t *bno_data) {
	return true;
}

// Thread Function
void sensors_thread_function(void *argument) {
    printf("Sensor thread started\n");

    uint32_t ulNotificationValue;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);

    // Start timers for polled sensors
    xTimerStart(xBaroTimer, 0);
    xTimerStart(xBnoTimer, 0);

    // Print status every 5 seconds
    TickType_t last_stats_time = xTaskGetTickCount();

    for (;;) {
        // Wait for notification (interrupt, timer, or DMA complete)
        ulNotificationValue = ulTaskNotifyTake(pdFALSE, xMaxBlockTime);

        if (ulNotificationValue == 0) {
            // Timeout - check for stalled sensors
            continue;
        }

        // Check notification bits
        if (ulNotificationValue & SENSOR_NOTIFY_IMU_DRDY) {
            // IMU data ready - highest priority
            sensor_imu_read();
        }

        if (ulNotificationValue & SENSOR_NOTIFY_MAG_DRDY) {
            // Magnetometer data ready
            sensor_mag_read();
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BARO_TIMER) {
            // Barometer timer expired
            sensor_baro_read();
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BNO_TIMER) {
            // BNO055 timer expired
            sensor_bno_read();
        }

        if (ulNotificationValue & SENSOR_NOTIFY_DMA_COMPLETE) {
            // DMA transfer complete - process based on active sensor
            if (spi1_active_sensor == ACTIVE_SENSOR_IMU) {
                sensor_imu_process();
            }
            else if (spi1_active_sensor == ACTIVE_SENSOR_BARO) {
                sensor_baro_process();
            }
            else if (spi3_active_sensor == ACTIVE_SENSOR_MAG) {
                sensor_mag_process();
            }
            else if (i2c1_active_sensor == ACTIVE_SENSOR_BNO) {
                sensor_bno_process();
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_DMA_ERROR) {
            // DMA error - cleanup
            printf("ERROR: DMA error occurred\n");

            // Reset active sensors
            if (spi1_active_sensor != ACTIVE_SENSOR_NONE) {
                CS_IMU_HIGH();
                CS_BARO_HIGH();
                spi1_active_sensor = ACTIVE_SENSOR_NONE;
            }
            if (spi3_active_sensor != ACTIVE_SENSOR_NONE) {
                CS_MAG_HIGH();
                spi3_active_sensor = ACTIVE_SENSOR_NONE;
            }
            if (i2c1_active_sensor != ACTIVE_SENSOR_NONE) {
                i2c1_active_sensor = ACTIVE_SENSOR_NONE;
            }
        }

        // Print statistics periodically
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

void sensors_thread_init(void) {
		printf("Initializing sensors...\n");

	    // Initialize all sensors
	    printf("Initializing IMU...\n");
	    if (!imu_init() || !imu_configure()) {
	        printf("ERROR: IMU initialization failed!\n");
	    }

	    printf("Initializing Magnetometer...\n");
	    if (!mag_init() || !mag_configure()) {
	        printf("ERROR: Magnetometer initialization failed!\n");
	    }

	    printf("Initializing Barometer...\n");
	    if (!baro_init() || !baro_configure()) {
	        printf("ERROR: Barometer initialization failed!\n");
	    }

	    printf("Initializing BNO055...\n");
	    if (!bno_init() || !bno_configure()) {
	        printf("ERROR: BNO055 initialization failed!\n");
	    }
}

