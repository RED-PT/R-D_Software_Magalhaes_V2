/*
 * sensors_thread.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Tomas Teixeira
 */

#include "sensors_thread.h"

// Sensor Instances
ASM330LHHX_t imu_device;
MMC5983MA_t mag_device;
MS5607_t baro_device;
BNO055_t bno_device;
UBLOX_GPS_t gps_device;

// Bus Tracking (make these accessible to hal_callbacks.c)
volatile active_sensor_t spi1_active_sensor = ACTIVE_SENSOR_NONE;
volatile active_sensor_t spi3_active_sensor = ACTIVE_SENSOR_NONE;
volatile active_sensor_t i2c1_active_sensor = ACTIVE_SENSOR_NONE;

// Statistics
static struct {
    uint32_t imu_samples;
    uint32_t mag_samples;
    uint32_t baro_samples;
    uint32_t bno_samples;
    uint32_t gps_samples;
    uint32_t imu_errors;
    uint32_t mag_errors;
    uint32_t baro_errors;
    uint32_t bno_errors;
    uint32_t gps_errors;
    uint32_t dma_errors;
} sensor_stats = {0};

// Timer Callbacks
void vBaroTimerCallback(TimerHandle_t xTimer) {
    if (osKernelGetState() == osKernelRunning) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_BARO_TIMER, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void vBnoTimerCallback(TimerHandle_t xTimer) {
    if (osKernelGetState() == osKernelRunning) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(sensors_thread_id, SENSOR_NOTIFY_BNO_TIMER, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Initialization
void sensors_thread_init(void) {
    printf("Initializing sensors...\r\n");

    // Initialize IMU
    printf("Initializing IMU...\r\n");
    bool imu_ok = ASM330LHHX_Init(&imu_device, SPI_IMU_BARO);
    fsm_report_init_status("IMU_INIT", imu_ok);

    if (!imu_ok) {
        printf("ERROR: IMU init failed!\r\n");
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    } else {
        bool imu_cfg = ASM330LHHX_Configure(&imu_device);
        fsm_report_init_status("IMU_CONFIG", imu_cfg);
        if (!imu_cfg) {
            printf("ERROR: IMU configure failed!\r\n");
        }
    }

    // Initialize Magnetometer
    printf("Initializing Magnetometer...\r\n");
    bool mag_ok = MMC5983MA_Init(&mag_device, SPI_MAG);
    fsm_report_init_status("MAG_INIT", mag_ok);

    if (!mag_ok) {
        printf("ERROR: MAG init failed!\r\n");
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
    } else {
        bool mag_cfg = MMC5983MA_Configure(&mag_device);
        fsm_report_init_status("MAG_CONFIG", mag_cfg);
        if (!mag_cfg) {
            printf("ERROR: MAG configure failed!\r\n");
        }
    }

    // Initialize Barometer
    printf("Initializing Barometer...\r\n");
    bool baro_ok = MS5607_Init(&baro_device, SPI_IMU_BARO, CS_BARO_PORT, CS_BARO_PIN);
    fsm_report_init_status("BARO_INIT", baro_ok);
    if (!baro_ok) {
        printf("ERROR: BARO init failed!\r\n");
    }

    // Initialize BNO055
    printf("Initializing BNO055...\r\n");
    bool bno_ok = BNO055_Init(&bno_device, I2C_BNO);
    fsm_report_init_status("BNO_INIT", bno_ok);

    if (!bno_ok) {
        printf("ERROR: BNO init failed!\r\n");
    } else {
        bool bno_cfg = BNO055_Configure(&bno_device);
        fsm_report_init_status("BNO_CONFIG", bno_cfg);
        if (!bno_cfg) {
            printf("ERROR: BNO configure failed!\r\n");
        }
    }

    // Initialize GPS
    printf("Initializing GPS...\r\n");
    bool gps_ok = UBLOX_GPS_Init(&gps_device, UART_UBLOX);
    fsm_report_init_status("GPS_INIT", gps_ok);

    if (!gps_ok) {
        printf("ERROR: GPS init failed!\r\n");
    } else {
        HAL_Delay(500);
        UBLOX_GPS_ConfigureMinimal(&gps_device);
        HAL_Delay(200);
        UBLOX_GPS_SaveConfig(&gps_device);
        HAL_Delay(500);

        bool gps_dma = UBLOX_GPS_StartDMA(&gps_device);
        fsm_report_init_status("GPS_CONFIG", gps_dma);
        if (!gps_dma) {
            printf("ERROR: GPS DMA start failed!\r\n");
        }
    }

    printf("Sensors initialization complete!\r\n");
}

// Thread Main Loop - UNCHANGED from your original
void sensors_thread_function(void *argument) {
	data_handler_init();
	sensors_thread_init();

	printf("Sensors thread started...\r\n");
	fsm_report_thread_started("SENSORS");

    uint32_t ulNotificationValue;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(10);

    ulTaskNotifyTake(pdTRUE, 0);

    xTimerStart(xBaroTimer, 0);
    xTimerStart(xBnoTimer, 0);

    TickType_t last_stats_time = xTaskGetTickCount();

    while(1) {
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, xMaxBlockTime);

        // [REST OF YOUR ORIGINAL LOOP CODE - UNCHANGED]

        if (ulNotificationValue & SENSOR_NOTIFY_IMU_DRDY) {
            if (spi1_active_sensor == ACTIVE_SENSOR_NONE) {
                spi1_active_sensor = ACTIVE_SENSOR_IMU;
                if (!ASM330LHHX_StartReadDMA(&imu_device)) {
                    spi1_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.imu_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_MAG_DRDY) {
            if (spi3_active_sensor == ACTIVE_SENSOR_NONE) {
                spi3_active_sensor = ACTIVE_SENSOR_MAG;
                if (!MMC5983MA_StartReadDMA(&mag_device)) {
                    spi3_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.mag_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BARO_TIMER) {
            BARO_t baro_data;
            if (MS5607_ReadTemperatureandPressure(&baro_device, &baro_data)) {
                sensor_stats.baro_samples++;
                data_handler_store_baro(&baro_data);
            } else {
                sensor_stats.baro_errors++;
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BNO_TIMER) {
            if (i2c1_active_sensor == ACTIVE_SENSOR_NONE) {
                i2c1_active_sensor = ACTIVE_SENSOR_BNO;
                if (!BNO055_StartReadDMA(&bno_device)) {
                    i2c1_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.bno_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_GPS_DATA) {
            if (UBLOX_GPS_Update(&gps_device)) {
                ulNotificationValue |= SENSOR_NOTIFY_GPS_DR;
            }
        }

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

        if (ulNotificationValue & SENSOR_NOTIFY_GPS_DR) {
            GPS_t gps_data;
            if (UBLOX_GPS_ProcessData(&gps_device, &gps_data)) {
                sensor_stats.gps_samples++;
                data_handler_store_gps(&gps_data);
            } else {
                sensor_stats.gps_errors++;
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_DMA_ERROR) {
            printf("ERROR: DMA error occurred\r\n");
            CS_IMU_HIGH();
            CS_BARO_HIGH();
            CS_MAG_HIGH();
            spi1_active_sensor = ACTIVE_SENSOR_NONE;
            spi3_active_sensor = ACTIVE_SENSOR_NONE;
            i2c1_active_sensor = ACTIVE_SENSOR_NONE;
        }

        if ((xTaskGetTickCount() - last_stats_time) > pdMS_TO_TICKS(10000)) {
            printf("Sensor Statistics\r\n");
            printf("IMU:  %lu samples, %lu errors\r\n", sensor_stats.imu_samples, sensor_stats.imu_errors);
            printf("MAG:  %lu samples, %lu errors\r\n", sensor_stats.mag_samples, sensor_stats.mag_errors);
            printf("BARO: %lu samples, %lu errors\r\n", sensor_stats.baro_samples, sensor_stats.baro_errors);
            printf("BNO:  %lu samples, %lu errors\r\n", sensor_stats.bno_samples, sensor_stats.bno_errors);
            printf("GPS:  %lu samples, %lu errors\r\n", sensor_stats.gps_samples, sensor_stats.gps_errors);
            printf("DMA errors: %lu\r\n", sensor_stats.dma_errors);
            last_stats_time = xTaskGetTickCount();
        }
    }
}
