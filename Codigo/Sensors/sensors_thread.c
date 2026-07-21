/**
 * @file sensors_thread.c
 * @brief Sensor Acquisition Thread Implementation
 * @author Tomás Teixeira
 * @date October 10, 2025
 * @version 2.0
 *
 * @details
 * Implements the sensor acquisition and processing for the Magalhães Flight
 * Computer. This thread manages all hardware sensors using DMA transfers
 * for efficient, low-latency data collection.
 *
 * Supports multiple board targets; peripheral instances are abstracted
 * through config.h macros (SPI_IMU_BARO, SPI_MAG, I2C_BNO, UART_UBLOX).
 *
 * ## Sensor Overview
 * | Sensor | config.h macro | Rate | Data Type |
 * |------------|----------------|--------|-------------------|
 * | ASM330LHHX | SPI_IMU_BARO | 416 Hz | 6-axis IMU (DMA) |
 * | MMC5983MA | SPI_MAG | 100 Hz | 3-axis MAG (DMA) |
 * | MS5607 | SPI_IMU_BARO | 50 Hz | Barometer (poll) |
 * | BNO055 | I2C_BNO | 100 Hz | 9-DOF Fusion (DMA)|
 * | u-blox GPS | UART_UBLOX | 1 Hz | Position/Vel (DMA)|
 *
 * ## Data Flow
 *
 * @verbatim
 *   Hardware Interrupts                     Sensor Thread
 *   ─────────────────                     ─────────────────
 *   IMU DRDY (EXTI) ─────────────────────► Start IMU DMA
 *   MAG DRDY (EXTI) ─────────────────────► Start MAG DMA
 *   Baro Timer ──────────────────────────► Poll MS5607
 *   BNO Timer ───────────────────────────► Start I2C DMA
 *   GPS UART RX ─────────────────────────► Parse NMEA
 *        │
 *        ▼
 *   DMA Complete (callback) ─────────────► Process & Store
 *        │
 *        ▼
 *   Circular Buffers (cb_imu, cb_baro, etc.)
 * @endverbatim
 *
 * ## Error Recovery
 * - I2C bus recovery after 100ms timeout
 * - DMA error handling with chip select reset
 * - Statistics tracking for diagnostics
 *
 * @see sensors_thread.h for interface documentation
 * @see hal_callbacks.c for interrupt handlers
 * @ingroup Sensors
 */

#include "sensors_thread.h"
#include "Flight Computer/flight_computer.h"  // For fsm_get_baro_calibration()
#include "cmsis_os.h"  // For osDelay()

/** @name Sensor Device Instances
 *  @brief Global sensor device structures
 *  @{
 */
DMA_BUFFER  ASM330LHHX_t imu_device;    /**< 6-axis IMU (accel + gyro) — SPI_IMU_BARO DMA */
BDMA_BUFFER MMC5983MA_t mag_device;     /**< 3-axis magnetometer — SPI_MAG BDMA */
MS5607_t baro_device;                   /**< Barometric pressure sensor — SPI_IMU_BARO blocking */
DMA_BUFFER  BNO055_t bno_device;        /**< 9-DOF orientation sensor — I2C_BNO DMA */
DMA_BUFFER  UBLOX_GPS_t gps_device;     /**< u-blox GPS receiver — UART_UBLOX DMA */
/** @} */

/** @brief Per-sensor readiness flags. Set by sensors_thread_init() once
 *  init+configure both succeed. The thread loop skips reads for sensors
 *  whose flag is false (i.e. absent or failed at boot). */
static bool imu_ready = false;
static bool mag_ready = false;
static bool baro_ready = false;
static bool bno_ready = false;

/** @name Bus Tracking Variables
 *  @brief Track which sensor is currently using each bus (for DMA routing)
 *  @{
 */
volatile active_sensor_t spi1_active_sensor = ACTIVE_SENSOR_NONE;  /**< SPI_IMU_BARO bus: IMU or BARO */
volatile active_sensor_t spi_mag_active_sensor = ACTIVE_SENSOR_NONE;  /**< SPI_MAG bus: magnetometer */
volatile active_sensor_t i2c1_active_sensor = ACTIVE_SENSOR_NONE;  /**< I2C_BNO bus: BNO055 */
/** @} */

/**
 * @brief Sensor statistics tracking structure
 * @details Tracks samples collected, errors encountered, and recovery events
 *          for diagnostic purposes. Reported every 10 seconds.
 */
static struct {
    uint32_t imu_samples;      /**< IMU samples successfully processed */
    uint32_t mag_samples;      /**< Magnetometer samples processed */
    uint32_t baro_samples;     /**< Barometer samples processed */
    uint32_t bno_samples;      /**< BNO055 samples processed */
    uint32_t gps_samples;      /**< GPS fixes processed */
    uint32_t imu_errors;       /**< IMU read/process errors */
    uint32_t mag_errors;       /**< Magnetometer errors */
    uint32_t baro_errors;      /**< Barometer errors */
    uint32_t bno_errors;       /**< BNO055 errors */
    uint32_t gps_errors;       /**< GPS parse errors */
    uint32_t dma_errors;       /**< DMA transfer errors */
    uint32_t i2c_timeouts;     /**< I2C bus timeouts */
    uint32_t i2c_recoveries;   /**< I2C bus recovery attempts */
} sensor_stats = {0};

// DMA timeout tracking
#define DMA_TIMEOUT_MS 100  // 100ms timeout for DMA operations
static uint32_t i2c1_dma_start_tick = 0;

// I2C bus recovery function
static void I2C_BusRecovery(I2C_HandleTypeDef *hi2c) {
    printf("[SENSORS] I2C bus recovery initiated\r\n");

    // Abort any pending DMA
    //HAL_I2C_Abort_IT(hi2c);
    osDelay(1);

    // De-init and re-init I2C
    HAL_I2C_DeInit(hi2c);
    osDelay(11);
    HAL_I2C_Init(hi2c);

    sensor_stats.i2c_recoveries++;
    printf("[SENSORS] I2C bus recovery complete\r\n");
}

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
        //sensors não estão wired up
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);

    } else {
        bool imu_cfg = ASM330LHHX_Configure(&imu_device);
        fsm_report_init_status("IMU_CONFIG", imu_cfg);
        if (!imu_cfg) {
            printf("ERROR: IMU configure failed!\r\n");
        }
        imu_ready = imu_cfg;
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
        mag_ready = mag_cfg;
    }

    // Initialize Barometer
    printf("Initializing Barometer...\r\n");
    bool baro_ok = MS5607_Init(&baro_device, SPI_IMU_BARO, CS_BARO_PORT, CS_BARO_PIN);
    fsm_report_init_status("BARO_INIT", baro_ok);
    if (!baro_ok) {
        printf("ERROR: BARO init failed!\r\n");
    } else {
        baro_ready = true;  // MS5607 needs no separate Configure step
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
        bno_ready = bno_cfg;
    }

    // Initialize GPS
    printf("Initializing GPS...\r\n");
    bool gps_ok = UBLOX_GPS_Init(&gps_device, UART_UBLOX);
    fsm_report_init_status("GPS_INIT", gps_ok);

    if (!gps_ok) {
        printf("ERROR: GPS init failed!\r\n");
    } else {
        // Wait for GPS to boot
        osDelay(1000);

        // Configure minimal output - NEO-7M won't save this!
        UBLOX_GPS_ConfigureMinimal(&gps_device);
        osDelay(200);

        #if UBLOX_GPS_HAS_FLASH
        UBLOX_GPS_SaveConfig(&gps_device);
        printf("GPS config saved to flash\r\n");
        osDelay(500);
        #else
        printf("GPS config NOT saved (ROM-only module)\r\n");
        #endif

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

    static uint32_t gps_update_calls = 0;
    static uint32_t gps_sentences_found = 0;
    static uint32_t gps_last_sat_count = 0;

    TickType_t last_stats_time = xTaskGetTickCount();

    while(1) {
        xTaskNotifyWait(0, ULONG_MAX, &ulNotificationValue, xMaxBlockTime);

        // [REST OF YOUR ORIGINAL LOOP CODE - UNCHANGED]

        if (ulNotificationValue & SENSOR_NOTIFY_IMU_DRDY) {
            if (imu_ready && spi1_active_sensor == ACTIVE_SENSOR_NONE) {
                spi1_active_sensor = ACTIVE_SENSOR_IMU;
                if (!ASM330LHHX_StartReadDMA(&imu_device)) {
                    spi1_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.imu_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_MAG_DRDY) {
            if (mag_ready && spi_mag_active_sensor == ACTIVE_SENSOR_NONE) {
                spi_mag_active_sensor = ACTIVE_SENSOR_MAG;
                if (!MMC5983MA_StartReadDMA(&mag_device)) {
                    spi_mag_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.mag_errors++;
                }
            }
        }

        if ((ulNotificationValue & SENSOR_NOTIFY_BARO_TIMER) && baro_ready) {
            /* Skip periodic read while FC-thread calibration is sampling the
             * MS5607 — both share the same SPI bus and baro_device. Without
             * this gate, calibration samples got corrupted by interleaved CS
             * toggles, producing a wrong reference pressure (e.g. 730 mbar
             * instead of ~1010 mbar at sea level). The skipped reads cost
             * nothing since calibration is short (~1 s) and altitude is
             * meaningless until cal completes anyway. */
            if (!fsm_is_baro_calibrating()) {
                BARO_t baro_data;
                if (MS5607_ReadWithCalibration(&baro_device, &baro_data, fsm_get_baro_calibration())) {
                    sensor_stats.baro_samples++;
                    data_handler_store_baro(&baro_data);
                } else {
                    sensor_stats.baro_errors++;
                }
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_BNO_TIMER) {
            if (bno_ready && i2c1_active_sensor == ACTIVE_SENSOR_NONE) {
                i2c1_active_sensor = ACTIVE_SENSOR_BNO;
                i2c1_dma_start_tick = HAL_GetTick();  // Track DMA start time
                if (!BNO055_StartReadDMA(&bno_device)) {
                    i2c1_active_sensor = ACTIVE_SENSOR_NONE;
                    sensor_stats.bno_errors++;
                }
            }
        }

        // Check for I2C DMA timeout (BNO055)
        if (i2c1_active_sensor != ACTIVE_SENSOR_NONE) {
            uint32_t elapsed = HAL_GetTick() - i2c1_dma_start_tick;
            if (elapsed > DMA_TIMEOUT_MS) {
                printf("[SENSORS] I2C1 DMA timeout! sensor=%d, elapsed=%lu ms\r\n",
                       i2c1_active_sensor, elapsed);
                sensor_stats.i2c_timeouts++;
                sensor_stats.bno_errors++;

                // Recover the I2C bus
                I2C_BusRecovery(bno_device.hi2c);

                // Reset active sensor flag
                i2c1_active_sensor = ACTIVE_SENSOR_NONE;
            }
        }

        if (ulNotificationValue & SENSOR_NOTIFY_GPS_DATA) {
            if (UBLOX_GPS_Update(&gps_device)) {
            	gps_sentences_found++;
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
            else if (spi_mag_active_sensor == ACTIVE_SENSOR_MAG) {
                MAG_t mag_data;
                if (MMC5983MA_ProcessData(&mag_device, &mag_data)) {
                    sensor_stats.mag_samples++;
                    data_handler_store_mag(&mag_data);
                } else {
                    sensor_stats.mag_errors++;
                }
                spi_mag_active_sensor = ACTIVE_SENSOR_NONE;
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
                gps_last_sat_count = gps_data.satellites;
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
            spi_mag_active_sensor = ACTIVE_SENSOR_NONE;
            i2c1_active_sensor = ACTIVE_SENSOR_NONE;
        }

        if ((xTaskGetTickCount() - last_stats_time) > pdMS_TO_TICKS(10000)) {
            printf("Sensor Statistics\r\n");
            printf("IMU:  %lu samples, %lu errors\r\n", sensor_stats.imu_samples, sensor_stats.imu_errors);
            printf("MAG:  %lu samples, %lu errors\r\n", sensor_stats.mag_samples, sensor_stats.mag_errors);
            printf("BARO: %lu samples, %lu errors\r\n", sensor_stats.baro_samples, sensor_stats.baro_errors);
            printf("BNO:  %lu samples, %lu errors\r\n", sensor_stats.bno_samples, sensor_stats.bno_errors);
            printf("GPS:  %lu samples, %lu errors, %lu sats, updates=%lu, sentences=%lu\r\n",
                   sensor_stats.gps_samples, sensor_stats.gps_errors,
                   gps_last_sat_count, gps_update_calls, gps_sentences_found);
            printf("DMA errors: %lu, I2C timeouts: %lu, I2C recoveries: %lu\r\n",
                   sensor_stats.dma_errors, sensor_stats.i2c_timeouts, sensor_stats.i2c_recoveries);
            last_stats_time = xTaskGetTickCount();
        }
    }
}
