/**
 * @file ASM330LHHX.c
 * @brief ASM330LHHX 6-Axis IMU Driver Implementation
 * @author Tomás Teixeira
 * @date 2025
 * @version 2.0
 *
 * @details
 * Implements the driver for the ST ASM330LHHX 6-axis IMU (accelerometer + gyroscope).
 * Uses DMA transfers for efficient, non-blocking sensor reads at high data rates.
 * Board-agnostic: SPI instance and chip-select pins are resolved via config.h macros
 * (SPI_IMU_BARO, CS_IMU_PORT/PIN).
 *
 * ## Sensor Specifications
 * | Parameter | Value |
 * |-----------|-------|
 * | Accelerometer Range | ±2g (configurable) |
 * | Gyroscope Range | ±2000 dps (configurable) |
 * | Output Data Rate | 6.667 kHz max |
 * | Interface | SPI (4-wire) |
 * | Interrupt | DRDY on INT1 |
 *
 * ## DMA Read Sequence
 * 1. DRDY interrupt triggers read start
 * 2. DMA reads 15 bytes (temp + accel + gyro)
 * 3. DMA complete callback parses buffer
 * 4. Data stored to circular buffer
 *
 * ## Register Map (Read Block)
 * | Offset | Register | Data |
 * |--------|----------|------|
 * | 0x20 | OUT_TEMP_L | Temperature LSB |
 * | 0x21 | OUT_TEMP_H | Temperature MSB |
 * | 0x22-0x27 | OUTX/Y/Z_L/H_A | Accel X,Y,Z |
 * | 0x28-0x2D | OUTX/Y/Z_L/H_G | Gyro X,Y,Z |
 *
 * @see ASM330LHHX.h for interface documentation
 * @see asm330lhhx_reg.c for ST HAL layer
 * @ingroup Sensors
 */

#include "ASM330LHHX.h"
#include <string.h>
#include "print.h"
#include "cmsis_os2.h"

// ST driver context (for ST library compatibility)
static stmdev_ctx_t dev_ctx;
static asm330lhhx_pin_int1_route_t int1_route;
static uint8_t whoami, rst;

// Platform functions for ST library
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

bool ASM330LHHX_Init(ASM330LHHX_t *dev, SPI_HandleTypeDef *hspi) {
	if (!dev || !hspi) {
        return false;
    }

    dev->hspi = hspi;
    dev->data_ready = 0;
    memset(dev->read_buffer, 0, sizeof(dev->read_buffer));

    // Setup ST driver context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = (void*)hspi;


    osDelay(BOOT_TIME);

    // Check device ID - initialize to invalid value first
    whoami = 0x00;
    if (asm330lhhx_device_id_get(&dev_ctx, &whoami) != 0) {
        printf("[IMU] Failed to read device ID\r\n");
        return false;
    }
    printf("[IMU] WHO_AM_I = 0x%02X (expected 0x%02X)\r\n", whoami, ASM330LHHX_ID);
    if (whoami != ASM330LHHX_ID) {
        printf("[IMU] Device ID mismatch!\r\n");
        return false;
    }

    return true;
}

bool ASM330LHHX_Configure(ASM330LHHX_t *dev) {
    if (!dev) {
        return false;
    }


    // Restore default configuration
    if (asm330lhhx_reset_set(&dev_ctx, PROPERTY_ENABLE) != 0) {return false;}
    do {
        asm330lhhx_reset_get(&dev_ctx, &rst);
    } while (rst);
    // Disable I3C interface
    if (asm330lhhx_i3c_disable_set(&dev_ctx, ASM330LHHX_I3C_DISABLE) != 0) {return false;}

    // Set full speed
    if (asm330lhhx_xl_data_rate_set(&dev_ctx, ASM330LHHX_XL_ODR_6667Hz) != 0) {return false;}
    if (asm330lhhx_gy_data_rate_set(&dev_ctx, ASM330LHHX_GY_ODR_6667Hz) != 0) {return false;}

    // Set full scales
    if (asm330lhhx_xl_full_scale_set(&dev_ctx, ASM330LHHX_2g)) {return false;}
    if (asm330lhhx_gy_full_scale_set(&dev_ctx, ASM330LHHX_2000dps)) {return false;}

    // Enable data ready interrupt
    if (asm330lhhx_pin_int1_route_get(&dev_ctx, &int1_route) != 0) {return false;}
    int1_route.md1_cfg.int1_ff = PROPERTY_ENABLE;
    if (asm330lhhx_pin_int1_route_set(&dev_ctx, &int1_route) != 0) {return false;}

    return true;
}

bool ASM330LHHX_StartReadDMA(ASM330LHHX_t *dev) {
    if (!dev || !dev->hspi) {
        return false;
    }

    // Prepare TX buffer: [COMMAND][DUMMIES]
    dev->tx_buffer[0] = (1 << 7) | (0x20 << 1);  // Read from TEMP_OUT_L (0x20)
    memset(&dev->tx_buffer[1], 0x00, 14);

    // Single DMA TransmitReceive
    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, dev->tx_buffer, dev->read_buffer, 15) != HAL_OK) {
        return false;
    }

    return true;
}

void ASM330LHHX_ParseDMABuffer(ASM330LHHX_t *dev) {
    if (!dev) {
        return;
    }

    // Parse received buffer (skip first byte which is command echo)
    dev->temp_raw = (int16_t)((dev->read_buffer[2] << 8) | dev->read_buffer[1]);

    dev->accel_raw[0] = (int16_t)((dev->read_buffer[4] << 8) | dev->read_buffer[3]);
    dev->accel_raw[1] = (int16_t)((dev->read_buffer[6] << 8) | dev->read_buffer[5]);
    dev->accel_raw[2] = (int16_t)((dev->read_buffer[8] << 8) | dev->read_buffer[7]);

    dev->gyro_raw[0] = (int16_t)((dev->read_buffer[10] << 8) | dev->read_buffer[9]);
    dev->gyro_raw[1] = (int16_t)((dev->read_buffer[12] << 8) | dev->read_buffer[11]);
    dev->gyro_raw[2] = (int16_t)((dev->read_buffer[14] << 8) | dev->read_buffer[13]);

    dev->data_ready = 1;
}

bool ASM330LHHX_ProcessData(ASM330LHHX_t *dev, IMU_t *output) {
    if (!dev || !output || !dev->data_ready) {
        return false;
    }

    // Convert raw values to physical units
    output->accel_x = asm330lhhx_from_fs2g_to_mg(dev->accel_raw[0]);
    output->accel_y = asm330lhhx_from_fs2g_to_mg(dev->accel_raw[1]);
    output->accel_z = asm330lhhx_from_fs2g_to_mg(dev->accel_raw[2]);

    output->gyro_x = asm330lhhx_from_fs2000dps_to_mdps(dev->gyro_raw[0]);
    output->gyro_y = asm330lhhx_from_fs2000dps_to_mdps(dev->gyro_raw[1]);
    output->gyro_z = asm330lhhx_from_fs2000dps_to_mdps(dev->gyro_raw[2]);

    output->temperature_c = asm330lhhx_from_lsb_to_celsius(dev->temp_raw);
    output->timestamp_ms = HAL_GetTick();

    dev->data_ready = 0;

    return true;
}

// Platform Functions

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
	HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, 100);
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, (uint8_t*)bufp, len, 100);
    HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, 100);
    HAL_SPI_Receive((SPI_HandleTypeDef*)handle, bufp, len, 100);
    HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET);
    return 0;
}

static void platform_delay(uint32_t ms) {
    osDelay(ms);
}
