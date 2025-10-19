/*
 * ASM330LHHX.c - 6-axis IMU Driver Implementation
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


    HAL_Delay(BOOT_TIME);

    // Check device ID
    if (asm330lhhx_device_id_get(&dev_ctx, &whoami) != 0) {return false;}
    if (whoami != ASM330LHHX_ID) {
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
    uint8_t tx_buffer[15];
    tx_buffer[0] = (1 << 7) | (0x20 << 1);  // Read from TEMP_OUT_L (0x20)
    memset(&tx_buffer[1], 0x00, 14);

    // Single DMA TransmitReceive
    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, tx_buffer, dev->read_buffer, 15) != HAL_OK) {
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
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, (uint8_t*)bufp, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive((SPI_HandleTypeDef*)handle, bufp, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET);
    return 0;
}

static void platform_delay(uint32_t ms) {
    HAL_Delay(ms);
}
