#include "ASM330LHHX.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


// asm330lhhx_reg variables
stmdev_ctx_t dev_ctx;
asm330lhhx_pin_int1_route_t int1_route;
static uint8_t whoamI, rst;

//Buffers and Info
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temp;

// Private Functions
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);

bool imu_init(void) {
	// Initialize mems driver interface.
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.mdelay = platform_delay;
	dev_ctx.handle = SPI_IMU_BARO;

	// Wait sensor boot time
	HAL_Delay(BOOT_TIME);

	// Check device ID
	asm330lhhx_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != ASM330LHHX_ID) return false;
	return true;
}

bool imu_configure(void){
	// Restore default configuration
	asm330lhhx_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do{ asm330lhhx_reset_get(&dev_ctx, &rst);}
	while (rst);

	// Disable I3C interface
	asm330lhhx_i3c_disable_set(&dev_ctx, ASM330LHHX_I3C_DISABLE);

	//Set Output Data Rate Full Speed
	asm330lhhx_xl_data_rate_set(&dev_ctx, ASM330LHHX_XL_ODR_6667Hz);
	asm330lhhx_gy_data_rate_set(&dev_ctx, ASM330LHHX_GY_ODR_6667Hz);

	// Set full scale
	asm330lhhx_xl_full_scale_set(&dev_ctx, ASM330LHHX_2g);
	asm330lhhx_gy_full_scale_set(&dev_ctx, ASM330LHHX_2000dps);

	//interrupt generation on Free Fall INT1 pin
	asm330lhhx_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.md1_cfg.int1_ff = PROPERTY_ENABLE;
	asm330lhhx_pin_int1_route_set(&dev_ctx, &int1_route);

	return true;
}

bool imu_start_read_dma(void) {

	uint8_t reg;
	uint32_t status;
	asm330lhhx_xl_flag_data_ready_get(&dev_ctx, &reg);
	if (reg){
	    //Read acceleration field data
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		status = asm330lhhx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		if (status != 0) {return false;}
	}

	// read output only if new gyro value is available
	asm330lhhx_gy_flag_data_ready_get(&dev_ctx, &reg);
	if (reg){
		// Read angular rate field data
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		status = asm330lhhx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		if (status != 0) {return false;}
	}

	// read output only if new temp is available
	asm330lhhx_temp_flag_data_ready_get(&dev_ctx, &reg);
	if (reg) {
		memset(&data_raw_temp, 0x00, sizeof(int16_t));
		status = asm330lhhx_temperature_raw_get(&dev_ctx, &data_raw_temp);
		if (status != 0) {return false;}
	}
	return true;
}

bool imu_process_data(IMU_t *imu_data) {

	uint8_t reg;
	uint32_t status;
	status = asm330lhhx_xl_flag_data_ready_get(&dev_ctx, &reg);
	if (status != 0) {return false;}
	if (reg){
		imu_data->accel_x = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[0]);
		imu_data->accel_y = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[1]);
		imu_data->accel_z = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[2]);
	}

	status = asm330lhhx_gy_flag_data_ready_get(&dev_ctx, &reg);
	if (status != 0) {return false;}
	if (reg){
		imu_data->gyro_x = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		imu_data->gyro_y = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		imu_data->gyro_z = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
	}

	status = asm330lhhx_temp_flag_data_ready_get(&dev_ctx, &reg);
	if (status != 0) {return false;}
	if (reg) {
		imu_data->temperature_c = asm330lhhx_from_lsb_to_celsius(data_raw_temp);
	}
	imu_data->timestamp_ms = xTaskGetTickCount();

	return true;
}

// Platform functions
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {

	HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(handle, &reg, 1);
	HAL_SPI_Transmit_DMA(handle, (uint8_t*) bufp, len);
	HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET);
	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){

	reg |= 0x80;
	HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(handle, &reg, 1);
	HAL_SPI_Receive_DMA(handle, bufp, len);
	HAL_GPIO_WritePin(CS_IMU_PORT, CS_IMU_PIN, GPIO_PIN_SET);
	return 0;
}

static void platform_delay(uint32_t ms) {
	HAL_Delay(ms);
}

