/**
 * @file e22_uart_dma.h
 * @brief E22 LoRa module driver using UART with DMA for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This driver provides an interface to the Ebyte E22 series LoRa modules
 * (E22-400T30D, E22-900T30D) using UART with DMA for efficient data transfer.
 *
 * @section e22_features Features
 * - UART communication with DMA (non-blocking TX/RX)
 * - Multiple operating modes (Normal, WOR, Config, Sleep)
 * - Circular receive buffer for continuous reception
 * - Transmission statistics tracking
 *
 * @section e22_modes Operating Modes
 * | Mode | M0 | M1 | Function |
 * |------|----|----|----------|
 * | Normal | 0 | 0 | TX/RX operation |
 * | WOR | 1 | 0 | Wake on Radio (low power) |
 * | Config | 0 | 1 | Configuration mode |
 * | Sleep | 1 | 1 | Sleep (lowest power) |
 *
 * @section e22_usage Usage
 * @code
 * // Initialize
 * E22_Init(&huart1);
 * E22_SetMode(E22_MODE_NORMAL);
 *
 * // Transmit
 * uint8_t data[] = {0x01, 0x02, 0x03};
 * E22_Transmit(data, sizeof(data));
 *
 * // Receive
 * if (E22_Available() > 0) {
 *     uint8_t buf[64];
 *     int len = E22_Receive(buf, sizeof(buf));
 * }
 * @endcode
 */

#ifndef RADIO_LORA_DRIVERS_E22_UART_DMA_H_
#define RADIO_LORA_DRIVERS_E22_UART_DMA_H_

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/**
 * @defgroup E22Modes Operating Mode Constants
 * @brief E22 module operating modes controlled by M0/M1 pins
 * @{
 */
#define E22_MODE_NORMAL  0      /**< M0=0, M1=0: Normal TX/RX operation */
#define E22_MODE_WOR     1      /**< M0=1, M1=0: Wake on Radio (low power RX) */
#define E22_MODE_CONFIG  2      /**< M0=0, M1=1: Configuration mode */
#define E22_MODE_SLEEP   3      /**< M0=1, M1=1: Sleep mode (lowest power) */
/** @} */

/**
 * @brief E22 driver status codes
 */
typedef enum {
    E22_OK = 0,             /**< Operation successful */
    E22_ERR_BUSY,           /**< Module busy (previous TX not complete) */
    E22_ERR_TIMEOUT,        /**< Operation timed out */
    E22_ERR_OVERFLOW,       /**< Receive buffer overflow */
    E22_ERR_INVALID_PARAM   /**< Invalid parameter provided */
} E22_Status_t;

/**
 * @brief E22 communication statistics
 *
 * Tracks packet and byte counts for debugging and monitoring.
 */
typedef struct {
    uint32_t tx_packets;    /**< Total packets transmitted */
    uint32_t tx_bytes;      /**< Total bytes transmitted */
    uint32_t rx_packets;    /**< Total packets received */
    uint32_t rx_bytes;      /**< Total bytes received */
    uint32_t rx_overruns;   /**< Receive buffer overrun count */
    uint32_t tx_failures;   /**< Failed transmission count */
} E22_Stats_t;

/**
 * @defgroup E22API E22 Public API
 * @brief Driver functions for E22 LoRa module
 * @{
 */

/**
 * @brief Initialize E22 driver
 *
 * Sets up the UART handle and initializes internal buffers.
 * Starts DMA reception in circular mode.
 *
 * @param[in] huart UART peripheral handle
 *
 * @return true if initialization successful
 * @return false if error
 */
bool E22_Init(UART_HandleTypeDef *huart);

/**
 * @brief Set E22 operating mode
 *
 * Controls M0/M1 GPIO pins to set the operating mode.
 * Module requires ~2ms to switch modes.
 *
 * @param[in] mode Operating mode (E22_MODE_*)
 */
void E22_SetMode(uint8_t mode);

/**
 * @brief Reset E22 module
 *
 * Performs a hardware reset using the reset pin.
 *
 * @return true if reset successful
 * @return false if error
 */
bool E22_Reset(void);

/**
 * @brief Transmit data
 *
 * Sends data via UART DMA. Non-blocking if previous TX is complete.
 *
 * @param[in] data   Pointer to data buffer
 * @param[in] length Number of bytes to transmit
 *
 * @return E22_OK if transmission started
 * @return E22_ERR_BUSY if previous TX not complete
 * @return E22_ERR_INVALID_PARAM if invalid parameters
 */
E22_Status_t E22_Transmit(uint8_t *data, uint16_t length);

/**
 * @brief Get number of bytes available to read
 *
 * @return Number of bytes in receive buffer
 */
uint16_t E22_Available(void);

/**
 * @brief Receive data
 *
 * Copies received data from internal buffer to user buffer.
 *
 * @param[out] buffer    Destination buffer
 * @param[in]  max_length Maximum bytes to copy
 *
 * @return Number of bytes actually copied
 * @retval -1 if error
 */
int E22_Receive(uint8_t *buffer, uint16_t max_length);

/**
 * @brief Flush receive buffer
 *
 * Clears all pending received data.
 */
void E22_FlushRx(void);

/**
 * @brief Check if module is busy
 *
 * Checks both AUX pin and internal TX state.
 *
 * @return true if busy
 * @return false if ready
 */
bool E22_IsBusy(void);

/**
 * @brief Get communication statistics
 *
 * @return Copy of current statistics structure
 */
E22_Stats_t E22_GetStats(void);

/**
 * @brief Reset statistics counters
 */
void E22_ResetStats(void);

/** @} */

/**
 * @defgroup E22Callbacks DMA Callback Functions
 * @brief Functions called from HAL UART callbacks
 * @{
 */

/**
 * @brief TX complete callback
 *
 * Call from HAL_UART_TxCpltCallback when using E22 UART.
 */
void E22_UART_TxCpltCallback(void);

/**
 * @brief RX complete callback
 *
 * Call from HAL_UART_RxCpltCallback when using E22 UART.
 */
void E22_UART_RxCpltCallback(void);

/**
 * @brief Error callback
 *
 * Call from HAL_UART_ErrorCallback when using E22 UART.
 */
void E22_UART_ErrorCallback(void);

/** @} */

#endif /* RADIO_LORA_DRIVERS_E22_UART_DMA_H_ */
