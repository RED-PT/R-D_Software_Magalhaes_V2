/*
 * GPS.h
 *
 *  Created on: Oct 19, 2025
 *      Author: texman
 */

#ifndef SENSORS_GPS_GPS_H_
#define SENSORS_GPS_GPS_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "defs.h"

// UART configuration
#define UBLOX_GPS_BAUD_RATE          38400  // Default NEO-M9N baud rate
#define UBLOX_GPS_UART_BUFFER_SIZE   256    // Size for continuous circular buffer
#define UBLOX_GPS_MAX_SENTENCE_LEN   90     // Max NMEA sentence length

// NMEA sentence prefixes
#define UBLOX_GPS_SENTENCE_GGA        "$GNGGA"  // Position/Time
#define UBLOX_GPS_SENTENCE_RMC        "$GPRMC"  // Recommended Minimum
#define UBLOX_GPS_SENTENCE_GGA_LEN    6

// Driver state
typedef enum {
    UBLOX_STATE_IDLE,
    UBLOX_STATE_SEARCHING,     // Looking for $ in buffer
    UBLOX_STATE_RECEIVING,     // Collecting sentence
    UBLOX_STATE_COMPLETE,      // Complete sentence available
} UBLOX_state_t;

// Driver context
typedef struct {
    UART_HandleTypeDef *huart;

    // Circular DMA buffer for continuous streaming
    uint8_t dma_buffer[UBLOX_GPS_UART_BUFFER_SIZE];
    volatile uint16_t dma_head;          // Current DMA position
    uint16_t parse_tail;                 // Our parsing position

    // Message parsing
    uint8_t message_buffer[UBLOX_GPS_MAX_SENTENCE_LEN];
    uint16_t message_len;

    // State machine
    volatile UBLOX_state_t state;
    volatile uint8_t data_ready;

    // Statistics
    uint32_t sentences_received;
    uint32_t parse_errors;
} UBLOX_GPS_t;

// Public API
bool UBLOX_GPS_Init(UBLOX_GPS_t *dev, UART_HandleTypeDef *huart);
bool UBLOX_GPS_StartDMA(UBLOX_GPS_t *dev);
bool UBLOX_GPS_Update(UBLOX_GPS_t *dev);  // Call periodically to parse buffer
bool UBLOX_GPS_ProcessData(UBLOX_GPS_t *dev, GPS_t *output);

// For DMA callback
uint16_t UBLOX_GPS_GetDMAPosition(UBLOX_GPS_t *dev);

#endif /* SENSORS_GPS_GPS_H_ */
