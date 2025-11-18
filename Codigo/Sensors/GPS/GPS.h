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
#include "dma_msg.h"

// UART configuration
#define UBLOX_GPS_BAUD_RATE          9600   // Default NEO-7M baud rate
#define UBLOX_GPS_UART_BUFFER_SIZE   256    // Size for continuous circular buffer
#define UBLOX_GPS_MAX_SENTENCE_LEN   90     // Max NMEA sentence length

// Processing limits
#define MAX_BYTES_PER_UPDATE         32     // Max bytes to process per Update() call

// NMEA sentence prefixes
#define UBLOX_GPS_SENTENCE_GGA        "$GNGGA"  // Position/Time
#define UBLOX_GPS_SENTENCE_RMC        "$GPRMC"  // Recommended Minimum
#define UBLOX_GPS_SENTENCE_GGA_LEN    6

// UBX Protocol - Configuration
#define UBX_SYNC1  0xB5
#define UBX_SYNC2  0x62

// UBX message classes
#define UBX_CLASS_CFG  0x06

// UBX message IDs
#define UBX_CFG_MSG    0x01  // Enable/disable messages
#define UBX_CFG_RATE   0x08  // Set measurement rate
#define UBX_CFG_CFG    0x09  // Save configuration

// NMEA message IDs for CFG-MSG
#define UBX_NMEA_GGA   0xF0, 0x00
#define UBX_NMEA_GLL   0xF0, 0x01
#define UBX_NMEA_GSA   0xF0, 0x02
#define UBX_NMEA_GSV   0xF0, 0x03
#define UBX_NMEA_RMC   0xF0, 0x04
#define UBX_NMEA_VTG   0xF0, 0x05
#define UBX_NMEA_GRS   0xF0, 0x06
#define UBX_NMEA_GST   0xF0, 0x07
#define UBX_NMEA_ZDA   0xF0, 0x08
#define UBX_NMEA_GBS   0xF0, 0x09

// NMEA parsing functions (existing)
void GPS_save_data(GPS_t* gps, char *GPS_dma);
int GPS_validate(char *nmeastr);
void GPS_parse(GPS_t* gps, char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);

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

// Public API - Driver functions
bool UBLOX_GPS_Init(UBLOX_GPS_t *dev, UART_HandleTypeDef *huart);
bool UBLOX_GPS_StartDMA(UBLOX_GPS_t *dev);
bool UBLOX_GPS_Update(UBLOX_GPS_t *dev);  // Call periodically to parse buffer
bool UBLOX_GPS_ProcessData(UBLOX_GPS_t *dev, GPS_t *output);
uint16_t UBLOX_GPS_GetDMAPosition(UBLOX_GPS_t *dev);

// Configuration functions (UBX protocol)
bool UBLOX_GPS_SendUBX(UBLOX_GPS_t *dev, uint8_t msg_class, uint8_t msg_id,
                       uint8_t *payload, uint16_t payload_len);
bool UBLOX_GPS_ConfigureMinimal(UBLOX_GPS_t *dev);
bool UBLOX_GPS_SetRate(UBLOX_GPS_t *dev, uint16_t rate_ms);
bool UBLOX_GPS_SaveConfig(UBLOX_GPS_t *dev);

#endif /* SENSORS_GPS_GPS_H_ */
