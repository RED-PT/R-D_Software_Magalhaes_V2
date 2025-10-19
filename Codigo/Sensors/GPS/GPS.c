/*
 * GPS.c
 *
 *  Created on: Oct 19, 2025
 *      Author: texman
 */

#include "GPS.h"
#include <string.h>

// External functions from gps.c (reuse existing code)
extern void GPS_save_data(GPS_t* gps, char *GPS_dma);
extern int GPS_validate(char *nmeastr);
extern void GPS_parse(GPS_t* gps, char *GPSstrParse);

bool UBLOX_GPS_Init(UBLOX_GPS_t *dev, UART_HandleTypeDef *huart) {
    if (!dev || !huart) {
        return false;
    }

    dev->huart = huart;
    dev->dma_head = 0;
    dev->parse_tail = 0;
    dev->message_len = 0;
    dev->state = UBLOX_STATE_IDLE;
    dev->data_ready = 0;
    dev->sentences_received = 0;
    dev->parse_errors = 0;

    memset(dev->dma_buffer, 0, UBLOX_GPS_UART_BUFFER_SIZE);
    memset(dev->message_buffer, 0, UBLOX_GPS_MAX_SENTENCE_LEN);

    return true;
}

bool UBLOX_GPS_StartDMA(UBLOX_GPS_t *dev) {
    if (!dev || !dev->huart) {
        return false;
    }

    // Start circular DMA receive on UART
    // This will run continuously and wrap around the buffer
    if (HAL_UART_Receive_DMA(dev->huart, dev->dma_buffer,
                             UBLOX_GPS_UART_BUFFER_SIZE) != HAL_OK) {
        return false;
    }
    // Disable half-transfer interrupt (we only care about new data)
    __HAL_DMA_DISABLE_IT(dev->huart->hdmarx, DMA_IT_HT);

    dev->state = UBLOX_STATE_RECEIVING;

    return true;
}

uint16_t UBLOX_GPS_GetDMAPosition(UBLOX_GPS_t *dev) {
    if (!dev || !dev->huart) {
        return 0;
    }

    // Get current DMA position from hardware counter
    return UBLOX_GPS_UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(dev->huart->hdmarx);
}

bool UBLOX_GPS_Update(UBLOX_GPS_t *dev) {
    if (!dev || dev->state != UBLOX_STATE_RECEIVING) {
        return false;
    }

    uint16_t dma_pos = UBLOX_GPS_GetDMAPosition(dev);
    uint16_t bytes_available;
    uint8_t byte;

    // Calculate bytes available in buffer
    if (dma_pos >= dev->parse_tail) {
        bytes_available = dma_pos - dev->parse_tail;
    } else {
        bytes_available = (UBLOX_GPS_UART_BUFFER_SIZE - dev->parse_tail) + dma_pos;
    }

    if (bytes_available == 0) {
        return false;  // No new data
    }

    // Process each available byte
    while (bytes_available > 0) {
        byte = dev->dma_buffer[dev->parse_tail];
        dev->parse_tail = (dev->parse_tail + 1) % UBLOX_GPS_UART_BUFFER_SIZE;
        bytes_available--;

        // Look for sentence start ($)
        if (byte == '$') {
            dev->message_len = 1;
            dev->message_buffer[0] = '$';
            continue;
        }

        // If we're collecting a sentence
        if (dev->message_len > 0) {
            // Check for end of sentence (newline or CR)
            if (byte == '\n' || byte == '\r') {
                if (dev->message_len > 6) {  // Minimum NMEA length
                    dev->message_buffer[dev->message_len] = '\0';
                    dev->state = UBLOX_STATE_COMPLETE;
                    dev->data_ready = 1;
                    return true;  // Complete sentence found
                }
                dev->message_len = 0;  // Reset on too short
                continue;
            }

            // Add byte to message if buffer not full
            if (dev->message_len < UBLOX_GPS_MAX_SENTENCE_LEN - 1) {
                dev->message_buffer[dev->message_len] = byte;
                dev->message_len++;
            } else {
                // Sentence too long, discard and look for next $
                dev->message_len = 0;
                dev->parse_errors++;
            }
        }
    }

    return false;  // No complete sentence yet
}

bool UBLOX_GPS_ProcessData(UBLOX_GPS_t *dev, GPS_t *output) {
    if (!dev || !output || !dev->data_ready || dev->state != UBLOX_STATE_COMPLETE) {
        return false;
    }

    // Validate NMEA sentence
    if (!GPS_validate((char*)dev->message_buffer)) {
        dev->parse_errors++;
        dev->data_ready = 0;
        dev->state = UBLOX_STATE_RECEIVING;
        return false;
    }

    // Parse NMEA sentence into GPS struct
    GPS_parse(output, (char*)dev->message_buffer);

    // Add timestamp
    output->timestamp_ms = HAL_GetTick();

    dev->sentences_received++;
    dev->data_ready = 0;
    dev->state = UBLOX_STATE_RECEIVING;

    return true;
}
