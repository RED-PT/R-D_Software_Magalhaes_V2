/*
 * telemetry.c
 *
 *  Created on: Oct 7, 2025
 *      Author: Tomas Teixeira
 */

#include "telemetry.h"

// CRC16-CCITT implementation (Modbus variant)
uint16_t calculate_crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}
