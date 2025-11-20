/*
 * crc16.h
 *
 *  Created on: Nov 19, 2025
 *      Author: texman
 */

#ifndef RADIO_CRC16_CRC16_H_
#define RADIO_CRC16_CRC16_H_

#include <stdint.h>

uint16_t crc16_calculate(const uint8_t *data, uint16_t length);


#endif /* RADIO_CRC16_CRC16_H_ */
