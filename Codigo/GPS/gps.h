/*
 * gps.h
 *
 *  Created on: Oct 7, 2025
 *      Author: texman
 */

#ifndef SENSORS_GPS_GPS_H_
#define SENSORS_GPS_GPS_H_

#define DMA_GPS_BUF_SIZE  90       // GPS buffer size
#define DMA_GPS_AUX_BUF_SIZE DMA_GPS_BUF_SIZE*2

// Include Defs
#include "defs.h"

// Prototypes
void GPS_save_data(GPS_t* gps, char *GPS_dma);
int GPS_validate(char *nmeastr);
void GPS_parse(GPS_t* gps, char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);

#endif /* SENSORS_GPS_GPS_H_ */
