/*
 * gps.c
 *
 *  Created on: Oct 7, 2025
 *      Author: texman
 */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "gps.h"
#include "stdbool.h"
#include "dma_msg.h"
#include "test.h"

//GPS_t GPS;

bool first_fix = true;

void GPS_save_data(GPS_t *gps, char *gps_buffer) {
	// validate message
	if (GPS_validate(gps_buffer)) {
		// parse message
		GPS_parse(gps, gps_buffer);

		// gps lock
		if (gps->lock) {
			// debugging
#if (GPS_FLAG == 1)
				printf("gps: Fix!\r\n");

			// print valores importantes
			printf("dec_longitude: %f\r\n", gps->dec_longitude);
			printf("dec_latitude: %f\r\n", gps->dec_latitude);
			printf("altitude_ft: %f\r\n", gps->altitude_ft);
			printf("date: %d\r\n", gps->date);
			printf("utc_time: %f\r\n", gps->utc_time);
			printf("speed_k: %f\r\n", gps->speed_k);
			printf("course_d: %f\r\n", gps->course_d);
			printf("course_m: %f\r\n", gps->magnetic_dev);
			printf("course_m_unit: %c\r\n", gps->magnetic_dev_unit);
			printf("\r\n");
			#endif

			if (first_fix) {
				first_fix = false;
				//reset_horizontal_kalman_state_estimate_extrapolation();
			}
		} else {
#if (GPS_FLAG)
			printf("gps: No Fix\r\n");
			#endif
		}
	} else
		printf("gps: mensagem invalida\r\n");
}

int GPS_validate(char *nmeastr) {
	char check[3];
	char checkcalcstr[3];
	int i;
	int calculated_check;
	i = 0;
	calculated_check = 0;
	// check to ensure that the string starts with a $
	if (nmeastr[i] == '$')
		i++;
	else
		return 0;
	//No NULL reached, 75 char largest possible NMEA message, no '*' reached
	while ((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < strlen(nmeastr))) {
		calculated_check ^= nmeastr[i]; // calculate the checksum
		i++;
	}
	if (i >= strlen(nmeastr)) {
		return 0; // the string was too long so return an error
	}
	if (nmeastr[i] == '*') {
		check[0] = nmeastr[i + 1];    //put hex chars in check string
		check[1] = nmeastr[i + 2];
		check[2] = 0;
	} else
		return 0;    // no checksum separator found there for invalid
	sprintf(checkcalcstr, "%02X", calculated_check);
	return ((checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1])) ?
			1 : 0;
}

void GPS_parse(GPS_t *gps, char *GPSstrParse) {
	if (!strncmp(GPSstrParse, "$GNGGA", 6)) {
		if (sscanf(GPSstrParse, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
				&gps->utc_time, &gps->nmea_latitude, &gps->ns,
				&gps->nmea_longitude, &gps->ew, &gps->lock, &gps->satelites,
				&gps->hdop, &gps->msl_altitude, &gps->msl_units) >= 1) {
			gps->dec_latitude = GPS_nmea_to_dec(gps->nmea_latitude, gps->ns);
			gps->dec_longitude = GPS_nmea_to_dec(gps->nmea_longitude, gps->ew);
			return;
		}
	}

	else if (!strncmp(GPSstrParse, "$GPGGA", 6)) {
		if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
				&gps->utc_time, &gps->nmea_latitude, &gps->ns,
				&gps->nmea_longitude, &gps->ew, &gps->lock, &gps->satelites,
				&gps->hdop, &gps->msl_altitude, &gps->msl_units) >= 1) {
			gps->dec_latitude = GPS_nmea_to_dec(gps->nmea_latitude, gps->ns);
			gps->dec_longitude = GPS_nmea_to_dec(gps->nmea_longitude, gps->ew);
			return;
		}
	}

	else if (!strncmp(GPSstrParse, "$GPRMC", 6)) {
		if (sscanf(GPSstrParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%6d,%f,%c",
				&gps->utc_time, &gps->rmc_status, &gps->nmea_latitude, &gps->ns,
				&gps->nmea_longitude, &gps->ew, &gps->speed_k, &gps->course_d,
				&gps->date, &gps->magnetic_dev, &gps->magnetic_dev_unit) >= 1) {
			gps->dec_latitude = GPS_nmea_to_dec(gps->nmea_latitude, gps->ns);
			gps->dec_longitude = GPS_nmea_to_dec(gps->nmea_longitude, gps->ew);
			if (gps->rmc_status == 'A') {
				gps->lock = 1;
			}
			return;
		}
	}
//    else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
//        if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &gps->nmea_latitude, &gps->ns, &gps->nmea_longitude, &gps->ew, &gps->utc_time, &gps->gll_status) >= 1)
//        	return;
//    }
//    else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
//        if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &gps->course_t, &gps->course_t_unit, &gps->course_m, &gps->course_m_unit, &gps->speed_k, &gps->speed_k_unit, &gps->speed_km, &gps->speed_km_unit) >= 1)
//            return;
//    }
}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
	int degree = (int) (deg_coord / 100);
	float minutes = deg_coord - degree * 100;
	float dec_deg = minutes / 60;
	float decimal = degree + dec_deg;
	if (nsew == 'S' || nsew == 'W') { // return negative
		decimal *= -1;
	}
	return decimal;
}



