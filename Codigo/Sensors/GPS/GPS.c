/**
 * @file GPS.c
 * @brief U-Blox GPS Driver and NMEA Parser Implementation
 * @author Tomas Teixeira
 * @date October 2025
 * @version 2.0
 *
 * @details
 * Implements NMEA sentence parsing (GGA, RMC) and the U-Blox GPS driver
 * with circular DMA reception and UBX binary protocol configuration.
 * Board-agnostic: UART instance is resolved via config.h macro (UART_UBLOX).
 *
 * @see GPS.h for interface documentation
 * @ingroup Sensors
 */

#include "GPS.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/** @brief Validate and parse a raw NMEA sentence, storing results in @p gps.
 *  @param[out] gps        GPS data structure to populate
 *  @param[in]  gps_buffer Raw NMEA sentence string (null-terminated)
 */
void GPS_save_data(GPS_t *gps, char *gps_buffer) {
    // validate message
    if (GPS_validate(gps_buffer)) {
        // parse message
        GPS_parse(gps, gps_buffer);

        // gps lock
        if (gps->lock) {
            // debugging
        }
    } else {
        printf("gps: mensagem invalida\r\n");
    }
}

/**
 * @brief Convert NMEA coordinate (DDDMM.MMMM) to decimal degrees.
 * @param[in] deg_coord NMEA-format coordinate value
 * @param[in] nsew      Hemisphere indicator ('N','S','E','W'); S/W yield negative result
 * @return Coordinate in decimal degrees
 */
float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord / 100);
    float minutes = deg_coord - degree * 100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}

/**
 * @brief Validate an NMEA sentence by verifying its XOR checksum.
 * @param[in] nmeastr Null-terminated NMEA sentence (must start with '$')
 * @return 1 if checksum valid, 0 otherwise
 */
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

/**
 * @brief Parse a validated NMEA sentence (GGA or RMC) into a GPS_t structure.
 *
 * Supports $GNGGA, $GPGGA, and $GPRMC sentence types. Coordinates are
 * converted from NMEA DDDMM.MMMM format to decimal degrees.
 *
 * @param[out] gps         GPS data structure to populate
 * @param[in]  GPSstrParse Null-terminated, checksum-validated NMEA sentence
 */
void GPS_parse(GPS_t *gps, char *GPSstrParse) {
    /* All numeric fields are parsed into local temporaries and assigned
     * afterwards. The old code passed (int*)&gps->lock / &gps->satellites
     * to sscanf — those fields are uint8_t, so each %d stored 4 bytes and
     * clobbered the neighbouring struct members (undefined behaviour and
     * real data corruption). */
    float nmea_lat, nmea_lon;
    char ns, ew;
    char msl_units;
    int lock_i, sats_i;
    float utc, hdop, msl_alt;
    int fields;

    if (!strncmp(GPSstrParse, "$GNGGA", 6) || !strncmp(GPSstrParse, "$GPGGA", 6)) {
        fields = sscanf(GPSstrParse + 6, ",%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
                        &utc, &nmea_lat, &ns, &nmea_lon, &ew,
                        &lock_i, &sats_i, &hdop, &msl_alt, &msl_units);

        if (fields >= 7) {
            /* Full position fix parsed */
            gps->utc_time      = utc;
            gps->dec_latitude  = GPS_nmea_to_dec(nmea_lat, ns);
            gps->dec_longitude = GPS_nmea_to_dec(nmea_lon, ew);
            gps->lock          = (uint8_t)lock_i;
            gps->satellites    = (uint8_t)sats_i;
            if (fields >= 8) gps->hdop = hdop;
            if (fields >= 9) gps->msl_altitude = msl_alt;
        } else {
            /* GGA with empty lat/lon fields = no fix. The old `>= 1` check
             * bailed out here without touching `lock`, so once a fix had
             * been seen the GS showed "GPS locked" forever. */
            gps->lock = 0;
            gps->satellites = 0;
            if (fields >= 1) gps->utc_time = utc;
        }
        return;
    }

    else if (!strncmp(GPSstrParse, "$GPRMC", 6)) {
        char rmc_status = 'V';
        int date;
        float mag_dev, speed, course;
        char mag_dev_unit;

        fields = sscanf(GPSstrParse, "$GPRMC,%f,%c,%f,%c,%f,%c,%f,%f,%6d,%f,%c",
                        &utc, &rmc_status, &nmea_lat, &ns, &nmea_lon, &ew,
                        &speed, &course, &date, &mag_dev, &mag_dev_unit);

        if (fields >= 6) {
            gps->utc_time      = utc;
            gps->dec_latitude  = GPS_nmea_to_dec(nmea_lat, ns);
            gps->dec_longitude = GPS_nmea_to_dec(nmea_lon, ew);
            if (fields >= 7) gps->speed_k  = speed;
            if (fields >= 8) gps->course_d = course;
            gps->lock = (rmc_status == 'A') ? 1 : 0;
        } else if (fields >= 2) {
            /* Status parsed but no position — 'V' (void) means no fix. */
            if (rmc_status != 'A') gps->lock = 0;
        }
        return;
    }
}

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
    if (HAL_UART_Receive_DMA(dev->huart, dev->dma_buffer, UBLOX_GPS_UART_BUFFER_SIZE) != HAL_OK) {
        return false;
    }

    // DISABLE half-transfer interrupt - only use complete and IDLE
    __HAL_DMA_DISABLE_IT(dev->huart->hdmarx, DMA_IT_HT);

    // Enable UART IDLE interrupt
    __HAL_UART_ENABLE_IT(dev->huart, UART_IT_IDLE);

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
    uint16_t bytes_to_process;
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

    // CRITICAL FIX: Limit bytes processed per call to avoid blocking thread
    bytes_to_process = (bytes_available > MAX_BYTES_PER_UPDATE) ?
                        MAX_BYTES_PER_UPDATE : bytes_available;

    // Process limited number of bytes
    while (bytes_to_process > 0) {
        byte = dev->dma_buffer[dev->parse_tail];
        dev->parse_tail = (dev->parse_tail + 1) % UBLOX_GPS_UART_BUFFER_SIZE;
        bytes_to_process--;

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
// Calculate UBX checksum
static void UBX_CalculateChecksum(uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

// Send UBX command to GPS
bool UBLOX_GPS_SendUBX(UBLOX_GPS_t *dev, uint8_t msg_class, uint8_t msg_id,
                       uint8_t *payload, uint16_t payload_len) {
    if (!dev || !dev->huart) {
        return false;
    }

    uint8_t buffer[256];
    uint16_t idx = 0;

    // Header
    buffer[idx++] = UBX_SYNC1;
    buffer[idx++] = UBX_SYNC2;
    buffer[idx++] = msg_class;
    buffer[idx++] = msg_id;
    buffer[idx++] = payload_len & 0xFF;
    buffer[idx++] = (payload_len >> 8) & 0xFF;

    // Payload
    if (payload && payload_len > 0) {
        memcpy(&buffer[idx], payload, payload_len);
        idx += payload_len;
    }

    // Checksum (over class, id, length, payload)
    uint8_t ck_a, ck_b;
    UBX_CalculateChecksum(&buffer[2], idx - 2, &ck_a, &ck_b);
    buffer[idx++] = ck_a;
    buffer[idx++] = ck_b;

    // Send via UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(dev->huart, buffer, idx, 100);
    osDelay(50);  // Give GPS time to process

    return (status == HAL_OK);
}

// Enable/disable specific NMEA message
static bool UBLOX_GPS_SetNMEAMessage(UBLOX_GPS_t *dev, uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
    uint8_t payload[3];
    payload[0] = msg_class;
    payload[1] = msg_id;
    payload[2] = rate;  // 0=disable, 1=enable

    return UBLOX_GPS_SendUBX(dev, UBX_CLASS_CFG, UBX_CFG_MSG, payload, 3);
}

// Configure GPS for minimal output - ONLY GGA sentence at 1Hz
bool UBLOX_GPS_ConfigureMinimal(UBLOX_GPS_t *dev) {

    // Disable all NMEA messages first
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x00, 0);  // GGA off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x01, 0);  // GLL off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x02, 0);  // GSA off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x03, 0);  // GSV off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x04, 0);  // RMC off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x05, 0);  // VTG off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x06, 0);  // GRS off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x07, 0);  // GST off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x08, 0);  // ZDA off
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x09, 0);  // GBS off

    osDelay(200);

    // Enable ONLY GGA (position + altitude)
    // Or enable ONLY RMC if you prefer (has speed but less accurate altitude)
    UBLOX_GPS_SetNMEAMessage(dev, 0xF0, 0x00, 1);  // GGA on
    return true;
}

// Set GPS update rate (default 1000ms = 1Hz)
bool UBLOX_GPS_SetRate(UBLOX_GPS_t *dev, uint16_t rate_ms) {
    uint8_t payload[6];

    // measRate (measurement rate in ms)
    payload[0] = rate_ms & 0xFF;
    payload[1] = (rate_ms >> 8) & 0xFF;

    // navRate (how many measurements per nav solution, usually 1)
    payload[2] = 0x01;
    payload[3] = 0x00;

    // timeRef (0=UTC, 1=GPS time)
    payload[4] = 0x00;
    payload[5] = 0x00;

    return UBLOX_GPS_SendUBX(dev, UBX_CLASS_CFG, UBX_CFG_RATE, payload, 6);
}

// Save configuration to GPS flash (survives power cycle)
bool UBLOX_GPS_SaveConfig(UBLOX_GPS_t *dev) {
    uint8_t payload[13] = {0};

    // Save to all storage layers.
    // clearMask must be 0: "clear all + save all" first reverted the stored
    // config to defaults — the standard save recipe is clear=0, save=all.
    payload[0] = 0x00;  // clearMask: none
    payload[1] = 0x00;
    payload[2] = 0x00;
    payload[3] = 0x00;

    payload[4] = 0xFF;  // saveMask (save all)
    payload[5] = 0xFF;
    payload[6] = 0xFF;
    payload[7] = 0xFF;

    payload[8] = 0x00;  // loadMask: none (don't reload over what we just set)
    payload[9] = 0x00;
    payload[10] = 0x00;
    payload[11] = 0x00;

    payload[12] = 0x17; // deviceMask (BBR, Flash, EEPROM, SPI Flash)

    return UBLOX_GPS_SendUBX(dev, UBX_CLASS_CFG, UBX_CFG_CFG, payload, 13);
}
