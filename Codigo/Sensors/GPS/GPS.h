/**
 * @file GPS.h
 * @brief U-Blox GPS driver for the Magalhaes Flight Computer
 * @author Tomas Teixeira
 * @date October 2025
 *
 * This driver provides an interface to U-Blox GPS modules (NEO-7M, NEO-M8, NEO-M9)
 * using NMEA protocol over UART with DMA for continuous data reception.
 *
 * @section gps_features Features
 * - Continuous NMEA sentence reception via circular DMA buffer
 * - GGA (position) and RMC (velocity) sentence parsing
 * - UBX protocol support for configuration
 * - Non-blocking operation suitable for RTOS
 *
 * @section gps_sentences Supported NMEA Sentences
 * | Sentence | Data Provided |
 * |----------|---------------|
 * | GGA | Position, altitude, fix quality, satellites |
 * | RMC | Position, speed, course, time |
 *
 * @section gps_usage Usage
 * @code
 * UBLOX_GPS_t gps;
 * GPS_t data;
 *
 * UBLOX_GPS_Init(&gps, &huart2);
 * UBLOX_GPS_StartDMA(&gps);
 *
 * // In main loop or timer callback
 * UBLOX_GPS_Update(&gps);  // Parse available data
 * if (gps.data_ready) {
 *     UBLOX_GPS_ProcessData(&gps, &data);
 * }
 * @endcode
 *
 * @see GPS_t for output data structure
 */

#ifndef SENSORS_GPS_GPS_H_
#define SENSORS_GPS_GPS_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "defs.h"
#include "dma_msg.h"

/**
 * @defgroup GPSConfig Configuration Constants
 * @brief GPS module configuration parameters
 * @{
 */
#define UBLOX_GPS_HAS_FLASH    0    /**< 0 for NEO-7M (ROM), 1 for NEO-M8/M9 (Flash) */

#define UBLOX_GPS_BAUD_RATE          9600    /**< Default UART baud rate */
#define UBLOX_GPS_UART_BUFFER_SIZE   256     /**< Circular DMA buffer size */
#define UBLOX_GPS_MAX_SENTENCE_LEN   90      /**< Maximum NMEA sentence length */

#define MAX_BYTES_PER_UPDATE         32      /**< Max bytes to process per Update() call */
/** @} */

/**
 * @defgroup GPSSentences NMEA Sentence Identifiers
 * @brief NMEA sentence type prefixes
 * @{
 */
#define UBLOX_GPS_SENTENCE_GGA        "$GNGGA"   /**< Position/altitude sentence */
#define UBLOX_GPS_SENTENCE_RMC        "$GPRMC"   /**< Recommended minimum data */
#define UBLOX_GPS_SENTENCE_GGA_LEN    6          /**< Length of GGA prefix */
/** @} */

/**
 * @defgroup UBXProtocol UBX Protocol Constants
 * @brief U-Blox binary protocol definitions
 * @{
 */
#define UBX_SYNC1  0xB5     /**< UBX sync byte 1 */
#define UBX_SYNC2  0x62     /**< UBX sync byte 2 */

#define UBX_CLASS_CFG  0x06     /**< Configuration message class */

#define UBX_CFG_MSG    0x01     /**< Enable/disable NMEA messages */
#define UBX_CFG_RATE   0x08     /**< Set measurement rate */
#define UBX_CFG_CFG    0x09     /**< Save/load configuration */
/** @} */

/**
 * @defgroup UBXNMEAIDs NMEA Message IDs for UBX CFG-MSG
 * @brief NMEA message identifiers for configuration
 * @{
 */
#define UBX_NMEA_GGA   0xF0, 0x00   /**< GGA message ID */
#define UBX_NMEA_GLL   0xF0, 0x01   /**< GLL message ID */
#define UBX_NMEA_GSA   0xF0, 0x02   /**< GSA message ID */
#define UBX_NMEA_GSV   0xF0, 0x03   /**< GSV message ID */
#define UBX_NMEA_RMC   0xF0, 0x04   /**< RMC message ID */
#define UBX_NMEA_VTG   0xF0, 0x05   /**< VTG message ID */
#define UBX_NMEA_GRS   0xF0, 0x06   /**< GRS message ID */
#define UBX_NMEA_GST   0xF0, 0x07   /**< GST message ID */
#define UBX_NMEA_ZDA   0xF0, 0x08   /**< ZDA message ID */
#define UBX_NMEA_GBS   0xF0, 0x09   /**< GBS message ID */
/** @} */

/**
 * @defgroup GPSParsing Legacy NMEA Parsing Functions
 * @brief Standalone NMEA parsing functions
 * @{
 */

/**
 * @brief Save GPS data from parsed sentence
 * @param[out] gps     GPS data structure to populate
 * @param[in]  GPS_dma Raw NMEA sentence string
 */
void GPS_save_data(GPS_t* gps, char *GPS_dma);

/**
 * @brief Validate NMEA sentence checksum
 * @param[in] nmeastr NMEA sentence string
 * @return 1 if valid, 0 if invalid
 */
int GPS_validate(char *nmeastr);

/**
 * @brief Parse NMEA sentence into GPS structure
 * @param[out] gps        GPS data structure to populate
 * @param[in]  GPSstrParse NMEA sentence to parse
 */
void GPS_parse(GPS_t* gps, char *GPSstrParse);

/**
 * @brief Convert NMEA coordinate to decimal degrees
 * @param[in] deg_coord NMEA format coordinate (DDDMM.MMMM)
 * @param[in] nsew      Direction indicator (N/S/E/W)
 * @return Decimal degrees (negative for S/W)
 */
float GPS_nmea_to_dec(float deg_coord, char nsew);

/** @} */

/**
 * @brief GPS driver state machine states
 */
typedef enum {
    UBLOX_STATE_IDLE,           /**< Waiting to start */
    UBLOX_STATE_SEARCHING,      /**< Looking for '$' start character */
    UBLOX_STATE_RECEIVING,      /**< Collecting sentence bytes */
    UBLOX_STATE_COMPLETE,       /**< Complete sentence available */
} UBLOX_state_t;

/**
 * @brief U-Blox GPS driver context
 *
 * Contains all state for continuous GPS operation including:
 * - UART handle and DMA buffers
 * - State machine for sentence parsing
 * - Statistics for debugging
 */
typedef struct {
    UART_HandleTypeDef *huart;      /**< UART peripheral handle */

    /* Circular DMA buffer for continuous streaming */
    uint8_t dma_buffer[UBLOX_GPS_UART_BUFFER_SIZE];  /**< DMA receive buffer */
    volatile uint16_t dma_head;     /**< Current DMA write position */
    uint16_t parse_tail;            /**< Current parse read position */

    /* Message parsing buffer */
    uint8_t message_buffer[UBLOX_GPS_MAX_SENTENCE_LEN];  /**< Sentence buffer */
    uint16_t message_len;           /**< Current message length */

    /* State machine */
    volatile UBLOX_state_t state;   /**< Current parser state */
    volatile uint8_t data_ready;    /**< Flag: complete data available */

    /* Statistics */
    uint32_t sentences_received;    /**< Total valid sentences parsed */
    uint32_t parse_errors;          /**< Checksum or format errors */
} UBLOX_GPS_t;

/**
 * @defgroup GPSAPI U-Blox GPS Public API
 * @brief Driver functions for U-Blox GPS
 * @{
 */

/**
 * @brief Initialize GPS driver
 *
 * Sets up the driver context with UART handle.
 * Does not start DMA reception.
 *
 * @param[out] dev   Pointer to driver context to initialize
 * @param[in]  huart UART peripheral handle
 *
 * @return true if initialization successful
 * @return false if invalid parameters
 */
bool UBLOX_GPS_Init(UBLOX_GPS_t *dev, UART_HandleTypeDef *huart);

/**
 * @brief Start DMA reception
 *
 * Begins continuous circular DMA reception of GPS data.
 * Must be called before Update() will return data.
 *
 * @param[in] dev Pointer to initialized driver context
 *
 * @return true if DMA started successfully
 * @return false if error
 */
bool UBLOX_GPS_StartDMA(UBLOX_GPS_t *dev);

/**
 * @brief Process received data
 *
 * Parses available data from the DMA buffer. Should be called
 * periodically (e.g., from timer callback or main loop).
 * Processes up to MAX_BYTES_PER_UPDATE bytes per call.
 *
 * @param[in,out] dev Pointer to driver context
 *
 * @return true if complete sentence parsed
 * @return false if no complete sentence yet
 *
 * @note Non-blocking, suitable for RTOS
 */
bool UBLOX_GPS_Update(UBLOX_GPS_t *dev);

/**
 * @brief Extract parsed GPS data
 *
 * Copies the last parsed GPS data to the output structure.
 * Clears the data_ready flag.
 *
 * @param[in]  dev    Pointer to driver context
 * @param[out] output Pointer to GPS_t structure to populate
 *
 * @return true if data extracted successfully
 * @return false if no data available
 */
bool UBLOX_GPS_ProcessData(UBLOX_GPS_t *dev, GPS_t *output);

/**
 * @brief Get current DMA buffer position
 *
 * Returns the current DMA write position for debugging.
 *
 * @param[in] dev Pointer to driver context
 *
 * @return Current DMA head position
 */
uint16_t UBLOX_GPS_GetDMAPosition(UBLOX_GPS_t *dev);

/** @} */

/**
 * @defgroup GPSConfig U-Blox Configuration Functions
 * @brief UBX protocol configuration functions
 * @{
 */

/**
 * @brief Send UBX protocol message
 *
 * Sends a UBX binary protocol message to configure the GPS.
 *
 * @param[in] dev         Pointer to driver context
 * @param[in] msg_class   UBX message class
 * @param[in] msg_id      UBX message ID
 * @param[in] payload     Pointer to payload data
 * @param[in] payload_len Length of payload
 *
 * @return true if message sent successfully
 * @return false if error
 */
bool UBLOX_GPS_SendUBX(UBLOX_GPS_t *dev, uint8_t msg_class, uint8_t msg_id,
                       uint8_t *payload, uint16_t payload_len);

/**
 * @brief Configure minimal NMEA output
 *
 * Disables unnecessary NMEA sentences, keeping only GGA and RMC.
 * Reduces UART traffic and parsing overhead.
 *
 * @param[in] dev Pointer to driver context
 *
 * @return true if configuration successful
 * @return false if error
 */
bool UBLOX_GPS_ConfigureMinimal(UBLOX_GPS_t *dev);

/**
 * @brief Set GPS update rate
 *
 * Configures the GPS measurement rate.
 *
 * @param[in] dev     Pointer to driver context
 * @param[in] rate_ms Update period in milliseconds (e.g., 100 for 10 Hz)
 *
 * @return true if configuration successful
 * @return false if error
 */
bool UBLOX_GPS_SetRate(UBLOX_GPS_t *dev, uint16_t rate_ms);

/**
 * @brief Save configuration to flash
 *
 * Saves current configuration to GPS module flash memory.
 * Only works on modules with flash (NEO-M8, NEO-M9).
 *
 * @param[in] dev Pointer to driver context
 *
 * @return true if save command sent
 * @return false if error or module has no flash
 */
bool UBLOX_GPS_SaveConfig(UBLOX_GPS_t *dev);

/** @} */

#endif /* SENSORS_GPS_GPS_H_ */
