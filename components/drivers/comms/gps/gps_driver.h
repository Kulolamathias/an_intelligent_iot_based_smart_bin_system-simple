/**
 * @file main/gps_driver.h
 * @brief Standalone GPS driver module – NMEA receive and parse.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This module is a self‑contained bring‑up proof for a NEO‑xM GPS receiver.
 * It directly uses the UART driver and provides a single blocking function
 * that initialises the peripheral and attempts to obtain a 3D fix.
 *
 * This module is **not** part of the layered architecture. It owns the
 * entire GPS interrogation cycle and logs results immediately. No events
 * are posted, no commands accepted – it functions as a simple, linear
 * procedure.
 *
 * Responsibilities:
 * - Define hardware pin mapping and UART configuration
 * - Declare a POD structure for parsed GPS data
 * - Expose an initialisation routine and a blocking fix‑acquisition function
 *
 * Forbidden:
 * - Must not interact with Core or Service layers
 * - Must not use dynamic memory or non‑deterministic loops
 *
 * =============================================================================
 * OWNERSHIP
 * =============================================================================
 * Owns:
 * - UART port, baud rate, pin assignments
 * - gps_data_t structure and its invariants
 *
 * Does not own:
 * - Other peripherals
 * - Any system‑level state or decision logic
 *
 * =============================================================================
 * INVARIANTS
 * =============================================================================
 * - UART is configured as 8N1, 9600 baud, hardware FIFO, standard NMEA.
 * - Parsed latitude/longitude are stored as decimal degrees (float64).
 * - Timeout is enforced by a tick‑based bound; loop cannot run forever.
 * - No null pointers are passed to the acquisition function (caller must
 *   provide a valid gps_data_t pointer).
 * - After a successful fix, latitude/longitude are guaranteed non‑zero
 *   (when fix_quality >= 1).
 *
 * @author Matthithyahu
 * @date 2026/05/10
 */

#ifndef GPS_PROOF_H
#define GPS_PROOF_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Hardware mapping & constants (can be overridden via project config)        */
/* -------------------------------------------------------------------------- */
#define GPS_UART_PORT          UART_NUM_1   /**< UART peripheral instance */
#define GPS_TXD_PIN            GPIO_NUM_13  /**< ESP32 TX -> GPS RX */
#define GPS_RXD_PIN            GPIO_NUM_14  /**< ESP32 RX -> GPS TX */
#define GPS_BAUDRATE           9600         /**< Standard NMEA baud */
#define GPS_UART_RX_BUF_SIZE   512          /**< Ring buffer size (bytes) */
#define GPS_UART_TX_BUF_SIZE   0            /**< No TX needed */

/* -------------------------------------------------------------------------- */
/* Public types                                                              */
/* -------------------------------------------------------------------------- */
/**
 * @brief Parsed GPS data (NMEA GGA/RMC summary).
 *
 * All fields are in standard decimal units.  Fields are populated only
 * when fix_quality >= 1; otherwise they contain zero.
 */
typedef struct {
    uint8_t  fix_quality;  /**< 0 = no fix, 1 = GPS fix, 2 = DGPS fix */
    double   latitude;     /**< Decimal degrees (positive = North) */
    double   longitude;    /**< Decimal degrees (positive = East) */
    float    altitude;     /**< Altitude above MSL in metres */
    float    utc_time;     /**< UTC time as hhmmss.ss (e.g. 104231.00) */
    uint8_t  satellites;   /**< Number of satellites used (from GGA) */
} gps_data_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise GPS UART and required hardware.
 *
 * Must be called once before any other GPS function.
 */
void gps_proof_init(void);

/**
 * @brief Blocking acquisition of a GPS fix.
 *
 * Attempts to read and parse NMEA sentences from the GPS module until a
 * valid 3D fix (fix_quality >= 1 and non‑zero coordinates) is obtained or
 * the timeout expires.
 *
 * @param[out] out_data   Pointer to a gps_data_t structure to fill.
 * @param[in]  timeout_ms Maximum time to wait for a fix, in milliseconds.
 *                        The loop is bounded by FreeRTOS ticks; it cannot
 *                        block indefinitely.
 *
 * @return
 *   - ESP_OK              Fix acquired and out_data populated.
 *   - ESP_ERR_TIMEOUT     No fix within the time limit.
 *   - ESP_ERR_INVALID_ARG if out_data is NULL.
 */
esp_err_t gps_proof_get_fix(gps_data_t *out_data, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* GPS_PROOF_H */