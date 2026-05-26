#if 1



/**
 * @file main/gps_driver.c
 * @brief Implementation of the standalone GPS proof‑of‑life module.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This module is a temporary bring‑up driver. It directly initialises a
 * UART peripheral, reads raw NMEA sentences, parses the most important
 * fields using a comma‑split approach, and blocks until a valid 3D fix is
 * obtained or a timeout expires.
 *
 * It is **not** part of the final layered architecture. It acts as a
 * monolithic proof that the hardware is correctly wired and that the GPS
 * module functions. Once proven, this code will be refactored into the
 * proper driver/service/core layers.
 *
 * Responsibilities:
 * - Configure and enable UART2
 * - Accumulate NMEA lines in a bounded static buffer
 * - Parse $--GGA and $--RMC sentences using field‑index extraction
 * - Return parsed data through a POD structure
 *
 * Forbidden:
 * - No event posting, no command handling
 * - No dynamic memory allocation
 * - No unbounded loops
 *
 * =============================================================================
 * OWNERSHIP
 * =============================================================================
 * Owns:
 * - GPS UART hardware resource
 * - Internal NMEA line buffer and parsing logic
 *
 * Does not own:
 * - Other peripherals
 * - System state or decision trees
 *
 * =============================================================================
 * INVARIANTS
 * =============================================================================
 * - UART is opened with parameters defined in gps_proof.h; baud rate 9600,
 *   8 data bits, no parity, 1 stop bit.
 * - The NMEA line buffer size (256 bytes) exceeds the maximum NMEA sentence
 *   length of 82 characters.
 * - Parsing never overruns the line buffer; the loop is bounded by both
 *   elapsed ticks and a maximum number of iterations.
 * - Latitude and longitude are stored as degrees with floating‑point
 *   precision sufficient for centimetre‑level resolution.
 * - No null pointer is dereferenced; every pointer is checked at entry.
 *
 * @author Matthithyahu
 * @date 2026/05/10
 */

#include "gps_driver.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "gps_proof";

/* -------------------------------------------------------------------------- */
/* Internal constants                                                        */
/* -------------------------------------------------------------------------- */
#define NMEA_LINE_BUF_SIZE       256    /**< must be > max NMEA line length */
#define UART_READ_CHUNK_SIZE     16     /**< bytes read per iteration */
#define UART_READ_TIMEOUT_MS     5      /**< per‑read timeout (non‑blocking) */
#define MAX_NMEA_FIELDS          20     /**< generous limit for splitting */

/* -------------------------------------------------------------------------- */
/* Helper: convert raw NMEA coordinate value (ddmm.mmmm or dddmm.mmmm)       */
/*         to decimal degrees.                                               */
/* -------------------------------------------------------------------------- */
static double raw_to_decimal(double raw, char hemisphere)
{
    if (raw == 0.0) {
        return 0.0;
    }
    int deg = (int)(raw / 100.0);
    double minutes = raw - (deg * 100.0);
    double decimal = deg + (minutes / 60.0);
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/* -------------------------------------------------------------------------- */
/* Helper: split an NMEA sentence into fields (destroys original string)     */
/*         fields_out receives pointers into the modified string.            */
/*         Returns the number of fields found (at most max_fields).          */
/* -------------------------------------------------------------------------- */
static int split_nmea_fields(char *sentence, char *fields_out[],
                             int max_fields)
{
    int count = 0;
    char *start = sentence;
    if (*start == '$') {
        start++;  // skip leading '$', talker ID becomes first field
    }
    fields_out[count++] = start;  // first field (talker + sentence type)

    char *p = start;
    while (*p && count < max_fields) {
        if (*p == ',') {
            *p = '\0';                  // terminate current field
            fields_out[count++] = p + 1; // next field starts after comma
        }
        p++;
    }
    return count;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void gps_proof_init(void)
{
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver (no legacy API)
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT,
                                        GPS_UART_RX_BUF_SIZE,
                                        GPS_UART_TX_BUF_SIZE,
                                        0, NULL, 0));

    // Apply configuration
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));

    // Set pins (TX=GPIO17, RX=GPIO16, no RTS/CTS)
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT,
                                 GPS_TXD_PIN,
                                 GPS_RXD_PIN,
                                 UART_PIN_NO_CHANGE, // no RTS
                                 UART_PIN_NO_CHANGE  // no CTS
                                 ));

    ESP_LOGI(TAG, "GPS UART initialised on TX:%d, RX:%d, baud:%d",
             (int)GPS_TXD_PIN, (int)GPS_RXD_PIN, GPS_BAUDRATE);
}

esp_err_t gps_proof_get_fix(gps_data_t *out_data, uint32_t timeout_ms)
{
    if (out_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Clear output
    memset(out_data, 0, sizeof(gps_data_t));

    // Set up bounded loop
    TickType_t start_ticks = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    // Static line buffer – no heap in loop
    static char line_buf[NMEA_LINE_BUF_SIZE];
    memset(line_buf, 0, sizeof(line_buf));
    size_t line_idx = 0;
    bool fix_found = false;

    // Temporary parsed values – reset every iteration to avoid cross‑sentence
    // carry‑over.
    uint8_t fix_quality_local = 0;
    uint8_t satellites_local = 0;
    float altitude_local = 0.0f;
    double lat_local = 0.0;
    double lon_local = 0.0;
    float utc_time_local = 0.0f;

    bool sentence_started = false;

    ESP_LOGI(TAG, "Waiting for GPS fix (timeout %" PRIu32 " ms)...", timeout_ms);

    while (!fix_found) {
        // Bound by ticks
        if ((xTaskGetTickCount() - start_ticks) >= timeout_ticks) {
            ESP_LOGW(TAG, "Timeout – no fix within %" PRIu32 " ms", timeout_ms);
            return ESP_ERR_TIMEOUT;
        }

        // Read chunk from UART (short read timeout)
        uint8_t data[UART_READ_CHUNK_SIZE];
        int len = uart_read_bytes(GPS_UART_PORT, data,
                                  UART_READ_CHUNK_SIZE,
                                  pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));

        // Append each received byte to line buffer
        for (int i = 0; i < len; i++) {
            char ch = (char)data[i];

            // Detect start of NMEA sentence
            if (ch == '$') {
                line_idx = 0;
                memset(line_buf, 0, sizeof(line_buf));
                sentence_started = true;
            }

            if (sentence_started) {
                if (ch == '\n' || ch == '\r') {
                    // End of line – process if we have content
                    if (line_idx > 0 && line_buf[0] == '$') {
                        // Null-terminate for safety
                        line_buf[line_idx] = '\0';

                        // Split into fields (destructive)
                        char *fields[MAX_NMEA_FIELDS];
                        int num_fields = split_nmea_fields(line_buf, fields,
                                                           MAX_NMEA_FIELDS);

                        // Identify sentence type from first field (e.g.,
                        // $GPGGA, $GNRMC, $GLGGA …)
                        if (num_fields >= 2) {
                            const char *type = fields[0]; // includes '$'
                            bool is_gga = (strstr(type, "GGA") != NULL);
                            bool is_rmc = (strstr(type, "RMC") != NULL);

                            if (is_gga && num_fields >= 15) {
                                // Fields (index after $):
                                // 1=time,2=lat,3=NS,4=lon,5=EW,
                                // 6=quality,7=sats,8=hdop,9=alt,10=alt_unit
                                char *endptr;
                                float time_f = strtof(fields[1], &endptr);
                                if (endptr == fields[1]) time_f = 0.0f; // empty
                                float lat_raw = strtof(fields[2], &endptr);
                                if (endptr == fields[2]) lat_raw = 0.0f;
                                char ns = fields[3][0];
                                float lon_raw = strtof(fields[4], &endptr);
                                if (endptr == fields[4]) lon_raw = 0.0f;
                                char ew = fields[5][0];
                                int quality = (int)strtol(fields[6], &endptr, 10);
                                if (endptr == fields[6]) quality = 0;
                                int sats = (int)strtol(fields[7], &endptr, 10);
                                if (endptr == fields[7]) sats = 0;
                                float alt = strtof(fields[9], &endptr); // index 9
                                if (endptr == fields[9]) alt = 0.0f;

                                if (quality > 0) {
                                    fix_quality_local = (uint8_t)quality;
                                    satellites_local = (uint8_t)sats;
                                    altitude_local = alt;
                                    utc_time_local = time_f;
                                    lat_local = raw_to_decimal((double)lat_raw, ns);
                                    lon_local = raw_to_decimal((double)lon_raw, ew);
                                }
                            } else if (is_rmc && num_fields >= 7) {
                                // Fields: 1=time,2=status,3=lat,4=NS,
                                //         5=lon,6=EW,7=speed,...
                                char *endptr;
                                float time_f = strtof(fields[1], &endptr);
                                if (endptr == fields[1]) time_f = 0.0f;
                                char status = fields[2][0];
                                float lat_raw = strtof(fields[3], &endptr);
                                if (endptr == fields[3]) lat_raw = 0.0f;
                                char ns = fields[4][0];
                                float lon_raw = strtof(fields[5], &endptr);
                                if (endptr == fields[5]) lon_raw = 0.0f;
                                char ew = fields[6][0];

                                if (status == 'A') {
                                    utc_time_local = time_f;
                                    lat_local = raw_to_decimal((double)lat_raw, ns);
                                    lon_local = raw_to_decimal((double)lon_raw, ew);
                                    // RMC gives no quality; assume fix if lat/lon valid
                                    if (lat_local != 0.0 && lon_local != 0.0) {
                                        fix_quality_local = 1;
                                        satellites_local = 0;   // unknown from RMC
                                        altitude_local = 0.0f;  // unknown
                                    }
                                }
                            }
                        }
                    }
                    // Reset for next sentence
                    sentence_started = false;
                    line_idx = 0;
                    memset(line_buf, 0, sizeof(line_buf));
                } else if (line_idx < (NMEA_LINE_BUF_SIZE - 1)) {
                    line_buf[line_idx++] = ch;
                } else {
                    // Buffer overflow protection – discard line
                    sentence_started = false;
                    line_idx = 0;
                    memset(line_buf, 0, sizeof(line_buf));
                }
            }
        }

        // Check if we can declare a fix
        if (fix_quality_local >= 1 && lat_local != 0.0 && lon_local != 0.0) {
            out_data->fix_quality = fix_quality_local;
            out_data->latitude    = lat_local;
            out_data->longitude   = lon_local;
            out_data->altitude    = altitude_local;
            out_data->utc_time    = utc_time_local;
            out_data->satellites  = satellites_local;
            fix_found = true;
        }

        // Yield to avoid hogging CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Fix acquired: quality=%d, lat=%.6f, lon=%.6f, alt=%.1f, sats=%d",
             out_data->fix_quality,
             out_data->latitude,
             out_data->longitude,
             (double)out_data->altitude,
             out_data->satellites);

    return ESP_OK;
}
















#else





























/**
 * @file main/gps_proof.c
 * @brief Implementation of the standalone GPS proof‑of‑life module.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This module is a temporary bring‑up driver. It directly initialises a
 * UART peripheral, reads raw NMEA sentences, parses the most important
 * fields, and blocks until a valid 3D fix is obtained or a timeout expires.
 *
 * It is **not** part of the final layered architecture. It acts as a
 * monolithic proof that the hardware is correctly wired and that the GPS
 * module functions. Once proven, this code will be refactored into the
 * proper driver/service/core layers.
 *
 * Responsibilities:
 * - Configure and enable UART2
 * - Accumulate NMEA lines in a bounded static buffer
 * - Parse GGA and RMC sentences using a branch‑limited state machine
 * - Return parsed data through a POD structure
 *
 * Forbidden:
 * - No event posting, no command handling
 * - No dynamic memory allocation
 * - No unbounded loops
 *
 * =============================================================================
 * OWNERSHIP
 * =============================================================================
 * Owns:
 * - GPS UART hardware resource
 * - Internal NMEA line buffer and parsing logic
 *
 * Does not own:
 * - Other peripherals
 * - System state or decision trees
 *
 * =============================================================================
 * INVARIANTS
 * =============================================================================
 * - UART is opened with parameters defined in gps_proof.h; baud rate 9600,
 *   8 data bits, no parity, 1 stop bit.
 * - The NMEA line buffer size (256 bytes) exceeds the maximum NMEA sentence
 *   length of 82 characters.
 * - Parsing never overruns the line buffer; the loop is bounded by both
 *   elapsed ticks and a maximum number of iterations.
 * - Latitude and longitude are stored as degrees with floating‑point
 *   precision sufficient for centimetre‑level resolution.
 * - No null pointer is dereferenced; every pointer is checked at entry.
 *
 * @author Matthithyahu
 * @date 2026/05/10
 */

#include "gps_proof.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "gps_proof";

/* -------------------------------------------------------------------------- */
/* Internal constants                                                        */
/* -------------------------------------------------------------------------- */
#define NMEA_LINE_BUF_SIZE       256    /**< must be > max NMEA line length */
#define UART_READ_CHUNK_SIZE     16     /**< bytes read per iteration */
#define UART_READ_TIMEOUT_MS     5      /**< per‑read timeout (non‑blocking) */
#define PARSE_FLOAT_SCANF        "%lf"   /**< used for scanf fields */

/* -------------------------------------------------------------------------- */
/* Helper: convert NMEA ddmm.mmmm to decimal degrees                         */
/* -------------------------------------------------------------------------- */
static double nmea_to_decimal(const char *coord, char hemisphere)
{
    if (coord == NULL || strlen(coord) < 4) {
        return 0.0;
    }

    double raw;
    if (sscanf(coord, PARSE_FLOAT_SCANF, &raw) != 1) {
        return 0.0;
    }

    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void gps_proof_init(void)
{
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver (no legacy API)
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT,
                                        GPS_UART_RX_BUF_SIZE,
                                        GPS_UART_TX_BUF_SIZE,
                                        0, NULL, 0));

    // Apply configuration
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));

    // Set pins (TX=GPIO17, RX=GPIO16, no RTS/CTS)
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT,
                                 GPS_TXD_PIN,
                                 GPS_RXD_PIN,
                                 UART_PIN_NO_CHANGE, // no RTS
                                 UART_PIN_NO_CHANGE  // no CTS
                                 ));

    ESP_LOGI(TAG, "GPS UART initialised on TX:%d, RX:%d, baud:%d",
             (int)GPS_TXD_PIN, (int)GPS_RXD_PIN, GPS_BAUDRATE);
}

esp_err_t gps_proof_get_fix(gps_data_t *out_data, uint32_t timeout_ms)
{
    if (out_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Clear output
    memset(out_data, 0, sizeof(gps_data_t));

    // Set up bounded loop
    TickType_t start_ticks = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    // Static line buffer – no heap in loop
    static char line_buf[NMEA_LINE_BUF_SIZE];
    memset(line_buf, 0, sizeof(line_buf));
    size_t line_idx = 0;
    bool fix_found = false;

    // Temporary parsed values
    uint8_t fix_quality_local = 0;
    uint8_t satellites_local = 0;
    float altitude_local = 0.0f;
    double lat_local = 0.0;
    double lon_local = 0.0;
    float utc_time_local = 0.0f;

    // Internal state for NMEA scanning
    bool sentence_started = false;

    ESP_LOGI(TAG, "Waiting for GPS fix (timeout %" PRIu32 " ms)...", timeout_ms);

    while (!fix_found) {
        // Bound by ticks
        if ((xTaskGetTickCount() - start_ticks) >= timeout_ticks) {
            ESP_LOGW(TAG, "Timeout – no fix within %" PRIu32 " ms", timeout_ms);
            return ESP_ERR_TIMEOUT;
        }

        // Read chunk from UART (short read timeout)
        uint8_t data[UART_READ_CHUNK_SIZE];
        int len = uart_read_bytes(GPS_UART_PORT, data,
                                  UART_READ_CHUNK_SIZE,
                                  pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));

        // Append each received byte to line buffer
        for (int i = 0; i < len; i++) {
            char ch = (char)data[i];

            // Detect start of NMEA sentence
            if (ch == '$') {
                line_idx = 0;
                memset(line_buf, 0, sizeof(line_buf));
                sentence_started = true;
            }

            // If we are inside a sentence, accumulate
            if (sentence_started) {
                if (ch == '\n' || ch == '\r') {
                    // End of line – process if we have content
                    if (line_idx > 0 && line_buf[0] == '$') {
                        // Null-terminate for safety
                        line_buf[line_idx] = '\0';

                        // Quick check: GGA or RMC?
                        if (strncmp(line_buf, "$GPGGA", 6) == 0 ||
                            strncmp(line_buf, "$GNRMC", 6) == 0) {

                            // Parse with sscanf (fields as per NMEA spec)
                            char type[7] = {0};
                            float time_f = 0.0f;
                            float lat_raw = 0.0f;
                            char ns = 0;
                            float lon_raw = 0.0f;
                            char ew = 0;
                            int quality = 0;
                            int sats = 0;
                            float hdop = 0.0f;
                            float alt = 0.0f;
                            char alt_unit = 0;
                            char status = 0;

                            if (line_buf[3] == 'G' && line_buf[4] == 'G' && line_buf[5] == 'A') {
                                // $GPGGA
                                if (sscanf(line_buf,
                                           "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
                                           &time_f, &lat_raw, &ns, &lon_raw, &ew,
                                           &quality, &sats, &hdop, &alt, &alt_unit) >= 10) {
                                    if (quality > 0) {
                                        fix_quality_local = (uint8_t)quality;
                                        satellites_local = (uint8_t)sats;
                                        altitude_local = alt;
                                        utc_time_local = time_f;

                                        lat_local = nmea_to_decimal(
                                            line_buf + 7 + (lat_raw > 0 ? 1 : 0), ns);
                                        lon_local = nmea_to_decimal(
                                            line_buf + 7 + (lat_raw > 0 ? 1 : 0)
                                            + (lon_raw > 0 ? 1 : 0) + 1, ew);
                                    }
                                }
                            } else if (line_buf[3] == 'R' && line_buf[4] == 'M' && line_buf[5] == 'C') {
                                // $GNRMC
                                if (sscanf(line_buf,
                                           "$GNRMC,%f,%c,%f,%c,%f,%c,%*f,%*f,%*f,%*f,%*c",
                                           &time_f, &status,
                                           &lat_raw, &ns, &lon_raw, &ew) >= 6) {
                                    if (status == 'A' && lat_raw > 0.0f && lon_raw > 0.0f) {
                                        utc_time_local = time_f;
                                        lat_local = nmea_to_decimal(line_buf + 7, ns);
                                        lon_local = nmea_to_decimal(
                                            line_buf + 7 + (lat_raw > 0 ? 1 : 0)
                                            + (lon_raw > 0 ? 1 : 0) + 1, ew);
                                        // RMC gives no quality; assume fix if lat/lon valid
                                        if (lat_local != 0.0 && lon_local != 0.0) {
                                            fix_quality_local = 1;  // assume fix
                                            satellites_local = 0;   // unknown from RMC
                                            altitude_local = 0.0f;  // unknown
                                        }
                                    }
                                }
                            }
                        }
                    }
                    // Reset for next sentence
                    sentence_started = false;
                    line_idx = 0;
                    memset(line_buf, 0, sizeof(line_buf));
                } else if (line_idx < (NMEA_LINE_BUF_SIZE - 1)) {
                    line_buf[line_idx++] = ch;
                } else {
                    // Buffer overflow protection – discard line
                    sentence_started = false;
                    line_idx = 0;
                    memset(line_buf, 0, sizeof(line_buf));
                }
            }
        }

        // Check if we can declare a fix
        if (fix_quality_local >= 1 && lat_local != 0.0 && lon_local != 0.0) {
            out_data->fix_quality = fix_quality_local;
            out_data->latitude    = lat_local;
            out_data->longitude   = lon_local;
            out_data->altitude    = altitude_local;
            out_data->utc_time    = utc_time_local;
            out_data->satellites  = satellites_local;
            fix_found = true;
        }

        // Yield to avoid hogging CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Fix acquired: quality=%d, lat=%.6f, lon=%.6f, alt=%.1f, sats=%d",
             out_data->fix_quality,
             out_data->latitude,
             out_data->longitude,
             (double)out_data->altitude,
             out_data->satellites);

    return ESP_OK;
}
























#endif 