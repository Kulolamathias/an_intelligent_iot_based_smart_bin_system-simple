/**
 * @file   components/services/location/gps_service.c
 * @brief  Implementation of the GPS Service – uses proven NMEA parser.
 *
 * =============================================================================
 * ARCHITECTURAL ROLE
 * =============================================================================
 * This service owns the GPS hardware via the proven driver (gps_proof).
 * It polls periodically, obtains a fix, and posts events to the core.
 * No parsing logic is duplicated here – the driver does all the work.
 *
 * @author  Matthithyahu
 * @date    2026/05/17
 */

#include "gps_service.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

/* The proven driver (copied from gps_proof.c/h) */
#include "gps_driver.h"

#include "service_interfaces.h"
#include "event_types.h"
#include "command_types.h"

/* ====================================================================== */
/* Configuration (hard‑coded, matches gps_proof)                         */
/* ====================================================================== */
#define GPS_POLL_PERIOD_MS     1000        /* Poll every 1 second */
#define GPS_GET_FIX_TIMEOUT_MS 500         /* Wait up to 500ms for a fresh fix */

/* ====================================================================== */
/* Static context                                                        */
/* ====================================================================== */
static const char *TAG = "gps_svc";

static esp_timer_handle_t   s_timer = NULL;
static SemaphoreHandle_t    s_mutex = NULL;
static gps_fix_t            s_last_fix = { .valid = false };
static bool                 s_was_valid = false;   /* for edge detection of lost fix */

/* ====================================================================== */
/* Forward declarations                                                  */
/* ====================================================================== */
static void timer_callback(void *arg);
static esp_err_t cmd_gps_start(void *ctx, void *params);
static esp_err_t cmd_gps_stop(void *ctx, void *params);

/* ====================================================================== */
/* Public API – service lifecycle                                        */
/* ====================================================================== */

esp_err_t gps_service_init(void)
{
    /* Create mutex for thread‑safe access to s_last_fix */
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Initialise the proven GPS driver (UART, NMEA parser) */
    gps_proof_init();

    /* Create periodic timer (initially stopped) */
    const esp_timer_create_args_t timer_args = {
        .callback = timer_callback,
        .arg      = NULL,
        .name     = "gps_poll"
    };
    esp_err_t ret = esp_timer_create(&timer_args, &s_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "GPS service initialised (poll period %d ms)", GPS_POLL_PERIOD_MS);
    return ESP_OK;
}

esp_err_t gps_service_register_handlers(void)
{
    esp_err_t ret;
    ret = service_register_command(CMD_GPS_START, cmd_gps_start, NULL);
    if (ret != ESP_OK) return ret;
    ret = service_register_command(CMD_GPS_STOP, cmd_gps_stop, NULL);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "Command handlers registered");
    return ESP_OK;
}

esp_err_t gps_service_start(void)
{
    /* Start periodic polling */
    esp_err_t ret = esp_timer_start_periodic(s_timer, GPS_POLL_PERIOD_MS * 1000ULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start polling timer: %s", esp_err_to_name(ret));
        return ret;
    }
    s_was_valid = false;
    ESP_LOGI(TAG, "GPS service started (polling every %d ms)", GPS_POLL_PERIOD_MS);
    return ESP_OK;
}

/* ====================================================================== */
/* Command handlers                                                       */
/* ====================================================================== */

static esp_err_t cmd_gps_start(void *ctx, void *params)
{
    (void)ctx; (void)params;
    esp_err_t ret = esp_timer_start_periodic(s_timer, GPS_POLL_PERIOD_MS * 1000ULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(ret));
        return ret;
    }
    s_was_valid = false;
    ESP_LOGI(TAG, "GPS polling started");
    return ESP_OK;
}

static esp_err_t cmd_gps_stop(void *ctx, void *params)
{
    (void)ctx; (void)params;
    esp_timer_stop(s_timer);
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_last_fix.valid = false;
        xSemaphoreGive(s_mutex);
    }
    s_was_valid = false;
    ESP_LOGI(TAG, "GPS polling stopped");
    return ESP_OK;
}

/* ====================================================================== */
/* Timer callback – obtains a fix from the proven driver and posts events */
/* ====================================================================== */
static void timer_callback(void *arg)
{
    (void)arg;

    /* Get a fresh fix (blocking up to GPS_GET_FIX_TIMEOUT_MS) */
    gps_data_t fix;
    esp_err_t ret = gps_proof_get_fix(&fix, GPS_GET_FIX_TIMEOUT_MS);

    if (ret == ESP_OK && fix.fix_quality >= 1 &&
        fix.latitude != 0.0 && fix.longitude != 0.0 &&
        isfinite(fix.latitude) && isfinite(fix.longitude)) {

        /* Update stored fix under mutex */
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            s_last_fix.valid          = true;
            s_last_fix.latitude       = fix.latitude;
            s_last_fix.longitude      = fix.longitude;
            s_last_fix.altitude_m     = fix.altitude;
            s_last_fix.satellites     = fix.satellites;
            s_last_fix.hdop           = 0.0f;   /* not provided by driver */
            s_last_fix.speed_kmh      = 0.0f;
            s_last_fix.timestamp_ms   = esp_timer_get_time() / 1000ULL;
            xSemaphoreGive(s_mutex);
        }

        /* Post EVENT_GPS_FIX_UPDATE to core */
        system_event_t ev = {
            .id = EVENT_GPS_FIX_UPDATE,
            .timestamp_us = esp_timer_get_time(),
            .source = 0,
            .data = { .gps_fix = {
                .valid = true,
                .latitude  = fix.latitude,
                .longitude = fix.longitude,
                .altitude_m = fix.altitude,
                .satellites = fix.satellites,
                .hdop = 0.0f,
                .speed_kmh = 0.0f,
                .timestamp_ms = s_last_fix.timestamp_ms
            } }
        };
        service_post_event(&ev);

        /* Reset fix‑lost tracking */
        s_was_valid = true;
        ESP_LOGD(TAG, "GPS fix posted: lat=%.6f lon=%.6f alt=%.1f sats=%u",
                 fix.latitude, fix.longitude, (double)fix.altitude, fix.satellites);
    }
    else if (ret == ESP_ERR_TIMEOUT && s_was_valid) {
        /* Fix lost – post event only once */
        system_event_t ev = {
            .id = EVENT_GPS_FIX_LOST,
            .timestamp_us = esp_timer_get_time(),
            .source = 0,
            .data = { { {0} } }
        };
        service_post_event(&ev);
        s_was_valid = false;
        ESP_LOGW(TAG, "GPS fix lost (timeout)");
    }
    /* Any other error (e.g., ESP_FAIL) is ignored – no event */
}

bool gps_service_get_last_fix(gps_fix_t *out_fix)
{
    if (out_fix == NULL) {
        return false;
    }
    if (s_mutex == NULL) {
        return false;
    }
    bool valid = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(out_fix, &s_last_fix, sizeof(gps_fix_t));
        valid = s_last_fix.valid;
        xSemaphoreGive(s_mutex);
    }
    return valid;
}