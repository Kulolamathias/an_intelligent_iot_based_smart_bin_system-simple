/**
 * @file components/services/sensing/ultrasonic_service/ultrasonic_service.c
 * @brief Ultrasonic Service – implementation with noise filtering.
 *
 * =============================================================================
 * IMPLEMENTATION NOTES
 * =============================================================================
 * - Manages two ultrasonic sensors: fill-level and intention.
 * - Uses the ultrasonic driver for hardware interaction.
 * - Provides a command handler for on-demand fill level reading.
 * - Uses an ESP timer for periodic fill level measurement and MQTT publishing.
 * - The intention sensor is monitored in a FreeRTOS task that posts events on state changes.
 * - Fill level readings are median‑filtered (window of 5) to remove outliers.
 * - A deadband (3%) and a minimum interval (500 ms) prevent event flooding from sensor noise.
 * - The service is designed to be policy‑agnostic; it simply provides sensor data and events.
 *
 * =============================================================================
 * @author matthithyahu
 * @date 2026-04-02
 */

#include "ultrasonic_service.h"
#include "ultrasonic_driver.h"
#include "service_interfaces.h"
#include "command_params.h"
#include "event_types.h"
#include "mqtt_topic.h"
#include "state_manager.h"   // for state_manager_copy_context
#include "gps_service.h"     // for gps_service_get_last_fix
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "ULTRASONIC_SVC";

/* ============================================================
 * Pin definitions (Kconfig candidates, hardcoded for now)
 * ============================================================ */
#define FILL_SENSOR_TRIG     25
#define FILL_SENSOR_ECHO     34
#define INTENT_SENSOR_TRIG   32
#define INTENT_SENSOR_ECHO   35

/* ============================================================
 * Bin geometry & thresholds
 * ============================================================ */
#define BIN_HEIGHT_CM        60
#define FULL_THRESHOLD_CM    55      /* distance above which bin is considered empty (0%) */
#define EMPTY_THRESHOLD_CM   5       /* distance below which bin is considered full (100%) */
#define INTENT_DISTANCE_CM   25      /* distance below which intention is confirmed */

/* ============================================================
 * Timing & polling
 * ============================================================ */
#define FILL_PUBLISH_INTERVAL_MS     500     /* 200 ms between automatic fill level reads */
#define INTENT_POLL_INTERVAL_MS      80      /* check intention every 80 ms */

/* ============================================================
 * Noise filtering parameters for fill level
 * ============================================================ */
#define FILL_FILTER_WINDOW_SIZE      10       /* number of raw distances to keep */
#define FILL_DEADBAND_PERCENT        10       /* ignore changes smaller than ±10% */
#define FILL_MIN_INTERVAL_MS         2000     /* minimum time between two posted events */

/* ============================================================
 * Static handles
 * ============================================================ */
static ultrasonic_handle_t s_fill_sensor = NULL;
static ultrasonic_handle_t s_intent_sensor = NULL;
static TaskHandle_t s_intent_task = NULL;
static esp_timer_handle_t s_fill_timer = NULL;

/* ============================================================
 * Fill level noise filter state
 * ============================================================ */
static uint32_t s_fill_distance_buffer[FILL_FILTER_WINDOW_SIZE] = {0};
static uint8_t  s_fill_buffer_index = 0;
static uint8_t  s_fill_buffer_count = 0;
static uint8_t  s_last_posted_fill_percent = 0xFF;   /* impossible initial value */
static uint64_t s_last_post_time_us = 0;

/* ============================================================
 * Forward declarations
 * ============================================================ */
static esp_err_t handle_read_fill_level(void *context, void *params);
static uint8_t distance_to_fill_percent(uint32_t distance_cm);
static uint32_t median_filter(uint32_t *values, uint8_t count);

/* ============================================================
 * Timer callback – triggers fill level reading periodically
 * ============================================================ */
static void fill_timer_callback(void *arg)
{
    (void)arg;
    handle_read_fill_level(NULL, NULL);
}

/* ============================================================
 * Convert raw distance (cm) to fill percentage (0‑100)
 * ============================================================ */
static uint8_t distance_to_fill_percent(uint32_t distance_cm)
{
    if (distance_cm >= FULL_THRESHOLD_CM) {
        return 0;
    }
    if (distance_cm <= EMPTY_THRESHOLD_CM) {
        return 100;
    }
    uint8_t percent = (uint8_t)(((FULL_THRESHOLD_CM - distance_cm) * 100) / FULL_THRESHOLD_CM);
    if (percent > 100) {
        percent = 100;
    }
    return percent;
}

/* ============================================================
 * Median filter – returns median of the first 'count' values in array
 * ============================================================ */
static uint32_t median_filter(uint32_t *values, uint8_t count)
{
    /* Simple bubble sort – acceptable for a tiny window (5) */
    uint32_t sorted[FILL_FILTER_WINDOW_SIZE];
    memcpy(sorted, values, count * sizeof(uint32_t));

    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = i + 1; j < count; j++) {
            if (sorted[i] > sorted[j]) {
                uint32_t tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }
    return sorted[count / 2];
}

/* ============================================================
 * Command handler: read fill level, filter, post event, publish MQTT
 * ============================================================ */
static esp_err_t handle_read_fill_level(void *context, void *params)
{
    (void)context;
    (void)params;

    if (!s_fill_sensor) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Perform ultrasonic measurement */
    uint32_t pulse_us;
    esp_err_t ret = ultrasonic_driver_measure(s_fill_sensor, &pulse_us);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Fill sensor measurement failed: %d", ret);
        return ret;
    }

    /* Convert pulse width to distance (cm) */
    uint32_t distance_cm = (pulse_us * 1715) / 100000;

    /* Store raw distance in circular buffer */
    s_fill_distance_buffer[s_fill_buffer_index] = distance_cm;
    s_fill_buffer_index = (s_fill_buffer_index + 1) % FILL_FILTER_WINDOW_SIZE;
    if (s_fill_buffer_count < FILL_FILTER_WINDOW_SIZE) {
        s_fill_buffer_count++;
    }

    /* Need at least 3 readings before applying median filter */
    if (s_fill_buffer_count < 3) {
        ESP_LOGD(TAG, "Fill buffer not ready (%d/%d)", s_fill_buffer_count, FILL_FILTER_WINDOW_SIZE);
        return ESP_OK;
    }

    /* Apply median filter */
    uint32_t filtered_distance = median_filter(s_fill_distance_buffer, s_fill_buffer_count);
    uint8_t new_percent = distance_to_fill_percent(filtered_distance);

    /* Deadband and minimum interval check */
    uint64_t now_us = esp_timer_get_time();
    uint8_t percent_diff = (new_percent > s_last_posted_fill_percent) ?
                           (new_percent - s_last_posted_fill_percent) :
                           (s_last_posted_fill_percent - new_percent);

    bool significant_change = (percent_diff >= FILL_DEADBAND_PERCENT);
    bool time_ok = ((now_us - s_last_post_time_us) >= (FILL_MIN_INTERVAL_MS * 1000ULL));

    if (!significant_change && !time_ok) {
        ESP_LOGD(TAG, "Fill level change ignored (diff=%d%%, time since last=%llu ms)",
                 percent_diff, (now_us - s_last_post_time_us) / 1000);
        return ESP_OK;
    }

    /* Update last posted values */
    s_last_posted_fill_percent = new_percent;
    s_last_post_time_us = now_us;

    /* Post fill level event to core */
    system_event_t ev = {
        .id = EVENT_FILL_LEVEL_UPDATED,
        .timestamp_us = now_us,
        .source = 0,
        .data = { { {0} } }
    };
    ev.data.fill_level.fill_percent = new_percent;
    service_post_event(&ev);

    /* --- MQTT publish with GPS data obtained directly from GPS service --- */
    gps_fix_t fix;
    bool have_fix = gps_service_get_last_fix(&fix);

    char topic[128];
    mqtt_topic_build(topic, sizeof(topic), "data");

    char json[128];
    if (have_fix && isfinite(fix.latitude) && isfinite(fix.longitude)) {
        snprintf(json, sizeof(json),
                 "{\"fill\":%u,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f}",
                 new_percent,
                 fix.latitude,
                 fix.longitude,
                 (double)fix.altitude_m);
    } else {
        snprintf(json, sizeof(json),
                 "{\"fill\":%u,\"lat\":null,\"lon\":null,\"alt\":null}",
                 new_percent);
    }

    cmd_publish_mqtt_params_t pub = {
        .topic = "",
        .payload = "",
        .payload_len = 0,
        .qos = 1,
        .retain = false
    };
    strlcpy(pub.topic, topic, sizeof(pub.topic));
    strlcpy((char*)pub.payload, json, sizeof(pub.payload));
    pub.payload_len = strlen(json);
    command_router_execute(CMD_PUBLISH_MQTT, &pub);

    return ESP_OK;
}

/* ============================================================
 * Command handler: read intention sensor (one‑shot)
 * ============================================================ */
static esp_err_t handle_read_intent_sensor(void *context, void *params)
{
    (void)context;
    (void)params;
    if (!s_intent_sensor) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t pulse_us;
    esp_err_t ret = ultrasonic_driver_measure(s_intent_sensor, &pulse_us);
    if (ret == ESP_OK) {
        uint32_t distance_cm = (pulse_us * 1715) / 100000;
        ESP_LOGD(TAG, "Intent distance: %lu cm", distance_cm);
        if (distance_cm < INTENT_DISTANCE_CM) {
            system_event_t ev = {
                .id = EVENT_CLOSE_RANGE_DETECTED,
                .timestamp_us = esp_timer_get_time(),
                .source = 0,
                .data = { { {0} } }
            };
            service_post_event(&ev);
        }
    } else {
        ESP_LOGW(TAG, "Intent sensor failed: %d", ret);
    }
    return ESP_OK;
}

/* ============================================================
 * FreeRTOS task – periodically polls intention sensor
 * ============================================================ */
static void intent_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        handle_read_intent_sensor(NULL, NULL);
        vTaskDelay(pdMS_TO_TICKS(INTENT_POLL_INTERVAL_MS));
    }
}

/* ============================================================
 * Public API – initialise sensors, timer, filter state
 * ============================================================ */
esp_err_t ultrasonic_service_init(void)
{
    /* Initialise fill‑level sensor */
    ultrasonic_config_t fill_cfg = {
        .trig_pin = FILL_SENSOR_TRIG,
        .echo_pin = FILL_SENSOR_ECHO,
        .timeout_us = 100000
    };
    esp_err_t ret = ultrasonic_driver_create(&fill_cfg, &s_fill_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fill sensor init failed: %d", ret);
        return ret;
    }

    /* Initialise intention sensor */
    ultrasonic_config_t intent_cfg = {
        .trig_pin = INTENT_SENSOR_TRIG,
        .echo_pin = INTENT_SENSOR_ECHO,
        .timeout_us = 100000
    };
    ret = ultrasonic_driver_create(&intent_cfg, &s_intent_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Intent sensor init failed: %d", ret);
        ultrasonic_driver_delete(s_fill_sensor);
        return ret;
    }

    /* Reset filter state */
    memset(s_fill_distance_buffer, 0, sizeof(s_fill_distance_buffer));
    s_fill_buffer_index = 0;
    s_fill_buffer_count = 0;
    s_last_posted_fill_percent = 0xFF;
    s_last_post_time_us = 0;

    /* Create periodic timer for fill level */
    const esp_timer_create_args_t timer_args = {
        .callback = fill_timer_callback,
        .arg = NULL,
        .name = "fill_timer",
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = false
    };
    esp_err_t err = esp_timer_create(&timer_args, &s_fill_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fill timer: %s", esp_err_to_name(err));
        /* Continue anyway – timer failure is not fatal, but log it */
    }

    ESP_LOGI(TAG, "Ultrasonic service initialised (filter window=%d, deadband=%d%%, min interval=%d ms)",
             FILL_FILTER_WINDOW_SIZE, FILL_DEADBAND_PERCENT, FILL_MIN_INTERVAL_MS);
    return ESP_OK;
}

/* ============================================================
 * Register command handlers with the command router
 * ============================================================ */
esp_err_t ultrasonic_service_register_handlers(void)
{
    esp_err_t ret;
    ret = service_register_command(COMMAND_READ_FILL_LEVEL, handle_read_fill_level, NULL);
    if (ret != ESP_OK) return ret;
    ret = service_register_command(COMMAND_READ_INTENT_SENSOR, handle_read_intent_sensor, NULL);
    if (ret != ESP_OK) return ret;
    ESP_LOGI(TAG, "Ultrasonic service command handlers registered");
    return ESP_OK;
}

/* ============================================================
 * Start the service: periodic fill timer and intention monitoring
 * ============================================================ */
esp_err_t ultrasonic_service_start(void)
{
    if (s_fill_timer) {
        esp_timer_start_periodic(s_fill_timer, FILL_PUBLISH_INTERVAL_MS * 1000);
    }

    /* Start intention monitoring task */
    BaseType_t ret = xTaskCreate(intent_monitor_task, "intent_monitor",
                                 4096, NULL, 5, &s_intent_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create intent monitor task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Ultrasonic service started (intention polling %d ms, fill interval %d ms)",
             INTENT_POLL_INTERVAL_MS, FILL_PUBLISH_INTERVAL_MS);
    return ESP_OK;
}

/* ============================================================
 * Stop the service: stop timers and delete task
 * ============================================================ */
esp_err_t ultrasonic_service_stop(void)
{
    if (s_fill_timer) {
        esp_timer_stop(s_fill_timer);
        esp_timer_delete(s_fill_timer);
        s_fill_timer = NULL;
    }

    if (s_intent_task) {
        vTaskDelete(s_intent_task);
        s_intent_task = NULL;
    }

    ESP_LOGI(TAG, "Ultrasonic service stopped");
    return ESP_OK;
}