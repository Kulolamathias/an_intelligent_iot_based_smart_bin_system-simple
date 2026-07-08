#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_efuse.h"
#include "esp_mac.h"
#include "ultrasonic_driver.h"
#include "pir_driver.h"
#include "servo_driver.h"
#include "led_driver.h"
#include "buzzer_driver.h"
#include "lcd_i2c_driver.h"
#include "gps_driver.h"
#include "gsm_sim800.h"
#include "wifi_driver_abstraction.h"
#include "mqtt_client_abstraction.h"
#include "mqtt_topic.h"
#include "geo_utils.h"
#include "location_lookup.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

static const char *TAG = "SMART_BIN";

/* ========== WiFi & MQTT Configuration ========== */
#define WIFI_SSID       "Mathias' Sxx U..."
#define WIFI_PASSWORD   "1234567890223"
#define MQTT_BROKER_URI "mqtt://102.223.8.140:1883"
#define MQTT_USER       "mqtt_user"
#define MQTT_PASS       "ega12345"

/* MQTT publish cadence */
#define MQTT_DATA_PUBLISH_INTERVAL_MS         400
#define MQTT_STATE_PUBLISH_INTERVAL_MS        5000
#define MQTT_STATE_CHANGE_MIN_INTERVAL_MS     2000
#define MQTT_HEARTBEAT_PUBLISH_INTERVAL_MS    15000
#define MQTT_DISCOVERY_PUBLISH_INTERVAL_MS    15000
#define MQTT_PEER_EXPIRY_CHECK_INTERVAL_MS    5000

/* ========== Pin Definitions (All Safe) ========== */
#define FILL_TRIG     18
#define FILL_ECHO     34
#define INTENT_TRIG   32
#define INTENT_ECHO   35
#define PIR_GPIO      36
#define SERVO_GPIO    33
#define LED_RED       19
#define LED_GREEN     4
#define BUZZER_GPIO   23

/* I2C for LCD */
#define I2C_MASTER_NUM    0
#define I2C_MASTER_SDA    GPIO_NUM_21
#define I2C_MASTER_SCL    GPIO_NUM_22
#define I2C_MASTER_FREQ   50000
#define LCD_I2C_ADDR      0x27
#define LCD_COLS          20
#define LCD_ROWS          4

/* Timing */
#define INTENT_TIMEOUT_MS      10000
#define LID_OPEN_DURATION_MS   5000
#define DISTANCE_THRESHOLD_CM  30
#define SERVO_OPEN_ANGLE       90.0f
#define SERVO_CLOSED_ANGLE     0.0f
#define SERVO_COMMAND_RETRIES  3
#define SERVO_RETRY_DELAY_MS   120
#define MAINTENANCE_SERVO_HOLD_MS 2000
#define GPS_FIX_STALE_MS       120000
#define GPS_MIN_LAT            (-6.30)
#define GPS_MAX_LAT            (-6.10)
#define GPS_MIN_LON            35.70
#define GPS_MAX_LON            35.90

/* GSM thresholds */
#define NEAR_FULL_PERCENT  75
#define FULL_PERCENT       94
#define DEBOUNCE_SEC       300
#define COLLECTOR_PHONE    "+255688173415"
#define MANAGER_PHONE      COLLECTOR_PHONE
#define GSM_PASSWORD       "SECRET123"
#define GSM_INIT_RETRY_INTERVAL_MS  30000
#define GSM_SMS_RETRY_INTERVAL_SEC  60
#define GSM_POLL_INTERVAL_MS        2000
#define GSM_SEND_TIMEOUT_MS         45000
#define GSM_SEND_ATTEMPTS           2
#define GSM_REINIT_AFTER_FAILURES   3
#define GSM_ESCALATION_DELAY_SEC    3600

/* Fill level calculation – configurable thresholds (all in cm) */
#define DISTANCE_0_PERCENT_CM      60.0f   /* sensor reading (cm) when bin is EMPTY (0% full) */
#define DISTANCE_100_PERCENT_CM    5.0f    /* sensor reading (cm) when bin is FULL (100% full) */
#define MAX_VALID_DISTANCE_CM      200.0f  /* ignore readings beyond this (sensor max range) */

/* Fill level stability */
#define FILL_MIN_UPDATE_INTERVAL_MS   700   // minimum time between updates (1 second)
#define FILL_DEADBAND_PERCENT         5      // ignore changes smaller than ±5%
#define FILL_MEDIAN_WINDOW_SIZE       7      // number of samples for median filter

/* Peer registry */
#define MAX_PEERS 16
#define PEER_TIMEOUT_SEC          180
#define REDIRECT_MAX_DISTANCE_M   1500
#define REDIRECT_DEBOUNCE_MS      10000
#define REDIRECT_FAILURE_RETRY_MS 1500
#define REDIRECT_DISPLAY_MS       12000
#define LOCATION_NAME_LEN         32
typedef struct {
    char id[13];
    double lat;
    double lon;
    float alt;
    uint8_t fill;
    bool active;
    uint32_t last_seen;
    char name[LOCATION_NAME_LEN];
} peer_t;

/* ========== Global Handles ========== */
static ultrasonic_handle_t s_fill_sensor = NULL;
static ultrasonic_handle_t s_intent_sensor = NULL;
static pir_handle_t s_pir = NULL;
static servo_handle_t s_servo = NULL;
static led_handle_t s_led_red = NULL;
static led_handle_t s_led_green = NULL;
static buzzer_handle_t s_buzzer = NULL;
static lcd_handle_t s_lcd = NULL;

/* ========== Shared Data ========== */
static SemaphoreHandle_t s_gps_mutex = NULL;
static gps_data_t s_last_gps = {0};
static bool s_gps_valid = false;
static uint32_t s_last_gps_fix_ms = 0;

static SemaphoreHandle_t s_servo_mutex = NULL;
static SemaphoreHandle_t s_peers_mutex = NULL;
static peer_t s_peers[MAX_PEERS] = {0};
static char s_bin_id[13] = {0};
static bool s_wifi_connected = false;
static bool s_mqtt_connected = false;
static uint32_t s_last_redirect_ms = 0;
static bool s_redirect_last_success = false;
static uint32_t s_redirect_display_until_ms = 0;
static char s_redirect_lcd_line3[21] = "";
static char s_redirect_lcd_line4[21] = "";
static volatile bool s_maintenance_unlocked = false;

static uint8_t s_stable_fill = 0;          // filtered, debounced fill level
static uint64_t s_last_fill_update_us = 0; // timestamp of last update

static uint8_t s_current_fill = 0;

typedef struct {
    bool sent;
    uint32_t last_attempt_s;
    uint8_t failure_count;
} gsm_alert_state_t;

/* ========== State Machine ========== */
typedef enum {
    STATE_IDLE,
    STATE_ATTENTION,
    STATE_LID_OPEN,
    STATE_LID_CLOSE_WAIT,
    STATE_MAINTENANCE
} system_state_t;
static volatile system_state_t s_state = STATE_IDLE;
static uint32_t s_attention_start_time = 0;
static uint32_t s_lid_open_start_time = 0;

static void on_sms_received(const received_sms_t *sms);

/* ========== Helper Functions ========== */
static uint32_t get_time_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static bool gps_coords_plausible(double lat, double lon)
{
    return lat >= GPS_MIN_LAT && lat <= GPS_MAX_LAT &&
           lon >= GPS_MIN_LON && lon <= GPS_MAX_LON;
}

static uint32_t median_filter_5(uint32_t *buf) {
    for (int i = 0; i < 4; i++) {
        for (int j = i+1; j < 5; j++) {
            if (buf[i] > buf[j]) {
                uint32_t t = buf[i]; buf[i] = buf[j]; buf[j] = t;
            }
        }
    }
    return buf[2];
}

static bool is_hand_detected(void) {
    uint32_t samples[5];
    int valid = 0;
    for (int i = 0; i < 5; i++) {
        uint32_t pulse_us;
        if (ultrasonic_driver_measure(s_intent_sensor, &pulse_us) == ESP_OK) {
            uint32_t dist = (pulse_us * 1715) / 100000;
            if (dist >= 2 && dist <= 400) samples[valid++] = dist;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (valid >= 3) {
        uint32_t dist = median_filter_5(samples);
        return (dist < DISTANCE_THRESHOLD_CM);
    }
    return false;
}

static uint8_t get_fill_percent(void)
{
    if (!s_fill_sensor) return 0;

    uint32_t samples[FILL_MEDIAN_WINDOW_SIZE];
    int valid = 0;

    for (int i = 0; i < FILL_MEDIAN_WINDOW_SIZE; i++) {
        uint32_t pulse_us;
        if (ultrasonic_driver_measure(s_fill_sensor, &pulse_us) == ESP_OK) {
            float dist = (pulse_us * 1715.0f) / 100000.0f;
            if (dist >= 2.0f && dist <= MAX_VALID_DISTANCE_CM) {
                samples[valid++] = (uint32_t)(dist * 10);  // store in 0.1 cm resolution
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid < (FILL_MEDIAN_WINDOW_SIZE / 2 + 1)) return 0;

    // Bubble sort for small window
    for (int i = 0; i < valid - 1; i++) {
        for (int j = i + 1; j < valid; j++) {
            if (samples[i] > samples[j]) {
                uint32_t t = samples[i]; samples[i] = samples[j]; samples[j] = t;
            }
        }
    }

    float median_cm = (float)samples[valid / 2] / 10.0f;

    // Clamp to defined thresholds
    if (median_cm <= DISTANCE_100_PERCENT_CM) return 100;
    if (median_cm >= DISTANCE_0_PERCENT_CM)   return 0;

    // Linear interpolation between empty and full
    float percent = 100.0f * (DISTANCE_0_PERCENT_CM - median_cm) / (DISTANCE_0_PERCENT_CM - DISTANCE_100_PERCENT_CM);
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    return (uint8_t)(percent + 0.5f);
}

/* ========== Buzzer Patterns ========== */
static void play_friendly_chirp(void) {
    for (int i = 0; i < 3; i++) {
        buzzer_driver_start_tone(s_buzzer, 2000, 50);
        vTaskDelay(pdMS_TO_TICKS(100));
        buzzer_driver_stop(s_buzzer);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void play_success_arpeggio(void) {
    buzzer_driver_start_tone(s_buzzer, 1500, 50); vTaskDelay(pdMS_TO_TICKS(80));
    buzzer_driver_start_tone(s_buzzer, 2000, 50); vTaskDelay(pdMS_TO_TICKS(80));
    buzzer_driver_start_tone(s_buzzer, 2500, 50); vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_driver_stop(s_buzzer);
}

static void play_gentle_fade(void) {
    buzzer_driver_start_tone(s_buzzer, 800, 50); vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_driver_start_tone(s_buzzer, 600, 50); vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_driver_start_tone(s_buzzer, 400, 50); vTaskDelay(pdMS_TO_TICKS(300));
    buzzer_driver_stop(s_buzzer);
}

/* ========== LCD Updates ========== */
static void lcd_format_line(char out[21], const char *text)
{
    char tmp[21];
    snprintf(tmp, sizeof(tmp), "%s", text ? text : "");
    snprintf(out, 21, "%-20.20s", tmp);
}

static void update_lcd_main(const char *line1, const char *line2) {
    static char last1[21] = "", last2[21] = "";
    if (strcmp(last1, line1) != 0 || strcmp(last2, line2) != 0) {
        lcd_driver_clear(s_lcd);
        vTaskDelay(pdMS_TO_TICKS(10)); // small delay after clear
        lcd_driver_write_string(s_lcd, 0, 0, line1);
        vTaskDelay(pdMS_TO_TICKS(5));
        lcd_driver_write_string(s_lcd, 1, 0, line2);
        strcpy(last1, line1);
        strcpy(last2, line2);
    }
}

static void update_lcd_gps_fill(void)
{
    static char line3[21] = "", line4[21] = "";
    static uint64_t last_update_us = 0;
    uint64_t now_us = esp_timer_get_time();

    if ((now_us - last_update_us) < 200000) return; // 200 ms minimum

    char new3[21], new4[21];
    if (s_redirect_display_until_ms > get_time_ms()) {
        lcd_format_line(new3, s_redirect_lcd_line3);
        lcd_format_line(new4, s_redirect_lcd_line4);
    } else if (s_gps_valid &&
               gps_coords_plausible(s_last_gps.latitude, s_last_gps.longitude) &&
               (get_time_ms() - s_last_gps_fix_ms) <= GPS_FIX_STALE_MS) {
        double lat, lon;
        if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            lat = s_last_gps.latitude;
            lon = s_last_gps.longitude;
            xSemaphoreGive(s_gps_mutex);
        } else { lat = 0; lon = 0; }
        char fill_line[21];
        char gps_line[21];
        snprintf(fill_line, sizeof(fill_line), "Fill: %3d%%", s_current_fill);
        snprintf(gps_line, sizeof(gps_line), "%.6f %.6f", lat, lon);
        lcd_format_line(new3, fill_line);
        lcd_format_line(new4, gps_line);
    } else {
        char fill_line[21];
        snprintf(fill_line, sizeof(fill_line), "Fill: %3d%%", s_current_fill);
        lcd_format_line(new3, fill_line);
        lcd_format_line(new4, "GPS: no fix");
    }
    if (strcmp(line3, new3) != 0) {
        lcd_driver_write_string(s_lcd, 2, 0, new3);
        strcpy(line3, new3);
        last_update_us = now_us;
    }
    if (strcmp(line4, new4) != 0) {
        lcd_driver_write_string(s_lcd, 3, 0, new4);
        strcpy(line4, new4);
        last_update_us = now_us;
    }
}

static void update_leds(system_state_t state) {
    led_driver_off(s_led_red);
    led_driver_off(s_led_green);
    if (state == STATE_ATTENTION) {
        led_driver_start_blink(s_led_red, 500, 50);
    } else if (state == STATE_LID_OPEN || state == STATE_LID_CLOSE_WAIT) {
        led_driver_on(s_led_green);
    } else if (state == STATE_MAINTENANCE) {
        led_driver_on(s_led_green);
    }
}

static esp_err_t lid_set_angle(float angle, const char *reason)
{
    if (!s_servo) {
        ESP_LOGE(TAG, "Servo command ignored: driver not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_servo_mutex &&
        xSemaphoreTake(s_servo_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Servo command timeout: %s", reason ? reason : "unknown");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t last_ret = ESP_FAIL;
    bool ok = false;
    for (int attempt = 1; attempt <= SERVO_COMMAND_RETRIES; attempt++) {
        last_ret = servo_driver_set_angle(s_servo, angle);
        if (last_ret == ESP_OK) {
            ok = true;
        } else {
            ESP_LOGW(TAG, "Servo command failed (%s) attempt %d/%d: %s",
                     reason ? reason : "unknown",
                     attempt,
                     SERVO_COMMAND_RETRIES,
                     esp_err_to_name(last_ret));
        }

        if (attempt < SERVO_COMMAND_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(SERVO_RETRY_DELAY_MS));
        }
    }

    if (s_servo_mutex) {
        xSemaphoreGive(s_servo_mutex);
    }

    if (!ok) {
        return last_ret;
    }

    ESP_LOGI(TAG, "Lid angle %.1f commanded (%s)",
             (double)angle,
             reason ? reason : "no reason");
    return ESP_OK;
}

static esp_err_t lid_open(const char *reason)
{
    return lid_set_angle(SERVO_OPEN_ANGLE, reason);
}

static esp_err_t lid_close(const char *reason)
{
    return lid_set_angle(SERVO_CLOSED_ANGLE, reason);
}

/* ========== Re-direction ========== */
typedef struct {
    peer_t peer;
    uint32_t distance_m;
    double bearing_deg;
    char direction[4];
    char display_name[LOCATION_NAME_LEN];
    bool has_friendly_name;
} redirect_target_t;

static bool get_current_gps(double *lat, double *lon)
{
    bool valid = false;
    uint32_t now_ms = get_time_ms();

    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        valid = s_gps_valid &&
                s_last_gps.latitude != 0.0 &&
                s_last_gps.longitude != 0.0 &&
                gps_coords_plausible(s_last_gps.latitude, s_last_gps.longitude) &&
                (now_ms - s_last_gps_fix_ms) <= GPS_FIX_STALE_MS;
        if (valid) {
            *lat = s_last_gps.latitude;
            *lon = s_last_gps.longitude;
        }
        xSemaphoreGive(s_gps_mutex);
    }

    return valid;
}

static bool resolve_location_name(double lat, double lon, char *name, size_t name_size)
{
    if (!name || name_size == 0) {
        return false;
    }

    name[0] = '\0';
    return location_lookup_find(lat, lon, name, name_size);
}

static bool get_current_position(double *lat,
                                 double *lon,
                                 float *alt,
                                 char *name,
                                 size_t name_size)
{
    bool valid = false;
    double local_lat = 0.0;
    double local_lon = 0.0;
    float local_alt = 0.0f;
    uint32_t now_ms = get_time_ms();

    if (name && name_size > 0) {
        name[0] = '\0';
    }

    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        valid = s_gps_valid &&
                s_last_gps.latitude != 0.0 &&
                s_last_gps.longitude != 0.0 &&
                gps_coords_plausible(s_last_gps.latitude, s_last_gps.longitude) &&
                (now_ms - s_last_gps_fix_ms) <= GPS_FIX_STALE_MS;
        if (valid) {
            local_lat = s_last_gps.latitude;
            local_lon = s_last_gps.longitude;
            local_alt = s_last_gps.altitude;
        }
        xSemaphoreGive(s_gps_mutex);
    }

    if (!valid) {
        return false;
    }

    if (lat) {
        *lat = local_lat;
    }
    if (lon) {
        *lon = local_lon;
    }
    if (alt) {
        *alt = local_alt;
    }
    if (name && name_size > 0) {
        resolve_location_name(local_lat, local_lon, name, name_size);
    }

    return true;
}

static void format_local_bin_payload(char *payload,
                                     size_t payload_size,
                                     const char *id,
                                     uint8_t fill,
                                     uint32_t now_s)
{
    double lat = 0.0;
    double lon = 0.0;
    float alt = 0.0f;
    char name[LOCATION_NAME_LEN] = "";

    if (get_current_position(&lat, &lon, &alt, name, sizeof(name))) {
        snprintf(payload, payload_size,
                 "{\"id\":\"%s\",\"fill\":%d,\"lat\":%.6f,\"lon\":%.6f,"
                 "\"alt\":%.1f,\"name\":\"%s\",\"timestamp\":%lu}",
                 id, fill, lat, lon, (double)alt, name,
                 (unsigned long)now_s);
    } else {
        snprintf(payload, payload_size,
                 "{\"id\":\"%s\",\"fill\":%d,\"lat\":null,\"lon\":null,"
                 "\"alt\":null,\"name\":\"\",\"timestamp\":%lu}",
                 id, fill, (unsigned long)now_s);
    }
}

static void redirect_set_detail_lines(const char *line3, const char *line4, uint32_t now_ms)
{
    snprintf(s_redirect_lcd_line3, sizeof(s_redirect_lcd_line3), "%s", line3 ? line3 : "");
    snprintf(s_redirect_lcd_line4, sizeof(s_redirect_lcd_line4), "%s", line4 ? line4 : "");
    s_redirect_display_until_ms = now_ms + REDIRECT_DISPLAY_MS;
}

static void redirect_show_status(const char *line1,
                                 const char *line2,
                                 const char *line3,
                                 const char *line4,
                                 uint32_t now_ms)
{
    update_lcd_main(line1, line2);
    redirect_set_detail_lines(line3, line4, now_ms);
    update_lcd_gps_fill();
}

static bool redirect_find_nearest(double my_lat, double my_lon, redirect_target_t *target)
{
    if (!target) {
        return false;
    }

    uint32_t now_s = get_time_ms() / 1000;
    bool found = false;
    uint32_t best_distance = UINT32_MAX;
    peer_t best_peer = {0};
    int active_count = 0;
    int fresh_count = 0;
    int gps_count = 0;
    int free_count = 0;
    int in_range_count = 0;

    if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    for (int i = 0; i < MAX_PEERS; i++) {
        const peer_t *peer = &s_peers[i];
        if (!peer->active) {
            continue;
        }
        active_count++;

        if ((now_s - peer->last_seen) > PEER_TIMEOUT_SEC) {
            continue;
        }
        fresh_count++;

        if (peer->lat == 0.0 ||
            peer->lon == 0.0 ||
            !gps_coords_plausible(peer->lat, peer->lon)) {
            continue;
        }
        gps_count++;

        if (peer->fill >= FULL_PERCENT) {
            continue;
        }
        free_count++;

        uint32_t distance = geo_distance(my_lat, my_lon, peer->lat, peer->lon);
        if (distance <= REDIRECT_MAX_DISTANCE_M) {
            in_range_count++;
        }
        if (distance <= REDIRECT_MAX_DISTANCE_M && distance < best_distance) {
            best_distance = distance;
            best_peer = *peer;
            found = true;
        }
    }

    xSemaphoreGive(s_peers_mutex);

    if (!found) {
        ESP_LOGW(TAG,
                 "Redirect peers checked: active=%d fresh=%d gps=%d free=%d in_range=%d",
                 active_count,
                 fresh_count,
                 gps_count,
                 free_count,
                 in_range_count);
        return false;
    }

    memset(target, 0, sizeof(*target));
    target->peer = best_peer;
    target->distance_m = best_distance;
    target->bearing_deg = geo_bearing(my_lat, my_lon, best_peer.lat, best_peer.lon);
    strlcpy(target->direction,
            geo_bearing_to_cardinal(target->bearing_deg),
            sizeof(target->direction));

    if (best_peer.name[0] != '\0') {
        strlcpy(target->display_name, best_peer.name, sizeof(target->display_name));
        target->has_friendly_name = true;
    } else if (resolve_location_name(best_peer.lat, best_peer.lon,
                                     target->display_name,
                                     sizeof(target->display_name))) {
        target->has_friendly_name = true;
    } else {
        snprintf(target->display_name, sizeof(target->display_name),
                 "Bin %.12s", best_peer.id);
        target->has_friendly_name = false;
    }

    return true;
}

static void redirect_show_nearest(uint32_t now_ms)
{
    uint32_t retry_ms = s_redirect_last_success ?
                        REDIRECT_DEBOUNCE_MS :
                        REDIRECT_FAILURE_RETRY_MS;
    if (s_last_redirect_ms != 0 &&
        (now_ms - s_last_redirect_ms) < retry_ms) {
        return;
    }
    s_last_redirect_ms = now_ms;

    double my_lat = 0.0;
    double my_lon = 0.0;
    if (!get_current_gps(&my_lat, &my_lon)) {
        s_redirect_last_success = false;
        redirect_show_status("Bin full", "GPS not ready",
                             "Cannot redirect", "Try nearby bin", now_ms);
        ESP_LOGW(TAG, "Redirect skipped: own GPS fix is not valid");
        return;
    }

    redirect_target_t target;
    if (!redirect_find_nearest(my_lat, my_lon, &target)) {
        s_redirect_last_success = false;
        redirect_show_status("Bin full", "No free bin found",
                             "Wait collector", "or try nearby", now_ms);
        ESP_LOGW(TAG, "Redirect skipped: no active available peer within %dm",
                 REDIRECT_MAX_DISTANCE_M);
        return;
    }

    char line2[21];
    char line3[21];
    snprintf(line2, sizeof(line2), "Use %.16s", target.display_name);
    snprintf(line3, sizeof(line3), "About %lum %s",
             (unsigned long)target.distance_m, target.direction);

    redirect_show_status("Bin full", line2, line3,
                         target.has_friendly_name ? "Nearest free bin" : "GPS shown on web",
                         now_ms);
    if (!target.has_friendly_name) {
        char coords[21];
        snprintf(coords, sizeof(coords), "%.4f %.4f",
                 target.peer.lat,
                 target.peer.lon);
        redirect_set_detail_lines(line3, coords, now_ms);
        update_lcd_gps_fill();
    }
    s_redirect_last_success = true;
    ESP_LOGI(TAG, "Redirect to %s (%s) at %lu m %s",
             target.peer.id,
             target.display_name,
             (unsigned long)target.distance_m,
             target.direction);
}

/* ========== GPS Task ========== */
static void gps_task(void *pvParameters) {
    gps_proof_init();
    while (1) {
        gps_data_t fix;
        esp_err_t ret = gps_proof_get_fix(&fix, 2000);
        if (ret == ESP_OK &&
            fix.fix_quality >= 1 &&
            fix.latitude != 0.0 &&
            fix.longitude != 0.0 &&
            gps_coords_plausible(fix.latitude, fix.longitude)) {
            if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(&s_last_gps, &fix, sizeof(gps_data_t));
                s_gps_valid = true;
                s_last_gps_fix_ms = get_time_ms();
                xSemaphoreGive(s_gps_mutex);
            }
        } else if (ret == ESP_OK && fix.latitude != 0.0 && fix.longitude != 0.0) {
            ESP_LOGW(TAG, "Ignoring implausible GPS fix: %.6f %.6f",
                     fix.latitude,
                     fix.longitude);
        } else if (s_gps_valid &&
                   (get_time_ms() - s_last_gps_fix_ms) > GPS_FIX_STALE_MS) {
            if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_gps_valid = false;
                xSemaphoreGive(s_gps_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#if 0
/* ========== GSM Task (with retry and boot SMS) ========== */
static void gsm_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    bool gsm_ok = false;
    static bool boot_sms_sent = false;

    for (int attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "GSM init attempt %d/3", attempt);
        gsm_config_t cfg = {
            .uart_port = UART_NUM_2,
            .tx_pin = GPIO_NUM_17,
            .rx_pin = GPIO_NUM_16,
            .baud_rate = 9600,
            .buf_size = 2048,
            .timeout_ms = 30000,
            .retry_count = 2,
        };
        if (gsm_init(&cfg) == ESP_OK) {
            const char *auth[] = { COLLECTOR_PHONE };
            gsm_set_authorized_numbers(auth, 1);
            gsm_set_password(GSM_PASSWORD);
            gsm_set_received_callback(on_sms_received);
            gsm_ok = true;
            ESP_LOGI(TAG, "GSM ready");
            break;
        }
        ESP_LOGW(TAG, "GSM init attempt %d failed", attempt);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if (!gsm_ok) {
        ESP_LOGW(TAG, "GSM init failed – continuing without GSM");
    } else {
        if (!boot_sms_sent) {
            char boot_msg[] = "Smart bin online. Ready for use.";
            esp_err_t ret = gsm_send_sms(COLLECTOR_PHONE, boot_msg, 30000);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Boot SMS sent to %s", COLLECTOR_PHONE);
                boot_sms_sent = true;
            } else {
                ESP_LOGE(TAG, "Failed to send boot SMS");
            }
        }
    }

    uint32_t last_check = 0;
    while (1) {
        uint32_t now = get_time_ms() / 1000;
        if (gsm_ok) {
            if ((get_time_ms() - last_check) > 5000) {
                gsm_check_sms(true);
                gsm_process_received_sms();
                last_check = get_time_ms();
            }
            // Fill level SMS
            if (s_current_fill >= FULL_PERCENT && s_last_sent_fill != FULL_PERCENT &&
                (now - s_last_sms_time) > DEBOUNCE_SEC) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Bin FULL at %d%% Send password to unlock.", s_current_fill);
                gsm_send_sms(COLLECTOR_PHONE, msg, 30000);
                s_last_sent_fill = FULL_PERCENT;
                s_last_sms_time = now;
            } else if (s_current_fill >= NEAR_FULL_PERCENT && s_current_fill < FULL_PERCENT &&
                       s_last_sent_fill != NEAR_FULL_PERCENT && (now - s_last_sms_time) > DEBOUNCE_SEC) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Bin near‑full (%d%%).", s_current_fill);
                gsm_send_sms(COLLECTOR_PHONE, msg, 30000);
                s_last_sent_fill = NEAR_FULL_PERCENT;
                s_last_sms_time = now;
            } else if (s_current_fill < NEAR_FULL_PERCENT) {
                s_last_sent_fill = 0xFF;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void on_sms_received(const received_sms_t *sms) {
    if (strstr(sms->message, GSM_PASSWORD) && strstr(sms->message, "UNLOCK")) {
        s_state = STATE_MAINTENANCE;
        update_lcd_main("Maintenance", "Unlocked");
        led_driver_on(s_led_green);
        servo_driver_set_angle(s_servo, 90.0f);
        ESP_LOGI(TAG, "Maintenance mode activated");
    }
}
#endif

/* ========== GSM Task (fail-soft deterministic manager) ========== */
static void gsm_alert_reset(gsm_alert_state_t *alert)
{
    if (alert) {
        memset(alert, 0, sizeof(*alert));
    }
}

static bool gsm_alert_due(const gsm_alert_state_t *alert, uint32_t now_s)
{
    if (!alert || alert->sent) {
        return false;
    }
    return alert->last_attempt_s == 0 ||
           (now_s - alert->last_attempt_s) >= GSM_SMS_RETRY_INTERVAL_SEC;
}

static bool sms_contains_ci(const char *message, const char *needle)
{
    if (!message || !needle || *needle == '\0') {
        return false;
    }

    size_t needle_len = strlen(needle);
    for (const char *p = message; *p; p++) {
        size_t i = 0;
        while (i < needle_len &&
               p[i] &&
               (char)toupper((unsigned char)p[i]) ==
               (char)toupper((unsigned char)needle[i])) {
            i++;
        }
        if (i == needle_len) {
            return true;
        }
    }
    return false;
}

static void normalize_phone_number(const char *src, char *dst, size_t dst_size)
{
    if (!src || !dst || dst_size == 0) {
        return;
    }

    size_t j = 0;
    for (size_t i = 0; src[i] && j < dst_size - 1; i++) {
        if (isdigit((unsigned char)src[i])) {
            dst[j++] = src[i];
        }
    }
    dst[j] = '\0';
}

static bool phone_digits_match(const char *a, const char *b)
{
    size_t len_a = strlen(a);
    size_t len_b = strlen(b);
    size_t min_len = len_a < len_b ? len_a : len_b;

    if (len_a == 0 || len_b == 0) {
        return false;
    }
    if (strcmp(a, b) == 0) {
        return true;
    }
    if (min_len < 9) {
        return false;
    }

    return strcmp(a + len_a - min_len, b + len_b - min_len) == 0;
}

static bool sms_sender_authorized(const char *sender)
{
    char sender_clean[20] = {0};
    char collector_clean[20] = {0};
    normalize_phone_number(sender, sender_clean, sizeof(sender_clean));
    normalize_phone_number(COLLECTOR_PHONE, collector_clean, sizeof(collector_clean));

    return phone_digits_match(sender_clean, collector_clean);
}

static bool gsm_start_once(void)
{
    gsm_config_t cfg = {
        .uart_port = UART_NUM_2,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_16,
        .baud_rate = 9600,
        .buf_size = 2048,
        .timeout_ms = GSM_SEND_TIMEOUT_MS,
        .retry_count = 2,
    };

    esp_err_t ret = gsm_init(&cfg);
    if (ret != ESP_OK || !gsm_is_ready()) {
        ESP_LOGW(TAG, "GSM unavailable (%s); bin continues without SMS",
                 esp_err_to_name(ret));
        gsm_deinit();
        return false;
    }

    static const char *auth[] = { COLLECTOR_PHONE };
    gsm_set_authorized_numbers(auth, 1);
    gsm_set_password(GSM_PASSWORD);
    gsm_set_received_callback(on_sms_received);
    ESP_LOGI(TAG, "GSM ready");
    return true;
}

static esp_err_t gsm_send_confirmed(const char *label, const char *phone, const char *message)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    for (int attempt = 1; attempt <= GSM_SEND_ATTEMPTS; attempt++) {
        if (!gsm_is_ready()) {
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "GSM send %s attempt %d/%d", label, attempt, GSM_SEND_ATTEMPTS);
        ret = gsm_send_sms(phone, message, GSM_SEND_TIMEOUT_MS);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "GSM send %s confirmed", label);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "GSM send %s failed: %s", label, esp_err_to_name(ret));
        gsm_reset();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return ret;
}

static bool gsm_try_alert(gsm_alert_state_t *alert,
                          uint32_t now_s,
                          const char *label,
                          const char *phone,
                          const char *message)
{
    if (!gsm_alert_due(alert, now_s)) {
        return true;
    }

    alert->last_attempt_s = now_s;
    esp_err_t ret = gsm_send_confirmed(label, phone, message);
    if (ret == ESP_OK) {
        alert->sent = true;
        alert->failure_count = 0;
        return true;
    }

    if (alert->failure_count < UINT8_MAX) {
        alert->failure_count++;
    }
    return false;
}

static void gsm_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    bool gsm_ok = false;
    uint32_t last_init_attempt_ms = 0;
    uint32_t last_sms_poll_ms = 0;
    uint8_t consecutive_gsm_failures = 0;
    bool full_episode_active = false;
    uint32_t full_episode_started_s = 0;

    gsm_alert_state_t boot_alert = {0};
    gsm_alert_state_t near_alert = {0};
    gsm_alert_state_t full_alert = {0};
    gsm_alert_state_t escalation_alert = {0};

    while (1) {
        uint32_t now_ms = get_time_ms();
        uint32_t now_s = now_ms / 1000;

        if (!gsm_ok) {
            if (last_init_attempt_ms == 0 ||
                (now_ms - last_init_attempt_ms) >= GSM_INIT_RETRY_INTERVAL_MS) {
                last_init_attempt_ms = now_ms;
                gsm_ok = gsm_start_once();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!gsm_is_ready()) {
            ESP_LOGW(TAG, "GSM lost readiness; reinitialising later");
            gsm_deinit();
            gsm_ok = false;
            consecutive_gsm_failures = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if ((now_ms - last_sms_poll_ms) >= GSM_POLL_INTERVAL_MS) {
            esp_err_t ret = gsm_check_sms(true);
            if (ret == ESP_ERR_TIMEOUT || ret == ESP_ERR_INVALID_STATE) {
                consecutive_gsm_failures++;
                ESP_LOGW(TAG, "GSM SMS poll failed: %s", esp_err_to_name(ret));
            } else {
                consecutive_gsm_failures = 0;
            }
            gsm_process_received_sms();
            last_sms_poll_ms = now_ms;
        }

        if (!boot_alert.sent) {
            if (!gsm_try_alert(&boot_alert, now_s, "boot", COLLECTOR_PHONE,
                               "Smart bin online. Ready for use.")) {
                consecutive_gsm_failures++;
            } else {
                consecutive_gsm_failures = 0;
            }
        }

        uint8_t fill = s_current_fill;
        if (fill < NEAR_FULL_PERCENT) {
            gsm_alert_reset(&near_alert);
            gsm_alert_reset(&full_alert);
            gsm_alert_reset(&escalation_alert);
            full_episode_active = false;
            full_episode_started_s = 0;
        } else if (fill >= FULL_PERCENT) {
            if (!full_episode_active) {
                full_episode_active = true;
                full_episode_started_s = now_s;
                gsm_alert_reset(&full_alert);
                gsm_alert_reset(&escalation_alert);
            }

            char msg[96];
            snprintf(msg, sizeof(msg), "Bin FULL at %d%%. Send password to unlock.", fill);
            if (!gsm_try_alert(&full_alert, now_s, "full", COLLECTOR_PHONE, msg)) {
                consecutive_gsm_failures++;
            }

            if ((now_s - full_episode_started_s) >= GSM_ESCALATION_DELAY_SEC) {
                if (!gsm_try_alert(&escalation_alert, now_s, "escalation", MANAGER_PHONE,
                                   "URGENT: Bin full for >1 hour. Collector unresponsive.")) {
                    consecutive_gsm_failures++;
                }
            }
        } else {
            full_episode_active = false;
            gsm_alert_reset(&full_alert);
            gsm_alert_reset(&escalation_alert);

            char msg[96];
            snprintf(msg, sizeof(msg), "Bin near-full (%d%%). Will lock when full.", fill);
            if (!gsm_try_alert(&near_alert, now_s, "near-full", COLLECTOR_PHONE, msg)) {
                consecutive_gsm_failures++;
            }
        }

        if (consecutive_gsm_failures >= GSM_REINIT_AFTER_FAILURES) {
            ESP_LOGW(TAG, "GSM had %d consecutive failures; reinitialising",
                     consecutive_gsm_failures);
            gsm_deinit();
            gsm_ok = false;
            consecutive_gsm_failures = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void on_sms_received(const received_sms_t *sms) {
    if (!sms || !sms_sender_authorized(sms->sender)) {
        return;
    }

    if (!sms_contains_ci(sms->message, GSM_PASSWORD)) {
        return;
    }

    if (sms_contains_ci(sms->message, "UNLOCK")) {
        s_maintenance_unlocked = true;
        s_state = STATE_MAINTENANCE;
        update_lcd_main("Maintenance", "Unlocked");
        update_leds(STATE_MAINTENANCE);
        esp_err_t lid_ret = lid_open("sms maintenance unlock");
        if (lid_ret == ESP_OK) {
            gsm_send_confirmed("unlock-ack", sms->sender, "Bin unlocked for maintenance.");
            ESP_LOGI(TAG, "Maintenance mode activated");
        } else {
            gsm_send_confirmed("unlock-ack-failed", sms->sender,
                               "Unlock received, but lid motor failed. Check servo power.");
            ESP_LOGE(TAG, "Maintenance unlock failed: %s", esp_err_to_name(lid_ret));
        }
    } else if (sms_contains_ci(sms->message, "MAINTENANCE DONE") ||
               sms_contains_ci(sms->message, "LOCK")) {
        s_maintenance_unlocked = false;
        esp_err_t lid_ret = lid_close("sms maintenance lock");
        s_state = STATE_IDLE;
        update_leds(s_state);
        update_lcd_main("Welcome", "");
        if (lid_ret == ESP_OK) {
            gsm_send_confirmed("maintenance-done-ack", sms->sender,
                               "Maintenance completed. Bin locked.");
            ESP_LOGI(TAG, "Maintenance mode completed");
        } else {
            gsm_send_confirmed("maintenance-lock-failed", sms->sender,
                               "Lock received, but lid motor failed. Check servo power.");
            ESP_LOGE(TAG, "Maintenance lock failed: %s", esp_err_to_name(lid_ret));
        }
    }
}

/* ========== WiFi & MQTT Callbacks ========== */
static void wifi_event_cb(wifi_driver_event_t event, void *data) {
    if (event == WIFI_DRIVER_EVENT_GOT_IP) {
        esp_netif_ip_info_t *ip = (esp_netif_ip_info_t*)data;
        ESP_LOGI(TAG, "WiFi IP: " IPSTR, IP2STR(&ip->ip));
        s_wifi_connected = true;
    } else if (event == WIFI_DRIVER_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected");
        s_wifi_connected = true;
    } else if (event == WIFI_DRIVER_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected");
        s_wifi_connected = false;
        s_mqtt_connected = false;
    }
}

static bool json_get_number(cJSON *root, const char *key, double *out)
{
    cJSON *item = cJSON_GetObjectItem(root, key);
    if (!cJSON_IsNumber(item)) {
        return false;
    }

    *out = item->valuedouble;
    return true;
}

static bool json_get_int(cJSON *root, const char *key, int *out)
{
    cJSON *item = cJSON_GetObjectItem(root, key);
    if (!cJSON_IsNumber(item)) {
        return false;
    }

    *out = item->valueint;
    return true;
}

static bool topic_extract_peer_id(const char *topic,
                                  const char *suffix,
                                  char *id,
                                  size_t id_size)
{
    const char *prefix = "smartbin/bin/";
    size_t prefix_len = strlen(prefix);

    if (!topic || !suffix || !id || id_size == 0 ||
        strncmp(topic, prefix, prefix_len) != 0) {
        return false;
    }

    const char *start = topic + prefix_len;
    const char *end = strstr(start, suffix);
    if (!end || end == start) {
        return false;
    }

    size_t len = (size_t)(end - start);
    if (len >= id_size) {
        len = id_size - 1;
    }

    memcpy(id, start, len);
    id[len] = '\0';
    return true;
}

static bool topic_extract_device_id(const char *topic,
                                    const char *suffix,
                                    char *id,
                                    size_t id_size)
{
    const char *prefix = "devices/";
    size_t prefix_len = strlen(prefix);

    if (!topic || !suffix || !id || id_size == 0 ||
        strncmp(topic, prefix, prefix_len) != 0) {
        return false;
    }

    const char *start = topic + prefix_len;
    const char *end = strstr(start, suffix);
    if (!end || end == start) {
        return false;
    }

    size_t len = (size_t)(end - start);
    if (len >= id_size) {
        len = id_size - 1;
    }

    memcpy(id, start, len);
    id[len] = '\0';
    return true;
}

static void peer_registry_mark_offline(const char *id)
{
    if (!id || id[0] == '\0' || strcmp(id, s_bin_id) == 0) {
        return;
    }

    if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < MAX_PEERS; i++) {
            if (s_peers[i].active && strcmp(s_peers[i].id, id) == 0) {
                s_peers[i].active = false;
                ESP_LOGI(TAG, "Peer %s marked offline", id);
                break;
            }
        }
        xSemaphoreGive(s_peers_mutex);
    }
}

static void peer_registry_upsert(const char *id,
                                 double lat,
                                 double lon,
                                 float alt,
                                 int fill,
                                 const char *name)
{
    if (!id || id[0] == '\0' || strcmp(id, s_bin_id) == 0 ||
        lat == 0.0 || lon == 0.0 ||
        !gps_coords_plausible(lat, lon)) {
        return;
    }

    char resolved_name[LOCATION_NAME_LEN] = "";
    if (name && name[0] != '\0') {
        strlcpy(resolved_name, name, sizeof(resolved_name));
    } else {
        resolve_location_name(lat, lon, resolved_name, sizeof(resolved_name));
    }

    if (fill < 0) {
        fill = 0;
    } else if (fill > 100) {
        fill = 100;
    }

    if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    peer_t *peer = NULL;
    for (int i = 0; i < MAX_PEERS; i++) {
        if (strcmp(s_peers[i].id, id) == 0) {
            peer = &s_peers[i];
            break;
        }
    }
    if (!peer) {
        for (int i = 0; i < MAX_PEERS; i++) {
            if (!s_peers[i].active && s_peers[i].id[0] == '\0') {
                peer = &s_peers[i];
                break;
            }
        }
    }
    if (!peer) {
        for (int i = 0; i < MAX_PEERS; i++) {
            if (!s_peers[i].active) {
                peer = &s_peers[i];
                break;
            }
        }
    }

    if (peer) {
        uint32_t now_s = get_time_ms() / 1000;
        bool should_log = peer->id[0] == '\0' ||
                          peer->fill != (uint8_t)fill ||
                          (now_s - peer->last_seen) >= 10;
        memset(peer, 0, sizeof(*peer));
        strlcpy(peer->id, id, sizeof(peer->id));
        peer->lat = lat;
        peer->lon = lon;
        peer->alt = alt;
        peer->fill = (uint8_t)fill;
        peer->active = true;
        peer->last_seen = now_s;
        strlcpy(peer->name, resolved_name, sizeof(peer->name));
        if (should_log) {
            ESP_LOGI(TAG, "Peer %s: fill=%d, name=%s, gps=%.6f %.6f",
                     id,
                     fill,
                     peer->name[0] ? peer->name : "unmapped",
                     peer->lat,
                     peer->lon);
        }
    }

    xSemaphoreGive(s_peers_mutex);
}

static void peer_registry_update_from_payload(const char *fallback_id, const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    if (!root) {
        return;
    }

    const char *id = cJSON_GetStringValue(cJSON_GetObjectItem(root, "id"));
    if (!id || id[0] == '\0') {
        id = fallback_id;
    }

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    int fill = 100;
    json_get_number(root, "lat", &lat);
    json_get_number(root, "lon", &lon);
    json_get_number(root, "alt", &alt);
    json_get_int(root, "fill", &fill);

    const char *name = cJSON_GetStringValue(cJSON_GetObjectItem(root, "name"));
    peer_registry_upsert(id, lat, lon, (float)alt, fill, name);
    cJSON_Delete(root);
}

static void mqtt_event_cb(mqtt_client_event_t event, void *data) {
    if (event == MQTT_CLIENT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected");
        s_mqtt_connected = true;
        // Subscribe to discovery and peer topics
        mqtt_client_subscribe("smartbin/discovery/announce", 1);
        mqtt_client_subscribe("smartbin/bin/+/state", 1);
        mqtt_client_subscribe("smartbin/bin/+/lwt", 1);
        mqtt_client_subscribe("devices/+/data", 1);
        // Publish online status (retained)
        char topic[128];
        mqtt_topic_build(topic, sizeof(topic), "status/online");
        mqtt_client_publish(topic, "online", strlen("online"), 1, true);
    } else if (event == MQTT_CLIENT_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "MQTT disconnected");
        s_mqtt_connected = false;
    } else if (event == MQTT_CLIENT_EVENT_DATA) {
        mqtt_client_data_t *msg = (mqtt_client_data_t*)data;
        char topic[128];
        memcpy(topic, msg->topic, msg->topic_len);
        topic[msg->topic_len] = '\0';
        char payload[256];
        memcpy(payload, msg->payload, msg->payload_len);
        payload[msg->payload_len] = '\0';

        if (strcmp(topic, "smartbin/discovery/announce") == 0) {
            peer_registry_update_from_payload(NULL, payload);
        } else {
            char peer_id[13];
            if (topic_extract_peer_id(topic, "/state", peer_id, sizeof(peer_id))) {
                peer_registry_update_from_payload(peer_id, payload);
            } else if (topic_extract_peer_id(topic, "/lwt", peer_id, sizeof(peer_id)) &&
                       strcmp(payload, "OFFLINE") == 0) {
                peer_registry_mark_offline(peer_id);
            } else if (topic_extract_device_id(topic, "/data", peer_id, sizeof(peer_id))) {
                peer_registry_update_from_payload(peer_id, payload);
            }
        }
    }
}

/* ========== MQTT Network Task (Non‑blocking, retries WiFi) ========== */
static void mqtt_network_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    wifi_driver_init();
    wifi_driver_register_callback(wifi_event_cb);
    wifi_driver_start();

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    char mac_str[13];
    snprintf(mac_str, sizeof(mac_str), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    strlcpy(s_bin_id, mac_str, sizeof(s_bin_id));
    char client_id[20];
    snprintf(client_id, sizeof(client_id), "bin_%s", mac_str);

    char base_topic[32];
    mqtt_topic_init(base_topic, sizeof(base_topic));

    // Build LWT topic string
    char lwt_topic[64];
    snprintf(lwt_topic, sizeof(lwt_topic), "smartbin/bin/%s/lwt", mac_str);

    uint32_t last_publish_ms = 0;
    uint32_t last_heartbeat_ms = 0;
    uint32_t last_state_ms = 0;
    uint32_t last_expiry_ms = 0;
    uint32_t last_discovery_ms = 0;
    uint8_t last_data_fill = UINT8_MAX;
    uint8_t last_state_fill = UINT8_MAX;
    bool mqtt_initialised = false;

    while (1) {
        uint32_t now_ms = get_time_ms();
        uint32_t now = now_ms / 1000;

        // Attempt WiFi connection if not already connected
        if (!s_wifi_connected) {
            static uint32_t last_wifi_attempt = 0;
            if ((now - last_wifi_attempt) >= 10) {
                ESP_LOGI(TAG, "Attempting WiFi connection...");
                wifi_driver_connect(WIFI_SSID, WIFI_PASSWORD);
                last_wifi_attempt = now;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // WiFi is connected – ensure MQTT is initialised
        if (!mqtt_initialised) {
            mqtt_client_config_t mqtt_cfg = {
                .broker_uri = MQTT_BROKER_URI,
                .client_id = client_id,
                .username = MQTT_USER,
                .password = MQTT_PASS,
                .keepalive = 120,
                .disable_clean_session = false,
                .lwt_qos = 1,
                .lwt_retain = true,
                .lwt_topic = lwt_topic,
                .lwt_message = "OFFLINE",
                .task_stack_size = 0,
                .task_priority = 0
            };
            if (mqtt_client_init(&mqtt_cfg) == ESP_OK) {
                mqtt_client_register_callback(mqtt_event_cb);
                mqtt_client_start();
                mqtt_initialised = true;
                ESP_LOGI(TAG, "MQTT initialised");
            } else {
                ESP_LOGE(TAG, "MQTT init failed, will retry in 10 seconds");
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;
            }
        }

        // If MQTT is connected, publish data
        if (s_mqtt_connected) {
            uint8_t current_fill = s_current_fill;
            bool data_fill_changed = (last_data_fill == UINT8_MAX ||
                                      current_fill != last_data_fill);
            bool state_fill_changed = (last_state_fill == UINT8_MAX ||
                                       current_fill != last_state_fill);

            // Publish dashboard data quickly. This is the main live web feed.
            if (last_publish_ms == 0 ||
                (now_ms - last_publish_ms) >= MQTT_DATA_PUBLISH_INTERVAL_MS) {
                char topic[128];
                mqtt_topic_build(topic, sizeof(topic), "data");
                char payload[256];
                format_local_bin_payload(payload, sizeof(payload), mac_str, current_fill, now);
                mqtt_client_publish(topic, payload, strlen(payload), 1, false);
                last_publish_ms = now_ms;
                last_data_fill = current_fill;
                if (data_fill_changed) {
                    ESP_LOGD(TAG, "Realtime MQTT data publish: fill=%d%%", current_fill);
                }
            }

            // Publish discovery so nearby bins can build their redirect peer lists.
            if (last_discovery_ms == 0 ||
                (now_ms - last_discovery_ms) >= MQTT_DISCOVERY_PUBLISH_INTERVAL_MS) {
                char payload[256];
                format_local_bin_payload(payload, sizeof(payload), mac_str, current_fill, now);
                mqtt_client_publish("smartbin/discovery/announce",
                                    payload,
                                    strlen(payload),
                                    1,
                                    false);
                last_discovery_ms = now_ms;
            }

            // Publish peer/web state periodically, and sooner when fill changes.
            if (last_state_ms == 0 ||
                (now_ms - last_state_ms) >= MQTT_STATE_PUBLISH_INTERVAL_MS ||
                (state_fill_changed &&
                 (now_ms - last_state_ms) >= MQTT_STATE_CHANGE_MIN_INTERVAL_MS)) {
                char topic[128];
                snprintf(topic, sizeof(topic), "smartbin/bin/%s/state", mac_str);
                char payload[256];
                format_local_bin_payload(payload, sizeof(payload), mac_str, current_fill, now);
                mqtt_client_publish(topic, payload, strlen(payload), 1, false);
                last_state_ms = now_ms;
                last_state_fill = current_fill;
            }

            // Heartbeat proves liveness, but live fill updates come from data/state.
            if (last_heartbeat_ms == 0 ||
                (now_ms - last_heartbeat_ms) >= MQTT_HEARTBEAT_PUBLISH_INTERVAL_MS) {
                char topic[128];
                snprintf(topic, sizeof(topic), "smartbin/cloud/bin/%s/heartbeat", mac_str);
                char payload[256];
                format_local_bin_payload(payload, sizeof(payload), mac_str, current_fill, now);
                mqtt_client_publish(topic, payload, strlen(payload), 1, false);
                last_heartbeat_ms = now_ms;
            }

            // Expire old peers
            if (last_expiry_ms == 0 ||
                (now_ms - last_expiry_ms) >= MQTT_PEER_EXPIRY_CHECK_INTERVAL_MS) {
                if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    for (int i = 0; i < MAX_PEERS; i++) {
                        if (s_peers[i].active &&
                            (now - s_peers[i].last_seen) > PEER_TIMEOUT_SEC) {
                            s_peers[i].active = false;
                        }
                    }
                    xSemaphoreGive(s_peers_mutex);
                }
                last_expiry_ms = now_ms;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ========== Main Control Task ========== */
static void control_task(void *pvParameters) {
    bool pir_detected = false;
    bool full_mode_active = false;
    bool full_user_mode = false;
    uint32_t last_maintenance_hold_ms = 0;
    while (1) {
        uint32_t now = get_time_ms();
        pir_driver_read(s_pir, &pir_detected);
        bool hand = is_hand_detected();

        // Stable fill level update with debounce
        uint8_t raw_fill = get_fill_percent();
        uint64_t now_us = esp_timer_get_time();
        if (abs((int)raw_fill - (int)s_stable_fill) >= FILL_DEADBAND_PERCENT ||
            (now_us - s_last_fill_update_us) >= (FILL_MIN_UPDATE_INTERVAL_MS * 1000ULL)) {
            s_stable_fill = raw_fill;
            s_last_fill_update_us = now_us;
            s_current_fill = s_stable_fill;
            ESP_LOGD(TAG, "Fill level updated: %d%%", s_current_fill);
        } else {
            // No significant change, keep previous stable value
            // s_current_fill remains unchanged
        }

        if (s_maintenance_unlocked || s_state == STATE_MAINTENANCE) {
            s_state = STATE_MAINTENANCE;
            if ((now - last_maintenance_hold_ms) >= MAINTENANCE_SERVO_HOLD_MS) {
                lid_open("maintenance hold");
                last_maintenance_hold_ms = now;
            }
            update_leds(STATE_MAINTENANCE);
            update_lcd_gps_fill();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        bool bin_full = (s_current_fill >= FULL_PERCENT);
        bool user_attempt = (pir_detected || hand);

        if (bin_full &&
            s_state != STATE_MAINTENANCE &&
            s_state != STATE_LID_OPEN &&
            s_state != STATE_LID_CLOSE_WAIT) {
            if (s_state != STATE_IDLE) {
                s_state = STATE_IDLE;
            }

            if (!full_mode_active || full_user_mode != user_attempt) {
                led_driver_off(s_led_green);
                led_driver_on(s_led_red);
                led_driver_start_blink(s_led_red,
                                       user_attempt ? 250 : 700,
                                       user_attempt ? 70 : 45);
                full_mode_active = true;
                full_user_mode = user_attempt;
            }

            if (user_attempt) {
                redirect_show_nearest(now);
            } else if (s_redirect_display_until_ms <= now) {
                update_lcd_main("Bin full", "Try nearby bin");
            }

            update_lcd_gps_fill();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!bin_full && full_mode_active) {
            s_redirect_display_until_ms = 0;
            full_mode_active = false;
            full_user_mode = false;
            update_leds(s_state);
            if (s_state == STATE_IDLE) {
                update_lcd_main("Welcome", "");
            }
        }

        switch (s_state) {
            case STATE_IDLE:
                if (pir_detected) {
                    s_state = STATE_ATTENTION;
                    s_attention_start_time = now;
                    update_leds(s_state);
                    update_lcd_main("Raise waste", "to open");
                    play_friendly_chirp();
                }
                break;
            case STATE_ATTENTION:
                if (hand) {
                    lid_open("user waste deposit");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    s_state = STATE_LID_OPEN;
                    s_lid_open_start_time = now;
                    update_leds(s_state);
                    update_lcd_main("Lid open", "Dispose waste");
                    play_success_arpeggio();
                } else if ((now - s_attention_start_time) > INTENT_TIMEOUT_MS) {
                    s_state = STATE_IDLE;
                    update_leds(s_state);
                    update_lcd_main("Welcome", "");
                }
                break;
            case STATE_LID_OPEN:
                if ((now - s_lid_open_start_time) > LID_OPEN_DURATION_MS) {
                    lid_close("auto close after deposit");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    s_state = STATE_LID_CLOSE_WAIT;
                    update_leds(s_state);
                    update_lcd_main("Thank you", "");
                    play_gentle_fade();
                }
                break;
            case STATE_LID_CLOSE_WAIT:
                if ((now - s_lid_open_start_time) > (LID_OPEN_DURATION_MS + 1000)) {
                    s_state = STATE_IDLE;
                    update_leds(s_state);
                    update_lcd_main("Welcome", "");
                }
                break;
            case STATE_MAINTENANCE:
                // Stay with lid open, no auto‑close
                break;
        }
        update_lcd_gps_fill();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ========== Driver Initialisation ========== */
static esp_err_t init_drivers(void) {
    s_gps_mutex = xSemaphoreCreateMutex();
    s_servo_mutex = xSemaphoreCreateMutex();
    s_peers_mutex = xSemaphoreCreateMutex();
    if (!s_gps_mutex || !s_servo_mutex || !s_peers_mutex) return ESP_ERR_NO_MEM;

    // I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .clk_source = I2C_CLK_SRC_APB,
        .glitch_ignore_cnt = 10,
        .flags = { .enable_internal_pullup = false },
    };
    i2c_master_bus_handle_t i2c_bus;
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (ret != ESP_OK) return ret;

    // LCD
    lcd_config_t lcd_cfg = {
        .i2c_bus = i2c_bus,
        .i2c_addr = LCD_I2C_ADDR,
        .cols = LCD_COLS,
        .rows = LCD_ROWS,
        .backlight_on_init = true,
    };
    ret = lcd_driver_create(&lcd_cfg, &s_lcd);
    if (ret != ESP_OK) return ret;
    // Ensure backlight is on
    lcd_driver_backlight(s_lcd, true);
    vTaskDelay(pdMS_TO_TICKS(200)); // extra delay for LCD to stabilise

    // Fill ultrasonic
    ultrasonic_config_t fill_cfg = { .trig_pin = FILL_TRIG, .echo_pin = FILL_ECHO, .timeout_us = 200000 };
    ret = ultrasonic_driver_create(&fill_cfg, &s_fill_sensor);
    if (ret != ESP_OK) return ret;

    // Intention ultrasonic
    ultrasonic_config_t intent_cfg = { .trig_pin = INTENT_TRIG, .echo_pin = INTENT_ECHO, .timeout_us = 200000 };
    ret = ultrasonic_driver_create(&intent_cfg, &s_intent_sensor);
    if (ret != ESP_OK) return ret;

    // PIR
    pir_config_t pir_cfg = { .gpio_num = PIR_GPIO };
    ret = pir_driver_create(&pir_cfg, &s_pir);
    if (ret != ESP_OK) return ret;

    // Servo – dedicated timer
    servo_config_t servo_cfg = {
        .gpio_num = SERVO_GPIO,
        .channel = LEDC_CHANNEL_2,
        .timer = LEDC_TIMER_2,
        .min_pulse_us = 500,
        .max_pulse_us = 2500,
        .freq_hz = 50
    };
    ret = servo_driver_create(&servo_cfg, &s_servo);
    if (ret != ESP_OK) return ret;

    // LEDs
    led_config_t red = { .gpio_num = LED_RED,   .channel = LEDC_CHANNEL_0, .timer = LEDC_TIMER_0, .freq_hz = 1000, .active_high = true };
    led_config_t green = { .gpio_num = LED_GREEN, .channel = LEDC_CHANNEL_1, .timer = LEDC_TIMER_0, .freq_hz = 1000, .active_high = true };
    ret = led_driver_create(&red, &s_led_red);
    if (ret != ESP_OK) return ret;
    ret = led_driver_create(&green, &s_led_green);
    if (ret != ESP_OK) return ret;

    // Buzzer – dedicated timer
    buzzer_config_t buzzer_cfg = {
        .gpio_num = BUZZER_GPIO,
        .channel = LEDC_CHANNEL_3,
        .timer = LEDC_TIMER_3,
        .default_freq_hz = 2000,
        .default_duty_percent = 50
    };
    ret = buzzer_driver_create(&buzzer_cfg, &s_buzzer);
    if (ret != ESP_OK) return ret;

    lid_close("startup close");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "All drivers initialised");
    return ESP_OK;
}

/* ========== Main Entry ========== */
void app_main(void)
{
    ESP_LOGI(TAG, "Smart bin with WiFi/MQTT starting...");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (init_drivers() != ESP_OK) {
        ESP_LOGE(TAG, "Driver init failed");
        return;
    }

    // Quick servo test
    lid_open("startup servo test");
    vTaskDelay(pdMS_TO_TICKS(1500));
    lid_close("startup servo test");
    vTaskDelay(pdMS_TO_TICKS(1500));

    xTaskCreate(gps_task, "gps", 4096, NULL, 3, NULL);
    xTaskCreate(gsm_task, "gsm", 8192, NULL, 4, NULL);
    xTaskCreate(mqtt_network_task, "mqtt", 8192, NULL, 2, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
