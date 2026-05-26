#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
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

/* GSM thresholds */
#define NEAR_FULL_PERCENT  75
#define FULL_PERCENT       100
#define DEBOUNCE_SEC       300
#define COLLECTOR_PHONE    "+255688173415"
#define GSM_PASSWORD       "SECRET123"

/* Peer registry */
#define MAX_PEERS 16
typedef struct {
    char id[13];
    float lat;
    float lon;
    uint8_t fill;
    bool active;
    uint32_t last_seen;
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

static SemaphoreHandle_t s_peers_mutex = NULL;
static peer_t s_peers[MAX_PEERS] = {0};
static bool s_wifi_connected = false;
static bool s_mqtt_connected = false;
static bool s_redirect_pending = false;   // set when bin becomes full

static uint8_t s_current_fill = 0;
static uint8_t s_last_sent_fill = 0xFF;
static uint32_t s_last_sms_time = 0;

/* ========== State Machine ========== */
typedef enum {
    STATE_IDLE,
    STATE_ATTENTION,
    STATE_LID_OPEN,
    STATE_LID_CLOSE_WAIT,
    STATE_MAINTENANCE
} system_state_t;
static system_state_t s_state = STATE_IDLE;
static uint32_t s_attention_start_time = 0;
static uint32_t s_lid_open_start_time = 0;

static void on_sms_received(const received_sms_t *sms);

/* ========== Helper Functions ========== */
static uint32_t get_time_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static float haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    double R = 6371000.0;
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double dphi = (lat2 - lat1) * M_PI / 180.0;
    double dlambda = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dphi/2)*sin(dphi/2) + cos(phi1)*cos(phi2)*sin(dlambda/2)*sin(dlambda/2);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(R * c);
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

static uint8_t get_fill_percent(void) {
    if (!s_fill_sensor) return 0;
    uint32_t samples[5];
    int valid = 0;
    for (int i = 0; i < 5; i++) {
        uint32_t pulse_us;
        if (ultrasonic_driver_measure(s_fill_sensor, &pulse_us) == ESP_OK) {
            uint32_t dist = (pulse_us * 1715) / 100000;
            if (dist >= 2 && dist <= 400) samples[valid++] = dist;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (valid == 0) return 0;
    uint32_t filtered = median_filter_5(samples);
    if (filtered >= 55) return 0;
    if (filtered <= 2) return 100;
    return (uint8_t)(((55 - filtered) * 100) / 55);
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

static void update_lcd_gps_fill(void) {
    static char line3[21] = "", line4[21] = "";
    char new3[21], new4[21];
    snprintf(new3, sizeof(new3), "Fill: %3d%%", s_current_fill);
    if (s_gps_valid) {
        double lat, lon;
        if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            lat = s_last_gps.latitude;
            lon = s_last_gps.longitude;
            xSemaphoreGive(s_gps_mutex);
        } else { lat = 0; lon = 0; }
        snprintf(new4, sizeof(new4), "%.6f %.6f", lat, lon);
    } else {
        snprintf(new4, sizeof(new4), "GPS: no fix");
    }
    if (strcmp(line3, new3) != 0) {
        lcd_driver_write_string(s_lcd, 2, 0, new3);
        vTaskDelay(pdMS_TO_TICKS(5));
        strcpy(line3, new3);
    }
    if (strcmp(line4, new4) != 0) {
        lcd_driver_write_string(s_lcd, 3, 0, new4);
        strcpy(line4, new4);
    }
}

static void update_leds(system_state_t state) {
    led_driver_off(s_led_red);
    led_driver_off(s_led_green);
    if (state == STATE_ATTENTION) {
        led_driver_start_blink(s_led_red, 500, 50);
    } else if (state == STATE_LID_OPEN || state == STATE_LID_CLOSE_WAIT) {
        led_driver_on(s_led_green);
    }
}

/* ========== GPS Task ========== */
static void gps_task(void *pvParameters) {
    gps_proof_init();
    while (1) {
        gps_data_t fix;
        esp_err_t ret = gps_proof_get_fix(&fix, 2000);
        if (ret == ESP_OK && fix.fix_quality >= 1 && fix.latitude != 0.0 && fix.longitude != 0.0) {
            if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(&s_last_gps, &fix, sizeof(gps_data_t));
                s_gps_valid = true;
                xSemaphoreGive(s_gps_mutex);
            }
        } else if (s_gps_valid) {
            if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                s_gps_valid = false;
                xSemaphoreGive(s_gps_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

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

static void mqtt_event_cb(mqtt_client_event_t event, void *data) {
    if (event == MQTT_CLIENT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected");
        s_mqtt_connected = true;
        // Subscribe to discovery and peer topics
        mqtt_client_subscribe("smartbin/discovery/announce", 1);
        mqtt_client_subscribe("smartbin/bin/+/state", 1);
        mqtt_client_subscribe("smartbin/bin/+/lwt", 1);
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
            cJSON *root = cJSON_Parse(payload);
            if (root) {
                const char *id = cJSON_GetStringValue(cJSON_GetObjectItem(root, "id"));
                double lat = cJSON_GetObjectItem(root, "lat")->valuedouble;
                double lon = cJSON_GetObjectItem(root, "lon")->valuedouble;
                int fill = cJSON_GetObjectItem(root, "fill")->valueint;
                if (id && lat != 0 && lon != 0) {
                    if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        peer_t *p = NULL;
                        for (int i = 0; i < MAX_PEERS; i++) {
                            if (s_peers[i].active && strcmp(s_peers[i].id, id) == 0) {
                                p = &s_peers[i]; break;
                            }
                        }
                        if (!p) {
                            for (int i = 0; i < MAX_PEERS; i++) {
                                if (!s_peers[i].active) {
                                    p = &s_peers[i]; break;
                                }
                            }
                        }
                        if (p) {
                            strlcpy(p->id, id, sizeof(p->id));
                            p->lat = lat;
                            p->lon = lon;
                            p->fill = fill;
                            p->active = true;
                            p->last_seen = get_time_ms() / 1000;
                        }
                        xSemaphoreGive(s_peers_mutex);
                    }
                }
                cJSON_Delete(root);
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
    char client_id[20];
    snprintf(client_id, sizeof(client_id), "bin_%s", mac_str);

    char base_topic[32];
    mqtt_topic_init(base_topic, sizeof(base_topic));

    // Build LWT topic string
    char lwt_topic[64];
    snprintf(lwt_topic, sizeof(lwt_topic), "smartbin/bin/%s/lwt", mac_str);

    uint32_t last_publish = 0, last_heartbeat = 0, last_state = 0, last_expiry = 0;
    bool mqtt_initialised = false;
    uint32_t last_wifi_check = 0;

    while (1) {
        uint32_t now = get_time_ms() / 1000;

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
            // Publish data to devices/ topic every 10 seconds
            if ((now - last_publish) >= 10) {
                char topic[128];
                mqtt_topic_build(topic, sizeof(topic), "data");
                char payload[256];
                if (s_gps_valid) {
                    double lat, lon;
                    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        lat = s_last_gps.latitude;
                        lon = s_last_gps.longitude;
                        xSemaphoreGive(s_gps_mutex);
                    } else { lat = 0; lon = 0; }
                    snprintf(payload, sizeof(payload),
                             "{\"fill\":%d,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f}",
                             s_current_fill, lat, lon, (double)s_last_gps.altitude);
                } else {
                    snprintf(payload, sizeof(payload), "{\"fill\":%d,\"lat\":null,\"lon\":null,\"alt\":null}",
                             s_current_fill);
                }
                mqtt_client_publish(topic, payload, strlen(payload), 1, false);
                last_publish = now;
            }

            // Publish to smartbin/bin/<mac>/state every 30 seconds
            if ((now - last_state) >= 30) {
                char topic[128];
                snprintf(topic, sizeof(topic), "smartbin/bin/%s/state", mac_str);
                char payload[256];
                if (s_gps_valid) {
                    double lat, lon;
                    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        lat = s_last_gps.latitude;
                        lon = s_last_gps.longitude;
                        xSemaphoreGive(s_gps_mutex);
                    } else { lat = 0; lon = 0; }
                    snprintf(payload, sizeof(payload),
                             "{\"fill\":%d,\"lat\":%.6f,\"lon\":%.6f,\"timestamp\":%lu}",
                             s_current_fill, lat, lon, now);
                } else {
                    snprintf(payload, sizeof(payload),
                             "{\"fill\":%d,\"lat\":null,\"lon\":null,\"timestamp\":%lu}",
                             s_current_fill, now);
                }
                mqtt_client_publish(topic, payload, strlen(payload), 1, false);
                last_state = now;
            }

            // Publish to smartbin/cloud/bin/<mac>/heartbeat every 30 seconds
            if ((now - last_heartbeat) >= 30) {
                char topic[128];
                snprintf(topic, sizeof(topic), "smartbin/cloud/bin/%s/heartbeat", mac_str);
                char payload[256];
                if (s_gps_valid) {
                    double lat, lon;
                    if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        lat = s_last_gps.latitude;
                        lon = s_last_gps.longitude;
                        xSemaphoreGive(s_gps_mutex);
                    } else { lat = 0; lon = 0; }
                    snprintf(payload, sizeof(payload),
                             "{\"fill\":%d,\"lat\":%.6f,\"lon\":%.6f,\"timestamp\":%lu}",
                             s_current_fill, lat, lon, now);
                } else {
                    snprintf(payload, sizeof(payload),
                             "{\"fill\":%d,\"lat\":null,\"lon\":null,\"timestamp\":%lu}",
                             s_current_fill, now);
                }
                mqtt_client_publish(topic, payload, strlen(payload), 1, false);
                last_heartbeat = now;
            }

            // Expire old peers
            if ((now - last_expiry) >= 10) {
                if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    for (int i = 0; i < MAX_PEERS; i++) {
                        if (s_peers[i].active && (now - s_peers[i].last_seen) > 30) {
                            s_peers[i].active = false;
                        }
                    }
                    xSemaphoreGive(s_peers_mutex);
                }
                last_expiry = now;
            }

            // Handle redirect when bin becomes full
            if (s_redirect_pending) {
                s_redirect_pending = false;
                peer_t *nearest = NULL;
                float min_dist = INFINITY;
                if (xSemaphoreTake(s_peers_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (s_gps_valid) {
                        double my_lat, my_lon;
                        if (xSemaphoreTake(s_gps_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            my_lat = s_last_gps.latitude;
                            my_lon = s_last_gps.longitude;
                            xSemaphoreGive(s_gps_mutex);
                        } else { my_lat = 0; my_lon = 0; }
                        for (int i = 0; i < MAX_PEERS; i++) {
                            if (s_peers[i].active && s_peers[i].fill < 100) {
                                float dist = haversine_distance(my_lat, my_lon,
                                                                 s_peers[i].lat, s_peers[i].lon);
                                if (dist < min_dist) {
                                    min_dist = dist;
                                    nearest = &s_peers[i];
                                }
                            }
                        }
                    }
                    xSemaphoreGive(s_peers_mutex);
                }
                if (nearest) {
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Use bin %.0fm", min_dist);
                    update_lcd_main("Bin full - redirect", msg);
                    ESP_LOGI(TAG, "Redirect to %s at %.0f m", nearest->id, min_dist);
                } else {
                    update_lcd_main("Bin full", "No other bin");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ========== Main Control Task ========== */
static void control_task(void *pvParameters) {
    bool pir_detected = false;
    while (1) {
        uint32_t now = get_time_ms();
        pir_driver_read(s_pir, &pir_detected);
        bool hand = is_hand_detected();
        uint8_t fill = get_fill_percent();
        s_current_fill = fill;

        // Request redirect when bin becomes full (and not already in maintenance)
        if (fill >= 100 && s_state != STATE_MAINTENANCE && !s_redirect_pending) {
            s_redirect_pending = true;
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
                    servo_driver_set_angle(s_servo, 90.0f);
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
                    servo_driver_set_angle(s_servo, 0.0f);
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
    s_peers_mutex = xSemaphoreCreateMutex();
    if (!s_gps_mutex || !s_peers_mutex) return ESP_ERR_NO_MEM;

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

    servo_driver_set_angle(s_servo, 0.0f);
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
    servo_driver_set_angle(s_servo, 90.0f);
    vTaskDelay(pdMS_TO_TICKS(1500));
    servo_driver_set_angle(s_servo, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(1500));

    xTaskCreate(gps_task, "gps", 4096, NULL, 3, NULL);
    xTaskCreate(gsm_task, "gsm", 8192, NULL, 4, NULL);
    xTaskCreate(mqtt_network_task, "mqtt", 8192, NULL, 2, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}