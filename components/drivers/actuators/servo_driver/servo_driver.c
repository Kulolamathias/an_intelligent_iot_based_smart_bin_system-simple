#include "servo_driver.h"
#include "esp_log.h"
#include <stdlib.h>
#include <math.h>

static const char *TAG = "SERVO_DRV";

#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION_STEPS (1 << LEDC_RESOLUTION_BITS)

struct servo_handle_t {
    ledc_channel_t channel;
    ledc_timer_t timer;
    gpio_num_t gpio_num;
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
    uint32_t freq_hz;
    bool initialized;
};

/* Convert angle to duty cycle (0..1023) */
static uint32_t angle_to_duty(servo_handle_t handle, float angle_deg)
{
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;

    uint32_t pulse_us = handle->min_pulse_us +
        (uint32_t)((handle->max_pulse_us - handle->min_pulse_us) * angle_deg / 180.0f);

    uint32_t period_us = 1000000 / handle->freq_hz;
    uint32_t duty = (pulse_us * LEDC_RESOLUTION_STEPS) / period_us;
    if (duty >= LEDC_RESOLUTION_STEPS) duty = LEDC_RESOLUTION_STEPS - 1;
    return duty;
}

esp_err_t servo_driver_create(const servo_config_t *cfg, servo_handle_t *out_handle)
{
    if (!cfg || !out_handle) return ESP_ERR_INVALID_ARG;

    servo_handle_t handle = calloc(1, sizeof(struct servo_handle_t));
    if (!handle) return ESP_ERR_NO_MEM;

    handle->gpio_num = cfg->gpio_num;
    handle->channel = cfg->channel;
    handle->timer = cfg->timer;
    handle->min_pulse_us = cfg->min_pulse_us;
    handle->max_pulse_us = cfg->max_pulse_us;
    handle->freq_hz = cfg->freq_hz;

    /* Configure LEDC timer – only once */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION_BITS,
        .timer_num = handle->timer,
        .freq_hz = handle->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        free(handle);
        return ret;
    }

    /* Configure LEDC channel */
    ledc_channel_config_t ch_cfg = {
        .gpio_num = handle->gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = handle->channel,
        .timer_sel = handle->timer,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        free(handle);
        return ret;
    }

    handle->initialized = true;
    *out_handle = handle;
    ESP_LOGI(TAG, "Servo created: GPIO %d, channel %d, timer %d, freq %" PRIu32 " Hz",
             handle->gpio_num, handle->channel, handle->timer, handle->freq_hz);
    return ESP_OK;
}

esp_err_t servo_driver_set_angle(servo_handle_t handle, float angle_deg)
{
    if (!handle || !handle->initialized) return ESP_ERR_INVALID_STATE;

    uint32_t duty = angle_to_duty(handle, angle_deg);
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, handle->channel, duty);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, handle->channel);
    return ret;
}

esp_err_t servo_driver_stop(servo_handle_t handle)
{
    if (!handle || !handle->initialized) return ESP_ERR_INVALID_STATE;
    return ledc_set_duty(LEDC_LOW_SPEED_MODE, handle->channel, 0);
}

esp_err_t servo_driver_delete(servo_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    servo_driver_stop(handle);
    free(handle);
    ESP_LOGI(TAG, "Servo deleted");
    return ESP_OK;
}