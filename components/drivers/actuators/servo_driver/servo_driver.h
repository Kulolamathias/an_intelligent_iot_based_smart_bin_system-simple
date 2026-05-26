#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "esp_err.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct servo_handle_t *servo_handle_t;

typedef struct {
    gpio_num_t gpio_num;
    ledc_channel_t channel;
    ledc_timer_t timer;
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
    uint32_t freq_hz;
} servo_config_t;

esp_err_t servo_driver_create(const servo_config_t *cfg, servo_handle_t *out_handle);

esp_err_t servo_driver_set_angle(servo_handle_t handle, float angle_deg);

esp_err_t servo_driver_stop(servo_handle_t handle);

esp_err_t servo_driver_delete(servo_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif