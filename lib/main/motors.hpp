#pragma once

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class motor_t {
private:
    ledc_channel_config_t ledc_motor_channel;
    int min_period;
    int max_period;
    ledc_timer_bit_t ledc_timer_resolution;
    uint32_t ledc_frequency;
    double throttle_ = 0;

public:
    motor_t(ledc_channel_t channel, int gpio_num, ledc_timer_bit_t timer_resolution,
            ledc_mode_t ledc_mode, uint32_t frequency, int T_min, int T_max)
    : min_period(T_min), max_period(T_max), ledc_timer_resolution(timer_resolution), ledc_frequency(frequency) {
        ESP_LOGI("MOTOR", "IN CONSTRUCTOR: init on %d", gpio_num);
        ledc_motor_channel = {
            .gpio_num       = gpio_num,
            .speed_mode     = ledc_mode,
            .channel        = channel,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0,
            .hpoint         = 0,
            .sleep_mode      = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = 0
        };
    }

    double get_throttle() const noexcept { return throttle_; }
    void   set_throttle(double throttle) { return throttle_ = throttle; }

    void init_motor() {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_motor_channel));
    }

    void update_duty() {
        ESP_LOGI("MOTOR", "motor_t at pin %d set at %%%0.2f throttle", ledc_motor_channel.gpio_num, throttle_);
        float duty_ms = min_period*(1 + (max_period/min_period - 1)*(throttle_/100));

        uint32_t max_duty = (1 << ledc_timer_resolution) - 1;
        uint32_t duty = (duty_ms * ledc_frequency * max_duty) / 1000000;
        
        ESP_ERROR_CHECK(ledc_set_duty(ledc_motor_channel.speed_mode, ledc_motor_channel.channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(ledc_motor_channel.speed_mode, ledc_motor_channel.channel));
    }
};