#include "led.h"
#include "app_state.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LEDC_SPEED   LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER   LEDC_TIMER_0
#define LEDC_FREQ_HZ 5000
#define LEDC_BITS    LEDC_TIMER_8_BIT

static led_mode_t s_mode = LED_MODE_SCANNING;

uint32_t led_brightness_to_duty(uint8_t brightness) {
    return (uint32_t)(255 - brightness);
}

static void set_raw_duty(uint32_t duty) {
    ledc_set_duty(LEDC_SPEED, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED, LEDC_CHANNEL);
}

void led_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_SPEED,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_BITS,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_SPEED,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .gpio_num   = LED_GPIO,
        .duty       = 255, /* off at start (active LOW) */
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);
}

void led_set_mode(led_mode_t mode) {
    s_mode = mode;
}

void led_set_level(uint8_t level) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_app_state.led_level = level;
    bool on = g_app_state.led_on;
    xSemaphoreGive(g_state_mutex);
    if (s_mode == LED_MODE_ZHA && on) {
        set_raw_duty(led_brightness_to_duty(level));
    }
}

void led_set_on(bool on) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_app_state.led_on    = on;
    uint8_t level         = g_app_state.led_level;
    xSemaphoreGive(g_state_mutex);
    if (s_mode == LED_MODE_ZHA) {
        set_raw_duty(on ? led_brightness_to_duty(level) : 255);
    }
}

void led_task(void *arg) {
    bool blink_state = false;
    while (1) {
        switch (s_mode) {
            case LED_MODE_SCANNING:
                blink_state = !blink_state;
                set_raw_duty(blink_state ? 0 : 255);
                vTaskDelay(pdMS_TO_TICKS(LED_FAST_BLINK_MS));
                break;
            case LED_MODE_JOINING:
                blink_state = !blink_state;
                set_raw_duty(blink_state ? 0 : 255);
                vTaskDelay(pdMS_TO_TICKS(LED_SLOW_BLINK_MS));
                break;
            case LED_MODE_ZHA:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}
