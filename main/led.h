#pragma once
#include <stdbool.h>
#include <stdint.h>

#define LED_GPIO               15
#define LED_DEFAULT_BRIGHTNESS 128
#define LED_FAST_BLINK_MS      200
#define LED_SLOW_BLINK_MS      1000

typedef enum {
    LED_MODE_SCANNING, /* fast blink — not joined */
    LED_MODE_JOINING,  /* slow blink — joining    */
    LED_MODE_ZHA,      /* ZHA controlled           */
} led_mode_t;

uint32_t led_brightness_to_duty(uint8_t brightness);
void     led_init(void);
void     led_set_mode(led_mode_t mode);
void     led_set_level(uint8_t level); /* 0–254 */
void     led_set_on(bool on);
void     led_task(void *arg);          /* FreeRTOS task */
