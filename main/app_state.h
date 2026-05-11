#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    bool    zigbee_joined;
    bool    zigbee_joining;
    uint8_t led_level;
    bool    led_on;
} app_state_t;

extern app_state_t       g_app_state;
extern SemaphoreHandle_t g_state_mutex;
