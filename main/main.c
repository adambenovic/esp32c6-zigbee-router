#include "app_state.h"
#include "led.h"
#include "zigbee.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

app_state_t       g_app_state   = { .led_level = LED_DEFAULT_BRIGHTNESS, .led_on = true };
SemaphoreHandle_t g_state_mutex = NULL;

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex);

    led_init();

    xTaskCreate(led_task,    "led",    2048, NULL, 5, NULL);
    xTaskCreate(zigbee_task, "zigbee", 8192, NULL, 6, NULL);

    ESP_LOGI(TAG, "started");
}
