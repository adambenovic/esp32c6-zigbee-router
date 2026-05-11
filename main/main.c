#include "app_state.h"
#include "led.h"
#include "wifi_sniffer.h"
#include "zigbee.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

/* Globals declared extern in app_state.h */
app_state_t       g_app_state       = { .led_level = LED_DEFAULT_BRIGHTNESS, .led_on = true };
SemaphoreHandle_t g_state_mutex     = NULL;
QueueHandle_t     g_occupancy_queue = NULL;

/* Bridges WiFi sniffer queue → Zigbee report (runs in its own task) */
static void occupancy_bridge_task(void *arg) {
    bool occupied;
    while (1) {
        if (xQueueReceive(g_occupancy_queue, &occupied, portMAX_DELAY) == pdTRUE) {
            zigbee_report_occupancy(occupied);
        }
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_state_mutex     = xSemaphoreCreateMutex();
    g_occupancy_queue = xQueueCreate(4, sizeof(bool));
    configASSERT(g_state_mutex);
    configASSERT(g_occupancy_queue);

    led_init();
    wifi_sniffer_init();

    xTaskCreate(led_task,                      "led",       2048, NULL, 5, NULL);
    xTaskCreate(wifi_sniffer_channel_hop_task, "wifi_hop",  2048, NULL, 4, NULL);
    xTaskCreate(wifi_sniffer_eval_task,        "wifi_eval", 2048, NULL, 4, NULL);
    xTaskCreate(occupancy_bridge_task,         "occ_bridge",2048, NULL, 4, NULL);
    xTaskCreate(zigbee_task,                   "zigbee",    8192, NULL, 6, NULL);

    ESP_LOGI(TAG, "all tasks started");
}
