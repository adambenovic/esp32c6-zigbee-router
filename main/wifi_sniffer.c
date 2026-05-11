#include "wifi_sniffer.h"
#include "app_state.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wifi_sniffer";

static volatile int64_t s_last_activity = 0;
static volatile bool    s_ever_seen     = false;

bool wifi_sniffer_occupancy_check(bool ever_seen, int64_t now_us, int64_t last_us) {
    if (!ever_seen) return false;
    return (now_us - last_us) < ((int64_t)OCCUPANCY_TIMEOUT_SEC * 1000000LL);
}

static void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;
    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    uint8_t subtype = (pkt->payload[0] >> 4) & 0x0F;
    if (subtype == 0x04) { /* probe request */
        s_last_activity = esp_timer_get_time();
        s_ever_seen     = true;
    }
}

void wifi_sniffer_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb));
    ESP_LOGI(TAG, "initialized, scanning channels %d-%d",
             WIFI_CHANNEL_MIN, WIFI_CHANNEL_MAX);
}

void wifi_sniffer_channel_hop_task(void *arg) {
    uint8_t ch = WIFI_CHANNEL_MIN;
    while (1) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool joined = g_app_state.zigbee_joined;
        xSemaphoreGive(g_state_mutex);

        if (joined) {
            esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
            ch = (ch >= WIFI_CHANNEL_MAX) ? WIFI_CHANNEL_MIN : ch + 1;
        }
        vTaskDelay(pdMS_TO_TICKS(WIFI_CHANNEL_DWELL_MS));
    }
}

void wifi_sniffer_eval_task(void *arg) {
    bool prev_occupied = false;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WIFI_EVAL_INTERVAL_MS));
        int64_t now      = esp_timer_get_time();
        bool    occupied = wifi_sniffer_occupancy_check(s_ever_seen, now, s_last_activity);
        if (occupied == prev_occupied) continue;

        prev_occupied = occupied;
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        g_app_state.occupied = occupied;
        xSemaphoreGive(g_state_mutex);
        xQueueSend(g_occupancy_queue, &occupied, 0);
        ESP_LOGI(TAG, "occupancy -> %s", occupied ? "OCCUPIED" : "UNOCCUPIED");
    }
}
