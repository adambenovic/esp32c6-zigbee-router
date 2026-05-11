#include "zigbee.h"
#include "app_state.h"
#include "led.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zigbee";
#define NVS_NS        "led_state"
#define NVS_KEY_LEVEL "level"
#define NVS_KEY_ON    "on"

/* HA Occupancy Sensor device ID (0x0107 per ZHA spec) */
#define ESP_ZB_HA_OCCUPANCY_SENSOR_DEVICE_ID 0x0107U

/* ── NVS helpers ─────────────────────────────────────────── */

static void nvs_save_led(uint8_t level, bool on) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, NVS_KEY_LEVEL, level);
    nvs_set_u8(h, NVS_KEY_ON, on ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_load_led(uint8_t *level, bool *on) {
    *level = LED_DEFAULT_BRIGHTNESS;
    *on    = true;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    nvs_get_u8(h, NVS_KEY_LEVEL, level);
    uint8_t on_val = 1;
    nvs_get_u8(h, NVS_KEY_ON, &on_val);
    *on = (on_val != 0);
    nvs_close(h);
}

/* ── Scheduler callback wrappers ─────────────────────────── */

static void bdb_start_commissioning_cb(uint8_t mode_mask) {
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

/* ── Occupancy report ────────────────────────────────────── */

void zigbee_report_occupancy(bool occupied) {
    uint8_t val = occupied ? 1 : 0;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(
        1,
        ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
        &val, false);

    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd.src_endpoint = 1,
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID    = ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
        .attributeID  = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
    };
    esp_zb_zcl_report_attr_cmd_req(&cmd);
    esp_zb_lock_release();
}

/* ── ZHA → LED action handler ────────────────────────────── */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *msg) {
    if (cb_id != ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) return ESP_OK;

    const esp_zb_zcl_set_attr_value_message_t *m =
        (const esp_zb_zcl_set_attr_value_message_t *)msg;
    if (m->info.dst_endpoint != 2) return ESP_OK;

    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        bool on = *(bool *)m->attribute.data.value;
        led_set_on(on);
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        uint8_t level = g_app_state.led_level;
        xSemaphoreGive(g_state_mutex);
        nvs_save_led(level, on);

    } else if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
               m->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) {
        uint8_t level = *(uint8_t *)m->attribute.data.value;
        led_set_level(level);
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool on = g_app_state.led_on;
        xSemaphoreGive(g_state_mutex);
        nvs_save_led(level, on);
    }
    return ESP_OK;
}

/* ── Signal handler ──────────────────────────────────────── */

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_s) {
    esp_zb_app_signal_type_t sig = *(esp_zb_app_signal_type_t *)signal_s->p_app_signal;
    esp_err_t err = signal_s->esp_err_status;

    switch (sig) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err == ESP_OK) {
                led_set_mode(LED_MODE_JOINING);
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_app_state.zigbee_joining = true;
                xSemaphoreGive(g_state_mutex);
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGW(TAG, "Init failed, retrying");
                esp_zb_scheduler_alarm(bdb_start_commissioning_cb,
                    ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Joined network on channel %d", esp_zb_get_current_channel());
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_app_state.zigbee_joined  = true;
                g_app_state.zigbee_joining = false;
                xSemaphoreGive(g_state_mutex);
                uint8_t level;
                bool on;
                nvs_load_led(&level, &on);
                led_set_mode(LED_MODE_ZHA);
                led_set_level(level);
                led_set_on(on);
            } else {
                ESP_LOGW(TAG, "Steering failed, retrying");
                esp_zb_scheduler_alarm(bdb_start_commissioning_cb,
                    ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;

        default:
            ESP_LOGD(TAG, "signal 0x%x status %s", sig, esp_err_to_name(err));
            break;
    }
}

/* ── Endpoint creation ───────────────────────────────────── */

static void create_endpoints(void) {
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01, /* mains */
    };

    /* EP1 — Occupancy Sensor */
    esp_zb_cluster_list_t *ep1 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep1,
        esp_zb_basic_cluster_create(&basic_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_occupancy_sensing_cluster_cfg_t occ_cfg = {
        .occupancy = 0,
        .sensor_type =
            ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC,
        .sensor_type_bitmap = 0x02,
    };
    esp_zb_cluster_list_add_occupancy_sensing_cluster(ep1,
        esp_zb_occupancy_sensing_cluster_create(&occ_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* EP2 — Dimmable Light */
    esp_zb_cluster_list_t *ep2 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(ep2,
        esp_zb_basic_cluster_create(&basic_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_on_off_cluster_cfg_t on_off_cfg = { .on_off = 1 };
    esp_zb_cluster_list_add_on_off_cluster(ep2,
        esp_zb_on_off_cluster_create(&on_off_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_level_cluster_cfg_t level_cfg = { .current_level = LED_DEFAULT_BRIGHTNESS };
    esp_zb_cluster_list_add_level_cluster(ep2,
        esp_zb_level_cluster_create(&level_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Register */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_ep_list_add_ep(ep_list, ep1, (esp_zb_endpoint_config_t){
        .endpoint        = 1,
        .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id   = ESP_ZB_HA_OCCUPANCY_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    });

    esp_zb_ep_list_add_ep(ep_list, ep2, (esp_zb_endpoint_config_t){
        .endpoint        = 2,
        .app_profile_id  = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id   = ESP_ZB_HA_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    });

    esp_zb_device_register(ep_list);
}

/* ── Public API ──────────────────────────────────────────── */

void zigbee_init(void) {
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg.max_children = 10,
    };
    esp_zb_init(&zb_cfg);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    create_endpoints();
    esp_zb_core_action_handler_register(zb_action_handler);
}

void zigbee_task(void *arg) {
    zigbee_init();
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}
