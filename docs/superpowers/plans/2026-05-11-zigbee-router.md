# ESP32-C6 Zigbee Router + Presence Sensor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build ESP-IDF firmware for the XIAO ESP32-C6 that acts as a Zigbee mesh router, reports room-level occupancy via passive WiFi probe sniffing to ZHA, and exposes the onboard LED as a ZHA-controllable dimmable light.

**Architecture:** Three FreeRTOS tasks (WiFi sniffer, Zigbee stack, LED driver) share state via a mutex-protected struct and a FreeRTOS queue for occupancy updates. Zigbee runs as a router with two endpoints: EP1 occupancy sensor, EP2 dimmable light. Zigbee API calls from non-Zigbee tasks use `esp_zb_lock_acquire/release` for thread safety.

**Tech Stack:** ESP-IDF v5.3 (Docker `espressif/idf:v5.3`), esp-zigbee-sdk (IDF Component Manager), FreeRTOS, LEDC PWM, NVS, GitHub Actions, ESP Web Tools

---

## File Map

| File | Role |
|---|---|
| `CMakeLists.txt` | Top-level project CMake |
| `main/CMakeLists.txt` | Main component sources |
| `main/idf_component.yml` | IDF Component Manager: esp-zigbee-sdk |
| `sdkconfig.defaults` | Zigbee router + coex + WiFi config |
| `partitions.csv` | Custom partition table with Zigbee NVS storage |
| `Makefile` | Docker build/flash/monitor |
| `main/app_state.h` | Shared state struct, mutex, occupancy queue — extern declarations |
| `main/led.h` | LED constants, public API |
| `main/led.c` | LEDC PWM driver, pre/post-join state machine |
| `main/wifi_sniffer.h` | Occupancy constants, sniffer API |
| `main/wifi_sniffer.c` | Promiscuous mode, channel hop task, occupancy eval task |
| `main/zigbee.h` | Zigbee public API |
| `main/zigbee.c` | Router init, EP1 occupancy, EP2 dimmable light, NVS, signal handler |
| `main/main.c` | app_main: NVS init, globals, task startup, occupancy bridge task |
| `test/host/test_led.c` | Host unit test: `led_brightness_to_duty` |
| `test/host/test_occupancy.c` | Host unit test: `wifi_sniffer_occupancy_check` |
| `test/host/Makefile` | `gcc` build for host tests |
| `.github/workflows/build.yml` | CI: build on push, release merged binary on tag |
| `docs/manifest.json` | ESP Web Tools firmware manifest |
| `docs/index.html` | ESP Web Tools web flasher page (GitHub Pages) |
| `README.md` | Setup, build, flash, pairing instructions |

---

## Task 1: Project scaffold

**Files:**
- Create: `CMakeLists.txt`
- Create: `main/CMakeLists.txt`
- Create: `main/idf_component.yml`
- Create: `sdkconfig.defaults`
- Create: `partitions.csv`
- Create: `Makefile`

- [ ] **Step 1: Create top-level CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32c6_zigbee_router)
```

- [ ] **Step 2: Create main/CMakeLists.txt**

```cmake
idf_component_register(
    SRCS
        "main.c"
        "led.c"
        "wifi_sniffer.c"
        "zigbee.c"
    INCLUDE_DIRS "."
)
```

- [ ] **Step 3: Create main/idf_component.yml**

```yaml
dependencies:
  espressif/esp-zboss-port:
    version: ">=1.5.0"
  espressif/esp-zigbee-lib:
    version: ">=1.5.0"
  idf: ">=5.3.0"
```

- [ ] **Step 4: Create sdkconfig.defaults**

```
CONFIG_IDF_TARGET="esp32c6"
CONFIG_ZB_ENABLED=y
CONFIG_ZB_ZCZR=y
CONFIG_ZB_RADIO_NATIVE=y
CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y
CONFIG_ESP_WIFI_ENABLED=y
CONFIG_IEEE802154_ENABLED=y
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_NVS_ENCRYPTION=n
```

- [ ] **Step 5: Create partitions.csv**

```
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x180000,
zb_stor,  data, fat,     0x190000,0x8000,
```

- [ ] **Step 6: Create Makefile**

```makefile
IMAGE  := espressif/idf:v5.3
PORT   ?= /dev/ttyACM0
DOCKER := docker run --rm -v $(PWD):/project -w /project

.PHONY: build flash monitor clean

build:
	$(DOCKER) $(IMAGE) idf.py build

flash:
	$(DOCKER) --device $(PORT) $(IMAGE) idf.py -p $(PORT) flash

monitor:
	$(DOCKER) -it --device $(PORT) $(IMAGE) idf.py -p $(PORT) monitor

clean:
	$(DOCKER) $(IMAGE) idf.py fullclean
```

- [ ] **Step 7: Pull Docker image**

```bash
docker pull espressif/idf:v5.3
```
Expected: image pulled or "Status: Image is up to date"

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt main/CMakeLists.txt main/idf_component.yml sdkconfig.defaults partitions.csv Makefile
git commit -m "feat: project scaffold — CMake, sdkconfig, partitions, Makefile"
```

---

## Task 2: Shared state

**Files:**
- Create: `main/app_state.h`

- [ ] **Step 1: Create main/app_state.h**

```c
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

typedef struct {
    bool    occupied;
    bool    zigbee_joined;
    bool    zigbee_joining;
    uint8_t led_level;   /* 0–254, Zigbee Level Control range */
    bool    led_on;
} app_state_t;

extern app_state_t       g_app_state;
extern SemaphoreHandle_t g_state_mutex;
extern QueueHandle_t     g_occupancy_queue; /* bool items */
```

- [ ] **Step 2: Commit**

```bash
git add main/app_state.h
git commit -m "feat: shared app state header"
```

---

## Task 3: LED driver

**Files:**
- Create: `test/host/Makefile`
- Create: `test/host/test_led.c`
- Create: `main/led.h`
- Create: `main/led.c`

- [ ] **Step 1: Create test/host/Makefile**

```makefile
.PHONY: test clean

test: test_led test_occupancy
	./test_led
	./test_occupancy

test_led: test_led.c
	gcc -Wall -Werror -o test_led test_led.c

test_occupancy: test_occupancy.c
	gcc -Wall -Werror -o test_occupancy test_occupancy.c

clean:
	rm -f test_led test_occupancy
```

- [ ] **Step 2: Write failing unit test for brightness_to_duty**

Create `test/host/test_led.c`:

```c
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Active LOW: 255 = LED off, 0 = LED full on. duty = 255 - brightness */
static uint32_t led_brightness_to_duty(uint8_t brightness) {
    return 9999; /* placeholder — will fail */
}

int main(void) {
    assert(led_brightness_to_duty(0)   == 255); /* off */
    assert(led_brightness_to_duty(254) == 1);   /* max brightness */
    assert(led_brightness_to_duty(127) == 128); /* ~50% */
    printf("test_led PASSED\n");
    return 0;
}
```

- [ ] **Step 3: Run — verify it fails**

```bash
cd test/host && make test_led && ./test_led
```
Expected: assertion failure (`9999 != 255`)

- [ ] **Step 4: Create main/led.h**

```c
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
```

- [ ] **Step 5: Fix test — implement led_brightness_to_duty and verify passes**

Update `test/host/test_led.c`, replace the placeholder:

```c
static uint32_t led_brightness_to_duty(uint8_t brightness) {
    return (uint32_t)(255 - brightness);
}
```

Run:
```bash
cd test/host && make test_led && ./test_led
```
Expected: `test_led PASSED`

- [ ] **Step 6: Create main/led.c**

```c
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
```

- [ ] **Step 7: Commit**

```bash
git add main/led.h main/led.c test/host/test_led.c test/host/Makefile
git commit -m "feat: LED driver — LEDC PWM, state machine, host unit test"
```

---

## Task 4: WiFi sniffer

**Files:**
- Create: `test/host/test_occupancy.c`
- Create: `main/wifi_sniffer.h`
- Create: `main/wifi_sniffer.c`

- [ ] **Step 1: Write failing unit test for occupancy_check**

Create `test/host/test_occupancy.c`:

```c
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define OCCUPANCY_TIMEOUT_SEC 300

static bool wifi_sniffer_occupancy_check(bool ever_seen, int64_t now_us, int64_t last_us) {
    return true; /* placeholder — will fail */
}

int main(void) {
    int64_t t = (int64_t)OCCUPANCY_TIMEOUT_SEC * 1000000LL;

    /* never seen → always unoccupied */
    assert(wifi_sniffer_occupancy_check(false, t + 1000000LL, 0) == false);

    /* seen 1s ago, well within timeout → occupied */
    assert(wifi_sniffer_occupancy_check(true, t + 1000000LL, t) == true);

    /* seen at t=0, checked at 2*timeout+1s → unoccupied */
    assert(wifi_sniffer_occupancy_check(true, (2 * t) + 1000000LL, 0) == false);

    printf("test_occupancy PASSED\n");
    return 0;
}
```

- [ ] **Step 2: Run — verify it fails**

```bash
cd test/host && make test_occupancy && ./test_occupancy
```
Expected: assertion failure on the first `assert` (`true != false`)

- [ ] **Step 3: Create main/wifi_sniffer.h**

```c
#pragma once
#include <stdbool.h>
#include <stdint.h>

#define OCCUPANCY_TIMEOUT_SEC  300
#define WIFI_CHANNEL_MIN         1
#define WIFI_CHANNEL_MAX        13
#define WIFI_CHANNEL_DWELL_MS  200
#define WIFI_EVAL_INTERVAL_MS  30000

bool wifi_sniffer_occupancy_check(bool ever_seen, int64_t now_us, int64_t last_us);
void wifi_sniffer_init(void);
void wifi_sniffer_channel_hop_task(void *arg);
void wifi_sniffer_eval_task(void *arg);
```

- [ ] **Step 4: Fix test — implement occupancy_check and verify passes**

Update `test/host/test_occupancy.c`, replace the placeholder:

```c
static bool wifi_sniffer_occupancy_check(bool ever_seen, int64_t now_us, int64_t last_us) {
    if (!ever_seen) return false;
    return (now_us - last_us) < ((int64_t)OCCUPANCY_TIMEOUT_SEC * 1000000LL);
}
```

Run:
```bash
cd test/host && make test_occupancy && ./test_occupancy
```
Expected: `test_occupancy PASSED`

- [ ] **Step 5: Run all host tests**

```bash
cd test/host && make test
```
Expected:
```
test_led PASSED
test_occupancy PASSED
```

- [ ] **Step 6: Create main/wifi_sniffer.c**

```c
#include "wifi_sniffer.h"
#include "app_state.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
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
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb));
    ESP_LOGI(TAG, "initialized, scanning channels %d–%d",
             WIFI_CHANNEL_MIN, WIFI_CHANNEL_MAX);
}

void wifi_sniffer_channel_hop_task(void *arg) {
    uint8_t ch = WIFI_CHANNEL_MIN;
    while (1) {
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(WIFI_CHANNEL_DWELL_MS));
        ch = (ch >= WIFI_CHANNEL_MAX) ? WIFI_CHANNEL_MIN : ch + 1;
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
        ESP_LOGI(TAG, "occupancy → %s", occupied ? "OCCUPIED" : "UNOCCUPIED");
    }
}
```

- [ ] **Step 7: Commit**

```bash
git add main/wifi_sniffer.h main/wifi_sniffer.c test/host/test_occupancy.c
git commit -m "feat: WiFi sniffer — promiscuous mode, channel hop, occupancy eval"
```

---

## Task 5: Zigbee stack

**Files:**
- Create: `main/zigbee.h`
- Create: `main/zigbee.c`

- [ ] **Step 1: Create main/zigbee.h**

```c
#pragma once
#include <stdbool.h>

void zigbee_init(void);
void zigbee_task(void *arg);
void zigbee_report_occupancy(bool occupied);
```

- [ ] **Step 2: Create main/zigbee.c**

```c
#include "zigbee.h"
#include "app_state.h"
#include "led.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ha_standard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zigbee";
#define NVS_NS        "led_state"
#define NVS_KEY_LEVEL "level"
#define NVS_KEY_ON    "on"

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

/* ── Occupancy report (call with Zigbee lock held) ───────── */

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
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attrID       = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
    };
    esp_zb_zcl_report_attr_cmd_req(&cmd);
    esp_zb_lock_release();
}

/* ── ZHA → LED action handler ────────────────────────────── */

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *msg) {
    if (cb_id != ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) return ESP_OK;

    const esp_zb_zcl_set_attr_value_message_t *m =
        (const esp_zb_zcl_set_attr_value_message_t *)msg;
    if (m->info.dst.endpoint != 2) return ESP_OK;

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

static void zb_signal_handler(esp_zb_app_signal_t *signal_s) {
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
                esp_zb_scheduler_alarm(
                    (esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
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
                led_set_level(level);
                led_set_on(on);
                led_set_mode(LED_MODE_ZHA);
            } else {
                ESP_LOGW(TAG, "Steering failed, retrying");
                esp_zb_scheduler_alarm(
                    (esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
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
        .occupancy_sensor_type =
            ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC,
        .occupancy_sensor_type_bitmap = 0x02,
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
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    esp_zb_cfg_t zb_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_cfg);
    create_endpoints();
    esp_zb_core_action_handler_register(zb_action_handler);
}

void zigbee_task(void *arg) {
    zigbee_init();
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}
```

- [ ] **Step 3: Commit**

```bash
git add main/zigbee.h main/zigbee.c
git commit -m "feat: Zigbee stack — router, EP1 occupancy, EP2 dimmable light, NVS"
```

---

## Task 6: Main entry point

**Files:**
- Create: `main/main.c`

- [ ] **Step 1: Create main/main.c**

```c
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
```

- [ ] **Step 2: Commit**

```bash
git add main/main.c
git commit -m "feat: app_main — NVS init, shared state, task startup"
```

---

## Task 7: Build verification

- [ ] **Step 1: Run host unit tests**

```bash
cd test/host && make test
```
Expected:
```
test_led PASSED
test_occupancy PASSED
```

- [ ] **Step 2: Build firmware**

```bash
make build
```
Expected: ends with `Project build complete.`

If components are missing on first run, the IDF Component Manager will download them automatically — the build may take a few minutes. If it fails with a Zigbee API error, check the downloaded component version in `main/managed_components/` and adjust API calls to match.

- [ ] **Step 3: Flash to XIAO ESP32-C6**

Connect XIAO via USB-C. Verify it appears:
```bash
ls /dev/ttyACM*
```
Expected: `/dev/ttyACM0` (or similar)

Flash:
```bash
make flash PORT=/dev/ttyACM0
```

- [ ] **Step 4: Open serial monitor and verify boot sequence**

```bash
make monitor PORT=/dev/ttyACM0
```

Expected output:
```
I (xxx) main: all tasks started
I (xxx) wifi_sniffer: initialized, scanning channels 1–13
I (xxx) zigbee: Joined network on channel N
```

LED should: fast-blink (scanning) → slow-blink (joining) → steady/ZHA-controlled once paired.

- [ ] **Step 5: Pair with ZHA**

In Home Assistant: Settings → Devices & Services → ZHA → Add Device.
Put coordinator in pairing mode. The XIAO should appear as a new device with two entities:
- Occupancy sensor (EP1)
- Light (EP2, dimmable)

- [ ] **Step 6: Verify occupancy detection**

Bring a phone within 5–10 metres. Within 30 seconds (eval interval) the occupancy sensor should change to `Occupied` in ZHA. After 5 minutes without probe requests it should return to `Unoccupied`.

- [ ] **Step 7: Verify LED control from ZHA**

In ZHA, find the Light entity on the device. Toggle on/off and adjust brightness. LED should respond. Power-cycle the device — LED should restore to the last ZHA state.

- [ ] **Step 8: Commit any fixups**

```bash
git add -p
git commit -m "fix: integration corrections"
```

---

## Task 8: CI/CD + web flasher + README

**Files:**
- Create: `.github/workflows/build.yml`
- Create: `docs/manifest.json`
- Create: `docs/index.html`
- Create: `README.md`

- [ ] **Step 1: Create .github/workflows/build.yml**

```yaml
name: Build

on:
  push:
    branches: ["**"]
  release:
    types: [created]

jobs:
  build:
    runs-on: ubuntu-latest
    container:
      image: espressif/idf:v5.3

    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive

      - name: Build firmware
        run: idf.py build

      - name: Merge binaries into single flashable .bin
        if: github.event_name == 'release'
        run: |
          python $IDF_PATH/components/esptool_py/esptool/esptool.py \
            --chip esp32c6 merge_bin \
            -o build/firmware-merged.bin \
            --flash_mode dio \
            --flash_size 4MB \
            @build/flash_args

      - name: Attach merged binary to release
        if: github.event_name == 'release'
        uses: softprops/action-gh-release@v2
        with:
          files: build/firmware-merged.bin
```

- [ ] **Step 2: Create docs/manifest.json**

Replace `YOUR_GITHUB_USERNAME` and `YOUR_REPO_NAME` with actual values:

```json
{
  "name": "ESP32-C6 Zigbee Router + Presence Sensor",
  "version": "1.0.0",
  "new_install_prompt_erase": true,
  "builds": [
    {
      "chipFamily": "ESP32-C6",
      "parts": [
        {
          "path": "https://github.com/YOUR_GITHUB_USERNAME/YOUR_REPO_NAME/releases/latest/download/firmware-merged.bin",
          "offset": 0
        }
      ]
    }
  ]
}
```

- [ ] **Step 3: Create docs/index.html**

Replace `YOUR_GITHUB_USERNAME` and `YOUR_REPO_NAME`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ESP32-C6 Zigbee Router — Web Flasher</title>
  <script type="module"
    src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module">
  </script>
  <style>
    body { font-family: sans-serif; max-width: 480px; margin: 4rem auto; padding: 0 1rem; }
    h1   { font-size: 1.25rem; }
    p    { color: #555; font-size: 0.9rem; }
  </style>
</head>
<body>
  <h1>ESP32-C6 Zigbee Router + Presence Sensor</h1>
  <p>Connect your XIAO ESP32-C6 via USB-C, then click <strong>Install</strong>.</p>
  <p>Requires Chrome or Edge (WebSerial). The device will be erased before flashing.</p>
  <esp-web-install-button
    manifest="https://YOUR_GITHUB_USERNAME.github.io/YOUR_REPO_NAME/manifest.json">
  </esp-web-install-button>
  <p>Source: <a href="https://github.com/YOUR_GITHUB_USERNAME/YOUR_REPO_NAME">GitHub</a></p>
</body>
</html>
```

- [ ] **Step 4: Create README.md**

```markdown
# ESP32-C6 Zigbee Router + Room Presence Sensor

Firmware for the [Seeed XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/) that:

- **Extends your Zigbee mesh** as a ZHA router
- **Detects room occupancy** passively via WiFi probe request sniffing (no sensors needed)
- **Exposes the onboard LED** as a ZHA dimmable light (controllable from HA)

## Flash from browser (no tools required)

Visit the [web flasher](https://YOUR_GITHUB_USERNAME.github.io/YOUR_REPO_NAME/) in Chrome or Edge with your XIAO connected via USB-C.

## Flash with esptool (CLI)

```bash
pip install esptool
esptool.py --chip esp32c6 write_flash 0x0 firmware-merged.bin
```

Download `firmware-merged.bin` from [Releases](https://github.com/YOUR_GITHUB_USERNAME/YOUR_REPO_NAME/releases).

## Build from source

Requires Docker.

```bash
git clone https://github.com/YOUR_GITHUB_USERNAME/YOUR_REPO_NAME
cd YOUR_REPO_NAME
make build
make flash PORT=/dev/ttyACM0
make monitor PORT=/dev/ttyACM0
```

## Pairing with ZHA

1. Home Assistant → Settings → Devices & Services → ZHA → Add Device
2. Power the XIAO — LED fast-blinks (scanning) then slow-blinks (joining)
3. Device joins and appears with two entities: occupancy sensor + dimmable light

## Tuning constants

Edit and rebuild:

| Constant | File | Default | Meaning |
|---|---|---|---|
| `OCCUPANCY_TIMEOUT_SEC` | `main/wifi_sniffer.h` | `300` | Seconds of silence → unoccupied |
| `WIFI_CHANNEL_MIN` | `main/wifi_sniffer.h` | `1` | First channel to scan |
| `WIFI_CHANNEL_MAX` | `main/wifi_sniffer.h` | `13` | Last channel to scan |
| `WIFI_CHANNEL_DWELL_MS` | `main/wifi_sniffer.h` | `200` | ms per channel |
| `LED_DEFAULT_BRIGHTNESS` | `main/led.h` | `128` | Brightness on first join (0–254) |
```

- [ ] **Step 5: Enable GitHub Pages in repo settings**

In GitHub: Settings → Pages → Source: Deploy from branch `master`, folder `/docs`.

- [ ] **Step 6: Commit**

```bash
git add .github/workflows/build.yml docs/manifest.json docs/index.html README.md
git commit -m "feat: CI build pipeline, web flasher, README"
```

---

## Self-review notes

- All `esp_zb_*` calls from non-Zigbee tasks go through `zigbee_report_occupancy` which uses `esp_zb_lock_acquire/release` — thread safe.
- NVS reads in `zb_action_handler` use local copies read under mutex — no race on `g_app_state`.
- `led_set_mode(LED_MODE_ZHA)` is called only after successful steering; LED state machine is the single owner of `set_raw_duty`.
- `s_ever_seen` ensures unoccupied state at startup even if `s_last_activity == 0`.
- `YOUR_GITHUB_USERNAME` / `YOUR_REPO_NAME` appear in 3 files (manifest.json, index.html, README.md) — must be replaced before publishing.
