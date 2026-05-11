#pragma once
#include <stdbool.h>
#include <stdint.h>

#define OCCUPANCY_TIMEOUT_SEC  300
#define WIFI_CHANNEL_MIN         1
#define WIFI_CHANNEL_MAX        13
#define WIFI_CHANNEL_DWELL_MS  200
#define WIFI_EVAL_INTERVAL_MS  10000

bool wifi_sniffer_occupancy_check(bool ever_seen, int64_t now_us, int64_t last_us);
void wifi_sniffer_init(void);
void wifi_sniffer_channel_hop_task(void *arg);
void wifi_sniffer_eval_task(void *arg);
