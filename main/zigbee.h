#pragma once
#include <stdbool.h>

void zigbee_init(void);
void zigbee_task(void *arg);
void zigbee_report_occupancy(bool occupied);
