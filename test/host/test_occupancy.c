#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define OCCUPANCY_TIMEOUT_SEC 300

static bool wifi_sniffer_occupancy_check(bool ever_seen, int64_t now_us, int64_t last_us) {
    if (!ever_seen) return false;
    return (now_us - last_us) < ((int64_t)OCCUPANCY_TIMEOUT_SEC * 1000000LL);
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
