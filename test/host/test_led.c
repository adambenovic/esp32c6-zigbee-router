#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Active LOW: 255 = LED off, 0 = LED full on. duty = 255 - brightness */
static uint32_t led_brightness_to_duty(uint8_t brightness) {
    return (uint32_t)(255 - brightness);
}

int main(void) {
    assert(led_brightness_to_duty(0)   == 255); /* off */
    assert(led_brightness_to_duty(254) == 1);   /* max brightness */
    assert(led_brightness_to_duty(127) == 128); /* ~50% */
    printf("test_led PASSED\n");
    return 0;
}
