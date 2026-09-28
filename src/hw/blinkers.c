#include "hw/blinkers.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "core/log.h"

static const struct gpio_dt_spec led_fl = GPIO_DT_SPEC_GET(DT_ALIAS(led_fl), gpios);
static const struct gpio_dt_spec led_fr = GPIO_DT_SPEC_GET(DT_ALIAS(led_fr), gpios);
static const struct gpio_dt_spec led_rl = GPIO_DT_SPEC_GET(DT_ALIAS(led_rl), gpios);
static const struct gpio_dt_spec led_rr = GPIO_DT_SPEC_GET(DT_ALIAS(led_rr), gpios);

int blinkers_init(void)
{
    if (gpio_pin_configure_dt(&led_fl, GPIO_OUTPUT_INACTIVE) < 0 ||
        gpio_pin_configure_dt(&led_fr, GPIO_OUTPUT_INACTIVE) < 0 ||
        gpio_pin_configure_dt(&led_rl, GPIO_OUTPUT_INACTIVE) < 0 ||
        gpio_pin_configure_dt(&led_rr, GPIO_OUTPUT_INACTIVE) < 0) {
        DEBUG_PRINT("Error: Failed to configure turn signal LEDs\n");
        return -1;
    }
    return 0;
}

void blinkers_set_front_left(bool state) { gpio_pin_set_dt(&led_fl, state ? 1 : 0); }
void blinkers_set_front_right(bool state) { gpio_pin_set_dt(&led_fr, state ? 1 : 0); }
void blinkers_set_rear_left(bool state) { gpio_pin_set_dt(&led_rl, state ? 1 : 0); }
void blinkers_set_rear_right(bool state) { gpio_pin_set_dt(&led_rr, state ? 1 : 0); }

void blinkers_toggle_all(void)
{
    gpio_pin_toggle_dt(&led_fl);
    gpio_pin_toggle_dt(&led_fr);
    gpio_pin_toggle_dt(&led_rl);
    gpio_pin_toggle_dt(&led_rr);
}
