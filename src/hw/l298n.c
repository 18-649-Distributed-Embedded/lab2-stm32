#include "hw/l298n.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

static const struct gpio_dt_spec dir_r1 = GPIO_DT_SPEC_GET(DT_ALIAS(dir_r1), gpios);
static const struct gpio_dt_spec dir_r2 = GPIO_DT_SPEC_GET(DT_ALIAS(dir_r2), gpios);
static const struct gpio_dt_spec dir_l1 = GPIO_DT_SPEC_GET(DT_ALIAS(dir_l1), gpios);
static const struct gpio_dt_spec dir_l2 = GPIO_DT_SPEC_GET(DT_ALIAS(dir_l2), gpios);

static const struct pwm_dt_spec pwm_r = PWM_DT_SPEC_GET(DT_ALIAS(pwm_r));
static const struct pwm_dt_spec pwm_l = PWM_DT_SPEC_GET(DT_ALIAS(pwm_l));

int l298n_init(void)
{
    if (!gpio_is_ready_dt(&dir_r1) || gpio_pin_configure_dt(&dir_r1, GPIO_OUTPUT_INACTIVE) < 0 ||
        !gpio_is_ready_dt(&dir_r2) || gpio_pin_configure_dt(&dir_r2, GPIO_OUTPUT_INACTIVE) < 0 ||
        !gpio_is_ready_dt(&dir_l1) || gpio_pin_configure_dt(&dir_l1, GPIO_OUTPUT_INACTIVE) < 0 ||
        !gpio_is_ready_dt(&dir_l2) || gpio_pin_configure_dt(&dir_l2, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("Error: Failed to configure L298N direction GPIOs\n");
        return -1;
    }
    if (!pwm_is_ready_dt(&pwm_r)) {
        printk("Error: Right motor PWM (TIM3) is not ready\n");
        return -1;
    }
    if (!pwm_is_ready_dt(&pwm_l)) {
        printk("Error: Left motor PWM (TIM3) is not ready\n");
        return -1;
    }
    return 0;
}

void l298n_set_right(int16_t val)
{
    if (val == 0) {
        gpio_pin_set_dt(&dir_r1, 0);
        gpio_pin_set_dt(&dir_r2, 0);
        pwm_set_pulse_dt(&pwm_r, 0);
    } else if (val > 0) {
        gpio_pin_set_dt(&dir_r1, 1);
        gpio_pin_set_dt(&dir_r2, 0);
        uint32_t pulse = (pwm_r.period * (uint32_t)val) >> 10;
        pwm_set_pulse_dt(&pwm_r, pulse);
    } else {
        gpio_pin_set_dt(&dir_r1, 0);
        gpio_pin_set_dt(&dir_r2, 1);
        uint32_t pulse = (pwm_r.period * (uint32_t)(-val)) >> 10;
        pwm_set_pulse_dt(&pwm_r, pulse);
    }
}

void l298n_set_left(int16_t val)
{
    if (val == 0) {
        gpio_pin_set_dt(&dir_l1, 0);
        gpio_pin_set_dt(&dir_l2, 0);
        pwm_set_pulse_dt(&pwm_l, 0);
    } else if (val > 0) {
        gpio_pin_set_dt(&dir_l1, 1);
        gpio_pin_set_dt(&dir_l2, 0);
        uint32_t pulse = (pwm_l.period * (uint32_t)val) >> 10;
        pwm_set_pulse_dt(&pwm_l, pulse);
    } else {
        gpio_pin_set_dt(&dir_l1, 0);
        gpio_pin_set_dt(&dir_l2, 1);
        uint32_t pulse = (pwm_l.period * (uint32_t)(-val)) >> 10;
        pwm_set_pulse_dt(&pwm_l, pulse);
    }
}

void l298n_brake(void) {
    // Fast Motor Stop (Aggressive Braking) requires IN1=1, IN2=1, ENA=1
    // This shorts the motor coils, dissipating the motor's kinetic energy instantly.
    gpio_pin_set_dt(&dir_r1, 1);
    gpio_pin_set_dt(&dir_r2, 1);
    pwm_set_pulse_dt(&pwm_r, pwm_r.period); // 100% duty cycle

    gpio_pin_set_dt(&dir_l1, 1);
    gpio_pin_set_dt(&dir_l2, 1);
    pwm_set_pulse_dt(&pwm_l, pwm_l.period); // 100% duty cycle
}
