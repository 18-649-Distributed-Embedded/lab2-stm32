#include "hw/servo.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include "core/log.h"

static const struct pwm_dt_spec pwm_servo = PWM_DT_SPEC_GET(DT_ALIAS(pwm_servo));

int servo_init(void)
{
    if (!pwm_is_ready_dt(&pwm_servo)) {
        DEBUG_PRINT("Error: Servo PWM (TIM4) is not ready\n");
        return -1;
    }
    // Center it initially
    pwm_set_pulse_dt(&pwm_servo, SERVO_CENTER_PULSE_NS);
    return 0;
}

void servo_set_pulse_ns(uint32_t pulse_ns)
{
    if (pulse_ns < SERVO_MIN_PULSE_NS) {
        pulse_ns = SERVO_MIN_PULSE_NS;
    }
    if (pulse_ns > SERVO_MAX_PULSE_NS) {
        pulse_ns = SERVO_MAX_PULSE_NS;
    }
    pwm_set_pulse_dt(&pwm_servo, pulse_ns);
}
