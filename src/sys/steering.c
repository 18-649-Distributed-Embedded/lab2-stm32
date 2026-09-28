#include "sys/steering.h"
#include <zephyr/kernel.h>
#include "core/cmd.h"
#include "hw/servo.h"

#define STEERING_THREAD_STACK_SIZE 1024
#define STEERING_THREAD_PRIORITY   3
#define STEERING_LOOP_DELAY_MS     50

#include "sys/uart_parser.h"

K_SEM_DEFINE(steering_sem, 0, 1);

static void steering_thread_func(void *p1, void *p2, void *p3) {
    while (1) {
        // Event-driven: Wake instantly when a new command arrives, OR timeout every 50ms 
        // to guarantee a maximum latency of 50ms even if commands are sparse.
        k_sem_take(&steering_sem, K_MSEC(50));

        // Read lock-free mailbox
        uint32_t sp_val = atomic_get(&mailbox_steering);
        int8_t steer_angle = (int8_t)(sp_val & 0xFF);

        if (atomic_get(&system_safety_flags) != 0) {
            // Keep current angle or center
        } else {
            // Map -128..127 to 1.0ms..2.0ms
            int32_t range = SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS;
            int32_t offset = (steer_angle + 128) * range / 255;
            servo_set_pulse_ns(SERVO_MIN_PULSE_NS + offset);
        }
    }
}

K_THREAD_STACK_DEFINE(steering_stack, STEERING_THREAD_STACK_SIZE);
struct k_thread steering_thread_data;

void steering_init(void) {
    k_thread_create(&steering_thread_data, steering_stack,
                    K_THREAD_STACK_SIZEOF(steering_stack),
                    steering_thread_func,
                    NULL, NULL, NULL,
                    STEERING_THREAD_PRIORITY, 0, K_NO_WAIT);
}
