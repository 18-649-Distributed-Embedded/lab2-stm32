#include "sys/health.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/atomic.h>
#include "core/cmd.h"
#include "core/log.h"
#include "sys/uart_parser.h"
#include "sys/drivetrain.h"
#include "hw/adc.h"

#define HEALTH_THREAD_STACK_SIZE 1024
#define HEALTH_THREAD_PRIORITY   5
#define HEALTH_LOOP_DELAY_MS     20
#define FAIL_SAFE_TIMEOUT_MS     150

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));

// Initialize safety interlock with WATCHDOG set (fail-safe by default until parser talks)
atomic_t system_safety_flags = ATOMIC_INIT(BIT(SAFETY_FLAG_WATCHDOG_TIMEOUT));

static void send_status_frame(void) {
    // Status frame: [0xBB][SafetyFlags][L_Curr_H][L_Curr_L][R_Curr_H][R_Curr_L][S_Curr_H][S_Curr_L][0x66]
    uint8_t frame[9];
    frame[0] = 0xBB;
    
    // We send down the raw safety flags so the proxy knows WHY the car is stopped
    frame[1] = (uint8_t)(atomic_get(&system_safety_flags) & 0xFF);

    int32_t l_curr = adc_get_left_current_mv();
    int32_t r_curr = adc_get_right_current_mv();
    int32_t s_curr = adc_get_servo_current_mv();

    frame[2] = (l_curr >> 8) & 0xFF;
    frame[3] = l_curr & 0xFF;
    frame[4] = (r_curr >> 8) & 0xFF;
    frame[5] = r_curr & 0xFF;
    frame[6] = (s_curr >> 8) & 0xFF;
    frame[7] = s_curr & 0xFF;
    frame[8] = 0x66;

    for (int i = 0; i < sizeof(frame); i++) {
        uart_poll_out(uart_dev, frame[i]);
    }
}

static void health_thread_func(void *p1, void *p2, void *p3) {
    bool last_was_failsafe = true;
    uint32_t last_deadline_misses = 0;

    while (1) {
        // Watchdog check
        uint32_t current_time = k_uptime_get_32();
        if ((current_time - last_cmd_time) > FAIL_SAFE_TIMEOUT_MS) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_WATCHDOG_TIMEOUT);
        } else {
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_WATCHDOG_TIMEOUT);
        }

        bool currently_failsafe = (atomic_get(&system_safety_flags) != 0);
        
        if (currently_failsafe && !last_was_failsafe) {
            DEBUG_PRINT("Safety Interlock Tripped! Flags: %d\n", (int)atomic_get(&system_safety_flags));
            last_was_failsafe = true;
        } else if (!currently_failsafe && last_was_failsafe) {
            DEBUG_PRINT("Link restored and Safety Clear: transitioning to NORMAL\n");
            last_was_failsafe = false;
        }

        // Deadline miss logging
        if (pid_deadline_misses > last_deadline_misses) {
            DEBUG_PRINT("WARNING: PID Math Thread missed deadline! Total misses: %u\n", pid_deadline_misses);
            last_deadline_misses = pid_deadline_misses;
        }

        send_status_frame();

        k_msleep(HEALTH_LOOP_DELAY_MS);
    }
}

K_THREAD_STACK_DEFINE(health_stack, HEALTH_THREAD_STACK_SIZE);
struct k_thread health_thread_data;

void health_init(void) {
    k_thread_create(&health_thread_data, health_stack,
                    K_THREAD_STACK_SIZEOF(health_stack),
                    health_thread_func,
                    NULL, NULL, NULL,
                    HEALTH_THREAD_PRIORITY, 0, K_NO_WAIT);
}
