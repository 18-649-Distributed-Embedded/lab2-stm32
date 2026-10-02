#include "sys/blinkers_control.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "hw/blinkers.h"
#include "core/cmd.h"
#include "sys/uart_parser.h"

#define BLINKERS_THREAD_STACK_SIZE 1024
#define BLINKERS_THREAD_PRIORITY   6

K_SEM_DEFINE(blinkers_sem, 0, 1);
K_THREAD_STACK_DEFINE(blinkers_stack, BLINKERS_THREAD_STACK_SIZE);
struct k_thread blinkers_thread_data;

static void blinkers_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        // Wait for a new Turn_Sync or Hazard_Sync command from the upstream proxy.
        k_sem_take(&blinkers_sem, K_MSEC(50));

        uint32_t bk_val = atomic_get(&mailbox_blinkers);
        uint8_t turn_req = bk_val & 0xFF;
        uint8_t turn_sync = (bk_val >> 8) & 0xFF;
        uint8_t hazard_sync = (bk_val >> 16) & 0xFF;

        bool is_failsafe = (atomic_get(&system_safety_flags) != 0);

        // Default all off
        blinkers_set_front_left(false);
        blinkers_set_front_right(false);
        blinkers_set_rear_left(false);
        blinkers_set_rear_right(false);

        if (is_failsafe) {
            // Pi proxy might be dead, so rely on local clock 
            uint32_t time_ms = k_uptime_get_32();
            bool on = (time_ms % 500) < 250;
            blinkers_set_front_left(on);
            blinkers_set_front_right(on);
            blinkers_set_rear_left(on);
            blinkers_set_rear_right(on);
        } else if (turn_req == BLINKER_HAZARD) {
            // Commanded hazard via Pi clock
            bool on = hazard_sync;
            blinkers_set_front_left(on);
            blinkers_set_front_right(on);
            blinkers_set_rear_left(on);
            blinkers_set_rear_right(on);
        } else {
            // Normal operation driven by Turn_Sync (1Hz clock from Pi)
            if (turn_req == BLINKER_LEFT) {
                blinkers_set_front_left(turn_sync);
                blinkers_set_rear_left(turn_sync);
            } else if (turn_req == BLINKER_RIGHT) {
                blinkers_set_front_right(turn_sync);
                blinkers_set_rear_right(turn_sync);
            }
        }
    }
}

void blinkers_control_init(void)
{
    k_thread_create(&blinkers_thread_data, blinkers_stack,
                    K_THREAD_STACK_SIZEOF(blinkers_stack),
                    blinkers_thread_func,
                    NULL, NULL, NULL,
                    BLINKERS_THREAD_PRIORITY, 0, K_NO_WAIT);
}
