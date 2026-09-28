#ifndef SYS_BLINKERS_CONTROL_H
#define SYS_BLINKERS_CONTROL_H

#include <zephyr/kernel.h>
extern struct k_sem blinkers_sem;

void blinkers_control_init(void);

#endif // SYS_BLINKERS_CONTROL_H
