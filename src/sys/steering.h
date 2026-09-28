#ifndef SYS_STEERING_H
#define SYS_STEERING_H

#include <zephyr/kernel.h>
extern struct k_sem steering_sem;

void steering_init(void);

#endif // SYS_STEERING_H
