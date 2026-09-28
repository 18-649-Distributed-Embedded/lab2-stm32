#ifndef HW_SERVO_H
#define HW_SERVO_H

#include <stdint.h>

#define SERVO_MIN_PULSE_NS    1000000  // 1.0ms
#define SERVO_CENTER_PULSE_NS 1500000  // 1.5ms
#define SERVO_MAX_PULSE_NS    2000000  // 2.0ms

int servo_init(void);
void servo_set_pulse_ns(uint32_t pulse_ns);

#endif // HW_SERVO_H
