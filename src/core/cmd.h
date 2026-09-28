#ifndef CORE_CMD_H
#define CORE_CMD_H

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/sys/atomic.h>

// Safety Interlock Bitmask Flags
#define SAFETY_FLAG_WATCHDOG_TIMEOUT 0
#define SAFETY_FLAG_ESTOP            1

extern atomic_t system_safety_flags;

// Blinker state flags
#define BLINKER_OFF    0x00
#define BLINKER_LEFT   0x01
#define BLINKER_RIGHT  0x02
#define BLINKER_HAZARD 0x03



#endif // CORE_CMD_H
