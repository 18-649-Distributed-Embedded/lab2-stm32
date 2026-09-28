#ifndef HW_BLINKERS_H
#define HW_BLINKERS_H

#include <stdint.h>
#include <stdbool.h>

int blinkers_init(void);
void blinkers_set_front_left(bool state);
void blinkers_set_front_right(bool state);
void blinkers_set_rear_left(bool state);
void blinkers_set_rear_right(bool state);
void blinkers_toggle_all(void);

#endif // HW_BLINKERS_H
