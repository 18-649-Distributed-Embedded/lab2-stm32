#ifndef HW_L298N_H
#define HW_L298N_H

#include <stdint.h>

int l298n_init(void);
void l298n_set_right(int16_t val);
void l298n_set_left(int16_t val);
void l298n_brake(void);

#endif // HW_L298N_H
