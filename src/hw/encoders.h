#ifndef HW_ENCODERS_H
#define HW_ENCODERS_H

#include <stdint.h>

int encoders_init(void);
uint32_t encoders_get_left_ticks(void);
uint16_t encoders_get_right_ticks(void);

#endif // HW_ENCODERS_H
