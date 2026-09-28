#ifndef HW_ADC_H
#define HW_ADC_H

#include <stdint.h>

int adc_init(void);
int32_t adc_get_left_current_mv(void);
int32_t adc_get_right_current_mv(void);
int32_t adc_get_servo_current_mv(void);

#endif // HW_ADC_H
