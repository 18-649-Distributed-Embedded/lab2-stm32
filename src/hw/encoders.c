#include "hw/encoders.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "core/log.h"
#include <stm32f4xx.h>

int encoders_init(void) {
    const struct device *qdec1 = DEVICE_DT_GET(DT_NODELABEL(qdec1));
    const struct device *qdec2 = DEVICE_DT_GET(DT_NODELABEL(qdec2));

    if (!device_is_ready(qdec1)) {
        DEBUG_PRINT("Error: QDEC1 (Right Encoder) not ready\n");
        return -1;
    }
    if (!device_is_ready(qdec2)) {
        DEBUG_PRINT("Error: QDEC2 (Left Encoder) not ready\n");
        return -1;
    }

    // Flip right encoder polarity
    TIM1->CCER |= TIM_CCER_CC1P;

    // Zero the counters
    TIM1->CNT = 0;
    TIM2->CNT = 0;

    return 0;
}

uint32_t encoders_get_left_ticks(void) {
    return TIM2->CNT;
}

uint16_t encoders_get_right_ticks(void) {
    return TIM1->CNT;
}
