#include <zephyr/kernel.h>
#include "hw/l298n.h"
#include "hw/encoders.h"
#include "hw/servo.h"
#include "hw/adc.h"
#include "hw/blinkers.h"
#include "sys/drivetrain.h"
#include "sys/steering.h"
#include "sys/blinkers_control.h"
#include "sys/health.h"
#include "sys/uart_parser.h"
#include "core/log.h"

int main(void)
{
    DEBUG_PRINT("Initializing Hardware...\n");
    if (l298n_init() != 0) return -1;
    if (encoders_init() != 0) return -1;
    if (servo_init() != 0) return -1;
    if (adc_init() != 0) return -1;
    if (blinkers_init() != 0) return -1;
    
    DEBUG_PRINT("Initializing System...\n");
    if (uart_parser_init() != 0) return -1;
    
    // Explicitly start threads after all hardware is checked and initialized
    drivetrain_init();
    steering_init();
    blinkers_control_init();
    health_init();
    
    DEBUG_PRINT("Initialization Complete. RTOS threads taking over.\n");

    return 0;
}
