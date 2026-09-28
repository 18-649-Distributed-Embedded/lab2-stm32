#ifndef SYS_UART_PARSER_H
#define SYS_UART_PARSER_H

#include "../core/cmd.h"
#include <zephyr/kernel.h>

#include <zephyr/sys/atomic.h>

extern atomic_t mailbox_drivetrain;
extern atomic_t mailbox_steering;
extern atomic_t mailbox_blinkers;
extern volatile uint32_t last_cmd_time;
extern volatile uint32_t rx_byte_count;
extern volatile uint32_t parse_success_count;

int uart_parser_init(void);

#endif // SYS_UART_PARSER_H
