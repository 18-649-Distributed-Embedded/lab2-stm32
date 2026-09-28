#include "sys/uart_parser.h"
#include <zephyr/drivers/uart.h>
#include "core/log.h"
#include "core/cmd.h"
#include "sys/steering.h"
#include "sys/blinkers_control.h"

// 127 is the center idle value for throttle. If we initialize to 0, 
// the car will jerk backwards if failsafe clears before the first motion packet.
atomic_t mailbox_drivetrain = ATOMIC_INIT(127 << 16);
atomic_t mailbox_steering = ATOMIC_INIT(0);
atomic_t mailbox_blinkers = ATOMIC_INIT(0);

// UART Device
static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));

#define FRAME_SYNC_BYTE 0xAA
#define FRAME_END_BYTE  0x55
#define MAX_PAYLOAD_LEN 8

typedef enum {
    STATE_WAIT_SYNC,
    STATE_READ_ID_L,
    STATE_READ_ID_H,
    STATE_READ_DLC,
    STATE_READ_PAYLOAD,
    STATE_READ_CHECKSUM,
    STATE_WAIT_END
} parser_state_t;

static parser_state_t parser_state = STATE_WAIT_SYNC;
static uint16_t current_id = 0;
static uint8_t current_dlc = 0;
static uint8_t payload[MAX_PAYLOAD_LEN];
static uint8_t payload_idx = 0;
static uint8_t expected_checksum = 0;

volatile uint32_t rx_byte_count = 0;
volatile uint32_t last_cmd_time = 0;
volatile uint32_t parse_success_count = 0;

static void process_can_message(void)
{
    if (current_id == 0x020 && current_dlc >= 2) {
        // Msg_Heartbeat_Cockpit
        if (payload[1] == 0xFF) {
            atomic_set_bit(&system_safety_flags, SAFETY_FLAG_ESTOP);
        } else {
            atomic_clear_bit(&system_safety_flags, SAFETY_FLAG_ESTOP);
        }
    }
    else if (current_id == 0x010 && current_dlc >= 2) {
        // Cmd_Brake
        uint8_t brake_active = payload[1];
        uint32_t dt_val = atomic_get(&mailbox_drivetrain);
        dt_val = (dt_val & 0xFFFF0000) | brake_active;
        atomic_set(&mailbox_drivetrain, dt_val);
    }
    else if (current_id == 0x100 && current_dlc >= 4) {
        // Cmd_Motion
        uint8_t throttle = payload[1]; // 0-255
        int8_t steer = (int8_t)payload[2]; // -128 to 127
        uint8_t turn_req = payload[3];

        uint32_t dt_val = atomic_get(&mailbox_drivetrain);
        dt_val = (dt_val & 0x0000FFFF) | ((uint32_t)throttle << 16);
        atomic_set(&mailbox_drivetrain, dt_val);

        atomic_set(&mailbox_steering, (uint32_t)(uint8_t)steer);
        
        uint32_t bk_val = atomic_get(&mailbox_blinkers);
        bk_val = (bk_val & 0xFFFFFF00) | turn_req;
        atomic_set(&mailbox_blinkers, bk_val);

        k_sem_give(&steering_sem);
    }
    else if (current_id == 0x200 && current_dlc >= 2) {
        // Cmd_Turn_Sync
        uint8_t sync_state = payload[1];
        uint32_t bk_val = atomic_get(&mailbox_blinkers);
        bk_val = (bk_val & 0xFFFF00FF) | ((uint32_t)sync_state << 8);
        atomic_set(&mailbox_blinkers, bk_val);
        k_sem_give(&blinkers_sem);
    }
    else if (current_id == 0x011 && current_dlc >= 2) {
        // Cmd_Hazard_Sync
        uint8_t hz_state = payload[1];
        uint32_t bk_val = atomic_get(&mailbox_blinkers);
        bk_val = (bk_val & 0xFF00FFFF) | ((uint32_t)hz_state << 16);
        atomic_set(&mailbox_blinkers, bk_val);
        k_sem_give(&blinkers_sem);
    }
    
    last_cmd_time = k_uptime_get_32();
    parse_success_count++;
}

static void uart_rx_isr(const struct device *dev, void *user_data)
{
    uint8_t c;
    uart_irq_update(dev);

    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) == 1) {
            rx_byte_count++;
            switch (parser_state) {
                case STATE_WAIT_SYNC:
                    if (c == FRAME_SYNC_BYTE) {
                        parser_state = STATE_READ_ID_L;
                        expected_checksum = 0;
                    }
                    break;
                case STATE_READ_ID_L:
                    current_id = c;
                    expected_checksum += c;
                    parser_state = STATE_READ_ID_H;
                    break;
                case STATE_READ_ID_H:
                    current_id |= (c << 8);
                    expected_checksum += c;
                    parser_state = STATE_READ_DLC;
                    break;
                case STATE_READ_DLC:
                    current_dlc = c > MAX_PAYLOAD_LEN ? MAX_PAYLOAD_LEN : c;
                    expected_checksum += c;
                    payload_idx = 0;
                    if (current_dlc > 0) parser_state = STATE_READ_PAYLOAD;
                    else parser_state = STATE_READ_CHECKSUM;
                    break;
                case STATE_READ_PAYLOAD:
                    payload[payload_idx++] = c;
                    expected_checksum += c;
                    if (payload_idx >= current_dlc) parser_state = STATE_READ_CHECKSUM;
                    break;
                case STATE_READ_CHECKSUM:
                    if (c == expected_checksum) parser_state = STATE_WAIT_END;
                    else parser_state = STATE_WAIT_SYNC;
                    break;
                case STATE_WAIT_END:
                    if (c == FRAME_END_BYTE) process_can_message();
                    parser_state = STATE_WAIT_SYNC;
                    break;
            }
        }
    }
}

int uart_parser_init(void)
{
    if (!device_is_ready(uart_dev)) {
        DEBUG_PRINT("Error: UART device is not ready\n");
        return -1;
    }

    uart_irq_callback_set(uart_dev, uart_rx_isr);
    uart_irq_rx_enable(uart_dev);

    DEBUG_PRINT("UART parser initialized on usart2 (Hardware ISR Mode)\n");
    return 0;
}
