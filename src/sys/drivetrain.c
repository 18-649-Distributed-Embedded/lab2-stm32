#include "sys/drivetrain.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/irq.h>
#include <stm32f4xx.h>
#include "core/cmd.h"
#include "hw/l298n.h"
#include "hw/encoders.h"
#include "sys/uart_parser.h"

// Semaphore to wake the PID thread precisely at 1kHz from the hardware timer ISR
K_SEM_DEFINE(pid_sem, 0, 1);

volatile uint32_t pid_deadline_misses = 0;

static uint32_t left_last_ticks = 0;
static uint16_t right_last_ticks = 0;

// Variables to pass exact velocity from ISR to Math Thread
static volatile int32_t isr_left_vel = 0;
static volatile int16_t isr_right_vel = 0;

// HARDWARE TIMER ISR (TIM4)
static void tim4_isr(const void *arg)
{
    // Check and clear the update interrupt flag
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;
        
        // Read wheel encoder ticks
        uint32_t left_curr = encoders_get_left_ticks();
        uint16_t right_curr = encoders_get_right_ticks();
        
        // Calculate delta
        isr_left_vel = (int32_t)(left_curr - left_last_ticks);
        isr_right_vel = (int16_t)(right_curr - right_last_ticks);
        
        left_last_ticks = left_curr;
        right_last_ticks = right_curr;

        // DEADLINE MISS DETECTION & GRACEFUL HANDLING
        // If the semaphore is > 0, the math thread stalled. Instead of E-stopping,
        // we increment the miss counter and SKIP giving the semaphore.
        // This effectively "drops" the stalled tick, allowing the thread to instantly 
        // calculate using the freshest delta when it finally wakes up.
        if (k_sem_count_get(&pid_sem) > 0) {
            pid_deadline_misses++;
        } else {
            // Wake PID control loop thread
            k_sem_give(&pid_sem);
        }
    }
}

// PID MATH THREAD
#define DRIVETRAIN_THREAD_STACK_SIZE 1024
#define DRIVETRAIN_THREAD_PRIORITY   1 // High priority for control loop

static void drivetrain_thread_func(void *p1, void *p2, void *p3)
{
    while (1) {
        // Block until ISR triggers
        k_sem_take(&pid_sem, K_FOREVER);

        // Read target value
        uint32_t sp_val = atomic_get(&mailbox_drivetrain);
        uint8_t throttle = (uint8_t)((sp_val >> 16) & 0xFF);
        uint8_t brake_active = (uint8_t)(sp_val & 0xFF);

        // Check the atomic safety interlock bitmask
        if (atomic_get(&system_safety_flags) != 0 || brake_active > 0) {
            l298n_brake();
            continue;
        }

        // Copy wheel velocities
        int32_t left_vel = isr_left_vel;
        int16_t right_vel = isr_right_vel;

        // Scale throttle (0 to 255) to target velocity
        // Assuming 127 is center/idle, 0 is full reverse, 255 is full forward
        // (128 * 8 = 1024, which exactly matches our 10-bit L298N driver bounds!)
        int32_t target_vel = ((int32_t)throttle - 127) * 8;

        int32_t left_error = target_vel - left_vel;
        int32_t right_error = target_vel - right_vel;

        int32_t left_out = left_error * 1;
        int32_t right_out = right_error * 1;

        if (left_out > 1024) left_out = 1024;
        if (left_out < -1024) left_out = -1024;
        if (right_out > 1024) right_out = 1024;
        if (right_out < -1024) right_out = -1024;

        l298n_set_left(left_out);
        l298n_set_right(right_out);
    }
}

K_THREAD_STACK_DEFINE(drivetrain_stack, DRIVETRAIN_THREAD_STACK_SIZE);
struct k_thread drivetrain_thread_data;

void drivetrain_init(void)
{
    left_last_ticks = encoders_get_left_ticks();
    right_last_ticks = encoders_get_right_ticks();

    // Configure hardware timer (TIM4) for exactly 1kHz
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // APB1 timer clock on F401 is typically 84MHz.
    // PSC = 84-1 (1MHz clock), ARR = 1000-1 (1kHz interrupt)
    TIM4->PSC = 84 - 1;
    TIM4->ARR = 1000 - 1;
    TIM4->DIER |= TIM_DIER_UIE; // Enable update interrupt

    // Connect and enable the raw hardware IRQ in Zephyr
    IRQ_CONNECT(TIM4_IRQn, 1, tim4_isr, NULL, 0);
    irq_enable(TIM4_IRQn);

    // Start the hardware timer
    TIM4->CR1 |= TIM_CR1_CEN;

    // Start thread explicitly
    k_thread_create(&drivetrain_thread_data, drivetrain_stack,
                    K_THREAD_STACK_SIZEOF(drivetrain_stack),
                    drivetrain_thread_func,
                    NULL, NULL, NULL,
                    DRIVETRAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
}
