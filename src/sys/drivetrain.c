#include "sys/drivetrain.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "core/cmd.h"
#include "core/log.h"
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

// HARDWARE TIMER ISR (Via Zephyr k_timer to avoid Clock-Gating conflicts)
static void pid_timer_isr(struct k_timer *timer_id)
{
    // Read wheel encoder ticks
    uint32_t left_curr = encoders_get_left_ticks();
    uint16_t right_curr = encoders_get_right_ticks();
    
    // Calculate delta
    isr_left_vel = (int32_t)(left_curr - left_last_ticks);
    isr_right_vel = (int16_t)(right_curr - right_last_ticks);
    
    left_last_ticks = left_curr;
    right_last_ticks = right_curr;

    // DEADLINE MISS DETECTION & GRACEFUL HANDLING
    if (k_sem_count_get(&pid_sem) > 0) {
        pid_deadline_misses++;
    } else {
        k_sem_give(&pid_sem);
    }
}

K_TIMER_DEFINE(pid_timer, pid_timer_isr, NULL);

// PID MATH THREAD
#define DRIVETRAIN_THREAD_STACK_SIZE 1024
#define DRIVETRAIN_THREAD_PRIORITY   1 // High priority for control loop

static void drivetrain_thread_func(void *p1, void *p2, void *p3)
{
    int debug_counter = 0;
    
    // PID State
    int32_t filtered_tps = 0;
    int32_t integral = 0;

    while (1) {
        // Block until ISR triggers (1kHz loop)
        k_sem_take(&pid_sem, K_FOREVER);

        // Read target value
        uint32_t sp_val = atomic_get(&mailbox_drivetrain);
        uint8_t throttle = (uint8_t)((sp_val >> 16) & 0xFF);
        uint8_t brake_active = (uint8_t)(sp_val & 0xFF);

        // Safety interlock / Brake
        if (atomic_get(&system_safety_flags) != 0 || brake_active > 0) {
            l298n_brake();
            integral = 0;       // Reset integral on brake to prevent windup
            filtered_tps = 0;   // Reset filter
            
            if (++debug_counter >= 1000) {
                DEBUG_PRINT("[Drivetrain] BRAKED/ESTOP. Throttle: %d, BrakeActive: %d, Flags: %d\n", throttle, brake_active, (int)atomic_get(&system_safety_flags));
                debug_counter = 0;
            }
            continue;
        }

        // ==========================================
        // 1. Calculate Target Ticks Per Second (TPS)
        // ==========================================
        // Max Throttle Delta = 127. 
        // Max Motor RPM ~330 RPM = 5.5 rev/s. 
        // 30:1 Gearbox * 11 PPR * 4 edges = 1320 ticks/wheel_rev.
        // Max TPS = 5.5 * 1320 = ~7260 ticks/s.
        // Scale Factor = 7260 / 127 ≈ 57
        int32_t target_tps = ((int32_t)throttle - 127) * 57;

        // ==========================================
        // 2. Calculate Current Velocity (TPS)
        // ==========================================
        // isr_right_vel is delta over 1ms. Multiply by 1000 for TPS.
        // Note: The left encoder is broken, so we solely rely on right_vel.
        int32_t instantaneous_tps = (int16_t)isr_right_vel * 1000;
        
        // Low-Pass Filter to smooth out 1kHz quantization noise
        filtered_tps = (filtered_tps * 3 + instantaneous_tps) / 4;

        // ==========================================
        // 3. PI Controller
        // ==========================================
        int32_t error = target_tps - filtered_tps;
        
        integral += error;

        // Anti-Windup (Limit integral to ~100% duty cycle equivalent)
        // If Ki divisor is 2000, 2000 * 1024 = 2,048,000 max integral sum.
        int32_t integral_max = 2048000; 
        if (integral > integral_max) integral = integral_max;
        if (integral < -integral_max) integral = -integral_max;

        // Gains (Tuned for 1kHz discrete-time loop)
        // Kp = 0.5 (Integer div 2), Ki = 0.0005 (Integer div 2000)
        int32_t p_term = error / 2;
        int32_t i_term = integral / 2000;
        
        int32_t out = p_term + i_term;

        // Bound to L298N limits (-1024 to 1024)
        if (out > 1024) out = 1024;
        if (out < -1024) out = -1024;

        // Apply identical effort to both wheels (since Left encoder is broken)
        l298n_set_right(out);
        l298n_set_left(out);

        // Debug logging (1Hz)
        if (++debug_counter >= 1000) {
            // Note: If the motor runs away (instant max speed), it means the 
            // encoder phase A/B wires are swapped relative to motor polarity!
            DEBUG_PRINT("[PID] Tgt_TPS: %d | Cur_TPS: %d | Err: %d | Out: %d\n", 
                        target_tps, filtered_tps, error, out);
            debug_counter = 0;
        }
    }
}

K_THREAD_STACK_DEFINE(drivetrain_stack, DRIVETRAIN_THREAD_STACK_SIZE);
struct k_thread drivetrain_thread_data;

void drivetrain_init(void)
{
    left_last_ticks = encoders_get_left_ticks();
    right_last_ticks = encoders_get_right_ticks();

    // Start the Zephyr hardware-backed timer
    // Since CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 (10kHz) in prj.conf, 
    // a 1ms duration equates to exactly 10 hardware ticks.
    k_timer_start(&pid_timer, K_MSEC(1), K_MSEC(1));

    // Start thread explicitly
    k_thread_create(&drivetrain_thread_data, drivetrain_stack,
                    K_THREAD_STACK_SIZEOF(drivetrain_stack),
                    drivetrain_thread_func,
                    NULL, NULL, NULL,
                    DRIVETRAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
}
