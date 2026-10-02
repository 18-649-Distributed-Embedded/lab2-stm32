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
static void pid_timer_isr(struct k_timer *timer_id) {
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

volatile int32_t dt_target_tps = 0;
volatile int32_t dt_filtered_tps_l = 0;
volatile int32_t dt_filtered_tps_r = 0;

static void drivetrain_thread_func(void *p1, void *p2, void *p3) {
    // PID State
    int32_t filtered_tps_l = 0;
    int32_t filtered_tps_r = 0;
    int32_t integral_l = 0;
    int32_t integral_r = 0;

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
            integral_l = 0;
            integral_r = 0;
            filtered_tps_l = 0;
            filtered_tps_r = 0;
            
            dt_target_tps = 0;
            dt_filtered_tps_l = 0;
            dt_filtered_tps_r = 0;
            continue;
        }

        // ==========================================
        // 1. Calculate Target Ticks Per Second (TPS)
        // ==========================================
        int32_t target_tps = ((int32_t)throttle - 127) * 57;

        // ==========================================
        // 2. Calculate Current Velocity (TPS)
        // ==========================================
        int32_t inst_tps_l = (int32_t)isr_left_vel * 1000;
        int32_t inst_tps_r = (int16_t)isr_right_vel * 1000;
        
        // Low-Pass Filter to smooth out 1kHz quantization noise
        filtered_tps_l = (filtered_tps_l * 3 + inst_tps_l) / 4;
        filtered_tps_r = (filtered_tps_r * 3 + inst_tps_r) / 4;

        // ==========================================
        // 3. PI Controller
        // ==========================================
        int32_t error_l = target_tps - filtered_tps_l;
        int32_t error_r = target_tps - filtered_tps_r;
        
        integral_l += error_l;
        integral_r += error_r;

        // Anti-Windup
        int32_t integral_max = 2048000; 
        if (integral_l > integral_max) integral_l = integral_max;
        if (integral_l < -integral_max) integral_l = -integral_max;
        if (integral_r > integral_max) integral_r = integral_max;
        if (integral_r < -integral_max) integral_r = -integral_max;

        // Gains (Kp = 0.5, Ki = 0.0005)
        int32_t p_term_l = error_l / 2;
        int32_t i_term_l = integral_l / 2000;
        int32_t out_l = p_term_l + i_term_l;

        int32_t p_term_r = error_r / 2;
        int32_t i_term_r = integral_r / 2000;
        int32_t out_r = p_term_r + i_term_r;

        // Bound outputs
        if (out_l > 1024) out_l = 1024;
        if (out_l < -1024) out_l = -1024;
        if (out_r > 1024) out_r = 1024;
        if (out_r < -1024) out_r = -1024;

        // Apply independent effort to both wheels
        l298n_set_left(out_l);
        l298n_set_right(out_r);

        // Expose state for slow health logging
        dt_target_tps = target_tps;
        dt_filtered_tps_l = filtered_tps_l;
        dt_filtered_tps_r = filtered_tps_r;
    }
}

K_THREAD_STACK_DEFINE(drivetrain_stack, DRIVETRAIN_THREAD_STACK_SIZE);
struct k_thread drivetrain_thread_data;

void drivetrain_init(void) {
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
