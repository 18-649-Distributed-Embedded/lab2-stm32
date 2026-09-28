#include "hw/encoders.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "core/log.h"

// STM32 registers are accessible when including this
#include <stm32f4xx.h>

int encoders_init(void)
{
    // ==========================================
    // HARDWARE OVERRIDE: FORCE PIN MULTIPLEXER
    // ==========================================

    // 1. Enable GPIOA Clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // 2. Force PA1 and PA15 into Alternate Function Mode (Binary 10)
    // This rips PA1 out of Analog mode and PA15 out of JTAG mode
    GPIOA->MODER &= ~(GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER15_Msk);
    GPIOA->MODER |= (GPIO_MODER_MODER1_1 | GPIO_MODER_MODER15_1);

    // 3. Connect PA1 to AF1 (TIM2) in the Low Register
    GPIOA->AFR[0] &= ~(0xFU << (1 * 4));
    GPIOA->AFR[0] |= (0x1U << (1 * 4));

    // 4. Connect PA15 to AF1 (TIM2) in the High Register
    GPIOA->AFR[1] &= ~(0xFU << ((15 - 8) * 4));
    GPIOA->AFR[1] |= (0x1U << ((15 - 8) * 4));

    // 5. Force internal Pull-Ups on PA1 and PA15 (Binary 01)
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD1_Msk | GPIO_PUPDR_PUPD15_Msk);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD1_0 | GPIO_PUPDR_PUPD15_0);

    // ==========================================
    // HARDWARE OVERRIDE: FORCE TIM2 ENCODER MODE
    // ==========================================
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Map inputs to TI1 and TI2, set Encoder Mode 3
    TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM2->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);

    TIM2->ARR = 0xFFFFFFFF; // 32-bit limit
    TIM2->CNT = 0;          // Zero the counter
    TIM2->CR1 |= TIM_CR1_CEN; // Enable counter

    // 1. Ensure Peripheral Clocks are ON
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // 2. Map IC1 to TI1 and IC2 to TI2 (CC1S=01, CC2S=01)
    TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);

    // 3. Set Encoder Mode 3 (Count on both edges of TI1 and TI2)
    TIM1->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
    TIM2->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);

    // 4. Force Limits
    TIM1->ARR = 0xFFFF;       
    TIM2->ARR = 0xFFFFFFFF;   

    // (Optional) Flip Right Encoder Polarity
    TIM1->CCER |= TIM_CCER_CC1P; 

    // Zero the counters
    TIM1->CNT = 0;
    TIM2->CNT = 0;

    // 5. Enable the Counters
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;

    return 0;
}

uint32_t encoders_get_left_ticks(void)
{
    return TIM2->CNT;
}

uint16_t encoders_get_right_ticks(void)
{
    return TIM1->CNT;
}
