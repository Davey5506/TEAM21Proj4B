#include "hat.h"
#include <stdio.h>

#define PWM_FREQ_HZ 50 // Standard servo PWM frequency
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720

void SysTick_Handler(void){
}

int main(void){
    // Initialize USART2 for serial communication at 115200 baud
    init_usart(115200);
    init_sys_tick(SYSTEM_FREQ); // 1s tick
    // Initialize SSD
    init_ssd(10);
    display_num(0, 4);
    
    // Setup TIM3
    init_gp_timer(TIM3, SYSTEM_FREQ, 10000, false);
    TIM3->CCMR2 |= TIM_CCMR2_OC3CE | TIM_CCMR2_OC4CE; // PWM mode 1 for CH3 and CH4
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable CH3 and CH4 outputs
    TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
    TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;

    // Set up servos
    SERVO_t left_wheel = {
        .SERVO_PIN_PORT = GPIOC,
        .SERVO_PWM_PIN = 8,
        .SERVO_FEEDBACK_PIN = 16 // Does not exist, placeholder
    };
    SERVO_t right_wheel = {
        .SERVO_PIN_PORT = GPIOC,
        .SERVO_PWM_PIN = 9,
        .SERVO_FEEDBACK_PIN = 16 // Does not exist, placeholder
    };
    init_servo(&left_wheel);
    init_servo(&right_wheel);

    // Setup PMOD C for sensors
    init_pmod(PMOD_C);
    for(int i = 0; i < 4; i++){
        set_pin_mode(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i], INPUT);
        set_pin_pull(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i], PULL_DOWN);
    }

    // Configure EXTI for PMOD C pins (PC0-PC3)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC|SYSCFG_EXTICR1_EXTI1_PC|SYSCFG_EXTICR1_EXTI2_PC|SYSCFG_EXTICR1_EXTI3_PC;

    // Enable EXTI0-3 interrupts
    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1 | EXTI_IMR_MR2 | EXTI_IMR_MR3;
    EXTI->RTSR |= EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR2 | EXTI_RTSR_TR3;
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 1); 

    while(1){};
    return 0;
}