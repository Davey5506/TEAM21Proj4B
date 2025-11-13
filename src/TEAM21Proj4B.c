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
    TIM3_INIT();

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

    // Configure EXTI for button on PC13
    set_pin_mode(GPIOC, 13, INPUT); //PC13 as input for button
    set_pin_pull(GPIOC, 13, PULL_UP); //Enable pull-up resistor
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk; //Enable SYSCFG clock
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;//Map EXTI13 to PC13
    NVIC_EnableIRQ(EXTI15_10_IRQn); //Enable EXTI15_10 interrupt
    NVIC_SetPriority(EXTI15_10_IRQn, 10); //Set priority to 1
    EXTI->IMR |= EXTI_IMR_MR13; //Unmask EXTI13
    EXTI->RTSR |= EXTI_RTSR_TR13; //Rising trigger

    // Initialize ADC1
    init_adc(ADC1, 10); // Initialize ADC1 on channel 10 (PC0)
    init_adc_interrupt(ADC1, 2); // Enable ADC interrupt with priority 2

    servo_control_set(1, SERVO_NEUTRAL_PULSE_WIDTH); //Set left wheel to neutral
    servo_control_set(2, SERVO_NEUTRAL_PULSE_WIDTH); //Set right 
    stop=1;

    while(1){};
    return 0;
}