#include "hat.h"
#include <stdio.h>

#define PWM_FREQ_HZ 50 // Standard servo PWM frequency
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720

enum PIN_VALUE sensors[4] = {PIN_ERROR, PIN_ERROR, PIN_ERROR, PIN_ERROR};
volatile uint8_t stop_lines = 0;
volatile uint16_t sensor = 0;

void drive_servo(void){
    if(sensors[0] && sensors[1] && sensors[2] && sensors[3] && !stop_lines){
        stop_lines++;
    }else if(sensors[0] && sensors[1] && sensors[2] && sensors[3]){
        stop_lines = 0;
        TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
        TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
    }else if(sensors[1] && sensors[2]){
        TIM3->CCR3 = 1280;
        TIM3->CCR4 = 1720;
    }else if(sensors[0] && sensors[1]){
        TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
        TIM3->CCR4 = 1620;
    }else if(sensors[2] && sensors[3]){
        TIM3->CCR3 = 1380;
        TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
    }
}

void read_sensors(void){
    sensor = 0;
    for(int i = 3; i >= 0; i--){
        uint8_t val = read_pin(PMOD_C.PIN_PORTS[i], PMOD_C.PIN_NUMS[i]);
        sensors[i] = val;
    }
    if(sensors[0]) sensor += 1000;
    if(sensors[1]) sensor += 100;
    if(sensors[2]) sensor += 10;
    if(sensors[3]) sensor += 1;

    display_num(sensor, 0);
}

void TIM3_IRQHandler(void){
    if(TIM3->SR & TIM_SR_UIF){
        read_sensors();
        drive_servo();
        TIM3->SR &= ~TIM_SR_UIF;
    }
}

int main(void){
    // Initialize SSD
    init_ssd(10);
    display_num(0000, 4);
    
    // Setup TIM3
    init_gp_timer(TIM3, SYSTEM_FREQ, 16000, false);
    init_timer_IRQ(TIM3, 5); // 1ms interval for servo control
    TIM3->CCMR2 |= TIM_CCMR2_OC3CE | TIM_CCMR2_OC4CE; // PWM mode 1 for CH3 and CH4
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; // Enable CH3 and CH4 outputs
    TIM3->CCR3 = SERVO_NEUTRAL_PULSE_WIDTH;
    TIM3->CCR4 = SERVO_NEUTRAL_PULSE_WIDTH;
    TIM3->CR1 |= TIM_CR1_CEN;

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