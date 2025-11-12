#include "hat.h"
#include <stdio.h>

#define ADC_CHANNEL 10 // PC0
#define PWM_FREQ_HZ 50 // Standard servo PWM frequency
#define TIM3_FREQ_HZ 1000000 // 1MHz timer clock for 1us resolution
#define TIM3_ARR ((TIM3_FREQ_HZ / PWM_FREQ_HZ) - 1) // Auto-reload value for 50Hz (20000 - 1 = 19999)
#define FEEDBACK_PERIOD_US 1100
#define FEEDBACK_MAX_DUTY_CYCLE 0.971f // Corresponds to 1070us pulse width
#define FEEDBACK_MIN_DUTY_CYCLE 0.029f // Corresponds to 30us pulse width
#define SERVO_NEUTRAL_PULSE_WIDTH 1500 // 1.5ms pulse width for neutral position

#define CW_MAX_PULSE 1480 
#define CW_MIN_PULSE 1280
#define CCW_MIN_PULSE 1520
#define CCW_MAX_PULSE 1720

SERVO_t wheel ={
    .SERVO_PIN_PORT = GPIOC,
    .SERVO_PWM_PIN = 6,
    .SERVO_FEEDBACK_PIN = 7
};

volatile uint32_t pulse_width = 0; // Pulse width in microseconds
volatile uint8_t direction = 0; // 0 for CW, 1 for CCW
volatile uint8_t stop = 0;
volatile uint32_t feedback_timings[2];
volatile uint32_t pulse_start_times[2];
volatile float angular_positions[2]; //Angular position in degrees
volatile float rpm = 0; //Rotational Speed in RPM
volatile uint16_t adc_value = 0; //ADC Value
volatile uint8_t value_ready = 0; //Flag to indicate new ADC value is ready
volatile uint8_t sensor_values[4] = {0, 0, 0, 0};//Array for sensor values

void SysTick_Handler(void){
    // Start ADC conversion, but don't wait for it in the ISR
    adc_swtstart(ADC1);
}

uint32_t lvl_to_pulse(uint16_t lvl, uint8_t direction){
    if(direction == 0){ //Clockwise
        return umap(lvl, 0,4095, CW_MIN_PULSE, CW_MAX_PULSE); 
    } else { //Counter-Clockwise
        return umap(lvl, 0, 4095, CCW_MIN_PULSE, CCW_MAX_PULSE);
    }
}

void TIM3_IRQHandler(void){ //meaures pulse width and period of feedback signal
    if (TIM3->SR & TIM_SR_UIF){ 
        TIM3->SR &= ~TIM_SR_UIF; 
    }
}

void EXTI15_10_IRQHandler(void){
    if(EXTI->PR & EXTI_PR_PR13){
        EXTI->PR |= EXTI_PR_PR13; //Clear pending register
        if(!stop){
            stop = !stop;
            pulse_width = SERVO_NEUTRAL_PULSE_WIDTH; //Stop the wheel
        }else{
            stop = !stop;
            direction = !direction;
        }
    }
}


int main(void){
    return 0;
}