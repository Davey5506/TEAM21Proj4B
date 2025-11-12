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





int main(void){
    return 0;
}