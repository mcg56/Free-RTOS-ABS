/*
 * pwm.h
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <stdbool.h>


// PWM configuration
#define PWM_START_RATE_HZ  250
#define PWM_RATE_STEP_HZ   50
#define PWM_RATE_MIN_HZ    50
#define PWM_RATE_MAX_HZ    400
#define PWM_FIXED_DUTY     40
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_16
#define PWM_DIVIDER        16

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE	     PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM	 SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5
#define WHEEL_FIXED_DUTY     50

/**
 * @brief PWM signal struct, used for each PWM signal to define it's parameters
 * @param duty Desired duty cycle (%)
 * @param freq Desired frequency (Hz)
 * @param base Base of PWM signal
 * @param gen PWM generator
 */
typedef struct{
uint32_t duty;
uint32_t freq;
int base;
int gen;
}pwmSignal;

void setPWM (uint32_t ui32Freq, uint32_t ui32Duty);
void setPWMGeneral(uint32_t ui32Freq, uint32_t ui32Duty, int base, int gen);
void initialisePWM (void);


#endif