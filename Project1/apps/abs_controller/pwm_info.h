#ifndef PWM_INFO_H_
#define PWM_INFO_H_

/** @file   pwm_info.h
    @author T. Peterson, M. Gardyne
    @date   22/08/22
    @brief  Contains input signal information
*/

//*************************************************************
// Wheel Input Definitions
//*************************************************************

// Front left wheel
#define FL_WHEEL_ID             "LF"
#define FL_WHEEL_GPIO_BASE      GPIO_PORTB_BASE
#define FL_WHEEL_GPIO_PIN       GPIO_PIN_0

// Front right wheel
#define FR_WHEEL_ID             "RF"
#define FR_WHEEL_GPIO_BASE      GPIO_PORTB_BASE
#define FR_WHEEL_GPIO_PIN       GPIO_PIN_1

// Rear left wheel
#define RL_WHEEL_ID             "LR"
#define RL_WHEEL_GPIO_BASE      GPIO_PORTB_BASE
#define RL_WHEEL_GPIO_PIN       GPIO_PIN_2

// Rear right wheel
#define RR_WHEEL_ID             "RR"
#define RR_WHEEL_GPIO_BASE      GPIO_PORTB_BASE
#define RR_WHEEL_GPIO_PIN       GPIO_PIN_4

// Steering signal
#define STEERING_ID             "Steering"
#define STEERING_GPIO_BASE      GPIO_PORTB_BASE
#define STEERING_GPIO_PIN       GPIO_PIN_5

// Brake pedal signal
#define BRAKE_PEDAL_ID          "BrakePedal"
#define BRAKE_PEDAL_GPIO_BASE   GPIO_PORTC_BASE
#define BRAKE_PEDAL_GPIO_PIN    GPIO_PIN_7

#endif