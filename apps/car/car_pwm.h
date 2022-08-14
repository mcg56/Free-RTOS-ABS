#ifndef CAR_PWM_H_
#define CAR_PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "ap_pwm_output.h"

#define PWM_WHEEL_FIXED_DUTY    50
#define PWM_WHEEL_START_HZ      0

// Left front wheel output PWM hardware details
#define PWM_LF_BASE	        PWM0_BASE
#define PWM_LF_GEN          PWM_GEN_1
#define PWM_LF_OUTNUM       PWM_OUT_3
#define PWM_LF_OUTBIT       PWM_OUT_3_BIT
#define PWM_LF_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_LF_PERIPH_GPIO  SYSCTL_PERIPH_GPIOB
#define PWM_LF_GPIO_BASE    GPIO_PORTB_BASE
#define PWM_LF_GPIO_CONFIG  GPIO_PB5_M0PWM3
#define PWM_LF_GPIO_PIN     GPIO_PIN_5

// Left rear wheel output PWM hardware details
#define PWM_LR_BASE	        PWM0_BASE
#define PWM_LR_GEN          PWM_GEN_2
#define PWM_LR_OUTNUM       PWM_OUT_4
#define PWM_LR_OUTBIT       PWM_OUT_4_BIT
#define PWM_LR_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_LR_PERIPH_GPIO  SYSCTL_PERIPH_GPIOE
#define PWM_LR_GPIO_BASE    GPIO_PORTE_BASE
#define PWM_LR_GPIO_CONFIG  GPIO_PE4_M0PWM4
#define PWM_LR_GPIO_PIN     GPIO_PIN_4

//Right front wheel output PWM hardware details
#define PWM_RF_BASE	        PWM0_BASE
#define PWM_RF_GEN          PWM_GEN_3
#define PWM_RF_OUTNUM       PWM_OUT_7
#define PWM_RF_OUTBIT       PWM_OUT_7_BIT
#define PWM_RF_PERIPH_PWM	SYSCTL_PERIPH_PWM0
#define PWM_RF_PERIPH_GPIO  SYSCTL_PERIPH_GPIOC
#define PWM_RF_GPIO_BASE    GPIO_PORTC_BASE
#define PWM_RF_GPIO_CONFIG  GPIO_PC5_M0PWM7
#define PWM_RF_GPIO_PIN     GPIO_PIN_5

//Right rear wheel output PWM hardware details
#define PWM_RR_BASE	        PWM1_BASE
#define PWM_RR_GEN          PWM_GEN_1
#define PWM_RR_OUTNUM       PWM_OUT_2
#define PWM_RR_OUTBIT       PWM_OUT_2_BIT
#define PWM_RR_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_RR_PERIPH_GPIO  SYSCTL_PERIPH_GPIOA
#define PWM_RR_GPIO_BASE    GPIO_PORTA_BASE
#define PWM_RR_GPIO_CONFIG  GPIO_PA6_M1PWM2
#define PWM_RR_GPIO_PIN     GPIO_PIN_6

//Brake level output PWM hardware details
#define PWM_BRAKE_BASE	        PWM1_BASE
#define PWM_BRAKE_GEN           PWM_GEN_3
#define PWM_BRAKE_OUTNUM        PWM_OUT_6
#define PWM_BRAKE_OUTBIT        PWM_OUT_6_BIT
#define PWM_BRAKE_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_BRAKE_PERIPH_GPIO   SYSCTL_PERIPH_GPIOF
#define PWM_BRAKE_GPIO_BASE     GPIO_PORTF_BASE
#define PWM_BRAKE_GPIO_CONFIG   GPIO_PF2_M1PWM6
#define PWM_BRAKE_GPIO_PIN      GPIO_PIN_2
#define PWM_BRAKE_FIXED_HZ      500
#define PWM_BRAKE_START_DUTY    5

//Steering wheel output PWM hardware details
#define PWM_STEER_BASE	        PWM1_BASE
#define PWM_STEER_GEN           PWM_GEN_3
#define PWM_STEER_OUTNUM        PWM_OUT_7
#define PWM_STEER_OUTBIT        PWM_OUT_7_BIT
#define PWM_STEER_PERIPH_PWM	SYSCTL_PERIPH_PWM1
#define PWM_STEER_PERIPH_GPIO   SYSCTL_PERIPH_GPIOF
#define PWM_STEER_GPIO_BASE     GPIO_PORTF_BASE
#define PWM_STEER_GPIO_CONFIG   GPIO_PF3_M1PWM7
#define PWM_STEER_GPIO_PIN      GPIO_PIN_3
#define PWM_STEERING_FIXED_HZ   PWM_BRAKE_FIXED_HZ // As on the same generator
#define PWM_STEERING_START_DUTY 50

extern PWMOutputHardwareDetails_t PWMHardwareDetailsLF;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsLR;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsRF;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsRR;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsBrake;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsSteering;


/**
 * @brief Function to initialise then turn on all car output PWM signals
 * off the output.
 * @return None
 */
void initializeCarPWMOutputs(void);

#endif