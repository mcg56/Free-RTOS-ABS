/**********************************************************
car_pwm.h

Header file for the car PWM outputs module. Contains 
global structs of the PWM signal hardware details and 
initialisation functions handles.

A.J Eason A. Musalov
Last modified:  19/08/22
***********************************************************/

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
#include "pwm_output.h"

//*************************************************************
// Public Constant Definitions
//*************************************************************
#define PWM_WHEEL_FIXED_DUTY    50
#define PWM_WHEEL_START_HZ      0
#define PWM_BRAKE_FIXED_HZ      500
#define PWM_STEERING_FIXED_HZ   PWM_BRAKE_FIXED_HZ // As on the same generator


//*****************************************************************************
// Global variables
//*****************************************************************************
extern PWMOutputHardwareDetails_t PWMHardwareDetailsLF;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsLR;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsRF;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsRR;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsBrake;
extern PWMOutputHardwareDetails_t PWMHardwareDetailsSteering;

/**
 * @brief Function to initialise the car_pwm module. Initialises pwm
 * input manager, output PWM channels, registers input PWM signal and
 * creates the queue and task for output PWM signals.
 * off the output.
 * @return None
 */
void initCarPwm(void);

/**
 * @brief Function to initialise then turn on all car output PWM signals
 * off the output.
 * @return None
 */
void initializeCarPWMOutputs(void);

#endif