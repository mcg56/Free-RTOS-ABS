/** @file   abs_controller.c
    @author T. Peterson, M. Gardyne
    @date   22/08/22
    @brief  Main controlling file for the ABS controller.
*/


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#include "Project1/libs/lib_buttons/buttons.h"
#include "Project1/libs/lib_uart/uart.h"
#include "Project1/libs/lib_pwm/pwm_input.h"
#include "Project1/libs/lib_pwm/pwm_output.h"
#include "abs_manager.h"

#include "brake_output.h"
#include "display.h"
#include "pwm_info.h"
#include "status_led.h"


int main (void)
{
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initButtons ();
    initPWMInputManager (ABS_PWM_MIN_FREQ);
    initialiseUSB_UART ();
    initBrakeOutput ();
    initDisplay ();
    initABSManager ();

    // Initialise input signals
    PWMSignal_t BrakePedalPWM = {.id = BRAKE_PEDAL_ID, .gpioPort = BRAKE_PEDAL_GPIO_BASE, .gpioPin = BRAKE_PEDAL_GPIO_PIN};
    registerPWMSignal(BrakePedalPWM); 

    PWMSignal_t LFPWM = {.id = FL_WHEEL_ID, .gpioPort = FL_WHEEL_GPIO_BASE, .gpioPin = FL_WHEEL_GPIO_PIN};
    registerPWMSignal(LFPWM);

    PWMSignal_t RFPWM = {.id = FR_WHEEL_ID, .gpioPort = FR_WHEEL_GPIO_BASE, .gpioPin = FR_WHEEL_GPIO_PIN};
    registerPWMSignal(RFPWM);

    PWMSignal_t LRPWM = {.id = RL_WHEEL_ID, .gpioPort = RL_WHEEL_GPIO_BASE, .gpioPin = RL_WHEEL_GPIO_PIN};
    registerPWMSignal(LRPWM);

    PWMSignal_t RRPWM = {.id = RR_WHEEL_ID, .gpioPort = RR_WHEEL_GPIO_BASE, .gpioPin = RR_WHEEL_GPIO_PIN};
    registerPWMSignal(RRPWM);  

    PWMSignal_t SteeringPWM = {.id = STEERING_ID, .gpioPort = STEERING_GPIO_BASE, .gpioPin = STEERING_GPIO_PIN};
    registerPWMSignal(SteeringPWM);
    
    vTaskStartScheduler();

    return 0;
}

// This is an error handling function called when FreeRTOS asserts.
// This should be used for debugging purposes
void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
    (void)pcFile; // unused
    (void)ulLine; // unused
    while (true) ;
}