/**********************************************************
 *
 * abs_controller.c - Main controlling file for the 
 *      ABS controller.
 *
 * T.R Peterson, M.C Gardyne
 * Last modified:  24.7.22
 **********************************************************/

/**
 * General plan
 * 
 * 1. In the background have all the PWM information updating
 * 2. Do some calculations based on the PWM states to determine if the car
 *    is slipping.
 * 3. Output a brake PWM according to calculation
 * 4. Update the screen to reflect the current state/information
 * 5. Update an LED
 * 
 */

/**
 * This will contain:
 * 
 * 1. The overall main and task creation stuff
 *
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

#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "libs/lib_uart/ap_uart.h"
#include "libs/lib_system/ap_system.h"

#include "pwm_manager.h"

int main (void)
{
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initButtons ();
    initPWMInputManager ();
    initialiseUSB_UART ();
    initialisePWM();

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    PWMSignal_t testPWM = {.id = "testPWM", .gpioPin = GPIO_PIN_0};
    registerPWMSignal(testPWM);

    // PWMSignal_t testPWM2 = {.id = "testPWM2", .gpioPin = GPIO_PIN_1};
    // trackPWMSignal(testPWM2);

    setPWM(41, 96);


    char str[100];
    PWMSignal_t signal;
    UARTSend("\n\rWaiting for press...\r\n");
    while (true)
    {
        updateButtons();

        if (checkButton(LEFT) == PUSHED)
        {
            updatePWMInput("testPWM");
            
            // Details of first PWM
            signal = getPWMInputSignal("testPWM");
            sprintf(str, "Frequency = %ld Hz\r\n", signal.frequency);
            UARTSend(str);
            sprintf(str, "Duty : %ld\r\n", signal.duty);
            UARTSend(str);

            // // Details of second PWM
            // signal = getPWMInputSignals("testPWM2");
            // sprintf(str, "Frequency = %ld Hz\r\n", signal.frequency);
            // UARTSend(str);
            // sprintf(str, "Duty : %ld\r\n", signal.duty);
            // UARTSend(str);
        }
    }

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
