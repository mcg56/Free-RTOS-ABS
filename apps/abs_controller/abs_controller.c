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

#include "libs/lib_buttons/buttons.h"
#include "libs/lib_uart/uart.h"
#include "libs/lib_system/system.h"
#include "libs/lib_pwm/pwm_input.h"
#include "libs/lib_pwm/pwm_output.h"
#include "abs_manager.h"

#include "brake_output.h"
#include "display.h"
#include "pwm_info.h"

#include "status_led.h"

TaskHandle_t testHandle;
TaskHandle_t updateAllPWMInputsHandle;
TaskHandle_t calculatePWMPropertiesHandle;


void
printPWM(char* id)
{
    char str[100];
    PWMSignal_t signal;
    // Details of first PWM
    signal = getPWMInputSignal(id);
    sprintf(str, "Signal ID = %s\r\n", id);
    UARTSend(str);
    sprintf(str, "Frequency = %ld Hz\r\n", signal.frequency);
    UARTSend(str);
    sprintf(str, "Duty : %ld\r\n\n", signal.duty);
    UARTSend(str);
}

void testTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    UARTSend("\n\rWaiting for press...\r\n");
    while (true)
    {
        // updateButtons();
        // char str[100];

        // if (checkButton(LEFT) == PUSHED)
        // {
        //     printPWM("LF");
        //     printPWM("RF");
        //     printPWM("LR");
        //     printPWM("RR");
        //     printPWM("Steering");
        //     printPWM("BrakePedal");
        // }

        // if (checkButton(RIGHT) == PUSHED)
        // {
        //     // toggleABS();
        //     // setStatusLEDBlinkRate(5);
        // }

        // // if (getPWMInputSignal("RR").frequency < 60)
        // // {
        // //     setABS(ABS_ON);
        // // }
        // // else
        // // {
        // //     setABS(ABS_OFF);
        // // }
        // if (checkButton(UP) == PUSHED)
        // {
        //     setABSDuty(getABSDuty() + 5);
        // }
        // if (checkButton(DOWN) == PUSHED)
        // {
        //     setABSDuty(getABSDuty() - 5);
        // }

        vTaskDelay(xDelay);
    }
}

int main (void)
{
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initButtons ();
    initPWMInputManager (ABS_PWM_MIN_FREQ);
    initialiseUSB_UART ();
    initBrakeOutput ();
    initDisplay ();
    initABSManager ();

    // TO DO: Should all this PWM stuff be its own module? pwm_manager?
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

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

    PWMSignal_t BrakePedalPWM = {.id = BRAKE_PEDAL_ID, .gpioPort = BRAKE_PEDAL_GPIO_BASE, .gpioPin = BRAKE_PEDAL_GPIO_PIN};
    registerPWMSignal(BrakePedalPWM); 

    xTaskCreate(&testTask, "testTask", 256, NULL, 0, &testHandle);

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

// TO DO: Fix pwm numbers accurate