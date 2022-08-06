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
#include "libs/lib_uart/ap_uart.h"
#include "libs/lib_system/ap_system.h"
#include "libs/lib_pwm/ap_pwm_input.h"
#include "libs/lib_pwm/ap_pwm_output.h"

#include "brake_output.h"
#include "display.h"

TaskHandle_t updateButtonsHandle;
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

void updateButtonsTask(void* args)
{
    (void)args;
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;

    UARTSend("\n\rWaiting for press...\r\n");
    while (true)
    {
        updateButtons();

        if (checkButton(LEFT) == PUSHED)
        {
            printPWM("LF");
            printPWM("RF");
            printPWM("LR");
            printPWM("RR");
            printPWM("Steering");
            printPWM("BrakePedal");
        }

        if (checkButton(RIGHT) == PUSHED)
        {
            toggleABS();
        }

        // if (getPWMInputSignal("RR").frequency < 60)
        // {
        //     setABS(ABS_ON);
        // }
        // else
        // {
        //     setABS(ABS_OFF);
        // }
        if (checkButton(UP) == PUSHED)
        {
            setABSDuty(getABSDuty() + 5);
        }
        if (checkButton(DOWN) == PUSHED)
        {
            setABSDuty(getABSDuty() - 5);
        }

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
    

    // TO DO: Should all this PWM stuff be its own module? pwm_manager?
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    PWMSignal_t LFPWM = {.id = "LF", .gpioPin = GPIO_PIN_0};
    registerPWMSignal(LFPWM);

    PWMSignal_t RFPWM = {.id = "RF", .gpioPin = GPIO_PIN_1};
    registerPWMSignal(RFPWM);

    PWMSignal_t LRPWM = {.id = "LR", .gpioPin = GPIO_PIN_2};
    registerPWMSignal(LRPWM);

    PWMSignal_t RRPWM = {.id = "RR", .gpioPin = GPIO_PIN_4};
    registerPWMSignal(RRPWM);  

    PWMSignal_t SteeringPWM = {.id = "Steering", .gpioPin = GPIO_PIN_5};
    registerPWMSignal(SteeringPWM);

    PWMSignal_t BrakePedalPWM = {.id = "BrakePedal", .gpioPin = GPIO_PIN_6};
    registerPWMSignal(BrakePedalPWM); 

    xTaskCreate(&updateButtonsTask, "updateButtons", 256, NULL, 0, &updateButtonsHandle);
    xTaskCreate(&updateAllPWMInputsTask, "updateAllPWMInputs", 256, NULL, 0, &updateAllPWMInputsHandle);
    // xTaskCreate(&calculatePWMPropertiesTask, "calculatePWMProperties", 256, NULL, 0, &calculatePWMPropertiesHandle);  

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
