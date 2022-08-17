/**********************************************************
 *
 * pwmGen.c - Example code which generates a single PWM
 *    output on J4-05 (M0PWM7) with duty cycle fixed and
 *    the frequency controlled by UP and DOWN buttons in
 *    the range 50 Hz to 400 Hz.
 * 2017: Modified for Tiva and using straightforward, polled
 *    button debouncing implemented in 'buttons4' module.
 *
 * P.J. Bones   UCECE
 * Last modified:  7.2.2018
 **********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "pwm_output.h"
#include <FreeRTOS.h>
#include <queue.h>
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include <stdio.h>

/************************************************************************************
*****************************PRIVATE FUNCTION PROTOTYPES*****************************
************************************************************************************/


/************************************************************************************
**********************************GLOBAL VARIABLES**********************************
************************************************************************************/
QueueHandle_t updatePWMQueue = NULL;
TaskHandle_t updatePWMOutputsTaskHandle;

/************************************************************************************
**********************************PUBLIC FUNCTIONS**********************************
************************************************************************************/

void createPWMQueue(void)
{
    // Create queue for passing signals to PWM task
    updatePWMQueue = xQueueCreate(10, sizeof(pwmOutputUpdate_t));
}

void
initialisePWM (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);  	// turn on main pwm peripheral
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);	// turn on GPIO for PWM mode

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);			// config for GPIO_PC5_M0PWM7  (J4 pin 5)
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);  // use GPIO Pin 5

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    
    // Set the initial PWM parameters
    setPWM (PWM_START_RATE_HZ, PWM_FIXED_DUTY);	// start rate is 250 Hz, 67% duty cycle

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
}


void
setPWM (uint32_t ui32Freq, uint32_t ui32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
        ui32Period * ui32Duty / 100);
}

void setPWMGeneral(uint32_t ui32Freq, uint32_t ui32Duty, uint32_t base, uint32_t gen, uint32_t outnum)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / ui32Freq;
    PWMGenPeriodSet(base, gen, ui32Period);
    PWMPulseWidthSet(base, outnum, ui32Period * ui32Duty / 100);
}

void updatePWMOutputsTask(void* args) 
{
    (void)args; // unused
    while(true)
    {
        // Wait until a new pwm is to be updated, when its added to queue
        pwmOutputUpdate_t requestedPWM;
        portBASE_TYPE status = xQueueReceive(updatePWMQueue, &requestedPWM, 100);
        if (status == pdPASS)
        {
            setPWMGeneral(requestedPWM.freq, requestedPWM.duty, requestedPWM.pwmOutput.base, requestedPWM.pwmOutput.gen, requestedPWM.pwmOutput.outnum);
        } else continue;
        
    }
}

void initializePWMGeneral(PWMOutputHardwareDetails_t PWM, uint32_t startHz, uint32_t startDuty)
{
    // Now enable the peripherals
    SysCtlPeripheralEnable(PWM.periphPWM);  // turn on pwm peripheral
    SysCtlPeripheralEnable(PWM.periphGPIO);	// turn on GPIO for PWM mode

    // Configure GPIO for PWM
    GPIOPinConfigure(PWM.gpioConfig);
    GPIOPinTypePWM(PWM.gpioBase, PWM.gpioPin);

    PWMGenConfigure(PWM.base, PWM.gen,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the PWM clock rate (using the prescaler) and reset the GPIO pins
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    
    // Set the initial PWM parameters
    setPWMGeneral (startHz, startDuty, PWM.base, PWM.gen, PWM.outnum);

    // Enable timer/counter
    PWMGenEnable(PWM.base, PWM.gen);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM.base, PWM.outbit, false);
}





