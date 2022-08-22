/** @file   pwm_output.c
    @author A.J Eason A. Musalov
    @date   21/08/22
    @brief  Module for initialising and managing PWM signals.
            The module creates a pwm ouput task which manages the any desired 
            PWM output. The PWM information is shared using a queue.

            Original Code: P.J. Bones UCECE pwm_gen.c
*/

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

//*****************************************************************************
// Global variables
//*****************************************************************************

QueueHandle_t updatePWMQueue = NULL;
TaskHandle_t updatePWMOutputsTaskHandle;

//*****************************************************************************
// Functions
//*****************************************************************************

void createPWMQueue(void)
{
    // Create queue for passing signals to PWM task
    updatePWMQueue = xQueueCreate(10, sizeof(pwmOutputUpdate_t));
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
    TickType_t ticksToWait = 100;
    while(true)
    {
        // Wait until a new pwm is to be updated, when its added to queue
        pwmOutputUpdate_t requestedPWM;
        portBASE_TYPE status = xQueueReceive(updatePWMQueue, &requestedPWM, ticksToWait);
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





