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
#include "ap_pwm_output.h"
#include <FreeRTOS.h>
#include <queue.h>
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include <stdio.h>

/************************************************************************************
*****************************PRIVATE FUNCTION PROTOTYPES*****************************
************************************************************************************/

/**
 * @brief Function to initialise a given PWM. Sets a start Hz and duty but turns
 * off the output.
 * @param PWM Hardware details of the PWM to initialize
 * @param startHz Starting Hz
 * @param startDuty Starting duty
 * @return None
 */
void initializePWMGeneral(PWMHardwareDetails PWM, uint32_t startHz, uint32_t startDuty);


/************************************************************************************
**********************************GLOBAL VARIABLES**********************************
************************************************************************************/
QueueHandle_t updatePWMQueue = NULL;
PWMHardwareDetails PWMHardwareDetailsMAIN = {PWM_MAIN_BASE, PWM_MAIN_GEN, PWM_MAIN_OUTNUM, PWM_MAIN_OUTBIT, PWM_MAIN_PERIPH_PWM, PWM_MAIN_PERIPH_GPIO, PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_CONFIG, PWM_MAIN_GPIO_PIN};
PWMHardwareDetails PWMHardwareDetailsLF = {PWM_LF_BASE, PWM_LF_GEN, PWM_LF_OUTNUM, PWM_LF_OUTBIT, PWM_LF_PERIPH_PWM, PWM_LF_PERIPH_GPIO, PWM_LF_GPIO_BASE, PWM_LF_GPIO_CONFIG, PWM_LF_GPIO_PIN};
PWMHardwareDetails PWMHardwareDetailsLR = {PWM_LR_BASE, PWM_LR_GEN, PWM_LR_OUTNUM, PWM_LR_OUTBIT, PWM_LR_PERIPH_PWM, PWM_LR_PERIPH_GPIO, PWM_LR_GPIO_BASE, PWM_LR_GPIO_CONFIG, PWM_LR_GPIO_PIN};
PWMHardwareDetails PWMHardwareDetailsRF = {PWM_RF_BASE, PWM_RF_GEN, PWM_RF_OUTNUM, PWM_RF_OUTBIT, PWM_RF_PERIPH_PWM, PWM_RF_PERIPH_GPIO, PWM_RF_GPIO_BASE, PWM_RF_GPIO_CONFIG, PWM_RF_GPIO_PIN};
PWMHardwareDetails PWMHardwareDetailsRR = {PWM_RR_BASE, PWM_RR_GEN, PWM_RR_OUTNUM, PWM_RR_OUTBIT, PWM_RR_PERIPH_PWM, PWM_RR_PERIPH_GPIO, PWM_RR_GPIO_BASE, PWM_RR_GPIO_CONFIG, PWM_RR_GPIO_PIN};
PWMHardwareDetails PWMHardwareDetailsSteering = {PWM_STEER_BASE, PWM_STEER_GEN, PWM_STEER_OUTNUM, PWM_STEER_OUTBIT, PWM_STEER_PERIPH_PWM, PWM_STEER_PERIPH_GPIO, PWM_STEER_GPIO_BASE, PWM_STEER_GPIO_CONFIG, PWM_STEER_GPIO_PIN};
PWMHardwareDetails PWMHardwareDetailsBrake = {PWM_BRAKE_BASE, PWM_BRAKE_GEN, PWM_BRAKE_OUTNUM, PWM_BRAKE_OUTBIT, PWM_BRAKE_PERIPH_PWM, PWM_BRAKE_PERIPH_GPIO, PWM_BRAKE_GPIO_BASE, PWM_BRAKE_GPIO_CONFIG, PWM_BRAKE_GPIO_PIN};



/************************************************************************************
**********************************PUBLIC FUNCTIONS**********************************
************************************************************************************/


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
        // Wait until a new pwm is to be made, as its added to queue
        pwmSignal requestedPWM;
        portBASE_TYPE status = xQueueReceive(updatePWMQueue, &requestedPWM, 100);
        if (status == pdPASS)
        {
            setPWMGeneral(requestedPWM.freq, requestedPWM.duty, requestedPWM.base, requestedPWM.gen, requestedPWM.outnum);
        } else continue;
        
    }
}



void initializePWMGeneral(PWMHardwareDetails PWM, uint32_t startHz, uint32_t startDuty)
{
    // Reset peripherals before enabling them. DO WE BEED TO RESET WHOLE PERIPHERAL?
    // Means this function must come very early in setup and being after initOLED or LED 
    // will make them not work
    //SysCtlPeripheralReset (PWM.periphGPIO); // Used for PWM output
    //SysCtlPeripheralReset (PWM.periphPWM); // GPIO for PWM

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



void initializeCarPWMOutputs(void)
{
    initializePWMGeneral(PWMHardwareDetailsLF, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsLR, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsRF, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsRR, PWM_WHEEL_START_HZ, PWM_WHEEL_FIXED_DUTY);
    initializePWMGeneral(PWMHardwareDetailsSteering, PWM_STEERING_FIXED_HZ, PWM_STEERING_START_DUTY); // Need to check the nominal freq/duty of this
    initializePWMGeneral(PWMHardwareDetailsBrake, PWM_BRAKE_FIXED_HZ, PWM_BRAKE_START_DUTY); // Need to check the nominal freq/duty of this
    
    // Initialisation is complete, so turn on the pwm output.
    PWMOutputState(PWMHardwareDetailsLF.base, PWMHardwareDetailsLF.outbit, true);
    PWMOutputState(PWMHardwareDetailsLR.base, PWMHardwareDetailsLR.outbit, true);
    PWMOutputState(PWMHardwareDetailsRF.base, PWMHardwareDetailsRF.outbit, true);
    PWMOutputState(PWMHardwareDetailsRR.base, PWMHardwareDetailsRR.outbit, true);
    PWMOutputState(PWMHardwareDetailsSteering.base, PWMHardwareDetailsSteering.outbit, true);
    PWMOutputState(PWMHardwareDetailsBrake.base, PWMHardwareDetailsBrake.outbit, true);
}

