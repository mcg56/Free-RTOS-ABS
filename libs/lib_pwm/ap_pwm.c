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
#include "ap_pwm.h"
#include <FreeRTOS.h>
#include <queue.h>



QueueHandle_t updatePWMQueue = NULL;


/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used
 *********************************************************/
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

/********************************************************
 * Function to set the freq, duty cycle of M0PWM7
 ********************************************************/
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

/**
 * @brief Set the frequency and duty cycle of any PWM signal
 * @param ui32Freq Desired PWM frequency (Hz)
 * @param ui32Duty Desired PWM duty cycle (%)
 * @param base Base of PWM signal to change
 * @param gen PWM generator to change
 * @return No return
 */
void setPWMGeneral(uint32_t ui32Freq, uint32_t ui32Duty, int base, int gen, int outnum)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / ui32Freq;
    PWMGenPeriodSet(base, gen, ui32Period);
    PWMPulseWidthSet(base, outnum, ui32Period * ui32Duty / 100);
}


/**
 * @brief Task that changes PWM output
 * @param args Unused
 * @return None
 */
void updatePWMTask(void* args) 
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