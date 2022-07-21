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

/**********************************************************
 * Generates a single PWM signal on Tiva board pin J4-05 =
 * PC5 (M0PWM7).  This is the same PWM output as the
 * helicopter main rotor.
 **********************************************************/

/**********************************************************
 * Constants
 **********************************************************/
// Systick configuration


/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 *********************************************************/
void
initialisePWM (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    setPWM (PWM_START_RATE_HZ, PWM_FIXED_DUTY);

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


// int
// main (void)
// {
//     uint32_t ui32Freq = PWM_START_RATE_HZ;

//     initClocks ();

//     // As a precaution, make sure that the peripherals used are reset
//     SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
//     SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
//     SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
//     SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO

//     initButtons ();  // Initialises 4 pushbuttons (UP, DOWN, LEFT, RIGHT)
//     initialisePWM ();
//     initSysTick ();

//     // Initialisation is complete, so turn on the output.
//     PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

//     //
//     // Enable interrupts to the processor.
//     IntMasterEnable ();

//     //
//     // Loop forever, controlling the PWM frequency and
//     // maintaining the the PWM duty cycle.
//     while (1)
//     {
//         // Background task: Check for button pushes and control
//         // the PWM frequency within a fixed range.
//         if ((checkButton (UP) == PUSHED) && (ui32Freq < PWM_RATE_MAX_HZ))
//         {
//     	    ui32Freq += PWM_RATE_STEP_HZ;
//     	    setPWM (ui32Freq, PWM_FIXED_DUTY);
//         }
//         if ((checkButton (DOWN) == PUSHED) && (ui32Freq > PWM_RATE_MIN_HZ))
//         {
//     	    ui32Freq -= PWM_RATE_STEP_HZ;
//     	    setPWM (ui32Freq, PWM_FIXED_DUTY);
//         }
//     }
// }
