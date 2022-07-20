/*
 * pwmGen5.c
 *
 *  Created on: Mar 17, 2018
 *  Updated on: Jul 8, 2022 for ENCE464
 *  Author: steve weddell, with help from several quite brilliant TAs over the years...
 *  Purpose: To simply the implementation of generating PWM from ENCE361 at
 *  300-level for application using freeRTOS in ENCE464.
 */

#include <stdint.h>
#include <stdbool.h>
#include "utils/ustdlib.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "buttons4.h"
#include "stdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"

/**********************************************************
 * Generates a single PWM signal on Tiva board pin J4-05 =
 * PC5 (M0PWM7)....
 **********************************************************/

/**********************************************************
 * Constants
 **********************************************************/
// Systick configuration
#define SYSTICK_RATE_HZ    100

// PWM configuration
#define PWM_START_RATE_HZ  250
#define PWM_RATE_STEP_HZ   50
#define PWM_RATE_MIN_HZ    50
#define PWM_RATE_MAX_HZ    400
#define PWM_FIXED_DUTY     67
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        4

#define PWM_DUTY_MIN		5
#define PWM_DUTY_RATE		5
#define PWM_DUTY_MAX		95

//  PWM Hardware Details M0PWM7 (gen 3)
//  PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

/*******************************************
 *      Local prototypes
 *******************************************/
void SysTickIntHandler (void);
void initDisplay(void);
void initClocks (void);
void initSysTick (void);
void initialisePWM (void);
void setPWM (uint32_t u32Freq, uint32_t u32Duty);
void displayButtonState (char *, uint8_t, uint8_t);
void displayUpdate (char *, uint8_t, uint8_t);


/***********************************************************
 * ISR for the SysTick interrupt (used for button debouncing).
 ***********************************************************/
void
SysTickIntHandler (void)
{
    //
    // Poll the buttons
    updateButtons();
    //
    // It is not necessary to clear the SysTick interrupt.
}

/***********************************************************
 * Initialisation functions: clock, SysTick, PWM
 ***********************************************************
 * Clock
 ***********************************************************/
void
initClocks (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
}

// *******************************************************
void
initDisplay (void)
{
    // Initialise the Orbit OLED display
	OLEDInitialise ();
}

/*************************************************************
 * SysTick interrupt
 ************************************************************/
void
initSysTick (void)
{
    //
    // Set up the period for the SysTick timer.  The SysTick
    // timer period is set as a function of the system clock.
    SysTickPeriodSet (SysCtlClockGet() / SYSTICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister (SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable ();
    SysTickEnable ();
}

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

//*****************************************************************************
// Function to display a changing message on the display.
// The display has 4 rows of 16 characters, with 0, 0 at top left.
//*****************************************************************************
void
displayUpdate (char *str1, uint8_t num, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%s %3d", str1, num);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

// The main program simply initialises the Tiva microcontroller and particularly the
// PWM attributes for this demonstration. It also initialises the Orbit board, which
// allows the extra buttons to be used.
// All the main loop does is check for button presses and updates the Orbit display.
//
// For porting this across to a freeRTOS implementation, use a task to handle the display,
// a data queue stream the data between other tasks, i.e., showing frequency and duty
// cycle information, and a GiveSemaphoreISR to check for interrupts from input PWM
// signals. Also, look at the demo_freeRTOS for implementation details, and check
// the freeRTOS manual for other implementation details. This should get you going
// on your ENCE464 Term-3 project.
//
int
main (void)
{
    uint32_t ui32Freq = PWM_START_RATE_HZ;	// PWM rate is is initialised with orig value (250)
    uint32_t ui32Duty = PWM_FIXED_DUTY; 	// PWM duty cycle is initialised with orig value (67)

    initClocks ();

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    displayUpdate ("Freq", ui32Freq, 0);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    displayUpdate ("Freq", ui32Freq, 0);
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    displayUpdate ("Freq", ui32Freq, 0);
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO

    initButtons ();  // Initialises 4 pushbuttons (UP, DOWN, LEFT, RIGHT)
    initialisePWM ();
    initSysTick ();
	initDisplay();

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);

    //
    // Enable interrupts to the processor.
    IntMasterEnable ();

    //
    // Loop forever, controlling the PWM frequency and
    // maintaining the the PWM duty cycle.
    while (1)
    {
        // Background task: Check for button pushes and control
        // the PWM frequency within a fixed range.
        if ((checkButton (UP) == PUSHED) && (ui32Freq < PWM_RATE_MAX_HZ))
        {
            ui32Freq += PWM_RATE_STEP_HZ;
            setPWM (ui32Freq, PWM_FIXED_DUTY);
        }
        if ((checkButton (DOWN) == PUSHED) && (ui32Freq > PWM_RATE_MIN_HZ))
        {
            ui32Freq -= PWM_RATE_STEP_HZ;
            setPWM (ui32Freq, PWM_FIXED_DUTY);
        }
        if ((checkButton (LEFT) == PUSHED) && (ui32Duty < PWM_DUTY_MAX))
        {
                   ui32Duty += PWM_DUTY_RATE;
                   setPWM (ui32Freq, ui32Duty);
       }
       if ((checkButton (RIGHT) == PUSHED) && (ui32Duty > PWM_DUTY_MIN))
       {
                   ui32Duty -= PWM_DUTY_RATE;
                   setPWM (ui32Freq, ui32Duty);
       }

       OLEDStringDraw ("Lab PWM-DEMO", 0, 0);
       displayUpdate ("Freq", ui32Freq, 1);
       displayUpdate ("Duty", ui32Duty, 2);

       SysCtlDelay (SysCtlClockGet () / 150);    // Approx 50 Hz
    }
}



