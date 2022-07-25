/************************************************************
 system.c:
 File containing general functions related to the system such
 as the clock and sysTick interrupt. These are responsible for
 the timing of the program and also the interrupts.

 Authors:  Anton Musalov

 Acknowledgements: Megan Belton - Angus Eason

 ***********************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"

#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"

#define SAMPLE_RATE_HZ 10
//*****************************************************************************
// Global variables
//*****************************************************************************




//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************

void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    //SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    //SysTickIntEnable();
    //SysTickEnable();
}

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}