#ifndef SYSTEM_H_
#define SYSTEM_H_

/************************************************************
 system.h:
 File containing general functions related to the system such
 as the clock and sysTick interrupt. These are responsible for
 the timing of the program and also the interrupts.

 Authors:  Anton Musalov

 Acknowledgements: Megan Belton - Angus Eason

 ***********************************************************/

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Constants
//*****************************************************************************

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************

void
initClock (void);

void
initDisplay (void);

#endif /*SYSTEM_H_*/
