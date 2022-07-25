#ifndef SYSTEM_H_
#define SYSTEM_H_

// *******************************************************
// buttons4.h
//
// Support for a set of FOUR specific buttons on the Tiva/Orbit.
// ENCE361 sample code.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva.
//
// P.J. Bones UCECE
// Last modified:  7.2.2018
// 
// *******************************************************

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Global variables
//*****************************************************************************


//*****************************************************************************
// Constants
//*****************************************************************************

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************

void
initClock (void);

void
initDisplay (void);

#endif /*SYSTEM_H_*/
