//*****************************************************************************
//
// ADCdemo1.c - Simple interrupt driven program which samples with AIN0
//
// Author:  P.J. Bones	UCECE
// Last modified:	8.2.2018
//
//*****************************************************************************
// Based on the 'convert' series from 2016
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "libs/lib_adc/ap_adc.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "libs/lib_system/ap_system.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 10



int
main(void)
{
	uint16_t i;
	int32_t sum;
	
	initClock ();
	initADC ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);

    //
    // Enable interrupts to the processor.
    IntMasterEnable();

	while (1)
	{
		//
		// Background task: calculate the (approximate) mean of the values in the
		// circular buffer and display it, together with the sample number.
		sum = 0;

		for (i = 0; i < BUF_SIZE; i++)
			sum = sum + readCircBuf (&g_inBuffer);
		// Calculate and display the rounded mean of the buffer contents
		displayMeanVal ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE, g_ulSampCnt);

		SysCtlDelay (SysCtlClockGet() / 6);  // Update display at ~ 2 Hz
	}
}

// This is an error handling function called when FreeRTOS asserts.
// This should be used for debugging purposes
void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
    (void)pcFile; // unused
    (void)ulLine; // unused
    while (true) ;
}