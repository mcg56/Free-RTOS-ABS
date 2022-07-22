//*****************************************************************************
// Milestone 1 without freeRTOS functionality
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "stdlib.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"
#include "ap_buttons.h"



//*****************************************************************************
// Initialisation functions: clock, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}

// *******************************************************
void
initDisplay (void)
{
  // intialise the Orbit OLED display
	OLEDInitialise ();
}


//*****************************************************************************
// Function to display the mean interval in usec
//*****************************************************************************
void
displayButtonState (char *butStr, char *stateStr, uint8_t numPushes, uint8_t charLine)
{
    char string[17]; // Display fits 16 characters wide.
	
    OLEDStringDraw ("                ", 0, charLine);
    usnprintf (string, sizeof(string), "%s - %s %2d", butStr, stateStr, numPushes);
    OLEDStringDraw (string, 0, charLine);
}


int
main(void)
{
	int8_t upPushes = 0;
    bool butValue = 1;
	
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    initialisePWM ();

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    setPWM(500, 30);
    
	initDisplay ();
	initButtons ();
    
    displayButtonState ("PWM   ", "RELS", upPushes, 0);


	while(1)
	{
		uint8_t butState;

        updateButtons ();		// Poll the buttons
        // check state of each button and display if a change is detected
        butState = checkButton (UP);
        switch (butState)
        {
        case PUSHED:
        	displayButtonState ("UP   ", "PUSH", ++upPushes, 0);
            butValue = !butValue;
        	break;

        }
        if (butValue) {
            setPWM(500, 30);
            SysCtlDelay (SysCtlClockGet () / 115);
            setPWM(0, 0);
		    SysCtlDelay (SysCtlClockGet () / 115);	// Approx 50 Hz polling
        } else {
            setPWM(500,30);
        }
	}
}

void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
    (void)pcFile; // unused
    (void)ulLine; // unused
    while (true) ;
}