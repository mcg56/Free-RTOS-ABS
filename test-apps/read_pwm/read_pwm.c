#include <stdint.h>
#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_pwm/ap_pwm.h"
#include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
#include "inc/tm4c123gh6pm.h"

#define TIMER_RATE 30000

static int intCheck = 0;
static uint32_t lastTimeStamp = 0;
static uint32_t diffTimeStamp = 0;
static uint32_t currTimeStamp = 0;

//*************************************************************
// GPIO Pin Interrupt
//*************************************************************
void
GPIOPinIntHandler (void)
{
    // Clean up, clearing the interrupt
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);

    intCheck++;

//     currTimeStamp = TimerLoadGet(TIMER0_BASE, TIMER_A);

//     if (currTimeStamp < lastTimeStamp)
//     {   
//         diffTimeStamp = currTimeStamp + TIMER_RATE - lastTimeStamp;
//     }
//     else
//     {
//         diffTimeStamp = currTimeStamp - lastTimeStamp;
//     }

//     lastTimeStamp = currTimeStamp;
}

//*************************************************************
// Intialise GPIO Pins
// PB0 and PB1 are used for quadrature encoding
// PC4 is used for reference yaw input
//*************************************************************
void
initGPIOPins (void)
{
    // Enable port peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Set pin 0,1 and 4 as input
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);

    // Set what pin interrupt conditions
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);

    // Register interrupt
    GPIOIntRegister(GPIO_PORTB_BASE, GPIOPinIntHandler);

    // Enable pins
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0);
}

//*****************************************************************************
// Intialise timer for PI control update
//*****************************************************************************
void
initResponseTimer (void)
{
    SYSCTL->RCGCTIMER |= 1;
}




int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initGPIOPins();
    initButtons ();
    initResponseTimer();

    while (true)
    {
        SysCtlDelay(3000);

        displayButtonState ("Freq", "", TimerLoadGet(TIMER0_BASE, TIMER_A), 0);
        displayButtonState ("Int Test", "", intCheck, 1);
    }

    vTaskStartScheduler();

    return 0;
}


// This is an error handling function called when FreeRTOS asserts.
// This should be used for debugging purposes
void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
    (void)pcFile; // unused
    (void)ulLine; // unused
    while (true) ;
}

