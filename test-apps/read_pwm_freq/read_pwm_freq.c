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
#include "libs/lib_uart/ap_uart.h"

#define TIMER_RATE 1

static uint32_t lastTimeStamp = 0;
static uint32_t diffTimeStamp = 0;
static uint32_t currTimeStamp = 0;
static uint32_t currTimeStampLow = 0;
static uint32_t duty = 0;

//*************************************************************
// GPIO Pin Interrupt
//*************************************************************
void
GPIOPinIntHandler (void)
{
    // Clean up, clearing the interrupt
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);
    
    if (GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_0))
    {
        lastTimeStamp = currTimeStamp;
        currTimeStamp = TimerValueGet(TIMER0_BASE, TIMER_A);

        if (currTimeStamp < lastTimeStamp)
        {   
            diffTimeStamp = currTimeStamp + SysCtlClockGet() / TIMER_RATE - lastTimeStamp;
        }
        else
        {
            diffTimeStamp = currTimeStamp - lastTimeStamp;
        }
        duty = 100*(currTimeStampLow - lastTimeStamp) / (currTimeStamp - lastTimeStamp);
    } else if (currTimeStamp != 0)
    {
        currTimeStampLow = TimerValueGet(TIMER0_BASE, TIMER_A);
    }
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
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);

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
    // The Timer0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    TimerDisable(TIMER0_BASE, TIMER_BOTH);

    // Configure Timer0B as a 16-bit periodic timer.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);

    // Set timer value
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / TIMER_RATE);
    TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet() / TIMER_RATE);

    // Enable timer
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
}




int main(void) {
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initGPIOPins();
    initButtons ();
    initResponseTimer();
    OLEDInitialise ();
    initialiseUSB_UART ();

    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO); // Used for PWM output
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    initialisePWM();
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);


    setPWM(130, 80);


    char str[80];
    while (true)
    {
        //sprintf(str, "%d\r\n", SysCtlClockGet() / diffTimeStamp);
        sprintf(str, "%d\r\n", duty);
        //(currTimeStampLow - lastTimeStamp) / (currTimeStamp - lastTimeStamp)
        UARTSend(str);

        // Don't recommmend using OLED with this. Unreliable output
        // displayButtonState ("Freq", "=", SysCtlClockGet() / diffTimeStamp, 0);
        // displayButtonState ("", "", currTimeStamp, 1);
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

