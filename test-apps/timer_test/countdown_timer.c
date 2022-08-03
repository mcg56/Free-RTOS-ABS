#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>

#include <FreeRTOS.h>
#include <task.h>

#include "libs/lib_buttons/ap_buttons.h"
#include "libs/lib_uart/ap_uart.h"

#define TIMEOUT_TIMER_PERIPH    SYSCTL_PERIPH_TIMER2
#define TIMEOUT_TIMER_BASE      TIMER2_BASE
#define TIMEOUT_TIMER           TIMER_A
#define TIMEOUT_TIMER_CONFIG    TIMER_CFG_A_PERIODIC
#define TIMEOUT_TIMER_INT_FLAG  TIMER_TIMA_TIMEOUT
#define TIMEOUT_RATE            1 // [Hz]

static bool timerOn = false;
static int hitCount = 0;

/**
 * @brief Interrupt handler for PWM reading timeout
 * @return None
 */
static void
TimeoutHandler (void)
{
    TimerIntClear(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    char str[30];
    sprintf(str, "Hit count = %d\r\n", hitCount++);
    UARTSend(str);
}

/**
 * @brief Initialise the timer for PWM edge timeout
 * @return None
 */
static void
initTimeoutTimer (void)
{
    SysCtlPeripheralEnable(TIMEOUT_TIMER_PERIPH);

    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);

    TimerConfigure(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_CONFIG);

    TimerIntRegister(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, TimeoutHandler);

    TimerLoadSet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, SysCtlClockGet() / TIMEOUT_RATE);

    TimerIntEnable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER_INT_FLAG);

    TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);
}

void toggleTimeout (void)
{
    if (timerOn)
    {
        TimerDisable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);

        timerOn = false;
    }
    else
    {
        char str[50];
        sprintf(str, "Timer Value = %ld\r\n", TimerValueGet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER));
        UARTSend(str);

        TimerLoadSet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER, SysCtlClockGet() / TIMEOUT_RATE);

        sprintf(str, "Timer Value = %ld\r\n", TimerValueGet(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER));
        UARTSend(str);

        TimerEnable(TIMEOUT_TIMER_BASE, TIMEOUT_TIMER);



        timerOn = true;
    }
}

int main (void)
{
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    initButtons ();
    initialiseUSB_UART ();
    initTimeoutTimer ();
 
    UARTSend("Running\r\n");
    while (true)
    {
        updateButtons();

        if (checkButton(LEFT) == PUSHED)
        {
            toggleTimeout();
        }
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