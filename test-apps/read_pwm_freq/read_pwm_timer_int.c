// #include <stdint.h>
// #include <stdbool.h>

// #include <inc/hw_memmap.h>
// #include <inc/hw_types.h>
// #include <driverlib/sysctl.h>
// #include <driverlib/gpio.h>

// #include <FreeRTOS.h>
// #include <task.h>

// #include "driverlib/pwm.h"
// #include "driverlib/timer.h"

// #include "libs/lib_buttons/ap_buttons.h"
// #include "libs/lib_pwm/ap_pwm.h"
// #include "libs/lib_OrbitOled/OrbitOLEDInterface.h"
// #include "libs/lib_uart/ap_uart.h"

// void 
// updateTitty (void)
// {
//     int risingEdge;
//     while (1)
//     {
//         TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
//         while(/*Wait for event to occur*/);
//     }
// }


// //*****************************************************************************
// // Intialise timer for PI control update
// //*****************************************************************************
// void
// initResponseTimer (void)
// {
//     // The Timer0 peripheral must be enabled for use.
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

//     TimerDisable(TIMER0_BASE, TIMER_A);

//     // Configure Timer0B as a 16-bit periodic timer.
//     TimerConfigure(TIMER0_BASE, TIMER_CFG_A_CAP_TIME_UP);

//     // Set timer value
//     TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());

//     // Rising edge
//     TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

//     // Enable interrupt
//     TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);

//     // Enable timer
//     TimerEnable(TIMER0_BASE, TIMER_A);
// }

// //*************************************************************
// // Intialise GPIO Pins
// // PB0 and PB1 are used for quadrature encoding
// // PC4 is used for reference yaw input
// //*************************************************************
// void
// initGPIOPins (void)
// {
//     // Enable port peripheral
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//     // Set pin 0,1 and 4 as input
//     GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
// }


// int main(void) {
//     SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

//     initGPIOPins();
//     initResponseTimer();
//     OLEDInitialise ();
//     initialiseUSB_UART ();


//     char str[80];
//     while (true)
//     {
//         sprintf(str, "%d\r\n", SysCtlClockGet() / diffTimeStamp);
//         UARTSend(str);

//         // Don't recommmend using OLED with this. Unreliable output
//         // displayButtonState ("Freq", "=", SysCtlClockGet() / diffTimeStamp, 0);
//         // displayButtonState ("", "", currTimeStamp, 1);
//     }

//     vTaskStartScheduler();

//     return 0;
// }


// // This is an error handling function called when FreeRTOS asserts.
// // This should be used for debugging purposes
// void vAssertCalled( const char * pcFile, unsigned long ulLine ) {
//     (void)pcFile; // unused
//     (void)ulLine; // unused
//     while (true) ;
// }