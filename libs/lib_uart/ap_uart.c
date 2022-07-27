/*************************************************************
 uart.c:
 File containing all the functions related to the UART
 functionality on the Tiva/Orbit board.
 The UART is responsible for transmitting information on the
 status of the helicopter via a serial link from UART0 at 9600
 baud, with 1 stop bit and no parity bit in each transmitted
 byte. Status information includes the desired and actual yaw
 (in degrees), the desired and actual altitude (as a percentage
 of the maximum altitude), the duty cycle of each of the PWM
 signals controlling the rotors (%, with 0 meaning off) and
 the current operating mode.

 Author:  Phil Bones

 Acknowledgements:
 Code from labs ~ Author Phil Bones:
     Code from UARTDemo.c:
      -initialiseUSB_UART
      -UARTSend

 ************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "ap_uart.h"

/**********************************************************************
 initialiseUSB_UART: The UART is responsible for transmitting
 information on the status of the helicopter via a serial link from
 UART0 at 9600 baud, with 1 stop bit and no parity bit in each
 transmitted byte.
 *********************************************************************/
void initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

/**********************************************************************
 UARTSend: Transmits the status information of the helicopter as a
 string via UART0. The transmission occurs using a FIFO buffer.
 *********************************************************************/
void UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}
