#ifndef UART_H
#define UART_H

/*************************************************************
 uart.h:
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

/**********************************************************************
 Constants
 *********************************************************************/
//---USB Serial communication: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE UART0_BASE
#define UART_USB_PERIPH_UART SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX GPIO_PIN_1
#define UART_USB_GPIO_PINS UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX
#define MAX_STR_LEN 100

#define VT100_ESC 0x1B
#define VT100_HOME "[H"
#define VT100_CLS "[2J"
#define VT100_ONE_DOWN "[1B"
#define VT100_TWO_DOWN "[2B"
#define VT100_THREE_DOWN "[3B"
#define VT100_FOUR_DOWN "[3B"
#define VT100_FIVE_DOWN "[3B"
#define VT100_SIX_DOWN "[6B"
#define VT100_SEVEN_DOWN "[7B"
#define VT100_NINE_DOWN "[9B"
#define VT100_ELEVEN_DOWN "[11B"

#define VT100_NORMAL "[m"
#define VT100_FG_YELLOW "[33m"
#define VT100_FG_WHITE "[37m"
#define VT100_BG_BLACK "[40m"
#define VT100_BG_MAGENTA "[45m"

/**********************************************************************
 Functions
 *********************************************************************/

/**********************************************************************
 initialiseUSB_UART: The UART is responsible for transmitting
 information on the status of the helicopter via a serial link from
 UART0 at 9600 baud, with 1 stop bit and no parity bit in each
 transmitted byte.
 *********************************************************************/
void initialiseUSB_UART (void);

/**********************************************************************
 UARTSend: Transmits the status information of the helicopter as a
 string via UART0. The transmission occurs using a FIFO buffer.
 *********************************************************************/
void UARTSend (char *pucBuffer);

#endif /*UART_H*/
