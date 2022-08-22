/** @file   uart.c
    @author A.J Eason A. Musalov
    @date   17/08/22
    @brief  File containing all the functions related to the UART
            functionality on the Tiva/Orbit board. Contains
            ANSI escape sequences for UART user interface printing.
            Also contains functions to print information used by 
            both the car simulator and abs controller. E.g 
            steering angle, wheel speed, brake pressure, abs state,
            wheel slippage etc. Useful for debugging and knowing 
            the state of the car/controller.

            Acknowledgements:
            Code from labs ~ Author Phil Bones:
                Code from UARTDemo.c:
                -initialiseUSB_UART
                -UARTSend
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "uart.h"

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

void vt100_set_yellow(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_FG_YELLOW);
    UARTSend (ANSIString);
}

void vt100_set_white(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    sprintf (ANSIString, "%c%s", VT100_ESC, VT100_FG_WHITE);
    UARTSend (ANSIString);
}

void vt100_clear(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_HOME);
    UARTSend (ANSIString);
    sprintf(ANSIString,"%c%s", VT100_ESC, VT100_CLS);
    UARTSend (ANSIString);
    //printf("%c%s", VT100_ESC, VT100_BOLD);
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_BG_BLACK);
    UARTSend (ANSIString);
    vt100_set_yellow();
}


void vt100_set_line_number(int line) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    char buf[6] = {0};
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_HOME);
    UARTSend (ANSIString);
    sprintf(buf, "[%dB", line);
    sprintf(ANSIString, "%c%s", VT100_ESC, buf);
    UARTSend (ANSIString);
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_CLR);
    UARTSend (ANSIString);
}

void vt100_print_steering_angle(uint8_t duty, char alphaStr[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(3);
    sprintf (ANSIString, "Duty: %2d%%    Angle: %5s degrees\r\n\n", duty, alphaStr);
    UARTSend (ANSIString);
}

void vt100_print_car_speed(char speedStr[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(5);
    sprintf (ANSIString, "%5s km/h\r\n\n", speedStr);
    UARTSend (ANSIString);
}

void vt100_print_wheel_speed(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(7);
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
}

void vt100_print_brake_pressure(uint8_t pressure) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(13);
    sprintf(ANSIString, "%d %%", pressure);
    UARTSend (ANSIString);
}


void vt100_print_pedal(bool pedal) {
    vt100_set_line_number(15);
    if (pedal == 1)
    {
        UARTSend ("ON");
    }
    else{
        UARTSend ("OFF");
    }
}

void vt100_print_slipage(bool slipArray[4], bool ABSstate) 
{
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(19);
    char buf[4];
    if (ABSstate)
    {
        strncpy(buf, "ON", 4);
    } 
    else
    {
        strncpy(buf, "OFF", 4);
    } 
    sprintf(ANSIString, "LF: %d LR: %d RF: %d RR: %d ABS: %s", slipArray[0], slipArray[1],slipArray[2], slipArray[3], buf);
    UARTSend (ANSIString);
}
