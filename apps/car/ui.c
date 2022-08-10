#include "ui.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "libs/lib_uart/ap_uart.h"

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

void vt100_print_text(void) {
    vt100_clear();
    vt100_set_yellow();
    UARTSend ("*** ABS SIM (12.07.22) ***");
    vt100_set_line_number(2);
    UARTSend ("Steering -> (1, 2):");
    vt100_set_line_number(4);
    UARTSend ("Car speed (km/h) -> (q, w):");
    vt100_set_line_number(6);
    UARTSend ("Wheel speed (km/h):");
    vt100_set_line_number(8);
    UARTSend ("Wheel PRR (Hz):");
    vt100_set_line_number(10);
    UARTSend ("Wheel radii (m)");
    vt100_set_line_number(12);
    UARTSend ("Brake pedal pressure -> ([, ]):");
    vt100_set_line_number(14);
    UARTSend ("Brake pedal push/release -> (b):");
    vt100_set_line_number(16);
    UARTSend ("Road Condition -> (r):");
    vt100_set_line_number(18);
    UARTSend ("Wheel Slip -> (LF LR RF RR):");
    vt100_set_white();
}

void vt100_print_steering_angle(uint8_t duty, char alphaStr[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(3);
    sprintf (ANSIString, "Duty: %2d%%    Angle: %5s degrees\r\n\n", duty, alphaStr);
    UARTSend (ANSIString);
}

void vt100_print_car_speed(uint8_t speed) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(5);
    sprintf (ANSIString, "%2d km/h\r\n\n", speed);
    UARTSend (ANSIString);
}

void vt100_print_wheel_speed(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(7);
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
}

void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(11);
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
}

void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(9);
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

void vt100_print_condition(uint8_t condition) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(17);
    sprintf(ANSIString, "%s", get_condition(condition));
    UARTSend (ANSIString);
}

const char* get_condition(uint8_t condition){
    
    if (condition == 0)
    {
        return "DRY";
    }
    else if (condition == 1)
    {
        return "WET";
    }
    else{
        return "ICY";
    }
}

void vt100_print_slipage(bool slipArray[4]) 
{
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(19);
    sprintf(ANSIString, "LF: %d LR: %d RF: %d RR: %d", slipArray[0], slipArray[1],slipArray[2],slipArray[3]);
    UARTSend (ANSIString);
}

