#include "ui.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include <stdio.h>
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
    int num;
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_HOME);
    UARTSend (ANSIString);
    num = sprintf(buf, "[%dB", line);
    // buf[num] = 0; // null termination
    sprintf(ANSIString, "%c%s", VT100_ESC, buf);
    UARTSend (ANSIString);
    sprintf(ANSIString, "%c%s", VT100_ESC, VT100_CLR);
    UARTSend (ANSIString);
}

void vt100_print_text(void) {
    vt100_clear();
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
    UARTSend ("Wheel radii (0.4m - 0.9m) -> (4, 5, 6, 7, 8, 9):");
    vt100_set_line_number(12);
    UARTSend ("Brake pressure %% -> ([, ]):");
}

void vt100_print_steering_angle(uint8_t duty, char alphaStr[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(3);
    vt100_set_white();
    sprintf (ANSIString, "Duty: %2d%%    Angle: %5s degrees\r\n\n", duty, alphaStr);
    UARTSend (ANSIString);
    vt100_set_yellow();
}

void vt100_print_car_speed(uint8_t speed) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(5);
    vt100_set_white();
    sprintf (ANSIString, "%2d km/h\r\n\n", speed);
    UARTSend (ANSIString);
    vt100_set_yellow();
}

void vt100_print_wheel_speed(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(7);
    vt100_set_white();
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
    vt100_set_yellow();
}

void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(11);
    vt100_set_white();
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
    vt100_set_yellow();
}

void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(9);
    vt100_set_white();
    sprintf (ANSIString, "Lf: %5s, Lr: %5s, Rf: %5s, Rr: %5s\r\n\n", LF, LR, RF, RR);
    UARTSend (ANSIString);
    vt100_set_yellow();
}



void vt100_print_brake_pressure(void) {
    char ANSIString[MAX_STR_LEN + 1]; // For uart message
    vt100_set_line_number(13);
    vt100_set_white();
    sprintf(ANSIString, "%3ld", get_pressure());
    UARTSend (ANSIString);
    vt100_set_yellow();
}

int32_t get_pressure(void){
    return 69;
}
