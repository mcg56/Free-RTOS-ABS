#ifndef UI_H_
#define UI_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "utils/ustdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "wheels.h"

void vt100_set_yellow(void);
void vt100_set_white(void);
void vt100_set_line_number(int line);
void vt100_print_text(void);
void vt100_print_steering_angle(uint8_t duty, char alphaStr[6]);
void vt100_print_car_speed(char speedStr[6]);
void vt100_print_wheel_speed(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_radii(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_prr(char LF[6],char LR[6],char RF[6],char RR[6]);
void vt100_print_brake_pressure(uint8_t pressure);
void vt100_print_pedal(bool pedal);
void vt100_print_condition(uint8_t condition);
const char* get_condition(uint8_t condition);
void vt100_print_slipage(bool slipArray[4], bool ABSstate) ;

/**
 * @brief Update the UART terminal with data about the car.
 * @param args Unused
 * @return No return
 */
void updateUARTTask(void* args);

#endif

