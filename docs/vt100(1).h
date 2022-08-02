#ifndef VT100_H_
#define VT100_H_

#include "car.hpp"
#include "brake.hpp"

// limits in % & Hz:
#define MAX_PWM_MAIN_ROTOR 80
#define MAX_PWM_TAIL_ROTOR 70

// VT100 commands
#define VT100_ESC 0x1B
#define VT100_HOME "[H"
#define VT100_CLS "[2J"
#define VT100_CLR "[K"
#define VT100_BOLD "[1m"
#define VT100_NORMAL "[m"
#define VT100_FG_YELLOW "[33m"
#define VT100_FG_WHITE "[37m"
#define VT100_BG_BLACK "[40m"
#define VT100_BG_MAGENTA "[45m"

void vt100_init(int baud);

void vt100_print_text(void);
void vt100_print_steering_angle(Car);
void vt100_print_car_speed(Car);
void vt100_print_wheel_speed(Car);
void vt100_print_prr(Car);
void vt100_print_circumference(Car);
void vt100_print_brake_pressure(Brake);

extern int vt100_getc(void);

#endif